"""Graphical teleop for the CSEN934 Webots car.

Publishes AckermannDrive commands on /Car_Ackrm and shows live telemetry from
/Car_Odom_GT (actual speed, heading, trajectory).

Keys (hold):  W / Up     throttle   (release to cruise at that speed)
              S / Down   brake, then press again to reverse
              A D / Left Right   steer (re-centers on release if enabled)
              Space      stop          X   center steering
              R          reset the car to its start pose (calls /Car_Reset)
Closing the window (or Ctrl+C in the terminal) stops the car.
"""
import math
import signal
import sys
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from PyQt5.QtCore import QEvent, QObject, QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import (QBrush, QColor, QFont, QFontDatabase, QLinearGradient, QPainter,
                         QPainterPath, QPen, QPolygonF, QRadialGradient)
from PyQt5.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout, QLabel,
                             QMainWindow, QPushButton, QSizePolicy, QSlider, QVBoxLayout,
                             QWidget)

# ----------------------------------------------------------------------------- theme
BG = "#0B0F14"
CARD = "#11171F"
CARD_BORDER = "#1E2733"
TILE = "#0D1219"
TRACK = "#1C2530"
TEXT = "#E6EDF3"
MUTED = "#7D8A99"
FAINT = "#3A4656"
ACCENT = "#38BDF8"    # commanded values
REVERSE = "#FB923C"   # reverse / braking
LIVE = "#4ADE80"
WARN = "#FBBF24"
DANGER = "#EF4444"

UI_FAMILY = "Sans Serif"
MONO_FAMILY = "Monospace"

KEYMAP = {
    Qt.Key_W: "up", Qt.Key_Up: "up",
    Qt.Key_S: "down", Qt.Key_Down: "down",
    Qt.Key_A: "left", Qt.Key_Left: "left",
    Qt.Key_D: "right", Qt.Key_Right: "right",
    Qt.Key_Space: "stop", Qt.Key_X: "center", Qt.Key_R: "reset",
}

STYLE = f"""
QMainWindow, QWidget#root {{ background: {BG}; }}
QFrame#card {{ background: {CARD}; border: 1px solid {CARD_BORDER}; border-radius: 16px; }}
QFrame#tile {{ background: {TILE}; border: 1px solid {CARD_BORDER}; border-radius: 12px; }}
QFrame#accentBar {{ background: {ACCENT}; border-radius: 3px; }}
QLabel {{ color: {TEXT}; background: transparent; border: none; }}
QLabel#muted, QLabel#cardTitle {{ color: {MUTED}; }}
QPushButton#stop {{ background: {DANGER}; color: white; border: none; border-radius: 14px; }}
QPushButton#stop:hover {{ background: #F26464; }}
QPushButton#stop:pressed {{ background: #B91C1C; }}
QPushButton#ghost {{ background: transparent; color: {TEXT}; border: 1px solid #2A3644;
                     border-radius: 10px; padding: 7px 10px; }}
QPushButton#ghost:hover {{ border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton#ghost:pressed {{ background: #0E2A38; }}
QPushButton#mini {{ background: transparent; color: {MUTED}; border: none; padding: 0 4px; }}
QPushButton#mini:hover {{ color: {ACCENT}; }}
QSlider {{ min-height: 24px; }}
QSlider::groove:horizontal {{ height: 6px; background: {TRACK}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: {TEXT}; width: 18px; height: 18px; margin: -6px 0;
                              border-radius: 9px; }}
QSlider::handle:horizontal:hover {{ background: {ACCENT}; }}
QToolTip {{ background: {CARD}; color: {TEXT}; border: 1px solid {CARD_BORDER}; }}
"""


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def ui_font(size, weight=QFont.Normal, mono=False, spacing=0.0):
    f = QFont(MONO_FAMILY if mono else UI_FAMILY)
    f.setPixelSize(max(1, int(round(size))))
    f.setWeight(weight)
    if spacing:
        f.setLetterSpacing(QFont.AbsoluteSpacing, spacing)
    return f


def pick_fonts():
    global UI_FAMILY, MONO_FAMILY
    families = set(QFontDatabase().families())
    UI_FAMILY = next((f for f in ("Inter", "Ubuntu", "Segoe UI", "DejaVu Sans")
                      if f in families), UI_FAMILY)
    MONO_FAMILY = next((f for f in ("JetBrains Mono", "Ubuntu Mono", "DejaVu Sans Mono")
                        if f in families), MONO_FAMILY)


def pen(color, width, cap=Qt.RoundCap):
    return QPen(QColor(color), width, Qt.SolidLine, cap, Qt.RoundJoin)


def with_alpha(color, alpha):
    c = QColor(color)
    c.setAlpha(alpha)
    return c


# ----------------------------------------------------------------------------- ROS node
class TeleopNode(Node):
    """Holds the current command (published at 20 Hz) and the latest odometry."""

    def __init__(self):
        super().__init__("teleop_gui")
        par = self.declare_parameter
        self.max_speed = float(par("max_speed", 20.0).value)      # m/s forward
        self.max_reverse = float(par("max_reverse", 5.0).value)   # m/s backward
        self.max_steer = float(par("max_steer", 0.5).value)       # rad
        self.accel = float(par("accel", 4.0).value)               # m/s per second of key hold
        self.steer_rate = float(par("steer_rate", 0.8).value)     # rad per second of key hold
        self.cmd_topic = par("cmd_topic", "/Car_Ackrm").value
        self.odom_topic = par("odom_topic", "/Car_Odom_GT").value
        self.reset_service = par("reset_service", "/Car_Reset").value

        self.speed = 0.0
        self.steer = 0.0

        self._lock = threading.Lock()
        self._pose = None           # (x, y, yaw)
        self._fwd_speed = 0.0
        self._yaw_rate = 0.0
        self._sim_time = 0.0
        self._last_odom = None      # monotonic time of the last odom message
        self._odom_count = 0
        self._trail = deque(maxlen=6000)

        self.pub = self.create_publisher(AckermannDrive, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.reset_client = self.create_client(Trigger, self.reset_service)
        self.create_timer(0.05, self.publish_cmd)   # 20 Hz keeps the command alive

    def set_command(self, speed, steer):
        self.speed = clamp(float(speed), -self.max_reverse, self.max_speed)
        self.steer = clamp(float(steer), -self.max_steer, self.max_steer)

    def publish_cmd(self, speed=None, steer=None):
        msg = AckermannDrive()
        msg.speed = float(self.speed if speed is None else speed)
        msg.steering_angle = float(self.steer if steer is None else steer)
        self.pub.publish(msg)

    def clear_trail(self):
        with self._lock:
            self._trail.clear()

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        tw = msg.twist.twist
        # The Webots GPS speed vector is in the world frame: project it on the heading
        fwd = tw.linear.x * math.cos(yaw) + tw.linear.y * math.sin(yaw)
        with self._lock:
            last = self._trail[-1] if self._trail else None
            if last and math.hypot(p.x - last[0], p.y - last[1]) > 5.0:
                self._trail.clear()     # the car teleported: simulation was reset
                last = None
            if last is None or math.hypot(p.x - last[0], p.y - last[1]) > 0.15:
                self._trail.append((p.x, p.y))
            self._pose = (p.x, p.y, yaw)
            self._fwd_speed = fwd
            self._yaw_rate = tw.angular.z
            self._sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._last_odom = time.monotonic()
            self._odom_count += 1

    def snapshot(self):
        with self._lock:
            age = None if self._last_odom is None else time.monotonic() - self._last_odom
            return {
                "pose": self._pose, "trail": list(self._trail), "speed": self._fwd_speed,
                "yaw_rate": self._yaw_rate, "sim_time": self._sim_time,
                "age": age, "count": self._odom_count,
            }


# ----------------------------------------------------------------------------- widgets
class Speedometer(QWidget):
    """270-degree gauge: coloured arc = target speed, white tick = actual speed."""

    def __init__(self, vmin, vmax, parent=None):
        super().__init__(parent)
        self.vmin, self.vmax = vmin, vmax
        self.target = 0.0
        self.actual = None
        self.setMinimumSize(280, 250)

    def set_values(self, target, actual):
        self.target, self.actual = target, actual
        self.update()

    def _angle(self, v):
        f = (clamp(v, self.vmin, self.vmax) - self.vmin) / (self.vmax - self.vmin)
        return 225.0 - 270.0 * f

    @staticmethod
    def _pt(cx, cy, r, deg):
        a = math.radians(deg)
        return QPointF(cx + r * math.cos(a), cy - r * math.sin(a))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w * 0.42, h * 0.46)
        cx, cy = w / 2, h / 2 + r * 0.06
        thick = max(8.0, r * 0.10)
        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)

        # track, with the reverse zone tinted
        p.setPen(pen(TRACK, thick))
        p.drawArc(rect, 225 * 16, -270 * 16)
        a0 = self._angle(0.0)
        p.setPen(pen(with_alpha(REVERSE, 45), thick, Qt.FlatCap))
        p.drawArc(rect, 225 * 16, int((a0 - 225) * 16))

        # target arc (with a soft glow)
        a1 = self._angle(self.target)
        col = ACCENT if self.target >= 0 else REVERSE
        if abs(a1 - a0) > 0.3:
            p.setPen(pen(with_alpha(col, 50), thick * 2.0))
            p.drawArc(rect, int(a0 * 16), int((a1 - a0) * 16))
            p.setPen(pen(col, thick))
            p.drawArc(rect, int(a0 * 16), int((a1 - a0) * 16))

        # ticks + labels
        p.setFont(ui_font(r * 0.085, QFont.DemiBold))
        for v in range(int(math.ceil(self.vmin)), int(math.floor(self.vmax)) + 1):
            major = v % 5 == 0
            deg = self._angle(v)
            r1 = r - thick * 0.95
            r2 = r1 - (thick * 0.75 if major else thick * 0.35)
            p.setPen(pen(MUTED if major else FAINT, 2.0 if major else 1.2))
            p.drawLine(self._pt(cx, cy, r1, deg), self._pt(cx, cy, r2, deg))
            if major:
                c = self._pt(cx, cy, r2 - r * 0.11, deg)
                p.setPen(QColor(MUTED))
                p.drawText(QRectF(c.x() - 20, c.y() - 10, 40, 20), Qt.AlignCenter, str(v))

        # actual-speed marker
        live = self.actual is not None
        if live:
            deg = self._angle(self.actual)
            p.setPen(pen(with_alpha(TEXT, 60), 9))
            p.drawLine(self._pt(cx, cy, r - thick * 0.8, deg), self._pt(cx, cy, r + thick * 0.8, deg))
            p.setPen(pen(TEXT, 3.5))
            p.drawLine(self._pt(cx, cy, r - thick * 0.8, deg), self._pt(cx, cy, r + thick * 0.8, deg))

        # gear chip
        gear = "R" if self.target < -0.05 else ("D" if self.target > 0.05 else "N")
        gcol = {"R": REVERSE, "D": ACCENT, "N": MUTED}[gear]
        chip = QRectF(cx - r * 0.13, cy - r * 0.55, r * 0.26, r * 0.19)
        p.setPen(pen(with_alpha(gcol, 160), 1.5))
        p.setBrush(with_alpha(gcol, 28))
        p.drawRoundedRect(chip, 6, 6)
        p.setBrush(Qt.NoBrush)
        p.setFont(ui_font(r * 0.12, QFont.Bold))
        p.setPen(QColor(gcol))
        p.drawText(chip, Qt.AlignCenter, gear)

        # big readout
        value = self.actual if live else self.target
        p.setFont(ui_font(r * 0.42, QFont.Bold, mono=True))
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(cx - r, cy - r * 0.34, 2 * r, r * 0.52), Qt.AlignCenter, f"{abs(value):.1f}")
        p.setFont(ui_font(r * 0.095, QFont.DemiBold, spacing=1.0))
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(cx - r, cy + r * 0.18, 2 * r, r * 0.15), Qt.AlignCenter,
                   "M/S  ·  ACTUAL" if live else "M/S  ·  TARGET")

        p.setFont(ui_font(r * 0.095, QFont.Bold, mono=True))
        p.setPen(QColor(col))
        p.drawText(QRectF(cx - r, cy + r * 0.62, 2 * r, r * 0.16), Qt.AlignCenter,
                   f"TARGET {self.target:+.1f} m/s")
        p.setPen(QColor(MUTED))
        p.drawText(QRectF(cx - r, cy + r * 0.80, 2 * r, r * 0.16), Qt.AlignCenter,
                   f"{abs(value) * 3.6:.0f} km/h")


class SteeringWidget(QWidget):
    """Steering wheel that turns with the command, plus a left/right bar."""

    def __init__(self, max_steer, parent=None):
        super().__init__(parent)
        self.max_steer = max_steer
        self.steer = 0.0
        self.setMinimumSize(240, 250)

    def set_steer(self, s):
        self.steer = s
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        bar_h = 66
        r = min(w * 0.34, (h - bar_h) * 0.40)
        cx, cy = w / 2, (h - bar_h) / 2 + 6
        frac = self.steer / self.max_steer if self.max_steer else 0.0
        rim_w = r * 0.17

        p.save()
        p.translate(cx, cy)
        p.rotate(-frac * 120.0)     # +steer = left = counter-clockwise
        p.setPen(pen("#1F2935", r * 0.14))
        for x, y in ((-r, 0), (r, 0), (0, r)):
            p.drawLine(QPointF(0, 0), QPointF(x * 0.9, y * 0.9))
        p.setPen(pen("#273241", rim_w))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(0, 0), r, r)
        p.setPen(pen(with_alpha(TEXT, 22), rim_w * 0.35))
        p.drawArc(QRectF(-r, -r, 2 * r, 2 * r), 35 * 16, 110 * 16)
        grad = QRadialGradient(QPointF(0, -r * 0.08), r * 0.32)
        grad.setColorAt(0, QColor("#2A3645"))
        grad.setColorAt(1, QColor("#151C26"))
        p.setPen(pen("#2E3A4A", 1.5))
        p.setBrush(QBrush(grad))
        p.drawEllipse(QPointF(0, 0), r * 0.28, r * 0.28)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(ACCENT))
        p.drawRoundedRect(QRectF(-r * 0.05, -r - rim_w * 0.5, r * 0.10, rim_w), 3, 3)
        p.restore()

        p.setFont(ui_font(r * 0.15, QFont.Bold, mono=True))
        p.setPen(QColor(TEXT))
        p.drawText(QRectF(cx - r * 0.3, cy - r * 0.15, r * 0.6, r * 0.3), Qt.AlignCenter,
                   f"{math.degrees(self.steer):+.0f}°")

        # left/right bar
        by = h - bar_h + 14
        bx0, bx1 = w * 0.08, w * 0.92
        mid = (bx0 + bx1) / 2
        x = mid - frac * (bx1 - bx0) / 2       # left steer moves the knob left
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(TRACK))
        p.drawRoundedRect(QRectF(bx0, by, bx1 - bx0, 6), 3, 3)
        if abs(x - mid) > 0.5:
            p.setBrush(QColor(ACCENT))
            p.drawRoundedRect(QRectF(min(x, mid), by, abs(x - mid), 6), 3, 3)
        p.setPen(pen(FAINT, 2))
        p.drawLine(QPointF(mid, by - 5), QPointF(mid, by + 11))
        p.setPen(pen(BG, 2))
        p.setBrush(QColor(TEXT))
        p.drawEllipse(QPointF(x, by + 3), 8, 8)

        p.setFont(ui_font(10, QFont.Bold, spacing=1.2))
        p.setPen(QColor(MUTED))
        lab = QRectF(bx0, by + 16, bx1 - bx0, 18)
        p.drawText(lab, Qt.AlignLeft | Qt.AlignVCenter, "LEFT")
        p.drawText(lab, Qt.AlignRight | Qt.AlignVCenter, "RIGHT")
        p.setFont(ui_font(12, QFont.DemiBold, mono=True))
        p.setPen(QColor(TEXT))
        p.drawText(lab.adjusted(0, 0, 0, 22), Qt.AlignHCenter | Qt.AlignTop,
                   f"{self.steer:+.2f} rad")


class MiniMap(QWidget):
    """Top-down trajectory from odometry, auto-zoomed, with the car's heading."""

    def __init__(self, topic, parent=None):
        super().__init__(parent)
        self.topic = topic
        self.trail = []
        self.pose = None
        self._view = None       # smoothed [center_x, center_y, span]
        self.setMinimumSize(260, 230)

    def set_data(self, trail, pose):
        self.trail, self.pose = trail, pose
        self.update()

    def reset_view(self):
        self._view = None

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        frame = QRectF(0, 0, w, h)
        clip = QPainterPath()
        clip.addRoundedRect(frame, 12, 12)
        p.fillPath(clip, QColor("#0A0E13"))
        p.setClipPath(clip)

        if self.pose is None:
            p.setPen(QColor(MUTED))
            p.setFont(ui_font(13, QFont.DemiBold))
            p.drawText(frame.adjusted(0, -12, 0, -12), Qt.AlignCenter, "Waiting for odometry…")
            p.setFont(ui_font(11, mono=True))
            p.setPen(QColor(FAINT))
            p.drawText(frame.adjusted(0, 22, 0, 22), Qt.AlignCenter, self.topic)
            return

        pts = self.trail + [self.pose[:2]]
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        tcx, tcy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        tspan = max(max(xs) - min(xs), max(ys) - min(ys), 40.0) * 1.25
        if self._view is None:
            self._view = [tcx, tcy, tspan]
        else:
            for i, t in enumerate((tcx, tcy, tspan)):
                self._view[i] += (t - self._view[i]) * 0.15
        vx, vy, span = self._view
        s = min(w, h) / span

        def to_screen(x, y):
            return QPointF(w / 2 + (x - vx) * s, h / 2 - (y - vy) * s)

        # grid
        step = next((st for st in (1, 2, 5, 10, 20, 50, 100, 200, 500) if st * s >= 38), 1000)
        p.setPen(QPen(QColor("#141B24"), 1))
        gx = math.floor((vx - w / 2 / s) / step) * step
        while gx <= vx + w / 2 / s:
            sx = to_screen(gx, 0).x()
            p.drawLine(QPointF(sx, 0), QPointF(sx, h))
            gx += step
        gy = math.floor((vy - h / 2 / s) / step) * step
        while gy <= vy + h / 2 / s:
            sy = to_screen(0, gy).y()
            p.drawLine(QPointF(0, sy), QPointF(w, sy))
            gy += step

        # trail
        car = to_screen(*self.pose[:2])
        if self.trail:
            path = QPainterPath(to_screen(*self.trail[0]))
            for x, y in self.trail[1:]:
                path.lineTo(to_screen(x, y))
            path.lineTo(car)
            p.setBrush(Qt.NoBrush)
            p.setPen(pen(with_alpha(ACCENT, 45), 8))
            p.drawPath(path)
            p.setPen(pen(ACCENT, 2.5))
            p.drawPath(path)
            p.setPen(pen(MUTED, 2))
            p.drawEllipse(to_screen(*self.trail[0]), 5, 5)

        # car
        p.save()
        p.translate(car)
        p.rotate(-math.degrees(self.pose[2]))
        halo = QRadialGradient(QPointF(0, 0), 26)
        halo.setColorAt(0, with_alpha(ACCENT, 90))
        halo.setColorAt(1, with_alpha(ACCENT, 0))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(halo))
        p.drawEllipse(QPointF(0, 0), 26, 26)
        p.setPen(pen(BG, 1.5))
        p.setBrush(QColor(TEXT))
        p.drawPolygon(QPolygonF([QPointF(13, 0), QPointF(-8, 8), QPointF(-4, 0), QPointF(-8, -8)]))
        p.restore()

        # scale bar + axes hint
        px = step * s
        p.setPen(pen(MUTED, 2, Qt.FlatCap))
        p.drawLine(QPointF(14, h - 16), QPointF(14 + px, h - 16))
        p.drawLine(QPointF(14, h - 20), QPointF(14, h - 12))
        p.drawLine(QPointF(14 + px, h - 20), QPointF(14 + px, h - 12))
        p.setFont(ui_font(11, QFont.DemiBold, mono=True))
        p.drawText(QRectF(20 + px, h - 26, 80, 20), Qt.AlignLeft | Qt.AlignVCenter, f"{step} m")
        p.setPen(QColor(FAINT))
        p.drawText(QRectF(w - 110, 8, 100, 18), Qt.AlignRight | Qt.AlignVCenter, "x →   y ↑")


class Toggle(QWidget):
    toggled = pyqtSignal(bool)

    def __init__(self, checked=True, parent=None):
        super().__init__(parent)
        self.checked = checked
        self.setFixedSize(42, 24)
        self.setCursor(Qt.PointingHandCursor)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.checked = not self.checked
            self.update()
            self.toggled.emit(self.checked)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(ACCENT if self.checked else TRACK))
        p.drawRoundedRect(QRectF(0, 0, 42, 24), 12, 12)
        p.setBrush(QColor(BG if self.checked else MUTED))
        p.drawEllipse(QPointF(30 if self.checked else 12, 12), 8, 8)


class KeyCap(QWidget):
    """On-screen key that lights up while held (keyboard or mouse)."""

    pressed = pyqtSignal()
    released = pyqtSignal()

    def __init__(self, letter, caption, color=ACCENT, parent=None):
        super().__init__(parent)
        self.letter, self.caption, self.color = letter, caption, color
        self.active = False
        self.setFixedSize(76, 68)
        self.setCursor(Qt.PointingHandCursor)

    def set_active(self, active):
        if active != self.active:
            self.active = active
            self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.pressed.emit()

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.released.emit()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        base = QRectF(2, 2, self.width() - 4, self.height() - 9)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#06090D"))
        p.drawRoundedRect(base.translated(0, 5), 11, 11)
        top = base.translated(0, 4 if self.active else 0)
        grad = QLinearGradient(top.topLeft(), top.bottomLeft())
        if self.active:
            p.setBrush(with_alpha(self.color, 55))
            p.drawRoundedRect(top.adjusted(-2, -2, 2, 2), 12, 12)
            grad.setColorAt(0, QColor(self.color).lighter(115))
            grad.setColorAt(1, QColor(self.color))
            border, fg, sub = QColor(self.color), QColor(BG), with_alpha(BG, 170)
        else:
            grad.setColorAt(0, QColor("#1D2633"))
            grad.setColorAt(1, QColor("#151C25"))
            border, fg, sub = QColor("#2A3644"), QColor(TEXT), QColor(MUTED)
        p.setBrush(QBrush(grad))
        p.setPen(QPen(border, 1))
        p.drawRoundedRect(top, 10, 10)
        p.setPen(fg)
        p.setFont(ui_font(21, QFont.Bold))
        p.drawText(top.adjusted(0, 2, 0, -16), Qt.AlignCenter, self.letter)
        p.setPen(sub)
        p.setFont(ui_font(9, QFont.Bold, spacing=1.0))
        p.drawText(top.adjusted(0, 0, 0, -7), Qt.AlignHCenter | Qt.AlignBottom, self.caption)


class StatTile(QFrame):
    def __init__(self, label, unit, parent=None):
        super().__init__(parent)
        self.setObjectName("tile")
        v = QVBoxLayout(self)
        v.setContentsMargins(12, 9, 12, 10)
        v.setSpacing(2)
        lab = QLabel(label.upper())
        lab.setObjectName("muted")
        lab.setFont(ui_font(10, QFont.Bold, spacing=1.2))
        row = QHBoxLayout()
        row.setSpacing(5)
        self.val = QLabel("—")
        self.val.setFont(ui_font(20, QFont.Bold, mono=True))
        u = QLabel(unit)
        u.setObjectName("muted")
        u.setFont(ui_font(11))
        row.addWidget(self.val)
        row.addWidget(u, 0, Qt.AlignBottom)
        row.addStretch()
        v.addWidget(lab)
        v.addLayout(row)

    def set(self, text):
        if self.val.text() != text:
            self.val.setText(text)


def make_card(title, extra=None):
    frame = QFrame()
    frame.setObjectName("card")
    v = QVBoxLayout(frame)
    v.setContentsMargins(18, 14, 18, 16)
    v.setSpacing(10)
    head = QHBoxLayout()
    lbl = QLabel(title.upper())
    lbl.setObjectName("cardTitle")
    lbl.setFont(ui_font(11, QFont.Bold, spacing=1.6))
    head.addWidget(lbl)
    head.addStretch()
    if extra is not None:
        head.addWidget(extra)
    v.addLayout(head)
    return frame, v


class KeyFilter(QObject):
    """App-wide key capture so driving keys work whatever widget has focus."""

    def __init__(self, window):
        super().__init__(window)
        self.window = window

    def eventFilter(self, obj, ev):
        if ev.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            action = KEYMAP.get(ev.key())
            if action is None:
                return False
            if not ev.isAutoRepeat():
                if ev.type() == QEvent.KeyPress:
                    self.window.key_down(action)
                else:
                    self.window.key_up(action)
            return True
        return False


# ----------------------------------------------------------------------------- window
class TeleopWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.held = set()
        self._lock = None             # key that braked to zero; it must be released first
        self._steer_by_keys = False   # auto-center only applies to key steering
        self.autocenter = True
        self._t = time.monotonic()
        self._rate_t, self._rate_n, self._rate = time.monotonic(), 0, 0.0
        self._pill_text = None
        self._reset_future = None
        self._flash_id = 0

        self.setWindowTitle("CSEN934 Teleop")
        self.resize(1280, 800)
        self.setMinimumSize(1120, 700)
        self.setStyleSheet(STYLE)

        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(24, 20, 24, 16)
        outer.setSpacing(16)
        outer.addLayout(self._build_header())

        grid = QGridLayout()
        grid.setSpacing(16)
        grid.addWidget(self._build_speed(), 0, 0)
        grid.addWidget(self._build_steering(), 0, 1)
        grid.addWidget(self._build_map(), 0, 2)
        grid.addWidget(self._build_keyboard(), 1, 0)
        grid.addWidget(self._build_manual(), 1, 1)
        grid.addWidget(self._build_telemetry(), 1, 2)
        for c in range(3):
            grid.setColumnStretch(c, 1)
        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 2)
        outer.addLayout(grid, 1)

        self._hint_text = ("Hold  W / S  to change speed (it cruises when released)   ·   "
                           "A / D  steer   ·   Space  stop   ·   X  center   ·   R  reset   ·   "
                           "closing this window stops the car")
        self.hint = QLabel(self._hint_text)
        self.hint.setFont(ui_font(12))
        self.hint.setAlignment(Qt.AlignCenter)
        self.hint.setStyleSheet(f"color: {MUTED};")
        outer.addWidget(self.hint)

        self._filter = KeyFilter(self)
        QApplication.instance().installEventFilter(self._filter)
        QApplication.instance().applicationStateChanged.connect(self._on_app_state)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(25)   # 40 fps UI + command ramping
        self._refresh()

    # ---- layout builders
    def _build_header(self):
        row = QHBoxLayout()
        row.setSpacing(14)
        bar = QFrame()
        bar.setObjectName("accentBar")
        bar.setFixedSize(6, 44)
        row.addWidget(bar)
        tbox = QVBoxLayout()
        tbox.setSpacing(2)
        title = QLabel("Vehicle Teleop")
        title.setFont(ui_font(24, QFont.Bold))
        sub = QLabel(f"CSEN934  ·  Webots BMW X5  ·  publishing {self.node.cmd_topic}"
                     f"  ·  listening {self.node.odom_topic}")
        sub.setObjectName("muted")
        sub.setFont(ui_font(12))
        tbox.addWidget(title)
        tbox.addWidget(sub)
        row.addLayout(tbox)
        row.addStretch()
        self.pill = QLabel()
        self.pill.setFont(ui_font(12, QFont.Bold, spacing=0.8))
        row.addWidget(self.pill, 0, Qt.AlignVCenter)
        return row

    def _build_speed(self):
        frame, v = make_card("Speed")
        self.speedo = Speedometer(-self.node.max_reverse, self.node.max_speed)
        v.addWidget(self.speedo, 1)
        return frame

    def _build_steering(self):
        frame, v = make_card("Steering")
        self.wheel = SteeringWidget(self.node.max_steer)
        v.addWidget(self.wheel, 1)
        return frame

    def _build_map(self):
        clear = QPushButton("Clear trail")
        clear.setObjectName("mini")
        clear.setFont(ui_font(11, QFont.DemiBold))
        clear.setFocusPolicy(Qt.NoFocus)
        clear.setCursor(Qt.PointingHandCursor)
        clear.clicked.connect(self._clear_trail)
        frame, v = make_card("Trajectory", clear)
        self.map = MiniMap(self.node.odom_topic)
        v.addWidget(self.map, 1)
        return frame

    def _build_keyboard(self):
        frame, v = make_card("Keyboard")
        row = QHBoxLayout()
        row.setSpacing(16)
        grid = QGridLayout()
        grid.setSpacing(8)
        self.keycaps = {
            "up": KeyCap("W", "THROTTLE"),
            "left": KeyCap("A", "LEFT"),
            "down": KeyCap("S", "BRAKE", REVERSE),
            "right": KeyCap("D", "RIGHT"),
        }
        grid.addWidget(self.keycaps["up"], 0, 1)
        grid.addWidget(self.keycaps["left"], 1, 0)
        grid.addWidget(self.keycaps["down"], 1, 1)
        grid.addWidget(self.keycaps["right"], 1, 2)
        for action, cap in self.keycaps.items():
            cap.pressed.connect(lambda a=action: self.key_down(a))
            cap.released.connect(lambda a=action: self.key_up(a))
        row.addLayout(grid)

        side = QVBoxLayout()
        side.setSpacing(8)
        stop = QPushButton("STOP\nspace")
        stop.setObjectName("stop")
        stop.setFont(ui_font(17, QFont.Bold, spacing=1.5))
        stop.setMinimumSize(104, 84)
        stop.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        stop.setFocusPolicy(Qt.NoFocus)
        stop.setCursor(Qt.PointingHandCursor)
        stop.clicked.connect(self.stop)
        center = QPushButton("Center  ·  X")
        center.setObjectName("ghost")
        center.setFont(ui_font(12, QFont.DemiBold))
        center.setFocusPolicy(Qt.NoFocus)
        center.setCursor(Qt.PointingHandCursor)
        center.clicked.connect(lambda: self.key_down("center"))
        reset = QPushButton("↺  Reset  ·  R")
        reset.setObjectName("ghost")
        reset.setFont(ui_font(12, QFont.DemiBold))
        reset.setFocusPolicy(Qt.NoFocus)
        reset.setCursor(Qt.PointingHandCursor)
        reset.setToolTip("Put the car back at its start pose")
        reset.clicked.connect(self.reset_car)
        side.addWidget(stop, 1)
        side.addWidget(center)
        side.addWidget(reset)
        row.addLayout(side, 1)
        v.addStretch()
        v.addLayout(row)
        v.addStretch()
        return frame

    def _labelled_row(self, text):
        row = QHBoxLayout()
        lab = QLabel(text)
        lab.setFont(ui_font(13, QFont.DemiBold))
        val = QLabel()
        val.setFont(ui_font(13, QFont.Bold, mono=True))
        row.addWidget(lab)
        row.addStretch()
        row.addWidget(val)
        return row, val

    def _build_manual(self):
        frame, v = make_card("Manual input")
        n = self.node

        row, self.speed_val = self._labelled_row("Target speed")
        v.addLayout(row)
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(int(-n.max_reverse * 10), int(n.max_speed * 10))
        self.speed_slider.setFocusPolicy(Qt.NoFocus)
        self.speed_slider.setCursor(Qt.PointingHandCursor)
        self.speed_slider.valueChanged.connect(self._on_speed_slider)
        v.addWidget(self.speed_slider)

        presets = QHBoxLayout()
        presets.setSpacing(6)
        for val in [s for s in (0, 5, 10, 15, 20) if s <= n.max_speed]:
            b = QPushButton(f"{val}")
            b.setObjectName("ghost")
            b.setFont(ui_font(12, QFont.DemiBold, mono=True))
            b.setFocusPolicy(Qt.NoFocus)
            b.setCursor(Qt.PointingHandCursor)
            b.setToolTip(f"Cruise at {val} m/s")
            b.clicked.connect(lambda _=False, s=val: self.node.set_command(s, self.node.steer))
            presets.addWidget(b)
        unit = QLabel("m/s")
        unit.setObjectName("muted")
        unit.setFont(ui_font(11))
        presets.addWidget(unit)
        v.addLayout(presets)
        v.addSpacing(4)

        row, self.steer_val = self._labelled_row("Steering")
        v.addLayout(row)
        self.steer_slider = QSlider(Qt.Horizontal)
        self.steer_slider.setRange(int(-n.max_steer * 100), int(n.max_steer * 100))
        self.steer_slider.setInvertedAppearance(True)   # handle left = steer left (+)
        self.steer_slider.setFocusPolicy(Qt.NoFocus)
        self.steer_slider.setCursor(Qt.PointingHandCursor)
        self.steer_slider.valueChanged.connect(self._on_steer_slider)
        v.addWidget(self.steer_slider)

        trow = QHBoxLayout()
        trow.setSpacing(10)
        tog = Toggle(self.autocenter)
        tog.toggled.connect(lambda on: setattr(self, "autocenter", on))
        tlab = QLabel("Auto-center steering when A / D are released")
        tlab.setObjectName("muted")
        tlab.setFont(ui_font(12))
        trow.addWidget(tog)
        trow.addWidget(tlab)
        trow.addStretch()
        v.addSpacing(2)
        v.addLayout(trow)
        v.addStretch()
        return frame

    def _build_telemetry(self):
        frame, v = make_card("Telemetry  ·  ground truth")
        grid = QGridLayout()
        grid.setSpacing(8)
        self.tiles = {
            "x": StatTile("Position x", "m"), "y": StatTile("Position y", "m"),
            "heading": StatTile("Heading", "°"), "speed": StatTile("Speed", "m/s"),
            "yaw_rate": StatTile("Yaw rate", "°/s"), "time": StatTile("Sim time", "s"),
        }
        for i, key in enumerate(("x", "y", "heading", "speed", "yaw_rate", "time")):
            grid.addWidget(self.tiles[key], i // 3, i % 3)
        v.addLayout(grid)
        v.addStretch()
        return frame

    # ---- input handling
    def key_down(self, action):
        if action == "stop":
            self.stop()
        elif action == "reset":
            self.reset_car()
        elif action == "center":
            self.node.set_command(self.node.speed, 0.0)
        else:
            self.held.add(action)
            if action in ("left", "right"):
                self._steer_by_keys = True

    def key_up(self, action):
        self.held.discard(action)
        if self._lock == action:
            self._lock = None

    def stop(self):
        self.held.clear()
        self._lock = None
        self.node.set_command(0.0, 0.0)
        self.node.publish_cmd()

    def reset_car(self):
        """Stop, then ask the Webots controller to put the car back at its start pose."""
        self.stop()
        if self._reset_future is not None and not self._reset_future.done():
            return
        if not self.node.reset_client.service_is_ready():
            self._flash(f"Reset unavailable: {self.node.reset_service} not found. "
                        "Is the Webots simulation running?", WARN)
            return
        self._reset_future = self.node.reset_client.call_async(Trigger.Request())
        self._flash("Resetting car…", ACCENT)

    def _check_reset(self):
        fut = self._reset_future
        if fut is None or not fut.done():
            return
        self._reset_future = None
        res = fut.result()
        if res is not None and res.success:
            self._clear_trail()
            self._flash("✓  Car reset to its start pose", LIVE)
        else:
            self._flash(f"Reset failed: {res.message if res else fut.exception()}", DANGER)

    def _flash(self, text, color, ms=3500):
        """Show a short status message in the footer, then restore the key hints."""
        self._flash_id += 1
        fid = self._flash_id
        self.hint.setText(text)
        self.hint.setStyleSheet(f"color: {color};")

        def restore():
            if fid == self._flash_id:
                self.hint.setText(self._hint_text)
                self.hint.setStyleSheet(f"color: {MUTED};")
        QTimer.singleShot(ms, restore)

    def _on_app_state(self, state):
        if state != Qt.ApplicationActive:   # window lost focus: never leave a key "stuck"
            self.held.clear()
            self._lock = None

    def _on_speed_slider(self, value):
        self.node.set_command(value / 10.0, self.node.steer)

    def _on_steer_slider(self, value):
        self._steer_by_keys = False
        self.node.set_command(self.node.speed, value / 100.0)

    def _clear_trail(self):
        self.node.clear_trail()
        self.map.reset_view()

    # ---- loop
    def _tick(self):
        now = time.monotonic()
        dt = min(now - self._t, 0.1)
        self._t = now
        n = self.node
        speed, steer = n.speed, n.steer
        up, down = "up" in self.held, "down" in self.held
        if up and not down and self._lock != "up":
            new = speed + n.accel * (2.0 if speed < 0 else 1.0) * dt
            if speed < 0 <= new:
                new, self._lock = 0.0, "up"
            speed = new
        elif down and not up and self._lock != "down":
            new = speed - n.accel * (2.0 if speed > 0 else 1.0) * dt
            if speed > 0 >= new:
                new, self._lock = 0.0, "down"
            speed = new

        left, right = "left" in self.held, "right" in self.held
        if left and not right:
            steer += n.steer_rate * dt
        elif right and not left:
            steer -= n.steer_rate * dt
        elif self.autocenter and self._steer_by_keys:
            step = 2.0 * n.steer_rate * dt
            steer = 0.0 if abs(steer) <= step else steer - math.copysign(step, steer)
        n.set_command(speed, steer)
        self._check_reset()
        self._refresh()

    def _sync_slider(self, slider, value):
        if not slider.isSliderDown() and slider.value() != value:
            slider.blockSignals(True)
            slider.setValue(value)
            slider.blockSignals(False)

    def _refresh(self):
        n = self.node
        snap = n.snapshot()
        live = snap["age"] is not None and snap["age"] < 1.0

        self.speedo.set_values(n.speed, snap["speed"] if live else None)
        self.wheel.set_steer(n.steer)
        self.map.set_data(snap["trail"], snap["pose"])
        for action, cap in self.keycaps.items():
            cap.set_active(action in self.held)
        self._sync_slider(self.speed_slider, int(round(n.speed * 10)))
        self._sync_slider(self.steer_slider, int(round(n.steer * 100)))
        self.speed_val.setText(f"{n.speed:+.1f} m/s")
        self.steer_val.setText(f"{n.steer:+.2f} rad")

        pose = snap["pose"]
        t = self.tiles
        if pose is None:
            for tile in t.values():
                tile.set("—")
        else:
            t["x"].set(f"{pose[0]:.1f}")
            t["y"].set(f"{pose[1]:.1f}")
            t["heading"].set(f"{math.degrees(pose[2]):.0f}")
            t["speed"].set(f"{snap['speed']:.1f}")
            t["yaw_rate"].set(f"{math.degrees(snap['yaw_rate']):.0f}")
            t["time"].set(f"{snap['sim_time']:.1f}")

        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self._rate = (snap["count"] - self._rate_n) / (now - self._rate_t)
            self._rate_t, self._rate_n = now, snap["count"]
        if live:
            self._set_pill(f"●  LIVE  ·  {self._rate:.0f} Hz", LIVE)
        elif snap["age"] is not None:
            self._set_pill("●  ODOMETRY PAUSED", WARN)
        else:
            self._set_pill("●  WAITING FOR WEBOTS", WARN)

    def _set_pill(self, text, color):
        if text == self._pill_text:
            return
        self._pill_text = text
        c = QColor(color)
        rgb = f"{c.red()},{c.green()},{c.blue()}"
        self.pill.setText(text)
        self.pill.setStyleSheet(
            f"color: {color}; background: rgba({rgb},26); border: 1px solid rgba({rgb},90);"
            " border-radius: 14px; padding: 5px 14px;")

    def closeEvent(self, event):
        self._timer.stop()
        self.stop()
        event.accept()


# ----------------------------------------------------------------------------- main
def main():
    # Keep the ROS context alive on Ctrl+C so the final stop command can be sent
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv[:1])     # ROS args stay with rclpy
    app.setApplicationName("CSEN934 Teleop")
    pick_fonts()
    app.setFont(ui_font(13))

    win = TeleopWindow(node)
    win.show()
    signal.signal(signal.SIGINT, lambda *_: win.close())   # Ctrl+C in the terminal

    try:
        app.exec_()
    finally:
        node.set_command(0.0, 0.0)
        node.publish_cmd()
        time.sleep(0.1)     # let the stop command leave before tearing down
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
