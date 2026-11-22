"""Drives the car around the city loop (pure pursuit on /Car_Odom_GT -> /Car_Ackrm).

Used for the SLAM trials: one lap is ~500 m and ends where it started, so the SLAM
packages can close the loop.  Parameters: speed (m/s, default 8), laps (0 = forever),
lookahead (m, default 10).  Ctrl+C stops the car.
"""
import math

from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from rclpy.node import Node

from csen934_project_pkg.node_runner import run

WHEELBASE = 2.995
MAX_STEER = 0.5
INSIDE_OFFSET = 6.0   # m left of the road centre line: clear of the barrels and cones


def loop_centerline():
    """Road centre line of the loop through the start intersection, counter-clockwise.

    Built from the road segments of city.wbt: four 40.5 m-radius curves joined by straights.
    Returns (x, y, heading) samples every ~1 m.
    """
    pts = []

    def line(x0, y0, x1, y1):
        n = max(int(math.hypot(x1 - x0, y1 - y0)), 1)
        h = math.atan2(y1 - y0, x1 - x0)
        for i in range(n):
            pts.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, h))

    def arc(cx, cy, r, a0, a1):          # counter-clockwise, angles in degrees
        n = max(int(r * math.radians(a1 - a0)), 1)
        for i in range(n):
            a = math.radians(a0 + (a1 - a0) * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), a + math.pi / 2))

    line(-45.0, 45.0, -64.5, 45.0)
    arc(-64.5, 4.5, 40.5, 90, 180)
    line(-105.0, 4.5, -105.0, -64.5)
    arc(-64.5, -64.5, 40.5, 180, 270)
    line(-64.5, -105.0, 4.5, -105.0)
    arc(4.5, -64.5, 40.5, 270, 360)
    line(45.0, -64.5, 45.0, 4.5)
    arc(4.5, 4.5, 40.5, 0, 90)
    line(4.5, 45.0, -45.0, 45.0)
    return pts


def loop_path(offset=INSIDE_OFFSET):
    """Centre line shifted to the left (= inside of the counter-clockwise loop)."""
    return [(x - offset * math.sin(h), y + offset * math.cos(h)) for x, y, h in loop_centerline()]


class Autopilot(Node):
    def __init__(self):
        super().__init__("autopilot")
        self.speed = float(self.declare_parameter("speed", 8.0).value)
        self.laps = int(self.declare_parameter("laps", 0).value)
        self.lookahead = float(self.declare_parameter("lookahead", 10.0).value)
        self.path = loop_path()
        self.idx = None
        self.travelled = 0      # path samples covered (~m)
        self.lap = 0
        self.done = False
        self.pose = None
        self.pub = self.create_publisher(AckermannDrive, "/Car_Ackrm", 10)
        self.create_subscription(Odometry, "/Car_Odom_GT", self.odom_cb, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info(f"Autopilot: {self.speed} m/s around the city loop "
                               f"({'forever' if self.laps == 0 else f'{self.laps} lap(s)'})")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def publish(self, speed, steer):
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steer)
        self.pub.publish(msg)

    def step(self):
        if self.pose is None:
            return
        if self.done:
            self.publish(0.0, 0.0)
            return
        x, y, yaw = self.pose
        n = len(self.path)
        if self.idx is None:      # first fix: closest point of the whole path
            self.idx = min(range(n), key=lambda i: math.hypot(self.path[i][0] - x,
                                                              self.path[i][1] - y))
        else:                     # afterwards: search a little ahead only
            window = [(self.idx + k) % n for k in range(40)]
            best = min(window, key=lambda i: math.hypot(self.path[i][0] - x,
                                                        self.path[i][1] - y))
            self.travelled += (best - self.idx) % n
            if self.travelled >= (self.lap + 1) * n:
                self.lap += 1
                self.get_logger().info(f"Lap {self.lap} done")
                if self.laps and self.lap >= self.laps:
                    self.done = True
                    self.publish(0.0, 0.0)
                    return
            self.idx = best

        # Pure pursuit on the rear axle (the odometry origin)
        tx, ty = self.path[(self.idx + int(self.lookahead)) % n]
        dx, dy = tx - x, ty - y
        alpha = math.atan2(dy, dx) - yaw
        ld = max(math.hypot(dx, dy), 1e-3)
        steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        self.publish(self.speed, max(-MAX_STEER, min(MAX_STEER, steer)))


def main():
    run(Autopilot, on_exit=lambda node: node.publish(0.0, 0.0))   # stop the car on exit


if __name__ == "__main__":
    main()
