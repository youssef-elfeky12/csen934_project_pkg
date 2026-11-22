"""Drives the car around the whole city (pure pursuit on /Car_Odom_GT -> /Car_Ackrm).

The roads of city.wbt form two loops that cross at the intersections (-45, 45) and (45, -45).
One "lap" is a figure-8 (~1 km) that covers both loops and ends where it started, so the
SLAM packages map the whole city and can close the loop:
  loop A (counter-clockwise, west/south part) -> right turn at (-45, 45) ->
  loop B (clockwise, north/east part)         -> left turn at (-45, 45) -> start

Parameters: speed (m/s on the loops, default 8), laps (0 = forever).  Ctrl+C stops the car.
"""
import math

from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from rclpy.node import Node

from csen934_project_pkg.node_runner import run

WHEELBASE = 2.995
MAX_STEER = 0.5
TURN_SPEED = 4.0      # m/s in the tight turns between the two loops


def figure_eight(speed):
    """Route samples (x, y, heading, max speed), ~1 m apart.

    Built from the road segments in city.wbt (curves of 40.5 m radius, 21.5 m wide roads).
    The route keeps 6 m (loop A) / 5 m (loop B) inside the centre line: clear of the cones
    and barrels standing on the centre lines and of the signs at the road edges.
    """
    pts = []

    def line(x0, y0, x1, y1, vmax=speed):
        n = max(int(math.hypot(x1 - x0, y1 - y0)), 1)
        h = math.atan2(y1 - y0, x1 - x0)
        for i in range(n):
            pts.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, h, vmax))

    def arc(cx, cy, r, a0, a1, vmax=speed):       # angles in degrees; a1 < a0 = clockwise
        n = max(int(r * math.radians(abs(a1 - a0))), 1)
        turn = math.pi / 2 if a1 > a0 else -math.pi / 2
        for i in range(n):
            a = math.radians(a0 + (a1 - a0) * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a), a + turn, vmax))

    # Loop A, counter-clockwise, 6 m inside the centre line
    line(-47.0, 43.0, -60.0, 39.0)          # merge in after the left turn
    line(-60.0, 39.0, -64.5, 39.0)
    arc(-64.5, 4.5, 34.5, 90, 180)
    line(-99.0, 4.5, -99.0, -64.5)
    arc(-64.5, -64.5, 34.5, 180, 270)
    line(-64.5, -99.0, 4.5, -99.0)
    arc(4.5, -64.5, 34.5, 270, 360)
    line(39.0, -64.5, 39.0, 4.5)
    arc(4.5, 4.5, 34.5, 0, 90)
    line(4.5, 39.0, -32.0, 39.0)
    # Right turn into loop B at the start intersection
    arc(-32.0, 47.0, 8.0, 270, 180, TURN_SPEED)
    # Loop B, clockwise, 5 m inside the centre line
    line(-40.0, 47.0, -40.0, 64.5)
    arc(-4.5, 64.5, 35.5, 180, 90)
    line(-4.5, 100.0, 64.5, 100.0)
    arc(64.5, 64.5, 35.5, 90, 0)
    line(100.0, 64.5, 100.0, -4.5)
    arc(64.5, -4.5, 35.5, 0, -90)
    line(64.5, -40.0, -4.5, -40.0)
    arc(-4.5, -4.5, 35.5, 270, 180)
    line(-40.0, -4.5, -40.0, 36.0)
    # Left turn back onto loop A, inside the intersection (clear of the road-works cones
    # and signs at its south-west corner)
    arc(-47.0, 36.0, 7.0, 0, 90, TURN_SPEED)
    return pts


class Autopilot(Node):
    def __init__(self):
        super().__init__("autopilot")
        self.speed = float(self.declare_parameter("speed", 8.0).value)
        self.laps = int(self.declare_parameter("laps", 0).value)
        self.path = figure_eight(self.speed)
        self.idx = None
        self.travelled = 0      # path samples covered (~m)
        self.lap = 0
        self.done = False
        self.pose = None
        self.pub = self.create_publisher(AckermannDrive, "/Car_Ackrm", 10)
        self.create_subscription(Odometry, "/Car_Odom_GT", self.odom_cb, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info(
            f"Autopilot: figure-8 through the whole city ({len(self.path)} m per lap), "
            f"{self.speed} m/s, {'forever' if self.laps == 0 else f'{self.laps} lap(s)'}")

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def publish(self, speed, steer):
        msg = AckermannDrive()
        msg.speed = float(speed)
        msg.steering_angle = float(steer)
        self.pub.publish(msg)

    def dist(self, i, x, y):
        return math.hypot(self.path[i][0] - x, self.path[i][1] - y)

    def step(self):
        if self.pose is None:
            return
        if self.done:
            self.publish(0.0, 0.0)
            return
        x, y, yaw = self.pose
        n = len(self.path)
        if self.idx is None:
            # First fix: closest point that also points the way the car faces
            # (the two loops cross, so position alone is ambiguous)
            ahead = [i for i in range(n) if math.cos(self.path[i][2] - yaw) > 0.5]
            self.idx = min(ahead or range(n), key=lambda i: self.dist(i, x, y))
        else:                     # afterwards: search a little ahead only
            best = min(((self.idx + k) % n for k in range(40)), key=lambda i: self.dist(i, x, y))
            self.travelled += (best - self.idx) % n
            self.idx = best
            if self.travelled >= (self.lap + 1) * n:
                self.lap += 1
                self.get_logger().info(f"Lap {self.lap} done")
                if self.laps and self.lap >= self.laps:
                    self.done = True
                    self.publish(0.0, 0.0)
                    return

        # Slow down early for the tight turns, and look less far ahead in them
        target_speed = min(self.path[(self.idx + k) % n][3] for k in range(25))
        lookahead = max(5, min(10, int(round(1.25 * target_speed))))
        tx, ty = self.path[(self.idx + lookahead) % n][:2]

        # Pure pursuit on the rear axle (the odometry origin)
        dx, dy = tx - x, ty - y
        alpha = math.atan2(dy, dx) - yaw
        ld = max(math.hypot(dx, dy), 1e-3)
        steer = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        self.publish(target_speed, max(-MAX_STEER, min(MAX_STEER, steer)))


def main():
    run(Autopilot, on_exit=lambda node: node.publish(0.0, 0.0))   # stop the car on exit


if __name__ == "__main__":
    main()
