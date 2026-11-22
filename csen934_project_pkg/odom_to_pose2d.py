"""Republishes x, y, yaw of the three odometry topics as geometry_msgs/Pose2D for rqt_plot.

  /Car_Odom_GT       -> /Car_Pose2D_GT
  /Car_Odom_Noisy    -> /Car_Pose2D_Noisy
  /Car_Odom_Filtered -> /Car_Pose2D_Filtered

theta is the yaw in rad, unwrapped (no +-pi jumps) and kept on the same 2*pi branch as the
ground truth so the three curves overlap.
"""
import math

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.node import Node

from csen934_project_pkg.node_runner import run

SOURCES = ("GT", "Noisy", "Filtered")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def closest_branch(angle, reference):
    """angle + k*2pi closest to reference."""
    return angle + 2.0 * math.pi * round((reference - angle) / (2.0 * math.pi))


class OdomToPose2D(Node):
    def __init__(self):
        super().__init__("odom_to_pose2d")
        self.last = {}          # unwrapped yaw per source
        self.pubs = {}
        for name in SOURCES:
            self.pubs[name] = self.create_publisher(Pose2D, f"/Car_Pose2D_{name}", 10)
            self.create_subscription(Odometry, f"/Car_Odom_{name}",
                                     lambda msg, n=name: self.callback(n, msg), 10)

    def callback(self, name, msg):
        yaw = yaw_of(msg.pose.pose.orientation)
        reference = self.last.get("GT", self.last.get(name))
        if reference is not None:
            yaw = closest_branch(yaw, reference)
        self.last[name] = yaw
        out = Pose2D()
        out.x = msg.pose.pose.position.x
        out.y = msg.pose.pose.position.y
        out.theta = yaw
        self.pubs[name].publish(out)


def main():
    run(OdomToPose2D)


if __name__ == "__main__":
    main()
