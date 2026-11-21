import copy
import math
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Standard deviations from the milestone specification
STD_XY = 0.3                    # m      (x and y position)
STD_YAW = math.radians(1.0)     # rad    (1 degree)
STD_LIN = 0.3                   # m/s    (linear speed)
STD_ANG = math.radians(1.0)     # rad/s  (1 degree/s)

# Covariance matrices are 6x6, stored row by row as 36 numbers.
# The variance of variable i sits at index 7*i (the diagonal).
# Pose order:  x, y, z, roll, pitch, yaw
# Twist order: vx, vy, vz, wx, wy, wz
POSE_COV = [0.0] * 36
POSE_COV[0] = STD_XY ** 2       # x
POSE_COV[7] = STD_XY ** 2       # y
POSE_COV[35] = STD_YAW ** 2     # yaw

TWIST_COV = [0.0] * 36
TWIST_COV[0] = STD_LIN ** 2     # vx
TWIST_COV[7] = STD_LIN ** 2     # vy
TWIST_COV[35] = STD_ANG ** 2    # wz (yaw rate)


def euler_to_quaternion(roll, pitch, yaw):
    """Euler angles (rad) -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quaternion_to_euler(x, y, z, w):
    """Quaternion -> Euler angles (roll, pitch, yaw) in rad."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class NoiseNode(Node):
    def __init__(self):
        super().__init__("localization_add_noise_node")
        self.pub = self.create_publisher(Odometry, "/Car_Odom_Noisy", 10)
        self.create_subscription(Odometry, "/Car_Odom_GT", self.callback, 10)
        self.get_logger().info("Adding noise: /Car_Odom_GT -> /Car_Odom_Noisy")

    def callback(self, gt):
        noisy = copy.deepcopy(gt)  # keeps header, frame ids, z and everything we don't touch

        # Position: noise on x and y
        noisy.pose.pose.position.x += random.gauss(0.0, STD_XY)
        noisy.pose.pose.position.y += random.gauss(0.0, STD_XY)

        # Orientation: quaternion -> Euler, noise on yaw only, back to quaternion
        q = gt.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        yaw += random.gauss(0.0, STD_YAW)
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        noisy.pose.pose.orientation.x = qx
        noisy.pose.pose.orientation.y = qy
        noisy.pose.pose.orientation.z = qz
        noisy.pose.pose.orientation.w = qw

        # Velocities: noise on linear x/y and on the yaw rate
        noisy.twist.twist.linear.x += random.gauss(0.0, STD_LIN)
        noisy.twist.twist.linear.y += random.gauss(0.0, STD_LIN)
        noisy.twist.twist.angular.z += random.gauss(0.0, STD_ANG)

        # Covariances now describe the noise we added
        noisy.pose.covariance = POSE_COV
        noisy.twist.covariance = TWIST_COV

        self.pub.publish(noisy)


def main():
    rclpy.init()
    node = NoiseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
