import copy
import math

import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

try:
    from ackermann_msgs.msg import AckermannDrive
except ModuleNotFoundError:
    AckermannDrive = None


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return 0.0, 0.0, qz, qw


class BicycleEKF:
    """5-state bicycle-model EKF: [x, y, yaw, speed, yaw_rate]."""

    def __init__(self, wheelbase=2.7):
        self.wheelbase = wheelbase
        self.state = np.zeros(5, dtype=float)
        self.covariance = np.diag([0.25, 0.25, 0.01, 0.09, 0.01])
        self.initialized = False

        self.measurement_covariance = np.diag(
            [0.09, 0.09, math.radians(1.0) ** 2,
             0.09, math.radians(1.0) ** 2]
        )
        self.process_covariance = np.diag(
            [0.02, 0.02, math.radians(0.5) ** 2,
             0.05, math.radians(0.5) ** 2]
        )

    def initialize(self, x, y, yaw, speed=0.0, yaw_rate=0.0):
        self.state = np.array([x, y, normalize_angle(yaw),
                               speed, yaw_rate], dtype=float)
        self.covariance = np.diag([0.09, 0.09,
                                   math.radians(1.0) ** 2,
                                   0.09, math.radians(1.0) ** 2])
        self.initialized = True

    def predict(self, dt, speed, steering_angle):
        dt = max(0.0, min(float(dt), 1.0))
        if dt == 0.0:
            return

        x, y, yaw, _, _ = self.state
        speed = float(speed)
        yaw_rate = speed * math.tan(float(steering_angle)) / self.wheelbase

        self.state[0] = x + speed * math.cos(yaw) * dt
        self.state[1] = y + speed * math.sin(yaw) * dt
        self.state[2] = normalize_angle(yaw + yaw_rate * dt)
        self.state[3] = speed
        self.state[4] = yaw_rate

        jacobian = np.eye(5)
        jacobian[0, 2] = -speed * math.sin(yaw) * dt
        jacobian[0, 3] = math.cos(yaw) * dt
        jacobian[1, 2] = speed * math.cos(yaw) * dt
        jacobian[1, 3] = math.sin(yaw) * dt
        jacobian[2, 4] = dt

        q = self.process_covariance * dt
        self.covariance = (
            jacobian @ self.covariance @ jacobian.T + q
        )

    def update(self, x, y, yaw, speed, yaw_rate):
        measurement = np.array(
            [x, y, normalize_angle(yaw), speed, yaw_rate],
            dtype=float,
        )
        residual = measurement - self.state
        residual[2] = normalize_angle(residual[2])

        innovation = (
            self.covariance
            + self.measurement_covariance
        )
        kalman_gain = np.linalg.solve(
            innovation,
            self.covariance,
        ).T

        self.state = self.state + kalman_gain @ residual
        self.state[2] = normalize_angle(self.state[2])
        identity = np.eye(5)
        self.covariance = (
            (identity - kalman_gain) @ self.covariance
            @ (identity - kalman_gain).T
            + kalman_gain @ self.measurement_covariance
            @ kalman_gain.T
        )


class Localization_EKF_Node(Node):
    def __init__(self):
        super().__init__("Localization_EKF_Node")

        wheelbase = float(
            self.declare_parameter("wheelbase", 2.7).value
        )
        self.ekf = BicycleEKF(wheelbase=wheelbase)
        self.speed_command = 0.0
        self.steering_command = 0.0
        self.last_odom_stamp = None

        self.filtered_publisher = self.create_publisher(
            Odometry, "/Car_Odom_Filtered", 10
        )
        self.create_subscription(
            Odometry,
            "/Car_Odom_Noisy",
            self.odometry_callback,
            10,
        )
        self.create_subscription(
            AckermannDrive,
            "/Car_Ackrm",
            self.control_callback,
            10,
        )
        self.get_logger().info(
            "EKF localization: /Car_Odom_Noisy + /Car_Ackrm "
            "-> /Car_Odom_Filtered"
        )

    def control_callback(self, msg):
        self.speed_command = float(msg.speed)
        self.steering_command = float(msg.steering_angle)

    def odometry_callback(self, noisy):
        stamp_seconds = (
            noisy.header.stamp.sec
            + noisy.header.stamp.nanosec * 1e-9
        )

        x = noisy.pose.pose.position.x
        y = noisy.pose.pose.position.y
        yaw = quaternion_to_yaw(noisy.pose.pose.orientation)
        measured_speed = noisy.twist.twist.linear.x
        measured_yaw_rate = noisy.twist.twist.angular.z

        if not self.ekf.initialized:
            self.ekf.initialize(
                x, y, yaw, measured_speed, measured_yaw_rate
            )
            dt = 0.0
        else:
            if self.last_odom_stamp is None:
                dt = 0.0
            else:
                dt = max(0.0, stamp_seconds - self.last_odom_stamp)
            self.ekf.predict(
                dt, self.speed_command, self.steering_command
            )
            self.ekf.update(
                x, y, yaw, measured_speed, measured_yaw_rate
            )

        self.last_odom_stamp = stamp_seconds
        self.publish_filtered(noisy.header)

    def publish_filtered(self, source_header):
        filtered = Odometry()
        filtered.header = copy.deepcopy(source_header)
        filtered.header.frame_id = (
            source_header.frame_id or "odom"
        )
        filtered.child_frame_id = "base_link"

        x, y, yaw, speed, yaw_rate = self.ekf.state
        filtered.pose.pose.position.x = float(x)
        filtered.pose.pose.position.y = float(y)
        filtered.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        filtered.pose.pose.orientation.x = qx
        filtered.pose.pose.orientation.y = qy
        filtered.pose.pose.orientation.z = qz
        filtered.pose.pose.orientation.w = qw

        filtered.twist.twist.linear.x = float(speed)
        filtered.twist.twist.linear.y = 0.0
        filtered.twist.twist.linear.z = 0.0
        filtered.twist.twist.angular.x = 0.0
        filtered.twist.twist.angular.y = 0.0
        filtered.twist.twist.angular.z = float(yaw_rate)

        pose_covariance = [0.0] * 36
        pose_covariance[0] = float(self.ekf.covariance[0, 0])
        pose_covariance[7] = float(self.ekf.covariance[1, 1])
        pose_covariance[35] = float(self.ekf.covariance[2, 2])
        filtered.pose.covariance = pose_covariance

        twist_covariance = [0.0] * 36
        twist_covariance[0] = float(self.ekf.covariance[3, 3])
        twist_covariance[35] = float(self.ekf.covariance[4, 4])
        filtered.twist.covariance = twist_covariance

        self.filtered_publisher.publish(filtered)


def main():
    rclpy.init()
    node = Localization_EKF_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()