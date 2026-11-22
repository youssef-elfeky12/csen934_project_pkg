"""EKF localization: fuses noisy odometry with the Ackermann control input.

Node "Localization_EKF_Node"
  subscribes  /Car_Odom_Noisy  nav_msgs/Odometry               measurement z
              /Car_Ackrm       ackermann_msgs/AckermannDrive   control input u
  publishes   /Car_Odom_Filtered nav_msgs/Odometry             filtered estimate

State  s = [x, y, yaw, v, w]   position (m), heading (rad), forward speed (m/s), yaw rate (rad/s)
Input  u = [v_cmd, delta]      commanded speed and steering angle

Prediction (kinematic bicycle model, wheelbase L):
    x   <- x + v cos(yaw) dt
    y   <- y + v sin(yaw) dt
    yaw <- yaw + w dt
    v   <- v + (v_cmd - v) dt / tau_v              the car's speed controller tracks v_cmd
    w   <- w + (v tan(delta) / L - w) dt / tau_w   yaw rate follows the steering geometry
Update: z = [x, y, yaw, v, w] measured directly (H = I), with R taken from the
covariance fields of /Car_Odom_Noisy.
"""
import math

import numpy as np
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from rclpy.node import Node

from csen934_project_pkg.node_runner import run

# Fallback measurement std devs (same as the noise node) if a message has no covariance
DEFAULT_R = np.diag([0.3 ** 2, 0.3 ** 2, math.radians(1.0) ** 2,
                     0.3 ** 2, math.radians(1.0) ** 2])


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class BicycleEKF:
    """5-state EKF on a kinematic bicycle model: [x, y, yaw, speed, yaw_rate]."""

    def __init__(self, wheelbase=2.995, tau_v=1.0, tau_w=0.3):
        self.wheelbase = wheelbase
        self.tau_v = tau_v
        self.tau_w = tau_w
        self.state = np.zeros(5)
        self.covariance = DEFAULT_R.copy()
        self.initialized = False
        # Process noise spectral density (per second): how much the real car may deviate
        # from the model (speed-controller lag, slip, unmodelled steering dynamics)
        self.process_noise = np.diag([0.005, 0.005, 1e-5, 2.0, 0.05])

    def initialize(self, x, y, yaw, speed=0.0, yaw_rate=0.0):
        self.state = np.array([x, y, normalize_angle(yaw), speed, yaw_rate], dtype=float)
        self.covariance = DEFAULT_R.copy()
        self.initialized = True

    def predict(self, dt, speed_command, steering_angle):
        dt = min(max(float(dt), 0.0), 0.5)
        if not self.initialized or dt == 0.0:
            return
        x, y, yaw, v, w = self.state
        c, s = math.cos(yaw), math.sin(yaw)
        k = math.tan(float(steering_angle)) / self.wheelbase   # curvature
        av = min(dt / self.tau_v, 1.0)
        aw = min(dt / self.tau_w, 1.0)

        self.state = np.array([
            x + v * c * dt,
            y + v * s * dt,
            normalize_angle(yaw + w * dt),
            v + (float(speed_command) - v) * av,
            w + (v * k - w) * aw,
        ])

        F = np.eye(5)                      # Jacobian of the motion model
        F[0, 2] = -v * s * dt
        F[0, 3] = c * dt
        F[1, 2] = v * c * dt
        F[1, 3] = s * dt
        F[2, 4] = dt
        F[3, 3] = 1.0 - av
        F[4, 3] = k * aw
        F[4, 4] = 1.0 - aw
        self.covariance = F @ self.covariance @ F.T + self.process_noise * dt

    def update(self, x, y, yaw, speed, yaw_rate, R=None):
        R = DEFAULT_R if R is None else R
        z = np.array([x, y, normalize_angle(yaw), speed, yaw_rate], dtype=float)
        residual = z - self.state                      # H = I
        residual[2] = normalize_angle(residual[2])
        # Something the model cannot predict (e.g. hitting a barrier and spinning) shows up
        # as an innovation far outside its expected spread: widen that state's uncertainty
        # so the filter re-syncs with the measurements instead of lagging behind them.
        spread = np.sqrt(np.diag(self.covariance) + np.diag(R))
        outlier = np.abs(residual) > 5.0 * spread
        if outlier.any():
            self.covariance[outlier, outlier] += residual[outlier] ** 2
        S = self.covariance + R
        K = self.covariance @ np.linalg.inv(S)
        self.state = self.state + K @ residual
        self.state[2] = normalize_angle(self.state[2])
        I_K = np.eye(5) - K
        self.covariance = I_K @ self.covariance @ I_K.T + K @ R @ K.T   # Joseph form


def measurement_covariance(msg):
    """R from the covariance fields of the noisy odometry (diagonal entries)."""
    p, t = msg.pose.covariance, msg.twist.covariance
    diag = [p[0], p[7], p[35], t[0], t[35]]
    if all(d > 0.0 for d in diag):
        return np.diag(diag)
    return DEFAULT_R


class Localization_EKF_Node(Node):
    def __init__(self):
        super().__init__("Localization_EKF_Node")
        self.ekf = BicycleEKF(wheelbase=float(self.declare_parameter("wheelbase", 2.995).value))
        self.speed_command = 0.0
        self.steering_command = 0.0
        self.last_stamp = None

        self.filtered_publisher = self.create_publisher(Odometry, "/Car_Odom_Filtered", 10)
        self.create_subscription(Odometry, "/Car_Odom_Noisy", self.odometry_callback, 10)
        self.create_subscription(AckermannDrive, "/Car_Ackrm", self.control_callback, 10)
        self.get_logger().info("EKF: /Car_Odom_Noisy + /Car_Ackrm -> /Car_Odom_Filtered")

    def control_callback(self, msg):
        self.speed_command = float(msg.speed)
        self.steering_command = float(msg.steering_angle)

    def odometry_callback(self, noisy):
        stamp = noisy.header.stamp.sec + noisy.header.stamp.nanosec * 1e-9
        x = noisy.pose.pose.position.x
        y = noisy.pose.pose.position.y
        yaw = quaternion_to_yaw(noisy.pose.pose.orientation)
        speed = noisy.twist.twist.linear.x          # forward speed (twist is in base_link)
        yaw_rate = noisy.twist.twist.angular.z

        jumped = self.ekf.initialized and math.hypot(x - self.ekf.state[0],
                                                     y - self.ekf.state[1]) > 10.0
        if not self.ekf.initialized or jumped or self.last_stamp is None or stamp < self.last_stamp:
            self.ekf.initialize(x, y, yaw, speed, yaw_rate)   # first message, or car was reset
        else:
            self.ekf.predict(stamp - self.last_stamp, self.speed_command, self.steering_command)
            self.ekf.update(x, y, yaw, speed, yaw_rate, measurement_covariance(noisy))
        self.last_stamp = stamp
        self.publish_filtered(noisy.header)

    def publish_filtered(self, header):
        msg = Odometry()
        msg.header.stamp = header.stamp
        msg.header.frame_id = header.frame_id or "odom"
        msg.child_frame_id = "base_link"

        x, y, yaw, speed, yaw_rate = (float(v) for v in self.ekf.state)
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = yaw_to_quaternion(yaw)
        msg.twist.twist.linear.x = speed
        msg.twist.twist.angular.z = yaw_rate

        P = self.ekf.covariance
        pose_cov = [0.0] * 36           # order: x y z roll pitch yaw
        for i, a in ((0, 0), (1, 1), (5, 2)):
            for j, b in ((0, 0), (1, 1), (5, 2)):
                pose_cov[6 * i + j] = float(P[a, b])
        twist_cov = [0.0] * 36          # order: vx vy vz wx wy wz
        twist_cov[0] = float(P[3, 3])
        twist_cov[35] = float(P[4, 4])
        twist_cov[5] = twist_cov[30] = float(P[3, 4])
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.filtered_publisher.publish(msg)


def main():
    run(Localization_EKF_Node)


if __name__ == "__main__":
    main()
