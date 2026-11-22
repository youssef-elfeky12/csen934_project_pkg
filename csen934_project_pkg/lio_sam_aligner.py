"""Static TF odom -> lio_map, so LIO-SAM's output lines up with the ground truth.

LIO-SAM starts its map frame (lio_map) at the lidar's first pose.  When LIO-SAM publishes its
first pose estimate, this node takes the ground-truth lidar pose at that same instant and
publishes odom -> lio_map = T(odom, lidar) * T(lio_map, lidar)^-1.
"""
from collections import deque

import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster

from csen934_project_pkg.node_runner import run

LIDAR_OFFSET = np.array([1.0, 0.0, 1.70])   # lidar_3d in base_link (see the controller)


def quat_to_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(m):
    w = np.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 1e-12)) / 2.0
    return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w),
            (m[1, 0] - m[0, 1]) / (4 * w), w)


def pose_of(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return np.array([p.x, p.y, p.z]), quat_to_matrix((q.x, q.y, q.z, q.w))


def stamp_of(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class LioSamAligner(Node):
    def __init__(self):
        super().__init__("lio_sam_aligner")
        self.gt = deque(maxlen=1000)
        self.done = False
        self.br = StaticTransformBroadcaster(self)
        self.create_subscription(Odometry, "/Car_Odom_GT", lambda m: self.gt.append(m), 50)
        # LIO-SAM publishes its odometry best-effort
        self.create_subscription(Odometry, "/lio_sam/mapping/odometry", self.on_lio,
                                 qos_profile_sensor_data)

    def on_lio(self, msg):
        if self.done or not self.gt:
            return
        t = stamp_of(msg)
        gt = min(self.gt, key=lambda m: abs(stamp_of(m) - t))
        p_b, R_b = pose_of(gt)
        p_w, R_w = p_b + R_b @ LIDAR_OFFSET, R_b           # lidar in odom (world)
        p_l, R_l = pose_of(msg)                             # lidar in lio_map
        R = R_w @ R_l.T
        p = p_w - R @ p_l

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "lio_map"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = matrix_to_quat(R)
        self.br.sendTransform(tf)
        self.done = True
        self.get_logger().info(f"odom -> lio_map set at x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}")


def main():
    run(LioSamAligner)


if __name__ == "__main__":
    main()
