"""Broadcasts the odom -> base_link TF from an odometry topic (default: the EKF output).

The 2D SLAM packages need this transform as their odometry prior.
"""
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from csen934_project_pkg.node_runner import run


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster")
        topic = self.declare_parameter("odom_topic", "/Car_Odom_Filtered").value
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, topic, self.callback, 50)
        self.get_logger().info(f"TF odom -> base_link from {topic}")

    def callback(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id or "base_link"
        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    run(OdomTfBroadcaster)


if __name__ == "__main__":
    main()
