"""Control node: publishes a constant AckermannDrive command on /Car_Ackrm.

Defaults are the milestone values: speed = 10 m/s, steering angle = 0 rad (straight line).
Override with --ros-args -p speed:=5.0 -p steering_angle:=0.1
"""
from ackermann_msgs.msg import AckermannDrive
from rclpy.node import Node

from csen934_project_pkg.node_runner import run


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.speed = float(self.declare_parameter("speed", 10.0).value)          # m/s
        self.steer = float(self.declare_parameter("steering_angle", 0.0).value)  # rad, +left
        self.pub = self.create_publisher(AckermannDrive, "/Car_Ackrm", 10)
        self.create_timer(0.05, self.publish_cmd)  # 20 Hz
        self.get_logger().info(
            f"Publishing constant command: speed={self.speed} m/s, steering={self.steer} rad")

    def publish_cmd(self, speed=None, steer=None):
        msg = AckermannDrive()
        msg.speed = self.speed if speed is None else speed
        msg.steering_angle = self.steer if steer is None else steer
        self.pub.publish(msg)


def main():
    run(ControlNode, on_exit=lambda node: node.publish_cmd(0.0, 0.0))   # stop the car on exit


if __name__ == "__main__":
    main()
