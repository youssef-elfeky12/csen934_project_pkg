import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")
        self.pub = self.create_publisher(AckermannDrive, "/Car_Ackrm", 10)
        self.create_timer(0.05, self.publish_cmd)  # 20 Hz

    def publish_cmd(self):
        msg = AckermannDrive()
        msg.speed = 10.0           # m/s
        msg.steering_angle = 0.0   # rad
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
