import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDrive


class ControlNode(Node):
    """Open-loop controller: publishes a constant speed / steering command."""

    def __init__(self):
        super().__init__("control_node")
        # Override at launch, e.g. --ros-args -p speed:=5.0 -p steering_angle:=0.1
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
    # Keep the context alive on Ctrl+C so the final stop command can still be sent
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)  # stop the car on exit
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
