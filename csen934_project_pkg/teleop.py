import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDrive
from std_srvs.srv import Trigger

MAX_SPEED = 20.0      # m/s forward
MIN_SPEED = -5.0      # m/s reverse
MAX_STEER = 0.5       # rad
SPEED_STEP = 1.0      # m/s per key press
STEER_STEP = 0.05     # rad per key press

HELP = """
Teleop for /Car_Ackrm
---------------------
  w / Up      : speed +1 m/s
  s / Down    : speed -1 m/s
  a / Left    : steer left  (+0.05 rad)
  d / Right   : steer right (-0.05 rad)
  x           : center the steering
  space       : stop (speed = 0, steering = 0)
  r           : reset the car to its start pose
  q           : quit
"""


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.pub = self.create_publisher(AckermannDrive, "/Car_Ackrm", 10)
        self.reset_client = self.create_client(Trigger, "/Car_Reset")
        self.speed = 0.0
        self.steer = 0.0
        self.create_timer(0.05, self.publish_cmd)  # 20 Hz, keeps the last command alive

    def reset_car(self):
        self.speed = 0.0
        self.steer = 0.0
        if not self.reset_client.service_is_ready():
            print("\r/Car_Reset not available. Is the Webots simulation running?")
            return
        future = self.reset_client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: print(f"\r{f.result().message}" if f.result()
                                                 else "\rReset failed"))

    def publish_cmd(self):
        msg = AckermannDrive()
        msg.speed = float(self.speed)
        msg.steering_angle = float(self.steer)
        self.pub.publish(msg)

    def handle_key(self, key):
        if key in ("w", "UP"):
            self.speed = min(self.speed + SPEED_STEP, MAX_SPEED)
        elif key in ("s", "DOWN"):
            self.speed = max(self.speed - SPEED_STEP, MIN_SPEED)
        elif key in ("a", "LEFT"):
            self.steer = min(self.steer + STEER_STEP, MAX_STEER)
        elif key in ("d", "RIGHT"):
            self.steer = max(self.steer - STEER_STEP, -MAX_STEER)
        elif key == "x":
            self.steer = 0.0
        elif key == " ":
            self.speed = 0.0
            self.steer = 0.0
        elif key == "r":
            self.reset_car()


def read_key():
    """Return a key name, or None if nothing was pressed."""
    if not select.select([sys.stdin], [], [], 0)[0]:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":  # arrow keys arrive as ESC [ A/B/C/D
        if select.select([sys.stdin], [], [], 0.01)[0]:
            seq = sys.stdin.read(2)
            return {"[A": "UP", "[B": "DOWN", "[C": "RIGHT", "[D": "LEFT"}.get(seq)
        return None
    return ch


def main():
    # Keep the context alive on Ctrl+C so the final stop command can still be sent
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = TeleopNode()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            key = read_key()
            if key == "q" or key == "\x03":  # q or Ctrl+C
                break
            if key:
                node.handle_key(key)
                print(f"\rspeed = {node.speed:5.1f} m/s   steering = {node.steer:+.2f} rad   ",
                      end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # restore the terminal
        node.speed = 0.0
        node.steer = 0.0
        node.publish_cmd()  # last command: stop the car
        node.destroy_node()
        rclpy.shutdown()
        print()


if __name__ == "__main__":
    main()
