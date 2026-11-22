"""Shared main loop: spin a node and exit cleanly on Ctrl+C (no tracebacks)."""
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def run(node_class, on_exit=None):
    """Spin node_class() until Ctrl+C / SIGTERM, then call on_exit(node) (e.g. stop the car).

    rclpy's own signal handling is disabled so the ROS context is still alive in on_exit,
    and Python's handler is installed explicitly (a process started in the background
    inherits SIGINT as "ignored").  Spinning in short slices lets the handler run even
    when no messages arrive.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    node = node_class()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if on_exit is not None and rclpy.ok():
            on_exit(node)
        node.destroy_node()
        rclpy.try_shutdown()
