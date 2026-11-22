"""rqt_plot of x, y and yaw over time: ground truth vs noisy vs filtered (one window each)."""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

PKG = "csen934_project_pkg"


def plot(field):
    return Node(package="rqt_plot", executable="rqt_plot",
                arguments=[f"/Car_Pose2D_{src}/{field}" for src in ("GT", "Noisy", "Filtered")])


def generate_launch_description():
    return LaunchDescription([
        Node(package=PKG, executable="odom_to_pose2d"),
        # rqt_plot can only add topics it already sees: give odom_to_pose2d time to advertise
        TimerAction(period=3.0, actions=[plot("x"), plot("y"), plot("theta")]),
    ])
