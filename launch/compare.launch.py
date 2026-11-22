"""Comparison of the saved SLAM outputs (2D grids + 3D cloud) overlaid in RViz."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "csen934_project_pkg"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package=PKG, executable="map_compare", output="screen"),
        # The saved maps are all in the odom (world) frame; give RViz that frame
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "map", "--child-frame-id", "odom"]),
        Node(package="rviz2", executable="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "compare.rviz")]),
    ])
