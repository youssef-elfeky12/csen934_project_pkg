"""2D SLAM trial 2: Cartographer 2D + 3D lidar flattened by pointcloud_to_laserscan, with RViz."""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

PKG = "csen934_project_pkg"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    sim = {"use_sim_time": True}
    odom = LaunchConfiguration("odom")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("odom", default_value="/Car_Odom_Filtered",
                              description="odometry used by Cartographer and for the TF"),
        Node(package=PKG, executable="odom_tf_broadcaster",
             parameters=[{"odom_topic": odom}, sim]),
        # PointCloud2 (16 layers) -> LaserScan: keep points 0.2-2.0 m above base_link
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
             remappings=[("cloud_in", "/Car_PointCloud"), ("scan", "/Car_Scan_3D")],
             parameters=[{
                 "target_frame": "base_link", "transform_tolerance": 0.05,
                 "min_height": 0.2, "max_height": 2.0,
                 "angle_min": -math.pi, "angle_max": math.pi,
                 "angle_increment": 2 * math.pi / 900, "scan_time": 0.1,
                 "range_min": 2.0, "range_max": 100.0, "use_inf": True,
             }, sim]),
        Node(package="cartographer_ros", executable="cartographer_node", output="screen",
             arguments=["-configuration_directory", os.path.join(share, "config"),
                        "-configuration_basename", "cartographer_2d.lua"],
             remappings=[("scan", "/Car_Scan_3D"), ("odom", odom)], parameters=[sim]),
        Node(package="cartographer_ros", executable="cartographer_occupancy_grid_node",
             arguments=["-resolution", "0.1", "-publish_period_sec", "1.0"], parameters=[sim]),
        Node(package=PKG, executable="trajectory_publisher",
             parameters=[{"slam_map_frame": "map", "slam_base_frame": "base_link"}, sim]),
        Node(package="rviz2", executable="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")), parameters=[sim],
             arguments=["-d", os.path.join(share, "rviz", "slam_2d.rviz")]),
    ])
