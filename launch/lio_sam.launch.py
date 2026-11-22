"""3D SLAM trial: LIO-SAM (16-layer lidar + IMU), with RViz (3D map, trajectories)."""
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
    params = os.path.join(share, "config", "lio_sam.yaml")
    sim = {"use_sim_time": True}
    lio_nodes = [
        Node(package="lio_sam", executable=f"lio_sam_{name}", name=f"lio_sam_{name}",
             output="screen", parameters=[params, sim])
        for name in ("imuPreintegration", "imageProjection", "featureExtraction",
                     "mapOptimization")
    ]
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
             arguments=["--frame-id", "lio_map", "--child-frame-id", "lio_odom"]),
        # Lidar mount on the car: LIO-SAM tracks lio_lidar and publishes lio_odom -> lio_base_link
        Node(package="tf2_ros", executable="static_transform_publisher", parameters=[sim],
             arguments=["--x", "1.0", "--z", "1.70",
                        "--frame-id", "lio_base_link", "--child-frame-id", "lio_lidar"]),
        *lio_nodes,
        Node(package=PKG, executable="lio_sam_aligner", parameters=[sim]),
        Node(package=PKG, executable="trajectory_publisher",
             parameters=[{"slam_map_frame": "lio_map", "slam_base_frame": "lio_base_link"}, sim]),
        Node(package="rviz2", executable="rviz2",
             condition=IfCondition(LaunchConfiguration("rviz")), parameters=[sim],
             arguments=["-d", os.path.join(share, "rviz", "lio_sam.rviz")]),
    ])
