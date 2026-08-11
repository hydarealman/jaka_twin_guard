#!/usr/bin/env python3
"""Independent bottle-pose launch. Does not start the D455 by default."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("d455_6d_pose")
    start_camera = LaunchConfiguration("start_camera")
    output_frame = LaunchConfiguration("output_frame")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "true",
            "align_depth.enable": "true",
            "pointcloud.enable": "false",
        }.items(),
    )
    estimator = Node(
        package="d455_6d_pose",
        executable="d455_pose_node",
        name="d455_pose_node",
        parameters=[
            os.path.join(share, "config", "common.yaml"),
            os.path.join(share, "config", "bottle.yaml"),
            {"output_frame": output_frame},
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_camera",
                default_value="false",
                description="Start one RealSense driver only when no other stack owns it",
            ),
            DeclareLaunchArgument(
                "output_frame", default_value="camera_color_optical_frame"
            ),
            camera,
            estimator,
        ]
    )
