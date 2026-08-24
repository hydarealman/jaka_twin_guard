#!/usr/bin/env python3
"""Start only the tuned D455 driver used by fruit perception."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("fruit_picking_arm")
    config = os.path.join(share, "config", "d455_fruit_camera.yaml")

    serial_no = LaunchConfiguration("serial_no")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    enable_depth = LaunchConfiguration("enable_depth")
    enable_temporal_filter = LaunchConfiguration("enable_temporal_filter")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        parameters=[
            config,
            {
                "serial_no": ParameterValue(serial_no, value_type=str),
                "rgb_camera.color_profile": ParameterValue(
                    color_profile, value_type=str
                ),
                "depth_module.depth_profile": ParameterValue(
                    depth_profile, value_type=str
                ),
                "enable_depth": ParameterValue(enable_depth, value_type=bool),
                "align_depth.enable": ParameterValue(enable_depth, value_type=bool),
                "pointcloud.enable": ParameterValue(
                    enable_pointcloud, value_type=bool
                ),
                "spatial_filter.enable": ParameterValue(
                    enable_depth, value_type=bool
                ),
                "temporal_filter.enable": ParameterValue(
                    enable_temporal_filter, value_type=bool
                ),
            },
        ],
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription(
        [
            # Empty means use the first available RealSense device. A fixed
            # serial made a different D455 wait forever with no image topics.
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("color_profile", default_value="424,240,15"),
            DeclareLaunchArgument(
                "depth_profile",
                default_value="424,240,15",
                description="D455 RGB-D profile targeting 15 Hz on WSL2/usbipd",
            ),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="true",
                description="Disable only for the WSL colour-only diagnostic path",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description="Optional driver cloud; real perception derives a compact cloud from measured depth",
            ),
            DeclareLaunchArgument(
                "enable_temporal_filter",
                default_value="false",
                description="Keep disabled for moving fruit; temporal history can trail RGB-D motion",
            ),
            LogInfo(
                msg="[D455] RGB/depth profiles are launch-configurable; real perception uses measured depth and a compact derived XYZ cloud"
            ),
            camera,
        ]
    )
