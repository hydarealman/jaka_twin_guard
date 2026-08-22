#!/usr/bin/env python3
"""Architecture B real hardware: stable fruit targets are sent to the C board."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jaka_single_arm"),
                "launch",
                "architecture_b_target_serial.launch.py",
            ])
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "start_camera": LaunchConfiguration("start_camera"),
            "color_profile": LaunchConfiguration("color_profile"),
            "depth_profile": LaunchConfiguration("depth_profile"),
            "enable_temporal_filter": LaunchConfiguration("enable_temporal_filter"),
            "start_debug_view": LaunchConfiguration("start_debug_view"),
            "start_image_view": LaunchConfiguration("start_image_view"),
            "start_target_bridge": LaunchConfiguration("start_target_bridge"),
            "model_license_approved": LaunchConfiguration("model_license_approved"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("color_profile", default_value="424,240,15"),
        DeclareLaunchArgument("depth_profile", default_value="424,240,15"),
        DeclareLaunchArgument("enable_temporal_filter", default_value="false"),
        DeclareLaunchArgument("start_debug_view", default_value="true"),
        DeclareLaunchArgument("start_image_view", default_value="true"),
        DeclareLaunchArgument("start_target_bridge", default_value="true"),
        DeclareLaunchArgument("model_license_approved", default_value="false"),
        LogInfo(msg="============================================================"),
        LogInfo(msg="[MODE] Architecture B / REAL HARDWARE"),
        LogInfo(msg=["[DANGER] Physical serial device: ", LaunchConfiguration("serial_port")]),
        LogInfo(msg="[FLOW] RealSense -> stable FruitTarget -> target serial -> C board"),
        LogInfo(msg="[CHECK] C-board IK, limits, workspace checks and E-stop are required"),
        LogInfo(msg="============================================================"),
        real_stack,
    ])
