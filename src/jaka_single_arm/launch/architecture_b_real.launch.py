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
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        LogInfo(msg="============================================================"),
        LogInfo(msg="[MODE] Architecture B / REAL HARDWARE"),
        LogInfo(msg=["[DANGER] Physical serial device: ", LaunchConfiguration("serial_port")]),
        LogInfo(msg="[FLOW] RealSense -> stable FruitTarget -> target serial -> C board"),
        LogInfo(msg="[CHECK] C-board IK, limits, workspace checks and E-stop are required"),
        LogInfo(msg="============================================================"),
        real_stack,
    ])

