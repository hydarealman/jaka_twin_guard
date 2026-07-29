#!/usr/bin/env python3
"""Architecture A real hardware: MoveIt trajectories are sent to the C board."""

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
                "architecture_a_moveit_serial.launch.py",
            ])
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "start_camera": LaunchConfiguration("start_camera"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "model_license_approved": LaunchConfiguration("model_license_approved"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("model_license_approved", default_value="false"),
        LogInfo(msg="============================================================"),
        LogInfo(msg="[MODE] Architecture A / REAL HARDWARE"),
        LogInfo(msg=["[DANGER] Physical serial device: ", LaunchConfiguration("serial_port")]),
        LogInfo(msg="[FLOW] RealSense -> perception -> MoveIt -> trajectory serial -> C board"),
        LogInfo(msg="[CHECK] E-stop, joint limits and hand-eye calibration are required"),
        LogInfo(msg="============================================================"),
        real_stack,
    ])
