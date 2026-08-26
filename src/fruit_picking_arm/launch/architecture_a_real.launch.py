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
                FindPackageShare("fruit_picking_arm"),
                "launch",
                "architecture_a_moveit_serial.launch.py",
            ])
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "robot_serial": LaunchConfiguration("robot_serial"),
            "kinematics_mode": LaunchConfiguration("kinematics_mode"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "start_camera": LaunchConfiguration("start_camera"),
            "color_profile": LaunchConfiguration("color_profile"),
            "depth_profile": LaunchConfiguration("depth_profile"),
            "enable_temporal_filter": LaunchConfiguration("enable_temporal_filter"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "start_debug_view": LaunchConfiguration("start_debug_view"),
            "start_image_view": LaunchConfiguration("start_image_view"),
            "start_perception": LaunchConfiguration("start_perception"),
            "start_robot_stack": LaunchConfiguration("start_robot_stack"),
            "run_task": LaunchConfiguration("run_task"),
            "model_license_approved": LaunchConfiguration("model_license_approved"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument(
            "robot_serial", default_value="",
            description="Required when the physical robot stack is enabled",
        ),
        DeclareLaunchArgument("kinematics_mode", default_value="nominal"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("color_profile", default_value="424,240,15"),
        DeclareLaunchArgument("depth_profile", default_value="424,240,15"),
        DeclareLaunchArgument("enable_temporal_filter", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_debug_view", default_value="true"),
        DeclareLaunchArgument("start_image_view", default_value="true"),
        DeclareLaunchArgument("start_perception", default_value="false"),
        DeclareLaunchArgument(
            "start_robot_stack",
            default_value="false",
            description="Safe default: no MoveIt or serial controller during perception bring-up",
        ),
        DeclareLaunchArgument(
            "run_task", default_value="false",
            description="Safe default: only start perception and hardware bring-up",
        ),
        DeclareLaunchArgument("model_license_approved", default_value="false"),
        LogInfo(msg="============================================================"),
        LogInfo(msg="[MODE] Architecture A / REAL HARDWARE"),
        LogInfo(msg=["[DANGER] Physical serial device: ", LaunchConfiguration("serial_port")]),
        LogInfo(msg="[FLOW] RealSense -> perception -> MoveIt -> trajectory serial -> C board"),
        LogInfo(msg=["[KINEMATICS] requested mode: ", LaunchConfiguration("kinematics_mode")]),
        LogInfo(msg="[CHECK] E-stop, joint limits and hand-eye calibration are required"),
        LogInfo(msg="============================================================"),
        real_stack,
    ])
