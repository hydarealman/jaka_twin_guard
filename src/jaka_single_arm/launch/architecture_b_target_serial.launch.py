#!/usr/bin/env python3
"""Architecture B — RealSense vision sends fruit targets directly to C board.

This launch deliberately contains no MoveIt, planner, behaviour tree,
ros2_control or trajectory controller.  Firmware owns the entire grasp and
motion sequence after accepting a FruitTarget.
"""

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
    port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    start_camera = LaunchConfiguration("start_camera")
    model_license_approved = LaunchConfiguration("model_license_approved")
    jaka_share = get_package_share_directory("jaka_single_arm")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("jaka_single_arm"), "launch", "d455_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_depth": "true",
            "enable_pointcloud": "true",
        }.items(),
    )
    hand_eye = Node(
        package="jaka_single_arm",
        executable="hand_eye_static_tf",
        parameters=[os.path.join(jaka_share, "config", "hand_eye_params.yaml")],
        output="screen",
    )
    localizer = Node(
        package="jaka_single_arm",
        executable="fruit_target_node",
        parameters=[{
            "camera_type": "realsense",
            "output_frame": "base_link",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "allow_scene_fallback": False,
            "force_table_center_z": False,
            "enable_table_z_fallback": False,
            "perception_license_mode": "production",
            "model_license_approved": model_license_approved,
        }],
        output="screen",
    )
    target_bridge = Node(
        package="jaka_single_arm",
        executable="serial_fruit_target_bridge",
        parameters=[
            os.path.join(jaka_share, "config", "architecture_b_serial.yaml"),
            {"serial_port": port, "baudrate": baudrate},
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument(
            "model_license_approved", default_value="false",
            description="Set true only after the model/data license audit is approved",
        ),
        camera,
        hand_eye,
        localizer,
        target_bridge,
    ])
