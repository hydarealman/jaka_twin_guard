#!/usr/bin/env python3
"""D455 RGB-D and ROI fruit quality debug, without robot motion or serial."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_camera = LaunchConfiguration("start_camera")
    enable_depth = LaunchConfiguration("enable_depth")
    show_image = LaunchConfiguration("show_image")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("jaka_single_arm"), "launch", "d455_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_depth": enable_depth,
            "enable_pointcloud": enable_depth,
        }.items(),
    )
    detector = Node(
        package="jaka_single_arm",
        executable="fruit_target_node",
        name="fruit_quality_debug_node",
        parameters=[{
            "camera_type": "realsense",
            # Debug before hand-eye calibration: keep geometry and projection
            # in the D455 optical frame. Real A/B launches use the robot frame.
            "output_frame": "camera_color_optical_frame",
            "allow_scene_fallback": False,
            "force_table_center_z": False,
            "enable_table_z_fallback": False,
            "perception_license_mode": "development",
            "model_license_approved": False,
        }],
        output="screen",
    )
    viewer = Node(
        package="image_view",
        executable="image_view",
        name="fruit_debug_view",
        condition=IfCondition(show_image),
        remappings=[("image", "/perception/health_annotated")],
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="true",
                description="Must be true for ROI localisation; use native Ubuntu",
            ),
            DeclareLaunchArgument("show_image", default_value="true"),
            LogInfo(msg="[MODE] D455 RGB-D ROI fruit-quality DEBUG; no robot motion"),
            camera,
            detector,
            viewer,
        ]
    )
