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
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    enable_temporal_filter = LaunchConfiguration("enable_temporal_filter")
    start_debug_view = LaunchConfiguration("start_debug_view")
    start_image_view = LaunchConfiguration("start_image_view")
    start_target_bridge = LaunchConfiguration("start_target_bridge")
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
            "enable_pointcloud": "false",
            "color_profile": color_profile,
            "depth_profile": depth_profile,
            "enable_temporal_filter": enable_temporal_filter,
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
            "output_frame": "Link_00",
            "localizer_backend": "yolo_depth",
            "process_rate": 5.0,
            "sync_tolerance_s": 0.033,
            "stable_min_frames": 3,
            "stable_window_size": 5,
            "stable_position_std": 0.010,
            "association_distance": 0.080,
            "tracker_stale_after_s": 0.25,
            "enable_rgb_debug_candidates": False,
            "camera_info_topic": "/camera/camera/color/camera_info",
            "allow_scene_fallback": False,
            "force_table_center_z": False,
            "enable_table_z_fallback": False,
            "real_mode": True,
            "data_timeout_s": 1.0,
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
        condition=IfCondition(start_target_bridge),
        respawn=True,
        respawn_delay=2.0,
    )
    debug_viewer = Node(
        package="jaka_single_arm",
        executable="fruit_debug_viewer",
        condition=IfCondition(start_debug_view),
        parameters=[{
            "show_windows": False,
            "publish_rate": 10.0,
            "depth_display_min_m": 0.20,
            "depth_display_max_m": 2.00,
        }],
        output="screen",
    )
    debug_window = Node(
        package="jaka_single_arm",
        executable="fruit_debug_window",
        condition=IfCondition(start_debug_view),
        parameters=[{"display_rate": 10.0}],
        output="screen",
    )
    image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="real_fruit_debug_view",
        condition=IfCondition(start_image_view),
        arguments=["/perception/debug/fruit_view"],
        output="log",
    )
    depth_image_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="real_depth_debug_view",
        condition=IfCondition(start_image_view),
        arguments=["/perception/debug/depth_view"],
        output="log",
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
        DeclareLaunchArgument(
            "start_target_bridge", default_value="true",
            description="Enable physical target transmission to the C board",
        ),
        DeclareLaunchArgument(
            "model_license_approved", default_value="false",
            description="Set true only after the model/data license audit is approved",
        ),
        camera,
        hand_eye,
        localizer,
        target_bridge,
        debug_viewer,
        debug_window,
        image_view,
        depth_image_view,
    ])
