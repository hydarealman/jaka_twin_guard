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
    enable_pointcloud = LaunchConfiguration("enable_pointcloud")
    show_image = LaunchConfiguration("show_image")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    enable_temporal_filter = LaunchConfiguration("enable_temporal_filter")
    fruit_detector_model = LaunchConfiguration("fruit_detector_model")
    fruit_detector_confidence = LaunchConfiguration("fruit_detector_confidence")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fruit_picking_arm"), "launch", "d455_camera.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_depth": enable_depth,
            "enable_pointcloud": enable_pointcloud,
            "color_profile": color_profile,
            "depth_profile": depth_profile,
            "enable_temporal_filter": enable_temporal_filter,
        }.items(),
    )
    detector = Node(
        package="fruit_picking_arm",
        executable="fruit_target_node",
        name="fruit_quality_debug_node",
        parameters=[{
            "camera_type": "realsense",
            "scene_config_file": "scene_params_real.yaml",
            # Debug before hand-eye calibration: keep geometry and projection
            # in the D455 optical frame. Real A/B launches use the robot frame.
            "output_frame": "camera_color_optical_frame",
            # The shared perception YAML ROI is expressed in the robot/world
            # frame.  This debug launch has no hand-eye TF, so use a generous
            # optical-frame ROI and let registered depth/radius gates reject
            # invalid samples instead of silently dropping every apple.
            "detection_roi_min": [-2.0, -2.0, 0.10],
            "detection_roi_max": [2.0, 2.0, 5.0],
            "localizer_backend": "yolo_depth",
            "fruit_detector_model": fruit_detector_model,
            "fruit_detector_confidence": fruit_detector_confidence,
            "process_rate": 30.0,
            "publish_kalman_predictions": True,
            "max_kalman_prediction_age_s": 0.18,
            "sync_tolerance_s": 0.033,
            "stable_min_frames": 3,
            "stable_window_size": 5,
            # The generic COCO apple score is calibrated separately from the
            # MobileNet health score; do not compare both against 0.60.
            "stable_min_detection_confidence": 0.10,
            "stable_position_std": 0.010,
            "association_distance": 0.080,
            "tracker_stale_after_s": 0.25,
            "enable_rgb_debug_candidates": False,
            "allow_scene_fallback": False,
            "force_table_center_z": False,
            "enable_table_z_fallback": False,
            "perception_license_mode": "development",
            "model_license_approved": False,
        }],
        output="screen",
    )
    viewer = Node(
        package="fruit_picking_arm",
        executable="fruit_debug_window",
        name="d455_fruit_debug_windows",
        condition=IfCondition(show_image),
        parameters=[{"display_rate": 10.0}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("color_profile", default_value="424,240,15"),
            DeclareLaunchArgument("depth_profile", default_value="424,240,15"),
            DeclareLaunchArgument("enable_temporal_filter", default_value="false"),
            DeclareLaunchArgument(
                "enable_depth",
                default_value="true",
                description="Must be true for ROI localisation; use native Ubuntu",
            ),
            DeclareLaunchArgument(
                "enable_pointcloud",
                default_value="false",
                description="Optional diagnostic cloud; YOLO RGB-D localisation does not require it",
            ),
            DeclareLaunchArgument("show_image", default_value="true"),
            DeclareLaunchArgument(
                "fruit_detector_model",
                default_value="yolov8n.pt",
                description="Pinned apple-specific weight is supplied by the safe wrapper",
            ),
            DeclareLaunchArgument("fruit_detector_confidence", default_value="0.25"),
            LogInfo(msg="[MODE] D455 RGB-D ROI fruit-quality DEBUG; no robot motion"),
            camera,
            detector,
            viewer,
        ]
    )
