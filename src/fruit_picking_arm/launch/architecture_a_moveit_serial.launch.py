#!/usr/bin/env python3
"""Architecture A: D455 + MoveIt + physical serial trajectory controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from fruit_picking_arm.calibration_profile import load_calibration_selection


def _enabled(context, substitution):
    return context.perform_substitution(substitution).strip().lower() in {"1", "true", "yes", "on"}


def _robot_actions(context, *_, **kwargs):
    robot_share = kwargs["robot_share"]
    fruit_arm_share = kwargs["fruit_arm_share"]
    start_robot_stack = LaunchConfiguration("start_robot_stack")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    run_task = LaunchConfiguration("run_task")
    serial = context.perform_substitution(LaunchConfiguration("robot_serial")).strip()
    mode = context.perform_substitution(LaunchConfiguration("kinematics_mode"))
    profile = context.perform_substitution(LaunchConfiguration("calibration_file"))
    physical_requested = _enabled(context, start_robot_stack) or _enabled(context, run_task)
    if not serial and not physical_requested and mode.strip().lower() == "nominal":
        serial = "perception-only"
    selection = load_calibration_selection(mode, serial, profile)

    initial_positions = os.path.join(robot_share, "config", "initial_positions.yaml")
    mappings = {
        "initial_positions_file": initial_positions,
        "hardware_plugin": "mock_components/GenericSystem",
        **selection.xacro_mappings,
    }
    moveit_config = (
        MoveItConfigsBuilder("fruit_picking_arm", package_name="fruit_arm_moveit_config")
        .robot_description(
            file_path=os.path.join(robot_share, "config", "fruit_picking_arm.urdf.xacro"),
            mappings=mappings,
        )
        .robot_description_semantic(file_path=os.path.join(robot_share, "config", "fruit_picking_arm.srdf"))
        .trajectory_execution(
            file_path=os.path.join(
                robot_share, "config", "moveit_controllers_real.yaml"
            )
        )
        .joint_limits(file_path=os.path.join(robot_share, "config", "joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    hand_eye_parameters = (
        [selection.calibrated_camera_parameters]
        if selection.calibrated_camera_parameters is not None
        else [os.path.join(fruit_arm_share, "config", "hand_eye_params.yaml")]
    )
    hand_eye = Node(
        package="fruit_picking_arm", executable="hand_eye_static_tf",
        parameters=hand_eye_parameters, output="screen",
    )
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[moveit_config.robot_description], output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        parameters=[moveit_config.to_dict()], output="screen",
    )
    serial_controller = Node(
        package="fruit_picking_arm", executable="serial_trajectory_controller",
        parameters=[
            os.path.join(fruit_arm_share, "config", "architecture_a_serial.yaml"),
            {"serial_port": LaunchConfiguration("serial_port"), "baudrate": LaunchConfiguration("baudrate")},
        ],
        output="screen", respawn=True, respawn_delay=2.0,
    )
    rviz = Node(
        package="rviz2", executable="rviz2", condition=IfCondition(start_rviz),
        arguments=["-d", PathJoinSubstitution([
                       FindPackageShare("fruit_arm_moveit_config"), "config", rviz_config,
                   ]),
                   "--qwindowgeometry", "1400x900+40+40"],
        parameters=[moveit_config.to_dict()], output="log",
    )
    rviz_guard = Node(
        package="fruit_picking_arm", executable="rviz_window_guard",
        condition=IfCondition(start_rviz), output="screen",
    )
    mode_message = (
        f"[KINEMATICS] robot={selection.robot_serial}, mode={selection.mode}, "
        + (f"profile={selection.profile_path}" if selection.mode == "calibrated" else "nominal CAD model")
    )
    if selection.mode == "nominal":
        mode_message += "; WARNING: no per-robot kinematic correction is active"
    return [
        LogInfo(msg=mode_message),
        TimerAction(
            period=4.0,
            actions=[hand_eye, state_publisher, move_group, serial_controller],
            condition=IfCondition(start_robot_stack),
        ),
        TimerAction(period=12.0, actions=[rviz, rviz_guard]),
    ]


def generate_launch_description():
    robot_share = get_package_share_directory("fruit_arm_moveit_config")
    fruit_arm_share = get_package_share_directory("fruit_picking_arm")
    start_camera = LaunchConfiguration("start_camera")
    start_debug_view = LaunchConfiguration("start_debug_view")
    start_image_view = LaunchConfiguration("start_image_view")
    start_perception = LaunchConfiguration("start_perception")
    start_fruit_goal_bridge = LaunchConfiguration("start_fruit_goal_bridge")
    fruit_detector_confidence = LaunchConfiguration("fruit_detector_confidence")
    stable_min_detection_confidence = LaunchConfiguration(
        "stable_min_detection_confidence"
    )
    stable_min_frames = LaunchConfiguration("stable_min_frames")
    detection_roi_min_z = LaunchConfiguration("detection_roi_min_z")
    detection_roi_max_z = LaunchConfiguration("detection_roi_max_z")
    enable_table_perception = LaunchConfiguration("enable_table_perception")
    run_task = LaunchConfiguration("run_task")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("fruit_picking_arm"), "launch", "d455_camera.launch.py"])
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_depth": "true", "enable_pointcloud": "false",
            "color_profile": LaunchConfiguration("color_profile"),
            "depth_profile": LaunchConfiguration("depth_profile"),
            "enable_temporal_filter": LaunchConfiguration("enable_temporal_filter"),
        }.items(),
    )
    debug_window = Node(
        package="fruit_picking_arm", executable="fruit_debug_window",
        condition=IfCondition(start_debug_view), parameters=[{"display_rate": 10.0}], output="screen",
    )
    perception_debug = Node(
        package="fruit_picking_arm", executable="fruit_target_node", name="fruit_target_debug_node",
        condition=IfCondition(start_perception),
        parameters=[{
            "camera_type": "realsense",
            "scene_config_file": "scene_params_real.yaml",
            "output_frame": LaunchConfiguration("perception_output_frame"),
            "output_topic": LaunchConfiguration("perception_output_topic"),
            "localizer_backend": "yolo_depth",
            "fruit_detector_confidence": ParameterValue(
                fruit_detector_confidence, value_type=float
            ),
            "detection_roi_min": [-1.2, -0.8, 0.15],
            "detection_roi_max": [1.2, 0.8, 2.5],
            "detection_roi_min_z": ParameterValue(
                detection_roi_min_z, value_type=float
            ),
            "detection_roi_max_z": ParameterValue(
                detection_roi_max_z, value_type=float
            ),
            "enable_table_perception": ParameterValue(
                enable_table_perception, value_type=bool
            ),
            "point_cloud_downsample": 3,
            "voxel_leaf_size": 0.01,
            "ransac_max_iterations": 80,
            "cluster_tolerance": 0.025,
            "min_cluster_size": 25,
            "max_cluster_size": 2500,
            "camera_info_topic": "/camera/camera/color/camera_info",
            "allow_scene_fallback": False,
            "force_table_center_z": False,
            "enable_table_z_fallback": False,
            "real_mode": True,
            "data_timeout_s": 1.0,
            "process_rate": 30.0,
            "publish_kalman_predictions": True,
            "max_kalman_prediction_age_s": 0.18,
            "sync_tolerance_s": 0.033,
            "stable_min_frames": ParameterValue(stable_min_frames, value_type=int),
            "stable_window_size": 5,
            "stable_position_std": 0.010,
            "association_distance": 0.080,
            "tracker_stale_after_s": 0.25,
            "stable_min_detection_confidence": ParameterValue(
                stable_min_detection_confidence, value_type=float
            ),
            "enable_rgb_debug_candidates": False,
            "perception_license_mode": "production",
            "model_license_approved": LaunchConfiguration("model_license_approved"),
        }],
        output="screen", respawn=True, respawn_delay=2.0,
    )
    fruit_goal_bridge = Node(
        package="fruit_picking_arm",
        executable="fruit_rviz_goal_bridge",
        name="fruit_rviz_goal_bridge",
        condition=IfCondition(start_fruit_goal_bridge),
        parameters=[{
            "target_topic": LaunchConfiguration("perception_output_topic"),
            "scene_config_file": "scene_params_real.yaml",
            "world_frame": "world",
            "planning_group": "arm",
            "ik_link": "gripper_tcp",
            "top_down_yaw": 3.141592654,
            "finger_tip_beyond_tcp": 0.037,
            "clearance_above_fruit": 0.050,
            "require_perceived_table": True,
            # The D455 hardware clock converges against WSL system time after
            # startup.  Source timestamps remain monotonic; these bounds only
            # tolerate the measured cross-clock offset in this operator-gated mode.
            "max_detection_age_s": 1.50,
            "max_future_detection_s": 0.60,
            # The target node already enforces fresh RGB-D data and monotonic
            # sensor timestamps. D455 hardware time is not phase-locked to
            # WSL system time, so absolute cross-clock age is not meaningful.
            "use_source_age_gate": False,
        }],
        output="screen",
    )
    image_view = Node(
        package="rqt_image_view", executable="rqt_image_view", name="real_fruit_debug_view",
        condition=IfCondition(start_image_view), arguments=["/perception/debug/fruit_view"], output="log",
    )
    depth_view = Node(
        package="rqt_image_view", executable="rqt_image_view", name="real_depth_debug_view",
        condition=IfCondition(start_image_view), arguments=["/perception/debug/depth_view"], output="log",
    )
    runner = TimerAction(
        period=15.0,
        actions=[Node(
            package="fruit_picking_arm", executable="pick_place_runner", condition=IfCondition(run_task),
            arguments=["--scene-config", "scene_params_real.yaml"],
            parameters=[{
                "camera_type": "realsense", "perception_output_frame": "world",
                "force_table_center_z": False, "enable_table_z_fallback": False,
                "allow_scene_fallback": False, "real_mode": True, "data_timeout_s": 1.0,
                "camera_info_topic": "/camera/camera/color/camera_info",
                "perception_license_mode": "production",
                "model_license_approved": LaunchConfiguration("model_license_approved"),
                "use_external_perception": True,
            }],
            output="screen",
        )],
    )
    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("robot_serial", default_value=""),
        DeclareLaunchArgument("kinematics_mode", default_value="nominal"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("color_profile", default_value="424,240,15"),
        DeclareLaunchArgument("depth_profile", default_value="424,240,15"),
        DeclareLaunchArgument("enable_temporal_filter", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config", default_value="fruit_picking_arm.rviz",
            description="RViz config filename from fruit_arm_moveit_config/config",
        ),
        DeclareLaunchArgument("start_debug_view", default_value="true"),
        DeclareLaunchArgument("start_image_view", default_value="true"),
        DeclareLaunchArgument("start_perception", default_value="false"),
        DeclareLaunchArgument("start_fruit_goal_bridge", default_value="false"),
        DeclareLaunchArgument("fruit_detector_confidence", default_value="0.25"),
        DeclareLaunchArgument(
            "stable_min_detection_confidence", default_value="0.10"
        ),
        DeclareLaunchArgument("stable_min_frames", default_value="3"),
        DeclareLaunchArgument("detection_roi_min_z", default_value="-0.10"),
        DeclareLaunchArgument("detection_roi_max_z", default_value="1.20"),
        DeclareLaunchArgument(
            "perception_output_frame", default_value="camera_color_optical_frame"
        ),
        DeclareLaunchArgument(
            "perception_output_topic",
            default_value="/perception/debug/stable_fruit_targets_camera",
        ),
        DeclareLaunchArgument("start_robot_stack", default_value="false"),
        DeclareLaunchArgument("run_task", default_value="false"),
        DeclareLaunchArgument("enable_table_perception", default_value="false"),
        DeclareLaunchArgument("model_license_approved", default_value="false"),
    ]
    return LaunchDescription(arguments + [
        camera, debug_window, image_view, depth_view,
        TimerAction(period=8.0, actions=[perception_debug]),
        TimerAction(period=12.0, actions=[fruit_goal_bridge]),
        OpaqueFunction(
            function=_robot_actions,
            kwargs={"robot_share": robot_share, "fruit_arm_share": fruit_arm_share},
        ),
        runner,
    ])
