#!/usr/bin/env python3
"""Architecture A — RealSense + MoveIt + serial trajectory controller.

The C board only tracks joint trajectories and closes low-level loops.  No
ros2_control arm controller is launched; SerialTrajectoryController owns the
standard /arm_controller/follow_joint_trajectory action instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    start_camera = LaunchConfiguration("start_camera")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    enable_temporal_filter = LaunchConfiguration("enable_temporal_filter")
    start_rviz = LaunchConfiguration("start_rviz")
    start_debug_view = LaunchConfiguration("start_debug_view")
    start_image_view = LaunchConfiguration("start_image_view")
    start_perception = LaunchConfiguration("start_perception")
    run_task = LaunchConfiguration("run_task")
    model_license_approved = LaunchConfiguration("model_license_approved")

    robot_share = get_package_share_directory("single_arm_jaka_c5_pick_place")
    jaka_share = get_package_share_directory("jaka_single_arm")
    initial_positions = os.path.join(robot_share, "config", "initial_positions.yaml")
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_pick_place", package_name="single_arm_jaka_c5_pick_place"
        )
        .robot_description(
            file_path=os.path.join(robot_share, "config", "jaka_c5_pick_place.urdf.xacro"),
            mappings={
                "initial_positions_file": initial_positions,
                # MoveIt only needs the model here; execution is the serial action server.
                "hardware_plugin": "mock_components/GenericSystem",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(robot_share, "config", "jaka_c5_pick_place.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(robot_share, "config", "moveit_controllers.yaml")
        )
        .joint_limits(file_path=os.path.join(robot_share, "config", "joint_limits.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

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
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config.to_dict()],
        output="screen",
    )
    serial_controller = Node(
        package="jaka_single_arm",
        executable="serial_trajectory_controller",
        parameters=[
            os.path.join(jaka_share, "config", "architecture_a_serial.yaml"),
            {"serial_port": port, "baudrate": baudrate},
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    debug_viewer = Node(
        package="jaka_single_arm",
        executable="fruit_debug_viewer",
        condition=IfCondition(start_debug_view),
        parameters=[{
            "show_windows": False,
            "annotated_topic": "/perception/detection_annotated",
            "target_topic": "/perception/debug/stable_fruit_targets_camera",
            "publish_rate": 15.0,
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
    perception_debug = Node(
        package="jaka_single_arm",
        executable="fruit_target_node",
        name="fruit_target_debug_node",
        condition=IfCondition(start_perception),
        parameters=[{
            "camera_type": "realsense",
            # This node is visualization-only. Detect in the measured camera
            # frame so boxes work before hand-eye calibration. The production
            # task/serial path still requires Link_00 and never consumes this
            # debug-only target topic.
            "output_frame": "camera_color_optical_frame",
            "output_topic": "/perception/debug/stable_fruit_targets_camera",
            "localizer_backend": "yolo_depth",
            "detection_roi_min": [-1.2, -0.8, 0.15],
            "detection_roi_max": [1.2, 0.8, 2.5],
            # Debug needs responsive windows more than full-density grasp
            # precision. Production perception keeps the stricter defaults.
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
            # The fast YOLO path runs independently from the latest-only
            # MobileNet worker. Ten hertz keeps fresh boxes visible while the
            # camera callbacks remain isolated from inference.
            "process_rate": 30.0,
            "publish_kalman_predictions": True,
            "max_kalman_prediction_age_s": 0.18,
            "sync_tolerance_s": 0.033,
            "stable_min_frames": 3,
            "stable_window_size": 5,
            "stable_position_std": 0.010,
            "association_distance": 0.080,
            "tracker_stale_after_s": 0.25,
            # The tiled COCO baseline's raw box score is not calibrated like
            # the MobileNet health score. Repeated depth + health + motion
            # gates still decide stability; keep production B's detector
            # floor at the stricter default.
            "stable_min_detection_confidence": 0.10,
            "enable_rgb_debug_candidates": False,
            "perception_license_mode": "production",
            "model_license_approved": model_license_approved,
        }],
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
    runner = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="jaka_single_arm",
                executable="pick_place_runner",
                condition=IfCondition(run_task),
                parameters=[{
                    "camera_type": "realsense",
                    "perception_output_frame": "world",
                    "force_table_center_z": False,
                    "enable_table_z_fallback": False,
                    "allow_scene_fallback": False,
                    "real_mode": True,
                    "data_timeout_s": 1.0,
                    "camera_info_topic": "/camera/camera/color/camera_info",
                    "perception_license_mode": "production",
                    "model_license_approved": model_license_approved,
                }],
                output="screen",
            )
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(start_rviz),
        arguments=[
            "-d",
            os.path.join(robot_share, "config", "pick_place.rviz"),
            # WSLg ignores the saved QMainWindow size on some starts and
            # otherwise creates a tiny grey RViz window.
            "--qwindowgeometry",
            "1400x900+40+40",
        ],
        parameters=[moveit_config.to_dict()],
        output="log",
    )
    rviz_window_guard = Node(
        package="jaka_single_arm",
        executable="rviz_window_guard",
        condition=IfCondition(start_rviz),
        output="screen",
    )

    # Start camera/debug first, then spread CPU/IO-heavy model, MoveIt and
    # RViz initialization over time. On WSL2 starting all of them together
    # can starve usbip/vhci camera callbacks for tens of seconds.
    robot_stack = TimerAction(
        period=4.0,
        actions=[hand_eye, robot_state_publisher, move_group, serial_controller],
    )
    perception_stack = TimerAction(period=8.0, actions=[perception_debug])
    rviz_stack = TimerAction(period=12.0, actions=[rviz, rviz_window_guard])

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("color_profile", default_value="424,240,15"),
        DeclareLaunchArgument("depth_profile", default_value="424,240,15"),
        DeclareLaunchArgument("enable_temporal_filter", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_debug_view", default_value="true"),
        DeclareLaunchArgument("start_image_view", default_value="true"),
        DeclareLaunchArgument(
            "start_perception", default_value="false",
            description="Run continuous real fruit perception for debug only",
        ),
        DeclareLaunchArgument(
            "run_task", default_value="false",
            description="Enable autonomous physical pick-and-place after bring-up",
        ),
        DeclareLaunchArgument(
            "model_license_approved", default_value="false",
            description="Set true only after the model/data license audit is approved",
        ),
        camera,
        debug_viewer,
        debug_window,
        image_view,
        depth_image_view,
        robot_stack,
        perception_stack,
        rviz_stack,
        runner,
    ])
