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
    start_rviz = LaunchConfiguration("start_rviz")
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
            "enable_pointcloud": "true",
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
    runner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="jaka_single_arm",
                executable="pick_place_runner",
                parameters=[{
                    "camera_type": "realsense",
                    "perception_output_frame": "world",
                    "force_table_center_z": False,
                    "enable_table_z_fallback": False,
                    "allow_scene_fallback": False,
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
        arguments=["-d", os.path.join(robot_share, "config", "pick_place.rviz")],
        parameters=[moveit_config.to_dict()],
        output="log",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument(
            "model_license_approved", default_value="false",
            description="Set true only after the model/data license audit is approved",
        ),
        camera,
        hand_eye,
        robot_state_publisher,
        move_group,
        serial_controller,
        rviz,
        runner,
    ])
