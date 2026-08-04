#!/usr/bin/env python3
"""D455 eye-to-hand calibration only; no fruit detector or pick runner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    start_moveit = LaunchConfiguration("start_moveit")
    start_rviz = LaunchConfiguration("start_rviz")
    output_yaml = LaunchConfiguration("output_yaml")

    robot_share = get_package_share_directory("single_arm_jaka_c5_pick_place")
    package_share = get_package_share_directory("jaka_single_arm")
    initial_positions = os.path.join(robot_share, "config", "initial_positions.yaml")
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_pick_place", package_name="single_arm_jaka_c5_pick_place"
        )
        .robot_description(
            file_path=os.path.join(
                robot_share, "config", "jaka_c5_pick_place.urdf.xacro"
            ),
            mappings={
                "initial_positions_file": initial_positions,
                "hardware_plugin": "mock_components/GenericSystem",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(
                robot_share, "config", "jaka_c5_pick_place.srdf"
            )
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
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
        launch_arguments={
            "enable_color": "true",
            "enable_depth": "false",
            "pointcloud.enable": "false",
        }.items(),
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )
    serial_controller = Node(
        package="jaka_single_arm",
        executable="serial_trajectory_controller",
        parameters=[
            os.path.join(package_share, "config", "architecture_a_serial.yaml"),
            {"serial_port": port, "baudrate": baudrate},
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        condition=IfCondition(start_moveit),
        parameters=[moveit_config.to_dict()],
        output="screen",
    )
    calibrator = Node(
        package="jaka_single_arm",
        executable="eye_to_hand_calibrator",
        parameters=[
            os.path.join(package_share, "config", "eye_to_hand_calibration.yaml"),
            {"output_yaml": output_yaml},
        ],
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(start_rviz),
        arguments=["-d", os.path.join(robot_share, "config", "pick_place.rviz")],
        parameters=[moveit_config.to_dict()],
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("start_moveit", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument(
                "output_yaml",
                default_value=os.path.join(
                    package_share, "config", "hand_eye_params.yaml"
                ),
                description="Writable YAML destination loaded by real launches",
            ),
            camera,
            state_publisher,
            serial_controller,
            move_group,
            calibrator,
            rviz,
        ]
    )
