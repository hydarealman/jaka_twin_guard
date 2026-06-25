#!/usr/bin/env python3
"""真实硬件启动 — JAKA C5 单臂抓取。

启动:
   ros2 launch jaka_single_arm real_hardware.launch.py

为后续实车部署 + 电控联调预留。
使用真实 JAKA C5 硬件接口替代 mock_components / Gazebo。

ROBOT MODEL SWAPPING:
   ros2 launch jaka_single_arm real_hardware.launch.py \
       hardware_plugin:=jaka_c5_hardware/JakaC5SystemPositionOnly \
       description_package:=my_custom_robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    # ── 参数 ──
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    description_pkg = LaunchConfiguration("description_package")
    scene = LaunchConfiguration("scene")

    hardware_plugin_arg = DeclareLaunchArgument(
        "hardware_plugin",
        default_value="mock_components/GenericSystem",
        description="Hardware plugin: mock_components/GenericSystem | jaka_c5_hardware/JakaC5SystemPositionOnly",
    )
    desc_pkg_arg = DeclareLaunchArgument(
        "description_package",
        default_value="single_arm_jaka_c5_pick_place",
        description="Package for robot URDF/SRDF",
    )
    scene_arg = DeclareLaunchArgument(
        "scene",
        default_value="a",
        description="Scene variant",
    )

    robot_pkg_share = get_package_share_directory(
        "single_arm_jaka_c5_pick_place"
    )
    jaka_single_share = get_package_share_directory("jaka_single_arm")

    initial_positions = os.path.join(robot_pkg_share, "config", "initial_positions.yaml")

    # ── MoveIt 配置 ──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_pick_place",
            package_name="single_arm_jaka_c5_pick_place",
        )
        .robot_description(
            file_path=os.path.join(robot_pkg_share, "config", "jaka_c5_pick_place.urdf.xacro"),
            mappings={
                "initial_positions_file": initial_positions,
                "hardware_plugin": hardware_plugin,
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(robot_pkg_share, "config", "jaka_c5_pick_place.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(robot_pkg_share, "config", "moveit_controllers.yaml")
        )
        .joint_limits(
            file_path=os.path.join(robot_pkg_share, "config", "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── 节点 ──

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    ros2_controllers_path = os.path.join(
        robot_pkg_share, "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # 任务运行器（真机模式下 camera_type 默认为 realsense）
    pick_place_runner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="jaka_single_arm",
                executable="pick_place_runner",
                name="pick_place_runner",
                output="screen",
                arguments=["--scene", scene],
                parameters=[
                    {"use_sim_time": False},
                    {"camera_type": "realsense"},
                ],
            ),
        ],
    )

    return LaunchDescription([
        hardware_plugin_arg,
        desc_pkg_arg,
        scene_arg,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        pick_place_runner,
    ])
