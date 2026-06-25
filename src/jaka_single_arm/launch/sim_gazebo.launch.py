#!/usr/bin/env python3
"""Gazebo 物理仿真启动 — 使用 gazebo_ros2_control，带真实物理。

启动:
   ros2 launch jaka_single_arm sim_gazebo.launch.py
   ros2 launch jaka_single_arm sim_gazebo.launch.py gui:=false    # headless
   ros2 launch jaka_single_arm sim_gazebo.launch.py world:=pick_place.world

ROBOT MODEL SWAPPING:
   ros2 launch jaka_single_arm sim_gazebo.launch.py \
       description_package:=my_robot_description \
       description_file:=config/my_robot.urdf.xacro

Gazebo 模式下:
  - camera_type 自动设置为 "gazebo"
  - hardware_plugin 使用 "gazebo_ros2_control/GazeboSystem"
  - RViz 订阅 Gazebo 发布的点云/图像话题
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def write_gazebo_robot_description(pkg_share: str) -> str:
    """处理 URDF xacro → 写入临时文件（供 Gazebo spawn 使用）。"""
    robot_xacro = os.path.join(pkg_share, "config", "jaka_c5_pick_place.urdf.xacro")
    gazebo_urdf = os.path.join(tempfile.gettempdir(), "jaka_c5_pick_place_gazebo.urdf")
    initial_positions = os.path.join(pkg_share, "config", "initial_positions.yaml")
    mappings = {
        "initial_positions_file": initial_positions,
        "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
    }
    document = xacro.process_file(robot_xacro, mappings=mappings)
    with open(gazebo_urdf, "w", encoding="utf-8") as f:
        f.write(document.toprettyxml(indent="  "))
    return gazebo_urdf


def launch_gazebo_with_scene(context, *args, **kwargs):
    """OpaqueFunction: 根据参数动态选择 world 文件。"""
    world_name = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    jaka_single_share = get_package_share_directory("jaka_single_arm")
    gazebo_share = get_package_share_directory("gazebo_ros")

    world_path = os.path.join(jaka_single_share, "worlds", world_name)

    gazebo_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path, "gui": gui}.items(),
    )
    return [gazebo_desc]


def generate_launch_description():
    jaka_single_share = get_package_share_directory("jaka_single_arm")
    # Use single_arm_jaka_c5_pick_place for URDF/SRDF/controllers
    robot_pkg_share = get_package_share_directory("single_arm_jaka_c5_pick_place")

    gazebo_robot_desc = write_gazebo_robot_description(robot_pkg_share)

    # ── MoveIt 配置 (Gazebo 模式) ──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_pick_place",
            package_name="single_arm_jaka_c5_pick_place",
        )
        .robot_description(
            file_path=os.path.join(robot_pkg_share, "config", "jaka_c5_pick_place.urdf.xacro"),
            mappings={
                "initial_positions_file": os.path.join(robot_pkg_share, "config", "initial_positions.yaml"),
                "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
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

    # Gazebo (OpaqueFunction 动态选世界)
    gazebo_action = OpaqueFunction(function=launch_gazebo_with_scene)

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

    # Spawn robot entity in Gazebo
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", gazebo_robot_desc,
                    "-entity", "jaka_c5_pick_place",
                    "-timeout", "120",
                    "-x", "0", "-y", "0", "-z", "0",
                ],
                output="screen",
            ),
        ],
    )

    # Controller spawners
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    # RViz (observes Gazebo data)
    rviz_config = os.path.join(robot_pkg_share, "config", "pick_place.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # Pick-and-place runner (delayed for Gazebo startup)
    pick_place_runner = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="jaka_single_arm",
                executable="pick_place_runner",
                name="pick_place_runner",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"camera_type": "gazebo"},
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="pick_place.world",
            description="Gazebo world file name"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show Gazebo GUI"),
        DeclareLaunchArgument(
            "scene", default_value="a",
            description="Scene variant"),
        LogInfo(msg=["[Gazebo] Starting physical simulation..."]),
        gazebo_action,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        spawn_robot,
        rviz_node,
        pick_place_runner,
    ])
