#!/usr/bin/env python3
"""Gazebo 物理仿真启动 — 使用 gazebo_ros2_control，带真实物理/重力。

启动:
   ros2 launch jaka_dual_arm sim_gazebo.launch.py                    # 场景 A (默认)
   ros2 launch jaka_dual_arm sim_gazebo.launch.py scene:=b           # 场景 B: 料框拣选
   ros2 launch jaka_dual_arm sim_gazebo.launch.py scene:=c           # 场景 C: 传送带分拣
   ros2 launch jaka_dual_arm sim_gazebo.launch.py gui:=false         # 无 GUI (headless)
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


SCENE_WORLDS = {
    "a": "scene_a_table_pick.world",
    "b": "scene_b_bin_pick.world",
    "c": "scene_c_conveyor.world",
}


def write_gazebo_robot_description(dual_arm_share: str) -> str:
    """处理 URDF xacro → 写入临时文件（用于 Gazebo spawn）。"""
    robot_xacro = os.path.join(dual_arm_share, "config", "jaka_c5_dual.urdf.xacro")
    gazebo_urdf = os.path.join(tempfile.gettempdir(), "jaka_c5_dual_gazebo.urdf")
    mappings = {
        "use_gazebo": "true",
        "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
        "use_flange": "true",
    }
    document = xacro.process_file(robot_xacro, mappings=mappings)
    with open(gazebo_urdf, "w", encoding="utf-8") as f:
        f.write(document.toprettyxml(indent="  "))
    return gazebo_urdf


def launch_gazebo_with_scene(context, *args, **kwargs):
    """OpaqueFunction: 根据 scene 参数动态选择世界文件并启动 Gazebo。"""
    scene = LaunchConfiguration("scene").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")
    gazebo_share = get_package_share_directory("gazebo_ros")

    world_file = SCENE_WORLDS.get(scene, SCENE_WORLDS["a"])
    world_path = os.path.join(jaka_dual_share, "worlds", world_file)

    gazebo_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world_path, "gui": gui}.items(),
    )
    return [gazebo_desc]


def generate_launch_description():
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")
    dual_arm_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")

    gazebo_robot_description = write_gazebo_robot_description(dual_arm_share)

    # ── MoveIt 配置（Gazebo 模式）──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_dual",
            package_name="dual_arm_jaka_c5_moveit_config",
        )
        .robot_description(
            file_path=f"{dual_arm_share}/config/jaka_c5_dual.urdf.xacro",
            mappings={
                "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
                "use_gazebo": "true",
                "use_flange": "true",
            },
        )
        .robot_description_semantic(
            file_path=f"{dual_arm_share}/config/jaka_c5_dual.srdf"
        )
        .trajectory_execution(
            file_path=f"{dual_arm_share}/config/moveit_controllers.yaml"
        )
        .joint_limits(
            file_path=f"{dual_arm_share}/config/joint_limits.yaml"
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── 节点 ──

    # Gazebo (OpaqueFunction 动态选世界文件)
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
        dual_arm_share, "config", "ros2_controllers.yaml"
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

    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", gazebo_robot_description,
                    "-entity", "jaka_c5_dual",
                    "-timeout", "120",
                    "-x", "0", "-y", "0", "-z", "0",
                ],
                output="screen",
            )
        ],
    )

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
    left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )
    right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    carry_runner = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="jaka_dual_arm",
                executable="carry_task_runner",
                name="carry_task_runner",
                output="screen",
                arguments=["--scene", LaunchConfiguration("scene")],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene", default_value="a",
            description="场景: a=桌到桌, b=料框拣选, c=传送带分拣"),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show Gazebo GUI"),
        LogInfo(msg=["[Gazebo] Starting physical simulation..."]),
        gazebo_action,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        left_spawner,
        right_spawner,
        spawn_robot,
        carry_runner,
    ])
