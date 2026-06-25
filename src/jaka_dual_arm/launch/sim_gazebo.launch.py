#!/usr/bin/env python3
"""Gazebo 物理仿真启动 — 使用 gazebo_ros2_control，带真实物理/重力。

启动:
   ros2 launch jaka_dual_arm sim_gazebo.launch.py                    # 场景 A (默认)
   ros2 launch jaka_dual_arm sim_gazebo.launch.py scene:=b           # 场景 B: 料框拣选
   ros2 launch jaka_dual_arm sim_gazebo.launch.py scene:=c           # 场景 C: 传送带分拣
   ros2 launch jaka_dual_arm sim_gazebo.launch.py gui:=false         # 无 GUI (headless)

启动时序 (严格串行避免竞态):
  0s  → Gazebo (gzserver + gzclient)
  3s  → robot_state_publisher + ros2_control_node
  10s → spawn robot entity in Gazebo
  18s → controller spawners (JSB + left + right)
  22s → move_group
  28s → carry_runner
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
    print("[DualArm Launch] Processing URDF xacro for Gazebo...")
    document = xacro.process_file(robot_xacro, mappings=mappings)
    with open(gazebo_urdf, "w", encoding="utf-8") as f:
        f.write(document.toprettyxml(indent="  "))
    print(f"[DualArm Launch] Gazebo URDF written to: {gazebo_urdf}")
    return gazebo_urdf


def launch_gazebo_with_scene(context, *args, **kwargs):
    """OpaqueFunction: 根据 scene 参数动态选择世界文件并启动 Gazebo。"""
    scene = LaunchConfiguration("scene").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")
    gazebo_share = get_package_share_directory("gazebo_ros")

    world_file = SCENE_WORLDS.get(scene, SCENE_WORLDS["a"])
    world_path = os.path.join(jaka_dual_share, "worlds", world_file)
    print(f"[DualArm Launch] Starting Gazebo with world: {world_path}, gui={gui}")

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

    ros2_controllers_path = os.path.join(
        dual_arm_share, "config", "ros2_controllers.yaml"
    )

    # ── 阶段 0: Gazebo (立即启动) ──
    gazebo_action = OpaqueFunction(function=launch_gazebo_with_scene)

    # ── 阶段 1: 基础节点 (3s 后) ──
    robot_state_publisher = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[DualArm] Starting robot_state_publisher + ros2_control_node..."),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[moveit_config.robot_description,
                            {"use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[ros2_controllers_path,
                            {"use_sim_time": True}],
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description"),
                ],
                output="screen",
            ),
        ],
    )

    # ── 阶段 2: Spawn Robot (10s 后) ──
    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[DualArm] Spawning robot entity in Gazebo..."),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", gazebo_robot_description,
                    "-entity", "jaka_c5_dual",
                    "-timeout", "60",
                    "-x", "0", "-y", "0", "-z", "0",
                ],
                output="screen",
            ),
        ],
    )

    # ── 阶段 3: Controller Spawners (18s 后，robot 已 spawn) ──
    controller_spawners = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg="[DualArm] Spawning controllers (jsb + left + right)..."),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                           "--controller-manager-timeout", "60"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_arm_controller", "-c", "/controller_manager",
                           "--controller-manager-timeout", "60"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller", "-c", "/controller_manager",
                           "--controller-manager-timeout", "60"],
                output="screen",
            ),
        ],
    )

    # ── 阶段 4: MoveIt (22s 后) ──
    move_group = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg="[DualArm] Starting move_group..."),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(),
                            {"use_sim_time": True}],
            ),
        ],
    )

    # ── 阶段 5: Runner (28s 后) ──
    carry_runner = TimerAction(
        period=28.0,
        actions=[
            LogInfo(msg="[DualArm] Starting carry_task_runner..."),
            Node(
                package="jaka_dual_arm",
                executable="carry_task_runner",
                name="carry_task_runner",
                output="screen",
                arguments=["--scene", LaunchConfiguration("scene")],
                parameters=[{"use_sim_time": True}],
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
        LogInfo(msg=["[DualArm] ===== Gazebo Physical Simulation ====="]),
        LogInfo(msg=["[DualArm] Startup: Gazebo → robot_state → spawn → controllers → MoveIt → runner"]),
        gazebo_action,
        robot_state_publisher,
        spawn_robot,
        controller_spawners,
        move_group,
        carry_runner,
    ])
