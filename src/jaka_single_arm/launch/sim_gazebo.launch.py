#!/usr/bin/env python3
"""Gazebo 物理仿真启动 — 使用 gazebo_ros2_control，带真实物理。

启动:
   ros2 launch jaka_single_arm sim_gazebo.launch.py
   ros2 launch jaka_single_arm sim_gazebo.launch.py gui:=false    # headless

Gazebo 模式下:
  - camera_type 自动设置为 "gazebo"
  - hardware_plugin 使用 "gazebo_ros2_control/GazeboSystem"
  - RViz 订阅 Gazebo 发布的点云/图像话题

启动时序 (严格串行避免竞态, 参照已验证的 sim_gazebo_massage.launch.py):
  0s  → gzserver (物理引擎) + gzclient (GUI, 可选)
  3s  → robot_state_publisher + ros2_control_node
  8s  → spawn robot entity in Gazebo (timeout 60s)
  16s → controller spawners (JSB + arm_controller, timeout 60s)
  20s → move_group
  26s → RViz
  30s → pick_place_runner
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
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
        "use_gazebo": "true",
    }
    print("[Launch] Processing URDF xacro for Gazebo...")
    document = xacro.process_file(robot_xacro, mappings=mappings)
    with open(gazebo_urdf, "w", encoding="utf-8") as f:
        f.write(document.toprettyxml(indent="  "))
    print(f"[Launch] Gazebo URDF written to: {gazebo_urdf}")
    return gazebo_urdf


def generate_launch_description():
    jaka_single_share = get_package_share_directory("jaka_single_arm")
    robot_pkg_share = get_package_share_directory("single_arm_jaka_c5_pick_place")
    gazebo_share = get_package_share_directory("gazebo_ros")

    print("[Launch] Generating URDF for Gazebo robot spawn...")
    gazebo_robot_desc = write_gazebo_robot_description(robot_pkg_share)

    # ── MoveIt 配置 (Gazebo 模式) ──
    print("[Launch] Building MoveIt configs...")
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
                "use_gazebo": "true",
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
    print("[Launch] MoveIt configs built OK.")

    ros2_controllers_path = os.path.join(
        robot_pkg_share, "config", "ros2_controllers.yaml"
    )

    # ═══ WSL2: 禁用 GPU 硬件加速，使用软件渲染 ═══
    wsl_env = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    wsl_env2 = SetEnvironmentVariable("QT_QUICK_BACKEND", "software")

    # ═══ 阶段 0: Gazebo (使用标准 gazebo.launch.py — 自动处理 gzserver + gzclient) ═══
    world_path = os.path.join(jaka_single_share, "worlds", "pick_place.world")
    print(f"[Launch] World path: {world_path}")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    # ── 阶段 1: 基础节点 (3s 后) ──
    robot_and_control = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[Launch] Starting robot_state_publisher + ros2_control_node..."),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    moveit_config.robot_description,
                    {"use_sim_time": True},
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[
                    ros2_controllers_path,
                    {"use_sim_time": True},
                ],
                remappings=[
                    ("/controller_manager/robot_description", "/robot_description"),
                ],
                output="screen",
            ),
        ],
    )

    # ── 阶段 2: Spawn Robot (8s 后，Gazebo 应该已就绪) ──
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[Launch] Spawning robot entity in Gazebo..."),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", gazebo_robot_desc,
                    "-entity", "jaka_c5_pick_place",
                    "-timeout", "60",
                    "-x", "0", "-y", "0", "-z", "0",
                ],
                output="screen",
            ),
        ],
    )

    # ── 阶段 3: Controller Spawners (16s 后，robot 已 spawn) ──
    controller_spawners = TimerAction(
        period=16.0,
        actions=[
            LogInfo(msg="[Launch] Spawning controllers (jsb + arm_controller)..."),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "-c", "/controller_manager",
                    "--controller-manager-timeout", "60",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "-c", "/controller_manager",
                    "--controller-manager-timeout", "60",
                ],
                output="screen",
            ),
        ],
    )

    # ── 阶段 4: MoveIt (20s 后) ──
    move_group = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg="[Launch] Starting move_group..."),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    {"use_sim_time": True},
                ],
            ),
        ],
    )

    # ── 阶段 5: RViz (26s 后) ──
    rviz_config = os.path.join(robot_pkg_share, "config", "pick_place.rviz")
    rviz_node = TimerAction(
        period=26.0,
        actions=[
            LogInfo(msg="[Launch] Starting RViz..."),
            Node(
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
                    {"use_sim_time": True},
                ],
            ),
        ],
    )

    # ── 阶段 6: Pick-and-place runner (30s 后) ──
    pick_place_runner = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg="[Launch] Starting pick_place_runner (Gazebo mode)..."),
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
            "gui", default_value="true",
            description="Show Gazebo GUI"),
        LogInfo(msg=["[Launch] ===== Gazebo Physical Simulation ====="]),
        LogInfo(msg=["[Launch] WSL2: LIBGL_ALWAYS_SOFTWARE=1"]),
        LogInfo(msg=["[Launch] Startup: gzserver → gzclient → spawn → controllers → MoveIt → RViz → runner"]),
        wsl_env,
        wsl_env2,
        gazebo,
        robot_and_control,
        spawn_robot,
        controller_spawners,
        move_group,
        rviz_node,
        pick_place_runner,
    ])
