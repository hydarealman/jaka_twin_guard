#!/usr/bin/env python3
"""工业级双臂按摩系统启动 — 力控架构 + 安全监控 + 60式13手法。

与旧 Demo 的关系:
    旧: ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py
    新: ros2 launch jaka_dual_arm industrial_massage.launch.py
    旧 Demo 代码不受影响，可继续独立运行。

升级要点:
    1. 全部参数从 YAML 加载 (massage_body_params.yaml + massage_stages.yaml)
    2. 集成 VirtualImpedance 柔顺力控
    3. 集成 SafetyMonitor 实时安全监控
    4. 逐阶段 MoveIt RRT 规划 (而非预计算整条轨迹)

启动:
    ros2 launch jaka_dual_arm industrial_massage.launch.py
    ros2 launch jaka_dual_arm industrial_massage.launch.py velocity:=0.30  # 低速调试
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    dual_arm_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")

    # 按摩专用初始位姿 (匹配 Stage 1 waypoint: 左C7 / 右中背)
    left_initial_positions = os.path.join(
        dual_arm_share, "config", "left_massage_initial_positions.yaml")
    right_initial_positions = os.path.join(
        dual_arm_share, "config", "right_massage_initial_positions.yaml")

    # ── MoveIt 配置 (按摩模式: 无法兰, 臂在床左右Y侧) ──
    moveit_config = (
        MoveItConfigsBuilder("jaka_c5_dual", package_name="dual_arm_jaka_c5_moveit_config")
        .robot_description(
            file_path="config/jaka_c5_dual.urdf.xacro",
            mappings={
                "left_initial_positions_file": left_initial_positions,
                "right_initial_positions_file": right_initial_positions,
                "use_flange": "false",         # 按摩无法兰
                "left_arm_x": "0.53",          # 左臂X
                "left_arm_y": "-0.45",         # 左臂Y (床左侧)
                "right_arm_x": "0.69",         # 右臂X
                "right_arm_y": "0.45",         # 右臂Y (床右侧)
            },
        )
        .robot_description_semantic(file_path="config/jaka_c5_dual.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── Launch 参数 ──
    velocity_arg = DeclareLaunchArgument(
        "velocity", default_value="0.45",
        description="MoveIt velocity scaling (0.10-1.0)")
    acceleration_arg = DeclareLaunchArgument(
        "acceleration", default_value="0.45",
        description="MoveIt acceleration scaling (0.10-1.0)")

    # ── RViz (按摩专用配置) ──
    massage_rviz = os.path.join(dual_arm_share, "config", "massage_demo.rviz")
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        output="log", arguments=["-d", massage_rviz],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # ── 核心节点 ──
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="both",
        parameters=[moveit_config.robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group",
        output="screen", parameters=[moveit_config.to_dict()],
    )

    ros2_control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[os.path.join(dual_arm_share, "config", "ros2_controllers.yaml")],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="both",
    )

    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    left_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )
    right_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    # ── 按摩运行器 (延迟启动，等待所有服务就绪) ──
    massage_runner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="jaka_dual_arm",
                executable="massage_runner",
                name="industrial_massage_runner",
                output="screen",
                parameters=[
                    {"velocity_scaling": LaunchConfiguration("velocity")},
                    {"acceleration_scaling": LaunchConfiguration("acceleration")},
                ],
            ),
        ],
    )

    return LaunchDescription([
        velocity_arg,
        acceleration_arg,
        rviz_node,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        left_spawner,
        right_spawner,
        massage_runner,
    ])
