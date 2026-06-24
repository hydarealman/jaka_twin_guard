#!/usr/bin/env python3
"""RViz 仿真启动 — 使用 mock_components，无需 Gazebo。

启动:
   ros2 launch jaka_dual_arm sim_rviz.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("jaka_dual_arm")
    dual_arm_share = get_package_share_directory(
        "dual_arm_jaka_c5_moveit_config"
    )

    # ── MoveIt 配置 ──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_dual",
            package_name="dual_arm_jaka_c5_moveit_config",
        )
        .robot_description(
            file_path=f"{dual_arm_share}/config/jaka_c5_dual.urdf.xacro",
            mappings={
                "hardware_plugin": "mock_components/GenericSystem",
                "use_gazebo": "false",
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

    # ── 参数文件路径 ──

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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(dual_arm_share, "config", "ros2_controllers.yaml")
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
    )
    right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
    )

    # ── 场景选择 ──
    scene_arg = DeclareLaunchArgument(
        "scene",
        default_value="a",
        description="场景选择: a=桌到桌, b=料框拣选, c=传送带分拣",
    )

    # ── 搬运任务运行器 ──
    # YAML 配置文件由 __main__.py 内部通过 ament_index 加载
    carry_runner = TimerAction(
        period=6.0,
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

    # ── RViz2 ──
    rviz_config = os.path.join(dual_arm_share, "config", "carry_demo.rviz")
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

    return LaunchDescription([
        scene_arg,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        left_spawner,
        right_spawner,
        rviz_node,
        carry_runner,
    ])
