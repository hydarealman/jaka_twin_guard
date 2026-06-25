#!/usr/bin/env python3
"""RViz 仿真启动 — 使用 mock_components，无需 Gazebo。

启动:
   ros2 launch jaka_single_arm sim_rviz.launch.py

ROBOT MODEL SWAPPING:
   ros2 launch jaka_single_arm sim_rviz.launch.py \
       description_package:=my_robot_description \
       description_file:=config/my_robot.urdf.xacro \
       srdf_file:=config/my_robot.srdf

   默认使用 JAKA C5 (single_arm_jaka_c5_pick_place 包)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    # ── 机器人模型参数（可替换）──
    desc_pkg = LaunchConfiguration("description_package")
    desc_file = LaunchConfiguration("description_file")
    srdf_file = LaunchConfiguration("srdf_file")
    rviz_config_file = LaunchConfiguration("rviz_config")
    scene = LaunchConfiguration("scene")

    desc_pkg_arg = DeclareLaunchArgument(
        "description_package",
        default_value="single_arm_jaka_c5_pick_place",
        description="Package containing robot URDF/SRDF/configs",
    )
    desc_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value="config/jaka_c5_pick_place.urdf.xacro",
        description="URDF/xacro file path relative to description_package share",
    )
    srdf_file_arg = DeclareLaunchArgument(
        "srdf_file",
        default_value="config/jaka_c5_pick_place.srdf",
        description="SRDF file path relative to description_package share",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="",
        description="RViz config file (empty = auto-detect from single_arm_jaka_c5_pick_place)",
    )
    scene_arg = DeclareLaunchArgument(
        "scene",
        default_value="a",
        description="Scene variant",
    )

    # ── 解析包路径 ──
    # 使用 OpaqueFunction 还是预计算？用 Python launch 的函数式
    # 这里用非 OpaqueFunction 方式
    pkg_share = get_package_share_directory("single_arm_jaka_c5_pick_place")
    jaka_single_share = get_package_share_directory("jaka_single_arm")

    # 默认值：如果 launch 参数未覆盖，使用这些
    default_desc_pkg = "single_arm_jaka_c5_pick_place"
    default_desc_file = os.path.join(pkg_share, "config", "jaka_c5_pick_place.urdf.xacro")
    default_srdf_file = os.path.join(pkg_share, "config", "jaka_c5_pick_place.srdf")
    default_rviz = os.path.join(pkg_share, "config", "pick_place.rviz")
    default_initial_positions = os.path.join(pkg_share, "config", "initial_positions.yaml")

    # ── MoveIt 配置 ──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_pick_place",
            package_name=default_desc_pkg,
        )
        .robot_description(
            file_path=default_desc_file,
            mappings={
                "initial_positions_file": default_initial_positions,
                "hardware_plugin": "mock_components/GenericSystem",
            },
        )
        .robot_description_semantic(file_path=default_srdf_file)
        .trajectory_execution(
            file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml")
        )
        .joint_limits(
            file_path=os.path.join(pkg_share, "config", "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── 节点 ──

    # MoveIt core
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", default_rviz],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    # TF broadcaster
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control (mock hardware)
    ros2_controllers_path = os.path.join(pkg_share, "config", "ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Controller spawners
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "30"],
    )

    # Pick-and-place runner (delayed)
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
                ],
            ),
        ],
    )

    return LaunchDescription([
        desc_pkg_arg,
        desc_file_arg,
        srdf_file_arg,
        rviz_arg,
        scene_arg,
        rviz_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        pick_place_runner,
    ])
