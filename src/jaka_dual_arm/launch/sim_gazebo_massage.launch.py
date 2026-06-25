#!/usr/bin/env python3
"""Gazebo 物理仿真按摩启动 — 真实物理 + 力反馈。

与纯 RViz 仿真的区别:
  - gazebo_ros2_control → 真实关节动力学 (质量/惯量/摩擦)
  - 机械臂末端可接触人体模型，产生接触力
  - F/T 传感器数据 → 虚拟阻抗控制器
  - 最终可部署到真机 (替换 Gazebo → 真机驱动)

启动:
   ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py
   ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py gui:=false
"""

import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

# ═══ CRITICAL: Disable Qt hardware acceleration ═══
# Must be set BEFORE any Qt import, otherwise Gazebo client crashes in WSL2
# with "GLShader::compileShader: fragment shader failed to compile"
os.environ["QT_QUICK_BACKEND"] = "software"
os.environ["LIBGL_ALWAYS_SOFTWARE"] = "1"
# Qt 6.7 dropped the 'software' backend. Fall back to 'offscreen' platform.
os.environ.pop("QT_QUICK_BACKEND", None)
os.environ["QT_RENDERING_BACKEND"] = "software"

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def write_gazebo_robot_description(dual_arm_share: str, jaka_dual_share: str) -> str:
    """处理 URDF xacro → 写入临时文件。

    Gazebo 模式: use_gazebo=true, use_flange=false (按摩无法兰)
    双臂基座位置从 YAML 配置读取。
    """
    robot_xacro = os.path.join(dual_arm_share, "config", "jaka_c5_dual.urdf.xacro")

    # Read arm base positions from YAML
    import yaml
    body_yaml = os.path.join(jaka_dual_share, "config", "massage_body_params.yaml")
    with open(body_yaml, "r") as f:
        body_cfg = yaml.safe_load(f)
    arms = body_cfg.get("arms", {})
    lb = arms.get("left_base", {"x": 0.53, "y": -0.45, "z": 0.0})
    rb = arms.get("right_base", {"x": 0.69, "y": 0.45, "z": 0.0})

    gazebo_urdf = os.path.join(tempfile.gettempdir(), "jaka_c5_dual_massage_gazebo.urdf")
    mappings = {
        "use_gazebo": "true",
        "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
        "use_flange": "false",                      # 按摩模式无法兰
        "left_arm_x": str(lb.get("x", 0.53)),
        "left_arm_y": str(lb.get("y", -0.45)),
        "right_arm_x": str(rb.get("x", 0.69)),
        "right_arm_y": str(rb.get("y", 0.45)),
    }
    document = xacro.process_file(robot_xacro, mappings=mappings)
    with open(gazebo_urdf, "w", encoding="utf-8") as f:
        f.write(document.toprettyxml(indent="  "))
    return gazebo_urdf


def generate_launch_description():
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")
    dual_arm_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    gazebo_share = get_package_share_directory("gazebo_ros")

    gazebo_robot_desc_path = write_gazebo_robot_description(
        dual_arm_share, jaka_dual_share
    )

    # ── MoveIt 配置 (Gazebo 模式, 按摩无法兰) ──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_dual",
            package_name="dual_arm_jaka_c5_moveit_config",
        )
        .robot_description(
            file_path=os.path.join(dual_arm_share, "config", "jaka_c5_dual.urdf.xacro"),
            mappings={
                "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
                "use_gazebo": "true",
                "use_flange": "false",
                "left_arm_x": "0.53",
                "left_arm_y": "-0.45",
                "right_arm_x": "0.69",
                "right_arm_y": "0.45",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(dual_arm_share, "config", "jaka_c5_dual.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(dual_arm_share, "config", "moveit_controllers.yaml")
        )
        .joint_limits(
            file_path=os.path.join(dual_arm_share, "config", "joint_limits.yaml")
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── 按摩世界文件 ──
    world_path = os.path.join(jaka_dual_share, "worlds", "massage.world")

    # ── Gazebo (带按摩世界) ──
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "gui": LaunchConfiguration("gui"),
            # Extra Gazebo args for WSL2 compatibility
            "extra_gazebo_args": "-s libgazebo_ros_init.so -s libgazebo_ros_factory.so",
        }.items(),
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

    # Spawn robot into Gazebo
    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file", gazebo_robot_desc_path,
                    "-entity", "jaka_c5_dual",
                    "-timeout", "120",
                    "-x", "0", "-y", "0", "-z", "0",
                ],
                output="screen",
            )
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

    # RViz (显示 Gazebo 中的机器人状态 + 规划场景)
    rviz_config = os.path.join(
        dual_arm_share, "config", "massage_demo.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # Massage runner (延迟启动, 等所有服务就绪)
    massage_runner = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="jaka_dual_arm",
                executable="massage_runner",
                name="massage_runner",
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show Gazebo GUI"
        ),
        LogInfo(msg=["[Gazebo Massage] Starting physics simulation with body model..."]),
        LogInfo(msg=["[Gazebo Massage] QT_QUICK_BACKEND=software (WSL2 compat)"]),
        gazebo,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        left_spawner,
        right_spawner,
        spawn_robot,
        rviz,
        massage_runner,
    ])
