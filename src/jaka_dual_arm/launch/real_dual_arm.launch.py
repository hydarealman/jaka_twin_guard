#!/usr/bin/env python3
"""真机双臂启动 — JAKA C5 真实机械臂 + F/T 传感器 + 安全 PLC。

启动前检查清单:
    1. 安全 PLC 处于 AUTO 模式
    2. 机械臂紧急停止按钮未按下
    3. F/T 传感器电源接通
    4. 机械臂速度倍率设置到 <20%
    5. 工作区域内无人员/障碍物

启动:
    ros2 launch jaka_dual_arm real_dual_arm.launch.py
    ros2 launch jaka_dual_arm real_dual_arm.launch.py scene:=b
    ros2 launch jaka_dual_arm real_dual_arm.launch.py speed:=0.10  # 10% 速度调试

架构:
    本 launch 文件启动:
    - JAKA 机械臂驱动节点 (ethernet/IP → ros2_control)
    - F/T 传感器驱动节点
    - 安全 PLC 监控节点
    - Cartesian 阻抗控制器 (可选)
    - MoveIt2 move_group (规划 + 碰撞检测)
    - 搬运任务运行器 (bt_runner)

参考:
  - franka_ros2 franka.launch.py — 真机启动模板
  - UR ROS2 Driver ur_control.launch.py — 驱动 + 安全 + 力控
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    dual_arm_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")

    # ── Launch Arguments ──
    scene_arg = DeclareLaunchArgument(
        "scene", default_value="a",
        description="场景: a=桌到桌, b=料框拣选, c=传送带分拣",
    )
    speed_arg = DeclareLaunchArgument(
        "speed", default_value="0.15",
        description="速度倍率: 0.05-1.0 (首次运行建议 0.10)",
    )
    force_control_arg = DeclareLaunchArgument(
        "force_control", default_value="true",
        description="启用 F/T 力控: true (阻抗) / false (纯位置)",
    )
    use_cartesian_controller_arg = DeclareLaunchArgument(
        "use_cartesian_controller", default_value="true",
        description="使用 Cartesian 阻抗控制器 (需要 F/T 传感器)",
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run", default_value="false",
        description="空跑模式: 只规划不执行 (验证轨迹安全性)",
    )

    # ── MoveIt 配置 (真机模式) ──
    moveit_config = (
        MoveItConfigsBuilder(
            "jaka_c5_dual",
            package_name="dual_arm_jaka_c5_moveit_config",
        )
        .robot_description(
            file_path=f"{dual_arm_share}/config/jaka_c5_dual.urdf.xacro",
            mappings={
                "hardware_plugin": "jaka_c5_hardware/JakaC5SystemPositionOnly",  # 真机硬件插件
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

    # ── 节点 ──

    # Robot State Publisher (加载 URDF, 发布 TF)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # MoveGroup (MoveIt2 规划 + 碰撞检测)
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # ros2_control 节点 (硬件接口管理器)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(dual_arm_share, "config", "ros2_controllers.yaml"),
            os.path.join(jaka_dual_share, "config", "real_hardware_params.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # 控制器 spawner — 位置控制
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )
    right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # TODO(Step 3 真机): F/T 传感器驱动 — 需要实现 ft_sensor_driver 可执行文件
    # 当前为架构预留，实际启动时自动跳过 (condition=False)
    #
    # left_ft_driver = Node(
    #     package="jaka_dual_arm",
    #     executable="ft_sensor_driver",
    #     ...
    #     condition=IfCondition(LaunchConfiguration("force_control")),
    # )

    # Cartesian 阻抗控制器 (可选)
    cartesian_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_cartesian_impedance_controller", "-c", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_cartesian_controller")),
        output="screen",
    )
    cartesian_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_cartesian_impedance_controller", "-c", "/controller_manager"],
        condition=IfCondition(LaunchConfiguration("use_cartesian_controller")),
        output="screen",
    )

    # TODO(Step 3 真机): 独立安全监控节点 — 需要实现 safety_monitor 可执行文件
    # 当前安全监控内嵌在 bt_runner.py 的 SafetyMonitor 中运行
    #
    # safety_monitor = Node(
    #     package="jaka_dual_arm",
    #     executable="safety_monitor",
    #     ...
    # )

    # 搬运任务运行器 (延迟启动，等待所有服务就绪)
    carry_runner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="jaka_dual_arm",
                executable="carry_task_runner",
                name="carry_task_runner",
                output="screen",
                parameters=[
                    {"use_real_hardware": True},
                    {"speed_override": LaunchConfiguration("speed")},
                    {"force_control": LaunchConfiguration("force_control")},
                    {"dry_run": LaunchConfiguration("dry_run")},
                ],
                arguments=["--scene", LaunchConfiguration("scene")],
            ),
        ],
    )

    return LaunchDescription([
        LogInfo(msg=["\n┌──────────────────────────────────────┐"]),
        LogInfo(msg=["│ JAKA C5 Dual-Arm — REAL HARDWARE     │"]),
        LogInfo(msg=["│ 确认: 安全PLC=AUTO, ESTOP未按下       │"]),
        LogInfo(msg=["│ 确认: 工作区域无人员                  │"]),
        LogInfo(msg=["│ 确认: 速度倍率≤20%                    │"]),
        LogInfo(msg=["└──────────────────────────────────────┘"]),
        scene_arg,
        speed_arg,
        force_control_arg,
        use_cartesian_controller_arg,
        dry_run_arg,
        robot_state_publisher,
        move_group,
        ros2_control_node,
        jsb_spawner,
        left_spawner,
        right_spawner,
        # left_ft_driver,       # TODO(Step 3 真机): 待实现 F/T 驱动
        # right_ft_driver,
        cartesian_left_spawner,
        cartesian_right_spawner,
        # safety_monitor,       # TODO(Step 3 真机): 待实现独立安全节点
        carry_runner,
    ])
