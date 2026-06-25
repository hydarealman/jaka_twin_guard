#!/usr/bin/env python3
"""Gazebo 物理仿真按摩 — 真实质量/惯量/重力/接触力。

架构: Gazebo (物理引擎) → /joint_states → MoveIt → 规划 → /joint_trajectory → ros2_control → Gazebo
       robot_state_publisher → /robot_description → spawn_entity.py → Gazebo 加载机器人
       F/T 传感器 → /ft_sensor → VirtualImpedanceController (真实数据闭环)

启动:
   ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py           # 带 GUI
   ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py gui:=false # headless

启动时序:
  0s  → Gazebo (gzserver + gzclient)
  2s  → robot_state_publisher + ros2_control_node
  5s  → spawn robot
  10s → controller spawners + move_group
  12s → RViz
  15s → massage_runner
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    jaka_dual_share = get_package_share_directory("jaka_dual_arm")
    dual_arm_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    gazebo_share = get_package_share_directory("gazebo_ros")

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
                # Fix B022: 必须显式传入按摩初始位置文件，否则 xacro 默认使用
                # left/right_initial_positions.yaml（搬运 demo 配置），导致机械臂
                # 以 joint_5=2.374rad 等极端角度启动，arm 折叠至地面以下不可见
                "left_initial_positions_file": os.path.join(
                    dual_arm_share, "config", "left_massage_initial_positions.yaml"),
                "right_initial_positions_file": os.path.join(
                    dual_arm_share, "config", "right_massage_initial_positions.yaml"),
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

    # ═══ 修复 B008: 清除 URDF 中的 XML 注释 ═══
    # 根因: URDF 注释中的 `:` 字符触发 rcl_parse_arguments() 解析错误
    # 参考: https://github.com/ros-controls/gazebo_ros2_control/issues/295
    if "robot_description" in moveit_config.robot_description:
        raw_urdf = moveit_config.robot_description["robot_description"]
        clean_urdf = re.sub(r'<!--.*?-->', '', raw_urdf, flags=re.DOTALL)

        # ═══ 修复 B021: 将 package://jaka_c5_description/ 替换为 file:// 绝对路径 ═══
        # 根因: Gazebo 通过 ament 资源索引解析 package:// URI。若 jaka_c5_description
        #       未被独立 colcon build，不在 ament 索引中，STL mesh 无法加载，机械臂
        #       物理存在但视觉不可见（所有 Link 无 STL 外观）。
        # 修复: 在传给 Gazebo 之前直接替换为 file:// 绝对路径，绕过 ament 索引依赖。
        try:
            jaka_c5_desc_share = get_package_share_directory("jaka_c5_description")
            clean_urdf = clean_urdf.replace(
                "package://jaka_c5_description/",
                f"file://{jaka_c5_desc_share}/",
            )
        except Exception:
            # jaka_c5_description 未安装时保留 package:// URI，依赖 ament 解析
            pass

        moveit_config.robot_description["robot_description"] = clean_urdf

    world_path = os.path.join(jaka_dual_share, "worlds", "massage.world")

    # ═══ WSL2: 禁用 GPU 硬件加速，使用软件渲染 ═══
    wsl_env = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    wsl_env2 = SetEnvironmentVariable("QT_QUICK_BACKEND", "software")

    # ═══ Gazebo 资源路径 — 必须在 gzserver 前设置 ═══
    # GazeboRosPaths.get_paths() 返回空字符串，需手动指定系统路径
    # 否则报: "Unable to find shader lib" → 渲染失败 → gzserver 崩溃 (exit 255)
    # ⚠️ 关键: 必须保留已有的 ROS2 包路径 (含 STL mesh 目录)，否则机械臂无模型！
    _existing_gz = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    _system_gz = "/usr/share/gazebo-11"
    if _existing_gz:
        _gz_resource_full = f"{_system_gz}:{_existing_gz}"
    else:
        _gz_resource_full = _system_gz
    gz_resource = SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", _gz_resource_full)
    # ⚠️ Fix B020: 同 B009 一样，不能硬覆盖，必须 append 保留已有值
    _existing_plugin = os.environ.get("GAZEBO_PLUGIN_PATH", "")
    _sys_plugin = "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins"
    _gz_plugin_full = f"{_sys_plugin}:{_existing_plugin}" if _existing_plugin else _sys_plugin
    gz_plugin = SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", _gz_plugin_full)

    _existing_model = os.environ.get("GAZEBO_MODEL_PATH", "")
    _sys_model = "/usr/share/gazebo-11/models"
    _gz_model_full = f"{_sys_model}:{_existing_model}" if _existing_model else _sys_model
    gz_model = SetEnvironmentVariable("GAZEBO_MODEL_PATH", _gz_model_full)

    gz_no_db = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")  # 禁用在线模型下载(避免WSL2网络卡顿)

    # ═══ Gazebo Server (物理引擎) ═══
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "pause": "false",
        }.items(),
    )

    # ═══ Gazebo Client (GUI, 可选) ═══
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    ros2_controllers_path = os.path.join(
        dual_arm_share, "config", "ros2_controllers.yaml"
    )

    # ── 阶段 1: 基础节点 (2s 后，确保 Gazebo 完全就绪) ──
    # ⚠️ Fix B019: 不启动独立的 ros2_control_node!
    # libgazebo_ros2_control.so 插件在 robot spawn 时自己创建 controller_manager。
    # 若同时运行外部 ros2_control_node，/controller_manager 命名空间冲突 →
    # 插件初始化失败 → spawn_entity.py 超时 → 机械臂不出现在 Gazebo。
    robot_and_control = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="[Massage] Starting robot_state_publisher..."),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[moveit_config.robot_description,
                            {"use_sim_time": True}],
                output="screen",
            ),
            # ros2_control_node 已移除 — controller_manager 由
            # libgazebo_ros2_control.so Gazebo 插件在 spawn 时内部创建。
        ],
    )

    # ── 阶段 2: Spawn robot (5s 后, robot_description 已发布 + Gazebo 服务就绪) ──
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[Massage] Spawning robot entity in Gazebo..."),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "/robot_description",
                    "-entity", "jaka_c5_dual",
                    "-timeout", "60",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ],
    )

    # ── 阶段 3: Controller Spawners (10s 后，robot 已 spawn + ros2_control ready) ──
    controller_spawners = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[Massage] Spawning controllers (jsb + left + right)..."),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                           "--controller-manager-timeout", "30"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["left_arm_controller", "-c", "/controller_manager",
                           "--controller-manager-timeout", "30"],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["right_arm_controller", "-c", "/controller_manager",
                           "--controller-manager-timeout", "30"],
                output="screen",
            ),
        ],
    )

    # ── 阶段 4: MoveIt (10s 后, 与 controllers 并行) ──
    move_group = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[Massage] Starting move_group..."),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(),
                            {"use_sim_time": True}],
            ),
        ],
    )

    # ── 阶段 5: RViz (12s 后) ──
    rviz_config = os.path.join(dual_arm_share, "config", "massage_demo.rviz")
    rviz = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[Massage] Starting RViz..."),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                # Fix B023: RViz 需要 robot_description_semantic (SRDF) 参数
                # 否则报 "Could not find parameter robot_description_semantic"
                # → RobotModel display 和 PlanningScene display 都无法加载
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    {"use_sim_time": True},
                ],
            ),
        ],
    )

    # ── 阶段 6: Runner (15s 后, 等待 move_group + controllers 完全就绪) ──
    massage_runner = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg="[Massage] Starting massage_runner..."),
            Node(
                package="jaka_dual_arm",
                executable="massage_runner",
                name="massage_runner",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show Gazebo GUI"
        ),
        LogInfo(msg=["[Massage] ===== Gazebo Physics Simulation ====="]),
        LogInfo(msg=["[Massage] WSL2: LIBGL_ALWAYS_SOFTWARE=1"]),
        wsl_env,
        wsl_env2,
        gz_resource,
        gz_plugin,
        gz_model,
        gz_no_db,
        gzserver,
        gzclient,
        robot_and_control,
        spawn_robot,
        controller_spawners,
        move_group,
        rviz,
        massage_runner,
    ])
