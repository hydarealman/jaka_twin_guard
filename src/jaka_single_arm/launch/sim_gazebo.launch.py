#!/usr/bin/env python3
"""Gazebo 物理仿真启动 — 使用 gazebo_ros2_control，带真实物理。

启动:
   ros2 launch jaka_single_arm sim_gazebo.launch.py
   ros2 launch jaka_single_arm sim_gazebo.launch.py gui:=false    # headless

Gazebo 模式下:
  - camera_type 自动设置为 "gazebo"
  - 固定眼在手外 RGB-D 相机发布与 RealSense 相同的话题
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
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def write_gazebo_robot_description(pkg_share: str) -> str:
    """Process xacro into a compact, comment-free URDF for Gazebo.

    gazebo_ros2_control on Humble passes robot_description through the ROS
    parameter override parser. Colons and multiline text inside XML comments
    can be interpreted as YAML and reject an otherwise valid URDF, so the
    simulation uses the same XML with comments and inter-tag whitespace
    removed.
    """
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
    robot_xml = document.toxml()
    robot_xml = re.sub(r"<!--.*?-->", "", robot_xml, flags=re.DOTALL)
    robot_xml = re.sub(r">\s+<", "><", robot_xml).strip()
    with open(gazebo_urdf, "w", encoding="utf-8") as f:
        f.write(robot_xml)
    print(f"[Launch] Gazebo URDF written to: {gazebo_urdf}")
    return gazebo_urdf


def generate_launch_description():
    jaka_single_share = get_package_share_directory("jaka_single_arm")
    robot_pkg_share = get_package_share_directory("single_arm_jaka_c5_pick_place")
    description_share = get_package_share_directory("jaka_c5_description")
    start_rviz = LaunchConfiguration("start_rviz")
    start_image_view = LaunchConfiguration("start_image_view")
    start_detector = LaunchConfiguration("start_detector")
    start_moveit = LaunchConfiguration("start_moveit")
    run_task = LaunchConfiguration("run_task")

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
            file_path=gazebo_robot_desc,
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

    # ═══ WSL2: 禁用 GPU 硬件加速，使用软件渲染 ═══
    wsl_env = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    wsl_env2 = SetEnvironmentVariable("QT_QUICK_BACKEND", "software")

    # ═══ Gazebo 资源路径：让苹果贴图材质（worlds/materials）可被 OGRE 加载 ═══
    worlds_dir = os.path.join(jaka_single_share, "worlds")
    existing_res = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    gazebo_res_env = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH",
        os.pathsep.join(filter(None, [
            worlds_dir,
            existing_res,
            "/usr/share/gazebo-11",
        ])),
    )
    gazebo_model_env = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.pathsep.join(filter(None, [
            os.environ.get("GAZEBO_MODEL_PATH", ""),
            os.path.dirname(description_share),
            os.path.dirname(robot_pkg_share),
            "/usr/share/gazebo-11/models",
        ])),
    )
    gazebo_plugin_env = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        os.pathsep.join(filter(None, [
            os.environ.get("GAZEBO_PLUGIN_PATH", ""),
            "/opt/ros/humble/lib",
            "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
        ])),
    )

    # ═══ 阶段 0: Gazebo server/client ═══
    # Start the standard Gazebo ROS processes directly. On some Humble + WSL
    # installations the nested gazebo.launch.py include stalls before sibling
    # TimerActions start, while the same gzserver command is reliable.
    world_path = os.path.join(jaka_single_share, "worlds", "pick_place.world")
    print(f"[Launch] World path: {world_path}")

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver", world_path, "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            "-s", "libgazebo_ros_force_system.so",
        ],
        output="both",
    )
    gzclient = ExecuteProcess(
        cmd=["gzclient", "--verbose"],
        output="both",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # ── 阶段 1: Robot state publisher (3s 后) ──
    # gazebo_ros2_control creates /controller_manager inside gzserver after the
    # robot is spawned. A separate ros2_control_node would conflict with it.
    robot_and_control = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[Launch] Starting robot_state_publisher + camera TF..."),
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
                package="tf2_ros",
                executable="static_transform_publisher",
                name="eye_to_hand_camera_body_tf",
                arguments=[
                    "--x", "1.25", "--y", "-0.80", "--z", "1.05",
                    "--roll", "0", "--pitch", "0.611", "--yaw", "2.268",
                    "--frame-id", "world",
                    "--child-frame-id", "eye_to_hand_camera_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="eye_to_hand_color_optical_tf",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
                    "--frame-id", "eye_to_hand_camera_link",
                    "--child-frame-id", "camera_color_optical_frame",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="eye_to_hand_depth_optical_tf",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "camera_color_optical_frame",
                    "--child-frame-id", "camera_depth_optical_frame",
                ],
                output="screen",
            ),
            Node(
                package="jaka_single_arm",
                executable="gazebo_scene_bridge",
                name="gazebo_scene_bridge",
                parameters=[{"use_sim_time": True}],
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
                condition=IfCondition(start_moveit),
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
                condition=IfCondition(start_rviz),
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

    # ── 阶段 5.5: 苹果好坏识别节点 (28s 后，相机已发布图像) ──
    fruit_detector = TimerAction(
        period=28.0,
        actions=[
            LogInfo(msg="[Launch] Starting fruit_detector_node (YOLOv8 苹果好坏识别)..."),
            Node(
                package="jaka_single_arm",
                executable="fruit_detector_node",
                name="fruit_detector_node",
                condition=IfCondition(start_detector),
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "image_topic": "/camera/camera/color/image_raw",
                }],
            ),
        ],
    )

    # 独立相机窗口：即使 YOLO 没有检出，也能直接确认虚拟相机图像是否正常。
    image_view = TimerAction(
        period=29.0,
        actions=[
            LogInfo(msg="[Launch] Starting RGB camera view..."),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="sim_camera_view",
                condition=IfCondition(start_image_view),
                arguments=["/camera/camera/color/image_raw"],
                output="log",
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
                condition=IfCondition(run_task),
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"camera_type": "gazebo"},
                    {"perception_output_frame": "world"},
                    {"camera_info_topic": "/camera/camera/color/camera_info"},
                    {"allow_scene_fallback": True},
                ],
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Show Gazebo GUI"),
        DeclareLaunchArgument(
            "start_rviz", default_value="true",
            description="Start RViz scene and point-cloud view"),
        DeclareLaunchArgument(
            "start_image_view", default_value="true",
            description="Open a separate RGB camera image window"),
        DeclareLaunchArgument(
            "start_detector", default_value="true",
            description="Run Healthy/Unhealthy YOLO inference"),
        DeclareLaunchArgument(
            "start_moveit", default_value="true",
            description="Start MoveIt move_group"),
        DeclareLaunchArgument(
            "run_task", default_value="true",
            description="Run the autonomous pick-and-place behaviour tree"),
        LogInfo(msg=["[Launch] ===== Gazebo Physical Simulation ====="]),
        LogInfo(msg=["[Launch] Camera: fixed eye-to-hand RGB-D (RealSense-compatible topics)"]),
        LogInfo(msg=["[Launch] WSL2: LIBGL_ALWAYS_SOFTWARE=1"]),
        LogInfo(msg=["[Launch] Startup: gzserver → gzclient → spawn → controllers → MoveIt → RViz → runner"]),
        wsl_env,
        wsl_env2,
        gazebo_res_env,
        gazebo_model_env,
        gazebo_plugin_env,
        gzserver,
        gzclient,
        robot_and_control,
        spawn_robot,
        controller_spawners,
        move_group,
        rviz_node,
        fruit_detector,
        image_view,
        pick_place_runner,
    ])
