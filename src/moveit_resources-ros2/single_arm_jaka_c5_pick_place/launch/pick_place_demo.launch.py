from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    package_share = get_package_share_directory("single_arm_jaka_c5_pick_place")
    rviz_config_file = os.path.join(package_share, "config", "pick_place.rviz")
    initial_positions = os.path.join(package_share, "config", "initial_positions.yaml")

    moveit_config = (
        MoveItConfigsBuilder("jaka_c5_pick_place", package_name="single_arm_jaka_c5_pick_place")
        .robot_description(
            file_path="config/jaka_c5_pick_place.urdf.xacro",
            mappings={
                "initial_positions_file": initial_positions,
            },
        )
        .robot_description_semantic(file_path="config/jaka_c5_pick_place.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # MoveIt core server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config_file,
        description="RViz config file to load.",
    )
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

    # TF broadcaster
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control (mock hardware)
    ros2_controllers_path = os.path.join(package_share, "config", "ros2_controllers.yaml")
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
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "-c",
            "/controller_manager",
        ],
    )

    # Demo script (delayed to allow everything to start up)
    pick_place_demo = TimerAction(
        period=4.5,
        actions=[
            Node(
                package="single_arm_jaka_c5_pick_place",
                executable="pick_place_demo.py",
                name="pick_place_demo",
                output="screen",
                parameters=[
                    {
                        "marker_topic": "/rviz_visual_tools",
                        "action_server_timeout_sec": 120.0,
                        "hold_seconds": 8.0,
                    }
                ],
            )
        ],
    )

    # 模拟深度相机（眼在手上，发布 /camera/depth/points 点云）
    # 当前 Demo 暂未使用相机数据（基于已知水果位置 IK），
    # 但保留此节点以便后续集成视觉引导抓取
    simulated_camera = Node(
        package="single_arm_jaka_c5_pick_place",
        executable="simulated_camera.py",
        name="simulated_camera",
        output="screen",
        parameters=[
            {
                "publish_rate": 10.0,
                "frame_id": "camera_depth_frame",
            }
        ],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            pick_place_demo,
            simulated_camera,
        ]
    )
