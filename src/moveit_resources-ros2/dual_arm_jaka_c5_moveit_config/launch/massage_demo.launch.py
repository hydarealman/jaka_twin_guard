from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import os


def generate_launch_description():
    package_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    massage_rviz_config = os.path.join(package_share, "config", "massage_demo.rviz")
    left_initial_positions = os.path.join(
        package_share,
        "config",
        "left_massage_initial_positions.yaml",
    )
    right_initial_positions = os.path.join(
        package_share,
        "config",
        "right_massage_initial_positions.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("jaka_c5_dual", package_name="dual_arm_jaka_c5_moveit_config")
        .robot_description(
            file_path="config/jaka_c5_dual.urdf.xacro",
            mappings={
                "left_initial_positions_file": left_initial_positions,
                "right_initial_positions_file": right_initial_positions,
            },
        )
        .robot_description_semantic(file_path="config/jaka_c5_dual.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = LaunchConfiguration("rviz_config")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=massage_rviz_config,
        description="RViz config file to load.",
    )
    velocity_scaling_arg = DeclareLaunchArgument(
        "velocity_scaling",
        default_value="0.45",
        description="MoveIt velocity scaling for the massage motion.",
    )
    acceleration_scaling_arg = DeclareLaunchArgument(
        "acceleration_scaling",
        default_value="0.45",
        description="MoveIt acceleration scaling for the massage motion.",
    )
    trajectory_time_scale_arg = DeclareLaunchArgument(
        "trajectory_time_scale",
        default_value="0.65",
        description="Scales the planned trajectory duration after collision checking.",
    )
    trajectory_start_delay_arg = DeclareLaunchArgument(
        "trajectory_start_delay",
        default_value="0.10",
        description="Delay before both trajectory controllers start moving.",
    )
    repeat_count_arg = DeclareLaunchArgument(
        "repeat_count",
        default_value="0",
        description="Number of massage cycles to run; 0 loops forever.",
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    ros2_controllers_path = os.path.join(
        package_share,
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "-c",
            "/controller_manager",
        ],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "-c",
            "/controller_manager",
        ],
    )

    massage_demo = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="dual_arm_jaka_c5_moveit_config",
                executable="dual_arm_massage_demo.py",
                name="dual_arm_massage_demo",
                output="screen",
                parameters=[
                    {
                        "marker_topic": "/rviz_visual_tools",
                        "hold_seconds": 8.0,
                        "velocity_scaling": LaunchConfiguration("velocity_scaling"),
                        "acceleration_scaling": LaunchConfiguration("acceleration_scaling"),
                        "trajectory_time_scale": LaunchConfiguration("trajectory_time_scale"),
                        "trajectory_start_delay": LaunchConfiguration("trajectory_start_delay"),
                        "repeat_count": LaunchConfiguration("repeat_count"),
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            velocity_scaling_arg,
            acceleration_scaling_arg,
            trajectory_time_scale_arg,
            trajectory_start_delay_arg,
            repeat_count_arg,
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            massage_demo,
        ]
    )
