import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def write_gazebo_robot_description(package_share):
    robot_xacro = os.path.join(package_share, "config", "jaka_c5_dual.urdf.xacro")
    gazebo_urdf = os.path.join(tempfile.gettempdir(), "jaka_c5_dual_gazebo.urdf")
    mappings = {
        "use_gazebo": "true",
        "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
    }
    document = xacro.process_file(robot_xacro, mappings=mappings)
    with open(gazebo_urdf, "w", encoding="utf-8") as robot_file:
        robot_file.write(document.toprettyxml(indent="  "))
    return gazebo_urdf


def generate_launch_description():
    package_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    gazebo_share = get_package_share_directory("gazebo_ros")
    world_path = os.path.join(package_share, "worlds", "dual_arm_carry.world")
    gazebo_robot_description = write_gazebo_robot_description(package_share)

    moveit_config = (
        MoveItConfigsBuilder("jaka_c5_dual", package_name="dual_arm_jaka_c5_moveit_config")
        .robot_description(
            file_path="config/jaka_c5_dual.urdf.xacro",
            mappings={
                "use_gazebo": "true",
                "hardware_plugin": "gazebo_ros2_control/GazeboSystem",
            },
        )
        .robot_description_semantic(file_path="config/jaka_c5_dual.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
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
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-file",
                    gazebo_robot_description,
                    "-entity",
                    "jaka_c5_dual",
                    "-timeout",
                    "120",
                    "-x",
                    "0",
                    "-y",
                    "0",
                    "-z",
                    "0",
                ],
                output="screen",
            )
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
    )

    carry_demo = TimerAction(
        period=14.0,
        actions=[
            Node(
                package="dual_arm_jaka_c5_moveit_config",
                executable="gazebo_carry_demo.py",
                name="gazebo_carry_demo",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo Classic graphical client.",
            ),
            LogInfo(msg=["Loading Gazebo world: ", world_path]),
            gazebo,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            spawn_robot,
            carry_demo,
        ]
    )
