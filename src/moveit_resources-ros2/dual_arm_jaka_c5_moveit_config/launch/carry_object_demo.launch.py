from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("dual_arm_jaka_c5_moveit_config")
    carry_rviz_config = os.path.join(package_share, "config", "carry_demo.rviz")

    base_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "demo.launch.py")
        ),
        launch_arguments={"rviz_config": carry_rviz_config}.items(),
    )

    carry_demo = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="dual_arm_jaka_c5_moveit_config",
                executable="dual_arm_carry_demo.py",
                name="dual_arm_carry_demo",
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

    return LaunchDescription([base_demo, carry_demo])
