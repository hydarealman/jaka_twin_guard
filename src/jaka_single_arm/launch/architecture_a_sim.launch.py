#!/usr/bin/env python3
"""Architecture A simulation: PC-side MoveIt planning controls Gazebo.

This entry point never opens a physical serial port.  It validates the
camera/perception/behavior/MoveIt side of architecture A with Gazebo's
ros2_control implementation.  The serial protocol has its own emulator E2E
test because a simulated C board cannot reproduce motor dynamics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("jaka_single_arm"),
                "launch",
                "sim_gazebo.launch.py",
            ])
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "start_rviz": LaunchConfiguration("start_rviz"),
            "start_image_view": LaunchConfiguration("start_image_view"),
            "start_moveit": "true",
            "run_task": LaunchConfiguration("run_task"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_image_view", default_value="true"),
        DeclareLaunchArgument(
            "run_task",
            default_value="true",
            description="Run the PC-side MoveIt pick-and-place behavior",
        ),
        LogInfo(msg="============================================================"),
        LogInfo(msg="[MODE] Architecture A / SIMULATION"),
        LogInfo(msg="[SAFE] Gazebo only; no /dev/ttyUSB* device will be opened"),
        LogInfo(msg="[FLOW] Sim RGB-D -> perception -> MoveIt -> Gazebo control"),
        LogInfo(msg="============================================================"),
        gazebo,
    ])
