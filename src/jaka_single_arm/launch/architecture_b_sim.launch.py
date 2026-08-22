#!/usr/bin/env python3
"""Architecture B simulation: simulated vision sends targets to a virtual C board.

The virtual board validates framing, ACK/result handling and target payloads.
It intentionally does not animate the robot: in architecture B, IK, trajectory
generation and motor control belong to the real C-board firmware.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("jaka_single_arm")
    start_serial = LaunchConfiguration("start_virtual_serial")
    host_port = "/tmp/jaka_architecture_b_host"
    board_port = "/tmp/jaka_architecture_b_board"

    # Remove only this launch's fixed symlinks. socat recreates and owns them.
    for path in (host_port, board_port):
        if os.path.islink(path):
            os.unlink(path)

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
            "start_moveit": "false",
            "run_task": "false",
        }.items(),
    )

    virtual_ports = ExecuteProcess(
        cmd=[
            "socat", "-d", "-d",
            f"pty,raw,echo=0,link={host_port}",
            f"pty,raw,echo=0,link={board_port}",
        ],
        condition=IfCondition(start_serial),
        output="screen",
    )
    board = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "jaka_single_arm", "serial_board_emulator",
                    "--port", board_port,
                    "--baudrate", "115200",
                    "--execution-delay", "0.5",
                ],
                condition=IfCondition(start_serial),
                output="screen",
            )
        ],
    )
    target_pipeline = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="jaka_single_arm",
                executable="fruit_target_node",
                name="architecture_b_sim_target_node",
                parameters=[{
                    "use_sim_time": True,
                    "camera_type": "gazebo",
                    "localizer_backend": "geometry",
                    "output_frame": "world",
                    "camera_info_topic": "/camera/camera/color/camera_info",
                    "allow_scene_fallback": True,
                    "force_table_center_z": True,
                    "enable_table_z_fallback": True,
                }],
                output="screen",
            ),
        ],
    )
    serial_bridge = TimerAction(
        period=33.0,
        actions=[
            Node(
                package="jaka_single_arm",
                executable="serial_fruit_target_bridge",
                name="architecture_b_sim_serial_bridge",
                condition=IfCondition(start_serial),
                parameters=[
                    os.path.join(share, "config", "architecture_b_serial.yaml"),
                    {
                        "serial_port": host_port,
                        "baudrate": 115200,
                        # The emulator will ACK/validate every command. Do not
                        # gate the simulation on an asynchronous READY frame.
                        "require_ready": False,
                    },
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_image_view", default_value="true"),
        DeclareLaunchArgument(
            "start_virtual_serial",
            default_value="true",
            description="Start socat and the virtual C-board protocol emulator",
        ),
        LogInfo(msg="============================================================"),
        LogInfo(msg="[MODE] Architecture B / SIMULATION"),
        LogInfo(msg="[SAFE] Gazebo + virtual serial only; no physical USB port"),
        LogInfo(msg="[FLOW] Sim RGB-D -> stable FruitTarget -> virtual serial -> C-board emulator"),
        LogInfo(msg="[NOTE] Robot motion belongs to future architecture-B firmware"),
        LogInfo(msg="============================================================"),
        virtual_ports,
        board,
        gazebo,
        target_pipeline,
        serial_bridge,
    ])
