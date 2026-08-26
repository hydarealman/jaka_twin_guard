#!/usr/bin/env python3
"""URDF-only joint zero, direction and hard-limit inspection.

This launch deliberately excludes MoveIt execution, ros2_control, serial,
cameras and task nodes.  joint_state_publisher_gui is the sole source of
/joint_states.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_share = get_package_share_directory("fruit_arm_moveit_config")
    package_share = get_package_share_directory("fruit_picking_arm")
    xacro_file = os.path.join(robot_share, "config", "fruit_picking_arm.urdf.xacro")
    initial_positions = os.path.join(robot_share, "config", "initial_positions.yaml")
    rviz_config = os.path.join(package_share, "config", "model_joint_check.rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " initial_positions_file:=",
                    initial_positions,
                    " hardware_plugin:=mock_components/GenericSystem",
                ]
            ),
            value_type=str,
        )
    }

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="model_check_robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="model_check_joint_state_publisher_gui",
                parameters=[robot_description, {"rate": 30.0}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="model_check_rviz",
                arguments=["-d", rviz_config, "--qwindowgeometry", "1400x900+40+40"],
                parameters=[robot_description],
                output="screen",
            ),
        ]
    )
