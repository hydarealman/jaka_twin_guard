#!/usr/bin/env python3
"""Entry point for single-arm pick-and-place task.

Usage:
    ros2 run jaka_single_arm pick_place_runner --ros-args -p scene:=a

Scenes:
    a — default: 4 fruits on table → bin
"""

from __future__ import annotations

import argparse
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_share: str, filename: str) -> dict:
    """Load a YAML config file from the package share/config directory."""
    path = os.path.join(package_share, "config", filename)
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def main() -> int:
    # Parse command-line arguments (ROS2-style via argparse)
    parser = argparse.ArgumentParser(description="JAKA C5 Single-Arm Pick-and-Place Runner")
    parser.add_argument("--scene", default="a", choices=["a"],
                        help="Scene variant (default: a)")
    args, _ = parser.parse_known_args()

    rclpy.init()

    # Load all YAML configurations
    package_share = get_package_share_directory("jaka_single_arm")

    robot_cfg = load_yaml(package_share, "robot_params.yaml")
    scene_cfg = load_yaml(package_share, "scene_params.yaml")
    perception_cfg = load_yaml(package_share, "perception_params.yaml")
    planner_cfg = load_yaml(package_share, "planner_params.yaml")
    skill_cfg = load_yaml(package_share, "skill_params.yaml")
    behavior_cfg = load_yaml(package_share, "behavior_params.yaml")
    safety_cfg = load_yaml(package_share, "safety_params.yaml")
    gripper_cfg = load_yaml(package_share, "gripper_params.yaml")

    # Create and run the pick-place runner
    from jaka_single_arm.behavior.bt_runner import PickPlaceRunner

    node = PickPlaceRunner(
        robot_cfg=robot_cfg,
        scene_cfg=scene_cfg,
        perception_cfg=perception_cfg,
        planner_cfg=planner_cfg,
        skill_cfg=skill_cfg,
        behavior_cfg=behavior_cfg,
        safety_cfg=safety_cfg,
        gripper_cfg=gripper_cfg,
    )

    success = False
    try:
        success = node.run()
        if not success:
            node.get_logger().error("Task completed with errors or no objects placed.")
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    main()
