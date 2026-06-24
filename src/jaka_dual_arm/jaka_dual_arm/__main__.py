#!/usr/bin/env python3
"""jaka_dual_arm 主入口 — 启动搬运任务运行器。

用法:
    ros2 run jaka_dual_arm carry_task_runner
"""

import sys
import yaml
import os

import rclpy
from ament_index_python.packages import get_package_share_directory

from jaka_dual_arm.behavior.bt_runner import CarryTaskRunner


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    rclpy.init()

    share_dir = get_package_share_directory("jaka_dual_arm")
    config_dir = os.path.join(share_dir, "config")

    # 加载所有 YAML 配置
    scene_cfg = load_yaml(os.path.join(config_dir, "scene_params.yaml"))
    robot_cfg = load_yaml(os.path.join(config_dir, "robot_params.yaml"))
    planner_cfg = load_yaml(os.path.join(config_dir, "planner_params.yaml"))
    skill_cfg = load_yaml(os.path.join(config_dir, "skill_params.yaml"))
    # behavior_cfg = load_yaml(os.path.join(config_dir, "behavior_params.yaml"))

    runner = CarryTaskRunner(
        scene_config=scene_cfg,
        robot_config=robot_cfg,
        planner_config=planner_cfg,
        skill_config=skill_cfg,
    )

    if not runner.setup_scene():
        runner.get_logger().error("Scene setup failed.")
        sys.exit(1)

    runner.run()


if __name__ == "__main__":
    main()
