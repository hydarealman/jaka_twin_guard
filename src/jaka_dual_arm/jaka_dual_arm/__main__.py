#!/usr/bin/env python3
"""jaka_dual_arm 主入口 — 启动搬运任务运行器。

用法:
    ros2 run jaka_dual_arm carry_task_runner                     # 场景 A (默认)
    ros2 run jaka_dual_arm carry_task_runner --scene a           # 场景 A: 桌到桌
    ros2 run jaka_dual_arm carry_task_runner --scene b           # 场景 B: 料框拣选
    ros2 run jaka_dual_arm carry_task_runner --scene c           # 场景 C: 传送带分拣

    ros2 launch jaka_dual_arm sim_rviz.launch.py scene:=b
"""

import argparse
import sys
import yaml
import os

import rclpy
from ament_index_python.packages import get_package_share_directory

from jaka_dual_arm.behavior.bt_runner import CarryTaskRunner


SCENE_CONFIGS = {
    "a": "scene_a_table_pick.yaml",
    "b": "scene_b_bin_pick.yaml",
    "c": "scene_c_conveyor.yaml",
    "default": "scene_params.yaml",
}

SCENE_LABELS = {
    "a": "Table-to-Table Pick & Place",
    "b": "Bin-to-Table Pick",
    "c": "Conveyor-to-Bin Sortation",
    "default": "Default Table Pick & Place",
}


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="JAKA Dual-Arm Carry Task Runner")
    parser.add_argument("--scene", type=str, default="a",
                        choices=["a", "b", "c", "default"],
                        help="场景选择: a=桌到桌, b=料框拣选, c=传送带分拣, default=默认")
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)

    share_dir = get_package_share_directory("jaka_dual_arm")
    config_dir = os.path.join(share_dir, "config")

    # 加载场景配置
    scene_file = SCENE_CONFIGS.get(args.scene, "scene_params.yaml")
    scene_cfg = load_yaml(os.path.join(config_dir, scene_file))
    label = SCENE_LABELS.get(args.scene, "Default")
    print(f"\n{'='*55}")
    print(f"  Scene: {label}")
    print(f"  Config: {scene_file}")
    print(f"{'='*55}\n")

    # 加载通用配置
    robot_cfg = load_yaml(os.path.join(config_dir, "robot_params.yaml"))
    planner_cfg = load_yaml(os.path.join(config_dir, "planner_params.yaml"))
    skill_cfg = load_yaml(os.path.join(config_dir, "skill_params.yaml"))

    runner = CarryTaskRunner(
        scene_config=scene_cfg,
        robot_config=robot_cfg,
        planner_config=planner_cfg,
        skill_config=skill_cfg,
    )

    runner.run()


if __name__ == "__main__":
    main()
