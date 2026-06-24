#!/usr/bin/env python3
"""工业级按摩系统入口。

用法:
    ros2 run jaka_dual_arm massage_runner
"""
import argparse
import os
import sys
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory

from jaka_dual_arm.massage.massage_runner import IndustrialMassageRunner


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="Industrial Dual-Arm Massage System")
    parser.add_argument("--body-config", type=str, default="massage_body_params.yaml",
                        help="Body model YAML config")
    parser.add_argument("--stages-config", type=str, default="massage_stages.yaml",
                        help="Stage definitions YAML config")
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)

    share_dir = get_package_share_directory("jaka_dual_arm")
    config_dir = os.path.join(share_dir, "config")

    body_cfg = load_yaml(os.path.join(config_dir, args.body_config))
    stages_cfg = load_yaml(os.path.join(config_dir, args.stages_config))

    # 可选: 加载力控和安全配置
    imp_path = os.path.join(config_dir, "impedance_params.yaml")
    safety_path = os.path.join(config_dir, "safety_params.yaml")
    imp_cfg = load_yaml(imp_path) if os.path.exists(imp_path) else {}
    safety_cfg = load_yaml(safety_path) if os.path.exists(safety_path) else {}

    print(f"\n{'='*60}")
    print(f"  Industrial Massage System — 60 Stages / 13 Techniques")
    print(f"  Body config: {args.body_config}")
    print(f"  Stages config: {args.stages_config}")
    print(f"  Force control: {'YAML loaded' if imp_cfg else 'default'}")
    print(f"  Safety monitor: {'YAML loaded' if safety_cfg else 'default'}")
    print(f"{'='*60}\n")

    runner = IndustrialMassageRunner(
        body_config=body_cfg,
        stages_config=stages_cfg,
        impedance_config=imp_cfg,
        safety_config=safety_cfg,
    )
    runner.run()


if __name__ == "__main__":
    main()
