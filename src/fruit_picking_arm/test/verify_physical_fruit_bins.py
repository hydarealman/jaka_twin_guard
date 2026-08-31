#!/usr/bin/env python3
"""Verify that released Gazebo fruit physically settled in the correct bins."""

from __future__ import annotations

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node


class FruitBinVerifier(Node):
    def __init__(self) -> None:
        super().__init__("physical_fruit_bin_verifier")
        self.client = self.create_client(GetEntityState, "/get_entity_state")

    def get_position(self, model_name: str):
        request = GetEntityState.Request()
        request.name = model_name
        request.reference_frame = "world"
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        result = future.result()
        if result is None or not result.success:
            return None
        return result.state.pose.position


def main() -> int:
    config_path = os.path.join(
        get_package_share_directory("fruit_picking_arm"),
        "config",
        "scene_params_sim.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as stream:
        scene = yaml.safe_load(stream) or {}

    rclpy.init()
    node = FruitBinVerifier()
    try:
        if not node.client.wait_for_service(timeout_sec=10.0):
            print("PHYSICAL_BIN_VERIFY_FAIL: /get_entity_state unavailable")
            return 2

        # Let the final released fruit fall and settle on the bin bottom.
        time.sleep(2.0)
        failures = []
        for fruit in scene.get("objects", []):
            model_name = str(fruit.get("id", ""))
            health = str(fruit.get("health", "")).lower()
            kind = "unhealthy" if health.startswith("un") else "healthy"
            bin_cfg = scene.get("bins", {}).get(kind, {})
            center = bin_cfg.get("center", {})
            position = node.get_position(model_name)
            if position is None:
                failures.append(f"{model_name}: state unavailable")
                continue
            dx = abs(position.x - float(center.get("x", 0.55)))
            dy = abs(position.y - float(center.get("y", 0.0)))
            # Walls are 0.20m square.  A small tolerance accepts contact with
            # a wall but rejects the original table position or wrong bin.
            in_xy = dx <= 0.11 and dy <= 0.11
            in_z = 0.14 <= position.z <= 0.50
            if not (in_xy and in_z):
                failures.append(
                    f"{model_name}: expected {kind} bin, got "
                    f"({position.x:.3f}, {position.y:.3f}, {position.z:.3f})"
                )

        if failures:
            print("PHYSICAL_BIN_VERIFY_FAIL: " + "; ".join(failures))
            return 1
        print("PHYSICAL_BIN_VERIFY_PASS: all fruit settled in their physical bins")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
