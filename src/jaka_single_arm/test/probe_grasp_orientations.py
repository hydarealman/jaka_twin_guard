#!/usr/bin/env python3
"""Find a top-down grasp orientation that respects measured joint limits."""

from __future__ import annotations

import math
import os
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node


JOINTS = [f"joint_{index}" for index in range(1, 7)]
SEED = [0.0, 0.0, 1.0, 0.0, -1.0, 0.0]


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Probe(Node):
    def __init__(self):
        super().__init__("grasp_orientation_probe")
        self.client = self.create_client(GetPositionIK, "/compute_ik")

    def solve(self, x, y, z, roll, pitch, yaw):
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        request.ik_request.ik_link_name = "tool_flange"
        request.ik_request.timeout.sec = 0
        request.ik_request.timeout.nanosec = 120_000_000
        request.ik_request.avoid_collisions = False
        request.ik_request.robot_state.is_diff = True
        request.ik_request.robot_state.joint_state.name = JOINTS
        request.ik_request.robot_state.joint_state.position = SEED
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        request.ik_request.pose_stamped = pose
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        result = future.result()
        if result is None or result.error_code.val != 1:
            return None
        values = dict(zip(
            result.solution.joint_state.name,
            result.solution.joint_state.position,
        ))
        return [values[name] for name in JOINTS]


def main() -> int:
    scene_path = os.path.join(
        get_package_share_directory("jaka_single_arm"),
        "config",
        "scene_params.yaml",
    )
    with open(scene_path, "r", encoding="utf-8") as stream:
        objects = (yaml.safe_load(stream) or {}).get("objects", [])

    rclpy.init()
    node = Probe()
    try:
        if not node.client.wait_for_service(timeout_sec=20.0):
            print("/compute_ik unavailable", file=sys.stderr)
            return 2

        # The CAD chain's nominal tool frame is not aligned with the mechanical
        # wrist reference planes, so search the full orientation space instead
        # of assuming that only a small roll/pitch correction is needed.
        preferred = [
            (math.radians(roll), math.radians(pitch), math.radians(yaw))
            for roll, pitch, yaw in (
                (0, 0, 0),
                (0, 0, 90),
                (0, 0, -90),
                (0, 0, 180),
                (10, 0, 0),
                (-10, 0, 0),
                (0, 10, 0),
                (0, -10, 0),
            )
        ]
        candidate_orientations = preferred
        all_valid = []
        for index, fruit in enumerate(objects, start=1):
            position = fruit["position"]
            radius = float(fruit["radius"])
            fruit_valid = []
            for roll, pitch, yaw_offset in candidate_orientations:
                yaw = math.atan2(position["y"], position["x"]) + yaw_offset
                solution = node.solve(
                    float(position["x"]),
                    float(position["y"]),
                    0.30 + radius + 0.086 + 0.12,
                    roll,
                    pitch,
                    yaw,
                )
                if solution is not None:
                    fruit_valid.append((
                        abs(solution[3]),
                        roll,
                        pitch,
                        yaw_offset,
                        solution,
                    ))
            fruit_valid.sort(key=lambda item: item[0])
            all_valid.append(fruit_valid)
            print(f"FRUIT_{index}_VALID_COUNT={len(fruit_valid)}")
            for max_j4, roll, pitch, yaw_offset, solution in fruit_valid[:8]:
                print(
                    "  VALID "
                    f"rpy_deg=({math.degrees(roll):.0f},"
                    f"{math.degrees(pitch):.0f},yaw+"
                    f"{math.degrees(yaw_offset):.0f}) "
                    f"abs_j4_deg={math.degrees(max_j4):.1f} "
                    "q=[" + ", ".join(
                        f"{value:.3f}" for value in solution
                    ) + "]"
                )
        return 0 if all(all_valid) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
