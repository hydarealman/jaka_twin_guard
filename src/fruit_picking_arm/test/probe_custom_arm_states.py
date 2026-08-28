#!/usr/bin/env python3
"""Probe collision-free ready states for the CAD-derived fruit arm.

Run while ``architecture_a_sim.launch.py run_task:=false`` is active.  The
script asks MoveIt itself (rather than relying on visual inspection) to check a
small grid around useful shoulder/elbow poses and prints valid tool poses.
"""

from __future__ import annotations

import itertools
import sys

import rclpy
from geometry_msgs.msg import Point
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = [f"joint_{index}" for index in range(1, 7)]


class StateProbe(Node):
    def __init__(self) -> None:
        super().__init__("custom_arm_state_probe")
        self.validity = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.fk = self.create_client(GetPositionFK, "/compute_fk")

    def call(self, client, request, timeout: float = 4.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def check(self, positions: list[float]):
        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.is_diff = True
        request.robot_state.joint_state = JointState(
            name=ARM_JOINTS, position=positions
        )
        return self.call(self.validity, request)

    def tool_position(self, positions: list[float]) -> Point | None:
        request = GetPositionFK.Request()
        request.header.frame_id = "world"
        request.fk_link_names = ["gripper_tcp"]
        request.robot_state.is_diff = True
        request.robot_state.joint_state = JointState(
            name=ARM_JOINTS, position=positions
        )
        result = self.call(self.fk, request)
        if result is None or result.error_code.val != 1 or not result.pose_stamped:
            return None
        return result.pose_stamped[0].pose.position


def main() -> int:
    rclpy.init()
    node = StateProbe()
    try:
        if not node.validity.wait_for_service(timeout_sec=20.0):
            print("/check_state_validity is not available", file=sys.stderr)
            return 2
        if not node.fk.wait_for_service(timeout_sec=10.0):
            print("/compute_fk is not available", file=sys.stderr)
            return 2

        values = {
            # Sample the useful interior of the J2 0..145 degree range.
            "joint_2": (
                0.147197551,
                0.447197551,
                0.747197551,
                1.047197551,
                1.347197551,
                1.647197551,
                1.947197551,
                2.247197551,
            ),
            "joint_3": (-3.0, -2.5, -2.0, -1.5, -1.0, -0.5, -0.1),
            "joint_4": (0.0,),
            "joint_5": (-1.4, -1.0, 0.0, 1.0, 1.4),
        }
        valid = []
        invalid_pairs: dict[str, int] = {}
        for joint_2, joint_3, joint_4, joint_5 in itertools.product(
            values["joint_2"],
            values["joint_3"],
            values["joint_4"],
            values["joint_5"],
        ):
            positions = [0.0, joint_2, joint_3, joint_4, joint_5, 0.0]
            result = node.check(positions)
            if result is None:
                continue
            if not result.valid:
                for contact in result.contacts:
                    pair = " / ".join(sorted(
                        (contact.contact_body_1, contact.contact_body_2)
                    ))
                    invalid_pairs[pair] = invalid_pairs.get(pair, 0) + 1
                continue
            point = node.tool_position(positions)
            if point is not None:
                valid.append((point.z, point.x, positions, point))

        valid.sort(reverse=True, key=lambda item: (item[0], item[1]))
        print(f"VALID_STATE_COUNT={len(valid)}")
        for _, _, positions, point in valid[:20]:
            joints = ", ".join(f"{value:.3f}" for value in positions)
            print(
                f"VALID q=[{joints}] "
                f"tool=({point.x:.3f}, {point.y:.3f}, {point.z:.3f})"
            )
        if invalid_pairs:
            print("MOST_COMMON_COLLISIONS:")
            for pair, count in sorted(
                invalid_pairs.items(), key=lambda item: item[1], reverse=True
            )[:10]:
                print(f"  {count:3d}  {pair}")
        return 0 if valid else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
