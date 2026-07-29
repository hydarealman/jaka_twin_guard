#!/usr/bin/env python3
"""Gripper Controller — parallel-jaw gripper position/force control.

Controls gripper open/close via the FollowJointTrajectory action interface.
All parameters from gripper_params.yaml.

Reference:
  - Robotiq 2F-85 ROS2 driver — standard gripper command pattern
  - jaka_dual_arm dual-arm gripper control pattern
"""

from __future__ import annotations

from rclpy.node import Node


class GripperController:
    """Parallel-jaw gripper position controller.

    Usage:
        grip = GripperController(node, planner, gripper_cfg)
        grip.open()
        grip.close()
        grip.move([0.02, -0.02])
    """

    def __init__(self, node: Node, planner, config: dict):
        """
        Args:
            node: ROS2 node.
            planner: SingleArmPlannerServer (for send_gripper_command).
            config: gripper_params.yaml dict.
        """
        self._node = node
        self._planner = planner
        self._logger = node.get_logger()
        self._config = config

        self._joints = config.get("joints", ["left_finger_joint", "right_finger_joint"])
        self._open_pos = config.get("open", [0.04, -0.04])
        self._closed_pos = config.get("closed", [0.005, -0.005])
        self._travel_time = config.get("travel_time", 1.0)
        self._max_velocity = config.get("max_velocity", 0.2)
        self._finger_thickness = config.get("finger_thickness", 0.012)
        self._grasp_clearance = config.get("grasp_clearance", -0.001)

    def open(self) -> bool:
        """Fully open gripper."""
        self._logger.info("Gripper: OPEN")
        return self._planner.send_gripper_command(self._open_pos, self._travel_time)

    def close(self) -> bool:
        """Fully close gripper (with small gap to avoid collision)."""
        self._logger.info("Gripper: CLOSE")
        return self._planner.send_gripper_command(self._closed_pos, self._travel_time)

    def close_for_radius(self, radius: float) -> bool:
        """Close until the inner finger faces reach a spherical fruit."""
        center_offset = (
            max(0.0, float(radius))
            + self._finger_thickness * 0.5
            + self._grasp_clearance
        )
        max_open = abs(float(self._open_pos[0]))
        center_offset = min(center_offset, max_open)
        positions = [center_offset, -center_offset]
        self._logger.info(
            f"Gripper: CONTACT CLOSE radius={radius:.3f} "
            f"positions={positions}"
        )
        return self._planner.send_gripper_command(
            positions, self._travel_time
        )

    def move(self, positions: list[float], duration: float = None) -> bool:
        """Move gripper to custom positions.

        Args:
            positions: [left, right] in meters.
            duration: Movement duration in seconds (default: travel_time).
        """
        if duration is None:
            duration = self._travel_time
        self._logger.info(f"Gripper: move to {positions}")
        return self._planner.send_gripper_command(positions, duration)

    @property
    def open_positions(self) -> list[float]:
        return list(self._open_pos)

    @property
    def closed_positions(self) -> list[float]:
        return list(self._closed_pos)

    @property
    def joints(self) -> list[str]:
        return list(self._joints)
