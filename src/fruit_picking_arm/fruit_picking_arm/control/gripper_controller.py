#!/usr/bin/env python3
"""Gripper Controller — binary real gripper and simulated jaw control.

Real hardware uses the standard GripperCommand action. Simulation keeps the
existing FollowJointTrajectory interface until its controller is migrated.
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

    def __init__(self, node: Node, planner, config: dict, real_mode: bool = False):
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
        self._real_mode = bool(real_mode)

        self._joints = config.get("joints", ["left_finger_joint", "right_finger_joint"])
        self._open_pos = config.get("open", [0.056, -0.056])
        self._closed_pos = config.get("closed", [0.0, 0.0])
        self._travel_time = config.get("travel_time", 1.0)
        self._max_velocity = config.get("max_velocity", 0.2)
        self._max_effort = config.get("max_effort", 50.0)
        self._finger_thickness = config.get("finger_thickness", 0.012)
        self._grasp_clearance = config.get("grasp_clearance", -0.001)

    def open(self) -> bool:
        """Fully open gripper."""
        self._logger.info("Gripper: OPEN")
        if self._real_mode:
            return self._planner.send_binary_gripper_command(
                float(self._open_pos[0]), self._max_effort
            )
        return self._planner.send_gripper_command(self._open_pos, self._travel_time)

    def close(self) -> bool:
        """Fully close the binary gripper."""
        self._logger.info("Gripper: CLOSE")
        if self._real_mode:
            return self._planner.send_binary_gripper_command(
                float(self._closed_pos[0]), self._max_effort
            )
        return self._planner.send_gripper_command(self._closed_pos, self._travel_time)

    def close_for_radius(self, radius: float) -> bool:
        """Close until the inner finger faces reach a spherical fruit."""
        if self._real_mode:
            self._logger.info(
                "Binary real gripper cannot command an intermediate radius; "
                "using the CLOSED endpoint"
            )
            return self.close()
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
        if self._real_mode:
            if len(positions) != 2:
                return False
            endpoint_tolerance = 0.002
            if abs(float(positions[0]) - float(self._open_pos[0])) <= endpoint_tolerance:
                return self.open()
            if abs(float(positions[0]) - float(self._closed_pos[0])) <= endpoint_tolerance:
                return self.close()
            self._logger.error("Binary real gripper rejects intermediate jaw positions")
            return False
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
