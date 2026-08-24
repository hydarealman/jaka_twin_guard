#!/usr/bin/env python3
"""Grasp Skill — descend to grasp pose and close gripper."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Quaternion, Point
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill


class GraspSkill(BaseSkill):
    """Plan descent from hover to object surface, then close gripper.

    Reads target_object from blackboard.
    Uses gripper_controller from blackboard to close gripper after descent.
    """

    def plan(self) -> Optional[JointTrajectory]:
        target = self._blackboard.get("target_object")
        if target is None:
            self._log("No target_object in blackboard")
            return None

        cartesian = self._get_param("cartesian", True)

        # Get target position
        if hasattr(target, "centroid"):
            tx, ty, tz = target.centroid
            radius = getattr(target, "radius", 0.03)
        elif self._blackboard.get("simulation_mode", False) and isinstance(target, dict):
            try:
                position = target["position"]
                tx = float(position["x"])
                ty = float(position["y"])
                radius = float(target["radius"])
                tz = float(
                    self._blackboard["scene_config"]["table"]["top_z"]
                ) + radius
            except (KeyError, TypeError, ValueError):
                self._log("Simulation target geometry is incomplete")
                return None
        else:
            self._log("Real grasp requires a perceived target centroid")
            return None

        # gripper_tcp is the centre between the finger tips.
        grasp_z = tz
        yaw = math.atan2(ty, tx)

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position = Point(x=tx, y=ty, z=grasp_z)
        # The custom gripper fingers extend along tool -Z, so zero roll/pitch
        # is the top-down grasp orientation.
        ps.pose.orientation = self._rpy_to_quat(0.0, 0.0, yaw)

        self._log(f"Grasp at ({tx:.3f}, {ty:.3f}, {grasp_z:.3f})")
        return self._planner.plan_pose_target(ps, cartesian=cartesian)

    def execute(self, trajectory: JointTrajectory) -> bool:
        """Execute descent THEN close gripper."""
        if not super().execute(trajectory):
            return False

        # Close gripper
        gripper = self._blackboard.get("gripper_controller")
        if gripper is not None:
            self._log("Closing gripper...")
            return gripper.close()
        self._log("No gripper controller — refusing to assume grasp success")
        return False

    @staticmethod
    def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        q = Quaternion()
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        q.w = cr * cp * cy + sr * sp * sy
        return q
