#!/usr/bin/env python3
"""Lift Skill — vertical lift after grasping object."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory

from jaka_single_arm.skills.base_skill import BaseSkill


class LiftSkill(BaseSkill):
    """Plan vertical Cartesian lift movement.

    Lifts the end-effector straight up by configured height.
    Vertical direction is +Z in world frame (up from table).
    """

    def plan(self) -> Optional[JointTrajectory]:
        lift_height = self._get_param("height", 0.15)
        cartesian = self._get_param("cartesian", True)

        # Simple approach: increase Z by lift_height in a target pose
        # Use IK to compute joint target for the lifted pose
        # First, get current EE position from joint state
        # Since we don't have FK easily available, use a joint-space plan
        # with a Cartesian path via IK

        # Get current position from the blackboard or compute an estimate
        target_obj = self._blackboard.get("target_object")
        if target_obj is None:
            self._log("No target_object in blackboard")
            return None
        if hasattr(target_obj, "centroid"):
            cx, cy, cz = target_obj.centroid
        elif self._blackboard.get("simulation_mode", False) and isinstance(target_obj, dict):
            try:
                position = target_obj["position"]
                cx = float(position["x"])
                cy = float(position["y"])
                cz = float(
                    self._blackboard["scene_config"]["table"]["top_z"]
                ) + float(target_obj["radius"])
            except (KeyError, TypeError, ValueError):
                self._log("Simulation target geometry is incomplete")
                return None
        else:
            self._log("Real lift requires a perceived target centroid")
            return None

        # Compute lifted pose (Z up)
        lifted_z = cz + 0.086 + lift_height  # grip_z + lift

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position = Point(x=cx, y=cy, z=lifted_z)
        ps.pose.orientation = Quaternion(w=1.0)  # vertical

        self._log(f"Lift to z={lifted_z:.3f}")
        return self._planner.plan_pose_target(ps, cartesian=cartesian)
