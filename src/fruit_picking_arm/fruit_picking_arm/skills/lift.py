#!/usr/bin/env python3
"""Lift Skill — vertical lift after grasping object."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill
from fruit_picking_arm.skills.top_down_pose import top_down_quaternion


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
        lifted_z = cz + lift_height

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position = Point(x=cx, y=cy, z=lifted_z)
        yaw = float(
            self._blackboard.get(
                "top_down_yaw", self._get_param("top_down_yaw", 3.141592654)
            )
        )
        ps.pose.orientation = top_down_quaternion(yaw)

        self._log(f"Lift to z={lifted_z:.3f}")
        return self._planner.plan_pose_target(ps, cartesian=cartesian)
