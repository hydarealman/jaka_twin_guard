#!/usr/bin/env python3
"""Lift Skill — vertical lift after grasping object."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
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

        # Get current tool_flange pose from planner's joint state
        # We use IK to get the current end-effector pose, then offset Z
        current_arm = self._planner.get_current_arm_positions()

        # Simple approach: increase Z by lift_height in a target pose
        # Use IK to compute joint target for the lifted pose
        # First, get current EE position from joint state
        # Since we don't have FK easily available, use a joint-space plan
        # with a Cartesian path via IK

        # Get current position from the blackboard or compute an estimate
        target_obj = self._blackboard.get("target_object")
        if target_obj is not None:
            if hasattr(target_obj, "centroid"):
                cx, cy, cz = target_obj.centroid
            else:
                cx = target_obj.get("x", target_obj.get("position", {}).get("x", 0.5))
                cy = target_obj.get("y", target_obj.get("position", {}).get("y", 0.0))
                cz = 0.30 + target_obj.get("radius", 0.03)
        else:
            self._log("No target_object in blackboard, using current pose")
            return self._plan_joint_lift(lift_height)

        # Compute lifted pose (Z up)
        lifted_z = cz + 0.086 + lift_height  # grip_z + lift

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position = Point(x=cx, y=cy, z=lifted_z)
        ps.pose.orientation = Quaternion(w=1.0)  # vertical

        self._log(f"Lift to z={lifted_z:.3f}")
        traj = self._planner.plan_pose_target(ps, cartesian=cartesian)
        if traj is not None:
            return traj

        # Fallback: joint-space lift
        self._log("Cartesian lift failed, trying joint-space")
        return self._plan_joint_lift(lift_height)

    def _plan_joint_lift(self, lift_height: float) -> Optional[JointTrajectory]:
        """Joint-space vertical lift (fallback)."""
        current = self._planner.get_current_arm_positions()
        # Simple heuristic: J2 and J3 affect end-effector height
        # Increase J2 slightly, decrease J3 slightly to lift
        target = list(current)
        if len(target) >= 3:
            target[1] += 0.15  # J2: extend slightly
            target[2] -= 0.10  # J3: retract slightly
        return self._planner.plan_joint_target(target, start=current)
