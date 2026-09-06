#!/usr/bin/env python3
"""Grasp Skill — descend to grasp pose and close gripper."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill
from fruit_picking_arm.skills.top_down_pose import (
    symmetric_yaw_candidates,
    top_down_quaternion,
)


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
        yaw = float(
            self._blackboard.get(
                "top_down_yaw", self._get_param("top_down_yaw", 3.141592654)
            )
        )
        # Mirror the field-validated RViz bridge: prefer the pregrasp yaw, but
        # try the same eight wrist orientations at the lower grasp pose.
        for index, candidate_yaw in enumerate(
            symmetric_yaw_candidates(yaw), start=1
        ):
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self._node.get_clock().now().to_msg()
            ps.pose.position = Point(x=tx, y=ty, z=grasp_z)
            # The gripper grows from its mount along local +Z. For a top-down
            # sleeve grasp that axis must point toward world -Z.
            ps.pose.orientation = top_down_quaternion(candidate_yaw)
            self._log(
                f"Grasp yaw candidate {index}/8 at "
                f"({tx:.3f}, {ty:.3f}, {grasp_z:.3f})"
            )
            trajectory = self._planner.plan_pose_target(
                ps, cartesian=cartesian
            )
            if trajectory is not None:
                self._blackboard["top_down_yaw"] = float(candidate_yaw)
                return trajectory

        self._log("No collision-free IK solution for any grasp yaw candidate")
        return None

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
