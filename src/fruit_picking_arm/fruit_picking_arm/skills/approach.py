#!/usr/bin/env python3
"""Approach Skill — move arm to hover position above target object."""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import PoseStamped, Point
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill
from fruit_picking_arm.skills.top_down_pose import (
    pregrasp_tcp_z,
    symmetric_yaw_candidates,
    top_down_quaternion,
)


class ApproachSkill(BaseSkill):
    """Plan and execute approach to hover above target object.

    Target is read from blackboard["target_object"] (DetectedObject or dict).
    Computes a pose at standoff_distance above the object centroid.
    """

    def plan(self) -> Optional[JointTrajectory]:
        target = self._blackboard.get("target_object")
        if target is None:
            self._log("No target_object in blackboard")
            return None

        tip_overhang = self._get_param("finger_tip_beyond_tcp", 0.037)
        clearance = self._get_param("clearance_above_fruit", 0.050)
        yaw = self._get_param("top_down_yaw", 3.141592654)
        cartesian = self._get_param("cartesian", True)

        # Get target position from DetectedObject or dict
        if hasattr(target, "centroid"):
            tx, ty, tz = target.centroid
            radius = float(getattr(target, "radius", 0.03))
        elif self._blackboard.get("simulation_mode", False) and isinstance(target, dict):
            try:
                position = target["position"]
                tx = float(position["x"])
                ty = float(position["y"])
                table_z = float(
                    self._blackboard["scene_config"]["table"]["top_z"]
                )
                tz = table_z + float(target["radius"])
                radius = float(target["radius"])
            except (KeyError, TypeError, ValueError):
                self._log("Simulation target geometry is incomplete")
                return None
        else:
            self._log("Real approach requires a perceived target centroid")
            return None

        # Cartesian targets are expressed at gripper_tcp (finger-tip centre).
        hover_z = pregrasp_tcp_z(
            tz, radius, tip_overhang, clearance
        )

        candidates = symmetric_yaw_candidates(yaw)
        for index, candidate_yaw in enumerate(candidates, start=1):
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self._node.get_clock().now().to_msg()
            ps.pose.position = Point(x=tx, y=ty, z=hover_z)
            ps.pose.orientation = top_down_quaternion(candidate_yaw)

            self._log(
                f"Approach yaw candidate {index}/{len(candidates)} "
                f"to ({tx:.3f}, {ty:.3f}, {hover_z:.3f})"
            )
            trajectory = self._planner.plan_pose_target(
                ps, cartesian=cartesian
            )
            if trajectory is not None:
                # The following stages start with the same yaw selected by
                # the field-validated RViz workflow.
                self._blackboard["top_down_yaw"] = float(candidate_yaw)
                return trajectory

        self._log("No collision-free IK solution for any of 8 top-down yaw candidates")
        return None
