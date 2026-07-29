#!/usr/bin/env python3
"""Approach Skill — move arm to hover position above target object."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from trajectory_msgs.msg import JointTrajectory

from jaka_single_arm.skills.base_skill import BaseSkill


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

        standoff = self._get_param("standoff_distance", 0.12)
        cartesian = self._get_param("cartesian", True)

        # Get target position from DetectedObject or dict
        if hasattr(target, "centroid"):
            tx, ty, tz = target.centroid
        else:
            tx = target.get("x", target.get("position", {}).get("x", 0.5))
            ty = target.get("y", target.get("position", {}).get("y", 0.0))
            # If from YAML, tz = table_top_z + radius
            tz = 0.30  # fallback
            if "radius" in target:
                tz += target["radius"]

        # The custom parallel-jaw abstraction extends 86 mm along tool -Z.
        # Position the finger tips, rather than the flange, at the requested
        # standoff above the fruit.
        tool_offset = 0.086
        hover_z = tz + tool_offset + standoff
        yaw = math.atan2(ty, tx)

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position = Point(x=tx, y=ty, z=hover_z)
        # tool -Z points down when roll/pitch are zero.
        ps.pose.orientation = self._rpy_to_quat(0.0, 0.0, yaw)

        self._log(f"Approach to ({tx:.3f}, {ty:.3f}, {hover_z:.3f})")
        return self._planner.plan_pose_target(ps, cartesian=cartesian)

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
