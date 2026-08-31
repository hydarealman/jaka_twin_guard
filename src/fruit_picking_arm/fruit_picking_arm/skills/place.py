#!/usr/bin/env python3
"""Place Skill — move to bin hover, descend, release, retract."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill
from fruit_picking_arm.skills.top_down_pose import top_down_quaternion
from fruit_picking_arm.scene.bin_geometry import (
    bin_drop_candidates,
    normalized_bin_config,
)


class PlaceSkill(BaseSkill):
    """Plan the three explicit place motions used by the behavior tree.

    Reads an explicitly configured health-specific bin from scene_config.
    Gripper release and trajectory execution remain separate BT stages so the
    real test mode can require human approval for each one.
    """

    def plan(self) -> Optional[JointTrajectory]:
        approach_h = self._get_param("approach_height", 0.12)
        release_clearance = self._get_param("release_clearance", 0.015)
        wall_clearance = self._get_param("drop_wall_clearance", 0.010)
        cartesian = self._get_param("cartesian", False)

        target = self._blackboard.get("target_object")
        health = str(getattr(target, "health", "unknown")).strip().lower()
        fruit_radius = max(0.0, float(getattr(target, "radius", 0.03)))
        if health not in ("healthy", "unhealthy", "good", "bad", "0", "1"):
            self._log("Refusing to place a target with unknown fruit quality")
            return None

        # 根据目标苹果的好坏选择料框（Healthy→healthy 框，其余→unhealthy 框）
        scene = self._blackboard.get("scene_config", {})
        bin_cfg = self._select_bin(scene)
        if bin_cfg is None:
            return None
        try:
            normalized_bin = normalized_bin_config(bin_cfg)
            bc = normalized_bin["center"]
            top_z = float(normalized_bin["top_z"])
            candidates = bin_drop_candidates(
                bin_cfg, fruit_radius, wall_clearance
            )
        except (KeyError, TypeError, ValueError):
            self._log(
                "Selected bin is invalid or the fruit cannot fit inside its opening"
            )
            return None
        if not all(math.isfinite(value) for value in (bc["x"], bc["y"], top_z)):
            self._log("Selected bin contains non-finite coordinates")
            return None

        # Release above the rim instead of forcing the gripper and attached
        # fruit into the bin.  The fruit bottom remains release_clearance above
        # top_z, then gravity performs the final drop after detachment.
        bz = top_z + fruit_radius + release_clearance
        hover_z = top_z + approach_h
        yaw = float(
            self._blackboard.get(
                "top_down_yaw", self._get_param("top_down_yaw", 3.141592654)
            )
        )

        # Crossing the workcell is a global collision-aware move.  Try safe
        # points inside the measured opening; never move or resize the bin to
        # make planning pass.
        for index, (bx, by) in enumerate(candidates, start=1):
            ps_hover = PoseStamped()
            ps_hover.header.frame_id = "world"
            ps_hover.header.stamp = self._node.get_clock().now().to_msg()
            ps_hover.pose.position = Point(x=bx, y=by, z=hover_z)
            ps_hover.pose.orientation = top_down_quaternion(yaw)
            self._log(
                f"Place hover candidate {index}/{len(candidates)} at "
                f"({bx:.3f}, {by:.3f}, {hover_z:.3f})"
            )
            traj = self._planner.plan_pose_target(ps_hover, cartesian=cartesian)
            if traj is None:
                continue
            self._blackboard["place_drop_z"] = bz
            self._blackboard["place_hover_z"] = hover_z
            self._blackboard["place_x"] = bx
            self._blackboard["place_y"] = by
            return traj
        self._log("No collision-free reachable hover point inside the selected bin")
        return None

    def _select_bin(self, scene: dict) -> Optional[dict]:
        """按目标 health 选料框：Healthy→bins.healthy，Unhealthy→bins.unhealthy。

        Missing or malformed configuration is an error; never choose another
        bin or built-in coordinate on behalf of the real robot.
        """
        target = self._blackboard.get("target_object")
        health = getattr(target, "health", "unknown") if target is not None else "unknown"

        normalized = str(health).strip().lower()
        if normalized in ("healthy", "good", "0"):
            kind = "healthy"
        elif normalized in ("unhealthy", "bad", "1"):
            kind = "unhealthy"
        else:
            self._log(f"Cannot select bin for unknown health={health}")
            return None

        bins = scene.get("bins")
        if not isinstance(bins, dict) or not isinstance(bins.get(kind), dict):
            self._log(f"Required '{kind}' bin is not configured")
            return None
        self._log(f"目标 health={health} → 放入 {kind} 料框")
        return bins[kind]

    def _stored_place_pose(self, key: str, description: str) -> Optional[JointTrajectory]:
        try:
            bx = float(self._blackboard["place_x"])
            by = float(self._blackboard["place_y"])
            target_z = float(self._blackboard[key])
        except (KeyError, TypeError, ValueError):
            self._log("Place state is incomplete; refusing default coordinates")
            return None
        yaw = float(
            self._blackboard.get(
                "top_down_yaw", self._get_param("top_down_yaw", 3.141592654)
            )
        )
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position = Point(x=bx, y=by, z=target_z)
        pose.pose.orientation = top_down_quaternion(yaw)
        self._log(f"{description} ({bx:.3f}, {by:.3f}, {target_z:.3f})")
        return self._planner.plan_pose_target(pose, cartesian=True)

    def plan_drop(self) -> Optional[JointTrajectory]:
        return self._stored_place_pose("place_drop_z", "Bin release descent")

    def plan_retract(self) -> Optional[JointTrajectory]:
        return self._stored_place_pose("place_hover_z", "Bin vertical retract")

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
