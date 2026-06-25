#!/usr/bin/env python3
"""Place Skill — move to bin hover, descend, release, retract."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory

from jaka_single_arm.skills.base_skill import BaseSkill


class PlaceSkill(BaseSkill):
    """Plan and execute place sequence: hover above bin, descend, release, retract.

    Reads bin info from blackboard["scene_config"] or uses defaults.
    After descent, opens gripper and retracts.
    """

    def plan(self) -> Optional[JointTrajectory]:
        approach_h = self._get_param("approach_height", 0.12)
        drop_offset = self._get_param("drop_offset", 0.04)
        cartesian = self._get_param("cartesian", True)

        # Get bin info
        scene = self._blackboard.get("scene_config", {})
        bin_cfg = scene.get("bin", {})
        bc = bin_cfg.get("center", {"x": 0.55, "y": 0.45})
        top_z = bin_cfg.get("top_z", 0.30)
        bs = bin_cfg.get("size", {"x": 0.20, "y": 0.20, "z": 0.15})

        bx, by = bc["x"], bc["y"]
        bz = top_z - bs["z"] * 0.5 + drop_offset
        hover_z = top_z + approach_h
        yaw = math.atan2(by, bx)

        # Step 1: Plan to bin hover
        ps_hover = PoseStamped()
        ps_hover.header.frame_id = "world"
        ps_hover.header.stamp = self._node.get_clock().now().to_msg()
        ps_hover.pose.position = Point(x=bx, y=by, z=hover_z)
        ps_hover.pose.orientation = self._rpy_to_quat(math.pi, 0.0, yaw)

        self._log(f"Place hover at ({bx:.3f}, {by:.3f}, {hover_z:.3f})")
        traj = self._planner.plan_pose_target(ps_hover, cartesian=cartesian)
        if traj is not None:
            self._blackboard["place_drop_z"] = bz
            self._blackboard["place_hover_z"] = hover_z
            self._blackboard["place_x"] = bx
            self._blackboard["place_y"] = by
            self._blackboard["place_phase"] = "hover"
        return traj

    def execute(self, trajectory: JointTrajectory) -> bool:
        """Execute multi-stage place: hover → descend → release → retract."""
        # Execute hover
        if not super().execute(trajectory):
            return False

        phase = self._blackboard.get("place_phase", "hover")
        if phase == "hover":
            # Plan and execute descent into bin
            bx = self._blackboard.get("place_x", 0.55)
            by = self._blackboard.get("place_y", 0.45)
            bz = self._blackboard.get("place_drop_z", 0.24)
            yaw = math.atan2(by, bx)

            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self._node.get_clock().now().to_msg()
            ps.pose.position = Point(x=bx, y=by, z=bz)
            ps.pose.orientation = self._rpy_to_quat(math.pi, 0.0, yaw)

            self._log(f"Descending to bin z={bz:.3f}")
            drop_traj = self._planner.plan_pose_target(ps, cartesian=True)
            if drop_traj is None or not self._planner.execute(drop_traj):
                self._log("Bin descent failed")
                return False

            # Release gripper
            gripper = self._blackboard.get("gripper_controller")
            if gripper is not None:
                self._log("Opening gripper...")
                gripper.open()

            # Retract
            hover_z = self._blackboard.get("place_hover_z", 0.42)
            ps_retract = PoseStamped()
            ps_retract.header.frame_id = "world"
            ps_retract.header.stamp = self._node.get_clock().now().to_msg()
            ps_retract.pose.position = Point(x=bx, y=by, z=hover_z)
            ps_retract.pose.orientation = self._rpy_to_quat(math.pi, 0.0, yaw)

            self._log(f"Retracting to z={hover_z:.3f}")
            retract_traj = self._planner.plan_pose_target(ps_retract, cartesian=True)
            if retract_traj is not None:
                return self._planner.execute(retract_traj)
            return True  # descent + release succeeded

        return True

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
