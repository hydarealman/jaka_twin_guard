#!/usr/bin/env python3
"""Place Skill — move to bin hover, descend, release, retract."""

from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill


class PlaceSkill(BaseSkill):
    """Plan and execute place sequence: hover above bin, descend, release, retract.

    Reads an explicitly configured health-specific bin from scene_config.
    After descent, opens gripper and retracts.
    """

    def plan(self) -> Optional[JointTrajectory]:
        approach_h = self._get_param("approach_height", 0.12)
        release_clearance = self._get_param("release_clearance", 0.015)
        cartesian = self._get_param("cartesian", True)

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
            bc = bin_cfg["center"]
            bx, by = float(bc["x"]), float(bc["y"])
            top_z = float(bin_cfg["top_z"])
        except (KeyError, TypeError, ValueError):
            self._log("Selected bin is missing valid center.x/center.y/top_z")
            return None
        if not all(math.isfinite(value) for value in (bx, by, top_z)):
            self._log("Selected bin contains non-finite coordinates")
            return None

        # Release above the rim instead of forcing the gripper and attached
        # fruit into the bin.  The fruit bottom remains release_clearance above
        # top_z, then gravity performs the final drop after detachment.
        bz = top_z + fruit_radius + release_clearance
        hover_z = top_z + approach_h
        yaw = math.atan2(by, bx)

        # Step 1: Plan to bin hover
        ps_hover = PoseStamped()
        ps_hover.header.frame_id = "world"
        ps_hover.header.stamp = self._node.get_clock().now().to_msg()
        ps_hover.pose.position = Point(x=bx, y=by, z=hover_z)
        ps_hover.pose.orientation = self._rpy_to_quat(0.0, 0.0, yaw)

        self._log(f"Place hover at ({bx:.3f}, {by:.3f}, {hover_z:.3f})")
        traj = self._planner.plan_pose_target(ps_hover, cartesian=cartesian)
        if traj is not None:
            self._blackboard["place_drop_z"] = bz
            self._blackboard["place_hover_z"] = hover_z
            self._blackboard["place_x"] = bx
            self._blackboard["place_y"] = by
            self._blackboard["place_phase"] = "hover"
        return traj

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

    def execute(self, trajectory: JointTrajectory) -> bool:
        """Execute multi-stage place: hover → descend → release → retract."""
        # Execute hover
        if not super().execute(trajectory):
            return False

        phase = self._blackboard.get("place_phase")
        if phase == "hover":
            # Plan and execute descent into bin
            try:
                bx = float(self._blackboard["place_x"])
                by = float(self._blackboard["place_y"])
                bz = float(self._blackboard["place_drop_z"])
                hover_z = float(self._blackboard["place_hover_z"])
            except (KeyError, TypeError, ValueError):
                self._log("Place state is incomplete; refusing default coordinates")
                return False
            yaw = math.atan2(by, bx)

            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = self._node.get_clock().now().to_msg()
            ps.pose.position = Point(x=bx, y=by, z=bz)
            ps.pose.orientation = self._rpy_to_quat(0.0, 0.0, yaw)

            self._log(f"Descending to bin z={bz:.3f}")
            drop_traj = self._planner.plan_pose_target(ps, cartesian=True)
            if drop_traj is None or not self._planner.execute(drop_traj):
                self._log("Bin descent failed")
                return False

            # Release gripper
            gripper = self._blackboard.get("gripper_controller")
            if gripper is None:
                self._log("No gripper controller; refusing to report place success")
                return False
            self._log("Opening gripper...")
            if not gripper.open():
                return False

            # In simulation the fruit is held by a Gazebo fixed constraint
            # created after finger contact.  Release that constraint only
            # after the fingers have opened, then let gravity drop the fruit
            # into the physical bin.  No teleport/set-state shortcut is used.
            if (
                self._blackboard.get("simulation_mode", False)
                and self._blackboard.get("simulation_grasp_attached", False)
            ):
                scene_mgr = self._blackboard.get("scene_manager")
                if scene_mgr is None or not scene_mgr.set_simulated_grasp(False):
                    self._log("Failed to release physical Gazebo grasp")
                    return False
                self._blackboard["simulation_grasp_attached"] = False

            # Retract
            ps_retract = PoseStamped()
            ps_retract.header.frame_id = "world"
            ps_retract.header.stamp = self._node.get_clock().now().to_msg()
            ps_retract.pose.position = Point(x=bx, y=by, z=hover_z)
            ps_retract.pose.orientation = self._rpy_to_quat(0.0, 0.0, yaw)

            self._log(f"Retracting to z={hover_z:.3f}")
            retract_traj = self._planner.plan_pose_target(ps_retract, cartesian=True)
            if retract_traj is None:
                self._log("Bin retract planning failed")
                return False
            if not self._planner.execute(retract_traj):
                self._log("Bin retract execution failed")
                return False
            return True

        self._log(f"Invalid place phase: {phase!r}")
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
