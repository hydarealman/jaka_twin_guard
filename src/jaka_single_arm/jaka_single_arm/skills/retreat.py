#!/usr/bin/env python3
"""Retreat Skill — return arm to HOME or safe standby pose."""

from __future__ import annotations

from typing import Optional

from trajectory_msgs.msg import JointTrajectory

from jaka_single_arm.skills.base_skill import BaseSkill


class RetreatSkill(BaseSkill):
    """Plan and execute retreat to HOME joint configuration.

    HOME pose is read from blackboard["home_pose"] or scene_config.
    """

    def plan(self) -> Optional[JointTrajectory]:
        # Get HOME pose from blackboard or config
        home = self._blackboard.get("home_pose")
        if home is None:
            scene = self._blackboard.get("scene_config", {})
            home = scene.get(
                "home_pose", [0.0, 0.0, 1.0, 0.0, -1.0, 0.0]
            )

        current = self._planner.get_current_arm_positions()

        self._log(f"Retreat to HOME: {[f'{v:.2f}' for v in home]}")
        return self._planner.plan_joint_target(home, start=current)
