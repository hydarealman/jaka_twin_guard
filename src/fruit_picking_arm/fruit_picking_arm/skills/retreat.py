#!/usr/bin/env python3
"""Retreat Skill — return arm to HOME or safe standby pose."""

from __future__ import annotations

import math
from typing import Optional

from trajectory_msgs.msg import JointTrajectory

from fruit_picking_arm.skills.base_skill import BaseSkill


class RetreatSkill(BaseSkill):
    """Plan and execute retreat to HOME joint configuration.

    HOME pose is read from blackboard["home_pose"] or scene_config.
    """

    def plan(self) -> Optional[JointTrajectory]:
        # Get HOME pose from blackboard or config
        home = self._blackboard.get("home_pose")
        if home is None:
            self._log("No explicitly configured HOME pose")
            return None
        if len(home) != 6 or not all(math.isfinite(float(v)) for v in home):
            self._log("HOME pose must contain six finite joint angles")
            return None

        current = self._planner.get_current_arm_positions()
        if len(current) != 6:
            self._log("Cannot retreat without a complete current joint state")
            return None

        self._log(f"Retreat to HOME: {[f'{v:.2f}' for v in home]}")
        return self._planner.plan_joint_target(home, start=current)
