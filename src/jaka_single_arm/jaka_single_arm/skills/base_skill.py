#!/usr/bin/env python3
"""Base Skill — abstract interface for all atomic manipulation skills.

Design: plan() → execute() → validate() lifecycle.
Simplified from jaka_dual_arm/skills/base_skill.py for single arm.

Reference:
  - jaka_dual_arm/skills/base_skill.py — dual-arm base skill
  - PickNik MoveIt Studio — plan→execute→validate pattern
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Optional

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class SkillResult(Enum):
    SUCCESS = auto()
    PLAN_FAILED = auto()
    EXECUTION_FAILED = auto()
    GRASP_FAILED = auto()
    DETECT_FAILED = auto()
    CANCELLED = auto()


class BaseSkill(ABC):
    """Abstract base for single-arm manipulation skills.

    Subclasses must implement plan().
    Can override execute() and validate() for custom behavior.

    Usage:
        skill = ApproachSkill()
        skill.configure(node, planner, skill_params)
        result = skill.run()
    """

    def __init__(self):
        self._node: Optional[Node] = None
        self._planner = None  # SingleArmPlannerServer
        self._params: dict = {}
        self._logger = None
        self._blackboard: dict = {}

    def configure(self, node: Node, planner, params: dict,
                  blackboard: dict = None):
        """Initialize skill with required components.

        Args:
            node: ROS2 node for logging and spinning.
            planner: SingleArmPlannerServer instance.
            params: Skill-specific parameters from YAML.
            blackboard: Shared BT blackboard dict.
        """
        self._node = node
        self._planner = planner
        self._params = params
        self._logger = node.get_logger()
        if blackboard is not None:
            self._blackboard = blackboard

    def run(self) -> SkillResult:
        """Execute full skill lifecycle: plan → execute → validate."""
        name = self.__class__.__name__
        self._logger.info(f"Skill [{name}]: starting")

        trajectory = self.plan()
        if trajectory is None:
            self._logger.error(f"Skill [{name}]: plan failed")
            return SkillResult.PLAN_FAILED

        ok = self.execute(trajectory)
        if not ok:
            self._logger.error(f"Skill [{name}]: execution failed")
            return SkillResult.EXECUTION_FAILED

        if not self.validate():
            self._logger.error(f"Skill [{name}]: validation failed")
            return SkillResult.GRASP_FAILED

        self._logger.info(f"Skill [{name}]: SUCCESS")
        return SkillResult.SUCCESS

    @abstractmethod
    def plan(self) -> Optional[JointTrajectory]:
        """Generate motion trajectory. Must be implemented by subclasses."""
        ...

    def execute(self, trajectory: JointTrajectory) -> bool:
        """Execute trajectory via planner."""
        if self._planner is None:
            self._logger.error("No planner configured")
            return False
        return self._planner.execute(trajectory)

    def validate(self) -> bool:
        """Post-condition validation. Override in subclasses."""
        return True

    # ── Helper utilities for subclasses ──────────────────────

    def _get_current_arm(self) -> list[float]:
        return self._planner.get_current_arm_positions()

    def _get_param(self, key: str, default=None):
        return self._params.get(key, default)

    def _log(self, msg: str):
        if self._logger:
            self._logger.info(f"[{self.__class__.__name__}] {msg}")
