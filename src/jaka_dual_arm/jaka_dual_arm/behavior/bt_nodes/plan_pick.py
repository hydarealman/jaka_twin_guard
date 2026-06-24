#!/usr/bin/env python3
"""PlanPick — 规划抓取轨迹，写入黑板。"""

from __future__ import annotations

import py_trees

from jaka_dual_arm.skills.grasp import GraspSkill
from jaka_dual_arm.skills.base_skill import SkillResult


class PlanPick(py_trees.behaviour.Behaviour):
    """规划并执行抓取。

    Blackboard 输入:
        target_pose (Pose) — 从 DetectObject 写入
    Blackboard 输出:
        grasp_result (SkillResult) — 抓取结果
    """

    def __init__(self, name: str, grasp_skill: GraspSkill):
        super().__init__(name)
        self._grasp_skill = grasp_skill

    def setup(self, **kwargs):
        self.blackboard.register_key(
            "target_pose", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "grasp_result", access=py_trees.common.Access.WRITE
        )

    def initialise(self):
        pass

    def update(self):
        target_pose = self.blackboard.target_pose
        if target_pose is None:
            return py_trees.common.Status.FAILURE

        self._grasp_skill.configure({"target_pose": target_pose})
        result = self._grasp_skill.run()
        self.blackboard.grasp_result = result
        self.feedback_message = f"Grasp result: {result.name}"

        if result == SkillResult.SUCCESS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
