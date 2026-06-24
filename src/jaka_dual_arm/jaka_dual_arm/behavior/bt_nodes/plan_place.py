#!/usr/bin/env python3
"""PlanPlace — 规划放置轨迹，写入黑板。"""

from __future__ import annotations

import py_trees

from geometry_msgs.msg import Pose
from jaka_dual_arm.skills.place import PlaceSkill
from jaka_dual_arm.skills.base_skill import SkillResult


class PlanPlace(py_trees.behaviour.Behaviour):
    """规划并执行放置。

    Blackboard 输入:
        place_pose (Pose) — 放置目标位姿
    Blackboard 输出:
        place_result (SkillResult)
    """

    def __init__(self, name: str, place_skill: PlaceSkill):
        super().__init__(name)
        self._place_skill = place_skill

    def setup(self, **kwargs):
        self.blackboard.register_key(
            "place_pose", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            "place_result", access=py_trees.common.Access.WRITE
        )

    def update(self):
        place_pose: Pose = self.blackboard.place_pose
        if place_pose is None:
            return py_trees.common.Status.FAILURE

        self._place_skill.configure({"place_pose": place_pose})
        result = self._place_skill.run()
        self.blackboard.place_result = result
        self.feedback_message = f"Place result: {result.name}"

        if result == SkillResult.SUCCESS:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
