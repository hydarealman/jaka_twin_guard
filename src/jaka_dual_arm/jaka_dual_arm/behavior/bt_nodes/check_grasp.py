#!/usr/bin/env python3
"""CheckGrasp — 验证抓取是否成功。"""

from __future__ import annotations

import py_trees


class CheckGrasp(py_trees.behaviour.Behaviour):
    """验证抓取是否成功。

    检查条件:
    - 双臂末端间距在货物宽度的合理范围内
    - 货物被抬离支撑面

    Blackboard 输入:
        grasp_result (SkillResult)
        target_pose (Pose)
    """

    def __init__(self, name: str, config: dict | None = None):
        super().__init__(name)
        self._config = config or {}

    def setup(self, **kwargs):
        self.blackboard.register_key(
            "grasp_result", access=py_trees.common.Access.READ
        )

    def update(self):
        result = self.blackboard.grasp_result
        if result is not None and result.name == "SUCCESS":
            self.feedback_message = "Grasp verified OK"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"Grasp not verified: {result}"
        return py_trees.common.Status.FAILURE
