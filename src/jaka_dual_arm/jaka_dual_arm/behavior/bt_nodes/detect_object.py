#!/usr/bin/env python3
"""DetectObject — 从感知接口获取物体 6DoF 位姿，写入黑板。"""

from __future__ import annotations

import py_trees


class DetectObject(py_trees.behaviour.Behaviour):
    """检测物体并写入黑板。

    Blackboard 输入:  (无 — 直接调用感知接口)
    Blackboard 输出:
        target_pose  (Pose) — 物体 6DoF 位姿
        object_found (bool) — 是否检测到物体

    参数:
        object_id (str): 物体标识符
        perception (PerceptionInterface): 感知接口实例
    """

    def __init__(self, name: str, object_id: str, perception):
        super().__init__(name)
        self.object_id = object_id
        self.perception = perception

    def setup(self, **kwargs):
        self.blackboard.register_key(
            "target_pose", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            "object_found", access=py_trees.common.Access.WRITE
        )

    def update(self):
        pose = self.perception.detect(self.object_id)
        if pose is None:
            self.blackboard.object_found = False
            self.logger.debug(f"Object '{self.object_id}' not detected.")
            return py_trees.common.Status.FAILURE

        self.blackboard.target_pose = pose
        self.blackboard.object_found = True
        self.feedback_message = (
            f"Detected {self.object_id} at "
            f"({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        return py_trees.common.Status.SUCCESS
