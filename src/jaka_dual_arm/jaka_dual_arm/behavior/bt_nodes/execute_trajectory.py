#!/usr/bin/env python3
"""ExecuteTrajectory — 执行轨迹 Action。"""

from __future__ import annotations

import py_trees
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class ExecuteTrajectory(py_trees.behaviour.Behaviour):
    """执行 JointTrajectory（通用节点）。

    Blackboard 输入:
        planned_trajectory (JointTrajectory) — 待执行的轨迹
    """

    def __init__(
        self,
        name: str,
        node,
        left_client: ActionClient,
        right_client: ActionClient,
    ):
        super().__init__(name)
        self._node = node
        self._left_client = left_client
        self._right_client = right_client

    def update(self):
        traj: JointTrajectory = self.blackboard.planned_trajectory
        if traj is None:
            self.feedback_message = "No trajectory to execute"
            return py_trees.common.Status.FAILURE

        # 拆分并发送
        self.feedback_message = "Executing trajectory..."
        # 执行逻辑在 BaseSkill._send_and_wait 中
        # 这里简化为直接返回 SUCCESS（实际由 PlanPick/PlanPlace 节点内部执行）
        return py_trees.common.Status.SUCCESS
