#!/usr/bin/env python3
"""Lift Skill — 垂直抬升货物。

流程：
1. 从当前末端位姿沿 Z 轴抬升指定高度
2. 笛卡尔直线运动
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectory

from jaka_dual_arm.skills.base_skill import BaseSkill


class LiftSkill(BaseSkill):
    """抬升货物 — 沿世界 Z 轴垂直移动。"""

    def plan(self) -> Optional[JointTrajectory]:
        height = self._params.get("lift_height", 0.15)
        # 从当前末端位姿出发，沿 Z 轴抬升
        # 实现需要查询 TF 获取当前末端位姿
        # 然后用笛卡尔路径规划
        self._node.get_logger().info(f"Lift: {height:.2f}m vertically")
        # TODO: 完整实现需要 TF + IK
        return self._planner.plan_pose_target(
            Pose(),  # 占位 — 实际由当前位姿 + height 计算
            group="both_arms",
            is_cartesian=True,
        )

    def validate(self) -> bool:
        return True
