#!/usr/bin/env python3
"""Approach Skill — 笛卡尔直线接近物体。

流程：
1. 计算目标点上方/前方悬停位姿
2. 调用 Planner 生成笛卡尔路径
3. 执行轨迹
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectory

from jaka_dual_arm.skills.base_skill import BaseSkill


class ApproachSkill(BaseSkill):
    """接近物体 — 从当前位置直线移动到目标上方的悬停点。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def plan(self) -> Optional[JointTrajectory]:
        """生成接近轨迹。"""
        standoff = self._params.get("standoff_distance", 0.10)
        target: Pose = self._params.get("target_pose")
        if target is None:
            self._node.get_logger().error("ApproachSkill: no target_pose in params")
            return None

        # 计算悬停位姿：目标上方 standoff 距离
        hover_pose = Pose()
        hover_pose.position.x = target.position.x
        hover_pose.position.y = target.position.y
        hover_pose.position.z = target.position.z + standoff
        hover_pose.orientation = target.orientation

        return self._planner.plan_pose_target(
            hover_pose,
            group="both_arms",
            is_cartesian=True,
        )

    def validate(self) -> bool:
        """验证是否到达悬停点。"""
        # TODO: 用 TF 验证末端位姿与悬停点的距离
        return True
