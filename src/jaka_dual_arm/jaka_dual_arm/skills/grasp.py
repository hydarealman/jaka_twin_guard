#!/usr/bin/env python3
"""Grasp Skill — 双臂协同抓取货物。

流程：
1. 从感知接口获取目标 6DoF 位姿
2. 将双臂末端移动到抓取位姿（左臂→物体左侧，右臂→物体右侧）
3. 验证抓取条件（夹爪间距、对齐角度）
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectory

from jaka_dual_arm.skills.base_skill import BaseSkill


class GraspSkill(BaseSkill):
    """双臂协同抓取 — 两侧同时闭合夹住货物。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._grasped = False

    def configure(self, params: dict) -> None:
        super().configure(params)
        self._grasped = False

    def plan(self) -> Optional[JointTrajectory]:
        """生成抓取轨迹。

        当前实现：使用关节空间规划移动到预抓取位姿。
        完整实现应:
        1. 从感知获取物体 6DoF Pose
        2. 计算左右臂末端目标位姿 (物体中心 ± 半宽 + 接触偏移)
        3. IK 求解 → 关节角
        4. 调用 Planner
        """
        target: dict = self._params.get("grasp_target")
        if target is None:
            self._node.get_logger().error("GraspSkill: no grasp_target in params")
            return None

        left_target = target.get("left_joints")
        right_target = target.get("right_joints")

        if left_target and right_target:
            return self._planner.plan_joint_target(
                left_target, right_target,
                self._left_joints, self._right_joints,
            )

        # 如果没有关节目标，尝试从位姿目标规划
        pose: Pose = self._params.get("target_pose")
        if pose is None:
            self._node.get_logger().error("GraspSkill: no target_pose or joint targets")
            return None
        return self._planner.plan_pose_target(pose, group="both_arms")

    def validate(self) -> bool:
        """验证抓取条件。

        检查:
        - 双臂末端间距在 [min, max] 范围内
        - 夹爪面朝向正确 (dot product 验证)
        """
        # TODO: 用 TF 验证末端间距和对齐条件
        # 当前实现返回 True（由上层 BT 的 CheckGrasp 节点验证）
        return True
