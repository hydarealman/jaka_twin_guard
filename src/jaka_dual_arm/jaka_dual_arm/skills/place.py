#!/usr/bin/env python3
"""Place Skill — 将货物放置到目标位置。

流程：
1. 移动到目标点上方悬停位姿
2. 沿 Z 轴下降至放置高度
3. 打开夹爪释放货物
4. 沿 Z 轴抬升退让
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory

from jaka_dual_arm.skills.base_skill import BaseSkill


class PlaceSkill(BaseSkill):
    """放置货物 — 下降到放置点并释放。"""

    def plan(self) -> Optional[JointTrajectory]:
        target: Pose = self._params.get("place_pose")
        if target is None:
            self._node.get_logger().error("PlaceSkill: no place_pose in params")
            return None

        # 先移动到悬停点，再下降
        approach_height = self._params.get("approach_height", 0.08)
        hover = Pose()
        hover.position.x = target.position.x
        hover.position.y = target.position.y
        hover.position.z = target.position.z + approach_height
        hover.orientation = target.orientation

        return self._planner.plan_pose_target(hover, group="both_arms", is_cartesian=True)

    def validate(self) -> bool:
        # TODO: 验证货物底部已接触桌面
        return True
