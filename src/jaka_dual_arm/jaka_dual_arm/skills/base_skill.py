#!/usr/bin/env python3
"""Skill 基类 — 所有操作技能的抽象接口。

设计原则：
- 每个 Skill 是一个独立的、可组合的原子操作
- 通过 PlannerServer 获取轨迹，通过 ActionClient 执行
- 参数从 YAML 配置加载，支持运行时覆盖

参考:
  - PickNik MoveIt Studio — 参数化 Behavior 库、plan→execute→validate 生命周期
  - ARIAC 2024 — Kitting/Assembly 任务的技能分解模式
  - multipanda_ros2 — 1kHz 实时控制 + Controller 热切换（本项目的硬件层参考）
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class SkillResult(Enum):
    SUCCESS = auto()
    PLAN_FAILED = auto()
    EXECUTION_FAILED = auto()
    GRASP_FAILED = auto()
    CANCELLED = auto()


class BaseSkill(ABC):
    """操作技能基类。

    子类需实现:
    - plan()    — 生成轨迹
    - execute() — 发送轨迹到控制器
    - validate() — 后置条件验证（可选）

    用法:
        skill = GraspSkill(node, planner, left_client, right_client)
        skill.configure(params_dict)
        result = skill.run()
    """

    def __init__(
        self,
        node: Node,
        planner,  # DualArmPlannerServer
        left_client: ActionClient,
        right_client: ActionClient,
        left_joints: list[str],
        right_joints: list[str],
    ):
        self._node = node
        self._planner = planner
        self._left_client = left_client
        self._right_client = right_client
        self._left_joints = left_joints
        self._right_joints = right_joints
        self._params: dict = {}

    def configure(self, params: dict) -> None:
        """配置技能参数。"""
        self._params = params

    def run(self) -> SkillResult:
        """执行技能的完整流程：plan → execute → validate。"""
        self._node.get_logger().info(f"Running skill: {self.__class__.__name__}")

        trajectory = self.plan()
        if trajectory is None:
            self._node.get_logger().error(
                f"Planning failed for {self.__class__.__name__}"
            )
            return SkillResult.PLAN_FAILED

        ok = self.execute(trajectory)
        if not ok:
            return SkillResult.EXECUTION_FAILED

        if not self.validate():
            return SkillResult.GRASP_FAILED

        self._node.get_logger().info(
            f"Skill {self.__class__.__name__} completed successfully."
        )
        return SkillResult.SUCCESS

    @abstractmethod
    def plan(self) -> Optional[JointTrajectory]:
        """生成运动轨迹。子类必须实现。"""
        ...

    def execute(self, trajectory: JointTrajectory) -> bool:
        """执行轨迹 — 发送给控制器 Action Server。"""
        left_traj, right_traj = self._split_trajectory(trajectory)
        return self._send_and_wait(left_traj, right_traj)

    def validate(self) -> bool:
        """后置条件验证。子类可以覆盖。"""
        return True

    # ── 内部工具 ────────────────────────────────────────

    def _split_trajectory(self, trajectory: JointTrajectory):
        """将组合轨迹拆分为左右臂独立轨迹。"""
        from builtin_interfaces.msg import Duration

        def _shifted(d: Duration, offset: float) -> Duration:
            s = float(d.sec) + float(d.nanosec) / 1e9 + offset
            whole = int(s)
            return Duration(sec=whole, nanosec=int((s - whole) * 1e9))

        idx = {n: i for i, n in enumerate(trajectory.joint_names)}

        left = JointTrajectory()
        left.joint_names = self._left_joints
        right = JointTrajectory()
        right.joint_names = self._right_joints

        for pt in trajectory.points:
            from trajectory_msgs.msg import JointTrajectoryPoint
            lp = JointTrajectoryPoint()
            lp.positions = [pt.positions[idx[n]] for n in self._left_joints]
            lp.time_from_start = _shifted(pt.time_from_start, 0.5)
            left.points.append(lp)

            rp = JointTrajectoryPoint()
            rp.positions = [pt.positions[idx[n]] for n in self._right_joints]
            rp.time_from_start = _shifted(pt.time_from_start, 0.5)
            right.points.append(rp)

        return left, right

    def _send_and_wait(
        self, left_traj: JointTrajectory, right_traj: JointTrajectory
    ) -> bool:
        """发送双臂轨迹并等待完成。"""
        import threading

        results = {"left": None, "right": None}
        event = threading.Event()

        def _done(future, label):
            result = future.result().result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                results[label] = True
                self._node.get_logger().info(f"{label} trajectory finished.")
            else:
                results[label] = False
                self._node.get_logger().error(
                    f"{label} failed: code={result.error_code}"
                )
            if results["left"] is not None and results["right"] is not None:
                event.set()

        def _send(client, traj, label):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            from builtin_interfaces.msg import Duration
            goal.goal_time_tolerance = Duration(sec=0, nanosec=500_000_000)
            future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self._node, future)
            handle = future.result()
            if not handle.accepted:
                self._node.get_logger().error(f"{label} goal rejected.")
                results[label] = False
                event.set()
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(lambda f: _done(f, label))

        _send(self._left_client, left_traj, "left")
        _send(self._right_client, right_traj, "right")

        # 等待双臂都完成（最多 60s）
        timeout = self._params.get("timeout", 60.0)
        event.wait(timeout=timeout)
        return results["left"] is True and results["right"] is True
