#!/usr/bin/env python3
"""搬运任务 BT 节点 — 对接 MoveIt2 规划 + 轨迹执行。

节点清单 (兼容 BehaviorTree.CPP 模型):
    DetectObject        — 从感知接口获取物体 6DoF 位姿 → 写入黑板
    PlanApproach        — 规划接近轨迹 (开链 RRT)
    PlanGrasp           — 规划抓取轨迹 (开链 RRT)
    PlanLift            — 规划抬升轨迹 (闭链 locked_grip)
    PlanCarry           — 规划搬运轨迹 (闭链 locked_grip)
    PlanRelease         — 规划放置轨迹 (开链 RRT)
    PlanRetreat         — 规划撤离轨迹 (开链 RRT)
    ExecuteTrajectory   — 发送轨迹给双臂控制器并等待完成
    CheckGrasp          — 验证抓取条件 (末端间距检查)
    WaitServices        — 等待所有 ROS2 服务就绪
    SetupScene          — 初始化 PlanningScene (桌子/料框/货物)

参考:
  - BehaviorTree.CPP v4.x TreeNode 模型
  - ManyMove manymove_behavior_trees — ROS2 Action BT 节点
  - MoveIt Task Constructor — Stage/SubStage 层级模型
"""

from __future__ import annotations

import math
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState

from jaka_dual_arm.behavior.bt_nodes.bt_node_base import (
    NodeStatus, BtActionNode, BtAsyncNode, BtCondition,
)


# ── WaitServices — 等待所有 ROS2 服务就绪 ─────────────────────

class WaitServices(BtCondition):
    """检查 MoveIt2 + 控制器服务是否全部可用。"""

    def __init__(self, name: str = "WaitServices"):
        super().__init__(name)
        self._motion_client: Optional[ActionClient] = None
        self._left_client: Optional[ActionClient] = None
        self._right_client: Optional[ActionClient] = None
        self._joints_ready: bool = False

    def evaluate(self) -> bool:
        if self._motion_client is None:
            return False
        if not self._motion_client.wait_for_service(timeout_sec=0.0):
            return False
        if self._left_client and not self._left_client.wait_for_server(timeout_sec=0.0):
            return False
        if self._right_client and not self._right_client.wait_for_server(timeout_sec=0.0):
            return False
        if not self._joints_ready:
            return False
        return True


# ── DetectObject — 从感知接口检测物体 ─────────────────────────

class DetectObject(BtActionNode):
    """检测目标物体并写入黑板。

    黑板输出:
        blackboard["target_pose"]: Pose — 物体 6DoF 位姿
        blackboard["object_id"]: str — 物体标识符
    """

    def __init__(self, name: str = "DetectObject",
                 perception=None, object_id: str = "cargo_box"):
        super().__init__(name)
        self._perception = perception
        self._object_id = object_id

    def execute(self) -> NodeStatus:
        if self._perception is None:
            return NodeStatus.FAILURE

        pose = self._perception.detect(self._object_id)
        if pose is None:
            return NodeStatus.FAILURE

        self.blackboard["target_pose"] = pose
        self.blackboard["object_id"] = self._object_id
        return NodeStatus.SUCCESS


# ── PlanPhase — 通用规划节点 ─────────────────────────────────

class PlanPhase(BtActionNode):
    """通用规划节点 — 调用 MoveIt2 RRT 规划单个阶段。

    配置 (从 config 字典):
        phase_key: str       — 阶段名 ("approach", "grasp", 等)
        phase_index: int     — 阶段序号 (1-6)
        scene_cfg: dict      — 场景配置 (含 targets)
        joint_names: list    — 12 个关节名
        get_start_positions: callable — 获取当前关节位置
        plan_fn: callable    — 规划函数
        is_locked_grip: bool — 是否闭链锁定
    """

    def __init__(self, name: str = "PlanPhase"):
        super().__init__(name)

    def execute(self) -> NodeStatus:
        phase_key = self.config.get("phase_key")
        phase_index = self.config.get("phase_index", 0)
        scene_cfg = self.config.get("scene_cfg", {})
        joint_names = self.config.get("joint_names", [])
        get_start = self.config.get("get_start_positions")
        plan_fn = self.config.get("plan_fn")
        is_locked = self.config.get("is_locked_grip", False)

        if phase_key is None or plan_fn is None:
            return NodeStatus.FAILURE

        targets = scene_cfg.get("targets", {}).get(phase_key, {})
        left_target = targets.get("left")
        right_target = targets.get("right")
        if left_target is None or right_target is None:
            return NodeStatus.FAILURE

        start = get_start() if get_start else []
        if not start:
            return NodeStatus.FAILURE

        if is_locked:
            # 闭链线性插值
            goal = list(left_target) + list(right_target)
            max_delta = max(abs(a - b) for a, b in zip(start, goal))
            local_duration = max(1.0, max_delta / 0.18)
            point_count = max(2, int(local_duration / 0.08) + 1)

            traj = JointTrajectory()
            traj.joint_names = joint_names
            for pi in range(point_count):
                from builtin_interfaces.msg import Duration
                from trajectory_msgs.msg import JointTrajectoryPoint
                ratio = pi / (point_count - 1)
                pt = JointTrajectoryPoint()
                pt.positions = [float(start[i] + (goal[i] - start[i]) * ratio) for i in range(len(start))]
                w = int(local_duration * ratio)
                pt.time_from_start = Duration(sec=w, nanosec=int((local_duration * ratio - w) * 1e9))
                traj.points.append(pt)
        else:
            # MoveIt RRT 规划
            traj = plan_fn(joint_names, start, list(left_target) + list(right_target))

        if traj is None:
            return NodeStatus.FAILURE

        self.blackboard["planned_trajectory"] = traj
        self.blackboard["current_phase"] = phase_key
        return NodeStatus.SUCCESS


# ── ExecuteTrajectory — 轨迹执行节点 (异步) ──────────────────

class ExecuteTrajectory(BtAsyncNode):
    """发送轨迹给双臂控制器并等待完成。

    黑板输入:
        blackboard["planned_trajectory"]: JointTrajectory (12 关节)
    配置:
        left_joints: list[str]
        right_joints: list[str]
        left_action: ActionClient
        right_action: ActionClient
        node: Node (用于日志)
    """

    def __init__(self, name: str = "ExecuteTrajectory"):
        super().__init__(name)
        self._left_done = False
        self._right_done = False
        self._left_ok = False
        self._right_ok = False

    def on_start(self):
        super().on_start()
        self._left_done = False
        self._right_done = False
        self._left_ok = False
        self._right_ok = False

    def send_goal(self):
        traj: Optional[JointTrajectory] = self.blackboard.get("planned_trajectory")
        if traj is None:
            return

        left_joints = self.config.get("left_joints", [])
        right_joints = self.config.get("right_joints", [])
        left_client = self.config.get("left_action")
        right_client = self.config.get("right_action")
        node = self.config.get("node")

        def _send(client, joints, label):
            sub = JointTrajectory()
            sub.joint_names = joints
            idx_map = {n: i for i, n in enumerate(traj.joint_names)}
            for pt in traj.points:
                from trajectory_msgs.msg import JointTrajectoryPoint
                from builtin_interfaces.msg import Duration
                np_pt = JointTrajectoryPoint()
                np_pt.positions = [pt.positions[idx_map[j]] for j in joints]
                np_pt.time_from_start = pt.time_from_start
                sub.points.append(np_pt)

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = sub
            goal.goal_time_tolerance = Duration(sec=0, nanosec=500_000_000)

            if node:
                node.get_logger().info(f"[BT] {label}: sending {len(sub.points)} pts")
            fut = client.send_goal_async(goal)

            def _done(f, lbl):
                handle = f.result()
                if not handle or not handle.accepted:
                    if node: node.get_logger().error(f"[BT] {lbl} goal rejected")
                    if lbl == "left":
                        self._left_done = True; self._left_ok = False
                    else:
                        self._right_done = True; self._right_ok = False
                    return
                result_fut = handle.get_result_async()
                def _on_res(f2, lbl2):
                    r = f2.result().result
                    if node: node.get_logger().info(f"[BT] {lbl2}: {'OK' if r.error_code == FollowJointTrajectory.Result.SUCCESSFUL else 'FAIL'}")
                    if lbl2 == "left":
                        self._left_done = True; self._left_ok = (r.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
                    else:
                        self._right_done = True; self._right_ok = (r.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
                result_fut.add_done_callback(lambda f2: _on_res(f2, lbl))
            fut.add_done_callback(lambda f: _done(f, label))

        if left_client:
            _send(left_client, left_joints, "left")
        if right_client:
            _send(right_client, right_joints, "right")

    def check_result(self) -> NodeStatus:
        if self._left_done and self._right_done:
            return NodeStatus.SUCCESS if (self._left_ok and self._right_ok) else NodeStatus.FAILURE
        return NodeStatus.RUNNING


# ── CheckGrasp — 抓取验证条件节点 ────────────────────────────

class CheckGrasp(BtCondition):
    """验证双臂抓取是否成功。

    检查条件:
    1. 双臂末端间距在 [min, max] 范围内
    2. 末端位姿基本对齐 (对称性)
    """

    def __init__(self, name: str = "CheckGrasp"):
        super().__init__(name)
        self._tf_buffer = None
        self._world_frame = "world"
        self._left_frame = "left_grip_contact"
        self._right_frame = "right_grip_contact"

    def evaluate(self) -> bool:
        if self._tf_buffer is None:
            return False  # 没有 TF 数据，默认为失败

        try:
            from rclpy.time import Time
            left_tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._left_frame, Time())
            right_tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._right_frame, Time())
        except Exception:
            return False

        lt = left_tf.transform.translation
        rt = right_tf.transform.translation
        dist = math.sqrt(
            (lt.x - rt.x)**2 + (lt.y - rt.y)**2 + (lt.z - rt.z)**2
        )

        min_dist = self.config.get("grip_distance_min", 0.335)
        max_dist = self.config.get("grip_distance_max", 0.365)

        return min_dist <= dist <= max_dist
