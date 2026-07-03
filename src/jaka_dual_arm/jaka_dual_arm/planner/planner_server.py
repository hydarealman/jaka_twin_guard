#!/usr/bin/env python3
"""Planner Server — MoveIt2 规划 Action Server 封装。

接收目标位姿 (PoseStamped) 和约束，返回 JointTrajectory。
支持开链规划 (RRTConnect) 和闭链同步插值 (locked_grip)。

参考:
  - MoveIt Task Constructor (PickNik) — 多阶段任务规划、Stage Pipeline 模式
  - MoveIt2 /plan_kinematic_path — 标准规划服务接口
  - ManyMove manymove_planner — ActionServer 封装 MoveIt2 的设计
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion


def _is_arm_link(name: str) -> bool:
    """判断碰撞体是否为机械臂连杆（用于区分臂-臂碰撞 vs 臂-体接触）。"""
    return name.startswith("left_Link_") or name.startswith("right_Link_")


def _is_robot_link_pair(name_a: str, name_b: str) -> bool:
    """True when both contact bodies are manipulator links."""
    return _is_arm_link(name_a) and _is_arm_link(name_b)


from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene

from rclpy.action import ActionClient


# ── Utility 函数 ────────────────────────────────────────────

def _duration_msg(seconds: float) -> Duration:
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


def _duration_seconds(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) / 1e9


def _shifted_duration(d: Duration, offset: float) -> Duration:
    return _duration_msg(_duration_seconds(d) + offset)


def _nearest_angle(target: float, reference: float) -> float:
    """Return target angle wrapped to the nearest equivalent value to reference."""
    return reference + math.atan2(
        math.sin(target - reference),
        math.cos(target - reference),
    )


def _soft_limit_angle(joint_name: str, value: float) -> float:
    """Keep continuous JAKA joints away from +/-2pi soft-limit edges."""
    if not (
        joint_name.endswith("_joint_1")
        or joint_name.endswith("_joint_5")
        or joint_name.endswith("_joint_6")
    ):
        return value
    return math.atan2(math.sin(value), math.cos(value))


# ── Planner Server Action 接口 ──────────────────────────────

class PlanRequest:
    """规划请求 — 从目标位姿到关节轨迹。

    属性:
        target_pose: Pose  —— 末端目标位姿 (世界坐标系)
        group: str          —— 规划组名 (如 "left_arm", "both_arms")
        planner_id: str     —— OMPL 规划器 ID
        is_cartesian: bool  —— 是否使用笛卡尔路径 (笛卡尔路径强制直线)
        max_velocity_scaling: float
        max_acceleration_scaling: float
    """

    def __init__(self):
        self.target_pose: Optional[Pose] = None
        self.group: str = "both_arms"
        self.planner_id: str = "RRTConnectkConfigDefault"
        self.is_cartesian: bool = False
        self.max_velocity_scaling: float = 0.55
        self.max_acceleration_scaling: float = 0.45
        self.start_joint_state: Optional[JointState] = None
        self.num_planning_attempts: int = 3
        self.allowed_planning_time: float = 1.5


# ── Planner Server ──────────────────────────────────────────

class DualArmPlannerServer(Node):
    """双臂运动规划 Action Server。

    封装 MoveIt2 /plan_kinematic_path 服务，并提供高层 API：
    - plan_joint_target()    — 关节空间规划
    - plan_pose_target()     — 笛卡尔 / 位姿目标规划
    - plan_locked_grip()     — 闭链同步插值 (抓取段)
    """

    def __init__(self):
        super().__init__("dual_arm_planner_server")

        # ── MoveIt 服务客户端 ──
        self._motion_plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        # ── IK 服务客户端 (lazy init) ──
        self._ik_client = None

        # ── 从 YAML 加载参数 ──
        self._declare_params()

        # ── 内部状态 ──
        self._shared_executor = None  # set by MassageRunnerNode for proper multi-node spin
        self._joint_state_lock = threading.Lock()
        self._current_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )

    def _declare_params(self):
        """声明所有 ROS2 参数。默认值可通过 launch 参数覆盖 (velocity/acceleration)。"""
        # planner_params
        self.declare_parameter("planning_group", "both_arms")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("num_planning_attempts", 3)
        self.declare_parameter("allowed_planning_time", 1.5)
        self.declare_parameter("max_velocity_scaling", 0.55)    # MoveIt 规划速度缩放
        self.declare_parameter("max_acceleration_scaling", 0.45)
        self.declare_parameter("controller_max_joint_velocity", 0.75)  # rad/s, 安全演示速度
        self.declare_parameter("controller_min_segment_dt", 0.06)      # s, 小动作不过快
        self.declare_parameter("direct_joint_max_vel", 0.65)          # rad/s, 直接插值模式最高速度
        self.declare_parameter("sample_period", 0.1)
        self.declare_parameter("joint_tolerance", 0.003)
        self.declare_parameter("trajectory_start_delay", 0.5)
        self.declare_parameter("locked_grip_speed", 0.18)
        self.declare_parameter("settle_speed", 0.12)
        self.declare_parameter("stage_settle_timeout", 0.15)
        self.declare_parameter("stage_settle_threshold", 0.08)
        self.declare_parameter("transition_pause_scale", 0.00)
        self.declare_parameter("transition_pause_max", 0.05)
        self.declare_parameter("preplan_window_massage_stages", 3)
        self.declare_parameter("use_cartesian_massage_path", False)
        self.declare_parameter("massage_path_lift", 0.08)
        self.declare_parameter("enable_dual_pose_planning", False)
        self.declare_parameter("coordination_sample_period", 0.05)
        self.declare_parameter("coordination_max_start_delay", 0.5)
        self.declare_parameter("coordination_delay_step", 0.25)
        self.declare_parameter("allow_parking_recovery", False)
        self.declare_parameter(
            "left_parking_joints",
            [0.0, 1.00, -1.10, 1.30, 1.57, 1.50],
        )
        self.declare_parameter(
            "right_parking_joints",
            [0.0, 1.00, -1.10, 1.30, 1.57, 1.50],
        )

    def _on_joint_state(self, msg: JointState):
        with self._joint_state_lock:
            self._current_joint_state = msg

    @property
    def current_joint_state(self) -> Optional[JointState]:
        with self._joint_state_lock:
            return self._current_joint_state

    def _spin_future(self, future, timeout_sec: float = 5.0):
        """Spin until future completes, using shared executor if available."""
        if self._shared_executor is not None:
            self._shared_executor.spin_until_future_complete(future, timeout_sec)
        else:
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=timeout_sec
            )

    # ── IK 服务 ────────────────────────────────────────────

    def _ensure_ik_client(self):
        """Lazy-init IK service client."""
        if self._ik_client is not None:
            return True
        from moveit_msgs.srv import GetPositionIK
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik"
        )
        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/compute_ik not available — IK will be unavailable")
            self._ik_client = None
            return False
        return True

    def compute_ik(
        self,
        pose_stamped: "PoseStamped",
        group: str,
        timeout_sec: float = 1.0,
        seed_joint_names: Optional[list[str]] = None,
        seed_positions: Optional[list[float]] = None,
    ) -> Optional[list[float]]:
        """通过 MoveIt /compute_ik 求解单个位姿 IK.

        使用当前关节状态作为种子, 保证收敛到附近构型。
        timeout_sec 内无响应则返回 None。

        Returns:
            [j1..j6] 针对指定规划组, 或 None (IK 失败)
        """
        from moveit_msgs.msg import PositionIKRequest
        from moveit_msgs.srv import GetPositionIK

        if not self._ensure_ik_client():
            return None

        req = GetPositionIK.Request()
        req.ik_request = PositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = int(timeout_sec)
        req.ik_request.timeout.nanosec = int((timeout_sec - int(timeout_sec)) * 1e9)

        if seed_joint_names is not None and seed_positions is not None:
            req.ik_request.robot_state.joint_state = JointState()
            req.ik_request.robot_state.joint_state.name = list(seed_joint_names)
            req.ik_request.robot_state.joint_state.position = list(seed_positions)
            req.ik_request.robot_state.is_diff = True
        else:
            # 用当前关节状态做种子
            js = self.current_joint_state
            if js is not None:
                req.ik_request.robot_state.joint_state = js
                req.ik_request.robot_state.is_diff = True

        future = self._ik_client.call_async(req)
        self._spin_future(future, timeout_sec=timeout_sec + 2.0)
        result = future.result()

        if result is None or result.error_code.val != 1:
            return None

        # 提取规划组对应的关节位置
        pos_dict = dict(zip(
            result.solution.joint_state.name,
            result.solution.joint_state.position,
        ))
        joints = self._get_joint_names_for_group(group)
        return [pos_dict.get(j, 0.0) for j in joints]

    # ── 碰撞检测 ─────────────────────────────────────────

    def check_state_collision_free(
        self,
        joint_positions: list[float],
        joint_names: list[str] | None = None,
        group: str = "both_arms",
    ) -> bool:
        """通过 MoveIt /check_state_validity 检查关节状态是否无碰撞。

        区分臂-臂碰撞（真正危险，返回 False）和臂-体接触（按摩正常，返回 True）。

        Args:
            joint_positions: 12 关节位置 [left_1..6, right_1..6]
            joint_names: 关节名称列表, 默认 ALL_JOINTS
            group: 规划组名, 默认 "both_arms"

        Returns:
            True = 无碰撞 / 仅臂-体接触 / 服务不可用 (安全默认),
            False = 检测到真实臂-臂碰撞
        """
        if joint_names is None:
            joint_names = (
                [f"left_joint_{i}" for i in range(1, 7)]
                + [f"right_joint_{i}" for i in range(1, 7)]
            )

        if not self._validity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "/check_state_validity 不可用, 跳过碰撞检测"
            )
            return True  # 无法检测 → 安全默认

        req = GetStateValidity.Request()
        req.group_name = group
        req.robot_state.is_diff = True
        req.robot_state.joint_state = JointState()
        req.robot_state.joint_state.name = joint_names
        req.robot_state.joint_state.position = list(joint_positions)

        future = self._validity_client.call_async(req)
        self._spin_future(future, timeout_sec=5.0)
        result = future.result()

        if result is None:
            self.get_logger().warn("碰撞检测超时, 跳过")
            return True

        if not result.valid:
            contacts = list(result.contacts)
            # 区分臂-臂碰撞（真正危险）和臂-体接触（按摩正常）
            arm_to_arm = any(
                _is_robot_link_pair(c.contact_body_1, c.contact_body_2)
                for c in contacts
            )
            if arm_to_arm:
                cs = ", ".join(
                    f"{c.contact_body_1}<->{c.contact_body_2}"
                    for c in contacts[:4]
                )
                self.get_logger().error(
                    f"⛔ 臂-臂碰撞检测失败: {cs}"
                )
                return False  # 真正危险，阻止执行
            else:
                # 臂-体接触是按摩的正常现象，允许通过
                self.get_logger().info(
                    f"  臂体接触（按摩正常）通过"
                )
                return True

        return True

    # ── 公共 API ──────────────────────────────────────────

    def plan_joint_target(
        self,
        left_target: Optional[list[float]] = None,
        right_target: Optional[list[float]] = None,
        left_joints: Optional[list[str]] = None,
        right_joints: Optional[list[str]] = None,
    ) -> Optional[JointTrajectory]:
        """关节空间规划：从当前状态到目标关节角。

        任一臂的 target 为 None 时保持该臂当前位置不变。
        """
        left_joints = left_joints or []
        right_joints = right_joints or []
        joint_names = left_joints + right_joints
        start_positions = self._get_start_positions(joint_names)
        if start_positions is None:
            return None

        # Fill None targets with current positions (keep arm in place)
        left_target = list(left_target) if left_target else start_positions[:len(left_joints)]
        right_target = list(right_target) if right_target else start_positions[len(left_joints):]

        return self._plan_joint_segment(
            start_positions,
            left_target + right_target,
            joint_names,
            "joint_target",
        )

    def plan_locked_grip_segment(
        self,
        start_positions: list[float],
        goal_positions: list[float],
        joint_names: list[str],
    ) -> JointTrajectory:
        """闭链同步插值：双臂末端相对位姿保持不变。"""
        return self._make_locked_grip_segment(
            start_positions, goal_positions, joint_names, "locked_grip"
        )

    def plan_pose_target(
        self,
        target_pose: Pose,
        group: str = "both_arms",
        is_cartesian: bool = False,
    ) -> Optional[JointTrajectory]:
        """位姿目标规划（通过 MoveIt 内置 IK + 规划）。"""
        joint_names = self._get_joint_names_for_group(group)
        start_positions = self._get_start_positions(joint_names)
        if start_positions is None:
            return None

        return self._plan_pose_segment(
            start_positions,
            target_pose,
            joint_names,
            group,
            is_cartesian,
        )

    def plan_pose_target_from_start(
        self,
        start_positions: list[float],
        target_pose: Pose,
        group: str = "both_arms",
        is_cartesian: bool = False,
    ) -> Optional[JointTrajectory]:
        """Plan a pose target from a predicted start state.

        Used by full-cycle preplanning so later stages are planned from the
        previous stage's final joint state, not from the robot's live state.
        """
        joint_names = self._get_joint_names_for_group(group)
        if len(start_positions) != len(joint_names):
            self.get_logger().error(
                f"Invalid start length for {group}: "
                f"{len(start_positions)} != {len(joint_names)}"
            )
            return None
        return self._plan_pose_segment(
            list(start_positions),
            target_pose,
            joint_names,
            group,
            is_cartesian,
        )

    def plan_joint_target_direct(
        self,
        left_target: Optional[list[float]] = None,
        right_target: Optional[list[float]] = None,
        left_joints: Optional[list[str]] = None,
        right_joints: Optional[list[str]] = None,
    ) -> Optional[JointTrajectory]:
        """直接关节空间插值 (不经过 OMPL 规划).

        从当前关节状态线性插值到目标, 用中央差分计算速度剖面.
        适用于短距移动 (按摩按压), 比 OMPL RRT 快且可预测.

        任一臂 target 为 None 时保持该臂当前位置不变.
        """
        left_joints = left_joints or []
        right_joints = right_joints or []
        joint_names = left_joints + right_joints
        n_joints = len(joint_names)
        if n_joints == 0:
            return None

        start = self._get_start_positions(joint_names)
        if start is None:
            return None

        # 填充 None target 为当前位置
        left_target = list(left_target) if left_target else start[:len(left_joints)]
        right_target = list(right_target) if right_target else start[len(left_joints):]
        goal = left_target + right_target

        # 直接插值用独立速度参数 (比 OMPL 路径快, 因为路径平滑无抖动)
        max_vel = max(float(self.get_parameter("direct_joint_max_vel").value), 0.05)
        min_dt = max(float(self.get_parameter("controller_min_segment_dt").value), 0.02)

        # ── 先 wrap goal 到 start 的最近等效角 ──
        # IK 解可能完整绕了一圈 (如 5.575 rad vs 0.0 rad),
        # 不 wrap 直接插值会走远路 (~5 rad) 导致轨迹过长+跟踪超差.
        wrapped_goal = [
            _soft_limit_angle(name, _nearest_angle(g, s))
            for name, g, s in zip(joint_names, goal, start)
        ]

        # ── 构建插值节点 ──
        max_delta = max(abs(a - b) for a, b in zip(start, wrapped_goal))
        n_points = max(3, min(20, int(max_delta / 0.03) + 3))

        waypoints: list[list[float]] = [list(start)]
        for i in range(1, n_points):
            ratio = i / (n_points - 1)
            wp = [start[j] + (wrapped_goal[j] - start[j]) * ratio
                  for j in range(n_joints)]
            waypoints.append(wp)

        # ── 段时长与累计时间 ──
        durations: list[float] = []
        for i in range(1, len(waypoints)):
            md = max(abs(waypoints[i][j] - waypoints[i - 1][j]) for j in range(n_joints))
            durations.append(max(min_dt, md / max_vel))

        times = [0.0]
        for d in durations:
            times.append(times[-1] + d)

        # ── 中央差分速度 (起止零速) ──
        n_wp = len(waypoints)
        velocities: list[list[float]] = []
        for i in range(n_wp):
            if i == 0 or i == n_wp - 1:
                v = [0.0] * n_joints
            else:
                dt_span = times[i + 1] - times[i - 1]
                if dt_span > 1e-9:
                    v = [(waypoints[i + 1][j] - waypoints[i - 1][j]) / dt_span
                         for j in range(n_joints)]
                else:
                    v = [0.0] * n_joints
            for j in range(n_joints):
                v[j] = max(-max_vel, min(max_vel, v[j]))
            velocities.append(v)

        # ── 构建轨迹 ──
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        for i in range(n_wp):
            pt = JointTrajectoryPoint()
            pt.positions = list(waypoints[i])
            pt.velocities = list(velocities[i])
            pt.accelerations = []
            pt.effort = []
            pt.time_from_start = _duration_msg(times[i])
            traj.points.append(pt)

        self.get_logger().info(
            f"Direct joint plan: {n_wp - 1} waypoints, "
            f"duration={times[-1]:.1f}s, max_delta={max_delta:.3f}rad"
        )
        return traj

    # ── 内部方法 ──────────────────────────────────────────

    def _get_start_positions(self, joint_names: list[str]) -> Optional[list[float]]:
        js = self.current_joint_state
        if js is None:
            self.get_logger().error("No joint state received yet.")
            return None
        pos_map = dict(zip(js.name, js.position))
        try:
            return [pos_map[name] for name in joint_names]
        except KeyError as e:
            self.get_logger().error(f"Joint {e} not found in current joint state.")
            return None

    def _get_joint_names_for_group(self, group: str) -> list[str]:
        left = [
            "left_joint_1", "left_joint_2", "left_joint_3",
            "left_joint_4", "left_joint_5", "left_joint_6",
        ]
        right = [
            "right_joint_1", "right_joint_2", "right_joint_3",
            "right_joint_4", "right_joint_5", "right_joint_6",
        ]
        if group == "both_arms":
            return left + right
        elif group == "left_arm":
            return left
        elif group == "right_arm":
            return right
        return left + right

    def _plan_joint_segment(
        self,
        start: list[float],
        goal: list[float],
        joint_names: list[str],
        label: str,
    ) -> Optional[JointTrajectory]:
        request = GetMotionPlan.Request()
        mr = request.motion_plan_request
        mr.group_name = self.get_parameter("planning_group").value
        mr.planner_id = self.get_parameter("planner_id").value
        mr.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        mr.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        mr.max_velocity_scaling_factor = self.get_parameter("max_velocity_scaling").value
        mr.max_acceleration_scaling_factor = self.get_parameter("max_acceleration_scaling").value

        mr.start_state.is_diff = True
        mr.start_state.joint_state = JointState()
        mr.start_state.joint_state.name = joint_names
        mr.start_state.joint_state.position = start

        goal_constraints = Constraints()
        tolerance = self.get_parameter("joint_tolerance").value
        for name, pos in zip(joint_names, goal):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
        mr.goal_constraints.append(goal_constraints)

        future = self._motion_plan_client.call_async(request)
        self._spin_future(
            future,
            timeout_sec=max(float(mr.allowed_planning_time) + 2.0, 4.0),
        )
        result = future.result()
        if result is None:
            self.get_logger().error(f"MoveIt did not respond for {label} (timeout).")
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"Joint plan FAILED for {label}: code={resp.error_code.val}, "
                f"attempts={mr.num_planning_attempts}, "
                f"time={mr.allowed_planning_time}s"
            )
            return None

        traj = self._stabilize_trajectory(
            resp.trajectory.joint_trajectory,
            start_positions=start,
        )
        if not traj.points:
            self.get_logger().error(f"Empty trajectory for {label}.")
            return None

        self.get_logger().info(
            f"Planned {label}: {len(traj.points)} pts, {resp.planning_time:.3f}s"
        )
        return traj

    def _plan_pose_segment(
        self,
        start: list[float],
        target: Pose,
        joint_names: list[str],
        group: str,
        cartesian: bool,
    ) -> Optional[JointTrajectory]:
        """Plan to a pose target using MoveIt with position+orientation constraints.

        MoveIt 内部处理 IK — 通过 position_constraints + orientation_constraints
        设置末端目标位姿，规划器自动求解 IK 并规划无碰撞路径。
        """
        self.get_logger().info(
            f"Pose target: [{target.position.x:.3f}, {target.position.y:.3f}, "
            f"{target.position.z:.3f}], group={group}"
        )

        request = GetMotionPlan.Request()
        mr = request.motion_plan_request
        mr.group_name = group
        mr.planner_id = self.get_parameter("planner_id").value
        mr.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        mr.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        mr.max_velocity_scaling_factor = self.get_parameter("max_velocity_scaling").value
        mr.max_acceleration_scaling_factor = self.get_parameter("max_acceleration_scaling").value

        # Start state
        mr.start_state.is_diff = True
        mr.start_state.joint_state = JointState()
        mr.start_state.joint_state.name = joint_names
        mr.start_state.joint_state.position = start

        # Determine end-effector link from group name
        # URDF uses "left_Link_06" / "right_Link_06" (uppercase L, two-digit 06)
        if "left" in group and "right" not in group:
            ee_link = "left_Link_06"
        elif "right" in group and "left" not in group:
            ee_link = "right_Link_06"
        else:
            ee_link = "left_Link_06"  # default for both_arms

        constraints = Constraints()

        # Position constraint: 1cm tolerance sphere around target
        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = ee_link
        pc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])
        pc.constraint_region.primitives.append(sphere)
        sphere_pose = Pose()
        sphere_pose.position = target.position
        sphere_pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(sphere_pose)
        constraints.position_constraints.append(pc)

        # Orientation constraint: ±0.3 rad (~17°) tolerance per axis
        # Relaxed from 0.1 rad to accommodate the unusual massage end-effector orientations
        oc = OrientationConstraint()
        oc.header.frame_id = "world"
        oc.link_name = ee_link
        oc.orientation = target.orientation
        oc.absolute_x_axis_tolerance = 0.3
        oc.absolute_y_axis_tolerance = 0.3
        oc.absolute_z_axis_tolerance = 0.3
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)

        mr.goal_constraints.append(constraints)

        future = self._motion_plan_client.call_async(request)
        self._spin_future(
            future,
            timeout_sec=max(float(mr.allowed_planning_time) + 2.0, 4.0),
        )
        result = future.result()
        if result is None:
            self.get_logger().error("MoveIt did not respond for pose target (timeout).")
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"Pose plan FAILED: code={resp.error_code.val}, "
                f"attempts={mr.num_planning_attempts}, "
                f"time={mr.allowed_planning_time}s"
            )
            return None

        traj = self._stabilize_trajectory(
            resp.trajectory.joint_trajectory,
            start_positions=start,
        )
        if not traj.points:
            self.get_logger().error("Empty trajectory for pose target.")
            return None

        self.get_logger().info(
            f"Pose plan: {len(traj.points)} pts, {resp.planning_time:.3f}s"
        )
        return traj

    def plan_dual_pose_target(
        self,
        left_pose: Pose,
        right_pose: Pose,
        is_cartesian: bool = False,
    ) -> Optional[JointTrajectory]:
        """双臂同时位姿规划 (both_arms 组, 两个 end-effector 各设约束).

        相比依次调用两次 plan_pose_target, 此方法让 MoveIt 同时考虑
        双臂末端位姿, 规划器在扩展 RRT 树时同时检查两个末端约束 +
        双臂自碰撞, 避免 sequential 模式中"左臂绕开 → 右臂无路可走"的问题.

        Args:
            left_pose: 左臂末端目标位姿 (left_Link_06)
            right_pose: 右臂末端目标位姿 (right_Link_06)
            is_cartesian: 是否使用笛卡尔路径

        Returns:
            JointTrajectory (12关节) 或 None
        """
        joint_names = self._get_joint_names_for_group("both_arms")
        start_positions = self._get_start_positions(joint_names)
        if start_positions is None:
            return None

        self.get_logger().info(
            f"Dual pose targets: L[{left_pose.position.x:.3f}, {left_pose.position.y:.3f}, {left_pose.position.z:.3f}] "
            f"R[{right_pose.position.x:.3f}, {right_pose.position.y:.3f}, {right_pose.position.z:.3f}]"
        )

        request = GetMotionPlan.Request()
        mr = request.motion_plan_request
        mr.group_name = "both_arms"
        mr.planner_id = self.get_parameter("planner_id").value
        mr.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        mr.allowed_planning_time = self.get_parameter("allowed_planning_time").value
        mr.max_velocity_scaling_factor = self.get_parameter("max_velocity_scaling").value
        mr.max_acceleration_scaling_factor = self.get_parameter("max_acceleration_scaling").value

        # Start state
        mr.start_state.is_diff = True
        mr.start_state.joint_state = JointState()
        mr.start_state.joint_state.name = joint_names
        mr.start_state.joint_state.position = start_positions

        # ── 左臂约束 (left_Link_06) ──
        # Both end-effectors must satisfy one combined goal constraint.
        # Multiple MotionPlanRequest.goal_constraints entries are alternatives.
        constraints = Constraints()
        # Position constraint
        lpc = PositionConstraint()
        lpc.header.frame_id = "world"
        lpc.link_name = "left_Link_06"
        lpc.weight = 1.0
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])
        lpc.constraint_region.primitives.append(sphere)
        sphere_pose = Pose()
        sphere_pose.position = left_pose.position
        sphere_pose.orientation.w = 1.0
        lpc.constraint_region.primitive_poses.append(sphere_pose)
        constraints.position_constraints.append(lpc)
        # Orientation constraint
        loc = OrientationConstraint()
        loc.header.frame_id = "world"
        loc.link_name = "left_Link_06"
        loc.orientation = left_pose.orientation
        loc.absolute_x_axis_tolerance = 0.3
        loc.absolute_y_axis_tolerance = 0.3
        loc.absolute_z_axis_tolerance = 0.3
        loc.weight = 1.0
        constraints.orientation_constraints.append(loc)

        # ── 右臂约束 (right_Link_06) ──
        rpc = PositionConstraint()
        rpc.header.frame_id = "world"
        rpc.link_name = "right_Link_06"
        rpc.weight = 1.0
        sphere2 = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.03])
        rpc.constraint_region.primitives.append(sphere2)
        sphere_pose2 = Pose()
        sphere_pose2.position = right_pose.position
        sphere_pose2.orientation.w = 1.0
        rpc.constraint_region.primitive_poses.append(sphere_pose2)
        constraints.position_constraints.append(rpc)
        roc = OrientationConstraint()
        roc.header.frame_id = "world"
        roc.link_name = "right_Link_06"
        roc.orientation = right_pose.orientation
        roc.absolute_x_axis_tolerance = 0.3
        roc.absolute_y_axis_tolerance = 0.3
        roc.absolute_z_axis_tolerance = 0.3
        roc.weight = 1.0
        constraints.orientation_constraints.append(roc)

        mr.goal_constraints.append(constraints)

        future = self._motion_plan_client.call_async(request)
        self._spin_future(future, timeout_sec=15.0)
        result = future.result()
        if result is None:
            self.get_logger().error("MoveIt did not respond for dual pose target.")
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"Dual pose plan FAILED: code={resp.error_code.val}"
            )
            return None

        traj = self._stabilize_trajectory(
            resp.trajectory.joint_trajectory,
            start_positions=start_positions,
        )
        if not traj.points:
            self.get_logger().error("Empty trajectory for dual pose target.")
            return None

        self.get_logger().info(
            f"Dual pose plan: {len(traj.points)} pts, {resp.planning_time:.3f}s"
        )
        return traj

    def split_dual_trajectory(
        self,
        traj: JointTrajectory,
    ) -> tuple[Optional[JointTrajectory], Optional[JointTrajectory]]:
        """Split a 12-joint both_arms trajectory into controller trajectories."""
        if traj is None or not traj.points:
            return None, None

        by_name = {name: i for i, name in enumerate(traj.joint_names)}
        left_names = self._get_joint_names_for_group("left_arm")
        right_names = self._get_joint_names_for_group("right_arm")

        def make_part(names: list[str]) -> Optional[JointTrajectory]:
            if not all(name in by_name for name in names):
                return None
            out = JointTrajectory()
            out.header = traj.header
            out.joint_names = list(names)
            for src in traj.points:
                pt = JointTrajectoryPoint()
                pt.positions = [src.positions[by_name[name]] for name in names]
                if src.velocities:
                    pt.velocities = [src.velocities[by_name[name]] for name in names]
                if src.accelerations:
                    pt.accelerations = [src.accelerations[by_name[name]] for name in names]
                pt.effort = []
                pt.time_from_start = src.time_from_start
                out.points.append(pt)
            return out

        return make_part(left_names), make_part(right_names)

    def trajectory_with_start_delay(
        self,
        traj: JointTrajectory,
        delay: float,
        start_positions: Optional[list[float]] = None,
    ) -> JointTrajectory:
        """Return a copy of traj that holds the current joint state before moving."""
        if traj is None or delay <= 1e-6:
            return traj

        out = JointTrajectory()
        out.header = traj.header
        out.joint_names = list(traj.joint_names)

        start = (
            list(start_positions)
            if start_positions is not None
            else self._get_start_positions(out.joint_names)
        )
        if start is not None and len(start) != len(out.joint_names):
            self.get_logger().warn(
                "Ignoring delayed-trajectory start with invalid joint count"
            )
            start = None
        if start is None and traj.points:
            start = list(traj.points[0].positions)
        if start is not None:
            hold = JointTrajectoryPoint()
            hold.positions = list(start)
            hold.velocities = [0.0] * len(start)
            hold.time_from_start = _duration_msg(0.0)
            out.points.append(hold)

        for src in traj.points:
            pt = JointTrajectoryPoint()
            pt.positions = list(src.positions)
            pt.velocities = list(src.velocities)
            pt.accelerations = list(src.accelerations)
            pt.effort = list(src.effort)
            pt.time_from_start = _duration_msg(
                _duration_seconds(src.time_from_start) + delay
            )
            out.points.append(pt)
        return out

    def find_safe_dual_timing(
        self,
        left_traj: JointTrajectory,
        right_traj: JointTrajectory,
        start_positions: Optional[list[float]] = None,
    ) -> Optional[tuple[float, float]]:
        """Search small start-time offsets that keep two arm trajectories collision-free."""
        max_delay = max(
            0.0,
            float(self.get_parameter("coordination_max_start_delay").value),
        )
        delay_step = max(
            0.05,
            float(self.get_parameter("coordination_delay_step").value),
        )

        candidates: list[tuple[float, float]] = [(0.0, 0.0)]
        n_steps = int(max_delay / delay_step)
        for i in range(1, n_steps + 1):
            d = round(i * delay_step, 6)
            candidates.append((0.0, d))
            candidates.append((d, 0.0))

        for left_delay, right_delay in candidates:
            merged = self.merge_dual_trajectories(
                left_traj,
                right_traj,
                left_delay=left_delay,
                right_delay=right_delay,
                start_positions=start_positions,
            )
            if merged is None:
                continue
            if self.validate_trajectory_collision_free(
                merged,
                label=f"dual timing L+{left_delay:.2f}/R+{right_delay:.2f}",
            ):
                if left_delay > 0.0 or right_delay > 0.0:
                    self.get_logger().info(
                        f"Coordinated timing selected: "
                        f"left_delay={left_delay:.2f}s, right_delay={right_delay:.2f}s"
                    )
                return left_delay, right_delay
        return None

    def merge_dual_trajectories(
        self,
        left_traj: Optional[JointTrajectory],
        right_traj: Optional[JointTrajectory],
        left_delay: float = 0.0,
        right_delay: float = 0.0,
        sample_period: Optional[float] = None,
        start_positions: Optional[list[float]] = None,
    ) -> Optional[JointTrajectory]:
        """Sample two controller trajectories into a full both_arms trajectory."""
        joint_names = self._get_joint_names_for_group("both_arms")
        start = (
            list(start_positions)
            if start_positions is not None
            else self._get_start_positions(joint_names)
        )
        if start is None:
            return None
        if len(start) != len(joint_names):
            self.get_logger().error(
                f"Invalid coordinated start length: {len(start)} != {len(joint_names)}"
            )
            return None
        start_map = dict(zip(joint_names, start))
        sample_period = sample_period or max(
            0.02,
            float(self.get_parameter("coordination_sample_period").value),
        )

        def duration(traj: Optional[JointTrajectory]) -> float:
            if traj is None or not traj.points:
                return 0.0
            return _duration_seconds(traj.points[-1].time_from_start)

        total = max(
            left_delay + duration(left_traj),
            right_delay + duration(right_traj),
            sample_period,
        )
        count = max(2, int(math.ceil(total / sample_period)) + 1)

        merged = JointTrajectory()
        merged.joint_names = list(joint_names)
        for i in range(count):
            t = min(total, i * sample_period)
            state = dict(start_map)
            self._sample_into_state(state, left_traj, t - left_delay, start_map)
            self._sample_into_state(state, right_traj, t - right_delay, start_map)
            pt = JointTrajectoryPoint()
            pt.positions = [state[name] for name in joint_names]
            pt.time_from_start = _duration_msg(t)
            merged.points.append(pt)
        return merged

    def validate_trajectory_collision_free(
        self,
        traj: JointTrajectory,
        label: str = "trajectory",
    ) -> bool:
        """Validate sampled states and reject robot-link collisions."""
        if traj is None or not traj.points:
            return True
        if not self._validity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f"/check_state_validity unavailable; cannot validate {label}"
            )
            return False

        sample_period = max(
            0.02,
            float(self.get_parameter("coordination_sample_period").value),
        )
        total = _duration_seconds(traj.points[-1].time_from_start)
        sample_times = {0.0, total}
        for pt in traj.points:
            sample_times.add(_duration_seconds(pt.time_from_start))
        count = max(1, int(math.ceil(total / sample_period)))
        for i in range(count + 1):
            sample_times.add(min(total, i * sample_period))

        for idx, sample_time in enumerate(sorted(sample_times)):
            positions = self._sample_trajectory_positions(traj, sample_time)
            req = GetStateValidity.Request()
            req.group_name = "both_arms"
            req.robot_state.is_diff = True
            req.robot_state.joint_state = JointState()
            req.robot_state.joint_state.name = list(traj.joint_names)
            req.robot_state.joint_state.position = positions
            future = self._validity_client.call_async(req)
            self._spin_future(future, timeout_sec=2.0)
            result = future.result()
            if result is None:
                self.get_logger().warn(
                    f"State validity timeout for {label} sample {idx} "
                    f"t={sample_time:.2f}s"
                )
                return False
            if result.valid:
                continue
            contacts = list(result.contacts)
            if any(
                _is_robot_link_pair(c.contact_body_1, c.contact_body_2)
                for c in contacts
            ):
                cs = ", ".join(
                    f"{c.contact_body_1}<->{c.contact_body_2}"
                    for c in contacts[:4]
                )
                self.get_logger().error(
                    f"Coordinated validation rejected {label} at sample {idx} "
                    f"t={sample_time:.2f}s: "
                    f"robot-link contact {cs}"
                )
                return False
            if not contacts:
                self.get_logger().error(
                    f"Coordinated validation rejected {label} at sample {idx} "
                    f"t={sample_time:.2f}s: "
                    "invalid state with no contact details"
                )
                return False
        return True

    def _sample_trajectory_positions(
        self,
        traj: JointTrajectory,
        sample_time: float,
    ) -> list[float]:
        if not traj.points:
            return []
        if sample_time <= _duration_seconds(traj.points[0].time_from_start):
            return list(traj.points[0].positions)

        prev = traj.points[0]
        for nxt in traj.points[1:]:
            t0 = _duration_seconds(prev.time_from_start)
            t1 = _duration_seconds(nxt.time_from_start)
            if sample_time <= t1:
                ratio = 1.0 if t1 <= t0 else (sample_time - t0) / (t1 - t0)
                ratio = max(0.0, min(1.0, ratio))
                return [
                    prev.positions[j]
                    + (nxt.positions[j] - prev.positions[j]) * ratio
                    for j in range(len(traj.joint_names))
                ]
            prev = nxt
        return list(traj.points[-1].positions)

    def plan_dual_joint_goal_from_trajectories(
        self,
        left_traj: Optional[JointTrajectory],
        right_traj: Optional[JointTrajectory],
    ) -> Optional[JointTrajectory]:
        """Replan in the 12-DOF both_arms space to the final state of two trajectories."""
        left_names = self._get_joint_names_for_group("left_arm")
        right_names = self._get_joint_names_for_group("right_arm")

        def final_positions(
            traj: Optional[JointTrajectory],
            names: list[str],
        ) -> Optional[list[float]]:
            if traj is None or not traj.points:
                return None
            by_name = {name: i for i, name in enumerate(traj.joint_names)}
            if not all(name in by_name for name in names):
                return None
            last = traj.points[-1]
            return [last.positions[by_name[name]] for name in names]

        left_goal = final_positions(left_traj, left_names)
        right_goal = final_positions(right_traj, right_names)
        if left_goal is None and right_goal is None:
            return None

        self.get_logger().warn(
            "Parallel trajectories predicted unsafe; replanning final target in both_arms"
        )
        traj = self.plan_joint_target(
            left_target=left_goal,
            right_target=right_goal,
            left_joints=left_names,
            right_joints=right_names,
        )
        if traj is None:
            return None
        if not self.validate_trajectory_collision_free(traj, label="dual joint replan"):
            return None
        return traj

    def plan_dual_joint_goal_via_parking(
        self,
        left_traj: Optional[JointTrajectory],
        right_traj: Optional[JointTrajectory],
    ) -> Optional[JointTrajectory]:
        """Recover unsafe dual-arm motion with parameterized parking waypoints."""
        joint_names = self._get_joint_names_for_group("both_arms")
        left_names = self._get_joint_names_for_group("left_arm")
        right_names = self._get_joint_names_for_group("right_arm")
        current = self._get_start_positions(joint_names)
        if current is None:
            return None

        current_left = current[: len(left_names)]
        current_right = current[len(left_names):]
        left_goal = self._final_positions_from_trajectory(left_traj, left_names)
        right_goal = self._final_positions_from_trajectory(right_traj, right_names)
        left_goal = left_goal if left_goal is not None else list(current_left)
        right_goal = right_goal if right_goal is not None else list(current_right)
        final_goal = left_goal + right_goal

        left_parking = self._get_parking_joints("left", current_left)
        right_parking = self._get_parking_joints("right", current_right)
        left_moves = not self._same_joint_state(current_left, left_goal)
        right_moves = not self._same_joint_state(current_right, right_goal)

        candidates: list[tuple[str, list[list[float]]]] = []
        if left_moves and right_moves:
            candidates.extend(
                [
                    (
                        "left-then-right",
                        [
                            current_left + right_parking,
                            left_goal + right_parking,
                            left_parking + right_parking,
                            left_parking + right_goal,
                        ],
                    ),
                    (
                        "right-then-left",
                        [
                            left_parking + current_right,
                            left_parking + right_goal,
                            left_parking + right_parking,
                            left_goal + right_parking,
                        ],
                    ),
                ]
            )
        if left_moves:
            candidates.append(
                (
                    "right-yields",
                    [
                        current_left + right_parking,
                        left_goal + right_parking,
                    ],
                )
            )
        if right_moves:
            candidates.append(
                (
                    "left-yields",
                    [
                        left_parking + current_right,
                        left_parking + right_goal,
                    ],
                )
            )
        candidates.append(
            (
                "both-park",
                [
                    left_parking + right_parking,
                ],
            )
        )
        if self._joint_state_collision_free(final_goal, joint_names, "dual final goal"):
            candidates.extend(
                [
                    (
                        "left-first-final",
                        [left_goal + current_right, final_goal],
                    ),
                    (
                        "right-first-final",
                        [current_left + right_goal, final_goal],
                    ),
                    (
                        "both-park-final",
                        [left_parking + right_parking, final_goal],
                    ),
                ]
            )

        self.get_logger().warn(
            "Trying parking-based recovery for unsafe dual-arm trajectory"
        )
        for label, waypoints in candidates:
            result = self._plan_joint_waypoint_sequence(
                current,
                waypoints,
                joint_names,
                label=f"parking recovery {label}",
            )
            if result is not None:
                self.get_logger().info(
                    f"Parking recovery selected: {label}"
                )
                return result
        return None

    def _final_positions_from_trajectory(
        self,
        traj: Optional[JointTrajectory],
        names: list[str],
    ) -> Optional[list[float]]:
        if traj is None or not traj.points:
            return None
        by_name = {name: i for i, name in enumerate(traj.joint_names)}
        if not all(name in by_name for name in names):
            return None
        last = traj.points[-1]
        return [last.positions[by_name[name]] for name in names]

    def _get_parking_joints(
        self,
        side: str,
        fallback: list[float],
    ) -> list[float]:
        param_name = f"{side}_parking_joints"
        value = list(self.get_parameter(param_name).value)
        if len(value) != len(fallback):
            self.get_logger().warn(
                f"{param_name} must contain {len(fallback)} joints; using current pose"
            )
            return list(fallback)
        return [float(v) for v in value]

    def _plan_joint_waypoint_sequence(
        self,
        start: list[float],
        waypoints: list[list[float]],
        joint_names: list[str],
        label: str,
    ) -> Optional[JointTrajectory]:
        segments: list[JointTrajectory] = []
        cur = list(start)
        for idx, waypoint in enumerate(waypoints):
            if self._same_joint_state(cur, waypoint):
                continue
            step_label = f"{label} step {idx + 1}"
            seg = self._make_direct_joint_segment(
                cur,
                waypoint,
                joint_names,
                label=step_label,
            )
            if not self.validate_trajectory_collision_free(seg, label=step_label):
                seg = self._plan_joint_segment(
                    cur,
                    waypoint,
                    joint_names,
                    label=step_label,
                )
                if seg is None:
                    return None
                if not self.validate_trajectory_collision_free(
                    seg,
                    label=step_label,
                ):
                    return None
            segments.append(seg)
            cur = list(waypoint)

        if not segments:
            return None
        combined = self._concat_trajectory_segments(segments, joint_names)
        if not self.validate_trajectory_collision_free(combined, label=label):
            return None
        return combined

    def _make_direct_joint_segment(
        self,
        start: list[float],
        goal: list[float],
        joint_names: list[str],
        label: str,
    ) -> JointTrajectory:
        max_vel = max(float(self.get_parameter("direct_joint_max_vel").value), 0.05)
        min_dt = max(float(self.get_parameter("controller_min_segment_dt").value), 0.02)
        wrapped_goal = [
            _soft_limit_angle(name, _nearest_angle(g, s))
            for name, g, s in zip(joint_names, goal, start)
        ]
        max_delta = max(abs(a - b) for a, b in zip(start, wrapped_goal))
        point_count = max(3, min(40, int(max_delta / 0.03) + 3))
        duration = max(min_dt * (point_count - 1), max_delta / max_vel)

        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        for i in range(point_count):
            ratio = i / (point_count - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [
                start[j] + (wrapped_goal[j] - start[j]) * ratio
                for j in range(len(start))
            ]
            pt.velocities = [0.0] * len(start)
            pt.effort = []
            pt.time_from_start = _duration_msg(duration * ratio)
            traj.points.append(pt)
        self.get_logger().info(
            f"Direct recovery segment {label}: {point_count} pts, "
            f"duration={duration:.1f}s"
        )
        return traj

    def _joint_state_collision_free(
        self,
        positions: list[float],
        joint_names: list[str],
        label: str,
    ) -> bool:
        traj = JointTrajectory()
        traj.joint_names = list(joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = _duration_msg(0.0)
        traj.points.append(pt)
        return self.validate_trajectory_collision_free(traj, label=label)

    def _concat_trajectory_segments(
        self,
        segments: list[JointTrajectory],
        joint_names: list[str],
    ) -> JointTrajectory:
        out = JointTrajectory()
        out.joint_names = list(joint_names)
        offset = 0.0
        for seg_idx, seg in enumerate(segments):
            by_name = {name: i for i, name in enumerate(seg.joint_names)}
            for pt in seg.points:
                local_time = _duration_seconds(pt.time_from_start)
                if seg_idx > 0 and local_time <= 1e-6:
                    continue
                new_pt = JointTrajectoryPoint()
                new_pt.positions = [
                    pt.positions[by_name[name]]
                    for name in joint_names
                ]
                if pt.velocities:
                    new_pt.velocities = [
                        pt.velocities[by_name[name]]
                        for name in joint_names
                    ]
                else:
                    new_pt.velocities = [0.0] * len(joint_names)
                if pt.accelerations:
                    new_pt.accelerations = [
                        pt.accelerations[by_name[name]]
                        for name in joint_names
                    ]
                new_pt.effort = []
                new_pt.time_from_start = _duration_msg(offset + local_time)
                out.points.append(new_pt)
            offset += _duration_seconds(seg.points[-1].time_from_start)
        return out

    def _same_joint_state(
        self,
        a: list[float],
        b: list[float],
        tol: float = 0.005,
    ) -> bool:
        if len(a) != len(b):
            return False
        if not a:
            return True
        return max(abs(x - y) for x, y in zip(a, b)) <= tol

    def _sample_into_state(
        self,
        state: dict[str, float],
        traj: Optional[JointTrajectory],
        local_time: float,
        fallback: dict[str, float],
    ) -> None:
        if traj is None or not traj.points:
            return
        if local_time <= 0.0:
            point = traj.points[0]
            for name, pos in zip(traj.joint_names, point.positions):
                state[name] = fallback.get(name, pos)
            return

        prev = traj.points[0]
        for nxt in traj.points[1:]:
            t0 = _duration_seconds(prev.time_from_start)
            t1 = _duration_seconds(nxt.time_from_start)
            if local_time <= t1:
                ratio = 1.0 if t1 <= t0 else (local_time - t0) / (t1 - t0)
                ratio = max(0.0, min(1.0, ratio))
                for j, name in enumerate(traj.joint_names):
                    state[name] = (
                        prev.positions[j]
                        + (nxt.positions[j] - prev.positions[j]) * ratio
                    )
                return
            prev = nxt

        last = traj.points[-1]
        for name, pos in zip(traj.joint_names, last.positions):
            state[name] = pos

    def _make_locked_grip_segment(
        self,
        start: list[float],
        goal: list[float],
        joint_names: list[str],
        label: str,
    ) -> JointTrajectory:
        """线性插值生成闭链同步轨迹。"""
        max_delta = max(abs(a - b) for a, b in zip(start, goal))
        speed = self.get_parameter("locked_grip_speed").value
        local_duration = max(1.0, max_delta / speed)
        point_count = max(2, int(local_duration / 0.08) + 1)

        traj = JointTrajectory()
        traj.joint_names = joint_names
        for i in range(point_count):
            ratio = i / (point_count - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [
                start[j] + (goal[j] - start[j]) * ratio
                for j in range(len(start))
            ]
            pt.time_from_start = _duration_msg(local_duration * ratio)
            traj.points.append(pt)

        self.get_logger().info(
            f"Locked-grip interpolation for {label}: {point_count} pts"
        )
        return traj

    def _stabilize_trajectory(
        self,
        traj: JointTrajectory,
        start_positions: Optional[list[float]] = None,
    ) -> JointTrajectory:
        """Robust trajectory retiming: decimate → wrap → central-difference velocities.

        Industrial-grade approach (TOTG-style):
          Instead of retaining MoveIt's velocity data (which may have non-zero end
          velocity, exceed our max_vel limits, or be inconsistent with our retiming),
          we DISCARD MoveIt's velocities and compute our own using central differences
          from the position deltas.

        Pipeline:
          1. DECIMATE — remove redundant waypoints (delta < 0.008 rad = skip)
          2. WRAP     — unfold ALL joints to nearest angle
          3. CENTRAL DIFFERENCE — compute smooth velocity profile with guaranteed
             zero start/end velocity.  v[i] = (p[i+1] - p[i-1]) / (t[i+1] - t[i-1])
          4. BUILD    — construct JointTrajectory with positions, velocities, times

        This approach is strictly more robust than retaining MoveIt velocities:
          - Zero start AND end velocity guaranteed by formula, patched at the end
          - Velocities clamped to max_vel (MoveIt may exceed it)
          - Consistency: velocity = position_delta / time_delta, matching controller
          - No dependency on MoveIt's TOTP velocity quality
        """
        if traj is None or not traj.points:
            return traj

        max_vel = max(float(self.get_parameter("controller_max_joint_velocity").value), 0.05)
        min_dt = max(float(self.get_parameter("controller_min_segment_dt").value), 0.02)

        stabilized = JointTrajectory()
        stabilized.header = traj.header
        stabilized.joint_names = list(traj.joint_names)

        n_joints = len(traj.joint_names)
        if n_joints == 0:
            return traj

        # ── Step 0: Read actual current positions ──
        current_positions = list(start_positions) if start_positions is not None else None
        js = self.current_joint_state
        if current_positions is None and js is not None:
            pos_map = dict(zip(js.name, js.position))
            current_positions = [pos_map.get(name, 0.0) for name in stabilized.joint_names]

        if current_positions is None:
            current_positions = list(traj.points[0].positions) if traj.points else []

        # ── Step 1: Decimate waypoints ──
        # Two-pass strategy for OMPL RRT paths:
        #   Pass A: remove trivial points (all joints < 0.01 rad delta)
        #   Pass B: if still > MAX_WAYPOINTS, subsample evenly
        #
        # Why threshold-only doesn't work: OMPL RRT paths have uniform joint-space
        # spacing (~0.003-0.01 rad/segment). A threshold like 0.025 barely filters
        # because at least one of 6 joints always moves > 0.025 rad per segment.
        # A threshold like 0.08 would reduce density but risks losing path fidelity.
        #
        # Solution: Keep it simple. Remove truly redundant points (<0.01 rad, which
        # means ~0.05° joint angle = <0.1mm end-effector), then force even subsampling
        # to a guaranteed-safe count. JTC spline interpolation handles the rest.
        MAX_WAYPOINTS = 20
        decimated: list[JointTrajectoryPoint] = []
        prev = None
        for pt in traj.points:
            if prev is not None and pt.positions:
                max_delta = max(abs(a - b) for a, b in zip(prev.positions, pt.positions))
                if max_delta < 0.01:  # rad — trivial redundancy removal only
                    continue
            decimated.append(pt)
            prev = pt

        if not decimated:
            # All points were identical — shouldn't happen, keep midpoint
            mid = len(traj.points) // 2
            decimated = [traj.points[mid]]

        if len(decimated) > MAX_WAYPOINTS:
            # Subsample evenly: keep MAX_WAYPOINTS uniformly spaced waypoints
            n = len(decimated)
            step = n / MAX_WAYPOINTS
            sampled = [decimated[0]]  # always keep first
            for i in range(1, MAX_WAYPOINTS - 1):
                sampled.append(decimated[int(round(i * step))])
            sampled.append(decimated[-1])  # always keep last
            decimated = sampled

        # ── Step 2: Build waypoints list with wrapping ──
        # waypoints[0] = current_positions (t=0, v=0)
        # waypoints[1..N] = decimated waypoint positions, each wrapped to prev
        waypoints: list[list[float]] = [list(current_positions)]
        for src_pt in decimated:
            wrapped = list(src_pt.positions) if src_pt.positions else list(current_positions)
            prev_wp = waypoints[-1]
            # Wrap ALL joints to nearest angle (prevent long-way-around)
            for i in range(n_joints):
                if i < len(wrapped) and i < len(prev_wp):
                    wrapped[i] = _nearest_angle(wrapped[i], prev_wp[i])
                    wrapped[i] = _soft_limit_angle(
                        stabilized.joint_names[i],
                        wrapped[i],
                    )
            waypoints.append(wrapped)

        # ── Step 3: Central-difference velocity computation ──
        # This is the key innovation: discard MoveIt velocities, compute our own
        # from position deltas.  Guarantees zero start/end velocity and max_vel clamp.
        n_wp = len(waypoints)  # = (1 origin + N decimated)

        # 3a: Segment durations from joint-space deltas
        durations: list[float] = []
        for i in range(1, n_wp):
            max_delta = max(
                abs(waypoints[i][j] - waypoints[i - 1][j])
                for j in range(n_joints)
            )
            durations.append(max(min_dt, max_delta / max_vel))

        # 3b: Cumulative times (n_wp points → n_wp-1 segments)
        times: list[float] = [0.0]
        for d in durations:
            times.append(times[-1] + d)

        # 3c: Central-difference velocities
        #   v[0] = 0           (start)
        #   v[n-1] = 0         (end — REQUIRED by allow_nonzero_velocity...=false)
        #   v[i] = (p[i+1] - p[i-1]) / (t[i+1] - t[i-1])   (2nd-order accurate)
        velocities: list[list[float]] = []
        for i in range(n_wp):
            if i == 0 or i == n_wp - 1:
                v = [0.0] * n_joints
            else:
                dt_span = times[i + 1] - times[i - 1]
                if dt_span > 1e-9:
                    v = [
                        (waypoints[i + 1][j] - waypoints[i - 1][j]) / dt_span
                        for j in range(n_joints)
                    ]
                else:
                    v = [0.0] * n_joints
            # Clamp to max_vel for safety
            for j in range(n_joints):
                v[j] = max(-max_vel, min(max_vel, v[j]))
            velocities.append(v)

        # ── Step 4: Build stabilized trajectory ──
        elapsed = times[-1]
        for i in range(n_wp):
            pt = JointTrajectoryPoint()
            pt.positions = list(waypoints[i])
            pt.velocities = list(velocities[i])
            pt.accelerations = []
            pt.effort = []
            pt.time_from_start = _duration_msg(times[i])
            stabilized.points.append(pt)

        self.get_logger().info(
            f"Traj stabilize: {len(traj.points)}→{n_wp - 1} pts, "
            f"duration={elapsed:.1f}s, max_vel={max_vel:.2f}"
        )
        return stabilized
