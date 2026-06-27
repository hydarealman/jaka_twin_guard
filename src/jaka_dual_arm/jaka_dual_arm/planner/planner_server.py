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
        self.max_velocity_scaling: float = 0.15
        self.max_acceleration_scaling: float = 0.15
        self.start_joint_state: Optional[JointState] = None
        self.num_planning_attempts: int = 10
        self.allowed_planning_time: float = 8.0


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
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time", 8.0)    # 8s — sufficient; press targets use approach-from-above (~0.1s)
        self.declare_parameter("max_velocity_scaling", 0.25)    # MoveIt 规划速度缩放
        self.declare_parameter("max_acceleration_scaling", 0.20)
        self.declare_parameter("controller_max_joint_velocity", 0.15)  # rad/s, 按摩需慢速(0.30→0.15)进一步缓解Gazebo过冲振荡
        self.declare_parameter("controller_min_segment_dt", 0.50)      # s, 段间隔 (0.25→0.50, 轨迹总时长+50%, 降低末端冲击)
        self.declare_parameter("direct_joint_max_vel", 0.5)           # rad/s, 直接插值模式最高速度 (平滑路径可更快)
        self.declare_parameter("sample_period", 0.1)
        self.declare_parameter("joint_tolerance", 0.003)
        self.declare_parameter("trajectory_start_delay", 0.5)
        self.declare_parameter("locked_grip_speed", 0.18)
        self.declare_parameter("settle_speed", 0.12)

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
                _is_arm_link(c.contact_body_1) and _is_arm_link(c.contact_body_2)
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
        wrapped_goal = [_nearest_angle(g, s) for g, s in zip(goal, start)]

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
        self._spin_future(future, timeout_sec=12.0)   # 8s planning + 4s overhead
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

        traj = self._stabilize_trajectory(resp.trajectory.joint_trajectory)
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
        self._spin_future(future, timeout_sec=12.0)   # must exceed allowed_planning_time (8s)
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

        traj = self._stabilize_trajectory(resp.trajectory.joint_trajectory)
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
        left_constraints = Constraints()
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
        left_constraints.position_constraints.append(lpc)
        # Orientation constraint
        loc = OrientationConstraint()
        loc.header.frame_id = "world"
        loc.link_name = "left_Link_06"
        loc.orientation = left_pose.orientation
        loc.absolute_x_axis_tolerance = 0.3
        loc.absolute_y_axis_tolerance = 0.3
        loc.absolute_z_axis_tolerance = 0.3
        loc.weight = 1.0
        left_constraints.orientation_constraints.append(loc)

        # ── 右臂约束 (right_Link_06) ──
        right_constraints = Constraints()
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
        right_constraints.position_constraints.append(rpc)
        roc = OrientationConstraint()
        roc.header.frame_id = "world"
        roc.link_name = "right_Link_06"
        roc.orientation = right_pose.orientation
        roc.absolute_x_axis_tolerance = 0.3
        roc.absolute_y_axis_tolerance = 0.3
        roc.absolute_z_axis_tolerance = 0.3
        roc.weight = 1.0
        right_constraints.orientation_constraints.append(roc)

        mr.goal_constraints.append(left_constraints)
        mr.goal_constraints.append(right_constraints)

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

        traj = self._stabilize_trajectory(resp.trajectory.joint_trajectory)
        if not traj.points:
            self.get_logger().error("Empty trajectory for dual pose target.")
            return None

        self.get_logger().info(
            f"Dual pose plan: {len(traj.points)} pts, {resp.planning_time:.3f}s"
        )
        return traj

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

    def _stabilize_trajectory(self, traj: JointTrajectory) -> JointTrajectory:
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
        current_positions = None
        js = self.current_joint_state
        if js is not None:
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
