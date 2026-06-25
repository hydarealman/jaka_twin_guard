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
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
)
from moveit_msgs.srv import GetMotionPlan
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
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

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
        """声明所有 ROS2 参数，默认值从 YAML 加载。"""
        # planner_params
        self.declare_parameter("planning_group", "both_arms")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time", 8.0)
        self.declare_parameter("max_velocity_scaling", 0.15)
        self.declare_parameter("max_acceleration_scaling", 0.15)
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
            rclpy.spin_until_future_complete(self, future, timeout_sec)

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
        self._spin_future(future, timeout_sec=12.0)
        result = future.result()
        if result is None:
            self.get_logger().error(f"MoveIt did not respond for {label}.")
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"MoveIt plan failed for {label}: code={resp.error_code.val}"
            )
            return None

        traj = resp.trajectory.joint_trajectory
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
        self._spin_future(future, timeout_sec=12.0)
        result = future.result()
        if result is None:
            self.get_logger().error("MoveIt did not respond for pose target.")
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(
                f"Pose plan failed: code={resp.error_code.val}"
            )
            return None

        traj = resp.trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().error("Empty trajectory for pose target.")
            return None

        self.get_logger().info(
            f"Pose plan: {len(traj.points)} pts, {resp.planning_time:.3f}s"
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
