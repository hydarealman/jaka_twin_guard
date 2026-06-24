#!/usr/bin/env python3
"""Task Runner — 完整的多阶段双臂搬运任务运行器。

6 阶段搬运流程（对齐旧 Demo 的 14 个 waypoints）:
  APPROACH → GRASP → LIFT → CARRY → RELEASE → RETREAT

其中 LIFT + CARRY 阶段使用锁定夹持线性插值（双臂闭链，不走 MoveIt RRT），
APPROACH / GRASP / RELEASE / RETREAT 阶段使用 MoveIt RRT 规划（开链）。

参考:
  - ManyMove (pastoriomarco/manymove) — BT 节点库 + ROS2 Action 集成
  - ARIAC 2024 — 多阶段装配任务编排
  - 旧 Demo: dual_arm_carry_demo.py — 已验证的 14 个 waypoints
"""

from __future__ import annotations

import copy
import sys
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time as RclpyTime
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from jaka_dual_arm.scene.scene_manager import SceneManager
from jaka_dual_arm.scene.perception_interface import PerceptionInterface


# ── 向量工具 ──────────────────────────────────────────

def _add(a: Point, b: Point) -> Point:
    return Point(x=a.x + b.x, y=a.y + b.y, z=a.z + b.z)

def _sub(a: Point, b: Point) -> Point:
    return Point(x=a.x - b.x, y=a.y - b.y, z=a.z - b.z)

def _scale(a: Point, s: float) -> Point:
    return Point(x=a.x * s, y=a.y * s, z=a.z * s)

def _norm(a: Point) -> float:
    return (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5

def _normalize(a: Point, fallback: Point) -> Point:
    length = _norm(a)
    return _scale(a, 1.0 / length) if length >= 1e-6 else fallback

def _dot(a: Point, b: Point) -> float:
    return a.x * b.x + a.y * b.y + a.z * b.z

def _cross(a: Point, b: Point) -> Point:
    return Point(x=a.y * b.z - a.z * b.y,
                 y=a.z * b.x - a.x * b.z,
                 z=a.x * b.y - a.y * b.x)

def _transform_point(transform, local_offset=(0.0, 0.0, 0.0)) -> Point:
    """从 TF 变换中提取位置（含旋转偏移）。"""
    t = transform.transform.translation
    q = transform.transform.rotation
    # 用四元数旋转局部偏移
    offset = Point(x=local_offset[0], y=local_offset[1], z=local_offset[2])
    if abs(q.w - 1.0) < 1e-6:
        rotated = offset
    else:
        uv = _cross(Point(x=q.x, y=q.y, z=q.z), offset)
        uuv = _cross(Point(x=q.x, y=q.y, z=q.z), uv)
        rotated = _add(offset, _scale(_add(_scale(uv, q.w), uuv), 2.0))
    return Point(x=t.x + rotated.x, y=t.y + rotated.y, z=t.z + rotated.z)


# ── Duration 工具 ──────────────────────────────────────

def _d(seconds: float) -> Duration:
    w = int(seconds)
    return Duration(sec=w, nanosec=int((seconds - w) * 1e9))


def _ds(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) / 1e9


# ── 6 阶段状态机 ─────────────────────────────────────────

class TaskState(Enum):
    INIT = auto()
    WAIT_SERVICES = auto()
    SETUP_SCENE = auto()
    DETECTING = auto()

    # 6 个搬运子阶段
    PLAN_APPROACH = auto()
    SEND_APPROACH = auto()
    WAIT_APPROACH = auto()

    PLAN_GRASP = auto()
    SEND_GRASP = auto()
    WAIT_GRASP = auto()

    PLAN_LIFT = auto()       # 锁定夹持（闭链插值）
    SEND_LIFT = auto()
    WAIT_LIFT = auto()

    PLAN_CARRY = auto()      # 锁定夹持（闭链插值）
    SEND_CARRY = auto()
    WAIT_CARRY = auto()

    PLAN_RELEASE = auto()
    SEND_RELEASE = auto()
    WAIT_RELEASE = auto()

    PLAN_RETREAT = auto()
    SEND_RETREAT = auto()
    WAIT_RETREAT = auto()

    SUCCESS = auto()
    FAILED = auto()


class CarryTaskRunner(Node):

    LEFT_JOINTS = [f"left_joint_{i}" for i in range(1, 7)]
    RIGHT_JOINTS = [f"right_joint_{i}" for i in range(1, 7)]
    ALL_JOINTS = LEFT_JOINTS + RIGHT_JOINTS

    # 6 阶段 → 配置 key 映射
    PHASE_KEYS = [
        "approach", "grasp", "lift", "carry", "release", "retreat",
    ]

    def __init__(self, scene_config: dict, robot_config: dict,
                 planner_config: dict, skill_config: dict):
        super().__init__("carry_task_runner")

        self._scene_cfg = scene_config
        self._robot_cfg = robot_config
        self._planner_cfg = planner_config
        self._skill_cfg = skill_config

        # ── TF 追踪（物块跟随机械臂）──
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._left_contact_frame = "left_grip_contact"
        self._right_contact_frame = "right_grip_contact"
        self._contact_offset_z = robot_config.get("end_effector", {}).get("contact_offset_z", 0.012)
        self._left_tip: Point = Point()
        self._right_tip: Point = Point()
        self._tf_ready = False
        self._cargo_state = "free"   # free | grasped | placed
        self._cargo_trail: list[Point] = []

        # ── Scene & Perception ──
        self.scene_mgr = SceneManager()
        self.scene_mgr.set_scene_dict(scene_config)
        self._world_frame = scene_config.get("world_frame", "world")
        self.perception = PerceptionInterface(self._world_frame)

        # ── 货物位姿从配置加载 ──
        cargo_cfg = scene_config.get("cargo", {})
        init = cargo_cfg.get("initial_pose", {"x": 0.36, "y": 0.02, "z": 0.06})
        self._cargo_pos = Point(x=init["x"], y=init["y"], z=init["z"])
        self._cargo_size = cargo_cfg.get("size", {"x": 0.18, "y": 0.345, "z": 0.12})
        self.get_logger().info(
            f"Cargo initial: ({self._cargo_pos.x:.2f}, {self._cargo_pos.y:.2f}, "
            f"{self._cargo_pos.z:.2f})"
        )

        # ── Marker ──
        self._marker_pub = self.create_publisher(MarkerArray, "/rviz_visual_tools", 10)

        # ── MoveIt 客户端 ──
        self._motion_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # ── 控制器 Action ──
        ctrl = robot_config.get("controllers", {})
        self._left_action = ActionClient(self, FollowJointTrajectory, ctrl["left"])
        self._right_action = ActionClient(self, FollowJointTrajectory, ctrl["right"])

        # ── 关节状态 ──
        self._current_joints: dict[str, float] = {}
        self._joints_received = False
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

        # ── 状态机 ──
        self._state = TaskState.INIT
        self._done = False
        self._target_pose: Optional[Pose] = None
        self._place_pose: Optional[Pose] = None

        # ── 阶段索引 (0-5: approach→retreat) ──
        self._phase_index = 0

        # ── 轨迹追踪 ──
        self._traj_done = {"left": True, "right": True}
        self._planned_traj = None

        # ── 场景标签 ──
        self._scene_label = scene_config.get("label", "Unknown Scene")
        self._cargo_state_label = "IDLE"

        # ── 关节等待日志抑制 ──
        self._last_joint_wait_log_ns = 0

        self._marker_timer = self.create_timer(0.1, self._publish_markers)

    # ── 关节状态 ───────────────────────────────────────

    def _on_joints(self, msg: JointState):
        if not self._joints_received and len(msg.name) > 0:
            self.get_logger().info(
                f"First /joint_states: {len(msg.name)} joints "
                f"(e.g. {msg.name[0]}={msg.position[0]:.3f})"
            )
            self._joints_received = True
        for n, p in zip(msg.name, msg.position):
            self._current_joints[n] = p

    def _get_start_positions(self, joint_names: list[str]) -> list[float]:
        try:
            return [self._current_joints[n] for n in joint_names]
        except KeyError as e:
            self.get_logger().error(f"Joint {e} not available")
            return []

    # ── 从配置读取阶段目标 ──────────────────────────────

    def _phase_target(self, phase_key: str) -> tuple[list[float], list[float]] | None:
        """从场景配置读取某个阶段的左右臂关节目标。"""
        targets = self._scene_cfg.get("targets", {})
        phase = targets.get(phase_key, {})
        left = phase.get("left")
        right = phase.get("right")
        if left is None or right is None:
            self.get_logger().error(f"No targets for phase '{phase_key}' in scene config")
            return None
        return (list(left), list(right))

    # ── 物块跟随（TF 追踪）──────────────────────────────

    def _update_cargo_from_tf(self) -> bool:
        """从 TF 查询双臂 grip_contact 位姿，更新物块中心位置。

        返回 True 表示 TF 数据可用，物块位置已更新。
        在 LIFT / CARRY / RELEASE 阶段调用。
        """
        try:
            left_tip = _transform_point(
                self._tf_buffer.lookup_transform(
                    self._world_frame, self._left_contact_frame, RclpyTime()),
                (0.0, 0.0, self._contact_offset_z),
            )
            right_tip = _transform_point(
                self._tf_buffer.lookup_transform(
                    self._world_frame, self._right_contact_frame, RclpyTime()),
                (0.0, 0.0, self._contact_offset_z),
            )
        except TransformException:
            return False

        self._left_tip = left_tip
        self._right_tip = right_tip

        # 物块中心 = 双臂接触点中点
        midpoint = _scale(_add(left_tip, right_tip), 0.5)
        self._cargo_pos = Point(x=midpoint.x, y=midpoint.y, z=midpoint.z)
        self._cargo_trail.append(self._cargo_pos)
        self._cargo_trail = self._cargo_trail[-500:]  # 保留最近 500 点
        return True

    def _update_cargo_state(self):
        """根据当前任务阶段更新物块状态（free / grasped / placed）。"""
        s = self._state
        # 锁定夹持阶段 → 物块跟随手臂
        if s in (TaskState.WAIT_LIFT, TaskState.PLAN_CARRY,
                 TaskState.SEND_CARRY, TaskState.WAIT_CARRY,
                 TaskState.PLAN_RELEASE, TaskState.SEND_RELEASE, TaskState.WAIT_RELEASE):
            if self._cargo_state != "grasped":
                self.get_logger().info("Cargo grasped — now tracking via TF.")
                self._cargo_state = "grasped"
        # 放置后 → 物块留在桌面
        elif s in (TaskState.PLAN_RETREAT, TaskState.SEND_RETREAT,
                   TaskState.WAIT_RETREAT, TaskState.SUCCESS):
            if self._cargo_state == "grasped":
                self._snap_cargo_to_table()
                self._cargo_state = "placed"
                self.get_logger().info("Cargo placed on table.")
        # 其他阶段 → 保持当前状态
        # 重置到初始位置（仅在任务的开始阶段）
        if s in (TaskState.DETECTING, TaskState.PLAN_APPROACH):
            if self._cargo_state != "free":
                self._cargo_state = "free"
                cargo_cfg = self._scene_cfg.get("cargo", {})
                init = cargo_cfg.get("initial_pose", {"x": 0.36, "y": 0.02, "z": 0.06})
                self._cargo_pos = Point(x=init["x"], y=init["y"], z=init["z"])

    def _snap_cargo_to_table(self):
        """将物块吸附到桌面（当物块在桌子上方时）。"""
        table = self._scene_cfg.get("table", {})
        if not table:
            return
        top_z = table.get("top_z", 0.30)
        table_cx = table.get("center", {}).get("x", 0.90)
        table_cy = table.get("center", {}).get("y", 0.0)
        half_x = table.get("size", {}).get("x", 0.75) / 2.0 + self._cargo_size["x"] / 2.0
        half_y = table.get("size", {}).get("y", 0.70) / 2.0 + self._cargo_size["y"] / 2.0

        if (abs(self._cargo_pos.x - table_cx) <= half_x and
            abs(self._cargo_pos.y - table_cy) <= half_y):
            self._cargo_pos.z = top_z + self._cargo_size["z"] / 2.0

    # ── Marker 可视化 ──────────────────────────────────

    def _publish_markers(self):
        # 物块跟随：在夹持阶段查询 TF 更新物块位置
        self._update_cargo_state()
        if self._cargo_state == "grasped":
            self._update_cargo_from_tf()

        ma = MarkerArray()
        ma.markers.append(self._ground_marker())
        ma.markers.append(self._cargo_marker())
        table = self._scene_cfg.get("table", {})
        if table:
            ma.markers.append(self._table_marker())
        bin_cfg = self._scene_cfg.get("bin", {})
        if bin_cfg.get("enabled", False):
            ma.markers.append(self._bin_marker())
            ma.markers.append(self._bin_marker_bottom())
        conv = self._scene_cfg.get("conveyor", {})
        if conv.get("enabled", False):
            ma.markers.append(self._conveyor_marker())
        # 接触球 + 支撑线 + 轨迹（夹持阶段）
        if self._cargo_state in ("grasped", "placed"):
            ma.markers.append(self._contact_marker(2))
            ma.markers.append(self._contact_marker(3))
            ma.markers.append(self._support_line_marker())
        if self._cargo_trail:
            ma.markers.append(self._trail_marker())
        ma.markers.append(self._scene_title_marker())
        ma.markers.append(self._state_label_marker())
        self._marker_pub.publish(ma)

    def _ground_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 0
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = 0.5; m.pose.position.z = -0.005; m.pose.orientation.w = 1.0
        m.scale.x = 2.5; m.scale.y = 2.5; m.scale.z = 0.01
        m.color.r = m.color.g = m.color.b = 0.30; m.color.a = 0.45
        return m

    def _cargo_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 1
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position = self._cargo_pos; m.pose.orientation.w = 1.0
        m.scale.x = self._cargo_size["x"]; m.scale.y = self._cargo_size["y"]
        m.scale.z = self._cargo_size["z"]
        if self._cargo_state == "grasped":
            m.color.r, m.color.g, m.color.b = 0.12, 0.42, 0.95  # 蓝色=搬运中
        elif self._cargo_state == "placed":
            m.color.r, m.color.g, m.color.b = 0.25, 0.75, 0.35  # 绿色=已放置
        else:
            m.color.r, m.color.g, m.color.b = 0.85, 0.36, 0.12  # 橙色=待抓取
        m.color.a = 0.85
        return m

    def _contact_marker(self, marker_id: int) -> Marker:
        """双臂接触点小球（绿色=夹持, 橙色=空闲）。"""
        tip = self._left_tip if marker_id == 2 else self._right_tip
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = marker_id
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position = tip; m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.035
        if self._cargo_state == "grasped":
            m.color.r, m.color.g, m.color.b = 0.25, 0.95, 0.35
        else:
            m.color.r, m.color.g, m.color.b = 0.95, 0.68, 0.18
        m.color.a = 0.95
        return m

    def _support_line_marker(self) -> Marker:
        """双臂接触点之间的支撑线。"""
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 11
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.points = [self._left_tip, self._right_tip]
        m.scale.x = 0.016
        if self._cargo_state == "grasped":
            m.color.r, m.color.g, m.color.b = 0.25, 0.95, 0.35; m.color.a = 0.90
        else:
            m.color.r = m.color.g = m.color.b = 0.55; m.color.a = 0.30
        return m

    def _trail_marker(self) -> Marker:
        """物块运动轨迹。"""
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 12
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.points = list(self._cargo_trail)
        m.scale.x = 0.010
        m.color.r, m.color.g, m.color.b = 0.05, 0.70, 0.85; m.color.a = 0.60
        return m

    def _table_marker(self) -> Marker:
        cfg = self._scene_cfg.get("table", {})
        sz = cfg.get("size", {"x": 0.75, "y": 0.70, "z": 0.04})
        c = cfg.get("center", {"x": 0.90, "y": 0.0})
        top_z = cfg.get("top_z", 0.30)
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 7
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = c["x"]; m.pose.position.y = c["y"]
        m.pose.position.z = top_z - sz["z"] / 2.0; m.pose.orientation.w = 1.0
        m.scale.x = sz["x"]; m.scale.y = sz["y"]; m.scale.z = sz["z"]
        m.color.r, m.color.g, m.color.b = 0.55, 0.44, 0.32; m.color.a = 0.75
        return m

    def _bin_marker(self) -> Marker:
        cfg = self._scene_cfg.get("bin", {})
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 8
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = cfg["center"]["x"]; m.pose.position.y = cfg["center"]["y"]
        m.pose.position.z = cfg["center"]["z"]; m.pose.orientation.w = 1.0
        m.scale.x = cfg["size"]["x"]; m.scale.y = cfg["size"]["y"]; m.scale.z = cfg["size"]["z"]
        m.color.r, m.color.g, m.color.b = 0.35, 0.35, 0.42; m.color.a = 0.30
        return m

    def _bin_marker_bottom(self) -> Marker:
        cfg = self._scene_cfg.get("bin", {})
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 9
        m.type = Marker.CUBE; m.action = Marker.ADD
        cx, cy, cz = cfg["center"]["x"], cfg["center"]["y"], cfg["center"]["z"]
        sx, sy, sz = cfg["size"]["x"], cfg["size"]["y"], cfg["size"]["z"]
        m.pose.position.x = cx; m.pose.position.y = cy
        m.pose.position.z = cz - sz / 2.0 + 0.003; m.pose.orientation.w = 1.0
        m.scale.x = sx; m.scale.y = sy; m.scale.z = 0.006
        m.color.r, m.color.g, m.color.b = 0.28, 0.28, 0.34; m.color.a = 0.85
        return m

    def _conveyor_marker(self) -> Marker:
        cfg = self._scene_cfg.get("conveyor", {})
        st, en = cfg["start"], cfg["end"]
        bw = cfg.get("belt_width", 0.30)
        mid_x = (st["x"] + en["x"]) / 2.0
        length = abs(en["x"] - st["x"])
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 10
        m.type = Marker.CUBE; m.action = Marker.ADD
        m.pose.position.x = mid_x; m.pose.position.y = st["y"]
        m.pose.position.z = st["z"]; m.pose.orientation.w = 1.0
        m.scale.x = length; m.scale.y = bw; m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b = 0.22, 0.24, 0.26; m.color.a = 0.80
        return m

    def _scene_title_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 5
        m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.x = 0.60; m.pose.position.y = 0.0; m.pose.position.z = 0.75
        m.scale.z = 0.08; m.text = self._scene_label
        m.color.r, m.color.g, m.color.b = 0.95, 0.85, 0.35; m.color.a = 0.90
        return m

    def _state_label_marker(self) -> Marker:
        m = Marker()
        m.header.frame_id = self._world_frame; m.ns = "scene"; m.id = 6
        m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
        m.pose.position.x = self._cargo_pos.x; m.pose.position.y = self._cargo_pos.y
        m.pose.position.z = self._cargo_pos.z + self._cargo_size["z"] + 0.08
        m.scale.z = 0.05
        # 优先显示 cargo_state，否则显示阶段标签
        if self._cargo_state == "grasped":
            m.text = "GRASPED"
        elif self._cargo_state == "placed":
            m.text = "PLACED"
        else:
            m.text = self._cargo_state_label
        m.color.r = m.color.g = m.color.b = 1.0; m.color.a = 0.85
        return m

    # ── Scene Setup ───────────────────────────────────

    def setup_scene(self) -> bool:
        self.get_logger().info("Setting up scene...")
        self.scene_mgr.register_table()
        bin_cfg = self._scene_cfg.get("bin", {})
        if bin_cfg.get("enabled", False):
            self.scene_mgr.register_bin()
        self.scene_mgr.register_cargo()

        cargo = self._scene_cfg.get("cargo", {})
        init = cargo.get("initial_pose", {})
        self.perception.add_mock_object(
            "cargo_box",
            (init.get("x", 0.36), init.get("y", 0.02), init.get("z", 0.06)),
        )

        table = self._scene_cfg.get("table", {})
        if table:
            self._place_pose = Pose()
            self._place_pose.position.x = table.get("center", {}).get("x", 0.90)
            self._place_pose.position.y = table.get("center", {}).get("y", 0.0)
            self._place_pose.position.z = table.get("top_z", 0.30)
            self._place_pose.orientation.w = 1.0
        self.get_logger().info(f"Scene ready: {self._scene_label}")
        return True

    # ── 状态机主循环 ──────────────────────────────────

    def run(self):
        self.get_logger().info("=" * 55)
        self.get_logger().info(f"  JAKA Dual-Arm | {self._scene_label}")
        self.get_logger().info("  6-Phase: APPROACH→GRASP→LIFT→CARRY→RELEASE→RETREAT")
        self.get_logger().info("=" * 55)
        self._state = TaskState.WAIT_SERVICES

        try:
            while rclpy.ok() and not self._done:
                self._tick()
                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            self.get_logger().info("Interrupted.")
        self.get_logger().info(f"Final: {self._state.name}")

    def _tick(self):
        s = self._state
        # Phase 0: Approach
        if s == TaskState.PLAN_APPROACH:   self._plan_phase("approach", TaskState.SEND_APPROACH)
        elif s == TaskState.SEND_APPROACH: self._send_phase(TaskState.WAIT_APPROACH)
        elif s == TaskState.WAIT_APPROACH: self._wait_phase(TaskState.PLAN_GRASP, "APPROACH")
        # Phase 1: Grasp
        elif s == TaskState.PLAN_GRASP:    self._plan_phase("grasp", TaskState.SEND_GRASP)
        elif s == TaskState.SEND_GRASP:    self._send_phase(TaskState.WAIT_GRASP)
        elif s == TaskState.WAIT_GRASP:    self._wait_phase(TaskState.PLAN_LIFT, "GRASP")
        # Phase 2: Lift (锁定夹持)
        elif s == TaskState.PLAN_LIFT:     self._plan_locked_grip("lift", TaskState.SEND_LIFT)
        elif s == TaskState.SEND_LIFT:     self._send_phase(TaskState.WAIT_LIFT)
        elif s == TaskState.WAIT_LIFT:     self._wait_phase(TaskState.PLAN_CARRY, "LIFT")
        # Phase 3: Carry (锁定夹持)
        elif s == TaskState.PLAN_CARRY:    self._plan_locked_grip("carry", TaskState.SEND_CARRY)
        elif s == TaskState.SEND_CARRY:    self._send_phase(TaskState.WAIT_CARRY)
        elif s == TaskState.WAIT_CARRY:    self._wait_phase(TaskState.PLAN_RELEASE, "CARRY")
        # Phase 4: Release (放置)
        elif s == TaskState.PLAN_RELEASE:  self._plan_phase("release", TaskState.SEND_RELEASE)
        elif s == TaskState.SEND_RELEASE:  self._send_phase(TaskState.WAIT_RELEASE)
        elif s == TaskState.WAIT_RELEASE:  self._wait_phase(TaskState.PLAN_RETREAT, "RELEASE")
        # Phase 5: Retreat (撤离)
        elif s == TaskState.PLAN_RETREAT:  self._plan_phase("retreat", TaskState.SEND_RETREAT)
        elif s == TaskState.SEND_RETREAT:  self._send_phase(TaskState.WAIT_RETREAT)
        elif s == TaskState.WAIT_RETREAT:  self._wait_phase(TaskState.SUCCESS, "RETREAT")
        # Meta states
        elif s == TaskState.WAIT_SERVICES: self._wait()
        elif s == TaskState.SETUP_SCENE:   self._move_to(TaskState.DETECTING, self.setup_scene)
        elif s == TaskState.DETECTING:     self._detect()
        elif s == TaskState.SUCCESS:       self._done = True
        elif s == TaskState.FAILED:        self._done = True

    # ── 通用状态方法 ──────────────────────────────────

    def _move_to(self, next_state: TaskState, action_fn=None):
        """执行一个动作后立即转移到下个状态。"""
        if action_fn:
            action_fn()
        self._state = next_state

    def _wait(self):
        self._cargo_state_label = "WAITING..."
        if not self._motion_client.wait_for_service(timeout_sec=0.1):
            return
        if not self._ik_client.wait_for_service(timeout_sec=0.1):
            return
        if not self._left_action.wait_for_server(timeout_sec=0.1):
            return
        if not self._right_action.wait_for_server(timeout_sec=0.1):
            return
        if not self._joints_received:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_joint_wait_log_ns > 2_000_000_000:
                self.get_logger().info("Waiting for /joint_states...")
                self._last_joint_wait_log_ns = now_ns
            return
        self.get_logger().info("All services + joint states ready.")
        self._state = TaskState.SETUP_SCENE

    def _detect(self):
        self._cargo_state_label = "DETECTING..."
        self.get_logger().info("[0/6] Detecting cargo...")
        pose = self.perception.detect("cargo_box")
        if pose is None:
            self.get_logger().error("cargo_box not found!")
            self._state = TaskState.FAILED
            return
        self._target_pose = pose
        self.get_logger().info(f"  Found at ({pose.position.x:.3f}, {pose.position.y:.3f})")
        self._phase_index = 0
        self._state = TaskState.PLAN_APPROACH

    # ── Phase: Open-Chain (MoveIt RRT) ─────────────────

    def _plan_phase(self, phase_key: str, next_state: TaskState):
        """使用 MoveIt RRT 规划开链阶段（approach/grasp/release/retreat）。"""
        idx = self.PHASE_KEYS.index(phase_key) + 1
        label = phase_key.upper()
        self._cargo_state_label = f"PLAN {label}..."
        self.get_logger().info(f"[{idx}/6] Planning {label} via MoveIt RRT...")

        target = self._phase_target(phase_key)
        if target is None:
            self._state = TaskState.FAILED; return
        left_j, right_j = target

        start = self._get_start_positions(self.ALL_JOINTS)
        if not start:
            self._state = TaskState.FAILED; return

        self._planned_traj = self._plan_joint_target(
            self.ALL_JOINTS, start, left_j + right_j,
            planning_time=8.0, velocity_scale=0.15,
        )
        if self._planned_traj is None:
            self.get_logger().error(f"{label} plan failed!")
            self._state = TaskState.FAILED; return
        self._state = next_state

    def _send_phase(self, next_state: TaskState):
        """发送轨迹到双臂控制器。"""
        self._traj_done["left"] = False
        self._traj_done["right"] = False
        self._send_trajectory("left")
        self._send_trajectory("right")
        self._state = next_state

    def _wait_phase(self, next_state: TaskState, label: str):
        """等待双臂轨迹执行完成。"""
        if self._traj_done["left"] and self._traj_done["right"]:
            self.get_logger().info(f"[OK] {label} complete.")
            self._state = next_state

    # ── Phase: Closed-Chain (Locked Grip Interpolation) ─

    def _plan_locked_grip(self, phase_key: str, next_state: TaskState):
        """锁定夹持线性插值 — 双臂保持恒定间距同步移动。"""
        idx = self.PHASE_KEYS.index(phase_key) + 1
        label = phase_key.upper()
        self._cargo_state_label = f"PLAN {label}..."
        self.get_logger().info(f"[{idx}/6] Planning {label} via locked-grip interpolation...")

        target = self._phase_target(phase_key)
        if target is None:
            self._state = TaskState.FAILED; return
        left_j, right_j = target

        start = self._get_start_positions(self.ALL_JOINTS)
        if len(start) < 12:
            self._state = TaskState.FAILED; return

        goal = list(left_j) + list(right_j)
        max_delta = max(abs(a - b) for a, b in zip(start, goal))
        local_duration = max(1.0, max_delta / 0.18)
        point_count = max(2, int(local_duration / 0.08) + 1)

        traj = JointTrajectory()
        traj.joint_names = self.ALL_JOINTS
        for pi in range(point_count):
            ratio = pi / (point_count - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [float(start[i] + (goal[i] - start[i]) * ratio) for i in range(12)]
            pt.time_from_start = _d(local_duration * ratio)
            traj.points.append(pt)

        self._planned_traj = traj
        self.get_logger().info(
            f"  Locked-grip: {point_count} pts, {local_duration:.1f}s "
            f"(max joint delta={max_delta:.3f} rad)"
        )
        self._state = next_state

    # ── 轨迹规划 (MoveIt RRT) ──────────────────────────

    def _plan_joint_target(self, joint_names: list[str],
                           start_positions: list[float],
                           target_positions: list[float],
                           planning_time: float = 8.0,
                           velocity_scale: float = 0.15) -> JointTrajectory | None:
        req = GetMotionPlan.Request()
        mr = req.motion_plan_request
        mr.group_name = "both_arms"
        mr.planner_id = "RRTConnectkConfigDefault"
        mr.num_planning_attempts = 10
        mr.allowed_planning_time = planning_time
        mr.max_velocity_scaling_factor = velocity_scale
        mr.max_acceleration_scaling_factor = velocity_scale

        mr.start_state.is_diff = True
        mr.start_state.joint_state = JointState()
        mr.start_state.joint_state.name = list(joint_names)
        mr.start_state.joint_state.position = [float(p) for p in start_positions]

        c = Constraints()
        for name, pos in zip(joint_names, target_positions):
            c.joint_constraints.append(
                JointConstraint(joint_name=name, position=float(pos),
                                tolerance_above=0.005, tolerance_below=0.005, weight=1.0))
        mr.goal_constraints.append(c)

        fut = self._motion_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=planning_time + 4.0)
        resp = fut.result()

        if resp is None:
            self.get_logger().error("MoveIt plan: no response")
            return None
        if resp.motion_plan_response.error_code.val != 1:
            self.get_logger().error(f"MoveIt plan error: {resp.motion_plan_response.error_code.val}")
            return None

        traj = resp.motion_plan_response.trajectory.joint_trajectory
        self.get_logger().info(
            f"  RRT plan OK: {len(traj.points)} pts, "
            f"duration={_ds(traj.points[-1].time_from_start):.1f}s"
        )
        return traj

    # ── 轨迹发送 ──────────────────────────────────────

    def _send_trajectory(self, side: str):
        if self._planned_traj is None:
            self.get_logger().error("No planned trajectory")
            return

        joints = self.LEFT_JOINTS if side == "left" else self.RIGHT_JOINTS
        client = self._left_action if side == "left" else self._right_action
        label = f"{side}_arm"

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"{label} action server unavailable")
            return

        traj = JointTrajectory()
        traj.joint_names = joints
        idx_map = {n: i for i, n in enumerate(self._planned_traj.joint_names)}
        for pt in self._planned_traj.points:
            np_pt = JointTrajectoryPoint()
            np_pt.positions = [pt.positions[idx_map[j]] for j in joints]
            np_pt.time_from_start = pt.time_from_start
            traj.points.append(np_pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = _d(2.0)

        self.get_logger().info(f"{label}: sending {len(traj.points)}-pt trajectory...")
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f, l=label: self._on_goal(f, l))

    def _on_goal(self, future, label: str):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().error(f"{label} goal rejected")
            side = label.split("_")[0]
            self._traj_done[side] = True
            return
        self.get_logger().info(f"{label} accepted. Executing...")
        handle.get_result_async().add_done_callback(
            lambda f, l=label: self._on_result(f, l))

    def _on_result(self, future, label: str):
        r = future.result().result
        side = label.split("_")[0]
        if r.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"{label} done.")
        else:
            self.get_logger().error(f"{label} failed: {r.error_string}")
        self._traj_done[side] = True
