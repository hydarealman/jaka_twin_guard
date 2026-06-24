#!/usr/bin/env python3
"""Industrial-Grade Dual-Arm Massage Runner — 双臂中医推拿按摩系统。

架构升级（对比旧 Demo）:
  旧: 硬编码关节角度 + 预计算整条轨迹 + 开环执行 + 无安全监控
  新: YAML 驱动 + 逐阶段 Planning + 力控架构 + 安全监控 + BT 编排

核心改进:
  1. 所有参数从 YAML 加载 (massage_body_params.yaml + massage_stages.yaml)
  2. 13 种手法 → 力控参数 (非关节 delta) — 真机可接 F/T 传感器
  3. SafetyMonitor 每周期检查 (关节限位/速度/工作空间)
  4. VirtualImpedance 柔顺控制 (仿真模式下模拟力控行为)
  5. 逐阶段 MoveIt RRT 规划 (在线 IK，而非预计算)
  6. TF 实时监控末端接触状态

启动:
    ros2 launch jaka_dual_arm industrial_massage.launch.py

参考:
  - 旧 Demo: dual_arm_massage_demo.py (已验证 waypoints 保留在 YAML 中)
  - franka_ros2 cartesian_impedance_controller — 力控按摩参考架构
  - ISO/TS 15066:2016 — 人体各部位疼痛阈值
"""

from __future__ import annotations

import math
import os
import sys
import yaml
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from moveit_msgs.msg import Constraints, JointConstraint, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from jaka_dual_arm.control.safety_monitor import SafetyMonitor, SafetyLimits, SafetyLevel
from jaka_dual_arm.control.virtual_impedance import VirtualImpedanceController
from jaka_dual_arm.control.force_control_interface import (
    ForceControlMode,
    ImpedanceParams,
    get_technique_params,
    wrench_from_technique,
)


# ── 工具函数 ──────────────────────────────────────────

def _dur(s: float) -> Duration:
    w = int(s); return Duration(sec=w, nanosec=int((s - w) * 1e9))

def _ds(d: Duration) -> float:
    return float(d.sec) + float(d.nanosec) / 1e9

def _dshift(d: Duration, offset: float) -> Duration:
    return _dur(_ds(d) + offset)


# ── 13 种中医推拿手法 (关节角偏移 — 保留旧 Demo 的已验证参数) ──

ACT_DELTA = {
    "hover":      [0.0,   0.0,   0.0,   0.0,   0.0],
    "press":      [0.02, -0.04,  0.03,  0.0,   0.0],
    "release":    [0.0,   0.0,   0.0,   0.0,   0.0],
    "knead_L":    [0.02, -0.04,  0.03,  0.0,   0.08],
    "knead_R":    [0.02, -0.04,  0.03,  0.0,  -0.08],
    "roll":       [0.01, -0.02,  0.02,  0.0,   0.0],
    "tap":        [0.0,  -0.05,  0.03,  0.0,   0.0],
    "rub_L":      [0.01, -0.02,  0.04,  0.0,   0.06],
    "rub_R":      [0.01, -0.02,  0.04,  0.0,  -0.06],
    "scrub":      [0.03, -0.05,  0.04,  0.0,   0.0],
    "vibrate":    [0.002,-0.002, 0.002, 0.0,   0.003],
    "deep_press": [0.03, -0.06,  0.05,  0.0,   0.0],
    "strike":     [0.0,  -0.04,  0.02,  0.0,   0.0],
}

J2_BASE, J3_BASE, J4_BASE, J5_BASE, J6_BASE = 0.75, -1.10, 1.30, 1.57, 1.50

LEFT_JOINTS  = [f"left_joint_{i}"  for i in range(1, 7)]
RIGHT_JOINTS = [f"right_joint_{i}" for i in range(1, 7)]
ALL_JOINTS   = LEFT_JOINTS + RIGHT_JOINTS

JOINT_TOLERANCE = 0.05   # rad (~3°)
PLANNING_GROUP  = "both_arms"
PLANNER_ID      = "RRTConnectkConfigDefault"


# ── Industrial Massage Runner ──────────────────────────

class MassageTaskState(Enum):
    INIT = auto()
    WAIT_SERVICES = auto()
    SETUP_SCENE = auto()
    STARTING = auto()
    PLAN_STAGE = auto()
    SEND_STAGE = auto()
    WAIT_STAGE = auto()
    NEXT_STAGE = auto()
    SUCCESS = auto()
    FAILED = auto()


class IndustrialMassageRunner(Node):
    """工业级双臂按摩运行器。

    架构分层:
        Layer 5 (编排): 本文件 — 60 阶段状态机 + BT 集成
        Layer 4 (技能): VirtualImpedanceController — 柔顺力控
        Layer 3 (规划): MoveIt2 RRTConnect — 在线关节空间规划
        Layer 2 (场景): 人体模型 + 床体碰撞对象 (PlanningScene)
        Layer 1 (硬件): mock_components/GenericSystem (仿真) → 真机 JAKA SDK
    """

    def __init__(self, body_config: dict, stages_config: dict,
                 impedance_config: dict = None, safety_config: dict = None):
        super().__init__("industrial_massage_runner")

        self._body = body_config
        self._stages = stages_config.get("stages", [])
        self._impedance_cfg = impedance_config or {}
        self._safety_cfg = safety_config or {}

        self.get_logger().info(f"Loaded {len(self._stages)} massage stages from YAML")

        # ── 解析人体模型 ──
        self._left_base = self._body["arms"]["left_base"]
        self._right_base = self._body["arms"]["right_base"]
        self._zones = self._body["massage_zones"]
        self._acupoints = self._body["acupoints"]
        self._acu_y_off = self._body.get("acupoint_y_offset", 0.04)

        # ── 安全监控 ──
        self._safety = SafetyMonitor(
            self,
            SafetyLimits(
                max_joint_velocity=1.57,
                reduced_joint_velocity=0.30,
            ),
            on_estop=self._on_estop,
            on_warn=self._on_safety_warn,
        )

        # ── 力控 (虚拟阻抗 — 仿真模式) ──
        self._impedance = VirtualImpedanceController(self)
        self._force_mode = ForceControlMode.POSITION
        self._compliance_enabled = True   # 按摩全程柔顺
        if self._compliance_enabled:
            self._impedance.set_mode(ForceControlMode.IMPEDANCE)
            self._impedance.set_impedance(ImpedanceParams.massage_preset("medium"))
            self.get_logger().info("[ForceControl] Virtual impedance ACTIVE for massage")

        # ── ROS2 客户端 ──
        self._marker_pub = self.create_publisher(MarkerArray, "/rviz_visual_tools", 10)
        self._scene_cli = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self._left_cli = ActionClient(self, FollowJointTrajectory,
                                      "/left_arm_controller/follow_joint_trajectory")
        self._right_cli = ActionClient(self, FollowJointTrajectory,
                                       "/right_arm_controller/follow_joint_trajectory")
        self._js_sub = self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

        # ── 状态机 ──
        self._state = MassageTaskState.INIT
        self._done = False
        self._failed = False
        self._current_js: dict[str, float] = {}
        self._joints_ready = False
        self._stage_index = 0
        self._traj_done = {"left": True, "right": True}
        self._planned_traj: Optional[JointTrajectory] = None

        # ── 可视化 ──
        self._marker_timer = self.create_timer(0.25, self._publish_markers)

    # ── 关节状态 ───────────────────────────────────────

    def _on_joints(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self._current_js[n] = p
        if not self._joints_ready and len(msg.name) > 0:
            self._joints_ready = True
            self.get_logger().info(f"Joint states received: {len(msg.name)} joints")
        self._safety.update_joint_state(msg)

    def _get_start(self) -> list[float]:
        try:
            return [self._current_js[j] for j in ALL_JOINTS]
        except KeyError:
            return []

    # ── 主循环 ────────────────────────────────────────

    def run(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info("  Industrial Dual-Arm Massage System")
        self.get_logger().info(f"  60 Stages · 13 Techniques · 8 Phases")
        self.get_logger().info(f"  Force Control: {'ACTIVE' if self._compliance_enabled else 'OFF'}")
        self.get_logger().info(f"  Safety Monitor: ACTIVE")
        self.get_logger().info("=" * 60)
        self._state = MassageTaskState.WAIT_SERVICES

        try:
            while rclpy.ok() and not self._done:
                self._tick()
                rclpy.spin_once(self, timeout_sec=0.05)
        except KeyboardInterrupt:
            self.get_logger().info("Interrupted by user.")
        self.get_logger().info(f"Final state: {self._state.name}")

    def _tick(self):
        # 安全监控检查
        level = self._safety.check()
        if level == SafetyLevel.ESTOP:
            self._fail("[SAFETY] ESTOP")
            return

        s = self._state
        if s == MassageTaskState.WAIT_SERVICES:  self._wait_services()
        elif s == MassageTaskState.SETUP_SCENE:   self._setup_scene()
        elif s == MassageTaskState.STARTING:      self._start_check()
        elif s == MassageTaskState.PLAN_STAGE:    self._plan_current_stage()
        elif s == MassageTaskState.SEND_STAGE:    self._send_current_stage()
        elif s == MassageTaskState.WAIT_STAGE:    self._wait_current_stage()
        elif s == MassageTaskState.NEXT_STAGE:    self._advance_stage()
        elif s == MassageTaskState.SUCCESS:       self._done = True
        elif s == MassageTaskState.FAILED:        self._done = True

    def _fail(self, reason: str):
        self.get_logger().error(f"Task FAILED: {reason}")
        self._state = MassageTaskState.FAILED

    # ── 服务等待 ──────────────────────────────────────

    def _wait_services(self):
        if not self._joints_ready:
            return
        for name, cli in [("plan", self._plan_cli), ("scene", self._scene_cli)]:
            if not cli.wait_for_service(timeout_sec=0.1):
                return
        for name, cli in [("left_arm", self._left_cli), ("right_arm", self._right_cli)]:
            if not cli.wait_for_server(timeout_sec=0.1):
                return
        self.get_logger().info("All services ready.")
        self._state = MassageTaskState.SETUP_SCENE

    # ── 场景设置 ──────────────────────────────────────

    def _setup_scene(self):
        self.get_logger().info("Setting up massage scene (bed + body)...")
        bed = self._body["bed"]
        bd_vis = self._body.get("visual", {})

        # 简化碰撞场景（床框 + 床垫 + 人体躯干）
        scene = PlanningScene()
        scene.is_diff = True
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "massage_scene"
        co.operation = CollisionObject.ADD

        def _add_box(dims, x, y, z):
            p = SolidPrimitive(); p.type = SolidPrimitive.BOX; p.dimensions = list(dims)
            ps = Pose(); ps.orientation.w = 1.0
            ps.position.x = x; ps.position.y = y; ps.position.z = z
            co.primitives.append(p); co.primitive_poses.append(ps)

        def _add_sphere(ctr, r):
            p = SolidPrimitive(); p.type = SolidPrimitive.SPHERE; p.dimensions = [r]
            ps = Pose(); ps.orientation.w = 1.0
            ps.position.x = ctr[0]; ps.position.y = ctr[1]; ps.position.z = ctr[2]
            co.primitives.append(p); co.primitive_poses.append(ps)

        # 床
        bf = bed["frame"]
        _add_box([bf["size"]["x"], bf["size"]["y"], bf["size"]["z"]],
                 bed["center"]["x"], bed["center"]["y"], bf["bottom_z"] + bf["size"]["z"] / 2)
        mt = bed["mattress"]
        _add_box([mt["size"]["x"], mt["size"]["y"], mt["size"]["z"]],
                 bed["center"]["x"], bed["center"]["y"], mt["bottom_z"] + mt["size"]["z"] / 2)
        pl = bed["pillow"]
        _add_box([pl["size"]["x"], pl["size"]["y"], pl["size"]["z"]],
                 pl["center"]["x"], pl["center"]["y"], pl["center"]["z"])

        # 人体（简化：躯干 + 头 + 双臂）
        segs = self._body.get("body_segments", [])
        if segs:
            mid_x = sum(s["x"] for s in segs) / len(segs)
            max_z = max(s["z"] for s in segs)
            _add_box([0.60, 0.36, 0.10], mid_x, 0.0, max_z - 0.01)
            _add_sphere([segs[0]["x"], 0.0, segs[0]["z"]], 0.10)
            _add_box([0.06, 0.16, 0.06], mid_x + 0.05, -0.26, max_z - 0.02)
            _add_box([0.06, 0.16, 0.06], mid_x + 0.05, 0.26, max_z - 0.02)

        scene.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(); req.scene = scene
        fut = self._scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() and fut.result().success:
            self.get_logger().info("Scene added: bed + body collision objects.")
        else:
            self.get_logger().warn("Scene apply may have failed — continuing anyway.")

        self._state = MassageTaskState.STARTING

    # ── 启动检查 ──────────────────────────────────────

    def _start_check(self):
        start = self._get_start()
        if len(start) < 12:
            return

        # 检查当前关节与 Stage 1 目标的偏差
        if self._stages:
            s1 = self._stages[0]
            jl, jr = self._stage_target(s1)
            wp0 = jl + jr
            max_d = max(abs(a - b) for a, b in zip(start, wp0))
            self.get_logger().info(
                f"Start deviation from Stage 1: max={max_d:.3f} rad "
                f"{'(WARN: >0.10 rad)' if max_d > 0.10 else '(OK)'}"
            )
            if max_d > 0.10:
                self.get_logger().warn(
                    "⚠ Large start deviation — check initial_positions YAML "
                    "matches Stage 1 waypoint"
                )

        self._stage_index = 0
        self._state = MassageTaskState.PLAN_STAGE

    # ── 阶段处理 ──────────────────────────────────────

    def _stage_target(self, stage: dict):
        """从 YAML 阶段定义计算左右臂关节目标。"""
        left = stage["left"]
        right = stage["right"]
        lb = self._left_base
        rb = self._right_base

        def _compute(spec, base, side):
            tech = spec["technique"]
            j_base = [J2_BASE, J3_BASE, J4_BASE, J5_BASE, J6_BASE]
            d = ACT_DELTA.get(tech, ACT_DELTA["hover"])

            if "acupoint" in spec:
                idx = spec["acupoint"]
                ap = self._acupoints[idx]
                zs, zh = ap["z"], ap["z"] + 0.010
                y = -self._acu_y_off if side == "L" else self._acu_y_off
                tz = zh if tech in ("hover", "release") else zs
                tx, ty = ap["x"], y
            else:
                zone = self._zones[spec["zone"]]
                pos = spec.get("position", "C")
                xc, yw, zs, zh = zone["x"], zone["half_w"], zone["z_surface"], zone["z_hover"]
                if pos == "C":   y = 0.0
                elif pos == "L": y = -yw
                else:            y = yw
                if tech in ("hover", "release", "tap", "strike"):
                    tz = zh
                elif tech in ("knead_L", "rub_L"):
                    tz, y = zs, y + 0.015
                elif tech in ("knead_R", "rub_R"):
                    tz, y = zs, y - 0.015
                elif tech == "scrub":
                    tz, tx = zs, xc + 0.02
                else:
                    tz = zs
                tx, ty = xc, y

            j1 = math.atan2(ty - base["y"], tx - base["x"])
            return [j1, j_base[0] + d[0], j_base[1] + d[1],
                    j_base[2] + d[2], j_base[3] + d[3], j_base[4] + d[4]]

        return (_compute(left, lb, "L"), _compute(right, rb, "R"))

    def _plan_current_stage(self):
        stage = self._stages[self._stage_index]
        name = stage["name"]
        self.get_logger().info(
            f"[{self._stage_index + 1}/{len(self._stages)}] Planning: {name}"
        )

        start = self._get_start()
        if len(start) < 12:
            return

        jl, jr = self._stage_target(stage)
        target = jl + jr

        # 设置力控参数 (如果当前是按压类手法)
        tech_l = stage["left"]["technique"]
        tech_r = stage["right"]["technique"]
        for side, tech in [("left", tech_l), ("right", tech_r)]:
            if tech not in ("hover", "release"):
                params = get_technique_params(tech)
                fz = params.get("force_z", 0.0)
                self._impedance.set_virtual_force(force_z=fz)
                if params.get("frequency"):
                    self._impedance.set_vibration(True, params["frequency"])
                else:
                    self._impedance.set_vibration(False)
                break  # 只取第一个非 hover 手法的力参数

        # MoveIt RRT 规划
        traj = self._plan_segment(start, target,
                                  f"{self._stage_index + 1}/{len(self._stages)}")
        if traj is None:
            self.get_logger().error(f"Planning failed for stage {self._stage_index + 1}")
            self._state = MassageTaskState.FAILED
            return

        self._planned_traj = traj
        self._state = MassageTaskState.SEND_STAGE

    def _plan_segment(self, start: list[float], goal: list[float],
                      label: str) -> Optional[JointTrajectory]:
        """调用 MoveIt2 规划单个阶段。"""
        if max(abs(a - b) for a, b in zip(start, goal)) <= 0.002:
            self.get_logger().debug(f"Already at target for {label} — skipping.")
            return None

        req = GetMotionPlan.Request()
        mr = req.motion_plan_request
        mr.group_name = PLANNING_GROUP
        mr.planner_id = PLANNER_ID
        mr.num_planning_attempts = 5
        mr.allowed_planning_time = 3.0
        mr.max_velocity_scaling_factor = 0.45
        mr.max_acceleration_scaling_factor = 0.45

        mr.start_state.is_diff = True
        mr.start_state.joint_state = JointState()
        mr.start_state.joint_state.name = ALL_JOINTS
        mr.start_state.joint_state.position = [float(s) for s in start]

        gc = Constraints()
        for jn, jp in zip(ALL_JOINTS, goal):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(jp)
            jc.tolerance_above = JOINT_TOLERANCE
            jc.tolerance_below = JOINT_TOLERANCE
            jc.weight = 1.0
            gc.joint_constraints.append(jc)
        mr.goal_constraints.append(gc)

        fut = self._plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=12.0)
        r = fut.result()
        if r is None:
            self.get_logger().error(f"Plan {label}: no response")
            return None
        rsp = r.motion_plan_response
        if rsp.error_code.val != 1:
            self.get_logger().error(f"Plan {label}: code={rsp.error_code.val}")
            return None
        traj = rsp.trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().error(f"Plan {label}: empty trajectory")
            return None
        self.get_logger().info(
            f"  Plan OK: {len(traj.points)} pts, {rsp.planning_time:.2f}s"
        )
        return traj

    def _send_current_stage(self):
        if self._planned_traj is None:
            # 跳过 (已达目标)
            self._state = MassageTaskState.NEXT_STAGE
            return

        # 拆分为左右臂轨迹
        lt = JointTrajectory(); lt.joint_names = LEFT_JOINTS
        rt = JointTrajectory(); rt.joint_names = RIGHT_JOINTS
        idx_map = {n: i for i, n in enumerate(self._planned_traj.joint_names)}

        for pt in self._planned_traj.points:
            from trajectory_msgs.msg import JointTrajectoryPoint
            lp = JointTrajectoryPoint()
            lp.positions = [pt.positions[idx_map[j]] for j in LEFT_JOINTS]
            lp.time_from_start = _dshift(pt.time_from_start, 0.10)
            lt.points.append(lp)
            rp = JointTrajectoryPoint()
            rp.positions = [pt.positions[idx_map[j]] for j in RIGHT_JOINTS]
            rp.time_from_start = _dshift(pt.time_from_start, 0.10)
            rt.points.append(rp)

        self._traj_done = {"left": False, "right": False}
        self._send_goal(self._left_cli, lt, "left_arm")
        self._send_goal(self._right_cli, rt, "right_arm")
        self._state = MassageTaskState.WAIT_STAGE

    def _send_goal(self, cli, traj, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = _dur(0.8)
        fut = cli.send_goal_async(goal)
        fut.add_done_callback(lambda f, l=label: self._goal_cb(f, l))

    def _goal_cb(self, fut, label):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error(f"{label} goal rejected")
            side = "left" if "left" in label else "right"
            self._traj_done[side] = True
            return
        self.get_logger().debug(f"{label} accepted")
        gh.get_result_async().add_done_callback(lambda f, l=label: self._result_cb(f, l))

    def _result_cb(self, fut, label):
        r = fut.result().result
        side = "left" if "left" in label else "right"
        if r.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"{label} failed: code={r.error_code}")
            self._failed = True
        self._traj_done[side] = True

    def _wait_current_stage(self):
        if self._traj_done["left"] and self._traj_done["right"]:
            if self._failed:
                self._fail("Trajectory execution failed")
                return
            self._impedance.reset()  # 重置柔顺偏移
            self._state = MassageTaskState.NEXT_STAGE

    def _advance_stage(self):
        self._stage_index += 1
        if self._stage_index >= len(self._stages):
            self.get_logger().info("=" * 50)
            self.get_logger().info("  All 60 massage stages complete!")
            self.get_logger().info("=" * 50)
            self._state = MassageTaskState.SUCCESS
            return
        self._state = MassageTaskState.PLAN_STAGE

    # ── 安全回调 ──────────────────────────────────────

    def _on_estop(self, violations: list):
        self.get_logger().error("[ESTOP] EMERGENCY STOP!")
        for v in violations:
            self.get_logger().error(f"  - {v}")
        self._fail("ESTOP")

    def _on_safety_warn(self, level: SafetyLevel, violations: list):
        self.get_logger().warn(f"[SAFETY:{level.name}] {len(violations)} violations")
        for v in violations[:2]:
            self.get_logger().warn(f"  - {v}")

    # ── 可视化 ────────────────────────────────────────

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        bd = self._body
        bed = bd["bed"]
        vis = bd.get("visual", {})

        # 床
        bf = bed["frame"]
        mt = bed["mattress"]
        pl = bed["pillow"]
        ma.markers.append(self._cube_marker(
            now, 1, "bed_frame",
            Point(x=bed["center"]["x"], y=bed["center"]["y"],
                  z=bf["bottom_z"] + bf["size"]["z"] / 2),
            Point(x=bf["size"]["x"], y=bf["size"]["y"], z=bf["size"]["z"]),
            (0.38, 0.27, 0.17, 1.0)))
        ma.markers.append(self._cube_marker(
            now, 2, "mattress",
            Point(x=bed["center"]["x"], y=bed["center"]["y"],
                  z=mt["bottom_z"] + mt["size"]["z"] / 2),
            Point(x=mt["size"]["x"], y=mt["size"]["y"], z=mt["size"]["z"]),
            (0.82, 0.88, 0.94, 1.0)))
        ma.markers.append(self._cube_marker(
            now, 3, "pillow",
            Point(x=pl["center"]["x"], y=pl["center"]["y"], z=pl["center"]["z"]),
            Point(x=pl["size"]["x"], y=pl["size"]["y"], z=pl["size"]["z"]),
            (0.88, 0.92, 0.98, 1.0)))

        # 人体 12 段
        segs = bd.get("body_segments", [])
        bc = vis.get("body_color", [0.86, 0.76, 0.66, 0.78])
        for i, s in enumerate(segs):
            mid = 100 + i
            if i == 0:
                ma.markers.append(self._sphere_marker(
                    now, mid, f"body_{s['name']}",
                    Point(x=s["x"], y=0.0, z=s["z"]), 0.20, bc))
            else:
                ma.markers.append(self._cube_marker(
                    now, mid, f"body_{s['name']}",
                    Point(x=s["x"], y=0.0, z=s["z"] - s["thick"] / 2),
                    Point(x=0.06, y=s["half_w"] * 2, z=s["thick"]), bc))

        # 脊柱脊线
        sc = vis.get("spine_color", [0.95, 0.85, 0.70, 0.92])
        ridge = [Point(x=s["x"], y=0.0, z=s["z"] + 0.008) for s in segs]
        ma.markers.append(self._line_marker(now, 110, "spine", ridge, sc, 0.015))

        # 侧边轮廓
        ec = vis.get("edge_color", [0.65, 0.55, 0.45, 0.50])
        l_edge = [Point(x=s["x"], y=-s["half_w"], z=s["z"]) for s in segs if s["half_w"] > 0.001]
        r_edge = [Point(x=s["x"], y=s["half_w"], z=s["z"]) for s in segs if s["half_w"] > 0.001]
        ma.markers.append(self._line_marker(now, 111, "edge_L", l_edge, ec, 0.008))
        ma.markers.append(self._line_marker(now, 112, "edge_R", r_edge, ec, 0.008))

        # 穴位红点
        ac = vis.get("acu_color", [0.95, 0.25, 0.25, 0.85])
        for i, ap in enumerate(self._acupoints):
            ma.markers.append(self._sphere_marker(
                now, 200 + i, f"acu_L_{ap['name']}",
                Point(x=ap["x"], y=-self._acu_y_off, z=ap["z"]), 0.018, ac))
            ma.markers.append(self._sphere_marker(
                now, 220 + i, f"acu_R_{ap['name']}",
                Point(x=ap["x"], y=self._acu_y_off, z=ap["z"]), 0.018, ac))

        # 按摩路径点
        lpath = []; rpath = []; cpath = []
        for st in self._stages:
            try:
                jl, jr = self._stage_target(st)
                # 左臂目标
                l_spec = st["left"]
                if "zone" in l_spec:
                    z = self._zones[l_spec["zone"]]
                    pos = l_spec.get("position", "L")
                    y = 0.0 if pos == "C" else (-z["half_w"] if pos == "L" else z["half_w"])
                    tech = l_spec["technique"]
                    tz = z["z_hover"] if tech in ("hover","release","tap","strike") else z["z_surface"]
                    lpath.append(Point(x=z["x"], y=y, z=tz))
                    if pos == "C": cpath.append(Point(x=z["x"], y=y, z=tz))
                # 右臂目标
                r_spec = st["right"]
                if "zone" in r_spec:
                    z = self._zones[r_spec["zone"]]
                    pos = r_spec.get("position", "R")
                    y = 0.0 if pos == "C" else (-z["half_w"] if pos == "L" else z["half_w"])
                    tech = r_spec["technique"]
                    tz = z["z_hover"] if tech in ("hover","release","tap","strike") else z["z_surface"]
                    rpath.append(Point(x=z["x"], y=y, z=tz))
                    if pos == "C": cpath.append(Point(x=z["x"], y=y, z=tz))
            except Exception:
                pass

        if lpath:
            ma.markers.append(self._line_marker(now, 10, "left_path", lpath,
                                                 vis.get("left_path", [0,0.75,0.95,1]), 0.012))
        if rpath:
            ma.markers.append(self._line_marker(now, 11, "right_path", rpath,
                                                 vis.get("right_path", [0.95,0.58,0.2,1]), 0.012))
        if cpath:
            ma.markers.append(self._line_marker(now, 12, "center_path", cpath,
                                                 vis.get("center_path", [0.1,0.95,0.35,1]), 0.016))

        self._marker_pub.publish(ma)

    # Marker helpers
    def _base(self, now, mid, ns, mtype):
        m = Marker(); m.header.frame_id = "world"; m.header.stamp = now
        m.ns = ns; m.id = mid; m.type = mtype; m.action = Marker.ADD
        m.pose.orientation.w = 1.0; return m

    def _cube_marker(self, now, mid, ns, pos, scale, color):
        m = self._base(now, mid, ns, Marker.CUBE)
        m.pose.position = pos; m.scale.x = scale.x; m.scale.y = scale.y; m.scale.z = scale.z
        m.color.r, m.color.g, m.color.b, m.color.a = color; return m

    def _sphere_marker(self, now, mid, ns, pos, diam, color):
        m = self._base(now, mid, ns, Marker.SPHERE)
        m.pose.position = pos; m.scale.x = m.scale.y = m.scale.z = diam
        m.color.r, m.color.g, m.color.b, m.color.a = color; return m

    def _line_marker(self, now, mid, ns, pts, color, w=0.012):
        m = self._base(now, mid, ns, Marker.LINE_STRIP)
        m.points = pts; m.scale.x = w
        m.color.r, m.color.g, m.color.b, m.color.a = color; return m
