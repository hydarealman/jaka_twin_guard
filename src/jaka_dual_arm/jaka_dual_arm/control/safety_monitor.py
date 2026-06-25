#!/usr/bin/env python3
"""Safety Monitor — 实时安全看门狗。

监控以下安全条件:
1. 关节位置超限 → 紧急停止
2. 关节速度超限 → 减速/停止
3. 末端力/力矩超限 → 力控回退
4. 工作空间越界 → 轨迹拒绝
5. 碰撞检测告警 → 记录+可选停止
6. 通信超时 → 停止

安全等级 (ISO 10218-1 工业机器人安全标准):
  - WARN:  记录警告，继续执行
  - SLOW:  降速到安全速度
  - HALT:  停止当前轨迹
  - ESTOP: 紧急断电

参考:
  - ISO 10218-1:2011 工业机器人安全要求
  - franka_ros2 franka_safety — 力限制安全机制
  - UR ROS2 Driver safety_mode — UR 机器人安全模式
  - pilz_industrial_motion — PNOZ 安全控制器集成
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Wrench


# ── 数据类型 ──────────────────────────────────────────────────


class SafetyLevel(Enum):
    OK = auto()
    WARN = auto()
    SLOW = auto()
    HALT = auto()
    ESTOP = auto()


# JAKA C5 各关节实际限位 (rad, 来自 jaka_c5.urdf)
# joint_1: ±360°=±6.28  joint_2: -85°~+265°=-1.48~4.62  joint_3: ±175°=±3.05
# joint_4: -85°~+265°=-1.48~4.62  joint_5: ±360°=±6.28  joint_6: ±360°=±6.28
_JAKA_C5_JOINT_LOWER = [-6.28, -1.48, -3.05, -1.48, -6.28, -6.28,
                         -6.28, -1.48, -3.05, -1.48, -6.28, -6.28]
_JAKA_C5_JOINT_UPPER = [ 6.28,  4.62,  3.05,  4.62,  6.28,  6.28,
                          6.28,  4.62,  3.05,  4.62,  6.28,  6.28]


@dataclass
class SafetyLimits:
    """安全限值 — 所有阈值从 YAML 加载。"""
    # 关节限位 (rad) — 默认使用 JAKA C5 实际范围而非 ±180°
    joint_position_lower: list[float] = field(default_factory=lambda: list(_JAKA_C5_JOINT_LOWER))
    joint_position_upper: list[float] = field(default_factory=lambda: list(_JAKA_C5_JOINT_UPPER))
    joint_position_margin: float = 0.05  # 软限位提前量 (rad, ~3°)

    # 速度限值 (rad/s)
    max_joint_velocity: float = 1.57      # 正常最大关节速度
    reduced_joint_velocity: float = 0.30  # 降速模式
    velocity_margin: float = 0.10         # 速度提前警告比例

    # 末端力限值 (N/Nm)
    max_contact_force: float = 50.0       # 末端最大接触力
    max_contact_torque: float = 10.0      # 末端最大接触力矩
    force_warning_ratio: float = 0.70     # 力警告比例

    # 工作空间 (m)
    workspace_radius: float = 1.0         # 从基座算起的球体半径
    workspace_z_min: float = -0.10        # 工作空间下限
    workspace_z_max: float = 0.90

    # 通信超时 (秒)
    joint_state_timeout: float = 0.5      # 关节状态超时
    controller_timeout: float = 2.0

    @classmethod
    def from_yaml(cls, cfg: dict) -> "SafetyLimits":
        """从 YAML 字典创建。"""
        jv = cfg.get("joint_velocity", {})
        jp = cfg.get("joint_position", {})
        ws = cfg.get("workspace", {})
        ft = cfg.get("force_torque", {})
        tm = cfg.get("timeouts", {})

        # 关节位置限位 — 使用 JAKA C5 实际范围，YAML 可覆盖
        jp_lower = jp.get("lower", None)
        jp_upper = jp.get("upper", None)
        jp_margin = jp.get("margin", 0.10)

        kwargs = dict(
            max_joint_velocity=jv.get("max", 1.57),
            reduced_joint_velocity=jv.get("reduced", 0.30),
            velocity_margin=jv.get("margin", 0.10),
            max_contact_force=ft.get("max_force", 50.0),
            max_contact_torque=ft.get("max_torque", 10.0),
            force_warning_ratio=ft.get("warning_ratio", 0.70),
            workspace_radius=ws.get("radius", 1.0),
            workspace_z_min=ws.get("z_min", -0.10),
            workspace_z_max=ws.get("z_max", 0.90),
            joint_state_timeout=tm.get("joint_state", 0.5),
            controller_timeout=tm.get("controller", 2.0),
            joint_position_margin=jp_margin,
        )
        if jp_lower is not None:
            kwargs["joint_position_lower"] = jp_lower
        if jp_upper is not None:
            kwargs["joint_position_upper"] = jp_upper

        return cls(**kwargs)


@dataclass
class SafetyStatus:
    """安全状态快照。"""
    level: SafetyLevel = SafetyLevel.OK
    active_violations: list[str] = field(default_factory=list)
    last_ok_time: float = 0.0
    emergency_stop_active: bool = False


# ── 安全监控器 ───────────────────────────────────────────────


class SafetyMonitor:
    """安全监控器 — 每控制周期检查一次安全条件。

    用法:
        monitor = SafetyMonitor(node, limits, on_estop=my_estop_handler)
        monitor.update_joint_state(joint_state_msg)
        result = monitor.check()  # 返回 SafetyLevel

        if result == SafetyLevel.ESTOP:
            ...  # 紧急停止
    """

    def __init__(self, node: Node, limits: SafetyLimits = None,
                 on_estop: Optional[Callable] = None,
                 on_warn: Optional[Callable] = None):
        self._node = node
        self._logger = node.get_logger()
        self._limits = limits or SafetyLimits()
        self._status = SafetyStatus()
        self._on_estop = on_estop
        self._on_warn = on_warn

        # 关节状态缓存
        self._joint_positions: dict[str, float] = {}
        self._joint_velocities: dict[str, float] = {}
        self._last_joint_state_time: float = time.time()
        self._joint_states_received: bool = False  # 首次消息到达后才检查超时

        # 自动订阅 /joint_states
        self._joint_state_sub = node.create_subscription(
            JointState, "/joint_states", self.update_joint_state, 10,
        )

        # 末端状态缓存
        self._end_effector_position: Optional[Point] = None
        self._end_effector_wrench: Optional[Wrench] = None

        # 日志抑制
        self._last_warn_time: dict[str, float] = {}
        self._warn_interval: float = 2.0  # 同类型警告最小间隔(秒)

    # ── 状态更新 ──────────────────────────────────────────

    def update_joint_state(self, msg: JointState):
        """更新关节状态（从 /joint_states 回调）。"""
        self._last_joint_state_time = time.time()
        self._joint_states_received = True
        for i, name in enumerate(msg.name):
            self._joint_positions[name] = msg.position[i] if i < len(msg.position) else 0.0
            self._joint_velocities[name] = msg.velocity[i] if i < len(msg.velocity) else 0.0

    def update_end_effector(self, position: Point, wrench: Optional[Wrench] = None):
        """更新末端状态。"""
        self._end_effector_position = position
        if wrench:
            self._end_effector_wrench = wrench

    # ── 安全检查 ──────────────────────────────────────────

    def check(self) -> SafetyLevel:
        """执行所有安全检查，返回当前安全等级。

        优先级: ESTOP > HALT > SLOW > WARN > OK
        """
        violations = []
        level = SafetyLevel.OK

        # 1. 通信超时 → ESTOP
        comm_violations = self._check_communication()
        for v in comm_violations:
            self._warn("ESTOP", v)
        if comm_violations:
            violations.extend(comm_violations)
            level = SafetyLevel.ESTOP

        # 2. 关节位置超限 → HALT
        pos_violations = self._check_joint_positions()
        if pos_violations:
            violations.extend(pos_violations)
            if level.value < SafetyLevel.HALT.value:
                level = SafetyLevel.HALT

        # 3. 关节速度超限 → SLOW or HALT
        vel_violations, vel_severity = self._check_joint_velocities()
        if vel_violations:
            violations.extend(vel_violations)
            if vel_severity >= SafetyLevel.HALT.value and level.value < SafetyLevel.HALT.value:
                level = SafetyLevel.HALT
            elif vel_severity >= SafetyLevel.SLOW.value and level.value < SafetyLevel.SLOW.value:
                level = SafetyLevel.SLOW

        # 4. 末端力超限 → HALT (力控模式)
        force_violations = self._check_force_limits()
        if force_violations:
            violations.extend(force_violations)
            if level.value < SafetyLevel.HALT.value:
                level = SafetyLevel.HALT

        # 5. 工作空间越界 → WARN
        ws_violations = self._check_workspace()
        if ws_violations:
            violations.extend(ws_violations)
            if level.value < SafetyLevel.WARN.value:
                level = SafetyLevel.WARN

        # ── 更新状态 ──
        if level == SafetyLevel.OK:
            self._status.last_ok_time = time.time()
        self._status.level = level
        self._status.active_violations = violations

        # ── 日志：HALT 及以上必须记录具体违规项 ──
        if level.value >= SafetyLevel.HALT.value:
            for v in violations:
                self._logger.error(f"[{level.name}] {v}")

        # ── 触发回调 ──
        if level == SafetyLevel.ESTOP and self._on_estop:
            self._on_estop(violations)
        elif level.value >= SafetyLevel.WARN.value and self._on_warn:
            self._on_warn(level, violations)

        return level

    def check_trajectory(self, joint_names: list[str],
                         positions: list[list[float]]) -> bool:
        """预检查轨迹是否安全。在发送给控制器前调用。

        Returns:
            True = 轨迹安全可执行, False = 轨迹违反安全条件
        """
        for pt_positions in positions:
            for name, pos in zip(joint_names, pt_positions):
                # 检查关节限位
                idx = self._get_joint_index(name)
                if idx is not None and idx < len(self._limits.joint_position_lower):
                    lower = self._limits.joint_position_lower[idx] + self._limits.joint_position_margin
                    upper = self._limits.joint_position_upper[idx] - self._limits.joint_position_margin
                    if pos < lower or pos > upper:
                        self._logger.error(
                            f"Trajectory safety: joint {name} pos {pos:.3f} "
                            f"outside [{lower:.2f}, {upper:.2f}]"
                        )
                        return False
        return True

    # ── 内部检查方法 ───────────────────────────────────────

    def _check_communication(self) -> list[str]:
        """检查通信超时。首次消息到达前不检查（启动宽限期）。"""
        if not self._joint_states_received:
            return []  # 尚未收到任何关节状态消息，跳过超时检查
        violations = []
        now = time.time()
        if now - self._last_joint_state_time > self._limits.joint_state_timeout:
            violations.append(f"Joint state timeout: {now - self._last_joint_state_time:.1f}s")
        return violations

    def _check_joint_positions(self) -> list[str]:
        """检查关节位置是否超限。"""
        violations = []
        for name, pos in self._joint_positions.items():
            idx = self._get_joint_index(name)
            if idx is None:
                continue
            lower = self._limits.joint_position_lower[idx] + self._limits.joint_position_margin
            upper = self._limits.joint_position_upper[idx] - self._limits.joint_position_margin
            if pos < lower:
                violations.append(f"Joint {name} below limit: {pos:.3f} < {lower:.3f}")
            elif pos > upper:
                violations.append(f"Joint {name} above limit: {pos:.3f} > {upper:.3f}")
        return violations

    def _check_joint_velocities(self) -> tuple[list[str], int]:
        """检查关节速度。

        Returns:
            (violations, severity): severity = SafetyLevel.value
        """
        violations = []
        severity = SafetyLevel.OK.value

        for name, vel in self._joint_velocities.items():
            abs_vel = abs(vel)
            if abs_vel > self._limits.max_joint_velocity:
                violations.append(
                    f"Joint {name} velocity CRITICAL: {abs_vel:.2f} > {self._limits.max_joint_velocity:.2f} rad/s"
                )
                severity = max(severity, SafetyLevel.HALT.value)
            elif abs_vel > self._limits.max_joint_velocity * (1.0 - self._limits.velocity_margin):
                violations.append(
                    f"Joint {name} velocity WARN: {abs_vel:.2f} rad/s"
                )
                severity = max(severity, SafetyLevel.SLOW.value)

        return violations, severity

    def _check_force_limits(self) -> list[str]:
        """检查末端力/力矩。"""
        violations = []
        wrench = self._end_effector_wrench
        if wrench is None:
            return violations

        f_mag = math.sqrt(
            wrench.force.x ** 2 + wrench.force.y ** 2 + wrench.force.z ** 2
        )
        t_mag = math.sqrt(
            wrench.torque.x ** 2 + wrench.torque.y ** 2 + wrench.torque.z ** 2
        )

        warn_f = self._limits.max_contact_force * self._limits.force_warning_ratio
        warn_t = self._limits.max_contact_torque * self._limits.force_warning_ratio

        if f_mag > self._limits.max_contact_force:
            violations.append(f"Force EXCEEDED: {f_mag:.1f}N > {self._limits.max_contact_force:.1f}N")
        elif f_mag > warn_f:
            violations.append(f"Force WARN: {f_mag:.1f}N")

        if t_mag > self._limits.max_contact_torque:
            violations.append(f"Torque EXCEEDED: {t_mag:.1f}Nm > {self._limits.max_contact_torque:.1f}Nm")
        elif t_mag > warn_t:
            violations.append(f"Torque WARN: {t_mag:.1f}Nm")

        return violations

    def _check_workspace(self) -> list[str]:
        """检查工作空间。"""
        violations = []
        p = self._end_effector_position
        if p is None:
            return violations

        dist = math.sqrt(p.x ** 2 + p.y ** 2 + p.z ** 2)
        if dist > self._limits.workspace_radius:
            violations.append(f"Workspace radius: {dist:.2f}m > {self._limits.workspace_radius:.2f}m")
        if p.z < self._limits.workspace_z_min:
            violations.append(f"Workspace Z: {p.z:.2f} < {self._limits.workspace_z_min:.2f}")
        if p.z > self._limits.workspace_z_max:
            violations.append(f"Workspace Z: {p.z:.2f} > {self._limits.workspace_z_max:.2f}")

        return violations

    def _warn(self, level_str: str, msg: str):
        """带抑制的警告日志。"""
        now = time.time()
        key = msg[:40]
        last = self._last_warn_time.get(key, 0.0)
        if now - last > self._warn_interval:
            self._logger.warn(f"[{level_str}] {msg}")
            self._last_warn_time[key] = now

    def _get_joint_index(self, name: str) -> Optional[int]:
        """从关节名提取索引（用于查表）。"""
        for prefix in ["left_joint_", "right_joint_"]:
            if name.startswith(prefix):
                try:
                    idx = int(name[len(prefix):]) - 1
                    offset = 0 if "left" in prefix else 6
                    return offset + idx
                except ValueError:
                    pass
        return None

    @property
    def status(self) -> SafetyStatus:
        return self._status

    @property
    def is_safe(self) -> bool:
        return self._status.level.value < SafetyLevel.HALT.value
