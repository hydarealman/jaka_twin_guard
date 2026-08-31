#!/usr/bin/env python3
"""Safety Monitor — joint-state watchdog for single-arm operations.

Monitors joint position/velocity limits and joint-state communication freshness.
Safety levels (ISO 10218-1): WARN → SLOW → HALT → ESTOP

Simplified from jaka_dual_arm/control/safety_monitor.py for single arm.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, Callable

from rclpy.node import Node
from sensor_msgs.msg import JointState


class SafetyLevel(Enum):
    OK = auto()
    WARN = auto()
    SLOW = auto()
    HALT = auto()
    ESTOP = auto()


# Proprietary fruit-arm mechanical hard limits (from the URDF). Keep these
# safe defaults even if a configuration file is temporarily unavailable.
_FRUIT_ARM_JOINT_LOWER = [
    -2.094395102, 0.0, -3.141592654,
    -2.879793266, -1.570796327, -3.141592654,
]
_FRUIT_ARM_JOINT_UPPER = [
    2.094395102, 2.530727415, 0.0,
    2.879793266, 1.570796327, 3.141592654,
]


@dataclass
class SafetyLimits:
    """Safety thresholds loaded from YAML."""
    joint_position_lower: list[float] = field(default_factory=lambda: list(_FRUIT_ARM_JOINT_LOWER))
    joint_position_upper: list[float] = field(default_factory=lambda: list(_FRUIT_ARM_JOINT_UPPER))
    joint_position_margin: float = 0.10

    max_joint_velocity: float = 3.14
    velocity_warn_ratio: float = 0.85
    velocity_halt_ratio: float = 0.95
    require_velocity_feedback: bool = True

    joint_state_timeout: float = 0.5
    controller_timeout: float = 2.0

    @classmethod
    def from_yaml(cls, cfg: dict) -> "SafetyLimits":
        """Create from safety_params.yaml dict."""
        jl = cfg.get("joint_limits", {})
        jv = cfg.get("joint_velocity", {})
        tm = cfg.get("timeouts", {})

        return cls(
            joint_position_lower=jl.get("lower", list(_FRUIT_ARM_JOINT_LOWER)),
            joint_position_upper=jl.get("upper", list(_FRUIT_ARM_JOINT_UPPER)),
            joint_position_margin=jl.get("margin", 0.10),
            max_joint_velocity=jv.get("max_velocity", 3.14),
            velocity_warn_ratio=jv.get("warn_scaling", 0.85),
            velocity_halt_ratio=jv.get("halt_scaling", 0.95),
            require_velocity_feedback=jv.get("require_feedback", True),
            joint_state_timeout=tm.get("joint_state", 0.5),
            controller_timeout=tm.get("controller", 2.0),
        )


@dataclass
class SafetyStatus:
    level: SafetyLevel = SafetyLevel.OK
    active_violations: list[str] = field(default_factory=list)
    last_ok_time: float = 0.0


class SafetyMonitor:
    """Safety watchdog — checks all safety conditions each cycle.

    Usage:
        monitor = SafetyMonitor(node, SafetyLimits.from_yaml(safety_cfg))
        monitor.update_joint_state(msg)
        level = monitor.check()
        if level == SafetyLevel.ESTOP:
            ...
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

        self._joint_positions: dict[str, float] = {}
        self._joint_velocities: dict[str, float] = {}
        self._last_joint_state_time: float = time.time()

        self._joint_state_sub = node.create_subscription(
            JointState, "/joint_states", self.update_joint_state, 10
        )

        self._last_warn_time: dict[str, float] = {}
        self._warn_interval: float = 2.0

        self._arm_joints: list[str] = []

    def set_arm_joints(self, joint_names: list[str]):
        """Configure which joints to monitor (arm joints only)."""
        self._arm_joints = list(joint_names)

    def update_joint_state(self, msg: JointState):
        self._last_joint_state_time = time.time()
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_positions[name] = msg.position[i]
            else:
                self._joint_positions.pop(name, None)
            if i < len(msg.velocity):
                self._joint_velocities[name] = msg.velocity[i]
            else:
                # Missing telemetry is not equivalent to a stopped motor.
                self._joint_velocities.pop(name, None)

    def check(self) -> SafetyLevel:
        violations = []
        level = SafetyLevel.OK

        # 1. Communication timeout → ESTOP
        comm = self._check_communication()
        if comm:
            violations.extend(comm)
            level = SafetyLevel.ESTOP

        # 2. Joint position → HALT
        pos = self._check_joint_positions()
        if pos:
            violations.extend(pos)
            if level.value < SafetyLevel.HALT.value:
                level = SafetyLevel.HALT

        # 3. Joint velocity → SLOW or HALT
        vel, vel_sev = self._check_joint_velocities()
        if vel:
            violations.extend(vel)
            level = max(level, vel_sev, key=lambda s: s.value)

        if level == SafetyLevel.OK:
            self._status.last_ok_time = time.time()
        self._status.level = level
        self._status.active_violations = violations

        if level.value >= SafetyLevel.HALT.value:
            for v in violations:
                self._logger.error(f"[{level.name}] {v}")

        if level == SafetyLevel.ESTOP and self._on_estop:
            self._on_estop(violations)
        elif level.value >= SafetyLevel.WARN.value and self._on_warn:
            self._on_warn(level, violations)

        return level

    def check_trajectory(self, joint_names: list[str],
                         positions: list[list[float]]) -> bool:
        for pt_positions in positions:
            for name, pos in zip(joint_names, pt_positions):
                idx = self._get_joint_index(name)
                if idx is not None and idx < len(self._limits.joint_position_lower):
                    lower = self._limits.joint_position_lower[idx] + self._limits.joint_position_margin
                    upper = self._limits.joint_position_upper[idx] - self._limits.joint_position_margin
                    if pos < lower or pos > upper:
                        self._logger.error(
                            f"Trajectory safety: {name} pos={pos:.3f} "
                            f"outside [{lower:.2f}, {upper:.2f}]"
                        )
                        return False
        return True

    def _check_communication(self) -> list[str]:
        violations = []
        now = time.time()
        if now - self._last_joint_state_time > self._limits.joint_state_timeout:
            violations.append(
                f"Joint state timeout: {now - self._last_joint_state_time:.1f}s"
            )
        return violations

    def _check_joint_positions(self) -> list[str]:
        violations = []
        for name in self._arm_joints:
            if name not in self._joint_positions:
                violations.append(f"{name} position feedback unavailable")
        for name, pos in self._joint_positions.items():
            idx = self._get_joint_index(name)
            if idx is None:
                continue
            lower = self._limits.joint_position_lower[idx] + self._limits.joint_position_margin
            upper = self._limits.joint_position_upper[idx] - self._limits.joint_position_margin
            if pos < lower:
                violations.append(f"{name}: {pos:.3f} < {lower:.3f}")
            elif pos > upper:
                violations.append(f"{name}: {pos:.3f} > {upper:.3f}")
        return violations

    def _check_joint_velocities(self) -> tuple[list[str], SafetyLevel]:
        violations = []
        severity = SafetyLevel.OK
        if self._limits.require_velocity_feedback:
            missing = [
                name for name in self._arm_joints
                if name not in self._joint_velocities
            ]
            if missing:
                return (
                    [f"Joint velocity feedback unavailable: {', '.join(missing)}"],
                    SafetyLevel.HALT,
                )
        for name, vel in self._joint_velocities.items():
            abs_vel = abs(vel)
            max_v = self._limits.max_joint_velocity
            if abs_vel > max_v * self._limits.velocity_halt_ratio:
                violations.append(f"{name} velocity CRITICAL: {abs_vel:.2f} rad/s")
                severity = SafetyLevel.HALT
            elif abs_vel > max_v * self._limits.velocity_warn_ratio:
                violations.append(f"{name} velocity WARN: {abs_vel:.2f} rad/s")
                if severity.value < SafetyLevel.SLOW.value:
                    severity = SafetyLevel.SLOW
        return violations, severity

    def _warn(self, level_str: str, msg: str):
        now = time.time()
        key = msg[:40]
        last = self._last_warn_time.get(key, 0.0)
        if now - last > self._warn_interval:
            self._logger.warn(f"[{level_str}] {msg}")
            self._last_warn_time[key] = now

    def _get_joint_index(self, name: str) -> Optional[int]:
        # Single arm: joint names like "joint_1" .. "joint_6"
        if name.startswith("joint_"):
            try:
                return int(name.split("_")[1]) - 1
            except (ValueError, IndexError):
                pass
        return None

    @property
    def status(self) -> SafetyStatus:
        return self._status

    @property
    def is_safe(self) -> bool:
        return self._status.level.value < SafetyLevel.HALT.value
