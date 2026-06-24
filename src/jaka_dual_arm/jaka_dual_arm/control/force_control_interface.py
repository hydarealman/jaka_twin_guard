#!/usr/bin/env python3
"""Force Control Interface — 力/位混合控制抽象层。

定义工业机械臂标准力控接口，支持：
- 阻抗控制 (Impedance Control): F_ext = K*(x_des - x) + D*(v_des - v)
- 导纳控制 (Admittance Control): x_des = admittance(F_ext)
- 力控制 (Direct Force Control): F → desired force

当前实现：
- 仿真模式: VirtualImpedance (Python 虚拟阻抗)
- 真机模式: CartesianImpedanceController (ros2_controllers 原生)
- 按摩 Demo 模式: 从 YAML 加载手法力度参数

参考:
  - franka_ros2 cartesian_impedance_controller — 1kHz 实时阻抗控制
  - UR ROS2 Driver force_mode_controller — 工业力控标准接口
  - cartesian_controllers (ros-controls/ros2_controllers) — 笛卡尔阻抗控制器
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench


# ── 数据类型 ──────────────────────────────────────────────────


class ForceControlMode(Enum):
    """力控模式。"""
    POSITION = auto()           # 纯位置控制
    IMPEDANCE = auto()          # 阻抗控制 (F=K·Δx+D·Δv)
    ADMITTANCE = auto()         # 导纳控制 (Δx=Y·F)
    FORCE = auto()              # 直接力控制 (F→desired)
    HYBRID = auto()             # 力/位混合 (某些轴力控，某些轴位控)


@dataclass
class ImpedanceParams:
    """阻抗控制参数。

    阻抗定律: F = K*(x_des - x) + D*(v_des - v)
    其中 K = 刚度矩阵 (6x6 对角线), D = 阻尼矩阵 (6x6 对角线)

    刚度/阻尼选择原则:
    - 高刚度 (≥1000 N/m): 精密操作 (抓取)
    - 中刚度 (200-1000 N/m): 一般交互
    - 低刚度 (50-200 N/m): 安全交互 (按摩、人机协作)

    阻尼: 临界阻尼 D = 2*sqrt(K*m) 或过阻尼 D = 3*sqrt(K*m)
    """
    # 平移刚度 (N/m) — [x, y, z]
    translational_stiffness: list[float] = field(default_factory=lambda: [500.0, 500.0, 500.0])
    # 旋转刚度 (Nm/rad) — [roll, pitch, yaw]
    rotational_stiffness: list[float] = field(default_factory=lambda: [30.0, 30.0, 30.0])
    # 平移阻尼 (Ns/m)
    translational_damping: list[float] = field(default_factory=lambda: [50.0, 50.0, 50.0])
    # 旋转阻尼 (Nms/rad)
    rotational_damping: list[float] = field(default_factory=lambda: [5.0, 5.0, 5.0])

    # 力/力矩限幅
    max_force: list[float] = field(default_factory=lambda: [50.0, 50.0, 50.0])      # N
    max_torque: list[float] = field(default_factory=lambda: [10.0, 10.0, 10.0])     # Nm

    @classmethod
    def from_yaml(cls, cfg: dict) -> "ImpedanceParams":
        """从 YAML 配置创建参数。"""
        imp = cfg.get("impedance", {})
        return cls(
            translational_stiffness=imp.get("translational_stiffness", [500.0, 500.0, 500.0]),
            rotational_stiffness=imp.get("rotational_stiffness", [30.0, 30.0, 30.0]),
            translational_damping=imp.get("translational_damping", [50.0, 50.0, 50.0]),
            rotational_damping=imp.get("rotational_damping", [5.0, 5.0, 5.0]),
            max_force=imp.get("max_force", [50.0, 50.0, 50.0]),
            max_torque=imp.get("max_torque", [10.0, 10.0, 10.0]),
        )

    @classmethod
    def massage_preset(cls, intensity: str = "medium") -> "ImpedanceParams":
        """按摩手法力度预设。

        Args:
            intensity: "light"(轻) | "medium"(中) | "deep"(深)
        """
        presets = {
            "light": cls(
                translational_stiffness=[200.0, 200.0, 100.0],
                rotational_stiffness=[15.0, 15.0, 15.0],
                translational_damping=[20.0, 20.0, 10.0],
                rotational_damping=[3.0, 3.0, 3.0],
                max_force=[10.0, 10.0, 5.0],
            ),
            "medium": cls(
                translational_stiffness=[500.0, 500.0, 300.0],
                rotational_stiffness=[30.0, 30.0, 30.0],
                translational_damping=[50.0, 50.0, 30.0],
                rotational_damping=[5.0, 5.0, 5.0],
                max_force=[30.0, 30.0, 15.0],
            ),
            "deep": cls(
                translational_stiffness=[800.0, 800.0, 500.0],
                rotational_stiffness=[50.0, 50.0, 50.0],
                translational_damping=[80.0, 80.0, 50.0],
                rotational_damping=[8.0, 8.0, 8.0],
                max_force=[50.0, 50.0, 25.0],
            ),
        }
        return presets.get(intensity, presets["medium"])


@dataclass
class ForceControlState:
    """力控状态快照。"""
    mode: ForceControlMode = ForceControlMode.POSITION
    # 参考位姿 (目标)
    reference_pose: Optional[Pose] = None
    # 实际位姿 (从 TF 读取)
    actual_pose: Optional[Pose] = None
    # 期望力/力矩 (wrench)
    desired_wrench: Optional[Wrench] = None
    # 实测力/力矩 (从 F/T 传感器)
    measured_wrench: Optional[Wrench] = None
    # 阻抗参数
    impedance: ImpedanceParams = field(default_factory=ImpedanceParams)
    # 是否处于接触状态
    in_contact: bool = False
    # 力误差 (N)
    force_error: float = 0.0


# ── 抽象接口 ──────────────────────────────────────────────────


class ForceControlInterface(ABC):
    """力/位混合控制抽象基类。

    子类实现:
    - VirtualImpedanceController   (仿真: Python 虚拟阻抗)
    - RealImpedanceController       (真机: ros2_controllers CartesianImpedanceController)

    用法:
        fci = VirtualImpedanceController(node, tf_buffer)
        fci.set_mode(ForceControlMode.IMPEDANCE)
        fci.set_impedance(ImpedanceParams.massage_preset("medium"))
        fci.set_reference_pose(target_pose)
        compliant_pose = fci.step(dt)  # 每周期计算柔顺位姿
    """

    def __init__(self):
        self._mode = ForceControlMode.POSITION
        self._state = ForceControlState()

    @property
    def mode(self) -> ForceControlMode:
        return self._mode

    @property
    def state(self) -> ForceControlState:
        return self._state

    def set_mode(self, mode: ForceControlMode):
        """切换力控模式。"""
        self._mode = mode

    def set_impedance(self, params: ImpedanceParams):
        """设置阻抗参数。"""
        self._state.impedance = params

    def set_reference_pose(self, pose: Pose):
        """设置参考位姿（位置控制目标）。"""
        self._state.reference_pose = pose

    def set_desired_wrench(self, wrench: Wrench):
        """设置期望力/力矩。"""
        self._state.desired_wrench = wrench

    def update_measured_wrench(self, wrench: Wrench):
        """更新实测力/力矩（从 F/T 传感器回调）。"""
        self._state.measured_wrench = wrench
        self._update_contact_state()

    def update_actual_pose(self, pose: Pose):
        """更新实际末端位姿（从 TF 回调）。"""
        self._state.actual_pose = pose

    @abstractmethod
    def compute_compliant_pose(self, dt: float) -> Optional[Pose]:
        """计算下一周期的柔顺位姿。

        Args:
            dt: 时间步长 (秒)，通常 = 1/control_frequency

        Returns:
            柔顺后的目标位姿，或 None (保持当前位置)
        """
        ...

    def _update_contact_state(self):
        """根据实测力更新接触状态。"""
        wrench = self._state.measured_wrench
        if wrench is None:
            self._state.in_contact = False
            return
        f_mag = math.sqrt(
            wrench.force.x ** 2 + wrench.force.y ** 2 + wrench.force.z ** 2
        )
        self._state.in_contact = f_mag > 1.0  # 1N 阈值
        self._state.force_error = (
            self._compute_force_error() if self._state.desired_wrench else 0.0
        )

    def _compute_force_error(self) -> float:
        """计算力误差 (N)。"""
        desired = self._state.desired_wrench
        measured = self._state.measured_wrench
        if desired is None or measured is None:
            return 0.0
        df = Vector3()
        df.x = desired.force.x - measured.force.x
        df.y = desired.force.y - measured.force.y
        df.z = desired.force.z - measured.force.z
        return math.sqrt(df.x ** 2 + df.y ** 2 + df.z ** 2)


# ── 按摩手法力度映射 ──────────────────────────────────────────


# 13 种中医推拿手法的力控参数
# 力度分级: 轻(1-5N) / 中(5-15N) / 深(15-30N)
TECHNIQUE_FORCE_MAP = {
    "hover":    {"force_z": 0.0,  "impedance": "light"},
    "press":    {"force_z": 8.0,  "impedance": "medium"},
    "deep_press": {"force_z": 20.0, "impedance": "deep"},
    "knead_L":  {"force_z": 6.0,  "impedance": "medium", "lateral_force": 3.0},
    "knead_R":  {"force_z": 6.0,  "impedance": "medium", "lateral_force": 3.0},
    "rub_L":    {"force_z": 4.0,  "impedance": "light",  "lateral_force": 2.0},
    "rub_R":    {"force_z": 4.0,  "impedance": "light",  "lateral_force": 2.0},
    "scrub":    {"force_z": 10.0, "impedance": "medium", "lateral_force": 5.0},
    "vibrate":  {"force_z": 3.0,  "impedance": "light",  "frequency": 8.0},  # Hz
    "roll":     {"force_z": 8.0,  "impedance": "medium", "lateral_force": 4.0},
    "tap":      {"force_z": 10.0, "impedance": "medium"},
    "strike":   {"force_z": 5.0,  "impedance": "light",  "frequency": 4.0},
    "release":  {"force_z": 0.0,  "impedance": "light"},
}


def get_technique_params(technique: str) -> dict:
    """获取指定手法的力控参数。"""
    return TECHNIQUE_FORCE_MAP.get(technique, TECHNIQUE_FORCE_MAP["press"])


def wrench_from_technique(technique: str) -> Wrench:
    """从手法名称生成期望 Wrench。"""
    params = get_technique_params(technique)
    w = Wrench()
    w.force.z = -params["force_z"]  # 向下为正
    w.force.x = params.get("lateral_force", 0.0)
    w.force.y = params.get("lateral_force", 0.0)
    return w
