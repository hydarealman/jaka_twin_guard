#!/usr/bin/env python3
"""Virtual Impedance Controller — Python 虚拟阻抗，用于仿真模式。

在无真实 F/T 传感器的情况下，模拟柔顺控制行为：
- 给定目标位姿 + 虚拟外力 → 计算柔顺位姿
- 模拟弹簧-阻尼系统的动力学响应
- 支持按摩 Demo 的力控手法参数

数学模型（质量-弹簧-阻尼系统）:
    M*x'' + D*(x' - x'_des) + K*(x - x_des) = F_ext + F_virtual

简化（忽略惯性，准静态阻抗）:
    Δx = F_virtual / K    (稳态位移)
    Δv = F_virtual / D    (稳态速度)

用于仿真:
    x_compliant(t) = x_des(t) + K^(-1) * F_virtual
    其中 K^(-1) 是 6x6 对角柔顺矩阵

参考:
  - franka_ros2 franka_example_controllers — 笛卡尔阻抗控制示例
  - UR ROS2 Driver cartesian_impedance_controller
  - cartesian_controllers (ros-controls/ros2_controllers)
"""

from __future__ import annotations

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Wrench

from jaka_dual_arm.control.force_control_interface import (
    ForceControlInterface,
    ForceControlMode,
    ForceControlState,
    ImpedanceParams,
)


def _euler_from_quaternion(q: Quaternion):
    """四元数 → 欧拉角 (roll, pitch, yaw)。"""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """欧拉角 → 四元数。"""
    q = Quaternion()
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class VirtualImpedanceController(ForceControlInterface):
    """虚拟阻抗控制器 — 仿真模式下的柔顺控制。

    在有真实 F/T 传感器前，用一个外部设置的"虚拟力"来模拟接触力，
    计算柔顺位姿。数据流与真机完全相同，切换只需换控制器实现。

    用法:
        vic = VirtualImpedanceController(node)
        vic.set_mode(ForceControlMode.IMPEDANCE)
        vic.set_impedance(ImpedanceParams.massage_preset("medium"))
        vic.set_reference_pose(desired_pose_on_body)
        vic.set_virtual_force(force_z=15.0)  # 模拟 15N 按压力

        while True:
            dt = 0.01  # 100Hz
            compliant = vic.step(dt)
            if compliant:
                send_pose_to_moveit(compliant)
    """

    def __init__(self, node: Node):
        super().__init__()
        self._node = node
        self._logger = node.get_logger()

        # 虚拟外力（仿真用）
        self._virtual_force_z: float = 0.0
        self._virtual_force_x: float = 0.0
        self._virtual_force_y: float = 0.0

        # 时间积分状态
        self._last_time: Optional[float] = None
        self._current_velocity = Vector3()
        self._compliant_offset = Vector3()  # 累计柔顺偏移

        # 振动参数
        self._vibration_enabled: bool = False
        self._vibration_frequency: float = 8.0   # Hz
        self._vibration_amplitude: float = 0.002  # m

        self._logger.info("VirtualImpedanceController initialized (simulation mode)")

    # ── 仿真接口 ──────────────────────────────────────────

    def set_virtual_force(self, force_z: float = 0.0,
                          force_x: float = 0.0, force_y: float = 0.0):
        """设置虚拟外力（仿真 F/T 传感器输入）。正Z=向下按压。"""
        self._virtual_force_x = force_x
        self._virtual_force_y = force_y
        self._virtual_force_z = force_z

    def set_vibration(self, enabled: bool, frequency: float = 8.0,
                      amplitude: float = 0.002):
        """控制振动效果（振法手法）。"""
        self._vibration_enabled = enabled
        self._vibration_frequency = frequency
        self._vibration_amplitude = amplitude

    # ── 核心计算 ──────────────────────────────────────────

    def compute_compliant_pose(self, dt: float) -> Optional[Pose]:
        """计算柔顺位姿（每周期调用一次）。

        基于虚拟外力 + 阻抗参数，计算柔顺偏移后的目标位姿。
        """
        ref = self._state.reference_pose
        if ref is None:
            return None

        K = self._state.impedance

        # ── 平移柔顺 ──
        # Δx = F / K (稳态阻抗关系)
        # Z轴柔顺: 被按下的目标向下移位
        kz = K.translational_stiffness[2]  # Z 刚度
        kx = K.translational_stiffness[0]  # X 刚度
        ky = K.translational_stiffness[1]  # Y 刚度

        # 防除零
        if kz < 1.0:
            kz = 1.0
        if kx < 1.0:
            kx = 1.0
        if ky < 1.0:
            ky = 1.0

        # 目标柔顺偏移（一阶低通滤波平滑）
        alpha = min(1.0, dt * 20.0)  # 20Hz 截止频率
        target_dz = self._virtual_force_z / kz
        target_dx = self._virtual_force_x / kx
        target_dy = self._virtual_force_y / ky

        self._compliant_offset.z += alpha * (target_dz - self._compliant_offset.z)
        self._compliant_offset.x += alpha * (target_dx - self._compliant_offset.x)
        self._compliant_offset.y += alpha * (target_dy - self._compliant_offset.y)

        # ── 振动叠加 ──
        vib_z = 0.0
        if self._vibration_enabled:
            t = time.time()
            vib_z = self._vibration_amplitude * math.sin(
                2.0 * math.pi * self._vibration_frequency * t
            )

        # ── 限幅 ──
        max_dz = K.max_force[2] / kz
        max_dx = K.max_force[0] / kx
        max_dy = K.max_force[1] / ky
        dz = max(-max_dz, min(max_dz, self._compliant_offset.z)) + vib_z
        dx = max(-max_dx, min(max_dx, self._compliant_offset.x))
        dy = max(-max_dy, min(max_dy, self._compliant_offset.y))

        # ── 构建柔顺位姿 ──
        compliant = Pose()
        compliant.position.x = ref.position.x + dx
        compliant.position.y = ref.position.y + dy
        compliant.position.z = ref.position.z + dz
        # 姿态保持不变（简化为平移柔顺，旋转柔顺在需要时扩展）
        compliant.orientation = ref.orientation

        self._state.actual_pose = compliant
        return compliant

    def reset(self):
        """重置柔顺偏移。"""
        self._compliant_offset = Vector3()
        self._virtual_force_z = 0.0
        self._virtual_force_x = 0.0
        self._virtual_force_y = 0.0
        self._vibration_enabled = False
