#!/usr/bin/env python3
"""F/T Sensor Interface — 六维力/力矩传感器抽象层。

支持的传感器型号:
    - ATI Net F/T (Ethernet/UDP, 1kHz) — 工业标准
    - Weiss KMS40 (EtherCAT, 1kHz)
    - Robotiq FT-300 (Modbus RTU, 100Hz) — 性价比高
    - 国产 坤维 KWR36 (Ethernet, 1kHz)
    - Mock (仿真模式，零值输出)

数据流:
    Sensor Hardware → Driver (C++) → /ft_sensor/wrench (WrenchStamped)
    → ForceControlInterface.update_measured_wrench()
    → VirtualImpedanceController / CartesianImpedanceController
    → SafetyMonitor (力超限检测)

校准流程:
    1. 零漂采集 (无负载，采集 1000 样本取均值)
    2. 6x6 标定矩阵加载 (厂家提供)
    3. 重力补偿 (已知末端执行器质量 + 位姿)
    4. 温度漂移补偿 (40°C 恒温箱标定)

参考:
  - ATI Net F/T 通信协议 (UDP 49152 端口, RDT 格式)
  - robotiq_ft_sensor ROS2 驱动 (github.com/ros-industrial/robotiq)
  - franka_ros2 franka_hw — F/T 传感器 ros2_control 集成
"""

from __future__ import annotations

import math
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped


# ── 数据类型 ──────────────────────────────────────────────────


class FtSensorModel(Enum):
    """F/T 传感器型号枚举。"""
    MOCK = auto()           # 仿真模式
    ATI_NET_FT = auto()     # ATI Net F/T (Ethernet/UDP)
    WEISS_KMS40 = auto()    # Weiss KMS40 (EtherCAT)
    ROBOTIQ_FT300 = auto()  # Robotiq FT-300 (Modbus RTU)
    KUNWEI_KWR36 = auto()   # 坤维 KWR36 (Ethernet)


@dataclass
class FtSensorSpec:
    """F/T 传感器规格。"""
    model: FtSensorModel = FtSensorModel.MOCK
    force_range: list[float] = field(default_factory=lambda: [500.0, 500.0, 900.0])  # N (Fx,Fy,Fz)
    torque_range: list[float] = field(default_factory=lambda: [30.0, 30.0, 30.0])    # Nm
    resolution_force: float = 0.025   # N
    resolution_torque: float = 0.00125  # Nm
    sample_rate: int = 1000           # Hz
    output_rate: int = 1000           # Hz

    # 厂商标定矩阵 (6x6, 行优先)
    calibration_matrix: list[float] = field(default_factory=lambda: [1.0] * 36)

    # 零漂值 (mV → 牛顿/牛米的偏移)
    zero_offset: list[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class FtSensorReading:
    """F/T 传感器单次读数。"""
    timestamp: float = 0.0
    force: Vector3 = field(default_factory=Vector3)    # N
    torque: Vector3 = field(default_factory=Vector3)    # Nm
    raw_adc: list[float] = field(default_factory=lambda: [0.0] * 6)
    status: int = 0  # 0=OK, 非0=错误码

    @property
    def force_magnitude(self) -> float:
        return math.sqrt(self.force.x**2 + self.force.y**2 + self.force.z**2)

    @property
    def torque_magnitude(self) -> float:
        return math.sqrt(self.torque.x**2 + self.torque.y**2 + self.torque.z**2)

    def to_wrench(self) -> Wrench:
        w = Wrench()
        w.force = self.force
        w.torque = self.torque
        return w

    def to_wrench_stamped(self, frame_id: str = "") -> WrenchStamped:
        ws = WrenchStamped()
        ws.header.frame_id = frame_id
        ws.header.stamp.sec = int(self.timestamp)
        ws.header.stamp.nanosec = int((self.timestamp % 1) * 1e9)
        ws.wrench = self.to_wrench()
        return ws


# ── 抽象接口 ──────────────────────────────────────────────────


class FtSensorInterface(ABC):
    """F/T 传感器抽象基类。

    子类实现:
    - MockFtSensor     (仿真: 全零输出)
    - AtiNetFtSensor   (真机: ATI UDP 通信)
    - RobotiqFt300     (真机: Modbus 通信)

    用法:
        sensor = AtiNetFtSensor("192.168.1.100")
        sensor.connect()
        sensor.set_zero()  # 零漂补偿
        while True:
            reading = sensor.read()
            if reading: process_force(reading)
    """

    def __init__(self, spec: FtSensorSpec = None):
        self._spec = spec or FtSensorSpec()
        self._connected = False
        self._zero_offset = list(self._spec.zero_offset)

    @property
    def connected(self) -> bool:
        return self._connected

    @abstractmethod
    def connect(self) -> bool:
        """连接传感器。"""
        ...

    @abstractmethod
    def disconnect(self):
        """断开连接。"""
        ...

    @abstractmethod
    def read(self) -> Optional[FtSensorReading]:
        """读取一次力/力矩数据 (非阻塞)。"""
        ...

    def set_zero(self, num_samples: int = 1000):
        """执行零漂校准: 采集 num_samples 次取均值作为零漂。"""
        if not self._connected:
            return

        sums = [0.0] * 6
        count = 0
        for _ in range(num_samples):
            r = self.read()
            if r:
                raw = r.raw_adc
                for i in range(6):
                    sums[i] += raw[i]
                count += 1
            time.sleep(1.0 / self._spec.sample_rate)

        if count > 0:
            self._zero_offset = [s / count for s in sums]

    @staticmethod
    def apply_calibration(
        raw_adc: list[float], cal_matrix: list[float], zero_offset: list[float]
    ) -> tuple[Vector3, Vector3]:
        """应用 6x6 标定矩阵 + 零漂补偿。

        F_calibrated = C * (F_raw - F_zero)
        """
        calibrated = [0.0] * 6
        for i in range(6):
            calibrated[i] = sum(
                cal_matrix[i * 6 + j] * (raw_adc[j] - zero_offset[j])
                for j in range(6)
            )

        force = Vector3(x=calibrated[0], y=calibrated[1], z=calibrated[2])
        torque = Vector3(x=calibrated[3], y=calibrated[4], z=calibrated[5])
        return force, torque


# ── Mock 实现 ──────────────────────────────────────────────────


class MockFtSensor(FtSensorInterface):
    """仿真 F/T 传感器 — 返回零值 (或可配置的预设值)。"""

    def __init__(self, spec: FtSensorSpec = None):
        super().__init__(spec or FtSensorSpec(model=FtSensorModel.MOCK))
        self._mock_force_z: float = 0.0

    def connect(self) -> bool:
        self._connected = True
        return True

    def disconnect(self):
        self._connected = False

    def set_mock_force(self, fz: float):
        """设置模拟 Z 轴力 (用于仿真力控模式)。"""
        self._mock_force_z = fz

    def read(self) -> Optional[FtSensorReading]:
        r = FtSensorReading(timestamp=time.time())
        r.force.z = self._mock_force_z
        r.raw_adc = [0.0] * 6
        return r


# ── ATI Net F/T 驱动 (真机参考实现) ────────────────────────────

class AtiNetFtSensor(FtSensorInterface):
    """ATI Net F/T 传感器 — UDP 通信驱动。

    ATI Net F/T 通信协议:
        - 传感器 IP: 192.168.1.1 (默认)
        - UDP 端口: 49152 (RDT 格式)
        - 数据速率: 1kHz (可配置 100Hz-7kHz)
        - 数据帧: 36 字节 RDT 记录 (6 个 32-bit int + 状态)

    依赖:
        pip install python-atifti  (或 socket 直接实现)

    参考:
        - ATI Net F/T 通信协议手册 (9620-05-Net F/T)
        - ati_ft_sensor ROS2 驱动 (ros-industrial)
    """

    def __init__(self, ip: str = "192.168.1.1", port: int = 49152,
                 spec: FtSensorSpec = None):
        super().__init__(spec or FtSensorSpec(model=FtSensorModel.ATI_NET_FT))
        self._ip = ip
        self._port = port
        self._sock = None
        self._recv_thread: Optional[threading.Thread] = None
        self._latest_reading: Optional[FtSensorReading] = None
        self._reading_lock = threading.Lock()
        self._running = False

    def connect(self) -> bool:
        """建立 UDP 连接并启动接收线程。"""
        import socket
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(0.01)
            self._sock.bind(("0.0.0.0", self._port))
            self._connected = True
            self._running = True
            self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._recv_thread.start()
            return True
        except OSError as e:
            print(f"[ATI] Failed to connect to {self._ip}:{self._port}: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)
        if self._sock:
            self._sock.close()
        self._connected = False

    def _recv_loop(self):
        """1kHz 接收循环 (后台线程)。"""
        import struct
        while self._running and self._sock:
            try:
                data, _ = self._sock.recvfrom(36)
                if len(data) == 36:
                    # RDT 格式: 6 个 int32 (big-endian) + 4 字节状态
                    raw = list(struct.unpack(">6i", data[:24]))
                    status = struct.unpack(">I", data[24:28])[0]
                    reading = FtSensorReading(
                        timestamp=time.time(),
                        raw_adc=[float(v) for v in raw],
                        status=status,
                    )
                    # 应用标定
                    force, torque = self.apply_calibration(
                        reading.raw_adc, self._spec.calibration_matrix, self._zero_offset
                    )
                    reading.force = force
                    reading.torque = torque

                    with self._reading_lock:
                        self._latest_reading = reading
            except (socket.timeout, BlockingIOError):
                pass

    def read(self) -> Optional[FtSensorReading]:
        with self._reading_lock:
            return self._latest_reading
