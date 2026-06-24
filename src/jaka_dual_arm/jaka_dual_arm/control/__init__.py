#!/usr/bin/env python3
"""Control Layer — 力/位混合控制、笛卡尔运动、安全监控。

模块:
    force_control_interface  — 力/力矩控制抽象 API（真机 F/T 传感器 + 阻抗控制器）
    virtual_impedance        — 虚拟阻抗（仿真模式下的模拟柔顺控制）
    safety_monitor           — 实时安全看门狗（力/速度/空间超限检测）
"""
