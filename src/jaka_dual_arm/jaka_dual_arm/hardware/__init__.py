"""Hardware Abstraction Layer — 真实 JAKA C5 机械臂接口。

模块:
    ft_sensor_interface    — F/T 传感器抽象 (ATI / Weiss / Robotiq / 国产)
    jaka_driver_interface  — JAKA C5 以太网/IP 驱动接口
    real_time_controller   — 1kHz 实时控制循环 (RT-preempt 内核)
    calibration            — 手眼标定 + F/T 传感器零漂补偿

参考:
  - JAKA C5 编程手册 v2.0 — TCP/IP 协议 / Modbus / SDK
  - ATI Net F/T 传感器 — Ethernet/UDP 1kHz 数据流
  - franka_ros2 — 1kHz 实时控制循环架构
  - UR ROS2 Driver — ur_robot_driver + hardware_interface
"""
