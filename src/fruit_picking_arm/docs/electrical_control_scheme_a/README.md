# 电控交付包说明（方案 A）

当前只使用方案 A：视觉端发送完整六轴轨迹，电控完整接收并校验后执行；电控周期回传真实六轴角度。

线缆字段、消息字节布局和时序以 [../SERIAL_CONTROL_PROTOCOL.md](../SERIAL_CONTROL_PROTOCOL.md) 为唯一标准；电控验收项见 [../C_BOARD_SERIAL_REQUIREMENTS.md](../C_BOARD_SERIAL_REQUIREMENTS.md)。

电控固件在本仓库外单独维护，不得把开发机桌面路径或未同步副本写成仓库依赖。
本仓库只提供协议文档、ROS 端 C++ 控制器和测试。电控侧必须自行实现/审核协议
解析、500 点轨迹缓存与插值、UART、CAN 电机闭环、PID、软硬限位、急停以及
700 ms 开环夹爪 HandTask，并完成分阶段烧录验收。
