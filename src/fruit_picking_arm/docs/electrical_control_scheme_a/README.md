# 电控交付包说明（方案 A）

当前只使用方案 A：视觉端发送完整六轴轨迹，电控完整接收并校验后执行；电控周期回传真实六轴角度。

线缆字段、消息字节布局和时序以 [../SERIAL_CONTROL_PROTOCOL.md](../SERIAL_CONTROL_PROTOCOL.md) 为唯一标准；电控验收项见 [../C_BOARD_SERIAL_REQUIREMENTS.md](../C_BOARD_SERIAL_REQUIREMENTS.md)。

可直接交给电控参考的独立 C 实现位于：

```text
C:\Users\dong\Desktop\Custom_contrllor
```

该目录只提供协议解析、轨迹缓存/插值接口、开环夹爪请求/状态、可靠结果回传和控制层适配钩子，不包含具体 UART DMA、CAN 电机驱动、PID、限位或继电器代码。夹爪 GPIO 与 700 ms 定时仍由电控 HandTask 实现；接入实车前，电控必须审核、编译、烧录并完成分阶段验收。
