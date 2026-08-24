# 电控 C 板串口整改要求

本文档是 `Custom_contrllor` 旧代码整改后的交付接口。旧代码中的
`A5 + 0x0302 + float[6]` 帧不属于方案 A v1，不能与当前上位机混用。

## 1. 物理层和通用帧

- 115200 baud、8N1、无流控、小端序。
- 帧格式：`AA 55 | VERSION | TYPE | FLAGS | SEQ:u16 | LENGTH:u16 | PAYLOAD | CRC16:u16`。
- `VERSION=1`；`FLAGS bit0=ACK_REQUIRED`，`bit1=RESPONSE`。
- CRC 使用 CRC-16/CCITT-FALSE，初值 `0xFFFF`、多项式 `0x1021`，不反射，CRC 不覆盖 `AA 55`。
- 单帧 payload 不得超过 4096 字节。
- CRC 参考向量：`123456789 -> 0x29B1`。

禁止将接收数组直接强转为 C 结构体。必须使用固定宽度整数、显式小端读写函数和有限状态机解析：找帧头、读固定头、检查长度、收齐 payload、校验 CRC、再分发消息。

## 2. 方案 A 消息

上位机发送：

1. `TRAJECTORY_BEGIN (0x20)`：`trajectory_id:u16, point_count:u16, joint_count:u8, flags:u8`。
2. `TRAJECTORY_POINT (0x21)`：`trajectory_id:u16, point_index:u16, time_ms:u32, position_urad[6]:i32, velocity_urad_s[6]:i32`。
3. `TRAJECTORY_END (0x22)`：`trajectory_id:u16, point_count:u16, points_crc32:u32`。

电控必须验证六轴固定顺序、点序号连续唯一、时间严格递增、角度/速度限位、所有点收齐和 CRC32，然后才允许驱动电机。每一条要求 ACK 的命令必须返回 ACK；同一个 `SEQ` 的重传只能重新 ACK，不能重复执行。

电控回传：

- `ROBOT_STATE (0x04)`：建议至少 10 Hz；`joint_position_urad[6]` 必须是编码器实际反馈。
- `MOTION_RESULT (0x30)`：动作完成或失败时发送，必须设置 `ACK_REQUIRED`；丢 ACK 时使用同一帧序号重发。

## 3. 本地安全要求

- 电控本地实现硬限位、软限位、速度/加速度限制、编码器闭环和急停。
- 连续约 1 秒没有心跳或有效控制帧时停止动作并进入 `ERROR`。
- `ESTOP` 优先级高于任何普通命令；普通串口命令不能解除急停。
- 必须提供 J1～J6 的关节顺序、方向、机械零点、减速比、编码器分辨率、软限位和硬限位表。
- 上位机发送的是关节轴微弧度，不是裸电机编码器计数；电控负责关节轴到电机单位的映射。

## 4. 旧代码整改项

`Custom_ctrl.c/.h` 当前存在以下问题：帧头不在 `data_t` 中、长度和接收起点不明确、CRC 校验未调用、错误命令仍会更新角度、没有发送路径、没有 ACK/序号/去重/状态反馈、裸 float 没有单位约束、位域布局依赖 ABI，且 `crc.h` 的 `static` 声明与 `crc.c` 的外部定义不一致。

整改完成后，电控应提交：协议解析/发送源码、编译日志、静态检查结果、golden frame 十六进制样例、ACK/重传/重复帧测试结果、真实编码器角度回传样例和上述轴参数表。
