# JAKA 单臂 C 板串口协议 v1

## 1. 物理串口

- 波特率：默认 `115200`
- 数据格式：`8N1`
- 流控：无
- 多字节整数：小端序
- 机械臂角度：弧度乘 `1,000,000`，编码为有符号微弧度
- 机械臂角速度：弧度/秒乘 `1,000,000`
- 夹爪：0–100 mm 净开度，不作为第 7、8 个旋转关节发送

实车应使用稳定设备路径，例如
`/dev/serial/by-id/usb-...`，不要依赖可能变化的 `/dev/ttyUSB0`。

## 2. 通用帧

```text
AA 55 | VERSION:u8 | TYPE:u8 | FLAGS:u8 | SEQ:u16 | LENGTH:u16
      | PAYLOAD:LENGTH bytes | CRC16:u16
```

- `VERSION` 当前为 `1`。
- `FLAGS bit0=ACK_REQUIRED`，`bit1=RESPONSE`。
- `CRC16` 使用 CRC-16/CCITT-FALSE，初值 `0xFFFF`，覆盖
  `VERSION` 到 `PAYLOAD`。
- 单帧 payload 最大 4096 字节。
- 要求 ACK 的发送帧在超时后用相同 `SEQ` 重发。
- 接收方必须记忆已执行的 `SEQ`；重复帧只能重发 ACK/结果，不能重复驱动电机。
- 电控连续约 1 秒未收到有效心跳或控制帧时，必须停止当前动作并进入 `ERROR`。

ACK `0x03`：

```c
uint16_t acked_seq;
uint8_t  status;       // 0 OK, 1 BAD_PAYLOAD, 2 BUSY, 3 UNSUPPORTED...
uint16_t error_code;
```

## 3. 方案 A：上位机规划，C 板跟踪六轴轨迹

上位机运行 MoveIt。C++ 节点
`serial_trajectory_controller` 接收 `FollowJointTrajectory`，按照固定顺序
提取 `joint_1` 到 `joint_6`。MoveIt 中的两个夹爪仿真关节不会混入角度数组。

### TRAJECTORY_BEGIN `0x20`

```c
uint16_t trajectory_id;
uint16_t point_count;
uint8_t  joint_count;       // 本项目实车固定为 6
uint8_t  flags;             // bit0: POINT 包含 velocity
```

### TRAJECTORY_POINT `0x21`

```c
uint16_t trajectory_id;
uint16_t point_index;
uint32_t time_ms;           // 相对轨迹起点
int32_t  position_urad[6];  // J1, J2, J3, J4, J5, J6
int32_t  velocity_urad_s[6];
```

### TRAJECTORY_END `0x22`

```c
uint16_t trajectory_id;
uint16_t point_count;
uint32_t points_crc32;      // 按 point_index 拼接全部 POINT payload 后计算
```

C 板只有在点数、索引和 CRC32 全部正确后才能执行。执行期间报告
`BUSY`，结束后发送最新 `ROBOT_STATE` 和 `MOTION_RESULT`。

### 单电机夹爪命令 GRIPPER_COMMAND `0x24`

实车夹爪只有一个电机。URDF 中的 `left_finger_joint` 和
`right_finger_joint` 是为了描述两个对向夹指的几何运动，二者必须等量反向；它们
不会作为两个电机发送。上位机检测到不对称目标时直接拒绝整条命令。

```c
uint16_t command_id;
uint8_t  mode;              // 0 STOP, 1 OPEN, 2 CLOSE, 3 POSITION
uint16_t opening_mm;        // 单个夹爪电机对应的净开度目标，0..100
uint16_t speed_mm_s;
uint16_t force_permille;    // 0..1000，对应驱动允许力的 0..100%
```

当前上位机把仿真夹爪位置换算为：

```text
opening_mm = clamp(
  (left_finger_joint - right_finger_joint - finger_thickness_m) * 1000,
  0, 100)
```

该 payload 只包含一个夹爪执行器目标，不包含左右两个电机角度。因此夹爪电机的
编码器零点、减速比/丝杆导程、电流到夹持力的换算由 C 板负责，
上位机不应直接猜测夹爪电机角度。

## 4. 方案 B：上位机只发送水果目标

FRUIT_TARGET `0x10`：

```c
uint16_t target_id;
uint8_t  fruit_class;       // 0 合格，1 不合格，2 未知
uint8_t  flags;
uint16_t confidence_1000;
int32_t  x_mm, y_mm, z_mm;  // base_link/world 中的水果中心
uint16_t radius_mm;
uint16_t ttl_ms;
uint32_t capture_time_ms;
```

方案 B 中，C 板负责抓取姿态、逆解、轨迹、六轴闭环和夹爪时序。C 板必须
拒绝未知类别、越界、过期、重复和 `BUSY` 期间的新目标。

## 5. C 板回传

ROBOT_STATE `0x04`：

```c
uint32_t timestamp_ms;
uint8_t  mode;                 // 0 BOOTING, 1 READY, 2 BUSY, 3 ERROR, 4 ESTOP
uint8_t  gripper_opening_mm;   // 0..100；未知可填 255
uint16_t error_code;
int32_t  tcp_x_mm, tcp_y_mm, tcp_z_mm;
int32_t  tcp_roll_mdeg, tcp_pitch_mdeg, tcp_yaw_mdeg;
uint8_t  joint_count;          // 正常为 6
int32_t  joint_position_urad[joint_count];
```

`joint_position_urad` 必须来自各轴实际编码器反馈，而不是上位机最后一次命令。
这样上位机才能知道机械臂真正运动到哪里。

MOTION_RESULT `0x30`：

```c
uint16_t command_seq;
uint16_t object_id;     // target_id、trajectory_id 或 gripper command_id
uint8_t  result_code;   // 0 成功；其余见 protocol.py 的 ResultCode
uint16_t error_code;
```

`MOTION_RESULT` 必须设置 `ACK_REQUIRED` 并重发到收到 ACK 为止。

电控重发 `MOTION_RESULT` 时必须保持相同的帧 `SEQ`，不能为同一结果生成新的
序号。上位机收到结果后会自动回 ACK。

## 6. 联调顺序

1. 先用 `serial_board_emulator` 和虚拟串口验证解析、CRC、ACK 与重发。
2. 电控只接逻辑电源，不使能电机，验证 `ROBOT_STATE` 和急停状态。
3. 单轴低速、无负载测试方向、零点、软限位和编码器反馈。
4. 六轴依次测试，再发送一个短距离、低速多点轨迹。
5. 独立测试夹爪 100、80、50、20、0 mm，并校准开度和力限制。
6. 最后接入 MoveIt、视觉目标和完整抓取分类流程。
