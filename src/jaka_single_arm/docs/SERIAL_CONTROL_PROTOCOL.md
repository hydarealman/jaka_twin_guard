# JAKA C板串口协议 v1

## 帧格式

```text
AA 55 | VERSION:u8 | TYPE:u8 | FLAGS:u8 | SEQ:u16 | LENGTH:u16
      | PAYLOAD:LENGTH bytes | CRC16:u16
```

- 所有多字节字段为 little-endian。
- CRC使用 CRC-16/CCITT-FALSE，初值 `0xFFFF`，覆盖从 VERSION 到 PAYLOAD。
- `FLAGS bit0=ACK_REQUIRED`，`bit1=RESPONSE`。
- C板发送的 `MOTION_RESULT` 必须设置 `ACK_REQUIRED`，未收到ACK时按同一SEQ重发。
- 同一个 `SEQ` 的重传不得重复执行，只重新返回ACK/已有结果。
- 每帧最大payload为4096字节。

## 公共消息

| TYPE | 值 | 方向 | 说明 |
|---|---:|---|---|
| HELLO | 0x01 | 双向 | 版本握手 |
| HEARTBEAT | 0x02 | 双向 | 在线检测 |
| ACK | 0x03 | C板→PC | 接收/校验结果 |
| ROBOT_STATE | 0x04 | C板→PC | 当前状态、TCP、关节角 |
| ERROR | 0x05 | C板→PC | 异步错误 |
| MOTION_RESULT | 0x30 | C板→PC | 目标或轨迹最终结果 |

ACK payload：

```c
uint16_t acked_seq;
uint8_t  status;       // 0 OK, 1 BAD_PAYLOAD, 2 BUSY, 3 UNSUPPORTED...
uint16_t error_code;
```

ROBOT_STATE固定部分：

```c
uint32_t timestamp_ms;
uint8_t  mode;                 // 0 BOOTING, 1 READY, 2 BUSY, 3 ERROR, 4 ESTOP
uint8_t  gripper_state;
uint16_t error_code;
int32_t  tcp_x_mm, tcp_y_mm, tcp_z_mm;
int32_t  tcp_roll_mdeg, tcp_pitch_mdeg, tcp_yaw_mdeg;
uint8_t  joint_count;
int32_t  joint_position_urad[joint_count];
```

MOTION_RESULT：

```c
uint16_t command_seq;
uint16_t object_id;    // target_id 或 trajectory_id
uint8_t  result_code;  // 0成功，其他见 protocol.py ResultCode
uint16_t error_code;
```

## 方案B：FRUIT_TARGET 0x10

```c
uint16_t target_id;
uint8_t  fruit_class;       // 0 Healthy, 1 Unhealthy, 2 Unknown
uint8_t  flags;
uint16_t confidence_1000;
int32_t  x_mm, y_mm, z_mm;  // robot base坐标系中的水果中心
uint16_t radius_mm;
uint16_t ttl_ms;            // C板从接收时刻开始计时
uint32_t capture_time_ms;   // 仅日志；不用于跨设备直接比较时间
```

C板必须拒绝 Unknown、越界目标、过期目标、BUSY期间的新目标和重复目标。

## 方案A：轨迹消息 0x20~0x22

TRAJECTORY_BEGIN `0x20`：

```c
uint16_t trajectory_id;
uint16_t point_count;
uint8_t  joint_count;
uint8_t  flags;             // bit0: point含velocity
```

TRAJECTORY_POINT `0x21`：

```c
uint16_t trajectory_id;
uint16_t point_index;
uint32_t time_ms;           // 从轨迹起点开始
int32_t  position_urad[joint_count];
int32_t  velocity_urad_s[joint_count];
```

TRAJECTORY_END `0x22`：

```c
uint16_t trajectory_id;
uint16_t point_count;
uint32_t points_crc32;      // 按point_index拼接所有POINT payload后计算
```

C板收到 BEGIN 后进入接收状态，只有 POINT数量、索引和CRC32全部正确才可以开始
执行。执行期间回传 BUSY，结束后先回传最新 ROBOT_STATE，再回传 MOTION_RESULT。

ABORT `0x23` 要求立即停止接收/执行轨迹并进入安全状态。

## 串口联调顺序

1. 只测帧解析、噪声重同步和CRC错误。
2. 测重复SEQ、丢ACK和超时重发。
3. 测READY/BUSY/ERROR状态机。
4. 方案A先发送2个点的小轨迹，再增加点数。
5. 方案B先使用固定安全坐标，再接视觉实时目标。
6. 最后测试断线、急停和进程重启。
