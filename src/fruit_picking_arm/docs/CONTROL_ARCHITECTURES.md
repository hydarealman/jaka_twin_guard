# 图纸水果机械臂的两种电控架构

两种架构的设计目标是运行时完全独立并共享视觉算法；当前只有方案 A 的固定长度
串口协议实现完整，方案 B 尚未接入可发送的目标消息。
不得同时启动两个串口桥，也不得让 MoveIt 和 C 板同时规划同一台机械臂。

> 当前实现状态：方案 A 的实车 MoveIt→串口轨迹已经跑通；方案 B 是必须保留的
> 未来并行方案，但当前固定长度协议尚未定义 `FRUIT_TARGET`，`ControlLink` 也没有
> `send_fruit_target()`。因此 B 的 launch/桥接文件只能作为待续框架，不能用于实车。

## 目录职责

```text
fruit_picking_arm/
├── fruit_picking_arm/
│   ├── perception/
│   │   ├── fruit_quality_classifier.py   # 共享：ROI种类/好坏识别
│   │   ├── object_detector.py            # 共享：点云三维检测
│   │   ├── health_fusion.py              # 共享：2D类别/3D物体融合
│   │   ├── target_tracker.py              # 共享：多帧稳定
│   │   ├── fruit_target_node.py           # 方案B：稳定目标发布
│   │   └── hand_eye_static_tf.py          # 共享：眼在手外TF
│   ├── communication/
│   │   ├── protocol.py                    # 方案A Python测试兼容：固定帧/CRC/payload
│   │   ├── transport.py                   # Python测试：ACK、重传、接收线程
│   │   ├── control_link.py                # 旧测试控制API；当前没有水果目标发送方法
│   │   ├── board_emulator.py              # 方案A无硬件协议联调
│   │   └── serial_fruit_target_bridge.py  # 方案B预留桥；目标发送API尚缺失
│   ├── planner/ + skills/ + behavior/     # 只属于方案A
│   └── control/                            # 只属于方案A上位机任务层
├── config/
│   ├── architecture_a_serial.yaml
│   ├── architecture_b_serial.yaml
│   └── hand_eye_params.yaml
├── launch/
│   ├── architecture_a_moveit_serial.launch.py
│   └── architecture_b_target_serial.launch.py
├── src/serial_trajectory_controller.cpp   # 方案A唯一串口适配器
└── src/serial_protocol.cpp                # 方案A串口协议实现
```

## 方案A：上位机 MoveIt 规划

```text
RealSense → YOLO框+对齐深度三维定位/ROI质量分类 → BT抓取策略 → MoveIt
         → FollowJointTrajectory → serial_trajectory_controller
         → C板轨迹跟踪/关节闭环 → 电机
```

C板必须实现：

- 最多 500 点的轨迹缓存、连续索引/严格递增时间和每帧 CRC16 核对；
- 按 `time_ms` 插值并跟踪关节位置；
- 关节位置/速度/加速度限制；
- 编码器闭环、急停和错误处理；
- 定期回传 `ROBOT_STATE`；
- 完成后回传 `MOTION_RESULT`。

低层 launch 直接启动时必须显式启用机器人栈：

```bash
ros2 launch fruit_picking_arm architecture_a_moveit_serial.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200 \
  robot_serial:=fruit-arm-primary start_robot_stack:=true
```

这个 launch 不启动 ros2_control 的 `arm_controller`。串口节点自己提供
`/arm_controller/follow_joint_trajectory` Action，因此现有 PlannerServer 不需要
知道底层已经换成串口。

## 方案B：C板接收水果目标并完成全部控制（预留设计）

```text
RealSense → YOLO框+对齐深度三维定位/ROI质量分类 → 多帧稳定 → serial_fruit_target_bridge
         → FruitTarget → C板抓取策略/IK/轨迹/关节闭环 → 电机
```

C板除方案A的低层功能外，还必须实现：

- 根据水果中心、半径和类别生成抓取/投放序列；
- TCP偏移、逆运动学和解选择；
- 路径生成、奇异点和不可达判断；
- 夹爪时序及好坏果料框策略；
- 目标序号去重和TTL检查。

`FruitTarget` 串口 payload 不包含 frame id，因此协议强制规定 XYZ 使用 `base_link`；
桥接节点会拒绝 `world`、相机光学帧或任何其他 frame，禁止静默混用坐标。

计划中的上位机入口：

```bash
ros2 launch fruit_picking_arm architecture_b_target_serial.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200
```

这个 launch 不加载 MoveIt、行为树、PlannerServer、ros2_control 或轨迹控制器。
不过当前运行到目标下发时会因协议/发送 API 未实现而失败；补齐前不要连接实车运行。

## 无C板时联调

在操作系统中建立一对互通的虚拟串口。当前只用它验收方案 A：

```bash
ros2 run fruit_picking_arm serial_board_emulator \
  --port /dev/ttyVIRTUAL1 --baudrate 115200
```

模拟器会发送 READY、校验方案 A 的轨迹/夹爪命令、返回 ACK/RESULT，并把最后
一个轨迹点作为新的关节状态回传。它只验证通信链路，不验证机械臂动力学。

仓库的历史 `run_serial_ros_e2e.sh` 在 A 用例后还会强制运行未完成的 B 用例，当前
不能作为整套协议的发布验收命令。只应把其 A 阶段输出
`ARCHITECTURE_A_E2E_PASS` 作为诊断信息；后续应先为脚本增加独立的 `SKIP_B` 开关，
再恢复自动验收。

有效的 A 阶段标志是：

```text
ARCHITECTURE_A_E2E_PASS
```

注意：当前脚本会继续进入历史 B 段并失败，因此看到 A 标志后也不能把最终脚本退出码
当成全协议通过。历史 `ARCHITECTURE_B_E2E_PASS` 不能替代缺失的 `FRUIT_TARGET` 线缆实现。

## 真机启动前的强制条件

1. 填写 `hand_eye_params.yaml` 并把 `calibrated` 改为 `true`；未标定时不会发布TF。
2. 核对 RealSense 实际 RGB、CameraInfo、点云话题及点云 `frame_id`。
3. A/B 配置中的关节顺序、坐标系、串口端口和波特率必须与固件一致。
4. 在无电机、再低速、最后带负载三个阶段逐级联调。
5. 方案B必须先由电控证明IK、限位和急停安全，才能接收真实水果目标。
