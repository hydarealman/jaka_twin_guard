# JAKA 单臂两种电控架构

两种架构运行时完全独立，只共享无状态的视觉算法与串口协议编解码库。
不得同时启动两个串口桥，也不得让 MoveIt 和 C 板同时规划同一台机械臂。

## 目录职责

```text
jaka_single_arm/
├── jaka_single_arm/
│   ├── perception/
│   │   ├── yolo_infer.py                 # 共享：好坏识别
│   │   ├── object_detector.py            # 共享：点云三维检测
│   │   ├── health_fusion.py              # 共享：2D类别/3D物体融合
│   │   ├── target_tracker.py              # 共享：多帧稳定
│   │   ├── fruit_target_node.py           # 方案B：稳定目标发布
│   │   └── hand_eye_static_tf.py          # 共享：眼在手外TF
│   ├── communication/
│   │   ├── protocol.py                    # 共享：帧、CRC和payload
│   │   ├── transport.py                   # 共享：ACK、重传、接收线程
│   │   ├── control_link.py                # 共享：目标/轨迹发送API
│   │   ├── board_emulator.py              # 共享：无硬件联调
│   │   ├── serial_trajectory_controller.py# 方案A唯一串口适配器
│   │   └── serial_fruit_target_bridge.py  # 方案B唯一串口适配器
│   ├── planner/ + skills/ + behavior/     # 只属于方案A
│   └── control/                            # 只属于方案A上位机任务层
├── config/
│   ├── architecture_a_serial.yaml
│   ├── architecture_b_serial.yaml
│   └── hand_eye_params.yaml
└── launch/
    ├── architecture_a_moveit_serial.launch.py
    └── architecture_b_target_serial.launch.py
```

## 方案A：上位机 MoveIt 规划

```text
RealSense → YOLO/3D定位 → BT抓取策略 → MoveIt
         → FollowJointTrajectory → serial_trajectory_controller
         → C板轨迹跟踪/关节闭环 → 电机
```

C板必须实现：

- 轨迹缓存及CRC32核对；
- 按 `time_ms` 插值并跟踪关节位置；
- 关节位置/速度/加速度限制；
- 编码器闭环、急停和错误处理；
- 定期回传 `ROBOT_STATE`；
- 完成后回传 `MOTION_RESULT`。

上位机独立启动：

```bash
ros2 launch jaka_single_arm architecture_a_moveit_serial.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200
```

这个 launch 不启动 ros2_control 的 `arm_controller`。串口节点自己提供
`/arm_controller/follow_joint_trajectory` Action，因此现有 PlannerServer 不需要
知道底层已经换成串口。

## 方案B：C板接收水果目标并完成全部控制

```text
RealSense → YOLO/3D定位 → 多帧稳定 → serial_fruit_target_bridge
         → FruitTarget → C板抓取策略/IK/轨迹/关节闭环 → 电机
```

C板除方案A的低层功能外，还必须实现：

- 根据水果中心、半径和类别生成抓取/投放序列；
- TCP偏移、逆运动学和解选择；
- 路径生成、奇异点和不可达判断；
- 夹爪时序及好坏果料框策略；
- 目标序号去重和TTL检查。

上位机独立启动：

```bash
ros2 launch jaka_single_arm architecture_b_target_serial.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200
```

这个 launch 不加载 MoveIt、行为树、PlannerServer、ros2_control 或轨迹控制器。

## 无C板时联调

在操作系统中建立一对互通的虚拟串口。让模拟器打开一端，A或B打开另一端：

```bash
ros2 run jaka_single_arm serial_board_emulator \
  --port /dev/ttyVIRTUAL1 --baudrate 115200
```

模拟器会发送 READY、校验两种命令、返回 ACK/RESULT，并在轨迹模式下把最后
一个轨迹点作为新的关节状态回传。它只验证通信链路，不验证机械臂动力学。

两条ROS接口与虚拟串口的自动验收：

```bash
cd /path/to/jaka_twin_guard
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/jaka_single_arm/test/run_serial_ros_e2e.sh "$PWD"
```

通过标志必须同时出现：

```text
ARCHITECTURE_A_E2E_PASS
ARCHITECTURE_B_E2E_PASS
SERIAL_ROS_E2E_ALL_PASS
```

## 真机启动前的强制条件

1. 填写 `hand_eye_params.yaml` 并把 `calibrated` 改为 `true`；未标定时不会发布TF。
2. 核对 RealSense 实际 RGB、CameraInfo、点云话题及点云 `frame_id`。
3. A/B 配置中的关节顺序、坐标系、串口端口和波特率必须与固件一致。
4. 在无电机、再低速、最后带负载三个阶段逐级联调。
5. 方案B必须先由电控证明IK、限位和急停安全，才能接收真实水果目标。
