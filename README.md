# JAKA Twin Guard

基于 ROS 2 Humble、MoveIt 2、Gazebo 和 Intel RealSense D455 的自研六轴机械臂水果抓取与好坏分拣项目。

当前主线是 `fruit_picking_arm` 单臂系统，包含：

- 基于项目 CAD 装配体生成的六轴机械臂 URDF、碰撞模型和夹爪模型；
- D455 RGB-D 点云定位、水果几何检测和 ROI 质量分类；
- 多帧三维目标稳定跟踪；
- MoveIt 2/KDL IK、OMPL 轨迹规划和行为树抓取流程；
- 面向电控 C 板的串口协议、ACK/重传、轨迹发送和目标发送；
- Gazebo 物理抓取仿真、虚拟串口模拟器和自动化测试。

> 当前仓库同时保留了 PR2、Fanuc、Panda、双臂 JAKA C5 等 MoveIt 示例资源。本文档以水果抓取单臂系统为准，其他资源仅作为通用 MoveIt 测试或历史演示使用。

## 当前状态和重要边界

1. 项目有两套互斥的实车控制架构，不能同时启动。
2. 方案 A 的 MoveIt→C++ 串口轨迹链路已经在实车低速 Plan/Execute 中跑通。电控固件在仓库外单独维护；本仓库不会编译、修改或发布该固件。
3. 方案 B 作为以后与 A 并行的预留方案保留。其 launch、目标筛选和桥接框架仍在，但当前固定长度协议没有 `FRUIT_TARGET` 消息，`ControlLink` 也没有可用的 `send_fruit_target()` 实现，因此不能作为可运行的实车入口。
4. `fruit_arm_description` 中的网格和 CAD 参数属于项目方资产，发布或交付前需要确认许可证。
5. 当前实车苹果定位使用 D455 适配的 YOLO 权重，苹果好坏使用 `fruit_quality_mobilenet_v3.onnx` ROI 分类；模型许可与坏果现场验收仍是正式交付项。
6. 当前这台电脑在 WSL/USBIP 下采用已实测稳定的 `424×240@15` RGB-D。更高分辨率只能在确认彩色、深度同步且无超时后使用，详见 [D455_FRUIT_TUNING.md](src/fruit_picking_arm/docs/D455_FRUIT_TUNING.md)。

## 两种控制架构

| 架构 | 上位机负责 | 电控板负责 | 上位机发送内容 | 启动入口 |
|---|---|---|---|---|
| A：上位机规划 | 感知、抓取策略、IK、MoveIt 规划 | 轨迹插值、关节闭环、电机和夹爪 | 六轴关节轨迹 | `architecture_a_sim.launch.py` / `architecture_a_real.launch.py` |
| B：C 板规划（预留） | 感知、多帧稳定和目标筛选 | 抓取策略、IK、轨迹、关节闭环、电机和夹爪 | 计划发送水果三维目标；当前协议未实现 | 保留 `architecture_b_*.launch.py`，暂不可部署 |

不要同时启动两个串口桥，也不要让 MoveIt 和 C 板同时规划同一台机械臂。

### 方案 A：上位机 MoveIt 规划

```text
D455 点云
  → 三维水果检测 + ROI 质量分类
  → 多对象抓取行为树
  → 水果中心生成 gripper_tcp 目标位姿
  → MoveIt /compute_ik
  → MoveIt /plan_kinematic_path
  → FollowJointTrajectory
  → serial_trajectory_controller
  → 串口 TRAJECTORY_* 帧
  → C 板轨迹跟踪和电机闭环
```

当前方案 A 中没有手写的 `x,y,z → 六个关节角` 解析公式。`planner_server.py` 将目标位姿交给 MoveIt，MoveIt 根据 URDF 中的关节轴、原点、限位和当前关节状态使用 KDL 求解 IK。

### 方案 B：C 板接收水果目标

```text
D455 点云
  → 三维水果检测 + ROI 质量分类
  → 多帧稳定目标
  → serial_fruit_target_bridge
  → FRUIT_TARGET
  → C 板 IK、轨迹和电机控制
```

方案 B 的文件保留用于后续并行开发；当前不能把已有模拟器测试解释为完整的 `FRUIT_TARGET` 实车闭环。

详细架构说明见 [CONTROL_ARCHITECTURES.md](src/fruit_picking_arm/docs/CONTROL_ARCHITECTURES.md)。

## 环境和构建

推荐环境：

- Ubuntu 22.04；
- ROS 2 Humble；
- MoveIt 2；
- Gazebo Classic 与 `gazebo_ros2_control`；
- Python 3、NumPy、SciPy、OpenCV、PySerial；
- `onnxruntime`，用于水果质量分类。

在 ROS 2 环境中：

```bash
cd /path/to/jaka_twin_guard
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
python3 -m pip install onnxruntime

colcon build --symlink-install --packages-up-to fruit_picking_arm
source install/setup.bash
```

如果只修改了机械臂模型或描述包，也可以显式构建：

```bash
colcon build --symlink-install \
  --packages-select fruit_arm_description \
  fruit_arm_moveit_config \
  fruit_picking_arm
```

## 运行模式

所有入口都位于 `src/fruit_picking_arm/launch/`。运行前先加载工作空间：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 方案 A 仿真：完整抓取分拣

```bash
ros2 launch fruit_picking_arm architecture_a_sim.launch.py
```

该入口启动 Gazebo、仿真 RGB-D 相机、点云检测、质量分类、MoveIt、RViz 和抓取行为树。轨迹通过 Gazebo `ros2_control` 执行，不打开真实串口。

只检查场景、相机和 MoveIt，不自动抓取：

```bash
ros2 launch fruit_picking_arm architecture_a_sim.launch.py run_task:=false
```

### 方案 A 实车：先人工验收，再自动运行

```bash
# 只验证当前姿态以及任意目标的 RViz Plan / Execute
bash scripts/single_arm/start_architecture_a_real_plan_execute.sh

# 识别水果，按阶段人工 Plan、检查、Execute
bash scripts/single_arm/start_architecture_a_real_fruit_plan_execute.sh

# 上述流程验收通过后，才使用 RViz/OpenCV 监视下的自动任务
bash scripts/single_arm/start_architecture_a_real_run.sh
```

三个入口都使用真实 D455、手眼静态 TF、真实 `/joint_states`、MoveIt 和 C++ `serial_trajectory_controller`，不会启动 Gazebo。人工水果入口按
`pregrasp → open → grasp → close → lift → bin hover → release → retract → home`
逐段放行；每一段机械臂动作都必须先在 RViz 点击 `Plan` 检查，再点击 `Execute`。
自动入口是另一份独立脚本，不会删除或替换人工调试入口；两者通过运行锁保证不能
同时占用机械臂，并共享同一验收模型、5 帧稳定门限和运动目标参数。

直接调用 `architecture_a_real.launch.py` 的安全默认值是 `start_perception:=false`、
`start_robot_stack:=false`、`run_task:=false`，不会自动得到上述完整运行模式。日常实车操作应使用脚本；只有调试 launch 参数时才直接调用 launch。

### 方案 B（预留，暂不可部署）

方案 B 的 launch、目标跟踪、工作空间门控和串口桥源文件继续保留，不允许删除。
但当前线缆协议只实现方案 A 的轨迹/夹爪消息，尚无 `FRUIT_TARGET` 类型；在补齐并做电控端到端验收前，不要运行 `architecture_b_real.launch.py` 控制实车。

### D455 独立调试

不启动机械臂和串口，只检查真实 RGB-D、三维检测和质量分类：

```bash
bash scripts/single_arm/start_d455_fruit_debug.sh
```

当前 WSL 主机已使用 `424×240@15` 同步 RGB-D；仅彩色模式不能生成三维抓取目标。
如果独立启动了相机，必须先停止该入口再启动实车架构，避免两个 RealSense 节点同时占用设备。

调试入口显示三个相互独立的窗口：

- `YOLO Detector (Raw)`：只画当前神经网络实际输出；
- `KF Tracker Projection`：把通过深度、分类和状态机的三维 KF 坐标重投影到 RGB，并显示 `TRACK/COAST`、速度和预测方向；
- `RGB-D Depth Debug`：显示对齐深度和有效像素比例。

水果人工 Plan/Execute 调试入口加载桌面感知、MoveIt、真实串口控制器和 RViz，
但不会自动夹取：

```bash
bash scripts/single_arm/start_architecture_a_real_fruit_plan_execute.sh
```

若只需感知，请使用：

```bash
bash scripts/single_arm/start_d455_fruit_debug.sh
```

## 单臂抓取的代码流程

方案 A 处理一个水果时，主要调用链为：

```text
PickPlaceRunner.run()
  → SetupTree
    → SetupScene
    → WaitServices
  → SenseTree（每颗水果前重新运行）
    → DetectObjects
  → PickPlaceTree
    → PlanApproach
    → ExecuteTrajectory
    → ControlGripper(prepare_open)
    → PlanGrasp
    → ExecuteTrajectory
    → ControlGripper(close，开环完成但未验证夹取)
    → PlanLift
    → ExecuteTrajectory
    → PlanPlace/PlanPlaceDrop/ControlGripper(open)/PlanPlaceRetract
    → PlanRetreat
    → ExecuteTrajectory
```

重点文件：

| 文件 | 作用 |
|---|---|
| `fruit_picking_arm/behavior/trees/pick_place_task.xml` | 行为树流程 |
| `fruit_picking_arm/behavior/bt_runner.py` | 创建各层组件并执行行为树 |
| `fruit_picking_arm/behavior/bt_nodes/pick_place_nodes.py` | 行为树节点 |
| `fruit_picking_arm/skills/approach.py` | 生成水果上方的接近姿态 |
| `fruit_picking_arm/skills/grasp.py` | 生成抓取姿态 |
| `fruit_picking_arm/skills/lift.py` | 垂直抬升 |
| `fruit_picking_arm/skills/place.py` | 按好坏类别选择料框并投放 |
| `fruit_picking_arm/planner/planner_server.py` | IK、MoveIt 规划和轨迹执行 |
| `fruit_picking_arm/control/safety_monitor.py` | 关节状态、速度、通信和安全检查 |

典型位姿计算为：

```text
抓取姿态：四指夹爪局部 +Z 指向 world -Z，gripper_tcp 对准水果中心
预抓取高度：水果中心 + 半径 + 37 mm 指尖超出量 + 50 mm 安全间隙
腕部 yaw：默认 π；人工调试和自动模式都会依次尝试相同的 8 个对称 yaw，寻找无碰撞 IK
```

实际的六个关节角由 MoveIt/KDL 根据 `gripper_tcp` 目标位姿求解，不由水果检测节点直接计算。86 mm 法兰到指尖偏移只在 URDF 中定义一次。

## 机器人模型、零位和坐标系

自研机械臂模型位于：

```text
src/moveit_resources-ros2/fruit_arm_moveit_config/config/fruit_arm_macro.xacro
src/moveit_resources-ros2/fruit_arm_description/meshes/visual/Link_00.STL ... Link_06.STL
```

当前六个关节名称固定为：

```text
joint_1 joint_2 joint_3 joint_4 joint_5 joint_6
```

夹爪在 URDF 中使用两个相反方向的仿真平移关节：

```text
left_finger_joint right_finger_joint
```

实车夹爪协议只有一个物理执行器，左右夹指的仿真位置会被转换为 `opening_mm`，不会作为两个旋转电机发送。

关节角使用机械基准角和弧度。URDF 中已经吸收了 CAD 装配零位偏置，电控板不能再次重复增加 CAD 偏置；但电控仍必须根据真实编码器零点、方向、减速比和驱动器单位完成电机位置映射。

当前模型的规划基座链为：

```text
world → base_link → cad_base_link → arm_link_1 ... arm_link_6
      → tool_flange → gripper_tcp
```

`world` 是工位/MoveIt 全局坐标；`base_link` 是 400×400 mm 底座外轮廓底面的水平中心；`cad_base_link` 只吸收 CAD 内部平移和 `Rz(-90°)` 轴向适配，应用层不得使用它。当前实车 `world → base_link` 为单位变换。方案 A 的感知和规划使用 `world`；方案 B 未来若实现无 frame-id 的目标协议，则必须使用 `base_link`。

机械限位、软限位、机械零位和待机姿态见 [MECHANICAL_SAFETY_LIMITS.md](src/fruit_picking_arm/docs/MECHANICAL_SAFETY_LIMITS.md)。实车不能直接使用未经机械/电控确认的 CAD 推定参数。

## 感知和坐标变换

当前感知采用两阶段流程：

真实 D455 与仿真使用不同后端。实机链路为：

```text
最新 RGB → YOLO 苹果框（立即发布调试框）
  → 对齐深度反投影、半径与工作区门控
  → 最新任务优先的异步 MobileNetV3 质量分类
  → 三维恒速 KF
  → Detecting → Tracking → Coasting → Lost 状态机
  → /perception/stable_fruit_targets
```

`Coasting` 只允许已经通过深度、几何、质量和多帧门控的轨迹进行短时
预测，默认上限 180 ms。未确认目标不发布预测；不同空间目标不能抢占已有
track ID；采集时间倒退或非法时立即清空运动状态。Gazebo/Mock 仍可使用
点云平面分割与聚类后端。

主要节点和话题：

| 节点/文件 | 作用 |
|---|---|
| `perception/realsense_camera.py` | 订阅 RealSense 点云、深度和彩色图像 |
| `perception/object_detector.py` | 点云平面分割、聚类和水果几何估计 |
| `perception/fruit_quality_classifier.py` | ROI 分类模型推理 |
| `perception/health_fusion.py` | 将质量分类结果融合到三维对象 |
| `perception/motion_kalman.py` | 基于采集时间戳的三维恒速 KF 和新息门控 |
| `perception/target_tracker.py` | 显式状态机、目标关联、短时预测和稳定性检查 |
| `perception/fruit_target_node.py` | 发布稳定三维水果目标 |
| `/perception/stable_fruit_targets` | `vision_msgs/Detection3DArray` |

真实相机到机器人基座的关系由眼在手外标定得到：

```text
base_T_camera_color_optical_frame
```

标定步骤和质量门槛见 [EYE_TO_HAND_CALIBRATION.md](src/fruit_picking_arm/docs/EYE_TO_HAND_CALIBRATION.md)。未标定时 `hand_eye_static_tf` 不发布 TF。

## 串口协议摘要

协议完整定义见 [SERIAL_CONTROL_PROTOCOL.md](src/fruit_picking_arm/docs/SERIAL_CONTROL_PROTOCOL.md)。
电控整改交付要求见 [C_BOARD_SERIAL_REQUIREMENTS.md](src/fruit_picking_arm/docs/C_BOARD_SERIAL_REQUIREMENTS.md)。

物理串口默认参数：

```text
115200 baud, 8N1, 无流控，小端序
```

通用帧：

```text
AA 55 | TYPE:u8 | SEQ:u16 | TYPE 对应的固定长度 PAYLOAD | CRC16:u16
```

线上没有 VERSION、FLAGS 或 LENGTH 字段；CRC16 覆盖 `TYPE + SEQ + PAYLOAD`。

主要消息：

| 消息 | 方向 | 用途 |
|---|---|---|
| `TRAJECTORY_BEGIN (0x01)` | 上位机 → C 板 | 方案 A 的轨迹开始和点数 |
| `TRAJECTORY_POINT (0x02)` | 上位机 → C 板 | 轨迹时间、六轴角度和速度 |
| `TRAJECTORY_END (0x03)` | 上位机 → C 板 | 完成接收，电控校验后执行 |
| `STOP (0x04)` | 上位机 → C 板 | 受控停止当前动作 |
| `HEARTBEAT (0x05)` | 上位机 → C 板 | 10 Hz 通信看门狗 |
| `CLAW_COMMAND (0x06)` | 上位机 → C 板 | 二值开/闭/停 |
| `ACK (0x80)` | 双向 | 确认可靠帧 |
| `ROBOT_STATE (0x81)` | C 板 → 上位机 | 六轴真实机械关节角和状态 |
| `MOTION_RESULT (0x82)` | C 板 → 上位机 | 轨迹完成、失败或停止结果 |
| `CLAW_RESULT/STATE (0x83/0x84)` | C 板 → 上位机 | 开环夹爪动作结果和未验证状态 |

角度编码：

```text
position_urad = round(position_rad × 1,000,000)
velocity_urad_s = round(velocity_rad_s × 1,000,000)
```

电控板必须使用真实编码器反馈回传 `ROBOT_STATE`，不能把上位机最后一次命令当作实际状态。上位机据此发布 `/joint_states`、判断 READY/BUSY/ERROR/ESTOP、计算速度并进行安全监控。

没有真实电控板时，可使用 `serial_board_emulator` 和虚拟串口验证方案 A 的固定帧协议，但模拟器不验证动力学和电机安全。方案 B 的 `FRUIT_TARGET` 尚未实现，不能用该模拟器宣称 B 已通过。

## 手眼标定和实车准备

标定板刚性安装在机械臂末端后，可启动标定入口：

```bash
ros2 launch fruit_picking_arm eye_to_hand_calibration.launch.py \
  serial_port:=/dev/serial/by-id/usb-YOUR_BOARD
```

采集完成后使用服务：

```bash
ros2 service call /hand_eye_calibration/capture std_srvs/srv/Trigger '{}'
ros2 service call /hand_eye_calibration/solve std_srvs/srv/Trigger '{}'
ros2 service call /hand_eye_calibration/save std_srvs/srv/Trigger '{}'
```

实车联调必须分级进行：

1. 电机断电或仅逻辑电源，验证串口、READY、ROBOT_STATE、急停和错误码。
2. 单轴 5% 额定速度以内，验证编码器方向、零位和软限位。
3. 六轴无负载低速轨迹，验证角度映射、速度、加速度和停止行为。
4. 单独测试夹爪 OPEN/CLOSE/STOP、700 ms 输出、互斥 GPIO 和断线撤销；当前没有真实开度、夹持力或抓取成功反馈。
5. 空载完整轨迹，再接入水果和料框。
6. 最后才允许启用自动抓取和带负载运行。

软件限位、急停、通信超时和编码器闭环不能只依赖上位机，电控板必须独立实现。

## 测试和验收

纯 Python 协议和目标跟踪测试：

```bash
export PYTHONPATH="$PWD/src/fruit_picking_arm"
python3 -m pytest -q \
  src/fruit_picking_arm/test/test_serial_protocol.py \
  src/fruit_picking_arm/test/test_serial_integration.py \
  src/fruit_picking_arm/test/test_motion_kalman.py \
  src/fruit_picking_arm/test/test_fast_box_tracker.py \
  src/fruit_picking_arm/test/test_target_tracker.py
```

ROS 2 包测试：

```bash
colcon test --packages-select fruit_picking_arm --event-handlers console_direct+
colcon test-result --verbose
```

历史 `run_serial_ros_e2e.sh` 在有效的方案 A 用例后还会强制进入未完成的方案 B
用例，当前不能作为整套发布验收命令。方案 A 的 C++ 协议/控制器以本包的
GTest、pytest 和实车低速 Plan/Execute 结果为准；补齐 B 协议前不要把脚本最终
退出码写成“所有架构通过”。

自研机械臂 Gazebo/MoveIt 验收：

```bash
bash src/fruit_picking_arm/test/run_custom_arm_gazebo_e2e.sh "$PWD"
bash src/fruit_picking_arm/test/run_custom_arm_pick_e2e.sh "$PWD"
bash src/fruit_picking_arm/test/run_custom_arm_state_probe.sh "$PWD"
```

仿真通过不等于实车安全通过。真实交付还必须完成编码器零位、方向、限位、TCP、手眼外参和夹爪负载验收。

## 目录结构

```text
src/fruit_picking_arm/
├── config/                         # 感知、规划、安全、串口和标定参数
├── docs/                           # 架构、协议、标定、模型和安全说明
├── fruit_picking_arm/
│   ├── perception/                 # D455、点云检测、质量分类、多帧跟踪
│   ├── planner/                    # MoveIt IK、规划和轨迹执行
│   ├── skills/                     # 接近、抓取、抬升、投放、回零
│   ├── behavior/                   # 行为树和抓取任务编排
│   ├── communication/              # 协议、传输、桥接器和电控模拟器
│   ├── control/                    # 上位机安全监控和夹爪控制
│   └── scene/                      # PlanningScene 管理
├── launch/                         # 四种运行模式、D455 和标定入口
├── models/                         # 开发阶段质量分类模型及许可说明
├── src/serial_trajectory_controller.cpp
└── test/                           # 单测、协议测试和 ROS/Gazebo 验收脚本

src/moveit_resources-ros2/
├── fruit_arm_description/          # 自研机械臂 CAD 网格
├── fruit_arm_moveit_config/  # URDF、SRDF、MoveIt 和 Gazebo 配置
├── jaka_c5_description/            # JAKA C5 描述资源
└── 其他 MoveIt 示例资源
```

## 推荐阅读顺序

如果要理解“水果坐标如何变成电机运动”，建议按以下顺序阅读：

1. [RUN_MODES.md](src/fruit_picking_arm/docs/RUN_MODES.md)：先分清 A/B 两条控制链。
2. `fruit_arm_macro.xacro`、`gripper.xacro`、SRDF、KDL 和关节限位：理解关节轴、零位和模型。
3. `object_detector.py`、`fruit_target_node.py`、手眼标定文档：理解坐标来源。
4. `pick_place_task.xml`、`bt_runner.py`、各个 `skills/*.py`：理解抓取任务时序。
5. `planner_server.py`：理解目标位姿、IK、规划轨迹和执行接口。
6. `protocol.py`、`control_link.py`、`serial_trajectory_controller.cpp`：理解串口帧、轨迹和反馈。
7. `SERIAL_CONTROL_PROTOCOL.md`、`MECHANICAL_SAFETY_LIMITS.md`：对照电控固件实现安全边界。

## 相关文档

- [控制架构](src/fruit_picking_arm/docs/CONTROL_ARCHITECTURES.md)
- [运行模式](src/fruit_picking_arm/docs/RUN_MODES.md)
- [串口协议](src/fruit_picking_arm/docs/SERIAL_CONTROL_PROTOCOL.md)
- [眼在手外标定](src/fruit_picking_arm/docs/EYE_TO_HAND_CALIBRATION.md)
- [机械安全限位](src/fruit_picking_arm/docs/MECHANICAL_SAFETY_LIMITS.md)
- [自研 CAD 模型](src/fruit_picking_arm/docs/CUSTOM_ARM_CAD_MODEL.md)
- [D455 调参记录](src/fruit_picking_arm/docs/D455_FRUIT_TUNING.md)
- [模型许可证审计](src/fruit_picking_arm/docs/MODEL_LICENSE_AUDIT.md)
- [架构和历史分析](ARCHITECTURE_ANALYSIS.md)
- [问题记录](BUGLOG.md)

## 许可证

软件包的许可证信息以各目录中的 `package.xml` 为准。自研机械臂网格和 CAD 描述包标记为 Proprietary；水果质量模型和数据集的许可边界见 [MODEL_LICENSE_AUDIT.md](src/fruit_picking_arm/docs/MODEL_LICENSE_AUDIT.md)。未经项目方确认，不要将模型、CAD 或训练数据用于商业交付或再分发。
