# Dual-Arm JAKA C5 防碰撞系统

基于 ROS2 Humble + MoveIt2 的**双 JAKA C5 机械臂碰撞避免演示系统**。两个机械臂并排放置，工作空间深度重叠，利用 OMPL 运动规划器和 FCL 碰撞检测库实时规划无碰撞轨迹，在 RViz 中可视化碰撞状态。

---

## 目录

- [系统概览](#系统概览)
- [机械臂规格](#机械臂规格)
- [系统架构](#系统架构)
- [项目结构](#项目结构)
- [依赖](#依赖)
- [快速开始](#快速开始)
- [配置详解](#配置详解)
  - [URDF / XACRO](#urdf--xacro)
  - [SRDF 碰撞矩阵](#srdf-碰撞矩阵)
  - [运动规划](#运动规划)
  - [控制器](#控制器)
  - [运动学](#运动学)
  - [关节限制](#关节限制)
- [碰撞避免机制](#碰撞避免机制)
- [RViz 操作指南](#rviz-操作指南)
- [故障排除](#故障排除)
- [相关资源](#相关资源)

---

## 系统概览

```
┌─────────────────────────────────────────────────────────────────┐
│                           RViz2                                   │
│     MotionPlanning 面板 · 碰撞高亮 · 交互标记 · 轨迹动画          │
└────────────────────────────┬────────────────────────────────────┘
                             │  ROS Topic / Action
┌────────────────────────────┴────────────────────────────────────┐
│                         move_group                                │
│                                                                   │
│  ┌──────────────────────────────────────────┐                    │
│  │   PlanningPipeline (OMPL)                │  ← ompl_planning   │
│  │   ┌──────────────────────────────────┐   │       .yaml        │
│  │   │ Request Adapters (链式处理)      │   │                    │
│  │   │ ResolveConstraintFrames          │   │                    │
│  │   │ → FixWorkspaceBounds             │   │                    │
│  │   │ → FixStartStateBounds            │   │                    │
│  │   │ → FixStartStateCollision         │   │                    │
│  │   │ → AddTimeOptimalParameterization │   │  ← Ruckig TOTP     │
│  │   └──────────────────────────────────┘   │                    │
│  └──────────────────────────────────────────┘                    │
│                                                                   │
│  ┌──────────────────────────────────────────┐                    │
│  │   MoveItSimpleControllerManager          │  ← moveit_         │
│  │   left_arm_controller                    │    controllers.yaml │
│  │   right_arm_controller                   │                    │
│  └──────────────────────────────────────────┘                    │
└────────────────────────────┬────────────────────────────────────┘
                             │  FollowJointTrajectory Action
┌────────────────────────────┴────────────────────────────────────┐
│                     ros2_control                                  │
│                                                                   │
│  ┌─────────────────────┐  ┌──────────────────────┐              │
│  │ left_arm_controller │  │ right_arm_controller │              │
│  │ JointTrajectoryCtrl │  │ JointTrajectoryCtrl  │              │
│  │    left_joint_1..6  │  │    right_joint_1..6  │              │
│  └─────────┬───────────┘  └──────────┬───────────┘              │
│            │                         │                            │
│  ┌─────────┴─────────────────────────┴───────────┐              │
│  │          joint_state_broadcaster               │              │
│  └────────────────────────────────────────────────┘              │
│            │                                                      │
│  ┌─────────┴─────────────────────────────────────┐              │
│  │   mock_components/GenericSystem (Fake HW)     │              │
│  │   12 joints · position+velocity state · 100Hz │              │
│  └───────────────────────────────────────────────┘              │
└──────────────────────────────────────────────────────────────────┘
```

---

## 机械臂规格

| 参数 | 值 |
|------|------|
| 型号 | JAKA C5 |
| 自由度 | 6 (J1→J6) |
| 臂展 | ≈ 0.9 m |
| 关节范围 | J1/J5/J6: ±360°, J2/J4: -85°~+265°, J3: ±175° |
| 额定速度 | 3.14 rad/s |
| 关节类型 | 全旋转 (revolute) |
| 网格文件 | 7 个 STL (Link_00 至 Link_06) |

### D-H 关节布局

```
  world ──[fixed]──→ Link_00 (基座)
                        │
                     joint_1 (Z轴旋转)
                        │
                     Link_01
                        │
                     joint_2 (Z轴, 绕X+90°)
                        │
                     Link_02 ─── (0.43m 横向延伸)
                        │
                     joint_3 (Z轴)
                        │
                     Link_03
                        │
                     joint_4 (Z轴) ─── (0.3685m 横向)
                        │
                     Link_04
                        │
                     joint_5 (Z轴, 绕X+90°)
                        │
                     Link_05
                        │
                     joint_6 (Z轴, 绕X-90°)
                        │
                     Link_06 (末端)
```

---

## 项目结构

### `jaka_c5_description` — 机械臂模型包

```
jaka_c5_description/
├── CMakeLists.txt              # 安装 urdf/ 和 meshes/ 到共享目录
├── package.xml                 # 依赖 urdf, xacro
├── urdf/
│   └── jaka_c5.urdf            # 单臂 7-link 6-joint URDF
└── meshes/
    └── jaka_c5_meshes/         # 7 个 STL 碰撞/视觉网格
        ├── Link_00.STL
        ├── Link_01.STL
        ├── Link_02.STL
        ├── Link_03.STL
        ├── Link_04.STL
        ├── Link_05.STL
        └── Link_06.STL
```

### `dual_arm_jaka_c5_moveit_config` — MoveIt 配置与启动

```
dual_arm_jaka_c5_moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── BUG_SUMMARY.md               # 调试经验 & Bug 总结
├── launch/
│   └── demo.launch.py           # 一键启动：move_group + RViz + ros2_control
└── config/
    ├── jaka_c5_dual.urdf.xacro   # ═══ 顶层 URDF ═══
    ├── jaka_c5_arm_macro.xacro   #    参数化单臂宏 (prefix, origin)
    ├── jaka_c5.ros2_control.xacro#    ros2_control FakeSystem 宏
    │
    ├── jaka_c5_dual.srdf         # ═══ SRDF ═══
    │                              #    规划组 + 碰撞禁用矩阵
    │
    ├── kinematics.yaml           # KDL 逆运动学 (每组独立超时)
    ├── joint_limits.yaml         # 关节速度/加速度限制
    ├── ompl_planning.yaml        # OMPL 规划器 + 适配器链
    ├── moveit_controllers.yaml   # MoveIt 端控制器映射
    ├── ros2_controllers.yaml     # ros2_control 端控制器定义
    │
    ├── left_initial_positions.yaml  # 左臂初始关节值 (mock HW)
    ├── right_initial_positions.yaml # 右臂初始关节值 (mock HW)
    └── moveit.rviz               # RViz 面板布局
```

---

## 依赖

### 系统依赖

```bash
ros-humble-moveit                    # MoveIt2 完整套件
ros-humble-ruckig                    # 时间最优轨迹参数化 (TOTP)
ros-humble-ros2-control              # ros2_control 框架
ros-humble-joint-trajectory-controller
ros-humble-joint-state-broadcaster
ros-humble-xacro
```

### 工作空间内依赖

| 包 | 说明 |
|---|------|
| `jaka_c5_description` | JAKA C5 URDF + STL 网格（同仓库） |

---

## 快速开始

### 1. 前置条件

- Ubuntu 22.04 (WSL2 或原生)
- ROS2 Humble (通过 apt 安装)
- 确认 Ruckig 已安装：
  ```bash
  ros2 pkg list | grep ruckig
  ```

### 2. 编译

```bash
cd /mnt/d/jaka_twin_guard   # WSL2 中的路径

colcon build --packages-select jaka_c5_description dual_arm_jaka_c5_moveit_config
source install/setup.bash
```

### 3. 启动

```bash
ros2 launch dual_arm_jaka_c5_moveit_config demo.launch.py
```

RViz 自动打开，展示两个 JAKA C5 机械臂。

### 4. 验证碰撞检测

1. 在 **MotionPlanning** 面板中选择 `left_arm`
2. 勾选 **Collision Detection** 查看碰撞状态
3. 拖动左臂末端到右臂附近 → FCL 检测到碰撞时连杆变红
4. 点击 **Plan** → OMPL 规划无碰撞路径
5. 点击 **Plan & Execute** → 机械臂执行该路径

---

## 配置详解

### URDF / XACRO

**XACRO 宏实例化** — 单臂定义为参数化宏，通过两次调用生成双机械臂：

```xml
<!-- jaka_c5_dual.urdf.xacro -->
<robot name="jaka_c5_dual">
  <link name="world"/>

  <!-- 左臂：y=-0.25m，prefix="left_" -->
  <xacro:jaka_c5_arm name="left_jaka_c5"  prefix="left_"  parent="world">
    <origin xyz="0 -0.25 0" rpy="0 0 0"/>
  </xacro:jaka_c5_arm>

  <!-- 右臂：y=+0.25m，prefix="right_" -->
  <xacro:jaka_c5_arm name="right_jaka_c5" prefix="right_" parent="world">
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
  </xacro:jaka_c5_arm>
</robot>
```

宏参数 `prefix` 确保所有 link/joint 名称不冲突：
- 左臂：`left_Link_00` ~ `left_Link_06`，`left_joint_1` ~ `left_joint_6`
- 右臂：`right_Link_00` ~ `right_Link_06`，`right_joint_1` ~ `right_joint_6`

### SRDF 碰撞矩阵

SRDF 文件 (`jaka_c5_dual.srdf`) 定义：

- **规划组** — `left_arm` 和 `right_arm`，各自为 `Link_00 → Link_06` 的链
- **组状态** — `ready`（弯曲姿态）和 `home`（零位）
- **碰撞禁用** — 每臂 8 对 (`Adjacent` × 6 + `Never` × 2)
- **跨臂碰撞** — **不禁用任何 left↔right 对** ← 这是碰撞避免的核心

```xml
<!-- ✅ 禁用的：同臂相邻连杆 -->
<disable_collisions link1="left_Link_00" link2="left_Link_01" reason="Adjacent"/>

<!-- ❌ 不禁用的：跨臂任意连杆对 -->
<!-- 例如 left_Link_03 ↔ right_Link_05 碰撞会被检测 -->
```

### 运动规划

#### OMPL 配置 (`ompl_planning.yaml`)

```yaml
planning_plugins:
  - ompl_interface/OMPLPlanner

request_adapters:
  "default_planner_request_adapters/ResolveConstraintFrames
   default_planner_request_adapters/FixWorkspaceBounds
   default_planner_request_adapters/FixStartStateBounds
   default_planner_request_adapters/FixStartStateCollision
   default_planner_request_adapters/AddTimeOptimalParameterization"

response_adapters:
  "default_planner_response_adapters/ValidateSolution
   default_planner_response_adapters/DisplayMotionPath"
```

**适配器链执行顺序：**

| 阶段 | 适配器 | 作用 |
|------|--------|------|
| 1 | `ResolveConstraintFrames` | 将约束从碰撞对象/子帧解析到机器人连杆 |
| 2 | `FixWorkspaceBounds` | 修正工作空间边界 |
| 3 | `FixStartStateBounds` | 修正起始状态超出关节限位的情况 |
| 4 | `FixStartStateCollision` | 微调有碰撞的起始状态 |
| 5 | **规划器** (OMPL RRTConnect) | 执行实际运动规划 |
| 6 | `AddTimeOptimalParameterization` | **Ruckig** 计算时间最优轨迹时间戳 |
| 7 | `ValidateSolution` | 验证最终路径有效性 |
| 8 | `DisplayMotionPath` | 发布可视化消息到 RViz |

支持的规划器：SBL, EST, KPIECE, RRT, **RRTConnect** (默认), RRT*, TRRT, PRM, PRM*, FMT, BFMT, PDST, STRIDE, BiTRRT, LBTRRT, BiEST, ProjEST, LazyPRM, LazyPRM*, SPARS, SPARStwo, TrajOpt

#### `joint_limits.yaml`

为 Ruckig 时间参数化提供**加速度限制**（URDF 中不含此信息）：

```yaml
default_velocity_scaling_factor: 0.1     # 演示安全速度 10%
default_acceleration_scaling_factor: 0.1 # 演示安全加速度 10%

joint_limits:
  left_joint_1:
    has_velocity_limits: true
    max_velocity: 1.57       # rad/s
    has_acceleration_limits: true
    max_acceleration: 3.0    # rad/s²
  # ... 全部 12 个关节同理
```

> ⚠️ **加速度限制是 TOTP 正常工作的必要条件。** 缺失会导致时间戳计算失败。

### 控制器

#### MoveIt 端 (`moveit_controllers.yaml`)

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  left_arm_controller:
    type: FollowJointTrajectory      # 使用 FollowJointTrajectory Action
    action_ns: follow_joint_trajectory
    joints: [left_joint_1, ..., left_joint_6]

  right_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints: [right_joint_1, ..., right_joint_6]
```

#### ros2_control 端 (`ros2_controllers.yaml`)

两个独立的 `JointTrajectoryController` + 一个 `JointStateBroadcaster`：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

left_arm_controller:
  ros__parameters:
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    joints: [left_joint_1, ..., left_joint_6]

right_arm_controller:
  ros__parameters:
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    joints: [right_joint_1, ..., right_joint_6]
```

### 运动学

```yaml
# kinematics.yaml
left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005  # 搜索精度
  kinematics_solver_timeout: 0.05             # 单次求解超时 (s)

right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
```

---

## 碰撞避免机制

### 布局设计

```
                  world (0,0,0)
                      │
         ┌────────────┴────────────┐
         │                         │
    y = -0.25m               y = +0.25m
         │                         │
    ┌────┴────┐              ┌────┴────┐
    │ 左臂    │              │ 右臂    │
    │ JAKA C5 │  ← 50cm →   │ JAKA C5 │
    │         │              │         │
    │ 臂展~0.9m│             │ 臂展~0.9m│
    └─────────┘              └─────────┘
         └──────────┬──────────┘
              工作空间显著重叠
```

### 碰撞检测流程

1. **FCL 加载** — 每个连杆的 STL 网格作为碰撞几何体
2. **请求适配器处理** — `FixStartStateCollision` 检查并修正起始状态
3. **OMPL 规划** — 每次状态采样都调用 FCL 检查：
   - 同臂连杆对 → 根据 SRDF `disable_collisions` 跳过
   - 跨臂连杆对 → **始终检查**
4. **轨迹验证** — `ValidateSolution` 对最终路径密集采样验证
5. **RViz 可视化** — 碰撞连杆高亮红色，`DisplayMotionPath` 发布轨迹

### 臂间碰撞矩阵

| 左臂连杆 | 右臂连杆 | 碰撞检测 |
|----------|----------|----------|
| left_Link_00 | right_Link_XX | ✅ 检测 |
| left_Link_01 | right_Link_02 | ✅ 检测 |
| left_Link_03 | right_Link_05 | ✅ 检测 |
| left_Link_XX | left_Link_XX | ⚡ 仅相邻+Never跳过 |

---

## RViz 操作指南

### MotionPlanning 面板

1. **选择规划组** — 在 `Planning Group` 下拉菜单选择 `left_arm` 或 `right_arm`
2. **设置起始状态** — 可选 `<current>` 或 `<ready>`
3. **设置目标** — 拖动机械臂末端的**彩色交互标记**：
   - 红色/绿色/蓝色箭头 → 平移
   - 红/绿/蓝色圆环 → 旋转
4. **规划** — 点击 `Plan` 按钮
5. **查看轨迹** — 规划成功后在 RViz 中显示为半透明动画
6. **执行** — 点击 `Plan & Execute`（或在规划后点 `Execute`）

### 碰撞检测开关

在 `Displays` 面板 → `MotionPlanning` → `Scene Robot`：
- 勾选 `Show Collision Contact`：碰撞时显示接触点
- 碰撞连杆自动高亮为红色

---

## 故障排除

### 常见问题

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 规划成功但不执行 | Ruckig 时间参数化失败 | 确认 `AddTimeOptimalParameterization` 在 `request_adapters` 中 |
| RViz 中看不见机械臂 | robot_state_publisher 未启动 | 检查 launch 文件中的 `robot_state_publisher` 节点 |
| 拖动交互标记无反应 | IK 求解超时 | 增大 `kinematics_solver_timeout` 或减小搜索精度 |
| move_group 启动崩溃 | 参数 YAML 格式错误 | 用 `moveit_config.to_dict()` 代替手动 YAML 文件 |
| 规划速度很慢 | OMPL 搜索空间过大 | 检查 `longest_valid_segment_fraction` 或调整规划器参数 |

### 调试命令

```bash
# 查看 move_group 日志（包含规划过程细节）
ros2 launch dual_arm_jaka_c5_moveit_config demo.launch.py 2>&1 | grep move_group

# 列出所有 MoveIt 适配器插件
cat /opt/ros/humble/share/moveit_ros_planning/planning_request_adapters_plugin_description.xml

# 检查控制器是否激活
ros2 control list_controllers

# 手动发送轨迹测试控制器
ros2 topic pub /left_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..."

# 验证 Ruckig 是否可用
ros2 pkg list | grep ruckig
```

---

## 相关资源

| 资源 | 链接 |
|------|------|
| MoveIt2 文档 | https://moveit.ros.org/ |
| OMPL 规划器 | https://ompl.kavrakilab.org/ |
| Ruckig 时间参数化 | https://github.com/pantor/ruckig |
| JAKA 机器人官网 | https://www.jaka.com/ |
| 参考实现 (单臂 Panda) | [panda_moveit_config](../panda_moveit_config/) |
| Bug 调试总结 | [BUG_SUMMARY.md](BUG_SUMMARY.md) |
