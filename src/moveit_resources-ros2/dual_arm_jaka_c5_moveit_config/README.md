# Dual-Arm JAKA C5 防碰撞 Demo

基于 ROS2 Humble + MoveIt2 的**双 JAKA C5 机械臂防碰撞演示系统**。两个机械臂并排放置（间距 50cm），工作空间重叠，使用 OMPL 规划器自动规划无碰撞运动轨迹，在 RViz 中实时显示碰撞检测。

## 系统架构

```
┌──────────────────────────────────────────────────────┐
│                      RViz2                           │
│            (MotionPlanning 面板 + 碰撞可视化)          │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────┐
│                   move_group                          │
│  ┌────────────────────────────────────────────────┐  │
│  │        PlanningPipeline (OMPL)                  │  │
│  │  request_adapters:                             │  │
│  │    ResolveConstraintFrames                     │  │
│  │    → FixWorkspaceBounds                        │  │
│  │    → FixStartStateBounds                       │  │
│  │    → FixStartStateCollision                    │  │
│  │    → AddTimeOptimalParameterization (Ruckig)   │  │
│  └────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────┐  │
│  │  MoveItSimpleControllerManager                 │  │
│  │   left_arm_controller  (FollowJointTrajectory) │  │
│  │   right_arm_controller (FollowJointTrajectory) │  │
│  └────────────────────────────────────────────────┘  │
└──────────────────────┬───────────────────────────────┘
                       │
┌──────────────────────┴───────────────────────────────┐
│            ros2_control (FakeSystem)                  │
│  ┌─────────────────┐  ┌──────────────────┐           │
│  │ left_arm_controller│  │ right_arm_controller│       │
│  │ (6 joints)       │  │ (6 joints)        │       │
│  └─────────────────┘  └──────────────────┘           │
│  ┌──────────────────────────────────────────────┐    │
│  │    joint_state_broadcaster                   │    │
│  └──────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────┘
```

## 项目结构

```
dual_arm_jaka_c5_moveit_config/
├── CMakeLists.txt
├── package.xml
├── README.md
├── BUG_SUMMARY.md               # 调试过程中遇到的 Bug 总结
├── launch/
│   └── demo.launch.py           # 启动 move_group + RViz + ros2_control
└── config/
    ├── jaka_c5_dual.urdf.xacro  # 顶层 URDF：实例化两个机械臂
    ├── jaka_c5_arm_macro.xacro  # XACRO 宏：单臂参数化模板
    ├── jaka_c5.ros2_control.xacro # ros2_control 硬件接口
    ├── jaka_c5_dual.srdf        # SRDF：规划组 + 碰撞矩阵
    ├── kinematics.yaml          # KDL 逆运动学求解器
    ├── joint_limits.yaml        # 关节速度/加速度限制
    ├── ompl_planning.yaml       # OMPL 规划器配置
    ├── moveit_controllers.yaml  # MoveIt 控制器配置
    ├── ros2_controllers.yaml    # ros2_control 控制器配置
    ├── moveit.rviz              # RViz 布局
    ├── left_initial_positions.yaml
    └── right_initial_positions.yaml
```

## 依赖

| 依赖包 | 用途 |
|--------|------|
| `jaka_c5_description` | JAKA C5 机械臂 URDF + STL 网格 |
| `moveit_ros_move_group` | MoveIt 运动规划服务器 |
| `moveit_planners_ompl` | OMPL 规划器接口 |
| `moveit_simple_controller_manager` | 控制器管理 |
| `moveit_ros_visualization` | RViz 可视化 |
| `joint_trajectory_controller` | 关节轨迹执行 |
| `joint_state_broadcaster` | 关节状态发布 |
| `controller_manager` | ros2_control 控制器生命周期 |
| `ruckig` | 时间最优轨迹参数化 |

## 快速开始

### 前置条件

- Ubuntu 22.04 (WSL2 或原生)
- ROS2 Humble
- MoveIt2 (通过 apt 安装)

```bash
# 安装依赖（如未安装）
sudo apt install ros-humble-moveit ros-humble-ruckig ros-humble-ros2-control
```

### 编译

```bash
cd d:/jaka_twin_guard    # Windows 路径
# 或在 WSL2 中：
cd /mnt/d/jaka_twin_guard

colcon build --packages-select jaka_c5_description dual_arm_jaka_c5_moveit_config
source install/setup.bash
```

### 运行

```bash
ros2 launch dual_arm_jaka_c5_moveit_config demo.launch.py
```

RViz 会自动打开，显示两个 JAKA C5 机械臂并排放置。

### 在 RViz 中操作

1. 在 **MotionPlanning** 面板选择规划组（`left_arm` 或 `right_arm`）
2. 拖动机械臂末端的**交互标记**设置目标位姿
3. 在 Planning 标签页点击 **Plan** → 查看无碰撞轨迹
4. 点击 **Plan & Execute** → 机械臂执行轨迹

## 碰撞避免机制

### 核心设计

两个机械臂的基座间距 **50cm**（各自偏离中心 ±25cm），JAKA C5 臂展约 0.9m，工作空间显著重叠。

```
         世界坐标系 (world)
              │
     ┌────────┴────────┐
     │                 │
  y=-0.25           y=+0.25
     │                 │
  ┌──┴──┐         ┌──┴──┐
  │左臂 │         │右臂 │
  │JAKA │  50cm   │JAKA │
  │ C5  │←───────→│ C5  │
  └─────┘         └─────┘
```

### SRDF 碰撞矩阵

- **臂内碰撞禁用**：同臂相邻关节对和已验证永不会碰撞的关节对
- **跨臂碰撞启用**：`left_*` ↔ `right_*` 之间**不禁用**任何碰撞检测

这意味着规划 `left_arm` 的运动时，`right_arm` 的所有连杆都被视为障碍物，OMPL 必须寻找绕过它们的路径。

### FCL 碰撞检测

使用 FCL (Flexible Collision Library) 进行高效的 STL 网格碰撞检测。碰撞发生时会在 RViz 中高亮显示。

## 初始姿态

两个机械臂的初始姿态为弯曲肘部、J1 略微相对的姿态，容易产生碰撞：

| 关节 | 左臂 (left) | 右臂 (right) |
|------|-------------|--------------|
| J1   | 0.5 rad     | -0.5 rad     |
| J2   | 1.0 rad     | 1.0 rad      |
| J3   | -1.5 rad    | -1.5 rad     |
| J4   | 1.0 rad     | 1.0 rad      |
| J5   | 0.0 rad     | 0.0 rad      |
| J6   | 0.0 rad     | 0.0 rad      |

## 硬件接口

本项目使用 `mock_components/GenericSystem` 模拟硬件：
- 支持位置指令接口（position command interface）
- 提供位置和速度状态接口（position + velocity state interface）
- 100Hz 控制循环
- 初始位置从 YAML 文件加载

## 已修复的关键问题

详见 [BUG_SUMMARY.md](BUG_SUMMARY.md)，最重要的问题是：

**AddTimeOptimalParameterization 命名空间错误** — `ompl_planning.yaml` 使用 `default_planner_response_adapters/` 命名空间，但实际插件注册在 `default_planner_request_adapters/`，导致时间参数化失败，轨迹时间戳全为零。

## 相关资源

- [MoveIt2 官方文档](https://moveit.ros.org/)
- [OMPL 规划器](https://ompl.kavrakilab.org/)
- [JAKA 机器人](https://www.jaka.com/)
- [参考：panda_moveit_config](src/moveit_resources-ros2/panda_moveit_config/)
