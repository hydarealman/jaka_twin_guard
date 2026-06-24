# jaka_dual_arm — 工业级双臂操作框架

> JAKA C5 双臂协作机械臂的通用化操作软件栈。  
> 架构参考工业界最佳实践和开源标杆项目，从"玩具 Demo"升级为"可部署到真实机器人的操作系统"。

## 架构概览

```
┌──────────────────────────────────────────────────────────┐
│  Layer 5: 任务编排 (Task Orchestration)                   │
│  behavior/bt_runner.py + behavior/bt_nodes/               │
│  ┌────────────────────────────────────────────────────┐   │
│  │  Layer 4: 操作技能 (Manipulation Skills)             │   │
│  │  skills/ → Approach / Grasp / Lift / Place / Slide  │   │
│  │  ┌────────────────────────────────────────────────┐ │   │
│  │  │  Layer 3: 运动规划 (Motion Planning)             │ │   │
│  │  │  planner/planner_server.py (封装 MoveIt2)        │ │   │
│  │  │  ┌────────────────────────────────────────────┐ │ │   │
│  │  │  │  Layer 2: 场景感知 (Scene & Perception)      │ │ │   │
│  │  │  │  scene/scene_manager.py + perception_interface│ │ │   │
│  │  │  └────────────────────────────────────────────┘ │ │   │
│  │  └────────────────────────────────────────────────┘ │   │
│  └────────────────────────────────────────────────────┘   │
│  ┌────────────────────────────────────────────────────┐   │
│  │  Layer 1: 硬件抽象 (Hardware Abstraction)            │   │
│  │  ros2_control + URDF/SRDF (引用旧包)                 │   │
│  └────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────┘
```

## 快速开始

```bash
# 编译
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --packages-select jaka_dual_arm
source install/setup.bash

# 运行搬运任务
ros2 run jaka_dual_arm carry_task_runner
```

## 文件组织

```
jaka_dual_arm/
├── README.md                     ← 本文件
├── jaka_dual_arm/                ← Python 包
│   ├── __init__.py               ← 包文档
│   ├── __main__.py               ← 入口: ros2 run jaka_dual_arm carry_task_runner
│   ├── scene/                    ← Layer 2: 场景感知
│   │   ├── scene_manager.py      ← PlanningScene 管理器
│   │   └── perception_interface.py ← 感知接口（先 mock，后可接真实相机）
│   ├── planner/                  ← Layer 3: 运动规划
│   │   └── planner_server.py     ← 封装 MoveIt2 (关节/位姿/闭链规划)
│   ├── skills/                   ← Layer 4: 操作技能
│   │   ├── base_skill.py         ← Skill 基类 (plan→execute→validate)
│   │   ├── approach.py           ← 接近物体 (笛卡尔直线)
│   │   ├── grasp.py              ← 抓取 (双臂同步)
│   │   ├── lift.py               ← 抬升 (垂直)
│   │   └── place.py              ← 放置 (悬停→下降→释放)
│   └── behavior/                 ← Layer 5: 行为树编排
│       ├── bt_runner.py          ← BehaviorTree 运行器
│       ├── bt_nodes/             ← BT 节点库
│       │   ├── detect_object.py  ← 检测物体 → 黑板
│       │   ├── plan_pick.py      ← 规划抓取
│       │   ├── plan_place.py     ← 规划放置
│       │   ├── check_grasp.py    ← 抓取验证
│       │   └── execute_trajectory.py ← 执行轨迹
│       └── trees/                ← BT XML 定义
│           └── carry_task.xml    ← 搬运任务
├── config/                       ← YAML 参数 (零硬编码)
│   ├── scene_params.yaml         ← 场景布局
│   ├── robot_params.yaml         ← 机器人参数 (换机器人只改此文件)
│   ├── planner_params.yaml       ← 规划参数
│   ├── skill_params.yaml         ← 技能参数
│   └── behavior_params.yaml      ← 行为树参数
├── launch/
│   └── sim_rviz.launch.py        ← RViz 仿真启动
├── scripts/
│   └── carry_task_runner         ← 可执行入口
├── package.xml
├── setup.py
└── CMakeLists.txt
```

## 核心设计决策

### 1. 数据流：从"硬编码轨迹"到"感知驱动"

```
旧架构:
  LEFT_WAYPOINTS[硬编码] → MoveIt 规划 → 执行
  ↑ 问题: 换任务、换物体、换场景都需要改代码

新架构:
  /detect_object → 6DoF Pose → BehaviorTree Blackboard
  → Skill(Approach/Grasp/Lift/Place) → PlannerServer[Action]
  → MoveIt2 在线规划 → FollowJointTrajectory[Action]
  ↑ 优势: 换任务=改 BT XML, 换物体=感知适配, 换场景=YAML 配置
```

### 2. 机器人无关性

换机械臂（JAKA→UR→Franka）只需修改：
- `config/robot_params.yaml` — 关节名、基座位置、FK 参数
- URDF/SRDF 模型文件（在旧包中）

核心代码（planner、skills、behavior）**零修改**。

### 3. 原子技能可自由组合

```python
# 简单的 Pick & Place
runner.build(["Detect", "Approach", "Grasp", "Lift", "Place"])

# 整理散落物体
runner.build(["Detect", "Approach", "Slide", "Place"])

# 轴孔装配 (未来)
runner.build(["Detect", "Approach", "PegInHole", "Release"])
```

### 4. 感知接口可替换

```python
# Phase 1: Mock 感知 (当前)
perception = PerceptionInterface()
perception.add_mock_object("cargo_box", (0.36, 0.02, 0.06))

# Phase 2: 真实相机 (未来)
perception = RealSensePerception(camera_topic="/camera/depth/points")
perception.detect("cargo_box")  # YOLO + PCL → 6DoF Pose
```

## 与旧代码的关系

| 代码 | 状态 | 说明 |
|------|------|------|
| `jaka_dual_arm/` | 🆕 新框架 | 本包，工业级架构 |
| `dual_arm_carry_demo.py` | 📦 保留参考 | 旧的 RViz 搬运 Demo |
| `gazebo_carry_demo.py` | 📦 保留参考 | 旧的 Gazebo 搬运 Demo |
| `dual_arm_massage_demo.py` | ✅ 不受影响 | 按摩 Demo，完全独立 |
| `jaka_c5_description/` | 📚 共享模型库 | STL 模型，新包引用它 |
| `dual_arm_jaka_c5_moveit_config/config/` | 📚 共享配置 | URDF/SRDF/YAML，新包引用它 |

## 参考的开源项目

本框架的设计灵感来自以下开源项目（按影响程度排序）：

| 项目 | 借鉴的设计 | 许可 |
|------|-----------|------|
| **[ManyMove](https://github.com/pastoriomarco/manymove)** | 分层架构 (Planner→Skills→BT)、BehaviorTree ROS2 节点模式、机器人品牌无关的 MoveIt2 封装 | MIT |
| **[multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2)** (TUM) | Controllet 模式（单一控制器管理多台机器人）、1kHz 实时控制循环、MuJoCo Sim2Real | BSD-3 |
| **[ARIAC 2024](https://pages.nist.gov/ARIAC_docs/en/2024.1.0/)** (NIST) | 工业自动化场景设计（传送带+AGV+双臂）、动态故障注入与恢复、传感器-规划-执行闭环 | Public Domain |
| **[MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor)** (PickNik) | Stage 式任务规划 (SerialContainer/ParallelContainer)、抓取候选生成与评分、笛卡尔路径约束 | BSD-3 |
| **[BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)** | 行为树 XML 定义、黑板数据共享、Selector/Fallback/Retry 节点模式、Groot 可视化编辑 | MIT |
| **[py_trees](https://github.com/splintered-reality/py_trees)** | Python 行为树库（本项目直接依赖）、Sequence/Selector/Retry 装饰器 | BSD-3 |
| **[OpenAMRobot](https://github.com/openAMRobot)** | 全栈开源双臂移动机器人、硬件-固件-软件三层分离设计 | MIT |
| **[UW Bimanual System](https://digital.lib.washington.edu/researchworks/items/d848d02a-b099-42c3-8bd5-84a25bf32081)** | 双 Barrett WAM 系统架构、Task Space Regions (TSR) 约束规划、IKFast 解析解 | Academic |

## 依赖

- ROS2 Humble (Ubuntu 22.04)
- MoveIt2 + OMPL
- py_trees (v2.4+)
- ros2_control
- dual_arm_jaka_c5_moveit_config (同仓库的旧包)

## License

BSD
