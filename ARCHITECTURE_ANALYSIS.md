# moveit_ws 工作空间架构分析

> **历史归档**：本文分析的是早期 MoveIt 示例工作空间，不包含当前自研六轴
> `fruit_picking_arm` 实车主线。请勿从本文复制当前启动命令、包关系或坐标约定；
> 当前说明见根目录 `README.md` 和 `src/fruit_picking_arm/docs/RUN_MODES.md`。

## 1. 项目概述

`moveit_ws` 是一个 **ROS2 (Humble) MoveIt2 机器人运动规划** 工作空间。它包含了 `moveit_resources-ros2` 元仓库，为 MoveIt2 提供测试用的机器人模型和运动规划配置。

源代码仓库：[ros-planning/moveit_resources](https://github.com/ros-planning/moveit_resources)
版本：3.1.1

---

## 2. 整体架构

```
moveit_ws/
├── src/                                  # 源代码
│   └── moveit_resources-ros2/            # 元仓库
│       ├── moveit_resources/             # ☂ 元包 (umbrella package)
│       ├── panda_description/            # 🐼 Panda机械臂URDF描述
│       ├── panda_moveit_config/          # ⚙ Panda MoveIt运动规划配置
│       ├── dual_arm_panda_moveit_config/ # 🤝 双机械臂Panda MoveIt配置
│       ├── fanuc_description/            # 🏭 Fanuc机械臂URDF描述
│       ├── fanuc_moveit_config/          # ⚙ Fanuc MoveIt运动规划配置
│       └── pr2_description/             # 🤖 PR2机器人描述
├── build/                                # 编译中间文件
├── install/                              # 安装输出
└── log/                                  # 编译日志
```

---

## 3. 包的层次依赖关系

```
┌──────────────────────────────────────────────────┐
│                moveit_resources                   │  ← 元包（umbrella）
│            (依赖所有子包，统一管理)                 │
├──────────────────────────────────────────────────┤
│                                                   │
│  ┌─────────────────────┐  ┌─────────────────────┐ │
│  │ panda_description   │  │ fanuc_description   │ │  ← 机器人描述层
│  │  (URDF模型文件)      │  │  (URDF模型文件)      │ │
│  └────────┬────────────┘  └────────┬────────────┘ │
│           │                        │              │
│  ┌────────▼────────────┐  ┌───────▼─────────────┐ │
│  │ panda_moveit_config │  │ fanuc_moveit_config │ │  ← 运动规划配置层
│  │  (SRDF + 规划参数)   │  │  (SRDF + 规划参数)   │ │
│  └─────────────────────┘  └─────────────────────┘ │
│                                                   │
│  ┌─────────────────────────────────────────────┐ │
│  │     dual_arm_panda_moveit_config            │ │  ← 高级配置层
│  │    (双机械臂 + 规划参数)                       │ │
│  │    依赖: panda_description                   │ │
│  └─────────────────────────────────────────────┘ │
│                                                   │
│  ┌─────────────────────────────────────────────┐ │
│  │        pr2_description                      │ │  ← 独立描述
│  │    (PR2机器人URDF + 碰撞模型)                  │ │
│  └─────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────┘
```

---

## 4. 启动流程 (以 panda_moveit_config 为例)

```
launch 文件: demo.launch.py
═══════════════════════════════════════════════════════

MoveItConfigsBuilder
├── robot_description (config/panda.urdf.xacro)
│   ├── include: panda_description/urdf/panda.urdf.xacro
│   ├── include: panda.ros2_control.xacro       (硬件接口宏)
│   └── include: panda_hand.ros2_control.xacro   (手抓硬件接口宏)
├── robot_description_semantic (config/panda.srdf)
│   ├── 运动组定义: panda_arm, hand, panda_arm_hand
│   ├── 组状态: ready, extended, transport
│   ├── 末端执行器: hand → panda_arm
│   ├── 虚拟关节: world → panda_link0
│   └── 碰撞禁用矩阵
├── trajectory_execution (gripper_moveit_controllers.yaml)
│   ├── panda_arm_controller (FollowJointTrajectory)
│   └── panda_hand_controller (GripperCommand)
├── planning_pipelines: [ompl, chomp, pilz, stomp]
└── to_moveit_configs()
```

---

## 5. Panda 机器人运动学链

```
world ──[virtual_joint]──► panda_link0 ──[joint1: revolute, Z]──► panda_link1
                                                    │
                         panda_link2 ◄──[joint2: revolute, Z]──┘
                              │
                         [joint3: revolute, Z]
                              │
                         panda_link3
                              │
                         [joint4: revolute, Z]
                              │
                         panda_link4
                              │
                         [joint5: revolute, Z]
                              │
                         panda_link5
                              │
                         [joint6: revolute, Z]
                              │
                         panda_link6
                              │
                         [joint7: revolute, Z]
                              │
                         panda_link7
                              │
                         [joint8: fixed]  ← 固定关节
                              │
                         panda_link8
                              │
                         [panda_hand_joint: fixed]
                              │
                    ┌─────────┴─────────┐
                    │                   │
              panda_hand        [finger_joint1,2: prismatic]
                    │                   │
              ┌─────┴─────┐    ┌────────┴────────┐
         panda_leftfinger  panda_rightfinger
```

---

## 6. 关键配置文件说明

| 配置文件 | 作用 |
|---------|------|
| **URDF (panda.urdf)** | 机器人几何描述：连杆(link)、关节(joint)、碰撞模型、可视化模型 |
| **XACRO** | URDF的宏扩展语言，支持参数化和复用 |
| **SRDF (panda.srdf)** | 语义描述：运动规划组、碰撞禁用矩阵、末端执行器、预定义姿态 |
| **kinematics.yaml** | 运动学求解器配置 (IK solver) |
| **ompl_planning.yaml** | OMPL运动规划器配置，含30+规划算法参数 |
| **joint_limits.yaml** | 关节速度/加速度/急动度限制 |
| **moveit_controllers.yaml** | MoveIt层控制器管理配置 |
| **ros2_controllers.yaml** | ros2_control层控制器定义 |
| **pilz_cartesian_limits.yaml** | 笛卡尔空间运动限制 |

---

## 7. ROS2 节点启动图

```
demo.launch.py 启动的节点:
═══════════════════════════════════════════════════════
                         │
          ┌──────────────┼──────────────┐
          │              │              │
          ▼              ▼              ▼
   ┌────────────┐ ┌────────────┐ ┌──────────────┐
   │ static_tf  │ │robot_state │ │ move_group   │
   │ publisher  │ │ publisher  │ │   (核心)      │
   │(world→link0)│ │(TF发布器)  │ │ 运动规划服务器 │
   └────────────┘ └────────────┘ └──────┬───────┘
                                        │ 参数:
          ┌─────────────────────────────┤  robot_description
          │              │              │  robot_description_semantic
          ▼              ▼              │  kinematics
   ┌────────────┐ ┌────────────┐       │  joint_limits
   │ controller │ │ controller │       │  planning_pipelines
   │  manager   │ │  spawner   │       │  trajectory_execution
   │(ros2_control│ │  ×3个       │       │
   │ 核心节点)   │ │            │       │
   └─────┬──────┘ └─────┬──────┘       │
         │              │              │
    ┌────┴──────────────┴──────┐       │
    │                           │      │
    ▼                           ▼      ▼
┌──────────────┐  ┌───────────────────────┐
│joint_state   │  │ panda_arm_controller  │
│broadcaster   │  │ (7轴轨迹控制器)         │
│(状态发布器)   │  │ FollowJointTrajectory │
└──────────────┘  └───────────────────────┘
                  │
                  │ panda_hand_controller
                  │ (夹爪控制器)
                  │ GripperActionController
                  └───────────────────────┘

   ┌────────────┐ 
   │   RViz2    │  ← 可视化 + 交互式运动规划
   │  (可视化)   │
   └────────────┘

   ┌────────────┐
   │  MongoDB   │  ← 可选：持久化规划场景
   │ warehouse  │     (warehouse_ros_mongo)
   └────────────┘
```

---

## 8. 控制流程时序

```
用户设定目标位姿
      │
      ▼
┌─────────────┐    ┌──────────────┐    ┌───────────────┐
│   RViz2     │───►│  MoveGroup   │───►│  Planning     │
│ (目标输入)   │    │  (动作服务器)  │    │  Pipeline     │
└─────────────┘    └──────────────┘    │  (OMPL/CHOMP  │
                                        │   /Pilz/STOMP)│
                                        └───────┬───────┘
                                                │
                                        ┌───────▼───────┐
                                        │  轨迹规划结果   │
                                        │  (joint       │
                                        │   trajectory) │
                                        └───────┬───────┘
                                                │
                                        ┌───────▼───────┐
                                        │  MoveIt       │
                                        │  Controller   │
                                        │  Manager      │
                                        └───────┬───────┘
                                                │
                                        ┌───────▼───────┐
                                        │  ros2_control │
                                        │  (硬件接口层)   │
                                        │  FakeSystem   │
                                        │  / 真实硬件     │
                                        └───────┬───────┘
                                                │
                                        ┌───────▼───────┐
                                        │  关节状态反馈   │
                                        │  (joint_state │
                                        │   broadcaster) │
                                        └───────────────┘
```

---

## 9. 三种机器人配置对比

| 特性 | Panda (单臂) | Panda (双机械臂) | Fanuc M-10iA |
|------|-------------|-----------------|--------------|
| **自由度** | 7 DOF + 夹爪 | 7+7 DOF + 2夹爪 | 6 DOF |
| **规划组** | panda_arm, hand | left_panda_arm, right_panda_arm | fanuc_arm |
| **控制器** | panda_arm_controller, panda_hand_controller | left_arm_controller, right_arm_controller | fanuc_controller |
| **控制器管理器** | MoveItSimpleControllerManager | Ros2ControlManager | Ros2ControlManager |
| **规划管道** | OMPL + CHOMP + Pilz + STOMP | 仅 OMPL | OMPL |
| **虚拟关节** | world→panda_link0 | world→left_base, world→right_base | world→base_link |
| **硬件接口** | mock_components + isaac | mock_components | mock_components |

---

## 10. XACRO 宏展开流程 (dual_arm_panda)

```
panda.urdf.xacro
      │
      ├── xacro:include "panda_arm_macro.xacro"
      │         │
      │         └── xacro:macro name="panda_arm"
      │               params: name, prefix, parent, origin, initial_positions_file
      │               │
      │               ├── 创建 base_joint (fixed): parent→{prefix}panda_link0
      │               ├── 创建 link0~link8 + joint1~joint8 (7×revolute + fixed)
      │               ├── 创建 hand + fingers (2×prismatic, mimic)
      │               ├── include: panda.ros2_control.xacro
      │               └── include: panda_hand.ros2_control.xacro
      │
      ├── panda_arm(name="left_panda", prefix="left_", parent="world",
      │             origin=xyz="0 -1.5 0")     ← 左臂偏置 -1.5m
      │
      └── panda_arm(name="right_panda", prefix="right_", parent="world",
                    origin=xyz="0 1.5 0")      ← 右臂偏置 +1.5m
```

---

## 11. 碰撞检测优化策略

SRDF 文件中的 `<disable_collisions>` 标签定义了碰撞检测禁用矩阵：

- **Adjacent（相邻）**：相邻连杆永不碰撞检测（如 link0-link1）
- **Never**：空间上不可能碰撞的连杆对（如 link0-link3）
- **Default**：默认禁用（如左右手指之间）

这极大减少了运动规划时的碰撞检测计算量。

---

## 12. 运动规划器的4条管道 (Pipeline)

| 管道 | 说明 |
|------|------|
| **OMPL** | 开源运动规划库，30+种算法（RRT, PRM, EST, etc.） |
| **CHOMP** | 基于梯度的轨迹优化（Covariant Hamiltonian Optimization） |
| **Pilz Industrial Motion Planner** | 工业级笛卡尔空间规划器（直线/圆弧运动） |
| **STOMP** | 随机轨迹优化（Stochastic Trajectory Optimization） |

---

**分析完成日期**：2026-06-12
