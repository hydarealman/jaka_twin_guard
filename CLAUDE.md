# JAKA Twin Guard — 项目开发指南

> 最后更新：2026-06-24 | 当前分支：main | 负责：hydarealman

## 项目概览

本项目基于 ROS2 Humble + MoveIt2，在 WSL2 (Ubuntu 22.04) 上运行。包含 JAKA C5 机械臂的多个仿真 Demo：

| 子包 | 说明 | 状态 |
|------|------|------|
| `jaka_dual_arm` | **工业级双臂操作框架** (搬运 + 按摩) | ✅ v0.3.0 |
| `dual_arm_jaka_c5_moveit_config` | 旧 Demo (按摩 + 搬运) + MoveIt 配置 | ✅ 保留 |
| `single_arm_jaka_c5_pick_place` | 单臂 Pick-and-Place 水果抓取 | 🟡 待目视确认 |
| `jaka_c5_description` | JAKA C5 STL 模型库（只读，所有包共用） | ✅ 稳定 |

## 快速开始

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
# jaka_c5_description 必须构建，否则 Gazebo 无法加载 STL mesh (B021)
colcon build --packages-select jaka_dual_arm dual_arm_jaka_c5_moveit_config jaka_c5_description
source install/setup.bash
```

### 工业级按摩系统（必须用 Gazebo 真实仿真）
```bash
ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py
# 数据流: Gazebo物理引擎 → /joint_states(真实) → MoveIt规划 → ros2_control → Gazebo
#         └→ RViz 直接订阅 /joint_states 显示真实关节状态
# 架构: YAML驱动 + 力控(虚拟阻抗) + 安全监控 + 逐阶段MoveIt RRT
# 人体模型: massage.world 中 33根Catmull-Rom样条圆柱, 平滑弧形背部
# ⚠️ 废弃: industrial_massage.launch.py (mock假数据, 不要用)
```

### 工业级搬运系统
```bash
ros2 launch jaka_dual_arm sim_rviz.launch.py                  # 场景A: 桌到桌
ros2 launch jaka_dual_arm sim_rviz.launch.py scene:=b         # 场景B: 料框拣选
ros2 launch jaka_dual_arm sim_rviz.launch.py scene:=c         # 场景C: 传送带分拣
ros2 launch jaka_dual_arm sim_gazebo.launch.py                # Gazebo物理仿真
```

### 旧 Demo（保留，不受影响）
```bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py      # 旧按摩
ros2 launch dual_arm_jaka_c5_moveit_config carry_object_demo.launch.py # 旧搬运
```

## 目录结构

```
jaka_twin_guard/
├── CLAUDE.md
├── README.md
└── src/
    ├── jaka_dual_arm/                              # 工业级框架 (v0.3.0)
    │   ├── jaka_dual_arm/
    │   │   ├── control/                            # 控制层
    │   │   │   ├── force_control_interface.py      #   力/位混合控制抽象
    │   │   │   ├── virtual_impedance.py            #   虚拟阻抗控制器(仿真)
    │   │   │   └── safety_monitor.py               #   5级安全监控(ISO 10218-1)
    │   │   ├── hardware/                           # 硬件抽象
    │   │   │   └── ft_sensor_interface.py          #   F/T传感器接口(4种型号)
    │   │   ├── planner/                            # 运动规划
    │   │   │   └── planner_server.py               #   MoveIt2 Action封装
    │   │   ├── scene/                              # 场景管理
    │   │   │   ├── scene_manager.py                #   PlanningScene碰撞对象
    │   │   │   └── perception_interface.py         #   感知接口(Mock/真机)
    │   │   ├── skills/                             # 操作技能
    │   │   │   ├── base_skill.py                   #   Skill基类
    │   │   │   ├── approach.py / grasp.py          #   接近/抓取
    │   │   │   ├── lift.py / place.py              #   抬升/放置
    │   │   ├── behavior/                           # 任务编排
    │   │   │   ├── bt_engine.py                    #   BT执行引擎(XML+tick)
    │   │   │   ├── bt_runner.py                    #   搬运任务运行器
    │   │   │   ├── bt_nodes/                       #   自定义BT节点
    │   │   │   │   ├── bt_node_base.py             #     BT基类(兼容BT.CPP v4)
    │   │   │   │   └── carry_nodes.py              #     搬运BT节点
    │   │   │   └── trees/
    │   │   │       └── carry_task.xml              #     搬运BT定义
    │   │   ├── massage/                            # 按摩模块
    │   │   │   ├── __main__.py                     #   入口点
    │   │   │   └── massage_runner.py               #   工业级按摩运行器
    │   │   └── __main__.py                         # 搬运入口点
    │   ├── config/                                 # YAML配置(全参数化)
    │   │   ├── massage_body_params.yaml            #   人体模型/穴位/区域
    │   │   ├── massage_stages.yaml                 #   60阶段编排定义
    │   │   └── ...
    │   ├── launch/
    │   │   ├── industrial_massage.launch.py         # 按摩启动
    │   │   ├── sim_rviz.launch.py                  # 搬运RViz启动
    │   │   └── real_dual_arm.launch.py             # 真机启动(预留)
    │   └── setup.py
    │
    ├── moveit_resources-ros2/
    │   ├── jaka_c5_description/                    # STL模型(只读)
    │   ├── dual_arm_jaka_c5_moveit_config/          # 旧Demo + MoveIt配置
    │   └── single_arm_jaka_c5_pick_place/          # 单臂Pick&Place
```

## 工业级按摩 vs 旧 Demo 核心区别

| | 旧 Demo | 新系统 |
|---|---------|--------|
| 参数 | 硬编码 | YAML 全部可配 |
| 手法 | 关节角 delta | 力控参数 + 关节角(兼容) |
| 规划 | 预计算1条巨轨迹 | 逐阶段 MoveIt RRT |
| 安全 | 碰撞非致命 | 5级安全监控 |
| 力控 | 无 | 虚拟阻抗(仿真) → F/T传感器(真机) |
