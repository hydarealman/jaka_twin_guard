# JAKA Twin Guard — 项目开发指南

> 最后更新：2026-08-30 | 负责：hydarealman

## 项目概览

本项目基于 ROS2 Humble + MoveIt2，在 WSL2 (Ubuntu 22.04) 上运行。当前主线是
自研六轴机械臂的 D455 水果识别、MoveIt 规划、实车串口执行和好坏分拣；双臂
JAKA C5、Panda、Fanuc、PR2 等目录作为并行方案、参考资源或历史演示保留。

| 子包 | 说明 | 状态 |
|------|------|------|
| `jaka_dual_arm` | 历史双臂操作框架（搬运 + 按摩） | ✅ 保留，不是当前实车主线 |
| `dual_arm_jaka_c5_moveit_config` | 旧 Demo（按摩 + 搬运）+ MoveIt 配置 | ✅ 保留 |
| `fruit_picking_arm` | 当前单臂主线：D455、分拣任务、实车串口、标定 | 🟢 实车人工 Plan/Execute 已跑通，继续验收自动任务 |
| `fruit_arm_moveit_config` | 自研机械臂 URDF/SRDF/MoveIt/RViz 配置 | 🟢 当前使用 |
| `jaka_c5_description` | JAKA C5 STL 模型库（只读，所有包共用） | ✅ 稳定 |

## 快速开始

### 当前水果主线（WSL2 实车/仿真）

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fruit_picking_arm
```

实车入口以 `scripts/single_arm/README.md` 为准；人工水果验收使用：

```bash
bash scripts/single_arm/start_architecture_a_real_fruit_plan_execute.sh
```

### 历史双臂 Docker 环境

> 📖 完整文档 → [docker/README.md](docker/README.md)

```bash
# 安装 Docker Engine (仅首次)
bash docker/wsl2/setup-docker.sh && sudo service docker start

# 构建 + 启动
docker compose -f docker/docker-compose-wsl2.yml build    # 首次约 20-60min
docker compose -f docker/docker-compose-wsl2.yml run --rm jaka-twin-guard /launch/build.sh
bash docker/wsl2/start.sh

# 一键按摩仿真
docker exec -it jaka_twin_guard /launch/massage-gazebo.sh

# 其他启动模式: /launch/carry-rviz.sh, /launch/pick-place.sh 等, 见 docker/README.md
```

### 历史双臂裸机环境

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

### 单臂 Pick&Place + 苹果好坏识别分拣（fruit_picking_arm）
```bash
# 依赖（推理后端为 pip 包，非 rosdep）——国内用清华镜像更快：
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple onnxruntime opencv-python   # 默认后端(轻量)
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple ultralytics                 # 可选后端(需 torch)
# 想永久默认走清华源：
#   pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# 备用源: 阿里 https://mirrors.aliyun.com/pypi/simple/  中科大 https://pypi.mirrors.ustc.edu.cn/simple/

ros2 launch fruit_picking_arm architecture_a_sim.launch.py                   # 完整 Gazebo 抓取分拣
ros2 launch fruit_picking_arm architecture_a_sim.launch.py run_task:=false   # 只检查场景/相机/MoveIt

# 识别节点也可独立运行：
ros2 run fruit_picking_arm fruit_target_node
ros2 topic echo /perception/stable_fruit_targets
```
实车日常操作优先使用 `scripts/single_arm/` 下的脚本，特别是人工验收入口
`start_architecture_a_real_fruit_plan_execute.sh` 和自动入口
`start_architecture_a_real_run.sh`；直接调用实车 launch 的默认值不会启动机器人栈或任务。

实车识别先由 D455 适配 YOLO 找苹果框，再用对齐深度反投影得到三维中心/半径，
最后把紧 ROI 送入 MobileNetV3 分类器。模型目录包含 v1/v2 苹果检测器和一个
质量分类 ONNX；弱结果、非苹果和冲突结果保持 Unknown。
- **融合层** `perception/health_fusion.py`：用相机内参+TF 把检测框投影匹配到点云 3D 物体，赋 `health` 标签；仅仿真允许从 `scene_params_sim.yaml` 的对象提示回退（日志标 `[sim-fallback]`），实车禁止该回退
- **分拣**：`skills/place.py` 按 health 选料框（Healthy→healthy 框, Unhealthy→unhealthy 框）
- **配置**：`config/perception_params.yaml::classifier`（backend/阈值/话题）；实车工作台/分拣箱使用 `config/scene_params_real.yaml`，仿真场景使用 `config/scene_params_sim.yaml`
- Gazebo默认允许scene标签兜底；真机禁止标签兜底。

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
    │   └── fruit_arm_moveit_config/          # 单臂Pick&Place
```

## 历史双臂按摩与旧 Demo 的区别

| | 旧 Demo | 新系统 |
|---|---------|--------|
| 参数 | 硬编码 | YAML 全部可配 |
| 手法 | 关节角 delta | 力控参数 + 关节角(兼容) |
| 规划 | 预计算1条巨轨迹 | 逐阶段 MoveIt RRT |
| 安全 | 碰撞非致命 | 5级安全监控 |
| 力控 | 无 | 虚拟阻抗(仿真) → F/T传感器(真机) |
