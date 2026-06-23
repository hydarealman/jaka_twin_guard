MoveIt Resources
================

This repository includes various resources (URDFs, meshes, moveit_config packages) needed for MoveIt testing.

GitHub Actions: [![Formatting (pre-commit))](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml?query=branch%3Aros2) [![Build and Test](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml?query=branch%3Aros2)

## Included Robots

- PR2
- Fanuc M-10iA
- Franka Emika Panda
- **JAKA C5** — 双臂协同搬运、双臂按摩、单臂 Pick-and-Place（本项目新增）

## JAKA C5 双臂按摩演示（中医推拿50式）

> **脚本**：[dual_arm_massage_demo.py](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/scripts/dual_arm_massage_demo.py)（~700行）
> **启动文件**：[massage_demo.launch.py](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/launch/massage_demo.launch.py)
> **最后更新**：2026-06-23 | 60式中医推拿（推→按揉→点穴→摩→擦→滚→振→击法收功）

### 运行方式

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_arm_jaka_c5_moveit_config
source install/setup.bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py
```

默认 `repeat_count:=0` 无限循环，`Ctrl+C` 停止。固定轮数：

```bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py repeat_count:=2
```

调节速度（越小越快，建议 ≥0.45）：

```bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py trajectory_time_scale:=0.55
```

### 场景布局（2026-06-23 终版 — 床横放长沿X）

```
地面z=0 ─────────────────────────────────
  │
  ├── 床框 z=0.04 (底z=0, 长X=1.20 宽Y=0.66, 厚0.08m)
  │   └── 床垫 z=0.11 (底z=0.08, 顶z=0.14, 厚0.06m)
  │       └── 枕头 (x=0.28, 床头)
  │
  ├── 人体（俯卧，脊柱沿X, y=0床中线, z=0.171~0.205）
  │   头x=0.27, C7=0.38, 腰椎=0.77, 骶=0.86
  │   脊柱脊线(LINE_STRIP y=0) + 侧边轮廓(y=±半宽) + 膀胱经14穴(y=±0.04)
  │
  ├── 左臂基座 (0.53, -0.45) ── 肩z=0.12 ── 负责身体左侧(y<0)
  └── 右臂基座 (0.69,  0.45) ── 肩z=0.12 ── 负责身体右侧(y>0)
      双臂Y对称±0.45, X错开0.16m → 工作空间隔离
```

**关键设计**：
- 床**直接贴地**（床框底z=0），床垫顶z=0.14
- **左臂**在床左侧(y=-0.45)，负责脊柱中线(y=0)+身体**左侧**(y<0)
- **右臂**在床右侧(y=0.45)，负责脊柱中线(y=0)+身体**右侧**(y>0)
- 双臂通过pos参数('C'/'L'/'R')控制按摩位置，**不跨中线**
- 双臂**永不同时指向同一脊柱X位置**，"双按"阶段ΔX≥0.22m
- 按摩专用**无末端法兰**(`use_flange=false`)，搬运Demo不受影响

### 按摩流程：60式中医推拿（8阶段13手法）

> 基于中医推拿十三法设计：推→按→揉→点→摩→擦→滚→振→击→拍
> 左臂→脊柱中线(y=0)+左侧(y<0)，右臂→脊柱中线(y=0)+右侧(y>0)
> 新增手法：摩法(rub_L/R)、擦法(scrub)、振法(vibrate)、深按(deep_press)、击法(strike)

| 段 | 式数 | 手法 | 中线覆盖 | 新增手法 | 说明 |
|----|------|------|---------|---------|------|
| **1. 推法** | 1-6 | 长推 | ✅全中线(y=0) | — | 沿脊柱中线推扫热身 |
| **2. 按揉法** | 7-16 | 按压+揉捏 | ✅中线+双侧 | deep_press | 深层组织三线交替 |
| **3. 点穴法** | 17-30 | 点按 | — | — | 膀胱经7穴逐穴点按 |
| **4. 摩法** | 31-36 | 圆周揉摩 | ✅中线+双侧 | rub_L/R | 小幅度腕部圆周运动 |
| **5. 擦法** | 37-42 | 直线推擦 | ✅全中线 | scrub | 沿脊柱前后直线推擦 |
| **6. 滚揉法** | 43-48 | 滚动揉捏 | ✅全中线 | — | 沿中线肌肉松解滚动 |
| **7. 振法** | 49-54 | 快速颤动 | ✅中线+左侧 | vibrate | 微小关节振荡放松 |
| **8. 击法+收功** | 55-60 | 敲击+深按 | ✅中线+双侧 | strike, deep_press | 冷却收功 |

#### 第一阶段：推法 Gliding（式1-6）— 脊柱中线热身

> **规则**：双臂全程沿脊柱中线(y=0)推扫，错区配对ΔX≥0.22m

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 1 | 推法·双悬 | C7中(y=0)悬停 | 中背中(y=0)悬停 | 0.25 |
| 2 | 推法·左推C7中 | C7中按压 | 中背中悬停(避让) | 0.25 |
| 3 | 推法·右按下胸中 | 肩中悬停(避让) | 下胸中按压 | 0.23 |
| 4 | 推法·双换 | 上背中悬停 | 腰椎中悬停 | 0.22 |
| 5 | 推法·左按上背中 | 上背中按压 | 骶中悬停(避让) | 0.29 |
| 6 | 推法·双归位 | 肩中释放 | 腰椎中释放 | — |

#### 第二阶段：按揉法 Press-Knead（式7-16）— 三线深层组织

> 中线深按+双侧揉捏：C7↔中背→肩↔下胸→上背↔腰

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 7 | 按揉·双悬 | C7左侧悬停 | 中背右侧悬停 | 0.25 |
| 8 | 按揉·左按C7左 | C7左侧按压 | 中背右侧悬停(避让) | 0.25 |
| 9 | 按揉·右深按中背中 | C7中释放 | 中背中**深按** | 0.25 |
| 10 | 按揉·左揉C7左 | C7左侧揉捏 | 下胸右侧悬停(避让) | 0.32 |
| 11 | 按揉·右揉中背右 | 肩左侧悬停(避让) | 中背右侧揉捏 | 0.16 |
| 12 | 按揉·双悬中 | 肩中悬停 | 下胸中悬停 | 0.23 |
| 13 | 按揉·左深按肩中 | 肩中**深按** | 下胸中悬停(避让) | 0.23 |
| 14 | 按揉·右深按下胸中 | 肩中释放 | 下胸中**深按** | 0.23 |
| 15 | 按揉·左揉肩左 | 肩左侧揉捏 | 腰椎右侧悬停(避让) | 0.30 |
| 16 | 按揉·右揉下胸右 | 上背左侧悬停(避让) | 下胸右侧揉捏 | 0.15 |

#### 第三阶段：点穴法 Acupressure（式17-30）— 膀胱经穴位

> 左臂→左侧穴(y=-0.04)，右臂→右侧穴(y=+0.04)，错区配对ΔX≥0.21m

| 式 | 名称 | 左臂穴位 | 右臂穴位 | ΔX |
|----|------|---------|---------|-----|
| 17-19 | 点穴·BL11/BL17 | BL11大杼(x=0.41) 悬→按→释 | BL17膈俞(x=0.63) 悬→悬→按 | 0.22 |
| 20-22 | 点穴·BL13/BL18 | BL13肺俞(x=0.48) 悬→按→释 | BL18肝俞(x=0.70) 悬→悬→按 | 0.22 |
| 23-25 | 点穴·BL15/BL23 | BL15心俞(x=0.55) 悬→按→释 | BL23肾俞(x=0.77) 悬→悬→按 | 0.22 |
| 26-28 | 点穴·BL17/BL25 | BL17膈俞(x=0.63) 悬→按→释 | BL25大肠俞(x=0.84) 悬→悬→按 | 0.21 |
| 29-30 | 点穴·双按收 | BL11大杼(x=0.41) 悬→按 | BL23肾俞(x=0.77) 悬→按 | **0.36** |

#### 第四阶段：摩法 Rub（式31-36）— 圆周揉摩（新增）

> 小幅度腕部圆周按摩：中线+左侧+右侧交替

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 31 | 摩法·双悬 | C7中悬停 | 中背中悬停 | 0.25 |
| 32 | 摩法·左摩C7左 | C7左**摩法** | 下胸右悬停(避让) | 0.32 |
| 33 | 摩法·右摩中背右 | 肩左悬停(避让) | 中背右**摩法** | 0.16 |
| 34 | 摩法·左摩上背中 | 上背中**摩法** | 腰椎中悬停(避让) | 0.22 |
| 35 | 摩法·右摩骶中 | C7中悬停(避让) | 骶中**摩法** | 0.46 |
| 36 | 摩法·双摩收 | 肩中**摩法** | 骶中**摩法** | **0.37** |

#### 第五阶段：擦法 Scrub（式37-42）— 直线推擦（新增）

> 沿脊柱中线前后直线推擦，更深更长的擦动

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 37 | 擦法·双悬 | C7中悬停 | 中背中悬停 | 0.25 |
| 38 | 擦法·左擦肩中 | 肩中**擦法** | 中背中悬停(避让) | 0.16 |
| 39 | 擦法·右擦腰中 | 上背中悬停(避让) | 腰椎中**擦法** | 0.22 |
| 40 | 擦法·右擦骶中 | C7中悬停(避让) | 骶中**擦法** | 0.46 |
| 41 | 擦法·左擦肩中 | 肩中**擦法** | 骶中悬停(避让) | 0.37 |
| 42 | 擦法·双擦收 | 上背中**擦法** | 腰椎中**擦法** | **0.22** |

#### 第六阶段：滚揉法 Rolling（式43-48）— 中线滚揉

> 沿脊柱中线从肩到骶骨滚动揉捏，肌肉松解

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 43-44 | 滚揉·肩/下胸 | 肩中滚揉(x=0.47) | 下胸中滚揉(x=0.70,交替) | 0.23 |
| 45-46 | 滚揉·上背/腰椎 | 上背中滚揉(x=0.55) | 腰椎中滚揉(x=0.77,交替) | 0.22 |
| 47 | 滚揉·右滚骶中 | C7中悬停(避让) | 骶中滚揉(x=0.84) | 0.46 |
| 48 | 滚揉·双滚收 | 肩中滚揉(x=0.47) | 腰椎中滚揉(x=0.77) | **0.30** |

#### 第七阶段：振法 Vibration（式49-54）— 快速颤动（新增）

> 保持接触的微小快速关节振荡，每分钟约200-300次频率

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 49 | 振法·双悬 | C7中悬停 | 中背中悬停 | 0.25 |
| 50 | 振法·左振C7中 | C7中**振法** | 下胸中悬停(避让) | 0.32 |
| 51 | 振法·右振中背中 | 肩中悬停(避让) | 中背中**振法** | 0.16 |
| 52 | 振法·左振上背中 | 上背中**振法** | 腰椎中悬停(避让) | 0.22 |
| 53 | 振法·左振C7左 | C7左**振法** | 骶右悬停(避让) | 0.46 |
| 54 | 振法·双振收 | 肩中**振法** | 腰椎中**振法** | **0.30** |

#### 第八阶段：击法+收功（式55-60）— 冷却收功

| 式 | 名称 | 左臂 | 右臂 | ΔX |
|----|------|------|------|-----|
| 55 | 收功·双悬 | C7中悬停 | 骶中悬停 | 0.46 |
| 56 | 击法·左击肩左 | 肩左**击法** | 下胸右悬停(避让) | 0.23 |
| 57 | 击法·右击中背右 | C7左悬停(避让) | 中背右**击法** | 0.25 |
| 58 | 击法·右击腰中 | 上背中悬停(避让) | 腰椎中**击法** | 0.22 |
| 59 | 收功·终末深按 | C7中**深按** | 骶中**深按** | **0.46** |
| 60 | 收功·完成 | C7中释放 | 骶中释放 | — |

### 碰撞与规划说明

- 使用 MoveIt `both_arms` 规划组（12轴联合规划），双臂交叉碰撞检测全程启用
- 每段动作调用 `/plan_kinematic_path`（RRTConnect，5次尝试，3s超时）
- **双臂分区负责**：左臂→中线+左侧(y≤0)，右臂→中线+右侧(y≥0)，双臂永不同X区
- 13种中医推拿手法通过关节偏移(ACT_DELTA)实现：悬停/按/释/揉左/揉右/滚/拍 + 摩左/摩右/擦/振/深按/击
- 关节目标容差 `0.05 rad`（~3°），给IK足够灵活性
- 轨迹采样碰撞检测（0.1s间隔），碰撞点**记录警告但不阻断执行**（非致命模式）
- 规划失败的阶段自动使用线性插值fallback
- RViz中 `PlanningScene Show Scene Geometry=false`（避免绿色方块遮挡人体模型）
- 按摩模式 `use_flange=false`：末端无grip_pad法兰，Link_06直接作为按摩接触面

### 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `repeat_count` | `0` | 执行轮数。`0` 无限循环 |
| `trajectory_time_scale` | `0.65` | 轨迹时间缩放，越小越快（建议 ≥0.45） |
| `velocity_scaling` | `0.45` | MoveIt 规划速度缩放 |
| `acceleration_scaling` | `0.45` | MoveIt 规划加速度缩放 |
| `trajectory_start_delay` | `0.10` | 控制器轨迹开始延迟 |

## JAKA C5 双臂协同搬运框架

> **入口脚本**：[dual_arm_carry_demo.py](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/scripts/dual_arm_carry_demo.py)（1386 行）
> **启动文件**：[carry_object_demo.launch.py](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/launch/carry_object_demo.launch.py)
> **调试历史**：[BUG_SUMMARY.md](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/BUG_SUMMARY.md)（7 个 Bug，全部已修复）

### 运行方式

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_jaka_c5_moveit_config carry_object_demo.launch.py
```

### 整体架构（5 层）

```
┌──────────────────────────────────────────────────────────────────────────┐
│                    carry_object_demo.launch.py                            │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  demo.launch.py (IncludeLaunchDescription)                          │  │
│  │  ┌───────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │  │
│  │  │  move_group    │  │  robot_state_   │  │  rviz2               │   │  │
│  │  │  (MoveIt 核心)  │  │  publisher      │  │  (可视化)            │   │  │
│  │  └───────┬───────┘  └────────┬────────┘  └─────────────────────┘   │  │
│  │          │                   │                                       │  │
│  │  ┌───────┴───────────────────┴───────────────────────────────────┐  │  │
│  │  │  ros2_control_node + 3 个 spawner                              │  │  │
│  │  │  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────┐ │  │  │
│  │  │  │ left_arm_ctrl    │  │ right_arm_ctrl   │  │ joint_state_ │ │  │  │
│  │  │  │ (JTC position)   │  │ (JTC position)   │  │ broadcaster  │ │  │  │
│  │  │  └──────────────────┘  └──────────────────┘  └──────────────┘ │  │  │
│  │  └────────────────────────────────────────────────────────────────┘  │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  TimerAction(4s) → dual_arm_carry_demo.py (业务逻辑节点)           │  │
│  └────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────┘
```

#### 第 1 层：物理模型（URDF/XACRO）

**文件链**: `jaka_c5_dual.urdf.xacro` → `jaka_c5_arm_macro.xacro` → `jaka_c5.ros2_control.xacro`

```
world (根连杆)
├── left_Link_00 ─[J1]─ Link_01 ─[J2]─ Link_02 ─[J3]─ Link_03 ─[J4]─ Link_04 ─[J5]─ Link_05 ─[J6]─ Link_06
│                                                                                                │
│       left_grip_pad ←── [fixed] ── Link_06                                                     │
│       left_grip_contact (虚拟碰撞帧, z=0.012m, 用作接触点 TF 查询)                                │
│
├── right_Link_00 ─[J1]─ Link_01 ─[J2]─ Link_02 ─[J3]─ Link_03 ─[J4]─ Link_04 ─[J5]─ Link_05 ─[J6]─ Link_06
│                                                                                                │
│       right_grip_pad ←── [fixed] ── Link_06                                                    │
│       right_grip_contact (虚拟碰撞帧, z=0.012m)
```

| 设计项 | 说明 |
|--------|------|
| 双臂 XACRO 复用 | `jaka_c5_arm` 宏通过 `prefix` 参数（`left_`/`right_`）生成两套镜像连杆-关节树 |
| 末端法兰条件化 | `use_flange` 参数控制 `grip_pad` + `grip_contact` 是否渲染。搬运 `true`，按摩 `false` |
| grip_pad | 85×85×12mm 扁平盒子，通过 fixed joint 固定在 Link_06。夹持货物侧面的接触面 |
| grip_contact | 虚拟空连杆，定义在 grip_pad **外表面**（z = 0.012m）。`publish_markers()` 通过 TF 查询此帧获取接触点世界坐标 |
| 关节参数 | 6 个 revolute，J1/J5/J6 全范围 ±360°，J2/J4 约 85°–265°，J3 ±175° |
| 碰撞模型 | 使用与 visual 相同的 STL mesh，无简化代理体 |

**双臂位置参数化**（`jaka_c5_dual.urdf.xacro:11-14`）:

| 参数 | 搬运默认值 | 按摩值 | 说明 |
|------|-----------|--------|------|
| left_arm_x | 0 | 0.53 | 左臂X（偏左，近身体左侧） |
| left_arm_y | -0.25 | -0.45 | 左臂Y（床左侧，指向身体左侧y<0） |
| right_arm_x | 0 | 0.69 | 右臂X（偏右，近身体右侧） |
| right_arm_y | 0.25 | 0.45 | 右臂Y（床右侧，指向身体右侧y>0） |

#### 第 2 层：运动学与规划（MoveIt2 SRDF）

**规划组**（[jaka_c5_dual.srdf](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/config/jaka_c5_dual.srdf)）:

| 组名 | 类型 | 关节 | 用途 |
|------|------|------|------|
| `left_arm` | chain | left_joint_1~6 | 单独控制左臂（RViz IK 交互） |
| `right_arm` | chain | right_joint_1~6 | 单独控制右臂 |
| `both_arms` | **joint**（12 轴联合） | 全部 12 个关节 | **搬运规划使用此组** |

**碰撞矩阵**:
- 单臂内部相邻/必然不碰撞对: 各 9 对 disabled
- **双臂交叉碰撞全部启用**：左臂任意连杆 ↔ 右臂任意连杆全部检测（无 disable_collisions 跨臂配置）

**规划参数**:
```yaml
# ompl_planning.yaml
longest_valid_segment_fraction: 0.005   # 强制路径细分（JAKA C5 等效 ~0.063 rad/段）
request_adapters:
  - default_planner_request_adapters/AddTimeOptimalParameterization
planner: RRTConnectkConfigDefault
```
- 运动学求解器: KDL 数值 IK

#### 第 3 层：硬件接口（ros2_control）

**硬件模拟**: `mock_components/GenericSystem` — 命令接口值镜像复制到状态接口（"假执行"）

**控制器**（[ros2_controllers.yaml](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/config/ros2_controllers.yaml)）:
```
controller_manager (100Hz)
├── joint_state_broadcaster      → /joint_states
├── left_arm_controller          → /left_arm_controller/follow_joint_trajectory
│   └── type: JointTrajectoryController (position 模式, 6 关节)
└── right_arm_controller         → /right_arm_controller/follow_joint_trajectory
    └── type: JointTrajectoryController (position 模式, 6 关节)
```

**MoveIt 控制器管理**（[moveit_controllers.yaml](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/config/moveit_controllers.yaml)）:
```
moveit_simple_controller_manager/MoveItSimpleControllerManager
├── left_arm_controller → FollowJointTrajectory action
└── right_arm_controller → FollowJointTrajectory action
```
- 执行容差: `allowed_execution_duration_scaling: 1.2` + margin 0.5s
- 起始容差: `allowed_start_tolerance: 0.01 rad`

#### 第 4 层：启动编排（launch）

`carry_object_demo.launch.py` → IncludeLaunchDescription → `demo.launch.py`:

```
启动顺序:
1. robot_state_publisher        → 发布 /robot_description + /tf_static
2. ros2_control_node            → 加载硬件接口
3. joint_state_broadcaster      → 发布 /joint_states
4. left_arm_controller spawner  → 启动左臂 JTC
5. right_arm_controller spawner → 启动右臂 JTC
6. move_group                   → MoveIt 核心（规划 + 执行服务）
7. rviz2                        → 可视化
8. [4 秒延迟] carry_demo 脚本   → 开始搬运业务逻辑
```

#### 第 5 层：业务逻辑（dual_arm_carry_demo.py）

详见下方「搬运流程」章节。

### 搬运流程（5 阶段状态机）

```
          ┌─────────────────────────────────────────┐
          │        PHASE 0: 启动等待 (start())       │
          │  等待 Action Server + 关节稳定 +         │
          │  桌面碰撞体注册到 MoveIt PlanningScene    │
          └───────────────────┬─────────────────────┘
                              │
                              ▼
          ┌─────────────────────────────────────────┐
          │      PHASE 1: 轨迹规划 (14 waypoints)    │
          │  MoveIt 逐段规划 → 拼接 → 拆分双臂轨迹    │
          └───────────────────┬─────────────────────┘
                              │
                              ▼
          ┌─────────────────────────────────────────┐
          │    PHASE 2: 轨迹验证 (采样碰撞检测)       │
          │  /check_state_validity (0.1s 采样)       │
          │  + FK 夹持几何验证                        │
          └───────────────────┬─────────────────────┘
                              │
                              ▼
          ┌─────────────────────────────────────────┐
          │   PHASE 3: 双臂同步执行 (异步 Action)     │
          │  left/right Action 并行发送 → 等待 done   │
          └───────────────────┬─────────────────────┘
                              │
          ┌───────────────────┴─────────────────────┐
          │      PHASE 4: 物理模拟 (50Hz timer)      │
          │  ┌─────────┐   ┌─────────┐   ┌────────┐ │
          │  │  FREE   │──▶│ GRASPED │──▶│ PLACED │ │
          │  │ (重力)  │   │ (跟随臂) │   │ (桌面) │ │
          │  └─────────┘   └─────────┘   └────────┘ │
          └─────────────────────────────────────────┘
```

#### Phase 0：启动等待

```
1. 等待 left/right Action Server 就绪（默认 120s 超时）
2. 轮询 /joint_states，检测 12 个关节位置稳定
   （STARTUP_STABLE_SECONDS=0.6s 内最大偏移 ≤ 0.002 rad）
3. 验证 TF：left_grip_contact / right_grip_contact 在 world 帧中可见
4. 验证起始关节角与 LEFT_WAYPOINTS[0] + RIGHT_WAYPOINTS[0] 偏差 ≤ 0.04 rad
5. 调用 /apply_planning_scene 注册桌面碰撞体（1 桌面 box + 4 桌腿 box）
```

#### Phase 1：轨迹规划（14 个 Waypoints）

**混合规划策略**：接近/释放阶段用 MoveIt 完整规划，夹持搬运阶段用线性插值。

```
时刻     阶段                    关节规划方式
──────   ──────────────────────  ──────────────────────────
 0.0s   起始位姿（双臂分开）      (起点, 不规划)
 1.0s   双臂向中间靠拢            MoveIt RRTConnect 关节规划
 4.5s   夹持接触点闭合 ← 抓取点   MoveIt RRTConnect 关节规划
 ────────────── grasp_enable_time ─────────────────────────
 5.2s   夹持上升阶段 1            locked-grip 线性插值
 5.9s   夹持上升阶段 2            locked-grip 线性插值
 6.6s   夹持上升阶段 3            locked-grip 线性插值
 7.3s   夹持上升阶段 4            locked-grip 线性插值
 8.0s   夹持上升完成（最高点）     locked-grip 线性插值
 9.0s   水平搬运至桌面上方         MoveIt RRTConnect 关节规划
10.0s   下降至桌面                 MoveIt RRTConnect 关节规划
11.5s   释放阶段 1（微张开）       MoveIt RRTConnect 关节规划
13.0s   释放阶段 2（保持）         MoveIt RRTConnect 关节规划
14.0s   释放阶段 3（保持）         MoveIt RRTConnect 关节规划
15.5s   双臂收回                   MoveIt RRTConnect 关节规划
```

**两种规划模式**:

| 模式 | 适用阶段 | 实现 |
|------|---------|------|
| **MoveIt 完整规划** | 阶段 0→2, 9→14 | 调用 `/plan_kinematic_path`，RRTConnect，10 次尝试，8s 超时，速度/加速度缩放 0.15 |
| **Locked-grip 插值** | 阶段 3→8（夹持搬运中） | 跳过 MoveIt 规划，**线性插值**生成轨迹点。保证双臂间距恒定，避免碰撞检测过度保守 |

> **设计意图**: 夹持搬运时货物在两臂之间，MoveIt 可能因碰撞检测过于保守而规划失败。Locked-grip 确保双臂**同步运动**，维持夹持间距不变。

#### Phase 2：轨迹验证

对拼接后的完整轨迹采样（每 0.1s），逐点验证：

```
采样点验证：
├── /check_state_validity（MoveIt 碰撞检测）
├── FK 计算左右 grip_contact 世界位姿
├── 夹持阶段额外检查 (_validate_grip_contact_geometry):
│   ├── 接触点间距 ∈ [0.325, 0.370] m（货物宽度 0.345 ± 公差）
│   ├── 夹持面法向量指向货物侧面 (dot > 0.90)
│   └── 夹持面垂直度 (dot(world_up) > 0.85)
└── _validate_cargo_table_clearance:
    └── 货物底部不穿透桌面 (bottom_z ≥ TABLE_TOP_Z - 0.005)
```

#### Phase 3：轨迹发送

```python
# 1. 拆分 12 轴联合轨迹 → left/right 各 6 轴
left_trajectory, right_trajectory = _split_combined_trajectory(planned)

# 2. 异步并行发送（不等待完成）
left_client.send_goal_async(left_goal)
right_client.send_goal_async(right_goal)

# 3. pending_results = 2，每个完成时 -1，归零时日志输出
# 4. 时间偏移 TRAJECTORY_START_DELAY = 0.5s 加入每个轨迹点
```

#### Phase 4：货物物理模拟 + RViz 可视化（50Hz）

**货物状态机**:

```
                      can_grasp(): 间距∈[0.335,0.350] & 中心误差≤0.16m
      ┌─────────────────────────────────────────────────────┐
      │                                                     │
      ▼                                                     │
  ┌───────┐        双臂合拢夹住                         ┌─────────┐
  │ FREE  │────────────────────────────────────────────▶│ GRASPED │
  │ (重力) │                                            │ (跟随臂) │
  └───────┘                                            └─────────┘
      ▲                                                     │
      │    间距>0.365 或 timed_release                      │
      └─────────────────────────────────────────────────────┘
                                                            │
                                                ┌───────────┘
                                                │ cargo 在桌面上方
                                                ▼
                                          ┌────────┐
                                          │ PLACED │
                                          │ (桌面)  │
                                          └────────┘
```

**捕获/保持/释放条件**:

| 条件 | 函数 | 阈值 |
|------|------|------|
| 捕获 | `_can_grasp()` | 间距 ∈ [0.335, 0.350]m, 中心误差 ≤ 0.16m, z ≥ 0.054m |
| 保持 | `_can_hold()` | 间距 ∈ [0.335, 0.365]m |
| 定时释放 | `elapsed ≥ total_duration + 0.8s` | release_delay_seconds = 0.8 |
| 意外脱手 | 间距 > 0.365m | grip_distance_max |

**物理模拟**:
- **FREE**: 货物受重力自由落体 (`vz -= g*dt`)，碰桌面/地面停止并衰减水平速度
- **GRASPED**: 货物位姿 = 左右 grip_contact 中点 + 方向矩阵，完全跟随机械臂
- **PLACED**: 货物固定在 `TABLE_TOP_Z + CARGO_SIZE_Z/2`，速度归零

**RViz Marker 清单**（7 种，发布在 `/rviz_visual_tools`）:

| Marker ID | 类型 | 内容 | 颜色规则 |
|-----------|------|------|----------|
| 1 | CUBE | 货物本体 (0.18×0.345×0.12m) | GRASPED=蓝, 其他=橙 |
| 2 | SPHERE | 左 grip_contact 接触点 (r=0.035) | GRASPED=绿, 其他=黄 |
| 3 | SPHERE | 右 grip_contact 接触点 (r=0.035) | GRASPED=绿, 其他=黄 |
| 4 | LINE_STRIP | 货物运动轨迹（最近 300 帧）| 青色 |
| 5 | LINE_STRIP | 两接触点连线 | GRASPED=绿, 其他=灰 |
| 6 | TEXT | 状态文字 "GRASPED"/"PLACED"/"FREE" | 白色 |
| 7 | CUBE | 桌面 (0.75×0.70×0.04m) | 棕色 |

### 场景布局

```
                    世界坐标系 (俯视图, X 朝右, Y 朝上)

                              ┌─────────────────┐
                              │    桌面          │
                              │  0.75 × 0.70    │  ← TABLE_X = 0.90
                              │  中心 (0.90, 0)  │
                              └─────────────────┘
                                       │
                                       │
                         ┌─────────────┴─────────────┐
                         │                           │
                    ┌────┴────┐                 ┌────┴────┐
                    │ 货物起点 │                 │         │
                    │ (0.36,   │                 │         │
                    │  0.02)   │                 │         │
                    └─────────┘                 │         │
                                                │         │
             左臂底座 ●                          │         ● 右臂底座
           (0, -0.25)                           │    (0, 0.25)
                                                │
```

| 对象 | 位置 | 尺寸 |
|------|------|------|
| 货物（起点） | (0.36, 0.02, 0.06) | 0.18 × 0.345 × 0.12 m |
| 桌面 | (0.90, 0, 0.28) | 0.75 × 0.70 × 0.04 m |
| 左臂底座 | (0, -0.25, 0) | — |
| 右臂底座 | (0, 0.25, 0) | — |

### 关键数据流

```
┌──────────────────┐     /joint_states         ┌──────────────────────┐
│ ros2_control     │ ─────────────────────────▶│ dual_arm_carry_demo  │
│ (mock hardware)  │                           │ _on_joint_state()    │
└──────────────────┘                           └──────────┬───────────┘
                                                          │
┌──────────────────┐     TF (/tf, /tf_static)             │ FK 验证
│ robot_state_     │ ─────────────────────────▶ ┌─────────▼───────────┐
│ publisher        │                            │ tf2_ros.Buffer      │
└──────────────────┘                            │ → left/right_contact│
                                                │   变换查询           │
                                                └──────────┬──────────┘
                                                           │
┌──────────────────┐     /plan_kinematic_path               │
│ move_group       │ ◀───────────────────── ┌──────────────▼──────────┐
│ (RRTConnect)     │                        │ _plan_joint_segment()   │
└──────────────────┘                        └─────────────────────────┘

┌──────────────────┐     /left_arm_controller/follow_joint_trajectory
│ ros2_control     │ ◀───────────────────── left_trajectory (异步)
│ (JTC)            │     /right_arm_controller/follow_joint_trajectory
└──────────────────┘ ◀───────────────────── right_trajectory (异步)
```

### Gazebo 版本（可选）

项目同时提供了 Gazebo Classic 版本的搬运 Demo：[gazebo_carry_demo.py](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/scripts/gazebo_carry_demo.py)

| 维度 | RViz 版本 | Gazebo 版本 |
|------|----------|-------------|
| 货物物理 | 自定义简单物理（重力+碰撞） | Gazebo 物理引擎 |
| 货物渲染 | `/rviz_visual_tools` MarkerArray | Gazebo EntityState 同步 |
| 臂位姿来源 | `/tf` 实时查询 `grip_contact` | FK 数学库 + Gazebo SetEntityState |
| 关节同步 | mock_components（被动跟随） | FK → Gazebo SetEntityState（每 100ms） |

Gazebo 版本额外通过 FK 计算 14 个连杆位姿（每臂 7 个: Link_00~Link_06），每 100ms 调用 `/gazebo/set_entity_state` 同步到 Gazebo 场景。

### 核心设计模式

| 模式 | 说明 |
|------|------|
| **混合规划** | 接近/释放阶段用 MoveIt RRTConnect 完整规划（避碰），夹持搬运阶段用 locked-grip 线性插值（保证同步） |
| **先规划后验证** | 完整轨迹拼接后采样 0.1s 逐点调用 `/check_state_validity` + FK 夹持几何验证 |
| **双臂松散耦合** | 12 轴联合规划 → 拆分为两条 6 轴轨迹 → 各自独立发送 action（异步并行） |
| **物理模拟解耦** | 货物状态机（free/grasped/placed）完全由脚本计算，不依赖 Gazebo |
| **参数化分离** | arm 位置通过 xacro:arg 参数化，搬运 vs 按摩场景切换只需改 arg 值 |

### BUG_SUMMARY 速览

> 详见 [BUG_SUMMARY.md](src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/BUG_SUMMARY.md)

| Bug | 类别 | 严重度 | 修复方式 |
|-----|------|--------|----------|
| AddTimeOptimalParameterization 命名空间错误 | 配置 | 🔴 阻塞 | 移到 request_adapters + 正确命名空间 |
| 路径验证发现隐藏碰撞（双臂交叉穿透） | 配置 | 🔴 阻塞 | `longest_valid_segment_fraction: 0.005` |
| 交互标记无法平移 | 配置 | 🟡 中等 | 删除不完整的 end effector 定义 |
| CHOMP 被误选为默认规划器 | 依赖 | 🟡 中等 | 卸载 CHOMP 包 |
| YAML 参数格式错误（move_group 崩溃） | 配置 | 🔴 崩溃 | 改用 Python 字典传参 |
| Ros2ControlManager 加载失败 | 插件 | 🔴 崩溃 | 改用 MoveItSimpleControllerManager |
| OMPL 配置名无效 | 配置 | 🟡 中等 | 删除无效的 default_planner_config |

## 单臂 Pick-and-Place 开发日志

> 最后更新：2026-06-22 | 详见 [BUG_SUMMARY.md](src/moveit_resources-ros2/single_arm_jaka_c5_pick_place/BUG_SUMMARY.md)

本节记录 JAKA C5 单臂水果抓取 Demo 从零到可用的完整调试过程。

### 项目背景

- **目标**：JAKA C5 单臂从桌面抓取 4 个彩色水果（apple/orange/plum/lime），放入料框
- **技术栈**：ROS2 Humble + MoveIt2 + OMPL RRTConnect + KDL IK 求解器
- **硬件模拟**：`mock_components/GenericSystem`（关节命令镜像到状态）
- **控制器**：`joint_trajectory_controller/JointTrajectoryController`（8 轴：6 转动 + 2 夹爪）
- **可视化**：RViz2（RobotModel + TF + MarkerArray，不含 MotionPlanning 插件）

### Bug 清单

| # | 问题 | 严重度 | 状态 | 根因类别 |
|---|------|--------|------|----------|
| 1 | 机械臂只规划不执行 | 🔴 阻塞 | ✅ 已解决 | RViz 渲染（TF 禁用 + MotionPlanning 插件冲突） |
| 2 | 水果小球不显示 | 🟡 中等 | ✅ 已解决 | MotionPlanning 碰撞物体遮挡 |
| 3 | RViz 无法拖动视角 | 🟡 中等 | ✅ 已解决 | MotionPlanning 交互标记捕获鼠标 |
| 4 | Orange IK 求解失败 (code=-31) | 🔴 阻塞 | ✅ 已解决 | 目标位置超出工作半径 |
| 5 | RViz 机械臂 3D 模型不渲染 | 🔴 阻塞 | ✅ 已解决 | QoS TRANSIENT_LOCAL 不匹配 |
| 6 | 水果抓取仿真效果差 | 🟡 中等 | ✅ 已解决 | 缺少 TF 跟踪 + 刚体附着 |

### Bug 1：机械臂只规划不执行

**现象**：终端日志中 MoveIt 规划成功、controller goal accepted，但 RViz 中机械臂不动。

**诊断过程**：
1. 检查 `/joint_states` → 正常发布，关节角度在变化（如 J6: 0→-1.781 rad）
2. 检查 `arm_controller` → `Goal reached, success!`
3. 确认机械臂**实际上在运动**，问题出在 RViz 渲染

**根因（两重）**：
1. `pick_place.rviz` 中 **TF display 被禁用** → RobotModel 无法计算连杆位姿
2. **MotionPlanning 插件即使在 `Enabled: false` 状态下仍会在后台异步初始化** `planning_scene_monitor`、`MoveGroup`、`interactive_marker_display`，其 `PlanningScene` 渲染加载后与独立 `RobotModel` 冲突导致 RViz 渲染线程冻结

**修复**：
- 启用 TF display + RobotModel display
- **完全移除** MotionPlanning 显示插件（非仅禁用）—— 关键突破
- RViz 中只保留 4 个干净显示：TF / Grid / RobotModel / MarkerArray

### Bug 2：水果小球不显示

**现象**：之前只能看到 MotionPlanning 渲染的绿色碰撞球体，看不到 `/rviz_visual_tools` 发布的彩色 Marker。

**根因**：MotionPlanning 插件的 PlanningScene 碰撞几何体渲染层遮挡了 MarkerArray。

**修复**：移除 MotionPlanning 后障碍消除；`_publish_markers()` 每 0.2s 发布彩色球体，颜色随状态变化。

### Bug 3：RViz 无法拖动视角

**现象**：鼠标左键点击机器人模型时被捕获，无法 Orbit 旋转视角。

**根因**：MotionPlanning 的 `InteractiveMarkerDisplay` 在初始化时加载末端拖动手柄，点击模型时捕获鼠标事件。

**修复**：移除 MotionPlanning 插件 → 无交互标记 → 鼠标恢复正常。后续还补充了 Tools 面板（MoveCamera/Interact/Select）和 Orbit 视图。

### Bug 4：Orange IK 求解失败

**现象**：
```
[pick_place_demo]: IK failed: code=-31
[pick_place_demo]: IK failed for hover_fruit_orange, skip
```
Apple、Plum、Lime 的 IK 全部成功，唯独 Orange 失败。

**根因**：Orange 原位置 `(x=0.70, y=0.10)` 距离底座太远。JAKA C5 工作半径约 0.7m，加上 tool_flange 的 Z 偏移，末端姿态要求 Z 轴向下指向桌面，使该位置超出有效 IK 范围。

**修复**：Orange 从 `(0.70, 0.10)` 移到 `(0.55, -0.05)`，同步更新 `simulated_camera.py`。

### Bug 5：RViz 机械臂 3D 模型不渲染

**现象**：RViz 中可以看到 TF 坐标系（Link_01、Link_02...）在运动，但机械臂的 STL 三维模型完全不显示。

**诊断过程**：对比双臂正常工作的 `carry_demo.rviz` 配置，发现 RobotModel 的 `Description Topic` 缺少显式 QoS 声明。

**根因**：`robot_state_publisher` 以 `TRANSIENT_LOCAL`（latched）模式发布 `/robot_description`，但 RViz RobotModel display 的 Description Topic 默认使用 `VOLATILE` QoS，订阅者收不到已 latched 的消息。这与此前 `/tf_static` 的问题完全同源。

**修复**：`pick_place.rviz` 中 RobotModel 的 Description Topic 显式指定：
```yaml
Description Topic:
  Depth: 5
  Durability Policy: Transient Local
  History Policy: Keep Last
  Reliability Policy: Reliable
  Value: /robot_description
```

### Bug 6：水果抓取仿真效果差

**现象（用户反馈）**：
- 绿色水果夹取后缩小但不消失，残留在桌面
- 已放入料框的水果在桌面留下残影
- 水果标记完全不跟随机械臂运动

**根因**：`_publish_markers()` 在水果状态变化时只调整了 alpha（透明度）和 scale（大小），标记始终发布在桌面原始坐标。

**修复（三处改动）**：
1. **引入 TF 监听**：`tf2_ros.Buffer + TransformListener` 每 0.05s 查询 `tool_flange → world` 变换
2. **记录抓取偏移**：抓取瞬间计算 `fruit_world − tool_flange_world`，存入 `grasped_fruit_offsets`
3. **按状态渲染**：
   - `free` → 桌面原位，α=0.90
   - `grasped` → `tool_flange_world + offset`（跟随夹爪运动），α=0.90
   - `placed` → 料框底部，α=0.55（无残影）

### 关键技术教训

1. **ROS2 QoS 陷阱**：`TRANSIENT_LOCAL`（latched）发布者与 `VOLATILE`（默认）订阅者不兼容。`/tf_static` 和 `/robot_description` 都受此影响。修复方法是在订阅端显式声明 `DurabilityPolicy.TRANSIENT_LOCAL`。

2. **MotionPlanning 插件副作用**：`moveit_rviz_plugin/MotionPlanning` 即使设为 `Enabled: false`，其内部组件（planning_scene_monitor、MoveGroup、interactive_marker_display）仍会在后台异步初始化。与独立 RobotModel 共存时会导致渲染冲突。唯一安全的处理方式是**完全移除**该显示插件。

3. **关节方向约定**：`joint_2 > 0` → 大臂后仰（安全方向），`joint_3 < 0` → 肘部前弯。HOME 位姿 `[0, 1.50, -1.50, 1.50, 1.57, 0]` 确保机械臂初始姿态远离桌面和料框。

4. **诊断日志设计**：在 `_execute()` 中打印轨迹点数、时长、起点/终点 delta，能快速区分"规划了但不动"和"根本没规划"两种场景。退化轨迹检测（终点 Δ<0.005rad 警告）帮助发现无效规划。

### 当前状态（2026-06-22）

- ✅ 6 个 Bug 全部在代码层面修复
- ✅ 终端日志验证通过：4 个水果全程 `Goal reached, success!`
- 🟡 待用户在 RViz 中目视确认：3D 模型渲染、水果跟随抓取、视角拖动

### 快速验证命令

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --packages-select single_arm_jaka_c5_pick_place
source install/setup.bash
ros2 launch single_arm_jaka_c5_pick_place pick_place_demo.launch.py
```

## U 盘拷贝指南

为了避免程序拷贝到 U 盘后出现文件为空或数据没有完整写入的情况，请按下面三步操作。

1. 拷贝程序到 U 盘：

   ```bash
   sudo cp -r /home/cat/lab_plant /media/usb1/
   ```

2. 强制将缓存数据写入磁盘：

   ```bash
   sync
   ```

   `sync` 会把内存中尚未写入磁盘的数据立即刷到 U 盘。执行后请稍等片刻，确认 U 盘指示灯停止闪烁。

3. 正确卸载 U 盘：

   ```bash
   sudo umount /media/usb1
   ```

   只有在 `umount` 命令执行完成且没有任何报错后，才安全拔出 U 盘。如果命令提示设备正忙，请先关闭正在访问 U 盘目录的终端、文件管理器或程序，再重新执行卸载命令。

## Notes

The benchmarking resources have been moved to https://github.com/ros-planning/moveit_benchmark_resources.
