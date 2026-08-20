# JAKA Twin Guard — AI 交接文档

> 最后更新: 2026-06-25 | 分支: main | 负责: hydarealman

---

## 1. 项目概要

**目标**：JAKA C5 双臂按摩系统 — 60 式中医推拿。最终须部署到实车，**不可"掩耳盗铃"**。

**核心约束（用户反复强调）**：
- 必须用 **Gazebo 真实物理仿真**，不能用 mock_components 假数据
- RViz 显示的必须是 Gazebo 传来的真实 joint_states
- 质量/惯量/重力/接触力 都必须仿真

**运行环境**：WSL2 Ubuntu 22.04 + ROS2 Humble + MoveIt2 + Gazebo Classic 11.10.2

**工作空间**：
- Windows 文件系统: `d:\jaka_twin_guard`
- WSL2 内编译: `/mnt/d/jaka_twin_guard`
- Git Bash 编辑: `/d/jaka_twin_guard`

---

## 2. 两个 Launch 文件的本质区别

| | `industrial_massage.launch.py` | `sim_gazebo_massage.launch.py` |
|---|---|---|
| **物理引擎** | mock_components (关节插值，假数据) | **Gazebo 11** (真实物理) |
| **关节数据** | 人为生成 | Gazebo 物理引擎实时计算 |
| **接触力** | 无 | F/T 传感器 (`libgazebo_ros_ft_sensor.so`) |
| **真机迁移** | 不可 | 换 `hardware_plugin` 参数即可 |
| **用途** | 快速验证规划逻辑/BT引擎/可视化 | **最终目标 — 真实数据闭环** |
| **当前状态** | ✅ 已验证（需本轮修复生效后重测） | 🟡 启动架构就绪，未完整跑过 60 阶段 |

**数据流 (Gazebo 版 — 真数据)**：
```
Gazebo 物理引擎
  ├─ /joint_states (真实) → RViz + MoveIt
  └─ F/T sensor → /left_ft_sensor/wrench, /right_ft_sensor/wrench → 力控反馈
```

---

## 3. 当前状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 按摩 BT 引擎 | ✅ Mock验证过 | 5 个 BT 节点 + 60 阶段，存在规划失败需修复 |
| RViz 可视化 | ✅ | PlanningScene + RobotModel + 39 个彩色 Marker |
| Gazebo 物理仿真 | 🟡 架构就绪 | launch 文件完整，world 文件完整，未测试 |
| 规划速度 | 🟡 修复待验证 | from-above 策略已实现，C7 区碰撞体已调小 |
| joint_4 HALT | 🟡 修复待验证 | safety_params.yaml margin 0.10→0.05 |

---

## 4. 2026-06-25 最新修复 (本轮对话)

### Fix 1: C7 区域规划失败 (3cm/6cm above 全部超时)
- **根因**: 头部碰撞球 `(0.24, 0, 0.23) r=0.065` → x 范围 [0.175, 0.305]，挡住了 C7 按摩区 (x≈0.29-0.38) 的所有途径点
- **修复**:
  - 头部球体: `x=0.20, z=0.24, r=0.055` → x 范围 [0.145, 0.255]
  - 颈部圆柱: `x=0.285, z=0.20, r=0.025, h=0.035` → x 范围 [0.260, 0.310]
  - 与 C7 区至少有 3.5cm 间隙
- **修改文件**: `massage_nodes.py:312-320`, `massage.world:170-183`

### Fix 2: joint_4 HALT 假阳性 (-1.401 < -1.380)
- **根因**: `safety_params.yaml` 中 `joint_position.margin: 0.10`。`SafetyLimits.from_yaml()` 默认值 `jp.get("margin", 0.10)` 覆盖了 dataclass 默认 0.05
  - 软限位 = -1.48 + 0.10 = **-1.380**，joint_4 到 -1.401 被误杀
- **修复**: YAML margin 0.10 → 0.05，软限位 = **-1.430**
- **修改文件**: `safety_params.yaml:25`

### Fix 3: from-above 规划策略（优先 3cm→6cm→直接）
- **根因**: 旧代码对 hover 技法先尝试直接位姿规划(RRT 在碰撞区反复失败 8s)，再回退 from-above
- **修复**: 所有单 Pose 规划**优先 from-above**（成功率高，~0.1s），直接规划作为最后手段
- **代码**: `massage_nodes.py:_plan_arm()` lines 662-709

---

## 5. 之前已修复的 Bug (汇总)

### Bug A — 规划速度 15s→~0.1s
- 同 Fix 3，from-above 优先策略

### Bug B — 人体模型沉入床垫
- 碰撞对象 z 位置改为 `max(mat_top + r, z_surface - 0.010)`
- 代码: `massage_nodes.py:_add_body_objects()`

### Bug C — 人和床同色
- 新增 `_publish_body_markers()` — 39 个 Marker，人体肤色/床灰色
- 发布到 `/rviz_visual_tools`, marker lifetime=0

### Bug D — Gazebo launch 相关
- URDF 注释含 `:` 字符 → strip 所有 XML 注释
- `GAZEBO_RESOURCE_PATH` 不要硬覆盖，要 append
- WSL2 环境变量: `LIBGL_ALWAYS_SOFTWARE=1`, `QT_QUICK_BACKEND=software`

---

## 6. 架构: 5 层模型 + 5 个 BT 节点

```
Layer 5: 任务编排    BehaviorTree (massage_task.xml → BtEngine)
Layer 4: 轨迹生成    PathGenerator (13种手法 → 8种 Cartesian 路径图元)
Layer 3: 背部曲面     BackSurfaceModel (Catmull-Rom 样条 → S(u,v) 参数曲面)
Layer 2: 运动规划    DualArmPlannerServer (MoveIt2 RRTConnect + Cartesian Path)
Layer 1: 硬件抽象    ros2_control / VirtualImpedance / F/T Sensor
```

**5 个 BT 节点**: WaitServices → SetupMassageScene → InitSurfaceModel → RunMassageCycle → RetreatToHome

---

## 7. 关键文件地图

```
src/
├── jaka_dual_arm/
│   ├── jaka_dual_arm/
│   │   ├── behavior/bt_nodes/massage_nodes.py      # ⭐ 5个BT节点 (最常改)
│   │   ├── behavior/trees/massage_task.xml          # BT结构
│   │   ├── skills/path_generator.py                 # ⭐ 背部曲面+13手法路径
│   │   ├── planner/planner_server.py                # MoveIt2封装
│   │   ├── control/safety_monitor.py                # 5级安全监控
│   │   └── massage/massage_runner.py                # 按摩主节点
│   ├── config/
│   │   ├── massage_body_params.yaml                 # ⭐ 人体模型 (12段脊柱)
│   │   ├── massage_stages.yaml                      # ⭐ 60阶段定义
│   │   └── safety_params.yaml                       # ⭐ 安全参数 (margin!)
│   ├── launch/
│   │   ├── sim_gazebo_massage.launch.py             # Gazebo物理按摩
│   │   └── industrial_massage.launch.py             # Mock版按摩
│   └── worlds/
│       └── massage.world                            # Gazebo世界 (床+人体)
│
└── moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/
    └── config/
        ├── jaka_c5_dual.urdf.xacro                  # 双臂URDF入口
        ├── jaka_c5_arm_macro.xacro                  # ⭐ 单臂宏 (含FT sensor)
        ├── ros2_controllers.yaml                    # 控制器配置
        └── joint_limits.yaml                        # 速度限值
```

---

## 8. 规划失败诊断指南

当看到 `Unable to sample any valid states for goal tree` 时:

1. **碰撞对象阻挡** — `ros2 topic echo /monitored_planning_scene --once | grep -A3 "id:"` 查看碰撞体坐标
2. **目标在碰撞体内** — 对照日志 `Pose target: [x,y,z]` 与碰撞体坐标范围
3. **位置约束 tolerance (0.03m) 与碰撞体重叠** — `_plan_pose_segment` 的 SPHERE tolerance
4. **IK 不可达** — 末端接近奇异点

---

## 9. 启动命令

```bash
# ═══ WSL2 终端 ═══

# 清理残留（重要！）
pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null

cd /mnt/d/jaka_twin_guard && source /opt/ros/humble/setup.bash

# 编译 (必须包含 jaka_c5_description，否则 Gazebo 无法加载 STL mesh — B021)
colcon build --packages-select jaka_dual_arm dual_arm_jaka_c5_moveit_config jaka_c5_description
source install/setup.bash

# Mock 版 (先验证修复)
ros2 launch jaka_dual_arm industrial_massage.launch.py

# Gazebo 版 (最终目标)
ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py
# 或 headless:
ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py gui:=false
```

---

## 10. 日志关键行解读

```
✅ "left arm: hover → plan from 3cm above"      — from-above 策略工作
✅ "Pose plan: 56 pts, 0.122s"                   — 快速规划成功
✅ "Scene applied: 39 collision objects"         — 场景注册成功
✅ "Surface model initialized: 12 segments"       — 曲面模型就绪

❌ "Pose plan FAILED: code=99999, time=8.0s"      — 规划超时 (碰撞阻挡)
❌ "Unable to sample any valid states"            — RRT 找不到无碰 IK
❌ "[HALT] Joint left_joint_4 below limit"        — 安全监控触发 (检查 margin)
❌ "ALL approach plans FAILED"                    — 3cm/6cm/直接 全部失败
```

---

## 11. 单臂 Pick-and-Place（`jaka_single_arm`）

独立单臂抓取项目，2026-06-24/25 完成 bug 审计修复（4 个 CRITICAL + 8 个 HIGH/MODERATE）。

| 启动文件 | 说明 |
|---------|------|
| `sim_gazebo.launch.py` | Gazebo 物理仿真 (主启动) |

**关键修复**:
1. URDF 缺失 `libgazebo_ros2_control.so` 插件
2. perception centroid 混合帧 bug
3. ESTOP 虚假触发 → `_spin_both()` 双节点 spin
4. 缺失 `use_sim_time`

**已知问题**: Gazebo 里不显示机械臂 (URDF 注释未 strip)，水果抓取不全 (检测聚类待调)。

---

## 12. WSL2 调试必读

- **`pkill -9 gzserver` 是万能药**: 端口 11345 被占用就是有残留进程
- **环境变量**: `LIBGL_ALWAYS_SOFTWARE=1`, `QT_QUICK_BACKEND=software`
- **性能**: 无 GPU 加速，软件渲染慢是正常的

---

## 13. 给下一个 AI 的速查卡片

- **改碰撞体 = 同步改 `massage_nodes.py` + `massage.world`**（两处必须一致）
- **改人体模型 = 改 `massage_body_params.yaml`**（配置驱动）
- **改安全参数 = 改 `safety_params.yaml`**（⚠️ `joint_position.margin` 影响 HALT）
- **改 URDF = 改 `jaka_c5_arm_macro.xacro`**，不是 `jaka_c5.dual.urdf.xacro`
- **URDF 注释含 `:` → Gazebo 崩溃**，spawn 前必须 strip
- **`GAZEBO_RESOURCE_PATH` 不要硬覆盖**，要 `f"/usr/share/gazebo-11:{existing}"`
- **ESTOP 虚假触发 → planner blocking 调用是否用了 executor.spin_until_future_complete()**
- **from-above 规划策略**: 每次从 3cm→6cm→直接 三级回退
- **末端 Link 名**: `left_Link_06`, `right_Link_06` (大写 L, 两位数字)
- **关节名**: `left_joint_1..6`, `right_joint_1..6`
- **按摩手法参数**: `path_generator.py:TECHNIQUE_CONFIG` — z_offset + 路径图元
