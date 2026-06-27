# Gazebo + ros2_control 位置控制稳定性指南

> 本文档记录 JAKA Twin Guard 项目在 Gazebo Classic 仿真中遇到的位置控制稳定性问题、
> 根因分析、工业界解决方案调研，以及我们的实际修复方案。
> 最后更新: 2026-06-26 | 对应平台: ROS2 Humble + Gazebo Classic 11

---

## 目录

1. [问题现象链](#1-问题现象链)
2. [根因分析（6 层）](#2-根因分析6-层)
3. [工业界方案调研](#3-工业界方案调研)
4. [我们的修复方案](#4-我们的修复方案)
5. [参数调优指南](#5-参数调优指南)
6. [诊断工具与方法](#6-诊断工具与方法)
7. [常见问答](#7-常见问答)
8. [参考资源](#8-参考资源)

---

## 1. 问题现象链

### 症状 1: 关节速度尖峰 → HALT

```
[Stage 2] left_joint_2 velocity CRITICAL: 2.26 > 1.57 rad/s
[Stage 2] left_joint_5 velocity CRITICAL: 7.32 > 1.57 rad/s
[Stage 2] left_joint_6 velocity CRITICAL: 18.75 > 1.57 rad/s
→ Safety HALT at stage 3, skipping stage
```

所有后续阶段全部 HALT skip，即使机械臂已经物理静止。

### 症状 2: JTC 容差失败 → ABORT

```
State tolerances failed for joint 4:
  Position Error: -5.254339, Position Tolerance: 0.300000
State tolerances failed for joint 5:
  Position Error: 1.678273, Position Tolerance: 0.300000
→ Aborted due to state tolerance violation
```

Joint 4 的位置误差高达 5.25 rad ≈ 300°，远超 0.30 rad 容差。

### 症状 3: 追踪执行时间与预期不符

规划 8s + 执行 > 50s（预期约 15s），其中大量时间消耗在因冗余路径点导致的慢速段上。

---

## 2. 根因分析（6 层）

### 第 1 层: 轨迹点密度过高

MoveIt 的 `AddTimeOptimalParameterization` 规划适配器会对路径进行高密度采样。
252 个路径点用于一个单次按压运动，相邻路径点之间的关节角变化量有时只有
0.001-0.003 rad。

```
问题: JTC spline 插值通过多个几乎重合的路径点时，
      样条曲线在数值上容易产生病态振荡（龙格现象）。
```

### 第 2 层: 速度/加速度清零破坏样条插值

`_stabilize_trajectory()` 清零了 `velocities` 和 `accelerations` 字段。

```
后果: JTC 收到只有 positions 的 JointTrajectoryPoint，
      被迫使用 "linear sub-strategy" —— 假设每个路径点速度为 0。
      这意味着在每个路径点之间是 "起停" 运动。
      
      当路径点间距极小（0.001 rad）时，"起停" 产生高频小幅度振荡。
```

**参考**: [ROS2 Humble joint_trajectory_controller 文档](https://docs.ros.org/en/humble/p/joint_trajectory_controller/doc/trajectory.html)

> *"Without velocity information, spline interpolation falls back to a linear sub-strategy,
> yielding trajectories with discontinuous velocities at the waypoints."*

### 第 3 层: 关节角缠绕未展开

只对 joint_1/5/6 做了 `_nearest_angle()` 缠绕展开，**但 joint_4 的 [-1.48, 4.62] rad 
范围也覆盖了超过 2π 的区间**。当 trajectory 中一个路径点令 joint_4 = -1.4 rad
而相邻路径点令 joint_4 = 4.5 rad 时，JTC 按"最短路径"应该是走 0.7 rad，
但由于没有 pre-wrap，MoveIt 给出的路径点是绕过整个关节范围的大圈。

```
--1.4 rad ────→ 4.5 rad
   ↑ 理论上应该走 0.7 rad (经过 π 边界)
   ↑ 但实际上 trajectory 走 5.9 rad (绕大圈)
   ↑ JTC 跟丢 → Position Error = 5.25 rad → ABORT
```

### 第 4 层: min_dt 累积效应

`min_dt = 0.12s` 的意图是防止段间速度过快，但对 252 个点：

```
252 points × 0.12s = 30.2s (仅 min_dt 基数)
+ 大段移动增量
≈ 38-52s 总执行时间
```

### 第 5 层: 段结束无沉降 → 级联抖动

JTC 报告 SUCCESSFUL 但 Gazebo 物理关节因惯性仍在微振。
下一段 trajectory 从非零速度开始 → 控制器推算的首段速度被放大 → 振荡积累。

### 第 6 层（根本原因）: 位置控制接口的特性

Gazebo 的 `GazeboSystem` 硬件接口将 `position` command 转换为**正比于误差的速度**：
```
V = K * (commanded_position - current_position)
```

任何被控位置的突变 → 速度尖峰 → 物理振铃。这是所有 `command_interfaces: [position]`
系统的固有问题。

**参考**: [ros-controls/gz_ros2_control Issue #608](https://github.com/ros-controls/gz_ros2_control/issues/608)

---

## 3. 工业界方案调研

### 3.1 轨迹预处理

| 方法 | 原理 | 效果 |
|------|------|------|
| **降采样 (Decimation)** | 移除相邻冗余路径点（变化量 < 阈值） | 减少 60-80% 路径点，消除数值振荡 |
| **Ruckig 在线重规划** | 考虑 jerk 限制的在线轨迹生成 | 最平滑，但计算量大 |
| **Time-Optimal Path Parameterization** | 在路径约束下最优化时间分配 | MoveIt 内置，但会过度采样 |
| **低通滤波** | 对位置指令做一阶/二阶低通 | 简单有效，但引入时延 |

### 3.2 控制器策略

| 方法 | 适用场景 | 说明 |
|------|---------|------|
| `interpolation_method: splines` | 全轨迹执行 | **推荐**, 但需要 velocity 字段 |
| `interpolation_method: linear` | 短节拍点对点 | 无过冲, 但速度不连续 |
| `JointGroupPositionController` | 高频直接控制 | 绕过 JTC, 自己管插值 |
| 梯形速度发生器 | 位置接口速率限制 | 在硬件接口层做, 透明于 ROS |

### 3.3 物理参数

| 参数 | 推荐 | 原理 |
|------|------|------|
| URDF joint damping | 2.0-8.0 (N·m·s/rad) | 增加被动阻尼抑制振铃 |
| ODE solver iters | 50-100 | 更精确的约束求解 |
| ODE max_step_size | 0.001-0.002 | 小步长 = 高精度 |
| contact_surface_layer | 0.001-0.003 | STL 碰撞网格的缓冲层 |

### 3.4 业界参考案例

- **UR10 + Gazebo 稳定性问题**: [ros-controls/gazebo_ros2_control#73](https://github.com/ros-controls/gazebo_ros2_control/issues/73)
  - 解决方案: 降低 URDF 惯性值, 提高 controller update_rate
  
- **MoveIt2 Servo 振荡**: [moveit/moveit2#1857](https://github.com/moveit/moveit2/issues/1857)
  - 解决方案: 禁用 Butterworth 滤波, 降低 publish_rate
  
- **Cartesian 路径 joint-space 距离问题**: [moveit2@a7fe0df](https://github.com/moveit/moveit2/commit/a7fe0df4c565db15710581bdcba1d33d70aa0ddb)
  - 修复: Cartespace 近但 jointspace 远的路径点不 interpolate，改用 jointspace 距离度量

---

## 4. 我们的修复方案

### 4.1 轨迹降采样 (planner_server.py)

```python
DECIMATE_EPS = 0.025  # rad — 0.008 was too conservative for OMPL paths
# OMPL RRT 路径点间距 ~0.001-0.005 rad，0.008 几乎不滤
# 0.025 rad ≈ 1.4° = ~2-3mm 末端行程，足够平滑
# Effect: 60+ pts → ~15 pts, 消除 JTC spline 数值病态
```

效果: JTC 收到 40 个有实际意义的路径点，样条插值不会数值病态。

### 4.2 全关节角度展开

```python
wrap_indices = list(range(n_joints))  # ALL joints, not just 1/5/6
# 每个路径点的每个关节都做 _nearest_angle()
```

效果: 消除所有关节的"绕大圈"问题。

### 4.3 中央差分速度计算（替代保留 MoveIt velocity）

之前尝试保留 MoveIt 的 velocity 数据来让 JTC 使用 cubic/quintic spline，但这引入了新问题：
- MoveIt `AddTimeOptimalParameterization` 不保证末端零速度 → `allow_nonzero_velocity_at_trajectory_end: false` 拒绝轨迹
- MoveIt 的 velocity 可能超过我们设定的 `max_vel` 限制
- 边界条件需要事后修补，不够干净

**工业级方案**：抛弃 MoveIt 的 velocity/acceleration 数据，从路径点位置独立计算速度剖面。

算法（中央差分，2 阶精度）：

```
输入: waypoints[0..N] (含 t=0 原点)
输出: velocities[0..N]

1. 段间时长:  dt[i]  = max(min_dt, max_j(|p[i+1][j] - p[i][j]|) / max_vel)
2. 累积时间:  t[0]   = 0,  t[i+1] = t[i] + dt[i]
3. 速度:
   v[0]   = [0]*N                           # 起点零速（已知静止）
   v[N]   = [0]*N                           # 终点零速（控制器要求）
   v[i]   = (p[i+1] - p[i-1]) / (t[i+1] - t[i-1])   # 中央差分
4. 钳位:    v[i][j] = clamp(v[i][j], ±max_vel)
```

优点：
- **零起点 + 零终点由公式保证**，不需要事后修补
- **速度值 = 位置变化 / 时间**，与控制器的期望天然一致
- **钳位 max_vel**，永不超速
- **无依赖** MoveIt 的 velocity 质量

### 4.4 中央差分 pipeline 集成

在中央差分框架下，t=0 锚定自然成为 pipeline 的第一步：

```python
# waypoints[0] = current_positions (t=0, v=0)  ← 锚定
# waypoints[1..N] = decimated + wrapped waypoints
```

效果: 无需单独维护 t=0 锚定逻辑，中央差分的 velocity 公式自动保证从零速开始。

### 4.5 执行后沉降等待

```python
settle_threshold = 0.08 rad/s  # 关节速度低于此值 = 静止
settle_timeout = 20.0s         # 最长等待（Gazebo过冲严重时）
```

效果: 振荡能量在每段轨迹间耗散。阈值为 0.08（非 0.04），因为 PID D 项 5.0 下残余振动 <0.08 可接受。

### 4.6 URDF Gazebo PID — 解决位置控制过冲的根本方案

**问题**: `gazebo_ros2_control` 的 SimPID 默认 P=100, D=0, I=0。纯比例控制过冲严重（v_max>0.89 rad/s），且 `<command_interface name="position">` 内设的 `<param name="kp">` 不被 `gz_ros2_control` 读取。

**正确方案**: 在 URDF `<gazebo>` extension 中设置 PID：

```xml
<gazebo reference="${prefix}joint_1">
  <implicitSpringDamper>true</implicitSpringDamper>
  <pid>20.0 0.0 5.0</pid>
  <maxEffort>2.0</maxEffort>
</gazebo>
```

**为什么选 P=20, D=5:**
| 参数 | 值 | 效果 |
|------|------|------|
| P=100 (默认) | 响应快，过冲 ~2.0 rad/s, 沉降 20s+ | 不可接受 |
| P=20 | 响应慢 5x，过冲降至 ~0.5 rad/s | 按摩场景可接受（手臂不要求快速响应） |
| D=5 | 主动阻尼，抑制振荡 | 沉降时间缩短到 ~3-5s |
| maxEffort=2.0 | 限制电机最大输出，防止过冲反弹 | 安全裕度 |

**注意事项:**
- `<command_interface name="position">` 内的 `<param name="kp/ki/kd">` 参数虽不会报错，但被 `gz_ros2_control` Humble 版本**忽略**。必须放在 `<gazebo reference="...">` extension 中
- `<implicitSpringDamper>` 和 `<pid>` 可以共存。前者使用 URDF `<dynamics>` 值，后者控制 SimPID
- velocity command interface 曾尝试作为速度前馈调低 P，但引发 gzserver SIGSEGV（见 B038）

---

## 5. 参数调优指南

### 5.1 速度参数速查表

| 参数 | 默认 | 慢速(物理不稳定时) | 快速(稳定后) | 对应修改文件 |
|------|------|-------------------|-------------|------------|
| `controller_max_joint_velocity` | 0.30 rad/s | 0.15 | 0.50 | `planner_server.py:declare_parameter()` |
| `controller_min_segment_dt` | 0.25 s | 0.50 | 0.15 | 同上 |
| URDF joint damping | 1.0 N·m·s/rad | 15.0 | 5.0 | `jaka_c5_arm_macro.xacro:<dynamics>` |
| URDF joint friction | 0.10 | 0.20 | 0.10 | 同上 |
| Gazebo PID (P/I/D) | 100/0/0 | 20/0/5 | 50/0/2 | 同上 `<gazebo><pid>` |
| Gazebo maxEffort | 默认(∞) | 2.0 | 5.0 | 同上 `<gazebo><maxEffort>` |
| settle_threshold | 0.04 rad/s | 0.08 | 0.04 | `massage_nodes.py:_execute_trajectory()` |
| settle_timeout | 10 s | 20 | 5 | 同上 |
| `max_velocity_scaling` | 0.25 | 0.10 | 0.35 | 同上 |
| `max_acceleration_scaling` | 0.20 | 0.05 | 0.30 | 同上 |
| `DECIMATE_EPS` | 0.025 rad | 0.040 | 0.015 | `planner_server.py:_stabilize_trajectory()` |
| `MAX_WAYPOINTS` | 20 | 15 | 30 | `planner_server.py:_stabilize_trajectory()` |
| 沉降 timeout | 10.0 s | 15.0 | 5.0 | `massage_nodes.py:_execute_trajectory()` |
| 沉降 threshold | 0.08 rad/s | 0.05 | 0.15 | 同上 |

### 5.2 调优步骤

1. **先用保守参数跑通所有 Stage**
   - `max_vel=0.35`, `min_dt=0.25`, scaling=0.08
   - 验证无 HALT、无 JTC ABORT
   
2. **逐步提高速度**
   - `max_vel` 每次 +0.10
   - `min_dt` 每次 -0.05
   - 观察日志中的 `Traj stabilize: N→M pts` —— N 应为 MoveIt 原点数，M 应为降采样后点数
   
3. **如果出现 JTC ABORT**
   - 先查是哪个关节、多少误差（`Position Error: X > Y`）
   - 如果是大误差(>1 rad) → 降采样不够或缠绕未展开正确
   - 如果是小误差(0.3-0.5 rad) → 速度太快或阻尼不够
   
4. **如果出现速度 HALT**
   - 检查沉降等待是否生效（日志 `settle done (v_max=0.XX)`）
   - 增大 `settle_timeout` 或减小 `settle_threshold`

### 5.3 控制器参数

```yaml
# ros2_controllers.yaml
left_arm_controller:
  ros__parameters:
    interpolation_method: splines        # 保留 (需要 velocity 字段)
    allow_nonzero_velocity_at_trajectory_end: false  # 必需
    constraints:
      goal_time: 2.0
      stopped_velocity_tolerance: 0.05
      left_joint_1: { trajectory: 0.30, goal: 0.05 }  # 轨迹容差 0.30 rad
```

**容差说明**:
- `trajectory` 容差 (0.30 rad): 路径中间点的最大允许误差。如果 JTC spline 插值实际输出与期望路径点的偏差超过此值 → ABORT。
- `goal` 容差 (0.05 rad): 终点允许偏差。
- 如果频繁 ABORT，可放大 `trajectory` 到 0.50，但这会掩盖真正的问题（通常是缠绕或降采样问题）。

---

## 6. 诊断工具与方法

### 6.1 日志关键标签搜索

```bash
# 找 JTC ABORT 和原因
grep -E "Aborted|tolerances|error_code" log.txt

# 找轨迹降采样效果
grep "Traj stabilize" log.txt
# 期望输出: Traj stabilize: 252→38 pts, duration=15.2s, max_vel=0.70

# 找速度 HALT
grep -E "velocity CRITICAL|velocity WARN" log.txt

# 找沉降等待
grep "settle done" log.txt
```

### 6.2 快速诊断流程

```
机械臂抽风/抖动？
├── 日志有 HALT skip?
│   └── → 速度尖峰 → 沉降不充分
│
├── 日志有 JTC ABORT?
│   ├── Position Error > 1 rad
│   │   └── → 关节缠绕未展开 / 降采样不够
│   │       确认: 轨迹 point count (252→N)
│   └── Position Error 0.3-0.5 rad
│       └── → 速度太快 / 阻尼不够
│           确认: velocity 字段是否保留
│
└── 无报错但视觉抖动？
    └── → min_dt 太小 / ODE 不稳定
        确认: ODE iters ≥ 50, cfm ≤ 0.00001
```

### 6.3 轨迹可视化 (Python)

```python
# 离线检查 trajectory 质量
def check_trajectory(traj: JointTrajectory):
    n = len(traj.points)
    print(f"Points: {n}")
    for i in range(1, n):
        deltas = [abs(a - b) for a, b in zip(
            traj.points[i].positions,
            traj.points[i-1].positions)]
        max_d = max(deltas)
        if max_d > 1.0:
            joint_idx = deltas.index(max_d)
            print(f"  WARN: point {i}, joint {traj.joint_names[joint_idx]} "
                  f"delta = {max_d:.3f} rad")
```

---

## 7. 常见问答

### Q: 为什么不用 `JointGroupPositionController` 替代 `JointTrajectoryController`?

A: JTC 的 spline 插值是全轨迹执行的关键功能。JGPController 只接受单次位置指令，
  需要自己在应用层管理插值和时间分配。对于按摩这种预设轨迹的应用，JTC 更合适。

### Q: 为什么清零 velocity 反而会导致振荡？

A: JTC 收到空的 velocity 字段时，调用 `linear` 子策略，假设每点的到达/离开速度
  均为 0。当路径点间距极小（0.001 rad）时，JTC 必须每段从 0 加速到某个速度再
  减到 0 —— 产生锯齿状的 stop-and-go 运动，在关节柔性/阻尼不足时表现为振荡。

### Q: 为什么只在 Gazebo 有这个问题，RViz mock 没有？

A: RViz 的 `mock_components` 只是简单的关节位置设定，没有物理惯性、没有阻尼、
  没有 ODE 求解器。Gazebo 的 `GazeboSystem` 将位置指令转换为速度指令
  `V = K * error`，物理关节有惯性、摩擦和弹性 —— 动力学响应会被真实物理放大。

### Q: 降采样会不会丢失轨迹精度？

A: 降采样阈值 0.005 rad 意味着每个关节 0.005 rad ≈ 0.29° 的范围内忽略变化。
  JAKA C5 的重复定位精度是 ±0.02mm（约 0.001° 关节角），0.29° 远大于此。
  对于按摩应用（按摩点间距通常 2-3cm），0.29° 的关节角当量在末端约 0.5mm，
  完全可接受。而且 JTC 的 spline 插值会在保留的路径点之间自动平滑过渡。

---

## 8. 参考资源

### ROS2 官方

- [joint_trajectory_controller 文档 (Humble)](https://docs.ros.org/en/humble/p/joint_trajectory_controller/doc/trajectory.html)
- [ros2_control 控制器类型](https://control.ros.org/master/doc/ros2_control/controllers/doc/userdoc.html)

### 已知问题追踪

- [gazebo_ros2_control #73 — UR10 位置控制不稳定](https://github.com/ros-controls/gazebo_ros2_control/issues/73)
- [gz_ros2_control #608 — 梯形速度曲线功能请求](https://github.com/ros-controls/gz_ros2_control/issues/608)
- [moveit2 #1857 — Servo Gazebo 位置漂移](https://github.com/moveit/moveit2/issues/1857)
- [Cartesian 路径 joint-space 距离修复](https://github.com/moveit/moveit2/commit/a7fe0df4c565db15710581bdcba1d33d70aa0ddb)

### 算法

- [Ruckig — 在线轨迹生成 (jerk 约束)](https://github.com/pantor/ruckig)
- [Reflexxes Type II — 在线轨迹平滑](https://www.reflexxes.ws/)
- [TOTG — 时间最优轨迹生成](https://github.com/balakumar-s/totg)

### 我们的 Bug 记录

- [BUGLOG.md](BUGLOG.md) — B028 抖动级联, B029 速度 HALT
