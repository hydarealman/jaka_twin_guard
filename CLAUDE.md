# JAKA Twin Guard — 项目开发指南

> 最后更新：2026-06-23 | 当前分支：main | 负责：hydarealman

## 项目概览

本项目基于 ROS2 Humble + MoveIt2，在 WSL2 (Ubuntu 22.04) 上运行。包含 JAKA C5 机械臂的多个仿真 Demo：

| 子包 | 说明 | 状态 |
|------|------|------|
| `dual_arm_jaka_c5_moveit_config` | 双臂协同搬运 + 双臂按摩 Demo | ✅ 正常工作（按摩Demo 2026-06-23重设计） |
| `single_arm_jaka_c5_pick_place` | **单臂 Pick-and-Place 水果抓取** | 🟡 待目视确认（RViz 3 项修复已提交） |
| `jaka_c5_description` | JAKA C5 STL 模型库（只读，所有包共用） | ✅ 稳定 |

## 快速开始

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --packages-select <package_name>
source install/setup.bash
```

### 双臂按摩 Demo（2026-06-23 重设计）
```bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py
```

**新特性**：
- 机械臂放在**床两侧中部** (x=0.62, y=±0.40)，而非床头
- 详细人体模型（头/颈/肩/背/腰/骨盆/四肢）用于可视化 + 碰撞检测
- 沿脊柱轮廓按摩：48阶段 = 8区域（颈→肩→上背→中背→下背→腰椎→骶骨→返回）× 6动作（悬停/按压/释放/左揉/右揉/滚揉）
- 按摩Demo专用机械臂**无末端法兰**(grip_pad)，不影响搬运Demo
- 双臂交叉碰撞检测保持启用

### 双臂协同搬运 Demo（已稳定）
```bash
ros2 launch dual_arm_jaka_c5_moveit_config carry_object_demo.launch.py
```

### 单臂 Pick-and-Place Demo（调试中）
```bash
colcon build --packages-select single_arm_jaka_c5_pick_place
source install/setup.bash
ros2 launch single_arm_jaka_c5_pick_place pick_place_demo.launch.py
```

## 目录结构

```
jaka_twin_guard/
├── CLAUDE.md                          # <-- 你在这里
├── README.md
└── src/moveit_resources-ros2/
    ├── jaka_c5_description/           # STL 模型文件（只读）
    ├── dual_arm_jaka_c5_moveit_config/
    │   ├── config/                    # URDF/XACRO, SRDF, YAML 配置
    │   ├── launch/                    # demo.launch.py, carry/massage launch
    │   ├── scripts/                   # dual_arm_carry_demo.py, dual_arm_massage_demo.py
    │   └── BUG_SUMMARY.md             # 双臂调试历史（7个Bug，已全部修复）
    └── single_arm_jaka_c5_pick_place/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   ├── jaka_c5_pick_place.urdf.xacro   # 顶层 URDF（arm+gripper+camera）
        │   ├── jaka_c5_arm_macro.xacro          # 单臂宏（关节、惯量、STL）
        │   ├── gripper.xacro                    # 平行夹爪（2个 prismatic 关节）
        │   ├── camera_mount.xacro               # 眼在手上深度相机（装在 Link_05 手腕）
        │   ├── jaka_c5.ros2_control.xacro        # mock_components/GenericSystem 硬件
        │   ├── jaka_c5_pick_place.srdf           # 规划组、碰撞矩阵、位姿预设
        │   ├── kinematics.yaml                   # KDL IK 求解器
        │   ├── joint_limits.yaml                 # 速度/加速度限制
        │   ├── ompl_planning.yaml                # OMPL RRTConnect + TOTP 适配器
        │   ├── moveit_controllers.yaml           # MoveItSimpleControllerManager
        │   ├── ros2_controllers.yaml             # arm_controller + joint_state_broadcaster
        │   ├── initial_positions.yaml            # 起始关节角（HOME 位姿）
        │   └── pick_place.rviz                   # RViz 布局
        ├── launch/
        │   └── pick_place_demo.launch.py         # 启动文件
        └── scripts/
            ├── pick_place_demo.py                # 主 Demo（IK 动态求解）
            └── simulated_camera.py               # 模拟深度相机（未集成到 launch）

## 双臂按摩 Demo 架构（2026-06-23 v3 终版）

### 臂位置（X错开0.16m防碰撞）

| 参数 | 搬运默认 | 按摩值 | 说明 |
|------|---------|--------|------|
| left_arm_x | 0 | **0.53** | 左臂X（负责上段：颈→上背） |
| left_arm_y | -0.25 | **-0.45** | 左臂Y（床左侧0.12m间隙） |
| right_arm_x | 0 | **0.69** | 右臂X（负责下段：中背→骶） |
| right_arm_y | 0.25 | **0.45** | 右臂Y（床右侧） |
| use_flange | true | **false** | 按摩无末端法兰 |

**关键设计**：双臂X错开0.16m → 工作空间自然隔离 → 即使同时伸向床面，肘部也不会碰撞。
左臂 joint_1≈1.28~1.98, 右臂 joint_1≈-1.75~-1.14（对称取反）。
交替动作：左按/揉时右臂hover避让，反之亦然。

### 人体模型（12段脊柱轮廓 + 膀胱经7穴）

背部S曲线（俯卧）：C7最高(0.397)→胸椎渐降→腰椎凹陷(0.375)→骶骨(0.371)
脊柱脊线(LINE_STRIP +0.008m凸起) + 左右侧边轮廓线 + 14个膀胱经穴位红点

### 按摩编排：50式中医推拿（5阶段）

| 阶段 | 式数 | 手法 | 说明 |
|------|------|------|------|
| 1.推法 Gliding | 1-6 | 长推 | 沿脊柱推扫，热身放松 |
| 2.按揉法 Press-Knead | 7-18 | 按压+揉捏 | 深层组织，左右交替 |
| 3.点穴法 Acupressure | 19-32 | 点按 | 膀胱经7穴，逐穴点按 |
| 4.滚揉法 Rolling | 33-42 | 滚揉 | 肌肉松解，滚动按摩 |
| 5.拍法+收功 | 43-50 | 轻拍+深按 | 冷却收功 |

每阶段双臂交替动作（左按右避/右揉左避），MoveIt碰撞检测全程启用。
关节容忍度放宽到0.05rad（~3°）给IK足够灵活性。

### RViz：PlanningScene Show Scene Geometry=false（避免绿色方块遮挡人体模型）

## 单臂 Pick-and-Place 架构

```
机械臂模型: JAKA C5 × 1 (6 个转动关节)
末端执行器: 平行夹爪 (2 个 prismatic 关节, Y 轴, 最大开口 80mm)
相机: 眼在手上深度相机 (装在 Link_05 手腕)
硬件模拟: mock_components/GenericSystem
控制器: joint_trajectory_controller/JointTrajectoryController (8 轴)
规划器: OMPL RRTConnect + /compute_ik (动态 IK)
```

### 坐标约定
- **世界原点**: 机械臂底座中心
- **桌面**: 中心 (0.70, 0, 0.30)，尺寸 0.75×0.70×0.04
- **料框**: 中心 (0.55, 0.45, 0.30)，尺寸 0.20×0.20×0.15
- **水果**: 4 个彩色球体散布桌面（apple, orange, plum, lime）

### 关节方向约定（重要！）
- `joint_2 > 0` → 大臂**后仰**（安全方向，远离桌面）
- `joint_3 < 0` → 肘部**前弯/下弯**
- HOME 位姿: `[0, 1.50, -1.50, 1.50, 1.57, 0]`（大臂后仰，远离桌面/料框）

### Pick-and-Place 流程
```
HOME → hover(fruit, IK) → grasp(fruit, IK) → 夹紧 → retreat
     → bin_hover(IK) → bin_drop(IK) → 松开 → bin_retract
     → 下一水果 ... → HOME
```

## 调试状态（4 个 Bug 已修复 + 2 个新问题已修复，2026-06-22 第四轮）

### ✅ Bug 1：机械臂只规划不执行 → 已解决
**根因**: 两重问题：
1. **RViz 渲染**：`pick_place.rviz` 中 TF display 被禁用 → RobotModel 无法计算连杆位姿
2. **MotionPlanning 插件冲突（2026-06-22 确认）**：即使 `Enabled: false`，该插件仍会在后台异步初始化 planning_scene_monitor + MoveGroup + interactive_marker_display，其 PlanningScene 加载后与独立 RobotModel 冲突导致 RViz 渲染冻结
**修复**: 启用 TF + RobotModel，**完全移除** MotionPlanning 显示插件（非仅禁用）

### 🟡 Bug 2：RViz 水果小球不显示 → 已修复（待目视确认）
**根因**: MotionPlanning 插件的 PlanningScene 渲染遮挡了 `/rviz_visual_tools` 的彩色 Marker。
**修复**: MotionPlanning 完全移除后彩色 Marker 不受遮挡；`_publish_markers()` 每 0.2s 发布，颜色随状态变化（free→α=0.90, grasped→α=0.65, placed→α=0.45）。终端日志确认 Marker 正常发布，需在 RViz 中目视确认。

### ✅ Bug 3：RViz 无法拖动视角 → 已解决
**根因**: MotionPlanning 插件的 `InteractiveMarkerDisplay` 在 RViz 初始化时加载交互标记，用户点击机器人模型时捕获鼠标，阻止 Orbit 操作。
**修复**: `pick_place.rviz` 中 MotionPlanning 插件**完全移除**（非仅禁用）。机械臂渲染改用独立 RobotModel + TF display，无需 MotionPlanning 即可显示。用户如需交互标记可在 RViz 面板手动 Add。

### ✅ Bug 4：Orange 水果 IK 求解失败 (code=-31) → 已解决
**根因**: Orange 原位置 `(0.70, 0.10)` 超出 JAKA C5 工作半径（约 0.7m 含末端偏移）。
**修复**: Orange 移到 `(0.55, -0.05)`，靠近料框方向，同步更新 `simulated_camera.py` 中的位置。

### ✅ Bug 5：RViz 机械臂 3D 模型不渲染 → 已解决（2026-06-22）
**根因**: `robot_state_publisher` 以 `TRANSIENT_LOCAL` latched 模式发布 `/robot_description`，但 RViz RobotModel display 的 Description Topic 未显式指定 QoS，默认 VOLATILE 订阅者收不到已 latched 的消息。与之前 `/tf_static` 问题同源。
**修复**: `pick_place.rviz` 中 RobotModel 的 `Description Topic` 显式指定 `Durability Policy: Transient Local`。

### ✅ Bug 6：水果抓取仿真效果差 → 已解决（2026-06-22）
**现象**: (1) 绿色小球夹取后缩小但不消失 (2) 已放置水果在桌面留残影 (3) 水果标记不跟随机械臂。
**根因**: `_publish_markers()` 只改变 alpha/scale，标记始终留在桌面原位。缺少 TF 跟踪 tool_flange 位姿。
**修复**: 引入 `tf2_ros.Buffer + TransformListener` 实时查询 tool_flange → world 变换；抓取时记录 fruit→tool_flange 偏移；渲染时 grasped 水果附加到 tool_flange 实时位姿，placed 水果移到料框底部。

### 🟡 RViz 视角拖动 → 待验证
**修复**: `pick_place.rviz` 对齐双臂工作配置——添加 Tools 面板（MoveCamera/Interact/Select）、视图从 XYOrbit 改为 Orbit、添加 Transformation 段。

### 新增诊断功能
- `_execute()` 中打印轨迹点数、时长、起点/终点关节 delta，检测退化轨迹（终点Δ<0.005rad 警告）
- `_print_joint_status()` 每 3s 打印当前关节角度 + TF 诊断（动态帧/静态帧计数）
- `/tf` 和 `/tf_static` 监听器检测 Link_00 等关键帧是否存在
- **`/tf_static` QoS 修复（2026-06-22）**：订阅改用 `TRANSIENT_LOCAL` durability（robot_state_publisher 发布 `/tf_static` 使用 latched 模式，默认 VOLATILE 订阅者收不到缓存消息）。修复后静态帧正确显示 9 帧。
- `simulated_camera.py` 已集成到 launch 文件，发布 `/camera/depth/points` 点云

## 关键技术细节

### mock_components/GenericSystem
- 在系统中不作为独立 ROS2 包存在（`ros2 pkg list | grep mock` 为空）
- 但插件加载成功（日志: `Successful initialization of hardware`）
- 可能内嵌在 `ros2_control` 或 `hardware_interface` 中
- 功能：将命令接口的值镜像复制到状态接口

### 碰撞检测配置
从双臂 BUG_SUMMARY.md Bug 7 继承的教训：
- `longest_valid_segment_fraction: 0.005` ✅ 已配置
- `AddTimeOptimalParameterization` 在 `request_adapters` ✅ 已配置

### 手指闭合间隙
- `GRIPPER_CLOSED = [0.005, -0.005]`（留 2mm 间隙）
- 避免手指碰撞盒重叠（手指厚 0.008m，在 Y=±0.005 时内边距=0.002m）
- SRDF 中也禁用了 left_finger↔right_finger 碰撞检查

## 通信约定
- **所有文本输出必须用中文** — 用户明确要求
- 代码注释可以中英混合
- 日志输出用中文（如 `"抓取 Apple ..."`, `"释放 Plum ..."`）

## 相关文档
- `src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/BUG_SUMMARY.md` — 双臂 7 个 Bug 的详细记录
- `src/moveit_resources-ros2/single_arm_jaka_c5_pick_place/BUG_SUMMARY.md` — 单臂当前问题详细记录
- `README.md` — 项目主 README（按摩 Demo 文档）
