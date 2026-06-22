# 单臂 Pick-and-Place Demo 调试记录

> 本文档记录 JAKA C5 单臂水果抓取 Demo 的调试过程。对照双臂项目的 [BUG_SUMMARY.md](../dual_arm_jaka_c5_moveit_config/BUG_SUMMARY.md)（7 个已修复 Bug，已全部规避）。

---

## Bug 1：机械臂只规划不执行

**✅ 已确认不是运动问题，是 RViz 渲染问题（2026-06-21）**

**根因：**
关节角度日志证明机械臂**一直在动**（如 J6 从 0→-1.780 rad，J1 从 0→-0.591 rad），
`mock_components/GenericSystem`、`arm_controller`、`/joint_states` 全部正常工作。

问题出在 RViz 端两个配置错误：
1. **TF display 被禁用**（`Enabled: false`）→ RobotModel 虽启用但无法计算连杆位姿
2. **MotionPlanning 插件先于 `planning_scene_monitor` 启动** → 报错 `No robot state or robot model loaded`

**修复（2026-06-21）：**
- `pick_place.rviz` 中启用 TF、启用 Grid、保持 RobotModel 启用
- MotionPlanning 插件默认禁用（避免交互标记捕获鼠标 + 时序冲突）
- 用户如需查看碰撞物体/规划路径，可在 RViz 面板手动勾选 MotionPlanning
- 新增轨迹内容诊断日志（打印每个轨迹的起点/终点关节值 + 最大 Δ）

---

## Bug 2：RViz 水果小球显示异常

**现象：**
- 之前只能看到 2 个绿色小球（Planning Scene 碰撞物体的默认渲染色）
- 真正的彩色 Marker（`/rviz_visual_tools`）可能因为 MotionPlanning 插件渲染冲突被遮挡

**当前状态（2026-06-21）：**
- `pick_place.rviz` 中已配置 `Scene Markers` display 订阅 `/rviz_visual_tools`
- MotionPlanning 默认禁用后，彩色水果 Marker 不受遮挡
- `_publish_markers()` 每 0.2 秒发布一次，颜色随水果状态变化：`free`→α=0.85, `grasped`→α=0.35, `placed`→α=0.22
- 待重新编译运行后验证

---

## Bug 3：RViz 无法拖动视角

**✅ 已修复（2026-06-21）**

**根因：**
MotionPlanning 插件的 `InteractiveMarkerDisplay` 在 RViz 初始化时加载交互标记（末端拖动手柄），
用户点击机器人模型时捕获鼠标，阻止 Orbit 操作。

**修复：**
`pick_place.rviz` 中 MotionPlanning 插件默认 `Enabled: false`。
机械臂渲染改用独立的 RobotModel display + TF，无需 MotionPlanning 插件即可显示。
用户如需交互标记，可在 RViz 面板手动勾选 MotionPlanning。

---

## Bug 4：Orange 水果 IK 求解失败 (code=-31)

**现象：**
```
[pick_place_demo]: IK failed: code=-31
[pick_place_demo]: IK failed for hover_fruit_orange, skip
```
Apple、Plum、Lime 的 IK 全部成功，唯独 Orange 失败。

**根因：**
Orange 原位置 `(x=0.70, y=0.10)` 距离机械臂底座太远。JAKA C5 工作半径约 0.7m，加上 tool_flange 的 Z 偏移（抓取时 tool_flange 需在水果上方 0.086m），末端姿态要求 Z 轴向下指向桌面，使得该位置超出有效 IK 求解范围。

**修复（2026-06-21）：**
- Orange 移到 `(x=0.55, y=-0.05)` — 靠近料框方向，缩短与底座距离
- 同步更新 `simulated_camera.py` 中的 FRUITS 列表
- 状态：**✅ 已修复，待重新编译运行验证**

---

## 新增诊断日志（2026-06-21）

在 `_execute()` 中添加了轨迹内容打印：
- 打印轨迹点数、时长、起点/终点与当前关节角的最大偏差（`maxΔ`）
- 如果终点 `maxΔ < 0.005rad`，警告 **轨迹退化**（终点位置与当前位置几乎相同）
- 打印前 2 个点和最后 1 个点的 6 个关节角值，方便人工对比

同时：
- `simulated_camera.py` 现在已在 `pick_place_demo.launch.py` 中启动（发布 `/camera/depth/points` 点云）
- 移除了 launch 文件中未使用的参数（`plan_group`, `gripper_open_*`, `gripper_closed_*`）

---

## 已规避的双臂 Bug（对照清单）

以下双臂项目遇到过的 Bug 在当前单臂配置中均已预防：

| 双臂 Bug | 当前状态 |
|----------|----------|
| Bug 1: AddTimeOptimalParameterization 命名空间错误 | ✅ `request_adapters` + 正确命名空间 |
| Bug 2: 交互标记只能旋转不能平移 | ✅ 不涉及（无 end effector 交互标记配置） |
| Bug 3: CHOMP 被误选 | ✅ 只安装 OMPL |
| Bug 4: YAML 参数格式错误 | ✅ 使用 `MoveItConfigsBuilder.to_dict()` |
| Bug 5: Ros2ControlManager 崩溃 | ✅ 使用 `MoveItSimpleControllerManager` |
| Bug 6: RRTConnect 配置名错误 | ✅ 使用 `RRTConnectkConfigDefault` |
| Bug 7: 路径隐含碰撞 | ✅ `longest_valid_segment_fraction: 0.005` |

---

## 当前状态总览（2026-06-21 第二轮）

| Bug | 状态 | 修复内容 |
|-----|------|----------|
| 🔴 Bug 1: 机械臂不运动 | ✅ **已解决** | 确认机械臂一直在动，问题在 RViz 渲染；启用 TF + RobotModel，禁用 MotionPlanning |
| 🔴 Bug 2: 水果小球不显示 | 🟡 **待验证** | Scene Markers display 已配，MotionPlanning 禁用后不再遮挡 |
| 🔴 Bug 3: RViz 无法拖动 | ✅ **已修复** | MotionPlanning 默认禁用，消除交互标记鼠标捕获 |
| 🔴 Bug 4: Orange IK 失败 | ✅ **已修复** | Orange 从 (0.70,0.10) 移到 (0.55,-0.05) |

**改动文件清单（均在 `single_arm_jaka_c5_pick_place/` 内，不影响双臂）：**
- `scripts/pick_place_demo.py` — Orange 位置 + 轨迹诊断日志
- `scripts/simulated_camera.py` — Orange 位置同步
- `config/pick_place.rviz` — 启用 TF + RobotModel，禁用 MotionPlanning
- `launch/pick_place_demo.launch.py` — 集成 simulated_camera，清理死参数
- `BUG_SUMMARY.md` — 本文档更新

---

## 运行验证（2026-06-22 第一轮）

### 编译 ✅
`colcon build --packages-select single_arm_jaka_c5_pick_place` → 1 package finished [5.82s]，零错误。

### 运行结果

| 检查项 | 状态 | 证据 |
|--------|------|------|
| Bug 1: 机械臂运动 | ✅ **确认在动** | J6: 0→-1.778 rad, J1: 0→-0.588 rad, J3: -1.5→-1.056 rad |
| Bug 2: 水果小球显示 | ⏳ **待用户确认** | Marker 每 0.2s 发布，需在 RViz 中目视确认 |
| Bug 3: RViz 拖动 | ⏳ **待用户确认** | MotionPlanning 禁用，无交互标记 |
| Bug 4: Orange IK | ✅ **已修复** | 无 `code=-31` 错误，Orange 全程抓取成功 |
| 规划/校验/执行 | ✅ **全部正常** | 4 个水果 Plan→Validate→Execute 全部 `Goal reached, success!` |

### 🟡 新发现：TF 静态帧诊断 QoS 不匹配（2026-06-22）

**现象：** 日志反复打印 `Link_00 不在 /tf_static 中！RobotModel 无法找到机械臂根连杆` + `TF: 静态=0帧`

**根因：** `robot_state_publisher` 发布 `/tf_static` 使用 `TRANSIENT_LOCAL`（latched），
但 `pick_place_demo.py` 中的订阅使用默认 QoS（`VOLATILE`）。
ROS2 中 VOLATILE 订阅者**不会**收到 TRANSIENT_LOCAL 发布者的缓存消息。

**影响：** **仅影响诊断日志**，不影响实际系统。RViz 通过 `tf2_ros` 内部库正确使用 TRANSIENT_LOCAL 订阅，能正常解析静态帧。

**修复（2026-06-22）：**
```python
# 修复前
self.tf_static_sub = self.create_subscription(
    TFMessage, "/tf_static", self._on_tf_static, 10,
)
# 修复后
from rclpy.qos import DurabilityPolicy, QoSProfile
self.tf_static_sub = self.create_subscription(
    TFMessage, "/tf_static", self._on_tf_static,
    QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
```

### RViz 已知无害错误

以下错误来自 MotionPlanning 显示插件（已 `Enabled: false`）的异步初始化过程，不影响功能：
- `No robot state or robot model loaded` — 轨迹可视化插件在模型加载前尝试渲染
- `Action server: /recognize_objects not available` — 感知功能未启用
- `interactive_marker_display` 连接日志 — 插件初始化副作用，Disabled 状态下不会渲染

---

## 运行验证（2026-06-22 第二轮）：MotionPlanning 插件冲突

### 🔴 发现：MotionPlanning 即使在 Disabled 状态也会导致 RViz 冻结

**现象：** RViz 启动后机械臂运动正常，但当 MotionPlanning 插件的 PlanningScene
渲染加载后（~9秒），RViz 中机械臂停止更新渲染。

**根因：** `moveit_rviz_plugin/MotionPlanning` 即使设为 `Enabled: false`，其内部组件
（planning_scene_monitor、MoveGroup、interactive_marker_display）仍会在后台
异步初始化。PlanningScene 对复杂 STL 网格的碰撞几何计算 + RobotModel 双重渲染
导致 RViz 渲染线程冻结。

**修复（2026-06-22）：**
从 `pick_place.rviz` 中**完全移除** MotionPlanning 显示插件（而非仅禁用）。
渲染由独立的 `RobotModel`（机械臂）+ `MarkerArray`（场景）cover。
用户如需碰撞物体/规划路径可视化，可在 RViz 面板手动 Add。
