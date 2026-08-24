# 单臂 Pick-and-Place Demo 调试记录

> 历史记录中的 `JAKA C5`、`Link_00...06` 和以 `tool_flange` 为 TCP 的描述，
> 仅指重命名前的旧实现；当前契约见 `fruit_picking_arm/docs/CUSTOM_ARM_CAD_MODEL.md`。

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
- `fruit_picking_arm.rviz` 中启用 TF、启用 Grid、保持 RobotModel 启用
- MotionPlanning 插件默认禁用（避免交互标记捕获鼠标 + 时序冲突）
- 用户如需查看碰撞物体/规划路径，可在 RViz 面板手动勾选 MotionPlanning
- 新增轨迹内容诊断日志（打印每个轨迹的起点/终点关节值 + 最大 Δ）

---

## Bug 2：RViz 水果小球显示异常

**现象：**
- 之前只能看到 2 个绿色小球（Planning Scene 碰撞物体的默认渲染色）
- 真正的彩色 Marker（`/rviz_visual_tools`）可能因为 MotionPlanning 插件渲染冲突被遮挡

**当前状态（2026-06-21）：**
- `fruit_picking_arm.rviz` 中已配置 `Scene Markers` display 订阅 `/rviz_visual_tools`
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
`fruit_picking_arm.rviz` 中 MotionPlanning 插件默认 `Enabled: false`。
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

**改动文件清单（均在 `fruit_arm_moveit_config/` 内，不影响双臂）：**
- `scripts/pick_place_demo.py` — Orange 位置 + 轨迹诊断日志
- `scripts/simulated_camera.py` — Orange 位置同步
- `config/fruit_picking_arm.rviz` — 启用 TF + RobotModel，禁用 MotionPlanning
- `launch/pick_place_demo.launch.py` — 集成 simulated_camera，清理死参数
- `BUG_SUMMARY.md` — 本文档更新

---

## 运行验证（2026-06-22 第一轮）

### 编译 ✅
`colcon build --packages-select fruit_arm_moveit_config` → 1 package finished [5.82s]，零错误。

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
从 `fruit_picking_arm.rviz` 中**完全移除** MotionPlanning 显示插件（而非仅禁用）。
渲染由独立的 `RobotModel`（机械臂）+ `MarkerArray`（场景）cover。
用户如需碰撞物体/规划路径可视化，可在 RViz 面板手动 Add。

---

## 运行验证（2026-06-22 第三轮）：MotionPlanning 移除 + QoS 修复确认

### 编译 ✅
`colcon build` → 1 package finished [2.80s]，零错误。

### 关键修复验证

| 修复项 | 状态 | 日志证据 |
|--------|------|----------|
| MotionPlanning 移除 | ✅ **确认有效** | RViz 日志中**零条** `planning_scene_monitor` / `MoveGroup` / `interactive_marker_display` / `No robot state or robot model loaded` |
| TF 静态帧 QoS | ✅ **确认修复** | `静态=9帧(Link_00 camera_bracket camera_color_frame camera_depth_frame camera_link gripper_base left_finger_tip right_finger_tip...)` — 不再误报 0 帧 |
| Bug 1: 机械臂运动 | ✅ **确认** | 所有关节角度大幅变化，如 J6: 0→-1.781 rad, J1: 0→-0.590 rad |
| Bug 4: Orange IK | ✅ **确认** | 无 `code=-31`，Orange 全程抓取成功 |
| 规划/校验/执行 | ✅ **全部正常** | 所有阶段 `Goal reached, success!` |

### 机械臂运动轨迹摘要

**Apple** (29+12+12+21+12+11 = 97 轨迹点):
```
HOME → hover(29pt,2.79s) → grasp(12pt,1.02s) → 夹紧 → retreat(12pt,1.03s)
     → bin_hover(21pt,1.96s) → bin_drop(12pt,1.06s) → 松开 → bin_retract(11pt,0.99s)
```

**Orange** (17+12+12+17+12+12 = 82 轨迹点):
```
retreat_pos → hover(17pt,1.57s) → grasp(12pt,1.03s) → 夹紧 → retreat(12pt,1.04s)
            → bin_hover(17pt,1.57s) → bin_drop(12pt,1.06s) → 松开 → bin_retract(12pt,1.05s)
```

**Plum** (15+11+11+... = 已开始，日志截断前正常):
```
retreat_pos → hover(15pt,1.38s, J2:+1.022→+1.691 Δ0.669) → grasp(11pt,0.99s) → 夹紧 → retreat(11pt,0.99s) → ...
```

### 待用户目视确认

虽然所有日志指标正常，以下项目需在 RViz 中目视确认：
1. **机械臂模型**是否从 HOME 开始全程流畅运动（不再中途冻结）
2. **4 个彩色水果小球**是否可见（红/橙/紫/绿，对应 apple/orange/plum/lime）
3. **RViz 视角拖动**是否正常（鼠标左键 Orbit、滚轮缩放）
4. **Lime** 是否完成全部 pick-and-place（日志截断前 Plum 还在进行，Lime 未开始）

### 当前状态总览

| Bug | 状态 | 证据 |
|-----|------|------|
| Bug 1: 机械臂不运动 | ✅ **已解决** | 终端日志：关节角度大幅变化；RViz：待目视确认不冻结 |
| Bug 2: 水果小球不显示 | 🟡 **待目视** | Marker 每 0.2s 发布，MotionPlanning 不再遮挡 |
| Bug 3: RViz 无法拖动 | 🟡 **待目视** | MotionPlanning 已移除，无交互标记捕获鼠标 |
| Bug 4: Orange IK 失败 | ✅ **已解决** | Orange 全程抓取成功，无 code=-31 |

**改动文件清单（5个，均未提交）：**
- `scripts/pick_place_demo.py` — Orange 位置 + 诊断日志 + `/tf_static` QoS 修复
- `scripts/simulated_camera.py` — Orange 位置同步
- `config/fruit_picking_arm.rviz` — **移除** MotionPlanning 插件（非仅禁用）
- `launch/pick_place_demo.launch.py` — 集成 simulated_camera
- `BUG_SUMMARY.md` — 本文档

---

## 运行验证（2026-06-22 第四轮）：RobotModel QoS + 水果刚体附着

### 🔴 新发现：RobotModel 不渲染

**现象：** RViz 中只能看到 TF 坐标系，机械臂 3D 模型完全不显示。

**根因：** 与 `/tf_static` QoS 问题同源 — `robot_state_publisher` 以 `TRANSIENT_LOCAL`（latched）发布 `/robot_description`，
但 RViz 的 RobotModel display 的 Description Topic 未显式指定 QoS，默认 `VOLATILE` 订阅者收不到已 latched 的消息。

**修复（2026-06-22）：**
`fruit_picking_arm.rviz` 中 RobotModel 的 `Description Topic` 显式指定：
```yaml
Description Topic:
  Depth: 5
  Durability Policy: Transient Local
  History Policy: Keep Last
  Reliability Policy: Reliable
  Value: /robot_description
```

### 🔴 新发现：水果抓取仿真效果差

**现象（用户反馈）：**
1. 绿色小球（lime）夹取后没有消失反而变小了
2. 其他小球夹取后还有残影留存
3. 抓取后水果标记不跟随机械臂运动

**根因：** `_publish_markers()` 中水果状态切换时只改变了 alpha 和 scale，
标记始终停留在桌面原位。缺少 TF 监听跟踪 tool_flange 实时位姿。

**修复（2026-06-22）：**
1. 引入 `tf2_ros.Buffer + TransformListener` 实时查询 tool_flange 世界位姿
2. 抓取瞬间计算 fruit → tool_flange 偏移量（`grasped_fruit_offsets`）
3. `_publish_markers()` 按状态分别处理：
   - `free`：桌面原位，α=0.90
   - `grasped`：tool_flange 实时位姿 + 偏移，跟随机械臂运动，α=0.90
   - `placed`：料框底部，α=0.55（无残影）
4. 已放入料框的水果不再在桌面留下残影

### 🟡 RViz 视角拖动修复

**修复：** `fruit_picking_arm.rviz` 对齐双臂工作配置：
- 添加 `Tools` 面板（MoveCamera, Interact, Select 等）
- 视图类型从 `XYOrbit` 改为 `Orbit`（标准 RViz 相机控制器）
- 添加 `Transformation` 段（TF 集成）

### 新增依赖

`pick_place_demo.py` 新增 import：
```python
from tf2_ros import Buffer, TransformException, TransformListener
```

**改动文件清单（5个，均未提交）：**
- `scripts/pick_place_demo.py` — TF 监听 + 水果刚体附着 + `/tf_static` QoS 修复
- `scripts/simulated_camera.py` — Orange 位置同步
- `config/fruit_picking_arm.rviz` — RobotModel QoS + Tools 面板 + Orbit 视图 + MotionPlanning 移除
- `launch/pick_place_demo.launch.py` — 集成 simulated_camera
- `BUG_SUMMARY.md` — 本文档
