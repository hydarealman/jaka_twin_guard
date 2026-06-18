# 双机械臂防碰撞 Demo 调试总结

> 本文档记录了搭建 JAKA C5 双机械臂防碰撞演示项目过程中遇到的关键 Bug 及其解决方案。

---

## Bug 1：轨迹时间戳全为零，Plan 成功但 Execute 失败

**现象：**
- RViz 中 Plan 能找到无碰撞路径（OMPL 规划成功）
- Plan & Execute 时机械臂不动
- move_group 日志：`Time between points 0 and 1 is not strictly increasing, it is 0.000000`
- 手动发送带时间戳的轨迹到 controller 可以正常执行

**根因：**
`AddTimeOptimalParameterization`（时间最优轨迹时间参数化）响应适配器没有被正确加载。

在 ROS2 Humble 的 MoveIt2 中，该类注册在：
```
default_planner_request_adapters/AddTimeOptimalParameterization
```

但我们的 `ompl_planning.yaml` 中把它放在了 `response_adapters`，且用了**错误的命名空间**：
```yaml
# ❌ 错误配置（双机械臂套件常见错误）
response_adapters: "default_planner_response_adapters/AddTimeOptimalParameterization ..."
```

`default_planner_response_adapters/` 这个命名空间下没有注册任何插件，导致 `AddTimeOptimalParameterization` 加载失败，轨迹的时间戳全为零。

对比单机械臂 panda_moveit_config（可正常工作）：
```yaml
# ✅ 正确配置
request_adapters: "... default_planner_request_adapters/AddTimeOptimalParameterization"
```

**解决：**
将 `AddTimeOptimalParameterization` 移到 `request_adapters` 链中，使用正确的命名空间 `default_planner_request_adapters/`。

**教训：**
- 不要混用 `default_planner_request_adapters/` 和 `default_planner_response_adapters/`
- 所有 MoveIt2 Humble 的 motion planning adapter 都注册在 `default_planner_request_adapters/` 下
- 用 `cat /opt/ros/humble/share/moveit_ros_planning/planning_request_adapters_plugin_description.xml` 可查看所有已注册的适配器

---

## Bug 2：RViz 拖动交互标记只能旋转不能平移

**现象：**
- RViz 中拖动机械臂末端的交互标记时，只能改变末端姿态（旋转），不能改变位置（平移）
- 之前可以正常拖动，切换为 OMPL 后出现问题

**根因：**
在 SRDF 中定义了末端执行器（end effector）组，但配置不完整——定义了 `parent` 和 `group` 但不匹配，导致 MoveIt 的逆运动学（IK）求解器无法正确计算末端位置对应的关节角。

**解决：**
删除了不完整的末端执行器定义，回归之前的简洁配置。在没有末端执行器的情况下，RViz 使用规划组（left_arm / right_arm）的 tip link 作为交互标记的参考点，IK 求解正常工作。

---

## Bug 3：CHOMP 规划器被误选为默认规划器

**现象：**
- move_group 日志显示使用了 `chomp_interface/CHOMPPlanner`
- CHOMP 规划失败或行为不符合预期
- 明确配置了 `planning_plugin: "ompl_interface/OMPLPlanner"` 但无效

**根因：**
MoveIt2 的规划器选择逻辑：当系统中安装了多个规划器插件时（CHOMP、OMPL、STOMP 等），MoveIt 可能选择非预期的默认规划器。`planning_plugin` 参数在某些配置路径下被忽略。

**解决：**
卸载 CHOMP 规划器包，确保 OMPL 是唯一可用的规划器：
```bash
sudo apt remove ros-humble-moveit-planners-chomp
```

---

## Bug 4：move_group 启动崩溃 — YAML 参数格式错误

**现象：**
```
[FATAL] Cannot have a value before ros__parameters
```
move_group 进程启动后立即崩溃。

**根因：**
`move_group_params.yaml`（或类似命名的 ROS 参数文件）格式不正确。ROS2 Humble 的 `rclcpp` 要求参数文件必须以 `/**` 节点名开头，然后是 `ros__parameters` 键：

```yaml
# ❌ 错误格式
planning_plugin: "ompl_interface/OMPLPlanner"

# ✅ 正确格式
/**:
  ros__parameters:
    planning_plugin: "ompl_interface/OMPLPlanner"
```

**解决：**
最终放弃了独立的参数 YAML 文件，改为在 launch 文件中通过 Python 字典传递参数（`moveit_config.to_dict()`），这由 `MoveItConfigsBuilder` 自动处理。

---

## Bug 5：Ros2ControlManager 崩溃 — 插件未找到

**现象：**
```
[FATAL] [moveit_ros_control_interface]: The 'moveit_ros_control_interface/Ros2ControlManager' plugin failed to load
```
move_group 启动后崩溃。

**根因：**
`Ros2ControlManager` 需要特定的插件库，在当前 ROS2 Humble apt 安装的版本中不可用或未正确注册。

**解决：**
切换为 `MoveItSimpleControllerManager`，配置 `FollowJointTrajectory` 动作接口：

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - left_arm_controller
    - right_arm_controller
  left_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints: [left_joint_1, ..., left_joint_6]
```

---

## Bug 6：OMPL 配置 "Could not find the planner configuration 'RRTConnect'"

**现象：**
move_group 日志报错找不到 `RRTConnect` 规划器配置。

**根因：**
`ompl_planning.yaml` 中设置了一个指向不存在配置名称的 `default_planner_config`：
```yaml
# ❌ 错误
default_planner_config: RRTConnect
```
实际的配置名是 `RRTConnectkConfigDefault`（带 `kConfigDefault` 后缀）。

**注意：** `longest_valid_segment_fraction` **不是** MoveIt1 专属配置项，在 MoveIt2 中仍然有效且重要。详见 Bug 7。

**解决：**
删除错误的 `default_planner_config: RRTConnect`（配置名缺少 `kConfigDefault` 后缀）。

---

## Bug 7：路径验证阶段发现隐藏碰撞 — 缺少路径细分参数

**现象：**
- OMPL 规划成功，找到数百个路径点的轨迹
- 但 `ValidateSolution` 阶段报告路径中有多个碰撞点
- move_group 日志：
  ```
  [ERROR] Computed path is not valid. Invalid states at index locations: [ 0 12 13 16 17 18 19 20 21 22 ] out of 659.
  [INFO] Motion plan was found but it seems to be invalid (possibly due to postprocessing). Not executing.
  ```
- 碰撞对：`left_Link_06 ↔ right_Link_05`、`left_Link_02 ↔ right_Link_03`
- 规划"成功"但验证失败，问题间歇性出现（OMPL 随机性导致）

**根因分析：**

这是 OMPL 碰撞检测粒度的问题。MoveIt 在规划时对路径的碰撞检查分为两个阶段：

```
阶段1（规划器端）：OMPL 每生成一个路径点，调用 FCL 检查该点的碰撞
   ┌───┐         ┌───┐         ┌───┐
   │ A │─────────│ B │─────────│ C │   路径点
   └───┘         └───┘         └───┘
     ✅            ✅            ✅     每个点单独通过

阶段2（验证端）：ValidateSolution 检查路径段之间的所有插值点
   ┌───┐    ┌┐┌┐┌    ┌───┐    ┌┐┌┐┌┐    ┌───┐
   │ A │────│││││────│ B │────││││││────│ C │
   └───┘    └┘└┘└┘    └───┘    └┘└┘└┘    └───┘
     ✅     ↑ 中间插值点    ✅     ↑ 中间插值点
            ❌ 有碰撞！            ❌ 有碰撞！
```

**关键参数：`longest_valid_segment_fraction`**

这个参数控制路径段的**最大允许角度跨度**（占关节运动范围的百分比）：

| 参数值 | 含义 | 效果 |
|--------|------|------|
| 未设置 | 默认值（可能较粗） | 跨臂碰撞未检测到 ❌ |
| `0.005` | 关节范围的 0.5% | JAKA C5: ~0.06 rad/段 → 密集检查 ✅ |

**原理：** OMPL RRTConnect 生成的路径点间距不均匀。如果 A 到 B 之间某个关节转动了 0.5 rad，而 `longest_valid_segment_fraction=0.005`（等效约 0.06 rad），MoveIt 会把 A→B 拆成 ~8 段，对每段末端的姿态逐一调用 FCL 碰撞检测。

不设这个参数时，MoveIt 用默认值。在双机械臂场景下——两个臂的工作空间深度重叠——默认粒度不足以捕获路径段中间的跨臂碰撞。

**解决：**

在 `ompl_planning.yaml` 中为每个规划组添加 `longest_valid_segment_fraction: 0.005`：

```yaml
left_arm:
  longest_valid_segment_fraction: 0.005    # ← 新增：强制细分路径段
  planner_configs:
    - SBLkConfigDefault
    ...

right_arm:
  longest_valid_segment_fraction: 0.005    # ← 新增：强制细分路径段
  planner_configs:
    - RRTConnectkConfigDefault
    ...
```

`0.005` 表示相邻路径点之间任意关节的转动量不能超过关节总行程的 0.5%。对于 JAKA C5（J1/J5/J6 行程 ±6.28 rad ≈ 12.56 rad），每段最大跨度 = 12.56 × 0.005 ≈ 0.063 rad ≈ 3.6°。

**注意：** 这个参数使碰撞检测更密集，规划耗时略有增加（通常 < 10%），但对于双机械臂防碰撞场景是**必须的**。

---

## 总结

| Bug | 类别 | 严重程度 | 修复方式 |
|---|---|---|---|
| AddTimeOptimalParameterization 命名空间错误 | 配置 | 🔴 阻塞 | 移到 request_adapters + 正确命名空间 |
| 路径验证发现隐藏碰撞 | 配置 | 🔴 阻塞 | 添加 `longest_valid_segment_fraction: 0.005` |
| 交互标记无法平移 | 配置 | 🟡 中等 | 删除不完整的 end effector 定义 |
| CHOMP 被误选 | 依赖 | 🟡 中等 | 卸载 CHOMP 包 |
| YAML 参数格式错误 | 配置 | 🔴 崩溃 | 改用 Python 字典传参 |
| Ros2ControlManager 加载失败 | 插件 | 🔴 崩溃 | 改用 MoveItSimpleControllerManager |
| OMPL 配置名无效 | 配置 | 🟡 中等 | 删除无效的 default_planner_config |

**核心教训：** 
1. 参考官方单机械臂配置（panda_moveit_config）是验证双机械臂配置正确性的最可靠方法。当双机械臂配置出现问题时，逐项对比与单机械臂配置的差异是最有效的调试策略。
2. 双机械臂场景下，跨臂碰撞检测需要比单臂更高的路径分辨率。`longest_valid_segment_fraction` 是容易被忽略但至关重要的参数。
