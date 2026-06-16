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

**解决：**
删除 `default_planner_config` 和 `longest_valid_segment_fraction` 键（这些是 MoveIt1 的配置项，MoveIt2 中不再支持或移动到其他位置）。

---

## 总结

| Bug | 类别 | 严重程度 | 修复方式 |
|---|---|---|---|
| AddTimeOptimalParameterization 命名空间错误 | 配置 | 🔴 阻塞 | 移到 request_adapters + 正确命名空间 |
| 交互标记无法平移 | 配置 | 🟡 中等 | 删除不完整的 end effector 定义 |
| CHOMP 被误选 | 依赖 | 🟡 中等 | 卸载 CHOMP 包 |
| YAML 参数格式错误 | 配置 | 🔴 崩溃 | 改用 Python 字典传参 |
| Ros2ControlManager 加载失败 | 插件 | 🔴 崩溃 | 改用 MoveItSimpleControllerManager |
| OMPL 配置名无效 | 配置 | 🟡 中等 | 删除无效的 default_planner_config |

**核心教训：** 参考官方单机械臂配置（panda_moveit_config）是验证双机械臂配置正确性的最可靠方法。当双机械臂配置出现问题时，逐项对比与单机械臂配置的差异是最有效的调试策略。
