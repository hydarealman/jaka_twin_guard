MoveIt Resources
================

This repository includes various resources (URDFs, meshes, moveit_config packages) needed for MoveIt testing.

GitHub Actions: [![Formatting (pre-commit))](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml?query=branch%3Aros2) [![Build and Test](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml?query=branch%3Aros2)

## Included Robots

- PR2
- Fanuc M-10iA
- Franka Emika Panda

## JAKA C5 双臂 RViz 按摩演示

本项目新增了一个独立的 RViz 双臂按摩演示，入口为：

- `src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/launch/massage_demo.launch.py`
- `src/moveit_resources-ros2/dual_arm_jaka_c5_moveit_config/scripts/dual_arm_massage_demo.py`

这个演示和双臂协同搬运物体的程序独立运行。按摩演示会在 RViz 中创建床、床垫、枕头和人体示意模型，然后让左右两个 JAKA C5 机械臂在床上方执行循环按摩动作。

滚压动作使用机械臂原本的 `Link_06` 末端法兰圆，不额外添加滚轮模型。脚本会在滚压阶段把法兰圆横过来，让原始法兰的圆柱弧形侧面贴近人体表面做前后滚压。

### 运行方式

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py
```

默认参数 `repeat_count:=0` 表示无限循环执行按摩流程，按 `Ctrl+C` 停止。

注意：如果命令里写了 `repeat_count:=1`，机械臂只会执行一轮按摩后停下；想保持连续循环，请不要传这个参数，或者显式传 `repeat_count:=0`。

如果只想执行固定轮数，例如 2 轮：

```bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py repeat_count:=2
```

如果希望动作更快或更慢，可以调节轨迹时间缩放：

```bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py trajectory_time_scale:=0.55
```

`trajectory_time_scale` 越小，播放越快；建议不要低于 `0.45`，否则 RViz 里动作会显得不太像按摩。

### 按摩动作流程

一轮按摩共有 35 个阶段。整体思路是先在肩背区域热身按压，再做左右交替揉捏和滚压，然后移动到中背区域做推按、二次压缩、掌心滚压、敲击式按压和法兰圆横向前后滚压，最后回到肩背区域收尾释放。

| 阶段 | 动作名称 | 说明 |
| --- | --- | --- |
| 1 | hover above shoulders | 双臂移动到肩部上方悬停位，作为一轮动作的起点。 |
| 2 | paired shoulder press | 左右双臂同步下压肩部区域，模拟双手同时按压。 |
| 3 | release shoulder pressure | 双臂从肩部按压位置回弹，释放压力。 |
| 4 | left shoulder knead | 左臂保持按压并通过末端角度变化模拟左侧肩部揉捏。 |
| 5 | right shoulder knead | 右臂执行对应的肩部揉捏动作。 |
| 6 | paired shoulder roll inward | 双臂腕部向内滚动，模拟掌心向内滚压。 |
| 7 | paired shoulder roll outward | 双臂腕部向外滚动，模拟掌心向外滚压。 |
| 8 | left shoulder percussion tap | 左臂做一次肩部敲击式按压，右臂保持避让姿态。 |
| 9 | right shoulder percussion tap | 右臂做一次肩部敲击式按压，左臂保持避让姿态。 |
| 10 | sweep toward mid back | 双臂从肩部区域沿床身方向推到中背区域。 |
| 11 | left mid-back knead | 左臂在中背区域做揉捏动作。 |
| 12 | right mid-back knead | 右臂在中背区域做揉捏动作。 |
| 13 | paired mid-back press | 双臂同步按压中背区域。 |
| 14 | release mid-back pressure | 双臂从中背按压位回弹。 |
| 15 | second mid-back compression | 双臂再次压向中背区域，形成二次压缩。 |
| 16 | roll palms inward | 双臂在中背区域做掌心向内滚压。 |
| 17 | roll palms outward | 双臂在中背区域做掌心向外滚压。 |
| 18 | left mid-back percussion tap | 左臂在中背区域做敲击式按压。 |
| 19 | right mid-back percussion tap | 右臂在中背区域做敲击式按压。 |
| 20 | center mid-back squeeze | 双臂在中背中心区域形成一次夹压/挤压式动作。 |
| 21 | horizontal original flange preload | 双臂把原始末端法兰圆横过来，贴近中背滚压起点，准备使用法兰圆柱弧面接触。 |
| 22 | original flange side-roll forward 1 | 左右原始法兰保持横向姿态，沿床身方向向前滚压一小段。 |
| 23 | original flange side-roll backward 1 | 双臂沿相反方向回滚，末端关节同步转动，模拟法兰圆柱弧面的回滚。 |
| 24 | original flange side-roll forward 2 | 第二次向前滚压，增强连续滚动按摩效果。 |
| 25 | original flange side-roll backward 2 | 第二次回滚，保持左右臂对称避让。 |
| 26 | original flange side-roll forward 3 | 第三次向前滚压，形成完整的前后往复滚动。 |
| 27 | original flange side-roll backward 3 | 第三次回滚，回到滚压起点附近。 |
| 28 | horizontal original flange release | 双臂从横置法兰滚压高度抬起，离开中背区域表面。 |
| 29 | return sweep | 双臂沿床身方向从中背区域回到肩部区域。 |
| 30 | return shoulder press | 回到肩部后再次同步按压，作为收尾动作的开始。 |
| 31 | left finishing knead | 左臂做最后一次肩部揉捏。 |
| 32 | right finishing knead | 右臂做最后一次肩部揉捏。 |
| 33 | paired finishing press | 左右双臂同步做最后一次按压。 |
| 34 | release shoulder pressure | 双臂从最终按压位释放。 |
| 35 | release to hover | 双臂回到肩部上方悬停位，准备进入下一轮循环。 |

### 碰撞与规划说明

按摩 demo 使用 MoveIt 的 `both_arms` 规划组进行双臂联合规划，因此左右机械臂之间的碰撞不会被单独忽略。脚本会先把床、床垫和枕头加入 MoveIt planning scene，随后对每一段动作调用 `/plan_kinematic_path` 规划轨迹。

轨迹规划完成后，脚本还会按时间采样调用 `/check_state_validity` 检查整条轨迹。只有所有采样状态都通过碰撞检查后，才会把同步轨迹发送给 `left_arm_controller` 和 `right_arm_controller` 执行。

法兰圆横置滚压阶段没有新增任何末端工具模型，仍然使用机器人原始 `Link_06` 几何。为了避免继续用端面向下按压，滚压阶段会把 `Link_06` 的局部 Z 轴从接近竖直调整为接近水平，再通过前后位移和 `joint_6` 转动表现圆柱弧面的滚动按摩。

### 常用参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `repeat_count` | `0` | 执行轮数。`0` 表示无限循环，正整数表示固定执行几轮。 |
| `trajectory_time_scale` | `0.65` | 对已规划轨迹做整体时间缩放。越小越快。 |
| `velocity_scaling` | `0.45` | MoveIt 规划阶段的速度缩放。 |
| `acceleration_scaling` | `0.45` | MoveIt 规划阶段的加速度缩放。 |
| `trajectory_start_delay` | `0.10` | 给控制器发送轨迹后，实际开始运动前的短延迟。 |

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
