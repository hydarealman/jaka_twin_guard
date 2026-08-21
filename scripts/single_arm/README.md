# 单臂水果抓取启动脚本

这些脚本面向 WSL2/Ubuntu + ROS 2 Humble。首次使用前需要先构建工作空间：

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to jaka_single_arm
```

之后可直接使用以下入口。

## 方案 A 实车

启动：

```bash
bash scripts/single_arm/start_architecture_a_real.sh
```

脚本会自动寻找 `/dev/serial/by-id/` 下的电控板串口，默认波特率为 `115200`。
如果没有找到稳定设备路径，则使用 `/dev/ttyUSB0`。
脚本中的 `MODEL_LICENSE_APPROVED` 默认是 `true`，只有在模型许可审核完成后才应保持为 `true`。

关闭：

```bash
bash scripts/single_arm/stop_architecture_a_real.sh
```

## 方案 A 仿真

启动完整仿真：

```bash
bash scripts/single_arm/start_architecture_a_sim.sh
```

只启动场景和 MoveIt、不执行自动抓取：

```bash
bash scripts/single_arm/start_architecture_a_sim.sh run_task:=false
```

无 GUI：

```bash
bash scripts/single_arm/start_architecture_a_sim.sh \
  gui:=false start_rviz:=false start_image_view:=false
```

关闭：

```bash
bash scripts/single_arm/stop_architecture_a_sim.sh
```

## 方案 B 仿真

启动：

```bash
bash scripts/single_arm/start_architecture_b_sim.sh
```

关闭：

```bash
bash scripts/single_arm/stop_architecture_b_sim.sh
```

## 方案 B 实车

启动：

```bash
bash scripts/single_arm/start_architecture_b_real.sh
```

关闭：

```bash
bash scripts/single_arm/stop_architecture_b_real.sh
```

每个启动实例的日志保存在：

```text
.runtime/single_arm/logs/
```

脚本只记录并停止自己启动的 ROS launch 进程组，不使用 `pkill ros2` 或 `pkill gzserver`，因此不会主动结束其他任务。
