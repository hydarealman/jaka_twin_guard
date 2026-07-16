# 四种独立启动模式

四个入口按“控制方案 × 运行环境”拆分。不要同时启动两个入口。

| 控制方案 | 仿真入口 | 实车入口 | 上位机发送内容 |
|---|---|---|---|
| A：上位机规划 | `architecture_a_sim.launch.py` | `architecture_a_real.launch.py` | 关节轨迹 |
| B：C 板规划 | `architecture_b_sim.launch.py` | `architecture_b_real.launch.py` | 水果三维目标 |

## 方案 A：仿真

```bash
ros2 launch jaka_single_arm architecture_a_sim.launch.py
```

启动 Gazebo、仿真眼在手外 RGB-D 相机、果品识别、MoveIt、RViz 和抓取任务。
它不打开真实串口。MoveIt 轨迹由 Gazebo `ros2_control` 执行，以便看到机械臂运动。
释放水果时，仿真专用场景桥会把对应 Gazebo 水果模型放入好果或坏果料框；这用于演示
分拣结果，不代表已经验证真实夹爪的接触、摩擦和负载能力。

如果只想检查场景和相机，不执行抓取：

```bash
ros2 launch jaka_single_arm architecture_a_sim.launch.py run_task:=false
```

## 方案 A：实车

```bash
ros2 launch jaka_single_arm architecture_a_real.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200
```

启动 RealSense、识别、MoveIt 和串口轨迹控制器。上位机把完整关节轨迹发给 C 板。
此入口不启动 Gazebo。

## 方案 B：仿真

```bash
ros2 launch jaka_single_arm architecture_b_sim.launch.py
```

启动 Gazebo 相机、三维水果目标生成、虚拟串口和 C 板协议模拟器。它验证
`FruitTarget -> ACK -> MotionResult` 通信闭环，不打开真实 USB 串口。

方案 B 的机械臂运动由 C 板中的 IK、轨迹和电机控制固件负责。在这部分固件尚未实现时，
PC 侧模拟器只确认目标和协议，不能真实复现 B 方案的机械臂运动。

## 方案 B：实车

```bash
ros2 launch jaka_single_arm architecture_b_real.launch.py \
  serial_port:=/dev/ttyUSB0 baudrate:=115200
```

启动 RealSense、果品识别、稳定三维目标跟踪和目标串口桥。上位机只发送水果坐标、
半径、好坏类别和目标编号；MoveIt、行为树和轨迹控制器不会启动。

## 启动前通用步骤

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
source install/setup.bash
```

实车入口还必须完成手眼标定、串口权限、坐标系核对、急停和软硬限位检查。
