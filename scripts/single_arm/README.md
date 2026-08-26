# 单臂水果抓取启动脚本

这些脚本面向 WSL2/Ubuntu + ROS 2 Humble。首次使用前需要先构建工作空间：

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to fruit_picking_arm
```

之后可直接使用以下入口。
启动脚本会自动加载 `/opt/ros/humble/setup.bash` 和当前工作空间的 `install/setup.bash`，不需要提前手动执行 `source`。

## URDF 零位、方向和限位检查

只启动机械臂 URDF、关节滑块和 RViz，不启动相机、串口、MoveIt
轨迹执行器、`ros2_control` 或实车电机：

```bash
bash scripts/single_arm/start_model_joint_check.sh
```

在 `joint_state_publisher_gui` 中拖动 `joint_1`～`joint_6`；滑块范围是
URDF/Xacro 的硬限位。精确角度（弧度）可在另一个终端查看：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /joint_states
```

启动终端按 `Ctrl+C` 即可关闭全部模型检查节点。该模式不能用于验证 MoveIt
软限位，也不会向电控发送任何数据。

## D455 安全 RGB-D 调试入口

只验证相机、YOLO 苹果检测、MobileNetV3 好坏分类和 RGB/深度调试话题，不启动 MoveIt、串口、电机或抓取任务：

```bash
bash scripts/single_arm/start_d455_fruit_debug.sh
```

脚本会按 `VID:PID=8086:0b5c` 查找 Windows 侧 D455，自动请求 `usbipd.exe attach --wsl Ubuntu-22.04 --busid <BUSID> --auto-attach`，不会执行 `initial_reset` 或 detach。权限不足时会把管理员 PowerShell 命令写到终端和 `.runtime/single_arm/logs/usbipd_attach.log`。默认使用 WSL 已验证的 `424,240,15`；可用环境变量试验其它档位：

```bash
D455_COLOR_PROFILE=640,480,15 D455_DEPTH_PROFILE=640,480,15 \
  bash scripts/single_arm/start_d455_fruit_debug.sh
```

WSL 当前实测 `640×480×15 RGB + 424×240×15 depth` 会间歇产生下半帧灰屏，`640×480×15` 双流会严重掉帧，`848×480×15` 会超时，因此实机默认固定为双流 `424×240×15`。原生 Ubuntu/USB3 可在确认完整帧和同步稳定后覆盖为更高分辨率。

停止：

```bash
bash scripts/single_arm/stop_d455_fruit_debug.sh
```

安全入口只接受 RGB 与彩色对齐深度时间差不超过 33 ms 的帧对；USB/IP 造成的深度传输延迟超过该门限时会显示等待状态并丢弃该帧，不会用旧深度或仿真数据制造目标。

## 方案 A 实车

调试启动（RGB、深度、OpenCV水果窗口和持续感知；默认关闭 RViz 以保护 USB/IP 帧率）：

```bash
bash scripts/single_arm/start_architecture_a_real.sh
```

脚本会自动寻找 `/dev/serial/by-id/` 下的电控板串口，默认波特率为 `115200`。
如果没有找到稳定设备路径，则使用 `/dev/ttyUSB0`。
启动后会打开水果 RGB 识别窗口和 D455 深度伪彩窗口。A 实车脚本默认只做设备、相机、识别和串口状态调试，不自动执行抓取动作；需要 RViz 时可在命令后增加 `start_rviz:=true`，但 WSL 软件渲染会明显抢占 CPU。
RGB 和深度窗口由独立的 `fruit_debug_window` 创建，同时发布 `/perception/debug/fruit_view` 和 `/perception/debug/depth_view` 话题。默认不自动启动 rqt，避免多个空白 rqt 窗口干扰调试；如需 rqt，可手动运行 `ros2 run rqt_image_view rqt_image_view` 后选择这两个调试话题。
脚本中的 `MODEL_LICENSE_APPROVED` 默认是 `true`，只有在模型许可审核完成后才应保持为 `true`。

实车感知将 YOLO CPU 推理限制为单线程、检测频率约 5 Hz，调试图发布/窗口刷新约 10 Hz；这是为了避免 PyTorch、OpenCV、RViz 和 RealSense USB/IP 同时抢占全部 CPU。若苹果在 424×240 画面中只有十几像素，COCO 权重无法保证召回，应优先使用原生 Ubuntu 的 640×480 彩色 profile，或把苹果放到至少约 30 像素直径后再验收。

没有串口时脚本不会退出：相机、识别和调试窗口仍会启动，串口节点会在后台每 2 秒重试。没有 D455 时相机节点同样独立等待/重启，调试窗口显示 `CAMERA: WAITING/OFFLINE`，不会显示旧图像或生成仿真水果。实车模式不会启动 Gazebo、MockCamera、串口模拟器或 `socat`，也不会用仿真数据替代真实数据。

关闭：

```bash
bash scripts/single_arm/stop_architecture_a_real.sh
```

实车运行启动（不打开任何调试窗口，执行任务）：

```bash
bash scripts/single_arm/start_architecture_a_real_run.sh
```

关闭仍使用：

```bash
bash scripts/single_arm/stop_architecture_a_real.sh
```

如果是在 Windows + WSL2 中运行，D455 必须先挂载到 WSL。Windows PowerShell（管理员）执行：

```powershell
usbipd list
usbipd.exe attach --wsl Ubuntu-22.04 --busid <D455_BUSID>
```

然后在 WSL 检查：

```bash
lsusb | grep -i 8086
```

启动脚本会自动检查这一点。只有 ROS 日志出现 `RealSense devices were found` 且 `/camera/camera/color/image_raw` 有发布者时，窗口才会有真实画面。

## 方案 A 仿真

仿真调试启动（与实车调试显示一致，但数据严格来自 Gazebo）：

```bash
bash scripts/single_arm/start_architecture_a_sim.sh
```

该脚本会启动 Gazebo server、MoveIt、RViz、仿真水果感知、RGB 窗口和深度窗口，但不会自动执行抓取；它不会打开实车串口，也不会被实车故障触发。默认关闭 Gazebo client，避免 WSL 软件渲染抢占相机帧率；RViz 和两个图像窗口仍会打开。仿真感知允许 Gazebo 专用的场景桌面高度辅助，这些参数不会传入实车启动链。仿真相机使用 424×240×15，与实车调试档位一致；在 WSL 下 Gazebo Classic 的深度点云实时帧率受主机性能影响，低于实车 D455 是正常的仿真性能限制。

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

调试启动（只识别和显示，不向电控板发送水果目标）：

```bash
bash scripts/single_arm/start_architecture_b_real.sh
```

关闭：

```bash
bash scripts/single_arm/stop_architecture_b_real.sh
```

实车运行启动（不打开调试窗口，允许向电控板发送新鲜目标）：

```bash
bash scripts/single_arm/start_architecture_b_real_run.sh
```

关闭仍使用：

```bash
bash scripts/single_arm/stop_architecture_b_real.sh
```

方案 B 只有在真实相机数据新鲜、串口电控板重新报告 `READY` 且水果目标未过期时才会下发目标；相机或串口断开期间不会继续发送旧目标。

每个启动实例的日志保存在：

```text
.runtime/single_arm/logs/
```

脚本只记录并停止自己启动的 ROS launch 进程组，不使用 `pkill ros2` 或 `pkill gzserver`，因此不会主动结束其他任务。

## 实车调试窗口

水果窗口显示 `/perception/debug/fruit_view`：没有识别结果时显示原始 D455 彩色图像；识别运行后显示水果标注、目标数量、目标坐标和 `/joint_states` 状态。
深度窗口显示 `/perception/debug/depth_view`：将真实 `Z16/16UC1` 或 `32FC1` 深度转换为可读的 Turbo 伪彩，不再直接显示接近全黑的 16 位原始图像。
窗口左上角会显示实际收到的 RGB/深度帧率。当前 WSL2/usbipd 默认使用 424×240×15 的 RGB-D 流，实测图像约 14 Hz；640×480 配置在当前 USB/IP 链路上会触发 D455 帧超时。原始深度由感知节点按真实相机内参生成紧凑 XYZ 点云，避免驱动高密度点云拖慢取流。
RViz 用于查看真实关节反馈驱动的机械臂姿态、D455 相机坐标系原始点云、实测桌面点云、水果标记和 `/perception/detected_objects` 三维标记。原始点云在未完成手眼标定时以相机坐标系显示，不会假装与机械臂对齐；完成标定后，`Measured Table Plane`、水果点云和水果标记才会与机器人坐标系正确重合。当前 URDF 的机械臂基座名是 `Link_00`，不是 `base_link`。串口未连接或电控板未发布真实 `/joint_states` 时，RViz 不会伪造初始关节姿态。

如果窗口没有出现，先在另一个终端检查：

```bash
ros2 topic list | grep -E 'camera|perception|joint_states'
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /perception/debug/fruit_view
```

确认要自动抓取时，再手动启动：

```bash
ros2 launch fruit_picking_arm architecture_a_real.launch.py run_task:=true
```
