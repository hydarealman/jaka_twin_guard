# D455 眼在手外标定（ROS 2 / C++）

本项目的相机固定在机械臂外部，标定板刚性安装在 `tool_flange`。求解器观测：

```text
base_link_T_camera_color_optical_frame
```

保存时利用 RealSense 已发布的内部外参换算并只发布 `base_link -> camera_link`。
`camera_link -> camera_color_optical_frame` 和深度/彩色外参继续由 RealSense 驱动拥有，
从而保证每个 TF child 只有一个 parent。不要单独标定深度光学帧。

## 为什么项目自带 C++ 程序

- `easy_handeye2` 支持 ROS 2 和 eye-on-base，但主体为 Python，许可证为 LGPL-3.0。
- MoveIt Calibration 有 C++ 图形界面和 eye-to-hand 支持，但公开的完整教程和发行工作流主要面向 ROS 1。
- 本项目因此只依赖 ROS 2、OpenCV、cv_bridge 和 tf2，调用 OpenCV 官方
  `calibrateHandEye` 的五种求解器，并用闭环残差自动选择结果。

D455 不需要特殊手眼标定驱动；`realsense2_camera` 发布的彩色图像、`CameraInfo`
和 `camera_color_optical_frame` 就是输入。相机自身的 on-chip/self calibration 不能
代替机械臂与相机之间的手眼标定。

## 标定板参数必须先改

编辑 `config/eye_to_hand_calibration.yaml`。默认的 30 mm / 22 mm 只是示例：

- ChArUco 的 `charuco_squares_x/y` 是方格数量；
- `square_length_m` 是一个方格的实测边长；
- `marker_length_m` 是黑色 ArUco 标记的实测边长；
- 如果用普通棋盘格，选择 `target_type: chessboard`，`corners_x/y` 填内角点数。

长度填错 1%，最终平移尺度大约也会错 1%。玻璃板应装在刚性可拆支架上，软胶只作
薄层缓冲，不能让板相对法兰晃动。整个采样期间相机和标定板支架都不能移动。

## 编译与启动

```bash
cd /mnt/d/jaka_twin_guard
source /opt/ros/humble/setup.bash
colcon build --packages-select fruit_picking_arm --symlink-install
source install/setup.bash

ros2 launch fruit_picking_arm eye_to_hand_calibration.launch.py \
  serial_port:=/dev/ttyUSB0
```

该启动脚本只启动 D455、机器人 TF、方案 A 串口控制、MoveIt、RViz 和 C++ 标定节点，
不会启动水果识别或自动夹取。标定时不要同时启动 `hand_eye_static_tf`，否则会形成重复 TF。

观察识别叠加图：

```bash
ros2 run rqt_image_view rqt_image_view /hand_eye_calibration/annotated
```

如果 D455 由另一个终端启动，可加 `start_camera:=false`。如果只调试识别而没有电控板，
还可以加 `start_moveit:=false start_rviz:=false`，但无法采集机械臂位姿样本。

## 采样、求解与保存

每次让机械臂完全停止，保证标定板清晰、完整或大部分可见，然后执行一次：

```bash
ros2 service call /hand_eye_calibration/capture std_srvs/srv/Trigger '{}'
```

建议采集 20～30 个姿态。位置要覆盖工作区，尤其要让末端绕至少两个不同方向旋转；只做
平移无法可靠求解旋转。程序会拒绝相邻的重复姿态、过期图像、重投影误差过大的图像，且
总旋转跨度低于 15° 时不求解。

常用服务：

```bash
ros2 service call /hand_eye_calibration/status std_srvs/srv/Trigger '{}'
ros2 service call /hand_eye_calibration/remove_last std_srvs/srv/Trigger '{}'
ros2 service call /hand_eye_calibration/clear std_srvs/srv/Trigger '{}'
ros2 service call /hand_eye_calibration/solve std_srvs/srv/Trigger '{}'
ros2 service call /hand_eye_calibration/save std_srvs/srv/Trigger '{}'
```

默认质量门槛为平移闭环 RMS 不超过 5 mm、旋转 RMS 不超过 1°。失败结果默认禁止保存。
保存会生成：

- `hand_eye_params.yaml`：实车启动直接读取的静态 TF；
- `hand_eye_params.yaml.samples.yaml`：所有原始变换和误差，便于审计和复算；
- 旧结果的 `.bak` 备份。

## 实车验收

重新启动 `architecture_a_real.launch.py` 后检查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

然后把一个可识别目标放在固定位置，从不同视角/不同机械臂姿态测量其 `base_link` 坐标。
建议工作区内至少验证 10 个点；除了求解器的闭环 RMS，还要记录实际三维定位误差。若相机、
相机支架、机械臂底座或标定板安装关系改变，必须重新标定。
