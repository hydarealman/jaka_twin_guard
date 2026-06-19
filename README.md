MoveIt Resources
================

This repository includes various resources (URDFs, meshes, moveit_config packages) needed for MoveIt testing.

GitHub Actions: [![Formatting (pre-commit))](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/format.yml?query=branch%3Aros2) [![Build and Test](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml/badge.svg?branch=ros2)](https://github.com/ros-planning/moveit_resources/actions/workflows/industrial_ci_action.yml?query=branch%3Aros2)

## Included Robots

- PR2
- Fanuc M-10iA
- Franka Emika Panda

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
