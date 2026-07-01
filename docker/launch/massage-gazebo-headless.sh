#!/bin/bash
# 无 GUI 按摩仿真 (Gazebo headless, 不需要 X11 显示)
set -e
echo "=== Starting Gazebo Massage (headless) ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py gui:=false
