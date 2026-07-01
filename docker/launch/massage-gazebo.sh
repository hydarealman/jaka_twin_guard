#!/bin/bash
# 工业级按摩仿真 — Gazebo 物理引擎 (推荐)
set -e
echo "=== Starting Gazebo Massage Simulation ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch jaka_dual_arm sim_gazebo_massage.launch.py
