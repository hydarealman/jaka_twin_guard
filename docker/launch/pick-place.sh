#!/bin/bash
set -e
echo "=== Starting Single-Arm Pick-and-Place ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch fruit_arm_moveit_config pick_place_demo.launch.py
