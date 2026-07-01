#!/bin/bash
set -e
echo "=== Starting Single-Arm Pick-and-Place ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch single_arm_jaka_c5_pick_place pick_place_demo.launch.py
