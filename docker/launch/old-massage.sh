#!/bin/bash
set -e
echo "=== Starting Old Massage Demo ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch dual_arm_jaka_c5_moveit_config massage_demo.launch.py
