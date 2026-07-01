#!/bin/bash
set -e
echo "=== Starting Old Carry Demo ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch dual_arm_jaka_c5_moveit_config carry_object_demo.launch.py
