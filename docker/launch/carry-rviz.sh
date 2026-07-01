#!/bin/bash
set -e
echo "=== Starting RViz Carry Demo ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch jaka_dual_arm sim_rviz.launch.py
