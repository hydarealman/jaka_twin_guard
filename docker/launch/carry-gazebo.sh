#!/bin/bash
set -e
echo "=== Starting Gazebo Carry Demo ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch jaka_dual_arm sim_gazebo.launch.py
