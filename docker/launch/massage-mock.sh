#!/bin/bash
set -e
echo "=== Starting Mock Massage (RViz only, no Gazebo) ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch jaka_dual_arm industrial_massage.launch.py
