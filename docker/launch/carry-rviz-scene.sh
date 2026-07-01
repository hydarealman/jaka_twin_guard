#!/bin/bash
set -e
SCENE="${1:-a}"
echo "=== Starting RViz Carry Demo (Scene: ${SCENE}) ==="
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 launch jaka_dual_arm sim_rviz.launch.py scene:=${SCENE}
