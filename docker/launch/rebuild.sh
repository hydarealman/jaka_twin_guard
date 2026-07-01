#!/bin/bash
set -e
echo "=== Clean rebuild ==="
cd /workspace
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build \
    --packages-select jaka_dual_arm dual_arm_jaka_c5_moveit_config jaka_c5_description \
    --symlink-install \
    --event-handlers console_direct+
echo "=== Rebuild complete ==="
