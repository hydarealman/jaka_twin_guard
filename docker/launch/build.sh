#!/bin/bash
# ============================================================
# 构建 JAKA Twin Guard workspace (colcon)
# 只编译需要的 3 个包: jaka_dual_arm, dual_arm_jaka_c5_moveit_config, jaka_c5_description
# ============================================================
set -e

echo "=== Building JAKA Twin Guard workspace ==="
cd /workspace

source /opt/ros/humble/setup.bash

colcon build \
    --packages-select jaka_dual_arm dual_arm_jaka_c5_moveit_config jaka_c5_description \
    --symlink-install \
    --event-handlers console_direct+

echo ""
echo "=== Build complete ==="
echo "Source with: source /workspace/install/setup.bash"
