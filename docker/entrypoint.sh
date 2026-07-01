#!/bin/bash
# ============================================================
# JAKA Twin Guard — Container Entrypoint
# 负责: source ROS2 setup, 设置 Gazebo 环境, 清理残留进程
# ============================================================
set -e

echo "=== JAKA Twin Guard Container ==="
echo "ROS2 Distro: ${ROS_DISTRO:-humble}"
echo "Display: ${DISPLAY:-not set}"
echo ""

# 1. Source ROS2 Humble setup
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[entrypoint] Sourced /opt/ros/humble/setup.bash"
else
    echo "[entrypoint] WARNING: /opt/ros/humble/setup.bash not found!"
fi

# 2. Source colcon workspace (if already built)
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
    echo "[entrypoint] Sourced /workspace/install/setup.bash"
else
    echo "[entrypoint] Workspace not built yet. Run /launch/build.sh first."
fi

# 3. Gazebo 资源路径 — 必须用 APPEND 模式, 不能覆盖原有值!
#    原有路径来自 /usr/share/gazebo-11/setup.sh
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}"
export GAZEBO_PLUGIN_PATH="/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}"

# 禁用 Gazebo 在线模型下载 (WSL2 网络环境下避免超时卡顿)
export GAZEBO_MODEL_DATABASE_URI=""

# 4. 软件渲染 (Mesa llvmpipe) — 由环境变量控制, 默认开启
#    GPU 路径: 设置 LIBGL_ALWAYS_SOFTWARE=0 并通过 NVIDIA runtime 启动
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_QUICK_BACKEND="${QT_QUICK_BACKEND:-software}"

echo "[entrypoint] LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE}"
echo "[entrypoint] GAZEBO_MODEL_DATABASE_URI=${GAZEBO_MODEL_DATABASE_URI}"

# 5. 清理残留 Gazebo 进程 (上次崩溃可能残留)
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true

# 6. ROS 日志目录
export ROS_LOG_DIR="${ROS_LOG_DIR:-/workspace/log}"
mkdir -p "$ROS_LOG_DIR"

echo "[entrypoint] Ready. Executing: $@"
echo ""

# 7. 执行用户命令
exec "$@"
