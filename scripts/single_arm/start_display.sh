#!/usr/bin/env bash
# Monitor-only display for the physical monitor: RViz + OpenCV debug windows.
#
# Purely subscribes to the already-running real task (robot model, fruit
# markers, camera images).  It never touches motion, services or the serial
# board, so it is safe to start/stop at any time without affecting picking.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
ensure_runtime_dir

RVIZ_CONFIG="$(ros2 pkg prefix fruit_arm_moveit_config)/share/fruit_arm_moveit_config/config/fruit_picking_arm.rviz"
[[ -f "${RVIZ_CONFIG}" ]] || die "RViz config not found: ${RVIZ_CONFIG}"

# ROS Humble's Qt ships without a wayland platform plugin, so both rviz2 and
# the OpenCV windows must use xcb.  GNOME Wayland serves them through
# Xwayland; this is invisible to the user and needed on every session type.
export QT_QPA_PLATFORM=xcb

echo "[fruit-arm] starting monitor-only display: RViz + OpenCV debug windows"
nohup setsid ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}" --qwindowgeometry 1400x900+40+40 \
  > "${LOG_DIR}/display_rviz.log" 2>&1 < /dev/null &
nohup setsid ros2 run fruit_picking_arm fruit_debug_window \
  --ros-args -p display_rate:=10.0 \
  > "${LOG_DIR}/display_debug.log" 2>&1 < /dev/null &

echo "[fruit-arm] display started (logs: .runtime/single_arm/logs/display_*.log)"
