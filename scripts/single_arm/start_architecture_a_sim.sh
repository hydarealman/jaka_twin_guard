#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment

echo "[single-arm] starting Architecture A simulation debug mode"
echo "[single-arm] Gazebo is the only camera/robot data source; no real serial device is used"
echo "[single-arm] RGB/depth OpenCV debug windows enabled; automatic rqt windows disabled"

start_launch "a_sim" "architecture_a_sim.launch.py" \
  "gui:=false" \
  "start_rviz:=true" \
  "start_debug_view:=true" \
  "start_image_view:=false" \
  "start_perception:=true" \
  "run_task:=false"
