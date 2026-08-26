#!/usr/bin/env bash
set -eo pipefail

# Physical execution entry point: no RViz, rqt or OpenCV debug windows.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
report_real_device_visibility
ensure_apple_detector_model

PORT="$(find /dev/serial/by-id -maxdepth 1 -type c 2>/dev/null | head -n 1 || true)"
if [[ -z "${PORT}" ]]; then
  PORT="/dev/ttyUSB0"
  echo "[fruit-arm] serial unavailable; execution will remain fail-closed"
else
  echo "[fruit-arm] detected serial device: ${PORT}"
fi

echo "[fruit-arm] starting Architecture A physical execution without debug windows"
echo "[fruit-arm] verify E-stop, zero positions, limits, READY state and hand-eye calibration"

start_launch "a_real" "architecture_a_real.launch.py" \
  "serial_port:=${PORT}" \
  "baudrate:=115200" \
  "color_profile:=${D455_COLOR_PROFILE:-424,240,15}" \
  "depth_profile:=${D455_DEPTH_PROFILE:-424,240,15}" \
  "start_rviz:=false" \
  "start_debug_view:=false" \
  "start_image_view:=false" \
  "start_perception:=false" \
  "run_task:=true" \
  "model_license_approved:=true"

require_real_rgbd_frames "a_real" 40
