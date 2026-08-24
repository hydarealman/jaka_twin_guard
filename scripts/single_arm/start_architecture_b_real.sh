#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
report_real_device_visibility
ensure_apple_detector_model

PORT="$(find /dev/serial/by-id -maxdepth 1 -type c 2>/dev/null | head -n 1 || true)"
if [[ -z "${PORT}" ]]; then
  PORT="/dev/ttyUSB0"
  echo "[fruit-arm] serial unavailable; camera and debug components will still start"
  echo "[fruit-arm] serial bridge will retry ${PORT} in the background"
else
  echo "[fruit-arm] detected serial device: ${PORT}"
fi
BAUDRATE="115200"
# 模型许可审核完成后使用 true；否则改为 false。
MODEL_LICENSE_APPROVED="true"

echo "[fruit-arm] WARNING: starting physical hardware through Architecture B on ${PORT}"
echo "[fruit-arm] the C board owns IK, trajectory generation and gripper sequencing"

start_launch "b_real" "architecture_b_real.launch.py" \
  "serial_port:=${PORT}" \
  "baudrate:=${BAUDRATE}" \
  "color_profile:=${D455_COLOR_PROFILE:-424,240,15}" \
  "depth_profile:=${D455_DEPTH_PROFILE:-424,240,15}" \
  "start_debug_view:=true" \
  "start_image_view:=true" \
  "start_target_bridge:=false" \
  "model_license_approved:=${MODEL_LICENSE_APPROVED}"
