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
  echo "[single-arm] serial unavailable; camera and debug components will still start"
else
  echo "[single-arm] detected serial device: ${PORT}"
fi
BAUDRATE="115200"
COLOR_PROFILE="${D455_COLOR_PROFILE:-424,240,15}"
DEPTH_PROFILE="${D455_DEPTH_PROFILE:-424,240,15}"
# 模型许可审核完成后使用 true；否则改为 false。
MODEL_LICENSE_APPROVED="true"
START_ROBOT_STACK="${JAKA_START_ROBOT_STACK:-false}"

if [[ "${START_ROBOT_STACK}" == "true" ]]; then
  echo "[single-arm] robot stack explicitly enabled: MoveIt and serial will start"
  echo "[single-arm] WARNING: starting physical hardware on ${PORT}"
  echo "[single-arm] verify E-stop, power state, zero positions, limits and hand-eye calibration"
else
  echo "[single-arm] perception-only bring-up: MoveIt and serial are disabled"
  echo "[single-arm] set JAKA_START_ROBOT_STACK=true only after perception, calibration and hardware checks"
fi

echo "[single-arm] RViz is disabled in the low-latency bring-up; start it separately when needed"

start_launch "a_real" "architecture_a_real.launch.py" \
  "serial_port:=${PORT}" \
  "baudrate:=${BAUDRATE}" \
  "color_profile:=${COLOR_PROFILE}" \
  "depth_profile:=${DEPTH_PROFILE}" \
  "enable_temporal_filter:=false" \
  "start_rviz:=false" \
  "start_debug_view:=true" \
  "start_image_view:=false" \
  "start_perception:=true" \
  "start_robot_stack:=${START_ROBOT_STACK}" \
  "run_task:=false" \
  "model_license_approved:=${MODEL_LICENSE_APPROVED}"

wait_for_real_rgbd_frames 40
wait_for_topic_message "/perception/apple_detections_2d" 45
