#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
report_real_device_visibility
ensure_apple_detector_model

PORT=""
for candidate in /dev/serial/by-id/*; do
  if [[ -e "${candidate}" ]]; then
    PORT="${candidate}"
    break
  fi
done
if [[ -z "${PORT}" ]]; then
  PORT="/dev/ttyUSB0"
  echo "[fruit-arm] serial unavailable; camera and debug components will still start"
else
  echo "[fruit-arm] detected serial device: ${PORT}"
fi
BAUDRATE="115200"
COLOR_PROFILE="${D455_COLOR_PROFILE:-424,240,15}"
DEPTH_PROFILE="${D455_DEPTH_PROFILE:-424,240,15}"
# 模型许可审核完成后使用 true；否则改为 false。
MODEL_LICENSE_APPROVED="true"
# Real-hardware bring-up now starts the complete observation/planning stack by
# default.  run_task remains false below, so launching this script does not
# automatically submit a trajectory or start fruit picking.
START_ROBOT_STACK="${JAKA_START_ROBOT_STACK:-true}"
START_RVIZ="${JAKA_START_RVIZ:-true}"
ROBOT_SERIAL="${FRUIT_ARM_ROBOT_SERIAL:-}"
KINEMATICS_MODE="${FRUIT_ARM_KINEMATICS_MODE:-nominal}"
CALIBRATION_FILE="${FRUIT_ARM_CALIBRATION_FILE:-}"

if [[ "${START_ROBOT_STACK}" == "true" ]]; then
  [[ -n "${ROBOT_SERIAL}" ]] || \
    die "FRUIT_ARM_ROBOT_SERIAL is required when the physical robot stack is enabled"
  echo "[fruit-arm] robot stack explicitly enabled: MoveIt and serial will start"
  echo "[fruit-arm] robot=${ROBOT_SERIAL}, kinematics=${KINEMATICS_MODE}"
  echo "[fruit-arm] WARNING: starting physical hardware on ${PORT}"
  echo "[fruit-arm] verify E-stop, power state, zero positions, limits and hand-eye calibration"
else
  echo "[fruit-arm] perception-only bring-up: MoveIt and serial are disabled"
  echo "[fruit-arm] JAKA_START_ROBOT_STACK=false explicitly selected"
fi

if [[ "${START_RVIZ}" == "true" ]]; then
  echo "[fruit-arm] RViz enabled"
else
  echo "[fruit-arm] RViz disabled; set JAKA_START_RVIZ=true to enable it"
fi

start_launch "a_real" "architecture_a_real.launch.py" \
  "serial_port:=${PORT}" \
  "baudrate:=${BAUDRATE}" \
  "robot_serial:=${ROBOT_SERIAL}" \
  "kinematics_mode:=${KINEMATICS_MODE}" \
  "calibration_file:=${CALIBRATION_FILE}" \
  "color_profile:=${COLOR_PROFILE}" \
  "depth_profile:=${DEPTH_PROFILE}" \
  "enable_temporal_filter:=false" \
  "start_rviz:=${START_RVIZ}" \
  "start_debug_view:=true" \
  "start_image_view:=false" \
  "start_perception:=true" \
  "start_robot_stack:=${START_ROBOT_STACK}" \
  "run_task:=false" \
  "model_license_approved:=${MODEL_LICENSE_APPROVED}"

wait_for_real_rgbd_frames 40
if [[ "${REAL_RGBD_READY:-0}" == 1 ]]; then
  wait_for_log_pattern "YOLO RGB-D stats: frames=" 45
else
  echo "[fruit-arm] skipping perception readiness wait until the D455 is available"
fi
