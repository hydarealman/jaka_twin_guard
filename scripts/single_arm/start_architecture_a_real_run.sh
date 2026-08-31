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
ROBOT_SERIAL="${FRUIT_ARM_ROBOT_SERIAL:-fruit-arm-primary}"
KINEMATICS_MODE="${FRUIT_ARM_KINEMATICS_MODE:-nominal}"
CALIBRATION_FILE="${FRUIT_ARM_CALIBRATION_FILE:-}"

echo "[fruit-arm] starting Architecture A physical execution without debug windows"
echo "[fruit-arm] verify E-stop, zero positions, limits, READY state and hand-eye calibration"

LAUNCH_ARGS=( \
  "serial_port:=${PORT}" \
  "baudrate:=115200" \
  "robot_serial:=${ROBOT_SERIAL}" \
  "kinematics_mode:=${KINEMATICS_MODE}" \
  "color_profile:=${D455_COLOR_PROFILE:-424,240,15}" \
  "depth_profile:=${D455_DEPTH_PROFILE:-424,240,15}" \
  "start_rviz:=false" \
  "start_debug_view:=false" \
  "start_image_view:=false" \
  "start_perception:=true" \
  "enable_table_perception:=true" \
  "perception_output_frame:=world" \
  "perception_output_topic:=/perception/fruit_targets_world" \
  "detection_roi_min_z:=${FRUIT_ARM_DETECTION_ROI_MIN_Z:--0.10}" \
  "detection_roi_max_z:=${FRUIT_ARM_DETECTION_ROI_MAX_Z:-1.20}" \
  "start_fruit_goal_bridge:=false" \
  "start_robot_stack:=true" \
  "run_task:=true" \
  "model_license_approved:=true"
)
if [[ -n "${CALIBRATION_FILE}" ]]; then
  LAUNCH_ARGS+=("calibration_file:=${CALIBRATION_FILE}")
fi
start_launch "a_real" "architecture_a_real.launch.py" "${LAUNCH_ARGS[@]}"

require_real_rgbd_frames "a_real" 40
require_real_robot_state "a_real" 20
