#!/usr/bin/env bash
set -eo pipefail

# Physical production entry point: RViz is monitor-only and the OpenCV
# overlay remains visible. Motion starts only through the explicit service.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
report_real_device_visibility
configure_validated_field_perception

PORT="$(find_control_serial)" || die "control-board serial is unavailable"
echo "[fruit-arm] detected serial device: ${PORT}"
ROBOT_SERIAL="${FRUIT_ARM_ROBOT_SERIAL:-fruit-arm-primary}"
KINEMATICS_MODE="${FRUIT_ARM_KINEMATICS_MODE:-nominal}"
CALIBRATION_FILE="${FRUIT_ARM_CALIBRATION_FILE:-}"

echo "[fruit-arm] starting Architecture A monitored automatic execution"
echo "[fruit-arm] field detector: ${FIELD_DETECTOR_MODEL}"
echo "[fruit-arm] detector gates: acquire >=${DETECTOR_CONFIDENCE}; stable >=${STABLE_DETECTION_CONFIDENCE} for ${STABLE_MIN_FRAMES} frames"
echo "[fruit-arm] RViz is display-only; no Plan/Execute click is required"
echo "[fruit-arm] each cycle locks one fruit, completes it, then senses again"
echo "[fruit-arm] light vision reset runs automatically after each completed HOME return"
echo "[fruit-arm] motion scaling: velocity 30%; acceleration 24%"
echo "[fruit-arm] verify E-stop, zero positions, limits, READY state and hand-eye calibration"

LAUNCH_ARGS=( \
  "serial_port:=${PORT}" \
  "baudrate:=115200" \
  "robot_serial:=${ROBOT_SERIAL}" \
  "kinematics_mode:=${KINEMATICS_MODE}" \
  "color_profile:=${D455_COLOR_PROFILE:-424,240,15}" \
  "depth_profile:=${D455_DEPTH_PROFILE:-424,240,15}" \
  "enable_temporal_filter:=false" \
  "start_rviz:=true" \
  "rviz_config:=fruit_picking_arm.rviz" \
  "start_debug_view:=true" \
  "start_image_view:=false" \
  "start_perception:=true" \
  "enable_table_perception:=true" \
  "perception_output_frame:=world" \
  "perception_output_topic:=/perception/fruit_targets_world" \
  "fruit_detector_confidence:=${DETECTOR_CONFIDENCE}" \
  "stable_min_detection_confidence:=${STABLE_DETECTION_CONFIDENCE}" \
  "stable_min_frames:=${STABLE_MIN_FRAMES}" \
  "detection_roi_min_z:=${FRUIT_ARM_DETECTION_ROI_MIN_Z:--0.10}" \
  "detection_roi_max_z:=${FRUIT_ARM_DETECTION_ROI_MAX_Z:-1.20}" \
  "start_fruit_goal_bridge:=false" \
  "start_robot_stack:=true" \
  "run_task:=true" \
  "require_auto_start_signal:=true" \
  "continuous_auto_task:=true" \
  "model_license_approved:=true"
)
if [[ -n "${CALIBRATION_FILE}" ]]; then
  LAUNCH_ARGS+=("calibration_file:=${CALIBRATION_FILE}")
fi
start_launch "a_real_auto" "architecture_a_real.launch.py" "${LAUNCH_ARGS[@]}"

require_real_rgbd_frames "a_real" 40
require_real_robot_state "a_real" 20
require_real_robot_control_ready "a_real" 20
wait_for_log_pattern "YOLO RGB-D stats: frames=" 45 || {
  stop_launch "a_real" || true
  die "perception inference did not become ready"
}
request_light_vision_reset 20 || {
  stop_launch "a_real" || true
  die "initial light vision reset was not accepted"
}
request_auto_task_start 35 || {
  stop_launch "a_real" || true
  die "automatic-task start was not accepted"
}

echo "[fruit-arm] automatic mode is running and will wait for each next fruit"
echo "[fruit-arm] stop with: bash scripts/single_arm/stop_architecture_a_real.sh"
