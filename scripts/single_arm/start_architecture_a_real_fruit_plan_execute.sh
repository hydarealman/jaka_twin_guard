#!/usr/bin/env bash
set -eo pipefail

# Staged real-fruit RViz Plan/Execute.  Every arm segment is exposed only after
# the preceding operator-approved Execute succeeds.  The helper never sends an
# arm trajectory itself.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
report_real_device_visibility
configure_validated_field_perception

PORT="$(find_control_serial)" || die "control-board serial is unavailable"
ROBOT_SERIAL="${FRUIT_ARM_ROBOT_SERIAL:-fruit-arm-primary}"
KINEMATICS_MODE="${FRUIT_ARM_KINEMATICS_MODE:-nominal}"
CALIBRATION_FILE="${FRUIT_ARM_CALIBRATION_FILE:-}"
echo "[fruit-arm] starting Architecture A fruit-assisted RViz Plan/Execute mode"
echo "[fruit-arm] robot feedback: real /joint_states from ${PORT}"
echo "[fruit-arm] automatic pick task: DISABLED"
echo "[fruit-arm] staged task: pregrasp -> open -> grasp -> close -> lift -> bin hover -> release -> retract -> home"
echo "[fruit-arm] field detector: ${FIELD_DETECTOR_MODEL}"
echo "[fruit-arm] detector gates: acquire >=${DETECTOR_CONFIDENCE}; stable >=${STABLE_DETECTION_CONFIDENCE} for ${STABLE_MIN_FRAMES} frames"
echo "[fruit-arm] multi-fruit mode: keep up to 5 apples; sort one complete RViz-gated task at a time"
echo "[fruit-arm] RViz planning group: arm; velocity scaling: 25%; acceleration scaling: 20%"
echo "[fruit-arm] MoveIt execution timeout: planned duration x1.2 + 15s serial/ACK margin"
echo "[fruit-arm] Plan only computes/displays; Execute sends the inspected trajectory"
echo "[fruit-arm] after pregrasp/grasp/release Execute succeeds, the binary gripper opens/closes/opens automatically"
echo "[fruit-arm] after the final HOME Execute, light vision reset runs automatically and requires a new 5-frame target"
echo "[fruit-arm] DO NOT use Plan & Execute during initial validation"

LAUNCH_ARGS=(
  "serial_port:=${PORT}"
  "baudrate:=115200"
  "robot_serial:=${ROBOT_SERIAL}"
  "kinematics_mode:=${KINEMATICS_MODE}"
  "color_profile:=${D455_COLOR_PROFILE:-424,240,15}"
  "depth_profile:=${D455_DEPTH_PROFILE:-424,240,15}"
  "enable_temporal_filter:=false"
  "start_rviz:=true"
  "rviz_config:=fruit_picking_arm_plan_execute.rviz"
  "start_debug_view:=true"
  "start_image_view:=false"
  "start_perception:=true"
  "start_fruit_goal_bridge:=true"
  "enable_table_perception:=true"
  "perception_output_frame:=world"
  "perception_output_topic:=/perception/fruit_targets_world"
  # This mode remains human-gated by RViz Plan/Execute and retains
  # depth/radius/ROI, five-frame stability, collision and IK checks.
  "fruit_detector_confidence:=${DETECTOR_CONFIDENCE}"
  "stable_min_detection_confidence:=${STABLE_DETECTION_CONFIDENCE}"
  "stable_min_frames:=${STABLE_MIN_FRAMES}"
  # base_link is the base-bottom centre; allow a small below-plane margin.
  "detection_roi_min_z:=${FRUIT_ARM_DETECTION_ROI_MIN_Z:--0.10}"
  "detection_roi_max_z:=${FRUIT_ARM_DETECTION_ROI_MAX_Z:-1.20}"
  "start_robot_stack:=true"
  "run_task:=false"
  "model_license_approved:=true"
)
if [[ -n "${CALIBRATION_FILE}" ]]; then
  LAUNCH_ARGS+=("calibration_file:=${CALIBRATION_FILE}")
fi

start_launch "a_real" "architecture_a_real.launch.py" "${LAUNCH_ARGS[@]}"
require_real_rgbd_frames "a_real" 40
require_real_robot_state "a_real" 20
wait_for_log_pattern "YOLO RGB-D stats: frames=" 45
request_light_vision_reset 20 || {
  stop_launch "a_real" || true
  die "initial light vision reset was not accepted"
}

echo "[fruit-arm] waiting for a stable classified fruit and the first RViz stage"
echo "[fruit-arm] place one or several separated fruits in view; wait for FRUIT_RVIZ_GOAL_READY"
echo "[fruit-arm] for EACH NEXT stage: click Plan, inspect robot/fruit/target/table/path, then Execute"
echo "[fruit-arm] manual vision reset when idle: bash scripts/single_arm/reset_vision.sh"
echo "[fruit-arm] stop with: bash scripts/single_arm/stop_architecture_a_real.sh"
