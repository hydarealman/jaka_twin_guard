#!/usr/bin/env bash
set -eo pipefail

# Manual real-robot MoveIt bring-up. This mode never starts pick_place_runner:
# the operator plans in RViz first and explicitly chooses whether to execute.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment
report_real_device_visibility
ensure_apple_detector_model

PORT="$(find_control_serial)" || die "control-board serial is unavailable"
ROBOT_SERIAL="${FRUIT_ARM_ROBOT_SERIAL:-fruit-arm-primary}"
KINEMATICS_MODE="${FRUIT_ARM_KINEMATICS_MODE:-nominal}"
CALIBRATION_FILE="${FRUIT_ARM_CALIBRATION_FILE:-}"

echo "[fruit-arm] starting Architecture A manual RViz Plan/Execute mode"
echo "[fruit-arm] robot feedback: real /joint_states from ${PORT}"
echo "[fruit-arm] automatic pick task: DISABLED"
echo "[fruit-arm] RViz planning group: arm; velocity scaling: 25%; acceleration scaling: 20%"
echo "[fruit-arm] MoveIt execution timeout: planned duration x1.2 + 15s serial/ACK margin"
echo "[fruit-arm] use Plan first; inspect the complete path before Execute"
echo "[fruit-arm] WARNING: Execute sends a real trajectory to the C board"

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

echo "[fruit-arm] RViz is ready for manual Plan, then separate Execute"
echo "[fruit-arm] stop with: bash scripts/single_arm/stop_architecture_a_real.sh"
