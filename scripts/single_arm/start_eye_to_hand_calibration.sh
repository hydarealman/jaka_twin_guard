#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

source_ros_environment
report_real_device_visibility

PORT="$(find_control_serial || true)"
if [[ -z "${PORT}" ]]; then
  die "control-board serial device was not found"
fi

OUTPUT_YAML="${PROJECT_ROOT}/src/fruit_picking_arm/config/hand_eye_params.yaml"
DASHBOARD_URL="http://localhost:8765"
CALIBRATION_COLOR_PROFILE="${D455_CALIBRATION_COLOR_PROFILE:-1280,800,5}"

echo "[fruit-arm] starting D455 eye-to-hand calibration"
echo "[fruit-arm] board parameters: ${PROJECT_ROOT}/src/fruit_picking_arm/config/eye_to_hand_calibration.yaml"
echo "[fruit-arm] final transform: ${OUTPUT_YAML}"
echo "[fruit-arm] D455 calibration color profile: ${CALIBRATION_COLOR_PROFILE}"

start_launch "eye_calibration" "eye_to_hand_calibration.launch.py" \
  "serial_port:=${PORT}" \
  "baudrate:=115200" \
  "start_camera:=true" \
  "color_profile:=${CALIBRATION_COLOR_PROFILE}" \
  "start_moveit:=true" \
  "start_rviz:=true" \
  "start_dashboard:=true" \
  "dashboard_port:=8765" \
  "output_yaml:=${OUTPUT_YAML}"

echo "[fruit-arm] waiting for calibration dashboard"
for _ in $(seq 1 40); do
  if curl --noproxy "*" --silent --fail "${DASHBOARD_URL}/api/info" >/dev/null 2>&1; then
    echo "[fruit-arm] dashboard ready: ${DASHBOARD_URL}"
    if command -v powershell.exe >/dev/null 2>&1; then
      powershell.exe -NoProfile -Command "Start-Process '${DASHBOARD_URL}'" >/dev/null 2>&1 || true
    fi
    exit 0
  fi
  sleep 0.5
done

echo "[fruit-arm] dashboard did not open automatically; inspect ${RUNTIME_DIR}/logs/eye_calibration.log" >&2
echo "[fruit-arm] you can still open ${DASHBOARD_URL} manually" >&2
exit 1
