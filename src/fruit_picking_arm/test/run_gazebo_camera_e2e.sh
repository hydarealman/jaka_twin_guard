#!/usr/bin/env bash
set -eo pipefail

ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
LOG_FILE="/tmp/jaka_gazebo_camera_e2e.$$.log"
LAUNCH_PID=""
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + ($$ % 100)))}"

source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 16); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

fail() {
  echo "GAZEBO_CAMERA_E2E_FAIL: $*" >&2
  tail -n 160 "${LOG_FILE}" >&2 || true
  exit 1
}

ros2 launch fruit_picking_arm sim_gazebo.launch.py \
  gui:=false \
  start_rviz:=false \
  start_image_view:=false \
  start_detector:=false \
  start_moveit:=false \
  run_task:=false >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

required_topics=(
  /camera/camera/color/image_raw
  /camera/camera/color/camera_info
  /camera/camera/depth/image_rect_raw
  /camera/camera/depth/color/points
  /joint_states
)

ready=false
for _ in $(seq 1 120); do
  kill -0 "${LAUNCH_PID}" 2>/dev/null || fail "launch process exited early"
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  missing=false
  for topic in "${required_topics[@]}"; do
    if ! grep -qx "${topic}" <<<"${topic_list}"; then
      missing=true
      break
    fi
  done
  if [[ "${missing}" == false ]]; then
    ready=true
    break
  fi
  sleep 1
done
[[ "${ready}" == true ]] || fail "camera/controller topics did not appear in 120 s"

camera_width=""
for _ in $(seq 1 5); do
  camera_width="$(timeout 12s ros2 topic echo --once \
    /camera/camera/color/camera_info --field width 2>/dev/null || true)"
  grep -q '424' <<<"${camera_width}" && break
done
grep -q '424' <<<"${camera_width}" || fail "camera_info width is not 424"

point_width=""
for _ in $(seq 1 5); do
  point_width="$(timeout 12s ros2 topic echo --once \
    /camera/camera/depth/color/points --field width 2>/dev/null || true)"
  grep -q '424' <<<"${point_width}" && break
done
grep -q '424' <<<"${point_width}" || fail "point cloud width is not 424"

controllers="$(ros2 control list_controllers -c /controller_manager 2>/dev/null || true)"
grep -q 'arm_controller.*active' <<<"${controllers}" || fail "arm_controller is not active"
grep -q 'joint_state_broadcaster.*active' <<<"${controllers}" || \
  fail "joint_state_broadcaster is not active"

tf_output="$(timeout 8s ros2 run tf2_ros tf2_echo \
  world camera_color_optical_frame 2>&1 || true)"
grep -q 'Translation: \[1.250, -0.800, 1.050\]' <<<"${tf_output}" || \
  fail "eye-to-hand camera TF is unavailable or incorrect"

echo "GAZEBO_CAMERA_E2E_PASS: RGB, depth, point cloud, TF and controllers verified"
