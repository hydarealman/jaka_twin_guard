#!/usr/bin/env bash
set -eo pipefail

ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
LOG_FILE="/tmp/grasp_orientation_launch.$$.log"
RESULT_FILE="/tmp/grasp_orientation_results.$$.log"
LAUNCH_PID=""

source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 launch jaka_single_arm architecture_a_sim.launch.py \
  gui:=false start_rviz:=false start_image_view:=false run_task:=false \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 75); do
  if grep -q "You can start planning now" "${LOG_FILE}"; then
    python3 "${ROOT}/src/jaka_single_arm/test/probe_grasp_orientations.py" \
      2>&1 | tee "${RESULT_FILE}"
    echo "RESULT_FILE=${RESULT_FILE}"
    exit "${PIPESTATUS[0]}"
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    tail -n 160 "${LOG_FILE}" >&2
    exit 2
  fi
  sleep 1
done

echo "MoveIt did not become ready" >&2
tail -n 160 "${LOG_FILE}" >&2
exit 2
