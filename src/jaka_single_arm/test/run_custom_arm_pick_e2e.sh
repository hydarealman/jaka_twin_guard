#!/usr/bin/env bash
set -eo pipefail

ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
LOG_FILE="/tmp/custom_arm_pick_e2e.$$.log"
VERIFY_LOG="/tmp/custom_arm_pick_verify.$$.log"
LAUNCH_PID=""

source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]]; then
    # The launch leader can exit before one of its Gazebo children.  Signal
    # the process group even when the leader itself is already gone.
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

fail() {
  echo "CUSTOM_ARM_PICK_E2E_FAIL: $*" >&2
  tail -n 220 "${LOG_FILE}" >&2 || true
  exit 1
}

setsid ros2 launch jaka_single_arm architecture_a_sim.launch.py \
  gui:=false start_rviz:=false start_image_view:=false run_task:=true \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

for _ in $(seq 1 300); do
  kill -0 "${LAUNCH_PID}" 2>/dev/null || fail "launch process exited early"
  if grep -q 'Task Complete:' "${LOG_FILE}"; then
    if grep -q 'Task Complete: 4/4 objects placed' "${LOG_FILE}"; then
      attach_count="$(grep -c 'PHYSICAL_GRASP_ATTACHED' "${LOG_FILE}" || true)"
      detach_count="$(grep -c 'PHYSICAL_GRASP_DETACHED' "${LOG_FILE}" || true)"
      if [[ "${attach_count}" -ne 4 || "${detach_count}" -ne 4 ]]; then
        fail "expected 4 physical attach/detach cycles, got ${attach_count}/${detach_count}"
      fi
      if python3 "${ROOT}/src/jaka_single_arm/test/verify_physical_fruit_bins.py" \
          2>&1 | tee "${VERIFY_LOG}"; then
        pass_message="CUSTOM_ARM_PICK_E2E_PASS: all 4 fruit physically grasped, carried, released and sorted"
        echo "${pass_message}" | tee -a "${VERIFY_LOG}"
        exit 0
      fi
      fail "fruit did not physically settle in the correct bins"
    fi
    fail "task completed without sorting all 4 fruit"
  fi
  if grep -q 'Setup failed:' "${LOG_FILE}"; then
    fail "perception/setup failed"
  fi
  if grep -q 'Safety HALT' "${LOG_FILE}"; then
    fail "safety monitor halted the task"
  fi
  sleep 1
done

fail "task did not complete within 300 seconds"
