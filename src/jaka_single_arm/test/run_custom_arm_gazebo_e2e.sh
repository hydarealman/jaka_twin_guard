#!/usr/bin/env bash
set -eo pipefail

ROOT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
LOG_FILE="/tmp/custom_arm_gazebo_e2e.$$.log"
LAUNCH_PID=""

source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
# A Windows/WSL reboot can leave the ros2cli daemon socket in a stale state.
# This smoke test uses daemon-free graph discovery and starts from a clean CLI
# daemon so it never mistakes cached controllers/topics for the current run.
ros2 daemon stop >/dev/null 2>&1 || true
set -u

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 "${LAUNCH_PID}" 2>/dev/null || break
      sleep 0.5
    done
    kill -TERM "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

fail() {
  echo "CUSTOM_ARM_GAZEBO_E2E_FAIL: $*" >&2
  tail -n 180 "${LOG_FILE}" >&2 || true
  exit 1
}

ros2 launch jaka_single_arm sim_gazebo.launch.py \
  gui:=false start_rviz:=false start_image_view:=false \
  start_detector:=false start_moveit:=true run_task:=false \
  >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

joint_topic_ready=false
for _ in $(seq 1 90); do
  kill -0 "${LAUNCH_PID}" 2>/dev/null || fail "launch process exited early"
  topics="$(timeout 4s ros2 topic list --no-daemon 2>/dev/null || true)"
  if grep -qx '/joint_states' <<<"${topics}"; then
    joint_topic_ready=true
    break
  fi
  sleep 1
done
[[ "${joint_topic_ready}" == true ]] || fail "/joint_states did not appear"

joint_state="$(timeout 15s ros2 topic echo /joint_states --once --no-daemon 2>/dev/null || true)"
for joint in joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 left_finger_joint right_finger_joint; do
  grep -q "${joint}" <<<"${joint_state}" || fail "${joint} missing from /joint_states"
done

goal_output="$(timeout 30s ros2 action send_goal --feedback --timeout 20 \
  /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, left_finger_joint, right_finger_joint], points: [{positions: [0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04, -0.04], time_from_start: {sec: 3}}]}}" \
  2>&1 || true)"
grep -q 'Goal accepted' <<<"${goal_output}" || fail "trajectory goal was not accepted: ${goal_output}"
grep -q 'error_code=0' <<<"${goal_output}" || \
  grep -q 'error_code: 0' <<<"${goal_output}" || \
  fail "trajectory did not finish successfully: ${goal_output}"

ik_service_ready=false
for _ in $(seq 1 45); do
  services="$(timeout 4s ros2 service list --no-daemon 2>/dev/null || true)"
  if grep -qx '/compute_ik' <<<"${services}"; then
    ik_service_ready=true
    break
  fi
  sleep 1
done
[[ "${ik_service_ready}" == true ]] || fail "MoveIt /compute_ik service did not appear"

# The query is the CAD zero-pose tool flange transformed into the world frame.
# A successful solution proves the inferred chain is accepted by MoveIt/KDL.
ik_output="$(timeout 20s ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK \
  "{ik_request: {group_name: arm, ik_link_name: tool_flange, pose_stamped: {header: {frame_id: world}, pose: {position: {x: -0.147349, y: 0.028700, z: 0.319809}, orientation: {x: 0.0, y: 0.70710678, z: 0.0, w: 0.70710678}}}, timeout: {sec: 2}, avoid_collisions: false}}" \
  2>&1 || true)"
grep -q 'val=1' <<<"${ik_output}" || grep -q 'val: 1' <<<"${ik_output}" || \
  fail "MoveIt IK failed for the CAD zero pose: ${ik_output}"

echo "CUSTOM_ARM_GAZEBO_E2E_PASS: CAD robot spawned, 8 joints published, controller motion and MoveIt IK succeeded"
