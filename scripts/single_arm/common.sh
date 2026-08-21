#!/usr/bin/env bash
set -eo pipefail

# Shared process management for the single-arm launch wrappers.
# These scripts are intended to run inside WSL2/Ubuntu with ROS 2 sourced.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
RUNTIME_DIR="${PROJECT_ROOT}/.runtime/single_arm"
LOG_DIR="${RUNTIME_DIR}/logs"
ACTIVE_LOCK="${RUNTIME_DIR}/active.lock"
ACTIVE_PID_FILE="${RUNTIME_DIR}/active.pid"
ACTIVE_MODE_FILE="${RUNTIME_DIR}/active.mode"

die() {
  echo "[single-arm] ERROR: $*" >&2
  exit 1
}

ensure_runtime_dir() {
  mkdir -p "${LOG_DIR}"
}

source_ros_environment() {
  [[ -f /opt/ros/humble/setup.bash ]] || \
    die "ROS 2 Humble not found at /opt/ros/humble/setup.bash"
  [[ -f "${PROJECT_ROOT}/install/setup.bash" ]] || \
    die "workspace is not built: ${PROJECT_ROOT}/install/setup.bash not found"

  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/install/setup.bash"
}

pid_is_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

process_or_group_is_alive() {
  local pid="$1"
  pid_is_alive "${pid}" || kill -0 -- "-${pid}" 2>/dev/null
}

read_file_or_empty() {
  local file="$1"
  if [[ -f "${file}" ]]; then
    tr -d '\r\n' < "${file}"
  fi
}

active_mode() {
  read_file_or_empty "${ACTIVE_MODE_FILE}"
}

active_pid() {
  read_file_or_empty "${ACTIVE_PID_FILE}"
}

remove_runtime_state() {
  rm -f "${ACTIVE_PID_FILE}" "${ACTIVE_MODE_FILE}"
  rm -f "${ACTIVE_LOCK}/starter.pid"
  rmdir "${ACTIVE_LOCK}" 2>/dev/null || true
}

acquire_active_lock() {
  ensure_runtime_dir

  if [[ -d "${ACTIVE_LOCK}" ]]; then
    local old_pid="$(active_pid)"
    local starter_pid="$(read_file_or_empty "${ACTIVE_LOCK}/starter.pid")"

    if process_or_group_is_alive "${old_pid}"; then
      die "another single-arm mode is already running: mode=$(active_mode), pid=${old_pid}"
    fi
    if pid_is_alive "${starter_pid}"; then
      die "another single-arm mode is still starting (launcher pid=${starter_pid})"
    fi

    # The previous wrapper crashed or was terminated before cleanup. The
    # target is our explicit runtime lock directory, not a workspace path.
    rm -rf "${ACTIVE_LOCK}"
    rm -f "${ACTIVE_PID_FILE}" "${ACTIVE_MODE_FILE}"
  fi

  mkdir "${ACTIVE_LOCK}" || die "could not acquire single-arm start lock"
  echo "$$" > "${ACTIVE_LOCK}/starter.pid"
}

start_launch() {
  local mode="$1"
  local launch_file="$2"
  shift 2

  acquire_active_lock
  source_ros_environment

  local log_file="${LOG_DIR}/${mode}.log"
  : > "${log_file}"

  echo "${mode}" > "${ACTIVE_MODE_FILE}"
  echo "[single-arm] starting ${mode}"
  echo "[single-arm] log: ${log_file}"

  # A separate session lets the stop script signal the complete ROS launch
  # process group, including Gazebo and nodes spawned by ros2 launch.
  nohup setsid ros2 launch jaka_single_arm "${launch_file}" "$@" \
    > "${log_file}" 2>&1 < /dev/null &
  local launch_pid=$!
  echo "${launch_pid}" > "${ACTIVE_PID_FILE}"
  rm -f "${ACTIVE_LOCK}/starter.pid"

  sleep 2
  if ! process_or_group_is_alive "${launch_pid}"; then
    echo "[single-arm] launch exited during startup; recent log:" >&2
    tail -n 80 "${log_file}" >&2 || true
    remove_runtime_state
    return 1
  fi

  echo "[single-arm] ${mode} started, launcher pid=${launch_pid}"
  echo "[single-arm] stop with the matching stop script"
}

mode_matches() {
  local expected="$1"
  local current="$2"

  [[ "${expected}" == "any" ]] && return 0
  [[ "${expected}" == "${current}" ]] && return 0
  [[ "${expected}" == "b" && "${current}" == b_* ]] && return 0
  return 1
}

send_group_signal() {
  local signal="$1"
  local pid="$2"
  # The group signal is the normal path. The direct PID fallback covers
  # systems where setsid was unable to create a process group.
  kill "-${signal}" -- "-${pid}" 2>/dev/null || \
    kill "-${signal}" "${pid}" 2>/dev/null || true
}

stop_launch() {
  local expected_mode="$1"
  ensure_runtime_dir

  local current_mode="$(active_mode)"
  local launch_pid="$(active_pid)"

  if [[ -z "${current_mode}" || -z "${launch_pid}" ]]; then
    echo "[single-arm] no recorded single-arm launch is running"
    remove_runtime_state
    return 0
  fi

  mode_matches "${expected_mode}" "${current_mode}" || \
    die "active mode is ${current_mode}, not ${expected_mode}; use the matching stop script"

  echo "[single-arm] stopping ${current_mode}, launcher pid=${launch_pid}"
  send_group_signal INT "${launch_pid}"

  for _ in $(seq 1 30); do
    process_or_group_is_alive "${launch_pid}" || break
    sleep 0.5
  done

  if process_or_group_is_alive "${launch_pid}"; then
    echo "[single-arm] graceful stop timed out; sending TERM"
    send_group_signal TERM "${launch_pid}"
    for _ in $(seq 1 10); do
      process_or_group_is_alive "${launch_pid}" || break
      sleep 0.5
    done
  fi

  if process_or_group_is_alive "${launch_pid}"; then
    echo "[single-arm] forced stop: sending KILL" >&2
    send_group_signal KILL "${launch_pid}"
  fi

  remove_runtime_state
  echo "[single-arm] ${current_mode} stopped"
}
