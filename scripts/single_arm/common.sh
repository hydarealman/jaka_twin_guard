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
  echo "[fruit-arm] ERROR: $*" >&2
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
  echo "[fruit-arm] ROS 2 Humble and workspace environment sourced"
}

ensure_apple_detector_model() {
  local model_dir="${PROJECT_ROOT}/artifacts/models"
  local calibrated_path="${PROJECT_ROOT}/src/fruit_picking_arm/models/d455_apple_detector_v2.pt"
  local calibrated_sha="e1917b61f008e996855d89bea1138fe7420eeb244962d1aa07cbb806028d096e"
  local model_path="${model_dir}/s24_apple_detector_best.pt"
  local expected_sha="66309c65f5b44bd5ec70efcc349f295bc74aa09ede0e1dfb20440cf34ebfe642"

  if [[ -f "${calibrated_path}" ]] &&
     echo "${calibrated_sha}  ${calibrated_path}" | sha256sum --check --status; then
    export FRUIT_PICKING_DETECTOR_MODEL="${calibrated_path}"
    echo "[fruit-arm] using the D455 motion/light/edge apple detector v2"
    return 0
  fi
  if [[ -f "${calibrated_path}" ]]; then
    echo "[fruit-arm] WARNING: ignoring D455 apple detector with unexpected SHA-256" >&2
  fi

  if [[ ! -f "${model_path}" ]] || \
     ! echo "${expected_sha}  ${model_path}" | sha256sum --check --status; then
    mkdir -p "${model_dir}"
    echo "[fruit-arm] downloading the pinned apple-detector evaluation weight"
    curl -L --fail --retry 3 \
      "https://huggingface.co/Shadyemad/s24-apple-detector/resolve/main/best.pt" \
      -o "${model_path}.part"
    echo "${expected_sha}  ${model_path}.part" | sha256sum --check --status || \
      die "downloaded apple detector failed SHA-256 verification"
    mv "${model_path}.part" "${model_path}"
  fi
  export FRUIT_PICKING_DETECTOR_MODEL="${model_path}"
}

ensure_d455_wsl_attached() {
  # A D455 plugged into Windows is not automatically visible to the WSL ROS
  # process. Attach only the known VID/PID, never reset or detach a device.
  if command -v lsusb >/dev/null 2>&1 && \
     lsusb 2>/dev/null | grep -qi "8086:0b5c"; then
    echo "[fruit-arm] D455 is visible inside WSL"
    return 0
  fi

  local d455_busid=""
  local wsl_distribution="${WSL_DISTRO_NAME:-Ubuntu-22.04}"
  if command -v usbipd.exe >/dev/null 2>&1; then
    d455_busid="$(usbipd.exe list 2>/dev/null | grep -i "8086:0b5c" | awk '{print $1}' | head -n 1 | tr -d '\r')"
  fi
  if [[ -z "${d455_busid}" ]]; then
    echo "[fruit-arm] WARNING: Windows usbipd did not report VID:PID 8086:0b5c"
    echo "[fruit-arm] Administrator PowerShell: usbipd list"
    return 0
  fi

  ensure_runtime_dir
  local attach_log="${LOG_DIR}/usbipd_attach.log"
  echo "[fruit-arm] D455 ${d455_busid} is not visible in WSL; requesting auto-attach"
  nohup usbipd.exe attach --wsl "${wsl_distribution}" --busid "${d455_busid}" \
    --auto-attach >"${attach_log}" 2>&1 < /dev/null &
  for _ in $(seq 1 15); do
    if command -v lsusb >/dev/null 2>&1 && \
       lsusb 2>/dev/null | grep -qi "8086:0b5c"; then
      echo "[fruit-arm] D455 auto-attached to ${wsl_distribution} (bus ${d455_busid})"
      return 0
    fi
    sleep 1
  done
  echo "[fruit-arm] WARNING: D455 is still not visible inside WSL"
  echo "[fruit-arm] usbipd output: ${attach_log}"
  echo "[fruit-arm] If permission was denied, run in Administrator PowerShell:"
  echo "[fruit-arm]   usbipd.exe attach --wsl ${wsl_distribution} --busid ${d455_busid} --auto-attach"
}

report_real_device_visibility() {
  ensure_d455_wsl_attached
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

  local log_file="${LOG_DIR}/${mode}.log"
  : > "${log_file}"

  echo "${mode}" > "${ACTIVE_MODE_FILE}"
  echo "[fruit-arm] starting ${mode}"
  echo "[fruit-arm] log: ${log_file}"

  # A separate session lets the stop script signal the complete ROS launch
  # process group, including Gazebo and nodes spawned by ros2 launch.
  nohup setsid ros2 launch fruit_picking_arm "${launch_file}" "$@" \
    > "${log_file}" 2>&1 < /dev/null &
  local launch_pid=$!
  echo "${launch_pid}" > "${ACTIVE_PID_FILE}"
  rm -f "${ACTIVE_LOCK}/starter.pid"

  sleep 2
  if ! process_or_group_is_alive "${launch_pid}"; then
    echo "[fruit-arm] launch exited during startup; recent log:" >&2
    tail -n 80 "${log_file}" >&2 || true
    remove_runtime_state
    return 1
  fi

  echo "[fruit-arm] ${mode} started, launcher pid=${launch_pid}"
  echo "[fruit-arm] stop with the matching stop script"
}

wait_for_real_rgbd_frames() {
  local timeout_s="${1:-40}"

  # Callers can use this status to distinguish "the readiness probe
  # completed" from "real RGB-D frames were received".  Keep the function's
  # existing zero return status so diagnostic launchers remain alive while a
  # temporarily detached usbipd device is retried in the background.
  REAL_RGBD_READY=0

  ensure_d455_wsl_attached

  if command -v lsusb >/dev/null 2>&1 && \
     ! lsusb 2>/dev/null | grep -qi "8086:0b5c"; then
    echo "[fruit-arm] camera unavailable; nodes remain alive and will retry in the background"
    return 0
  fi

  echo "[fruit-arm] waiting for fresh REAL D455 RGB and depth frames (up to ${timeout_s}s)"
  # WSL2 usbipd can briefly stream and pause while the rest of the launch is
  # initialising. Delay the probe so an early transient frame is not reported
  # as a healthy RGB-D stream.
  sleep 5

  local rgb_wait_pid depth_wait_pid rgb_ready=0 depth_ready=0
  timeout "${timeout_s}" ros2 topic echo --once \
    --qos-reliability best_effort \
    /camera/camera/color/image_raw >/dev/null 2>&1 &
  rgb_wait_pid=$!
  timeout "${timeout_s}" ros2 topic echo --once \
    --qos-reliability best_effort \
    /camera/camera/aligned_depth_to_color/image_raw >/dev/null 2>&1 &
  depth_wait_pid=$!

  if wait "${rgb_wait_pid}"; then rgb_ready=1; fi
  if wait "${depth_wait_pid}"; then depth_ready=1; fi

  if [[ "${rgb_ready}" == 1 && "${depth_ready}" == 1 ]]; then
    REAL_RGBD_READY=1
    echo "[fruit-arm] D455 RGB-D ready: fresh real RGB and depth frames received"
  else
    echo "[fruit-arm] WARNING: D455 is USB-visible but fresh RGB-D frames were not both received within ${timeout_s}s"
    echo "[fruit-arm] camera/debug nodes remain alive; no simulation fallback is used"
    echo "[fruit-arm] inspect: ${LOG_DIR}/$(active_mode).log"
  fi
}

wait_for_log_pattern() {
  local pattern="$1"
  local timeout_s="${2:-30}"
  local log_file="${LOG_DIR}/$(active_mode).log"
  local launch_pid="$(active_pid)"
  local deadline=$((SECONDS + timeout_s))

  echo "[fruit-arm] waiting for perception inference (up to ${timeout_s}s)"
  while ((SECONDS < deadline)); do
    if ! process_or_group_is_alive "${launch_pid}"; then
      echo "[fruit-arm] ERROR: launch exited before perception became ready" >&2
      return 1
    fi
    if [[ -f "${log_file}" ]] && grep -Fq "${pattern}" "${log_file}"; then
      echo "[fruit-arm] perception ready: real inference result received"
      return 0
    fi
    sleep 1
  done
  echo "[fruit-arm] ERROR: perception produced no inference result within ${timeout_s}s" >&2
  echo "[fruit-arm] inspect: ${log_file}" >&2
  return 1
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
  # The group signal is the escalation path. The direct PID fallback covers
  # systems where setsid was unable to create a process group.
  kill "-${signal}" -- "-${pid}" 2>/dev/null || \
    kill "-${signal}" "${pid}" 2>/dev/null || true
}

send_launcher_signal() {
  local signal="$1"
  local pid="$2"
  # Let ros2 launch receive the first interrupt by itself. It then marks the
  # launch service as shutting down before stopping child processes, which
  # prevents respawn=true nodes from being relaunched during normal shutdown.
  # The process-group path remains the escalation fallback below.
  kill "-${signal}" "${pid}" 2>/dev/null || true
}

stop_launch() {
  local expected_mode="$1"
  ensure_runtime_dir

  local current_mode="$(active_mode)"
  local launch_pid="$(active_pid)"

  if [[ -z "${current_mode}" || -z "${launch_pid}" ]]; then
    echo "[fruit-arm] no recorded single-arm launch is running"
    remove_runtime_state
    return 0
  fi

  mode_matches "${expected_mode}" "${current_mode}" || \
    die "active mode is ${current_mode}, not ${expected_mode}; use the matching stop script"

  echo "[fruit-arm] stopping ${current_mode}, launcher pid=${launch_pid}"
  send_launcher_signal INT "${launch_pid}"

  for _ in $(seq 1 30); do
    process_or_group_is_alive "${launch_pid}" || break
    sleep 0.5
  done

  if process_or_group_is_alive "${launch_pid}"; then
    echo "[fruit-arm] graceful stop timed out; sending TERM"
    send_group_signal TERM "${launch_pid}"
    for _ in $(seq 1 10); do
      process_or_group_is_alive "${launch_pid}" || break
      sleep 0.5
    done
  fi

  if process_or_group_is_alive "${launch_pid}"; then
    echo "[fruit-arm] forced stop: sending KILL" >&2
    send_group_signal KILL "${launch_pid}"
  fi

  remove_runtime_state
  echo "[fruit-arm] ${current_mode} stopped"
}
