#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$(pwd)}"
TEST_DIR="$ROOT/src/jaka_single_arm/test"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
RUN_ID="$$"
HOST_PORT="/tmp/jaka_host_${RUN_ID}"
BOARD_PORT="/tmp/jaka_board_${RUN_ID}"

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    # Every background command owns a session. Kill the whole process group so
    # the Python executable launched by `ros2 run` cannot survive the wrapper.
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  sleep 0.2
  for pid in "${PIDS[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -f "$HOST_PORT" "$BOARD_PORT"
  PIDS=()
}
trap cleanup EXIT

start_pair() {
  cleanup
  setsid socat pty,raw,echo=0,link="$HOST_PORT" \
    pty,raw,echo=0,link="$BOARD_PORT" >/tmp/jaka_socat.log 2>&1 &
  PIDS+=("$!")
  for _ in $(seq 1 50); do
    [[ -e "$HOST_PORT" && -e "$BOARD_PORT" ]] && return
    sleep 0.05
  done
  echo "virtual serial ports were not created" >&2
  exit 1
}

echo "[A] Starting virtual board and trajectory controller"
start_pair
setsid ros2 run jaka_single_arm serial_board_emulator --port "$BOARD_PORT" \
  --baudrate 115200 --execution-delay 0.05 >/tmp/jaka_board_a.log 2>&1 &
PIDS+=("$!")
setsid ros2 run jaka_single_arm serial_trajectory_controller --ros-args \
  -p serial_port:="$HOST_PORT" -p baudrate:=115200 -p require_ready:=true \
  >/tmp/jaka_node_a.log 2>&1 &
PIDS+=("$!")
python3 "$TEST_DIR/ros_architecture_a_e2e.py"

echo "[B] Starting virtual board and direct-target bridge"
start_pair
setsid ros2 run jaka_single_arm serial_board_emulator --port "$BOARD_PORT" \
  --baudrate 115200 --execution-delay 0.05 >/tmp/jaka_board_b.log 2>&1 &
PIDS+=("$!")
setsid ros2 run jaka_single_arm serial_fruit_target_bridge --ros-args \
  -p serial_port:="$HOST_PORT" -p baudrate:=115200 -p require_ready:=true \
  >/tmp/jaka_node_b.log 2>&1 &
PIDS+=("$!")
python3 "$TEST_DIR/ros_architecture_b_e2e.py"

echo "SERIAL_ROS_E2E_ALL_PASS"
