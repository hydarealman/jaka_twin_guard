#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment

echo "[fruit-arm] starting URDF model joint checker"
echo "[fruit-arm] no camera, serial, MoveIt executor or motor control will start"
echo "[fruit-arm] drag joint_1..joint_6 in the joint_state_publisher_gui window"
echo "[fruit-arm] four-finger model: left_finger_joint 0=closed, 0.056=open"
echo "[fruit-arm] right_finger_joint is the retained symmetric serial-command companion"
echo "[fruit-arm] exact radians: ros2 topic echo /joint_states"
echo "[fruit-arm] press Ctrl+C here to stop the model checker"

exec ros2 launch fruit_picking_arm model_joint_check.launch.py
