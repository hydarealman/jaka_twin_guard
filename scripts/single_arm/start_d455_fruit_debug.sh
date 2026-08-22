#!/usr/bin/env bash
set -eo pipefail

# Safe D455 bring-up: camera, RGB-D detector and debug image topics only.
# No MoveIt, robot state publisher, serial bridge, or motion task is started.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

source_ros_environment
ensure_d455_wsl_attached

ensure_apple_detector_model

COLOR_PROFILE="${D455_COLOR_PROFILE:-424,240,15}"
DEPTH_PROFILE="${D455_DEPTH_PROFILE:-424,240,15}"
start_launch "d455_debug" "d455_fruit_debug.launch.py" \
  "start_camera:=true" \
  "enable_depth:=true" \
  "enable_pointcloud:=false" \
  "show_image:=true" \
  "fruit_detector_model:=${JAKA_FRUIT_DETECTOR_MODEL}" \
  "fruit_detector_confidence:=0.25" \
  "color_profile:=${COLOR_PROFILE}" \
  "depth_profile:=${DEPTH_PROFILE}"

wait_for_real_rgbd_frames 40
