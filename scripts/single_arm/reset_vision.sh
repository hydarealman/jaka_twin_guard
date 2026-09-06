#!/usr/bin/env bash
set -eo pipefail

# Fruit-only perception reset for either real-hardware run mode.  This keeps
# D455/hand-eye calibration and the perceived table, clears temporal trackers,
# and forces the next target to pass a new five-frame stability window.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
source_ros_environment

request_light_vision_reset 20 || \
  die "light vision reset failed; keep automatic motion stopped and inspect the active log"

echo "[fruit-arm] vision is reset; the next fruit must become stable for 5 new frames"
