#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

stop_launch "b_sim"
rm -f /tmp/jaka_architecture_b_host /tmp/jaka_architecture_b_board
