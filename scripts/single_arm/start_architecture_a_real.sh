#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"

PORT="$(find /dev/serial/by-id -maxdepth 1 -type c 2>/dev/null | head -n 1 || true)"
PORT="${PORT:-/dev/ttyUSB0}"
BAUDRATE="115200"
# 模型许可审核完成后使用 true；否则改为 false。
MODEL_LICENSE_APPROVED="true"

[[ -e "${PORT}" ]] || die "没有找到电控板串口：${PORT}；请检查 USB 连接和串口权限"

echo "[single-arm] WARNING: starting physical hardware on ${PORT}"
echo "[single-arm] verify E-stop, power state, zero positions, limits and hand-eye calibration"

start_launch "a_real" "architecture_a_real.launch.py" \
  "serial_port:=${PORT}" \
  "baudrate:=${BAUDRATE}" \
  "model_license_approved:=${MODEL_LICENSE_APPROVED}"
