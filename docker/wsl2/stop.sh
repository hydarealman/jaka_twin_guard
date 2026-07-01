#!/bin/bash
# ============================================================
# 方案A: 停止 JAKA Twin Guard 容器
# ============================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "[stop] 停止 JAKA Twin Guard 容器..."
cd "$PROJECT_DIR"
docker compose -f docker/docker-compose-wsl2.yml down

echo "[stop] 容器已停止 (编译产物保留在 ~/.jaka_docker/)"
