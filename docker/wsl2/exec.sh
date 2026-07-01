#!/bin/bash
# ============================================================
# 方案A: 快捷进入容器
# 用法: bash docker/wsl2/exec.sh [command]
#   bash docker/wsl2/exec.sh              → 交互式 bash
#   bash docker/wsl2/exec.sh /launch/build.sh  → 执行构建
# ============================================================
set -e

if [ $# -gt 0 ]; then
    docker exec -it jaka_twin_guard "$@"
else
    echo "进入 JAKA Twin Guard 容器..."
    docker exec -it jaka_twin_guard bash
fi
