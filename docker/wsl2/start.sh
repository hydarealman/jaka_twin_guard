#!/bin/bash
# ============================================================
# 方案A: 启动 JAKA Twin Guard 容器
# 用法: bash docker/wsl2/start.sh
# ============================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "============================================"
echo " JAKA Twin Guard — 启动容器 (WSL2 原生)"
echo "============================================"

# 1. 检查 Docker daemon 是否运行
if ! pgrep -x dockerd > /dev/null; then
    echo "[start] 启动 Docker daemon..."
    sudo service docker start
    sleep 2
fi

# 2. 动态获取 DISPLAY (WSL2 网关 IP)
WSL_HOST_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
export DISPLAY="${WSL_HOST_IP}:0.0"
echo "[start] DISPLAY=${DISPLAY}"

# 3. 检查 VcXsrv 是否可达
if command -v timeout &> /dev/null; then
    if timeout 1 bash -c "echo > /dev/tcp/${WSL_HOST_IP}/6000" 2>/dev/null; then
        echo "[start] VcXsrv 已就绪 (${WSL_HOST_IP}:6000)"
    else
        echo "[start] WARNING: VcXsrv 端口 6000 不可达! 请确认 VcXsrv 已在 Windows 侧启动。"
        echo "         启动参数: -ac -nowgl -multiwindow"
    fi
fi

# 4. 创建 WSL2 本地数据目录 (编译产物, ext4 快速)
mkdir -p ~/.jaka_docker/{install,build,log}

# 5. 设置环境变量
export PROJECT_ROOT="${PROJECT_ROOT:-/mnt/d/jaka_twin_guard}"
export DOCKER_DATA="${DOCKER_DATA:-${HOME}/.jaka_docker}"

# 6. 构建并启动容器
cd "$PROJECT_DIR"
docker compose -f docker/docker-compose-wsl2.yml up -d

echo ""
echo "[start] 容器已启动!"
echo "  进入容器: bash docker/wsl2/exec.sh"
echo "  构建:     docker exec -it jaka_twin_guard /launch/build.sh"
echo "  按摩:     docker exec -it jaka_twin_guard /launch/massage-gazebo.sh"
echo "  停止:     bash docker/wsl2/stop.sh"
