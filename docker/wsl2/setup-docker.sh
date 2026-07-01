#!/bin/bash
# ============================================================
# WSL2 Ubuntu 22.04 — 一键安装 Docker Engine
# 不需要 Docker Desktop, 直接在 WSL2 内运行 Docker daemon
# ============================================================
set -e

echo "============================================"
echo " JAKA Twin Guard — Docker Engine 安装向导"
echo " 目标: WSL2 Ubuntu 22.04"
echo "============================================"
echo ""

# 检查是否为 Ubuntu
if [ ! -f /etc/os-release ]; then
    echo "[ERROR] 无法检测操作系统。请确认在 WSL2 Ubuntu 22.04 中运行。"
    exit 1
fi

source /etc/os-release
if [ "$ID" != "ubuntu" ] || [ "$VERSION_ID" != "22.04" ]; then
    echo "[WARNING] 推荐 Ubuntu 22.04, 当前为: $ID $VERSION_ID"
    echo "          按 Enter 继续, Ctrl+C 取消"
    read -r
fi

# 1. 修复第三方仓库 GPG 密钥 (避免 apt update 失败)
echo "[1/7] 修复第三方仓库 GPG 密钥..."
# influxdata (如果有)
if [ -f /etc/apt/sources.list.d/influxdata.list ] || grep -r "influxdata" /etc/apt/sources.list.d/ 2>/dev/null; then
    curl -fsSL https://repos.influxdata.com/influxdata-archive_compat.key | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/influxdata.gpg 2>/dev/null || true
fi
# 通用: 用 --allow-releaseinfo-change 避免签名过期问题
# 清除已有问题
sudo mv /etc/apt/sources.list.d/influxdata.list /etc/apt/sources.list.d/influxdata.list.bak 2>/dev/null || true
sudo mv /etc/apt/sources.list.d/influxdb.list /etc/apt/sources.list.d/influxdb.list.bak 2>/dev/null || true

# 2. 清理旧版本
echo "[2/7] 清理旧版本 Docker..."
sudo apt-get remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true

# 3. 安装依赖
echo "[3/7] 安装依赖..."
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg lsb-release

# 4. 添加 Docker GPG 密钥 (国内网络先从镜像源获取)
echo "[4/7] 添加 Docker GPG 密钥..."

DOCKER_GPG_OK=false

# 尝试官方源
if curl -fsSL --connect-timeout 5 https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg 2>/dev/null; then
    DOCKER_GPG_OK=true
    echo "[4/7] 官方 GPG 密钥获取成功"
fi

# 官方失败, 用阿里云镜像
if [ "$DOCKER_GPG_OK" = false ]; then
    echo "[4/7] 官方超时, 尝试阿里云镜像..."
    curl -fsSL --connect-timeout 10 https://mirrors.aliyun.com/docker-ce/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    DOCKER_GPG_OK=true
fi

sudo chmod a+r /etc/apt/keyrings/docker.gpg

# 5. 添加 Docker apt 源 (自动选择可用源)
echo "[5/7] 添加 Docker apt 源..."

# 测试哪个源可用
DOCKER_REPO_URL=""
if curl -fsSL --connect-timeout 5 -o /dev/null https://download.docker.com/linux/ubuntu/ 2>/dev/null; then
    DOCKER_REPO_URL="https://download.docker.com/linux/ubuntu"
    echo "[5/7] 使用 Docker 官方源"
elif curl -fsSL --connect-timeout 5 -o /dev/null https://mirrors.aliyun.com/docker-ce/linux/ubuntu/ 2>/dev/null; then
    DOCKER_REPO_URL="https://mirrors.aliyun.com/docker-ce/linux/ubuntu"
    echo "[5/7] 使用 阿里云 镜像源"
elif curl -fsSL --connect-timeout 5 -o /dev/null https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu/ 2>/dev/null; then
    DOCKER_REPO_URL="https://mirrors.ustc.edu.cn/docker-ce/linux/ubuntu"
    echo "[5/7] 使用 中科大 镜像源"
else
    # 默认用阿里云 (在国内大概率通)
    DOCKER_REPO_URL="https://mirrors.aliyun.com/docker-ce/linux/ubuntu"
    echo "[5/7] 默认使用 阿里云 镜像源"
fi

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  ${DOCKER_REPO_URL} $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# 6. 安装 Docker Engine + Compose Plugin
echo "[6/7] 安装 Docker Engine + Compose Plugin..."
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

# 7. 免 sudo 运行 docker
echo "[7/7] 配置用户权限..."
sudo usermod -aG docker "$USER"

# 8. 配置 Docker Hub 镜像加速器
echo ""
echo "[extra] 配置 Docker Hub 镜像加速器..."
sudo mkdir -p /etc/docker
cat << 'DAEMONJSON' | sudo tee /etc/docker/daemon.json > /dev/null
{
  "registry-mirrors": [
    "https://docker.m.daocloud.io",
    "https://dockerproxy.com",
    "https://hub-mirror.c.163.com",
    "https://mirror.baidubce.com",
    "https://docker.1panel.live"
  ],
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
DAEMONJSON
echo "[extra] 镜像加速器已配置"

echo ""
echo "============================================"
echo " 安装完成!"
echo "============================================"
echo ""
echo " 下一步:"
echo "   1. 关闭并重新打开 WSL2 终端"
echo "      (或者运行: newgrp docker)"
echo "   2. 每次打开 WSL2 后启动 Docker daemon:"
echo "      sudo service docker start"
echo "   3. 验证安装:"
echo "      docker run hello-world"
echo ""
echo " 自动启动 Docker daemon (可选):"
echo "   在 ~/.bashrc 末尾添加:"
echo "   if ! pgrep -x dockerd > /dev/null; then"
echo '     sudo service docker start > /dev/null 2>&1'
echo "   fi"
echo ""
