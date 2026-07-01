#!/bin/bash
# ============================================================
# Docker Hub 镜像加速器配置 (国内用户)
# 用法: bash docker/wsl2/configure-mirrors.sh
# ============================================================
set -e

echo "=== 配置 Docker Hub 镜像加速器 ==="

sudo service docker stop 2>/dev/null || true

sudo mkdir -p /etc/docker
cat << 'JSON' | sudo tee /etc/docker/daemon.json
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
JSON

sudo service docker start
echo "=== 镜像加速器已配置 ==="
docker info 2>/dev/null | grep -A10 "Registry Mirrors" || echo "验证中..."
