# JAKA Twin Guard — Docker 容器化部署

> 将 JAKA C5 双机械臂仿真环境打包为 Docker 容器，一键启动、环境隔离、可复现。

---

## 快速决策：选哪个方案？

| | **方案A：WSL2 原生 Docker** | **方案B：Docker Desktop** |
|---|---|---|
| 适合谁 | 已有 WSL2，偏好命令行 | 需要图形化管理界面 |
| 安装步骤 | 1 条命令 | 下载安装 Docker Desktop |
| 资源占用 | 轻量 | +500MB 内存 |
| 日常操作 | `bash docker/wsl2/start.sh` | `.\docker\win\start.ps1` |
| 详细指南 | [wsl2/README.md](wsl2/README.md) | [win/README.md](win/README.md) |
| **推荐度** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |

**如果你已经有 WSL2（当前项目的运行方式），选方案A。**

---

## 目录结构

```
docker/
├── README.md                          # ← 你在这里 (总览)
├── Dockerfile                         # 多阶段构建 (deps → dev/prod)
├── entrypoint.sh                      # 容器入口 (source ROS2, 设环境变量)
├── .dockerignore                      # 排除 build/.git/__pycache__
│
├── docker-compose.yml                 # 方案B: Docker Desktop 专用
├── docker-compose-wsl2.yml            # 方案A: WSL2 原生 Docker 专用
├── docker-compose.gpu.yml            # GPU 加速覆盖 (两份方案通用)
├── .env                               # 方案B 环境变量
├── .env.wsl2                          # 方案A 环境变量
│
├── launch/                            # 14 个一键启动脚本
│   ├── build.sh                       # colcon build (3个JAKA包)
│   ├── rebuild.sh                     # 清理 + 重新构建
│   ├── massage-gazebo.sh              # ★ 按摩仿真 (Gazebo 物理引擎)
│   ├── massage-gazebo-headless.sh    # 无头按摩 (无 GUI, 适合 CI)
│   ├── massage-mock.sh               # Mock 按摩 (无 Gazebo)
│   ├── carry-rviz.sh                  # 搬运仿真 (RViz)
│   ├── carry-rviz-scene.sh            # 搬运 + 指定场景 (a/b/c)
│   ├── carry-gazebo.sh               # 搬运仿真 (Gazebo)
│   ├── carry-gazebo-scene.sh         # 搬运 + 指定场景 (a/b/c)
│   ├── old-massage.sh                 # 旧按摩 Demo
│   ├── old-carry.sh                   # 旧搬运 Demo
│   ├── pick-place.sh                  # 单臂抓取
│   └── clean.sh                       # 清理缓存 + 残留进程
│
├── config/
│   └── cyclonedds.xml                 # Cyclone DDS 配置 (可选)
│
├── wsl2/                              # 方案A 管理工具
│   ├── README.md                      # 方案A 完整操作指南
│   ├── setup-docker.sh               # 一键安装 Docker Engine
│   ├── configure-mirrors.sh          # Docker Hub 镜像加速
│   ├── start.sh                       # 启动容器
│   ├── stop.sh                        # 停止容器
│   └── exec.sh                        # 快捷进入容器
│
└── win/                               # 方案B 管理工具
    ├── README.md                      # 方案B 完整操作指南
    ├── start.ps1                      # 启动容器 (PowerShell)
    ├── stop.ps1                       # 停止容器
    ├── restart.ps1                    # 重启容器
    ├── check-env.ps1                  # 环境诊断
    └── setup-firewall.ps1            # 防火墙配置
```

---

## 架构概览

### 方案A（WSL2 原生 Docker）— 推荐

```
Windows 10 桌面
├─ VcXsrv (X11 Server, TCP 6000)  ← Gazebo/RViz 窗口显示
└─ WSL2 (Ubuntu 22.04)
   ├─ Docker Engine (dockerd)
   └─ 容器 jaka_twin_guard
      ├─ gzserver + gzclient (Gazebo 11)
      ├─ move_group (MoveIt2)
      ├─ rviz2
      ├─ controller_manager (via libgazebo_ros2_control.so)
      └─ massage_runner / carry_task_runner
      源码: /mnt/d/jaka_twin_guard/src (volume mount)
      编译: ~/.jaka_docker/ (WSL2 ext4, 快速)
```

### 方案B（Docker Desktop）— 备选

```
Windows 10 桌面
├─ Docker Desktop (WSL2 后端)
│  └─ 容器 jaka_twin_guard  (同上)
└─ VcXsrv (X11 Server)
```

---

## 方案A 快速开始（5 分钟）

### 前置条件

- WSL2 Ubuntu 22.04 + [VcXsrv](https://sourceforge.net/projects/vcxsrv/)（X11 显示转发）

### 安装

```bash
# 1. 安装 Docker Engine (WSL2 内)
cd /mnt/d/jaka_twin_guard
bash docker/wsl2/setup-docker.sh
# 关闭并重新打开 WSL2 终端

# 2. 启动 Docker
sudo service docker start

# 3. 创建数据目录
mkdir -p ~/.jaka_docker/{install,build,log}
```

### 构建 & 运行

```bash
# 4. 构建镜像 (首次约 20-60 分钟, 约 5GB 下载)
docker compose -f docker/docker-compose-wsl2.yml build

# 5. 编译 ROS2 workspace
docker compose -f docker/docker-compose-wsl2.yml run --rm jaka-twin-guard /launch/build.sh

# 6. 启动容器
bash docker/wsl2/start.sh

# 7. 进入容器, 启动按摩
docker exec -it jaka_twin_guard /launch/massage-gazebo.sh
# Gazebo + RViz 窗口出现在 Windows 桌面!
```

### 日常使用

```bash
# 启动 → 进入 → 按摩，一条龙
bash docker/wsl2/start.sh && docker exec -it jaka_twin_guard /launch/massage-gazebo.sh
```

---

## 方案B 快速开始

详见 [win/README.md](win/README.md)

---

## 容器内命令速查

| 命令 (`/launch/`) | 说明 | GUI |
|---|---|---|
| `build.sh` | colcon 构建 | - |
| `rebuild.sh` | 清理 + 重新构建 | - |
| `massage-gazebo.sh` | ★ 工业级按摩 (Gazebo) | Gazebo + RViz |
| `massage-gazebo-headless.sh` | 无头按摩 (无 GUI) | ❌ |
| `massage-mock.sh` | Mock 按摩 (仅 RViz) | RViz |
| `carry-rviz.sh` | 搬运仿真 (RViz) | RViz |
| `carry-rviz-scene.sh b` | 搬运场景B (料框) | RViz |
| `carry-gazebo.sh` | 搬运仿真 (Gazebo) | Gazebo + RViz |
| `carry-gazebo-scene.sh c` | 搬运场景C (传送带) | Gazebo + RViz |
| `pick-place.sh` | 单臂抓取 | Gazebo + RViz |
| `old-massage.sh` | 旧按摩 Demo | RViz |
| `old-carry.sh` | 旧搬运 Demo | RViz |
| `clean.sh` | 清理缓存 | - |

---

## 目录映射

| 宿主机 | 容器内 | 说明 |
|--------|--------|------|
| `d:\jaka_twin_guard\src\` (Windows) | `/workspace/src/` | 源码 (读写, IDE 编辑) |
| `~/.jaka_docker/build/` (WSL2 ext4) | `/workspace/build/` | 编译中间文件 |
| `~/.jaka_docker/install/` (WSL2 ext4) | `/workspace/install/` | 编译产物 |
| `~/.jaka_docker/log/` (WSL2 ext4) | `/workspace/log/` | ROS2 日志 |

---

## 渲染模式

| 模式 | 环境变量 | 性能 | 硬件要求 |
|------|---------|------|----------|
| **软件渲染** (默认) | `LIBGL_ALWAYS_SOFTWARE=1` | Gazebo 5-15 FPS | 无 |
| **GPU 加速** (可选) | `LIBGL_ALWAYS_SOFTWARE=0` + NVIDIA runtime | 接近原生 | NVIDIA 显卡 |

GPU 模式启动：
```bash
# 方案A
docker compose -f docker/docker-compose-wsl2.yml -f docker/docker-compose.gpu.yml up -d

# 方案B
docker compose -f docker/docker-compose.yml -f docker/docker-compose.gpu.yml up -d
```

---

## 常见问题

| 问题 | 解决 |
|------|------|
| Docker 连不上 | `sudo service docker start` |
| 镜像下载慢 | `bash docker/wsl2/configure-mirrors.sh` |
| VcXsrv 窗口没出现 | 确认 VcXsrv 启动参数 `-ac -nowgl -multiwindow` |
| 容器内 `colcon` 失败 | 先跑 `/launch/rebuild.sh` 清理重来 |
| 端口 6000 不通 | Windows 防火墙放行 `vcxsrv.exe` |

---

## 更多信息

- 详细故障排除：[wsl2/README.md](wsl2/README.md) 和 [win/README.md](win/README.md)
- 项目总览：[CLAUDE.md](../CLAUDE.md)
