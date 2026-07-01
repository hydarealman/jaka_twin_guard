# 方案A：WSL2 原生 Docker 部署指南

> 在 WSL2 Ubuntu 22.04 中直接安装 Docker Engine，无需 Docker Desktop。

## 前置条件

- Windows 10/11 + WSL2 (Ubuntu 22.04)
- [VcXsrv](https://sourceforge.net/projects/vcxsrv/)（X11 Server，用于显示 Gazebo/RViz 窗口）

## 一次性安装（10 分钟）

### 1. 安装 VcXsrv（Windows 侧）

1. 下载并安装 [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. 启动 XLaunch，配置：
   - Display settings: Multiple windows, Display number = 0
   - **勾选 "Disable access control"**（必须！）
   - Extra settings: `-ac -nowgl -multiwindow`
3. 保存配置到 `%APPDATA%\Xming\Xlaunch\jaka_config.xlaunch`
4. **Windows 防火墙**：允许 `vcxsrv.exe` 通过（控制面板 → 防火墙 → 允许应用）

### 2. 安装 Docker Engine（WSL2 终端）

```bash
cd /mnt/d/jaka_twin_guard
bash docker/wsl2/setup-docker.sh
# 安装完成后，关闭并重新打开 WSL2 终端
```

### 3. 首次准备

```bash
# 每次打开 WSL2 后，启动 Docker daemon
sudo service docker start

# 创建本地数据目录（编译产物，放 ext4 加速）
mkdir -p ~/.jaka_docker/{install,build,log}

# 确保 VcXsrv 已启动（Windows 桌面右下角系统托盘）

# 构建 Docker 镜像
cd /mnt/d/jaka_twin_guard
docker compose -f docker/docker-compose-wsl2.yml build

# 首次构建 ROS2 workspace
docker compose -f docker/docker-compose-wsl2.yml run --rm jaka-twin-guard /launch/build.sh
```

## 日常使用

### 快速启动

```bash
# 在 WSL2 终端中（确保 VcXsrv 已启动）
cd /mnt/d/jaka_twin_guard
bash docker/wsl2/start.sh      # 启动容器
bash docker/wsl2/exec.sh       # 进入容器
```

### 在容器内

```bash
# 修改代码后重新构建
/launch/build.sh

# 彻底清理重来
/launch/rebuild.sh

# 启动按摩仿真（Gazebo + RViz）
/launch/massage-gazebo.sh

# 启动搬运仿真
/launch/carry-rviz.sh

# 更多选项见下方命令表
```

### 停止

```bash
bash docker/wsl2/stop.sh
```

## 便捷别名

追加到 `~/.bashrc`：

```bash
alias jaka-start='cd /mnt/d/jaka_twin_guard && sudo service docker start && \
  docker compose -f docker/docker-compose-wsl2.yml up -d'
alias jaka-enter='docker exec -it jaka_twin_guard bash'
alias jaka-stop='docker compose -f docker/docker-compose-wsl2.yml down'
alias jaka-massage='docker exec -it jaka_twin_guard /launch/massage-gazebo.sh'
alias jaka-build='docker exec -it jaka_twin_guard /launch/build.sh'
```

## 可用启动命令

| 命令 | 说明 | 需要 GUI |
|------|------|----------|
| `/launch/massage-gazebo.sh` | 工业级按摩（Gazebo 物理引擎） | ✅ Gazebo + RViz |
| `/launch/massage-gazebo-headless.sh` | 无头按摩（无 GUI） | ❌ |
| `/launch/massage-mock.sh` | Mock 按摩（仅 RViz） | ✅ RViz |
| `/launch/carry-rviz.sh` | 搬运仿真（RViz） | ✅ RViz |
| `/launch/carry-gazebo.sh` | 搬运仿真（Gazebo） | ✅ Gazebo + RViz |
| `/launch/carry-rviz-scene.sh b` | 搬运场景B（料框拣选） | ✅ RViz |
| `/launch/carry-gazebo-scene.sh c` | 搬运场景C（传送带） | ✅ Gazebo + RViz |
| `/launch/pick-place.sh` | 单臂抓取 | ✅ Gazebo + RViz |
| `/launch/old-massage.sh` | 旧按摩 Demo | ✅ RViz |
| `/launch/old-carry.sh` | 旧搬运 Demo | ✅ RViz |

## 故障排除

### VcXsrv 连接不上

```bash
# 检查 VcXsrv 是否运行（Windows PowerShell）
Get-Process vcxsrv

# 检查 DISPLAY 是否正确
echo $DISPLAY  # 应类似 172.x.x.x:0.0

# 检查端口连通
WSL_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}')
timeout 1 bash -c "echo > /dev/tcp/${WSL_IP}/6000" && echo "OK" || echo "FAIL"
```

### Docker daemon 未启动

```bash
# 启动 dockerd
sudo service docker start

# 或设置自动启动（添加到 ~/.bashrc）
if ! pgrep -x dockerd > /dev/null; then
    sudo service docker start > /dev/null 2>&1
fi
```

### 构建失败（colcon error）

```bash
# 清理重来
docker exec -it jaka_twin_guard /launch/rebuild.sh
```

### 镜像重新构建

```bash
# 源码脚本改了？不需要重构建镜像（volume 挂载）
# Dockerfile 或依赖改了？重构建镜像：
cd /mnt/d/jaka_twin_guard
docker compose -f docker/docker-compose-wsl2.yml build --no-cache
```
