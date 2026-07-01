# 方案B：Docker Desktop for Windows 部署指南

> 使用 Docker Desktop (WSL2 后端) 在 Windows 10/11 上运行 JAKA Twin Guard。

## 前置条件

- Windows 10 2004+ 或 Windows 11
- [Docker Desktop for Windows](https://www.docker.com/products/docker-desktop/)（WSL2 后端）
- [VcXsrv](https://sourceforge.net/projects/vcxsrv/)（X11 Server）

## 一次性安装（10 分钟）

### 1. 安装 Docker Desktop

1. 下载 [Docker Desktop](https://www.docker.com/products/docker-desktop/)
2. 安装时确保勾选 **"Use WSL 2 instead of Hyper-V"**
3. Settings → Resources → WSL Integration → 启用你的 Ubuntu 发行版
4. 重启电脑

### 2. 安装 VcXsrv

1. 下载 [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. 启动 XLaunch，配置：Multiple windows, Display 0, **勾选 Disable access control**
3. 填入 Extra: `-ac -nowgl -multiwindow`
4. Windows 防火墙：以**管理员**运行 `.\docker\win\setup-firewall.ps1`

### 3. 构建镜像

```powershell
cd d:\jaka_twin_guard
docker compose -f docker/docker-compose.yml build
```

### 4. 首次编译

```powershell
docker compose -f docker/docker-compose.yml run --rm jaka-twin-guard /launch/build.sh
```

## 日常使用

### 快速启动

```powershell
# PowerShell（管理员或普通用户均可）
cd d:\jaka_twin_guard
.\docker\win\start.ps1
```

### 在容器内

```bash
# 构建
/launch/build.sh

# 按摩仿真
/launch/massage-gazebo.sh

# 搬运仿真
/launch/carry-rviz.sh
```

### 停止

```powershell
.\docker\win\stop.ps1
```

### 重新构建镜像

```powershell
.\docker\win\restart.ps1 -Rebuild
```

## 环境诊断

```powershell
.\docker\win\check-env.ps1
```

检查 Docker Desktop、VcXsrv、端口、镜像是否全部就绪。

## 常见问题

| 问题 | 解决方案 |
|------|---------|
| 容器连不上 VcXsrv | 确认 VcXsrv 已启动且 `Disable access control` 已勾选 |
| Docker Desktop 未启动 | 检查系统托盘，启动 Docker Desktop |
| Gazebo 窗口全黑 | 确认 `LIBGL_ALWAYS_SOFTWARE=1` 已设置 |
| 端口 6000 被占用 | `netstat -ano \| findstr 6000` 查看，关闭占用进程 |
