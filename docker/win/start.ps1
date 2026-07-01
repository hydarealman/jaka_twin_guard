# ============================================================
# 方案B: 启动 JAKA Twin Guard (Docker Desktop)
# 用法: .\docker\win\start.ps1
# ============================================================
param(
    [switch]$Rebuild      # 启动前重新构建镜像
)

$ErrorActionPreference = "Stop"

Write-Host "============================================" -ForegroundColor Cyan
Write-Host " JAKA Twin Guard — 启动容器 (Docker Desktop)" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# 1. 检查 Docker Desktop 是否运行
try {
    $null = docker info 2>&1
    Write-Host "[OK] Docker Desktop 已运行" -ForegroundColor Green
} catch {
    Write-Host "[ERROR] Docker Desktop 未运行! 请先启动 Docker Desktop。" -ForegroundColor Red
    exit 1
}

# 2. 启动 VcXsrv (如果未运行)
$vcxsrv = Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue
if (-not $vcxsrv) {
    Write-Host "[start] 启动 VcXsrv..."
    $vcxsrvPath = "C:\Program Files\VcXsrv\vcxsrv.exe"
    $vcxsrvPathAlt = "C:\Program Files (x86)\VcXsrv\vcxsrv.exe"

    if (Test-Path $vcxsrvPath) {
        Start-Process -FilePath $vcxsrvPath -ArgumentList "-ac -nowgl -multiwindow"
    } elseif (Test-Path $vcxsrvPathAlt) {
        Start-Process -FilePath $vcxsrvPathAlt -ArgumentList "-ac -nowgl -multiwindow"
    } else {
        Write-Host "[WARNING] VcXsrv 未安装! 请从 https://sourceforge.net/projects/vcxsrv/ 下载安装" -ForegroundColor Yellow
        Write-Host "          启动参数: -ac -nowgl -multiwindow"
    }
    Start-Sleep -Seconds 2
} else {
    Write-Host "[OK] VcXsrv 已运行" -ForegroundColor Green
}

# 3. 设置 DISPLAY 环境变量
$env:DISPLAY = "host.docker.internal:0.0"
Write-Host "[start] DISPLAY=$env:DISPLAY"

# 4. 切换到项目目录
$projectDir = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
Set-Location $projectDir

# 5. 重新构建镜像 (如果指定)
if ($Rebuild) {
    Write-Host "[start] 重新构建镜像..."
    docker compose -f docker/docker-compose.yml build --no-cache
}

# 6. 启动容器
Write-Host "[start] 启动容器..."
docker compose -f docker/docker-compose.yml up -d

Write-Host ""
Write-Host "[start] 容器已启动!" -ForegroundColor Green
Write-Host "  进入容器: docker exec -it jaka_twin_guard bash"
Write-Host "  构建:     docker exec -it jaka_twin_guard /launch/build.sh"
Write-Host "  按摩:     docker exec -it jaka_twin_guard /launch/massage-gazebo.sh"
Write-Host "  停止:     .\docker\win\stop.ps1"
Write-Host ""
