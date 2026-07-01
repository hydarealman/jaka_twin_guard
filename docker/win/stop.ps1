# ============================================================
# 方案B: 停止 JAKA Twin Guard 容器
# ============================================================
$ErrorActionPreference = "Stop"

Write-Host "[stop] 停止 JAKA Twin Guard 容器..." -ForegroundColor Yellow

$projectDir = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path))
Set-Location $projectDir

docker compose -f docker/docker-compose.yml down

Write-Host "[stop] 容器已停止" -ForegroundColor Green
