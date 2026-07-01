# ============================================================
# 方案B: 诊断脚本 — 检查环境是否就绪
# ============================================================
Write-Host "============================================" -ForegroundColor Cyan
Write-Host " JAKA Twin Guard — 环境诊断" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

$allOk = $true

# 1. Docker Desktop
Write-Host "[1/4] Docker Desktop..." -NoNewline
try {
    $null = docker info 2>&1
    Write-Host " OK" -ForegroundColor Green
} catch {
    Write-Host " FAIL (Docker Desktop 未运行)" -ForegroundColor Red
    $allOk = $false
}

# 2. VcXsrv
Write-Host "[2/4] VcXsrv X11 Server..." -NoNewline
$vcxsrv = Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue
if ($vcxsrv) {
    Write-Host " OK (PID: $($vcxsrv.Id))" -ForegroundColor Green
} else {
    Write-Host " FAIL (VcXsrv 未运行)" -ForegroundColor Red
    $allOk = $false
}

# 3. 端口 6000
Write-Host "[3/4] X11 端口 6000..." -NoNewline
$tcp = Get-NetTCPConnection -LocalPort 6000 -ErrorAction SilentlyContinue
if ($tcp -and $tcp.State -eq "Listen") {
    Write-Host " OK (监听中)" -ForegroundColor Green
} else {
    Write-Host " WARN (可能有防火墙阻止)" -ForegroundColor Yellow
}

# 4. 容器镜像
Write-Host "[4/4] Docker 镜像..." -NoNewline
$image = docker images -q jaka-twin-guard:desktop 2>&1
if ($image) {
    Write-Host " OK" -ForegroundColor Green
} else {
    Write-Host " 未构建 (运行: docker compose -f docker/docker-compose.yml build)" -ForegroundColor Yellow
}

Write-Host ""
if ($allOk) {
    Write-Host "环境就绪!" -ForegroundColor Green
} else {
    Write-Host "存在问题，请根据上面的提示修复。" -ForegroundColor Yellow
}
