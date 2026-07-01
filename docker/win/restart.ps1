# ============================================================
# 方案B: 重启 JAKA Twin Guard 容器
# 用法: .\docker\win\restart.ps1 [-Rebuild]
# ============================================================
param([switch]$Rebuild)

$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Path

# 停止
& "$scriptPath\stop.ps1"

# 启动 (传递 Rebuild 参数)
if ($Rebuild) {
    & "$scriptPath\start.ps1" -Rebuild
} else {
    & "$scriptPath\start.ps1"
}
