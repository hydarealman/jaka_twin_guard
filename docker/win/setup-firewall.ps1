# ============================================================
# 方案B: 配置 Windows 防火墙, 允许 VcXsrv 入站连接
# 需要管理员权限运行
# ============================================================
Write-Host "============================================" -ForegroundColor Cyan
Write-Host " JAKA Twin Guard — 防火墙配置" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host ""

# 检查管理员权限
if (-NOT ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {
    Write-Host "[ERROR] 需要管理员权限! 请以管理员身份运行 PowerShell 后重新执行。" -ForegroundColor Red
    Write-Host "        右键 PowerShell → 以管理员身份运行"
    exit 1
}

# 检查 VcXsrv 路径
$vcxsrvPaths = @(
    "C:\Program Files\VcXsrv\vcxsrv.exe",
    "C:\Program Files (x86)\VcXsrv\vcxsrv.exe"
)

$vcxsrvPath = $null
foreach ($p in $vcxsrvPaths) {
    if (Test-Path $p) {
        $vcxsrvPath = $p
        break
    }
}

if (-not $vcxsrvPath) {
    Write-Host "[WARNING] 未检测到 VcXsrv 安装。如果已安装到其他路径，请手动添加防火墙规则。" -ForegroundColor Yellow
    Write-Host "          需要允许 vcxsrv.exe 的 TCP 6000 入站连接。"
    exit 0
}

Write-Host "[setup] 找到 VcXsrv: $vcxsrvPath"

# 添加防火墙规则
$ruleName = "VcXsrv X11 Server (JAKA Twin Guard)"
$existingRule = Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue

if ($existingRule) {
    Write-Host "[setup] 防火墙规则已存在，更新中..."
    Remove-NetFirewallRule -DisplayName $ruleName
}

New-NetFirewallRule -DisplayName $ruleName `
    -Direction Inbound `
    -Protocol TCP `
    -LocalPort 6000 `
    -Program $vcxsrvPath `
    -Action Allow `
    -Profile Any

Write-Host "[OK] 防火墙规则已添加: $ruleName" -ForegroundColor Green
