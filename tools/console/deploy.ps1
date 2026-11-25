# deploy.ps1 —— 把 Pi 端网关 raspbot_console + tools/console 同步到 Pi 并构建
#
# 用法（在仓库根或任意位置）：
#   pwsh -File tools\console\deploy.ps1
#   pwsh -File tools\console\deploy.ps1 -PiHost ubuntu@192.168.50.98 -NoBuild
#   pwsh -File tools\console\deploy.ps1 -Askpass E:\code\raspberrypi\.dsh\askpass.cmd
#
# 只同步并构建 `ros2_ws/src/raspbot_console`（不碰其他包），同时把 tools/console
# 复制到 Pi 的 ~/tools/console（用于 acceptance_sil.sh）。
# 前置：PC 能 ssh/scp 到 Pi；Pi 上 ROS 2 Lyrical 与 ~/ros2_ws 已存在。
param(
  [string]$PiHost = "ubuntu@192.168.50.98",
  [switch]$NoBuild,
  [switch]$SkipTests,
  [string]$Askpass = "",
  [string]$IdentityFile = "",
  [int]$TimeoutSec = 300
)
$ErrorActionPreference = 'Continue'
$Root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
Write-Output "== 仓库根: $Root =="

if ($Askpass) {
  $env:SSH_ASKPASS = $Askpass
  $env:SSH_ASKPASS_REQUIRE = 'force'
  $env:DISPLAY = 'localhost:0'
}
$sshOpts = @('-o', 'StrictHostKeyChecking=accept-new', '-o', 'ConnectTimeout=10',
             '-o', 'LogLevel=ERROR', '-o', 'ServerAliveInterval=15')
if ($IdentityFile) { $sshOpts += @('-i', $IdentityFile) }

# ---------- 1. 打包（显式限定范围；排除缓存） ----------
$stage = Join-Path $Root '.dsh'
if (-not (Test-Path $stage)) { New-Item -ItemType Directory -Path $stage | Out-Null }
$tgz = Join-Path $stage 'console_deploy.tgz'
if (Test-Path $tgz) { Remove-Item $tgz -Force }
$tar = Get-Command tar.exe -ErrorAction SilentlyContinue
if (-not $tar) { Write-Output '[FAIL] 找不到 tar.exe（Windows 10+ 自带）'; exit 1 }
Write-Output '== 打包 ros2_ws/src/raspbot_console + tools/console =='
& $tar.Source -czf $tgz -C $Root `
  --exclude '__pycache__' --exclude '*.pyc' --exclude 'pytest-cache-files-*' `
  ros2_ws/src/raspbot_console tools/console 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0 -or -not (Test-Path $tgz)) {
  Write-Output "[FAIL] 打包失败（tar rc=$LASTEXITCODE）"; exit 1
}
$kb = [math]::Round((Get-Item $tgz).Length / 1KB, 1)
Write-Output "  包大小: $kb KB"

# ---------- 2. 上传 ----------
Write-Output '== 上传到 Pi =='
& scp @sshOpts -- $tgz "${PiHost}:/tmp/console_deploy.tgz" 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Output "[FAIL] scp 失败（rc=$LASTEXITCODE）"; exit 1 }

# ---------- 3. 远端安装 + 构建 + 纯逻辑测试 ----------
$build = if ($NoBuild) { 'SKIP' } else { 'BUILD' }
$test = if ($SkipTests) { 'SKIP' } else { 'TEST' }
$remote = @"
set -e
echo '== 解包（覆盖 raspbot_console 与 tools/console）=='
tar xzf /tmp/console_deploy.tgz -C `$HOME
rm -rf /tmp/console_deploy.tgz
chmod +x `$HOME/tools/console/*.sh 2>/dev/null || true
source /opt/ros/lyrical/setup.bash
if [ "$build" = "BUILD" ]; then
  echo '== colcon build（仅 raspbot_console）=='
  cd `$HOME/ros2_ws
  colcon build --symlink-install --packages-select raspbot_console > /tmp/console_build.log 2>&1
  RC=`$?
  tail -30 /tmp/console_build.log
  echo "BUILD_RC=`$RC"
  [ `$RC -eq 0 ] || exit `$RC
else
  echo 'SKIP_BUILD'
fi
if [ "$test" = "TEST" ]; then
  echo '== 纯逻辑单元测试（不依赖运行中的 ROS 图）=='
  cd `$HOME/ros2_ws/src/raspbot_console
  PYTHONPATH=`$PWD python3 -m unittest discover -s test -v 2>&1 | tail -40
fi
echo '== 安装后自检 =='
source `$HOME/ros2_ws/install/setup.bash
ros2 pkg prefix raspbot_console
ros2 pkg executables raspbot_console
echo DEPLOY_OK
"@
$remoteFile = Join-Path $stage 'console_deploy.sh'
[IO.File]::WriteAllText($remoteFile, $remote.Replace("`r`n", "`n"))
& scp @sshOpts -- $remoteFile "${PiHost}:/tmp/console_deploy.sh" 2>&1 | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Output '[FAIL] 上传构建脚本失败'; exit 1 }
& ssh @sshOpts $PiHost "bash /tmp/console_deploy.sh; rc=`$?; rm -f /tmp/console_deploy.sh; exit `$rc" 2>&1
$rc = $LASTEXITCODE
if ($rc -ne 0) { Write-Output "[FAIL] 远端部署失败（rc=$rc）"; exit $rc }

Write-Output ''
Write-Output '== 部署完成。下一步（在 Pi 上）=='
Write-Output '  1) 手动启动：source /opt/ros/lyrical/setup.bash && source ~/ros2_ws/install/setup.bash'
Write-Output '                 ros2 run raspbot_console console_bridge --port 8080'
Write-Output '  2) 或安装 systemd：bash ~/tools/console/install_service.sh --start'
Write-Output '  3) 台架 SIL 验收：bash ~/tools/console/acceptance_sil.sh'
Write-Output '  4) 若 raspbot.service（bringup）已启用，请二选一：'
Write-Output '     sudo systemctl stop raspbot.service   # 交给网关托管；或网关加 --attach'
exit 0
