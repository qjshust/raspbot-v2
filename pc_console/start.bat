@echo off
rem RASPBOT-V2 PC 控制台 —— 真车模式（默认连接 192.168.50.98:8080）
rem 用法: start.bat [额外参数，例如 --pi 192.168.50.98:8080 --port 8090]
chcp 65001 >nul
cd /d "%~dp0"
python main.py %*
if errorlevel 1 (
  echo.
  echo 启动失败（退出码 %errorlevel%）。请检查上方日志。
  pause
)
