@echo off
rem RASPBOT-V2 PC 控制台 —— 演示模式（内置仿真，无需 Pi/ROS）
rem 用法: start-demo.bat [额外参数，例如 --port 8099 --no-browser]
chcp 65001 >nul
cd /d "%~dp0"
python main.py --demo %*
if errorlevel 1 (
  echo.
  echo 启动失败（退出码 %errorlevel%）。请检查上方日志。
  pause
)
