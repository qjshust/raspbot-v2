#!/usr/bin/env bash
# install_service.sh —— 安装 Pi 端上位机网关 systemd 服务（可选）
#
# 用法（在 Pi 上，需要 sudo；本脚本会 sudo）：
#   bash ~/tools/console/install_service.sh                  # 安装并 enable，不立即启动
#   bash ~/tools/console/install_service.sh --start          # 安装并立即启动（推荐）
#   bash ~/tools/console/install_service.sh --attach         # 附加模式（外部已有 bringup 栈）
#   bash ~/tools/console/install_service.sh --port 8080 --token mytoken
#   bash ~/tools/console/install_service.sh --start --force  # 允许与 raspbot.service 同时启用
#   可选：--maps-dir DIR（默认 ~/ros2_ws/maps）、--maps-fallback DIR、
#         --ros-log-dir DIR（默认 /tmp/console_roslog，避开易掉盘的 /mnt/ssd）
#
# 说明：
# - 服务安装为 raspbot-console.service，开机自启，崩溃 3 s 后重启。
# - 与 raspbot.service（bringup）二选一：网关托管 bringup 时，应 disable raspbot.service；
#   若希望 systemd 启动 bringup，则网关必须用 --attach（或至少不要拉起会话）。
# - 卸载：bash install_service.sh --uninstall
set -euo pipefail

PORT=8080
TOKEN=""
ATTACH=0
START=0
FORCE=0
UNINSTALL=0
# 台架默认避开 USB SSD（本机 /mnt/ssd 有 UAS 掉盘史：日志 I/O 会阻塞节点线程）
MAPS_DIR="$HOME/ros2_ws/maps"
MAPS_FALLBACK="$HOME/ros2_ws/maps"
ROS_LOG_DIR="/tmp/console_roslog"
UNIT=/etc/systemd/system/raspbot-console.service

usage() {
  sed -n '2,18p' "$0"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --port) PORT="${2:?}"; shift 2 ;;
    --token) TOKEN="${2:?}"; shift 2 ;;
    --maps-dir) MAPS_DIR="${2:?}"; shift 2 ;;
    --maps-fallback) MAPS_FALLBACK="${2:?}"; shift 2 ;;
    --ros-log-dir) ROS_LOG_DIR="${2:?}"; shift 2 ;;
    --attach) ATTACH=1; shift ;;
    --start) START=1; shift ;;
    --force) FORCE=1; shift ;;
    --uninstall) UNINSTALL=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1" >&2; usage; exit 2 ;;
  esac
done

if [[ "$UNINSTALL" -eq 1 ]]; then
  sudo systemctl disable --now raspbot-console.service 2>/dev/null || true
  sudo rm -f "$UNIT"
  sudo systemctl daemon-reload
  echo "[OK] 已卸载 raspbot-console.service"
  exit 0
fi

if [[ -z "$TOKEN" ]]; then
  echo "[WARN] 未设置 --token。网关将允许局域网内任意客户端访问，请只在可信内网使用。" >&2
fi

# 二选一检查：外部 bringup 栈 + 网关托管会互相抢 ROS 图
if systemctl is-active --quiet raspbot.service 2>/dev/null; then
  if [[ "$ATTACH" -eq 0 && "$FORCE" -ne 1 ]]; then
    echo "[FAIL] raspbot.service 正在运行，而本脚本默认让网关托管 bringup。" >&2
    echo "       请任选其一：" >&2
    echo "         sudo systemctl disable --now raspbot.service   # 网关托管" >&2
    echo "         或加 --attach 参数（网关只附加、不启停栈）" >&2
    echo "         或加 --force 跳过本检查" >&2
    exit 3
  fi
  [[ "$ATTACH" -eq 1 ]] && echo "[info] 使用 --attach：网关不会启停 bringup 栈"
fi

WS="$HOME/ros2_ws"
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "[FAIL] 未找到 $WS/install/setup.bash，请先 colcon build（用 tools/console/deploy.ps1）" >&2
  exit 1
fi

EXTRA=""
[[ "$ATTACH" -eq 1 ]] && EXTRA="$EXTRA --attach"
[[ -n "$TOKEN" ]] && EXTRA="$EXTRA --token $TOKEN"
EXTRA="$EXTRA --maps-dir $MAPS_DIR --maps-fallback $MAPS_FALLBACK --ros-log-dir $ROS_LOG_DIR"

echo "== 写入 $UNIT =="
sudo tee "$UNIT" > /dev/null <<EOF
[Unit]
Description=RASPBOT-V2 PC console gateway (raspbot_console)
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
Group=$USER
Environment=HOME=/home/$USER
ExecStartPre=/bin/mkdir -p $ROS_LOG_DIR
ExecStart=/bin/bash -lc 'source /opt/ros/lyrical/setup.bash && source $WS/install/setup.bash && exec ros2 run raspbot_console console_bridge --host 0.0.0.0 --port $PORT$EXTRA'
Restart=always
RestartSec=3
# 网关本身不碰 I2C；退出时由 ROS 看门狗兜底停车
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
EOF
sudo chmod 600 "$UNIT"

echo "== daemon-reload + enable =="        # 幂等
sudo systemctl daemon-reload
sudo systemctl enable raspbot-console.service

if [[ "$START" -eq 1 ]]; then
  sudo systemctl restart raspbot-console.service
  sleep 2
  systemctl status raspbot-console.service --no-pager || true
  echo
  echo "[OK] 已启动。查看日志：journalctl -u raspbot-console.service -n 100 -f"
  echo "     健康检查：curl -s http://127.0.0.1:$PORT/api/health"
else
  echo "[OK] 已安装（未启动）。启动：sudo systemctl start raspbot-console.service"
fi
echo "     PC 端连接：http://$(hostname -I | awk '{print $1}'):$PORT"
