#!/usr/bin/env bash
# install_systemd.sh — 幂等安装 raspbot.service（FR-H-01 / SYS-F01）。
#
# 用法（在 Pi 上执行，需 sudo）：
#   bash system/install_systemd.sh            # 拷贝 unit + daemon-reload + enable
#   bash system/install_systemd.sh --start    # 同上并立即 systemctl start
#
# 卸载：
#   sudo systemctl disable --now raspbot.service
#   sudo rm /etc/systemd/system/raspbot.service && sudo systemctl daemon-reload
#
# ---- udev 规则（D-09：vcgencmd 读不到 /dev/vcio）----
# 本脚本不安装 udev 规则；如需修复 vcgencmd 权限，手动执行：
#   sudo cp system/99-raspbot-vcio.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger
#   # 确认：ls -l /dev/vcio 应为 root:video 0660；用户需在 video 组（groups）
# ---- powerguard（J-10 / SYS-S06）----
# 部署低电压保护时记得把 tools/powerguard/powerguard.json 的 dry_run 改为 false，
# 并安装其 systemd unit：tools/powerguard/powerguard.service。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNIT="raspbot.service"
DEST="/etc/systemd/system/${UNIT}"
START=0
if [[ "${1:-}" == "--start" ]]; then
    START=1
elif [[ -n "${1:-}" ]]; then
    echo "用法: $0 [--start]" >&2
    exit 2
fi

echo "[1/3] 拷贝 unit -> ${DEST}"
sudo cp "${SCRIPT_DIR}/${UNIT}" "${DEST}"
sudo chmod 644 "${DEST}"

echo "[2/3] daemon-reload + enable"
sudo systemctl daemon-reload
sudo systemctl enable raspbot.service

if [[ ${START} -eq 1 ]]; then
    echo "[3/3] systemctl start raspbot.service"
    sudo systemctl start raspbot.service
else
    echo "[3/3] 未启动（加 --start 立即启动）"
fi

echo "---- status ----"
systemctl status raspbot.service --no-pager || true
