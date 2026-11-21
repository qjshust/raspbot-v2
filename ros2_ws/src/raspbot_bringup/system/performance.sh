#!/usr/bin/env bash
# performance.sh — 把所有在线 CPU 的 governor 设为 performance（D-08）。
#
# 背景：当前 Pi 5 默认为 ondemand，控制周期抖动偏大（测试方案 §4.4 D-08）。
# SW-P03 等性能基线必须在 performance 条件下记录。
#
# 用法（幂等，可重复执行）：
#   sudo bash system/performance.sh
# 注意：governor 在重启后复位。需要持久化可追加到 /etc/rc.local，
# 或在 raspbot.service 里加 ExecStartPre（平台暂不默认启用）。
set -euo pipefail
shopt -s nullglob

if [[ ${EUID} -ne 0 ]]; then
    echo "[info] 需要 root，以 sudo 重新执行…" >&2
    exec sudo bash "$0" "$@"
fi

files=(/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)
if ((${#files[@]} == 0)); then
    echo "[warn] 未发现 cpufreq scaling_governor（无 cpufreq 子系统），跳过。"
    exit 0
fi

fails=0
for f in "${files[@]}"; do
    if printf 'performance\n' >"${f}" 2>/dev/null; then
        echo "ok   ${f} -> $(cat "${f}")"
    else
        echo "fail ${f}"
        fails=1
    fi
done

if ((fails == 0)); then
    echo "done: 全部 governor=performance（重启后需重新执行或持久化）"
fi
exit ${fails}
