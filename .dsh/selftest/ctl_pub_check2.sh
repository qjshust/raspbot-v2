#!/usr/bin/env bash
LC_ALL=C
echo "== cmd_vel_nav_raw / cmd_vel 在控制器日志中的出现 =="
grep -nE 'cmd_vel' /tmp/st_ctl.log | head -10 | sed 's/^/  /'
echo
echo "== 8832-8845 时间窗内所有 publisher 初始化 =="
grep -E 'Initializing publisher' /tmp/st_ctl.log | sed 's/^/  /' | head -20
echo
echo "== 目标时段（8697-8710 附近）的 rcl 发布相关 =="
grep -E 'publish|Publisher' /tmp/st_ctl.log | grep -vE 'Publishing velocity' | head -10 | sed 's/^/  /'
