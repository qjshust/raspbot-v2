#!/usr/bin/env bash
LC_ALL=C
echo "== cmd_vel_nav_raw 的 DDS 端点创建 =="
grep -nE 'cmd_vel_nav_raw' /tmp/st_ctl.log | sed 's/^/  /'
echo
echo "== rmw_fastrtps 错误/警告 =="
grep -iE 'rmw_fastrtps.*(error|fail|warn)|open_and_lock|init_port' /tmp/st_ctl.log | tail -10 | sed 's/^/  /'
echo
echo "== 控制器进程的 DDS 端口相关 =="
grep -E 'fastdds_port' /tmp/st_ctl.log | tail -8 | sed 's/^/  /'
