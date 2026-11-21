#!/usr/bin/env bash
LC_ALL=C
echo "== 激活时间线 =="
grep -E 'Activating|Created BT|Error loading BT|not available|planner_server|lifecycle set|Configuring planner|Activating planner' /tmp/st_probe.log | tail -25 | sed 's/^/  /'
echo
echo "== planner 生命周期 =="
timeout -s KILL 8 ros2 lifecycle get /planner_server 2>&1 | sed 's/^/  /'
echo "== bt 生命周期 =="
timeout -s KILL 8 ros2 lifecycle get /bt_navigator 2>&1 | sed 's/^/  /'
echo "== action list =="
timeout -s KILL 8 ros2 action list 2>&1 | sed 's/^/  /'
