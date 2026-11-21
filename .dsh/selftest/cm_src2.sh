#!/usr/bin/env bash
LC_ALL=C
find /opt/ros/lyrical/include -path '*collision_monitor*' -name '*.hpp' | head -10
echo "== 节点源关键逻辑（转发 vs 事件发布）=="
F=$(find /opt/ros/lyrical/include -path '*collision_monitor*' -name 'collision_monitor_node.hpp' | head -1)
echo "F=$F"
grep -n -B2 -A10 'cmd_vel_out_pub_\|void process\|publishTwist\|out_action' "$F" 2>/dev/null | head -50
