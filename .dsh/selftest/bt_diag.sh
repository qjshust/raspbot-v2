#!/usr/bin/env bash
LC_ALL=C
echo "== bt_navigator helper 相关符号 =="
strings /opt/ros/lyrical/lib/libnav2_bt_navigator*.so 2>/dev/null | grep -iE 'wait|timeout|action_server' | head -20
echo
echo "== bt_action_node 头文件 =="
find /opt/ros/lyrical/include -path '*nav2*' -name '*.hpp' | xargs grep -ln 'wait_for_action_server' 2>/dev/null | head -5
find /opt/ros/lyrical/include -name 'bt_action_node*.hpp' | head -3
echo
echo "== 1.5.2 bt_action_node 实现（等待窗口）=="
F=$(find /opt/ros/lyrical/include -name 'bt_action_node_impl.hpp' | head -1)
grep -n -B2 -A8 'wait_for_action_server\|seconds(' "$F" 2>/dev/null | head -30
echo
echo "== navigate_to_pose helper 节点参数 =="
strings /opt/ros/lyrical/lib/libnav2_bt_navigator.so 2>/dev/null | grep -E 'action_server|timeout|wait' | head -10
