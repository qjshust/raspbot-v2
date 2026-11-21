#!/usr/bin/env bash
LC_ALL=C
F=/opt/ros/lyrical/include/nav2_behavior_tree/nav2_behavior_tree/bt_action_node.hpp
grep -n -B3 -A12 'wait_for_action_server\|std::chrono' "$F" | head -50
echo "== 库文件 =="
ls -l /opt/ros/lyrical/lib/libnav2_bt_navigator* 2>/dev/null
ls /opt/ros/lyrical/lib/libnav2_behavior_tree* 2>/dev/null
