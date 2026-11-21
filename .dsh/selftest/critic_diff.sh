#!/usr/bin/env bash
LC_ALL=C
F=/opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml
echo "== 官方 FollowPath 段（critics 及参数）=="
awk '/^  controller_server:/{c++} c==1 && /FollowPath:/{f=1} f{print} f && /^  [a-z_]+:$/ && !/FollowPath/{exit}' "$F" | grep -E 'critics:|scale|threshold|forward_point|sum_scores|slow|lookahead|enabled' | head -30
echo
echo "== 本工程 critics 段 =="
grep -E 'critics:|scale|threshold|forward_point|sum_scores|slow|lookahead|enabled' ~/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/nav2/nav2_params.yaml | head -30
