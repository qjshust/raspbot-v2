#!/usr/bin/env bash
LC_ALL=C
F=/opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml
grep -n 'FollowPath:' "$F" | head -2
echo "== 官方 FollowPath 全文（critic 参数）=="
awk 'NR>=1 && /FollowPath:/{f=NR} f && NR>f && NR<f+80' "$F" | grep -E 'critic|scale|threshold|forward_point|sum_scores|slowing|lookahead|x_only|oscillation|RotateToGoal|Oscillation|BaseObstacle' | head -30
