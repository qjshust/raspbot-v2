#!/usr/bin/env bash
LC_ALL=C
grep -iE 'Received a goal|FollowPath|computeControl|compute control|begin computing|progress|MPPIController|controller_server' /tmp/st_ctl.log | grep -viE 'rcl|Subscription' | tail -25 | sed 's/^/  /'
echo "== raw 话题消息数 =="
timeout -s KILL 5 ros2 topic hz /cmd_vel_nav_raw 2>&1 | grep -E 'average rate|no new' | sed 's/^/  /'
echo "== raw 值 =="
timeout -s KILL 5 ros2 topic echo /cmd_vel_nav_raw --once 2>&1 | grep -A3 linear | sed 's/^/  /'
