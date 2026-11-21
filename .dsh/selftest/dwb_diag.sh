#!/usr/bin/env bash
LC_ALL=C
grep -E 'controller_server|dwb|DWB|critic|Critic' /tmp/st_probe.log | grep -vE 'Critic plugin initialized|Using critic' | tail -30 | sed 's/^/  /'
echo "== 官方默认 critics 参数（对照）=="
grep -A60 'FollowPath:' /opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml | grep -E 'scale|critics:|forward_point|sim_time' | head -25
