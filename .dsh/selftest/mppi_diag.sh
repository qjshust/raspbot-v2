#!/usr/bin/env bash
LC_ALL=C
grep -iE 'mppi|optimizer|controller_server' /tmp/st_probe.log | grep -vE 'Critic loaded|Critic instantiated|progress|abort|follow_path|lifecycle|bond|Configuring|Activating|initialized' | head -30 | sed 's/^/  /'
echo "== local costmap 更新情况 =="
grep -E 'local_costmap' /tmp/st_probe.log | tail -8 | sed 's/^/  /'
