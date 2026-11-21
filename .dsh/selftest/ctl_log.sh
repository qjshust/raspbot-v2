#!/usr/bin/env bash
LC_ALL=C
echo "== controller 日志（目标运行时段）=="
grep -E 'controller_server|MPPI|mppi' /tmp/st_probe.log | grep -viE 'Critic loaded|Critic instantiated|Creating|enabled at startup|lifecycle|bond|Configuring|Activating|Deactivating|initialized|frequency set|getting|Created|Loaded|progress checker' | tail -25 | sed 's/^/  /'
