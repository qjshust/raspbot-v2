#!/usr/bin/env bash
LC_ALL=C
grep -B2 -A2 'not available' /tmp/st_probe.log | tail -20 | sed 's/^/  /'
echo "== 每次激活尝试的时间线 =="
grep -E 'Activating|Created BT|Error loading BT' /tmp/st_probe.log | tail -15 | sed 's/^/  /'
