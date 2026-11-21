#!/usr/bin/env bash
LC_ALL=C
echo "== MPPI/optimizer/critic 相关 debug 行 =="
grep -iE 'mppi|optimizer|critic|noise|control sequence|trajector|batch|velocity' /tmp/st_ctl.log | grep -viE 'Publishing velocity|rcl|Subscription' | tail -50 | sed 's/^/  /'
