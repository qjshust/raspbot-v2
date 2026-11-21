#!/usr/bin/env bash
LC_ALL=C
echo "== 控制器日志中 cmd/publisher 相关 =="
grep -iE 'cmd_vel|publisher|publish' /tmp/st_ctl.log | grep -viE 'Publishing velocity|discovery|rmw' | head -20 | sed 's/^/  /'
echo
echo "== Publishing velocity 行数 =="
grep -c 'Publishing velocity' /tmp/st_ctl.log | sed 's/^/  /'
echo
echo "== 发送器日志 =="
tail -6 /tmp/sender.log | sed 's/^/  /'
