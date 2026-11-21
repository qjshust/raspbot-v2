#!/usr/bin/env bash
LC_ALL=C
echo "== st_ctl.log 尾部 =="
tail -40 /tmp/st_ctl.log | sed 's/^/  /'
echo
echo "== 进程 =="
pgrep -af 'controller_server|send_follow' | sed 's/^/  /'
