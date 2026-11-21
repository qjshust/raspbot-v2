#!/usr/bin/env bash
LC_ALL=C
echo "== raspbot.service 状态 =="
systemctl status raspbot.service --no-pager 2>&1 | head -12 | sed 's/^/  /'
echo
echo "== journal 尾部 =="
sudo journalctl -u raspbot.service --no-pager 2>&1 | tail -20 | sed 's/^/  /'
echo
echo "== 当前进程 =="
pgrep -af 'base_node|bringup' | sed 's/^/  /'
