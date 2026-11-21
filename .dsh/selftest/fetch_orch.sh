#!/usr/bin/env bash
LC_ALL=C
echo "== orch.log 是否存在 =="
ls -l /tmp/orch.log 2>/dev/null
echo
echo "== orch.log 内容 =="
cat /tmp/orch.log 2>/dev/null | tail -60
echo
echo "== 残留进程 =="
pgrep -af 'sil_orch|bringup|orch' | head -5
