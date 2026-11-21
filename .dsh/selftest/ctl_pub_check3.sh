#!/usr/bin/env bash
LC_ALL=C
echo "== 8842.35 之后 10 行 =="
sed -n '1181,1200p' /tmp/st_ctl.log | sed 's/^/  /'
