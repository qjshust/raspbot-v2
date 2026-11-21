#!/usr/bin/env bash
LC_ALL=C
ls /opt/ros/lyrical/lib/ | grep -iE 'bt_navigator|behavior_tree' | head -20
echo "== 逐个 strings 找参数 =="
for f in /opt/ros/lyrical/lib/*bt_navigator*.so; do
  echo "-- $f"
  strings "$f" 2>/dev/null | grep -iE 'wait_for|service_timeout|declare_parameter' | head -8
done
