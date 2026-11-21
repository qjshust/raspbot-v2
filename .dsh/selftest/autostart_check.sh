#!/usr/bin/env bash
LC_ALL=C
grep -n 'autostart' ~/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/launch/bringup.launch.py
echo "== src 侧 =="
grep -n 'autostart' ~/ros2_ws/src/raspbot_bringup/launch/bringup.launch.py
