#!/usr/bin/env bash
LC_ALL=C
echo "== st_mock_slam.log 关键行 =="
grep -iE 'mock|fake|error|process|not found|No such' /tmp/st_mock_slam.log 2>/dev/null | head -20 | sed 's/^/  /'
echo "== bringup 里 mock 的 include 条件 =="
grep -n 'mock\|use_mock\|MOCK' ~/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/launch/bringup.launch.py | head -12 | sed 's/^/  /'
