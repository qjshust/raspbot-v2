#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
cd ~/ros2_ws
colcon test-result --verbose 2>&1 | grep -B3 -A8 'failure message' | head -60
