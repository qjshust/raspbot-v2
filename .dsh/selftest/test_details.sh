#!/usr/bin/env bash
LC_ALL=C
. /opt/ros/lyrical/setup.bash
cd ~/ros2_ws
colcon test-result --verbose 2>&1 | grep -B2 -A12 'failure message' | head -80
