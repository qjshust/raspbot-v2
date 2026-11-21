#!/usr/bin/env bash
LC_ALL=C
F=/opt/ros/lyrical/include/nav2_collision_monitor/collision_monitor_node.hpp
grep -n -B3 -A12 'cmd_vel_out_\|publish\|process(' "$F" | head -60
