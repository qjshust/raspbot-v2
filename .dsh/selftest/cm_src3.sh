#!/usr/bin/env bash
LC_ALL=C
F=/opt/ros/lyrical/include/nav2_collision_monitor/nav2_collision_monitor/collision_monitor_node.hpp
sed -n '175,235p' "$F"
