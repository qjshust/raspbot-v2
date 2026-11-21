#!/usr/bin/env bash
LC_ALL=C
echo "== controller_server 二进制中的 use_stamped 相关 =="
strings /opt/ros/lyrical/lib/nav2_controller/controller_server 2>/dev/null | grep -iE 'use_stamped|stamped' | head -10
echo
echo "== 1.5.2 controller_server 头文件中的 cmd_vel_out 类型 =="
grep -rn 'TwistStamped\|use_stamped\|cmd_vel_out' /opt/ros/lyrical/include/nav2_controller/ 2>/dev/null | head -15
