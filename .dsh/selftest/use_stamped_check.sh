#!/usr/bin/env bash
LC_ALL=C
echo "== 部署的 yaml 中 use_stamped =="
grep -n 'use_stamped' ~/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/nav2/nav2_params.yaml | sed 's/^/  /'
echo
echo "== 控制器参数 dump（运行时）=="
. /opt/ros/lyrical/setup.bash
. ~/ros2_ws/install/setup.bash
timeout -s KILL 10 ros2 param get /controller_server use_stamped 2>&1 | sed 's/^/  /'
