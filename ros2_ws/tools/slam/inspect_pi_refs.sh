#!/usr/bin/env bash
# RASPBOT-V2 · WP-F 只读检查脚本（不 build、不启动任何硬件节点）
# 用途：从已安装的 ROS 2 Lyrical 包中读取参考结构，为 amcl.yaml / launch 提供证据。
# 注意：不要在这里开 set -u（ROS setup.bash 会引用未定义变量）。
LC_ALL=C

echo "===== [1] slam_toolbox mapper_params_online_async.yaml（头部 40 行） ====="
head -40 /opt/ros/lyrical/share/slam_toolbox/config/mapper_params_online_async.yaml
echo
echo "===== [2] nav2_params.yaml 顶层键（确认 amcl 段键名） ====="
grep -n '^[A-Za-z0-9_/]\+:' /opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml
echo
echo "===== [3] nav2_params.yaml amcl 段全文 ====="
sed -n '/^amcl:/,/^[^ ]/p' /opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml
echo
echo "===== [4] nav2_params.yaml amcl_map_server 段（若存在） ====="
grep -n -A5 'map_server\|amcl' /opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml | head -30
echo
echo "===== [5] nav2_bringup localization_launch.py 全文 ====="
cat /opt/ros/lyrical/share/nav2_bringup/launch/localization_launch.py
echo
echo "===== [6] localization_launch.py 声明的 launch 参数 ====="
. /opt/ros/lyrical/setup.bash
timeout -s KILL 20 ros2 launch nav2_bringup localization_launch.py --show-args 2>&1 | head -30
echo
echo "===== [7] map_saver_cli 帮助（确认 -f / save_map_timeout 等参数名） ====="
timeout -s KILL 15 ros2 run nav2_map_server map_saver_cli --help 2>&1 | head -50
echo
echo "===== [8] amcl 组件包版本与节点名 ====="
timeout -s KILL 10 ros2 pkg prefix nav2_amcl 2>&1
timeout -s KILL 10 ros2 component list 2>/dev/null | grep -i amcl
dpkg -l | grep -E 'ros-lyrical-nav2-(amcl|map-server)' 2>/dev/null
echo
echo "===== 完成 ====="
