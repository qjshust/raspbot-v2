#!/usr/bin/env bash
# 准备 A/B 实验参数文件并分别跑两臂
LC_ALL=C
. /opt/ros/lyrical/setup.bash
. "$HOME/ros2_ws/install/setup.bash"
bash "$HOME/tools/selftest/killall.sh" >/dev/null 2>&1

# 官方参数对照文件：仅修帧名与 use_sim_time（本工程命名契约）
OFF=/tmp/nav2_official_ab.yaml
cp /opt/ros/lyrical/share/nav2_bringup/params/nav2_params.yaml "$OFF"
sed -i 's/use_sim_time: true/use_sim_time: false/' "$OFF"
echo "官方对照参数: $OFF ($(wc -l < "$OFF") 行)"

echo
echo "########## 臂 A：官方参数 ##########"
bash "$HOME/tools/selftest/08_ctl_ab.sh" "$OFF" "A_OFFICIAL"

echo
echo "########## 臂 B：本工程参数 ##########"
OURS="$HOME/ros2_ws/install/raspbot_bringup/share/raspbot_bringup/nav2/nav2_params.yaml"
bash "$HOME/tools/selftest/08_ctl_ab.sh" "$OURS" "B_OURS"
