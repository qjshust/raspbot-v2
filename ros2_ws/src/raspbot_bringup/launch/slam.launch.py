#!/usr/bin/env python3
# RASPBOT-V2 · WP-F · 建图 launch（slam_toolbox online_async）
#
# 契约要点（CONTRACTS v1.2）：
#   - map -> odom 由 F 发布：本 launch 的 slam_toolbox 是唯一发布者；
#   - odom -> base_link 由 E（EKF）发布，静态外参由 robot_state_publisher（E 维护 URDF）发布；
#   - 本文件【不启动任何 TF 发布器】：无 static_transform_publisher、无 robot_state_publisher。
#
# 已知坑（docs/SLAM环境验证与首次冒烟_2026-11-20.md）：
#   - launch 参数名必须是 slam_params_file（写 params_file 会被 online_async_launch.py 静默忽略）；
#   - slam_toolbox 是 lifecycle 节点，autostart 必须 true 才会 configure+activate；
#   - base_frame 必须 base_link（默认 base_footprint 会刷 "Failed to compute odom pose"）。
#
# 用法（最终系统，E/C 节点由 bringup 统一拉起后）：
#   ros2 launch raspbot_bringup slam.launch.py
#   ros2 launch raspbot_bringup slam.launch.py slam_params_file:=/abs/path/slam_params.yaml
# 保存地图：tools/slam/save_map.sh <地图名>
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    raspbot_bringup_share = get_package_share_directory('raspbot_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params_default = os.path.join(
        raspbot_bringup_share, 'slam', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_default,
            description='slam_toolbox 参数文件绝对路径'
                        '（注意参数名必须是 slam_params_file，不是 params_file）'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时钟（Gazebo）；真车必须为 false'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='lifecycle 自动 configure+activate'
                        '（必须 true，否则 /map 永远不发布）'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': LaunchConfiguration('autostart'),
            }.items(),
        ),
    ])
