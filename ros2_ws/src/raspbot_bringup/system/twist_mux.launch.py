"""twist_mux 启动（WP-H）：cmd_vel_nav(10) + cmd_vel_teleop(50) -> /cmd_vel。

参数文件 system/twist_mux.yaml；结构与官方示例
/opt/ros/lyrical/share/twist_mux/config/twist_mux_topics.yaml 一致。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    yaml = os.path.join(
        get_package_share_directory('raspbot_bringup'),
        'system', 'twist_mux.yaml')

    cmd_vel_out = LaunchConfiguration('cmd_vel_out')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_out', default_value='/cmd_vel',
            description='twist_mux output topic (must stay /cmd_vel)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='use simulation time (bag replay)'),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            # twist_mux 自带 diagnostic_updater，默认直发 /diagnostics；
            # remap 到专属话题，由 H 的聚合器合并进唯一 /diagnostics（SW-C04）。
            remappings=[('/cmd_vel_out', cmd_vel_out),
                        ('diagnostics', 'twist_mux/diagnostics')],
            parameters=[
                {'use_sim_time': use_sim_time},
                yaml,
            ],
        ),
    ])
