"""Bring up the RASPBOT-V2 base driver.

Usage:
    ros2 launch raspbot_base base.launch.py
    ros2 launch raspbot_base base.launch.py params_file:=/path/to/params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspbot_base')
    default_params = os.path.join(share, 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML file with raspbot_base parameters')

    base = Node(
        package='raspbot_base',
        executable='base_node',
        name='raspbot_base',
        output='screen',
        parameters=[params_file],
        # CONTRACTS v1.2: /diagnostics 的唯一发布者是 WP-H 的聚合节点。
        # diagnostic_updater 默认发布 /diagnostics，这里重映射到专属话题，
        # H 的聚合器订阅所有 DiagnosticArray 话题并合并发布 /diagnostics。
        remappings=[('diagnostics', 'raspbot_base/diagnostics')],
        # SW-F03: if base_node dies unexpectedly (e.g. kill -9), exit the
        # whole launch so systemd Restart=always restarts the full bringup;
        # the fresh base_node then zeroes the wheels within 1 s.
        on_exit=Shutdown(),
    )

    return LaunchDescription([declare_params, base])
