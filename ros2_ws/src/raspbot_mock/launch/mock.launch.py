"""Mock 三件套一起启动（SW-I04）：fake_base + fake_imu + fake_scan。

用法：
    ros2 launch raspbot_mock mock.launch.py
    ros2 launch raspbot_mock mock.launch.py params_file:=/path/to/mock.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspbot_mock')
    default_params = os.path.join(share, 'config', 'mock.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML file with mock node parameters')

    base = Node(
        package='raspbot_mock', executable='fake_base', name='fake_base',
        output='screen', parameters=[params_file])
    imu = Node(
        package='raspbot_mock', executable='fake_imu', name='fake_imu',
        output='screen', parameters=[params_file])
    scan = Node(
        package='raspbot_mock', executable='fake_scan', name='fake_scan',
        output='screen', parameters=[params_file])

    return LaunchDescription([declare_params, base, imu, scan])
