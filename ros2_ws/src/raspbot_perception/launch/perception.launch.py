"""WP-I 感知：hailo_node（C++ 可执行）+ 参数加载。

用法：
    ros2 launch raspbot_perception perception.launch.py
    ros2 launch raspbot_perception perception.launch.py params_file:=/path/to/perception.yaml

（raspbot_bringup/launch/perception.launch.py 与本文件等价，二选一即可。）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('raspbot_perception')
    default_params = os.path.join(share, 'config', 'perception.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='YAML 文件（hailo_node 参数：hef_path/score_threshold/'
                    'max_rate/publish_annotated/device_timeout/fail_fast）')

    hailo_node = Node(
        package='raspbot_perception',
        executable='hailo_node',
        name='hailo_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([declare_params, hailo_node])
