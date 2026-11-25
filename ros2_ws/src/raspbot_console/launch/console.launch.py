#!/usr/bin/env python3
"""raspbot_console 网关 launch（可选，等价于手工 ``ros2 run``）。

用法::

    ros2 launch raspbot_console console.launch.py
    ros2 launch raspbot_console console.launch.py port:=8080 token:=mytoken
    ros2 launch raspbot_console console.launch.py attach:=true
    ros2 launch raspbot_console console.launch.py maps_dir:=/mnt/ssd/maps

参数默认值读自 ``config/console.yaml``（与 ``raspbot_console/config.py`` 的
``DEFAULTS`` 一致）；也可以用 launch 参数逐项覆盖。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 与 config.py DEFAULTS 一致的兜底默认值（console.yaml 缺失时使用）
FALLBACK_DEFAULTS = {
    'host': '0.0.0.0',
    'port': 8080,
    'token': '',
    'max_sse': 4,
    'workspace': '~/ros2_ws',
    'ros_setup': '/opt/ros/lyrical/setup.bash',
    'maps_dir': '/mnt/ssd/maps',
    'maps_fallback': '~/ros2_ws/maps',
    'attach': False,
    'session_manage': True,
    'max_linear': 0.30,
    'max_lateral': 0.30,
    'max_angular': 1.5,
    'log_level': 'info',
}


def _load_defaults():
    """优先读 share/config/console.yaml 的 console_bridge.ros__parameters。"""
    defaults = dict(FALLBACK_DEFAULTS)
    try:
        import yaml  # ROS 环境自带 PyYAML
        share = get_package_share_directory('raspbot_console')
        path = os.path.join(share, 'config', 'console.yaml')
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        section = (data.get('console_bridge') or {}).get('ros__parameters') \
            or {}
        for key, value in section.items():
            if key in defaults:
                defaults[key] = value
    except Exception:
        pass
    return defaults


def _to_str(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def _launch_setup(context):
    """解析 launch 参数 → console_bridge 命令行参数列表。"""
    values = {}
    for key in ('host', 'port', 'token', 'max_sse', 'workspace', 'ros_setup',
                'maps_dir', 'maps_fallback', 'attach', 'session_manage',
                'max_linear', 'max_lateral', 'max_angular', 'log_level'):
        values[key] = LaunchConfiguration(key).perform(context)

    arguments = [
        '--host', values['host'],
        '--port', str(int(values['port'])),
        '--max-sse', str(int(values['max_sse'])),
        '--workspace', values['workspace'],
        '--ros-setup', values['ros_setup'],
        '--maps-dir', values['maps_dir'],
        '--maps-fallback', values['maps_fallback'],
        '--max-linear', str(float(values['max_linear'])),
        '--max-lateral', str(float(values['max_lateral'])),
        '--max-angular', str(float(values['max_angular'])),
        '--log-level', values['log_level'],
    ]
    token = str(values['token'])
    if token:
        arguments.extend(['--token', token])
    attach = _to_str(values['attach']).strip().lower()
    if attach in ('true', '1', 'yes'):
        arguments.append('--attach')
    manage = _to_str(values['session_manage']).strip().lower()
    if manage in ('false', '0', 'no'):
        arguments.append('--no-session-manage')

    return [
        Node(
            package='raspbot_console',
            executable='console_bridge',
            name='raspbot_console',
            output='screen',
            emulate_tty=True,
            arguments=arguments,
        ),
    ]


def generate_launch_description():
    defaults = _load_defaults()
    declared = []
    for key in ('host', 'port', 'token', 'max_sse', 'workspace', 'ros_setup',
                'maps_dir', 'maps_fallback', 'max_linear', 'max_lateral',
                'max_angular', 'log_level'):
        declared.append(DeclareLaunchArgument(
            key, default_value=_to_str(defaults[key]),
            description='raspbot_console 参数 %s' % key))
    declared.append(DeclareLaunchArgument(
        'attach', default_value=_to_str(defaults['attach']),
        description='附加到外部 ROS 栈（true/false）'))
    declared.append(DeclareLaunchArgument(
        'session_manage', default_value=_to_str(defaults['session_manage']),
        description='是否管理 bringup 会话（true/false）'))
    return LaunchDescription(declared + [OpaqueFunction(
        function=_launch_setup)])
