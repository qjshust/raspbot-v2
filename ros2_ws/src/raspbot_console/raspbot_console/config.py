"""网关参数与默认值（纯标准库；可选 JSON 配置文件）。

优先级：**命令行 > JSON 配置 > 默认值**（与设计文档 §4.1 的 PC 端约定一致）。
JSON 配置只接受 :data:`DEFAULTS` 中出现的键，避免拼写错误静默生效。

运行入口参数见设计文档 §5.6::

    ros2 run raspbot_console console_bridge --port 8080
    ros2 run raspbot_console console_bridge --attach --port 8080
"""

from __future__ import annotations

import argparse
import json
import os

#: 全部默认参数（唯一权威；config/console.yaml 为同值的可读参考）
DEFAULTS = {
    # 网络
    'host': '0.0.0.0',
    'port': 8080,
    'token': '',
    'max_sse': 4,
    # 路径
    'workspace': '~/ros2_ws',
    'ros_setup': '/opt/ros/lyrical/setup.bash',
    'maps_dir': '/mnt/ssd/maps',
    'maps_fallback': '~/ros2_ws/maps',
    # 会话
    'attach': False,
    'session_manage': True,
    'session_timeout_s': 120.0,
    'stop_sigint_timeout_s': 20.0,
    'stop_sigterm_timeout_s': 5.0,
    'session_log_maxlen': 500,
    # 速度上限（§3.1 limits；与 base_node 默认参数一致）
    'max_linear': 0.30,
    'max_lateral': 0.30,
    'max_angular': 1.5,
    # 控制时序
    'teleop_rate_hz': 20.0,
    'teleop_timeout_s': 0.5,
    'estop_resend_s': 1.0,
    'initialpose_rate_hz': 2.0,
    'initialpose_timeout_s': 20.0,
    # 推送 / 数据面
    'snapshot_rate_hz': 10.0,
    'scan_rate_hz': 5.0,
    'path_rate_hz': 2.0,
    'scan_max_points': 360,
    'path_max_points': 200,
    'map_png_cache_size': 4,
    'gateway_log_maxlen': 300,
    'max_sse_queue': 200,
    # 日志
    'log_level': 'info',
}

#: 允许缩短的日志级别
LOG_LEVELS = ('debug', 'info', 'warning', 'error')


class Config:
    """网关配置对象；属性与 :data:`DEFAULTS` 的键一一对应。"""

    def __init__(self, **overrides):
        for key, value in DEFAULTS.items():
            setattr(self, key, value)
        for key, value in overrides.items():
            if key not in DEFAULTS:
                raise TypeError('未知配置项: %r' % (key,))
            setattr(self, key, value)
        self.validate()

    # -- 派生属性 -------------------------------------------------------
    @property
    def maps_dirs(self):
        """地图扫描目录（优先级：``maps_dir`` → ``maps_fallback``）。"""
        return [self.maps_dir, self.maps_fallback]

    @property
    def limits(self):
        return {
            'max_linear': float(self.max_linear),
            'max_lateral': float(self.max_lateral),
            'max_angular': float(self.max_angular),
        }

    @property
    def managed(self):
        return bool(self.session_manage) and not bool(self.attach)

    def expanded(self, path):
        return os.path.abspath(os.path.expanduser(str(path)))

    def to_dict(self):
        return {key: getattr(self, key) for key in DEFAULTS}

    # -- 校验 -----------------------------------------------------------
    def validate(self):
        if not isinstance(self.host, str) or not self.host:
            raise ValueError('host 必须是非空字符串')
        try:
            port = int(self.port)
        except (TypeError, ValueError):
            raise ValueError('port 必须是整数')
        if not (0 <= port <= 65535):
            raise ValueError('port 必须在 0-65535 之间（0 = 随机端口，测试用）')
        self.port = port
        if not isinstance(self.token, str):
            raise ValueError('token 必须是字符串')
        try:
            max_sse = int(self.max_sse)
        except (TypeError, ValueError):
            raise ValueError('max-sse 必须是整数')
        if not (1 <= max_sse <= 32):
            raise ValueError('max-sse 必须在 1-32 之间')
        self.max_sse = max_sse
        for name in ('max_linear', 'max_lateral', 'max_angular'):
            value = float(getattr(self, name))
            if not (value > 0.0):
                raise ValueError('%s 必须为正数' % name)
            setattr(self, name, value)
        for name in ('session_timeout_s', 'stop_sigint_timeout_s',
                     'stop_sigterm_timeout_s', 'teleop_rate_hz',
                     'teleop_timeout_s', 'estop_resend_s',
                     'initialpose_rate_hz', 'initialpose_timeout_s',
                     'snapshot_rate_hz', 'scan_rate_hz', 'path_rate_hz'):
            value = float(getattr(self, name))
            if not (value > 0.0):
                raise ValueError('%s 必须为正数' % name)
            setattr(self, name, value)
        for name in ('scan_max_points', 'path_max_points',
                     'map_png_cache_size', 'gateway_log_maxlen',
                     'max_sse_queue', 'session_log_maxlen'):
            value = int(getattr(self, name))
            if value <= 0:
                raise ValueError('%s 必须为正整数' % name)
            setattr(self, name, value)
        if self.log_level not in LOG_LEVELS:
            raise ValueError('log-level 必须是 %s 之一' % '/'.join(LOG_LEVELS))
        self.attach = bool(self.attach)
        self.session_manage = bool(self.session_manage)
        return self


def load_config_file(path):
    """读取 JSON 配置文件（顶层必须是对象）；返回 dict。"""
    with open(path, 'r', encoding='utf-8') as fh:
        try:
            data = json.load(fh)
        except ValueError as exc:
            raise ValueError('配置文件不是合法 JSON: %s' % exc)
    if not isinstance(data, dict):
        raise ValueError('配置文件顶层必须是 JSON 对象')
    unknown = [key for key in data if key not in DEFAULTS]
    if unknown:
        raise ValueError('配置文件包含未知参数: %s' % ', '.join(sorted(unknown)))
    return data


def build_argument_parser():
    """构造 argparse 解析器（``ros2 run raspbot_console console_bridge --help``）。"""
    parser = argparse.ArgumentParser(
        prog='console_bridge',
        description='RASPBOT-V2 Pi 端 HTTP/SSE 网关（Wire API v1）',
    )
    parser.add_argument('--host', default=None, help='监听地址（默认 0.0.0.0）')
    parser.add_argument('--port', type=int, default=None,
                        help='监听端口（默认 8080）')
    parser.add_argument('--token', default=None,
                        help='可选访问 token（X-Console-Token / ?token=）')
    parser.add_argument('--max-sse', type=int, default=None,
                        help='SSE 客户端上限（默认 4）')
    parser.add_argument('--workspace', default=None,
                        help='ROS 工作区（默认 ~/ros2_ws）')
    parser.add_argument('--ros-setup', default=None,
                        help='默认 ROS setup 脚本（默认 /opt/ros/lyrical/setup.bash）')
    parser.add_argument('--maps-dir', default=None,
                        help='地图主目录（默认 /mnt/ssd/maps）')
    parser.add_argument('--maps-fallback', default=None,
                        help='地图回退目录（默认 ~/ros2_ws/maps）')
    parser.add_argument('--attach', action='store_true', default=None,
                        help='附加到外部已启动的 ROS 栈（托管=false，start/stop 409）')
    parser.add_argument('--no-session-manage', dest='session_manage',
                        action='store_false', default=None,
                        help='不管理 bringup 会话（等价于 --attach 的只读模式）')
    parser.add_argument('--max-linear', type=float, default=None,
                        help='遥控线速度上限 m/s（默认 0.30）')
    parser.add_argument('--max-lateral', type=float, default=None,
                        help='遥控横移速度上限 m/s（默认 0.30）')
    parser.add_argument('--max-angular', type=float, default=None,
                        help='遥控角速度上限 rad/s（默认 1.5）')
    parser.add_argument('--session-timeout', type=float, default=None,
                        dest='session_timeout_s',
                        help='会话启动就绪超时 s（默认 120）')
    parser.add_argument('--log-level', default=None,
                        choices=LOG_LEVELS, help='日志级别（默认 info）')
    parser.add_argument('--config', default=None,
                        help='可选 JSON 配置文件（键名同本文件默认值）')
    parser.add_argument('--version', action='version',
                        version='raspbot_console 1.0.0 (Wire API v1)')
    return parser


def parse_argv(argv=None):
    """解析命令行 + JSON 配置，返回 :class:`Config`（CLI 优先）。

    使用 ``parse_known_args``：``ros2 launch``/``ros2 run`` 会追加
    ``--ros-args ...`` 等 ROS 专用参数，网关忽略它们（本节点不用 ROS 参数/
    remap；参数一律走本文件定义的命令行开关）。
    """
    parser = build_argument_parser()
    args, _unknown = parser.parse_known_args(argv)
    overrides = {}
    if args.config:
        overrides.update(load_config_file(args.config))
    cli = {key: value for key, value in vars(args).items()
           if key != 'config' and value is not None}
    overrides.update(cli)
    return Config(**overrides)


#: 默认配置实例（只读用途；测试/桥接应各自构造）
DEFAULT = Config()
