#!/usr/bin/env python3
# RASPBOT-V2 · WP-F · 定位 launch（nav2 map_server + AMCL）
#
# 契约要点（CONTRACTS v1.2）：
#   - map -> odom 由 F 发布：本 launch 的 amcl 是唯一发布者；
#   - odom -> base_link 由 E（EKF）发布，静态外参由 robot_state_publisher（E 维护 URDF）发布；
#   - 本文件【不启动任何 TF 发布器】（amcl 本身广播 map->odom 是 F 的契约职责）。
#
# 实现：Include nav2_bringup 的 localization_launch.py，并补一个 lifecycle_manager：
#   - nav2 1.5.2 的 localization_launch.py 只起 map_server + amcl 两个 lifecycle 节点，
#     自身【不含】 lifecycle manager（文件里 get_lifecycle_nodes 定义了却未被使用）；
#   - nav2_amcl/nav2_map_server 1.5.2 也不支持 autostart 参数（amcl_node.cpp 源码无此参数，
#     Pi 上实测：无论参数文件里是否带 autostart: true，两节点都停在 unconfigured [1]）；
#   - 因此本文件显式追加 nav2_lifecycle_manager（与 Jazzy 时代官方 localization_launch.py
#     的写法一致，Pi 上实测：map_server/amcl 均转为 active [3]，/map、/amcl_pose 正常）；
#     manager 经 TimerAction 延迟 5 s 拉起并放宽 bond_timeout，规避启动竞态（详见文件内注释）。
#   - map_server 以 Transient Local 发布 /map；amcl 订阅 /scan、/map 与 odom->base_link TF。
#
# 用法（最终系统）：
#   ros2 launch raspbot_bringup localization.launch.py
#     map:=/abs/path/room.yaml   # 默认 /home/ubuntu/ros2_ws/maps/room.yaml
#     use_sim_time:=false
#
# 地图文件不存在时本文件会在节点启动前直接报错退出，避免 map_server 半死不活。
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_MAP = '/home/ubuntu/ros2_ws/maps/room.yaml'


def _check_map_file_exists(context):
    """map 文件不存在的清晰报错（在 Include 之前执行）。"""
    map_path = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(map_path):
        raise RuntimeError(
            '[localization.launch.py] 地图文件不存在："{}"\n'
            '  请先用 WP-F 建图流程生成地图并用 tools/slam/save_map.sh <地图名> 保存，\n'
            '  或通过 map:= 传入正确的地图 yaml 绝对路径'
            '（默认 {}）。'.format(map_path, _DEFAULT_MAP))
    return []


def generate_launch_description():
    raspbot_bringup_share = get_package_share_directory('raspbot_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    amcl_params = os.path.join(raspbot_bringup_share, 'slam', 'amcl.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=_DEFAULT_MAP,
            description='地图 yaml 文件绝对路径（nav2_map_server 加载）'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时钟（Gazebo）；真车必须为 false'),
        OpaqueFunction(function=_check_map_file_exists),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': amcl_params,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        # nav2 1.5.2 的 localization_launch.py 不带 lifecycle manager，
        # 且 amcl/map_server 1.5.2 不支持 autostart 参数（Pi 实测停在 unconfigured），
        # 这里补一个 manager 自动 configure+activate 两个节点。
        # 启动竞态加固（lead 实测复现过 amcl 停在 unconfigured 的情况）：
        #   - TimerAction(period=5.0)：延迟 5 s 再拉起 manager，等 map_server/amcl 的
        #     lifecycle 服务确定就绪后再 autostart，避免 manager 早到、服务调用落空；
        #   - bond_timeout: 10.0：bond 判断超时从默认 4 s 放宽，避免 amcl configure
        #     等待 /map 期间被误判为节点失败；
        #   - attempt_respawn_reconnection: True：manager 与节点断连后重试恢复。
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'autostart': True},
                        {'bond_timeout': 10.0},
                        {'service_timeout': 10.0},
                        {'attempt_respawn_reconnection': True},
                        {'node_names': ['map_server', 'amcl']},
                    ],
                ),
            ],
        ),
    ])
