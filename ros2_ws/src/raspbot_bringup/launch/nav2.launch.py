#!/usr/bin/env python3
# ==============================================================================
# RASPBOT-V2 Nav2 启动文件（WP-G）—— raspbot_bringup/launch/nav2.launch.py
#
# 依据：CONTRACTS v1.2（WP-G 独占本文件 + nav2/）、测试方案 v1.0
# 基线：Nav2 1.5.2-2resolute（/opt/ros/lyrical/share/nav2_bringup）
#
# 架构说明（1.5.2 实测结论，与任务书假设的差异见 README 与回报）：
#   1) navigation_launch.py(1.5.2) 硬编码了 remappings：
#        controller_server / behavior_server: ('cmd_vel' -> 'cmd_vel_nav')
#        velocity_smoother:            ('cmd_vel' -> 'cmd_vel_nav')
#      且没有 remappings LaunchConfiguration 透传参数；collision_monitor 节点
#      也由 navigation_launch.py 直接启动（不要再单独 include
#      collision_monitor_node.launch.py，会重名冲突）。
#   2) 因此本文件用 GroupAction + SetRemap（launch_ros 提供的组级 remap，
#      会先于节点自身 remappings 生效，rcl 按顺序匹配第一条命中规则）把
#      Nav2 内部 'cmd_vel' 改投到 /cmd_vel_nav_raw，再让 collision_monitor
#      订阅 cmd_vel_nav_raw、输出契约话题 /cmd_vel_nav：
#        controller/behavior_server --cmd_vel_nav_raw--> collision_monitor
#                                                       --cmd_vel_nav--> twist_mux(H)
#      /cmd_vel_nav 单发布者 = collision_monitor（契约 SW-C04 要求）。
#   3) 生命周期：navigation_launch.py(1.5.2) 本身不带 lifecycle_manager
#      （只有 bringup_launch.py 会经 OpaqueFunction 启动），因此本文件自行
#      启动 lifecycle_manager，只管理实际配置的 5 个节点。
#      代价地图是 controller/planner 进程内的子生命周期节点，无需列入。
#   4) /diagnostics 契约（H 单发布者）：Nav2 各节点经 diagnostic_updater
#      库以绝对话题 /diagnostics 直发（libdiagnostic_updater.so 中字符串
#      实测为 "/diagnostics"），统一 remap ('/diagnostics' -> '/nav2/diagnostics')，
#      由 H 的 safety_node 动态发现聚合。
#   5) SetRemap 写入 launch 上下文的 ros_remaps 会泄漏到同一 launch 上下文中
#      之后启动的所有节点（可能劫持 twist_mux 的 cmd_vel 输出！），因此
#      include 之后立刻用 OpaqueFunction 精确清除本文件添加的两条规则。
#   6) map_server / amcl 不在本文件（F 的 localization.launch.py 管理）；
#      /map、/tf、/odom、/scan 全部为外部契约话题。
#   7) rviz 可选，不在此处启动。
# ==============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


# 本文件经 SetRemap 注入组作用域的两条 remap（清除时按此精确比对）
_NAV2_GROUP_REMAPS = [
    ('cmd_vel', 'cmd_vel_nav_raw'),
    ('/diagnostics', '/nav2/diagnostics'),
]


def _remove_group_remaps(context, *args, **kwargs):
    """导航 include 结束后，从 launch 上下文清除本文件注入的 remap 规则，
    防止污染同一 launch 上下文中后续启动的节点（如 H 的 twist_mux）。"""
    remaps = context.launch_configurations.get('ros_remaps')
    if remaps is not None:
        context.launch_configurations['ros_remaps'] = [
            (src, dst) for (src, dst) in remaps
            if (src, dst) not in _NAV2_GROUP_REMAPS
        ]
    return []


def generate_launch_description() -> LaunchDescription:
    raspbot_bringup_dir = get_package_share_directory('raspbot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # 实车：false（NFR/契约要求）
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            raspbot_bringup_dir, 'nav2', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file (WP-G)',
    )

    # --------------------------------------------------------------------------
    # Nav2 主体节点：官方 navigation_launch.py（1.5.2）
    #   - params_file / use_sim_time / autostart 传入；map 相关参数不传（F 管）。
    #   - 组内 SetRemap 把 Nav2 内部 cmd_vel 与 /diagnostics 重定向（见文件头说明）。
    # --------------------------------------------------------------------------
    nav2_group = GroupAction(actions=[
        SetRemap(src='/diagnostics', dst='/nav2/diagnostics'),
        SetRemap(src='cmd_vel', dst='cmd_vel_nav_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'use_composition': 'False',
            }.items(),
        ),
        # 清除本组注入的 remap 规则（防泄漏到后续节点，如 twist_mux）
        OpaqueFunction(function=_remove_group_remaps),
    ])

    # --------------------------------------------------------------------------
    # 生命周期管理器（1.5.2 navigation_launch.py 不含，需自行启动）。
    # 只管理实际配置的节点；velocity_smoother/smoother_server/route_server/
    # waypoint_follower/docking/following 由 include 启动但保持 unconfigured
    # （无参数段、无话题，空转成本极低）。
    # 注意顺序：必须在 nav2_group 之后启动（节点先于管理器存在）。
    # --------------------------------------------------------------------------
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_nav2',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {'autostart': autostart},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'collision_monitor',
            ]},
            {'use_sim_time': use_sim_time},
            # 高负载下 bond/service 调用易超时（实测复现），放宽并允许重连
            {'bond_timeout': 10.0},
            {'service_timeout': 10.0},
            {'attempt_respawn_reconnection': True},
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/diagnostics', '/nav2/diagnostics'),  # 契约：/diagnostics 属 H
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(nav2_group)
    ld.add_action(lifecycle_manager_cmd)
    return ld
