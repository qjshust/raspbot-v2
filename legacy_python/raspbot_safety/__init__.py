"""raspbot_safety — 平台安全监控与 /diagnostics 聚合（FR-H-03）。

safety_node: 1 Hz 发布 /diagnostics（自身状态 + 聚合子诊断 + 安全告警）。
sysinfo / thresholds: 纯函数解析与阈值判断，离线可测（不依赖 ROS）。
"""
