"""RASPBOT-V2 Pi 端 HTTP/SSE 网关（WP-C1）。

模块分层（导入期依赖约束）：

- ``config`` / ``protocol`` / ``mapio`` / ``state`` / ``session`` / ``controller`` /
  ``http_api``：**纯标准库**，不导入 rclpy，可在无 ROS 的机器上离线测试；
- ``ros_runtime``：唯一导入 ``rclpy`` / ``tf2_ros`` / ROS 消息包的模块；
- ``bridge``：进程入口，组装上面两层并管理线程与生命周期。

契约来源：``docs/PC上位机_设计文档_v1.md``（Wire API v1）与 ``ros2_ws/CONTRACTS.md``。
"""

__version__ = '1.0.0'

API_VERSION = 1

#: GET /api/health 返回的网关名
SERVER_NAME = 'raspbot-console'

#: 网关实现的 Wire API 版本（与设计文档 §3 一致）
__all__ = ['__version__', 'API_VERSION', 'SERVER_NAME']
