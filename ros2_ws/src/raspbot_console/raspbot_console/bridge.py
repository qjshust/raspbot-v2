"""进程入口：参数解析、rclpy 初始化、HTTP/会话监控线程、优雅退出。

启动方式（设计文档 §5.6）::

    ros2 run raspbot_console console_bridge --port 8080
    python -m raspbot_console --port 8080
    ros2 run raspbot_console console_bridge --attach --port 8080

退出路径（SIGINT/SIGTERM）：停止遥控（零速）→ 停止托管会话（SIGINT →
等待 → SIGKILL 进程组）→ 关闭事件总线/HTTP → 停 ROS 执行器；总耗时 < 30 s。
"""

from __future__ import annotations

import logging
import signal
import sys
import threading
import time

from . import __version__
from .config import parse_argv
from .controller import Controller
from .http_api import HttpApi
from .session import SessionManager
from .state import StateStore

LOGGER = logging.getLogger('raspbot_console.bridge')

#: 监控线程每秒构建快照的次数上限（10 Hz 契约）
_SESSION_TICK_INTERVAL_S = 0.5


def _configure_logging(level_name):
    level = getattr(logging, str(level_name).upper(), logging.INFO)
    logging.basicConfig(
        level=level,
        format='[%(asctime)s] %(levelname)s %(name)s: %(message)s',
    )


def _install_signal_handlers(stop_event):
    """SIGINT/SIGTERM → 设置退出事件（rclpy 默认 SIGINT 处理被覆盖）。"""
    def handler(signum, _frame):
        LOGGER.info('收到信号 %s，开始优雅退出…', signum)
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):  # 非主线程/平台不支持时忽略
            pass


def _monitor_loop(controller, stop_event, snapshot_period_s, io_period_s):
    """10 Hz 推送 state 事件 + 0.5 s 推进会话状态机。"""
    session_counter = 0
    while not stop_event.is_set():
        started = time.monotonic()
        try:
            controller.publish_state_snapshot()
        except Exception:  # pragma: no cover - 监控线程兜底
            LOGGER.exception('构建状态快照失败')
        if session_counter % io_period_s == 0:
            try:
                controller.tick_session()
            except Exception:  # pragma: no cover
                LOGGER.exception('会话状态机 tick 失败')
        session_counter += 1
        elapsed = time.monotonic() - started
        stop_event.wait(max(0.0, snapshot_period_s - elapsed))


def run(config):
    """按配置启动网关；阻塞直到收到退出信号；返回进程退出码。"""
    _configure_logging(config.log_level)
    LOGGER.info('raspbot_console %s 启动中（Wire API v1）', __version__)

    state = StateStore(
        config.limits,
        max_logs=config.gateway_log_maxlen,
        max_sse=config.max_sse,
        log_recent=20,
    )
    session = SessionManager(
        workspace=config.workspace,
        ros_setup=config.ros_setup,
        timeout_s=config.session_timeout_s,
        stop_sigint_timeout_s=config.stop_sigint_timeout_s,
        stop_sigterm_timeout_s=config.stop_sigterm_timeout_s,
        session_log_maxlen=config.session_log_maxlen,
        attach=config.attach,
        manage=config.session_manage,
    )
    controller = Controller(config, state, session)

    # -- ROS 运行时（唯一 rclpy 依赖） ---------------------------------
    rclpy = None
    runtime = None
    try:
        import rclpy as _rclpy  # noqa: F401 - 延迟导入，便于 --help 离线可用
        rclpy = _rclpy
        from .ros_runtime import RosRuntime
        # 不把网关自己的命令行参数交给 ROS（避免 --port 等被 rcl_init 误解析）
        rclpy.init(args=[])
        runtime = RosRuntime(config, state, controller)
        controller.runtime = runtime
        runtime.start()
    except Exception as exc:  # ImportError / rclpy 初始化失败
        LOGGER.error('ROS 运行时初始化失败（网关需要 ROS 2 环境）：%r', exc)
        if rclpy is not None:
            try:
                rclpy.shutdown()
            except Exception:  # pragma: no cover
                pass
        return 2

    # -- HTTP 服务 -----------------------------------------------------
    http_api = HttpApi(controller, config)
    try:
        http_api.start()
    except OSError as exc:
        LOGGER.error('HTTP 监听失败 %s:%s：%s',
                     config.host, config.port, exc)
        controller.shutdown()
        runtime.stop()
        try:
            rclpy.shutdown()
        except Exception:  # pragma: no cover
            pass
        return 3

    host, port = http_api.address or (config.host, config.port)
    controller.log('info', '网关已启动：http://%s:%d/（api=v1，版本 %s）'
                   % (host, port, __version__))
    LOGGER.info('HTTP/SSE 网关监听 http://%s:%d/', host, port)

    stop_event = threading.Event()
    _install_signal_handlers(stop_event)

    snapshot_period_s = 1.0 / max(1.0, float(config.snapshot_rate_hz))
    io_period = max(1, int(round(
        _SESSION_TICK_INTERVAL_S / max(snapshot_period_s, 1e-3))))
    monitor = threading.Thread(
        target=_monitor_loop,
        args=(controller, stop_event, snapshot_period_s, io_period),
        name='console-monitor', daemon=True)
    monitor.start()

    try:
        while not stop_event.is_set():
            stop_event.wait(0.2)
    except KeyboardInterrupt:  # pragma: no cover - 信号处理器通常先触发
        stop_event.set()

    # -- 优雅退出 ------------------------------------------------------
    LOGGER.info('开始关闭：停止遥控/会话/HTTP…')
    try:
        monitor.join(timeout=2.0)
    except Exception:  # pragma: no cover
        pass
    try:
        controller.shutdown()
    except Exception:  # pragma: no cover
        LOGGER.exception('controller 退出收尾失败')
    try:
        session.shutdown(timeout_s=30.0)
    except Exception:  # pragma: no cover
        LOGGER.exception('会话停止失败')
    try:
        state.event_bus.close()
    except Exception:  # pragma: no cover
        pass
    try:
        http_api.stop()
    except Exception:  # pragma: no cover
        LOGGER.exception('HTTP 关闭失败')
    try:
        if runtime is not None:
            runtime.stop()
    except Exception:  # pragma: no cover
        LOGGER.exception('ROS 运行时关闭失败')
    try:
        if rclpy is not None:
            rclpy.shutdown()
    except Exception:  # pragma: no cover
        pass
    LOGGER.info('raspbot_console 已退出')
    return 0


def main(argv=None):
    """``console_bridge`` 入口（setup.py console_scripts）。"""
    try:
        config = parse_argv(argv)
    except ValueError as exc:
        print('参数错误：%s' % exc, file=sys.stderr)
        return 2
    try:
        return run(config)
    except KeyboardInterrupt:  # pragma: no cover
        return 130


if __name__ == '__main__':  # pragma: no cover
    sys.exit(main())
