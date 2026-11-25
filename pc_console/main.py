"""RASPBOT-V2 PC 控制台程序入口。

用法::

    python main.py                  # 连接真车（默认 192.168.50.98:8080）
    python main.py --demo           # 无硬件演示模式（内置仿真）
    python main.py --pi 192.168.50.98:8080 --port 8090
    python main.py --help

本文件只做参数解析、浏览器启动与 Ctrl-C 生命周期；服务实现全部在
``console_server`` 包内，测试可直接 ``create_server(config, demo=True)``。
"""

from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
import time
import webbrowser
from pathlib import Path

from console_server.config import (
    BROWSER_CHOICES,
    VERSION,
    ConfigError,
    load_config,
    parse_pi_target,
)
from console_server.http_server import create_server
from console_server.upstream import probe_health

log = logging.getLogger("console.main")

IS_WINDOWS = sys.platform.startswith("win")

#: Edge / Chrome 常见安装路径（app 窗口模式）
BROWSER_CANDIDATES: tuple[str, ...] = (
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    os.path.expandvars(r"%LOCALAPPDATA%\Google\Chrome\Application\chrome.exe"),
    "/usr/bin/microsoft-edge",
    "/usr/bin/google-chrome",
    "/usr/bin/chromium",
    "/usr/bin/chromium-browser",
)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="main.py",
        description="RASPBOT-V2 PC 上位机控制台（本地 HTTP/SSE 代理 + 演示仿真）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "示例:\n"
            "  python main.py --demo --port 8099 --no-browser\n"
            "  python main.py --pi 192.168.50.98:8080 --token secret\n"
            "  python main.py --demo --browser app\n"
        ),
    )
    parser.add_argument(
        "--host",
        default=None,
        help="本地监听地址（默认 127.0.0.1，仅本机）",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=None,
        help="本地监听端口（默认 8090；0=由系统分配临时端口）",
    )
    parser.add_argument(
        "--pi",
        default=None,
        metavar="HOST[:PORT]",
        help="Pi 网关地址（默认 192.168.50.98:8080）",
    )
    parser.add_argument(
        "--token",
        default=None,
        help="访问 Pi 网关的 X-Console-Token（可留空）",
    )
    parser.add_argument(
        "--browser",
        choices=BROWSER_CHOICES,
        default=None,
        help="浏览器启动方式：auto=Windows 优先 app 窗口 / app / default / none",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="不自动打开浏览器（等价 --browser none）",
    )
    parser.add_argument(
        "--fallback-demo",
        action="store_true",
        help="启动时探测不到 Pi 网关则自动切换为演示模式",
    )
    parser.add_argument(
        "--demo",
        action="store_true",
        help="演示模式：使用内置仿真后端，无需 Pi/ROS",
    )
    parser.add_argument(
        "--config",
        default=None,
        metavar="PATH",
        help="配置文件路径（默认 pc_console/config.json）",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="输出调试日志（含每个 HTTP 请求）",
    )
    parser.add_argument(
        "--version",
        action="version",
        version=f"RASPBOT-V2 PC Console {VERSION}",
    )
    return parser.parse_args(argv)


# ===================================================================
# 浏览器启动
# ===================================================================
def find_app_browser() -> str | None:
    """查找可用的 Edge/Chrome 可执行文件（用于 ``--app`` 窗口）。"""
    for candidate in BROWSER_CANDIDATES:
        try:
            if candidate and Path(candidate).is_file():
                return candidate
        except OSError:  # pragma: no cover - 路径异常直接跳过
            continue
    return None


def open_browser(url: str, mode: str = "auto") -> None:
    """打开浏览器；任何失败都不影响服务运行（只打印 URL）。"""
    mode = (mode or "auto").lower()
    if mode == "none":
        print(f"  浏览器: 未自动打开（--browser none），请手动访问 {url}")
        return
    try:
        if mode in ("app", "auto"):
            exe = find_app_browser()
            if exe:
                cmd = [exe, f"--app={url}", "--window-size=1440,900"]
                log.info("启动浏览器 app 窗口: %s", exe)
                subprocess.Popen(  # noqa: S603 - 固定候选路径，无 shell
                    cmd,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL,
                )
                return
            if mode == "app":
                log.warning("未找到 Edge/Chrome，回退系统默认浏览器")
        if mode == "auto" and not IS_WINDOWS:
            log.info("非 Windows 平台，使用系统默认浏览器")
        webbrowser.open(url)
    except Exception as exc:  # noqa: BLE001 - 浏览器启动失败不影响服务
        log.warning("打开浏览器失败（服务仍在运行）: %s", exc)
        print(f"  请手动在浏览器打开: {url}")


# ===================================================================
# 主流程
# ===================================================================
def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    overrides: dict = {}
    if args.host is not None:
        overrides["host"] = args.host
    if args.port is not None:
        overrides["port"] = args.port
    if args.token is not None:
        overrides["token"] = args.token
    if args.browser is not None:
        overrides["browser"] = args.browser
    if args.no_browser:
        overrides["browser"] = "none"
    try:
        if args.pi is not None:
            pi_host, pi_port = parse_pi_target(args.pi)
            overrides["pi_host"] = pi_host
            overrides["pi_port"] = pi_port
        config = load_config(args.config, overrides, create=True)
    except ConfigError as exc:
        print(f"配置错误：{exc.message}", file=sys.stderr)
        return 2
    config.verbose = bool(args.verbose)

    demo = bool(args.demo)
    if not demo and args.fallback_demo:
        print(f"正在探测 Pi 网关 {config.pi_host}:{config.pi_port} …")
        probe = probe_health(
            config.pi_host, config.pi_port, config.token, timeout=2.0
        )
        if probe["reachable"]:
            print(f"  Pi 网关可达（{probe['latency_ms']} ms），使用真车模式")
        else:
            print(
                "  未探测到 Pi 网关"
                f"（{probe.get('error')}），自动切换为演示模式（--fallback-demo）"
            )
            demo = True

    try:
        server = create_server(config, demo=demo)
        port = server.start()
    except OSError as exc:
        print(f"启动失败：{exc}", file=sys.stderr)
        return 1

    url = server.url
    mode_text = "演示模式（内置仿真，无需 Pi/ROS）" if demo else (
        f"真车模式 → Pi 网关 {config.pi_host}:{config.pi_port}"
    )
    print()
    print(f"RASPBOT-V2 PC 控制台 v{VERSION}")
    print(f"  模式: {mode_text}")
    print(f"  地址: {url}")
    print(f"  配置: {config.config_path}")
    if port == 0:  # pragma: no cover - 理论上 start() 后已拿到实际端口
        print("  端口: 由系统分配")
    if demo:
        print("  提示: 演示模式可完整体验 建图 → 保存地图 → 导航 → 遥控 → 急停")
    else:
        print("  提示: 若浏览器提示无法连接，请检查 Pi 地址/端口（/local/config）")
    print("  按 Ctrl-C 退出（退出不会停止 Pi 上的 ROS 会话）")
    # 确保即使 stdout 被重定向（如后台运行/日志采集）也能立即看到实际 URL
    sys.stdout.flush()

    browser_mode = "none" if args.no_browser else (args.browser or config.browser)
    open_browser(url, browser_mode)

    try:
        while not server.wait(0.5):
            pass
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，正在退出…")
    finally:
        server.stop()
        print("控制台已退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
