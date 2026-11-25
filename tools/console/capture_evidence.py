#!/usr/bin/env python3
"""采集 PC 上位机演示模式的界面截图证据（WP-C4 工具，仅标准库）。

前置：
  1. 已启动演示服务（另开终端）：
       cd pc_console
       python main.py --demo --port 8099 --no-browser
  2. 本机有 Edge 或 Chrome（脚本自动探测，或用 --edge 指定）。
  3. 无人使用该演示服务（脚本会启停会话、发导航目标、触发急停）。

用法：
    python tools/console/capture_evidence.py
    python tools/console/capture_evidence.py --base http://127.0.0.1:8099 \
        --out docs/evidence --initialpose 1.5 1.5 0 --goal 4.5 1.5 0
    python tools/console/capture_evidence.py --keep-running   # 结束后保留 slam 状态

产出：docs/evidence/pc_console_*.png + capture_report.json
退出码：0 成功；1 有截图失败。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request

EDGE_CANDIDATES = [
    r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe',
    r'C:\Program Files\Microsoft\Edge\Application\msedge.exe',
    r'C:\Program Files\Google\Chrome\Application\chrome.exe',
    r'C:\Program Files (x86)\Google\Chrome\Application\chrome.exe',
    '/usr/bin/google-chrome', '/usr/bin/chromium', '/usr/bin/chromium-browser',
    '/usr/bin/microsoft-edge',
]


def find_browser(explicit: str = '') -> str:
    if explicit:
        if os.path.isfile(explicit):
            return explicit
        raise SystemExit(f'[FAIL] 指定的浏览器不存在: {explicit}')
    env = os.environ.get('RASPBOT_BROWSER', '')
    if env and os.path.isfile(env):
        return env
    for p in EDGE_CANDIDATES:
        if os.path.isfile(p):
            return p
    raise SystemExit('[FAIL] 未找到 Edge/Chrome，请用 --edge 指定可执行文件路径')


class Api:
    def __init__(self, base: str, timeout: float = 10.0):
        self.base = base.rstrip('/')
        self.timeout = timeout

    def call(self, method: str, path: str, payload=None):
        data = None
        headers = {'Accept': 'application/json'}
        if payload is not None:
            data = json.dumps(payload).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        req = urllib.request.Request(self.base + path, data=data,
                                     headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as r:
                return r.status, json.loads(r.read().decode('utf-8') or '{}')
        except urllib.error.HTTPError as e:
            raw = e.read()
            try:
                return e.code, json.loads(raw.decode('utf-8'))
            except Exception:
                return e.code, {'raw': raw.decode('utf-8', 'replace')}
        except Exception as e:  # noqa: BLE001
            raise SystemExit(f'[FAIL] {method} {path}: {e}')

    def health_ok(self, timeout: float = 45.0) -> dict:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                status, body = self.call('GET', '/api/health')
                if status == 200 and body.get('ok'):
                    return body
            except SystemExit:
                pass
            time.sleep(0.5)
        raise SystemExit(f'[FAIL] {self.base} 无法访问 /api/health（演示服务未启动？）')

    def state(self) -> dict:
        status, body = self.call('GET', '/api/state')
        return body if status == 200 else {}

    def wait(self, predicate, label: str, timeout: float, interval: float = 0.4):
        t0 = time.monotonic()
        last = {}
        while time.monotonic() - t0 < timeout:
            last = self.state()
            try:
                if predicate(last):
                    return last
            except Exception:
                pass
            time.sleep(interval)
        raise SystemExit(f'[FAIL] 等待超时（{timeout}s）: {label}；最后状态={json.dumps(last, ensure_ascii=False)[:400]}')


def png_size(data: bytes):
    if len(data) < 24 or data[:8] != b'\x89PNG\r\n\x1a\n' or data[12:16] != b'IHDR':
        return None
    return int.from_bytes(data[16:20], 'big'), int.from_bytes(data[20:24], 'big')


def file_png_size(path: str):
    with open(path, 'rb') as f:
        head = f.read(33)
    return png_size(head)


def screenshot(browser: str, base: str, out_path: str, width: int, height: int,
               timeout_s: int = 60) -> tuple[bool, str]:
    profile = tempfile.mkdtemp(prefix='raspbot_shot_')
    args = [
        browser,
        '--headless=new',
        '--disable-gpu',
        '--hide-scrollbars',
        '--no-first-run',
        '--no-default-browser-check',
        '--force-device-scale-factor=1',
        f'--window-size={width},{height}',
        f'--user-data-dir={profile}',
        '--virtual-time-budget=5000',
        f'--timeout={max(10000, timeout_s * 1000 // 2)}',
        f'--screenshot={out_path}',
        base.rstrip('/') + '/',
    ]
    try:
        cp = subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            timeout=timeout_s)
        log_tail = cp.stdout.decode('utf-8', 'replace').strip().splitlines()[-3:]
    except subprocess.TimeoutExpired:
        log_tail = ['browser timeout']
    finally:
        shutil.rmtree(profile, ignore_errors=True)
    if not os.path.isfile(out_path):
        return False, '; '.join(log_tail)
    size = os.path.getsize(out_path)
    dims = file_png_size(out_path)
    if size < 10_000 or not dims:
        return False, f'截图异常 size={size} dims={dims}'
    return True, f'{size // 1024} KB, {dims[0]}x{dims[1]}'


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='采集 PC 上位机演示模式截图证据')
    ap.add_argument('--base', default='http://127.0.0.1:8099')
    ap.add_argument('--out', default='docs/evidence')
    ap.add_argument('--edge', default='', help='Edge/Chrome 可执行文件路径')
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=1000)
    ap.add_argument('--initialpose', nargs=3, type=float, default=[1.5, 1.5, 0.0])
    ap.add_argument('--goal', nargs=3, type=float, default=[4.5, 1.5, 0.0])
    ap.add_argument('--nav-map', default='demo_home',
                    help='导航使用的地图名（来自 /api/maps，默认 demo_home）')
    ap.add_argument('--slam-teleop', type=float, default=3.0,
                    help='建图截屏前遥控探测的秒数（默认 3）')
    ap.add_argument('--keep-running', action='store_true',
                    help='结束后不停止会话（默认恢复 idle）')
    args = ap.parse_args(argv)

    browser = find_browser(args.edge)
    os.makedirs(args.out, exist_ok=True)
    report = {'base': args.base, 'browser': browser, 'shots': [], 'steps': []}

    def log(msg):
        print(msg, flush=True)
        report['steps'].append(msg)

    def shot(name: str) -> bool:
        path = os.path.join(args.out, name)
        if os.path.exists(path):
            os.remove(path)
        ok, detail = screenshot(browser, args.base, path, args.width, args.height)
        report['shots'].append({'file': path, 'ok': ok, 'detail': detail})
        print(('  [OK] ' if ok else '  [FAIL] ') + f'{name} —— {detail}', flush=True)
        return ok

    api = Api(args.base)
    info = api.health_ok()
    log(f'服务可达: {info.get("name")} {info.get("version")}')

    # ---- 0. 回到 idle ----
    api.call('POST', '/api/session/stop', {})
    api.wait(lambda s: (s.get('session') or {}).get('state') == 'idle',
             'idle', timeout=90)
    shot('pc_console_00_idle.png')

    # ---- 1. 建图 ----
    status, body = api.call('POST', '/api/session/start', {'mode': 'slam'})
    if status not in (200, 202):
        raise SystemExit(f'[FAIL] 启动建图失败: {status} {body}')
    api.wait(lambda s: (s.get('session') or {}).get('state') == 'running',
             '建图 running', timeout=120)
    api.wait(lambda s: (s.get('map') or {}).get('available') is True,
             '首帧地图', timeout=90)
    log('建图已启动，开始遥控探测以揭示地图…')
    t_end = time.monotonic() + max(args.slam_teleop, 1.0)
    seq = 0
    while time.monotonic() < t_end:
        # 走一小段弧线，既有位移也有转角
        vx = 0.12 if (t_end - time.monotonic()) > 0.8 else 0.0
        api.call('POST', '/api/teleop', {'vx': vx, 'vy': 0.0, 'wz': 0.25,
                                        'seq': seq})
        seq += 1
        time.sleep(0.1)
    api.call('POST', '/api/teleop/stop', {})
    time.sleep(1.0)
    api.wait(lambda s: (s.get('session') or {}).get('state') == 'running',
             '建图仍运行', timeout=10)
    shot('pc_console_01_mapping.png')

    # 保存地图（同时验证保存链路）
    name = 'evidence_' + time.strftime('%Y%m%d_%H%M%S')
    status, body = api.call('POST', '/api/map/save', {'name': name, 'overwrite': True})
    saved = body.get('yaml') or body.get('path') if isinstance(body, dict) else None
    log(f'保存地图: {name} -> {saved} (HTTP {status})')

    # ---- 2. 导航 ----
    api.call('POST', '/api/session/stop', {})
    api.wait(lambda s: (s.get('session') or {}).get('state') == 'idle',
             '停止建图', timeout=90)
    # 选一张完整地图（演示默认 demo_home；真实 Pi 可用 --nav-map 或本次保存的地图）
    status, maps_body = api.call('GET', '/api/maps')
    entries = maps_body.get('maps', []) if isinstance(maps_body, dict) else []
    nav_map = None
    for m in entries:
        if m.get('name') == args.nav_map and m.get('valid', True):
            nav_map = m.get('path')
            break
    if not nav_map and saved:
        nav_map = saved
    if not nav_map:
        raise SystemExit('[FAIL] 找不到可用于导航的地图（demo 应自带 demo_home）')
    log(f'导航地图: {nav_map}')
    status, body = api.call('POST', '/api/session/start', {'mode': 'nav', 'map': nav_map})
    if status not in (200, 202):
        raise SystemExit(f'[FAIL] 启动导航失败: {status} {body}')
    api.wait(lambda s: (s.get('session') or {}).get('state') == 'running',
             '导航 running', timeout=120)
    api.wait(lambda s: (s.get('nav') or {}).get('state') in
             ('waiting_initialpose', 'activating', 'ready'),
             '导航入口', timeout=120)
    ix, iy, iyaw = args.initialpose
    api.call('POST', '/api/initialpose', {'x': ix, 'y': iy, 'yaw': iyaw})
    api.wait(lambda s: (s.get('nav') or {}).get('state') == 'ready',
             'Nav2 激活 ready', timeout=180)
    time.sleep(1.0)
    shot('pc_console_02_nav_ready.png')

    gx, gy, gyaw = args.goal
    status, body = api.call('POST', '/api/goal', {'x': gx, 'y': gy, 'yaw': gyaw})
    log(f'发送目标 ({gx}, {gy}, {gyaw}): HTTP {status}')
    api.wait(lambda s: (s.get('nav') or {}).get('state') in
             ('goal_active', 'succeeded', 'failed'),
             '目标执行', timeout=60)
    time.sleep(2.0)
    shot('pc_console_03_navigating.png')

    # ---- 3. 急停 ----
    api.call('POST', '/api/estop', {'active': True})
    api.wait(lambda s: (s.get('estop') or {}).get('active') is True,
             '急停回显', timeout=10)
    time.sleep(0.8)
    shot('pc_console_04_estop.png')
    api.call('POST', '/api/estop', {'active': False})
    api.wait(lambda s: (s.get('estop') or {}).get('active') is False,
             '急停解除', timeout=10)

    if not args.keep_running:
        api.call('POST', '/api/session/stop', {})
        try:
            api.wait(lambda s: (s.get('session') or {}).get('state') == 'idle',
                     '恢复 idle', timeout=90)
        except SystemExit as e:
            log(f'[WARN] {e}')

    ok = all(s['ok'] for s in report['shots'])
    report_path = os.path.join(args.out, 'capture_report.json')
    with open(report_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(f'\n== 截图 {sum(1 for s in report["shots"] if s["ok"])}/{len(report["shots"])} 成功，'
          f'报告: {report_path} ==')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
