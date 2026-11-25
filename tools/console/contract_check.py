#!/usr/bin/env python3
"""PC 上位机契约一致性检查（WP-C4 工具，仅标准库）。

用途
----
对「PC 端演示后端」或「Pi 端 raspbot_console 网关」做黑盒协议验收，验证
docs/PC上位机_设计文档_v1.md 的 HTTP/SSE 契约是否被满足。

用法
----
    # 只读检查（安全，不改变车辆状态）
    python tools/console/contract_check.py --base http://127.0.0.1:8090

    # 全流程检查（会启停会话、遥控、设目标！必须台架/空旷场地，车辆悬空或有人监护）
    python tools/console/contract_check.py --base http://127.0.0.1:8090 --full

    # 带 token（Pi 网关启用鉴权时）
    python tools/console/contract_check.py --base http://192.168.50.98:8080 --token abc --full

    # 自定义导航目标（默认 0.8, 0.0, yaw=0；需在地图空闲区域内）
    python tools/console/contract_check.py --base ... --full --goal 1.0 0.0 0.0

退出码：0 全部 PASS（允许 SKIP/WARN），1 存在 FAIL，2 参数/连接错误。
报告：--out report.json 可输出机器可读结果。
"""

from __future__ import annotations

import argparse
import http.client
import json
import math
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

PASS, FAIL, WARN, SKIP = 'PASS', 'FAIL', 'WARN', 'SKIP'
RESULTS = []  # type: list[dict]


# --------------------------------------------------------------------------
# 记录与断言
# --------------------------------------------------------------------------
def record(name: str, status: str, detail: str = '', elapsed: float = 0.0) -> None:
    RESULTS.append({'name': name, 'status': status, 'detail': detail,
                    'elapsed': round(elapsed, 3)})
    color = {PASS: 'PASS', FAIL: 'FAIL', WARN: 'WARN', SKIP: 'SKIP'}[status]
    print(f'  [{color}] {name}' + (f' —— {detail}' if detail else ''))


def check(name: str, ok: bool, detail: str = '', elapsed: float = 0.0) -> bool:
    record(name, PASS if ok else FAIL, detail, elapsed)
    return ok


def warn(name: str, detail: str = '') -> None:
    record(name, WARN, detail)


def skip(name: str, detail: str = '') -> None:
    record(name, SKIP, detail)


class Fatal(Exception):
    """连接不上目标等不可继续的错误。"""


# --------------------------------------------------------------------------
# HTTP 客户端
# --------------------------------------------------------------------------
class Client:
    def __init__(self, base: str, token: str = '', timeout: float = 10.0):
        base = base.rstrip('/')
        if not base.startswith('http://') and not base.startswith('https://'):
            base = 'http://' + base
        self.base = base
        self.token = token
        self.timeout = timeout
        u = urllib.parse.urlsplit(base)
        self.scheme = u.scheme
        self.host = u.hostname or '127.0.0.1'
        self.port = u.port or (443 if u.scheme == 'https' else 80)
        self.prefix = u.path.rstrip('/')

    def _url(self, path: str) -> str:
        return self.base + path

    def _headers(self) -> dict:
        h = {'Accept': 'application/json'}
        if self.token:
            h['X-Console-Token'] = self.token
        return h

    def get_json(self, path: str, timeout: float | None = None):
        """返回 (status, obj)；网络错误抛 Fatal。"""
        req = urllib.request.Request(self._url(path), headers=self._headers())
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                body = r.read()
                ctype = r.headers.get('Content-Type', '')
                if 'json' in ctype:
                    return r.status, json.loads(body.decode('utf-8'))
                return r.status, body
        except urllib.error.HTTPError as e:
            body = e.read()
            try:
                return e.code, json.loads(body.decode('utf-8'))
            except Exception:
                return e.code, body
        except (urllib.error.URLError, ConnectionError, socket.timeout) as e:
            raise Fatal(f'GET {path} 失败: {e}') from e

    def get_bytes(self, path: str, timeout: float | None = None):
        req = urllib.request.Request(self._url(path), headers=self._headers())
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                return r.status, r.headers, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers, e.read()
        except (urllib.error.URLError, ConnectionError, socket.timeout) as e:
            raise Fatal(f'GET {path} 失败: {e}') from e

    def post_json(self, path: str, payload: dict, timeout: float | None = None):
        data = json.dumps(payload).encode('utf-8')
        headers = self._headers()
        headers['Content-Type'] = 'application/json'
        req = urllib.request.Request(self._url(path), data=data, headers=headers,
                                     method='POST')
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                return r.status, json.loads(r.read().decode('utf-8') or '{}')
        except urllib.error.HTTPError as e:
            body = e.read()
            try:
                return e.code, json.loads(body.decode('utf-8'))
            except Exception:
                return e.code, {'raw': body.decode('utf-8', 'replace')}
        except (urllib.error.URLError, ConnectionError, socket.timeout) as e:
            raise Fatal(f'POST {path} 失败: {e}') from e


def read_sse(base: str, token: str, duration: float = 3.0) -> list[dict]:
    """读取 SSE duration 秒，返回 [{'event','data'}]。"""
    u = urllib.parse.urlsplit(base)
    host = u.hostname or '127.0.0.1'
    port = u.port or 80
    conn_cls = http.client.HTTPSConnection if u.scheme == 'https' else http.client.HTTPConnection
    conn = conn_cls(host, port, timeout=max(duration + 5, 10))
    path = (u.path.rstrip('/') if u.path else '') + '/api/events'
    if token:
        path += '?token=' + urllib.parse.quote(token)
    events = []
    try:
        conn.request('GET', path, headers={'Accept': 'text/event-stream'})
        resp = conn.getresponse()
        if resp.status != 200:
            return [{'event': '__http_error__', 'data': str(resp.status)}]
        deadline = time.monotonic() + duration
        ev_name, data_lines = None, []

        def flush():
            if ev_name:
                events.append({'event': ev_name,
                               'data': '\n'.join(data_lines)})
            return None, []

        while time.monotonic() < deadline:
            line = resp.readline()
            if not line:
                break
            text = line.decode('utf-8', 'replace').rstrip('\r\n')
            if text == '':
                ev_name, data_lines = flush()
            elif text.startswith(':'):
                continue
            elif text.startswith('event:'):
                ev_name = text[6:].strip()
            elif text.startswith('data:'):
                data_lines.append(text[5:].strip())
            elif text.startswith('retry:'):
                continue
        # flush 未结束的最后一帧
        if ev_name and data_lines:
            events.append({'event': ev_name, 'data': '\n'.join(data_lines)})
    except (socket.timeout, OSError, http.client.HTTPException):
        pass
    finally:
        try:
            conn.close()
        except Exception:
            pass
    return events


# --------------------------------------------------------------------------
# Schema 校验（宽松：额外字段允许）
# --------------------------------------------------------------------------
TYPES = {
    'float': (int, float),
    'int': (int,),
    'str': (str,),
    'bool': (bool,),
    'list': (list,),
    'dict': (dict,),
}


def validate(obj, spec: dict, prefix: str, errors: list[str]) -> None:
    if not isinstance(obj, dict):
        errors.append(f'{prefix} 不是对象')
        return
    for key, raw in spec.items():
        path = f'{prefix}.{key}' if prefix else key
        if key not in obj:
            errors.append(f'缺少字段 {path}')
            continue
        val = obj[key]
        nullable = isinstance(raw, tuple) and raw[0] == 'nullable'
        if nullable:
            if val is None:
                continue
            raw = raw[1]
        if isinstance(raw, dict):
            validate(val, raw, path, errors)
        elif isinstance(raw, (list, tuple)):
            kind = raw[0]
            if val is None:
                errors.append(f'{path} 不能为 null')
            elif not isinstance(val, TYPES[kind]):
                errors.append(f'{path} 类型应为 {kind}，实际 {type(val).__name__}')
        elif isinstance(raw, str):
            if not isinstance(val, TYPES[raw]):
                errors.append(f'{path} 类型应为 {raw}，实际 {type(val).__name__}')


STATE_SPEC = {
    'api': 'int',
    'server_time': 'float',
    'uptime_s': 'float',
    'session': {
        'mode': 'str', 'state': 'str', 'mock': 'bool', 'managed': 'bool',
        'external': 'bool', 'available_modes': 'list', 'progress': 'float',
        'message': 'str',
    },
    'ros': {
        'ok': 'bool', 'nodes': 'int',
        'ready': {'base': 'bool', 'teleop': 'bool', 'map': 'bool',
                  'localization': 'bool', 'nav2': 'bool', 'diagnostics': 'bool'},
    },
    'limits': {'max_linear': 'float', 'max_lateral': 'float',
               'max_angular': 'float'},
    'nav': {
        'state': 'str', 'message': 'str', 'goal_state': 'str',
        'lifecycle': 'dict',
        'activation': {'done': 'int', 'total': 'int', 'phase': 'str'},
    },
    'map': {'available': 'bool', 'version': ('nullable', 'int'),
            'width': ('nullable', 'int'), 'height': ('nullable', 'int'),
            'resolution': ('nullable', 'float'),
            'origin': ('nullable', {'x': 'float', 'y': 'float', 'yaw': 'float'}),
            'frame_id': ('nullable', 'str'), 'stamp': ('nullable', 'float')},
    'estop': {'active': 'bool'},
    'teleop': {'active': 'bool', 'vx': 'float', 'vy': 'float', 'wz': 'float'},
    'diagnostics': {
        'level': 'str',
        'counts': {'ok': 'int', 'warn': 'int', 'error': 'int'},
        'items': 'list',
    },
    'peripherals': {
        'buzzer': 'bool',
        'led': {'r': 'float', 'g': 'float', 'b': 'float'},
        'servo': {'pan': 'float', 'tilt': 'float'},
    },
    'logs': 'list',
}


def png_size(data: bytes):
    """解析 PNG IHDR 宽高；非 PNG 返回 None。"""
    if len(data) < 24 or data[:8] != b'\x89PNG\r\n\x1a\n':
        return None
    if data[12:16] != b'IHDR':
        return None
    width = int.from_bytes(data[16:20], 'big')
    height = int.from_bytes(data[20:24], 'big')
    return width, height


# --------------------------------------------------------------------------
# 等待工具
# --------------------------------------------------------------------------
def wait_until(client: Client, predicate, label: str, timeout: float,
               interval: float = 0.5):
    """轮询 /api/state 直到 predicate(state) 为真。返回 (ok, state, elapsed)。"""
    t0 = time.monotonic()
    last_state = None
    while time.monotonic() - t0 < timeout:
        status, state = client.get_json('/api/state')
        if status == 200 and isinstance(state, dict):
            last_state = state
            try:
                if predicate(state):
                    return True, state, time.monotonic() - t0
            except Exception:
                pass
        time.sleep(interval)
    return False, last_state, time.monotonic() - t0


def session_state(state: dict) -> str:
    return ((state or {}).get('session') or {}).get('state', '?') + '/' + \
        ((state or {}).get('session') or {}).get('mode', '?')


# --------------------------------------------------------------------------
# 检查项
# --------------------------------------------------------------------------
def check_readonly(client: Client, args) -> dict:
    report = {}
    t0 = time.monotonic()
    try:
        status, health = client.get_json('/api/health', timeout=5)
    except Fatal as e:
        raise Fatal(f'无法连接目标 {client.base}：{e}') from e
    check('/api/health 可访问且 ok=true',
          status == 200 and isinstance(health, dict) and health.get('ok') is True,
          f'HTTP {status}, name={health.get("name") if isinstance(health, dict) else health}',
          time.monotonic() - t0)
    if not (isinstance(health, dict) and health.get('ok') is True):
        return report
    missing = [k for k in ('name', 'version', 'api', 'server_time') if k not in health]
    check('/api/health 必填字段', not missing, f'缺少 {missing}' if missing else '')

    t0 = time.monotonic()
    status, state = client.get_json('/api/state')
    errors = []  # type: list[str]
    if status != 200:
        record('/api/state 可访问', FAIL, f'HTTP {status}')
    else:
        record('/api/state 可访问', PASS, f'{round((time.monotonic()-t0)*1000)} ms')
        validate(state, STATE_SPEC, '', errors)
        check('/api/state schema 符合契约', not errors,
              ('; '.join(errors[:8]) + (' …' if len(errors) > 8 else '')) if errors else '')
        report['state'] = state
        report['session_start_state'] = {
            'mode': state.get('session', {}).get('mode'),
            'state': state.get('session', {}).get('state'),
        }

    # SSE：3 秒内至少收到 state 事件；全流程模式还要求 scan/map
    t0 = time.monotonic()
    events = read_sse(client.base, client.token, duration=3.0)
    names = [e['event'] for e in events]
    ok = 'state' in names
    counts = {n: names.count(n) for n in set(names)}
    detail = f'3 s 内事件: {counts}' if names else '3 s 内未收到任何 SSE 事件'
    check('/api/events SSE 推送 state', ok, detail, time.monotonic() - t0)
    if names:
        try:
            sample = json.loads([e for e in events if e['event'] == 'state'][0]['data'])
            inner = sample.get('state', sample)
            report['sse_state'] = inner
        except Exception as e:  # noqa: BLE001
            warn('SSE state 事件可解析', str(e))
    report['sse_event_names'] = sorted(set(names))

    # 地图接口
    t0 = time.monotonic()
    status, meta = client.get_json('/api/map/meta')
    map_available = bool(isinstance(meta, dict) and meta.get('available'))
    check('/api/map/meta 可访问', status == 200, f'HTTP {status}, available={map_available}',
          time.monotonic() - t0)
    if map_available:
        status, headers, png = client.get_bytes(
            f"/api/map.png?v={meta.get('version', 0)}")
        size = png_size(png)
        expected = (meta.get('width'), meta.get('height'))
        check('/api/map.png 为合法 PNG 且尺寸与 meta 一致',
              status == 200 and size is not None and size == expected,
              f'HTTP {status}, png={size}, meta={expected}')
        report['map'] = meta
    else:
        skip('/api/map.png 校验', '当前无地图（idle/base 模式正常）')

    # 地图列表
    status, maps = client.get_json('/api/maps')
    ok = status == 200 and isinstance(maps, dict) and isinstance(maps.get('maps'), list)
    detail = f'HTTP {status}, 地图 {len(maps.get("maps", [])) if isinstance(maps, dict) else "?"} 张'
    check('/api/maps 可访问且结构正确', ok, detail)
    if ok:
        for m in maps['maps'][:20]:
            bad = [k for k in ('name', 'path', 'mtime', 'valid') if k not in m]
            if bad:
                warn(f'地图条目缺少字段 {bad}', str(m)[:120])
        report['maps'] = maps['maps']

    # 错误路径
    status, body = client.get_json('/api/this-does-not-exist')
    check('未知端点返回 404 JSON', status == 404 and isinstance(body, dict) and 'error' in body,
          f'HTTP {status}')
    return report


def ensure_idle(client: Client, label: str) -> bool:
    status, body = client.post_json('/api/session/stop', {})
    if status not in (200, 202, 409):
        record(f'{label}: 停止会话', FAIL, f'HTTP {status} {body}')
        return False
    ok, state, elapsed = wait_until(
        client, lambda s: (s.get('session') or {}).get('state') == 'idle',
        f'{label}: 等待 idle', timeout=90)
    if not ok:
        record(f'{label}: 等待 idle', FAIL,
               f'90 s 未 idle（{session_state(state)}）')
        return False
    record(f'{label}: 已就绪（idle）', PASS, f'{elapsed:.1f} s')
    return True


def start_session(client: Client, mode: str, map_path: str | None, label: str,
                  timeout: float = 150) -> bool:
    payload = {'mode': mode, 'use_mock': False}
    if map_path:
        payload['map'] = map_path
    status, body = client.post_json('/api/session/start', payload)
    if status not in (200, 202):
        record(f'{label}: 启动 {mode}', FAIL, f'HTTP {status} {body}')
        return False
    ok, state, elapsed = wait_until(
        client, lambda s: ((s.get('session') or {}).get('state') == 'running'
                           and (s.get('session') or {}).get('mode') == mode),
        f'{label}: 等待 running', timeout=timeout)
    err = (state or {}).get('session', {}).get('error')
    check(f'{label}: 启动 {mode} 并 running', ok,
          f'{elapsed:.1f} s' if ok else f'{session_state(state)} error={err}')
    return ok


def check_teleop(client: Client, label: str) -> None:
    ok, st0, _ = wait_until(client, lambda s: s.get('robot') is not None,
                            f'{label}: 等待机器人位姿', timeout=15)
    if not ok or not st0.get('robot'):
        skip(f'{label}: 遥控位姿变化', '无机器人位姿数据')
        return
    r0 = st0['robot']
    vx = 0.08
    wz = 0.25
    sent = 0
    t_end = time.monotonic() + 1.6
    while time.monotonic() < t_end:
        status, _ = client.post_json('/api/teleop',
                                     {'vx': vx, 'vy': 0.0, 'wz': wz,
                                      'seq': 1000 + sent})
        if status == 200:
            sent += 1
        elif status == 409:
            record(f'{label}: 遥控', FAIL, '遥控被拒绝（会话/急停状态异常）')
            return
        else:
            record(f'{label}: 遥控', FAIL, f'HTTP {status}')
            return
        time.sleep(0.1)
    client.post_json('/api/teleop/stop', {})
    time.sleep(1.0)
    _, st1 = client.get_json('/api/state')
    r1 = (st1 or {}).get('robot') or r0
    dx = float(r1.get('x', 0)) - float(r0.get('x', 0))
    dy = float(r1.get('y', 0)) - float(r0.get('y', 0))
    dyaw = float(r1.get('yaw', 0)) - float(r0.get('yaw', 0))
    moved = math.hypot(dx, dy) > 0.02 or abs(dyaw) > 0.05
    check(f'{label}: 遥控 {sent} 帧后位姿变化', moved,
          f'dx={dx:.3f} dy={dy:.3f} dyaw={dyaw:.3f}')
    # 看门狗：停止发指令 >0.5 s 后速度应归零
    time.sleep(1.2)
    _, st2 = client.get_json('/api/state')
    speed = ((st2 or {}).get('robot') or {}).get('speed') or {}
    idle = abs(float(speed.get('vx', 0))) < 0.05 and abs(float(speed.get('wz', 0))) < 0.1
    check(f'{label}: 遥控看门狗 0.5 s 后停车', idle, f'speed={speed}')


def check_estop(client: Client, label: str) -> None:
    status, body = client.post_json('/api/estop', {'active': True})
    check(f'{label}: 急停锁存', status == 200 and body.get('estop', {}).get('active') is True,
          f'HTTP {status}')
    ok, state, _ = wait_until(client, lambda s: (s.get('estop') or {}).get('active') is True,
                              f'{label}: 状态回显 estop', timeout=10)
    check(f'{label}: /api/state 回显急停', ok, session_state(state))
    status, body = client.post_json('/api/teleop', {'vx': 0.1, 'vy': 0.0, 'wz': 0.0})
    check(f'{label}: 急停期间遥控被拒（409）', status == 409, f'HTTP {status}')
    status, body = client.post_json('/api/goal', {'x': 0.5, 'y': 0.0, 'yaw': 0.0})
    check(f'{label}: 急停期间目标被拒（409）', status == 409, f'HTTP {status}')
    status, _ = client.post_json('/api/estop', {'active': False})
    ok, state, _ = wait_until(client, lambda s: (s.get('estop') or {}).get('active') is False,
                              f'{label}: 急停解除', timeout=10)
    check(f'{label}: 急停解除', status == 200 and ok, session_state(state))


def check_map_save(client: Client, label: str) -> str | None:
    name = 'contract_check_' + time.strftime('%Y%m%d_%H%M%S')
    status, body = client.post_json('/api/map/save', {'name': name, 'overwrite': False})
    if status != 200:
        record(f'{label}: 保存地图', FAIL, f'HTTP {status} {body}')
        return None
    path = body.get('yaml') or body.get('path')
    ok = bool(path) and str(path).endswith('.yaml')
    check(f'{label}: 保存地图成功', ok, f'{name} → {path}')
    status2, body2 = client.post_json('/api/map/save', {'name': name, 'overwrite': False})
    check(f'{label}: 重名保存返回 409', status2 == 409, f'HTTP {status2}')
    status, maps = client.get_json('/api/maps')
    names = [m.get('name') for m in (maps.get('maps') if isinstance(maps, dict) else [])]
    check(f'{label}: 地图列表可见 {name}', name in names, f'共 {len(names)} 张')
    return path


def check_nav(client: Client, args, label: str) -> None:
    map_path = args.nav_map or args.saved_map
    if map_path and not map_path.endswith('.yaml'):
        # 允许传地图名：从 /api/maps 解析真实路径
        status, maps = client.get_json('/api/maps')
        entries = maps.get('maps', []) if isinstance(maps, dict) else []
        hit = next((m for m in entries if m.get('name') == map_path), None)
        if hit and hit.get('path'):
            map_path = hit['path']
        else:
            record(f'{label}: 解析已保存地图', FAIL, f'未找到地图名 {map_path}')
            return
    if not map_path:
        skip(f'{label}: 导航全流程', '无可用于导航的地图（保存失败/无图）')
        return
    if not start_session(client, 'nav', map_path, label):
        return
    ok, state, elapsed = wait_until(
        client, lambda s: (s.get('nav') or {}).get('state') in
        ('waiting_initialpose', 'activating', 'ready'),
        f'{label}: 等待导航模式就绪入口', timeout=120)
    nav_state = ((state or {}).get('nav') or {}).get('state')
    check(f'{label}: 导航模式进入 waiting_initialpose', ok, f'{nav_state} ({elapsed:.1f} s)')
    if not ok:
        return
    ix, iy, iyaw = args.initialpose
    t0 = time.monotonic()
    status, body = client.post_json('/api/initialpose', {'x': ix, 'y': iy, 'yaw': iyaw})
    check(f'{label}: 发布初始位姿', status == 200 and body.get('ok') is True, f'HTTP {status}')

    def nav_active(s):
        ns = (s.get('nav') or {}).get('state')
        return ns in ('ready', 'error')

    ok, state, elapsed = wait_until(client, nav_active,
                                    f'{label}: 等待 Nav2 激活', timeout=240)
    nav = (state or {}).get('nav') or {}
    check(f'{label}: Nav2 激活 → ready', ok and nav.get('state') == 'ready',
          f"{nav.get('state')} phase={nav.get('activation', {}).get('phase')} "
          f"done={nav.get('activation', {}).get('done')}/{nav.get('activation', {}).get('total')} "
          f"({elapsed:.1f} s)")
    if not (ok and nav.get('state') == 'ready'):
        return

    gx, gy, gyaw = args.goal
    status, body = client.post_json('/api/goal', {'x': gx, 'y': gy, 'yaw': gyaw})
    check(f'{label}: 发送导航目标', status == 202 and body.get('ok') is True,
          f'HTTP {status} goal={body.get("goal")}')
    if status != 202:
        return
    t0 = time.monotonic()
    ok, state, elapsed = wait_until(
        client,
        lambda s: (s.get('nav') or {}).get('state') in
        ('succeeded', 'failed', 'canceled', 'error'),
        f'{label}: 等待目标结束', timeout=args.nav_timeout)
    nav = (state or {}).get('nav') or {}
    check(f'{label}: 导航目标成功到达', ok and nav.get('state') == 'succeeded',
          f"{nav.get('state')} 剩余={nav.get('distance_remaining')} "
          f"用时={nav.get('navigation_time_s')} ({elapsed:.1f} s)")


def check_full(client: Client, args) -> dict:
    report = {}
    if not ensure_idle(client, '全流程'):
        report['full_skipped'] = '无法回到 idle（外部栈/附加模式？）'
        warn('全流程会话测试', report['full_skipped'])
        return report

    # ---- 建图 ----
    if not start_session(client, 'slam', None, '建图'):
        return report
    ok, state, elapsed = wait_until(
        client, lambda s: (s.get('map') or {}).get('available') is True,
        '建图: 等待首帧地图', timeout=90)
    check('建图: 2 s 量级内出现地图', ok, f'{elapsed:.1f} s')
    if ok:
        meta = state['map']
        status, _, png = client.get_bytes(f"/api/map.png?v={meta['version']}")
        check('建图: 地图 PNG 可解码', png_size(png) == (meta['width'], meta['height']),
              f'{png_size(png)}')
    check_teleop(client, '建图')
    check_estop(client, '建图')
    # 急停解除后确认位置仍可遥控（可选，不做）
    map_path = check_map_save(client, '建图')
    report['saved_map'] = map_path
    args.saved_map = map_path

    # 停止建图
    status, body = client.post_json('/api/session/stop', {})
    ok, state, elapsed = wait_until(
        client, lambda s: (s.get('session') or {}).get('state') == 'idle',
        '建图: 停止后 idle', timeout=90)
    check('建图: 停止会话回到 idle', status in (200, 202) and ok, f'{elapsed:.1f} s')

    # ---- 导航 ----
    check_nav(client, args, '导航')
    client.post_json('/api/session/stop', {})
    ok, state, elapsed = wait_until(
        client, lambda s: (s.get('session') or {}).get('state') == 'idle',
        '导航: 停止后 idle', timeout=90)
    check('导航: 停止会话回到 idle', ok, f'{elapsed:.1f} s')

    # ---- 错误路径 ----
    status, _ = client.post_json('/api/goal', {'x': 1.0, 'y': 0.0, 'yaw': 0.0})
    check('idle 时发送目标返回 409', status == 409, f'HTTP {status}')
    status, _ = client.post_json('/api/session/start', {'mode': 'nav'})
    check('nav 模式缺 map 参数返回 400', status == 400, f'HTTP {status}')
    status, _ = client.post_json('/api/teleop', {'vx': 0.1, 'vy': 0.0, 'wz': 0.0})
    check('idle 时遥控返回 409', status == 409, f'HTTP {status}')
    return report


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description='PC 上位机 / Pi 网关 HTTP+SSE 契约一致性检查')
    ap.add_argument('--base', default='http://127.0.0.1:8090',
                    help='目标 base URL（默认演示服务 http://127.0.0.1:8090）')
    ap.add_argument('--token', default='', help='X-Console-Token')
    ap.add_argument('--demo', action='store_true',
                    help='兼容设计文档中的演示模式命令；本工具不区分 demo/真网关')
    ap.add_argument('--full', action='store_true',
                    help='执行会改变车辆状态的全流程（启停会话/遥控/导航/急停）；'
                         '需台架或空旷场地')
    ap.add_argument('--goal', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                    default=[0.8, 0.0, 0.0], help='导航目标（默认 0.8 0.0 0.0）')
    ap.add_argument('--initialpose', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                    default=[0.0, 0.0, 0.0],
                    help='导航初始位姿（默认 0.0 0.0 0.0；需与地图匹配）')
    ap.add_argument('--nav-map', default='',
                    help='导航模式使用的地图（绝对路径或 /api/maps 中的名称）；'
                         '默认使用本次建图保存的地图')
    ap.add_argument('--nav-timeout', type=float, default=180.0,
                    help='等待导航成功的超时秒数（默认 180）')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='单次 HTTP 超时秒数（默认 10）')
    ap.add_argument('--out', default='', help='JSON 报告输出路径')
    args = ap.parse_args(argv)

    client = Client(args.base, args.token, args.timeout)
    print(f'== 契约检查: {client.base} ==')
    t0 = time.monotonic()
    report = {'base': client.base, 'full': bool(args.full), 'checks': RESULTS}
    try:
        report.update(check_readonly(client, args))
        if args.full:
            report.update(check_full(client, args))
    except Fatal as e:
        record('连接目标', FAIL, str(e))
        print(f'\n[FAIL] {e}')
        report['fatal'] = str(e)
        RESULTS.append({'name': '连接目标', 'status': FAIL, 'detail': str(e)})
    except KeyboardInterrupt:
        print('\n[中断] 用户 Ctrl-C')
        RESULTS.append({'name': '用户中断', 'status': WARN, 'detail': 'Ctrl-C'})
    report['checks'] = RESULTS
    report['elapsed_s'] = round(time.monotonic() - t0, 1)
    fails = [r for r in RESULTS if r['status'] == FAIL]
    warns = [r for r in RESULTS if r['status'] == WARN]
    skips = [r for r in RESULTS if r['status'] == SKIP]
    report['summary'] = {'pass': sum(1 for r in RESULTS if r['status'] == PASS),
                         'fail': len(fails), 'warn': len(warns), 'skip': len(skips)}
    print(f'\n== 汇总: {report["summary"]} 用时 {report["elapsed_s"]} s ==')
    if fails:
        print('失败项:')
        for r in fails:
            print(f'  - {r["name"]}: {r["detail"]}')
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f'报告已写入: {args.out}')
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
