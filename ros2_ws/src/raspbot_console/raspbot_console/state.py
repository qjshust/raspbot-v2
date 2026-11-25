"""线程安全状态仓 + SSE 事件总线 + 速率统计 + 诊断聚合（纯逻辑）。

本模块只依赖标准库，导入期不依赖 rclpy，可离线单元测试。

分工：

- :class:`EventBus`：每客户端独立 ``queue.Queue(maxsize=200)``，满时丢最旧；
  最多 ``max_sse`` 个客户端（契约 §3.0/§5.1）。
- :class:`RateMeter`：2 s 滑动窗口频率（§5.4）。
- :func:`aggregate_diagnostics`：``/diagnostics`` 1 Hz 聚合（§5.4）。
- :class:`StateStore`：保存所有字段并产出 §3.1 的完整快照；所有读写加锁，
  HTTP/Runtime/会话监控线程可并发访问。
"""

from __future__ import annotations

import copy
import queue
import threading
import time
from collections import deque

from . import protocol

#: 默认速度上限（契约 §3.1 ``limits``；与 base_node 默认参数一致）
DEFAULT_LIMITS = {
    'max_linear': 0.30,
    'max_lateral': 0.30,
    'max_angular': 1.5,
}

#: 可用会话模式（§3.1 session.available_modes）
AVAILABLE_MODES = ['base', 'slam', 'nav']

#: 诊断条目数上限（§5.4：最多 12 条，非 OK 优先）
DIAG_MAX_ITEMS = 12

#: 事件总线单客户端队列深度（§5.1）
SSE_QUEUE_SIZE = 200

#: 关闭事件总线的哨兵（SSE 线程收到即退出）
CLOSE_SENTINEL = ('__close__', None)

#: DiagnosticStatus.level → 契约 level 名（3=STALE 归入 warn）
_DIAG_LEVEL_NAME = {0: 'ok', 1: 'warn', 2: 'error', 3: 'warn'}
_DIAG_SEVERITY = {'ok': 0, 'warn': 1, 'error': 2}


def default_session_snapshot(since=None):
    """§3.1 ``session`` 对象的初始值。"""
    return {
        'mode': 'idle',
        'state': 'idle',
        'mock': False,
        'managed': True,
        'external': False,
        'available_modes': list(AVAILABLE_MODES),
        'since': float(since) if since is not None else None,
        'progress': 0.0,
        'message': '空闲：可选择运行模式',
        'error': None,
        'map': None,
    }


def default_nav_snapshot():
    """§3.1 ``nav`` 对象的初始值。"""
    return {
        'state': 'unavailable',
        'message': '未进入导航模式',
        'initialpose': None,
        'goal': None,
        'goal_state': 'idle',
        'distance_remaining': None,
        'navigation_time_s': None,
        'recoveries': None,
        'lifecycle': {},
        'activation': {'done': 0, 'total': 6, 'phase': ''},
    }


def default_peripherals():
    """§3.1 ``peripherals`` 的初始值（与 base_node 默认姿态一致）。"""
    return {
        'buzzer': False,
        'led': {'r': 0.0, 'g': 0.0, 'b': 0.0},
        'servo': {'pan': 90.0, 'tilt': 45.0},
    }


def default_ready():
    """§3.1 ``ros.ready`` 的初始值。"""
    return {
        'base': False,
        'teleop': False,
        'map': False,
        'localization': False,
        'nav2': False,
        'diagnostics': False,
    }


def _diag_level_name(level):
    try:
        return _DIAG_LEVEL_NAME.get(int(level), 'unknown')
    except (TypeError, ValueError):
        return 'unknown'


def aggregate_diagnostics(items):
    """把 DiagnosticStatus 列表聚合成 §3.1 ``diagnostics``。

    - ``level`` 取最严重（error > warn > ok；无数据由调用方置 unknown）；
    - ``counts`` 统计全部条目；
    - ``items`` 最多 12 条，非 OK 优先，同级保持原顺序。
    """
    mapped = []
    for index, item in enumerate(items or ()):
        if not isinstance(item, dict):
            continue
        level = _diag_level_name(item.get('level'))
        if level == 'unknown':
            level = 'warn'
        mapped.append((index, {
            'name': str(item.get('name', '')),
            'level': level,
            'message': str(item.get('message', '')),
            'hardware_id': str(item.get('hardware_id', '')),
        }))
    counts = {'ok': 0, 'warn': 0, 'error': 0}
    for _index, entry in mapped:
        counts[entry['level']] = counts.get(entry['level'], 0) + 1
    if not mapped:
        level = 'unknown'
    elif counts['error']:
        level = 'error'
    elif counts['warn']:
        level = 'warn'
    else:
        level = 'ok'

    def sort_key(pair):
        index, entry = pair
        non_ok = 0 if entry['level'] != 'ok' else 1
        return (non_ok, -_DIAG_SEVERITY[entry['level']], index)

    ordered = [entry for _index, entry in sorted(mapped, key=sort_key)]
    return {
        'level': level,
        'counts': counts,
        'items': ordered[:DIAG_MAX_ITEMS],
    }


class EventBus:
    """SSE 事件总线：每客户端一个队列，``publish`` 按注册顺序尽力投递。"""

    def __init__(self, max_clients=protocol.MAX_SSE_CLIENTS,
                 queue_size=SSE_QUEUE_SIZE):
        self.max_clients = int(max_clients)
        self.queue_size = int(queue_size)
        self._lock = threading.Lock()
        self._clients = {}
        self._next_id = 0
        self._closed = False

    # -- 生命周期 -------------------------------------------------------
    def subscribe(self):
        """注册一个 SSE 客户端，返回 ``(client_id, queue)``。

        超过上限 → :class:`EventBusFull`；总线已关闭 → :class:`EventBusClosed`。
        """
        with self._lock:
            if self._closed:
                raise EventBusClosed('事件总线已关闭')
            if len(self._clients) >= self.max_clients:
                raise EventBusFull('SSE 客户端已达上限 %d' % self.max_clients)
            client_id = self._next_id
            self._next_id += 1
            client_queue = queue.Queue(maxsize=self.queue_size)
            self._clients[client_id] = client_queue
            return client_id, client_queue

    def unsubscribe(self, client_id):
        with self._lock:
            self._clients.pop(client_id, None)

    @property
    def client_count(self):
        with self._lock:
            return len(self._clients)

    @property
    def closed(self):
        with self._lock:
            return self._closed

    def close(self):
        """关闭总线：所有客户端队列收到 :data:`CLOSE_SENTINEL`。"""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            clients = list(self._clients.values())
            self._clients.clear()
        for client_queue in clients:
            _put_drop_oldest(client_queue, CLOSE_SENTINEL)

    # -- 投递 -----------------------------------------------------------
    def publish(self, event, payload):
        """广播一条事件；返回投递到的客户端数（总线关闭时 0）。"""
        with self._lock:
            if self._closed:
                return 0
            clients = list(self._clients.values())
        for client_queue in clients:
            _put_drop_oldest(client_queue, (str(event), payload))
        return len(clients)


def _put_drop_oldest(client_queue, item):
    """入队；满时丢弃最旧一条再入队（§5.1）。"""
    try:
        client_queue.put_nowait(item)
        return
    except queue.Full:
        pass
    try:
        client_queue.get_nowait()
    except queue.Empty:
        pass
    try:
        client_queue.put_nowait(item)
    except queue.Full:  # pragma: no cover - 竞态下防御
        pass


class EventBusFull(Exception):
    """SSE 客户端超过上限。"""


class EventBusClosed(Exception):
    """事件总线已关闭。"""


class RateMeter:
    """滑动窗口频率计：``rate()`` 返回窗口内消息频率（Hz）。"""

    def __init__(self, window_s=2.0, clock=time.monotonic):
        self.window_s = float(window_s)
        self._clock = clock
        self._times = deque()

    def tick(self, now=None):
        stamp = self._clock() if now is None else float(now)
        self._times.append(stamp)
        self._prune(stamp)

    def rate(self, now=None):
        stamp = self._clock() if now is None else float(now)
        self._prune(stamp)
        count = len(self._times)
        if count < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0.0:
            return 0.0
        return (count - 1) / span

    def _prune(self, now):
        while self._times and now - self._times[0] > self.window_s:
            self._times.popleft()

    def reset(self):
        self._times.clear()


class LogBuffer:
    """网关事件日志环形缓冲（GET /api/logs 最多 300 条，§3.3）。"""

    def __init__(self, maxlen=protocol.MAX_GATEWAY_LOGS, clock=time.time):
        self.maxlen = int(maxlen)
        self._clock = clock
        self._items = deque(maxlen=self.maxlen)

    def add(self, level, message, ts=None):
        entry = {
            'ts': float(self._clock() if ts is None else ts),
            'level': str(level),
            'msg': str(message),
        }
        self._items.append(entry)
        return entry

    def recent(self, limit=None):
        items = list(self._items)
        if limit is not None:
            limit = int(limit)
            if limit <= 0:
                return []
            items = items[-limit:]
        return items

    def __len__(self):
        return len(self._items)


class StateStore:
    """网关共享状态仓；所有方法线程安全（内部 ``RLock``）。"""

    def __init__(self, limits=None, *, clock=time.time,
                 max_logs=protocol.MAX_GATEWAY_LOGS, max_sse=None,
                 log_recent=20, started_at=None):
        self._lock = threading.RLock()
        self._clock = clock
        self.event_bus = EventBus(
            max_sse if max_sse is not None else protocol.MAX_SSE_CLIENTS)
        self.logs = LogBuffer(max_logs, clock=clock)
        self.log_recent = int(log_recent)
        self.limits = dict(DEFAULT_LIMITS)
        if limits:
            self.limits.update({
                'max_linear': float(limits.get('max_linear',
                                               DEFAULT_LIMITS['max_linear'])),
                'max_lateral': float(limits.get('max_lateral',
                                                DEFAULT_LIMITS['max_lateral'])),
                'max_angular': float(limits.get('max_angular',
                                                DEFAULT_LIMITS['max_angular'])),
            })
        self._started_wall = float(
            started_at if started_at is not None else self._now())
        self._session = default_session_snapshot(since=self._started_wall)
        self._nav = default_nav_snapshot()
        self._robot = None
        self._map_grid = None
        self._map_meta = None
        self._map_version = 0
        self._map_hash = None
        self._map_received_ts = None
        self._estop = {'active': False, 'since': None}
        self._estop_commanded = False
        self._teleop = {'active': False, 'vx': 0.0, 'vy': 0.0, 'wz': 0.0,
                        'last_cmd': None}
        self._teleop_seq = None
        self._peripherals = default_peripherals()
        self._diagnostics = {
            'level': 'unknown',
            'counts': {'ok': 0, 'warn': 0, 'error': 0},
            'items': [],
            'received_ts': None,
        }
        self._ros = {'ok': False, 'nodes': 0, 'ready': default_ready()}
        self._rate_scan = RateMeter(2.0, clock=self._clock)
        self._rate_odom = RateMeter(2.0, clock=self._clock)

    # ------------------------------------------------------------------
    # 时间 / 日志
    # ------------------------------------------------------------------
    def _now(self):
        return float(self._clock())

    def now(self):
        return self._now()

    def add_log(self, level, message, ts=None):
        """追加网关日志并返回条目（事件推送由调用方统一做）。"""
        with self._lock:
            return self.logs.add(level, message, ts=ts)

    def log_entries(self, limit=None):
        with self._lock:
            return self.logs.recent(limit)

    # ------------------------------------------------------------------
    # 会话
    # ------------------------------------------------------------------
    def set_session(self, snapshot):
        with self._lock:
            merged = default_session_snapshot()
            merged.update(copy.deepcopy(dict(snapshot or {})))
            self._session = merged
            return dict(merged)

    def session_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._session)

    # ------------------------------------------------------------------
    # robot / nav
    # ------------------------------------------------------------------
    def set_robot(self, robot):
        with self._lock:
            self._robot = None if robot is None else copy.deepcopy(robot)

    def robot_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._robot)

    def set_nav(self, **fields):
        with self._lock:
            for key, value in fields.items():
                if key not in self._nav:
                    self._nav[key] = value
                else:
                    self._nav[key] = copy.deepcopy(value)
            return copy.deepcopy(self._nav)

    def nav_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._nav)

    def reset_nav(self):
        with self._lock:
            self._nav = default_nav_snapshot()
            return copy.deepcopy(self._nav)

    # ------------------------------------------------------------------
    # 地图
    # ------------------------------------------------------------------
    def set_map(self, grid, received_ts=None):
        """更新地图快照；内容 hash 变化才递增 version。

        返回 ``(changed, meta)``；``changed=True`` 时调用方应推送 ``map`` 事件。
        """
        with self._lock:
            stamp = self._now() if received_ts is None else float(received_ts)
            self._map_received_ts = stamp
            new_hash = grid.content_hash()
            changed = new_hash != self._map_hash
            if changed:
                self._map_version += 1
                self._map_hash = new_hash
                self._map_grid = grid
                self._map_meta = grid.meta(self._map_version)
                meta = copy.deepcopy(self._map_meta)
            else:
                meta = copy.deepcopy(self._map_meta)
            return changed, meta

    def map_grid(self):
        with self._lock:
            return self._map_grid

    def map_version(self):
        with self._lock:
            return self._map_version

    def map_snapshot(self):
        with self._lock:
            if self._map_meta is None:
                return {
                    'available': False,
                    'version': 0,
                    'width': None,
                    'height': None,
                    'resolution': None,
                    'origin': None,
                    'frame_id': None,
                    'stamp': None,
                }
            meta = copy.deepcopy(self._map_meta)
            meta['available'] = True
            return meta

    # ------------------------------------------------------------------
    # 急停 / 遥控 / 外设
    # ------------------------------------------------------------------
    def set_estop(self, active, since=None):
        """设置急停状态；返回是否发生变化。

        ``active=True`` 且已经激活时保留原 ``since``（诊断 1 Hz 重报不刷新时间）。
        """
        with self._lock:
            active = bool(active)
            old = self._estop['active']
            if active:
                if old and self._estop.get('since') is not None:
                    effective_since = self._estop['since']
                elif since is not None:
                    effective_since = float(since)
                else:
                    effective_since = self._now()
                self._estop = {'active': True, 'since': effective_since}
            else:
                self._estop = {'active': False, 'since': None}
            return old != active

    def estop_snapshot(self):
        with self._lock:
            return dict(self._estop)

    def estop_active(self):
        with self._lock:
            return bool(self._estop['active'])

    def mark_estop_commanded(self):
        """标记网关已主动发过 /estop 命令（之后不再用诊断同步真实值）。"""
        with self._lock:
            self._estop_commanded = True

    @property
    def estop_commanded(self):
        with self._lock:
            return self._estop_commanded

    def set_teleop(self, active, vx=0.0, vy=0.0, wz=0.0, last_cmd=None,
                   seq=None):
        with self._lock:
            self._teleop = {
                'active': bool(active),
                'vx': float(vx),
                'vy': float(vy),
                'wz': float(wz),
                'last_cmd': None if last_cmd is None else float(last_cmd),
            }
            if seq is not None:
                self._teleop_seq = int(seq)
            return dict(self._teleop)

    def get_teleop_seq(self):
        with self._lock:
            return self._teleop_seq

    def consume_teleop(self, now, timeout_s):
        """20 Hz 定时器取点：返回 ``(status, (vx, vy, wz))``。

        - ``status='idle'``：遥控未激活，无需发布；
        - ``status='expired'``：超过看门狗（默认 0.5 s），已自动置
          ``active=false``，调用方应发两次零速后停止发布（§5.1）；
        - ``status='active'``：返回当前速度设定值。
        """
        with self._lock:
            if not self._teleop['active']:
                return 'idle', None
            last = self._teleop['last_cmd']
            if last is None or (float(now) - last) > float(timeout_s):
                self._teleop = {'active': False, 'vx': 0.0, 'vy': 0.0,
                                'wz': 0.0, 'last_cmd': last}
                return 'expired', None
            return 'active', (self._teleop['vx'], self._teleop['vy'],
                              self._teleop['wz'])

    def teleop_snapshot(self):
        with self._lock:
            return dict(self._teleop)

    def stop_teleop(self):
        with self._lock:
            self._teleop = {'active': False, 'vx': 0.0, 'vy': 0.0, 'wz': 0.0,
                            'last_cmd': None}
            return dict(self._teleop)

    def set_peripherals(self, buzzer=None, led=None, servo=None):
        with self._lock:
            if buzzer is not None:
                self._peripherals['buzzer'] = bool(buzzer)
            if led is not None:
                self._peripherals['led'] = {
                    'r': float(led.get('r', self._peripherals['led']['r'])),
                    'g': float(led.get('g', self._peripherals['led']['g'])),
                    'b': float(led.get('b', self._peripherals['led']['b'])),
                }
            if servo is not None:
                self._peripherals['servo'] = {
                    'pan': float(servo.get('pan',
                                           self._peripherals['servo']['pan'])),
                    'tilt': float(servo.get(
                        'tilt', self._peripherals['servo']['tilt'])),
                }
            return copy.deepcopy(self._peripherals)

    def peripherals_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._peripherals)

    # ------------------------------------------------------------------
    # 诊断 / ROS 图 / 速率
    # ------------------------------------------------------------------
    def set_diagnostics(self, items, received_ts=None):
        """写入最近一次 /diagnostics 条目并完成聚合。"""
        with self._lock:
            aggregate = aggregate_diagnostics(items)
            self._diagnostics = {
                'level': aggregate['level'],
                'counts': aggregate['counts'],
                'items': aggregate['items'],
                'received_ts': float(self._now()
                                     if received_ts is None else received_ts),
            }
            return copy.deepcopy(self._diagnostics)

    def diagnostics_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._diagnostics)

    def set_ros(self, nodes=None, ready=None, ok=None):
        with self._lock:
            if nodes is not None:
                self._ros['nodes'] = int(nodes)
            if ready is not None:
                merged = default_ready()
                merged.update({k: bool(v) for k, v in dict(ready).items()})
                self._ros['ready'] = merged
            if ok is not None:
                self._ros['ok'] = bool(ok)
            return copy.deepcopy(self._ros)

    def ros_snapshot(self):
        with self._lock:
            return copy.deepcopy(self._ros)

    def tick_scan(self, now=None):
        self._rate_scan.tick(now)

    def tick_odom(self, now=None):
        self._rate_odom.tick(now)

    def scan_rate(self, now=None):
        return self._rate_scan.rate(now)

    def odom_rate(self, now=None):
        return self._rate_odom.rate(now)

    def map_age(self, now=None):
        with self._lock:
            if self._map_received_ts is None:
                return None
            return max(0.0, float(self._now() if now is None else now)
                       - self._map_received_ts)

    def diagnostics_age(self, now=None):
        with self._lock:
            received = self._diagnostics.get('received_ts')
            if received is None:
                return None
            return max(0.0, float(self._now() if now is None else now)
                       - received)

    # ------------------------------------------------------------------
    # 快照
    # ------------------------------------------------------------------
    def snapshot(self, now=None):
        """返回设计文档 §3.1 的完整状态快照（可直接 JSON 序列化）。"""
        with self._lock:
            now = self._now() if now is None else float(now)
            map_age = self.map_age(now)
            diag_age = self.diagnostics_age(now)
            diag = copy.deepcopy(self._diagnostics)
            diag['age_s'] = diag_age
            nav = copy.deepcopy(self._nav)
            snapshot = {
                'api': protocol.API_VERSION,
                'server_time': now,
                'uptime_s': max(0.0, now - self._started_wall),
                'session': copy.deepcopy(self._session),
                'ros': {
                    'ok': bool(self._ros['ok']),
                    'nodes': int(self._ros['nodes']),
                    'ready': copy.deepcopy(self._ros['ready']),
                    'rates': {
                        'scan_hz': round(float(self._rate_scan.rate()), 1),
                        'odom_hz': round(float(self._rate_odom.rate()), 1),
                        'map_age_s': map_age,
                        'diagnostics_age_s': diag_age,
                    },
                },
                'limits': dict(self.limits),
                'robot': copy.deepcopy(self._robot),
                'nav': nav,
                'map': self.map_snapshot(),
                'estop': dict(self._estop),
                'teleop': dict(self._teleop),
                'peripherals': copy.deepcopy(self._peripherals),
                'diagnostics': diag,
                'logs': self.logs.recent(self.log_recent),
            }
            return protocol.sanitize(snapshot)
