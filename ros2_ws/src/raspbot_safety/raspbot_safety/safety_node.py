"""safety_node — 平台诊断聚合与安全监控（FR-H-03 / SYS-S05 / SYS-S04 可见性）。

以 1 Hz（参数 rate）发布 /diagnostics（diagnostic_msgs/DiagnosticArray）：

1. 聚合：动态发现所有类型为 diagnostic_msgs/DiagnosticArray 且名字不是
   /diagnostics 的话题（get_topic_names_and_types），建订阅并把子 status
   合并进 /diagnostics（名称加话题前缀）。
2. 自身状态：heartbeat、CPU 负载（os.getloadavg）、CPU 使用率（/proc/stat
   两次采样差值）、内存（/proc/meminfo）、SoC 温度（thermal_zone0，m°C→°C）、
   uptime。
3. 安全：/estop 状态；/range 最近 1 s 最小值 <0.35 m → WARN；
   /imu/data(>1 s)、/odom_raw(>0.5 s)、/scan(>0.5 s) 失联 → WARN。

解析与阈值判断全部是纯函数（sysinfo.py / thresholds.py），离线可测。
"""

import os
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Bool

from raspbot_safety import sysinfo
from raspbot_safety import thresholds

DIAG_TYPE = 'diagnostic_msgs/msg/DiagnosticArray'
TEMP_PATH = '/sys/class/thermal/thermal_zone0/temp'
PROC_STAT = '/proc/stat'
PROC_MEMINFO = '/proc/meminfo'
PROC_UPTIME = '/proc/uptime'
PROC_LOADAVG = '/proc/loadavg'


def _qos(depth):
    return QoSProfile(
        depth=depth, reliability=ReliabilityPolicy.RELIABLE)


def _status(name, level, message, hardware_id, values=None):
    status = DiagnosticStatus(
        name=name, level=_norm_level(level), message=message,
        hardware_id=hardware_id)
    if values:
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values]
    return status


def _norm_level(value):
    """诊断 level → bytes(1)。

    Lyrical 新版 rosidl_generator_py 绑定里 DiagnosticStatus.level 是
    bytes 类型（OK=b'\\x00'，WARN=b'\\x01'，ERROR=b'\\x02'）；发布时
    convert_from_py 断言 PyBytes_Check。传 int（旧绑定习惯）会直接崩溃。
    字符串字段则相反：name/message/hardware_id 必须是 str（bytes 会触发
    PyUnicode_Check 断言），保持 str 不动。
    """
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    return bytes([int(value)])


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('rate', 1.0)
        self.declare_parameter('temp_warn_c', thresholds.TEMP_WARN_C)
        self.declare_parameter('cpu_warn_pct', thresholds.CPU_WARN_PCT)
        self.declare_parameter('mem_warn_pct', thresholds.MEM_WARN_PCT)
        self.declare_parameter('range_warn_m', thresholds.RANGE_WARN_M)
        self.declare_parameter('range_window_s', 1.0)
        self.declare_parameter('imu_stale_s', thresholds.IMU_STALE_S)
        self.declare_parameter('odom_stale_s', thresholds.ODOM_STALE_S)
        self.declare_parameter('scan_stale_s', thresholds.SCAN_STALE_S)
        self.declare_parameter('topic_scan_period_s', 5.0)

        def p(name):
            return self.get_parameter(name).value

        self.rate = float(p('rate'))
        self.temp_warn_c = float(p('temp_warn_c'))
        self.cpu_warn_pct = float(p('cpu_warn_pct'))
        self.mem_warn_pct = float(p('mem_warn_pct'))
        self.range_warn_m = float(p('range_warn_m'))
        self.range_window_s = float(p('range_window_s'))
        self.imu_stale_s = float(p('imu_stale_s'))
        self.odom_stale_s = float(p('odom_stale_s'))
        self.scan_stale_s = float(p('scan_stale_s'))
        self.topic_scan_period_s = float(p('topic_scan_period_s'))
        self.discovery_every = max(1, int(round(
            self.topic_scan_period_s * self.rate)))

        # 契约：/diagnostics 1 Hz, Reliable
        self.pub_diag = self.create_publisher(
            DiagnosticArray, '/diagnostics', _qos(10))

        # 安全相关订阅（契约 QoS：/estop Reliable depth 1；传感器 SensorData）
        self.create_subscription(Bool, '/estop', self._on_estop, _qos(1))
        self.create_subscription(
            Range, '/range', self._on_range,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Imu, '/imu/data', self._on_imu,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Odometry, '/odom_raw', self._on_odom, _qos(10))
        self.create_subscription(
            LaserScan, '/scan', self._on_scan,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        # 状态
        self._heartbeat = 0
        self._estop = False
        self._range_samples = deque()          # (monotonic_s, metres)
        self._seen = {'/imu/data': [0, None],  # [count, last_monotonic]
                      '/odom_raw': [0, None],
                      '/scan': [0, None]}
        self._aggregated = {}                  # topic -> DiagnosticArray
        self._diag_subs = {}                   # topic -> subscription
        self._prev_cpu = None
        self._prev_cpu_t = None
        self._ticks = 0

        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(
            'safety_node up: rate=%.1f Hz, publishing /diagnostics' % self.rate)

    # ------------------------------------------------------------------ subs

    def _on_estop(self, msg):
        self._estop = bool(msg.data)

    def _on_range(self, msg):
        now = time.monotonic()
        self._range_samples.append((now, float(msg.range)))
        while (self._range_samples
               and self._range_samples[0][0] < now - self.range_window_s):
            self._range_samples.popleft()

    def _mark(self, topic):
        entry = self._seen[topic]
        entry[0] += 1
        entry[1] = time.monotonic()

    def _on_imu(self, msg):
        self._mark('/imu/data')

    def _on_odom(self, msg):
        self._mark('/odom_raw')

    def _on_scan(self, msg):
        self._mark('/scan')

    def _on_diag(self, topic, msg):
        self._aggregated[topic] = msg

    # ------------------------------------------------------------- discovery

    def _discover(self):
        """动态订阅所有非 /diagnostics 的 DiagnosticArray 话题。"""
        try:
            topics = self.get_topic_names_and_types()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning('topic discovery failed: %s' % exc)
            return
        for name, types in topics:
            if name == '/diagnostics':
                continue
            if DIAG_TYPE in types and name not in self._diag_subs:
                self._diag_subs[name] = self.create_subscription(
                    DiagnosticArray, name,
                    lambda msg, n=name: self._on_diag(n, msg), _qos(10))
                self.get_logger().info(
                    'aggregating diagnostics from %s' % name)

    # ---------------------------------------------------------------- build

    def _read(self, path):
        try:
            with open(path, 'r', encoding='utf-8') as fh:
                return fh.read()
        except OSError as exc:
            self.get_logger().debug('read %s failed: %s' % (path, exc))
            return None

    def _cpu_status(self):
        text = self._read(PROC_STAT)
        cur = sysinfo.parse_cpu_stat(text) if text is not None else None
        usage = None
        if cur is not None and self._prev_cpu is not None:
            usage = sysinfo.cpu_percent(self._prev_cpu, cur)
        if cur is not None:
            self._prev_cpu = cur
            self._prev_cpu_t = time.monotonic()

        load = os.getloadavg()
        level, message = thresholds.cpu_level(usage, self.cpu_warn_pct)
        return _status(
            'Raspbot Platform: CPU', level, message, 'cpu',
            [('usage_pct', '%.1f' % usage if usage is not None else 'n/a'),
             ('load1', '%.2f' % load[0]),
             ('load5', '%.2f' % load[1]),
             ('load15', '%.2f' % load[2])])

    def _mem_status(self):
        text = self._read(PROC_MEMINFO)
        mem = sysinfo.parse_meminfo(text) if text is not None else {}
        used = sysinfo.mem_percent(mem)
        level, message = thresholds.mem_level(used, self.mem_warn_pct)
        return _status(
            'Raspbot Platform: Memory', level, message, 'mem',
            [('used_pct', '%.1f' % used if used is not None else 'n/a'),
             ('memtotal_kb', mem.get('memtotal', 'n/a')),
             ('memavailable_kb', mem.get('memavailable', 'n/a'))])

    def _temp_status(self):
        text = self._read(TEMP_PATH)
        temp = sysinfo.parse_temp_mc(text) if text is not None else None
        level, message = thresholds.temp_level(temp, self.temp_warn_c)
        return _status(
            'Raspbot Platform: Temperature', level, message, 'soc',
            [('temp_c', '%.1f' % temp if temp is not None else 'n/a')])

    def _uptime_status(self):
        text = self._read(PROC_UPTIME)
        up = sysinfo.parse_uptime(text) if text is not None else None
        return _status(
            'Raspbot Platform: Uptime', thresholds.OK,
            sysinfo.fmt_uptime(up), 'system',
            [('uptime_s', up if up is not None else 'n/a')])

    def _heartbeat_status(self):
        self._heartbeat += 1
        return _status(
            'Raspbot Platform: Heartbeat', thresholds.OK,
            'heartbeat %d' % self._heartbeat, 'safety_node',
            [('heartbeat', self._heartbeat)])

    def _estop_status(self):
        level, message = thresholds.estop_level(self._estop)
        return _status('Raspbot Safety: E-Stop', level, message, 'estop',
                       [('estop', self._estop)])

    def _range_status(self):
        min_range = None
        if self._range_samples:
            min_range = min(r for _, r in self._range_samples)
        level, message = thresholds.range_level(min_range, self.range_warn_m)
        return _status(
            'Raspbot Safety: Proximity', level, message, 'ultrasonic',
            [('min_range_m', '%.3f' % min_range if min_range is not None
              else 'n/a'),
             ('window_s', '%.1f' % self.range_window_s)])

    def _sensor_status(self, name, hardware, topic, limit_s):
        count, last = self._seen[topic]
        now = time.monotonic()
        if last is None:
            level, message = thresholds.stale_level(0.0, limit_s, 0, True)
        else:
            level, message = thresholds.stale_level(now - last, limit_s,
                                                    count, False)
        return _status(name, level, message, hardware,
                       [('topic', topic), ('stale_s', '%.1f' % limit_s),
                        ('msg_count', count)])

    # ----------------------------------------------------------------- tick

    def _tick(self):
        self._ticks += 1
        if self._ticks % self.discovery_every == 1:
            self._discover()

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        statuses = [
            self._heartbeat_status(),
            self._cpu_status(),
            self._mem_status(),
            self._temp_status(),
            self._uptime_status(),
            self._estop_status(),
            self._range_status(),
            self._sensor_status('Raspbot Sensors: imu/data', 'imu',
                                '/imu/data', self.imu_stale_s),
            self._sensor_status('Raspbot Sensors: odom_raw', 'base',
                                '/odom_raw', self.odom_stale_s),
            self._sensor_status('Raspbot Sensors: scan', 'lidar',
                                '/scan', self.scan_stale_s),
        ]
        for topic in sorted(self._aggregated):
            for sub in self._aggregated[topic].status:
                # 聚合子诊断：字段原样透传（level 是 bytes、字符串是 str，
                # 新版绑定要求如此），名称加话题前缀避免冲突。
                statuses.append(DiagnosticStatus(
                    name='%s: %s' % (topic, sub.name),
                    level=_norm_level(sub.level),
                    message=sub.message,
                    hardware_id=sub.hardware_id,
                    values=[KeyValue(key=v.key, value=v.value)
                            for v in sub.values]))
        arr.status = statuses
        self.pub_diag.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
