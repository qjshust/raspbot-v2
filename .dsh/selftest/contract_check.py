"""Contract checker (SW-C01..C06): topics, QoS, frame_ids, publishers, TF tree.

Run with the full stack up. Prints PASS/FAIL per item, exits nonzero on any
P0 failure. Usage: contract_check.py [--with-camera] [--window 6]
"""
import argparse
import re
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, LaserScan, Range
from std_msgs.msg import UInt8
from tf2_msgs.msg import TFMessage

FAILS = []
SENSOR_TOPICS = {'/scan', '/imu/data', '/range', '/image_raw'}
TF_STATIC_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def check(name, ok, detail=''):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}")
    if not ok:
        FAILS.append(name)


class Checker(Node):
    def __init__(self, window, with_camera):
        super().__init__('contract_checker')
        self.window = window
        self.with_camera = with_camera
        self.counts = {}
        self.frames = {}
        self.child_frames = {}
        self.tf = {}
        self.tf_static = {}
        subs = {
            '/scan': (LaserScan, None),
            '/imu/data': (Imu, None),
            '/odom_raw': (Odometry, None),
            '/odom': (Odometry, None),
            '/range': (Range, None),
            '/line_sensor': (UInt8, None),
            '/diagnostics': (DiagnosticArray, None),
            '/tf': (TFMessage, None),
            '/tf_static': (TFMessage, None),
        }
        if with_camera:
            subs['/image_raw'] = (Image, None)
        self._subs = {}
        for topic, (typ, _) in subs.items():
            if topic in SENSOR_TOPICS:
                # 深队列 BestEffort：C++ 节点发布节奏更紧凑，浅队列会因
                # python 执行器来不及取而被丢弃 → 频率误判（节点本身频率正常）
                qos = QoSProfile(
                    depth=1000,
                    reliability=ReliabilityPolicy.BEST_EFFORT)
            elif topic == '/tf_static':
                qos = TF_STATIC_QOS
            else:
                qos = 100
            self._subs[topic] = self.create_subscription(
                typ, topic,
                lambda m, t=topic: self._on_msg(t, m), qos)

    def _on_msg(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        if topic in ('/scan', '/imu/data', '/image_raw', '/range'):
            self.frames[topic] = msg.header.frame_id
        if topic in ('/odom_raw', '/odom'):
            self.frames[topic] = msg.header.frame_id
            self.child_frames[topic] = msg.child_frame_id
        if topic in ('/tf', '/tf_static'):
            target = self.tf if topic == '/tf' else self.tf_static
            for tr in msg.transforms:
                key = (tr.header.frame_id, tr.child_frame_id)
                target[key] = target.get(key, 0) + 1

    def run(self):
        deadline = time.monotonic() + self.window
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

    def hz(self, topic):
        return self.counts.get(topic, 0) / self.window


def qos_label(topic):
    import subprocess
    out = subprocess.run(
        ['ros2', 'topic', 'info', '-v', topic], capture_output=True,
        text=True, timeout=15)
    txt = out.stdout + out.stderr
    rel = 'BEST_EFFORT' if 'BEST_EFFORT' in txt else ('RELIABLE' if 'RELIABLE' in txt else '?')
    dur = 'TRANSIENT_LOCAL' if 'TRANSIENT_LOCAL' in txt else ('VOLATILE' if 'VOLATILE' in txt else '?')
    pubs = '?'
    for line in txt.splitlines():
        if 'Publisher count:' in line:
            pubs = line.split(':')[1].strip()
    return rel, dur, pubs


def measure_rates(topics, seconds=8.0):
    """用 ros2 topic hz（C++ 计数器）并行实测各话题频率。

    python 执行器在满载订阅下取数不及，会低估频率（节点本身正常）；
    CLI 计数器是权威测量。返回 {topic: hz or None}。
    """
    import concurrent.futures
    import subprocess

    def one(topic):
        try:
            out = subprocess.run(
                ['timeout', '-s', 'KILL', str(int(seconds)),
                 'ros2', 'topic', 'hz', topic],
                capture_output=True, text=True, timeout=seconds + 15)
            txt = out.stdout + out.stderr
            rates = re.findall(r'average rate:\s*([0-9.]+)', txt)
            return topic, float(rates[-1]) if rates else None
        except Exception:
            return topic, None

    result = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(topics)) as ex:
        for topic, hz in ex.map(one, topics):
            result[topic] = hz
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--with-camera', action='store_true')
    ap.add_argument('--window', type=float, default=6.0)
    args = ap.parse_args()
    rclpy.init()
    node = Checker(args.window, args.with_camera)
    node.run()

    print('== 1. 话题存在与频率（Hz）==')
    expect = {'/scan': (9.0, 11.0), '/imu/data': (90.0, 110.0),
              '/odom_raw': (27.0, 33.0), '/odom': (27.0, 33.0),
              '/range': (4.0, 6.0), '/line_sensor': (4.0, 6.0),
              '/diagnostics': (0.8, 1.4)}
    if args.with_camera:
        expect['/image_raw'] = (1.0, 60.0)
    rates = measure_rates(list(expect.keys()))
    for topic, (lo, hi) in expect.items():
        h = rates.get(topic)
        if h is None:
            check(f'{topic} hz=无消息 ({lo}~{hi})', False)
        else:
            ok = lo <= h <= hi
            check(f'{topic} hz={h:.2f} ({lo}~{hi})', ok)

    print('== 2. QoS（记录口径，J-13）与单发布者 ==')
    for topic in ('/scan', '/imu/data', '/odom_raw', '/odom', '/cmd_vel',
                  '/cmd_vel_safe', '/range', '/line_sensor', '/diagnostics',
                  '/tf', '/tf_static'):
        try:
            rel, dur, pubs = qos_label(topic)
        except Exception as exc:
            check(f'{topic} QoS 查询', False, f'异常 {exc}')
            continue
        # J-13：QoS 以实测为基线记录，不硬判（laser_filters 链为 Reliable 属已知差异）
        print(f'  [INFO] {topic}: QoS={rel}/{dur} publishers={pubs}')
        if topic in ('/scan', '/imu/data', '/odom_raw', '/odom',
                     '/cmd_vel', '/cmd_vel_safe', '/diagnostics',
                     '/tf_static', '/range', '/line_sensor'):
            check(f'{topic} 单发布者', pubs == '1', f'publishers={pubs}')
        elif topic == '/tf':
            # robot_state_publisher 会注册 /tf 发布器（固定关节全部走 /tf_static，
            # /tf 无实际消息）+ ekf_node 发动态 TF；判据为“无重复变换”而非发布器数
            check(f'{topic} 发布者≤2（RSP 占位 + EKF）', pubs in ('1', '2'),
                  f'publishers={pubs}')

    print('== 3. frame_id（J-01/J-09）==')
    check('/scan frame_id=laser', node.frames.get('/scan') == 'laser',
          str(node.frames.get('/scan')))
    check('/imu/data frame_id=imu_link',
          node.frames.get('/imu/data') == 'imu_link',
          str(node.frames.get('/imu/data')))
    check('/odom_raw frame=odom child=base_link',
          node.frames.get('/odom_raw') == 'odom'
          and node.child_frames.get('/odom_raw') == 'base_link',
          f"{node.frames.get('/odom_raw')}/{node.child_frames.get('/odom_raw')}")
    check('/odom frame=odom child=base_link',
          node.frames.get('/odom') == 'odom'
          and node.child_frames.get('/odom') == 'base_link',
          f"{node.frames.get('/odom')}/{node.child_frames.get('/odom')}")
    check('/range frame_id=ultrasonic',
          node.frames.get('/range') == 'ultrasonic',
          str(node.frames.get('/range')))
    if args.with_camera:
        check('/image_raw frame_id=camera_link',
              node.frames.get('/image_raw') == 'camera_link',
              str(node.frames.get('/image_raw')))

    print('== 4. TF 树（唯一权威）==')
    static_expect = {('base_link', 'laser'), ('base_link', 'imu_link'),
                     ('base_link', 'camera_link'),
                     ('base_link', 'ultrasonic'),
                     ('base_link', 'base_footprint')}
    have_static = set(node.tf_static.keys())
    for pair in static_expect:
        check(f'静态 TF {pair[0]}->{pair[1]}', pair in have_static)
    check('odom->base_link 为动态 TF（/tf，来自 EKF）',
          ('odom', 'base_link') in node.tf, str(sorted(node.tf.keys())))
    check('odom->base_link 不在 /tf_static（禁止静态占位）',
          ('odom', 'base_link') not in have_static)

    node.destroy_node()
    rclpy.shutdown()
    print()
    if FAILS:
        print(f'[FAIL] contract_check 失败 {len(FAILS)} 项: {FAILS}')
        sys.exit(1)
    print('[PASS] contract_check 全部通过')


if __name__ == '__main__':
    main()
