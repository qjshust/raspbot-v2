"""SW-P03: /odom_raw 周期抖动统计（≥1000 样本）。

输出：样本数、均值间隔、p50/p95/p99（ms）、最大间隔、连续超 1.5 倍周期次数。
判据：均值 33.3 ms，p99 ≤ 36.6 ms（+10%）。
"""
import statistics
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Jitter(Node):
    def __init__(self, target):
        super().__init__('jitter_check')
        self.target = target
        self.stamps = []
        self.sub = self.create_subscription(Odometry, '/odom_raw',
                                            self._cb, 10)

    def _cb(self, m):
        self.stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if len(self.stamps) >= self.target:
            self.sub.destroy()

    def run(self):
        while rclpy.ok() and len(self.stamps) < self.target:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    target = 1000
    rclpy.init()
    n = Jitter(target)
    n.run()
    s = n.stamps
    if len(s) < 100:
        print(f'[FAIL] 样本不足 {len(s)}')
        n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    dts = [1000 * (b - a) for a, b in zip(s, s[1:])]
    mean = statistics.mean(dts)
    p50 = statistics.median(dts)
    p95 = sorted(dts)[int(len(dts) * 0.95) - 1]
    p99 = sorted(dts)[int(len(dts) * 0.99) - 1]
    runs = 0
    cur = 0
    for d in dts:
        if d > 1.5 * 33.3:
            cur += 1
            runs = max(runs, cur)
        else:
            cur = 0
    print(f'  n={len(s)} mean={mean:.2f} ms p50={p50:.2f} p95={p95:.2f} '
          f'p99={p99:.2f} max={max(dts):.2f} 连续超限峰值={runs}')
    ok = mean <= 33.3 * 1.10 and p99 <= 36.6
    print(f"[{'PASS' if ok else 'FAIL'}] SW-P03 控制周期抖动 "
          f"(mean≤36.6, p99≤36.6 ms)")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
