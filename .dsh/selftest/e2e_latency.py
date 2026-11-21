"""SW-P04: 端到端软件链路时延（发布 → 积分 → /odom）。

方法（测试方案 §5.7）：5 Hz 方波发 /cmd_vel_teleop（0 ↔ 0.1 m/s）30 s，
对每个上升沿找 /odom.twist.linear.x 首次 >0.01 的时刻，差值即时延。
取中位数与 p95（排除前 3 个样本）。判据：中位 <100 ms、p95 <150 ms。
"""
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class E2E(Node):
    def __init__(self):
        super().__init__('e2e_check')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.edges = []   # rising edge publish times
        self.odom = []    # (t, vx)
        self.create_subscription(Odometry, '/odom', self._odom, 10)

    def _odom(self, m):
        self.odom.append((time.monotonic(), m.twist.twist.linear.x))

    def run(self, seconds, period):
        m = Twist()
        t0 = time.monotonic()
        last_high = False
        while time.monotonic() - t0 < seconds and rclpy.ok():
            high = int((time.monotonic() - t0) / period) % 2 == 0
            m.linear.x = 0.1 if high else 0.0
            self.pub.publish(m)
            if high and not last_high:
                self.edges.append(time.monotonic())
            last_high = high
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(1.0 / 30.0)
        m.linear.x = 0.0
        self.pub.publish(m)


def main():
    rclpy.init()
    n = E2E()
    n.run(30.0, 0.2)   # 5 Hz 方波 = 0↔0.1 m/s，周期 0.2 s
    time.sleep(0.5)
    rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.spin_once(n, timeout_sec=0.2)

    lats = []
    for edge in n.edges[3:]:
        hit = None
        for t, v in n.odom:
            if t >= edge - 0.01 and v > 0.01:
                hit = t - edge
                break
        if hit is not None:
            lats.append(1000 * hit)
    if len(lats) < 5:
        print(f'[FAIL] 有效上升沿样本不足（{len(lats)}）')
        n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    med = statistics.median(lats)
    p95 = sorted(lats)[int(len(lats) * 0.95) - 1]
    print(f'  上升沿 {len(n.edges)} 个，有效时延样本 {len(lats)} 个：'
          f'median={med:.0f} ms p95={p95:.0f} ms min={min(lats):.0f} '
          f'max={max(lats):.0f}')
    ok = med < 100 and p95 < 150
    print(f"[{'PASS' if ok else 'FAIL'}] SW-P04 端到端时延 "
          f"(median<100 ms, p95<150 ms)")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
