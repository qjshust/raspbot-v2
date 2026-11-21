"""SW-F01: 指令超时停车测量。

发布 0.2 m/s 到 /cmd_vel_teleop 2 s → 停止发布 → 测量从最后一条指令到
/cmd_vel_safe 归零的时延（链：twist_mux → limiter(0.4 s 输入超时) → base）。
判据 J-07：≤0.54 s。同时检查 /odom_raw.twist 归零。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SWF01(Node):
    def __init__(self):
        super().__init__('swf01')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.safe_events = []          # (t, |v|)
        self.odom_events = []          # (t, |v|)
        self.last_pub_t = 0.0
        self.create_subscription(Twist, '/cmd_vel_safe', self._safe, 10)
        self.create_subscription(Odometry, '/odom_raw', self._odom, 10)

    def _safe(self, m):
        self.safe_events.append((time.monotonic(), abs(m.linear.x)))

    def _odom(self, m):
        self.odom_events.append((time.monotonic(), abs(m.twist.twist.linear.x)))

    def publish(self, vx, dur):
        m = Twist()
        m.linear.x = float(vx)
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur and rclpy.ok():
            self.pub.publish(m)
            self.last_pub_t = time.monotonic()
            time.sleep(1.0 / 30.0)
        # 关键：最后一条指令保持非零，直接停止发布（真实“停发”故障注入）

    def spin_until(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = SWF01()
    print('== 注入 0.2 m/s × 2 s ==')
    node.publish(0.2, 2.0)
    t_stop = node.last_pub_t
    print(f'  最后指令时刻 t={t_stop:.3f}')
    node.spin_until(3.0)

    safe_after = [t for t, v in node.safe_events if t >= t_stop - 0.05]
    safe_zero = [t for t, v in node.safe_events
                 if v < 1e-4 and t >= t_stop - 0.05]
    dt_safe = (min(safe_zero) - t_stop) if safe_zero else None
    print(f'  /cmd_vel_safe 归零时延: {dt_safe:.3f} s' if dt_safe is not None
          else '  /cmd_vel_safe 未归零 [FAIL]')

    odom_after = [t for t, v in node.odom_events if t >= t_stop - 0.05]
    odom_zero = [t for t, v in node.odom_events
                 if v < 1e-4 and t >= t_stop - 0.05]
    dt_odom = (min(odom_zero) - t_stop) if odom_zero else None
    print(f'  /odom_raw.twist 归零时延: {dt_odom:.3f} s' if dt_odom is not None
          else '  /odom_raw 未归零 [FAIL]')

    ok = (dt_safe is not None and dt_safe <= 0.54
          and dt_odom is not None and dt_odom <= 0.60)
    print(f"[{'PASS' if ok else 'FAIL'}] SW-F01 超时停车 "
          f"(safe={dt_safe if dt_safe is not None else 'N/A'} s, "
          f"odom={dt_odom if dt_odom is not None else 'N/A'} s, 判据 ≤0.54 s)")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
