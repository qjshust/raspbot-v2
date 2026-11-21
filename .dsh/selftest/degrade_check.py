"""FR-E-02/SYS-S03: IMU 掉线降级测试（配合外部 kill imu_node 使用）。

注入 /cmd_vel_teleop = 0.2 m/s（> degraded 0.15），采样 /cmd_vel_safe 2 s，
期望输出被压到 degraded_max_linear（0.15）以内，且无异常尖峰。
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DegradeCheck(Node):
    def __init__(self):
        super().__init__('degrade_check')
        self.pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.safe = []
        self.create_subscription(Twist, '/cmd_vel_safe',
                                 lambda m: self.safe.append(
                                     (time.monotonic(), m.linear.x)), 10)

    def run(self, inject, seconds):
        m = Twist()
        m.linear.x = float(inject)
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds and rclpy.ok():
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(1.0 / 30.0)
        m.linear.x = 0.0
        self.pub.publish(m)
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.2)
        rclpy.spin_once(self, timeout_sec=0.2)


def main():
    rclpy.init()
    n = DegradeCheck()
    n.run(0.2, 2.5)
    vals = [v for t, v in n.safe if v > 0.001]
    if not vals:
        print('[FAIL] 未收到任何非零 /cmd_vel_safe')
        n.destroy_node(); rclpy.shutdown(); sys.exit(1)
    mx = max(vals)
    print(f'  注入 0.2 m/s，/cmd_vel_safe 观测最大 {mx:.4f} m/s（样本 {len(vals)}）')
    ok = mx <= 0.15 + 0.02
    print(f"[{'PASS' if ok else 'FAIL'}] IMU 掉线降级：输出 ≤ degraded(0.15)+0.02"
          f"（观测 {mx:.4f}）")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
