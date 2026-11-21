"""SYS-S02 / FR-E-04: estop 锁存测试（事件驱动版）。

流程：
  0. 发 teleop 0.2 m/s 1 s → 确认 /cmd_vel_safe 升到 ≈0.2（指令链健全）
  1. 持续发 teleop 0.2 + 持续发 estop true（5 Hz）→ 测从首个 estop 消息到
     /cmd_vel_safe ≤0.01 m/s 的时延（J-07 判据 ≤0.3 s）
  2. 停发 estop（仍锁存）继续发 teleop → safe 保持 0（锁存语义）
  3. 持续发 estop false → safe 重新升到 ≥0.05（解除恢复）
"""
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class EstopCheck(Node):
    def __init__(self):
        super().__init__('estop_check')
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self.safe = []          # (t_mono, vx)
        self.create_subscription(Twist, '/cmd_vel_safe',
                                 lambda m: self.safe.append(
                                     (time.monotonic(), m.linear.x)), 10)

    def cmd_loop(self, vx, dur, extra=None):
        """持续发布 teleop；extra 为可选并行的 estop 发布函数。
        期间每个循环 spin_once 以接收 safe 消息。"""
        m = Twist()
        m.linear.x = float(vx)
        t0 = time.monotonic()
        while time.monotonic() - t0 < dur and rclpy.ok():
            self.pub_cmd.publish(m)
            if extra is not None:
                extra()
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)
        m.linear.x = 0.0
        self.pub_cmd.publish(m)

    def spin(self, sec):
        d = time.monotonic() + sec
        while time.monotonic() < d and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)

    def max_since(self, t0):
        vals = [v for t, v in self.safe if t >= t0]
        return max(vals, default=None)

    def min_since(self, t0):
        vals = [v for t, v in self.safe if t >= t0]
        return min(vals, default=None)


def main():
    rclpy.init()
    n = EstopCheck()
    print('== 预热：等待 DDS 发现完成（3 s）==')
    n.spin(3.0)
    print('== 0. 指令链健全：teleop 0.2 m/s × 1.5 s，safe 应升到 ≈0.2 ==')
    t0 = time.monotonic()
    n.cmd_loop(0.2, 1.5)
    n.spin(0.3)
    mx0 = n.max_since(t0)
    print(f'  safe 最大值 {mx0:.3f} m/s' if mx0 is not None else '  无样本 [FAIL]')
    ok0 = mx0 is not None and mx0 > 0.10

    print('== 1. estop true（5 Hz 持续）→ safe 归零时延 ≤0.3 s ==')
    m_estop = Bool()
    m_estop.data = True
    t1 = time.monotonic()
    n.cmd_loop(0.2, 1.5, extra=lambda: n.pub_estop.publish(m_estop))
    # 从 t1 起找第一个 safe ≤0.01 的时刻
    zero_t = None
    for t, v in n.safe:
        if t >= t1 and v <= 0.01:
            zero_t = t
            break
    dt1 = (zero_t - t1) if zero_t is not None else None
    print(f'  estop→归零时延 {dt1:.3f} s' if dt1 is not None else '  未归零 [FAIL]')
    ok1 = dt1 is not None and dt1 <= 0.3

    print('== 2. 锁存：停发 estop，继续 teleop 0.2 → safe 保持 0 ==')
    t2 = time.monotonic()
    n.cmd_loop(0.2, 1.0)
    n.spin(0.2)
    mx2 = n.max_since(t2)
    print(f'  锁存期间 safe 最大值 {mx2:.4f} m/s' if mx2 is not None else '  无样本')
    ok2 = mx2 is not None and mx2 < 1e-3

    print('== 3. estop false（5 Hz 持续）→ safe 恢复 ≥0.05 ==')
    m_off = Bool()
    m_off.data = False
    t3 = time.monotonic()
    n.cmd_loop(0.2, 1.5, extra=lambda: n.pub_estop.publish(m_off))
    n.spin(0.2)
    mx3 = n.max_since(t3)
    print(f'  恢复后 safe 最大值 {mx3:.4f} m/s' if mx3 is not None else '  无样本')
    ok3 = mx3 is not None and mx3 > 0.05

    # 收尾：零指令 + 解除
    m_zero = Twist()
    n.pub_cmd.publish(m_zero)
    n.spin(0.2)

    ok = ok0 and ok1 and ok2 and ok3
    print(f"[{'PASS' if ok else 'FAIL'}] estop 锁存测试 "
          f"(0:{ok0} 1:{ok1} 2:{ok2} 3:{ok3})")
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
