"""SIL 导航一体诊断：激活 + initialpose + 目标 + 全链路采样。

单进程编排，避免 shell/CLI 抖动；每 2 s 打印各环数值。
Usage: sil_orchestrator.py [--goal-x 1.5] [--duration 90]
"""
import argparse
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from nav2_msgs.msg import CollisionMonitorState
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


def sh(cmd, timeout=15):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout)
        return (r.stdout + r.stderr).strip()
    except subprocess.TimeoutExpired:
        return 'TIMEOUT'


class Orchestrator(Node):
    def __init__(self, goal_x, duration):
        super().__init__('sil_orch')
        self.goal_x = goal_x
        self.duration = duration
        self.last = {'raw': 0.0, 'nav': 0.0, 'safe': 0.0, 'odom_x': 0.0,
                     'plan_len': 0, 'cm': 'n/a'}
        self.create_subscription(Twist, '/cmd_vel_nav_raw',
                                 lambda m: self.last.__setitem__('raw', m.linear.x), 10)
        self.create_subscription(Twist, '/cmd_vel_nav',
                                 lambda m: self.last.__setitem__('nav', m.linear.x), 10)
        self.create_subscription(Twist, '/cmd_vel_safe',
                                 lambda m: self.last.__setitem__('safe', m.linear.x), 10)
        self.create_subscription(Odometry, '/odom',
                                 lambda m: self.last.__setitem__('odom_x', m.pose.pose.position.x), 10)
        self.create_subscription(Path, '/plan',
                                 lambda m: self.last.__setitem__('plan_len', len(m.poses)), 10)
        self.create_subscription(
            CollisionMonitorState, '/collision_monitor_state',
            lambda m: self.__setattr__('cm', ','.join(p.action_type for p in m.polygons)), 10)
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def wait_goal_server(self, timeout=60.0):
        return self.goal_client.wait_for_server(timeout)

    def send_goal(self):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x = self.goal_x
        g.pose.pose.orientation.w = 1.0
        future = self.goal_client.send_goal_async(g)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        handle = future.result()
        if handle is None:
            print('[FAIL] goal rejected')
            return None
        print('[OK] goal accepted')
        return handle

    def spin_with_print(self, seconds, handle):
        deadline = time.monotonic() + seconds
        last_print = 0.0
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_print >= 2.0:
                st = ''
                if handle is not None:
                    r = handle.get_result()
                    if r is not None:
                        st = f" nav2_status={r.status} code={r.result.error_code}"
                print(f"t={now:.0f} raw={self.last['raw']:.3f} "
                      f"nav={self.last['nav']:.3f} safe={self.last['safe']:.3f} "
                      f"odom_x={self.last['odom_x']:.3f} plan={self.last['plan_len']} "
                      f"cm=[{self.cm}]{st}")
                last_print = now

    def result(self, handle):
        r = handle.get_result()
        if r is not None:
            print(f'[RESULT] status={r.status} error_code={r.result.error_code}')
            return r.status == 4
        print('[RESULT] unknown (timeout)')
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--goal-x', type=float, default=1.5)
    ap.add_argument('--duration', type=float, default=90.0)
    args = ap.parse_args()
    rclpy.init()
    n = Orchestrator(args.goal_x, args.duration)

    print('== 等待 amcl active ==')
    for i in range(60):
        if 'active [3]' in sh('ros2 lifecycle get /amcl'):
            print(f'  amcl active（第 {i+1} 轮）')
            break
        time.sleep(2)

    print('== 连续发布 initialpose（3 次，间隔 3 s，防 AMCL 订阅未就绪丢消息）==')
    from geometry_msgs.msg import PoseWithCovarianceStamped as P
    for _ in range(3):
        m = P()
        m.header.frame_id = 'map'
        m.header.stamp = n.get_clock().now().to_msg()
        m.pose.pose.orientation.w = 1.0
        m.pose.covariance = [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0,
                             0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]
        pub = n.create_publisher(P, '/initialpose', 10)
        for _ in range(3):
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(3)

    print('== 等 map→base_link ==')
    import tf2_ros
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, n)
    ok_tf = False
    for i in range(40):
        rclpy.spin_once(n, timeout_sec=0.1)
        try:
            tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            ok_tf = True
            print(f'  map→base_link OK（第 {i+1} 轮）')
            break
        except Exception:
            time.sleep(0.5)

    print('== 补激活 Nav2 ==')
    print(sh('bash ~/tools/selftest/activate_nav2.sh', timeout=180))

    print('== 发目标 ==')
    if not n.wait_goal_server(60.0):
        print('[FAIL] action server 不可用')
        n.destroy_node(); rclpy.shutdown(); return 1
    handle = n.send_goal()
    n.spin_with_print(args.duration, handle)
    ok = n.result(handle) if handle is not None else False
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
