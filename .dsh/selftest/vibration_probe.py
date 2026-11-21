"""Vibration probe: measure HWT101 gyro noise via ROS /imu/data.

Wheels spinning in air shake the chassis; the HWT101 gyro z channel noise
rises measurably. This is an objective proxy for "wheels physically spin"
when no human can watch the car.

Usage:
    vibration_probe.py --seconds 3 --label baseline
    vibration_probe.py --seconds 2 --label wheel0
Prints: label, n samples, std of angular_velocity.z (rad/s), max abs value.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Probe(Node):
    def __init__(self, seconds, label):
        import os as _os
        super().__init__('vibration_probe_%d' % _os.getpid())
        self.values = []
        self.label = label
        self.deadline = time.monotonic() + seconds
        self.sub = self.create_subscription(
            Imu, '/imu/data', self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        self.values.append(msg.angular_velocity.z)

    def run(self):
        while time.monotonic() < self.deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        import math
        import statistics
        n = len(self.values)
        if n < 10:
            print(f'{self.label}: samples={n}  [FAIL] too few samples')
            return 1
        std = statistics.stdev(self.values)
        mx = max(abs(v) for v in self.values)
        print(f'{self.label}: samples={n} std={std:.6f} rad/s max|.|={mx:.6f}')
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--label', default='probe')
    args = ap.parse_args()
    rclpy.init()
    node = Probe(args.seconds, args.label)
    rc = node.run()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
