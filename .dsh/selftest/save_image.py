"""Subscribe /perception/image and save one frame to disk.

Usage: save_image.py <topic> <output.jpg> [--timeout 20]
"""
import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Saver(Node):
    def __init__(self, topic, out):
        super().__init__('image_saver')
        self.out = out
        self.saved = False
        self.sub = self.create_subscription(
            Image, topic, self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        if self.saved:
            return
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == 'bgr8':
                img = arr.reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                img = arr.reshape(msg.height, msg.width, 3)[:, :, ::-1]
            else:
                print(f'[FAIL] unsupported encoding {msg.encoding}')
                return
            cv2.imwrite(self.out, img)
            self.saved = True
            print(f'[OK] saved {self.out} ({msg.width}x{msg.height} {msg.encoding})')
        except Exception as exc:
            print(f'[FAIL] save error: {exc}')

    def run(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok() and not self.saved:
            rclpy.spin_once(self, timeout_sec=0.1)
        return 0 if self.saved else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('topic')
    ap.add_argument('out')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    n = Saver(args.topic, args.out)
    rc = n.run(args.timeout)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
