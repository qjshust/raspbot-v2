"""TF poll: wait until a transform (parent->child) is published on /tf.

Direct python (no ros2 CLI wrapper child problem). Usage:
    tf_poll.py odom base_link --timeout 20
Prints FOUND and exits 0 when the transform is seen; exits 1 on timeout.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfPoll(Node):
    def __init__(self, parent, child):
        super().__init__('tf_poll')
        self.parent = parent
        self.child = child
        self.found = False
        self.sub = self.create_subscription(TFMessage, '/tf', self._cb, 100)

    def _cb(self, msg):
        for tr in msg.transforms:
            if tr.header.frame_id == self.parent and tr.child_frame_id == self.child:
                self.found = True

    def run(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok() and not self.found:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.found:
            print(f'FOUND {self.parent}->{self.child}')
            return 0
        print(f'NOT_FOUND {self.parent}->{self.child} in {timeout}s')
        return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('parent')
    ap.add_argument('child')
    ap.add_argument('--timeout', type=float, default=20.0)
    args = ap.parse_args()
    rclpy.init()
    n = TfPoll(args.parent, args.child)
    rc = n.run(args.timeout)
    n.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
