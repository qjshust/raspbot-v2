"""Grab camera frames with the gimbal tilted down to see the wheels.

Usage: wheel_cam.py --tilt 110 --pan 0 60 120 180 --outdir /tmp/wheelcam
Publishes /servo_pan /servo_tilt then saves frames.
"""
import argparse
import os
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class WheelCam(Node):
    def __init__(self, pans, tilt, outdir):
        super().__init__('wheel_cam')
        self.pan_pub = self.create_publisher(Float32, '/servo_pan', 10)
        self.tilt_pub = self.create_publisher(Float32, '/servo_tilt', 10)
        self.pans = pans
        self.tilt = tilt
        self.outdir = outdir

    def set_pan(self, v):
        m = Float32(); m.data = float(v)
        for _ in range(3):
            self.pan_pub.publish(m); time.sleep(0.12)

    def set_tilt(self, v):
        m = Float32(); m.data = float(v)
        for _ in range(3):
            self.tilt_pub.publish(m); time.sleep(0.12)

    def grab(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            return None
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            for _ in range(5):
                cap.read()
            time.sleep(0.8)
            ok, frame = cap.read()
            return frame if ok else None
        finally:
            cap.release()

    def run(self):
        self.set_tilt(self.tilt)
        time.sleep(1.0)
        for pan in self.pans:
            self.set_pan(pan)
            time.sleep(0.8)
            f = self.grab()
            if f is None:
                print(f'pan {pan}: [FAIL] no frame')
                return 1
            p = os.path.join(self.outdir, f'pan{int(pan)}_tilt{int(self.tilt)}.jpg')
            cv2.imwrite(p, f)
            print(f'pan {pan}: {p} {f.shape}')
        self.set_pan(90)
        self.set_tilt(60)
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pans', nargs='+', type=float, default=[0, 60, 120, 180])
    ap.add_argument('--tilt', type=float, default=110.0)
    ap.add_argument('--outdir', default='/tmp/wheelcam')
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    rclpy.init()
    n = WheelCam(args.pans, args.tilt, args.outdir)
    rc = n.run()
    n.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
