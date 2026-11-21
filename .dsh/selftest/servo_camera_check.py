"""Servo-via-camera check: drive a gimbal servo and verify the camera scene moves.

The USB camera sits on the pan/tilt gimbal, so pan/tilt motion must change the
scene. Usage:
    servo_camera_check.py --joint pan  --poses 0 90 180
    servo_camera_check.py --joint tilt --poses 0 60 110
Publishes /servo_pan or /servo_tilt, grabs a frame per pose, saves
<outdir>/<joint>_<deg>.jpg and prints pairwise mean-abs-diff between
consecutive poses (float image, 0..1 scale). Restores center afterwards.
"""
import argparse
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class ServoCam(Node):
    def __init__(self, joint, poses, outdir):
        super().__init__('servo_camera_check')
        topic = '/servo_pan' if joint == 'pan' else '/servo_tilt'
        self.pub = self.create_publisher(Float32, topic, 10)
        self.joint = joint
        self.poses = poses
        self.outdir = outdir

    def set_pose(self, deg):
        msg = Float32()
        msg.data = float(deg)
        for _ in range(3):
            self.pub.publish(msg)
            time.sleep(0.15)

    def grab(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            return None
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            for _ in range(4):
                cap.read()
            time.sleep(0.7)
            ok, frame = cap.read()
            return frame if ok else None
        finally:
            cap.release()

    def run(self):
        frames = []
        for pose in self.poses:
            self.set_pose(pose)
            time.sleep(0.6)
            frame = self.grab()
            if frame is None:
                print(f'[FAIL] no frame at pose {pose}')
                return 1
            path = os.path.join(self.outdir, f'{self.joint}_{int(pose)}.jpg')
            cv2.imwrite(path, frame)
            frames.append((pose, frame.astype(np.float32) / 255.0))
            print(f'  {self.joint} {pose}: saved {path}')
        print('== pairwise motion metric (mean abs diff, 0..1) ==')
        moved = 0
        for (p1, f1), (p2, f2) in zip(frames, frames[1:]):
            d = float(np.mean(np.abs(f1 - f2)))
            print(f'  {p1}->{p2}: {d:.4f}')
            if d > 0.01:
                moved += 1
        if moved == len(frames) - 1:
            print('[PASS] 每段均检测到画面变化（云台随动）')
            return 0
        print('[FAIL] 存在无变化的段（相机可能不在云台上或舵机未动）')
        return 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--joint', choices=['pan', 'tilt'], required=True)
    ap.add_argument('--poses', nargs='+', type=float, required=True)
    ap.add_argument('--outdir', default='/tmp/servo')
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    rclpy.init()
    node = ServoCam(args.joint, args.poses, args.outdir)
    rc = node.run()
    # 回中
    node.set_pose(90.0 if args.joint == 'pan' else 60.0)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == '__main__':
    main()
