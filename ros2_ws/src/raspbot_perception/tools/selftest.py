#!/usr/bin/env python3
"""Hailo-8 yolov8s 自检（WP-I 验收用例 TC-I1 的可执行脚本）。

用法（在 Pi 上，源码树内运行即可，无需 colcon build）：
    python3 tools/selftest.py --image /tmp/frame.jpg --runs 50
    python3 tools/selftest.py --image test.png --hef /opt/hailo-models/yolov8s_h8.hef
    python3 tools/selftest.py --runs 100            # 无图：随机输入纯性能测试

输出：
    1. HEF 输入/输出 vstream 信息；
    2. 真实推理输出的结构 / shape / dtype（布局证据）；
    3. 每个检测的 5 个原始数值（y_min, x_min, y_max, x_max, score）；
    4. 解析后的 (class, score, box) 原图坐标；
    5. N 次推理的平均 FPS 与单帧耗时。
"""

import argparse
import os
import sys
import time

import numpy as np

# 兼容两种运行方式：源码树（tools/ 下运行）或安装后（share 目录）
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from raspbot_perception.classes import class_name  # noqa: E402
from raspbot_perception.detector import HAILO_AVAILABLE, HailoDetector  # noqa: E402


def _read_image_bgr(path):
    """读 jpg/png/bmp -> BGR uint8。优先 cv2（Pi 已装），无 cv2 时退化报错。"""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError('读取图片需要 cv2（Pi 上已装 python3-opencv）') from exc
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise RuntimeError(f'无法读取图片：{path}')
    return img


def _dump_raw_evidence(raw):
    """打印 hailo 原生 NMS 输出结构与每个检测的 5 个原始数值（布局证据）。"""
    print('--- 输出布局实测证据 ---')
    print(f'raw 输出类型: {type(raw).__name__}')
    batch = list(raw)
    print(f'batch 帧数（shape[0]）: {len(batch)}')
    frame = batch[0]
    print(f'每帧类数组数: {len(frame)}')
    total = 0
    for class_id, arr in enumerate(frame):
        a = np.asarray(arr)
        if a.size == 0:
            continue
        total += a.shape[0]
        for row in a:
            print(f'  class={class_id} ({class_name(class_id)}) '
                  f'形状={a.shape} dtype={a.dtype} 原始5值='
                  f'[{row[0]:.4f}, {row[1]:.4f}, {row[2]:.4f}, {row[3]:.4f}, '
                  f'{row[4]:.4f}]')
    print(f'非空类数组数: {sum(1 for a in frame if np.asarray(a).size)}，'
          f'总检测行数: {total}')
    print('--- 布局证据结束 ---')


def main():
    parser = argparse.ArgumentParser(description='Hailo-8 yolov8s 自检')
    parser.add_argument('--image', default=None,
                        help='输入图片路径（jpg/png/bmp）；缺省用随机输入跑纯性能')
    parser.add_argument('--hef', default='/opt/hailo-models/yolov8s_h8.hef',
                        help='HEF 路径')
    parser.add_argument('--runs', type=int, default=50, help='推理次数（预热不计）')
    parser.add_argument('--score-threshold', type=float, default=0.4,
                        help='检测分数阈值')
    args = parser.parse_args()

    if not HAILO_AVAILABLE:
        print('hailo_platform 不可用：本脚本需在 Pi（HailoRT Python 绑定已装）上运行')
        sys.exit(2)

    image = None
    if args.image:
        image = _read_image_bgr(args.image)
        print(f'图片: {args.image} 尺寸={image.shape[1]}x{image.shape[0]}')

    detector = HailoDetector(args.hef, score_threshold=args.score_threshold)
    detector.start()
    try:
        print(f'HEF: {args.hef}')
        print(f'  输入 {detector.input_name} ({detector.input_size}, '
              f'{detector.input_size}, 3) UINT8')
        print(f'  输出 {detector.output_name} HAILO NMS BY CLASS (80 类)')

        # 预热 1 次
        if image is not None:
            first = detector.infer_frame(_letterbox_from_bgr(detector, image))
        else:
            first = detector.infer_frame(
                np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8))
        _dump_raw_evidence(first)

        t0 = time.perf_counter()
        if image is not None:
            for _ in range(args.runs):
                detector.detect(image)
        else:
            for _ in range(args.runs):
                detector.infer_frame(
                    np.random.randint(0, 255, (640, 640, 3), dtype=np.uint8))
        dt = time.perf_counter() - t0
        fps = args.runs / dt
        print(f'推理 {args.runs} 次：{dt:.3f} s -> {fps:.1f} FPS '
              f'（单帧 {1000.0 * dt / args.runs:.2f} ms）')

        if image is not None:
            print(f'--- 检测结果（score >= {args.score_threshold}，原图像素坐标）---')
            detections = detector.detect(image)
            if not detections:
                print('（无检测）')
            for det in detections:
                print(f"  {det['class_name']:<14} {det['score']:.3f} "
                      f"box=({det['x1']:.1f}, {det['y1']:.1f}, "
                      f"{det['x2']:.1f}, {det['y2']:.1f})")
    finally:
        detector.close()


def _letterbox_from_bgr(detector, image):
    """把 BGR 图做成 detector 期望的 letterbox RGB 输入（仅自检脚本用）。"""
    from raspbot_perception.letterbox import (
        PAD_VALUE, bgr_to_rgb, letterbox_image, letterbox_meta)

    rgb = bgr_to_rgb(image)
    meta = letterbox_meta(image.shape[1], image.shape[0], detector.input_size)
    return letterbox_image(rgb, meta, detector.input_size, PAD_VALUE)


if __name__ == '__main__':
    main()
