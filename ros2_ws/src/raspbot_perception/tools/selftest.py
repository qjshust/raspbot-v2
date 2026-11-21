#!/usr/bin/env python3
"""Hailo-8 yolov8s 自检（WP-I 验收工具，独立脚本，不依赖 raspbot_perception 包）。

C++ 重写后本工具保持独立可用（自含 letterbox/解析逻辑，仅依赖 hailo_platform +
numpy + cv2，均为 Pi 上已装组件）。用于在 Pi 上验证 HEF 输出布局与推理性能，
结果应与 C++ 节点（src/hailo_node.cpp + src/nms.cpp）解析出的布局一致。

用法（Pi 上源码树内运行，无需 colcon build）：
    python3 tools/selftest.py --image /tmp/frame.jpg --runs 50
    python3 tools/selftest.py --image test.png --hef /opt/hailo-models/yolov8s_h8.hef
    python3 tools/selftest.py --runs 100            # 无图：随机输入纯性能测试

输出：
    1. HEF 输入/输出 vstream 信息；
    2. 真实推理输出的结构 / shape / dtype（布局证据）；
    3. 每个检测的 5 个原始数值（[y_min, x_min, y_max, x_max, score]，0..1 归一化）；
    4. 解析后的 (class, score, box) 原图坐标；
    5. N 次推理的平均 FPS 与单帧耗时。
"""

import argparse
import sys
import time

import numpy as np

PAD_VALUE = 114
COCO_CLASSES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush",
)


def class_name(class_id):
    if 0 <= class_id < len(COCO_CLASSES):
        return COCO_CLASSES[class_id]
    return f"class_{class_id}"


def letterbox_meta(src_w, src_h, dst=640):
    scale = min(dst / src_w, dst / src_h)
    new_w = int(round(src_w * scale))
    new_h = int(round(src_h * scale))
    pad_x = (dst - new_w) // 2
    pad_y = (dst - new_h) // 2
    return float(scale), pad_x, pad_y, new_w, new_h


def letterbox_image(img_rgb, meta, dst=640, pad_value=PAD_VALUE):
    import cv2  # Pi 已装 python3-opencv
    scale, pad_x, pad_y, new_w, new_h = meta
    if (new_w, new_h) != (img_rgb.shape[1], img_rgb.shape[0]):
        resized = cv2.resize(img_rgb, (new_w, new_h),
                             interpolation=cv2.INTER_LINEAR)
    else:
        resized = img_rgb
    out = np.full((dst, dst, img_rgb.shape[2]), pad_value, dtype=np.uint8)
    out[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return out


def map_box(x1, y1, x2, y2, scale, pad_x, pad_y, src_w, src_h):
    x1 = min(max((x1 - pad_x) / scale, 0.0), float(src_w))
    y1 = min(max((y1 - pad_y) / scale, 0.0), float(src_h))
    x2 = min(max((x2 - pad_x) / scale, 0.0), float(src_w))
    y2 = min(max((y2 - pad_y) / scale, 0.0), float(src_h))
    return x1, y1, x2, y2


def parse_nms(output, class_count=80, score_threshold=0.4, dst=640):
    """解析 hailo_platform 原生 NMS 嵌套结构（列序 [y_min,x_min,y_max,x_max,score]）。"""
    frame = output[0]
    detections = []
    for class_id, arr in enumerate(frame):
        a = np.asarray(arr, dtype=np.float64)
        if a.size == 0:
            continue
        if a.ndim == 1:
            a = a.reshape(1, -1)
        for row in a:
            y_min, x_min, y_max, x_max, score = (float(v) for v in row[:5])
            if score < score_threshold:
                continue
            detections.append({
                'class_id': int(class_id),
                'score': score,
                'x1': min(x_min, x_max) * dst,
                'y1': min(y_min, y_max) * dst,
                'x2': max(x_min, x_max) * dst,
                'y2': max(y_min, y_max) * dst,
            })
    detections.sort(key=lambda d: d['score'], reverse=True)
    return detections


def dump_raw_evidence(raw):
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

    try:
        from hailo_platform import (
            ConfigureParams, FormatType, HailoStreamInterface, HEF,
            InferVStreams, InputVStreamParams, OutputVStreamParams, VDevice)
    except ImportError:
        print('hailo_platform 不可用：本脚本需在 Pi（HailoRT Python 绑定已装）上运行')
        return 2

    image = None
    if args.image:
        try:
            import cv2
        except ImportError:
            print('读取图片需要 cv2（Pi 上已装 python3-opencv）')
            return 2
        image = cv2.imread(args.image, cv2.IMREAD_COLOR)  # BGR
        if image is None:
            print(f'无法读取图片：{args.image}')
            return 2
        print(f'图片: {args.image} 尺寸={image.shape[1]}x{image.shape[0]}')

    hef = HEF(args.hef)
    in_info = hef.get_input_vstream_infos()[0]
    out_info = hef.get_output_vstream_infos()[0]
    print(f'HEF: {args.hef}')
    print(f'  输入 {in_info.name} {tuple(in_info.shape)} '
          f'{in_info.format.type}')
    print(f'  输出 {out_info.name}（NMS BY CLASS）')

    params = VDevice.create_params()
    with VDevice(params) as target:
        cfg = ConfigureParams.create_from_hef(
            hef, interface=HailoStreamInterface.PCIe)
        ng = target.configure(hef, cfg)[0]
        ng_params = ng.create_params()
        in_params = InputVStreamParams.make(ng, quantized=True,
                                            format_type=FormatType.UINT8)
        out_params = OutputVStreamParams.make(ng, quantized=False,
                                              format_type=FormatType.FLOAT32)
        with InferVStreams(ng, in_params, out_params) as pipeline:
            with ng.activate(ng_params):
                def make_input(rgb640):
                    # batch 维必须补 1（infer 拿 shape[0] 当 batch）
                    return {in_info.name: rgb640[np.newaxis, ...]}

                if image is not None:
                    rgb = image[..., ::-1].copy()
                    meta = letterbox_meta(image.shape[1], image.shape[0], 640)
                    lb = letterbox_image(rgb, meta, 640)
                    first = pipeline.infer(make_input(lb))[out_info.name]
                else:
                    first = pipeline.infer(
                        make_input(np.random.randint(
                            0, 255, (640, 640, 3), dtype=np.uint8)))[out_info.name]
                dump_raw_evidence(first)

                t0 = time.perf_counter()
                if image is not None:
                    for _ in range(args.runs):
                        pipeline.infer(make_input(lb))[out_info.name]
                else:
                    for _ in range(args.runs):
                        pipeline.infer(make_input(np.random.randint(
                            0, 255, (640, 640, 3), dtype=np.uint8)))[out_info.name]
                dt = time.perf_counter() - t0
                print(f'推理 {args.runs} 次：{dt:.3f} s -> {args.runs / dt:.1f} FPS '
                      f'（单帧 {1000.0 * dt / args.runs:.2f} ms）')

                if image is not None:
                    print(f'--- 检测结果（score >= {args.score_threshold}，'
                          f'原图像素坐标）---')
                    detections = parse_nms(
                        pipeline.infer(make_input(lb))[out_info.name],
                        class_count=80, score_threshold=args.score_threshold,
                        dst=640)
                    if not detections:
                        print('（无检测）')
                    scale, pad_x, pad_y, _, _ = meta
                    src_w, src_h = image.shape[1], image.shape[0]
                    for det in detections:
                        x1, y1, x2, y2 = map_box(
                            det['x1'], det['y1'], det['x2'], det['y2'],
                            scale, pad_x, pad_y, src_w, src_h)
                        print(f"  {class_name(det['class_id']):<14} "
                              f"{det['score']:.3f} box=({x1:.1f}, {y1:.1f}, "
                              f"{x2:.1f}, {y2:.1f})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
