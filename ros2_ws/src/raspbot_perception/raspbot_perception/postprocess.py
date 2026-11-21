"""YOLOv8 Hailo NMS 输出解析（与 hailo_platform 完全解耦，可用假数据离线测试）。

输出布局（代码依据 + 实测证据）
---------------------------------
HailoRT 源码依据（本地检视 v4.23.0 源码）：
- ``hailort/libhailort/include/hailo/hailort.h``:
  ``hailo_bbox_float32_t = { y_min, x_min, y_max, x_max, score }``（5 个 float32）；
- ``hailort/libhailort/src/net_flow/ops/yolov8_post_process.cpp`` ``decode()``:
  坐标除以 image_width/image_height，即输出为**归一化 [0,1] 坐标**（相对 640x640
  输入画布），**无需再除以 640**，还原像素时应乘以 640；
- ``bindings/python/platform/hailo_platform/pyhailort/pyhailort.py``
  ``HailoRTTransformUtils.output_raw_buffer_to_nms_format_single_frame()``:
  ``InferVStreams.infer()`` 对 ``HAILO_NMS_BY_CLASS`` 返回
  ``frames[batch] = [class0, class1, ..., class79]``，其中 ``class_i`` 是形状
  ``(N_i, 5)`` 的 float32 数组，列顺序即 ``[y_min, x_min, y_max, x_max, score]``。
  各 class 检测数 N_i 可以不同（ragged），不能用单一 (1,80,N,5) ndarray 表示。

Pi 5 实测（2026-11-20，HailoRT 4.23.0 + yolov8s_h8.hef + 真实帧，tools/selftest.py）：
    1) 结构：``raw = infer_frame(...)`` 为 list，``len(raw)=1``（batch），
       ``raw[0]`` 是 80 个类数组的 list，``class_i`` 形状 ``(N_i, 5)``
       dtype=float32。各 class 检测数可不同（ragged）。
    2) 列顺序实测判定：用 COCO 已知场景 bus.jpg（810x1080：左侧大巴、右侧行人）
       做决定性测试，原始 5 值样例：
         person [0.3696, 0.1734, 0.8382, 0.3501, 0.8948]  (4 个 person)
         bus    [0.2108, 0.1499, 0.6885, 0.8679, 0.8834]
       按 [y_min, x_min, y_max, x_max, score] 解读：person 高/宽 ≈ 0.47/0.18
       （人形，高窄 ✓），bus 高/宽 ≈ 0.48/0.72（大巴，宽扁 ✓）；
       若按 [x_min, y_min, x_max, y_max] 解读则人变宽扁、车变高窄，与真实场景
       矛盾 → **列顺序确认为 [y_min, x_min, y_max, x_max, score]**。
    3) 归一化：全部数值落在 [0,1]（个别略超 1，如 x_max=1.0001），即坐标是
       相对 640 画布的归一化值 → 换算像素**乘以 640**，无需再除 640。
    4) 测试帧 hwtest_camera_640x480.jpg 的原始行（score 均 < 0.4 的弱检测）：
         chair  [0.4687, 0.0639, 0.8676, 0.6244, 0.2665]
         toilet [0.3605, 0.5514, 0.8656, 1.0001, 0.2551]
         book   [0.2333, 0.7113, 0.3269, 0.7447, 0.2291]
"""

import numpy as np


def _class_arrays(output, class_count):
    """把各种输入形态归一化为 [class_0, ..., class_{n-1}] 的数组列表。

    支持三种输入：
    1. hailo_platform 原生结构：list（batch）-> list（class 数）-> ndarray(N_i, 5)；
    2. 均匀 ndarray (1, class_count, N, 5)（离线测试用假数据）；
    3. 均匀 ndarray (class_count, N, 5)。
    """
    if isinstance(output, (list, tuple)):
        batch = list(output)
        frame = batch[0] if batch else []
        if isinstance(frame, (list, tuple)):
            arrays = [np.asarray(a, dtype=np.float64) for a in frame]
            if len(arrays) != class_count:
                raise ValueError(
                    f"每个 batch 应有 {class_count} 个类数组，实际 {len(arrays)}")
            return arrays
        # batch 内直接是数组（不支持），落到下方统一报错
    arr = np.asarray(output, dtype=np.float64)
    if arr.ndim == 4:
        if arr.shape[0] != 1 or arr.shape[1] != class_count or arr.shape[3] != 5:
            raise ValueError(f"ndarray 输入形状应为 (1, {class_count}, N, 5)，实际 {arr.shape}")
        return [arr[0, i] for i in range(class_count)]
    if arr.ndim == 3:
        if arr.shape[0] != class_count or arr.shape[2] != 5:
            raise ValueError(f"ndarray 输入形状应为 ({class_count}, N, 5)，实际 {arr.shape}")
        return [arr[i] for i in range(class_count)]
    raise ValueError(f"无法识别的 NMS 输出结构：{type(output).__name__}")


def parse_nms(output, class_count=80, score_threshold=0.4, dst=640):
    """解析 Hailo NMS 输出为检测列表。

    Args:
        output: hailo_platform InferVStreams.infer() 返回值（原生嵌套结构），
            或形状 (1, class_count, N, 5) / (class_count, N, 5) 的 ndarray
            （列顺序 [y_min, x_min, y_max, x_max, score]，坐标为 0..1 归一化）。
        class_count: 类别数（yolov8s_h8.hef 为 80）。
        score_threshold: 分数阈值，低于阈值的检测丢弃。
        dst: letterbox 画布边长（640），用于把归一化坐标换算为画布像素。

    Returns:
        list[dict]：按 score 降序排列，每项
        ``{'class_id', 'score', 'x1', 'y1', 'x2', 'y2'}``，坐标为 letterbox
        画布（dst x dst）像素，可直接交给
        :func:`raspbot_perception.letterbox.map_box` 映回原图。
    """
    classes = _class_arrays(output, class_count)
    detections = []
    for class_id, arr in enumerate(classes):
        if arr.size == 0:
            continue
        rows = np.asarray(arr, dtype=np.float64)
        if rows.ndim == 1:
            rows = rows.reshape(1, -1)
        for row in rows:
            y_min, x_min, y_max, x_max, score = (float(v) for v in row[:5])
            if score < score_threshold:
                continue
            x1, x2 = sorted((x_min, x_max))
            y1, y2 = sorted((y_min, y_max))
            detections.append({
                'class_id': int(class_id),
                'score': score,
                'x1': x1 * dst,
                'y1': y1 * dst,
                'x2': x2 * dst,
                'y2': y2 * dst,
            })
    detections.sort(key=lambda d: d['score'], reverse=True)
    return detections
