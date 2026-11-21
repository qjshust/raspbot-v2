"""parse_nms 测试：用假输出数组（按实测布局构造）验证解析正确性。

不 import hailo_platform，完全离线可测（Windows 可跑）。

实测布局（Pi 5, HailoRT 4.23.0, yolov8s_h8.hef，见 README）：
frames[batch] = [class_0, ..., class_79]，class_i 为 (N_i, 5) float32 数组，
列顺序 [y_min, x_min, y_max, x_max, score]，坐标为 0..1 归一化（相对 640 画布）。
"""

import numpy as np
import pytest

from raspbot_perception.postprocess import parse_nms


def _empty_frame(class_count=80):
    return [np.zeros((0, 5), dtype=np.float32) for _ in range(class_count)]


def _make_hailo_output(class_id, rows, class_count=80):
    """构造 hailo_platform 原生输出结构（batch=1）。"""
    frame = _empty_frame(class_count)
    frame[class_id] = np.asarray(rows, dtype=np.float32)
    return [frame]


def test_single_detection_hailo_structure():
    """hailo 原生嵌套结构：列顺序 [y_min, x_min, y_max, x_max, score]，坐标归一化。"""
    rows = [[0.10, 0.20, 0.30, 0.40, 0.90]]
    out = _make_hailo_output(0, rows)
    dets = parse_nms(out, class_count=80, score_threshold=0.4, dst=640)
    assert len(dets) == 1
    d = dets[0]
    assert d['class_id'] == 0
    assert d['score'] == pytest.approx(0.90)
    # 归一化坐标 x 640 -> letterbox 像素
    assert d['x1'] == pytest.approx(0.20 * 640)
    assert d['y1'] == pytest.approx(0.10 * 640)
    assert d['x2'] == pytest.approx(0.40 * 640)
    assert d['y2'] == pytest.approx(0.30 * 640)


def test_ndarray_4d_input():
    """均匀 ndarray (1, 80, N, 5) 输入（离线假数据形态）。"""
    arr = np.zeros((1, 80, 2, 5), dtype=np.float32)
    arr[0, 15] = [[0.5, 0.5, 0.6, 0.6, 0.8], [0.0, 0.0, 0.1, 0.1, 0.9]]
    dets = parse_nms(arr, class_count=80, score_threshold=0.4, dst=640)
    assert len(dets) == 2
    assert [d['class_id'] for d in dets] == [15, 15]
    # 按 score 降序
    assert dets[0]['score'] == pytest.approx(0.9)
    assert dets[1]['score'] == pytest.approx(0.8)


def test_ndarray_3d_input():
    """均匀 ndarray (80, N, 5) 输入。"""
    arr = np.zeros((80, 1, 5), dtype=np.float32)
    arr[79] = [[0.0, 0.0, 0.25, 0.25, 0.6]]
    dets = parse_nms(arr, class_count=80, score_threshold=0.4, dst=640)
    assert len(dets) == 1
    assert dets[0]['class_id'] == 79
    assert dets[0]['x2'] == pytest.approx(160)


def test_score_threshold_filtering():
    """score < 阈值 的行被丢弃。"""
    rows = [
        [0.0, 0.0, 0.1, 0.1, 0.95],   # 保留
        [0.0, 0.0, 0.1, 0.1, 0.39],   # 丢弃（< 0.4）
        [0.0, 0.0, 0.1, 0.1, 0.40],   # 保留（>= 0.4）
    ]
    dets = parse_nms(_make_hailo_output(0, rows), score_threshold=0.4)
    assert len(dets) == 2
    assert [round(d['score'], 2) for d in dets] == [0.95, 0.4]


def test_multi_class_class_id_indexing():
    """每个类数组只产生对应 class_id 的检测。"""
    out = [ _empty_frame() ]
    out[0][0] = np.asarray([[0.0, 0.0, 0.1, 0.1, 0.5]], dtype=np.float32)
    out[0][62] = np.asarray([[0.1, 0.1, 0.2, 0.2, 0.7]], dtype=np.float32)
    out[0][79] = np.asarray([[0.2, 0.2, 0.3, 0.3, 0.9]], dtype=np.float32)
    dets = parse_nms(out, class_count=80, score_threshold=0.4)
    assert len(dets) == 3
    ids = {d['class_id'] for d in dets}
    assert ids == {0, 62, 79}


def test_sorted_by_score_desc():
    rows = [
        [0.0, 0.0, 0.1, 0.1, 0.5],
        [0.0, 0.0, 0.1, 0.1, 0.95],
        [0.0, 0.0, 0.1, 0.1, 0.7],
    ]
    dets = parse_nms(_make_hailo_output(0, rows), score_threshold=0.4)
    scores = [d['score'] for d in dets]
    assert scores == sorted(scores, reverse=True)


def test_swapped_corners_are_normalized():
    """x_min > x_max 等异常输入会被归一化为 x1 <= x2。"""
    rows = [[0.4, 0.3, 0.1, 0.2, 0.8]]
    dets = parse_nms(_make_hailo_output(0, rows), score_threshold=0.4)
    d = dets[0]
    assert d['x1'] <= d['x2']
    assert d['y1'] <= d['y2']


def test_empty_output():
    assert parse_nms(_make_hailo_output(0, []), score_threshold=0.4) == []


def test_ragged_hailo_structure():
    """各 class 检测数不同（ragged）也能解析。"""
    out = [_empty_frame()]
    out[0][0] = np.asarray([[0.0, 0.0, 0.1, 0.1, 0.5]] * 3, dtype=np.float32)
    out[0][1] = np.asarray([[0.0, 0.0, 0.1, 0.1, 0.6]] * 1, dtype=np.float32)
    dets = parse_nms(out, class_count=80, score_threshold=0.4)
    assert len(dets) == 4


def test_invalid_input_raises():
    with pytest.raises(ValueError):
        parse_nms(np.zeros((7, 7), dtype=np.float32), class_count=80)
    with pytest.raises(ValueError):
        parse_nms([np.zeros((80, 5))], class_count=80)  # 类数组数不符
