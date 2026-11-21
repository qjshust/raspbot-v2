"""letterbox 纯函数测试（Windows/任意平台可跑，无需 cv2/ROS/hailo）。"""

import numpy as np
import pytest

from raspbot_perception.letterbox import (
    PAD_VALUE,
    bgr_to_rgb,
    letterbox_image,
    letterbox_meta,
    map_box,
)


def test_meta_640x480():
    """640x480：最长边已等于 dst，scale=1.0，仅上下各补 80。"""
    scale, pad_x, pad_y, new_w, new_h = letterbox_meta(640, 480, 640)
    assert scale == pytest.approx(1.0)
    assert pad_x == 0
    assert pad_y == 80
    assert new_w == 640
    assert new_h == 480
    # 补边对称且占满画布
    assert new_w + 2 * pad_x <= 640
    assert new_h + 2 * pad_y == 640


def test_meta_portrait_1080x1920():
    """1080x1920 竖图：缩放到宽 360、高 640，左右各补 140，上下不补。"""
    scale, pad_x, pad_y, new_w, new_h = letterbox_meta(1080, 1920, 640)
    assert scale == pytest.approx(640 / 1920)
    assert new_w == 360
    assert new_h == 640
    assert pad_x == 140
    assert pad_y == 0
    assert new_w + 2 * pad_x == 640
    assert new_h + 2 * pad_y == 640


def test_meta_landscape_1920x1080():
    """1920x1080 横图：宽缩到 640，高 360，上下各补 140。"""
    scale, pad_x, pad_y, new_w, new_h = letterbox_meta(1920, 1080, 640)
    assert scale == pytest.approx(640 / 1920)
    assert new_w == 640
    assert new_h == 360
    assert pad_x == 0
    assert pad_y == 140


def test_map_box_identity_scale():
    """scale=1 时坐标平移 pad 即可回原图（640x480 场景）。"""
    # letterbox 画布 (320, 80+240)=(320,320) 对应原图中心 (320,240)
    x1, y1, x2, y2 = map_box(320, 320, 400, 400, 1.0, 0, 80, 640, 480)
    assert (x1, y1, x2, y2) == (320.0, 240.0, 400.0, 320.0)


def test_map_box_roundtrip():
    """任意框：原图 -> letterbox -> 原图 往返一致（含非 1 缩放）。"""
    src_w, src_h = 1080, 1920
    scale, pad_x, pad_y, _, _ = letterbox_meta(src_w, src_h, 640)
    boxes = [(100, 200, 800, 1500), (0, 0, src_w, src_h), (50, 50, 120, 130)]
    for (x1, y1, x2, y2) in boxes:
        # 原图 -> letterbox 画布
        lb_x1, lb_y1 = x1 * scale + pad_x, y1 * scale + pad_y
        lb_x2, lb_y2 = x2 * scale + pad_x, y2 * scale + pad_y
        back = map_box(lb_x1, lb_y1, lb_x2, lb_y2,
                       scale, pad_x, pad_y, src_w, src_h)
        assert back == pytest.approx((x1, y1, x2, y2))


def test_map_box_clamp():
    """越界坐标 clamp 到 [0, src_w] x [0, src_h]。"""
    x1, y1, x2, y2 = map_box(-100, -50, 9999, 9999, 1.0, 0, 0, 640, 480)
    assert (x1, y1) == (0.0, 0.0)
    assert (x2, y2) == (640.0, 480.0)


def test_bgr_to_rgb():
    """通道翻转且返回 C 连续副本。"""
    img = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
    out = bgr_to_rgb(img)
    assert out[0, 0].tolist() == [3, 2, 1]
    assert out[0, 1].tolist() == [6, 5, 4]
    assert out.flags['C_CONTIGUOUS']
    # 原数组不被修改
    assert img[0, 0].tolist() == [1, 2, 3]


def test_letterbox_image_pad_and_content():
    """letterbox_image：画布尺寸、补边值、中心内容正确。"""
    # 2x1 (w=2,h=1) -> dst=4：scale=2, new_w=4, new_h=2, pad_y=1
    img = np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
    out = letterbox_image(img, dst=4)
    assert out.shape == (4, 4, 3)
    assert out.dtype == np.uint8
    # 上下补边为 PAD_VALUE=114
    assert np.all(out[0] == PAD_VALUE)
    assert np.all(out[3] == PAD_VALUE)
    # 中心行是缩放后的内容（2x1 -> 4x2 双线性），不应全是补边值
    assert not np.all(out[1] == PAD_VALUE)


def test_letterbox_image_640x480():
    """640x480 -> 640x640：scale=1 时无插值，仅上下补边。"""
    rng = np.random.default_rng(0)
    img = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    out = letterbox_image(img, dst=640)
    assert out.shape == (640, 640, 3)
    assert np.array_equal(out[80:560], img)
    assert np.all(out[:80] == PAD_VALUE)
    assert np.all(out[560:] == PAD_VALUE)
