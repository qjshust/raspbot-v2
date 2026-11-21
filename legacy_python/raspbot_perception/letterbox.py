"""Letterbox 预处理与坐标映射（纯 numpy，无 cv2 / ROS 依赖，任意平台可离线测试）。

与 Hailo 官方 yolov8 示例对齐的约定：
- 等比例缩放至最长边 = dst（默认 640），居中补边；
- 补边像素值 114（Hailo 官方 letterbox 灰值）；
- 输入 HEF 期望 NHWC 640x640x3 UINT8，通道顺序 RGB（BGR 输入先经
  :func:`bgr_to_rgb` 用 numpy 索引翻转，不用 cv2.cvtColor，保证本模块可测）。
"""

import numpy as np

DEFAULT_SIZE = 640
PAD_VALUE = 114  # 与 Hailo 官方 yolov8 letterbox 示例一致


def letterbox_meta(src_w, src_h, dst=DEFAULT_SIZE):
    """计算 letterbox 缩放与补边参数。

    Args:
        src_w: 原图宽（像素）。
        src_h: 原图高（像素）。
        dst: 目标正方形边长，默认 640。

    Returns:
        (scale, pad_x, pad_y, new_w, new_h)：
        - scale: 等比例缩放系数（最长边缩放到 dst）；
        - pad_x / pad_y: 左 / 上补边像素（居中，右侧/下侧多出的 1 像素余量由
          letterbox_image 以 PAD_VALUE 填充）；
        - new_w / new_h: 缩放后图像尺寸。

    Examples:
        640x480  -> (1.0, 0, 80, 640, 480)
        1080x1920（竖图） -> (1/3, 140, 0, 360, 640)
    """
    scale = min(dst / src_w, dst / src_h)
    new_w = int(round(src_w * scale))
    new_h = int(round(src_h * scale))
    pad_x = (dst - new_w) // 2
    pad_y = (dst - new_h) // 2
    return float(scale), pad_x, pad_y, new_w, new_h


def map_box(x1, y1, x2, y2, scale, pad_x, pad_y, src_w, src_h):
    """把 letterbox 画布（dst 空间）像素坐标映回原图坐标并 clamp 到图像边界。

    Args:
        x1, y1, x2, y2: letterbox 画布中的角点像素坐标。
        scale, pad_x, pad_y: :func:`letterbox_meta` 的返回值。
        src_w, src_h: 原图尺寸。

    Returns:
        (x1, y1, x2, y2) 原图像素坐标（float，已 clamp 到 [0, src_w]/[0, src_h]）。
    """
    x1 = (x1 - pad_x) / scale
    y1 = (y1 - pad_y) / scale
    x2 = (x2 - pad_x) / scale
    y2 = (y2 - pad_y) / scale
    x1 = min(max(x1, 0.0), float(src_w))
    y1 = min(max(y1, 0.0), float(src_h))
    x2 = min(max(x2, 0.0), float(src_w))
    y2 = min(max(y2, 0.0), float(src_h))
    return x1, y1, x2, y2


def bgr_to_rgb(img):
    """BGR -> RGB 通道翻转（numpy 索引翻转，返回 C 连续副本，保证可测）。"""
    return img[..., ::-1].copy()


def rgb_to_bgr(img):
    """RGB -> BGR 通道翻转（与 bgr_to_rgb 对称）。"""
    return img[..., ::-1].copy()


def letterbox_image(img, meta=None, dst=DEFAULT_SIZE, pad_value=PAD_VALUE):
    """把 HxWxC uint8 图像 letterbox 到 dst x dst（居中，补 pad_value）。

    纯 numpy 双线性缩放（scale == 1 时跳过插值直接拷贝），不依赖 cv2。

    Args:
        img: numpy 数组，形状 (H, W, C)，uint8（通道顺序任意，本函数不关心）。
        meta: :func:`letterbox_meta` 的返回值；None 时按 img 尺寸计算。
        dst: 目标边长。
        pad_value: 补边像素值，默认 114（Hailo 官方约定）。

    Returns:
        (dst, dst, C) uint8 数组，C 连续。
    """
    if meta is None:
        meta = letterbox_meta(img.shape[1], img.shape[0], dst)
    scale, pad_x, pad_y, new_w, new_h = meta
    new_w = int(new_w)
    new_h = int(new_h)
    pad_x = int(pad_x)
    pad_y = int(pad_y)
    if (new_w, new_h) != (img.shape[1], img.shape[0]):
        resized = _resize_bilinear(img, new_w, new_h)
    else:
        resized = img
    out = np.full((dst, dst, img.shape[2]), pad_value, dtype=np.uint8)
    out[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return np.ascontiguousarray(out)


def _resize_bilinear(img, new_w, new_h):
    """纯 numpy 双线性缩放（H, W, C uint8 -> (new_h, new_w, C) uint8）。

    像素中心对齐（align_corners=False，与 OpenCV INTER_LINEAR 一致）。
    """
    src_h, src_w = img.shape[:2]
    ys = (np.arange(new_h, dtype=np.float64) + 0.5) * (src_h / new_h) - 0.5
    xs = (np.arange(new_w, dtype=np.float64) + 0.5) * (src_w / new_w) - 0.5
    ys = np.clip(ys, 0.0, src_h - 1.0)
    xs = np.clip(xs, 0.0, src_w - 1.0)
    y0 = np.floor(ys).astype(np.intp)
    x0 = np.floor(xs).astype(np.intp)
    y1 = np.minimum(y0 + 1, src_h - 1)
    x1 = np.minimum(x0 + 1, src_w - 1)
    wy = ys - y0  # (new_h,)
    wx = xs - x0  # (new_w,)
    wy0 = (1.0 - wy)  # (new_h,)
    wy1 = wy
    wx0 = (1.0 - wx)  # (new_w,)
    wx1 = wx

    # img[y, x] -> (new_h, new_w, C)
    a = img[y0[:, None], x0[None, :]].astype(np.float64)
    b = img[y1[:, None], x0[None, :]].astype(np.float64)
    c = img[y0[:, None], x1[None, :]].astype(np.float64)
    d = img[y1[:, None], x1[None, :]].astype(np.float64)

    w_a = (wy0[:, None] * wx0[None, :])[..., None]
    w_b = (wy1[:, None] * wx0[None, :])[..., None]
    w_c = (wy0[:, None] * wx1[None, :])[..., None]
    w_d = (wy1[:, None] * wx1[None, :])[..., None]

    out = a * w_a + b * w_b + c * w_c + d * w_d
    return np.rint(out).astype(np.uint8)
