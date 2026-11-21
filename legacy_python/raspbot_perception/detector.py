"""Hailo-8 YOLOv8 检测器（依赖 hailo_platform，仅能在 Pi 上加载）。

用法（节点内单推理线程；本类自带互斥锁作防御）::

    det = HailoDetector('/opt/hailo-models/yolov8s_h8.hef', score_threshold=0.4)
    det.start()
    results = det.detect(bgr_frame)   # list[dict]
    det.close()

要点：
- 输入 vstream 用 quantized UINT8（HEF 输入本身就是 UINT8，省去 float32→uint8
  的 host 端量化转换，比 float32 输入更快）；
- 输出 vstream 用 quantized=False FLOAT32；
- ``InferVStreams.infer`` 拿 ``input.shape[0]`` 当 batch 数，输入必须自带
  长度为 1 的 batch 维（shape (1, 640, 640, 3)），否则会被当成 640 帧；
- NMS 输出结构见 :mod:`raspbot_perception.postprocess` 模块头注释。
"""

import threading

import numpy as np

from .classes import COCO_CLASSES, COCO_CLASS_COUNT
from .letterbox import PAD_VALUE, bgr_to_rgb, letterbox_image, letterbox_meta, map_box
from .postprocess import parse_nms

try:  # pragma: no cover - import 失败仅发生在非 Pi 平台（Windows 开发机）
    from hailo_platform import (
        ConfigureParams,
        FormatType,
        HailoStreamInterface,
        HEF,
        InferVStreams,
        InputVStreamParams,
        OutputVStreamParams,
        VDevice,
    )
    HAILO_AVAILABLE = True
    _HAILO_IMPORT_ERROR = None
except Exception as _exc:  # noqa: BLE001
    HAILO_AVAILABLE = False
    _HAILO_IMPORT_ERROR = _exc

try:  # 加速路径（Pi 已装 python3-opencv）；缺失时退回纯 numpy 缩放
    import cv2
    _HAVE_CV2 = True
except Exception:  # noqa: BLE001
    _HAVE_CV2 = False


def _letterbox_rgb_fast(img_rgb, meta, dst, pad_value):
    """letterbox 的 cv2 快速路径（仅缩放用 cv2）；不可用时退回纯 numpy 实现。

    实测（Pi 5）：810x1080 帧纯 numpy 双线性缩放约 100 ms，cv2.resize
    INTER_LINEAR 仅 ~1-2 ms；完整 detect() 从 8.9 FPS（112.6 ms）提升到
    46.9 FPS（21.3 ms）。
    """
    if _HAVE_CV2:
        scale, pad_x, pad_y, new_w, new_h = meta
        new_w, new_h, pad_x, pad_y = map(int, (new_w, new_h, pad_x, pad_y))
        if (new_w, new_h) != (img_rgb.shape[1], img_rgb.shape[0]):
            resized = cv2.resize(
                img_rgb, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        else:
            resized = img_rgb
        out = np.full((dst, dst, img_rgb.shape[2]), pad_value, dtype=np.uint8)
        out[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
        return out
    return letterbox_image(img_rgb, meta, dst, pad_value)


class HailoDetector:
    """单网络组 YOLOv8 NMS 检测器（单流、同步、逐帧）。"""

    def __init__(self, hef_path, score_threshold=0.4, device_timeout=None):
        """加载 HEF 并配置设备；推理前需调用 :meth:`start`。

        Args:
            hef_path: HEF 文件路径（如 /opt/hailo-models/yolov8s_h8.hef）。
            score_threshold: 检测分数阈值。
            device_timeout: vstream 超时（ms）；None/<=0 用 HailoRT 默认值。

        Raises:
            RuntimeError: hailo_platform 不可用、HEF 结构不符、设备不可用等。
        """
        if not HAILO_AVAILABLE:
            raise RuntimeError(
                f"hailo_platform 不可用（{_HAILO_IMPORT_ERROR}）；"
                "HailoDetector 仅能在安装 HailoRT Python 绑定的 Pi 上使用")

        self.hef_path = hef_path
        self._score_threshold = float(score_threshold)
        self._lock = threading.Lock()  # 防御性锁：节点默认回调组本就是串行的
        self._vdevice = None
        self._pipeline_cm = None
        self._pipeline = None
        self._activation_cm = None

        try:
            self._hef = HEF(hef_path)
            in_infos = self._hef.get_input_vstream_infos()
            if not in_infos:
                raise RuntimeError(f"HEF {hef_path} 没有输入 vstream")
            self._input_info = in_infos[0]
            self._input_name = self._input_info.name
            self._dst = int(self._input_info.shape[0])
            if tuple(self._input_info.shape) != (self._dst, self._dst, 3):
                raise RuntimeError(
                    f"仅支持正方形 3 通道输入 HEF，实际 {tuple(self._input_info.shape)}")

            out_infos = self._hef.get_output_vstream_infos()
            if not out_infos:
                raise RuntimeError(f"HEF {hef_path} 没有输出 vstream")
            self._out_name = out_infos[0].name

            self._vdevice = VDevice(VDevice.create_params())
            cfg = ConfigureParams.create_from_hef(
                self._hef, interface=HailoStreamInterface.PCIe)
            self._ng = self._vdevice.configure(self._hef, cfg)[0]
            self._ng_params = self._ng.create_params()

            timeout_ms = None
            if device_timeout is not None and int(device_timeout) > 0:
                timeout_ms = int(device_timeout)
            # 输入 HEF 为 UINT8：quantized=True + UINT8 直通，避免 float32->uint8 转换开销
            self._in_params = InputVStreamParams.make(
                self._ng, quantized=True, format_type=FormatType.UINT8,
                timeout_ms=timeout_ms)
            self._out_params = OutputVStreamParams.make(
                self._ng, quantized=False, format_type=FormatType.FLOAT32,
                timeout_ms=timeout_ms)
            self._pipeline_cm = InferVStreams(
                self._ng, self._in_params, self._out_params)
        except Exception:
            self.close()
            raise

    # ------------------------------------------------------------------ 生命周期

    def start(self):
        """激活网络并创建推理管道（幂等；进程生命周期内调用一次）。"""
        with self._lock:
            if self._pipeline is not None:
                return
            # InferVStreams 的 __enter__ 才真正创建底层管道
            self._pipeline = self._pipeline_cm.__enter__()
            self._activation_cm = self._ng.activate(self._ng_params)
            self._activation_cm.__enter__()

    def close(self):
        """反激活网络、释放管道与 VDevice（幂等，可安全重复调用）。"""
        with self._lock:
            if self._activation_cm is not None:
                try:
                    self._activation_cm.__exit__(None, None, None)
                except Exception:
                    pass
                self._activation_cm = None
            if self._pipeline is not None:
                try:
                    self._pipeline_cm.__exit__(None, None, None)
                except Exception:
                    pass
                self._pipeline = None
        vdevice = getattr(self, '_vdevice', None)
        if vdevice is not None:
            try:
                vdevice.release()
            except Exception:
                pass
            self._vdevice = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.close()
        return False

    # ------------------------------------------------------------------ 推理

    def infer_frame(self, rgb_np_uint8_hwc):
        """单帧同步推理（RGB，HWC uint8，letterbox 后的 640x640）。

        Args:
            rgb_np_uint8_hwc: (640, 640, 3) uint8 RGB 图像（NHWC）。

        Returns:
            hailo 原生 NMS 输出结构：``frames[batch] = [class_0, ..., class_79]``，
            每个 ``class_i`` 为 (N_i, 5) float32 数组
            （列 = [y_min, x_min, y_max, x_max, score]，0..1 归一化坐标）。
            结构与布局见 :mod:`raspbot_perception.postprocess`。
        """
        if self._pipeline is None:
            raise RuntimeError('HailoDetector 尚未 start()')
        # 关键坑：InferVStreams.infer 拿 shape[0] 当 batch，必须补 batch 维
        data = np.ascontiguousarray(rgb_np_uint8_hwc, dtype=np.uint8)[
            np.newaxis, ...]
        with self._lock:
            out = self._pipeline.infer({self._input_name: data})
        return out[self._out_name]

    def detect(self, bgr_np):
        """对 BGR 原图做完整检测（BGR->RGB、letterbox、推理、解析、坐标映射）。

        Args:
            bgr_np: (H, W, 3) uint8 BGR 图像（任意尺寸）。

        Returns:
            list[dict]：按 score 降序，每项
            ``{'class_id', 'score', 'x1', 'y1', 'x2', 'y2'}``，坐标已映回原图
            像素并 clamp 到图像边界。
        """
        src_h, src_w = bgr_np.shape[0], bgr_np.shape[1]
        rgb = bgr_to_rgb(bgr_np)
        meta = letterbox_meta(src_w, src_h, self._dst)
        letterboxed = _letterbox_rgb_fast(rgb, meta, self._dst, PAD_VALUE)
        raw = self.infer_frame(letterboxed)
        detections = parse_nms(
            raw, class_count=COCO_CLASS_COUNT,
            score_threshold=self._score_threshold, dst=self._dst)
        scale, pad_x, pad_y, _, _ = meta
        results = []
        for det in detections:
            x1, y1, x2, y2 = map_box(
                det['x1'], det['y1'], det['x2'], det['y2'],
                scale, pad_x, pad_y, src_w, src_h)
            results.append({
                'class_id': det['class_id'],
                'class_name': COCO_CLASSES[det['class_id']]
                if 0 <= det['class_id'] < COCO_CLASS_COUNT else 'unknown',
                'score': det['score'],
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            })
        return results

    # ------------------------------------------------------------------ 属性

    @property
    def input_size(self):
        """输入画布边长（640）。"""
        return self._dst

    @property
    def input_name(self):
        return self._input_name

    @property
    def output_name(self):
        return self._out_name

    @property
    def score_threshold(self):
        return self._score_threshold
