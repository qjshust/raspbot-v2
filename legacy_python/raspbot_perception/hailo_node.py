"""hailo_node：Hailo-8 YOLOv8 感知节点（WP-I）。

订阅 /image_raw（sensor_msgs/Image，支持 bgr8/rgb8），推理后发布：
- /detections        vision_msgs/Detection2DArray（header.frame_id=camera_link，
                     header.stamp=输入帧 stamp；bbox.center 为归一化 (cx,cy)，
                     size 为归一化 (w,h)）
- /perception/image  sensor_msgs/Image（bgr8，画框标注后的原帧）

不依赖 cv_bridge：直接 np.frombuffer + reshape 解码 sensor_msgs/Image。
Lyrical 版 vision_msgs 注意：ObjectHypothesisWithPose 用
hypothesis.class_id/hypothesis.score，BoundingBox2D.center 是
vision_msgs/Pose2D（position: vision_msgs/Point2D + theta），勿用旧版字段。
"""

import sys
import time

import cv2  # 仅用于画框标注（python3-opencv），解码不使用 cv_bridge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

from .detector import HAILO_AVAILABLE, HailoDetector

SUPPORTED_ENCODINGS = ('bgr8', 'rgb8')


def image_to_bgr_numpy(msg):
    """把 sensor_msgs/Image 解码为 BGR uint8 numpy 数组（不依赖 cv_bridge）。

    仅支持 8 位 3 通道的 bgr8 / rgb8；其他编码抛 ValueError。

    Args:
        msg: sensor_msgs.msg.Image。

    Returns:
        (H, W, 3) uint8 BGR 数组。

    Raises:
        ValueError: 编码不支持、大端、或数据长度不足。
    """
    if msg.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(
            f"不支持的编码 '{msg.encoding}'（仅支持 {SUPPORTED_ENCODINGS}）")
    if msg.is_bigendian:
        raise ValueError("不支持大端序图像（is_bigendian=1）")
    channels = 3
    width, height = msg.width, msg.height
    step = msg.step if msg.step else width * channels
    data = np.frombuffer(msg.data, dtype=np.uint8)
    expected = step * height
    if data.size < expected:
        raise ValueError(f"图像数据不足：收到 {data.size} 字节 < 期望 {expected}")
    if step != width * channels:
        data = data[:expected].reshape(height, step)[:, :width * channels]
    frame = data.reshape(height, width, channels)
    if msg.encoding == 'rgb8':
        frame = frame[..., ::-1].copy()  # -> BGR
    return frame


class HailoPerceptionNode(Node):
    """Hailo-8 感知节点：/image_raw -> /detections + /perception/image。"""

    def __init__(self):
        super().__init__('hailo_node')

        self.declare_parameter('hef_path', '/opt/hailo-models/yolov8s_h8.hef')
        self.declare_parameter('score_threshold', 0.4)
        self.declare_parameter('max_rate', 10.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('device_timeout', 0)
        self.declare_parameter('fail_fast', True)

        self._hef_path = self.get_parameter('hef_path').value
        self._score_threshold = float(
            self.get_parameter('score_threshold').value)
        self._max_rate = float(self.get_parameter('max_rate').value)
        self._publish_annotated = bool(
            self.get_parameter('publish_annotated').value)
        self._device_timeout = int(self.get_parameter('device_timeout').value)
        self._fail_fast = bool(self.get_parameter('fail_fast').value)

        # 状态与统计
        self._detector = None
        self._retry_timer = None
        self._infer_count = 0
        self._error_count = 0
        self._last_infer_time = 0.0
        self._last_error_log = 0.0
        self._fps_ema = 0.0
        self._ms_ema = 0.0

        # 默认回调组互斥：图像回调、诊断定时器串行执行，
        # 保证 detector 只被单一线程使用（detector 内部另有互斥锁作防御）。
        cb_group = MutuallyExclusiveCallbackGroup()

        self._sub = self.create_subscription(
            Image, 'image_raw', self._on_image,
            qos_profile_sensor_data, callback_group=cb_group)
        self._det_pub = self.create_publisher(
            Detection2DArray, 'detections', qos_profile_sensor_data)
        self._img_pub = self.create_publisher(
            Image, 'perception/image', qos_profile_sensor_data)
        self._diag_timer = self.create_timer(
            1.0, self._on_diag, callback_group=cb_group)

        if not HAILO_AVAILABLE:
            self.get_logger().error(
                'hailo_platform 不可用：请按 docs/Hailo-8_驱动安装与验证.md §5 '
                '在 Pi 上安装 Python 绑定')
            if self._fail_fast:
                raise RuntimeError('hailo_platform 不可用')

        self.get_logger().info(
            f'参数：hef_path={self._hef_path} score_threshold={self._score_threshold} '
            f'max_rate={self._max_rate} publish_annotated={self._publish_annotated} '
            f'device_timeout={self._device_timeout} fail_fast={self._fail_fast}')
        self._try_load_detector()

    # ------------------------------------------------------------------ 初始化/重试

    def _try_load_detector(self):
        """加载 HEF 并激活网络。失败时按 fail_fast 决定退出或定时重试。"""
        try:
            timeout = self._device_timeout if self._device_timeout > 0 else None
            detector = HailoDetector(
                self._hef_path, score_threshold=self._score_threshold,
                device_timeout=timeout)
            detector.start()
        except Exception as exc:
            self.get_logger().error(
                f'HEF 加载/设备初始化失败: {exc}')
            if self._fail_fast:
                raise RuntimeError(
                    f'Hailo 设备初始化失败（fail_fast=true）：{exc}') from exc
            if self._retry_timer is None:
                self.get_logger().warning(
                    'fail_fast=false：每 5 s 重试加载，期间丢弃所有图像帧')
                self._retry_timer = self.create_timer(
                    5.0, self._on_retry,
                    callback_group=MutuallyExclusiveCallbackGroup())
            return
        self._detector = detector
        if self._retry_timer is not None:
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None
        self.get_logger().info(
            f'Hailo 检测器就绪：{self._hef_path}（输入 '
            f'{detector.input_size}x{detector.input_size} UINT8，输出 NMS BY CLASS）')

    def _on_retry(self):
        self._try_load_detector()

    # ------------------------------------------------------------------ 图像回调

    def _on_image(self, msg):
        if self._detector is None:
            now = time.monotonic()
            if now - self._last_error_log >= 5.0:
                self.get_logger().warning('检测器未就绪，丢弃图像帧')
                self._last_error_log = now
            return

        # max_rate 限频（<=0 表示不限频）
        now = time.monotonic()
        if self._max_rate > 0:
            period = 1.0 / self._max_rate
            if now - self._last_infer_time < period:
                return

        try:
            frame = image_to_bgr_numpy(msg)
        except ValueError as exc:
            self._rate_limited_warn(f'图像帧不合法，丢弃: {exc}')
            return

        try:
            t0 = time.perf_counter()
            detections = self._detector.detect(frame)
            elapsed = time.perf_counter() - t0
            self._last_infer_time = time.monotonic()
            self._infer_count += 1
            self._fps_ema = (0.2 / elapsed + 0.8 * self._fps_ema
                             if self._fps_ema else 1.0 / elapsed)
            self._ms_ema = (0.2 * elapsed * 1000 + 0.8 * self._ms_ema
                            if self._ms_ema else elapsed * 1000)

            self._publish_detections(msg, detections)
            if self._publish_annotated:
                annotated = self._draw(frame, detections)
                self._publish_annotated_image(msg, annotated)
        except Exception as exc:  # 推理偶发异常：计数 + 限频 WARN，继续运行
            self._error_count += 1
            self._rate_limited_warn(
                f'推理异常（累计 {self._error_count} 次）: {exc}')

    def _rate_limited_warn(self, text):
        now = time.monotonic()
        if now - self._last_error_log >= 10.0:
            self.get_logger().warning(text)
            self._last_error_log = now

    # ------------------------------------------------------------------ 发布

    def _publish_detections(self, msg, detections):
        arr = Detection2DArray()
        arr.header.stamp = msg.header.stamp
        arr.header.frame_id = 'camera_link'  # 契约 CONTRACTS.md §2
        width = float(msg.width)
        height = float(msg.height)
        for det in detections:
            d2d = Detection2D()
            d2d.header.stamp = msg.header.stamp
            d2d.header.frame_id = 'camera_link'
            # Lyrical 版 vision_msgs：ObjectHypothesisWithPose 用
            # hypothesis.class_id / hypothesis.score（非旧版顶层 id/score）
            hyp = ObjectHypothesisWithPose()
            oh = ObjectHypothesis()
            oh.class_id = str(det['class_id'])
            oh.score = float(det['score'])
            hyp.hypothesis = oh
            d2d.results.append(hyp)
            # bbox：CenterPosition + size，(x1,y1,x2,y2) -> 归一化 (cx, cy) + (w, h)
            # 注：vision_msgs 注释写 "in pixels"，但本车契约（WP-I 设计）约定
            # 归一化 [0,1] 相对坐标，下游按图像宽高换算即可，见 README。
            bbox = BoundingBox2D()
            center = Pose2D()
            center.position = Point2D(
                x=float((det['x1'] + det['x2']) / 2.0 / width),
                y=float((det['y1'] + det['y2']) / 2.0 / height))
            center.theta = 0.0
            bbox.center = center
            bbox.size_x = float((det['x2'] - det['x1']) / width)
            bbox.size_y = float((det['y2'] - det['y1']) / height)
            d2d.bbox = bbox
            arr.detections.append(d2d)
        self._det_pub.publish(arr)

    def _publish_annotated_image(self, msg, annotated):
        out = Image()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = msg.width * 3
        out.data = annotated.tobytes()
        self._img_pub.publish(out)

    @staticmethod
    def _draw(bgr, detections):
        img = bgr.copy()
        for det in detections:
            x1, y1 = int(round(det['x1'])), int(round(det['y1']))
            x2, y2 = int(round(det['x2'])), int(round(det['y2']))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{det['class_name']} {det['score']:.2f}"
            cv2.putText(img, label, (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                        cv2.LINE_AA)
        return img

    # ------------------------------------------------------------------ 诊断

    def _on_diag(self):
        if self._infer_count > 0:
            self.get_logger().info(
                f'[诊断] 推理 {self._fps_ema:.1f} FPS | 平均 {self._ms_ema:.1f} ms/帧 | '
                f'累计 {self._infer_count} 帧 | 异常 {self._error_count} 次 | '
                f'检测器 {"就绪" if self._detector else "未就绪"}')
        else:
            self.get_logger().info(
                f'[诊断] 尚无推理帧 | 检测器 '
                f'{"就绪" if self._detector else "未就绪"}')

    # ------------------------------------------------------------------ 关闭

    def destroy_node(self):
        if self._retry_timer is not None:
            self.destroy_timer(self._retry_timer)
            self._retry_timer = None
        if self._detector is not None:
            self._detector.close()
            self._detector = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = HailoPerceptionNode()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass  # 外部停机 → 退出码 0
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'[hailo_node] 致命错误: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
