# WP-D 相机 bringup（camera/）

## 1. 交付内容

| 文件 | 用途 |
|---|---|
| `camera/camera.yaml` | `v4l2_camera_node` 参数（YUYV 640×480@15 / rgb8 / camera_link / 内参 URL） |
| `camera/intrinsics.yaml` | CameraInfo 占位内参（待 CAR-C06 标定替换） |
| `camera/calibrate_camera.py` | 内参标定脚本（离线 --images / 在线 --device，覆盖 intrinsics.yaml） |
| `launch/camera.launch.py` | 启动 v4l2_camera_node（+ 可选 republish，默认注释） |

## 2. 话题契约

- `/image_raw`（`sensor_msgs/Image`，**encoding=rgb8**，`frame_id=camera_link`）+
  `/camera_info`（内参随图像发布，FR-D-01）。
- 发布者：`v4l2_camera` 节点的 image_transport CameraPublisher（QoS 为
  image_transport 默认 sensor data；以 `ros2 topic info -v` 实测记录，SW-C02/J-13 口径）。

## 3. 像素格式与帧率（裁决 J-03 落地）

- **像素格式 = YUYV**。⚠ MJPG 实测不可用（2026-11-20 Pi 实跑，lead 反馈日志）：
  v4l2_camera 0.8.1 报 `Current pixel format is not supported yet: MJPG`，
  且随后空 encoding 触发 `cv_bridge::Exception: Unrecognized image encoding []`
  崩溃。设备原生支持 YUYV（硬件报告 §5.3 格式表：`YUYV 640×480@15`）。
- **帧率 = 15 Hz 契约落地**：YUYV 640×480 的设备原生帧率就是 15 fps，
  与 CONTRACTS §2 / FR-D-01 / 裁决 J-03 的 15 Hz 完全一致，无需任何限频。
  器件能力 MJPG 640×480 28.2~29.6 fps（硬件报告 §5.3）仅作能力记录，不参与判定，
  且本驱动版本无法使用该格式。
- ⚠ **v4l2_camera 0.8.1 没有 `framerate` 参数**（证据：Pi 上
  `strings /opt/ros/lyrical/lib/libv4l2_camera.so` 的参数集合为
  video_device/pixel_format/output_encoding/image_size/camera_frame_id/
  camera_info_url；上游 boldhearts/ros2_v4l2_camera `src/v4l2_camera.cpp`
  的 declare_parameter 列表同样无 framerate）。`camera.yaml` 中的
  `framerate: 15.0` 键为**惰性记录值（不生效）**，实际频率由设备原生决定
  （YUYV 640×480 ≈ 15 Hz）。
- 如需更高帧率/更大分辨率：0.8.1 不支持 MJPG 与本摄像头 MJPG 流的组合，
  需换驱动方案（升级 v4l2_camera / gscam 等），超出本 WP 范围，另行立项。

## 4. 参数表（v4l2_camera_node，经 Pi 只读取证 + 上游源码交叉 + 实跑验证）

| 参数 | 值 | 证据 |
|---|---|---|
| `video_device` | `/dev/video0` | libv4l2_camera.so strings；默认值同上 |
| `pixel_format` | `YUYV` | 同上（默认 YUYV）；MJPG 实测崩溃（§3） |
| `output_encoding` | `rgb8` | 同上（默认 rgb8）；YUYV→rgb8 常规转码 |
| `image_size` | `[640, 480]` | 同上（默认 [640,480]）；FR-D-01 |
| `camera_frame_id` | `camera_link` | 同上（默认 `camera`）；SW-C03/TF 契约 §3 |
| `camera_info_url` | `package://raspbot_bringup/camera/intrinsics.yaml` | 同上 |
| `framerate` | `15.0`（惰性） | 见 §3，0.8.1 无此参数；YUYV 原生 15 fps |

> 白平衡/曝光/增益等相机控制项由 v4l2_camera 按设备能力自动暴露为参数
> （v4l2_camera.cpp `createParameters()` 遍历 V4L2 controls），本包不预设，
> 需要时 `ros2 param list /v4l2_camera` 查看。

## 5. 内参与标定（CAR-C06 / FR-D-02）

- `intrinsics.yaml` 现为占位值：K=fx=fy≈640、主点 (320,240)、畸变全 0，
  仅保证链路形态，**不得用于测量结论**。
- 标定（真人配合，棋盘格 8×6、方格 25 mm，重投影目标 <1 px）：
  ```bash
  # 查看计划（无需 cv2）
  python3 <share>/raspbot_bringup/camera/calibrate_camera.py --dry-run
  # 离线标定（图像目录）
  python3 <share>/raspbot_bringup/camera/calibrate_camera.py --images ./calib_images
  # 在线采集+标定（Pi 上，无头可用，慢速移动棋盘）
  python3 <share>/raspbot_bringup/camera/calibrate_camera.py --device /dev/video0
  ```
  脚本默认覆盖脚本同目录的 `intrinsics.yaml`；标定后重启
  `camera.launch.py` 生效。
- ⚠ **尺寸必须一致**：v4l2_camera 源码 `checkCameraInfo()` 在
  `camera_info` 与实发图像尺寸不一致时会**丢弃整个 CameraInfo（K 被清空）**；
  因此 FR-D-03 改分辨率时，必须同步更新 `camera.yaml` 的 `image_size` 与
  `intrinsics.yaml` 的 image_width/height（及 K/P）。

## 6. 启动方式

```bash
ros2 launch raspbot_bringup camera.launch.py
ros2 launch raspbot_bringup camera.launch.py \
    camera_params_file:=<share>/raspbot_bringup/camera/camera.yaml   # FR-D-03 覆盖口
```

## 7. 已知限制

- 无 `framerate` 参数（0.8.1），频率 = 设备原生（YUYV 640×480 ≈ 15 Hz，与契约一致）。
- **MJPG 不可用**（0.8.1 像素格式白名单外，实测崩溃），见 §3。
- `compressed` 转发未启用：Pi 上未安装 image_transport_plugins（
  `/opt/ros/lyrical/share` 下无 compressed_image_transport），且
  raspbot_bringup/package.xml 未声明该依赖（属 lead 维护）。`camera.launch.py`
  中 republish 节点保持注释并注明启用条件（/image_raw 为 rgb8 未压缩，
  如需跨机低带宽传输才有必要启用）。
- FR-D-03（分辨率可配）：参数切换需重启节点（launch 参数覆盖口径 SW-G09）；
  并注意 §5 的尺寸同步约束。
- 内参占位值阶段，任何依赖 K 的视觉测量结果无效（等待 CAR-C06）。

## 8. 验收用例对应

| 需求 | 用例 |
|---|---|
| FR-D-01（/image_raw+/camera_info） | `SW-C01`（YUYV 原生 ≈15 Hz，见 §3）、`CAR-B11` |
| FR-D-02（重投影 <1 px） | `CAR-C06`（calibrate_camera.py） |
| FR-D-03（分辨率可配） | `SW-G09` |
| J-03（15 Hz vs 30 fps） | YUYV 640×480@15 原生落地契约；MJPG 30 fps 仅能力记录 |
| SW-C03（frame_id 集合） | `camera_frame_id=camera_link` |
