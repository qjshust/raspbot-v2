# raspbot_perception — WP-I Hailo-8 YOLOv8 视觉感知（C++ 版）

RASPBOT-V2 智能小车工作包 I：Hailo-8 上的 YOLOv8s 目标检测。本包为
**C++ 重写版**（task-5）：核心库（letterbox/NMS/类名/图像工具）为纯函数、
不依赖 NPU；`hailo_node` 用 **HailoRT 原生 C++ API**（hailort.hpp 4.23，
VDevice → InferModel → ConfiguredInferModel → 异步 infer）。原 Python 源码
已归档 `legacy_python/raspbot_perception/`，Python 测试 `tests/*.py`
（3 文件 23 用例）原样保留作为 gtest 的参考规格。

| 要素 | 内容 |
|---|---|
| 订阅 | `/image_raw`（`sensor_msgs/Image`，bgr8/rgb8/yuyv/yuv422，SensorData QoS） |
| 发布 | `/detections`（`vision_msgs/Detection2DArray`，≤10 Hz，SensorData，`frame_id=camera_link`）；`/perception/image`（`sensor_msgs/Image` bgr8 标注图，SensorData） |
| 参数 | `hef_path`、`score_threshold`、`max_rate`、`publish_annotated`、`device_timeout`、`fail_fast`（与 Python 版一致，见下表） |
| 启动 | `ros2 launch raspbot_perception perception.launch.py`（或 `raspbot_bringup perception.launch.py`） |
| 测试 | `colcon test --packages-select raspbot_perception`（gtest，31 用例，不依赖 NPU） |
| 验收 | `python3 tools/selftest.py --image /tmp/frame.jpg --runs 50`（独立脚本，Pi 上验证 HEF 通路） |

## 依赖

- **ROS 包**（package.xml）：rclcpp、sensor_msgs、vision_msgs、
  geometry_msgs、std_msgs；launch_ros（exec）；test 依赖 ament_cmake_gtest。
- **HailoRT（系统级，无法用 rosdep 声明）**：Pi 实测安装路径
  - 头文件：`/usr/include/hailo/hailort.hpp`（`-I/usr/include`）
  - 库：`/usr/lib/libhailort.so`（→ `libhailort.so.4.23.0`）
  - CMakeLists 用 `find_path/find_library` 自动探测（含 `/opt/hailo` 兜底），
    找不到时**跳过 hailo_node 构建**，核心库与 gtest 不受影响；也可手动
    `-DHAILORT_INCLUDE_DIR=... -DHAILORT_LIBRARY=...`。
- **无 OpenCV / cv_bridge 依赖**：解码、letterbox、YUYV→RGB（BT.601 定点）、
  标注（内置 5×7 点阵字体）均为纯 C++ 实现。
- 模型：`/opt/hailo-models/yolov8s_h8.hef`（输入 UINT8 NHWC 640×640×3；
  输出 HAILO NMS BY CLASS FLOAT32，80 类）。**只支持 Hailo-8（HAILO8）编译的 HEF**。

## 架构

```
include/raspbot_perception/
  letterbox.hpp        letterbox_meta / map_box / bgr<->rgb / letterbox_image
  classes.hpp          COCO 80 类名表 + class_name
  nms.hpp              HailoBbox / Detection / parse_nms / parse_nms_raw
  image_utils.hpp      yuyv_to_rgb / draw_detection（点阵标注）
  hailo_backend.hpp    HailoRT 原生 API 封装（仅 HailoRT 可用时编译）
src/                  对应 .cpp + hailo_node.cpp（rclcpp 节点，可执行 hailo_node）
tests/*.cpp            gtest：23 用例 1:1 映射 Python 版 + 8 个 C++ 新增覆盖
tests/*.py             Python 原测试（字节不变，参考规格；可对 legacy_python 运行）
```

## 输出布局（实测证据，Pi 5 + HailoRT 4.23.0 + yolov8s_h8.hef，2026-11-20）

- HailoRT C++ API 读出的 `HAILO_NMS_BY_CLASS` 原始缓冲区（hailort.h 文档）：
  逐类交错 `[float32 count][count × 5 × float32]`；
- 每行 5 列 = **`[y_min, x_min, y_max, x_max, score]`**（`hailo_bbox_float32_t`
  布局）。实测判定（bus.jpg 决定性测试）：person 高窄、bus 宽扁，反序解读
  与真实场景矛盾。样例行：
  `person [0.3696,0.1734,0.8382,0.3501,0.8948]`、
  `bus [0.2108,0.1499,0.6885,0.8679,0.8834]`；
- 坐标为 **0..1 归一化**（相对 640 画布），换算像素**×640**，无需再除 640；
- 各 class 检测数不同（ragged），Python 绑定表现为 80×N_i×5 嵌套结构，C++ 为
  交错原始缓冲区——两者由同一实测证据固化（Python 版证据见
  `legacy_python/` 归档与 `tools/selftest.py`）。

## 参数（与 Python 版一致）

| 参数 | 默认 | 说明 |
|---|---|---|
| `hef_path` | `/opt/hailo-models/yolov8s_h8.hef` | HEF 模型路径 |
| `score_threshold` | `0.4` | 分数阈值（parse_nms 过滤） |
| `max_rate` | `10.0` | 推理限频（Hz，≤0 不限频；回调内跳帧） |
| `publish_annotated` | `true` | 是否发布 `/perception/image` 标注图 |
| `device_timeout` | `0` | 推理等待/调度超时（ms；0=默认 1000 ms 等待） |
| `fail_fast` | `true` | 设备不可用：true=ERROR+退出码 1；false=每 5 s 重试并丢帧 |

## 构建与测试

```bash
colcon build --packages-select raspbot_perception
colcon test --packages-select raspbot_perception   # gtest：31 用例（无需 NPU）
colcon test-result --verbose
```

gtest 用例清单（23 = Python tests/*.py 1:1）：

| 文件 | 用例 |
|---|---|
| `test_letterbox.cpp` | 9：Meta640x480 / MetaPortrait1080x1920 / MetaLandscape1920x1080 / MapBoxIdentityScale / MapBoxRoundtrip / MapBoxClamp / BgrToRgb / LetterboxImagePadAndContent / LetterboxImage640x480 |
| `test_nms.cpp` | 10（1:1）：SingleDetectionHailoStructure / Ndarray4dInput / Ndarray3dInput / ScoreThresholdFiltering / MultiClassClassIdIndexing / SortedByScoreDesc / SwappedCornersAreNormalized / EmptyOutput / RaggedHailoStructure / InvalidInputRaises |
| `test_classes.cpp` | 4：LengthIs80 / NoDuplicatesAndOrder / SampleIds / ClassNameUnknown |

C++ 新增覆盖（8，Python 无对应入口）：`test_nms.cpp::RawBufferInterleavedLayout`
（NMS 原始缓冲区入口）、`test_yuyv.cpp`×5（YUYV→RGB BT.601）、`test_draw.cpp`×2
（点阵标注/越界 clamp）。

Python 原测试仍可对归档源码运行：
`PYTHONPATH=legacy_python python3 -m pytest ros2_ws/src/raspbot_perception/tests -q`
（23 passed）。

## 契约对应（CONTRACTS.md §2）

- `/detections`：`header.stamp=输入帧 stamp`、`header.frame_id=camera_link`；
  `results[0].hypothesis.class_id=str(class_id)`、`.score`（Lyrical 字段结构，
  非旧版顶层 id/score）；`bbox.center(position.x/y)+size_x/size_y` 为**归一化
  [0,1] 相对坐标**（注意 vision_msgs 字段注释写 "in pixels"，本车契约约定
  归一化，下游按图像宽高换算）。
- `/perception/image`：bgr8，header 同输入帧。
- 本包不发布 TF、不碰 I2C/电机；两个话题各唯一发布者。

## 已知限制

1. 图像编码支持：bgr8✓、rgb8✓、yuyv/yuv422✓（宽须为偶数、无行填充），
   其他编码 WARN 丢弃；不依赖 cv_bridge。
2. 只支持正方形 3 通道输入 HEF（640×640×3，构造时校验）。
3. 单流同步推理：`run_async` 提交后同步 `wait`（单帧流水）；`max_rate` 限频为
   回调内跳帧，高帧率丢帧不排队。
4. `fail_fast=true` 时设备不可用直接退出（launch 随之终止）；降级设 false。
5. 坐标 clamp：letterbox 边界检测可能截到图像边缘。
6. 线程模型：订阅与诊断定时器同一互斥回调组，backend 单线程使用。
7. Python 版 `tools/selftest.py` 已改为独立脚本（不 import 本包，C++ 包无
   Python 模块）。
