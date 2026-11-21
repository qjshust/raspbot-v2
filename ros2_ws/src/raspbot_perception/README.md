# raspbot_perception — WP-I Hailo-8 YOLOv8 视觉感知

RASPBOT-V2 智能小车工作包 I：Hailo-8 上的 YOLOv8s 目标检测，订阅相机
`/image_raw`，发布 `/detections`（vision_msgs）与标注图 `/perception/image`。

| 要素 | 内容 |
|---|---|
| 订阅 | `/image_raw`（`sensor_msgs/Image`，bgr8/rgb8，来自 WP-D 相机，SensorData QoS） |
| 发布 | `/detections`（`vision_msgs/Detection2DArray`，≤10 Hz，SensorData）；`/perception/image`（`sensor_msgs/Image` bgr8 标注图，SensorData） |
| 参数 | `hef_path`、`score_threshold`、`max_rate`、`publish_annotated`、`device_timeout`、`fail_fast`（见下表） |
| 启动 | `ros2 launch raspbot_bringup perception.launch.py` |
| 验收 | `python3 tools/selftest.py --image /tmp/frame.jpg --runs 50`（对应设计文档 §10 的感知验收用例 TC-I1 的可执行形态） |

## 依赖说明

- **ROS 包**（package.xml 已声明 exec_depend）：rclpy、sensor_msgs、vision_msgs、
  std_msgs、launch_ros；rosdep key：python3-numpy、python3-opencv。
- **系统包（非 ROS）**：HailoRT 4.23.0（libhailort + `/dev/hailo0`）与
  `hailo_platform` Python 绑定。**hailo_platform 无法用 rosdep 声明**：
  它是源码编译安装的（CPython 3.14 专用），安装方法见
  [`docs/Hailo-8_驱动安装与验证.md`](../docs/Hailo-8_驱动安装与验证.md) §5 与
  `tools/install_pyhailort_source.sh`。升级 HailoRT 后必须按同一版本 tag 重编绑定。
  代码按需 import：非 Pi 平台（如 Windows 开发机）上 `letterbox/postprocess/classes`
  与全部单元测试依然可用，只有 `HailoDetector` 需要绑定。
- 模型：`/opt/hailo-models/yolov8s_h8.hef`（输入 `yolov8s/input_layer1`
  UINT8 NHWC 640×640×3；输出 `yolov8s/yolov8_nms_postprocess` FLOAT32，
  HAILO NMS BY CLASS，80 类）。**只支持 Hailo-8（HAILO8）编译的 HEF**，
  别拿 h8l 的模型。

## 参数说明

| 参数 | 默认 | 说明 |
|---|---|---|
| `hef_path` | `/opt/hailo-models/yolov8s_h8.hef` | HEF 模型路径 |
| `score_threshold` | `0.4` | NMS 输出分数阈值，低于阈值的检测丢弃 |
| `max_rate` | `10.0` | 推理限频（Hz，≤0 不限频）；相机 15 Hz 时约每 1.5 帧推理一次 |
| `publish_annotated` | `true` | 是否发布画框标注图 `/perception/image` |
| `device_timeout` | `0` | vstream 超时（ms，0=HailoRT 默认） |
| `fail_fast` | `true` | HEF 加载/设备不可用时：true=打印 ERROR 并退出（退出码 1）；false=每 5 s 重试，期间丢弃图像帧 |

## 启动命令

```bash
ros2 launch raspbot_bringup perception.launch.py
# 或指定参数文件
ros2 launch raspbot_bringup perception.launch.py params_file:=/path/to/perception.yaml
# 单节点（调试）
ros2 run raspbot_perception hailo_node --ros-args -p score_threshold:=0.5
# 自检（无需 colcon build，源码树内直接跑）
python3 tools/selftest.py --image /tmp/frame.jpg --runs 50
```

## 实测输出布局证据（Pi 5，2026-11-20）

环境：HailoRT 4.23.0 + hailo_platform（CPython 3.14）+
`/opt/hailo-models/yolov8s_h8.hef`。测试帧
`docs/hwtest_camera_640x480.jpg`（640×480）与 COCO 已知场景 `bus.jpg`
（810×1080，左侧大巴 + 右侧行人）scp 到 Pi 后由 `tools/selftest.py` 实测。

**① 结构实测**：`infer_frame()` 返回值是 list；`len(raw)=1`（batch 帧数），
`raw[0]` 是 **80 个类数组的 list**，`class_i` 形状 `(N_i, 5)`、dtype float32；
各 class 检测数可不同（ragged，bus.jpg 实测 class0 有 4 行、class5 有 1 行、
其余 78 个类为空 `(0,5)`），**不能**用单一 `(1, 80, N, 5)` ndarray 表示。

**② 列顺序实测判定**（决定性测试）：bus.jpg 中行人的 5 值样例

```
person [0.3696, 0.1734, 0.8382, 0.3501, 0.8948]   # 其余 3 个 person 同构
bus    [0.2108, 0.1499, 0.6885, 0.8679, 0.8834]
```

按 `[y_min, x_min, y_max, x_max, score]` 解读：person 高/宽 ≈ 0.47/0.18
（人形高窄 ✓），bus 高/宽 ≈ 0.48/0.72（大巴宽扁 ✓），与真实场景一致；
反过来解读则人变宽扁、车变高窄，矛盾。→ **列顺序 = [y_min, x_min, y_max,
x_max, score]**（与 HailoRT `hailo_bbox_float32_t` 结构体一致）。

**③ 归一化**：所有数值 ∈ [0,1]（个别略超 1，如 toilet 的 x_max=1.0001）——
输出是相对 640 画布的**归一化坐标**，换算像素**乘以 640**，**无需 /640**。
`parse_nms` 已按此固化，并兼容 `(1,80,N,5)`/`(80,N,5)` 均匀 ndarray（离线测试用）。

**④ 测试帧原始行**（score 均 <0.4 的弱检测，默认阈值下不发布）：

```
chair  [0.4687, 0.0639, 0.8676, 0.6244, 0.2665]
toilet [0.3605, 0.5514, 0.8656, 1.0001, 0.2551]
book   [0.2333, 0.7113, 0.3269, 0.7447, 0.2291]
```

**⑤ 真实检测样例**（bus.jpg，score≥0.4，原图 810×1080 像素坐标，cv2 快速路径）：
person 0.895 (52, 399, 243, 905)、person 0.883 (667, 394, 809, 880)、
bus 0.883 (28, 229, 803, 743)、person 0.857、person 0.534 ——
坐标经 letterbox 逆映射后落在正确位置，闭环验证通过。

## 性能预期

- 单流同步逐帧（Python 绑定，UINT8 输入，完整 detect() 管线：BGR→RGB、
  letterbox、推理、解析、坐标映射）：
  - 640×480（相机实况，scale=1 无插值）：**70.1 FPS / 14.3 ms**（两次实测
    70.2 / 70.1 FPS）；
  - 810×1080（非 1:1，cv2 快速路径）：**46.9 FPS / 21.3 ms**；纯 numpy 双线性
    缩放路径为 8.9 FPS / 112.6 ms（1080p 帧 numpy 缩放约 100 ms），因此
    detector 优先用 cv2.resize（Pi 已装 python3-opencv），缺失时才退回 numpy。
- `hailortcli run` batch 8 可达 157 FPS（CLI 流水线），本包为单流逐帧，
  不追求更高；相机仅 15 Hz，余量充足。
- 瓶颈在 Python 逐帧调用与 PCIe Gen2 ×1，详见
  [`docs/Hailo-8_驱动安装与验证.md`](../docs/Hailo-8_驱动安装与验证.md) §4。

## 离线测试（Windows 可跑）

```powershell
$env:PYTHONIOENCODING='utf-8'
& "<bundled python>\python.exe" -m pytest ros2_ws\src\raspbot_perception\tests -q
```

测试不含 hailo_platform / cv2 / ROS 依赖（letterbox 为纯 numpy 双线性缩放 +
BGR↔RGB numpy 通道翻转）。

## 已知限制

1. **只支持 bgr8/rgb8 编码**的 `/image_raw`；其他编码（mono8、yuv 等）WARN 丢弃。
   不依赖 cv_bridge（`np.frombuffer` 直接解码）。
2. **只支持正方形 3 通道输入的 HEF**（640×640×3，构造时校验）。
3. 单流同步推理：`max_rate` 限频在回调内跳帧实现，帧率过高时丢帧而非排队。
4. `fail_fast=true` 时设备不可用会直接退出节点（launch 随之终止）；需要降级
   运行请设 `fail_fast=false`。
5. `/detections` 与 `/perception/image` 都带 `header.stamp=输入帧 stamp`、
   `frame_id=camera_link`（契约 CONTRACTS.md §2）；本包**不发布 TF**。
6. 坐标 clamp：letterbox 边界附近的检测可能被截到图像边缘。
7. 线程模型：默认互斥回调组（图像回调 + 诊断定时器串行），detector 由单一
   线程使用；detector 内部另有互斥锁作防御。
8. **bbox 坐标约定**：本车契约约定 `/detections` 的 `bbox.center/size` 为
   **归一化 [0,1] 相对坐标**（×图像宽/高即得像素）。注意 vision_msgs 的
   字段注释写 "in pixels"，与归一化约定不同——下游请按本 README 约定换算。
9. **vision_msgs 版本**：按 ROS 2 Lyrical 的字段结构发布
   （`ObjectHypothesisWithPose.hypothesis.class_id/score`、
   `BoundingBox2D.center` 为 `vision_msgs/Pose2D`）；若迁移到仍用旧版
   `id/score` 顶层字段的发行版，需同步改 hailo_node.py。
