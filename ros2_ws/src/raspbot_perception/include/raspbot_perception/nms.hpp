// YOLOv8 Hailo NMS 输出解析（纯函数，无 HailoRT/ROS 依赖，可用假数据离线测试）。
//
// 输出布局（源码依据 + Pi 5 实测证据，2026-11-20，HailoRT 4.23.0 + yolov8s_h8.hef）：
// - HailoRT C++ API（InferModel/ConfiguredInferModel）读出的
//   HAILO_FORMAT_ORDER_HAILO_NMS_BY_CLASS 原始缓冲区布局（hailort.h 文档）：
//     对每个类，packed { float32 bbox_count; hailo_bbox_float32_t bbox[bbox_count]; }
//     即 [c0][c0 框...][c1][c1 框...] 交错排布（interleaved）。
// - hailo_bbox_float32_t = { y_min, x_min, y_max, x_max, score }（5 个 float32）。
// - 坐标为 0..1 归一化（相对 640 画布；HailoRT decode 已除以图像边长），
//   换算像素应 **乘以 dst**，无需再除 640。
// - 实测判定证据（bus.jpg 决定性测试）：按 [y_min,x_min,y_max,x_max,score]
//   解读 person 高窄、bus 宽扁，与真实场景一致；反序则矛盾。
//   样例行：person [0.3696,0.1734,0.8382,0.3501,0.8948]，
//           bus    [0.2108,0.1499,0.6885,0.8679,0.8834]。
//
// Python 版语义映射（tests/test_postprocess.py 10 用例 -> tests/test_nms.cpp）：
// - hailo 原生嵌套结构 / (1,80,N,5) / (80,N,5) ndarray -> 本模块的 per-class
//   vector<vector<HailoBbox>>（ragged）；
// - "非法输入抛 ValueError" -> parse_nms 校验类数组数不符抛 std::invalid_argument，
//   parse_nms_raw 校验原始缓冲区越界抛 std::runtime_error。

#pragma once

#include <cstdint>
#include <vector>

#include "raspbot_perception/letterbox.hpp"

namespace raspbot_perception {

// NMS 输出单行（float32 原始布局）。
struct HailoBbox {
  float y_min = 0.0f, x_min = 0.0f, y_max = 0.0f, x_max = 0.0f, score = 0.0f;
};

// 解析结果：坐标为 letterbox 画布（dst x dst）像素。
struct Detection {
  int class_id = 0;
  float score = 0.0f;
  float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;
};

// per-class 结构：classes[i] = 第 i 类的 (N_i, 5) 行集合，各类长度可不同（ragged）。
using ClassesBboxes = std::vector<std::vector<HailoBbox>>;

// 解析 per-class 结构（等价 Python parse_nms 的 hailo 嵌套结构输入）。
// classes.size() 必须 == class_count，否则抛 std::invalid_argument。
// 返回按 score 降序排列的检测列表。
std::vector<Detection> parse_nms(const ClassesBboxes& classes, int class_count,
                                 float score_threshold, int dst = kDefaultSize);

// 解析 HailoRT NMS BY CLASS 原始缓冲区（interleaved：[count][bboxes] 逐类交错）。
// 缓冲区损坏/越界抛 std::runtime_error。
std::vector<Detection> parse_nms_raw(const uint8_t* buf, size_t len, int class_count,
                                     float score_threshold, int dst = kDefaultSize);

}  // namespace raspbot_perception
