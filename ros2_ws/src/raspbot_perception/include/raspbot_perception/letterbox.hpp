// RASPBOT-V2 WP-I: letterbox 预处理与坐标映射（纯函数，无 HailoRT/ROS/OpenCV 依赖）。
//
// 与 Python 版（已归档 legacy_python/raspbot_perception/letterbox.py）语义 1:1：
// - 等比例缩放至最长边 = dst（默认 640），居中补边，补边像素 114（Hailo 官方约定）；
// - map_box 把 letterbox 画布坐标映回原图并 clamp；
// - BGR<->RGB 通道翻转（3 通道 HWC，支持原地）。

#pragma once

#include <cstdint>
#include <vector>

namespace raspbot_perception {

constexpr int kDefaultSize = 640;
constexpr uint8_t kPadValue = 114;  // 与 Hailo 官方 yolov8 letterbox 示例一致

struct LetterboxMeta {
  double scale = 1.0;
  int pad_x = 0;
  int pad_y = 0;
  int new_w = 0;
  int new_h = 0;
};

// 计算 letterbox 缩放与补边参数（语义与 Python letterbox_meta 一致）。
// 例：640x480 -> (1.0, 0, 80, 640, 480)；1080x1920 -> (1/3, 140, 0, 360, 640)。
LetterboxMeta letterbox_meta(int src_w, int src_h, int dst = kDefaultSize);

struct Box2D {
  double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
};

// letterbox 画布（dst 空间）像素坐标 -> 原图坐标，clamp 到 [0, src_w]/[0, src_h]。
Box2D map_box(double x1, double y1, double x2, double y2, const LetterboxMeta& meta,
              int src_w, int src_h);

// BGR <-> RGB 通道翻转（pixels = w*h）。src 与 dst 可指向同一块内存（原地翻转）。
void bgr_to_rgb(const uint8_t* src, uint8_t* dst, size_t pixels);
void rgb_to_bgr(const uint8_t* src, uint8_t* dst, size_t pixels);

// HxWx3 uint8（HWC）-> dst x dst x 3，居中补 pad_value（双线性缩放；scale==1 时
// 直接拷贝行）。out 会被 resize 为 dst*dst*3。
void letterbox_image(const uint8_t* img, int w, int h, const LetterboxMeta& meta,
                     std::vector<uint8_t>& out, int dst = kDefaultSize,
                     uint8_t pad_value = kPadValue);

}  // namespace raspbot_perception
