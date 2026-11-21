#include "raspbot_perception/letterbox.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace raspbot_perception {

LetterboxMeta letterbox_meta(int src_w, int src_h, int dst) {
  LetterboxMeta meta;
  // 与 Python 版运算顺序一致（double，先除后乘再 round），保证测试数值 1:1。
  meta.scale = std::min(static_cast<double>(dst) / src_w,
                        static_cast<double>(dst) / src_h);
  meta.new_w = static_cast<int>(std::round(src_w * meta.scale));
  meta.new_h = static_cast<int>(std::round(src_h * meta.scale));
  meta.pad_x = (dst - meta.new_w) / 2;
  meta.pad_y = (dst - meta.new_h) / 2;
  return meta;
}

Box2D map_box(double x1, double y1, double x2, double y2, const LetterboxMeta& meta,
              int src_w, int src_h) {
  x1 = (x1 - meta.pad_x) / meta.scale;
  y1 = (y1 - meta.pad_y) / meta.scale;
  x2 = (x2 - meta.pad_x) / meta.scale;
  y2 = (y2 - meta.pad_y) / meta.scale;
  Box2D box;
  box.x1 = std::clamp(x1, 0.0, static_cast<double>(src_w));
  box.y1 = std::clamp(y1, 0.0, static_cast<double>(src_h));
  box.x2 = std::clamp(x2, 0.0, static_cast<double>(src_w));
  box.y2 = std::clamp(y2, 0.0, static_cast<double>(src_h));
  return box;
}

namespace {
void flip_channels(const uint8_t* src, uint8_t* dst, size_t pixels) {
  if (src == dst) {
    for (size_t i = 0; i < pixels; ++i) {
      uint8_t* p = dst + i * 3;
      std::swap(p[0], p[2]);
    }
  } else {
    for (size_t i = 0; i < pixels; ++i) {
      const uint8_t* s = src + i * 3;
      uint8_t* d = dst + i * 3;
      d[0] = s[2];
      d[1] = s[1];
      d[2] = s[0];
    }
  }
}

// 双线性缩放（像素中心对齐，与 numpy/OpenCV INTER_LINEAR 一致；四舍五入）。
void resize_bilinear(const uint8_t* src, int sw, int sh, uint8_t* dst, int dw,
                     int dh) {
  for (int dy = 0; dy < dh; ++dy) {
    const double sy = std::clamp((dy + 0.5) * static_cast<double>(sh) / dh - 0.5,
                                 0.0, static_cast<double>(sh - 1));
    const int y0 = static_cast<int>(std::floor(sy));
    const int y1 = std::min(y0 + 1, sh - 1);
    const double wy = sy - y0;
    for (int dx = 0; dx < dw; ++dx) {
      const double sx = std::clamp((dx + 0.5) * static_cast<double>(sw) / dw - 0.5,
                                   0.0, static_cast<double>(sw - 1));
      const int x0 = static_cast<int>(std::floor(sx));
      const int x1 = std::min(x0 + 1, sw - 1);
      const double wx = sx - x0;
      const double w00 = (1.0 - wy) * (1.0 - wx);
      const double w10 = wy * (1.0 - wx);
      const double w01 = (1.0 - wy) * wx;
      const double w11 = wy * wx;
      uint8_t* d = dst + (dy * dw + dx) * 3;
      const uint8_t* a = src + (y0 * sw + x0) * 3;
      const uint8_t* b = src + (y1 * sw + x0) * 3;
      const uint8_t* c = src + (y0 * sw + x1) * 3;
      const uint8_t* e = src + (y1 * sw + x1) * 3;
      for (int ch = 0; ch < 3; ++ch) {
        const double v = w00 * a[ch] + w10 * b[ch] + w01 * c[ch] + w11 * e[ch];
        d[ch] = static_cast<uint8_t>(std::lround(v));
      }
    }
  }
}
}  // namespace

void bgr_to_rgb(const uint8_t* src, uint8_t* dst, size_t pixels) {
  flip_channels(src, dst, pixels);
}

void rgb_to_bgr(const uint8_t* src, uint8_t* dst, size_t pixels) {
  flip_channels(src, dst, pixels);
}

void letterbox_image(const uint8_t* img, int w, int h, const LetterboxMeta& meta,
                     std::vector<uint8_t>& out, int dst, uint8_t pad_value) {
  out.assign(static_cast<size_t>(dst) * dst * 3, pad_value);
  if (meta.new_w <= 0 || meta.new_h <= 0) {
    return;
  }
  if (meta.new_w == w && meta.new_h == h) {
    // scale == 1：直接逐行拷贝（与 Python 的 assign 语义一致）
    for (int row = 0; row < h; ++row) {
      std::memcpy(out.data() + (static_cast<size_t>(meta.pad_y + row) * dst +
                                meta.pad_x) *
                                   3,
                  img + static_cast<size_t>(row) * w * 3,
                  static_cast<size_t>(w) * 3);
    }
    return;
  }
  std::vector<uint8_t> resized(static_cast<size_t>(meta.new_w) * meta.new_h * 3);
  resize_bilinear(img, w, h, resized.data(), meta.new_w, meta.new_h);
  for (int row = 0; row < meta.new_h; ++row) {
    std::memcpy(out.data() + (static_cast<size_t>(meta.pad_y + row) * dst +
                              meta.pad_x) *
                                 3,
                resized.data() + static_cast<size_t>(row) * meta.new_w * 3,
                static_cast<size_t>(meta.new_w) * 3);
  }
}

}  // namespace raspbot_perception
