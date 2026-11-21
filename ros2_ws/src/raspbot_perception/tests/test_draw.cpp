// 检测框/标签绘制测试（C++ 新增：纯 C++ 5x7 点阵标注，替代 cv2.rectangle/putText）。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "raspbot_perception/image_utils.hpp"

using raspbot_perception::draw_detection;

namespace {
// BGR 绿色 (0,255,0)
bool is_green(const std::vector<uint8_t>& img, int w, int x, int y) {
  const auto* p = img.data() + (static_cast<size_t>(y) * w + x) * 3;
  return p[0] == 0 && p[1] == 255 && p[2] == 0;
}

int count_non_gray(const std::vector<uint8_t>& img, int w, int x0, int y0,
                   int x1, int y1) {
  int count = 0;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const auto* p = img.data() + (static_cast<size_t>(y) * w + x) * 3;
      if (!(p[0] == 60 && p[1] == 60 && p[2] == 60)) {
        ++count;
      }
    }
  }
  return count;
}
}  // namespace

TEST(DrawTest, BoxOutlineAndLabel) {
  // 100x80 灰底 (60,60,60)
  std::vector<uint8_t> img(static_cast<size_t>(100) * 80 * 3, 0);
  for (auto& v : img) {
    v = 60;
  }
  draw_detection(img.data(), 100, 80, 10, 30, 50, 60, "car 0.90");
  // 框左上角为绿色边框
  EXPECT_TRUE(is_green(img, 100, 10, 30));
  EXPECT_TRUE(is_green(img, 100, 50, 60));
  // 框外仍为灰底
  EXPECT_FALSE(is_green(img, 100, 9, 29));
  // 标签区域（框上方 y≈21..28）出现非灰像素（文字）
  const int text_pixels = count_non_gray(img, 100, 12, 20, 60, 28);
  EXPECT_GT(text_pixels, 10);
}

TEST(DrawTest, OutOfBoundsBoxClamped) {
  // 越界框被 clamp，不越界写内存、不崩溃
  std::vector<uint8_t> img(static_cast<size_t>(100) * 80 * 3, 60);
  draw_detection(img.data(), 100, 80, -500, -500, 500, 500, "person 1.00");
  EXPECT_TRUE(is_green(img, 100, 0, 0));
  EXPECT_TRUE(is_green(img, 100, 99, 79));
}
