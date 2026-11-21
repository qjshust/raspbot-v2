// YUYV(4:2:2) -> RGB 转换测试（C++ 新增功能，Python 版无此入口）。
// BT.601 limited range 定点公式，期望值与 OpenCV 结果一致（容差 ±3）。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "raspbot_perception/image_utils.hpp"

using raspbot_perception::yuyv_to_rgb;

namespace {
std::vector<uint8_t> yuyv_of(int y, int u, int v, int pairs = 1) {
  std::vector<uint8_t> buf;
  for (int i = 0; i < pairs; ++i) {
    buf.insert(buf.end(), {static_cast<uint8_t>(y), static_cast<uint8_t>(u),
                           static_cast<uint8_t>(y), static_cast<uint8_t>(v)});
  }
  return buf;
}
}  // namespace

TEST(YuyvTest, Black) {
  const auto yuyv = yuyv_of(16, 128, 128);
  std::vector<uint8_t> rgb(6, 255);
  yuyv_to_rgb(yuyv.data(), 2, 1, rgb.data());
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(rgb[i], 0, 3);
  }
}

TEST(YuyvTest, White) {
  const auto yuyv = yuyv_of(235, 128, 128);
  std::vector<uint8_t> rgb(6, 0);
  yuyv_to_rgb(yuyv.data(), 2, 1, rgb.data());
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(rgb[i], 255, 3);
  }
}

TEST(YuyvTest, RedAndGray) {
  // 两个像素对（4 像素）：红(Y=81,U=90,V=240) + 中灰(Y=128,U=128,V=128)
  // 布局 [Y0 U Y1 V][Y0 U Y1 V]；像素 0/1 红，像素 2/3 灰
  const std::vector<uint8_t> yuyv = {81, 90, 81, 240, 128, 128, 128, 128};
  std::vector<uint8_t> rgb(4 * 3, 0);
  yuyv_to_rgb(yuyv.data(), 4, 1, rgb.data());
  // 像素 0（红）：R≈255, G≈0, B≈0
  EXPECT_NEAR(rgb[0], 255, 4);
  EXPECT_NEAR(rgb[1], 0, 4);
  EXPECT_NEAR(rgb[2], 0, 4);
  // 像素 1（同对，红）：同上
  EXPECT_NEAR(rgb[3], 255, 4);
  EXPECT_NEAR(rgb[4], 0, 4);
  EXPECT_NEAR(rgb[5], 0, 4);
  // 像素 2（中灰）：三通道 ≈ 130（BT.601 limited 公式特性，与 OpenCV 一致）
  EXPECT_NEAR(rgb[6], 130, 4);
  EXPECT_NEAR(rgb[7], 130, 4);
  EXPECT_NEAR(rgb[8], 130, 4);
}

TEST(YuyvTest, TwoRowsOrder) {
  // 4x2：第 0 行全黑，第 1 行全白，验证行序与步长
  std::vector<uint8_t> yuyv;
  yuyv.insert(yuyv.end(), {16, 128, 16, 128, 16, 128, 16, 128});  // 行0 黑
  yuyv.insert(yuyv.end(), {235, 128, 235, 128, 235, 128, 235, 128});  // 行1 白
  std::vector<uint8_t> rgb(4 * 2 * 3, 0);
  yuyv_to_rgb(yuyv.data(), 4, 2, rgb.data());
  for (int i = 0; i < 12; ++i) {
    EXPECT_NEAR(rgb[i], 0, 3);  // 行0
  }
  for (int i = 12; i < 24; ++i) {
    EXPECT_NEAR(rgb[i], 255, 3);  // 行1
  }
}

TEST(YuyvTest, OddWidthThrows) {
  const std::vector<uint8_t> yuyv(3 * 2, 128);
  EXPECT_THROW(yuyv_to_rgb(yuyv.data(), 3, 1, nullptr),
               std::invalid_argument);
}
