// letterbox 数学测试（C++ 重写，1:1 映射 Python tests/test_letterbox.py 的 9 用例）。
// 纯函数测试，不依赖 NPU/ROS/OpenCV。

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

#include "raspbot_perception/letterbox.hpp"

using raspbot_perception::Box2D;
using raspbot_perception::LetterboxMeta;
using raspbot_perception::bgr_to_rgb;
using raspbot_perception::kPadValue;
using raspbot_perception::letterbox_image;
using raspbot_perception::letterbox_meta;
using raspbot_perception::map_box;

// Python: test_meta_640x480
TEST(LetterboxTest, Meta640x480) {
  const LetterboxMeta meta = letterbox_meta(640, 480, 640);
  EXPECT_NEAR(meta.scale, 1.0, 1e-12);
  EXPECT_EQ(meta.pad_x, 0);
  EXPECT_EQ(meta.pad_y, 80);
  EXPECT_EQ(meta.new_w, 640);
  EXPECT_EQ(meta.new_h, 480);
  EXPECT_LE(meta.new_w + 2 * meta.pad_x, 640);
  EXPECT_EQ(meta.new_h + 2 * meta.pad_y, 640);
}

// Python: test_meta_portrait_1080x1920
TEST(LetterboxTest, MetaPortrait1080x1920) {
  const LetterboxMeta meta = letterbox_meta(1080, 1920, 640);
  EXPECT_NEAR(meta.scale, 640.0 / 1920, 1e-12);
  EXPECT_EQ(meta.new_w, 360);
  EXPECT_EQ(meta.new_h, 640);
  EXPECT_EQ(meta.pad_x, 140);
  EXPECT_EQ(meta.pad_y, 0);
  EXPECT_EQ(meta.new_w + 2 * meta.pad_x, 640);
  EXPECT_EQ(meta.new_h + 2 * meta.pad_y, 640);
}

// Python: test_meta_landscape_1920x1080
TEST(LetterboxTest, MetaLandscape1920x1080) {
  const LetterboxMeta meta = letterbox_meta(1920, 1080, 640);
  EXPECT_NEAR(meta.scale, 640.0 / 1920, 1e-12);
  EXPECT_EQ(meta.new_w, 640);
  EXPECT_EQ(meta.new_h, 360);
  EXPECT_EQ(meta.pad_x, 0);
  EXPECT_EQ(meta.pad_y, 140);
}

// Python: test_map_box_identity_scale
TEST(LetterboxTest, MapBoxIdentityScale) {
  const LetterboxMeta meta{1.0, 0, 80, 640, 480};
  const Box2D box = map_box(320, 320, 400, 400, meta, 640, 480);
  EXPECT_DOUBLE_EQ(box.x1, 320.0);
  EXPECT_DOUBLE_EQ(box.y1, 240.0);
  EXPECT_DOUBLE_EQ(box.x2, 400.0);
  EXPECT_DOUBLE_EQ(box.y2, 320.0);
}

// Python: test_map_box_roundtrip
TEST(LetterboxTest, MapBoxRoundtrip) {
  const int src_w = 1080, src_h = 1920;
  const LetterboxMeta meta = letterbox_meta(src_w, src_h, 640);
  const double boxes[][4] = {{100, 200, 800, 1500},
                             {0, 0, static_cast<double>(src_w), static_cast<double>(src_h)},
                             {50, 50, 120, 130}};
  for (const auto& b : boxes) {
    const double lb_x1 = b[0] * meta.scale + meta.pad_x;
    const double lb_y1 = b[1] * meta.scale + meta.pad_y;
    const double lb_x2 = b[2] * meta.scale + meta.pad_x;
    const double lb_y2 = b[3] * meta.scale + meta.pad_y;
    const Box2D back = map_box(lb_x1, lb_y1, lb_x2, lb_y2, meta, src_w, src_h);
    EXPECT_NEAR(back.x1, b[0], 1e-6);
    EXPECT_NEAR(back.y1, b[1], 1e-6);
    EXPECT_NEAR(back.x2, b[2], 1e-6);
    EXPECT_NEAR(back.y2, b[3], 1e-6);
  }
}

// Python: test_map_box_clamp
TEST(LetterboxTest, MapBoxClamp) {
  const LetterboxMeta meta{1.0, 0, 0, 640, 480};
  const Box2D box = map_box(-100, -50, 9999, 9999, meta, 640, 480);
  EXPECT_DOUBLE_EQ(box.x1, 0.0);
  EXPECT_DOUBLE_EQ(box.y1, 0.0);
  EXPECT_DOUBLE_EQ(box.x2, 640.0);
  EXPECT_DOUBLE_EQ(box.y2, 480.0);
}

// Python: test_bgr_to_rgb
TEST(LetterboxTest, BgrToRgb) {
  // 2 像素：[[1,2,3],[4,5,6]]
  const std::vector<uint8_t> img = {1, 2, 3, 4, 5, 6};
  std::vector<uint8_t> out(6, 0);
  bgr_to_rgb(img.data(), out.data(), 2);
  EXPECT_EQ(out[0], 3);
  EXPECT_EQ(out[1], 2);
  EXPECT_EQ(out[2], 1);
  EXPECT_EQ(out[3], 6);
  EXPECT_EQ(out[4], 5);
  EXPECT_EQ(out[5], 4);
  // 原数组不变
  EXPECT_EQ(img[0], 1);
  EXPECT_EQ(img[1], 2);
  EXPECT_EQ(img[2], 3);
  // 支持原地翻转
  std::vector<uint8_t> in_place = img;
  bgr_to_rgb(in_place.data(), in_place.data(), 2);
  EXPECT_EQ(in_place[0], 3);
  EXPECT_EQ(in_place[1], 2);
  EXPECT_EQ(in_place[2], 1);
}

// Python: test_letterbox_image_pad_and_content
TEST(LetterboxTest, LetterboxImagePadAndContent) {
  // 2x1 (w=2,h=1) -> dst=4：scale=2, new_w=4, new_h=2, pad_y=1
  const std::vector<uint8_t> img = {10, 20, 30, 40, 50, 60};
  std::vector<uint8_t> out;
  const LetterboxMeta meta = letterbox_meta(2, 1, 4);
  letterbox_image(img.data(), 2, 1, meta, out, 4);
  ASSERT_EQ(out.size(), 4u * 4 * 3);
  // 上下补边为 114
  for (int x = 0; x < 4; ++x) {
    for (int c = 0; c < 3; ++c) {
      EXPECT_EQ(out[(0 * 4 + x) * 3 + c], kPadValue);
      EXPECT_EQ(out[(3 * 4 + x) * 3 + c], kPadValue);
    }
  }
  // 中心行是缩放内容，不应全是补边值
  bool non_pad = false;
  for (int x = 0; x < 4; ++x) {
    if (out[(1 * 4 + x) * 3 + 0] != kPadValue) {
      non_pad = true;
    }
  }
  EXPECT_TRUE(non_pad);
}

// Python: test_letterbox_image_640x480
TEST(LetterboxTest, LetterboxImage640x480) {
  // 640x480 -> 640x640：scale=1 无插值，仅上下补边，内容逐字节一致
  std::mt19937 rng(0);
  std::uniform_int_distribution<int> dist(0, 255);
  std::vector<uint8_t> img(static_cast<size_t>(480) * 640 * 3);
  for (auto& v : img) {
    v = static_cast<uint8_t>(dist(rng));
  }
  std::vector<uint8_t> out;
  const LetterboxMeta meta = letterbox_meta(640, 480, 640);
  letterbox_image(img.data(), 640, 480, meta, out, 640);
  ASSERT_EQ(out.size(), 640u * 640 * 3);
  // 中间 480 行与原图一致
  for (int row = 0; row < 480; ++row) {
    for (size_t c = 0; c < static_cast<size_t>(640) * 3; ++c) {
      EXPECT_EQ(out[(static_cast<size_t>(80 + row) * 640) * 3 + c],
                img[static_cast<size_t>(row) * 640 * 3 + c]);
    }
  }
  // 上下补边为 114
  for (int row = 0; row < 80; ++row) {
    for (size_t c = 0; c < static_cast<size_t>(640) * 3; ++c) {
      EXPECT_EQ(out[static_cast<size_t>(row) * 640 * 3 + c], kPadValue);
      EXPECT_EQ(out[(static_cast<size_t>(560 + row) * 640) * 3 + c], kPadValue);
    }
  }
}
