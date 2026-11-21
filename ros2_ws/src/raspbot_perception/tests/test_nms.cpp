// NMS 输出解析测试（C++ 重写，1:1 映射 Python tests/test_postprocess.py 的 10 用例）。
//
// 语义映射说明：
// - Python "hailo 原生嵌套结构" 与 "(1,80,N,5)/(80,N,5) ndarray" 输入
//   -> 本模块 per-class vector<vector<HailoBbox>>（ragged）与
//      HailoRT HAILO_NMS_BY_CLASS 原始缓冲区（parse_nms_raw）；
// - Python "非法输入抛 ValueError" -> parse_nms 类数组数不符抛
//   std::invalid_argument，parse_nms_raw 缓冲区损坏抛 std::runtime_error。

#include <gtest/gtest.h>

#include <cstring>
#include <stdexcept>
#include <vector>

#include "raspbot_perception/nms.hpp"

using raspbot_perception::ClassesBboxes;
using raspbot_perception::Detection;
using raspbot_perception::HailoBbox;
using raspbot_perception::parse_nms;
using raspbot_perception::parse_nms_raw;

namespace {

ClassesBboxes empty_frame(int class_count = 80) {
  return ClassesBboxes(class_count);
}

HailoBbox bbox(float y_min, float x_min, float y_max, float x_max, float score) {
  return HailoBbox{y_min, x_min, y_max, x_max, score};
}

}  // namespace

// Python: test_single_detection_hailo_structure
TEST(NmsTest, SingleDetectionHailoStructure) {
  ClassesBboxes out = empty_frame();
  out[0] = {bbox(0.10f, 0.20f, 0.30f, 0.40f, 0.90f)};
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 1u);
  const Detection& d = dets[0];
  EXPECT_EQ(d.class_id, 0);
  EXPECT_NEAR(d.score, 0.90f, 1e-6);
  EXPECT_NEAR(d.x1, 0.20 * 640, 1e-4);
  EXPECT_NEAR(d.y1, 0.10 * 640, 1e-4);
  EXPECT_NEAR(d.x2, 0.40 * 640, 1e-4);
  EXPECT_NEAR(d.y2, 0.30 * 640, 1e-4);
}

// Python: test_ndarray_4d_input（(1,80,2,5)：每类 2 行，仅 class 15 有检测）
TEST(NmsTest, Ndarray4dInput) {
  ClassesBboxes out(80, std::vector<HailoBbox>(2));
  out[15] = {bbox(0.5f, 0.5f, 0.6f, 0.6f, 0.8f),
             bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.9f)};
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_EQ(dets[0].class_id, 15);
  EXPECT_EQ(dets[1].class_id, 15);
  // 按 score 降序
  EXPECT_NEAR(dets[0].score, 0.9f, 1e-6);
  EXPECT_NEAR(dets[1].score, 0.8f, 1e-6);
}

// Python: test_ndarray_3d_input（(80,1,5)：每类 1 行，class 79 有检测）
TEST(NmsTest, Ndarray3dInput) {
  ClassesBboxes out(80, std::vector<HailoBbox>(1));
  out[79] = {bbox(0.0f, 0.0f, 0.25f, 0.25f, 0.6f)};
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_EQ(dets[0].class_id, 79);
  EXPECT_NEAR(dets[0].x2, 160.0f, 1e-4);
}

// Python: test_score_threshold_filtering
TEST(NmsTest, ScoreThresholdFiltering) {
  ClassesBboxes out = empty_frame();
  out[0] = {bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.95f),   // 保留
            bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.39f),   // 丢弃（< 0.4）
            bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.40f)};  // 保留（>= 0.4）
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_NEAR(dets[0].score, 0.95f, 1e-6);
  EXPECT_NEAR(dets[1].score, 0.40f, 1e-6);
}

// Python: test_multi_class_class_id_indexing
TEST(NmsTest, MultiClassClassIdIndexing) {
  ClassesBboxes out = empty_frame();
  out[0] = {bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.5f)};
  out[62] = {bbox(0.1f, 0.1f, 0.2f, 0.2f, 0.7f)};
  out[79] = {bbox(0.2f, 0.2f, 0.3f, 0.3f, 0.9f)};
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 3u);
  EXPECT_EQ(dets[0].class_id, 79);
  EXPECT_EQ(dets[1].class_id, 62);
  EXPECT_EQ(dets[2].class_id, 0);
}

// Python: test_sorted_by_score_desc
TEST(NmsTest, SortedByScoreDesc) {
  ClassesBboxes out = empty_frame();
  out[0] = {bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.5f),
            bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.95f),
            bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.7f)};
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 3u);
  EXPECT_GT(dets[0].score, dets[1].score);
  EXPECT_GT(dets[1].score, dets[2].score);
}

// Python: test_swapped_corners_are_normalized
TEST(NmsTest, SwappedCornersAreNormalized) {
  ClassesBboxes out = empty_frame();
  out[0] = {bbox(0.4f, 0.3f, 0.1f, 0.2f, 0.8f)};
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  ASSERT_EQ(dets.size(), 1u);
  EXPECT_LE(dets[0].x1, dets[0].x2);
  EXPECT_LE(dets[0].y1, dets[0].y2);
}

// Python: test_empty_output
TEST(NmsTest, EmptyOutput) {
  EXPECT_TRUE(parse_nms(empty_frame(), 80, 0.4f, 640).empty());
}

// Python: test_ragged_hailo_structure
TEST(NmsTest, RaggedHailoStructure) {
  ClassesBboxes out = empty_frame();
  out[0] = std::vector<HailoBbox>(3, bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.5f));
  out[1] = std::vector<HailoBbox>(1, bbox(0.0f, 0.0f, 0.1f, 0.1f, 0.6f));
  const auto dets = parse_nms(out, 80, 0.4f, 640);
  EXPECT_EQ(dets.size(), 4u);
}

// Python: test_invalid_input_raises（语义映射：类数组数不符 / 原始缓冲区损坏）
TEST(NmsTest, InvalidInputRaises) {
  // 类数组数不符（Python: [np.zeros((80,5))] 单数组输入）
  EXPECT_THROW(parse_nms(ClassesBboxes(1), 80, 0.4f, 640), std::invalid_argument);
  // 原始缓冲区越界：count 声称 999 但缓冲区不够（首个类即抛）
  std::vector<uint8_t> buf(4 + 8 * 5 * 4 + 8 * 4, 0);
  auto* counts = reinterpret_cast<float*>(buf.data());
  counts[0] = 999.0f;  // 非法大 count
  EXPECT_THROW(parse_nms_raw(buf.data(), buf.size(), 80, 0.4f, 640),
               std::runtime_error);
}

// ---------------------------------------------------------------------------
// 附加：HailoRT NMS BY CLASS 原始缓冲区解析（interleaved [count][bboxes]）。
// Python 版无此入口（绑定内部转换），属 C++ 新增入口的正向覆盖。

TEST(NmsTest, RawBufferInterleavedLayout) {
  // 布局：c0: count=1, bbox；c1: count=0；... c79: count=1, bbox
  std::vector<float> floats;
  auto push_bbox = [&](float y1, float x1, float y2, float x2, float s) {
    floats.insert(floats.end(), {y1, x1, y2, x2, s});
  };
  for (int c = 0; c < 80; ++c) {
    if (c == 0) {
      floats.push_back(1.0f);
      push_bbox(0.0f, 0.0f, 0.25f, 0.25f, 0.6f);
    } else if (c == 79) {
      floats.push_back(1.0f);
      push_bbox(0.5f, 0.5f, 0.75f, 0.75f, 0.9f);
    } else {
      floats.push_back(0.0f);
    }
  }
  const uint8_t* buf = reinterpret_cast<const uint8_t*>(floats.data());
  const auto dets = parse_nms_raw(buf, floats.size() * sizeof(float), 80, 0.4f,
                                  640);
  ASSERT_EQ(dets.size(), 2u);
  EXPECT_EQ(dets[0].class_id, 79);
  EXPECT_NEAR(dets[0].score, 0.9f, 1e-6);
  EXPECT_NEAR(dets[1].x2, 160.0f, 1e-4);
  EXPECT_EQ(dets[1].class_id, 0);
}
