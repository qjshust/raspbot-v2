#include "raspbot_perception/nms.hpp"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace raspbot_perception {

namespace {
// 按 [y_min, x_min, y_max, x_max, score] 行序解析一个 (N, 5) 块。
std::vector<Detection> detections_from_rows(const std::vector<HailoBbox>& rows,
                                            int class_id, float score_threshold,
                                            int dst) {
  std::vector<Detection> dets;
  for (const auto& row : rows) {
    if (row.score < score_threshold) {
      continue;
    }
    Detection det;
    det.class_id = class_id;
    det.score = row.score;
    // 与 Python 一致：异常角点归一化为 x1<=x2、y1<=y2，再乘 dst 得画布像素。
    det.x1 = static_cast<float>(std::min(row.x_min, row.x_max) * dst);
    det.x2 = static_cast<float>(std::max(row.x_min, row.x_max) * dst);
    det.y1 = static_cast<float>(std::min(row.y_min, row.y_max) * dst);
    det.y2 = static_cast<float>(std::max(row.y_min, row.y_max) * dst);
    dets.push_back(det);
  }
  return dets;
}
}  // namespace

std::vector<Detection> parse_nms(const ClassesBboxes& classes, int class_count,
                                 float score_threshold, int dst) {
  if (static_cast<int>(classes.size()) != class_count) {
    throw std::invalid_argument(
        "每个 batch 应有 " + std::to_string(class_count) + " 个类数组，实际 " +
        std::to_string(classes.size()));
  }
  std::vector<Detection> detections;
  for (int class_id = 0; class_id < class_count; ++class_id) {
    auto part = detections_from_rows(classes[class_id], class_id, score_threshold,
                                     dst);
    detections.insert(detections.end(), part.begin(), part.end());
  }
  // Python list.sort 是稳定排序：score 相同保持类序。
  std::stable_sort(detections.begin(), detections.end(),
                   [](const Detection& a, const Detection& b) {
                     return a.score > b.score;
                   });
  return detections;
}

std::vector<Detection> parse_nms_raw(const uint8_t* buf, size_t len,
                                     int class_count, float score_threshold,
                                     int dst) {
  // HailoRT HAILO_NMS_BY_CLASS 原始缓冲区：
  // 逐类交错 [float32 count][count x 5 x float32]（见 nms.hpp 头注释）。
  constexpr size_t kCountBytes = sizeof(float);
  constexpr size_t kBboxBytes = 5 * sizeof(float);
  size_t offset = 0;
  std::vector<Detection> detections;
  for (int class_id = 0; class_id < class_count; ++class_id) {
    if (offset + kCountBytes > len) {
      throw std::runtime_error("NMS 原始缓冲区越界（读取 bbox_count）");
    }
    float count_f = 0.0f;
    std::memcpy(&count_f, buf + offset, kCountBytes);
    offset += kCountBytes;
    if (count_f < 0.0f || count_f > 1e6f) {
      throw std::runtime_error("NMS 原始缓冲区 bbox_count 非法: " +
                               std::to_string(count_f));
    }
    const int count = static_cast<int>(count_f);
    if (offset + count * kBboxBytes > len) {
      throw std::runtime_error("NMS 原始缓冲区越界（读取 bbox 数据）");
    }
    std::vector<HailoBbox> rows(count);
    for (int i = 0; i < count; ++i) {
      const auto* p = reinterpret_cast<const float*>(buf + offset);
      rows[i] = HailoBbox{p[0], p[1], p[2], p[3], p[4]};
      offset += kBboxBytes;
    }
    auto part = detections_from_rows(rows, class_id, score_threshold, dst);
    detections.insert(detections.end(), part.begin(), part.end());
  }
  std::stable_sort(detections.begin(), detections.end(),
                   [](const Detection& a, const Detection& b) {
                     return a.score > b.score;
                   });
  return detections;
}

}  // namespace raspbot_perception
