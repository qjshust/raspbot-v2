// COCO 80 类名表（与 COCO 官方顺序一致，id 0..79）。
// 与 Python 版 classes.py（已归档 legacy_python/）1:1。

#pragma once

#include <array>
#include <string>

namespace raspbot_perception {

constexpr int kCocoClassCount = 80;

extern const std::array<std::string, kCocoClassCount> kCocoClasses;

// 返回类名；未知 id 返回 "class_<id>"（与 Python class_name() 一致）。
std::string class_name(int class_id);

}  // namespace raspbot_perception
