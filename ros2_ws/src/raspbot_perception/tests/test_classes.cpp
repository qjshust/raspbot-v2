// COCO 80 类名表测试（C++ 重写，1:1 映射 Python tests/test_classes.py 的 4 用例）。

#include <gtest/gtest.h>

#include <set>
#include <string>

#include "raspbot_perception/classes.hpp"

using raspbot_perception::class_name;
using raspbot_perception::kCocoClassCount;
using raspbot_perception::kCocoClasses;

// Python: test_length_is_80
TEST(ClassesTest, LengthIs80) {
  EXPECT_EQ(kCocoClassCount, 80);
  EXPECT_EQ(static_cast<int>(kCocoClasses.size()), 80);
}

// Python: test_no_duplicates_and_order
TEST(ClassesTest, NoDuplicatesAndOrder) {
  std::set<std::string> unique(kCocoClasses.begin(), kCocoClasses.end());
  EXPECT_EQ(unique.size(), 80u);
}

// Python: test_sample_ids
TEST(ClassesTest, SampleIds) {
  const std::pair<int, const char*> samples[] = {
      {0, "person"},   {1, "bicycle"},    {2, "car"},      {3, "motorcycle"},
      {5, "bus"},      {7, "truck"},      {14, "bird"},    {15, "cat"},
      {16, "dog"},     {32, "sports ball"}, {39, "bottle"},{41, "cup"},
      {46, "banana"},  {47, "apple"},     {56, "chair"},   {57, "couch"},
      {58, "potted plant"}, {62, "tv"},   {63, "laptop"},  {67, "cell phone"},
      {76, "scissors"},{77, "teddy bear"},{78, "hair drier"},
      {79, "toothbrush"},
  };
  for (const auto& [class_id, name] : samples) {
    EXPECT_EQ(kCocoClasses[class_id], name);
    EXPECT_EQ(class_name(class_id), name);
  }
}

// Python: test_class_name_unknown
TEST(ClassesTest, ClassNameUnknown) {
  EXPECT_EQ(class_name(-1), "class_-1");
  EXPECT_EQ(class_name(80), "class_80");
}
