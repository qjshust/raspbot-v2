// HailoRT 原生 C++ API 封装（hailort.hpp，4.23）：
//   VDevice::create -> create_infer_model(hef) -> InferModel(set_batch_size,
//   set_format_type) -> configure() -> ConfiguredInferModel -> create_bindings
//   -> activate -> run_async -> AsyncInferJob::wait。
// 仅在找到 HailoRT 头/库时编译（见 CMakeLists.txt）；纯函数库与 gtest 不依赖本类。

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hailo/hailort.hpp"

namespace raspbot_perception {

class HailoBackend {
 public:
  // 加载 HEF 并激活网络。失败抛 std::runtime_error（含 hailo_status）。
  // device_timeout_ms：>0 时作为 wait/scheduler 超时；<=0 用默认（1 s）。
  explicit HailoBackend(const std::string& hef_path, int device_timeout_ms = 0);
  ~HailoBackend();

  HailoBackend(const HailoBackend&) = delete;
  HailoBackend& operator=(const HailoBackend&) = delete;

  // 同步单帧推理：letterbox 后的 RGB 640x640x3（UINT8 输入）。
  // 返回 HAILO_NMS_BY_CLASS 原始输出缓冲区（interleaved，见 nms.hpp）。
  const std::vector<uint8_t>& infer(const uint8_t* rgb640);

  int dst() const { return dst_; }
  int class_count() const { return class_count_; }
  size_t output_frame_size() const { return output_frame_size_; }
  const std::string& input_name() const { return input_name_; }
  const std::string& output_name() const { return output_name_; }

 private:
  std::unique_ptr<hailort::VDevice> vdevice_;
  std::shared_ptr<hailort::InferModel> model_;
  hailort::ConfiguredInferModel configured_;
  hailort::ConfiguredInferModel::Bindings bindings_;
  std::string input_name_;
  std::string output_name_;
  int dst_ = 0;
  int class_count_ = 0;
  size_t input_frame_size_ = 0;
  size_t output_frame_size_ = 0;
  std::chrono::milliseconds wait_timeout_{1000};
  std::vector<uint8_t> raw_output_;
  std::vector<uint8_t> out_buf_;  // 手动注册的输出缓冲区（NMS raw）
  bool activated_ = false;
};

}  // namespace raspbot_perception
