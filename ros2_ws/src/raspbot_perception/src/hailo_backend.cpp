#include "raspbot_perception/hailo_backend.hpp"

#include <stdexcept>

namespace raspbot_perception {

namespace {
std::string status_str(hailo_status status) {
  return std::to_string(static_cast<int>(status));
}
}  // namespace

HailoBackend::HailoBackend(const std::string& hef_path, int device_timeout_ms) {
  // 1. VDevice
  auto vdevice = hailort::VDevice::create();
  if (!vdevice) {
    throw std::runtime_error("HailoBackend: VDevice::create 失败, status=" +
                             status_str(vdevice.status()));
  }
  vdevice_ = std::move(vdevice.value());

  // 2. InferModel
  auto model = vdevice_->create_infer_model(hef_path);
  if (!model) {
    throw std::runtime_error("HailoBackend: create_infer_model 失败 (" + hef_path +
                             "), status=" + status_str(model.status()));
  }
  model_ = std::move(model.value());
  model_->set_batch_size(1);

  // 3. 输入/输出格式（HEF 输入 UINT8 NHWC 640x640x3；输出 NMS BY CLASS FLOAT32）
  auto in = model_->input();
  if (!in) {
    throw std::runtime_error("HailoBackend: 无输入流, status=" +
                             status_str(in.status()));
  }
  input_name_ = in->name();
  in->set_format_type(HAILO_FORMAT_TYPE_UINT8);
  input_frame_size_ = in->get_frame_size();
  dst_ = static_cast<int>(in->shape().height);  // 640
  if (in->shape().width != static_cast<uint32_t>(dst_) ||
      in->shape().features != 3) {
    throw std::runtime_error(
        "HailoBackend: 仅支持正方形 3 通道输入 HEF，实际 " +
        std::to_string(in->shape().height) + "x" +
        std::to_string(in->shape().width) + "x" +
        std::to_string(in->shape().features));
  }

  auto out = model_->output();
  if (!out) {
    throw std::runtime_error("HailoBackend: 无输出流, status=" +
                             status_str(out.status()));
  }
  output_name_ = out->name();
  out->set_format_type(HAILO_FORMAT_TYPE_FLOAT32);
  output_frame_size_ = out->get_frame_size();
  auto nms_shape = out->get_nms_shape();
  if (nms_shape) {
    class_count_ = static_cast<int>(nms_shape->number_of_classes);
  }

  // 4. configure + bindings
  auto configured = model_->configure();
  if (!configured) {
    throw std::runtime_error("HailoBackend: configure 失败, status=" +
                             status_str(configured.status()));
  }
  configured_ = std::move(configured.value());

  auto bindings = configured_.create_bindings();
  if (!bindings) {
    throw std::runtime_error("HailoBackend: create_bindings 失败, status=" +
                             status_str(bindings.status()));
  }
  bindings_ = std::move(bindings.value());

  // 手动注册输出缓冲区（NMS raw interleaved）：
  // 实测调度器模式下框架托管输出在此 HEF 上找不到缓冲
  //（"Couldnt find output buffer for 'yolov8s/yolov8_nms_postprocess'"），
  // 显式 set_buffer 后 run_async 即正常。
  out_buf_.resize(output_frame_size_);
  auto out_stream = bindings_.output();
  if (!out_stream) {
    throw std::runtime_error("HailoBackend: bindings.output 失败, status=" +
                             status_str(out_stream.status()));
  }
  const hailo_status ob_st = out_stream->set_buffer(
      hailort::MemoryView(out_buf_.data(), output_frame_size_));
  if (ob_st != HAILO_SUCCESS) {
    throw std::runtime_error("HailoBackend: output set_buffer 失败, status=" +
                             status_str(ob_st));
  }

  // 5. 调度器模式：core-op 调度器默认启用，禁止手动 activate
  //（手动 activate 返回 HAILO_INVALID_OPERATION）。等待异步就绪即可推理。
  const hailo_status ready_st =
      configured_.wait_for_async_ready(std::chrono::seconds(5));
  if (ready_st != HAILO_SUCCESS) {
    throw std::runtime_error("HailoBackend: wait_for_async_ready 超时, status=" +
                             status_str(ready_st));
  }
  activated_ = true;

  if (device_timeout_ms > 0) {
    wait_timeout_ = std::chrono::milliseconds(device_timeout_ms);
    // 调度器模式下生效；非调度模式返回值可忽略
    configured_.set_scheduler_timeout(wait_timeout_);
  }
}

HailoBackend::~HailoBackend() {
  if (activated_) {
    configured_.shutdown();
  }
}

const std::vector<uint8_t>& HailoBackend::infer(const uint8_t* rgb640) {
  // 输入缓冲区绑定到本帧数据（异步作业期间内存须保持有效——同步 wait 保证）
  auto in_stream = bindings_.input();
  if (!in_stream) {
    throw std::runtime_error("HailoBackend: bindings.input 失败, status=" +
                             status_str(in_stream.status()));
  }
  const hailo_status st = in_stream->set_buffer(hailort::MemoryView(
      const_cast<uint8_t*>(rgb640), input_frame_size_));
  if (st != HAILO_SUCCESS) {
    throw std::runtime_error("HailoBackend: set_buffer 失败, status=" +
                             status_str(st));
  }

  // 异步提交 + 等待完成（单流逐帧；回调为空）
  auto job = configured_.run_async(bindings_, hailort::ASYNC_INFER_EMPTY_CALLBACK);
  if (!job) {
    throw std::runtime_error("HailoBackend: run_async 失败, status=" +
                             status_str(job.status()));
  }
  const hailo_status wait_st = job->wait(wait_timeout_);
  if (wait_st != HAILO_SUCCESS) {
    throw std::runtime_error("HailoBackend: infer 超时/失败, status=" +
                             status_str(wait_st));
  }

  auto out_stream = bindings_.output();
  if (!out_stream) {
    throw std::runtime_error("HailoBackend: bindings.output 失败, status=" +
                             status_str(out_stream.status()));
  }
  // 输出缓冲已手动注册（构造时 set_buffer），get_buffer 返回其 MemoryView
  auto view = out_stream->get_buffer();
  if (!view) {
    throw std::runtime_error("HailoBackend: get_buffer 失败, status=" +
                             status_str(view.status()));
  }
  raw_output_.assign(view->data(), view->data() + output_frame_size_);
  return raw_output_;
}

}  // namespace raspbot_perception
