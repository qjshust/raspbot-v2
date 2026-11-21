// hailo_node：Hailo-8 YOLOv8 感知节点（WP-I，C++ 版）。
//
// 契约（与 Python 版一致，见 CONTRACTS.md §2）：
// - 订阅 /image_raw（sensor_msgs/Image，bgr8/rgb8/yuyv/yuv422，SensorData QoS）
// - 发布 /detections（vision_msgs/Detection2DArray，header.frame_id=camera_link、
//   header.stamp=输入帧 stamp；bbox.center/size 为归一化 [0,1] 相对坐标）
// - 发布 /perception/image（sensor_msgs/Image bgr8 标注图，header 同输入帧）
// - 不依赖 cv_bridge / OpenCV：解码、letterbox、YUYV->RGB、标注均为纯 C++ 实现
// - 参数与 Python 版一致：hef_path/score_threshold/max_rate/publish_annotated/
//   device_timeout/fail_fast
// - 模型 /opt/hailo-models/yolov8s_h8.hef；输出布局 ragged 80x(N,5)
//   [y_min,x_min,y_max,x_max,score] 归一化 0..1（详见 nms.hpp 实测证据）

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/point2_d.hpp>
#include <vision_msgs/msg/pose2_d.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "raspbot_perception/classes.hpp"
#include "raspbot_perception/hailo_backend.hpp"
#include "raspbot_perception/image_utils.hpp"
#include "raspbot_perception/letterbox.hpp"
#include "raspbot_perception/nms.hpp"

namespace raspbot_perception {

namespace {

// 页对齐缓冲区：HailoRT 建议输入缓冲按 PAGE_SIZE 对齐以获最佳性能。
class PageAlignedBuffer {
 public:
  explicit PageAlignedBuffer(size_t size) : size_(size) {
    constexpr size_t kPage = 4096;
    const size_t rounded = ((size + kPage - 1) / kPage) * kPage;
    void* p = nullptr;
    if (posix_memalign(&p, kPage, rounded) != 0) {
      throw std::bad_alloc();
    }
    ptr_ = static_cast<uint8_t*>(p);
  }
  ~PageAlignedBuffer() { std::free(ptr_); }
  PageAlignedBuffer(const PageAlignedBuffer&) = delete;
  PageAlignedBuffer& operator=(const PageAlignedBuffer&) = delete;

  uint8_t* data() { return ptr_; }
  const uint8_t* data() const { return ptr_; }
  size_t size() const { return size_; }

 private:
  uint8_t* ptr_ = nullptr;
  size_t size_ = 0;
};

// sensor_msgs/Image -> BGR uint8（HWC）。支持 bgr8/rgb8/yuyv/yuv422；
// 其他编码或数据不完整返回 false。
bool decode_to_bgr(const sensor_msgs::msg::Image& msg, std::vector<uint8_t>& bgr,
                   int& w, int& h) {
  w = static_cast<int>(msg.width);
  h = static_cast<int>(msg.height);
  if (w <= 0 || h <= 0 || msg.is_bigendian) {
    return false;
  }
  bool flip = false;   // rgb8 -> bgr
  bool yuyv = false;
  if (msg.encoding == sensor_msgs::image_encodings::BGR8) {
    // 直通
  } else if (msg.encoding == sensor_msgs::image_encodings::RGB8) {
    flip = true;
  } else if (msg.encoding == "yuyv" || msg.encoding == "yuv422") {
    yuyv = true;
  } else {
    return false;  // 其他编码 WARN 丢弃（由调用方记录）
  }
  const size_t bytes_per_px = yuyv ? 2 : 3;
  const size_t step = msg.step ? msg.step : static_cast<size_t>(w) * bytes_per_px;
  if (msg.data.size() < step * static_cast<size_t>(h)) {
    return false;
  }
  bgr.assign(static_cast<size_t>(w) * h * 3, 0);
  if (yuyv) {
    if (step != static_cast<size_t>(w) * 2) {
      return false;  // 带行填充的 YUYV 暂不支持
    }
    yuyv_to_rgb(msg.data.data(), w, h, bgr.data());  // -> RGB
    rgb_to_bgr(bgr.data(), bgr.data(), static_cast<size_t>(w) * h);  // -> BGR
  } else if (step == static_cast<size_t>(w) * 3) {
    std::memcpy(bgr.data(), msg.data.data(), bgr.size());
    if (flip) {
      rgb_to_bgr(bgr.data(), bgr.data(), static_cast<size_t>(w) * h);
    }
  } else {
    for (int row = 0; row < h; ++row) {
      std::memcpy(bgr.data() + static_cast<size_t>(row) * w * 3,
                  msg.data.data() + row * step, static_cast<size_t>(w) * 3);
      if (flip) {
        rgb_to_bgr(bgr.data() + static_cast<size_t>(row) * w * 3,
                   bgr.data() + static_cast<size_t>(row) * w * 3, w);
      }
    }
  }
  return true;
}

}  // namespace

class HailoPerceptionNode : public rclcpp::Node {
 public:
  HailoPerceptionNode() : Node("hailo_node") {
    hef_path_ = declare_parameter<std::string>("hef_path",
                                               "/opt/hailo-models/yolov8s_h8.hef");
    score_threshold_ = static_cast<float>(
        declare_parameter<double>("score_threshold", 0.4));
    max_rate_ = declare_parameter<double>("max_rate", 10.0);
    publish_annotated_ = declare_parameter<bool>("publish_annotated", true);
    device_timeout_ = declare_parameter<int>("device_timeout", 0);
    fail_fast_ = declare_parameter<bool>("fail_fast", true);

    const auto cb_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group;
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "image_raw", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          on_image(msg);
        },
        sub_opts);
    det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
        "detections", rclcpp::SensorDataQoS());
    img_pub_ = create_publisher<sensor_msgs::msg::Image>(
        "perception/image", rclcpp::SensorDataQoS());
    diag_timer_ = create_wall_timer(
        std::chrono::seconds(1), [this]() { on_diag(); }, cb_group);

    RCLCPP_INFO(get_logger(),
                "参数：hef_path=%s score_threshold=%.2f max_rate=%.1f "
                "publish_annotated=%d device_timeout=%d fail_fast=%d",
                hef_path_.c_str(), score_threshold_, max_rate_,
                publish_annotated_ ? 1 : 0, device_timeout_, fail_fast_ ? 1 : 0);
    try_load_backend();
  }

  ~HailoPerceptionNode() override { backend_.reset(); }

 private:
  void try_load_backend() {
    try {
      auto backend = std::make_unique<HailoBackend>(hef_path_, device_timeout_);
      backend_ = std::move(backend);
      if (retry_timer_) {
        retry_timer_->cancel();
        retry_timer_.reset();
      }
      RCLCPP_INFO(get_logger(),
                  "Hailo 检测器就绪：%s（输入 %dx%d UINT8，输出 NMS BY CLASS %d 类）",
                  hef_path_.c_str(), backend_->dst(), backend_->dst(),
                  backend_->class_count());
    } catch (const std::exception& exc) {
      RCLCPP_ERROR(get_logger(), "HEF 加载/设备初始化失败: %s", exc.what());
      if (fail_fast_) {
        throw std::runtime_error(std::string("Hailo 设备初始化失败（fail_fast=true）：") +
                                 exc.what());
      }
      if (!retry_timer_) {
        RCLCPP_WARN(get_logger(),
                    "fail_fast=false：每 5 s 重试加载，期间丢弃所有图像帧");
        retry_timer_ = create_wall_timer(
            std::chrono::seconds(5), [this]() { try_load_backend(); });
      }
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    if (!backend_) {
      warn_rate_limited("检测器未就绪，丢弃图像帧", 5.0);
      return;
    }
    // max_rate 限频（<=0 不限频）
    const auto now = std::chrono::steady_clock::now();
    if (max_rate_ > 0) {
      const double period = 1.0 / max_rate_;
      if (std::chrono::duration<double>(now - last_infer_time_).count() <
          period) {
        return;
      }
    }

    int w = 0, h = 0;
    if (!decode_to_bgr(*msg, bgr_buf_, w, h)) {
      warn_rate_limited(
          "图像帧不合法（编码 " + msg->encoding + " 不支持或数据不足），丢弃",
          10.0);
      return;
    }

    try {
      const auto t0 = std::chrono::steady_clock::now();
      const std::vector<Detection> detections = detect(bgr_buf_.data(), w, h);
      const double elapsed_ms =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - t0)
              .count();
      last_infer_time_ = std::chrono::steady_clock::now();
      infer_count_ += 1;
      fps_ema_ = fps_ema_ == 0.0 ? 1000.0 / elapsed_ms
                                 : 0.2 * (1000.0 / elapsed_ms) + 0.8 * fps_ema_;
      ms_ema_ = ms_ema_ == 0.0 ? elapsed_ms : 0.2 * elapsed_ms + 0.8 * ms_ema_;

      publish_detections(msg, detections, w, h);
      if (publish_annotated_) {
        annotated_buf_ = bgr_buf_;
        for (const auto& det : detections) {
          char label[128];
          std::snprintf(label, sizeof(label), "%s %.2f",
                        class_name(det.class_id).c_str(), det.score);
          draw_detection(annotated_buf_.data(), w, h,
                         static_cast<int>(std::lround(det.x1)),
                         static_cast<int>(std::lround(det.y1)),
                         static_cast<int>(std::lround(det.x2)),
                         static_cast<int>(std::lround(det.y2)), label);
        }
        publish_annotated_image(msg, annotated_buf_, w, h);
      }
    } catch (const std::exception& exc) {
      error_count_ += 1;
      warn_rate_limited("推理异常（累计 " + std::to_string(error_count_) +
                            " 次）: " + exc.what(),
                        10.0);
    }
  }

  std::vector<Detection> detect(const uint8_t* bgr, int w, int h) {
    // BGR -> RGB -> letterbox（与 Python 版流程一致；非 1:1 缩放用 C++ 双线性）
    const size_t pixels = static_cast<size_t>(w) * h;
    rgb_buf_.resize(pixels * 3);
    bgr_to_rgb(bgr, rgb_buf_.data(), pixels);
    const LetterboxMeta meta = letterbox_meta(w, h, backend_->dst());
    letterbox_image(rgb_buf_.data(), w, h, meta, letterboxed_buf_,
                    backend_->dst(), kPadValue);
    const auto& raw = backend_->infer(letterboxed_buf_.data());
    std::vector<Detection> dets = parse_nms_raw(
        raw.data(), raw.size(), backend_->class_count(), score_threshold_,
        backend_->dst());
    // 映回原图坐标（clamp）
    for (auto& det : dets) {
      const Box2D box =
          map_box(det.x1, det.y1, det.x2, det.y2, meta, w, h);
      det.x1 = static_cast<float>(box.x1);
      det.y1 = static_cast<float>(box.y1);
      det.x2 = static_cast<float>(box.x2);
      det.y2 = static_cast<float>(box.y2);
    }
    return dets;
  }

  void publish_detections(const sensor_msgs::msg::Image::ConstSharedPtr msg,
                          const std::vector<Detection>& detections, int w,
                          int h) {
    vision_msgs::msg::Detection2DArray arr;
    arr.header.stamp = msg->header.stamp;
    arr.header.frame_id = "camera_link";  // 契约 CONTRACTS.md §2
    const double width = w, height = h;
    for (const auto& det : detections) {
      vision_msgs::msg::Detection2D d2d;
      d2d.header.stamp = msg->header.stamp;
      d2d.header.frame_id = "camera_link";
      // Lyrical 版 vision_msgs：hypothesis.class_id/score（非旧版顶层 id/score）
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(det.class_id);
      hyp.hypothesis.score = det.score;
      d2d.results.push_back(hyp);
      // (x1,y1,x2,y2) -> 归一化 (cx, cy) + (w, h)（本车契约：归一化相对坐标）
      vision_msgs::msg::BoundingBox2D bbox;
      bbox.center.position.x = (det.x1 + det.x2) / 2.0 / width;
      bbox.center.position.y = (det.y1 + det.y2) / 2.0 / height;
      bbox.center.theta = 0.0;
      bbox.size_x = (det.x2 - det.x1) / width;
      bbox.size_y = (det.y2 - det.y1) / height;
      d2d.bbox = bbox;
      arr.detections.push_back(d2d);
    }
    det_pub_->publish(arr);
  }

  void publish_annotated_image(const sensor_msgs::msg::Image::ConstSharedPtr msg,
                               const std::vector<uint8_t>& bgr, int w, int h) {
    sensor_msgs::msg::Image out;
    out.header = msg->header;
    out.height = static_cast<uint32_t>(h);
    out.width = static_cast<uint32_t>(w);
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.is_bigendian = 0;
    out.step = static_cast<uint32_t>(w) * 3;
    out.data = bgr;
    img_pub_->publish(out);
  }

  void on_diag() {
    if (infer_count_ > 0) {
      RCLCPP_INFO(get_logger(),
                  "[诊断] 推理 %.1f FPS | 平均 %.1f ms/帧 | 累计 %llu 帧 | "
                  "异常 %llu 次 | 检测器 %s",
                  fps_ema_, ms_ema_,
                  static_cast<unsigned long long>(infer_count_),
                  static_cast<unsigned long long>(error_count_),
                  backend_ ? "就绪" : "未就绪");
    } else {
      RCLCPP_INFO(get_logger(), "[诊断] 尚无推理帧 | 检测器 %s",
                  backend_ ? "就绪" : "未就绪");
    }
  }

  void warn_rate_limited(const std::string& text, double min_interval_s) {
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - last_warn_time_).count() >=
        min_interval_s) {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
      last_warn_time_ = now;
    }
  }

  // 参数
  std::string hef_path_;
  float score_threshold_ = 0.4f;
  double max_rate_ = 10.0;
  bool publish_annotated_ = true;
  int device_timeout_ = 0;
  bool fail_fast_ = true;

  // 通信
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;

  // 推理
  std::unique_ptr<HailoBackend> backend_;
  std::vector<uint8_t> bgr_buf_;
  std::vector<uint8_t> rgb_buf_;
  std::vector<uint8_t> letterboxed_buf_;
  std::vector<uint8_t> annotated_buf_;

  // 统计/限频
  uint64_t infer_count_ = 0;
  uint64_t error_count_ = 0;
  double fps_ema_ = 0.0;
  double ms_ema_ = 0.0;
  std::chrono::steady_clock::time_point last_infer_time_{};
  std::chrono::steady_clock::time_point last_warn_time_{};
};

}  // namespace raspbot_perception

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<raspbot_perception::HailoPerceptionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();  // SIGINT/ExternalShutdown 走这里 → 退出码 0
  } catch (const std::exception& exc) {
    RCLCPP_ERROR(rclcpp::get_logger("hailo_node"), "致命错误: %s", exc.what());
    rclcpp::shutdown();
    exit_code = 1;
  }
  return exit_code;
}
