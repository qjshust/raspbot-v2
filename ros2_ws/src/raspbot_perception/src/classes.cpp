#include "raspbot_perception/classes.hpp"

namespace raspbot_perception {

// 与 COCO 官方 80 类顺序一致（同 Ultralytics/COCO 2017），id 0..79。
const std::array<std::string, kCocoClassCount> kCocoClasses = {
    "person",        "bicycle",      "car",           "motorcycle",
    "airplane",      "bus",          "train",         "truck",
    "boat",          "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench",        "bird",          "cat",
    "dog",           "horse",        "sheep",         "cow",
    "elephant",      "bear",         "zebra",         "giraffe",
    "backpack",      "umbrella",     "handbag",       "tie",
    "suitcase",      "frisbee",      "skis",          "snowboard",
    "sports ball",   "kite",         "baseball bat",  "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",
    "wine glass",    "cup",          "fork",          "knife",
    "spoon",         "bowl",         "banana",        "apple",
    "sandwich",      "orange",       "broccoli",      "carrot",
    "hot dog",       "pizza",        "donut",         "cake",
    "chair",         "couch",        "potted plant",  "bed",
    "dining table",  "toilet",       "tv",            "laptop",
    "mouse",         "remote",       "keyboard",      "cell phone",
    "microwave",     "oven",         "toaster",       "sink",
    "refrigerator",  "book",         "clock",         "vase",
    "scissors",      "teddy bear",   "hair drier",    "toothbrush",
};

std::string class_name(int class_id) {
  if (class_id >= 0 && class_id < kCocoClassCount) {
    return kCocoClasses[class_id];
  }
  return "class_" + std::to_string(class_id);
}

}  // namespace raspbot_perception
