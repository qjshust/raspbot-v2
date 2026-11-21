"""COCO 80 类名表（与 COCO 官方顺序一致，id 0..79）。

顺序与 Ultralytics / COCO 2017 官方 80 类完全一致，直接以 id 索引。
"""

COCO_CLASSES = (
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
    "teddy bear", "hair drier", "toothbrush",
)

COCO_CLASS_COUNT = len(COCO_CLASSES)


def class_name(class_id):
    """返回类名；未知 id 返回 'class_<id>'。"""
    if 0 <= class_id < COCO_CLASS_COUNT:
        return COCO_CLASSES[class_id]
    return f"class_{class_id}"
