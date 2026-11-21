"""COCO_CLASSES 测试：长度 80、id 与名字抽样核对（COCO 官方顺序）。"""

from raspbot_perception.classes import COCO_CLASSES, COCO_CLASS_COUNT, class_name


def test_length_is_80():
    assert COCO_CLASS_COUNT == 80
    assert len(COCO_CLASSES) == 80


def test_no_duplicates_and_order():
    assert len(set(COCO_CLASSES)) == 80


def test_sample_ids():
    """与 COCO 官方 80 类顺序的抽样核对。"""
    samples = {
        0: 'person',
        1: 'bicycle',
        2: 'car',
        3: 'motorcycle',
        5: 'bus',
        7: 'truck',
        14: 'bird',
        15: 'cat',
        16: 'dog',
        32: 'sports ball',
        39: 'bottle',
        41: 'cup',
        46: 'banana',
        47: 'apple',
        56: 'chair',
        57: 'couch',
        58: 'potted plant',
        62: 'tv',
        63: 'laptop',
        67: 'cell phone',
        76: 'scissors',
        77: 'teddy bear',
        78: 'hair drier',
        79: 'toothbrush',
    }
    for class_id, name in samples.items():
        assert COCO_CLASSES[class_id] == name
        assert class_name(class_id) == name


def test_class_name_unknown():
    assert class_name(-1) == 'class_-1'
    assert class_name(80) == 'class_80'
