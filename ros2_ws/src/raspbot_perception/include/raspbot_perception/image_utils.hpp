// 图像工具：YUYV(4:2:2) -> RGB 转换与检测框/标签绘制（纯 C++，无 OpenCV 依赖）。
//
// YUYV -> RGB 采用 BT.601 limited range 整数定点公式（与 OpenCV 结果一致）：
//   R = (298*(Y-16) + 409*(V-128) + 128) >> 8
//   G = (298*(Y-16) - 100*(U-128) - 208*(V-128) + 128) >> 8
//   B = (298*(Y-16) + 516*(U-128) + 128) >> 8
// 每像素对布局：YUYV = [Y0 U Y1 V]。
// 文字用内置 5x7 点阵字体（字符集 0-9 a-z A-Z . - 空格），无第三方字体依赖。

#pragma once

#include <cstdint>
#include <string>

namespace raspbot_perception {

// YUYV 4:2:2 -> RGB24（HWC）。要求宽为偶数，否则抛 std::invalid_argument。
// yuyv 长度须 >= w*h*2。输出 rgb_out 长度须 >= w*h*3。
void yuyv_to_rgb(const uint8_t* yuyv, int w, int h, uint8_t* rgb_out);

// 在 BGR 图像（HWC，w x h x 3）上画框（绿色，线宽 2，坐标自动 clamp）并写标签
// "label"（5x7 点阵，绿色）。纯 C++ 实现（无 cv2.rectangle/putText 依赖）。
void draw_detection(uint8_t* bgr, int w, int h, int x1, int y1, int x2, int y2,
                    const std::string& label);

}  // namespace raspbot_perception
