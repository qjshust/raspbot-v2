# -*- coding: utf-8 -*-
"""示意：HWT101 装在车上时，哪些方向偏差能补、哪些不能。"""
import math
import os
from PIL import Image, ImageDraw, ImageFont

OUT = r'E:\code\raspberrypi\mech\render\imu_orientation.png'
FONT = r'C:\Windows\Fonts\msyh.ttc'
if not os.path.exists(FONT):
    FONT = r'C:\Windows\Fonts\simhei.ttf'

W, H = 900, 820
S = 2.5
CX, CY = W / 2, 470

f_t = ImageFont.truetype(FONT, 30)
f_s = ImageFont.truetype(FONT, 18)
f_m = ImageFont.truetype(FONT, 15)

img = Image.new('RGB', (W, H), 'white')
d = ImageDraw.Draw(img, 'RGBA')


def P(x, y):
    """车体坐标（+x 右、+y 前）-> 屏幕像素（车头朝上）。"""
    return CX + x * S, CY - y * S


def arrow(a, b, color, width=3, head=13):
    d.line([a, b], fill=color, width=width)
    ang = math.atan2(b[1] - a[1], b[0] - a[0])
    for k in (+1, -1):
        d.line([b, (b[0] - head * math.cos(ang + k * 0.42),
                    b[1] - head * math.sin(ang + k * 0.42))], fill=color, width=width)


def text(x, y, s, font=f_s, color=(30, 30, 30)):
    d.text((x, y), s, font=font, fill=color)


# ---- 车体 ----
d.rounded_rectangle([P(-45, 92), P(45, -98)], radius=20,
                    fill=(250, 250, 248), outline=(70, 70, 70), width=3)

# 车头参考轴
axc = P(0, 0)
arrow(axc, P(0, 118), (40, 40, 40), 3)
text(CX + 12, P(0, 112)[1] - 6, '车头', f_m, (40, 40, 40))

# ---- 模块：绕 Z 转 delta ----
delta = math.radians(22)
mx, my = 0, -10
w2, l2 = 23, 18
corners = []
for dx, dy in ((-w2, -l2), (w2, -l2), (w2, l2), (-w2, l2)):
    rx = dx * math.cos(delta) - dy * math.sin(delta)
    ry = dx * math.sin(delta) + dy * math.cos(delta)
    corners.append(P(mx + rx, my + ry))
d.polygon(corners, fill=(214, 232, 250), outline=(40, 90, 160))
d.line([corners[0], corners[1]], fill=(40, 90, 160), width=7)
d.text(P(mx, my), 'Z↑', font=f_m, fill=(40, 90, 160), anchor='mm')

# 模块坐标轴：δ=0 时 +X 朝车头、+Y 朝左
ux, uy = -math.sin(delta), math.cos(delta)       # +X
vx, vy = -math.cos(delta), -math.sin(delta)      # +Y
arrow(P(mx, my), P(mx + ux * 42, my + uy * 42), (205, 45, 45), 3)
arrow(P(mx, my), P(mx + vx * 38, my + vy * 38), (35, 135, 75), 3)
d.text(P(mx + ux * 50, my + uy * 50), '+X', font=f_s, fill=(205, 45, 45), anchor='mm')
d.text(P(mx + vx * 44, my + vy * 44), '+Y', font=f_s, fill=(35, 135, 75), anchor='lm')
d.text(P(mx - vx * 34, my - vy * 34), '排针边', font=f_m, fill=(40, 90, 160), anchor='mm')

# δ 角弧（从车头方向转到 +X）
r = 62
d.arc([P(mx - r, my + r), P(mx + r, my - r)], start=-112, end=-90,
      fill=(190, 130, 20), width=3)
d.text(P(mx - 20, my + 30), 'δ', font=f_s, fill=(190, 130, 20), anchor='rm')

# ---- 文字 ----
text(36, 28, 'IMU 安装方向：只有「Z 朝上」和「水平」是硬约束', f_t, (20, 40, 70))
text(36, 78, '① Z 轴必须朝上（PCB 平放）。装反 → 航向符号反，改任何参数都补不回来。', f_s, (175, 40, 40))
text(36, 108, '② 绕 Z 轴转多少都行：δ 只是零点偏移，量出来填 yaw_offset_deg 即可。', f_s, (30, 115, 65))
text(36, 138, '③ 倾斜（绕 X/Y 转）不能用参数补：转得越快误差越大，用垫片调平到 ≤0.5°。', f_s, (30, 30, 30))

text(36, 764, 'δ = 车头方向与模块 +X 的夹角，逆时针为正', f_m, (120, 120, 120))

img.save(OUT)
print('saved', OUT, img.size)


