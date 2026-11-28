# -*- coding: utf-8 -*-
"""画一张“双面胶安装位置”俯视示意图（mm，原点 = 甲板前缘中点，+y 为车头）。"""
import os
from PIL import Image, ImageDraw, ImageFont

OUT = r'E:\code\raspberrypi\mech\render\tape_zones.png'
FONT = r'C:\Windows\Fonts\msyh.ttc'
if not os.path.exists(FONT):
    FONT = r'C:\Windows\Fonts\simhei.ttf'

S = 8.0
X0, X1, Y0, Y1 = -62, 62, -188, 30
W, H = int((X1 - X0) * S), int((Y1 - Y0) * S)

f_t = ImageFont.truetype(FONT, 28)
f_s = ImageFont.truetype(FONT, 19)
f_m = ImageFont.truetype(FONT, 15)

img = Image.new('RGB', (W, H), 'white')
d = ImageDraw.Draw(img, 'RGBA')


def xy(x, y):
    return (x - X0) * S, (Y1 - y) * S


def rect(x0, y0, x1, y1, fill=None, outline=(60, 60, 60), width=2):
    d.rectangle([xy(min(x0, x1), max(y0, y1)), xy(max(x0, x1), min(y0, y1))],
                fill=fill, outline=outline, width=width)


def label(x, y, text, font=f_s, color=(30, 30, 30), anchor='mm'):
    d.text(xy(x, y), text, font=font, fill=color, anchor=anchor)


def dashed(x0, y0, x1, y1, color=(150, 150, 150)):
    import math
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 8))
    for i in range(0, n, 2):
        d.line([xy(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n),
                xy(x0 + (x1 - x0) * (i + 0.8) / n, y0 + (y1 - y0) * (i + 0.8) / n)],
               fill=color, width=1)


# 甲板
rect(-45, -170, 45, 0, fill=(250, 250, 248), outline=(40, 40, 40), width=2)
label(0, 8, '甲板前缘 = 车头', f_m, (110, 110, 110))

# 树莓派 + 驱动板
rect(-28, -139, 28, -54, fill=(216, 238, 218), outline=(90, 150, 100))
label(0, -100, '派 5 + 驱动板', f_s, (40, 90, 50))
label(0, -112, '本体高 10–22 mm', f_m, (95, 125, 100))

# 云台立柱
rect(-10.5, -29, 10.5, -16, fill=(242, 205, 205), outline=(190, 90, 90))
label(0, -22.5, '云台立柱', f_m, (150, 50, 50))

# 推荐区域
rect(-28, -54, 28, -31, fill=(92, 200, 122, 120), outline=(40, 150, 80), width=2)
label(0, -42.5, 'A  首选', f_s, (18, 100, 45))

rect(28, -139, 44, -56, fill=(150, 210, 170, 95), outline=(90, 170, 120), width=2)
rect(-44, -139, -28, -56, fill=(150, 210, 170, 95), outline=(90, 170, 120), width=2)
label(36, -97, 'B', f_s, (30, 110, 60))
label(-36, -97, 'B', f_s, (30, 110, 60))

rect(-28, -168, 28, -141, fill=(250, 226, 162, 120), outline=(200, 160, 60), width=2)
label(0, -154, 'C  可用（线最长）', f_m, (140, 105, 20))

# 轴距中点
cx, cy = xy(0, -77)
d.line([(cx - 16, cy), (cx + 16, cy)], fill=(200, 40, 40), width=2)
d.line([(cx, cy - 16), (cx, cy + 16)], fill=(200, 40, 40), width=2)
d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], outline=(200, 40, 40), width=2)
label(4, -68, '轴距中点（几何中心）', f_m, (170, 40, 40), anchor='lm')

# 轴距
dashed(-58, -27, 58, -27)
dashed(-58, -127, 58, -127)
label(-52, -22, '前轴', f_m, (140, 140, 140), anchor='lm')
label(-52, -122, '后轴', f_m, (140, 140, 140), anchor='lm')

# 图例
label(-60, 26, '优先级：A（居中·平整·线短）> B（两侧·离电机线远）> C（后部·线最长）',
      f_m, (20, 20, 20), anchor='lm')
label(-60, -182, '避免：云台立柱 / 派板发热面 / 电机动力线 30mm 内 / 外壳散热槽等悬空处',
      f_m, (150, 50, 50), anchor='lm')
label(-60, -176, '单位 mm，网格 10mm', f_m, (130, 130, 130), anchor='lm')

img.save(OUT)
print('saved', OUT, img.size)
