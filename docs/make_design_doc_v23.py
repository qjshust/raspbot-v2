# -*- coding: utf-8 -*-
"""Create v2.3 of the design doc: power budget + storage route updates.

Edits (all targeted, formatting preserved):
  1. Cover metadata table (T0): version, date, highlights
  2. Revision table (T1): add v2.3 row
  3. P6 summary: one added sentence for v2.3
  4. §5.4: power baseline bullet, SSD current-budget bullet (replaces P108),
     two new bullets (5 A handshake failure, low-battery / mount options)
  5. T29 storage row: SL500 as data disk
  6. BOM T41 r7: powered hub + capacitor + ferrite
  7. Expansion table T42: split USB SSD / M.2 HAT+ rows
"""
from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.table import _Row
from docx.text.paragraph import Paragraph

SRC = Path(r"E:\code\raspberrypi\docs\RASPBOT-V2_智能驾驶系统设计文档_v2.2.docx")
DST = Path(r"E:\code\raspberrypi\docs\RASPBOT-V2_智能驾驶系统设计文档_v2.3.docx")

BULLET_POWER = (
    "供电基线：扩展板 Type-C 为 5.1 V/5 A 输出（厂商手册明确“支持树莓派 5 供电协议，"
    "仅供电不通讯”），Pi 5 据此把 USB 外设预算放宽到 1.6 A（所有 USB 口与风扇共享）；"
)
BULLET_BUDGET = (
    "传感器 + SSD 合计约 1.2 A（雷达 1.15 W、相机 0.5 W、SL500 峰值 0.9 A），"
    "占用上述 1.6 A 预算，余量约 0.4 A，不得再接键盘、语音模块或第二块盘；"
)
BULLET_HANDSHAKE = (
    "SSD 接 Pi 5 蓝色 USB 3.0 口；若 dmesg 出现“This power supply is not capable of "
    "supplying 5A”，说明 5 A 握手失败（多因线缆无 e-marker），预算回落至 600 mA，"
    "须换 e-marker 线或改用自供电 USB 3.0 Hub；"
)
BULLET_LOWBATT = (
    "SSD 数据分区按 nofail,noatime 挂载；电池低于 6.6 V（约 3.3 V/节）时禁止建图与录包，"
    "避免 BMS 过放保护切断造成文件系统损坏；"
)


def set_text_keep_format(paragraph, text):
    """Replace paragraph text while keeping the first run's formatting."""
    if not paragraph.runs:
        paragraph.add_run(text)
        return
    paragraph.runs[0].text = text
    for r in paragraph.runs[1:]:
        r.text = ""


def set_cell(cell, text):
    set_text_keep_format(cell.paragraphs[0], text)
    for p in cell.paragraphs[1:]:
        for r in p.runs:
            r.text = ""


def insert_paragraph_after(paragraph, text):
    new_p = deepcopy(paragraph._p)
    paragraph._p.addnext(new_p)
    np = Paragraph(new_p, paragraph._parent)
    set_text_keep_format(np, text)
    return np


def insert_row_after(row, values):
    new_tr = deepcopy(row._tr)
    row._tr.addnext(new_tr)
    new_row = _Row(new_tr, row._parent)
    for cell, val in zip(new_row.cells, values):
        set_cell(cell, val)
    return new_row


doc = Document(str(SRC))

# ---- 1. cover metadata -------------------------------------------------
t0 = doc.tables[0]
set_cell(t0.cell(1, 1), "v2.3（替代 v2.2）")
set_cell(t0.cell(2, 1), "2026-11-14")
set_cell(
    t0.cell(4, 1),
    "供电与存储复核：补充 SSD 电流预算与 5 A 握手判据（§5.4），"
    "存储路线区分 USB SSD 与 M.2 HAT+（§5.3、BOM、扩容表）",
)

# ---- 2. revision table -------------------------------------------------
t1 = doc.tables[1]
insert_row_after(
    t1.rows[-1],
    [
        "v2.3",
        "2026-11-14",
        "供电与存储复核：§5.4 补充 SSD 电流预算、5 A 握手判据与低电保护；"
        "BOM 第 7 项改为 USB 3.0 自供电 Hub + 抗干扰件；扩容表拆分 USB/PCIe 存储路线",
        "项目组",
    ],
)

# ---- 3. revision summary paragraph ------------------------------------
p6 = next(p for p in doc.paragraphs if p.text.startswith("v2.0 与 v1.0 不兼容之处"))
set_text_keep_format(
    p6,
    p6.text + " v2.3 同样不改变话题/TF 契约，只补充供电预算与存储路线说明。",
)

# ---- 4. §5.4 -----------------------------------------------------------
p107 = next(p for p in doc.paragraphs if p.text.startswith("两个 USB 转串口设备"))
p108 = next(p for p in doc.paragraphs if p.text.startswith("传感器总电流"))

set_text_keep_format(p108, BULLET_BUDGET)
p_baseline = insert_paragraph_after(p107, BULLET_POWER)
p_handshake = insert_paragraph_after(p108, BULLET_HANDSHAKE)
p_lowbatt = insert_paragraph_after(p_handshake, BULLET_LOWBATT)

# ---- 5. §5.3 storage row ----------------------------------------------
t29 = doc.tables[29]
set_cell(t29.cell(3, 1), "microSD 58 GB + Lexar SL500 1TB（USB）")
set_cell(
    t29.cell(3, 2),
    "bag/地图/日志写 SL500，系统保留 SD；占用 Pi 5 1.6 A USB 预算（需 5 A 握手成立）",
)

# ---- 6. BOM ------------------------------------------------------------
t41 = doc.tables[41]
set_cell(t41.cell(7, 1), "分电与抗干扰")
set_cell(t41.cell(7, 2), "USB 3.0 自供电 Hub + 1000 µF 电容 + 磁环")
set_cell(t41.cell(7, 6), "SSD 独立供电/抗电机干扰（备用）")

# ---- 7. expansion table ------------------------------------------------
t42 = doc.tables[42]
set_cell(t42.cell(1, 0), "存储（USB）")
set_cell(t42.cell(1, 1), "Lexar SL500 1TB（已有）")
set_cell(t42.cell(1, 2), "0 元")
set_cell(t42.cell(1, 3), "约 400 MB/s，即插即用")
set_cell(t42.cell(1, 4), "建图/录包；受 1.6 A USB 预算约束")
insert_row_after(
    t42.rows[1],
    [
        "存储（PCIe）",
        "M.2 HAT+ + 256GB",
        "约 330 元",
        "约 800 MB/s，不占 USB 预算",
        "频繁建图/录包；受甲板空间约束",
    ],
)

doc.save(str(DST))
print("saved", DST)
print("paragraphs:", len(doc.paragraphs), "tables:", len(doc.tables))
