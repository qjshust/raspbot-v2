# -*- coding: utf-8 -*-
"""Create v2.4 (rebuild): storage layout, SD-write tuning and USB findings.

Tables are located by header text, never by index, so earlier insertions cannot
shift the target of a later edit.
"""
from copy import deepcopy
from pathlib import Path

from docx import Document
from docx.shared import Pt
from docx.table import _Row

SRC = Path(r"E:\code\raspberrypi\docs\RASPBOT-V2_智能驾驶系统设计文档_v2.3.docx")
DST = Path(r"E:\code\raspberrypi\docs\RASPBOT-V2_智能驾驶系统设计文档_v2.4.docx")

doc = Document(str(SRC))


def set_text_keep_format(paragraph, text):
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


def find_table(pred):
    for t in doc.tables:
        hdr = [c.text.strip() for c in t.rows[0].cells]
        if pred(hdr):
            return t
    raise LookupError("table not found")


# ---- capture every target BEFORE any insertion -------------------------
cover = doc.tables[0]
revision = doc.tables[1]
storage = find_table(lambda h: h == ["项目", "现状", "建议"])
expansion = find_table(lambda h: h == ["项目", "型号", "价格", "收益", "触发条件"])
bom = find_table(lambda h: h[:2] == ["序号", "名称"])
risks = find_table(lambda h: h == ["风险", "影响", "对策"])
risk_snapshot = [tuple(c.text for c in r.cells) for r in risks.rows]

ref_run = bom.rows[0].cells[0].paragraphs[0].runs
REF_SIZE = ref_run[0].font.size if ref_run and ref_run[0].font.size else Pt(9)
TABLE_STYLE = bom.style.name if bom.style is not None else "Table Grid"

# ---- cover -------------------------------------------------------------
set_cell(cover.cell(1, 1), "v2.4（替代 v2.3）")
set_cell(cover.cell(2, 1), "2026-11-14")
set_cell(
    cover.cell(4, 1),
    "存储落地：SD 系统盘 + SL500 1 TB 数据盘（/mnt/ssd）；SD 写入优化"
    "（journald 64 MB、zram、noatime、ROS 日志外置）；USB 端口实测与故障结论",
)

# ---- revision table ----------------------------------------------------
new_tr = deepcopy(revision.rows[-1]._tr)
revision.rows[-1]._tr.addnext(new_tr)
nr = _Row(new_tr, revision)
for cell, val in zip(
    nr.cells,
    [
        "v2.4",
        "2026-11-14",
        "存储与 USB 实测落地：新增 §5.4.1–5.4.3（存储布局、SD 写入优化、USB 端口与稳定性）；"
        "SSD 由系统盘候选改为数据盘；同步 §5.3 与扩容表",
        "项目组",
    ],
):
    set_cell(cell, val)

# ---- revision summary --------------------------------------------------
p6 = next(p for p in doc.paragraphs if p.text.startswith("v2.0 与 v1.0 不兼容之处"))
set_text_keep_format(
    p6, p6.text + " v2.4 同样不改变话题/TF 契约，只补充存储落地与 USB 实测结论。"
)

# ---- §5.3 storage row --------------------------------------------------
for row in storage.rows[1:]:
    if row.cells[0].text.strip() == "存储":
        set_cell(row.cells[1], "microSD 58 GB（系统）+ SL500 1 TB（数据盘 /mnt/ssd）")
        set_cell(
            row.cells[2],
            "bag/地图/日志写 /mnt/ssd；journald 64 MB、zram 3.9 GB、根分区 noatime 已实施（§5.4.2）",
        )
        break

# ---- expansion table ---------------------------------------------------
for row in expansion.rows[1:]:
    if row.cells[0].text.strip() == "存储（USB）":
        set_cell(row.cells[1], "Lexar SL500 1 TB（已部署为数据盘）")
        set_cell(row.cells[2], "0 元")
        set_cell(row.cells[3], "实测持续写 343 MB/s；已挂 /mnt/ssd")
        set_cell(row.cells[4], "已实施（系统盘仍为 SD）")
        break

# ---- §5.4.1 / 5.4.2 / 5.4.3 -------------------------------------------
anchor = next(p for p in doc.paragraphs if p.text.startswith("所有新增设备共地"))
cursor = anchor._p


def add_par(text, style="Normal"):
    global cursor
    p = doc.add_paragraph(text, style=style)
    cursor.addnext(p._p)
    cursor = p._p
    for r in p.runs:
        if r.font.size is None:
            r.font.size = Pt(10)
    return p


def add_table(header, rows):
    global cursor
    t = doc.add_table(rows=1 + len(rows), cols=len(header))
    try:
        t.style = TABLE_STYLE
    except KeyError:
        t.style = "Table Grid"
    cursor.addnext(t._tbl)
    cursor = t._tbl
    for j, h in enumerate(header):
        cell = t.cell(0, j)
        set_cell(cell, h)
        for r in cell.paragraphs[0].runs:
            r.bold = True
            r.font.size = REF_SIZE
    for i, row in enumerate(rows, start=1):
        for j, val in enumerate(row):
            cell = t.cell(i, j)
            set_cell(cell, val)
            for r in cell.paragraphs[0].runs:
                r.font.size = REF_SIZE
    return t


def add_code(text):
    p = add_par(text)
    for r in p.runs:
        r.font.name = "Consolas"
        r.font.size = Pt(9)
    return p


add_par("5.4.1  存储布局：SD 系统盘 + SSD 数据盘", style="Heading 3")
add_par(
    "系统盘保持 microSD，Lexar SL500 1 TB 作为纯数据盘：单分区 ext4、卷标 data，"
    "挂载到 /mnt/ssd。不把系统放在 USB 盘上——该盘在原 USB 口曾使 xHCI 控制器挂死"
    "（见 5.4.3），系统盘掉线等于整机失联。",
)
add_table(
    ["设备", "分区 / 卷标", "挂载点", "用途", "说明"],
    [
        ["microSD 64 GB", "p2 / writable", "/", "系统、ROS 2 工作区", "已用 8.3 GB，保留原分区结构"],
        ["Lexar SL500 1 TB", "sda1 / data", "/mnt/ssd", "bag、地图、日志", "ext4，UUID 4873f5ac…，929 GB 可用"],
        ["microSD 64 GB", "p1 / system-boot", "/boot/firmware", "启动文件", "512 MB vfat"],
    ],
)
add_par("fstab 条目（nofail 保证盘缺失时不影响启动）：")
add_code("UUID=4873f5ac-5a48-4aba-9f0e-19492be5c837 /mnt/ssd ext4 nofail,noatime 0 2")
add_par("数据目录约定：")
add_par("/mnt/ssd/bags —— ros2 bag 录制输出；", style="List Bullet")
add_par("/mnt/ssd/maps —— slam_toolbox 地图保存；", style="List Bullet")
add_par("/mnt/ssd/roslog —— ROS 2 节点日志（ROS_LOG_DIR）。", style="List Bullet")
add_code("ros2 bag record -o /mnt/ssd/bags/$(date +%Y%m%d_%H%M) /scan /odom /tf /camera/image_raw/compressed")

add_par("5.4.2  降低 SD 写入的配置（已实施并验证）", style="Heading 3")
add_par(
    "空闲时 SD 写入实测 44 KiB/30 s（约 126 MB/天），磨损主要来自跑车时的 bag、日志与内存换页。"
    "据此把写放大的路径移出 SD：",
)
add_table(
    ["项目", "配置", "效果"],
    [
        ["systemd journal", "SystemMaxUse=64M", "占用由 131.7 MB 降至 60.1 MB 并封顶"],
        ["ROS 日志", "ROS_LOG_DIR=/mnt/ssd/roslog（.bashrc + /etc/profile.d）", "运行日志不再写 SD"],
        ["swap", "zram 3.9 GB（lz4，优先级 100）+ 4 GB swapfile 兜底", "换页压缩在内存，不落 SD"],
        ["swappiness", "vm.swappiness=10（/etc/sysctl.d/99-robot.conf）", "减少换出倾向"],
        ["根分区挂载", "defaults,noatime", "去掉访问时间写入"],
        ["数据目录", "/mnt/ssd/{bags,maps,roslog}", "大文件写入全部落 SSD"],
    ],
)
add_par(
    "注意：bag 是最大写入源（图像+雷达约 2–5 MB/s，1 小时 7–18 GB），必须显式写到 /mnt/ssd/bags；"
    "地图保存路径同理。改造后 SD 的写入主要剩启动与 apt/snap 更新。",
)
add_par("验证命令：")
add_code("systemctl is-active zramswap; journalctl --disk-usage; findmnt -no OPTIONS /; findmnt -no SOURCE /mnt/ssd")

add_par("5.4.3  USB 端口与 SSD 稳定性（实测）", style="Heading 3")
add_table(
    ["项目", "结果"],
    [
        [
            "端口 A（bus 5，与摄像头同属 xhci-hcd.1）",
            "克隆至 11.2 GB 时 sda 掉线（device offline）；随后开机 uas 未绑定、INQUIRY 无响应，"
            "xHCI 报 “host controller not responding, assume dead”，控制器连同摄像头一并消失，系统进入紧急模式",
        ],
        [
            "端口 B（bus 3，xhci-hcd.0）",
            "10 GiB 直写 379 MB/s；100 GiB 连续写 343 MB/s（317 s）；零错误、零断开、卸载干净",
        ],
        [
            "供电",
            "EXT5V_V 5.097 V、throttled=0x0、无 “not capable of supplying 5A” 告警 → 排除供电不足",
        ],
        [
            "结论",
            "SSD 固定插 bus 3 对应端口，不与摄像头共用控制器；UAS 保持启用；"
            "若再现掉线，备选 usb-storage.quirks=21c4:0050:u（回落 BOT）",
        ],
    ],
)
add_par(
    "判定方法：lsusb -t 应见 Bus 003 下 Driver=uas、5000M；持续写入后检查 "
    "dmesg 无 disconnect / offline / assume dead。",
)

# ---- guard: the risk table must be untouched ---------------------------
now = [tuple(c.text for c in r.cells) for r in risks.rows]
assert now == risk_snapshot, "risk table was modified!"

doc.save(str(DST))
print("saved", DST)
print("paragraphs:", len(doc.paragraphs), "tables:", len(doc.tables))
