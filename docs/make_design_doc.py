# -*- coding: utf-8 -*-
"""Generate the RASPBOT-V2 smart-driving design document."""

import os

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

OUT = r'E:\code\raspberrypi\docs\RASPBOT-V2_智能驾驶系统设计文档_v1.0.docx'

BODY_FONT = '宋体'
HEAD_FONT = '微软雅黑'
MONO_FONT = 'Consolas'


# --------------------------------------------------------------------------- helpers

def set_east_asia(style, font_name):
    rpr = style.element.get_or_add_rPr()
    rfonts = rpr.find(qn('w:rFonts'))
    if rfonts is None:
        rfonts = OxmlElement('w:rFonts')
        rpr.append(rfonts)
    rfonts.set(qn('w:eastAsia'), font_name)
    rfonts.set(qn('w:ascii'), font_name)
    rfonts.set(qn('w:hAnsi'), font_name)


def run_east_asia(run, font_name):
    rpr = run._r.get_or_add_rPr()
    rfonts = rpr.find(qn('w:rFonts'))
    if rfonts is None:
        rfonts = OxmlElement('w:rFonts')
        rpr.append(rfonts)
    rfonts.set(qn('w:eastAsia'), font_name)


def setup_styles(doc):
    normal = doc.styles['Normal']
    normal.font.name = BODY_FONT
    normal.font.size = Pt(10.5)
    set_east_asia(normal, BODY_FONT)
    normal.paragraph_format.space_after = Pt(4)
    normal.paragraph_format.line_spacing = 1.25

    for name, size in (('Heading 1', 16), ('Heading 2', 13), ('Heading 3', 11.5)):
        style = doc.styles[name]
        style.font.name = HEAD_FONT
        style.font.size = Pt(size)
        style.font.bold = True
        style.font.color.rgb = RGBColor(0x1F, 0x3B, 0x57)
        set_east_asia(style, HEAD_FONT)
        style.paragraph_format.space_before = Pt(12)
        style.paragraph_format.space_after = Pt(6)

    for name in ('List Bullet', 'List Number'):
        style = doc.styles[name]
        style.font.name = BODY_FONT
        style.font.size = Pt(10.5)
        set_east_asia(style, BODY_FONT)


def add_page_number(paragraph):
    run = paragraph.add_run()
    fld_begin = OxmlElement('w:fldChar')
    fld_begin.set(qn('w:fldCharType'), 'begin')
    instr = OxmlElement('w:instrText')
    instr.set(qn('xml:space'), 'preserve')
    instr.text = 'PAGE'
    fld_end = OxmlElement('w:fldChar')
    fld_end.set(qn('w:fldCharType'), 'end')
    run._r.append(fld_begin)
    run._r.append(instr)
    run._r.append(fld_end)
    run.font.size = Pt(9)


def add_toc(doc):
    p = doc.add_paragraph()
    run = p.add_run()
    fld_begin = OxmlElement('w:fldChar')
    fld_begin.set(qn('w:fldCharType'), 'begin')
    instr = OxmlElement('w:instrText')
    instr.set(qn('xml:space'), 'preserve')
    instr.text = 'TOC \\o "1-3" \\h \\z \\u'
    fld_sep = OxmlElement('w:fldChar')
    fld_sep.set(qn('w:fldCharType'), 'separate')
    placeholder = OxmlElement('w:t')
    placeholder.text = '（在 Word 中右键此处 → 更新域，生成目录）'
    fld_end = OxmlElement('w:fldChar')
    fld_end.set(qn('w:fldCharType'), 'end')
    for node in (fld_begin, instr, fld_sep, placeholder, fld_end):
        run._r.append(node)


def add_table(doc, headers, rows, widths=None, font_size=9.0, header_fill='DCE6F1'):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = 'Table Grid'
    table.alignment = WD_TABLE_ALIGNMENT.CENTER

    header_cells = table.rows[0].cells
    for i, text in enumerate(headers):
        cell = header_cells[i]
        cell.text = ''
        p = cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        run = p.add_run(str(text))
        run.bold = True
        run.font.size = Pt(font_size)
        run_east_asia(run, HEAD_FONT)
        shd = OxmlElement('w:shd')
        shd.set(qn('w:fill'), header_fill)
        cell._tc.get_or_add_tcPr().append(shd)

    tr_pr = table.rows[0]._tr.get_or_add_trPr()
    tbl_header = OxmlElement('w:tblHeader')
    tbl_header.set(qn('w:val'), 'true')
    tr_pr.append(tbl_header)
    cant_split = OxmlElement('w:cantSplit')
    tr_pr.append(cant_split)

    for row in rows:
        cells = table.add_row().cells
        for i, value in enumerate(row):
            cells[i].text = ''
            p = cells[i].paragraphs[0]
            run = p.add_run(str(value))
            run.font.size = Pt(font_size)
            run_east_asia(run, BODY_FONT)
        row_tr_pr = table.rows[-1]._tr.get_or_add_trPr()
        row_cant_split = OxmlElement('w:cantSplit')
        row_tr_pr.append(row_cant_split)

    if widths:
        for row in table.rows:
            for i, width in enumerate(widths):
                row.cells[i].width = Cm(width)
    return table


def add_code(doc, text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.line_spacing = 1.0
    p_pr = p._p.get_or_add_pPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:fill'), 'F4F4F4')
    p_pr.append(shd)
    run = p.add_run(text)
    run.font.name = MONO_FONT
    run.font.size = Pt(8.5)
    run_east_asia(run, MONO_FONT)
    return p


def note(doc, text):
    p = doc.add_paragraph()
    run = p.add_run(text)
    run.italic = True
    run.font.size = Pt(9.5)
    run.font.color.rgb = RGBColor(0x59, 0x59, 0x59)
    run_east_asia(run, BODY_FONT)
    return p


def bullets(doc, items):
    for item in items:
        doc.add_paragraph(item, style='List Bullet')


# --------------------------------------------------------------------------- document

doc = Document()
setup_styles(doc)

section = doc.sections[0]
section.page_width = Cm(21.0)
section.page_height = Cm(29.7)
section.top_margin = Cm(2.2)
section.bottom_margin = Cm(2.0)
section.left_margin = Cm(2.4)
section.right_margin = Cm(2.0)

footer_p = section.footer.paragraphs[0]
footer_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
footer_run = footer_p.add_run('RASPBOT-V2 智能驾驶系统设计文档  |  第 ')
add_page_number(footer_p)
footer_p.add_run(' 页')

# ---- cover
title = doc.add_paragraph()
title.alignment = WD_ALIGN_PARAGRAPH.CENTER
title.paragraph_format.space_before = Pt(110)
run = title.add_run('RASPBOT-V2 智能驾驶小车')
run.font.name = HEAD_FONT
run.font.size = Pt(30)
run.font.bold = True
run.font.color.rgb = RGBColor(0x1F, 0x3B, 0x57)
run_east_asia(run, HEAD_FONT)

subtitle = doc.add_paragraph()
subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
run = subtitle.add_run('系统设计文档')
run.font.name = HEAD_FONT
run.font.size = Pt(22)
run.font.color.rgb = RGBColor(0x2E, 0x5A, 0x88)
run_east_asia(run, HEAD_FONT)

sub2 = doc.add_paragraph()
sub2.alignment = WD_ALIGN_PARAGRAPH.CENTER
sub2.paragraph_format.space_before = Pt(10)
run = sub2.add_run('硬件方案 · BOM · 软件架构 · 需求分析 · 开发排期')
run.font.size = Pt(12)
run.font.color.rgb = RGBColor(0x59, 0x59, 0x59)
run_east_asia(run, BODY_FONT)

doc.add_paragraph()
info = doc.add_table(rows=4, cols=2)
info.style = 'Table Grid'
info.alignment = WD_TABLE_ALIGNMENT.CENTER
info_rows = [
    ('文档编号', 'RB2-SDD-001'),
    ('版本', 'v1.0'),
    ('日期', '2026-11-10'),
    ('适用平台', '树莓派 5 (8GB) + RASPBOT-V2 + Ubuntu 26.04 + ROS 2 Lyrical'),
]
for i, (k, v) in enumerate(info_rows):
    for j, text in enumerate((k, v)):
        cell = info.rows[i].cells[j]
        cell.text = ''
        run = cell.paragraphs[0].add_run(text)
        run.font.size = Pt(10)
        run.bold = (j == 0)
        run_east_asia(run, BODY_FONT)

doc.add_page_break()

# ---- revision + toc
doc.add_heading('文档修订记录', level=1)
add_table(
    doc,
    ['版本', '日期', '修订内容', '作者'],
    [
        ('v0.1', '2026-11-10', '初稿：平台现状、硬件选型结论、软件架构草案', '项目组'),
        ('v1.0', '2026-11-10', '补充 BOM、需求清单、验收标准与六周排期', '项目组'),
    ],
    widths=[2.0, 2.6, 9.0, 3.0],
)

doc.add_heading('目录', level=1)
add_toc(doc)
doc.add_page_break()

# =========================================================== 1 概述
doc.add_heading('1  概述', level=1)

doc.add_heading('1.1  项目背景与目标', level=2)
doc.add_paragraph(
    '本项目以亚博智能 RASPBOT-V2 麦轮小车为移动平台，以树莓派 5（8GB）为上位机，'
    '基于 Ubuntu 26.04 LTS 与 ROS 2 Lyrical 构建一套具备室内外使用能力的智能驾驶系统。'
    '第一阶段目标是完成“可靠底盘控制 + 多传感器融合定位 + 建图导航”的闭环，'
    '并在后续演进中引入边缘 AI 加速（Hailo-8）承担视觉感知。'
)
doc.add_paragraph('阶段性目标按优先级排列：')
bullets(doc, [
    'P0：底盘可遥控、可发布可用里程计，跑通 ROS 2 控制链路；',
    'P0：室内完成建图与定点导航，具备基础避障与安全停车；',
    'P1：户外平整路面可建图、可循迹、可避障；',
    'P2：接入 Hailo-8 完成视觉检测/车道线识别，形成“安全层 + 语义层”的双层感知。',
])

doc.add_heading('1.2  平台现状', level=2)
doc.add_paragraph(
    '平台已完成组装与首次联调，操作系统、ROS 2 与底盘驱动均已就位。具体状态如下表。'
)
add_table(
    doc,
    ['项目', '现状', '说明'],
    [
        ('计算平台', '树莓派 5，8GB，Ubuntu 26.04.1 LTS (arm64)', '四核 Cortex-A76 @2.4GHz，SD 卡启动（58GB 已扩容）'),
        ('中间件', 'ROS 2 Lyrical Luth', '/opt/ros/lyrical，199 个 ros-lyrical 包，rosdep 已初始化'),
        ('底盘', 'RASPBOT-V2，四轮麦轮', '板载 MCU 经 I2C 0x2B 控制电机、舵机、灯效、超声、巡线'),
        ('里程反馈', '无编码器', '厂商库无转速/位置回读接口，这是本设计的关键约束'),
        ('相机', 'USB 相机 0c45:64ab（MJPG 640×480/1280×720）', '已实测出图；IMX219 若在 CSI 上需单独确认'),
        ('其他 I2C 设备', '0x3C（疑似 SSD1306 OLED）', '可用于状态显示，本期不展开'),
        ('网络', '有线 DHCP 固定为 192.168.50.98', '路由器已做 MAC-IP 绑定'),
    ],
    widths=[3.2, 5.6, 7.6],
)

doc.add_heading('1.3  已验证能力', level=2)
doc.add_paragraph('以下项目已在实车上逐项验证通过，可作为设计基线：')
add_table(
    doc,
    ['子系统', '验证方式', '结果'],
    [
        ('I2C 链路', 'i2cdetect -y 1 命中 0x2b；smbus 读写', '通过'),
        ('四路电机', '逐轮正反转、四轮同转', '全部动作正常'),
        ('云台舵机', '水平/俯仰扫动', '正常'),
        ('超声波', '0x1a/0x1b 组合读数', '153–170 mm 稳定输出'),
        ('四路巡线', '0x0a 读字节', '0x0f，四位均有读数'),
        ('蜂鸣器/RGB', '指令下发', '无 I2C 报错'),
        ('USB 相机', 'v4l2 MJPG 抓帧', '640×480 出图正常'),
        ('ROS 2 底盘节点', '/cmd_vel → 电机，/odom 积分', '0.1 m/s × 1.2 s → x=0.120 m'),
    ],
    widths=[3.4, 7.6, 5.4],
)

doc.add_heading('1.4  范围与不在范围内', level=2)
add_table(
    doc,
    ['范围', '内容'],
    [
        ('本期范围', '底盘驱动完善、IMU/激光雷达接入、里程计融合、室内 SLAM、Nav2 定点导航、基础避障与安全策略'),
        ('本期不做', '编码器改装、Hailo-8 视觉推理、GNSS 定位、机械结构改造、真实道路（机动车道）行驶'),
        ('后续演进', '编码器/轮速闭环、Hailo-8 感知、户外全局定位、多车协同'),
    ],
    widths=[3.0, 13.4],
)

doc.add_heading('1.5  术语与缩写', level=2)
add_table(
    doc,
    ['术语', '含义'],
    [
        ('SLAM', '同步定位与建图，本方案使用 slam_toolbox 的 2D 激光方案'),
        ('Nav2', 'ROS 2 Navigation Stack，负责任务规划、路径规划、局部控制与行为恢复'),
        ('EKF', '扩展卡尔曼滤波，本方案用 robot_localization 融合 IMU 与速度指令'),
        ('DTOF', '直接飞行时间测距，抗环境光能力强于三角测距'),
        ('TF', 'ROS 2 坐标变换树，描述各传感器与车体的相对位姿'),
        ('QoS', 'ROS 2 通信服务质量策略，影响话题的可靠性与实时性'),
        ('MOS', '最大操作速度，用于约束底盘指令范围'),
    ],
    widths=[3.0, 13.4],
)

# =========================================================== 2 需求
doc.add_page_break()
doc.add_heading('2  需求分析', level=1)

doc.add_heading('2.1  使用场景定义', level=2)
add_table(
    doc,
    ['场景', '环境特征', '速度', '关键能力'],
    [
        ('S1 室内走廊/教室', '结构化、光照稳定、地面平整、有玻璃与桌腿', '≤0.5 m/s', '建图、重定位、定点导航、低矮障碍检测'),
        ('S2 园区步道', '平整铺装、行人、日照变化、部分区域无特征', '≤1.0 m/s', '户外建图、避障、失位恢复'),
        ('S3 演示/比赛', '固定场地、需快速部署与稳定复现', '≤1.0 m/s', '一键启动、日志回放、快速恢复'),
        ('S4 教学调试', '桌面/架空、频繁改参', '—', '单模块可测、参数可热调、故障可诊断'),
    ],
    widths=[3.4, 6.0, 2.2, 7.0],
)

doc.add_heading('2.2  用户与用例', level=2)
add_table(
    doc,
    ['编号', '用例', '参与者', '主流程', '成功判据'],
    [
        ('UC-01', '遥控行驶', '操作员', '启动底盘节点 → 键盘/手柄发布 /cmd_vel → 车按指令运动', '方向正确、松手 0.5 s 内停车'),
        ('UC-02', '室内建图', '操作员', '启动 SLAM → 遥控走一圈 → 保存地图', '回到起点误差 <0.2 m，地图无明显重影'),
        ('UC-03', '定点导航', '操作员', '加载地图 → 设定目标点 → 车自主到达', '成功率 ≥90%，全程无碰撞'),
        ('UC-04', '避障', '系统', '前方出现障碍 → 代价地图膨胀 → 重规划或停车', '0.3 m 内停车，不接触障碍'),
        ('UC-05', '户外巡径', '操作员', '加载户外地图 → 沿路径行驶', '偏离路径 <0.3 m，可人工接管'),
        ('UC-06', '故障恢复', '系统', '指令超时/节点崩溃 → 停车并告警', '5 s 内停车，systemd 自动重启节点'),
    ],
    widths=[1.6, 2.6, 2.2, 6.4, 4.6],
)

doc.add_heading('2.3  功能需求', level=2)
add_table(
    doc,
    ['编号', '需求描述', '优先级', '验收方式'],
    [
        ('FR-01', '订阅 /cmd_vel（Twist），驱动四轮实现前进、后退、横移、自转', 'P0', '实车四方向验证'),
        ('FR-02', '发布 /odom 与 odom→base_link TF，航向优先采用 IMU', 'P0', '直行 2 m 误差 <5 cm'),
        ('FR-03', '指令超时 0.5 s 自动停车；节点退出时强制停车', 'P0', '断开发布端观察'),
        ('FR-04', '接入 IMU，发布 /imu/data（≥100 Hz，含姿态四元数与角速度）', 'P0', '静止零偏校准，转动比对'),
        ('FR-05', '接入 2D 雷达，发布 /scan（10 Hz，frame_id=laser）', 'P0', 'rviz2 点云与实物一致'),
        ('FR-06', '发布超声波 /range 与四路巡线 /line_sensor', 'P1', '遮挡/贴线测试'),
        ('FR-07', '支持云台舵机、蜂鸣器、RGB 的 ROS 2 接口', 'P1', '话题调用验证'),
        ('FR-08', '发布相机图像 /image_raw 与 camera_info', 'P1', '图像与标定参数正确'),
        ('FR-09', 'slam_toolbox 在线建图，支持地图保存与加载', 'P0', '20 m² 场景建图'),
        ('FR-10', '基于先验地图的定位（amcl 或 scan match）', 'P1', '初始位姿偏差 <0.2 m 可收敛'),
        ('FR-11', 'Nav2 定点导航（NavigateToPose），支持取消与重规划', 'P0', '5 个目标点成功率 ≥90%'),
        ('FR-12', '代价地图障碍层与膨胀层，障碍前减速/停车', 'P0', '障碍物 0.3 m 停车'),
        ('FR-13', '遥控接口（键盘/手柄），与自主模式互斥', 'P1', '模式切换无冲突'),
        ('FR-14', 'rosbag2 记录关键话题，支持回放复现', 'P1', '回放轨迹一致'),
        ('FR-15', '开机自启与进程守护（systemd），异常自动重启', 'P1', '断电重启后 60 s 内可导航'),
        ('FR-16', '运行诊断：节点心跳、I2C 错误计数、CPU/温度', 'P2', '/diagnostics 可见'),
        ('FR-17', '远程可视化（Foxglove 或 rviz2 跨机）', 'P2', '笔记本可查看 TF/点云/图像'),
        ('FR-18', '无 IMU 时自动回落到开环里程计并在日志告警', 'P1', '拔掉 IMU 仍可运行'),
    ],
    widths=[1.6, 8.6, 1.6, 5.6],
)

doc.add_heading('2.4  非功能需求', level=2)
add_table(
    doc,
    ['编号', '类别', '指标', '验证'],
    [
        ('NFR-01', '实时性', '底盘控制环 30 Hz；Nav2 控制 20 Hz，周期抖动 <10%', 'ros2 topic hz / 日志时间戳'),
        ('NFR-02', '端到端延迟', '传感器→指令输出 <100 ms（本地）', '打点测量'),
        ('NFR-03', '资源占用', '峰值内存 <3 GB；CPU 均值 <70%（4 核）', 'top / tegrastats'),
        ('NFR-04', '可靠性', '连续运行 2 h 无节点崩溃；I2C 错误率 <0.1%', '长稳测试'),
        ('NFR-05', '安全性', '任何异常路径下 1 s 内电机停转', '注入故障'),
        ('NFR-06', '可维护性', '模块化节点，参数外置 YAML，单模块可独立启停', '代码评审'),
        ('NFR-07', '可移植性', '不依赖厂商镜像，可在标准 Ubuntu 26.04 重建', '全新环境复现'),
        ('NFR-08', '启动时间', '上电到可导航 <60 s', '计时'),
        ('NFR-09', '日志', '关键事件结构化日志，保留 7 天', 'logrotate 检查'),
        ('NFR-10', '温控', '满载 30 min 不触发降频（<80 °C）', 'vcgencmd measure_temp'),
    ],
    widths=[1.8, 2.4, 9.0, 4.4],
)

doc.add_heading('2.5  约束与假设', level=2)
bullets(doc, [
    '底盘无编码器：里程计只能由“指令速度 + IMU 航向”推算，设计上必须显式声明精度边界。',
    '扩展板固件不开放：I2C 寄存器协议以厂商 Raspbot_Lib 为准，不做固件级改动。',
    '供电由车载电池经扩展板提供：新增传感器总功耗需控制在 5 W 以内。',
    '树莓派 5 无硬实时保证：控制环保持在 30 Hz，关键安全逻辑下沉到“超时即停”。',
    '户外仅限平整铺装路面，不进入机动车道，速度上限 1.0 m/s。',
])

# =========================================================== 3 总体架构
doc.add_page_break()
doc.add_heading('3  总体架构', level=1)

doc.add_heading('3.1  设计原则', level=2)
add_table(
    doc,
    ['原则', '说明'],
    [
        ('安全层与语义层分离', '避障/急停只依赖雷达、超声、ToF 等几何传感器；视觉检测用于语义理解，不参与安全决策'),
        ('实时与离线分离', 'Pi 5 负责实时控制与本地推理；NAS 负责模型训练、数据回放、离线建图后处理'),
        ('降级可用', '任一传感器缺失时系统仍能以较低能力运行，并明确告警'),
        ('参数外置', '所有标定值与阈值放在 YAML，不写死在代码里'),
        ('单一数据源', '每个物理量只有一个发布者，避免多节点争抢同一 I2C 设备'),
    ],
    widths=[4.2, 12.2],
)

doc.add_heading('3.2  系统分层', level=2)
add_table(
    doc,
    ['层次', '组件', '职责'],
    [
        ('应用层', 'Nav2（planner/controller/BT）、slam_toolbox、teleop、诊断', '任务编排、路径规划、建图与定位'),
        ('融合层', 'ekf_node（robot_localization）、安全监控节点', '多源位姿融合、速度限幅、故障降级'),
        ('抽象层', '标准话题：/cmd_vel、/odom、/scan、/imu/data、/image_raw', '屏蔽硬件差异，向上提供统一接口'),
        ('驱动层', 'raspbot_base、imu_driver、lidar_driver、camera_driver', 'I2C/UART/USB 设备读写与协议解析'),
        ('硬件层', 'Pi 5、扩展板（0x2B）、HWT101、STL-27L、相机', '物理执行与感知'),
    ],
    widths=[2.6, 7.4, 6.4],
)

doc.add_paragraph('运行时数据流（单箭头表示数据方向）：')
add_code(doc, """相机 ──/image_raw──┐
雷达 ──/scan────────┤
IMU  ──/imu/data────┼──> ekf_node ──/odometry/filtered + TF──> Nav2 ──/cmd_vel──> raspbot_base ──I2C──> 底盘
超声 ──/range───────┤                     ▲
巡线 ──/line_sensor─┘                     └── slam_toolbox ──/map──> 全局代价地图""")

doc.add_heading('3.3  部署视图', level=2)
add_table(
    doc,
    ['节点位置', '职责', '说明'],
    [
        ('树莓派 5（车载）', '全部实时节点：驱动、融合、SLAM、Nav2、安全监控', '断网也能自主运行'),
        ('笔记本（操作端）', 'rviz2 / Foxglove、遥控、参数调试、rosbag 分析', '通过有线或 5G WiFi 接入'),
        ('NAS（离线）', '模型训练与量化、数据集管理、建图后处理、长时数据归档', '不参与控制回路'),
        ('Hailo-8（后续）', '视觉推理加速，安装在 Pi 5 的 M.2 HAT+ 上', '不走网络，避免延迟抖动'),
    ],
    widths=[3.6, 7.6, 5.2],
)
note(doc, '说明：Hailo-8 放 NAS 只适合离线批处理；实时推理必须本地化，否则图像传输的延迟与抖动会破坏控制闭环。')

# =========================================================== 4 硬件方案
doc.add_page_break()
doc.add_heading('4  硬件方案设计', level=1)

doc.add_heading('4.1  IMU 选型论证', level=2)
doc.add_paragraph(
    '底盘无编码器，航向角只能来自 IMU，因此 IMU 是本项目的必选项，且需要“开箱可用”的融合输出。'
)
add_table(
    doc,
    ['型号', '价格', '接口', '优点', '缺点', '结论'],
    [
        ('HWT101', '约 180 元', 'UART TTL / I2C', '9 轴融合、100 Hz、金属外壳、中文资料、ROS 驱动现成', '动态精度略逊 BNO085', '选用'),
        ('BNO085', '约 100 元', 'I2C / UART-RVC', 'SH-2 融合最强、直接输出四元数、抗磁干扰好', '模块质量参差、I2C 接线繁琐', '备选'),
        ('JY901S', '约 130 元', 'UART / I2C', '便宜、资料全', '振动下漂移明显', '不选'),
        ('ICM-20948', '约 60 元', 'I2C / SPI', '便宜、裸传感器', '需自行实现融合与标定', '不选'),
    ],
    widths=[2.2, 2.0, 2.6, 5.6, 4.6, 1.6],
)
doc.add_paragraph(
    '结论：选用 HWT101。其出厂完成温度补偿与标定，串口直接输出姿态，可节省自研融合与标定的工作量；'
    '接口留 UART 与 I2C 两种，便于后续更换。'
)

doc.add_heading('4.2  激光雷达选型论证', level=2)
add_table(
    doc,
    ['型号', '价格', '量程（日光下）', '原理', 'ROS 2 支持', '结论'],
    [
        ('STL-27L', '约 700 元', '25 m（衰减小）', 'DTOF', 'ldlidar_stl_ros2', '选用'),
        ('RPLIDAR C1', '约 500 元', '12 m（6–8 m）', '三角测距', 'rplidar_ros', '备选'),
        ('LD19', '约 350 元', '12 m（6–8 m）', '三角测距', 'ldlidar_stl_ros2', '室内够用'),
        ('RPLIDAR A1M8', '约 600 元', '12 m', '三角测距', 'rplidar_ros', '型号过老'),
    ],
    widths=[2.4, 2.0, 3.6, 2.2, 3.4, 1.6],
)
doc.add_paragraph('选型依据与澄清：')
bullets(doc, [
    '低速避障并不需要 25 m 量程：1.5 m/s 时制动 + 延迟位移约 1.0 m，6 m 已有 6 倍余量；',
    '雷达的主要价值在“定位与建图”，三角测距在强光下不仅量程下降，还会产生随机噪点，直接破坏扫描匹配；',
    'DTOF 的抗环境光能力解决了户外建图的稳定性问题，因此按“未来户外也要愉快使用”的目标一次到位；',
    '2D 雷达只扫描一个平面，无法看到路沿、坑洞等低矮目标，需由超声/ToF 补盲。',
])

doc.add_heading('4.3  计算与存储', level=2)
add_table(
    doc,
    ['项目', '现状', '建议'],
    [
        ('CPU', '4×Cortex-A76 @2.4 GHz', 'Nav2 + SLAM + 感知峰值约 2 核，余量充足；建议 governor 设为 performance'),
        ('内存', '8 GB', '全套节点峰值约 1.5–2.5 GB，余量足够；已配置 4 GB swap 防突发'),
        ('存储', 'microSD 58 GB', '建议升级 NVMe（M.2 HAT+），避免建图与 rosbag 写入磨损'),
        ('散热', '需主动散热', '官方 Active Cooler，满载 30 min 不超 80 °C'),
        ('AI 加速', '暂无', '后续在 M.2 HAT+ 上安装 Hailo-8（26 TOPS），与 NVMe 二选一或使用双 M.2 方案'),
    ],
    widths=[2.6, 5.0, 8.8],
)

doc.add_heading('4.4  接口与供电', level=2)
add_table(
    doc,
    ['设备', '接口', '设备节点', '供电', '备注'],
    [
        ('扩展板', 'I2C（GPIO2/3）', '/dev/i2c-1 @0x2B', 'Pi 5 排针', '厂商库以 smbus 访问'),
        ('HWT101', 'UART（USB-TTL）', '/dev/ttyUSB0 → /dev/raspbot_imu', '5 V / <0.2 W', '100 Hz，115200 8N1'),
        ('STL-27L', 'USB（内置转串口）', '/dev/ttyUSB1 → /dev/raspbot_lidar', '5 V / 约 1 W', '10 Hz，230400 8N1'),
        ('USB 相机', 'USB 2.0/3.0', '/dev/video0', '5 V / 约 0.5 W', 'MJPG 640×480@30'),
        ('OLED（板载）', 'I2C', '0x3C', '—', '本期可选，用于显示 IP/状态'),
    ],
    widths=[2.8, 3.4, 4.6, 2.8, 3.0],
)
doc.add_paragraph('供电与布线要求：')
bullets(doc, [
    '两个 USB 转串口设备分别插 Pi 5 的不同 USB 控制器端口，避免带宽与中断争用；',
    '雷达与 IMU 的 5 V 取自 Pi 的 USB 口，总电流 <300 mA，无需额外分电；',
    'IMU 信号线与电机动力线分束走线，交叉处尽量垂直，减少 PWM 干扰；',
    '所有新增设备共地，且只通过 Pi 单点接地，避免地环路。',
])

doc.add_heading('4.5  机械安装', level=2)
add_table(
    doc,
    ['部件', '安装位置', '要求'],
    [
        ('HWT101', '车体几何中心附近，硬质底板', '刚性固定、远离电机与电源线、标注 X 轴朝向，安装后记录偏航角'),
        ('STL-27L', '车顶前部，离地 15–20 cm', '视场无遮挡、避开相机与云台、扫描面与地面平行（误差 <2°）'),
        ('USB 相机', '云台或车头，离地 20–25 cm', '俯角 5–10°，避免逆光，加遮光罩'),
        ('线束', '沿车体边缘固定', '留出云台运动余量，避免拉扯'),
    ],
    widths=[3.0, 6.4, 7.0],
)

doc.add_heading('4.6  时钟与同步', level=2)
bullets(doc, [
    '车载各节点同机运行，使用系统单调时钟；不使用仿真时间（use_sim_time=false）。',
    '跨机（笔记本/NAS）通信依赖 chrony 同步，偏差控制在 10 ms 内，否则 rosbag 回放与多机 TF 会错位。',
    'IMU 与雷达的时间戳以驱动接收时刻为准，融合时使用 message_filters 的近似时间同步（容差 20 ms）。',
])

# =========================================================== 5 BOM
doc.add_page_break()
doc.add_heading('5  BOM 清单', level=1)

doc.add_heading('5.1  已有设备', level=2)
add_table(
    doc,
    ['序号', '名称', '型号/规格', '数量', '参考单价', '小计', '备注'],
    [
        ('1', '上位机', '树莓派 5 8GB', '1', '750', '750', '已部署 Ubuntu 26.04 + ROS 2 Lyrical'),
        ('2', '小车底盘', '亚博 RASPBOT-V2 麦轮套装', '1', '1200', '1200', '含扩展板、电机、舵机、超声、巡线'),
        ('3', '相机', 'USB 相机（MJPG 640×480）', '1', '80', '80', '已实测出图；IMX219 待确认'),
        ('4', '存储', 'microSD 64GB', '1', '60', '60', '建议后续换 NVMe'),
        ('5', '散热', '主动散热器', '1', '60', '60', '防止满载降频'),
    ],
    widths=[1.2, 2.6, 4.6, 1.2, 2.2, 1.8, 4.4],
)

doc.add_heading('5.2  本次新增', level=2)
add_table(
    doc,
    ['序号', '名称', '型号/规格', '数量', '参考单价', '小计', '用途'],
    [
        ('1', 'IMU', 'HWT101（UART）', '1', '180', '180', '航向与姿态，里程计核心'),
        ('2', '2D 激光雷达', 'STL-27L（DTOF，25 m）', '1', '700', '700', '定位、建图、避障'),
        ('3', 'USB-TTL 模块', 'CH340/CP2102', '1', '20', '20', 'IMU 串口转接（若模块已带则免）'),
        ('4', '雷达支架', '铝合金/3D 打印', '1', '50', '50', '固定雷达并保证水平'),
        ('5', 'IMU 支架与螺丝', 'M2/M3 铜柱、螺丝包', '1', '20', '20', '刚性安装'),
        ('6', 'USB 延长线/理线', '20 cm，带磁环', '2', '15', '30', '走线与抗干扰'),
        ('7', '分电与开关', '5V 分线板 + 船型开关', '1', '30', '30', '独立控制传感器供电'),
        ('', '合计', '', '', '', '1030', ''),
    ],
    widths=[1.2, 3.0, 4.2, 1.2, 2.2, 1.6, 4.4],
)

doc.add_heading('5.3  可选升级（非本期）', level=2)
add_table(
    doc,
    ['项目', '型号', '价格', '收益', '触发条件'],
    [
        ('NVMe 存储', 'M.2 HAT+ + 256GB SSD', '约 330 元', '读写稳定、寿命长、启动快', '开始频繁建图/录包'),
        ('AI 加速', 'Hailo-8（M.2，26 TOPS）', '约 1000 元', '视觉检测/车道线实时化', '需语义感知'),
        ('低矮障碍补盲', 'VL53L5CX ToF 阵列', '约 80 元', '防跌落、低矮障碍', '户外台阶/坑洞场景'),
        ('编码器改装', 'AS5600 + TCA9548A + RP2040', '约 150 元', '轮速闭环、里程计精度', '需要定距移动或速度闭环'),
        ('全局定位', 'GNSS + RTK', '800–3000 元', '户外绝对定位', '户外大范围自主'),
    ],
    widths=[3.0, 4.6, 2.2, 4.6, 4.0],
)

doc.add_heading('5.4  采购与验收要点', level=2)
bullets(doc, [
    'HWT101 采购时确认“UART 输出、100 Hz、带外壳”版本，并要求提供协议手册。',
    'STL-27L 确认含 USB 转接板与数据线，且驱动支持 ROS 2（ldlidar_stl_ros2 的 stl27l 型号）。',
    '到货后先做单机验证：IMU 静止读数稳定、雷达在 rviz2 中点云与实物一致，再上车集成。',
])

# =========================================================== 6 软件架构
doc.add_page_break()
doc.add_heading('6  软件架构设计', level=1)

doc.add_heading('6.1  节点划分', level=2)
add_table(
    doc,
    ['节点', '包', '主要功能', '频率'],
    [
        ('raspbot_base', 'raspbot_base', 'I2C 底盘驱动、麦轮逆解、/odom、超声与巡线发布', '控制 30 Hz / 传感 5 Hz'),
        ('imu_driver', 'hwt101_driver（自研或现成）', '串口解析姿态、发布 /imu/data', '100 Hz'),
        ('lidar_driver', 'ldlidar_stl_ros2', '雷达数据解析、发布 /scan', '10 Hz'),
        ('camera_driver', 'v4l2_camera', '图像采集、发布 /image_raw 与 camera_info', '15–30 Hz'),
        ('ekf_node', 'robot_localization', '融合 IMU 与速度指令，输出 /odometry/filtered 与 TF', '30 Hz'),
        ('slam_toolbox', 'slam_toolbox', '在线建图、地图保存', '按扫描触发'),
        ('nav2 栈', 'nav2_bringup', '规划、控制、行为树、代价地图', '规划 1 Hz / 控制 20 Hz'),
        ('safety_monitor', 'raspbot_safety（自研）', '心跳、超时、I2C 错误、急停', '10 Hz'),
        ('teleop', 'teleop_twist_keyboard', '人工遥控', '事件驱动'),
    ],
    widths=[3.2, 4.4, 8.4, 3.4],
)

doc.add_heading('6.2  话题与服务', level=2)
add_table(
    doc,
    ['话题', '类型', '方向', '频率', 'QoS', '说明'],
    [
        ('/cmd_vel', 'geometry_msgs/Twist', '订阅', '≤30 Hz', 'Reliable', '速度指令，超时即停'),
        ('/odom', 'nav_msgs/Odometry', '发布', '30 Hz', 'Reliable', '开环积分 + IMU 航向'),
        ('/imu/data', 'sensor_msgs/Imu', '发布', '100 Hz', 'SensorData', '姿态与角速度'),
        ('/scan', 'sensor_msgs/LaserScan', '发布', '10 Hz', 'SensorData', '2D 雷达'),
        ('/range', 'sensor_msgs/Range', '发布', '5 Hz', 'SensorData', '超声波'),
        ('/line_sensor', 'std_msgs/UInt8', '发布', '5 Hz', 'Reliable', '四路巡线原始字节'),
        ('/image_raw', 'sensor_msgs/Image', '发布', '15 Hz', 'SensorData', '相机图像'),
        ('/buzzer', 'std_msgs/Bool', '订阅', '事件', 'Reliable', '蜂鸣器'),
        ('/led', 'std_msgs/ColorRGBA', '订阅', '事件', 'Reliable', '灯条颜色'),
        ('/servo_pan', 'std_msgs/Float32', '订阅', '事件', 'Reliable', '云台水平角'),
        ('/servo_tilt', 'std_msgs/Float32', '订阅', '事件', 'Reliable', '云台俯仰角'),
        ('/diagnostics', 'diagnostic_msgs/DiagnosticArray', '发布', '1 Hz', 'Reliable', '健康状态'),
        ('/map', 'nav_msgs/OccupancyGrid', '发布', '事件', 'Transient Local', 'SLAM 地图'),
        ('/odometry/filtered', 'nav_msgs/Odometry', '发布', '30 Hz', 'Reliable', 'EKF 输出'),
    ],
    widths=[3.0, 4.4, 1.6, 2.0, 2.6, 5.8],
)

doc.add_heading('6.3  TF 树', level=2)
add_code(doc, """map
 +-- odom                  (slam_toolbox / amcl 发布)
      +-- base_link         (ekf_node 发布，融合 IMU 与速度)
           +-- laser        (静态，x/y/z + yaw 标定)
           +-- imu_link     (静态，安装偏置)
           +-- camera_link  (静态，含俯角)
           +-- ultrasonic   (静态)
           +-- base_footprint (静态，地面投影)""")
note(doc, '静态 TF 全部写入 URDF/xacro，由 robot_state_publisher 统一发布，禁止在多个节点里重复发布同一变换。')

doc.add_heading('6.4  里程计与融合策略', level=2)
doc.add_paragraph(
    '由于底盘无编码器，里程计采用“IMU 航向 + 指令速度积分”的组合，必须明确其边界：'
)
add_table(
    doc,
    ['阶段', '方案', '精度预期', '适用'],
    [
        ('阶段一（本期）', '航向取 IMU 四元数，平移由指令速度在 IMU 航向下积分', '直线 2 m 误差 <5 cm；转动误差 <2°', '室内低速、短距离'),
        ('阶段二', '叠加雷达扫描匹配，ekf 融合', '20 m² 闭环误差 <0.2 m', '室内建图与导航'),
        ('阶段三（可选）', '加编码器或光流，形成轮速/速度真值', '接近常规移动机器人', '户外长距离'),
    ],
    widths=[3.2, 7.0, 5.2, 3.6],
)
doc.add_paragraph('融合实现要点：')
bullets(doc, [
    'IMU 安装后需标定：静止 10 s 估计零偏，记录安装偏航角写入参数。',
    '速度指令作为“模型输入”而非“观测”，在 EKF 中以 twist 形式低权重融合，避免把打滑误差当成真值。',
    '里程计协方差按实际误差设置（x/y 0.05–0.10，yaw 0.02），供代价地图与规划器参考。',
])

doc.add_heading('6.5  安全与降级', level=2)
add_table(
    doc,
    ['风险', '检测方式', '动作'],
    [
        ('指令丢失', '/cmd_vel 超时 0.5 s', '四轮置零并保持'),
        ('IMU 掉线', '话题超时 1 s', '回落开环航向，日志告警，限速 0.5 m/s'),
        ('雷达掉线', '/scan 超时 0.5 s', '停止自主导航，仅保留遥控'),
        ('I2C 连续错误', '错误计数 >10 次/分钟', '节点重启，重置扩展板'),
        ('节点崩溃', 'systemd 监测', '5 s 内重启，重启期间电机由看门狗置零'),
        ('温度过高', '>80 °C', '降低控制频率并告警'),
        ('急停', '物理按钮或 /estop 话题', '立即断使能，需人工复位'),
    ],
    widths=[3.0, 5.4, 8.0],
)

doc.add_heading('6.6  代码组织', level=2)
add_code(doc, """ros2_ws/
└── src/
    ├── raspbot_base/          # 底盘驱动（已完成）
    │   ├── raspbot_base/
    │   │   ├── base_node.py
    │   │   └── Raspbot_Lib.py
    │   ├── config/params.yaml
    │   └── launch/base.launch.py
    ├── raspbot_description/   # URDF/xacro、静态 TF
    ├── raspbot_bringup/       # 总 launch、nav2 参数、slam 参数
    ├── raspbot_safety/        # 安全监控与看门狗
    └── hwt101_driver/         # IMU 串口驱动（可用现成包替代）""")
doc.add_paragraph('规范：每个包单一职责；参数进 YAML；节点启动用 launch 组合；所有外设走 udev 固定名。')

doc.add_heading('6.7  运行与部署', level=2)
doc.add_paragraph('udev 规则（/etc/udev/rules.d/99-raspbot.rules）：')
add_code(doc, """# IMU: CH340
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="raspbot_imu"
# LiDAR: STL-27L（按实际 VID/PID 调整）
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="raspbot_lidar"
# 权限
KERNEL=="ttyUSB*", MODE="0666\"""")
doc.add_paragraph('systemd 服务（示例）：')
add_code(doc, """[Unit]
Description=RASPBOT base driver
After=network.target

[Service]
ExecStart=/bin/bash -lc 'source /opt/ros/lyrical/setup.bash && source /home/ubuntu/ros2_ws/install/setup.bash && ros2 run raspbot_base base_node'
Restart=always
RestartSec=5
User=ubuntu

[Install]
WantedBy=multi-user.target""")

# =========================================================== 7 排期
doc.add_page_break()
doc.add_heading('7  软件需求开发排期', level=1)

doc.add_heading('7.1  迭代计划', level=2)
doc.add_paragraph('按 6 周、6 个迭代推进，每个迭代末产出可演示的增量。起始日 2026-11-11。')
add_table(
    doc,
    ['迭代', '周期', '主要任务', '交付物', '里程碑'],
    [
        ('S1', '11/11–11/17', '硬件到货与安装；udev 固定设备名；IMU/雷达驱动接入；URDF 与静态 TF；遥控联调', '能遥控行驶，rviz2 中可见 TF、点云、图像', 'M1a'),
        ('S2', '11/18–11/24', 'IMU 标定与零偏；里程计融合改造；pwm_per_mps 与 k_geom 标定；速度闭环测试', '直行 2 m 误差 <5 cm，转 90° 误差 <2°', 'M1'),
        ('S3', '11/25–12/01', 'slam_toolbox 参数整定；建图流程；地图保存/加载；重定位初测', '20 m² 室内地图，闭环误差 <0.2 m', 'M2'),
        ('S4', '12/02–12/08', 'Nav2 配置（代价地图、控制器、行为树）；定点导航；恢复行为', '5 个目标点成功率 ≥90%', 'M3'),
        ('S5', '12/09–12/15', '避障与安全策略；异常恢复；性能与温控优化；rosbag 工具链', '障碍前 0.3 m 停车；断网/掉线可恢复', '—'),
        ('S6', '12/16–12/22', '户外场景验证；长稳测试；文档与移交', '户外演示通过；设计文档定稿', 'M4'),
    ],
    widths=[1.2, 2.8, 7.4, 4.4, 1.8],
)

doc.add_heading('7.2  里程碑定义', level=2)
add_table(
    doc,
    ['里程碑', '日期', '验收标准'],
    [
        ('M1a', '11/17', '遥控可用；IMU 与雷达数据在 rviz2 正常显示；TF 树完整'),
        ('M1', '11/24', '里程计指标达标；底盘标定参数固化进 YAML'),
        ('M2', '12/01', '室内地图可用，重定位成功'),
        ('M3', '12/08', '定点导航闭环，无碰撞'),
        ('M4', '12/22', '户外演示通过，文档与脚本齐全'),
    ],
    widths=[2.0, 2.6, 11.8],
)

doc.add_heading('7.3  资源与依赖', level=2)
add_table(
    doc,
    ['资源', '需求', '到位时间', '风险'],
    [
        ('HWT101', '1 套', 'S1 前', '缺货则用 BNO085 替代，接口层预留'),
        ('STL-27L', '1 套', 'S1 前', '缺货则先用 LD19 顶替，参数可切换'),
        ('调试场地', '室内 ≥20 m²，地面平整', 'S3 前', '场地受限则缩小地图范围'),
        ('笔记本', '跑 rviz2/Foxglove', '已有', '—'),
        ('人力', '1 人全栈 + 0.5 人测试', '全程', '—'),
    ],
    widths=[3.0, 5.4, 3.0, 5.0],
)

doc.add_heading('7.4  缓冲与裁剪', level=2)
bullets(doc, [
    '每周预留 0.5 天缓冲，用于硬件返工与参数返工。',
    '若 S3 建图受阻，优先保证 S4 的“静态地图 + 定点导航”，建图改为离线完成。',
    '若硬件延迟到货，S1 先用现有 IMU（如有）或跳过融合，直接做 Nav2 空跑验证。',
])

# =========================================================== 8 测试
doc.add_page_break()
doc.add_heading('8  测试与验收', level=1)

doc.add_heading('8.1  测试策略', level=2)
add_table(
    doc,
    ['层级', '内容', '工具'],
    [
        ('单元测试', '麦轮逆解、限幅、超时逻辑、协议解析', 'pytest / colcon test'),
        ('集成测试', '节点间话题连通、TF 一致性、QoS 匹配', 'ros2 topic hz/echo、tf2_echo'),
        ('实车测试', '直行、转向、横移、避障、建图、导航', '测试用例表'),
        ('长稳测试', '连续 2 h 运行，监控内存/温度/I2C 错误', 'top、vcgencmd、日志统计'),
        ('回归测试', '每次改动后跑固定 10 项用例', '脚本化'),
    ],
    widths=[2.6, 8.6, 5.2],
)

doc.add_heading('8.2  验收用例', level=2)
add_table(
    doc,
    ['编号', '用例', '步骤', '通过标准'],
    [
        ('TC-01', '直行精度', '架空校准后落地，发 0.2 m/s 持续 10 s', '位移 2.0 m ± 5 cm，横向偏差 <3 cm'),
        ('TC-02', '转向精度', '原地 0.5 rad/s 转 4 s', '实际 114.6° ± 2°'),
        ('TC-03', '横移', '发 linear.y=0.2，持续 2 s', '横向位移 0.4 m ± 5 cm，航向不变'),
        ('TC-04', '超时停车', '发布后立即杀掉发布端', '0.5 s 内四轮停转'),
        ('TC-05', '建图闭环', '20 m² 绕行一圈回起点', '位置误差 <0.2 m，地图无重影'),
        ('TC-06', '重定位', '给定偏离 0.3 m / 10° 的初始位姿', '10 s 内收敛，误差 <0.1 m'),
        ('TC-07', '定点导航', '随机 5 个目标点', '成功率 ≥90%，无碰撞'),
        ('TC-08', '动态避障', '路径中放置 30 cm 高障碍', '0.3 m 前停车或绕行'),
        ('TC-09', '传感器掉线', '运行中拔掉雷达', '自主导航退出，遥控仍可用'),
        ('TC-10', '长稳', '连续 2 h 巡航', '无崩溃，内存增长 <10%，温度 <80 °C'),
    ],
    widths=[1.6, 3.0, 6.6, 5.4],
)

doc.add_heading('8.3  验收标准汇总', level=2)
bullets(doc, [
    'P0 需求全部通过，P1 通过率 ≥80%，P2 可留待后续。',
    '所有安全相关用例（TC-04、TC-08、TC-09）必须 100% 通过。',
    '代码可在一台全新 Ubuntu 26.04 上按文档复现部署。',
    '关键参数、标定值、地图、launch 文件纳入版本管理。',
])

# =========================================================== 9 风险
doc.add_page_break()
doc.add_heading('9  风险登记册', level=1)
add_table(
    doc,
    ['编号', '风险', '概率', '影响', '对策'],
    [
        ('R-01', '无编码器导致里程计漂移', '高', '高', 'IMU 航向 + 雷达扫描匹配；限速；缩短单次任务距离'),
        ('R-02', '2D 雷达看不到低矮障碍/坑洞', '高', '高', '超声 + ToF 补盲；限定行驶区域；人工监护'),
        ('R-03', '户外阳光造成点云噪点', '中', '中', '选用 DTOF；调低 max_range；开启滤波'),
        ('R-04', 'Pi 5 热降频影响控制周期', '中', '中', '主动散热；performance governor；限制并发'),
        ('R-05', 'SD 卡磨损/IO 瓶颈', '中', '中', '换 NVMe；限制 rosbag 保留；日志轮转'),
        ('R-06', 'USB 带宽/供电不足', '低', '中', '设备分端口；独立 5V 分电；线材带磁环'),
        ('R-07', 'Nav2 参数与车体不匹配', '中', '中', '分阶段调参；先静态地图后动态避障'),
        ('R-08', 'IMU 磁干扰导致航向跳变', '中', '中', '远离电机与电源线；启用 9 轴融合的磁校准；必要时切 6 轴'),
        ('R-09', '户外无全局定位，走远失位', '高', '中', '限定范围；地图边界；后续加 GNSS'),
        ('R-10', '底盘机械极限（无悬挂、无刹车）', '高', '高', '限定平整路面与 1 m/s 上限；加装缓冲与急停'),
    ],
    widths=[1.4, 6.6, 1.6, 1.6, 8.4],
)

# =========================================================== 10 演进
doc.add_heading('10  后续演进路线', level=1)
add_table(
    doc,
    ['阶段', '内容', '关键收益', '前置条件'],
    [
        ('E1', '轮速编码器（AS5600 + TCA9548A + RP2040）', '轮速闭环、里程计精度提升', '确认轮轴空间，制作支架'),
        ('E2', 'Hailo-8 边缘推理（Pi 5 M.2 HAT+）', '检测/分割实时化，语义层成型', '散热与供电评估，模型量化'),
        ('E3', '多传感器融合升级（EKF 双实例）', '局部连续 + 全局跳变分离', 'E1/E2 数据可用'),
        ('E4', '户外全局定位（GNSS/RTK 或视觉重定位）', '大范围自主', '场地与法规允许'),
        ('E5', '底盘升级（阿克曼/悬挂/制动）', '真实道路能力', '预算与机械能力'),
    ],
    widths=[1.4, 6.8, 5.2, 4.8],
)

doc.add_heading('11  附录', level=1)

doc.add_heading('11.1  常用命令', level=2)
add_code(doc, """# 底盘
ros2 run raspbot_base base_node
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"

# 设备检查
sudo i2cdetect -y 1
ls -l /dev/raspbot_imu /dev/raspbot_lidar /dev/video0

# 建图
ros2 launch slam_toolbox online_async_launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/room

# 导航
ros2 launch nav2_bringup navigation_launch.py map:=/home/ubuntu/maps/room.yaml

# 诊断
ros2 topic hz /scan
ros2 run tf2_tools view_frames""")

doc.add_heading('11.2  参考路径', level=2)
add_table(
    doc,
    ['内容', '路径'],
    [
        ('厂商资料库（PDF/视频）', r'E:\开发资料库\RASPBOT-V2 AI视觉小车'),
        ('厂商库与示例源码', r'E:\code\raspberrypi\docs\raspbot\py'),
        ('ROS 2 底盘包', r'E:\code\raspberrypi\ros2_ws\src\raspbot_base'),
        ('项目技能文档', r'E:\code\raspberrypi\.dsh\skills\raspbot-v2'),
        ('相机抓帧样例', r'E:\code\raspberrypi\docs\raspbot\frame4.jpg'),
    ],
    widths=[6.0, 10.4],
)

doc.save(OUT)
print('SAVED', OUT)
print('SIZE', os.path.getsize(OUT))
