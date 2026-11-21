# -*- coding: utf-8 -*-
"""
RASPBOT-V2 —— IMU / 2D 激光雷达 安装件参数化生成器（Blender 4.2 无头运行）

用法：
    blender -b --factory-startup -P build.py -- [--render] [--stl]

坐标约定（所有零件通用，单位 mm）：
    x 向右，y 向前（车头），z 向上
    原点 = 上层甲板（顶盖）前缘中点，甲板顶面 z = 0
    甲板：x ∈ [-45, 45]，y ∈ [-170, 0]，板厚 2.0，顶面 z = 0

所有关键尺寸集中在 PARAMS 里，量完车直接改这里再生成。
"""
import bpy, bmesh, math, os, sys
from mathutils import Vector

# ---------------------------------------------------------------- PARAMETERS
P = dict(
    # ---- 车辆接口（用卡尺量一下这三处，和下面的默认值对齐即可）----
    deck_front_edge_y=0.0,     # 甲板前缘
    deck_thickness=2.0,        # 甲板(顶盖)厚度
    deck_width=90.0,           # 甲板宽度
    deck_top_z=0.0,            # 甲板顶面 = 0

    # ---- 激光雷达托盘 ----
    shelf_w=80.0,              # 托盘宽
    shelf_y0=-12.0,            # 托盘后缘（伸到甲板上方 12mm，用于卡爪）
    shelf_y1=58.0,             # 托盘前缘（C1 机体 55.6 见方，需要 58）
    shelf_t=5.0,               # 托盘厚
    shelf_gap=4.0,             # 托盘下表面离甲板顶面的高度
    clamp_x=31.0,              # 卡爪中心 x（左右对称）
    clamp_w=14.0,              # 卡爪宽
    clamp_depth=11.0,          # 卡爪咬入甲板的深度
    clamp_slot=3.4,            # 卡槽高度（容纳 2mm 甲板 + 余量）
    clamp_screw_d=2.5,         # M3 自攻底孔
    lidar_center_y=30.0,       # 雷达安装中心
    slot_r0=10.0, slot_r1=24.0, slot_w=3.4,   # 通用八槽（BCD 20–48，覆盖 C1）
    center_hole_d=22.0,        # 走线孔
    # ---- IMU（HWT101）----
    imu_frame_w=62.0, imu_frame_l=70.0, imu_frame_t=5.0,
    pi_hole_dx=49.0, pi_hole_dy=58.0, pi_hole_d=2.7,
    # HWT101 载板：先按 30×24 孔距设计，量到实际孔距后改 imu_hole_dx/dy
    imu_island_w=48.0, imu_island_l=40.0, imu_island_t=4.0,
    imu_hole_dx=30.0, imu_hole_dy=28.0,       # 槽中心（槽长 14，可覆盖 ±7）
    imu_slot_len=14.0, imu_slot_w=3.2,
    damper_x=11.0, damper_y=9.0, damper_d=3.4,
)

OUT_STL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
OUT_IMG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "render")
os.makedirs(OUT_STL, exist_ok=True)
os.makedirs(OUT_IMG, exist_ok=True)


# ---------------------------------------------------------------- helpers
def _new_mesh_obj(name, verts, faces):
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.validate()
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    return ob


def box(name, x0, x1, y0, y1, z0, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
         (2, 6, 7, 3), (3, 7, 4, 0)]
    return _new_mesh_obj(name, v, f)


def cyl(name, cx, cy, z0, z1, d, segs=48, rot=None):
    r = d / 2.0
    verts, faces = [], []
    for k, z in enumerate((z0, z1)):
        for i in range(segs):
            a = 2 * math.pi * i / segs
            verts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    for i in range(segs):
        j = (i + 1) % segs
        faces.append((i, j, segs + j, segs + i))
    faces.append(tuple(range(segs))[::-1])
    faces.append(tuple(range(segs, 2 * segs)))
    return _new_mesh_obj(name, verts, faces)


def cone(name, cx, cy, z0, z1, d0, d1, segs=32):
    """截锥（d0 在 z0，d1 在 z1），用于 90° 沉头孔。"""
    r0, r1 = d0 / 2.0, d1 / 2.0
    verts, faces = [], []
    for r, z in ((r0, z0), (r1, z1)):
        for i in range(segs):
            a = 2 * math.pi * i / segs
            verts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    for i in range(segs):
        j = (i + 1) % segs
        faces.append((i, j, segs + j, segs + i))
    faces.append(tuple(range(segs))[::-1])
    faces.append(tuple(range(segs, 2 * segs)))
    return _new_mesh_obj(name, verts, faces)


def boolean(target, cutter, op="DIFFERENCE"):
    m = target.modifiers.new("b", "BOOLEAN")
    m.operation = op
    m.object = cutter
    m.solver = "EXACT"
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=m.name)
    bpy.data.objects.remove(cutter, do_unlink=True)
    return target


def join_all(objs, name):
    for o in bpy.context.selected_objects:
        o.select_set(False)
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.object.join()
    ob = bpy.context.view_layer.objects.active
    ob.name = name
    ob.data.name = name
    return ob


def clean(ob, merge=0.02):
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=merge)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()


def bevel(ob, width=1.0, segments=2, angle=45):
    m = ob.modifiers.new("bev", "BEVEL")
    m.width = width
    m.segments = segments
    m.limit_method = "ANGLE"
    m.angle_limit = math.radians(angle)
    m.harden_normals = False
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.modifier_apply(modifier=m.name)


def export_stl(ob, path):
    for o in bpy.context.selected_objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True,
                          apply_modifiers=True, ascii_format=False)
    print("STL ->", path, os.path.getsize(path), "bytes")


# ---------------------------------------------------------------- part A
def build_lidar_shelf():
    """前伸托盘 + 甲板前缘卡爪 + 通用八槽雷达安装面"""
    z0 = P["deck_top_z"] + P["shelf_gap"]           # 托盘下表面
    z1 = z0 + P["shelf_t"]                          # 托盘上表面
    # 托盘主体
    ob = box("body", -P["shelf_w"] / 2, P["shelf_w"] / 2,
             P["shelf_y0"], P["shelf_y1"], z0, z1)

    # 两个卡爪：从托盘后段向下包住甲板（C 形，开口朝后），与托盘做并集
    for sx in (-1, 1):
        cx = sx * P["clamp_x"]
        jx0, jx1 = cx - P["clamp_w"] / 2, cx + P["clamp_w"] / 2
        jy0 = P["deck_front_edge_y"] - P["clamp_depth"]      # -11
        jaw = box(f"jaw{sx}", jx0, jx1, jy0, P["deck_front_edge_y"] - 0.5,
                  -9.0, z1)
        m = ob.modifiers.new("u", "BOOLEAN")
        m.operation = "UNION"
        m.object = jaw
        m.solver = "EXACT"
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.modifier_apply(modifier=m.name)
        bpy.data.objects.remove(jaw, do_unlink=True)

        # 卡槽：沿 -y 方向开口，前端留 2.8mm 连接上下颚
        slot_z0 = P["deck_top_z"] - P["deck_thickness"] - 0.2
        slot_z1 = slot_z0 + P["clamp_slot"]
        slot_front = P["deck_front_edge_y"] - 3.2
        cutter = box("slotc", jx0 - 1, jx1 + 1, jy0 - 1, slot_front,
                     slot_z0, slot_z1)
        ob = boolean(ob, cutter)
        # 自攻螺孔（从下往上顶住甲板背面）+ 螺钉头沉孔
        sy = slot_front - 2.4
        h = cyl("screw", cx, sy, -9.5, slot_z0 + 0.1, P["clamp_screw_d"], 24)
        ob = boolean(ob, h)
        cb = cyl("cb", cx, sy, -9.5, -7.0, 6.2, 24)
        ob = boolean(ob, cb)

    clean(ob)

    # ---- 通用八槽（雷达安装）----
    for k in range(8):
        a = math.radians(45 * k)
        cx, cy = 0.0, P["lidar_center_y"]
        L = P["slot_r1"] - P["slot_r0"]
        rm = (P["slot_r1"] + P["slot_r0"]) / 2
        # 沿径向的槽，用"旋转的方块"近似：先建轴对齐再旋转顶点
        dx, dy = math.cos(a), math.sin(a)
        px, py = -dy, dx
        hw = P["slot_w"] / 2
        verts = []
        for zz in (z0 - 1, z1 + 1):
            for s, t in ((P["slot_r0"], -hw), (P["slot_r1"], -hw),
                         (P["slot_r1"], hw), (P["slot_r0"], hw)):
                verts.append((cx + dx * s + px * t, cy + dy * s + py * t, zz))
        faces = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1),
                 (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
        cutter = _new_mesh_obj("slot", verts, faces)
        ob = boolean(ob, cutter)

    # 中间走线孔
    ob = boolean(ob, cyl("ch", 0, P["lidar_center_y"], z0 - 1, z1 + 1,
                         P["center_hole_d"], 64))

    # 线缆扎带孔（前部两侧，避开雷达槽区）
    for sx in (-1, 1):
        ob = boolean(ob, box("tie", sx * 32 - 2, sx * 32 + 2,
                             P["shelf_y1"] - 9, P["shelf_y1"] - 2, z0 - 1, z1 + 1))

    # 零位标记：前缘中央一个 4×4 三角缺口（打印出来就是指向标）
    ob = boolean(ob, box("idx", -2, 2, P["shelf_y1"] - 1, P["shelf_y1"] + 1,
                         z0 - 1, z1 + 1))
    clean(ob)
    return ob


# ---------------------------------------------------------------- part B
def build_imu_frame():
    """固定在树莓派 58×49 四孔上的刚性框，中央 4 个 M3 减震柱座"""
    w, l, t = P["imu_frame_w"], P["imu_frame_l"], P["imu_frame_t"]
    # 5mm 实心板：平放打印，无需支撑；减震柱直接坐在板面上
    t = 5.0
    ob = box("imu_frame", -w / 2, w / 2, -l / 2, l / 2, 0, t)
    # 2 条约 3mm 的散热/走线长槽
    for xx in (-14, 14):
        ob = boolean(ob, box("slot", xx - 1.5, xx + 1.5, -l / 2 + 6, l / 2 - 6, -1, t + 1))

    # 树莓派 58×49 孔（沉头从上面拧 M2.5）
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * P["pi_hole_dx"] / 2
            y = sy * P["pi_hole_dy"] / 2
            ob = boolean(ob, cyl("pih", x, y, -1, t + 1, P["pi_hole_d"], 24))
            ob = boolean(ob, cyl("pich", x, y, t - 2.0, t + 1, 5.6, 24))

    # 减震柱过孔（4 个 Ø3.4，下面穿 M3 螺钉）
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * P["damper_x"]
            y = sy * P["damper_y"]
            ob = boolean(ob, cyl("dh", x, y, -1, t + 1, P["damper_d"], 24))
    clean(ob)
    return ob


# ---------------------------------------------------------------- part C
def build_imu_carrier():
    """HWT101 载板：坐在 4 个减震柱上，带可调槽位 + 出线缺口 + 扎带孔。

    - 4 个 M3 沉头孔（减震柱），螺钉头与板面齐平，模块能贴平
    - 4 条长槽：覆盖 22–44mm 的孔距（HWT101 实测后改 imu_hole_dx/dy 即可）
    - 前后各一个出线缺口：弯针插头可以贴着板边出线，不顶住模块
    - 两侧扎带槽：线束应力释放
    """
    w, l, t = P["imu_island_w"], P["imu_island_l"], P["imu_island_t"]
    ob = box("imu_carrier", -w / 2, w / 2, -l / 2, l / 2, 0, t)

    # 减震柱：Ø3.4 过孔 + 90° 沉头（锥形），从上面拧 M3 沉头螺钉
    for sx in (-1, 1):
        for sy in (-1, 1):
            x = sx * P["damper_x"]
            y = sy * P["damper_y"]
            ob = boolean(ob, cyl("dh", x, y, -1, t + 1, P["damper_d"], 24))
            ob = boolean(ob, cone("cs", x, y, t - 1.8, t + 1, 6.4, 3.4, 28))

    # 模块安装长槽：y = ±dy/2 两排，每排左右各一条
    half = P["imu_slot_len"] / 2
    for sy in (-1, 1):
        y = sy * P["imu_hole_dy"] / 2
        for sx in (-1, 1):
            x0 = sx * (P["imu_hole_dx"] / 2 - half)
            x1 = sx * (P["imu_hole_dx"] / 2 + half)
            ob = boolean(ob, box("mslot", min(x0, x1), max(x0, x1),
                                 y - P["imu_slot_w"] / 2, y + P["imu_slot_w"] / 2,
                                 -1, t + 1))

    # 前后出线缺口（弯针/排线从这里出，贴着板面走）
    ob = boolean(ob, box("cableA", -7, 7, l / 2 - 7, l / 2 + 1, -1, t + 1))
    ob = boolean(ob, box("cableB", -7, 7, -l / 2 - 1, -l / 2 + 7, -1, t + 1))

    # 车头方向标记：左前角两个小缺口
    for k in (0, 1):
        x = -w / 2 + 3 + k * 3.2
        ob = boolean(ob, box("idx", x - 1.0, x + 1.0, l / 2 - 5.0, l / 2 - 1.0,
                             -1, t + 1))
    # 两侧扎带槽
    for sx in (-1, 1):
        ob = boolean(ob, box("tie", sx * w / 2 - 1.6, sx * w / 2 + 1.6,
                             -6, 6, -1, t + 1))
    clean(ob)
    return ob


# ---------------------------------------------------------------- part D
def build_lidar_riser():
    """可选加高块：把雷达底座抬高 18mm，用于光学面较高的雷达。"""
    w, l, h, t = 62.0, 44.0, 18.0, 2.4
    ob = box("riser", -w / 2, w / 2, -l / 2, l / 2, 0, t)
    top = box("top", -w / 2, w / 2, -l / 2, l / 2, h - t, h)
    m = ob.modifiers.new("u", "BOOLEAN")
    m.operation = "UNION"
    m.object = top
    m.solver = "EXACT"
    bpy.context.view_layer.objects.active = ob
    bpy.ops.object.modifier_apply(modifier=m.name)
    bpy.data.objects.remove(top, do_unlink=True)
    # 四根角柱
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = box("col", sx * (w / 2 - 5) - 5, sx * (w / 2 - 5) + 5,
                    sy * (l / 2 - 5) - 5, sy * (l / 2 - 5) + 5, 0, h)
            m = ob.modifiers.new("u", "BOOLEAN")
            m.operation = "UNION"
            m.object = c
            m.solver = "EXACT"
            bpy.context.view_layer.objects.active = ob
            bpy.ops.object.modifier_apply(modifier=m.name)
            bpy.data.objects.remove(c, do_unlink=True)
    # 通孔
    for sx in (-1, 1):
        for sy in (-1, 1):
            ob = boolean(ob, cyl("h", sx * (w / 2 - 5), sy * (l / 2 - 5),
                                 -1, h + 1, 3.4, 24))
    clean(ob)
    return ob


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    do_stl = "--stl" in argv or not argv

    shelf = build_lidar_shelf()
    frame = build_imu_frame()
    carrier = build_imu_carrier()
    riser = build_lidar_riser()

    if do_stl:
        export_stl(shelf, os.path.join(OUT_STL, "01_lidar_shelf.stl"))
        export_stl(frame, os.path.join(OUT_STL, "02_imu_frame.stl"))
        export_stl(carrier, os.path.join(OUT_STL, "03_imu_carrier.stl"))
        export_stl(riser, os.path.join(OUT_STL, "04_lidar_riser_18mm.stl"))



main()






