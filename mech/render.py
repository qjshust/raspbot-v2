# -*- coding: utf-8 -*-
"""Render pass: part renders + an assembly context render.

blender -b --factory-startup -P render.py
"""
import bpy, math, os
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
STL = os.path.join(HERE, "stl")
IMG = os.path.join(HERE, "render")
os.makedirs(IMG, exist_ok=True)


# ------------------------------------------------------------------ helpers
def prim_box(name, x0, x1, y0, y1, z0, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 1, 2, 3), (7, 6, 5, 4), (0, 4, 5, 1), (1, 5, 6, 2),
         (2, 6, 7, 3), (3, 7, 4, 0)]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v, [], f)
    me.validate()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    return ob


def prim_cyl(name, cx, cy, z0, z1, d, segs=64):
    r = d / 2
    verts, faces = [], []
    for z in (z0, z1):
        for i in range(segs):
            a = 2 * math.pi * i / segs
            verts.append((cx + r * math.cos(a), cy + r * math.sin(a), z))
    for i in range(segs):
        j = (i + 1) % segs
        faces.append((i, j, segs + j, segs + i))
    faces.append(tuple(range(segs))[::-1])
    faces.append(tuple(range(segs, 2 * segs)))
    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.validate()
    ob = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(ob)
    return ob


def import_stl(path, name):
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=path)
    new = [o for o in bpy.data.objects if o not in before]
    ob = new[0]
    ob.name = name
    return ob


def mat(name, color, rough=0.45, metal=0.0, alpha=1.0, emit=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    b.inputs["Alpha"].default_value = alpha
    if emit:
        b.inputs["Emission Color"].default_value = (*color, 1)
        b.inputs["Emission Strength"].default_value = emit
    if alpha < 1:
        m.blend_method = "BLEND"
    return m


def bbox(objs):
    lo = Vector((1e9,) * 3)
    hi = Vector((-1e9,) * 3)
    for ob in objs:
        for c in ob.bound_box:
            w = ob.matrix_world @ Vector(c)
            lo = Vector((min(lo[i], w[i]) for i in range(3)))
            hi = Vector((max(hi[i], w[i]) for i in range(3)))
    return lo, hi


# ------------------------------------------------------------------ scene
def scene():
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE_NEXT"
    sc.render.resolution_x, sc.render.resolution_y = 1500, 1100
    sc.render.film_transparent = False
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"
    w = bpy.data.worlds.new("w")
    w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = (0.02, 0.025, 0.035, 1)
    w.node_tree.nodes["Background"].inputs[1].default_value = 1.0
    sc.world = w
    cam_d = bpy.data.cameras.new("c")
    cam_d.lens = 70
    cam_d.sensor_width = 36
    cam_d.clip_start = 1.0
    cam_d.clip_end = 6000.0
    cam = bpy.data.objects.new("c", cam_d)
    bpy.context.collection.objects.link(cam)
    sc.camera = cam
    return cam


LIGHTS = []


def setup_lights():
    """太阳光：与场景单位无关，曝光稳定；只建一次，避免首帧灯光未更新。"""
    bpy.context.scene.world.node_tree.nodes["Background"].inputs[1].default_value = 0.45
    specs = [("key", 4.6, (1.0, -1.0, 1.2), (1.0, 0.97, 0.93)),
             ("fill", 2.0, (-1.0, -0.7, 0.55), (0.78, 0.87, 1.0)),
             ("rim", 2.6, (0.1, 1.1, 0.85), (1.0, 0.95, 0.92))]
    for name, e, direct, col in specs:
        ld = bpy.data.lights.new(name, "SUN")
        ld.energy = e
        ld.angle = math.radians(12)
        ld.color = col
        o = bpy.data.objects.new(name, ld)
        bpy.context.collection.objects.link(o)
        o.rotation_euler = Vector(direct).to_track_quat("Z", "Y").to_euler()
        LIGHTS.append(o)


def shoot(cam, objs, direction, out, margin=1.25, res=(1500, 1100)):
    lo, hi = bbox(objs)
    c = (lo + hi) / 2
    r = (hi - lo).length / 2
    fov = 2 * math.atan(cam.data.sensor_width / (2 * cam.data.lens))
    dist = r / math.sin(fov / 2) * margin
    d = Vector(direction).normalized()
    cam.location = c + d * dist
    cam.rotation_euler = (c - cam.location).to_track_quat("-Z", "Y").to_euler()
    print(f"  [shot] c={tuple(round(v,1) for v in c)} r={r:.1f} dist={dist:.1f} "
          f"cam={tuple(round(v,1) for v in cam.location)} lens={cam.data.lens}")
    sc = bpy.context.scene
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.filepath = out
    bpy.context.view_layer.update()
    bpy.ops.render.render(write_still=True)
    print("render ->", out)


# ------------------------------------------------------------------ main
def main():
    cam = scene()
    setup_lights()
    sc = bpy.context.scene

    M_PART = mat("part", (0.13, 0.28, 0.55), 0.40)
    M_PART2 = mat("part2", (0.42, 0.44, 0.47), 0.42)
    M_LIDAR = mat("lidar", (0.035, 0.035, 0.04), 0.55)
    M_PCB = mat("pcb", (0.06, 0.32, 0.14), 0.6)
    M_DECK = mat("deck", (0.10, 0.11, 0.13), 0.7, alpha=0.55)
    M_MOD = mat("mod", (0.05, 0.15, 0.35), 0.5)
    M_RUB = mat("rubber", (0.9, 0.35, 0.05), 0.8)

    shelf = import_stl(os.path.join(STL, "01_lidar_shelf.stl"), "shelf")
    riser = import_stl(os.path.join(STL, "04_lidar_riser_18mm.stl"), "riser")
    frame = import_stl(os.path.join(STL, "02_imu_frame.stl"), "frame")
    carrier = import_stl(os.path.join(STL, "03_imu_carrier.stl"), "carrier")
    shelf.data.materials.append(M_PART)
    riser.data.materials.append(M_PART2)
    frame.data.materials.append(M_PART2)
    carrier.data.materials.append(M_PART)

    # ---- 上下文零件 ----
    deck = prim_box("deck", -45, 45, -170, 0, -2, 0)
    deck.data.materials.append(M_DECK)

    # RPLIDAR C1：55.6 见方 × 41.3 高，扫描面在底座上方约 25mm（示意）
    lidar = prim_box("lidar", -27.8, 27.8, 2.2, 57.8, 9, 50.3)
    lidar.data.materials.append(M_LIDAR)
    band = prim_box("band", -28.3, 28.3, 1.7, 58.3, 31, 37)
    band.data.materials.append(mat("red", (0.9, 0.15, 0.1), 0.4, emit=0.6))

    pi = prim_box("pi", -28, 28, -139, -54, 8, 9.6)   # 56 x 85 PCB
    pi.data.materials.append(M_PCB)
    # HWT101 外形示意：约 46 × 36 × 14 的模块，一侧出线
    mod = prim_box("mod", -23, 23, -18, 18, 0, 14)
    mod.data.materials.append(M_MOD)
    header = prim_box("header", -12, 12, -22, -18, 0, 8)
    header.data.materials.append(mat("hdr", (0.02, 0.02, 0.02), 0.6))

    # 减震柱（示意）
    dampers = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            d = prim_cyl("damp", sx * 11, sy * 9, 6, 9, 7)
            d.data.materials.append(M_RUB)
            dampers.append(d)

    # ---- 位置 ----
    shelf.location = (0, 0, 0)                 # 卡在前缘
    lidar.location = (0, 0, 0)
    riser.location = (0, 30, 4)
    frame.location = (0, -96.5, 9.6)           # 装在 Pi 上方（Pi 中心 y=-96.5）
    carrier.location = (0, -96.5, 9.6 + 12)    # 坐减震柱
    mod.location = (0, -96.5, 9.6 + 12 + 4)
    header.location = (0, -96.5, 9.6 + 12 + 4)
    deck.location = (0, 0, 0)
    pi.location = (0, 0, 0)
    for d in dampers:
        d.location = (0, -96.5, 9.6)

    groups = {
        "shelf": [shelf, lidar, band],
        "bare": [shelf],
        "riser": [riser],
        "imu": [frame, carrier, pi, mod, header] + dampers,
        "all": [shelf, lidar, band, deck, frame, carrier, pi, mod, header] + dampers,
    }

    def show(names):
        keep = set()
        for g in names:
            keep.update(id(o) for o in groups[g])
        seen = set()
        for objs in groups.values():
            for o in objs:
                if id(o) in seen:
                    continue
                seen.add(id(o))
                o.hide_render = id(o) not in keep
                o.hide_viewport = id(o) not in keep

    # 1) 托盘：等轴测 + 底部 + 顶视
    show({"shelf"})
    shoot(cam, groups["shelf"], (0.95, -0.95, 0.55),
          os.path.join(IMG, "01_lidar_shelf_iso.png"))
    shoot(cam, groups["shelf"], (0.3, -0.9, -0.42),
          os.path.join(IMG, "01_lidar_shelf_under.png"))
    show({"bare"})
    shoot(cam, [shelf], (0.02, -0.12, 1.0),
          os.path.join(IMG, "01_lidar_shelf_top.png"))

    # 1b) 加高块
    show({"riser"})
    shoot(cam, groups["riser"], (0.9, -0.9, 0.7),
          os.path.join(IMG, "04_lidar_riser.png"))

    # 2) IMU 叠层
    show({"imu"})
    carrier.location = (0, -96.5, 9.6 + 26)    # 爆炸
    mod.location = (0, -96.5, 9.6 + 26 + 4)
    shoot(cam, groups["imu"], (0.85, -0.95, 0.75),
          os.path.join(IMG, "02_imu_exploded.png"))
    carrier.location = (0, -96.5, 9.6 + 12)
    mod.location = (0, -96.5, 9.6 + 16)
    shoot(cam, groups["imu"], (0.02, -0.12, 1.0),
          os.path.join(IMG, "02_imu_top.png"))

    # 3) 总装（俯视 + 前视等轴）
    show({"all"})
    carrier.location = (0, -96.5, 9.6 + 12)
    mod.location = (0, -96.5, 9.6 + 16)
    shoot(cam, [shelf, lidar, band], (0.9, -0.85, 0.6),
          os.path.join(IMG, "00_assembly_front.png"), margin=1.9)
    shoot(cam, groups["all"], (0.55, -0.9, 0.95),
          os.path.join(IMG, "00_assembly_top.png"), margin=1.18, res=(1400, 1200))
    sc.render.filepath = os.path.join(IMG, "00_assembly_top.png")


main()







