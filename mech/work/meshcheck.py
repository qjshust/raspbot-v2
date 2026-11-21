import bpy, bmesh, os, glob
for f in sorted(glob.glob(r"E:\code\raspberrypi\mech\stl\*.stl")):
    bpy.ops.wm.stl_import(filepath=f)
    ob = bpy.context.selected_objects[0]
    bm = bmesh.new(); bm.from_mesh(ob.data)
    nm = [e for e in bm.edges if not e.is_manifold]
    shells = 0
    seen = set()
    for v in bm.verts:
        if v.index in seen: continue
        shells += 1
        stack=[v]
        while stack:
            x=stack.pop()
            if x.index in seen: continue
            seen.add(x.index)
            for e in x.link_edges:
                o=e.other_vert(x)
                if o.index not in seen: stack.append(o)
    print(f"{os.path.basename(f):28s} tris={len(bm.faces):6d} verts={len(bm.verts):6d} nonmanifold_edges={len(nm):4d} shells={shells}")
    bm.free()
    bpy.data.objects.remove(ob, do_unlink=True)
