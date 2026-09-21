"""Independent check of a generated model in Blender (headless), plus a preview render.

    blender -b --factory-startup -P tools/assetgen/blender_check.py -- <model_id> [out.png] [joint=value ...]

Imports data/models/<id>/<id>.glb with Blender's own glTF importer, verifies
every joint node named in the rig exists, reports mesh/material counts and
optionally poses joints (rig ISO values, e.g. door_FL=1.0 steer_FL=0.4),
re-aims `aim` links for the pose, and renders a preview with EEVEE.
Camera: VIEW=iso|side|front|rear|top|iso_rear|iso_right|right|iso_rear_right|below,
or a custom one with DIR="x,y,z" (ISO direction from the target), CTR="x,y,z", DIST=metres.
Exit code 1 on any failure. Blender is a dev-time checker only; the
generator itself does not need it.
"""
import json
import math
import os
import sys

import bpy
from mathutils import Matrix, Quaternion, Vector


argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
mid = argv[0]
out_png = argv[1] if len(argv) > 1 and "=" not in argv[1] else None
poses = dict(a.split("=") for a in argv[1:] if "=" in a)
repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
glb = os.path.join(repo, "data", "models", mid, mid + ".glb")
rig = json.load(open(os.path.join(repo, "data", "models", mid, mid + ".rig.json"), encoding="utf-8"))

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=glb)
objs = {o.name: o for o in bpy.data.objects}
fails = []
for n in rig["nodes"] + rig["sockets"]:
    if n["name"] not in objs:
        fails.append(f"node missing after import: {n['name']}")
meshes = [o for o in bpy.data.objects if o.type == "MESH"]
tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in meshes)
print(f"[check] {mid}: {len(bpy.data.objects)} objects, {len(meshes)} meshes, {tris} tris, "
      f"{len(bpy.data.materials)} materials, {len(bpy.data.images)} images")


def iso_to_blender(v):
    # iso -> glTF is (x,y,z) -> (y,z,x); Blender's importer maps glTF (x,y,z) -> (x,-z,y),
    # so iso (x fwd, y left, z up) -> Blender (y, -x, z): the model faces Blender's -Y.
    g = (v[1], v[2], v[0])          # iso -> glTF
    return Vector((g[0], -g[2], g[1]))  # glTF -> Blender


# rest world matrices, for evaluating aim links after posing
rest_world = {o.name: o.matrix_world.copy() for o in bpy.data.objects}

for name, val in poses.items():
    node = next((n for n in rig["nodes"] if n["name"] == name), None)
    if node is None or "joint" not in node or name not in objs:
        fails.append(f"cannot pose {name}")
        continue
    j, o, val = node["joint"], objs[name], float(val)
    ax = iso_to_blender(j["axis"])
    if j["type"] == "prismatic":
        o.location = o.location + ax * val
    else:
        o.rotation_mode = "QUATERNION"
        o.rotation_quaternion = Quaternion(ax, val) @ o.rotation_quaternion

# aim links: rotate by the smallest rotation taking the rest direction onto the current one
bpy.context.view_layer.update()
for n in rig["nodes"]:
    j = n.get("joint")
    if not j or j["type"] != "aim" or n["name"] not in objs or j["target"] not in objs:
        continue
    o, t = objs[n["name"]], objs[j["target"]]
    p_rest = iso_to_blender(j["target_point"])
    p_now = t.matrix_world @ (rest_world[t.name].inverted() @ p_rest)
    pos = o.matrix_world.to_translation()
    d_rest = (p_rest - rest_world[o.name].to_translation())
    d_now = p_now - pos
    if d_rest.length < 1e-9 or d_now.length < 1e-9:
        continue
    q = d_rest.normalized().rotation_difference(d_now.normalized())
    M = q.to_matrix().to_4x4()
    if j.get("stretch"):
        s = d_now.length / d_rest.length
        d = d_rest.normalized()
        S = Matrix.Identity(4)
        for a in range(3):
            for b in range(3):
                S[a][b] += (s - 1) * d[a] * d[b]
        M = M @ S
    world = Matrix.Translation(pos) @ M @ rest_world[o.name].to_3x3().to_4x4()
    o.matrix_world = world
    bpy.context.view_layer.update()

if out_png:
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE" if "BLENDER_EEVEE" in [e.identifier for e in
                                                                bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items] else "BLENDER_EEVEE_NEXT"
    scene.render.resolution_x, scene.render.resolution_y = 1280, 720
    scene.render.film_transparent = False
    world = bpy.data.worlds.new("w")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.62, 0.7, 0.8, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    scene.world = world
    bpy.ops.mesh.primitive_plane_add(size=200)
    ground = bpy.context.active_object
    gm = bpy.data.materials.new("ground")
    gm.use_nodes = True
    gm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.35, 0.36, 0.34, 1)
    ground.data.materials.append(gm)
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 4.0
    sun.rotation_euler = (math.radians(40), math.radians(10), math.radians(-35))
    scene.collection.objects.link(sun)
    lo = Vector(rig["overall_bounds"]["min"])
    hi = Vector(rig["overall_bounds"]["max"])
    ctr_iso = (lo + hi) / 2
    ctr = iso_to_blender(ctr_iso)
    size = max(hi - lo)
    view = os.environ.get("VIEW", "iso")
    dirs = {"iso": (0.8, 0.75, 0.45), "side": (0.0, 1.0, 0.12), "front": (1.0, 0.0, 0.15),
            "rear": (-1.0, -0.2, 0.3), "top": (0.001, 0.0, 1.0), "iso_rear": (-0.8, 0.75, 0.45),
            "iso_right": (0.8, -0.75, 0.45), "right": (0.0, -1.0, 0.12), "iso_rear_right": (-0.8, -0.75, 0.45),
            "below": (0.2, 0.1, -1.0)}
    # custom camera: DIR="x,y,z" (ISO view direction from the target), CTR="x,y,z" (ISO target), DIST=m
    env_v = lambda k: tuple(float(c) for c in os.environ[k].split(",")) if os.environ.get(k) else None
    d = iso_to_blender(env_v("DIR") or dirs[view]).normalized()
    if env_v("CTR"):
        ctr = iso_to_blender(env_v("CTR"))
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    cam.data.lens = 50
    scene.collection.objects.link(cam)
    dist = float(os.environ.get("DIST", 0)) or size * 1.9 + 1.0
    cam.location = ctr + d * dist
    cam.rotation_mode = "QUATERNION"
    cam.rotation_quaternion = (ctr - cam.location).to_track_quat("-Z", "Y")
    scene.camera = cam
    scene.render.filepath = os.path.abspath(out_png)
    bpy.ops.render.render(write_still=True)
    print(f"[check] rendered {out_png}")

if fails:
    for f in fails:
        print("[check] FAIL", f)
    sys.exit(1)
print("[check] OK")
