"""Headless Blender preview renders of test_ground.glb (own script -- does
not touch tools/assetgen/blender_check.py, which is vehicle-rig specific).

  "S:\\programs\\Blender 5.2\\blender.exe" -b --factory-startup --python-exit-code 1 ^
      -P tools/trackgen/blender_preview.py

Writes out/renders/track_top.png (near-origin cluster, straight down),
track_persp.png (same area, perspective) and track_ramps.png (close-up top
view of the 5 ramps + side slope) -- short paths, since Windows image writing
fails over 260 characters.
"""
import math
import os
import sys

import bpy

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
GLB = os.path.join(REPO, "data", "tracks", "test_ground", "test_ground.glb")
OUT = os.path.join(REPO, "out", "renders")

# Near-origin cluster bounding box (straight, skidpad, patches, split-mu,
# ramps, side slope, circuit) -- from layout.py's coordinates.
CLUSTER = dict(x0=-520.0, x1=520.0, y0=-1080.0, y1=620.0)
RAMPS_AREA = dict(x0=-20.0, x1=200.0, y0=435.0, y1=610.0)


def world_to_blender(wx, wy):
    """Blender's glTF importer converts glTF Y-up back to Blender's own Z-up
    scene as blender(x,y,z) = gltf(x,-z,y); combined with our own export
    gltf(x,y,z) = world(y,z,x) (world = ISO/ENU x=east,y=north,z=up), that
    gives blender.x = world.y, blender.y = -world.x, blender.z = world.z.
    Verified empirically against the glb's own node bounding boxes."""
    return wy, -wx


def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for coll in list(bpy.data.collections):
        bpy.data.collections.remove(coll)


def import_glb():
    bpy.ops.import_scene.gltf(filepath=GLB)


def frame_top(area, margin=1.15):
    bx, by = world_to_blender((area["x0"] + area["x1"]) / 2.0, (area["y0"] + area["y1"]) / 2.0)
    span = max(area["x1"] - area["x0"], area["y1"] - area["y0"]) * margin
    cam_data = bpy.data.cameras.new("cam_top")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = span
    cam_data.clip_end = 20000.0
    cam = bpy.data.objects.new("cam_top", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    cam.location = (bx, by, 800.0)
    cam.rotation_euler = (0.0, 0.0, 0.0)  # looking down -Z: a plain top-down view
    bpy.context.scene.camera = cam
    sun = bpy.data.lights.new("sun", type="SUN")
    sun.energy = 3.0
    sun_obj = bpy.data.objects.new("sun", sun)
    sun_obj.location = (bx, by, 500.0)
    sun_obj.rotation_euler = (0.3, 0.2, 0.0)
    bpy.context.scene.collection.objects.link(sun_obj)


def frame_persp(area, margin=1.3):
    bx, by = world_to_blender((area["x0"] + area["x1"]) / 2.0, (area["y0"] + area["y1"]) / 2.0)
    span = max(area["x1"] - area["x0"], area["y1"] - area["y0"]) * margin
    cam_data = bpy.data.cameras.new("cam_persp")
    cam_data.lens = 24.0
    cam_data.clip_end = 20000.0
    cam = bpy.data.objects.new("cam_persp", cam_data)
    bpy.context.scene.collection.objects.link(cam)
    dist = span * 0.9
    cam.location = (bx - dist * 0.55, by - dist * 0.75, dist * 0.55)
    direction = (bx - cam.location[0], by - cam.location[1], 0.0 - cam.location[2])
    import mathutils
    cam.rotation_euler = mathutils.Vector(direction).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    sun = bpy.data.lights.new("sun2", type="SUN")
    sun.energy = 3.0
    sun_obj = bpy.data.objects.new("sun2", sun)
    sun_obj.location = (bx, by, 500.0)
    sun_obj.rotation_euler = (0.5, 0.2, 0.4)
    bpy.context.scene.collection.objects.link(sun_obj)


def render(path, size=900):
    scene = bpy.context.scene
    scene.render.resolution_x = size
    scene.render.resolution_y = size
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = path
    scene.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in [
        e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items
    ] else "BLENDER_EEVEE"
    bpy.ops.render.render(write_still=True)


def main():
    os.makedirs(OUT, exist_ok=True)
    clear_scene()
    import_glb()

    frame_top(CLUSTER)
    render(os.path.join(OUT, "track_top.png"))
    for o in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
        bpy.data.objects.remove(o, do_unlink=True)

    frame_persp(CLUSTER)
    render(os.path.join(OUT, "track_persp.png"))
    for o in [o for o in bpy.data.objects if o.type in ("CAMERA", "LIGHT")]:
        bpy.data.objects.remove(o, do_unlink=True)

    frame_top(RAMPS_AREA, margin=1.05)
    render(os.path.join(OUT, "track_ramps.png"))

    print("trackgen blender preview: done")


if __name__ == "__main__":
    main()
