# Assembled render of the LuckFox box with the SC3336 camera in place, from a
# COPY of drawings/LuckFox BOX/LuckFox Lid.blend (the design file is never
# modified). Run headless:
#
#   /Applications/Blender.app/Contents/MacOS/Blender -b <copy of LuckFox Lid.blend> \
#       --python tools/render_assembly.py -- <output dir>
#
# The parts are the 2026-10-09 versions (the same meshes as the STL export).
# In the design file they are laid out for printing. The assembly here
# (2026-10-13) is derived from the geometry, not guessed:
#   - lid, hinge base (holder A), post v3 and pan arm v2 already stack, with
#     the M3 bolts at the joints;
#   - box: same orientation, moved under the lid: its 21 mm rim meets the lid
#     ledge (1.5 mm), centred on the lid. The 180-degree alternative puts 6x
#     more lid-lip vertices inside the box walls;
#   - camera group (plates C + D + the SC3336 model): rotated -90 degrees
#     about Z so the lens faces -Y, then moved so plate C's clevis hole
#     (-53.77, 19.0, 77.9) lies on the tilt bolt axis (y -23.15, z 47.55),
#     with the clevis gap centred on the pan arm's 12.2 mm ear (x 16.68).
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

out_dir = sys.argv[sys.argv.index("--") + 1] if "--" in sys.argv else "."

SHOW = {
    "Luckfox_Ultra_case_body": (0.10, 0.11, 0.12),
    "Luckfox_Ultra_case_lid": (0.92, 0.92, 0.92),
    "RaspiCam_Holder_A_prejoin": (0.92, 0.92, 0.92),
    "RaspiCam_Holder_B_post_v3": (0.95, 0.45, 0.20),
    "RaspiCam_Holder_B_panarm_v2": (0.95, 0.45, 0.20),
    "RaspiCam_Holder_C.": (0.25, 0.45, 0.80),
    "RaspiCam_Holder_D.": (0.25, 0.65, 0.40),
}
HARDWARE = ["HW_M3x25_A_hinge", "HW_M3_nut_A_hinge", "HW_M3x20_pan", "HW_M3_nut_pan",
            "HW_M3x30_C_hinge", "HW_M3_nyloc_C_hinge"]
CAM_EMPTY = "SC3336_Cam_V2"

# World matrices of the shown parts in the 2026-10-09 state (the STL export's
# positions; read from the open design session 2026-10-13). The saved .blend
# has plate D elsewhere, so they are applied explicitly.
REFERENCE = {
    "Luckfox_Ultra_case_body": [1.0, 0.0, 0.0, -46.95731, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "Luckfox_Ultra_case_lid": [-1.0, -0.0, 0.0, 64.76475, 0.0, -1.0, 0.0, -28.1299, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "RaspiCam_Holder_A_prejoin": [0.0, -0.0, 1.0, -61.21711, 0.0, -1.0, -0.0, 5.86615, 1.0, 0.0, -0.0, 85.21352, 0.0, 0.0, 0.0, 1.0],
    "RaspiCam_Holder_B_post_v3": [0.0, -1.0, 0.0, 16.683, 1.0, 0.0, 0.0, -13.13, 0.0, 0.0, 1.0, 20.57, 0.0, 0.0, 0.0, 1.0],
    "RaspiCam_Holder_B_panarm_v2": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77985, 1.0, -0.0, -0.0, 95.21352, 0.0, 0.0, 0.0, 1.0],
    "RaspiCam_Holder_C.": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "RaspiCam_Holder_D.": [1.0, 0.0, 0.0, 0.13071, 0.0, 1.0, 0.0, 0.0015, 0.0, 0.0, 1.0, 0.00691, 0.0, 0.0, 0.0, 1.0],
    "SC3336_Cam_V2": [-0.0, -1.0, 0.0, -39.353, 1.0, -0.0, 0.0, 19.0, 0.0, 0.0, 1.0, 81.05, 0.0, 0.0, 0.0, 1.0],
    "HW_M3x25_A_hinge": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77986, 1.0, -0.0, -0.0, 90.21352, 0.0, 0.0, 0.0, 1.0],
    "HW_M3_nut_A_hinge": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77986, 1.0, -0.0, -0.0, 90.21352, 0.0, 0.0, 0.0, 1.0],
    "HW_M3x20_pan": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77986, 1.0, -0.0, -0.0, 95.21352, 0.0, 0.0, 0.0, 1.0],
    "HW_M3_nut_pan": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77986, 1.0, -0.0, -0.0, 94.59499, 0.0, 0.0, 0.0, 1.0],
    "HW_M3x30_C_hinge": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77986, 1.0, -0.0, -0.0, 95.21352, 0.0, 0.0, 0.0, 1.0],
    "HW_M3_nyloc_C_hinge": [0.0, 1.0, 0.0, 8.93689, 0.0, -0.0, 1.0, -89.77986, 1.0, -0.0, -0.0, 95.21352, 0.0, 0.0, 0.0, 1.0],
}


scene = bpy.context.scene
# Some parts are keyframed (exploded/assembled animation). Positions above
# were measured at the file's saved frame (237): evaluate that frame, then
# freeze the shown parts there by dropping their keyframes, so the moves
# below aren't overwritten when the render re-evaluates the animation.
scene.frame_set(scene.frame_current)
cam_parts = [o.name for o in bpy.data.objects if o.parent and o.parent.name == CAM_EMPTY]
visible = set(SHOW) | set(HARDWARE) | {CAM_EMPTY} | set(cam_parts)
for o in bpy.data.objects:
    on = o.name in visible
    o.hide_render = not on
    o.hide_viewport = not on
    o.hide_set(not on)

for name in visible:
    o = bpy.data.objects.get(name)
    if o and o.animation_data:
        mw = o.matrix_world.copy()
        o.animation_data_clear()
        o.matrix_world = mw

for name, m in REFERENCE.items():
    bpy.data.objects[name].matrix_world = Matrix([m[0:4], m[4:8], m[8:12], m[12:16]])
bpy.context.view_layer.update()

# materials for the printed parts
for name, rgb in SHOW.items():
    o = bpy.data.objects[name]
    m = bpy.data.materials.new(name + "_render")
    m.use_nodes = True
    bsdf = next(n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.55
    o.data.materials.clear()
    o.data.materials.append(m)

# box under the lid
body = bpy.data.objects["Luckfox_Ultra_case_body"]
body.matrix_world = Matrix.Translation(Vector((81.5, 2.4, -19.5))) @ body.matrix_world

# camera group onto the tilt joint
mount = Matrix.Translation(Vector((-2.32, -76.92, -30.35))) @ Matrix.Rotation(math.radians(-90), 4, "Z")
for name in ("RaspiCam_Holder_C.", "RaspiCam_Holder_D.", CAM_EMPTY):
    o = bpy.data.objects[name]
    o.matrix_world = mount @ o.matrix_world
bpy.context.view_layer.update()

# check: C's clevis hole now on the tilt bolt axis
hole = mount @ Vector((-53.77, 19.0, 77.9))
pcb = bpy.data.objects.get("SC3336_PCB")
if pcb:
    ws = [pcb.matrix_world @ Vector(c) for c in pcb.bound_box]
    print("camera PCB after mounting, min:", tuple(round(min(v[i] for v in ws), 1) for i in range(3)))
d = bpy.data.objects["RaspiCam_Holder_D."]
ws = [d.matrix_world @ Vector(c) for c in d.bound_box]
print("plate D after mounting, min:", tuple(round(min(v[i] for v in ws), 1) for i in range(3)))
print("clevis hole after mounting:", tuple(round(v, 2) for v in hole), "(bolt axis y -23.15, z 47.55)")

# framing: bounding box of everything shown
pts = []
for o in bpy.data.objects:
    if o.name in visible and o.type == "MESH":
        pts += [o.matrix_world @ Vector(c) for c in o.bound_box]
mn = Vector([min(p[i] for p in pts) for i in range(3)])
mx = Vector([max(p[i] for p in pts) for i in range(3)])
centre = (mn + mx) / 2
size = max(mx - mn)
print("assembly size (mm):", tuple(round(v, 1) for v in (mx - mn)))

# camera: three-quarter view from the front-left, slightly above (like the
# 2026-10-04 renders), looking at the centre
cam_data = bpy.data.cameras.new("render_cam")
cam_data.lens = 85
cam_data.clip_start = 1
cam_data.clip_end = 100000
cam = bpy.data.objects.new("render_cam", cam_data)
scene.collection.objects.link(cam)
direction = Vector((-0.55, -1.0, 0.75)).normalized()
cam.location = centre + direction * size * 3.3
cam.rotation_euler = (centre - cam.location).to_track_quat("-Z", "Y").to_euler()
scene.camera = cam

# lighting: soft area key + fill, light grey world
for loc, energy, sz in (((-1.0, -1.2, 1.6), 3.0e6, 300), ((1.4, -0.4, 0.9), 1.0e6, 300)):
    ld = bpy.data.lights.new("area", "AREA")
    ld.energy = energy
    ld.size = sz
    lo = bpy.data.objects.new("area", ld)
    lo.location = centre + Vector(loc) * size * 3
    lo.rotation_euler = (centre - lo.location).to_track_quat("-Z", "Y").to_euler()
    scene.collection.objects.link(lo)
world = scene.world or bpy.data.worlds.new("World")
scene.world = world
world.use_nodes = True
bg = next(n for n in world.node_tree.nodes if n.type == "BACKGROUND")
bg.inputs["Color"].default_value = (0.65, 0.65, 0.66, 1.0)
bg.inputs["Strength"].default_value = 0.8

r = scene.render
try:
    r.engine = "CYCLES"
    scene.cycles.samples = 128
    scene.cycles.use_denoising = True
    scene.cycles.device = "CPU"
except TypeError as e:
    print("Cycles unavailable:", e)
r.resolution_x = r.resolution_y = 1600
r.resolution_percentage = 100
r.image_settings.file_format = "PNG"
try:
    scene.view_settings.view_transform = "Standard"
except TypeError:
    pass

r.film_transparent = False
r.filepath = os.path.join(out_dir, "LuckFox Box assembled view.png")
bpy.ops.render.render(write_still=True)
r.film_transparent = True
r.image_settings.color_mode = "RGBA"
r.filepath = os.path.join(out_dir, "LuckFox Box assembled view (transparent).png")
bpy.ops.render.render(write_still=True)
print("rendered to", out_dir)
