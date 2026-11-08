# Enclosure part sources

Parametric CSG builds (manifold3d) of the lids and bases in `../`. All dimensions are in the
lid/base frames of the `Scene` scene in `../LuckFox Lid.blend`.

| Script | Output | Part |
|---|---|---|
| `build_lid.py` | `lid_std_csg.json`, `lid_plain_csg.json` | lid with camera-holder holes; plain lid |
| `build_lid_cm3.py` | `lid_cm3w_csg.json`, `lid_cm3w_tcs_csg.json` | slotless lid with Camera Module 3 Wide mounts; same plus a TCS34725 colour sensor (needs the tall base) |
| `build_base.py` | `base_std_csg.json`, `base_tripod_csg.json`, `base_tall_csg.json` | standard, tripod and tall (+8 mm, `DZ`) bases |
| `build_plug_antenna.py` | `plug_antenna_csg.json` | plug for the +X antenna opening (all bases) |

`plan.json` and `wall_sections.json` are inputs (lid outline, base wall and antenna-opening
sections). `to_stl.py [outdir]` turns the JSON meshes into print-oriented STLs.

```sh
python3 -m venv venv && venv/bin/pip install numpy manifold3d mapbox_earcut
cd src && ../venv/bin/python build_lid.py && ../venv/bin/python build_lid_cm3.py \
  && ../venv/bin/python build_base.py \
  && ../venv/bin/python build_plug_antenna.py && ../venv/bin/python to_stl.py /tmp/stl
```

The committed STLs also had a few sub-micron slivers merged in Blender after the build; a fresh
build may differ from them by a handful of triangles.
