# Convert the *_csg.json meshes written by the build scripts into print-oriented binary STLs.
# Lids: flipped top-face-down (y and z mirrored); bases: upright. All centred on XY, bed face at z 0.
import json, struct, sys, numpy as np

PARTS = {  # csg json -> (STL name, flip)
    'lid_std_csg.json':    ('LuckFox Lid.stl', True),
    'lid_plain_csg.json':  ('LuckFox Lid_plain.stl', True),
    'lid_cm3w_csg.json':   ('LuckFox Lid_cm3_wide.stl', True),
    'base_std_csg.json':   ('LuckFox Box.stl', False),
    'base_tripod_csg.json':('LuckFox Box tripod.stl', False),
}

def write(src, dst, flip):
    d = json.load(open(src)); V = np.array(d['v'], float); F = np.array(d['f'])
    if flip: V *= (1, -1, -1)   # two mirrored axes = 180 deg turn about X, winding unchanged
    lo, hi = V.min(0), V.max(0); V -= ((lo[0]+hi[0])/2, (lo[1]+hi[1])/2, lo[2])
    with open(dst, 'wb') as f:
        f.write(dst.encode()[:80].ljust(80, b' ')); f.write(struct.pack('<I', len(F)))
        for a, b, c in V[F]:
            n = np.cross(b-a, c-a); n /= max(np.linalg.norm(n), 1e-30)
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))
    print(dst, len(F), 'tris')

if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else '.'
    for src, (name, flip) in PARTS.items(): write(src, f'{out}/{name}', flip)
