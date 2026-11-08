# Antenna port plug: blanks the +X antenna opening of every base when no antenna is fitted.
# Same profile as the lid slot plugs: stem 0.1 mm under the opening per side with a 0.4 mm lead-in
# chamfer, then a cap with a 1 mm lip, 1 mm thick, and a 0.5 mm quarter round-over on its outer edge.
# Built in the base frame (Scene world): the cap seats on the +X recess floor (x = X1-1).
import numpy as np, json, math
from manifold3d import CrossSection, JoinType
from build_lid import loft
from build_base import xform, X1

CLR, LIP, CAP, RO, CH = 0.1, 1.0, 1.0, 0.5, 0.4
XF = X1 - 1.0              # recess floor, outer face of the wall at the opening
X_IN = -38.957             # inner face of the wall (antenna pocket)
L = XF - X_IN - 0.1        # stem stops 0.1 short of the inner face, like the lid plugs
NA = 720                   # samples per ring (polar, about the opening centre)

def antenna_profile():
    d = json.load(open('wall_sections.json'))
    return sorted(d['+X'], key=lambda P: abs(sum(P[i][0]*P[(i+1)%len(P)][1]-P[(i+1)%len(P)][0]*P[i][1]
                                                 for i in range(len(P)))))[0]

def ring(cs, c, delta):
    """offset the (u,v) cross-section by delta (round joins) and resample it on NA rays from c"""
    polys = (cs.offset(delta, JoinType.Round, circular_segments=128) if delta else cs).to_polygons()
    P = max(polys, key=len); P = np.array(P, float); Q = np.roll(P, -1, 0)
    out = []
    for k in range(NA):
        t = 2*math.pi*k/NA; dvec = np.array([math.cos(t), math.sin(t)])
        best = None
        for a, b in zip(P, Q):   # ray c + s*dvec against segment a..b
            e = b - a; M = np.array([dvec, -e]).T
            if abs(np.linalg.det(M)) < 1e-12: continue
            s, w = np.linalg.solve(M, a - c)
            if s > 0 and -1e-9 <= w <= 1+1e-9 and (best is None or s > best): best = s
        out.append(tuple(c + best*dvec))
    return out

def build():
    A = antenna_profile(); cs = CrossSection([A])
    lo, hi = np.min(A, 0), np.max(A, 0); c = (lo + hi)/2
    # local frame: u=y, v=z, w=x (outward); w measured from the recess floor
    stem = loft([(-L, ring(cs, c, -CLR-CH)), (-L+CH, ring(cs, c, -CLR)), (0.01, ring(cs, c, -CLR))])   # 0.01 into the cap to fuse
    rings = [(0.0, ring(cs, c, LIP)), (CAP, ring(cs, c, LIP))]
    for k in range(1, 9):
        th = math.pi/2*k/8
        rings.append((CAP + RO*math.sin(th), ring(cs, c, LIP - RO*(1-math.cos(th)))))
    cap = loft(rings)
    return xform(stem + cap, [(0, 1, 0), (0, 0, 1), (1, 0, 0)], (XF, 0, 0))

if __name__ == '__main__':
    m = build(); mesh = m.to_mesh()
    V = np.array(mesh.vert_properties)[:, :3].astype(float); T = np.array(mesh.tri_verts)
    print('plug_antenna status', m.status().name, 'genus', m.genus(), 'tris', len(T), 'volume', round(m.volume(), 3),
          'bbox', V.min(0).round(3), V.max(0).round(3))
    json.dump({'v': V.tolist(), 'f': T.tolist()}, open('plug_antenna_csg.json', 'w'))
