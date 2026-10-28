# Slotless lid with inner mounts for Raspberry Pi Camera Module 3 Wide (lens out through the top).
import json, math, numpy as np
from build_lid import *

# camera placement (lid build frame); board rotated so its FPC edge faces +Y
CX=14.86; YEDGE=6.461            # lens x; y of FPC (connector) edge
PCB_Z=-1.9                       # PCB front face; lens front = PCB_Z+8.3 = 6.4
def lp(u,v): return (CX+(12.5-u), YEDGE-v)    # board-local (from left edge, from FPC edge) -> lid
HOLES=[lp(u,v) for u in (2,23) for v in (2,14.5)]
LENS=lp(12.5,14.4)
R_SO=2.2; R_PILOT=0.9; ZPILOT=4.4; CHAM=0.3
POCKET=6.0; ZPOCKET=3.2          # 12 x 12 mm housing recess up to z 3.2
R_LENS=4.0                       # dia 8.0 lens hole (barrel dia 6.95)

def cyl(cx,cy,r,z0,z1,seg=96): return loft([(z0,circle(cx,cy,r,seg)),(z1,circle(cx,cy,r,seg))])

def build_cm3(plan):
    solid=plate(plan)
    for args in ((-13.035,-7.24,-38.423,-32.63,'--'),(31.76,37.565,-38.423,-32.63,'+-'),
                 (-13.035,-7.24,6.69,12.17,'-+'),(31.76,37.565,6.69,12.17,'++')):
        solid=solid+post(*args)
    for b in ((-7.24,31.76,10.17,12.17),(17.981,31.76,-38.423,-36.423),(-13.035,-11.04,-32.63,-26.93)):
        solid=solid+box(*b,-1.0,2.0)
    for c in ((-16.0375,-13.13),(43.6725,-13.13)): solid=solid+pin(*c)
    for h in HOLES: solid=solid+cyl(*h,R_SO,PCB_Z,2.0)
    cut=[]
    for c in ((46.265,10.27),(46.265,-36.53),(-16.538,10.27),(-16.538,-36.53)):
        cut.append(hole_cutter(lambda d,c=c: circle(c[0],c[1],1.0+d)))
    cut.append(hole_cutter(lambda d: circle(*LENS,R_LENS+d,128)))
    x,y=LENS; cut.append(box(x-POCKET,x+POCKET,y-POCKET,y+POCKET,-1.0,ZPOCKET))
    for h in HOLES:
        cut.append(loft([(PCB_Z-0.5,circle(*h,R_PILOT+CHAM+0.5,48)),(PCB_Z+CHAM,circle(*h,R_PILOT,48)),(ZPILOT,circle(*h,R_PILOT,48))]))
    for c in cut: solid=solid-c
    return solid

if __name__=='__main__':
    m=build_cm3(json.load(open('plan.json'))); mesh=m.to_mesh()
    V=np.array(mesh.vert_properties)[:,:3].astype(float); T=np.array(mesh.tri_verts)
    print('status',m.status().name,'genus',m.genus(),'tris',len(T),'volume',round(m.volume(),3),'holes',np.round(HOLES,3).tolist(),'lens',np.round(LENS,3).tolist())
    json.dump({'v':V.tolist(),'f':T.tolist()},open('lid_cm3w_csg.json','w'))
