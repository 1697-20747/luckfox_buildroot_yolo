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

# ---------------- TCS34725 colour sensor (Adafruit 1334 layout; DollaTek clone) ----------------
# Board 20.32 square, set into a pocket in the lid underside so it sits flush with it, component side up.
# Board-local (u right, v up, pins at the bottom, holes at the top, origin bottom-left, from Adafruit's print):
#   chip (10.414, 9.906), LED (7.112, 9.906), holes (2.54, 17.78) & (17.78, 17.78), pins v=1.524.
# Placed in the free strip between the camera and the -Y wall with the pin edge facing -X, so a right-angle
# header on the back sends the connector along the underside towards -X (channel below). Needs the tall base.
TB=20.32; T_XE=9.74; T_YL=-15.64        # lid x of the pin edge, lid y of the board's u=0 edge
def tp(u,v): return (T_XE+v, T_YL-u)    # board-local -> lid (v -> +x, u -> -y)
T_CHIP=tp(10.414,9.906); T_LED=tp(7.112,9.906); T_HOLES=[tp(2.54,17.78),tp(17.78,17.78)]
T_ZPCB=3.1                              # PCB front (component) face; back face flush with the lid underside
T_CLR=0.2                               # board pocket clearance per side
T_RELIEF=1.5                            # general component relief above the PCB
T_PINREL=2.6                            # relief over the header pin tips poking through the front
T_PILOT_TOP=6.4                         # M2 self-tap pilots (M2x5): 3.3 mm engagement, 0.6 mm skin
T_AP_CHIP=1.2; T_AP_LED=0.9             # aperture radii (dia 2.4 sensor window, dia 1.8 LED window)
T_RO=0.25                               # small round-over so the wall between the two windows survives
def rect(x0,x1,y0,y1): return [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
def bxy(x0,x1,y0,y1,z0,z1):
    lo_x,hi_x=min(x0,x1),max(x0,x1); lo_y,hi_y=min(y0,y1),max(y0,y1)
    return box(lo_x,hi_x,lo_y,hi_y,z0,z1)
def window(cx,cy,r,zlow,seg=96):
    rings=[(zlow,circle(cx,cy,r,seg)),(ZT-T_RO,circle(cx,cy,r,seg))]
    for k in range(1,9):
        th=math.pi/2*k/8; rings.append((ZT-T_RO+T_RO*math.sin(th),circle(cx,cy,r+T_RO*(1-math.cos(th)),seg)))
    rings.append((ZT+0.6,circle(cx,cy,r+T_RO,seg)))
    return loft(rings)
def tcs_cuts():
    x0,x1=T_XE-T_CLR,T_XE+TB+T_CLR; y1,y0=T_YL+T_CLR,T_YL-TB-T_CLR
    cuts=[box(x0,x1,y0,y1,-1.0,T_ZPCB)]                                   # board pocket
    # component relief (inset 0.4 from the board edge), keeping ledges round the holes and a baffle
    # block round the chip and LED that comes down onto the PCB
    rel=box(T_XE+0.4,T_XE+TB-0.4,T_YL-TB+0.4,T_YL-0.4,T_ZPCB-0.1,T_ZPCB+T_RELIEF)
    for h in T_HOLES: rel=rel-loft([(T_ZPCB-1,circle(*h,2.6,48)),(T_ZPCB+T_RELIEF+1,circle(*h,2.6,48))])
    cx,cy=T_CHIP; lx,ly=T_LED
    rel=rel-bxy(cx-1.6,cx+1.6,cy-1.6,ly+1.6,T_ZPCB-1,T_ZPCB+T_RELIEF+1)   # baffle round chip + LED
    cuts.append(rel)
    pu=[tp(2.54*k,1.524) for k in (1,7)]                                  # pin row ends
    cuts.append(bxy(T_XE+0.4,T_XE+3.0,pu[0][1]+1.6,pu[1][1]-1.6,T_ZPCB-0.1,T_ZPCB+T_PINREL))
    # chip and LED: body pockets in the baffle, then separate windows through the top
    cuts.append(bxy(cx-1.5,cx+1.5,cy-1.3,cy+1.3,T_ZPCB-0.1,T_ZPCB+0.9))   # TCS34725 2.0x2.4 body (+clearance)
    cuts.append(bxy(lx-1.3,lx+1.3,ly-1.0,ly+1.0,T_ZPCB-0.1,T_ZPCB+1.2))   # LED body
    cuts.append(window(cx,cy,T_AP_CHIP,T_ZPCB+0.5))
    cuts.append(window(lx,ly,T_AP_LED,T_ZPCB+0.5))
    for h in T_HOLES:                                                      # M2 self-tap pilots
        cuts.append(loft([(T_ZPCB-0.3,circle(*h,R_PILOT+0.3,48)),(T_ZPCB,circle(*h,R_PILOT,48)),(T_PILOT_TOP,circle(*h,R_PILOT,48))]))
    # connector channel: right-angle header/Dupont housings run -X from the pin edge under the lid
    cuts.append(box(-6.5,T_XE+0.5,T_YL-TB/2-9.65,T_YL-TB/2+9.65,-1.0,2.5))
    return cuts

def build_cm3(plan,tcs=False):
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
    if tcs: cut+=tcs_cuts()
    for c in cut: solid=solid-c
    return solid

if __name__=='__main__':
    plan=json.load(open('plan.json'))
    for name,tcs in (('lid_cm3w',False),('lid_cm3w_tcs',True)):
        m=build_cm3(plan,tcs); mesh=m.to_mesh()
        V=np.array(mesh.vert_properties)[:,:3].astype(float); T=np.array(mesh.tri_verts)
        print(name,'status',m.status().name,'genus',m.genus(),'tris',len(T),'volume',round(m.volume(),3),'holes',np.round(HOLES,3).tolist(),'lens',np.round(LENS,3).tolist())
        if tcs: print('  tcs chip',np.round(T_CHIP,3).tolist(),'led',np.round(T_LED,3).tolist(),'holes',np.round(T_HOLES,3).tolist())
        json.dump({'v':V.tolist(),'f':T.tolist()},open(name+'_csg.json','w'))
