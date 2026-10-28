import numpy as np, json, sys, math
import mapbox_earcut as earcut
from manifold3d import Manifold, Mesh

def loft(rings):
    """rings: list of (z, Nx2 CCW polygon) all same N, bottom->top. returns Manifold"""
    N=len(rings[0][1]); V=[]; T=[]
    for z,P in rings:
        for x,y in P: V.append((x,y,z))
    V=np.array(V)
    for r in range(len(rings)-1):
        a=r*N; b=(r+1)*N
        for i in range(N):
            j=(i+1)%N
            T.append((a+i,a+j,b+j)); T.append((a+i,b+j,b+i))
    def cap(off,P,top):
        tri=earcut.triangulate_float64(np.array(P,float),np.array([len(P)],np.uint32)).reshape(-1,3)
        for t in tri:
            t=[off+int(k) for k in t]
            T.append(tuple(t) if top else (t[0],t[2],t[1]))
    cap(0,rings[0][1],False); cap((len(rings)-1)*N,rings[-1][1],True)
    m=Manifold(Mesh(vert_properties=np.array(V,np.float32),tri_verts=np.array(T,np.uint32)))
    assert m.status().name=='NoError', m.status()
    return m

def ccw(P):
    a=sum(P[i][0]*P[(i+1)%len(P)][1]-P[(i+1)%len(P)][0]*P[i][1] for i in range(len(P)))
    return P if a>0 else P[::-1]

def offset_miter(P,d):
    """offset CCW polygon outward by d (negative=inward), keeping vertex count"""
    P=np.array(P,float); n=len(P); out=[]
    for i in range(n):
        a,b,c=P[i-1],P[i],P[(i+1)%n]
        e1=b-a; e2=c-b
        n1=np.array([e1[1],-e1[0]])/np.linalg.norm(e1); n2=np.array([e2[1],-e2[0]])/np.linalg.norm(e2)
        m=n1+n2; m/=np.linalg.norm(m); s=d/max(1e-9,m@n1)
        out.append(b+m*s)
    return out

def rrect(x0,x1,y0,y1,r,seg=16):
    pts=[]
    for cx,cy,a0 in ((x1-r,y0+r,-90),(x1-r,y1-r,0),(x0+r,y1-r,90),(x0+r,y0+r,180)):
        for k in range(seg+1):
            a=math.radians(a0+90*k/seg); pts.append((cx+r*math.cos(a),cy+r*math.sin(a)))
    return pts

def circle(cx,cy,r,seg=96):
    return [(cx+r*math.cos(2*math.pi*k/seg),cy+r*math.sin(2*math.pi*k/seg)) for k in range(seg)]

RO=0.5; ZB=1.5; ZRO=6.5; ZT=7.0; NRO=12
def roundover_profile():
    return [(ZRO+RO*math.sin(math.pi/2*k/NRO), RO*(1-math.cos(math.pi/2*k/NRO))) for k in range(NRO+1)]

X0,X1,Y0,Y1,CH,FR=-20.038,49.765,-40.63,14.37,1.4295,0.5
def fillet_poly(P,R,seg=8):
    P=np.array(P,float); n=len(P); out=[]
    for i in range(n):
        a,b,c=P[i-1],P[i],P[(i+1)%n]
        u=(a-b)/np.linalg.norm(a-b); v=(c-b)/np.linalg.norm(c-b)
        ang=math.acos(max(-1,min(1,u@v))); t=R/math.tan(ang/2)
        p1=b+u*t; p2=b+v*t
        bis=(u+v)/np.linalg.norm(u+v); ctr=b+bis*(R/math.sin(ang/2))
        a1=math.atan2(*(p1-ctr)[::-1]); a2=math.atan2(*(p2-ctr)[::-1])
        da=(a2-a1+math.pi)%(2*math.pi)-math.pi
        for k in range(seg+1): out.append(tuple(ctr+R*np.array([math.cos(a1+da*k/seg),math.sin(a1+da*k/seg)])))
    return out
def plan_outline(d):
    x0,x1,y0,y1=X0+d,X1-d,Y0+d,Y1-d; c=CH-d*(2-math.sqrt(2))   # inward offset of 45deg chamfer
    oct_=[(x0+c,y0),(x1-c,y0),(x1,y0+c),(x1,y1-c),(x1-c,y1),(x0+c,y1),(x0,y1-c),(x0,y0+c)]
    return fillet_poly(oct_,max(FR-d,0.002))
def plate(plan=None):
    rings=[(ZB,ccw(plan_outline(0)))]
    for z,d in roundover_profile(): rings.append((z,ccw(plan_outline(d))))
    return loft(rings)

def hole_cutter(gen, zlow=-5.0, ztop=7.6):
    """gen(delta)->CCW outline enlarged by delta"""
    rings=[(zlow,ccw(gen(0))),(ZRO,ccw(gen(0)))]
    for z,d in roundover_profile()[1:]: rings.append((z,ccw(gen(d))))
    rings.append((ztop,ccw(gen(RO))))
    return loft(rings)

def post(x0,x1,y0,y1,corner,r=0.55,ztop=2.0,ztaper=-1.2,zbot=-3.6296,slope=0.1244,seg=12):
    """corner: which corner is the outer one ('++','+-','-+','--' for x,y). outer faces taper."""
    sx= 1 if corner[0]=='+' else -1; sy= 1 if corner[1]=='+' else -1
    def outline(t):
        X0,X1,Y0,Y1=x0,x1,y0,y1
        if sx>0: X1-=t
        else: X0+=t
        if sy>0: Y1-=t
        else: Y0+=t
        cx=X1-r if sx>0 else X0+r; cy=Y1-r if sy>0 else Y0+r
        a0={(1,1):0,(-1,1):90,(-1,-1):180,(1,-1):270}[(sx,sy)]
        arc=[(cx+r*math.cos(math.radians(a0+90*k/seg)),cy+r*math.sin(math.radians(a0+90*k/seg))) for k in range(seg+1)]
        # assemble CCW: start after arc, go around rectangle corners
        corners={(1,-1):(X1,Y0),(1,1):(X1,Y1),(-1,1):(X0,Y1),(-1,-1):(X0,Y0)}
        order=[(1,-1),(1,1),(-1,1),(-1,-1)]
        pts=[]
        for c in order:
            if c==(sx,sy): pts+=arc
            else: pts.append(corners[c])
        return pts
    t_b=(ztaper-zbot)*slope
    return loft([(zbot,ccw(outline(t_b))),(ztaper,ccw(outline(0))),(ztop,ccw(outline(0)))])

def box(x0,x1,y0,y1,z0,z1):
    return loft([(z0,[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]),(z1,[(x0,y0),(x1,y0),(x1,y1),(x0,y1)])])

def pin(cx,cy,r=1.5,ztop=2.0,zch=-1.1,ztip=-1.5,seg=96):
    rt=r-(zch-ztip)
    return loft([(ztip,circle(cx,cy,rt,seg)),(zch,circle(cx,cy,r,seg)),(ztop,circle(cx,cy,r,seg))])

def build(plan, camera_holes):
    solid=plate(plan)
    XL,XR=-13.035,37.565; YN,YP=-38.43,12.17
    for args in ((-13.035,-7.24,-38.423,-32.63,'--'),(31.76,37.565,-38.423,-32.63,'+-'),
                 (-13.035,-7.24,6.69,12.17,'-+'),(31.76,37.565,6.69,12.17,'++')):
        solid=solid+post(*args)
    for b in ((-7.24,31.76,10.17,12.17),(17.981,31.76,-38.423,-36.423),(-13.035,-11.04,-32.63,-26.93)):
        solid=solid+box(*b,-1.0,2.0)
    for c in ((-16.0375,-13.13),(43.6725,-13.13)): solid=solid+pin(*c)
    cut=[]
    for (x0,x1,y0,y1) in ((-13.02,-6.94,-26.93,6.69),(31.48,37.56,-26.93,6.69),(2.567,5.567,-13.58,4.14)):
        cut.append(hole_cutter(lambda d,x0=x0,x1=x1,y0=y0,y1=y1: rrect(x0-d,x1+d,y0-d,y1+d,1.0+d)))
    for c in ((46.265,10.27),(46.265,-36.53),(-16.538,10.27),(-16.538,-36.53)):
        cut.append(hole_cutter(lambda d,c=c: circle(c[0],c[1],1.0+d)))
    if camera_holes:
        for c in ((7.683,-21.38),(25.683,-21.38),(16.683,-3.88)): cut.append(hole_cutter(lambda d,c=c: circle(c[0],c[1],2.1+d)))
        for c in ((25.683,-4.38),(16.683,-21.38),(7.683,-4.38)): cut.append(hole_cutter(lambda d,c=c: circle(c[0],c[1],1.0+d)))
    for c in cut: solid=solid-c
    return solid

if __name__=='__main__':
    plan=json.load(open('plan.json'))
    for name,cam in (('lid_std',True),('lid_plain',False)):
        m=build(plan,cam); mesh=m.to_mesh()
        V=np.array(mesh.vert_properties)[:,:3].astype(float); T=np.array(mesh.tri_verts)
        print(name,'status',m.status().name,'genus',m.genus(),'verts',len(V),'tris',len(T),'volume',round(m.volume(),3))
        json.dump({'v':V.tolist(),'f':T.tolist()},open(name+'_csg.json','w'))
