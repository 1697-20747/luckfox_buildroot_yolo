import numpy as np, json, math, sys
import mapbox_earcut as earcut
from manifold3d import Manifold, Mesh
from build_lid import loft, ccw, offset_miter, rrect, circle, fillet_poly

EPS=0.002
def fillet_var(P,radii,seg=8):
    """fillet each corner of sharp polygon P with its own radius"""
    P=np.array(P,float); n=len(P); out=[]
    for i in range(n):
        a,b,c=P[i-1],P[i],P[(i+1)%n]; R=radii[i]
        u=(a-b)/np.linalg.norm(a-b); v=(c-b)/np.linalg.norm(c-b)
        ang=math.acos(max(-1,min(1,u@v))); t=R/math.tan(ang/2)
        p1=b+u*t; p2=b+v*t
        bis=(u+v)/np.linalg.norm(u+v); ctr=b+bis*(R/math.sin(ang/2))
        a1=math.atan2(*(p1-ctr)[::-1]); a2=math.atan2(*(p2-ctr)[::-1])
        da=(a2-a1+math.pi)%(2*math.pi)-math.pi
        for k in range(seg+1): out.append(tuple(ctr+R*np.array([math.cos(a1+da*k/seg),math.sin(a1+da*k/seg)])))
    return out
def convex_flags(P):
    P=np.array(P); n=len(P); f=[]
    for i in range(n):
        a,b,c=P[i-1],P[i],P[(i+1)%n]
        f.append(((b-a)[0]*(c-b)[1]-(b-a)[1]*(c-b)[0])>0)
    return f
def offset_rounded(P,o):
    """P sharp CCW polygon; offset by o (outward +) and round the corners like a true offset"""
    Q=offset_miter(P,o) if abs(o)>1e-12 else [tuple(p) for p in P]
    cf=convex_flags(P)
    radii=[(max(o,EPS) if c else max(-o,EPS)) for c in cf]
    return fillet_var(Q,radii)

def xform(m,cols,origin=(0,0,0)):
    """cols: images of local x,y,z axes. builds 3x4 matrix"""
    A=np.array(cols,float).T; T=np.c_[A,np.array(origin,float)]
    return m.transform(T)

# ---------------- dimensions (Scene world frame of the base objects) ----------------
X0,X1,Y0,Y1=-101.76,-31.957,-43.0,12.0; CH=1.429
ZB,ZT=-2.5,21.0; BCH_IN,BCH_TOP=0.7867,0.786
def octagon(x0,x1,y0,y1,c):
    return [(x0+c,y0),(x1-c,y0),(x1,y0+c),(x1,y1-c),(x1-c,y1),(x0+c,y1),(x0,y1-c),(x0,y0+c)]
def shell():
    P=ccw(octagon(X0,X1,Y0,Y1,CH))
    prism=loft([(ZB,P),(ZT,P)])
    R=[(X0,Y0),(X1,Y0),(X1,Y1),(X0,Y1)]; Ri=[(X0+BCH_IN,Y0+BCH_IN),(X1-BCH_IN,Y0+BCH_IN),(X1-BCH_IN,Y1-BCH_IN),(X0+BCH_IN,Y1-BCH_IN)]
    fr=loft([(ZB,Ri),(BCH_TOP,R),(ZT+1,R)])
    return prism^fr

def recess(face):
    # local: x=u (along face), y=z (height), z=depth d (inward)
    if face in ('-X','+X'): u0,u1=-40.49,9.49
    elif face=='+Y': u0,u1=-99.25,-34.467
    else: u0,u1=-62.44,-34.467
    v0,v1=2.02,19.98; W=1.965; C=2.0
    rim=ccw(octagon(u0,u1,v0,v1,C)); flo=ccw(octagon(u0+W,u1-W,v0+W,v1-W,C))
    m=loft([(-1.0,rim),(0.0,rim),(1.0,flo)])
    if face=='-X': return xform(m,[(0,1,0),(0,0,1),(1,0,0)],(X0,0,0))
    if face=='+X': return xform(m,[(0,1,0),(0,0,1),(-1,0,0)],(X1,0,0))
    if face=='-Y': return xform(m,[(1,0,0),(0,0,1),(0,1,0)],(0,Y0,0))
    if face=='+Y': return xform(m,[(1,0,0),(0,0,1),(0,-1,0)],(0,Y1,0))

def box(x0,x1,y0,y1,z0,z1):
    return loft([(z0,[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]),(z1,[(x0,y0),(x1,y0),(x1,y1),(x0,y1)])])
def cyl(cx,cy,r,z0,z1,seg=96): return loft([(z0,circle(cx,cy,r,seg)),(z1,circle(cx,cy,r,seg))])

RO=0.5; NRO=10
def wall_hole(gen, face, w_in, w_face, w_out):
    """gen(delta)->CCW outline (u,v). w along outward normal; round-over of RO at w_face"""
    rings=[(w_in,ccw(gen(0))),(w_face-RO,ccw(gen(0)))]
    for k in range(1,NRO+1):
        th=math.pi/2*k/NRO; rings.append((w_face-RO+RO*math.sin(th),ccw(gen(RO*(1-math.cos(th))))))
    rings.append((w_out,ccw(gen(RO))))
    m=loft(rings)
    if face=='+Y': return xform(m,[(1,0,0),(0,0,1),(0,1,0)])     # u=x, v=z, w=y
    if face=='-Y': return xform(m,[(1,0,0),(0,0,1),(0,-1,0)])    # w=-y
def stadium(cx,cz,L,H,seg=32):
    r=H/2; a=L/2-r
    def g(d):
        R=r+d; pts=[]
        for k in range(seg+1):
            t=-math.pi/2+math.pi*k/seg; pts.append((cx+a+R*math.cos(t),cz+R*math.sin(t)))
        for k in range(seg+1):
            t=math.pi/2+math.pi*k/seg; pts.append((cx-a+R*math.cos(t),cz+R*math.sin(t)))
        return pts
    return g

def foot(cornx,corny,sx,sy,zbot,R_f=1.8,R_e=0.5,L=6.0,T=3.0):
    # sharp L outline: outer corner at (cornx,corny); legs extend by sx,sy (+1/-1) inward
    pts=[(cornx,corny),(cornx+sx*L,corny),(cornx+sx*L,corny+sy*T),(cornx+sx*T,corny+sy*T),(cornx+sx*T,corny+sy*L),(cornx,corny+sy*L)]
    P=ccw(pts)
    def off(z):
        dz=ZB-z; o=0.0
        if dz<=0: o+=R_f
        elif dz<R_f: o+=R_f-math.sqrt(R_f**2-(R_f-dz)**2)
        if z<zbot+R_e: o-=R_e-math.sqrt(max(0,R_e**2-(zbot+R_e-z)**2))
        return o
    zs=[ZB+0.5]+[ZB-R_f*(1-math.cos(math.pi/2*k/12)) for k in range(0,13)]
    zs+= [zbot+R_e-R_e*math.sin(math.pi/2*k/10) for k in range(0,11)]
    zs=sorted(set(round(z,6) for z in zs if z>=zbot-1e-9),reverse=True)
    rings=[(z,ccw(offset_rounded(P,off(z)))) for z in zs[::-1]]
    return loft(rings)

def boss(cx,cy,r=12.0,R_f=2.0,zface=-8.5,ch=0.5,seg=192):
    rings=[(zface,circle(cx,cy,r-ch,seg)),(zface+ch,circle(cx,cy,r,seg)),(ZB-R_f,circle(cx,cy,r,seg))]
    for k in range(1,13):
        th=math.pi/2*k/12; dz=R_f*(1-math.sin(th))   # from ZB-R_f up to ZB
        z=ZB-R_f+R_f*math.sin(th); o=R_f-R_f*math.cos(th)
        rings.append((z,circle(cx,cy,r+o,seg)))
    rings.append((ZB+0.5,circle(cx,cy,r+R_f,seg)))
    return loft(rings)

def thread_cutter(cx,cy,P=1.27,rmin=2.675,rmaj=3.275,wroot=0.26,flank=0.32,z_end=-1.6,z_root0=-7.46,zmouth=-8.5,csk=0.7,N=96):
    core=cyl(cx,cy,rmin,zmouth-1.0,z_end,seg=N)
    cone=loft([(zmouth-0.5,circle(cx,cy,rmin+csk+0.5,N)),(zmouth+csk,circle(cx,cy,rmin,N))])
    # helical groove: trapezoid in (r, dz) swept; root centre z(theta)=z_root0+P*theta/2pi
    rin=rmin-0.08; hw_in=wroot/2+flank*(rmaj-rin)/(rmaj-rmin)
    prof=[(rin,-hw_in),(rmaj,-wroot/2),(rmaj,wroot/2),(rin,hw_in)]
    th0=2*math.pi*((zmouth-1.2)-z_root0)/P; th1=2*math.pi*((z_end+0.8)-z_root0)/P
    steps=int(math.ceil((th1-th0)/(2*math.pi)*N))
    V=[];T=[]
    for s in range(steps+1):
        th=th0+(th1-th0)*s/steps; zc=z_root0+P*th/(2*math.pi); c,sn=math.cos(th),math.sin(th)
        for r,dz in prof: V.append((cx+r*c,cy+r*sn,zc+dz))
    for s in range(steps):
        a=4*s; b=4*(s+1)
        for i in range(4):
            j=(i+1)%4; T.append((a+i,b+i,b+j)); T.append((a+i,b+j,a+j))
    T.append((0,1,2)); T.append((0,2,3)); e=4*steps; T.append((e,e+2,e+1)); T.append((e,e+3,e+2))
    g=Manifold(Mesh(vert_properties=np.array(V,np.float32),tri_verts=np.array(T,np.uint32)))
    if g.status().name!='NoError' or g.volume()<0:
        T=[(a,c,b) for a,b,c in T]
        g=Manifold(Mesh(vert_properties=np.array(V,np.float32),tri_verts=np.array(T,np.uint32)))
    assert g.status().name=='NoError', g.status()
    clip=box(cx-10,cx+10,cy-10,cy+10,zmouth-3,z_end)
    return core+cone+(g^clip)

def build(tripod, antenna_poly):
    s=shell()
    zbot=-9.0 if tripod else -4.5
    for cx,cy,sx,sy in ((-98.76,9.0,1,-1),(-34.957,9.0,-1,-1),(-98.76,-40.0,1,1),(-34.957,-40.0,-1,1)):
        s=s+foot(cx,cy,sx,sy,zbot)
    CX,CY=-66.8585,-15.5
    if tripod: s=s+boss(CX,CY)
    for f in ('-X','+X','-Y','+Y'): s=s-recess(f)
    s=s-box(-94.76,-44.154,-40.803,9.803,3.0,22)
    s=s-box(-44.154,-38.957,-38.938,-20.756,5.0,22)
    s=s-box(-88.747,-63.743,-44,-40.5,5.0,22)
    s=s-box(-88.96,-63.743,-44,-41.998,5.0,22)
    for (cx,cy),(bx0,bx1,by0,by1) in (((-91.957,7.0),(-95.26,-89.957,5.0,10.303)),((-46.957,7.0),(-48.957,-43.654,5.0,10.303)),
                                       ((-91.957,-38.0),(-95.26,-89.957,-41.303,-36.0)),((-46.957,-38.0),(-48.957,-43.654,-41.303,-36.0))):
        s=s+(cyl(cx,cy,2.5,2.5,5.0)+box(bx0,bx1,by0,by1,2.5,5.0))
    for cx,cy in ((-91.957,7.0),(-46.957,7.0),(-91.957,-38.0),(-46.957,-38.0)): s=s-cyl(cx,cy,1.0,1.0,5.5)
    for cx,cy in ((-35.457,7.9),(-35.457,-38.9),(-98.26,7.9),(-98.26,-38.9)): s=s-cyl(cx,cy,1.0,7.73,22,48)
    for cx,cy in ((-97.76,-15.5),(-38.05,-15.5)):
        s=s-cyl(cx,cy,1.65,17.7,22,48)
        s=s-loft([(20.7,circle(cx,cy,1.65,48)),(21.0,circle(cx,cy,1.95,48)),(21.5,circle(cx,cy,2.45,48))])
    # +Y wall openings (cavity y 9.803 -> recess floor y 11.0)
    s=s-wall_hole(stadium(-69.467,8.27,10.004,4.006),'+Y',9.0,11.0,12.6)
    for cx,cz,r in ((-51.752,8.25,1.5),(-57.563,8.25,1.5),(-62.0,7.5,0.75)):
        s=s-wall_hole(lambda d,cx=cx,cz=cz,r=r: circle(cx,cz,r+d,64),'+Y',9.0,11.0,12.6)
    # -Y light hole (cavity y -40.803 -> recess floor y -42.0); w=-y
    s=s-wall_hole(lambda d: circle(-53.632,7.225,1.0+d,64),'-Y',40.0,42.0,43.6)
    # +X antenna opening: traced profile (y,z), prism along x
    A=ccw([tuple(p) for p in antenna_poly])
    s=s-xform(loft([(-39.5,A),(-31.0,A)]),[(0,1,0),(0,0,1),(1,0,0)])
    if tripod: s=s-thread_cutter(CX,CY)
    return s

if __name__=='__main__':
    d=json.load(open('wall_sections.json'))
    ant=sorted(d['+X'],key=lambda P: abs(sum(P[i][0]*P[(i+1)%len(P)][1]-P[(i+1)%len(P)][0]*P[i][1] for i in range(len(P)))))[0]
    for name,tri in (('base_std',False),('base_tripod',True)):
        m=build(tri,ant); mesh=m.to_mesh()
        V=np.array(mesh.vert_properties)[:,:3].astype(float); T=np.array(mesh.tri_verts)
        print(name,'status',m.status().name,'genus',m.genus(),'verts',len(V),'tris',len(T),'volume',round(m.volume(),3))
        json.dump({'v':V.tolist(),'f':T.tolist()},open(name+'_csg.json','w'))
