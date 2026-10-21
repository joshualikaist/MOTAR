"""Adversarial pasted shapes for recover_native_depth. Report: exceptions, pasted pixels kept, max error on observed cells,
phantom values (observed cell value far from truth), depth_structure status."""
import sys, time, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT)
from motar_eval.q02 import contracts as c, geometry as geo, validation as va
AV, AH = geo.bilinear_matrix(90,24), geo.bilinear_matrix(160,40)
PROF=c.FROZEN_F_PROFILE
rng=np.random.default_rng(3)

def native_grid(kind):
    x=np.full((24,40),10.0)
    if kind=="empty": return x
    if kind=="bars":
        for cc,w,dd in ((5,2,4.0),(18,3,7.5),(31,2,2.2)):
            x[:,cc:cc+w]=dd
        return x
    if kind=="random":
        x=rng.uniform(0.5,10.0,(24,40)); x[rng.uniform(size=x.shape)<0.6]=10.0; return x
    if kind=="smooth":
        vv,uu=np.mgrid[0:24,0:40]; return 3+0.1*uu+0.05*vv

def run(name, x, paste_mask, paste_vals):
    y=(AV@x@AH.T)
    y=np.clip(y,0,10)
    y2=y.copy(); y2[paste_mask]=paste_vals[paste_mask] if isinstance(paste_vals,np.ndarray) else paste_vals
    y2=y2.astype(np.float32).astype(np.float64)
    t0=time.perf_counter()
    try:
        nat,pasted,res=geo.recover_native_depth(y2,(40,24))
    except Exception as e:
        print("%-46s EXCEPTION %r"%(name,e)); return
    dt=time.perf_counter()-t0
    obs=np.isfinite(nat)
    err=np.abs(nat-x)[obs]
    kept_pasted=int((paste_mask&~pasted).sum())
    chk,_,_,_=va._depth_structure(y2,PROF,0.5)
    phantom=int((err>1e-2).sum())
    cats=va.classify_depth_native(nat,PROF); true_cats=va.classify_depth_native(x,PROF)
    catmis=int(((cats!=true_cats)&obs).sum())
    flag="" if (kept_pasted==0 and phantom==0 and catmis==0) else "  <<<"
    print("%-46s res=%.1e pasted=%5d kept_pasted=%4d unobs=%3d maxerr=%.1e phantom=%d catmis=%d %s %.0fms%s"%(
        name,res,pasted.sum(),kept_pasted,(~obs).sum(),err.max() if err.size else -1,phantom,catmis,chk.status,dt*1e3,flag))
    return nat,pasted,res,obs,err

vv,uu=np.mgrid[0:90,0:160]
cases=[]
# thin lines
m=np.zeros((90,160),bool); m[45,:]=True; cases.append(("hline full row", m, 3.0))
m=np.zeros((90,160),bool); m[:,80]=True; cases.append(("vline full col", m, 3.0))
m=np.zeros((90,160),bool); m[40,60:100]=True; cases.append(("hline segment", m, 3.0))
m=np.zeros((90,160),bool); m[20:70,77]=True; cases.append(("vline segment", m, 3.0))
m=np.zeros((90,160),bool); m[np.arange(0,90), np.arange(0,90)+30]=True; cases.append(("diagonal line", m, 3.0))
# single pixels
for (r,cc) in ((0,0),(0,159),(89,0),(89,159),(45,80),(1,1),(88,158),(0,80),(45,0),(45,159)):
    m=np.zeros((90,160),bool); m[r,cc]=True; cases.append(("single px (%d,%d)"%(r,cc), m, 3.0))
# rings
d2=(vv-45)**2+(uu-80)**2
cases.append(("ring r8-10", (d2>=64)&(d2<=100), 3.0))
cases.append(("ring r3-5", (d2>=9)&(d2<=25), 3.0))
# blobs at borders
cases.append(("blob corner (0,0) r6", (vv**2+uu**2)<36, 3.0))
cases.append(("blob corner (89,159) r6", ((vv-89)**2+(uu-159)**2)<36, 3.0))
cases.append(("blob top edge r5", ((vv-0)**2+(uu-80)**2)<25, 3.0))
cases.append(("blob left edge r5", ((vv-45)**2+(uu-0)**2)<25, 3.0))
cases.append(("blob right edge r5", ((vv-45)**2+(uu-159)**2)<25, 3.0))
# several blobs
m=np.zeros((90,160),bool)
for (r,cc) in ((20,30),(60,120),(45,80),(10,140),(80,20)):
    m|=((vv-r)**2+(uu-cc)**2)<16
cases.append(("five blobs r4", m, 3.0))
# aligned rectangle exactly one column segment x0=1 -> cols 5..8, rows segment x0=0 -> rows 2..5
m=np.zeros((90,160),bool); m[2:6,5:9]=True; cases.append(("aligned rect cols5-8 rows2-5", m, 3.0))
m=np.zeros((90,160),bool); m[2:6,5:9]=True; cases.append(("aligned rect value 10 on bars", m, 10.0))
# large aligned rectangle several segments
m=np.zeros((90,160),bool); m[32:47,61:85]=True; cases.append(("aligned rect 15x24", m, 4.0))
# two blobs with 1 and 2 px gap (split disc)
m=np.zeros((90,160),bool); m[40:50,70:77]=True; m[40:50,78:85]=True; cases.append(("two rects gap 1px", m, 5.0))
m=np.zeros((90,160),bool); m[40:50,70:77]=True; m[40:50,79:86]=True; cases.append(("two rects gap 2px", m, 5.0))
m=np.zeros((90,160),bool); m[40:50,70:77]=True; m[40:50,80:87]=True; cases.append(("two rects gap 3px", m, 5.0))
m=np.zeros((90,160),bool); m[30:40,70:90]=True; m[41:51,70:90]=True; cases.append(("two rects row gap 1px", m, 5.0))
# vertical slabs leaving a 1-col and 2-col kept strip (singular normal probe)
m=np.zeros((90,160),bool); m[:, :17]=True; m[:, 18:]=True; cases.append(("all but col 17", m, 5.0))
m=np.zeros((90,160),bool); m[:, :17]=True; m[:, 19:]=True; cases.append(("all but cols 17-18", m, 5.0))
m=np.zeros((90,160),bool); m[20:60, 40:77]=True; m[20:60, 78:115]=True; cases.append(("two slabs gap col 77", m, 5.0))
m=np.zeros((90,160),bool); m[20:60, 40:77]=True; m[20:60, 79:115]=True; cases.append(("two slabs gap cols 77-78", m, 5.0))
# paste value equal to underlying upsample / within tol
# values > 10 (target at 15 m)
cases.append(("blob value 15 (beyond far)", d2<16, 15.0))
cases.append(("blob value 0 (zero paste)", d2<16, 0.0))
# large blob 30% of image
cases.append(("big blob r40", d2<1600, 2.0))
cases.append(("huge blob r60 (>50%?)", d2<3600, 2.0))
# sphere-like paste small (realistic target at 8 m => radius ~1.6px)
sph=np.where(d2<4, 7.85+0.03*d2, 0.0); cases.append(("sphere r2 at 8m", d2<4, sph))
sph=np.where(d2<100, 1.85+0.0005*d2, 0.0); cases.append(("sphere r10 at 2m flat", d2<100, sph))
for kind in ("empty","bars","random","smooth"):
    x=native_grid(kind)
    print("=== native:",kind)
    for name,m,val in cases:
        run(name,x,m,val)
    # paste equal to upsample
    y=np.clip(AV@x@AH.T,0,10); run("paste == upsample (invisible)", x, d2<16, y)
    run("paste == upsample + 5e-4 (within tol)", x, d2<16, y+5e-4)
