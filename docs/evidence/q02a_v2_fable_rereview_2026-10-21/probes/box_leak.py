import sys, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
AV,AH=t.AV,t.AH
rs=np.random.default_rng(21)
vv,uu=np.mgrid[0:90,0:160]
def trial(w,h,r0,c0,depth0,slope):
    x=np.full((24,40),10.0); y=np.clip(AV@x@AH.T,0,10)
    m=np.zeros((90,160),bool); m[r0:r0+h,c0:c0+w]=True
    y2=y.copy(); y2[m]=depth0+slope*(uu[m]-c0)+0.5*slope*(vv[m]-r0)
    y2=y2.astype(np.float32).astype(np.float64)
    nat,pasted,res=geo.recover_native_depth(y2,(40,24))
    chk,_,_,_=va._depth_structure(y2,c.FROZEN_F_PROFILE,0.5)
    return int((m&~pasted).sum()), chk.status, float(np.nanmin(nat))
for (w,h) in ((8,8),(12,12),(16,10),(20,20),(24,12),(30,30)):
    n=leak=ok_leak=0
    for k in range(300):
        r0=int(rs.integers(0,90-h)); c0=int(rs.integers(0,160-w))
        kept,st,nmin=trial(w,h,r0,c0,1.5,0.002)
        n+=1
        if kept>0: leak+=1
        if kept>0 and st=="OK": ok_leak+=1
    print("rect %2dx%2d planar paste (slope 2 mm/px): leaks %3d/%d, with depth_structure OK %3d"%(w,h,leak,n,ok_leak))
# also with slope 0 (constant) and a bigger slope
for slope in (0.0,0.0005,0.005):
    n=leak=0
    for k in range(300):
        w,h=16,16; r0=int(rs.integers(0,90-h)); c0=int(rs.integers(0,160-w))
        kept,st,nmin=trial(w,h,r0,c0,1.5,slope); n+=1; leak+= (kept>0 and st=="OK")
    print("rect 16x16 slope %.4f: leaks(structure OK) %d/%d"%(slope,leak,n))
# hypothesis consequence: one leaking rectangle on the LOS with belief at 3 m (paste at 1.5 m => foreground), no flag
cnt=0; occ=0
for k in range(300):
    w,h=16,16; r0=int(rs.integers(30,50)); c0=int(rs.integers(65,85))
    x=np.full((24,40),10.0); y=np.clip(AV@x@AH.T,0,10); m=np.zeros((90,160),bool); m[r0:r0+h,c0:c0+w]=True
    y2=y.copy(); y2[m]=1.5+0.002*(uu[m]-c0); 
    inp=t.make(target_v=(3.,0.,0.), render_target=False, depth=y2.astype(np.float32), cov_pos=0.0025)
    r=ev.estimate(inp); cnt+=1
    if r.hypothesis==ev.OCCLUDED: occ+=1; ex=(r0,c0,r.reasons,(r.camera.rays,r.camera.foreground,r.camera.clear),r.pasted_pixels_in_footprint)
print("16x16 planar paste near LOS, belief 3 m, no flag: OCCLUDED %d/%d"%(occ,cnt), ex if occ else "")
