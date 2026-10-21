import sys, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
AV,AH=t.AV,t.AH
# 1. mechanism on the r40 constant disc over an empty native grid
vv,uu=np.mgrid[0:90,0:160]; d2=(vv-45)**2+(uu-80)**2
x=np.full((24,40),10.0); y=np.clip(AV@x@AH.T,0,10); m=d2<1600; y[m]=2.0
flag=geo._nonlinear_pixels(y,AV,AH,1e-3)
comps=geo._components(flag)
print("r40 disc: flagged px",flag.sum(),"components",len(comps),"sizes",sorted(len(cp) for cp in comps)[-6:])
# top edge rows of the disc: which rows/cols of the disc boundary are unflagged?
edge = m & ~np.roll(m,1,0) | m & ~np.roll(m,-1,0) | m & ~np.roll(m,1,1) | m & ~np.roll(m,-1,1)
unfl = edge & ~flag
print("boundary px",edge.sum(),"unflagged boundary px",unfl.sum(),"rows of unflagged:",sorted(set(np.nonzero(unfl)[0])))
segs_rows=[s for s in geo._segments(AV)]; print("row segment starts:",[int(s[0]) for s in segs_rows][:12],"...")
nat,pasted,res=geo.recover_native_depth(y,(40,24))
print("recovered: pasted",pasted.sum(),"kept pasted",(m&~pasted).sum(),"native min %.3f"%np.nanmin(nat),"cells==2.0",(np.abs(nat-2.0)<1e-6).sum(),"res %.1e"%res)
# 2. production-faithful: sphere target via scene(), flag False, belief farther than truth
def probe(tv, belief_dx, cov_pos=0.01, bars=()):
    inp0=t.make(target_v=tv, bars=bars, cov_pos=cov_pos)
    mean=np.array(inp0.track_mean); mean[0]+=belief_dx
    inp=t.make(target_v=tv, bars=bars, cov_pos=cov_pos, track_mean=mean)
    rep=va.validate(inp,t.CAM,t.LID)
    true_paste = inp.depth < np.clip(AV@np.minimum(t._cylinder_hits(np.array(t.CAM.camera_offset),geo.native_camera_rays(t.CAM,(40,24)),bars),10.0)@AH.T,0,10)-1e-6
    kept = int((true_paste & ~rep.depth_pasted).sum()) if rep.depth_pasted is not None else -1
    r=ev.estimate(inp)
    return rep.status("depth_structure"), int(true_paste.sum()), kept, r
print("\n--- sphere paste leak sweep (no flag, belief = truth + dx) ---")
for rng_ in (2.0,1.5,1.2,1.0,0.9,0.8,0.7,0.6,0.5):
    for dy in (0.0,0.05,-0.07):
        for dz in (0.0,0.04):
            st,npaste,kept,r=probe((rng_,dy,dz),1.0)
            if kept>0 or r.hypothesis==ev.OCCLUDED:
                print("range %.1f dy %+.2f dz %+.2f: structure=%s pasted_true=%d KEPT_PASTED=%d -> %s %s cam=%s pasted_fp=%d"%(rng_,dy,dz,st,npaste,kept,r.hypothesis,r.reasons,
                      (r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.unobserved),r.pasted_pixels_in_footprint))
print("--- same, belief +1.5 m, small lateral sigma 0.03 (cov_pos=0.001) ---")
for rng_ in (1.5,1.2,1.0,0.9,0.8,0.7,0.6,0.5):
    for dy in (0.0,0.05,-0.07):
        st,npaste,kept,r=probe((rng_,dy,0.0),1.5,cov_pos=0.001)
        if kept>0 or r.hypothesis==ev.OCCLUDED:
            print("range %.1f dy %+.2f: structure=%s pasted_true=%d KEPT_PASTED=%d -> %s %s cam=%s pasted_fp=%d"%(rng_,dy,st,npaste,kept,r.hypothesis,r.reasons,
                  (r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.unobserved),r.pasted_pixels_in_footprint))
print("--- dense sweep: count frames with kept pasted pixels (structure OK), range 0.5-2.5, random lateral offsets ---")
rs=np.random.default_rng(2); n=0; leak=0; occ=0
for k in range(150):
    rng_=rs.uniform(0.5,2.5); dy=rs.uniform(-0.3,0.3)*rng_/2; dz=rs.uniform(-0.2,0.2)*rng_/2
    st,npaste,kept,r=probe((rng_,dy,dz),1.0)
    n+=1
    if st=="OK" and kept>0: leak+=1
    if r.hypothesis==ev.OCCLUDED: occ+=1; print("  OCCLUDED at range %.2f dy %.2f dz %.2f kept %d %s"%(rng_,dy,dz,kept,r.reasons))
print("frames",n,"leaks(structure OK & pasted px kept)",leak,"OCCLUDED",occ)
