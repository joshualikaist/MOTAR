import sys, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
AV,AH=t.AV,t.AH
def probe(tv, belief_dx, cov_pos=0.01, detected=False):
    inp0=t.make(target_v=tv, cov_pos=cov_pos)
    mean=np.array(inp0.track_mean); mean[0]+=belief_dx
    inp=t.make(target_v=tv, cov_pos=cov_pos, track_mean=mean, detected=detected)
    rep=va.validate(inp,t.CAM,t.LID)
    true_paste = inp.depth < 10.0-1e-6
    kept = int((true_paste & ~rep.depth_pasted).sum()) if rep.depth_pasted is not None else -1
    return rep.status("depth_structure"), int(true_paste.sum()), kept, ev.estimate(inp), rep
# disc crossing top/bottom border: vertical half-FOV 29 deg. put the target so that its disc centre is near the border.
rs=np.random.default_rng(9); n=leak=occ=dm=0; examples=[]
for k in range(400):
    rng_=rs.uniform(0.5,2.0)                      # distance from vehicle centre along x
    x_cam=rng_-0.1
    ang_r=math.degrees(math.asin(min(0.15/max(x_cam,0.16),1.0)))   # angular radius of disc
    edge=rs.choice([-1,1])*math.radians(rs.uniform(29-ang_r-2, 29+ang_r-1))  # centre elevation near the top/bottom border
    dz=x_cam*math.tan(edge)+0.03
    dy=rs.uniform(-0.3,0.3)*x_cam
    st,npaste,kept,r,rep=probe((rng_,dy,dz),1.0,cov_pos=0.01)
    n+=1
    if st=="OK" and kept>0:
        leak+=1
        if len(examples)<8: examples.append((rng_,dy,dz,npaste,kept,r.hypothesis,r.reasons,(r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.unobserved),r.pasted_pixels_in_footprint, float(np.nanmin(rep.depth_native))))
    if r.hypothesis==ev.OCCLUDED: occ+=1
    if r.hypothesis==ev.DETECTION_MISS: dm+=1
print("border-crossing sphere sweep: frames",n,"leaks",leak,"OCCLUDED",occ,"DETECTION_MISS",dm)
for e in examples: print("  ",e)
# the 0.3 m case without flag and with small covariance
for cov in (0.01,0.0004):
    st,npaste,kept,r,rep=probe((0.3,0.,0.),0.0,cov_pos=cov)
    print("target 0.3 m belief exact cov %.4f: structure=%s paste=%d kept=%d -> %s %s cam=%s pasted_fp=%d natmin=%.3f"%(cov,st,npaste,kept,r.hypothesis,r.reasons,(r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.unobserved),r.pasted_pixels_in_footprint,np.nanmin(rep.depth_native)))
    st,npaste,kept,r,rep=probe((0.3,0.,0.),1.0,cov_pos=cov)
    print("target 0.3 m belief +1.0 cov %.4f: structure=%s paste=%d kept=%d -> %s %s cam=%s pasted_fp=%d"%(cov,st,npaste,kept,r.hypothesis,r.reasons,(r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.unobserved),r.pasted_pixels_in_footprint))
# sweep very close ranges 0.25..0.6 straight ahead, belief +1.0 and exact
print("very close straight-ahead sweep (belief exact / +1 m), no flag:")
for rng_ in (0.25,0.3,0.35,0.4,0.45,0.5,0.55,0.6):
    out=[]
    for dx in (0.0,1.0):
        st,npaste,kept,r,rep=probe((rng_,0.,0.),dx,cov_pos=0.0025)
        out.append("%s paste=%d kept=%d %s %s"%(st,npaste,kept,r.hypothesis,r.reasons[:2]))
    print("  %.2f m: "%rng_ + " | ".join(out))
