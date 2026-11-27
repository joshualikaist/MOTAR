import sys, math, traceback
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
CAM,LID,PROF=t.CAM,t.LID,t.PROF
print("classify_lidar boundaries:", list(va.classify_lidar([0.2,0.1999999,0.2000001,11.9999999,12.0,12.0000001,-0.0,1e-7,-1e-7,np.float32(12.0),12.000001],LID,PROF)))
print("classify_depth_native boundaries:", list(va.classify_depth_native([0.05,0.0499999,9.999,9.9989,10.0,10.5,-0.001,np.inf,-np.inf],PROF)))
def show(name, **kw):
    try:
        r=ev.estimate(t.make(**kw))
        print("%-50s %-19s %s cam=%s lid=%s near=%s nb=%d prob=%s"%(name,r.hypothesis,r.reasons,
              (r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.censored,r.camera.near_clipped,r.camera.unobserved),
              (r.lidar.rays,r.lidar.foreground,r.lidar.consistent,r.lidar.clear,r.lidar.censored,r.lidar.near_clipped), r.near_obstacle_warning, r.near_clipped_lidar_beams, r.validation_problems))
        return r
    except Exception as e:
        print("%-50s EXCEPTION %r"%(name,e)); traceback.print_exc()
base=t.make(render_target=False)
d=np.array(base.depth)
# NaN / Inf in depth
dn=d.copy(); dn[10,10]=np.nan; show("depth one NaN", render_target=False, depth=dn)
di=d.copy(); di[10,10]=np.inf; show("depth one Inf", render_target=False, depth=di)
# a single zero pixel in depth (pasted 0)
dz=d.copy(); dz[45,80]=0.0; show("depth single 0 at centre px", render_target=False, depth=dz)
# native-cell-sized zero patch at the centre (obstacle at ~0 m?) : paste 0 on whole 8x8 region
dz=d.copy(); dz[41:49,76:84]=0.0; show("depth 8x8 zero patch centre", render_target=False, depth=dz)
# depth near-clipped region made from a native 0.03 cell: emulate production native grid with 0.03 at the centre cell
x=np.full((24,40),10.0); x[11:13,19:21]=0.03; dd=np.clip(t.AV@x@t.AH.T,0,10); show("native 0.03 m cells on LOS (upsampled)", render_target=False, depth=dd)
x=np.full((24,40),10.0); x[11:13,19:21]=0.0; dd=np.clip(t.AV@x@t.AH.T,0,10); show("native 0.0 m cells on LOS (upsampled)", render_target=False, depth=dd)
# lidar: one NaN; one 0.1; one 12.5; one -0.0; all 12 except LOS 0.2 exactly
l=np.array(base.lidar).reshape(-1)
l1=l.copy(); l1[5]=np.nan; show("lidar one NaN", render_target=False, lidar=l1)
l1=l.copy(); l1[5]=0.1; show("lidar one 0.1 (0,0.2)", render_target=False, lidar=l1)
l1=l.copy(); l1[5]=12.5; show("lidar one 12.5", render_target=False, lidar=l1)
l1=l.copy(); l1[2*72+36]=0.2; show("lidar LOS beam exactly 0.2 m", render_target=False, lidar=l1)
l1=l.copy(); l1[2*72+36]=0.19; show("lidar LOS beam 0.19 m", render_target=False, lidar=l1)
l1=l.copy(); l1[2*72+36]=0.0; show("lidar LOS beam 0.0", render_target=False, lidar=l1)
l1=l.copy(); l1[2*72+36]=7.5; show("lidar LOS beam 7.5 (fg, target 8m)", render_target=False, lidar=l1)
l1=l.copy(); l1[2*72+36]=7.9; show("lidar LOS beam 7.9 (gap zone)", render_target=False, lidar=l1)
# both sensors invalid
show("depth NaN and lidar NaN", render_target=False, depth=dn, lidar=np.full(288,np.nan))
# zero covariance, zero range, target at camera
show("zero covariance", render_target=False, cov=np.zeros((6,6)))
show("target at own position", target_v=(0.,0.,0.), render_target=False)
show("target at camera origin", target_v=(0.1,0.,0.03), render_target=False)
show("target behind", target_v=(-3.,0.,0.), render_target=False)
show("target 0.3 m ahead", target_v=(0.3,0.,0.), render_target=False)
show("huge track mean", render_target=False, track_mean=np.array([1e9,0,0,0,0,0]))
show("age negative", render_target=False, age=-1.0)
show("cov with tiny negative eig", render_target=False, cov=np.diag([0.01]*3+[0.1]*3)-1e-12)
show("quat zero", render_target=False, quat=np.zeros(4))
show("depth all 10 (empty), lidar all 12", render_target=False, depth=np.full((90,160),10.0), lidar=np.full(288,12.0))
show("depth all 9.99 flat (hit everywhere)", render_target=False, depth=np.full((90,160),9.99))
show("depth all 10.5 (>far)", render_target=False, depth=np.full((90,160),10.5))
show("depth float32 10.0 pattern with 1e-6 noise", render_target=False, depth=np.full((90,160),10.0)+np.random.default_rng(1).normal(0,1e-6,(90,160)))
show("detected + depth NaN", detected=True, depth=dn)
show("detected + lidar NaN", detected=True, lidar=np.full(288,np.nan))
# determinism over random inputs: no crash fuzz
rng=np.random.default_rng(0); crashes=0
for k in range(200):
    kw=dict(target_v=tuple(rng.normal(size=3)*6), render_target=bool(rng.integers(2)), detected=bool(rng.integers(2)),
            bars=tuple((rng.uniform(1,10),rng.uniform(-3,3),rng.uniform(0.05,0.4)) for _ in range(rng.integers(0,4))),
            cov_pos=float(rng.uniform(0,0.5)), age=float(rng.uniform(0,3)), yaw=float(rng.uniform(-3,3)))
    try:
        a=ev.estimate(t.make(**kw)); b=ev.estimate(t.make(**kw)); assert a==b
    except Exception as e:
        crashes+=1; print("fuzz crash",kw,repr(e))
print("fuzz crashes:",crashes)
