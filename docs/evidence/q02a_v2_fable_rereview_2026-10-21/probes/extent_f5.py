import sys, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
def show(name, inp):
    r=ev.estimate(inp)
    print("%-62s %-19s %s ext=%s cam=%s lid=%s pasted=%d"%(name,r.hypothesis,r.reasons,r.lidar.foreground_extended,
          (r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.censored),
          (r.lidar.rays,r.lidar.foreground,r.lidar.consistent,r.lidar.clear,r.lidar.censored), r.pasted_pixels_in_footprint))
# F5 reproduction from the test, then add a bar elsewhere (90 deg to the left) at the same range as the target surface
base=t.make(target_v=(10.5,0.,0.), target_in_camera=False)
mean=np.array(base.track_mean); mean[0]+=1.0
show("F5 test: target 10.5, belief 11.5, no camera paste", t.make(target_v=(10.5,0.,0.), target_in_camera=False, track_mean=mean))
show("same + unrelated bar at 90deg left, range 10.3", t.make(target_v=(10.5,0.,0.), target_in_camera=False, track_mean=mean, bars=((0.,10.3,.3),)))
show("same + unrelated bar at 180deg behind, range 10.3", t.make(target_v=(10.5,0.,0.), target_in_camera=False, track_mean=mean, bars=((-10.3,0.,.3),)))
# production-reachable variant: target just outside camera image laterally while belief inside
# hfov 87 -> half 43.5 deg; target at bearing 45 deg range 6 m; belief at bearing 40 deg (inside image with small cov)
tv=(6*math.cos(math.radians(45)), 6*math.sin(math.radians(45)), 0.)
bv=np.array([6*math.cos(math.radians(40)), 6*math.sin(math.radians(40)), 0.])
inp=t.make(target_v=tv)
world=geo.vehicle_to_world(bv, t.POS, 0.0)
m=np.r_[world,0,0,0]
show("target at 45deg (outside image), belief 40deg", t.make(target_v=tv, track_mean=m))
show("same + unrelated bar behind at range 5.9", t.make(target_v=tv, track_mean=m, bars=((-5.9,0.,.3),)))
# split disc: target partially behind a thin bar (production-like), check recovery sanity
for bar_r in (0.05,0.1,0.2):
    inp=t.make(target_v=(9.,0.,0.), bars=((6.,0.,bar_r),))
    rep=va.validate(inp,t.CAM,t.LID)
    nat=rep.depth_native
    print("split disc bar r=%.2f: structure=%s residual=%s pasted=%d unobs=%d native min=%.3f  hyp=%s"%(bar_r, rep.status("depth_structure"), rep.depth_residual, rep.depth_pasted.sum(), np.isnan(nat).sum(), np.nanmin(nat), ev.estimate(inp).hypothesis))
    show("  partially occluded target (no flag)", inp)
    show("  partially occluded target (flag)", t.make(target_v=(9.,0.,0.), bars=((6.,0.,bar_r),), detected=True))
# close target (big blob)
for rng_ in (0.6,0.4,0.3):
    inp=t.make(target_v=(rng_,0.,0.))
    rep=va.validate(inp,t.CAM,t.LID)
    print("close target %.1f m: structure=%s pasted frac=%.2f unobs=%d"%(rng_,rep.status("depth_structure"),rep.depth_pasted.mean(),np.isnan(rep.depth_native).sum() if rep.depth_native is not None else -1))
    show("  close target (flag)", t.make(target_v=(rng_,0.,0.), detected=True))
