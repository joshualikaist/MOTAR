import sys, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
def show(name, inp):
    r=ev.estimate(inp)
    print("%-66s %-19s %-24s %s cam=%s lid=%s pasted=%d"%(name,r.hypothesis,r.identity_status,r.reasons,
          (r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.censored,r.camera.unobserved),
          (r.lidar.rays,r.lidar.foreground,r.lidar.consistent,r.lidar.clear,r.lidar.censored), r.pasted_pixels_in_footprint))
print("F4 thin bar case actual outcome:"); show("  bar (4,0.12,0.08)", t.make(render_target=False, bars=((4.,0.12,.08),)))
show("  bar (4,0.0,0.08) centred thin", t.make(render_target=False, bars=((4.,0.0,.08),)))
show("  bar (4,0.0,0.20) 0.4 m wide centred", t.make(render_target=False, bars=((4.,0.0,.20),)))
show("  bar (9,0.0,0.20) 0.4 m wide at 9 m, target 9.8", t.make(target_v=(9.8,0,0), render_target=False, bars=((9.,0.0,.20),)))
show("  bar (9,0.25,0.20) 0.4 m wide at 9 m off-centre, target 9.8", t.make(target_v=(9.8,0,0), render_target=False, bars=((9.,0.25,.20),)))
print("F6 footprint support ignores range / accepts any surface:")
base=t.make(target_v=(3.,0.,0.), detected=True)
m=np.array(base.track_mean); m[0]+=5.0   # belief at 8 m, real pasted target at 3 m, flag set
show("  target pasted at 3 m, belief 8 m, flag", t.make(target_v=(3.,0.,0.), detected=True, track_mean=m))
show("  no target, bar r=0.15 at 8 m (lookalike), flag", t.make(render_target=False, bars=((8.,0.,.15),), detected=True))
show("  no target, bar r=0.15 at 8 m (lookalike), no flag", t.make(render_target=False, bars=((8.,0.,.15),)))
show("  no target, wide bar r=0.4 at 8 m, flag", t.make(render_target=False, bars=((8.,0.,.4),), detected=True))
print("distractor-like second blob at a different range on the same bearing: paste a second sphere by hand")
inp=t.make(target_v=(8.,0.,0.), detected=True)
d=np.array(inp.depth); vv,uu=np.mgrid[0:90,0:160]; rays=geo.pixel_rays(uu,vv,t.CAM)
t2=geo.sphere_first_hit(np.array(t.CAM.camera_offset),rays,[3.,0.,0.],0.15); d=np.where(np.isfinite(t2),t2,d)
show("  target at 8 m + lookalike sphere pasted at 3 m, flag", t.make(target_v=(8.,0.,0.), detected=True, depth=d))
show("  target at 8 m + lookalike sphere pasted at 3 m, no flag", t.make(target_v=(8.,0.,0.), depth=d))
