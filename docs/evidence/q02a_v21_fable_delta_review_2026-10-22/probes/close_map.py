"""Fine close-range map: where does the production-faithful sphere paste pass as obstacle depth?"""
import sys, math, json
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
out = []
for r_ in np.arange(0.255, 0.405, 0.005):
    for lat in (0.0, 0.03, 0.06, 0.1, 0.15):
        for z in (0.0, 0.05):
            tv = (float(r_), lat, z)
            inp = t.make(target_v=tv, detected=True)
            rep = va.validate(inp, t.CAM, t.LID)
            rec = geo.recover_native_depth(np.asarray(inp.depth), (40, 24))
            ds, od = rep.status("depth_structure"), rep.status("obstacle_depth")
            nat = rec.native; seen = np.isfinite(nat)
            phantom = int((nat[seen] < 10 - 1e-3).sum()); nclip = int((nat[seen] < 0.05).sum())
            mn = float(nat[seen].min()) if seen.any() else float("nan")
            r = ev.estimate(inp)
            out.append(dict(r=round(float(r_), 3), lat=lat, z=z, ds=ds, od=od, status=rec.status, excl=round(float(rec.excluded.mean()), 3),
                            foreign=int(rec.foreign.sum()), residual=rec.residual, phantom=phantom, near_clipped_cells=nclip, native_min=mn,
                            hyp=r.hypothesis, ident=r.identity_status, leak=bool(od == va.OK and phantom > 0)))
leaks = [o for o in out if o["leak"]]
print("cases", len(out), "LEAKS (obstacle_depth OK with phantom cells):", len(leaks))
for o in leaks:
    print("  ", o)
print("\nflag lost (hyp != TARGET_VISIBLE with flag):")
for o in out:
    if o["hyp"] != ev.TARGET_VISIBLE:
        print("  r=%.3f lat=%.2f z=%.2f ds=%s od=%s status=%s excl=%.3f residual=%s hyp=%s" % (o["r"], o["lat"], o["z"], o["ds"], o["od"], o["status"], o["excl"], o["residual"], o["hyp"]))
print("\nwould 'any NEAR_CLIPPED native cell => UNAVAILABLE' close every leak?", all(o["near_clipped_cells"] > 0 for o in leaks))
print("leaks native_min:", [o["native_min"] for o in leaks])
json.dump(out, open(sys.argv[1], "w"), indent=1, default=float)
