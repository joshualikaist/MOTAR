"""1 mm steps across the 0.295-0.345 m transition band (lat 0/0.03, z 0/0.05), flag set."""
import sys, json, collections
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
out = []
for r_ in np.arange(0.295, 0.3451, 0.001):
    for lat in (0.0, 0.03):
        for z in (0.0, 0.05):
            tv = (float(r_), lat, z); inp = t.make(target_v=tv, detected=True)
            rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
            det = dict((c.name, c.detail) for c in rep.checks)["obstacle_depth"]
            nat = rep.depth_native; seen = None if nat is None else np.isfinite(nat)
            phantom = int((nat[seen] < 10 - 1e-3).sum()) if nat is not None else 0
            out.append(dict(r=round(tv[0], 3), lat=lat, z=z, ds=rep.status("depth_structure"), od=rep.status("obstacle_depth"), det=det.split()[0] if det else "", phantom=phantom, hyp=r.hypothesis, id=r.identity_status, warn=r.near_obstacle_warning))
print("cases", len(out), "leaks (od OK & phantom>0):", sum(o["od"] == "OK" and o["phantom"] > 0 for o in out))
c = collections.Counter((o["ds"], o["od"], o["det"], o["hyp"], o["id"]) for o in out)
for k, v in sorted(c.items(), key=lambda kv: -kv[1]): print("%4d" % v, k)
print("transitions along r at lat=0 z=0:")
prev = None
for o in out:
    if o["lat"] == 0.0 and o["z"] == 0.0:
        key = (o["ds"], o["od"], o["det"], o["hyp"])
        if key != prev: print("  r=%.3f -> %s" % (o["r"], key)); prev = key
print("UNKNOWN-with-flag ranges:", sorted(set(o["r"] for o in out if o["hyp"] != "TARGET_VISIBLE")))
json.dump(out, open(sys.argv[1], "w"), indent=0)
