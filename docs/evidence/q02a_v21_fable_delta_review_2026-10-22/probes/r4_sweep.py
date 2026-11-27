"""R4 close-range sweep + phantom-cell check + depth_native None invariant, production-faithful scene."""
import sys, math, json
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev

rows = []
bad = []
for rng_ in [round(0.20 + 0.01 * i, 2) for i in range(0, 61)] + [1.0, 1.5, 2.0, 3.0, 5.0, 8.0, 9.5, 9.9, 10.5, 12.0, 15.0, 19.0]:
    for lat in (0.0, 0.1, 0.25):
        for flag in (True, False):
            tv = (rng_, lat, 0.0)
            inp = t.make(target_v=tv, detected=flag)
            rep = va.validate(inp, t.CAM, t.LID)
            r = ev.estimate(inp)
            ds, od = rep.status("depth_structure"), rep.status("obstacle_depth")
            inv = (rep.depth_native is None) == (od != va.OK)
            phantom = None
            if rep.depth_native is not None:
                nat = rep.depth_native
                seen = np.isfinite(nat)
                phantom = int((nat[seen] < 10.0 - 1e-3).sum())     # scene has no bars: any hit is a phantom
            paste = np.asarray(inp.depth) < 10.0 - 1e-6
            kept_paste = int((paste & ~rep.depth_excluded).sum()) if rep.depth_excluded is not None else None
            rows.append(dict(range=rng_, lat=lat, flag=flag, ds=ds, od=od, hyp=r.hypothesis, ident=r.identity_status,
                             reasons=list(r.reasons), phantom=phantom, kept_paste=kept_paste, paste_px=int(paste.sum()),
                             foreign=int(rep.depth_pasted.sum()) if rep.depth_pasted is not None else None,
                             at_range=r.pasted_pixels_at_track_range, in_fp=r.pasted_pixels_in_footprint, inv=inv,
                             problems=list(r.validation_problems)))
            if not inv or (phantom and phantom > 0) or (flag and r.hypothesis != ev.TARGET_VISIBLE) or (flag and r.identity_status != "SUPPORTED_BY_FOOTPRINT" and ds == va.OK):
                bad.append(rows[-1])
print("rows", len(rows), "anomalies", len(bad))
for b in bad:
    print("  ANOM", {k: b[k] for k in ("range", "lat", "flag", "ds", "od", "hyp", "ident", "phantom", "kept_paste", "reasons", "problems")})
print("\n--- straight-ahead summary (flag=True) ---")
for b in rows:
    if b["lat"] == 0.0 and b["flag"]:
        print("  r=%.2f ds=%-14s od=%-11s hyp=%-18s ident=%-22s paste=%5d foreign=%5s kept=%s at_range=%s phantom=%s" % (
            b["range"], b["ds"], b["od"], b["hyp"], b["ident"], b["paste_px"], b["foreign"], b["kept_paste"], b["at_range"], b["phantom"]))
print("\n--- straight-ahead summary (flag=False) ---")
for b in rows:
    if b["lat"] == 0.0 and not b["flag"]:
        print("  r=%.2f ds=%-14s od=%-11s hyp=%-18s reasons=%s" % (b["range"], b["ds"], b["od"], b["hyp"], ",".join(b["reasons"])[:80]))
json.dump(rows, open(sys.argv[1], "w"), indent=1, default=str)
