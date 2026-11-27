"""D2: legitimate target pixels lost by the fill-value exclusion (|d-10|<=1e-3); targets beyond 10 m; fill pixels in depth_pasted."""
import sys, math
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
print("--- any fill-valued pixel in depth_pasted? (sweep 0.5..19 m + bars) ---")
bad = 0; n = 0
for r_ in list(np.arange(0.5, 19.01, 0.25)):
    for bars in ((), ((4.0, 0.5, 0.3),), ((6.0, -0.4, 0.3),)):
        inp = t.make(target_v=(float(r_), 0.0, 0.0), detected=True, bars=bars)
        rep = va.validate(inp, t.CAM, t.LID)
        if rep.depth_pasted is not None:
            n += 1; bad += int((rep.depth_pasted & (np.abs(np.asarray(inp.depth) - 10.0) <= 1e-3)).sum())
print("  frames", n, "fill-valued pixels in depth_pasted:", bad)
print("\n--- pixels lost: true paste pixels within 1e-3 of 10 m (never foreign now) ---")
tot_lost = 0; worst = (0, None)
for r_ in np.arange(9.90, 10.50, 0.01):
    for lat in (0.0, 0.3, -0.5):
        for z in (0.0, 0.1):
            tv = (float(r_), lat, z)
            inp = t.make(target_v=tv, detected=True)
            bg = np.asarray(t.make(target_v=tv, render_target=False).depth); d = np.asarray(inp.depth)
            paste = d != bg
            rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
            lost = int((paste & (np.abs(d - 10.0) <= 1e-3)).sum()); tot_lost += lost
            miss = int((paste & ~rep.depth_pasted).sum())
            if lost > worst[0]: worst = (lost, (tv, int(paste.sum()), int(rep.depth_pasted.sum()), r.identity_status, r.pasted_pixels_at_track_range))
            if r.identity_status != "SUPPORTED_BY_FOOTPRINT" or lost > 0:
                print("  tv=%s paste=%d foreign=%d paste_not_foreign=%d lost_fill=%d -> %s %s in=%d at=%d" % (tv, paste.sum(), rep.depth_pasted.sum(), miss, lost, r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range))
print("  total lost pixels over sweep:", tot_lost, " worst:", worst)
print("\n--- targets beyond 10 m (pasted with depth > 10): flag path and no-flag path ---")
for r_ in (10.2, 10.5, 11.0, 12.0, 15.0, 19.0):
    inp = t.make(target_v=(r_, 0.0, 0.0), detected=True); rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    rn = ev.estimate(t.make(target_v=(r_, 0.0, 0.0)))
    d = np.asarray(inp.depth)
    print("  r=%.1f ds=%s od=%s foreign=%d (>10: %d) flag: %s %s in=%d at=%d | noflag: %s %s" % (r_, rep.status("depth_structure"), rep.status("obstacle_depth"), rep.depth_pasted.sum(), int((rep.depth_pasted & (d > 10.001)).sum()), r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, rn.hypothesis, ",".join(rn.reasons)[:60]))
