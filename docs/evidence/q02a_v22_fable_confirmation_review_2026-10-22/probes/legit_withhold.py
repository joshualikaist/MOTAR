"""Does the contact guard withhold legitimate frames? Bars at 0.25-1.5 m from the vehicle, targets 0.45-1.5 m, no target."""
import sys, math, collections
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
cam_o = np.array(t.CAM.camera_offset)
print("--- bars only (no target in image; belief 8 m): bar centre distance d from vehicle centre, radius rb, bearing b ---")
rows = []
for rb in (0.2, 0.3, 0.4):
    for b in (0.0, 20.0, 40.0):
        for d in (0.25, 0.3, 0.35, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5):
            bars = ((d * math.cos(math.radians(b)), d * math.sin(math.radians(b)), rb),)
            inp = t.make(render_target=False, bars=bars)
            rep = va.validate(inp, t.CAM, t.LID)
            surf_cam = math.hypot(d * math.cos(math.radians(b)) - 0.1, d * math.sin(math.radians(b))) - rb
            nmin = float(np.nanmin(rep.depth_native)) if rep.depth_native is not None else float("nan")
            det = dict((c.name, c.detail) for c in rep.checks)["obstacle_depth"]
            r = ev.estimate(inp)
            rows.append((rb, b, d, round(surf_cam, 3), rep.status("obstacle_depth"), det[:22], round(nmin, 3), r.hypothesis, r.near_obstacle_warning))
for row in rows: print("  ", row)
wh = [r for r in rows if r[4] != "OK"]
print("withheld:", len(wh), "of", len(rows), "; max camera-to-surface distance among withheld: %.3f m" % max([r[3] for r in wh] or [float('nan')]),
      "; min camera-to-surface among OK: %.3f" % min([r[3] for r in rows if r[4] == 'OK'] or [float('nan')]))
print("\n--- target at 0.45-1.5 m, flag set, with/without a bar at 1 m ---")
for bars in ((), ((1.0, 0.3, 0.3),), ((0.7, -0.5, 0.3),)):
    for r_ in (0.45, 0.5, 0.6, 0.8, 1.0, 1.5):
        for lat in (0.0, 0.2):
            inp = t.make(target_v=(r_, lat, 0.0), detected=True, bars=bars)
            rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
            det = dict((c.name, c.detail) for c in rep.checks)["obstacle_depth"]
            print("  bars=%s r=%.2f lat=%.1f od=%s %-26s hyp=%s id=%s contact=%s" % (bars, r_, lat, rep.status("obstacle_depth"), det[:26], r.hypothesis, r.identity_status, rep.camera_contact_range))
