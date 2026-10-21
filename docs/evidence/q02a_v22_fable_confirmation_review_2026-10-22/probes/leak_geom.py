"""Are the 6 extended-map leaks physical? Sphere/bar intersection test, and the same scenes with the bar moved so the bodies do not overlap."""
import sys, math
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
cam_o = np.array(t.CAM.camera_offset); NR = geo.native_camera_rays(t.CAM, t.PROF.obstacle_native_wh)
def check(tv, bars):
    inp = t.make(target_v=tv, detected=True, bars=bars); rep = va.validate(inp, t.CAM, t.LID)
    tn = np.minimum(t._cylinder_hits(cam_o, NR, bars), 10.0)
    dev = nmin = float("nan")
    if rep.depth_native is not None:
        seen = np.isfinite(rep.depth_native); dev = float(np.abs(rep.depth_native[seen] - tn[seen]).max()); nmin = float(rep.depth_native[seen].min())
    det = dict((c.name, c.detail) for c in rep.checks)["obstacle_depth"]
    cx, cy, rb = bars[0]; d_centres = math.hypot(tv[0] - cx, tv[1] - cy)
    return "tv=%s bar=%s centre-dist=%.3f (sphere+bar radii=%.2f -> %s) od=%s %s dev=%.3f nmin=%.3f cam-to-bar-surface=%.3f" % (
        tv, bars[0], d_centres, 0.15 + rb, "INTERSECT" if d_centres < 0.15 + rb else "separate", rep.status("obstacle_depth"), det[:20], dev, nmin, math.hypot(cx - 0.1, cy) - rb)
for r_ in (0.33, 0.35, 0.40):
    for z in (0.0, 0.05):
        print(check((r_, 0.1, z), ((0.45, 0.0, 0.2),)))
print("\n--- same target poses, bar moved back so the bodies are separate (surface gap >= 0.02 m) ---")
for r_ in (0.33, 0.35, 0.40):
    for bx in (0.72, 0.8, 1.0):
        print(check((r_, 0.1, 0.0), ((bx, 0.0, 0.2),)))
print("\n--- sphere just in front of a bar (separate), contact-range band, various lateral offsets ---")
leaks = 0; n = 0
for r_ in np.arange(0.30, 0.46, 0.01):
    for lat in (0.0, 0.05, 0.1, 0.15, 0.2):
        for bx in (r_ + 0.36, r_ + 0.4, r_ + 0.5, r_ + 0.7):
            bars = ((float(bx), lat, 0.2),)
            inp = t.make(target_v=(float(r_), lat, 0.0), detected=True, bars=bars); rep = va.validate(inp, t.CAM, t.LID); n += 1
            if rep.depth_native is not None:
                tn = np.minimum(t._cylinder_hits(cam_o, NR, bars), 10.0); seen = np.isfinite(rep.depth_native)
                if np.abs(rep.depth_native[seen] - tn[seen]).max() > 1e-3:
                    leaks += 1; print("  LEAK r=%.2f lat=%.2f bar=%s dev=%.3f nmin=%.3f" % (r_, lat, bars[0], np.abs(rep.depth_native[seen] - tn[seen]).max(), rep.depth_native[seen].min()))
print("  separate sphere-in-front-of-bar frames:", n, "leaks:", leaks)
