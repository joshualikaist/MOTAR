"""R3: correct-track sweep must stay SUPPORTED; look for false SUPPORTED / false RANGE_CONTRADICTED."""
import sys, math, json
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
rng = np.random.default_rng(5)
fails = []; n = 0
# correct tracks, random bearing/elevation inside FOV, ranges 0.5..19 m, with 0..2 sigma track noise
for k in range(400):
    r_ = float(rng.choice([0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 9.5, 9.9, 10.2, 12.0, 15.0, 19.0]))
    az = math.radians(rng.uniform(-35, 35)); el = math.radians(rng.uniform(-22, 22))
    tv = (r_ * math.cos(el) * math.cos(az), r_ * math.cos(el) * math.sin(az), r_ * math.sin(el))
    base = t.make(target_v=tv, detected=True)
    mean = np.array(base.track_mean); sig = 0.1
    noise = rng.uniform(-2, 2, 3) * sig if k % 2 else np.zeros(3)
    mean[:3] += noise
    inp = t.make(target_v=tv, detected=True, track_mean=mean)
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    n += 1
    paste = (np.asarray(inp.depth) != np.asarray(t.make(target_v=tv, render_target=False).depth)).sum()
    if r.hypothesis != ev.TARGET_VISIBLE or r.identity_status != "SUPPORTED_BY_FOOTPRINT":
        fails.append(dict(k=k, range=r_, az=math.degrees(az), el=math.degrees(el), noise=noise.round(3).tolist(), paste=int(paste),
                          hyp=r.hypothesis, ident=r.identity_status, reasons=list(r.reasons), fov_in=r.fov_points_inside,
                          in_fp=r.pasted_pixels_in_footprint, at=r.pasted_pixels_at_track_range, problems=list(r.validation_problems),
                          ds=rep.status("depth_structure"), od=rep.status("obstacle_depth")))
print("correct-track cases", n, "not SUPPORTED:", len(fails))
for f in fails:
    print("  ", f)
# partially occluded target (bar edge covers half of it) with flag
print("\n--- partially occluded target, flag set ---")
for bar_y in (0.25, 0.15, 0.05, -0.05):
    inp = t.make(target_v=(6., 0., 0.), detected=True, bars=((4., bar_y, 0.3),))
    r = ev.estimate(inp)
    print("  bar_y=%.2f -> %-18s %-22s in_fp=%d at=%d reasons=%s" % (bar_y, r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, ",".join(r.reasons)[:90]))
# track range error sweep (flag set, target at 6 m): where does SUPPORTED turn into RANGE_CONTRADICTED?
print("\n--- track range error sweep, target 6 m, flag ---")
for err in (-1.5, -1.0, -0.8, -0.6, -0.5, -0.4, -0.3, 0.0, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.5):
    base = t.make(target_v=(6., 0., 0.), detected=True); mean = np.array(base.track_mean); mean[0] += err
    r = ev.estimate(t.make(target_v=(6., 0., 0.), detected=True, track_mean=mean))
    print("  err=%+.1f -> %-18s %-22s in_fp=%d at=%d" % (err, r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range))
# lateral error sweep
print("\n--- track lateral error sweep, target 6 m, flag ---")
for err in (0.0, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8):
    base = t.make(target_v=(6., 0., 0.), detected=True); mean = np.array(base.track_mean); mean[1] += err
    r = ev.estimate(t.make(target_v=(6., 0., 0.), detected=True, track_mean=mean))
    print("  lat=%+.1f -> %-18s %-22s in_fp=%d at=%d reasons=%s" % (err, r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, ",".join(r.reasons)[:60]))
# diffuse belief in the flag path (no cap applied there)
print("\n--- diffuse belief with flag (cap max_envelope_extra not applied in flag path?) ---")
for cp in (0.01, 0.25, 1.0, 4.0):
    base = t.make(target_v=(6., 0., 0.), detected=True, cov_pos=cp); mean = np.array(base.track_mean); mean[1] += 1.0; mean[0] += 1.0
    r = ev.estimate(t.make(target_v=(6., 0., 0.), detected=True, cov_pos=cp, track_mean=mean))
    print("  cov_pos=%.2f (sigma %.2f) track off by (1,1) m -> %-18s %-22s env_cam=%.2f at=%d reasons=%s" % (cp, math.sqrt(cp), r.hypothesis, r.identity_status, r.camera_envelope_m, r.pasted_pixels_at_track_range, ",".join(r.reasons)[:60]))
# stale track with flag
base = t.make(target_v=(6., 0., 0.), detected=True, age=5.0)
r = ev.estimate(base); print("\nstale track (age 5 s) + flag ->", r.hypothesis, r.identity_status, r.reasons)
# target beyond 10 m pasted (>10 m depth values present)
for rr in (10.2, 12.0, 19.0):
    inp = t.make(target_v=(rr, 0., 0.), detected=True); rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    print("target %.1f m flag: depth max %.2f ds=%s od=%s -> %s %s at=%d" % (rr, float(np.asarray(inp.depth).max()), rep.status("depth_structure"), rep.status("obstacle_depth"), r.hypothesis, r.identity_status, r.pasted_pixels_at_track_range))
    r2 = ev.estimate(t.make(target_v=(rr, 0., 0.), detected=False)); print("   no flag ->", r2.hypothesis, r2.reasons)
