"""Foreign-mask composition (synthetic + Q01 frames, read-only), REPAIR numbers, 1-D completeness, R6 spacing."""
import sys, math, json, os
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev, contracts as c
print("--- synthetic: foreign pixels that are background (10 m) vs pasted ---")
for r_ in (1.0, 2.0, 5.0, 8.0, 9.5):
    inp = t.make(target_v=(r_, 0., 0.), detected=True)
    bg = np.asarray(t.make(target_v=(r_, 0., 0.), render_target=False).depth)
    paste = np.asarray(inp.depth) != bg
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    f = rep.depth_pasted
    print("  r=%.1f paste=%4d foreign=%4d  foreign&paste=%4d  foreign&background=%4d  paste not foreign=%d  in_fp=%d at=%d excluded=%.3f" % (
        r_, paste.sum(), f.sum(), (f & paste).sum(), (f & ~paste).sum(), (paste & ~f).sum(), r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, rep.depth_excluded.mean()))
run_root = "/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON"
before = {p: os.stat(os.path.join(run_root, p)).st_mtime_ns for p in os.listdir(run_root)}
rows = [json.loads(l) for l in open(os.path.join(run_root, "sensors.jsonl")).read().splitlines()]
ck = lambda tt: c.Clock(tt["capture_sim_s"], tt["delivery_sim_s"], tt["provenance"], tt["frame"], tt["depth_convention"])
print("\n--- Q01-V2 frames (read-only) ---")
for i in range(3):
    z = dict(np.load(os.path.join(run_root, "sensor_%03d.npz" % i), allow_pickle=False))
    inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]), rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
    rec = geo.recover_native_depth(np.asarray(inp.depth), (40, 24))
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    seen = np.isfinite(rec.native)
    d = np.asarray(inp.depth)
    f = rec.foreign
    print("  frame%d status=%s residual=%.2e fill_cells=%d unobserved=%d excluded=%.3f foreign=%d (foreign at 10m: %d, foreign <10m: %d) depth<10 px=%d  hyp=%s ident=%s in_fp=%d at=%d flag=%s problems=%s" % (
        i, rec.status, rec.residual, int((rec.native[seen] >= 10 - 1e-3).sum()), int((~seen).sum()), rec.excluded.mean(), f.sum(), int((f & (d >= 10 - 1e-4)).sum()), int((f & (d < 10 - 1e-4)).sum()), int((d < 10 - 1e-4).sum()),
        r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, inp.camera_detected, r.validation_problems))
    print("     range %.2f m sigma_r %.3f sigma_lat %.3f env_cam %.3f cam=%s lid=%s" % (r.predicted_range_m, r.sigma_range_m, r.sigma_lateral_m, r.camera_envelope_m, r.camera, r.lidar))
after = {p: os.stat(os.path.join(run_root, p)).st_mtime_ns for p in os.listdir(run_root)}
print("  Q01 mtimes unchanged:", before == after)
# 1-D completeness numeric check
print("\n--- 1-D window-test completeness (numeric) ---")
rng = np.random.default_rng(1)
for name, a in (("rows 90<-24", t.AV), ("cols 160<-40", t.AH)):
    wins = geo._pair_windows(a)
    segs = geo._segments(a)
    lens = [len(s) for s in segs]
    fp = fn = 0
    for k in range(2000):
        x = rng.uniform(0, 10, a.shape[1]); y = a @ x
        if any((np.abs(p @ y[w]) > 1e-3).any() for w, p in wins): fp += 1
        y2 = y.copy(); j = rng.integers(0, a.shape[0]); y2[j] += rng.choice([-1, 1]) * rng.uniform(3e-3, 5)
        if not any((np.abs(p @ y2[w]) > 1e-3).any() for w, p in wins): fn += 1
    # minimum detectable single-pixel jump per output index
    mins = []
    for j in range(a.shape[0]):
        best = 0.0
        for w, p in wins:
            if j in w:
                e = np.zeros(len(w)); e[list(w).index(j)] = 1.0
                best = max(best, np.abs(p @ e).max())
        mins.append(1e-3 / best)
    print("  %s: segments=%d lengths=%s windows=%d  false flags on pure upsamples=%d/2000  missed single-pixel jumps(>=3e-3)=%d/2000  min detectable single-px jump: %.2e..%.2e" % (
        name, len(segs), sorted(set(lens)), len(wins), fp, fn, min(mins), max(mins)))
# R6 spacing
fxn = 40 / (2 * math.tan(math.radians(87) / 2)); fyn = 24 / (2 * math.tan(math.radians(58) / 2))
print("\n--- R6: native fx=%.2f fy=%.2f centre spacing %.2f x %.2f deg; edge spacing %.2f x %.2f deg; mean %.2f x %.2f" % (
    fxn, fyn, math.degrees(math.atan(1 / fxn)), math.degrees(math.atan(1 / fyn)),
    math.degrees(math.atan(20 / fxn) - math.atan(19 / fxn)), math.degrees(math.atan(12 / fyn) - math.atan(11 / fyn)), 87 / 40, 58 / 24))
rays = geo.native_camera_rays(t.CAM, (40, 24))
ang_h = math.degrees(math.acos(np.clip(rays[12, 20] @ rays[12, 19], -1, 1))); ang_v = math.degrees(math.acos(np.clip(rays[12, 20] @ rays[11, 20], -1, 1)))
print("    from native_camera_rays: centre neighbours %.2f (h) x %.2f (v) deg" % (ang_h, ang_v))
