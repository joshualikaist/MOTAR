"""D7 gates on the Q01-V2 frames (read-only): age, sigma_r, extra; v2.2 results; mtimes unchanged."""
import sys, os, json
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev, contracts as c
run_root = "/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON"
before = {p: os.stat(os.path.join(run_root, p)).st_mtime_ns for p in os.listdir(run_root)}
rows = [json.loads(l) for l in open(os.path.join(run_root, "sensors.jsonl")).read().splitlines()]
ck = lambda tt: c.Clock(tt["capture_sim_s"], tt["delivery_sim_s"], tt["provenance"], tt["frame"], tt["depth_convention"])
P = ev.EvidenceParams()
for i in range(3):
    z = dict(np.load(os.path.join(run_root, "sensor_%03d.npz" % i), allow_pickle=False))
    inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]), rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
    g = ev._belief_geometry(inp, t.CAM, t.LID, P)
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    d = np.asarray(inp.depth); f = rep.depth_pasted
    print("frame%d flag=%s age=%.3f sigma_r=%.3f extra=%.3f (gates %.1f/%.1f/%.1f) -> %s %s in=%d at=%d | foreign=%d (at 10 m: %d, <10: %d) depth<10 px=%d od=%s contact=%s warn=%s problems=%s" % (
        i, inp.camera_detected, inp.track_age_s, g["sigma_r"], g["extra"], P.max_belief_age_s, P.max_range_sigma_m, P.max_envelope_extra_m,
        r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, f.sum(), int((f & (np.abs(d - 10) <= 1e-3)).sum()), int((f & (d < 10 - 1e-3)).sum()), int((d < 10 - 1e-3).sum()),
        rep.status("obstacle_depth"), rep.camera_contact_range, r.near_obstacle_warning, r.validation_problems))
    nat = rep.depth_native; seen = np.isfinite(nat)
    print("   native: observed %d/960, min %.3f, cells<10: %d; residual %.2e" % (seen.sum(), nat[seen].min(), int((nat[seen] < 10 - 1e-3).sum()), rep.depth_residual))
after = {p: os.stat(os.path.join(run_root, p)).st_mtime_ns for p in os.listdir(run_root)}
print("Q01 mtimes unchanged:", before == after)
