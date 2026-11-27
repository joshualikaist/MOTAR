"""Q01-V2 frames: are the foreign pixels real target pixels? Compactness, depth spread vs predicted range (read-only)."""
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
for i in range(3):
    z = dict(np.load(os.path.join(run_root, "sensor_%03d.npz" % i), allow_pickle=False))
    inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]), rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp); d = np.asarray(inp.depth); f = rep.depth_pasted
    v, u = np.nonzero(f)
    g = ev._belief_geometry(inp, t.CAM, t.LID, ev.EvidenceParams())
    uu, vv, _, _ = geo.project(geo.vehicle_to_camera(g["p_v"], t.CAM)[None], t.CAM)
    comps = geo._components(f)
    print("frame%d predicted range %.2f m (cam %.2f), predicted pixel (u=%.1f, v=%.1f); foreign %d px in %d component(s): cols %d..%d rows %d..%d; depths %s" % (
        i, r.predicted_range_m, g["r_cam"], uu[0], vv[0], f.sum(), len(comps), u.min(), u.max(), v.min(), v.max(), np.round(np.sort(d[v, u]), 3)))
    # 3x3 neighbourhood depths around the blob: is the blob a depth discontinuity against its surroundings?
    r0, r1, c0, c1 = max(v.min() - 1, 0), min(v.max() + 2, 90), max(u.min() - 1, 0), min(u.max() + 2, 160)
    patch = d[r0:r1, c0:c1]; ring = np.ones(patch.shape, bool); ring[1:-1, 1:-1] = False
    print("   surrounding ring depths: min %.3f max %.3f ; expected sphere depth band [%.3f, %.3f]" % (patch[ring].min(), patch[ring].max(), g["r_cam"] - 0.15 - 2 * g["sigma_r"], g["r_cam"] + 2 * g["sigma_r"]))
print("Q01 mtimes unchanged:", before == {p: os.stat(os.path.join(run_root, p)).st_mtime_ns for p in os.listdir(run_root)})
