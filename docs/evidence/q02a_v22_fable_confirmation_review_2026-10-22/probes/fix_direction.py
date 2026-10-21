"""Candidate minimal fix for the blend-neighbour false SUPPORTED: count for `at` only foreign pixels flagged by BOTH the
row-direction and the column-direction window tests (segment-aligned paste box). Measured on family A (504), B (64), Q01 frames."""
import sys, os, json, collections
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev, contracts as c
P = ev.EvidenceParams()
def split_flags(y, tol=1e-3):
    av, ah = geo._operators(y.shape, (40, 24))[:2]
    fr = np.zeros(y.shape, bool); fc = np.zeros(y.shape, bool)
    for w, proj in geo._pair_windows(av):
        bad = (np.abs(proj @ y[w, :]) > tol).any(axis=0); fr[np.ix_(w, np.flatnonzero(bad))] = True
    for w, proj in geo._pair_windows(ah):
        bad = (np.abs(y[:, w] @ proj.T) > tol).any(axis=1); fc[np.ix_(np.flatnonzero(bad), w)] = True
    return fr, fc
def identity(inp, core_only):
    rep = va.validate(inp, t.CAM, t.LID); d = np.asarray(inp.depth, float)
    f = rep.depth_pasted.copy()
    if core_only:
        fr, fc = split_flags(d); f &= fr & fc
    v, u = np.nonzero(f); rays = geo.pixel_rays(u, v, t.CAM); g = ev._belief_geometry(inp, t.CAM, t.LID, P)
    inside, entry, exit_, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), rays, g["p_v"], g["env_cam"])
    dd = d[v, u]; at = inside & (dd >= entry - P.sigma_k * g["sigma_r"] - P.range_tol_m) & (dd <= exit_ + P.sigma_k * g["sigma_r"] + P.range_tol_m)
    return int(at.sum()), int(inside.sum()), int(f.sum())
def true_paste_at(inp, tv, bars):
    bg = np.asarray(t.make(target_v=tv, render_target=False, bars=bars).depth); d = np.asarray(inp.depth); paste = d != bg
    g = ev._belief_geometry(inp, t.CAM, t.LID, P); pv, pu = np.nonzero(paste); rays = geo.pixel_rays(pu, pv, t.CAM)
    pin, pen, pex, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), rays, g["p_v"], g["env_cam"]); pd = d[pv, pu]
    return int((pin & (pd >= pen - P.sigma_k * g["sigma_r"] - P.range_tol_m) & (pd <= pex + P.sigma_k * g["sigma_r"] + P.range_tol_m)).sum())
cnt = collections.Counter()
for R in (3.0, 5.0, 8.0, 9.5):
    for gap in (0.05, 0.1, 0.2, 0.3, 0.5, 0.8):
        for dR in (0.0, 0.4, -0.4):
            bars = ((R + dR, 0.15 + gap + 0.3, 0.3),)
            for e in (0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.2):
                tv = (R, 0., 0.); inp0 = t.make(target_v=tv, detected=True, bars=bars); mean = np.array(inp0.track_mean); mean[1] += e
                inp = t.make(target_v=tv, detected=True, bars=bars, track_mean=mean)
                tp = true_paste_at(inp, tv, bars); at_now = identity(inp, False)[0]; at_core = identity(inp, True)[0]
                cnt["cases"] += 1
                if tp == 0 and at_now > 0: cnt["false_now"] += 1
                if tp == 0 and at_core > 0: cnt["false_core"] += 1
                if tp > 0 and at_core == 0: cnt["lost_core"] += 1
print("family A:", dict(cnt))
cntb = collections.Counter()
for R in (3.0, 5.0, 8.0, 9.5):
    for gap in (0.05, 0.1, 0.2, 0.5):
        for dR in (0.0, 0.4, -0.4, -3.0):
            tv = (R, 0., 0.); bars = ((R + dR, 0.15 + gap + 0.3, 0.3),); inp = t.make(target_v=tv, detected=True, bars=bars)
            cntb["cases"] += 1; cntb["supported_now"] += identity(inp, False)[0] > 0; cntb["supported_core"] += identity(inp, True)[0] > 0
print("family B (correct track):", dict(cntb))
# plain sweep without bars: core must keep every SUPPORTED
cs = collections.Counter()
for r_ in np.arange(0.5, 19.01, 0.5):
    inp = t.make(target_v=(float(r_), 0., 0.), detected=True); cs["cases"] += 1; cs["now"] += identity(inp, False)[0] > 0; cs["core"] += identity(inp, True)[0] > 0
print("no-bar sweep 0.5..19 m:", dict(cs))
run_root = "/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON"
rows = [json.loads(l) for l in open(os.path.join(run_root, "sensors.jsonl")).read().splitlines()]
ck = lambda tt: c.Clock(tt["capture_sim_s"], tt["delivery_sim_s"], tt["provenance"], tt["frame"], tt["depth_convention"])
for i in range(3):
    z = dict(np.load(os.path.join(run_root, "sensor_%03d.npz" % i), allow_pickle=False))
    inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]), rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
    print("Q01 frame%d: (at, in, foreign) now=%s core=%s" % (i, identity(inp, False), identity(inp, True)))
