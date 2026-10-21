"""Reach of the adversarial smooth bump (false OCCLUDED?), 10-m fill pixels counted as pasted-at-range (false SUPPORTED?),
NOT_CONVERGED reachability, contact guard evaluation."""
import sys, math
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
VV, UU = np.mgrid[0:90, 0:160]
native = np.full((24, 40), 10.0); base = np.clip(t.AV @ native @ t.AH.T, 0, 10)
print("--- adversarial smooth bump A=1.0 L=80 ay=1.0 (passes as obstacle depth), track at 9.9 m ---")
g = np.clip(1 - ((UU - 79.5) / 80) ** 2 - ((VV - 44.5) / 80) ** 2, 0, None) ** 2
depth = np.clip(base - 1.0 * g, 0, 10).astype(np.float32)
for lidar_kw in ({}, dict(lidar=np.full(288, np.nan))):
    for rr in (9.0, 9.5, 9.9):
        inp = t.make(target_v=(rr, 0., 0.), render_target=False, depth=depth, **lidar_kw)
        rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
        nm = float(np.nanmin(rep.depth_native)) if rep.depth_native is not None else float("nan")
        print("  lidar=%s track=%.1f od=%s native_min=%.2f -> %-18s cam=%s reasons=%s" % ("nan" if lidar_kw else "ok", rr, rep.status("obstacle_depth"), nm, r.hypothesis, (r.camera.rays, r.camera.foreground, r.camera.clear), ",".join(r.reasons)[:70]))
print("\n--- 10-m fill pixels in the foreign mask counted as 'pasted depth at track range' ---")
for rr, lat in ((10.0, 0.0), (10.0, 0.5), (10.0, 0.6), (9.8, 0.5), (9.5, 0.5), (6.0, 0.5)):
    base_inp = t.make(target_v=(rr, 0., 0.), detected=True); mean = np.array(base_inp.track_mean); mean[1] += lat
    inp = t.make(target_v=(rr, 0., 0.), detected=True, track_mean=mean)
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    d = np.asarray(inp.depth); f = rep.depth_pasted
    v, u = np.nonzero(f); rays = geo.pixel_rays(u, v, t.CAM)
    g_ = ev._belief_geometry(inp, t.CAM, t.LID, ev.EvidenceParams())
    inside, entry, exit_, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), rays, g_["p_v"], g_["env_cam"])
    dd = d[v, u]; at = inside & (dd >= entry - 0.2 - 0.3) & (dd <= exit_ + 0.2 + 0.3)
    print("  target %.1f m, track lateral err %.1f -> %-18s %-22s in_fp=%d at=%d  [of 'at': %d are 10 m fill, %d are real paste]" % (
        rr, lat, r.hypothesis, r.identity_status, r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, int((at & (dd >= 10 - 1e-4)).sum()), int((at & (dd < 10 - 1e-4)).sum())))
print("\n--- NOT_CONVERGED reachability: 300 mixed smooth+disc pastes ---")
rng = np.random.default_rng(77); from collections import Counter; cnt = Counter(); worst = 0
for k in range(300):
    nat = rng.uniform(0.5, 10.0, (24, 40)); nat[rng.uniform(size=nat.shape) < 0.6] = 10.0
    dep = np.clip(t.AV @ nat @ t.AH.T, 0, 10)
    for _ in range(rng.integers(1, 4)):
        cv, cu, L = rng.uniform(0, 90), rng.uniform(0, 160), rng.uniform(5, 80)
        gg = np.clip(1 - ((UU - cu) / L) ** 2 - ((VV - cv) / (L * rng.uniform(0.3, 1.0))) ** 2, 0, None) ** rng.choice([1, 2, 4])
        dep = dep - rng.uniform(0.001, 3.0) * gg
    if rng.uniform() < 0.5:
        m = (VV - rng.uniform(0, 90)) ** 2 + (UU - rng.uniform(0, 160)) ** 2 < rng.uniform(2, 30) ** 2; dep[m] = rng.uniform(0.5, 9)
    rec = geo.recover_native_depth(np.clip(dep, 0, 10).astype(np.float32), (40, 24)); cnt[rec.status] += 1
print("  ", dict(cnt))
print("\n--- contact guard: production-shaped frames 0.5..9.9 m with bars; min observed native cell ---")
mins = []
for rr in (0.5, 0.6, 0.8, 1.0, 2.0, 4.0, 8.0):
    for bars in ((), ((0.6, 0.5, 0.3),), ((1.0, 0.0, 0.3),), ((3.0, 0.2, 0.4),)):
        inp = t.make(target_v=(rr, 0., 0.), detected=True, bars=bars)
        rep = va.validate(inp, t.CAM, t.LID)
        if rep.depth_native is not None:
            mins.append((rr, bars, round(float(np.nanmin(rep.depth_native)), 3), rep.status("obstacle_depth")))
print("  min native per frame (range, bars, min, od):")
for m_ in mins: print("   ", m_)
