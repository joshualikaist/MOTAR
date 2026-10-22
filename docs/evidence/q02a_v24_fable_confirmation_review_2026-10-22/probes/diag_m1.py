"""Mechanism check on one M1 false SUPPORTED case: which foreign pixels inside the envelope are 'at' range?"""
import sys, json, numpy as np
sys.path.insert(0, "/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_confirm_v24")
import attack_v24 as M
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
A, CAM, LID, CAM_O, P = M.A, M.CAM, M.LID, M.CAM_O, M.P
for (R, d_f, a, b, delta, cov) in ((4.0, 1.5, -0.4, -0.4, 0.0, 0.01), (3.0, 1.0, -0.4, -0.4, -0.3, 0.01), (6.0, 1.5, 0.0, 0.0, 0.0, 0.04)):
    geom = M.front_bar_corner(R, d_f, a, b)
    if geom is None: print("infeasible", R, d_f, a, b); continue
    tv, boxes, dr = geom; track = M.track_at_face(tv, dr, R - d_f, delta)
    depth, lidar, paste, native = A.scene(tv, boxes)
    inp = A.t.make(target_v=tuple(track), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov)
    r = ev.estimate(inp); rep = va.validate(inp, CAM, LID); g = ev._belief_geometry(inp, CAM, LID, P)
    f = rep.depth_pasted; v, u = np.nonzero(f); rays = geo.pixel_rays(u, v, CAM)
    inside, entry, exit_, _ = geo.ray_envelope(CAM_O, rays, g["p_v"], g["env_cam"])
    d = np.asarray(inp.depth, float)[v, u]; lo = entry - P.sigma_k * g["sigma_r"] - P.range_tol_m; hi = exit_ + P.sigma_k * g["sigma_r"] + P.range_tol_m
    at = inside & (d >= lo) & (d <= hi); near = inside & (d < lo); far = inside & (d > hi)
    pv, pu = np.nonzero(paste); box = (slice(pv.min(), pv.max() + 1), slice(pu.min(), pu.max() + 1))
    inbox = (v >= pv.min()) & (v <= pv.max()) & (u >= pu.min()) & (u <= pu.max())
    print("case R=%.1f d_f=%.1f a=%.1f b=%.1f delta=%.1f cov=%.3f -> %s %s reasons=%s" % (R, d_f, a, b, delta, cov, r.hypothesis, r.identity_status, r.reasons))
    print("   target range %.2f, track range %.2f (err %.2f), interval on central ray ~[%.2f, %.2f]; bar face x=%.2f" % (np.linalg.norm(tv - CAM_O), g["range"], g["range"] - np.linalg.norm(tv - CAM_O), lo[inside].min() if inside.any() else -1, hi[inside].max() if inside.any() else -1, R - d_f))
    print("   paste px %d, box rows %d..%d cols %d..%d; foreign px %d (in box %d, outside box %d)" % (paste.sum(), pv.min(), pv.max(), pu.min(), pu.max(), f.sum(), (f & np.isin(np.arange(f.size).reshape(f.shape), np.arange(f.size).reshape(f.shape)[box])).sum(), int(f.sum() - f[box].sum())))
    print("   record: in %d at %d near %d | truth(paste only): in %d at %d near %d far %d" % (r.pasted_pixels_in_footprint, r.pasted_pixels_at_track_range, r.pasted_pixels_nearer_than_track, *(int(x) for x in (inside[paste[v, u]].sum(), at[paste[v, u]].sum(), near[paste[v, u]].sum(), far[paste[v, u]].sum()))))
    sel = at & ~paste[v, u]
    print("   'at' pixels that are NOT paste: %d, depths %s (bar face range from camera ~%.2f), all inside tight box: %s" % (sel.sum(), np.round(np.unique(np.round(d[sel], 2)), 2)[:8], (R - d_f - CAM_O[0]) / dr[0], bool(inbox[sel].all())))
    print("   far pixels (paste behind the track): %d, depths %s" % (far.sum(), np.round(np.unique(np.round(d[far], 2)), 2)[:6]))
