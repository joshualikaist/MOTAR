"""Detail of one case: fit internals and how far the true pixels are from the track interval."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from sweep import *
def gaps(sc, track_v, cov):
    inp = A.t.make(target_v=tuple(track_v), detected=True, depth=sc["depth"], lidar=sc["lidar"].reshape(-1), cov_pos=cov)
    g = ev._belief_geometry(inp, CAM, LID, P)
    out = {}
    for name, m in (("true", sc["paste"]), ("mask", sc["mask"])):
        v, u = np.nonzero(m)
        rays = geo.pixel_rays(u, v, CAM)
        inside, entry, exit_, perp = geo.ray_envelope(CAM_O, rays, g["p_v"], g["env_cam"])
        dd = sc["d"][v, u]; k, tol = P.sigma_k, P.range_tol_m
        lo, hi = entry - k * g["sigma_r"] - tol, exit_ + k * g["sigma_r"] + tol
        out[name] = [(int(a), int(b), round(float(x), 3), round(float(l), 3), round(float(h), 3), bool(i), round(float(p), 3), bool(pp)) for a, b, x, l, h, i, p, pp in zip(v, u, dd, lo, hi, inside, perp, sc["paste"][v, u])]
    return out
# vedge R=6, d_f=2.5, cover 0.85 ; track 6.5 cov 0.0025
R, d_f, cover = 6.0, 2.5, 0.85
x_f = R - d_f; yt = (1 - 2 * cover) * CAM.target_radius; tv = np.array([R, yt, CAM_O[2]])
sc = scene_cache(tv, [(x_f + 0.3, -0.3, 0.3, 0.3, Z3[0], Z3[1])])
dr = (tv - CAM_O) / np.linalg.norm(tv - CAM_O)
print("winner", winner(sc), "nin", sc["nin"], "centre err", np.linalg.norm(sc["c"] - tv))
for rr, cov in ((6.5, 0.0025), (6.7, 0.01), (7.1, 0.04)):
    o = gaps(sc, CAM_O + rr * dr, cov)
    print("track %.1f cov %.4f" % (rr, cov)); print("  true (v,u,depth,lo,hi,inside,perp,paste):", o["true"]); print("  mask:", o["mask"])
# limb sliver loss: vedge R=3, d_f=1.0, cover 0.9
R, d_f, cover = 3.0, 1.0, 0.9
x_f = R - d_f; yt = (1 - 2 * cover) * CAM.target_radius; tv = np.array([R, yt, CAM_O[2]])
sc = scene_cache(tv, [(x_f + 0.3, -0.3, 0.3, 0.3, Z3[0], Z3[1])])
v, u = np.nonzero(sc["rep"].depth_pasted); d = sc["d"]
rays = geo.pixel_rays(u, v, CAM); pts = CAM_O + d[v, u][:, None] * rays
print("limb case: cand", len(v), "status", sc["st"], "pixels", list(zip(v.tolist(), u.tolist())))
tc = tv
print("  true-centre residuals", np.round(np.linalg.norm(pts - tc, axis=1) - 0.15, 6), "front (p-c).ray", np.round(np.einsum("ij,ij->i", pts - tc, rays), 4))
# Evaluate the true sphere's score as fit_target_sphere would
oc = tc - CAM_O; s = rays @ oc; perp2 = oc @ oc - s * s
print("  true sphere: inliers", int(((np.abs(np.linalg.norm(pts - tc, axis=1) - 0.15) < 5e-3) & (np.einsum('ij,ij->i', pts - tc, rays) < 0)).sum()))
