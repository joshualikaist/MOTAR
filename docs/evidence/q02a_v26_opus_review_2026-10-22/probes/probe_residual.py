import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_common import *
cases = [(Z2, 3.0, 0.4, -0.85, -0.85), (Z2, 4.0, 0.4, -0.7, -0.85), (Z2, 4.0, 0.7, -0.7, -0.7), (Z3, 10.0, 0.7, -0.4, -0.85)]
for pool, R, d_f, a, b in cases:
    tv, boxes0, dr = M.front_bar_corner(R, d_f, a, b, pool=pool)
    sc = J.cache(tv, [bx + (0.0,) for bx in boxes0]); f = sc["fit"]; paste = sc["paste"]; cand = sc["rep"].depth_pasted
    d = sc["d"]
    print("== pool", pool, "R", R, "d_f", d_f, a, b, "status", f.status, "n_in", f.inliers, "cerr %.3f" % np.linalg.norm(f.centre - tv),
          "paste", int(paste.sum()), "cand", int(cand.sum()), "cand&paste", int((cand & paste).sum()), "rival", f.rival_inliers, "band", f.band_pixels, "unexpl", f.farther_unexplained)
    # true-sphere evaluation on the candidate set
    v, u = np.nonzero(cand); rays = IS.PR[v, u]; t = d[v, u]; pts = CAM_O + t[:, None] * rays
    dist = np.linalg.norm(pts - tv, axis=1); front = np.einsum("ij,ij->i", pts - tv, rays) < 0
    tin = (np.abs(dist - 0.15) < 5e-3) & front
    oc = tv - CAM_O; s = rays @ oc; perp2 = oc @ oc - s * s; core = (s > 0) & (perp2 <= 0.13 ** 2)
    surf = s - np.sqrt(np.clip(0.0225 - perp2, 0, None)); farther = core & (t > surf + 5e-3)
    winl = f.inlier_mask[v, u]
    print("  true sphere: inliers", int(tin.sum()), "farther-in-core", int(farther.sum()), "of which winner inliers", int((farther & winl).sum()))
    print("  winner inlier pixels (v,u,depth,is_paste):", [(int(a_), int(b_), round(float(d[a_, b_]), 3), bool(paste[a_, b_])) for a_, b_ in zip(*np.nonzero(f.inlier_mask))][:12])
    print("  true paste pixels:", [(int(a_), int(b_), round(float(d[a_, b_]), 3), bool(cand[a_, b_])) for a_, b_ in zip(*np.nonzero(paste))][:12])
    print("  winner centre", np.round(f.centre, 3), "true", np.round(tv, 3), "range winner %.3f true %.3f" % (np.linalg.norm(f.centre - CAM_O), R))
