"""Full-path verification of a spurious-sphere false SUPPORTED (front 2 m bar corner, M1/L5 geometry)."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from common import *
R, d_f, a, b = 3.0, 0.4, -0.7, -0.7
tv, boxes, dr = M.front_bar_corner(R, d_f, a, b)
depth, lidar, paste, native = A.scene(tv, boxes)
print("target", np.round(tv, 3), "bar", boxes, "true paste px", int(paste.sum()))
inp0 = A.t.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar.reshape(-1))
rep = va.validate(inp0, CAM, LID)
cand = rep.depth_pasted; d = np.asarray(inp0.depth, float)
c, mask, st, nin = geo.fit_target_sphere(inp0.depth, cand, CAM, CAM.target_radius)
print("candidates", int(cand.sum()), "of which true", int((cand & paste).sum()), "fit", st, "inliers", nin, "centre", np.round(c, 3), "range of centre", round(float(np.linalg.norm(c - CAM_O)), 3))
print("mask pixels (v,u,depth,true?)", [(int(v), int(u), round(float(d[v, u]), 3), bool(paste[v, u])) for v, u in zip(*np.nonzero(mask))])
# score of the true sphere, computed exactly as fit_target_sphere scores a sphere
v, u = np.nonzero(cand); rays = geo.pixel_rays(u, v, CAM); t = d[v, u]; pts = CAM_O + t[:, None] * rays
def score(cc):
    dist = np.linalg.norm(pts - cc, axis=1); front = np.einsum("ij,ij->i", pts - cc, rays) < 0
    n_in = int(((np.abs(dist - 0.15) < 5e-3) & front).sum())
    oc = cc - CAM_O; s = rays @ oc; perp2 = oc @ oc - s * s
    core = (s > 0) & (perp2 <= (0.15 - 0.02) ** 2); surface = s - np.sqrt(np.clip(0.0225 - perp2, 0, None))
    return n_in, int((core & (t > surface + 5e-3)).sum())
print("true sphere (inliers, violations):", score(tv), " fitted sphere:", score(c))
viol = [(int(vv), int(uu), round(float(tt), 2)) for vv, uu, tt, pp in zip(v, u, t, paste[v, u]) if not pp]
print("non-target candidate pixels (v,u,depth):", viol[:40])
for rr, cov in ((3.0, 0.01), (6.3, 0.04), (6.6, 0.01), (2.6, 0.01)):
    trk = CAM_O + rr * dr
    r = M.run(target_v=tv, boxes=boxes, track_v=trk, cov_pos=cov)
    print("track %.1f m cov %.4f -> %s %s | true_in %d true_at %d true_near %d | target px in fp %s at range %s | reasons %s" % (
        rr, cov, r["hyp"], r["ident"], r["true_in"], r["true_at"], r["true_near"], "-", "-", r["reasons"]))
    inp = A.t.make(target_v=tuple(trk), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov)
    e = ev.estimate(inp); print("    record: target_sphere_found", e.target_sphere_found, "target_pixels_in_footprint", e.target_pixels_in_footprint, "at_range", e.target_pixels_at_track_range, "pasted_in", e.pasted_pixels_in_footprint)
