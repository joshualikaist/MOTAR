"""Author's independent reproduction of the Opus final-review findings G1 and G2 on Q02A-v2.5 (5fc64a3).

G1: the reviewer's L5 example (target 3 m, 2 m bar 0.4 m in front, corner offsets a=b=-0.7). The fitted "sphere" is
spurious (two blend pixels at 7.3 m), so the correct track is RANGE_CONTRADICTED and wrong tracks at the blend depth are
SUPPORTED with no target pixel at range. G2: a box-shaped target (physical-dynamics half extents 0.14/0.14/0.06 m)
is accepted as a sphere and SUPPORTED, contrary to the v2.5 record. CPU only; writes nothing.
Usage: python -B docs/evidence/q02a_v25_opus_final_review_2026-10-22/g1_g2_reproduction.py
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "q02a_v25_repair_2026-10-22"))
import identity_attack_v24_families as M
from motar_eval.q02 import evidence as ev, geometry as geo, validation as va

A, CAM, LID, CAM_O = M.A, M.CAM, M.LID, M.CAM_O
tv, boxes, dr = M.front_bar_corner(3.0, 0.4, -0.7, -0.7, pool=M.Z2)
depth, lidar, paste, _ = A.scene(tv, boxes, ())
d = np.asarray(depth, float)
inp0 = A.t.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar.reshape(-1))
rep = va.validate(inp0, CAM, LID)
c, mask, status, n_in = geo.fit_target_sphere(d, rep.depth_pasted, CAM, CAM.target_radius)
print("G1 paste px %d | fitted centre %.2f m from the target, inliers %d, mask %d (true paste %d), mask depths %s" % (
    paste.sum(), np.linalg.norm(c - tv), n_in, mask.sum(), (mask & paste).sum(), np.round(d[mask], 2)))
print("G1 correct track:", ev.estimate(inp0).identity_status)
for rt, cov in ((6.3, 0.04), (6.6, 0.01), (5.0, 0.01)):
    r = ev.estimate(A.t.make(target_v=tuple(CAM_O + rt * dr), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov))
    print("G1 track %.1f m on the target ray (cov %.2f): %s %s target_at=%d" % (rt, cov, r.hypothesis, r.identity_status,
                                                                             r.target_pixels_at_track_range))
vv, uu = np.mgrid[0:90, 0:160]
bt, h = np.array([4.0, 0.2, 0.0]), np.array([0.14, 0.14, 0.06])
tb = A.box_hits(CAM_O, geo.pixel_rays(uu, vv, CAM), [(bt[0], bt[1], h[0], h[1], bt[2] - h[2], bt[2] + h[2])])
bdepth = np.where(np.isfinite(tb) & (tb < 10), tb, np.clip(A.t.AV @ np.full((24, 40), 10.0) @ A.t.AH.T, 0, 10)).astype(np.float32)
r = ev.estimate(A.t.make(target_v=tuple(bt), detected=True, depth=bdepth))
print("G2 box-shaped target 4 m, correct track:", r.identity_status, "| sphere found:", r.target_sphere_found)
