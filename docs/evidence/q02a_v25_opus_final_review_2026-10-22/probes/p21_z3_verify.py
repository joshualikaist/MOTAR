import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from common import *
for (R, d_f, a, b) in ((5.0, 0.7, -0.7, -0.7), (5.0, 1.0, -0.4, -0.7), (6.0, 0.7, -0.7, -0.2)):
    tv, boxes, dr = M.front_bar_corner(R, d_f, a, b, pool=Z3)
    out = []
    for dR in (0.0, 1.5, 2.0, 2.5, 3.0):
        for cov in (0.01, 0.04):
            r = M.run(target_v=tv, boxes=boxes, track_v=CAM_O + (R + dR) * dr, cov_pos=cov)
            out.append((dR, cov, r["ident"][:9], r["true_at"], r["paste"]))
    print("Z3 R %.1f d_f %.1f a %.1f b %.1f:" % (R, d_f, a, b), out)
