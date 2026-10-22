import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_common import *
o = collections.Counter()
for R in (8.0, 10.0, 12.0, 14.0, 16.0, 18.0):
    row = collections.Counter()
    for lat in np.linspace(-0.5, 0.5, 11):
        for z in np.linspace(-0.3, 0.3, 7):
            tv = np.array([R, lat, z]); sc = J.cache(tv, [], 0.0)
            if sc is None or sc["paste"].sum() < 2: row["skip"] += 1; continue
            f = sc["fit"]; row[f.status] += 1
            r = full(sc, tv, 0.01); row["sup" if r.identity_status == SUP else "nosup_" + r.identity_status] += 1
            if R == 12.0 and lat == 0.0 and abs(z) < 1e-9:
                cand = sc["rep"].depth_pasted
                print("R12 centre: paste", int(sc["paste"].sum()), "cand", int(cand.sum()), "fit", f.status, f.inliers, "band", f.band_pixels,
                      "depths", np.round(sc["d"][sc["paste"]], 4), "cand depths", np.round(sc["d"][cand], 4))
    print(R, dict(row))
