"""Docstring claim: 'an occluder thinner than the native grid does not count against the true sphere'.
Target behind a sub-native occluder, with and without a non-fill background."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from common import *
cnt = collections.Counter(); ex = []
for R in (2.0, 3.0, 4.0, 6.0):
    for xo_frac in (0.4, 0.6, 0.8):
        for rc in (0.02, 0.03, 0.04, 0.06):
            for yo in (0.0, 0.05, -0.08):
                for bg in (None, 1.0, 2.5):
                    xo = xo_frac * R
                    cyls = [(xo, yo * xo_frac, rc, None, None)]
                    boxes = [] if bg is None else [(R + bg + 0.3, 0.0, 0.6, 0.6, -1.0, 2.0)]
                    tv = np.array([R, 0.0, 0.0])
                    r = detail(tv, boxes=boxes, cyls=cyls)
                    if r["paste"] < 2: continue
                    key = "bg" if bg is not None else "nobg"
                    cnt[key + "_cases"] += 1
                    sup = r["ident"] == "SUPPORTED_BY_FOOTPRINT"
                    cnt[key + "_lost"] += (not sup)
                    cnt[key + "_fit_" + r["fit"]] += 1
                    if not sup and len(ex) < 10: ex.append(dict(R=R, xo=xo_frac, rc=rc, yo=yo, bg=bg, paste=r["paste"], cand=r["cand"], cand_true=r["cand_true"], fit=r["fit"], nin=r["fit_inliers"], ident=r["ident"]))
print(dict(cnt)); [print(e) for e in ex]
