import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_classify import *
o = collections.Counter()
for R in (12.0, 14.0, 16.0, 18.0):
    for lat in np.linspace(-0.5, 0.5, 11):
        for z in np.linspace(-0.3, 0.3, 7):
            tv = np.array([R, lat, z]); sc = J.cache(tv, [], 0.0)
            if sc is None or sc["paste"].sum() < 2: continue
            f = sc["fit"]
            if f.status == geo.SPHERE_WEAK:
                ti = true_inliers(sc); o["weak"] += 1; o["weak_true_inliers_ge3"] += ti >= 3; o["weak_paste_ge3"] += int(sc["paste"].sum()) >= 3
print(dict(o))
