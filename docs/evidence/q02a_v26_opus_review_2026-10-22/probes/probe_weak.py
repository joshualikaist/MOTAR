import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_classify import *
import probe_families as PF
seed = int(sys.argv[1]); rng = np.random.default_rng(7000 + seed); PF.rng = rng
o = collections.Counter(); tries = 0
while o["scenes"] < 1500 and tries < 30000:
    tries += 1; tv, boxes = PF.gen_random(rng); yaw = float(rng.uniform(-math.pi, math.pi))
    sc = J.cache(tv, boxes, yaw)
    if sc is None or sc["paste"].sum() < 2: continue
    o["scenes"] += 1; st = sc["fit"].status
    if st in (geo.SPHERE_WEAK, geo.SPHERE_NONE, geo.SPHERE_VETOED):
        ti = true_inliers(sc); o[st] += 1; o[st + "_true_inliers_ge3"] += ti >= 3; o[st + "_paste_le2"] += int(sc["paste"].sum()) <= 2
print(seed, dict(o))
