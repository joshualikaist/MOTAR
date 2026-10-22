import sys, collections, math, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import identity_sweeps as H
J = H.Judge("v26", None); M = H.M
cnt = collections.Counter()
def tally(sc, tag):
    if sc is None or sc["paste"].sum() < 2 or sc["fit_status"] != "OK": return
    f = sc["fit"]; cnt[tag + "_ok"] += 1
    spur = float(np.linalg.norm(f.centre - sc["target_v"])) > 0.05 or bool((f.inlier_mask & ~sc["paste"]).any())
    cnt[tag + "_spurious_or_mixed"] += spur
    for N in (6, 10):
        flip = f.inliers < N and f.farther_unexplained > 0
        cnt["%s_flip_N%d" % (tag, N)] += flip; cnt["%s_flip_N%d_true_winner" % (tag, N)] += flip and not spur; cnt["%s_flip_N%d_catches_spurious" % (tag, N)] += flip and spur
for pool, tag, Rs in ((H.Z2, "z2", (1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0)), (H.Z3, "z3", (4.0, 5.0, 6.0, 8.0, 10.0))):
    for R in Rs:
        for d_f in (0.4, 0.7, 1.0, 1.5, 2.5, 3.5):
            for a in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
                for b in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
                    gm = M.front_bar_corner(R, d_f, a, b, pool=pool)
                    if gm is None: continue
                    tally(J.cache(gm[0], [bx + (0.0,) for bx in gm[1]]), tag)
# random scenes seed 4 (same generator as identity_sweeps.random_scenes, fit only)
import types
J2 = H.Judge("v26", None); seen = []
orig = H.sweep
H.sweep = lambda J_, sc, o, rng, lats=(0.0, 0.2): seen.append(sc)
H.random_scenes(J2, 4, 1500)
for sc in seen: tally(sc, "rnd4")
print(dict(sorted(cnt.items())))
