import sys, collections, json, math, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
import identity_sweeps as H
J = H.Judge("v26", None); M = H.M; rng = np.random.default_rng(0)
pool = H.Z2 if sys.argv[1] == "z2" else H.Z3
out = []; cnt = collections.Counter()
for R in ((1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0) if pool is H.Z2 else (4.0, 5.0, 6.0, 8.0, 10.0)):
    for d_f in (0.4, 0.7, 1.0, 1.5, 2.5, 3.5):
        for a in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
            for b in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
                gm = M.front_bar_corner(R, d_f, a, b, pool=pool)
                if gm is None: continue
                tv, boxes0, dr = gm
                sc = J.cache(tv, [bx + (0.0,) for bx in boxes0])
                if sc is None or sc["paste"].sum() < 2 or sc["fit_status"] != "OK": continue
                f = sc["fit"]; inl_false = int((f.inlier_mask & ~sc["paste"]).sum()); mask_false = int((f.mask & ~sc["paste"]).sum())
                cerr = float(np.linalg.norm(f.centre - tv))
                o = collections.Counter()
                for rr in np.arange(0.4, 12.01, 0.1):
                    for cov in (0.0025, 0.01, 0.04):
                        trk = H.CAM_O + rr * dr
                        st, g = J.decide(sc, trk, cov); t_at, gap, cin = H.truth(sc, g)
                        if st == H.SUP and t_at == 0 and not cin and not (gap is not None and gap <= 0.05): o["material"] += 1
                if o["material"]:
                    cls = "spurious_winner" if cerr > 0.05 else ("inliers_include_blend" if inl_false else "true_winner_other")
                    cnt[cls] += 1
                    out.append(dict(R=R, d_f=d_f, a=a, b=b, paste=int(sc["paste"].sum()), inliers=f.inliers, inl_false=inl_false, mask_false=mask_false,
                                    centre_err=round(cerr, 3), unexplained=f.farther_unexplained, material=o["material"], cls=cls))
print(sys.argv[1], dict(cnt)); [print(" ", x) for x in out[:12]]
json.dump(out, open(sys.argv[2], "w"))
