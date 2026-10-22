"""Corner family (2 m and 3 m pools) with classified false SUPPORTED over along-ray and lateral track sweeps."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from fast2 import *
out = sys.argv[1]; pool_name = sys.argv[2]; pool = Z2 if pool_name == "z2" else Z3
rng = np.random.default_rng(21 if pool_name == "z2" else 22)
tot = collections.Counter(); scenes = []; t0 = time.time()
for R in ((1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0) if pool_name == "z2" else (4.0, 5.0, 6.0, 8.0, 10.0)):
    for d_f in (0.4, 0.7, 1.0, 1.5, 2.5, 3.5):
        for a in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
            for b in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
                gm = M.front_bar_corner(R, d_f, a, b, pool=pool)
                if gm is None: continue
                tv, boxes0, dr = gm
                boxes = [bx + (0.0,) for bx in boxes0]
                sc = cache(tv, boxes, 0.0)
                if sc is None or sc["paste"].sum() < 2: tot["noflag"] += 1; continue
                tot["scenes"] += 1; w = winner(sc); tot["win_" + w] += 1
                o, ex = sweep2(sc, lats=(0.0, 0.2), rng=rng)
                for k, v in o.items(): tot[k] += v
                if o["FALSE"]: tot["scenes_with_FALSE"] += 1
                if o["FALSE_MATERIAL"]: tot["scenes_with_MATERIAL"] += 1
                if o["LOST_correct"]: tot["scenes_with_LOST"] += 1
                ers = [e["er"] for e in ex]
                scenes.append(dict(R=R, d_f=d_f, a=a, b=b, paste=int(sc["paste"].sum()), cand=int(sc["rep"].depth_pasted.sum()), winner=w, nin=sc["nin"],
                                   sweep=dict(o), material_er=(min(ers), max(ers)) if ers else None, ex=ex[:4]))
print(pool_name, dict(tot), round(time.time() - t0, 1))
for s in [s for s in scenes if s["sweep"].get("FALSE_MATERIAL")][:12]: print(" M", {k: s[k] for k in ("R", "d_f", "a", "b", "paste", "cand", "winner", "nin", "material_er")}, s["sweep"].get("FALSE_MATERIAL"))
json.dump(dict(total=dict(tot), scenes=scenes), open(out, "w"), default=float)
