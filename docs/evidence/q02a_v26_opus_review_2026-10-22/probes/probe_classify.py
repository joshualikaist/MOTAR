"""Classify accepted-but-offset sphere fits by mechanism: SEED_MISS / TIE_SEP_R_2R / TIE_OTHER / FEWER_TRUE_INLIERS."""
import sys, os, glob
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_common import *
R0 = CAM.target_radius

def true_inliers(sc):
    cand = sc["rep"].depth_pasted; d = sc["d"]; tv = sc["target_v"]
    v, u = np.nonzero(cand); rays = IS.PR[v, u]; t = d[v, u]; pts = CAM_O + t[:, None] * rays
    dist = np.linalg.norm(pts - tv, axis=1); front = np.einsum("ij,ij->i", pts - tv, rays) < 0
    return int(((np.abs(dist - R0) < 5e-3) & front).sum())

def classify(sc):
    f = sc["fit"]; ti = true_inliers(sc); sep = float(np.linalg.norm(f.centre - sc["target_v"]))
    if ti > f.inliers: m = "SEED_MISS"
    elif ti == f.inliers: m = "TIE_SEP_R_2R" if R0 < sep <= 2 * R0 else ("TIE_SEP_LE_R" if sep <= R0 else "TIE_SEP_GT_2R")
    else: m = "FEWER_TRUE_INLIERS"
    return m, ti, sep

if __name__ == "__main__":
    out = collections.defaultdict(collections.Counter); ex = collections.defaultdict(list)
    for p in sorted(glob.glob(sys.argv[1] + "/*.json")):
        j = json.load(open(p)); fam = j["family"]
        for r in j["worst"]:
            tg = r["tag"]; sc = J.cache(np.array(tg["tv"]), [tuple(b) for b in tg["boxes"]], tg["yaw"])
            m, ti, sep = classify(sc)
            out[fam]["offset_" + m] += 1
            if r["worst"]:
                out[fam]["WORST_" + m] += 1; ex[fam].append((m, ti, r["n_in"], round(sep, 3), r["npaste"], round(r["R"], 2)))
    # the committed residual corner scenes
    D = W + "/docs/evidence/q02a_v26_repair_2026-10-22/"
    for f_, pool in (("residual_corner_z2.json", Z2), ("residual_corner_z3.json", Z3)):
        for r in json.load(open(D + f_)):
            tv, boxes0, dr = M.front_bar_corner(r["R"], r["d_f"], r["a"], r["b"], pool=pool)
            sc = J.cache(tv, [bx + (0.0,) for bx in boxes0]); m, ti, sep = classify(sc)
            out["residual_" + f_[9:-5]][m] += 1; ex["residual_" + f_[9:-5]].append((m, ti, sc["fit"].inliers, round(sep, 3), r["paste"], r["R"]))
    for k in out: print(k, dict(out[k])); [print("   ", e) for e in ex[k][:15]]
