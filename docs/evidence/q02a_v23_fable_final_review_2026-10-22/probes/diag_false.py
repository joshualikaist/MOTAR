"""Diagnose false-SUPPORTED rows of a family JSON: which foreign non-paste pixels are counted at the track range,
their depths and where they sit relative to the paste bbox. Usage: python -B diag_false.py <family> <family.json> [max]"""
import sys, json
import numpy as np
sys.path.insert(0, "/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_final_v23")
import attack_c1 as A
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev

name, path = sys.argv[1], sys.argv[2]
mx = int(sys.argv[3]) if len(sys.argv) > 3 else 12
data = json.load(open(path))
labels = [{k: v for k, v in r.items() if k in ("R", "dR", "yb", "kind", "er", "el", "cover", "gap", "dRb", "zt", "ez", "yaw", "bearing", "elev", "cov")} for r in data["false_supported"]]
fams = {lab_key: kw for lab_key, kw in ((tuple(sorted(l.items())), kw) for l, kw in A.family(name))}
shown = 0
for lab in labels[:mx]:
    kw = fams[tuple(sorted(lab.items()))]
    tv = np.asarray(kw["target_v"], float)
    depth, lidar, paste, native = A.scene(tv, kw.get("boxes", ()), kw.get("cyls", ()))
    track_v = tv + np.asarray(kw.get("shift", (0, 0, 0)), float)
    inp = A.t.make(target_v=tuple(track_v), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=kw.get("cov_pos", 0.01), yaw=kw.get("yaw", 0.0))
    rep = va.validate(inp, A.CAM, A.LID); r = ev.estimate(inp); g = ev._belief_geometry(inp, A.CAM, A.LID, A.P)
    f = rep.depth_pasted; d = np.asarray(inp.depth, float)
    v, u = np.nonzero(f); rays = geo.pixel_rays(u, v, A.CAM)
    inside, entry, exit_, _ = geo.ray_envelope(A.CAM_O, rays, g["p_v"], g["env_cam"])
    at = inside & (d[v, u] >= entry - A.P.sigma_k * g["sigma_r"] - A.P.range_tol_m) & (d[v, u] <= exit_ + A.P.sigma_k * g["sigma_r"] + A.P.range_tol_m)
    pv, pu = np.nonzero(paste)
    bbox = (pv.min(), pv.max(), pu.min(), pu.max()) if pv.size else None
    cnt = [(int(vv), int(uu), round(float(d[vv, uu]), 3), bool(paste[vv, uu])) for vv, uu in zip(v[at], u[at])]
    print("%s -> %s/%s pasted_at=%d; track range %.2f sigma_r %.2f interval ~[%.2f, %.2f]; paste bbox rows %s; true paste depths %s" % (
        lab, r.hypothesis, r.identity_status, r.pasted_pixels_at_track_range, g["range"], g["sigma_r"],
        float((entry - 2 * g["sigma_r"] - 0.3)[inside].min()) if inside.any() else float("nan"),
        float((exit_ + 2 * g["sigma_r"] + 0.3)[inside].max()) if inside.any() else float("nan"), bbox,
        np.round(np.unique(d[paste])[:4], 3) if pv.size else "none"))
    print("    counted pixels (row, col, depth, is_paste):", cnt[:12])
    shown += 1
print("shown", shown, "of", len(labels))
