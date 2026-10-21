"""Evaluate candidate minimal fixes for the bbox-corner false SUPPORTED on the attack families (read-only on the repo).
Per case: the v2.3 decision, plus
  A: SUPPORTED iff pasted_at > 0 and no foreign pixel inside the envelope is NEARER than the track interval
     (a pasted surface in front of the track inside its footprint contradicts the range, as v2.1 did for farther pixels);
  B: SUPPORTED iff pasted_at > 0 and pasted_at >= 0.5 * pasted_in (majority of the footprint's pasted pixels at range);
  C: SUPPORTED iff pasted_at > pasted_near (more pasted pixels at range than nearer than it).
Usage: python -B fixeval.py <family> <out.json>"""
import sys, json, collections, time
import numpy as np
sys.path.insert(0, "/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_final_v23")
import attack_c1 as A
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev

SUP = "SUPPORTED_BY_FOOTPRINT"


def evaluate(kw):
    tv = np.asarray(kw["target_v"], float)
    depth, lidar, paste, native = A.scene(tv, kw.get("boxes", ()), kw.get("cyls", ()))
    track_v = tv + np.asarray(kw.get("shift", (0, 0, 0)), float)
    inp = A.t.make(target_v=tuple(track_v), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=kw.get("cov_pos", 0.01), yaw=kw.get("yaw", 0.0))
    r = ev.estimate(inp); rep = va.validate(inp, A.CAM, A.LID); g = ev._belief_geometry(inp, A.CAM, A.LID, A.P)
    d = np.asarray(inp.depth, float); k, tol = A.P.sigma_k, A.P.range_tol_m

    def counts(mask):
        v, u = np.nonzero(mask)
        if v.size == 0:
            return 0, 0, 0
        rays = geo.pixel_rays(u, v, A.CAM)
        inside, entry, exit_, _ = geo.ray_envelope(A.CAM_O, rays, g["p_v"], g["env_cam"])
        dd = d[v, u]; lo = entry - k * g["sigma_r"] - tol; hi = exit_ + k * g["sigma_r"] + tol
        return int(inside.sum()), int((inside & (dd >= lo) & (dd <= hi)).sum()), int((inside & (dd < lo)).sum())

    t_in, t_at, t_near = counts(paste)
    f = rep.depth_pasted if rep.depth_pasted is not None else np.zeros_like(paste)
    f_in, f_at, f_near = counts(f)
    v23 = r.identity_status == SUP
    testable = r.identity_status in (SUP, "RANGE_CONTRADICTED", "CONTRADICTED", "NOT_TESTABLE") and r.camera_detected
    a = testable and f_at > 0 and f_near == 0
    b = testable and f_at > 0 and f_at >= 0.5 * f_in
    c = testable and f_at > f_near
    return dict(ident=r.identity_status, true_in=t_in, true_at=t_at, true_near=t_near, f_in=f_in, f_at=f_at, f_near=f_near,
                v23=v23, A=a, B=b, C=c)


def main(name, out):
    t0 = time.time(); cnt = collections.Counter(); rows = []
    for label, kw in A.family(name):
        res = evaluate(kw); rows.append(dict(label, **res)); cnt["cases"] += 1
        truth = res["true_at"] > 0
        for key in ("v23", "A", "B", "C"):
            if res[key] and not truth:
                cnt["false_" + key] += 1
            if truth and not res[key]:
                cnt["lost_" + key] += 1
    summary = dict(family=name, counts=dict(cnt), seconds=round(time.time() - t0, 1))
    print(json.dumps(summary))
    json.dump(dict(summary=summary, rows=rows), open(out, "w"), indent=0, default=float)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
