"""Candidate identity rules evaluated on the committed families (identity_attack.py) and mine (attack_v24.py).
Per case: (in, at, near) from the v2.4 record, far = in - at - near, truth true_at. Rules:
  V24: at > near                      S1: at > near and at > far       S2: at > near + far (majority of all pasted in footprint)
  S3: at > near and far == 0 ... too strict, listed for reference.
Usage: python -B rule_eval.py <family> <out.json>  (family prefix W/L = committed, M/RND = mine)"""
import sys, json, collections, time, numpy as np
sys.path.insert(0, "/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_confirm_v24")
import attack_v24 as M
A = M.A
name, out = sys.argv[1], sys.argv[2]
mine = name.startswith(("M", "RND", "L5", "L6"))
gen = M.family(name) if mine else A.family(name)
cnt = collections.Counter(); rows = []; t0 = time.time()
for label, kw in gen:
    if mine:
        res = M.run(**kw)
    else:
        tv = np.asarray(kw["target_v"], float); kw2 = dict(target_v=tv, boxes=kw.get("boxes", ()), cyls=kw.get("cyls", ()),
                                                           track_v=tv + np.asarray(kw.get("shift", (0, 0, 0)), float), cov_pos=kw.get("cov_pos", 0.01), yaw=kw.get("yaw", 0.0))
        res = M.run(**kw2)
    if res["paste"] < 2: continue
    testable = res["ident"] in ("SUPPORTED_BY_FOOTPRINT", "RANGE_CONTRADICTED")   # pasted pixels in footprint, belief testable
    i, a, n = res["pasted_in"], res["pasted_at"], res["pasted_near"]; f = i - a - n
    truth = res["true_at"] > 0
    correct = abs(res["err_range"]) <= 0.3 and res["err_lat"] <= 0.15
    rules = dict(V24=testable and a > n, S1=testable and a > n and a > f, S2=testable and a > n + f)
    cnt["cases"] += 1; cnt["truth"] += truth; cnt["correct"] += correct
    for k, v in rules.items():
        if v and not truth: cnt["false_" + k] += 1
        if truth and not v: cnt["lost_correct_" + k if correct else "lost_wrong_" + k] += 1
    rows.append(dict(label, i=i, a=a, n=n, f=f, true_at=res["true_at"], err_range=round(res["err_range"], 2), err_lat=round(res["err_lat"], 2), ident=res["ident"]))
summary = dict(family=name, counts=dict(cnt), seconds=round(time.time() - t0, 1)); print(json.dumps(summary))
json.dump(dict(summary=summary, rows=rows), open(out, "w"), indent=0, default=float)
