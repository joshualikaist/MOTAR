"""Breakdown of attack_v24.py outputs: false SUPPORTED and lost-correct by condition."""
import json, sys, collections
from pathlib import Path
D = Path(sys.argv[1])
fams = sys.argv[2:] or sorted(p.stem for p in D.glob("*.json"))
tot = collections.Counter()
for fam in fams:
    p = D / (fam + ".json")
    if not p.exists():
        print(fam, "MISSING"); continue
    j = json.loads(p.read_text()); rows = [r for r in j["rows"] if r["paste"] >= 2]
    c = j["summary"]["counts"]
    print("%-22s cases %5d flagged %5d  FALSE %4d (near %d far %d)  LOST_correct %4d  LOST_wrong(failsafe) %4d  supported %5d  %ss" % (
        fam, c["cases"], c.get("flagged_cases", 0), c.get("FALSE_SUPPORTED", 0), c.get("FALSE_SUPPORTED_near_track", 0),
        c.get("FALSE_SUPPORTED_far_track", 0), c.get("LOST_correct", 0), c.get("LOST_wrong_track_failsafe", 0), c.get("supported", 0), j["summary"]["seconds"]))
    for k in ("cases", "flagged_cases", "FALSE_SUPPORTED", "LOST_correct", "LOST_wrong_track_failsafe", "correct_tracks", "true_at>0"):
        tot[k] += c.get(k, 0)
    fs = j["false_supported"]; ls = j["lost_correct"]
    if fs:
        keys = [k for k in ("R", "d_f", "x_f", "a", "b", "cover", "delta", "cov", "dR", "er", "el", "ez", "bearing", "elev", "mode", "nb") if k in fs[0]]
        by = collections.Counter(tuple((k, r[k]) for k in keys if k in ("R", "d_f", "cover", "delta", "mode")) for r in fs)
        print("   false by (R,d_f,cover,delta,mode):", dict(list(by.most_common(12))))
        er = collections.Counter(round(r["err_range"], 1) for r in fs); print("   false by err_range:", dict(sorted(er.items())))
        print("   false pasted_at/near/true_at/true_far examples:", [(r["pasted_at"], r["pasted_near"], r["true_at"], r["true_far"], r["fnp_at"]) for r in fs[:8]])
    if ls:
        by = collections.Counter(tuple((k, r[k]) for k in ("R", "d_f", "a", "b", "kind", "cover") if k in r) for r in ls)
        print("   lost-correct by geometry:", dict(list(by.most_common(12))))
        print("   lost examples (pasted_at, pasted_near, true_at, fnp_near, ident):", [(r["pasted_at"], r["pasted_near"], r["true_at"], r["fnp_near"], r["ident"]) for r in ls[:8]])
print("TOTAL", dict(tot))
