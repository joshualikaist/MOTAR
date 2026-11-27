import json, collections
from pathlib import Path
D = Path("/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_confirm_v24/mine")
def load(f): return json.loads((D / (f + ".json")).read_text())
for fam, keys in (("M1_near_corner", ("a", "b", "cov", "R", "d_f")), ("M1b_near_corner_far", ("a", "b", "cov", "R", "x_f")), ("M4_far_diag", ("el", "ez", "cov", "R", "dR", "er"))):
    j = load(fam); rows = [r for r in j["rows"] if r["paste"] >= 2]; fs = j["false_supported"]
    print("==", fam, "flagged", len(rows), "false", len(fs))
    for k in keys:
        tot = collections.Counter(r[k] for r in rows); bad = collections.Counter(r[k] for r in fs)
        print("   by %-4s" % k, {v: "%d/%d" % (bad[v], tot[v]) for v in sorted(tot)})
    # corner-inside-disc subset (|a|<1,|b|<1 all are); count cases where the bar actually hides part of the target: paste < full disc
    print("   paste sizes of false cases:", sorted(collections.Counter(r["paste"] for r in fs).items())[:12])
    print("   pasted_at of false cases:", sorted(collections.Counter(r["pasted_at"] for r in fs).items()))
    print("   hyp of false cases:", collections.Counter(r["hyp"] for r in fs), " od:", collections.Counter(r["od"] for r in fs))
for fam in ("RND_0", "RND_1"):
    j = load(fam); rows = [r for r in j["rows"] if r["paste"] >= 2]
    print("==", fam, "flagged", len(rows))
    for r in j["false_supported"]:
        print("   FALSE", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k != "reasons"})
    bins = collections.Counter(); sup = collections.Counter(); fls = collections.Counter(); lost = collections.Counter()
    for r in rows:
        e = r["err_range"]; b = ("near<-0.85" if e < -0.85 else "far>0.85" if e > 0.85 else "|er|<=0.85") + ("/lat>0.35" if r["err_lat"] > 0.35 else "/lat<=0.35")
        bins[b] += 1; s = r["ident"] == "SUPPORTED_BY_FOOTPRINT"; sup[b] += s
        if s and r["true_at"] == 0: fls[b] += 1
        if (not s) and r["true_at"] > 0: lost[b] += 1
    for b in sorted(bins): print("   %-22s n %4d supported %4d false %d lost %d" % (b, bins[b], sup[b], fls[b], lost[b]))
    print("   by mode:", collections.Counter((r["mode"], r["ident"]) for r in rows))
