import json, collections
from pathlib import Path
D = Path("/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_confirm_v24/rules")
fams = ["W1_behind","W2_front_behind","W3_two_beside","W4_top_corner","W5_yaw_elev_border","W6_sigma","L1_slivers","L2_far","L3_border","L4_close",
        "M1_near_corner","M1b_near_corner_far","M2_near_vedge","M3_near_hedge","M4_far_diag","M5_border_far","M6_two_behind","M7_small_far_behind","L5_corner_correct","L6_close_front","RND_0","RND_1"]
tot = collections.Counter(); print("%-22s %6s %6s | %-16s | %-16s | %-16s" % ("family", "cases", "truth", "V24 false/lostC/lostW", "S1 false/lostC/lostW", "S2 false/lostC/lostW"))
for f in fams:
    c = json.loads((D / (f + ".json")).read_text())["summary"]["counts"]
    for k, v in c.items(): tot[k] += v
    fmt = lambda r: "%d / %d / %d" % (c.get("false_" + r, 0), c.get("lost_correct_" + r, 0), c.get("lost_wrong_" + r, 0))
    print("%-22s %6d %6d | %-16s | %-16s | %-16s" % (f, c["cases"], c["truth"], fmt("V24"), fmt("S1"), fmt("S2")))
fmt = lambda r: "%d / %d / %d" % (tot.get("false_" + r, 0), tot.get("lost_correct_" + r, 0), tot.get("lost_wrong_" + r, 0))
print("%-22s %6d %6d | %-16s | %-16s | %-16s" % ("TOTAL", tot["cases"], tot["truth"], fmt("V24"), fmt("S1"), fmt("S2")))
print("correct tracks total:", tot["correct"])
# where does S1 still fail?
for f in fams:
    rows = json.loads((D / (f + ".json")).read_text())["rows"]
    bad = [r for r in rows if r["ident"] in ("SUPPORTED_BY_FOOTPRINT", "RANGE_CONTRADICTED") and r["a"] > r["n"] and r["a"] > r["f"] and r["true_at"] == 0]
    lostc = [r for r in rows if r["true_at"] > 0 and abs(r["err_range"]) <= 0.3 and r["err_lat"] <= 0.15 and not (r["ident"] in ("SUPPORTED_BY_FOOTPRINT", "RANGE_CONTRADICTED") and r["a"] > r["n"] and r["a"] > r["f"])]
    if bad: print("S1 residual false in", f, len(bad), "e.g.", [(r.get("R"), r.get("er", r.get("err_range")), r.get("el"), r.get("ez"), r["i"], r["a"], r["n"], r["f"]) for r in bad[:5]])
    if lostc: print("S1 lost correct in", f, len(lostc), "e.g.", [(r.get("R"), r.get("cover"), r.get("edge"), r["i"], r["a"], r["n"], r["f"], r["ident"]) for r in lostc[:5]])
