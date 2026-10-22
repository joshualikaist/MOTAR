import json, collections, sys
from pathlib import Path
D = Path(sys.argv[1]); c = collections.Counter(); ident = collections.Counter(); fam = collections.Counter()
for f in ("W1_behind", "W2_front_behind", "W3_two_beside", "W4_top_corner", "W5_yaw_elev_border", "W6_sigma"):
    j = json.loads((D / (f + ".json")).read_text())
    for r in j["lost_supported"]:
        er = r.get("er", r.get("dR")); c[round(abs(er), 1)] += 1; ident[r["ident"]] += 1; fam[f] += 1
print("lost wrong tracks by |er|:", dict(sorted(c.items())), "total", sum(c.values()))
print("by ident:", dict(ident)); print("by family:", dict(fam))
