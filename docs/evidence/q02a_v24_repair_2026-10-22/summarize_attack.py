"""Compact record of identity_attack.py outputs (the full per-case JSON stays outside Git).

Usage: python -B docs/evidence/q02a_v24_repair_2026-10-22/summarize_attack.py <dir with <family>.json> <out.json>
W* families use wrong tracks: FALSE_SUPPORTED (SUPPORTED with no true target pixel at range) must be 0;
their LOST_SUPPORTED counts wrong tracks that still had target pixels in range and are not SUPPORTED
(fail-safe). L* families use correct tracks: LOST_SUPPORTED must be 0.
"""
import json, sys
from pathlib import Path

FAMILIES = ("W1_behind", "W2_front_behind", "W3_two_beside", "W4_top_corner", "W5_yaw_elev_border", "W6_sigma",
            "L1_slivers", "L2_far", "L3_border", "L4_close")


def main(src, out):
    rows, total = {}, {"cases": 0, "FALSE_SUPPORTED": 0, "LOST_SUPPORTED_correct_tracks": 0, "LOST_SUPPORTED_wrong_tracks": 0}
    for fam in FAMILIES:
        j = json.loads((Path(src) / (fam + ".json")).read_text())
        c = j["summary"]["counts"]
        keep = ("range_m", "sigma_r", "true_in", "true_at", "pasted_in", "pasted_at", "ident")
        rows[fam] = dict(counts=c, false_supported=[{k: r.get(k) for k in r if k not in ("reasons",)} for r in j["false_supported"]],
                         lost_supported_examples=[{k: r.get(k) for k in r if k not in ("reasons",)} for r in j["lost_supported"][:10]])
        total["cases"] += c["cases"]; total["FALSE_SUPPORTED"] += c.get("FALSE_SUPPORTED", 0)
        total["LOST_SUPPORTED_correct_tracks" if fam.startswith("L") else "LOST_SUPPORTED_wrong_tracks"] += c.get("LOST_SUPPORTED", 0)
    Path(out).write_text(json.dumps(dict(total=total, families=rows), indent=1, default=float) + "\n")
    print(json.dumps(total))
    for fam in FAMILIES:
        c = rows[fam]["counts"]
        print("%-20s cases %5d  false %4d  lost %4d  supported %5d  true_at>0 %5d" % (fam, c["cases"], c.get("FALSE_SUPPORTED", 0),
              c.get("LOST_SUPPORTED", 0), c.get("supported", 0), c.get("true_at>0", 0)))


if __name__ == "__main__":
    main(*sys.argv[1:3])
