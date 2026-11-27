"""Compact record of the 22 identity attack families run against the current module (full outputs stay outside Git).

Usage: python -B docs/evidence/q02a_v25_repair_2026-10-22/summarize_families.py <dir with <family>.json> <out.json>
W*/L1-L4 come from ../q02a_v24_repair_2026-10-22/identity_attack.py, M*/L5/L6/RND from identity_attack_v24_families.py.
FALSE = SUPPORTED_BY_FOOTPRINT with no true target pixel at range inside the envelope. LOST_correct = a correct
track (|range error| <= 0.3 m, lateral <= 0.15 m; L1-L4 are correct by construction) with true pixels at range that
is not SUPPORTED. LOST_wrong = a wrong track that still had true pixels in range and is not SUPPORTED (fail-safe).
"""
import json, sys
from pathlib import Path

A_FAM = ("W1_behind", "W2_front_behind", "W3_two_beside", "W4_top_corner", "W5_yaw_elev_border", "W6_sigma",
         "L1_slivers", "L2_far", "L3_border", "L4_close")
B_FAM = ("M1_near_corner", "M1b_near_corner_far", "M2_near_vedge", "M3_near_hedge", "M4_far_diag", "M5_border_far",
         "M6_two_behind", "M7_small_far_behind", "L5_corner_correct", "L6_close_front", "RND_0", "RND_1")


def main(src, out):
    fams, total = {}, dict(cases=0, FALSE=0, LOST_correct=0, LOST_wrong=0)
    for fam in A_FAM + B_FAM:
        j = json.loads((Path(src) / (fam + ".json")).read_text()); c = j["summary"]["counts"]
        if fam in A_FAM:
            row = dict(cases=c["cases"], FALSE=c.get("FALSE_SUPPORTED", 0),
                       LOST_correct=c.get("LOST_SUPPORTED", 0) if fam.startswith("L") else 0,
                       LOST_wrong=0 if fam.startswith("L") else c.get("LOST_SUPPORTED", 0))
            examples = j["false_supported"][:5] + (j["lost_supported"][:5] if fam.startswith("L") else [])
        else:
            row = dict(cases=c.get("flagged_cases", 0), FALSE=c.get("FALSE_SUPPORTED", 0), LOST_correct=c.get("LOST_correct", 0),
                       LOST_wrong=c.get("LOST_wrong_track_failsafe", 0))
            examples = j["false_supported"][:5] + j["lost_correct"][:5]
        fams[fam] = dict(row, examples=[{k: v for k, v in e.items() if k != "reasons"} for e in examples])
        for k in total:
            total[k] += row[k]
    Path(out).write_text(json.dumps(dict(total=total, families=fams), indent=1, default=float) + "\n")
    print(json.dumps(total))
    for fam, r in fams.items():
        print("%-22s cases %5d  FALSE %3d  LOST_correct %3d  LOST_wrong %4d" % (fam, r["cases"], r["FALSE"], r["LOST_correct"], r["LOST_wrong"]))


if __name__ == "__main__":
    main(*sys.argv[1:3])
