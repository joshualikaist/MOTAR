"""Compact v2.5-vs-v2.6 record of identity_sweeps.py outputs (full outputs stay outside Git); families are never pooled.

Usage: python -B docs/evidence/q02a_v26_repair_2026-10-22/summarize_sweeps.py <dir with <name>_<v25|v26>.json> <out.json>
Per family and version: scenes; swept tracks; supported; FALSE_SUPPORT (supported with no true target pixel at range),
split MATERIAL / GAP_LE_5CM (true pixels <= 5 cm outside the interval) / CENTRE_IN (true centre inside the track's
interval); correct tracks with truth; correct_supported; INCORRECT_REJECTION and what they were rejected as; status counts
(AMBIGUOUS, RANGE_CONTRADICTED, NOT_TESTABLE, OTHER = fall-through to the camera-clear / not-testable rules); the same for
tracker-like decisions (TL_*: range error N(0, 1 m), lateral N(0, 0.15 m)); sphere-fit status per scene.
"""
import json, sys
from pathlib import Path

NAMES = ("corner_z2", "corner_z3", "random4", "random5", "box")
KEYS = ("scenes", "scenes_with_material_false", "tracks", "supported", "FALSE_SUPPORT", "FALSE_MATERIAL", "FALSE_GAP_LE_5CM",
        "FALSE_CENTRE_IN", "correct_tracks_with_truth", "correct_supported", "INCORRECT_REJECTION")


def pick(c, prefix=""):
    out = {k: c.get(prefix + k, 0) for k in KEYS if not (prefix and k in ("scenes", "scenes_with_material_false"))}
    out["status"] = {k[len(prefix) + 7:]: v for k, v in c.items() if k.startswith(prefix + "status_")}
    out["rejected_as"] = {k[len(prefix) + 12:]: v for k, v in c.items() if k.startswith(prefix + "rejected_as_")}
    return out


def main(src, out):
    res = {}
    for name in NAMES:
        for ver in ("v25", "v26"):
            p = Path(src) / ("%s_%s.json" % (name, ver))
            if not p.exists():
                continue
            j = json.loads(p.read_text()); c = j["counts"]
            res.setdefault(name, {})[ver] = dict(sweep=pick(c), tracker_like=pick(c, "TL_"), crosschecks=c.get("crosschecks", 0),
                                                fit={k[4:]: v for k, v in c.items() if k.startswith("fit_")},
                                                seed=j["seed"], n=j["n"])
    Path(out).write_text(json.dumps(res, indent=1) + "\n")
    print("%-10s %-4s %6s %9s %8s %7s %6s %6s %7s %8s %7s" % ("family", "ver", "scenes", "tracks", "false", "materl", "corr", "c_sup", "rej", "TL_fals", "TL_rej"))
    for name, vv in res.items():
        for ver, r in vv.items():
            s, t = r["sweep"], r["tracker_like"]
            print("%-10s %-4s %6d %9d %8d %7d %6d %6d %7d %8d %7d" % (name, ver, s["scenes"], s["tracks"], s["FALSE_SUPPORT"], s["FALSE_MATERIAL"],
                  s["correct_tracks_with_truth"], s["correct_supported"], s["INCORRECT_REJECTION"], t["FALSE_SUPPORT"], t["INCORRECT_REJECTION"]))


if __name__ == "__main__":
    main(*sys.argv[1:3])
