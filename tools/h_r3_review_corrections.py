#!/usr/bin/env python3
"""H-R3 pilot review corrections, 2026-10-18 (after the Fable 5.1 review; not preregistered).

tools/h_r3_analysis.py stays unchanged; its recorded output is kept. This tool records:
1. a recount under the preregistered terminal rule: a terminal step's observation holds the
   post-reset state, so it cannot be a reobservation (the v1 tool counted it on the same index);
2. motion under arm control only (the v1 "translation" also covers Policy-F steps after reobs),
   with the commanded speed and stalls under control;
3. the selection: matched events are exactly the envs branched by the coupling onset, the resets
   at the onset, and the first-detection step of the primary events;
4. the smoke coupling measurement (env-15 tracker change at the onset);
5. the binding of the v1 analysis JSON to the analysis tool (byte-for-byte re-run).
NumPy only, plus the NumPy-only tools/h_r3_analysis.py for the re-run.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import h_r3_analysis as ha  # noqa: E402

STALL_MPS = 0.1


def load(run, seed):
    out = {}
    for arm in ha.ARMS:
        with np.load(Path(run) / ("seed%d_%s" % (seed, arm)) / "h_r3_trace.npz") as z:
            out[arm] = {k: z[k] for k in z.files}
    return out


def strict_reobserved(tr, env, t0, window):
    """Preregistered rule: reobservation must come before the terminal step (whose obs is post-reset)."""
    for i in range(t0, min(t0 + window, tr["camera"].shape[0])):
        if tr["crashed"][i, env] or tr["captured"][i, env] or tr["reset"][i, env]:
            return False
        if tr["camera"][i, env] and tr["el0_visible_px"][i, env] >= 1:
            return True
    return False


def contrast(a, b):
    a, b = np.asarray(a, dtype=bool), np.asarray(b, dtype=bool)
    n10, n01 = int((a & ~b).sum()), int((~a & b).sum())
    return {"rate_diff": float(a.mean() - b.mean()) if len(a) else None, "n10": n10, "n01": n01, "mcnemar_p": ha.mcnemar_exact(n10, n01)}


def control_motion(tr, env, t0, window, max_velocity):
    ctrl = np.nonzero(tr["intervening"][t0:t0 + window, env])[0] + t0
    if not len(ctrl):
        return {"control_steps": 0, "translation_m": 0.0, "mean_cmd_speed_mps": None, "min_cmd_speed_mps": None}
    step = np.linalg.norm(tr["robot_pos"][ctrl, env] - tr["robot_pos"][ctrl - 1, env], axis=1)
    speed = np.linalg.norm(tr["action"][ctrl, env, :2], axis=1) * max_velocity
    return {"control_steps": int(len(ctrl)), "translation_m": float(step.sum()),
            "mean_cmd_speed_mps": float(speed.mean()), "min_cmd_speed_mps": float(speed.min())}


def corrections(run, smoke, analysis_path, max_velocity):
    analysis_bytes = Path(analysis_path).read_bytes()
    analysis = json.loads(analysis_bytes)
    window = int(ha.PLAN["event"]["window_steps"])
    rerun = (json.dumps(ha.analyse(run), indent=2) + "\n").encode()
    seeds = sorted({e["seed"] for e in analysis["events"]})
    traces = {s: load(run, s) for s in seeds}
    # 1. terminal-step recount
    changed, by_class = [], {}
    for e in analysis["events"]:
        tr = traces[e["seed"]]
        strict = {a: strict_reobserved(tr[a], e["env"], e["t0"], window) for a in ha.ARMS}
        for a in ha.ARMS:
            if strict[a] != e["results"][a]["reobserved"]:
                changed.append({"seed": e["seed"], "env": e["env"], "t0": e["t0"], "class": e["el0_class"], "arm": a,
                                "v1": e["results"][a]["reobserved"], "strict": strict[a]})
        by_class.setdefault(e["el0_class"], []).append(strict)
    recount = {}
    for cls, rows in sorted(by_class.items()):
        recount[cls] = {"n": len(rows), "reobserved": {a: sum(r[a] for r in rows) for a in ha.ARMS},
                        "contrasts": {x + "-" + y: contrast([r[x] for r in rows], [r[y] for r in rows]) for x, y in ha.CONTRASTS}}
    # 2. motion under arm control (primary events)
    motion, primary = [], [e for e in analysis["events"] if e["primary"]]
    for e in primary:
        tr = traces[e["seed"]]
        pattern = "".join("1" if e["results"][a]["reobserved"] else "0" for a in ("B1", "B2", "B3"))
        motion.append(dict({"seed": e["seed"], "env": e["env"], "pattern_B1B2B3": pattern},
                           **{a: control_motion(tr[a], e["env"], e["t0"], window, max_velocity) for a in ("B2", "B3")}))

    def mean_of(rows, arm, key):
        v = [r[arm][key] for r in rows if r[arm][key] is not None]
        return float(np.mean(v)) if v else None
    only3 = [r for r in motion if r["pattern_B1B2B3"] == "001"]
    motion_summary = {pop: {a: {k: mean_of(rows, a, k) for k in ("translation_m", "mean_cmd_speed_mps")} for a in ("B2", "B3")}
                      for pop, rows in (("primary", motion), ("translation_only_001", only3))}
    motion_summary["b2_stalled_in_001"] = [r["env"] for r in only3 if r["B2"]["min_cmd_speed_mps"] < STALL_MPS]
    motion_summary["stall_threshold_mps"] = STALL_MPS
    # 3. selection
    sel = {}
    for s in seeds:
        tr, onset = traces[s], analysis["coupling_onset_step_vs_B0"][str(s)]
        first = min(v for v in onset.values() if v is not None)
        t0 = tr["B0"]["t0"]
        matched = sorted(e["env"] for e in analysis["events"] if e["seed"] == s)
        early = sorted(int(i) for i in np.nonzero((t0 > 0) & (t0 <= first))[0])
        sel[str(s)] = {"coupling_onset_step": first, "matched_envs_equal_envs_branched_by_onset": matched == early,
                       "resets_at_onset": {a: np.nonzero(tr[a]["reset"][first])[0].tolist() for a in ha.ARMS},
                       "primary_first_detection_steps": sorted({int(np.argmax(tr["B0"]["camera"][:, e["env"]])) for e in primary if e["seed"] == s}),
                       "primary_t0_range": [min(e["t0"] for e in primary if e["seed"] == s), max(e["t0"] for e in primary if e["seed"] == s)]}
    # 4. smoke coupling (env 15 at the smoke onset)
    sm = load(smoke, 4502)
    k = 14
    smoke_rec = {"step": k, "env": 15,
                 "belief_diff_B1_vs_B0_m": float(np.abs(sm["B1"]["belief_pos"][k, 15] - sm["B0"]["belief_pos"][k, 15]).max()),
                 "target_diff_B1_vs_B0_m": float(np.abs(sm["B1"]["target_pos"][k, 15] - sm["B0"]["target_pos"][k, 15]).max()),
                 "env3_reset_steps": {a: np.nonzero(sm[a]["reset"][:, 3])[0][:2].tolist() for a in ha.ARMS},
                 "prefix_equal_before_step": bool(all(np.array_equal(sm[a]["digest"][:k, 15], sm["B0"]["digest"][:k, 15]) for a in ha.ARMS))}
    return {"schema": "motar.h-r3-review-corrections.v1", "label": "POST_REVIEW_CORRECTION_2026-10-18",
            "terminal_rule_recount": {"changed": changed, "by_class": recount},
            "control_motion": {"events": motion, "summary": motion_summary},
            "selection": sel, "smoke_coupling": smoke_rec,
            "binding": {"analysis_tool_sha256": hashlib.sha256((Path(__file__).resolve().parent / "h_r3_analysis.py").read_bytes()).hexdigest(),
                        "analysis_json_sha256": hashlib.sha256(analysis_bytes).hexdigest(),
                        "rerun_byte_identical": rerun == analysis_bytes}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--smoke", required=True)
    ap.add_argument("--analysis", required=True)
    ap.add_argument("--max-velocity", type=float, required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    result = corrections(args.run, args.smoke, args.analysis, args.max_velocity)
    with Path(args.out).open("x") as fh:
        json.dump(result, fh, indent=2)
        fh.write("\n")
    print(json.dumps({k: result[k] for k in ("selection", "smoke_coupling", "binding")}, indent=1))
    print(json.dumps({"changed": result["terminal_rule_recount"]["changed"], "motion": result["control_motion"]["summary"]}, indent=1))


if __name__ == "__main__":
    main()
