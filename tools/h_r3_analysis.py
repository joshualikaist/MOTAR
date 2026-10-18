#!/usr/bin/env python3
"""Independent analysis of the H-R3 pilot (docs/plans/h_r3_pilot_v1.json). NumPy only.

Matched events need an identical branch step t0 and identical per-env prefix digests in all four
arms. Event class = E-L0 class of the observation that triggered the branch (trace index t0-1).
Primary: identity-correct camera reobservation (camera detection and renderer target pixels >= 1)
in the observations after actions t0..t0+W-1; a crash, capture or reset before it is a failure.
Trace index i holds the observation after action i; a terminal step holds post-reset state, so
kinematic metrics stop one step before it.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

PLAN = json.loads((Path(__file__).resolve().parents[1] / "docs/plans/h_r3_pilot_v1.json").read_text())
ARMS = ("B0", "B1", "B2", "B3")
CONTRASTS = (("B3", "B2"), ("B3", "B1"), ("B2", "B1"), ("B1", "B0"), ("B2", "B0"), ("B3", "B0"))
PRIMARY = tuple(PLAN["event"]["classes"]["primary"])
LATER_STEPS = 30  # W + 3 s


def load(run):
    out = {}
    for d in sorted(Path(run).glob("seed*_B*")):
        if not d.is_dir():
            continue
        seed, arm = d.name.split("_")
        with np.load(d / "h_r3_trace.npz") as z:
            out.setdefault(int(seed[4:]), {})[arm] = ({k: z[k] for k in z.files}, json.loads((d / "h_r3_meta.json").read_text()))
    return out


def wilson(k, n, z=1.959964):
    if n == 0:
        return [None, None]
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [centre - half, centre + half]


def mcnemar_exact(n10, n01):
    n = n10 + n01
    if n == 0:
        return 1.0
    p = sum(math.comb(n, i) for i in range(min(n10, n01) + 1)) / 2 ** n
    return min(1.0, 2 * p)


def _mean(values):
    v = [x for x in values if x is not None]
    return float(np.mean(v)) if v else None


def coupling_step(tr, ref):
    """First step at which an env that has not branched yet differs between two arms (else None)."""
    t0 = ref["t0"]
    steps, n = ref["digest"].shape
    diff = tr["digest"] != ref["digest"]
    pre = (t0 < 0)[None, :] | (np.arange(steps)[:, None] < t0[None, :])
    hit = np.nonzero((diff & pre).any(axis=1))[0]
    return int(hit[0]) if len(hit) else None


def outcome(tr, env, t0, window, dt):
    steps = tr["camera"].shape[0]
    reobs = term = None
    crash = capture = crash_ctrl = False
    for i in range(t0, min(t0 + window, steps)):
        if reobs is None and tr["camera"][i, env] and tr["el0_visible_px"][i, env] >= 1:
            reobs = i
        crash |= bool(tr["crashed"][i, env])
        crash_ctrl |= bool(tr["crashed"][i, env] and tr["intervening"][i, env])
        capture |= bool(tr["captured"][i, env])
        if tr["crashed"][i, env] or tr["captured"][i, env] or tr["reset"][i, env]:
            term = i
            break
    last = (term - 1) if term is not None else min(t0 + window, steps) - 1   # last pre-terminal observation
    span = list(range(t0, last + 1))
    pos = tr["robot_pos"][[t0 - 1] + span, env]
    dist = lambda i: float(np.linalg.norm(tr["target_pos"][i, env] - tr["robot_pos"][i, env]))
    later = slice(t0, min(t0 + window + LATER_STEPS, steps))
    frac = lambda key: float(np.mean(tr[key][span, env])) if span else None
    success = reobs is not None
    crash_first = crash and (not success or term <= reobs)
    false_det = [i for i in range(t0, (term if term is not None else min(t0 + window, steps) - 1) + 1)
                 if tr["camera"][i, env] and tr["el0_visible_px"][i, env] < 1]
    return {"reobserved": success, "time_to_reobs_s": (reobs - t0 + 1) * dt if success else None,
            "time_censored_s": (reobs - t0 + 1) * dt if success else window * dt,
            "collision_in_window": crash, "collision_under_arm_control": crash_ctrl,
            "collision_free_reobs": bool(success and not crash_first),
            "capture_in_window": capture, "terminal_in_window": term is not None,
            "camera_detection_without_target_pixels": bool(false_det),
            "distance_change_m": dist(last) - dist(t0 - 1), "translation_m": float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
            "fov_retention": frac("el0_in_frustum"), "camera_fraction": frac("camera"), "lidar_only_fraction": frac("lidar_only"),
            "capture_by_w_plus_3s": bool(tr["captured"][later, env].any()), "crash_by_w_plus_3s": bool(tr["crashed"][later, env].any())}


def summarise(pop):
    rows = {}
    for a in ARMS:
        r = [e["results"][a] for e in pop]
        k = sum(x["reobserved"] for x in r)
        times = [x["time_to_reobs_s"] for x in r if x["reobserved"]]
        rows[a] = {"n": len(r), "reobserved": k, "rate": k / len(r) if r else None, "wilson95": wilson(k, len(r)),
                   "median_time_s": float(np.median(times)) if times else None,
                   "mean_time_censored_s": _mean([x["time_censored_s"] for x in r])}
        for key in ("collision_in_window", "collision_under_arm_control", "collision_free_reobs", "capture_in_window",
                    "terminal_in_window", "camera_detection_without_target_pixels", "capture_by_w_plus_3s", "crash_by_w_plus_3s"):
            rows[a][key] = sum(x[key] for x in r)
        for key in ("distance_change_m", "translation_m", "fov_retention", "camera_fraction", "lidar_only_fraction"):
            rows[a]["mean_" + key] = _mean([x[key] for x in r])
    contrasts = {}
    rng = np.random.default_rng(0)
    for x, y in CONTRASTS:
        if not pop:
            contrasts[x + "-" + y] = None
            continue
        a = np.array([e["results"][x]["reobserved"] for e in pop], dtype=float)
        b = np.array([e["results"][y]["reobserved"] for e in pop], dtype=float)
        ta = np.array([e["results"][x]["time_censored_s"] for e in pop])
        tb = np.array([e["results"][y]["time_censored_s"] for e in pop])
        idx = rng.integers(0, len(a), (10000, len(a)))
        boots, tboots = (a[idx] - b[idx]).mean(axis=1), (ta[idx] - tb[idx]).mean(axis=1)
        n10, n01 = int(((a == 1) & (b == 0)).sum()), int(((a == 0) & (b == 1)).sum())
        contrasts[x + "-" + y] = {"rate_diff": float((a - b).mean()), "n10": n10, "n01": n01, "mcnemar_p": mcnemar_exact(n10, n01),
                                  "bootstrap95": [float(np.percentile(boots, 2.5)), float(np.percentile(boots, 97.5))],
                                  "censored_time_diff_s": float((ta - tb).mean()),
                                  "censored_time_bootstrap95": [float(np.percentile(tboots, 2.5)), float(np.percentile(tboots, 97.5))]}
    patterns = {}
    for e in pop:
        key = "".join("1" if e["results"][a]["reobserved"] else "0" for a in ("B1", "B2", "B3"))
        patterns[key] = patterns.get(key, 0) + 1
    return {"arms": rows, "contrasts": contrasts, "patterns_B1B2B3": dict(sorted(patterns.items()))}


def analyse(run):
    data = load(run)
    window, dt = int(PLAN["event"]["window_steps"]), float(PLAN["event"]["step_dt_s"])
    events, unmatched, compute, coupling = [], [], {arm: [] for arm in ARMS}, {}
    for seed, arms in sorted(data.items()):
        if set(arms) != set(ARMS):
            raise ValueError("seed %d lacks an arm" % seed)
        classes = arms["B0"][1]["el0_classes"]
        ref = arms["B0"][0]
        for arm in ARMS:
            compute[arm].extend(arms[arm][0]["controller_ms"].tolist())
        coupling[seed] = {a: coupling_step(arms[a][0], ref) for a in ARMS[1:]}
        onset = min([s for s in coupling[seed].values() if s is not None], default=None)
        for env in range(ref["t0"].shape[0]):
            t0s = [int(arms[a][0]["t0"][env]) for a in ARMS]
            if all(t < 0 for t in t0s):
                continue
            t0 = t0s[0]
            same_t0 = len(set(t0s)) == 1 and t0 > 0
            same_prefix = same_t0 and all(np.array_equal(arms[a][0]["digest"][:t0, env], ref["digest"][:t0, env]) for a in ARMS)
            ident = {"seed": seed, "env": env, "t0": t0, "t0_by_arm": dict(zip(ARMS, t0s))}
            if not same_prefix:
                unmatched.append(dict(ident, reason="t0 differs" if not same_t0 else "prefix digest differs"))
                continue
            res = {a: outcome(arms[a][0], env, t0, window, dt) for a in ARMS}
            ends = [t0 + window]
            for a in ARMS:
                term = np.nonzero(arms[a][0]["reset"][t0:t0 + window, env])[0]
                if len(term):
                    ends.append(t0 + int(term[0]))
            end = min(ends)
            tgt = max(float(np.abs(arms[a][0]["target_pos"][t0:end, env] - ref["target_pos"][t0:end, env]).max(initial=0.0)) for a in ARMS)
            events.append(dict(ident, el0_class=classes[int(ref["el0_cls"][t0 - 1, env])],
                               primary=classes[int(ref["el0_cls"][t0 - 1, env])] in PRIMARY,
                               target_max_diff_pre_terminal_m=tgt, window_before_coupling=onset is None or t0 + window <= onset,
                               shadow_offset_m=float(ref["shadow_offset"][t0, env]), shadow_clear=bool(ref["shadow_clear"][t0, env]),
                               end_reason={a: arms[a][1]["end_reason"][env] for a in ARMS}, results=res))
    primary = [e for e in events if e["primary"]]
    by_class = {}
    for e in events:
        by_class.setdefault(e["el0_class"], []).append(e)
    stop = PLAN["run"]["smoke"]["collision_stop_fraction"]
    safety = {a: (sum(e["results"][a]["collision_in_window"] for e in events) / len(events) if events else None) for a in ARMS}
    failures = [{"seed": e["seed"], "env": e["env"], "t0": e["t0"], "class": e["el0_class"], "shadow_offset_m": e["shadow_offset_m"],
                 "outcomes": {a: ("reobs %.1fs" % e["results"][a]["time_to_reobs_s"] if e["results"][a]["reobserved"]
                                  else ("crash" if e["results"][a]["collision_in_window"] else
                                        ("terminal" if e["results"][a]["terminal_in_window"] else "miss"))) for a in ARMS}}
                for e in primary if not e["results"]["B3"]["reobserved"] or e["results"]["B3"]["collision_in_window"]
                or e["results"]["B3"]["reobserved"] != e["results"]["B2"]["reobserved"]]
    pct = lambda v, q: float(np.percentile(v, q)) if v else None
    tdiff = [e["target_max_diff_pre_terminal_m"] for e in events]
    return {"schema": "motar.h-r3-pilot-analysis.v1", "run": str(Path(run).resolve()),
            "counts": {"triggered_envs": len(events) + len(unmatched), "matched_events": len(events), "unmatched": len(unmatched),
                       "primary_events": len(primary), "by_class": {k: len(v) for k, v in sorted(by_class.items())},
                       "primary_window_before_coupling": sum(e["window_before_coupling"] for e in primary)},
            "unmatched": unmatched, "coupling_onset_step_vs_B0": coupling, "primary": summarise(primary),
            "primary_window_before_coupling": summarise([e for e in primary if e["window_before_coupling"]]),
            "separate_classes": {k: summarise(v) for k, v in sorted(by_class.items()) if k not in PRIMARY},
            "all_matched": summarise(events),
            "safety": {"collision_in_window_fraction_all_matched": safety, "stop_fraction": stop,
                       "unsafe_arms": [a for a, v in safety.items() if v is not None and a != "B0" and v > stop]},
            "target_independence": {"max_target_diff_pre_terminal_m": max(tdiff) if tdiff else None,
                                    "events_with_target_diff": sum(1 for t in tdiff if t > 1e-6)},
            "compute_ms": {a: {"mean": float(np.mean(v)), "p95": pct(v, 95), "p99": pct(v, 99)} for a, v in compute.items() if v},
            "primary_key_cases": failures, "events": events}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    result = analyse(args.run)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("x") as fh:
        json.dump(result, fh, indent=2)
        fh.write("\n")
    print(json.dumps({k: result[k] for k in ("counts", "unmatched", "coupling_onset_step_vs_B0", "safety", "target_independence",
                                             "compute_ms")}, indent=1))
    print(json.dumps({"primary": result["primary"]}, indent=1))


if __name__ == "__main__":
    main()
