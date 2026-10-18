#!/usr/bin/env python3
"""Post-hoc H-R3 confound diagnostics (written after the pilot measurement; not preregistered).

For each matched primary event of a tools/h_r3_analysis.py result, compare B3 with B2 inside the
window: the first step at which the actions differ, the first step at which the per-env digests
differ (digest i is the state after action i), and the commanded horizontal speed while each arm
controls. A digest difference before any action difference means the trajectories separated
through the noise stream rather than through the lateral command. NumPy only.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

Z_ALPHA, Z_POWER = 1.959964, 1.281552  # two-sided 0.05, power 0.90


def mcnemar_events(p10, p01):
    """Paired events for McNemar (normal approximation, Connor 1987) at two-sided 0.05 and 90 % power."""
    psi, delta = p10 + p01, p10 - p01
    return math.ceil((Z_ALPHA * math.sqrt(psi) + Z_POWER * math.sqrt(psi - delta * delta)) ** 2 / (delta * delta))


def first(mask):
    hit = np.nonzero(mask)[0]
    return int(hit[0]) if len(hit) else None


def diagnose(run, analysis, window):
    traces = {}
    rows = []
    for e in analysis["events"]:
        if not e["primary"]:
            continue
        seed, env, t0 = e["seed"], e["env"], e["t0"]
        for arm in ("B2", "B3", "B0"):
            if (seed, arm) not in traces:
                with np.load(Path(run) / ("seed%d_%s" % (seed, arm)) / "h_r3_trace.npz") as z:
                    traces[seed, arm] = {k: z[k] for k in ("action", "digest", "intervening", "shadow_offset")}
        b2, b3 = traces[seed, "B2"], traces[seed, "B3"]
        w = slice(t0, t0 + window)
        act_diff = np.abs(b3["action"][w, env] - b2["action"][w, env]).max(axis=1) > 0
        fa, fd = first(act_diff), first(b3["digest"][w, env] != b2["digest"][w, env])
        speed = lambda tr: float((np.linalg.norm(tr["action"][w, env][:, :2], axis=1) * analysis["max_velocity"])
                                 [tr["intervening"][w, env]].max(initial=0.0))
        ctrl3 = b3["intervening"][w, env]
        r = e["results"]
        rows.append({"seed": seed, "env": env, "t0": t0, "class": e["el0_class"],
                     "pattern_B1B2B3": "".join("1" if r[a]["reobserved"] else "0" for a in ("B1", "B2", "B3")),
                     "first_action_diff_step": fa, "first_digest_diff_step": fd,
                     "noise_before_action": fd is not None and (fa is None or fd < fa),
                     "b3_nonzero_offset_steps": int(((b3["shadow_offset"][w, env] != 0) & ctrl3).sum()),
                     "b3_control_steps": int(ctrl3.sum()),
                     "max_cmd_speed_mps": {"B2": speed(b2), "B3": speed(b3)}})
    disc = [x for x in rows if x["pattern_B1B2B3"][1] != x["pattern_B1B2B3"][2]]
    # Switch check over every step at which both arms control the same env from the same state.
    switch = {"steps": 0, "offset_zero": 0, "offset_zero_equal_action": 0, "offset_nonzero": 0, "offset_nonzero_equal_action": 0}
    for seed in sorted({s for s, _ in traces}):
        b2, b3 = traces[seed, "B2"], traces[seed, "B3"]
        same = np.zeros_like(b2["intervening"])
        same[1:] = b2["digest"][:-1] == b3["digest"][:-1]
        both = b2["intervening"] & b3["intervening"] & same
        equal = np.abs(b3["action"] - b2["action"]).max(axis=-1) == 0
        zero = b3["shadow_offset"] == 0
        for key, mask in (("steps", both), ("offset_zero", both & zero), ("offset_zero_equal_action", both & zero & equal),
                          ("offset_nonzero", both & ~zero), ("offset_nonzero_equal_action", both & ~zero & equal)):
            switch[key] += int(mask.sum())
    c = analysis["primary"]["contrasts"]["B3-B2"]
    n = analysis["counts"]["primary_events"]
    p10, p01 = c["n10"] / n, c["n01"] / n
    planning = {"contrast": "B3-B2", "pilot_p10": p10, "pilot_p01": p01, "alpha_two_sided": 0.05, "power": 0.90,
                "method": "McNemar normal approximation (Connor 1987)",
                "events_at_pilot_estimate": mcnemar_events(p10, p01),
                "events_at_half_effect": mcnemar_events((p10 + p01) / 2 + (p10 - p01) / 4, (p10 + p01) / 2 - (p10 - p01) / 4),
                "half_effect_assumption": "same discordant total, half the rate difference"}
    planning["primary_events_per_seed_observed"] = n / len({x["seed"] for x in rows})
    return {"schema": "motar.h-r3-posthoc.v1", "label": "POST_HOC_EXPLORATORY", "events": rows, "confirmatory_planning": planning,
            "b2_b3_switch_check": switch,
            "summary": {"primary_events": len(rows), "noise_before_action": sum(x["noise_before_action"] for x in rows),
                        "b2_b3_discordant": len(disc), "discordant_noise_before_action": sum(x["noise_before_action"] for x in disc),
                        "median_max_cmd_speed_mps": {a: float(np.median([x["max_cmd_speed_mps"][a] for x in rows])) for a in ("B2", "B3")}}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", required=True)
    ap.add_argument("--analysis", required=True)
    ap.add_argument("--max-velocity", type=float, required=True)
    ap.add_argument("--window", type=int, default=20)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    analysis = json.loads(Path(args.analysis).read_text())
    analysis["max_velocity"] = args.max_velocity
    result = diagnose(args.run, analysis, args.window)
    with Path(args.out).open("x") as fh:
        json.dump(result, fh, indent=2)
        fh.write("\n")
    print(json.dumps(result["summary"], indent=1))


if __name__ == "__main__":
    main()
