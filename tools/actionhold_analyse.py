"""G7 analysis (docs/plans/prereg_actionhold_g7_v1.md, fixed before the run). CPU only.

For each arm (k2, k4: actions held for 2 / 4 steps, open loop; d1: actions delayed by one 0.1 s step) against native
Policy F on the same cells (GR-0 S00, quota 16, E2, A3, seed 4501): quota crash and capture rates per cell, and the
crash difference pooled over the arm's cells (70 and 205 bars) with a 95 % percentile bootstrap (episodes resampled
within each cell for arm and F independently, 10,000 resamples, seed 20261105).
Reading per arm (crash, pp):  TOLERANT if delta <= +2 and CI upper <= +4;  HARMFUL if delta >= +5 and CI lower > 0;
otherwise INCONCLUSIVE. The identity control (k1d0) must equal F exactly (checked by the runner) and is reported.
usage: tools/actionhold_analyse.py G7_RUN_DIR OUT_JSON
"""
import hashlib
import json
import pathlib
import sys

import numpy as np

RUN, OUT = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2])
REF = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00")
ARMS = {"k2d0": (70, 205), "k4d0": (70, 205), "k1d1": (70, 205)}
SEED = 4501
NBOOT, BSEED = 10000, 20261105


def quota_outcomes(cell):
    crash, capture = [], []
    with open(cell / "episodes.jsonl") as f:
        for line in f:
            e = json.loads(line)
            if e["in_quota"]:
                crash.append(e["outcome"] == "crash")
                capture.append(e["outcome"] == "capture")
    return np.array(crash, float), np.array(capture, float)


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    run = json.loads((RUN / "g7_run_report.json").read_text())
    if run.get("reading") != "COMPLETE":
        raise SystemExit("G7 run is not COMPLETE: %s" % run.get("reading"))
    res = {"analysis": "G7 action hold / delay", "run_commit": run["commit"], "inputs_sha256": {}, "cells": {},
           "arms": {}, "identity": next((c.get("identity") for c in run["cells"] if c["arm"] == "k1d0"), None)}
    rng = np.random.default_rng(BSEED)
    for arm, dens in ARMS.items():
        boots, deltas_obs = np.zeros(NBOOT), 0.0
        n_total = 0
        for d in dens:
            cid = "E2_obstacle_aware__%dbars__seed%d" % (d, SEED)
            ca, pa = quota_outcomes(RUN / arm / cid)
            cf, pf = quota_outcomes(REF / cid)
            for tag, p in (("arm", RUN / arm / cid / "episodes.jsonl"), ("F", REF / cid / "episodes.jsonl")):
                res["inputs_sha256"]["%s/%d/%s" % (arm, d, tag)] = hashlib.sha256(p.read_bytes()).hexdigest()
            res["cells"]["%s/%d" % (arm, d)] = {"episodes": len(ca), "crash_pct": 100 * ca.mean(),
                                                "capture_pct": 100 * pa.mean(), "F_crash_pct": 100 * cf.mean(),
                                                "F_capture_pct": 100 * pf.mean(),
                                                "crash_delta_pp": 100 * (ca.mean() - cf.mean())}
            w = len(ca)
            n_total += w
            deltas_obs += w * (ca.mean() - cf.mean())
            ba = ca[rng.integers(0, len(ca), size=(NBOOT, len(ca)))].mean(1)
            bf = cf[rng.integers(0, len(cf), size=(NBOOT, len(cf)))].mean(1)
            boots += w * (ba - bf)
        delta = 100 * deltas_obs / n_total
        ci = [float(np.percentile(100 * boots / n_total, 2.5)), float(np.percentile(100 * boots / n_total, 97.5))]
        if delta <= 2.0 and ci[1] <= 4.0:
            reading = "TOLERANT"
        elif delta >= 5.0 and ci[0] > 0.0:
            reading = "HARMFUL"
        else:
            reading = "INCONCLUSIVE"
        res["arms"][arm] = {"crash_delta_pp": delta, "ci95": ci, "reading": reading, "densities": list(dens)}
    OUT.write_text(json.dumps(res, indent=1, sort_keys=True) + "\n")
    print(json.dumps(res["arms"], indent=1))
    print(OUT, hashlib.sha256(OUT.read_bytes()).hexdigest())


if __name__ == "__main__":
    main()
