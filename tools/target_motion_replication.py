#!/usr/bin/env python3
"""Independent seven-seed replication of the H / E0 / E1 / E2 target-motion evaluation.

Preregistration: docs/prereg_2026-09-24_target_motion_replication_7seed.md. This tool is the executable
half of it: the cell list, the guarded launcher and the analysis are fixed here before any data exist.

    python -B tools/target_motion_replication.py plan              # print the 112 cells; no GPU
    python -B tools/target_motion_replication.py run --stage canary   # GPU; needs user approval
    python -B tools/target_motion_replication.py run --stage main     # GPU; needs a passed canary
    python -B tools/target_motion_replication.py analyze              # CPU; after all 96 cells

`run` never starts by itself. It refuses unless the tree is clean, the preregistration is committed
and unmodified, and (for `main`) the canary passed on the same commit. Each cell runs the original
2026-09-18 launcher unchanged, with the episode ledger switched on through the environment.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
PREREG = "docs/prereg_2026-09-24_target_motion_replication_7seed.md"
LAUNCHER = ROOT / "aerial_gym/rl_training/rl_games/eval_navrl_target_behavior_arms.sh"
OUT_ROOT = ROOT / "results/target_motion_replication_7seed"
ORIGINAL = ROOT / "results/target_motion_e0_e2_evaluation_2026-09-18"

ARMS = ("H_historical", "E0_static", "E1_cv", "E2_obstacle_aware")
DENSITIES = (70, 115, 160, 205)
# The seven smallest integers above the original 4101-4103 with no prior use in any seed context.
SEEDS = (4104, 4105, 4106, 4107, 4108, 4109, 4110)
NUM_ENVS = 128
QUOTA_PER_ENV = 16
EPISODES_PER_CELL = NUM_ENVS * QUOTA_PER_ENV          # 2048, exactly
PLAYER_CAP = 1_000_000_000
CANARY_CELL = ("H_historical", 70, 4101)              # an ORIGINAL cell; consumes no fresh seed
CANARY_DIGEST_LABEL = "tm-replication-canary"
# Instrumentation non-interference only. A canary result is never evidence about target motion.
CANARY_CHECKS = ("1_return_codes_zero", "2_legacy_cell_reproduced", "3_result_identical_except_nonce",
                 "4_trajectory_digest_identical", "5_no_additional_rng_consumption",
                 "6_actor_observation_identical", "7_no_physics_control_reward_target_change",
                 "8_quota_ledger_2048_valid", "9_sixteen_per_environment", "10_episode_ids_unique_complete",
                 "11_min_distance_every_episode")
BASELINE_COMMIT = "839cc8ec72b606c624a6bbfe51752aa26176cd8e"   # documentation baseline before the ledger
OBS_DUMP_STRIDE, OBS_DUMP_MAX = 20, 65536                      # no decimation over a canary cell
# Simulator-code changes allowed against the baseline: path -> (lines added, lines removed).
ALLOWED_SIM_CHANGES = {"aerial_gym/task/navrl_task/navrl_task.py": (16, 0),
                       "aerial_gym/rl_training/rl_games/runner.py": (3, 1)}
EXPECTED_GPU = "NVIDIA GeForce RTX 3070"              # never pool with the 1650 Ti host

REFERENCE = "H_historical"
PRIMARY = (("E0_static", REFERENCE), ("E1_cv", REFERENCE), ("E2_obstacle_aware", REFERENCE))
PAIRS = PRIMARY + (("E1_cv", "E0_static"), ("E2_obstacle_aware", "E0_static"), ("E2_obstacle_aware", "E1_cv"))
ORIGINAL_SIGN = {"E0_static": -1, "E1_cv": -1, "E2_obstacle_aware": +1}   # capture vs H, 2026-09-18
METRICS = ("capture", "crash", "timeout")
ALPHA = 0.05
BOOT_RESAMPLES, BOOT_SEED = 20000, 4100               # as in the original analysis


def cells():
    """Seed-major order, so an interrupted run still holds complete seeds."""
    return [(arm, bars, seed) for seed in SEEDS for bars in DENSITIES for arm in ARMS]


def cell_id(arm, bars, seed):
    return "%s__%dbars__seed%d" % (arm, bars, seed)


# ---------------------------------------------------------------- statistics (fixed before data)
def mean(values):
    return sum(values) / len(values)


def exact_sign_flip_p(diffs):
    """Two-sided p over all 2^n sign patterns of the paired seed differences."""
    observed = abs(mean(diffs))
    patterns = list(itertools.product((1, -1), repeat=len(diffs)))
    hits = sum(1 for signs in patterns
               if abs(mean([s * d for s, d in zip(signs, diffs)])) >= observed - 1e-12)
    return hits / len(patterns)


def minimum_exact_p(n):
    return 2 / 2 ** n


def _cdf(x):
    return 0.5 * (1 + math.erf(x / math.sqrt(2)))


def _ppf(p):
    p = min(max(p, 1e-12), 1 - 1e-12)
    lo, hi = -10.0, 10.0
    for _ in range(200):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _cdf(mid) < p else (lo, mid)
    return (lo + hi) / 2


def bca_paired(diffs, resamples=BOOT_RESAMPLES, seed=BOOT_SEED, alpha=ALPHA):
    """The original analysis's BCa for a mean of paired seed differences, unchanged."""
    n = len(diffs)
    rng = random.Random(seed)
    theta = mean(diffs)
    boots = sorted(mean([diffs[rng.randrange(n)] for _ in range(n)]) for _ in range(resamples))
    lower = sum(1 for b in boots if b < theta) / len(boots)
    if lower <= 0.0 or lower >= 1.0:
        return (boots[int(alpha / 2 * len(boots))], boots[min(len(boots) - 1, int((1 - alpha / 2) * len(boots)))])
    z0 = _ppf(lower)
    jack = [mean(diffs[:i] + diffs[i + 1:]) for i in range(n)]
    jbar = mean(jack)
    den = 6.0 * (sum((jbar - j) ** 2 for j in jack) ** 1.5)
    a = sum((jbar - j) ** 3 for j in jack) / den if den else 0.0
    out = []
    for q in (alpha / 2, 1 - alpha / 2):
        zq = _ppf(q)
        adjusted = _cdf(z0 + (z0 + zq) / max(1e-12, 1 - a * (z0 + zq)))
        out.append(boots[min(len(boots) - 1, max(0, int(adjusted * len(boots))))])
    return tuple(out)


def holm(pvalues):
    ordered = sorted(pvalues.items(), key=lambda kv: kv[1])
    adjusted, running = {}, 0.0
    for i, (key, p) in enumerate(ordered):
        running = max(running, min(1.0, (len(ordered) - i) * p))
        adjusted[key] = running
    return adjusted


def contrast(values, arm, base, metric):
    """values[(arm, seed)][metric] -> the preregistered contrast record."""
    diffs = [values[(arm, s)][metric] - values[(base, s)][metric] for s in SEEDS]
    same = max(sum(d > 0 for d in diffs), sum(d < 0 for d in diffs))
    return {"arm": arm, "baseline": base, "metric": metric, "seed_diffs": dict(zip(map(str, SEEDS), diffs)),
            "mean_diff": mean(diffs), "bca95": list(bca_paired(diffs)), "exact_p": exact_sign_flip_p(diffs),
            "seeds_same_sign": same, "n_seeds": len(diffs)}


# ---------------------------------------------------------------- analysis
def load_cell(root, arm, bars, seed):
    directory = Path(root) / cell_id(arm, bars, seed)
    summary = json.loads((directory / "episode_ledger_summary.json").read_text())
    rows_file = directory / summary["rows_file"]
    if hashlib.sha256(rows_file.read_bytes()).hexdigest() != summary["rows_sha256"]:
        raise ValueError("rows hash mismatch: " + str(directory))
    rows = [json.loads(line) for line in rows_file.read_text().splitlines()]
    quota = [r for r in rows if r["in_quota"]]
    if summary["status"] != "COMPLETE" or len(quota) != EPISODES_PER_CELL:
        raise ValueError("cell not complete: %s (%s, %d)" % (directory, summary["status"], len(quota)))
    if (summary["context"]["seed"], summary["context"]["bars"]) != (seed, bars):
        raise ValueError("cell context mismatch: " + str(directory))
    legacy = [r for r in rows if r["in_legacy_window"]]

    def rates(subset):
        out = {m: sum(r[m] for r in subset) / len(subset) for m in METRICS}
        out["min_relative_distance_m"] = mean([r["min_relative_distance_m"] for r in subset])
        return out
    return {"quota": rates(quota), "legacy": rates(legacy), "legacy_episodes": len(legacy)}


def seed_level(per_cell, estimator):
    """Preregistered aggregation (Amendment 1 A1.3 of the original): per seed, the unweighted mean over
    the four density cells; seeds are the unit of inference."""
    values = {}
    for arm in ARMS:
        for seed in SEEDS:
            cellwise = [per_cell[(arm, bars, seed)][estimator] for bars in DENSITIES]
            values[(arm, seed)] = {k: mean([c[k] for c in cellwise]) for k in cellwise[0]}
    return values


def analyze(per_cell):
    report = {"schema": "motar.target-motion-replication-analysis.v1", "preregistration": PREREG,
              "design": "INDEPENDENT BIAS-CORRECTED REPLICATION",
              "estimand": "per-environment quota: first 16 completed episodes of each of 128 environments; "
                          "the 2026-09-18 campaign used the pooled legacy stopping window",
              "seeds": list(SEEDS), "n_seeds": len(SEEDS), "minimum_exact_two_sided_p": minimum_exact_p(len(SEEDS)),
              "holm_floor_three_contrasts": len(PRIMARY) * minimum_exact_p(len(SEEDS)),
              "estimators": {}}
    for estimator in ("quota", "legacy"):
        values = seed_level(per_cell, estimator)
        rows = [contrast(values, a, b, m) for a, b in PAIRS for m in METRICS + ("min_relative_distance_m",)]
        primary = [r for r in rows if r["metric"] == "capture" and (r["arm"], r["baseline"]) in PRIMARY]
        adjusted = holm({r["arm"]: r["exact_p"] for r in primary})
        for r in primary:
            r["holm_adjusted_p"] = adjusted[r["arm"]]
            r["original_sign"] = ORIGINAL_SIGN[r["arm"]]
            # Confirmed: the original sign and a Holm-adjusted exact p below alpha over the three contrasts.
            r["replicates"] = (math.copysign(1, r["mean_diff"]) == r["original_sign"]
                               and r["holm_adjusted_p"] < ALPHA)
            r["classification"] = ("SUCCESSFUL_DIRECTIONAL_STATISTICAL_REPLICATION" if r["replicates"]
                                   else "NOT_A_SUCCESSFUL_DIRECTIONAL_STATISTICAL_REPLICATION")
            # Seed sign consistency is descriptive only; it is never a pass/fail criterion.
        arm_means = {arm: mean([values[(arm, s)]["capture"] for s in SEEDS]) for arm in ARMS}
        per_density = {}
        for arm, _ in PRIMARY:
            for bars in DENSITIES:
                d = [per_cell[(arm, bars, s)][estimator]["capture"] - per_cell[(REFERENCE, bars, s)][estimator]["capture"]
                     for s in SEEDS]
                per_density["%s-H@%d" % (arm, bars)] = {"seed_diffs": d, "mean_diff": mean(d),
                                                         "seeds_same_sign": max(sum(x > 0 for x in d), sum(x < 0 for x in d))}
        report["estimators"][estimator] = {
            "arm_mean_capture": arm_means, "contrasts": rows, "primary": primary,
            "confirmed_contrasts": [r["arm"] for r in primary if r["replicates"]],
            "ordering_static_lowest_obstacle_aware_highest_descriptive": (
                min(arm_means, key=arm_means.get) == "E0_static"
                and max(arm_means, key=arm_means.get) == "E2_obstacle_aware"),
            "per_density_exploratory": per_density}
    q, l = report["estimators"]["quota"], report["estimators"]["legacy"]
    # The verdict uses the quota estimator only; the legacy window is a diagnostic.
    report["verdict"] = ("REPLICATED" if len(q["confirmed_contrasts"]) == len(PRIMARY)
                         else "PARTIALLY_REPLICATED" if q["confirmed_contrasts"] else "NOT_REPLICATED")
    report["legacy_window_role"] = "diagnostic only; comparable with the 2026-09-18 counting rule"
    report["length_bias_quota_minus_legacy_pp"] = {
        arm: 100 * (q["arm_mean_capture"][arm] - l["arm_mean_capture"][arm]) for arm in ARMS}
    return report


# ---------------------------------------------------------------- guarded execution
def git(*args):
    return subprocess.run(["git", "-C", str(ROOT)] + list(args), capture_output=True, text=True, check=True).stdout


def preconditions(stage, out_root):
    problems = []
    head = git("rev-parse", "HEAD").strip()
    if git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked files are modified; commit or stash first")
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG], capture_output=True).returncode:
        problems.append("the preregistration is not committed")
    try:
        gpus = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"], capture_output=True,
                              text=True, timeout=30).stdout.strip().splitlines()
    except (OSError, subprocess.TimeoutExpired):
        gpus = []
    if gpus != [EXPECTED_GPU]:
        problems.append("expected exactly one %s, found %s" % (EXPECTED_GPU, gpus))
    if stage == "main":
        canary = out_root / "canary" / "canary_report.json"
        report = json.loads(canary.read_text()) if canary.is_file() else {}
        if report.get("verdict") != "CANARY_PASS" or report.get("git_commit") != head:
            problems.append("main needs a CANARY_PASS on this commit (%s)" % canary)
    return head, problems


def run_cell(arm, bars, seed, out_root, ledger, digest=False, obs_dump=None):
    env = dict(os.environ, ARMS=arm, DENSITIES=str(bars), SEEDS=str(seed), GAMES=str(EPISODES_PER_CELL),
               NUM_ENVS=str(NUM_ENVS), OUT_ROOT=str(out_root))
    for key in ("NAVRL_TRAJECTORY_DIGEST", "NAVRL_TRAJECTORY_DIGEST_JSON", "NAVRL_TRAJECTORY_DIGEST_LABEL",
                "NAVRL_OBS_DUMP", "NAVRL_OBS_DUMP_STRIDE", "NAVRL_OBS_DUMP_MAX",
                "NAVRL_EPISODE_LEDGER", "NAVRL_EPISODE_QUOTA_PER_ENV", "NAVRL_PLAYER_GAMES_CAP"):
        env.pop(key, None)
    if digest:
        # Hashes position, orientation and executed command every step up to the aggregate export,
        # which is the same window in a ledger-on and a ledger-off run.
        env.update(NAVRL_TRAJECTORY_DIGEST="1", NAVRL_TRAJECTORY_DIGEST_LABEL=CANARY_DIGEST_LABEL,
                   NAVRL_TRAJECTORY_DIGEST_JSON=str(out_root / cell_id(arm, bars, seed) / "trajectory_digest.json"))
    if obs_dump:
        env.update(NAVRL_OBS_DUMP=str(obs_dump), NAVRL_OBS_DUMP_STRIDE=str(OBS_DUMP_STRIDE),
                   NAVRL_OBS_DUMP_MAX=str(OBS_DUMP_MAX))
    if ledger:
        env.update(NAVRL_EPISODE_LEDGER="1", NAVRL_EPISODE_QUOTA_PER_ENV=str(QUOTA_PER_ENV),
                   NAVRL_PLAYER_GAMES_CAP=str(PLAYER_CAP))
    return subprocess.run(["bash", str(LAUNCHER)], env=env).returncode


# ---------------------------------------------------------------- canary gates (pure, CPU-testable)
def ledger_gates(summary, rows):
    """Gates 8-11 from the ledger summary and its rows."""
    quota = [r for r in rows if r["in_quota"]]
    per_env = {}
    for r in quota:
        per_env[r["env_index"]] = per_env.get(r["env_index"], 0) + 1
    ordinals = {}
    for r in rows:
        ordinals.setdefault(r["env_index"], []).append(r["env_episode_index"])
    ids = [r["episode_id"] for r in rows]
    distances = [r["min_relative_distance_m"] for r in rows]
    return {
        "8_quota_ledger_2048_valid": (summary["status"] == "COMPLETE" and summary["partition_violations"] == 0
                                      and summary["quota"]["episodes"] == EPISODES_PER_CELL == len(quota)
                                      and all(r["outcome"] in METRICS for r in quota)),
        "9_sixteen_per_environment": (len(per_env) == NUM_ENVS
                                      and all(n == QUOTA_PER_ENV for n in per_env.values())),
        "10_episode_ids_unique_complete": (len(set(ids)) == len(ids)
                                           and all(v == list(range(len(v))) for v in ordinals.values())
                                           and [r["completion_rank"] for r in rows] == list(range(1, len(rows) + 1))),
        "11_min_distance_every_episode": (len(distances) == len(rows) > 0
                                          and all(isinstance(d, float) and math.isfinite(d) and d >= 0
                                                  for d in distances)),
    }


def observation_gate(dump_off, dump_on):
    """Gate 6: every actor-observation row sampled by both runs is bitwise identical."""
    import numpy as np
    key_on = {(int(c), int(e)): k for k, (c, e) in enumerate(zip(dump_on["call_index"], dump_on["env_id"]))}
    common = [(k, key_on[(int(c), int(e))]) for k, (c, e) in enumerate(zip(dump_off["call_index"], dump_off["env_id"]))
              if (int(c), int(e)) in key_on]
    if not common:
        return False, 0
    a = np.asarray(dump_off["obs"])[[i for i, _ in common]]
    b = np.asarray(dump_on["obs"])[[j for _, j in common]]
    return bool(np.array_equal(a.view(np.uint32), b.view(np.uint32))), len(common)


def source_gate(baseline=BASELINE_COMMIT):
    """Part of gate 7: against the baseline, simulator code gains only the ledger and its guarded hook."""
    out = git("diff", "--numstat", baseline, "--", "aerial_gym/")
    changed = {}
    for line in out.strip().splitlines():
        added, removed, path = line.split("\t")
        changed[path] = (int(added), int(removed))
    ledger = "aerial_gym/task/navrl_task/navrl_episode_ledger.py"
    changed.pop(ledger, None)
    return changed == ALLOWED_SIM_CHANGES and (ROOT / ledger).is_file(), changed


def cpu_rng_test_passes():
    """Part of gate 5: the CPU test that the ledger consumes no torch, numpy or Python RNG state."""
    env = dict(os.environ, PYTHONNOUSERSITE="1", CUDA_VISIBLE_DEVICES="")
    return subprocess.run([sys.executable, "-B", "-m", "unittest",
                           "test_navrl_episode_ledger.NonInterferenceTest"], cwd=ROOT / "tests", env=env,
                          capture_output=True).returncode == 0


def comparable(result):
    """A cell result without its per-run nonce: two runs of one cell must agree on everything else."""
    out = {k: v for k, v in result.items() if k != "condition"}
    out["condition"] = {k: v for k, v in result["condition"].items() if k != "evaluation_nonce"}
    return out


def run(stage, out_root):
    head, problems = preconditions(stage, out_root)
    if problems:
        print(json.dumps({"stage": stage, "status": "REFUSED", "problems": problems}, indent=2))
        return 2
    if stage == "canary":
        import numpy as np
        import time
        arm, bars, seed = CANARY_CELL
        cid = cell_id(arm, bars, seed)
        root = out_root / "canary"
        off, on = root / "ledger_off", root / "ledger_on"
        if root.exists():
            print(json.dumps({"stage": stage, "status": "REFUSED", "problems": ["canary output exists: " + str(root)]}))
            return 2
        timings = {}
        started = time.time()
        rc_off = run_cell(arm, bars, seed, off, ledger=False, digest=True, obs_dump=off / "obs_dump.npz")
        timings["ledger_off_s"] = round(time.time() - started, 1)
        started = time.time()
        rc_on = run_cell(arm, bars, seed, on, ledger=True, digest=True, obs_dump=on / "obs_dump.npz")
        timings["ledger_on_s"] = round(time.time() - started, 1)
        report = {"purpose": "instrumentation non-interference only; not evidence about target motion",
                  "git_commit": head, "cell": cid, "timings": timings, "return_codes": [rc_off, rc_on]}
        checks, detail = {}, {}
        try:
            load = lambda d, name="result.json": json.loads((d / cid / name).read_text())  # noqa: E731
            original, r_off, r_on = load(ORIGINAL), load(off), load(on)
            d_off, d_on = load(off, "trajectory_digest.json"), load(on, "trajectory_digest.json")
            summary = load(on, "episode_ledger_summary.json")
            rows = [json.loads(line) for line in (on / cid / summary["rows_file"]).read_text().splitlines()]
            obs_ok, n_common = observation_gate(np.load(off / "obs_dump.npz"), np.load(on / "obs_dump.npz"))
            src_ok, src_changed = source_gate()
            rng_cpu = cpu_rng_test_passes()
            checks["1_return_codes_zero"] = rc_off == 0 and rc_on == 0
            checks["2_legacy_cell_reproduced"] = (r_off["outcome"] == original["outcome"]
                                                  and r_off["actual_episodes"] == original["actual_episodes"]
                                                  and r_off["requested_episodes"] == original["requested_episodes"])
            checks["3_result_identical_except_nonce"] = comparable(r_off) == comparable(r_on)
            checks["4_trajectory_digest_identical"] = ((d_off["sha256"], d_off["steps"]) == (d_on["sha256"], d_on["steps"])
                                                      and d_off["steps"] > 0)
            checks["6_actor_observation_identical"] = obs_ok
            checks["5_no_additional_rng_consumption"] = (checks["3_result_identical_except_nonce"]
                                                         and checks["4_trajectory_digest_identical"] and obs_ok and rng_cpu)
            checks["7_no_physics_control_reward_target_change"] = (src_ok and checks["3_result_identical_except_nonce"]
                                                                   and checks["4_trajectory_digest_identical"])
            checks.update(ledger_gates(summary, rows))
            detail = {"original_outcome": original["outcome"], "ledger_off_outcome": r_off["outcome"],
                      "digest_steps": [d_off["steps"], d_on["steps"]], "digest_sha256": [d_off["sha256"], d_on["sha256"]],
                      "observation_rows_compared": n_common, "cpu_rng_test_passed": rng_cpu,
                      "simulator_changes_vs_baseline": src_changed, "ledger_rows": len(rows),
                      "ledger_quota_counts": summary["quota"]["counts"],
                      "crash_rows_with_distance": sum(1 for r in rows if r["crash"])}
        except Exception as exc:  # a missing or unreadable artifact is a failure, preserved as found
            detail["error"] = "%s: %s" % (type(exc).__name__, exc)
        checks = {name: bool(checks.get(name, False)) for name in CANARY_CHECKS}
        report.update(verdict="CANARY_PASS" if all(checks.values()) else "CANARY_FAIL", checks=checks, detail=detail,
                      finished_at=datetime.now(timezone.utc).isoformat())
        root.mkdir(parents=True, exist_ok=True)
        (root / "canary_report.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["verdict"] == "CANARY_PASS" else 1
    for arm, bars, seed in cells():
        head_now, problems = preconditions(stage, out_root)
        if problems or head_now != head:
            print(json.dumps({"status": "STOPPED", "before": cell_id(arm, bars, seed), "problems": problems}))
            return 2
        directory = out_root / cell_id(arm, bars, seed)
        if (directory / "episode_ledger_summary.json").is_file():
            continue
        if (directory / "result.json").exists():
            # A result without a ledger summary is a broken cell; never overwrite or skip it silently.
            print(json.dumps({"status": "STOPPED", "partial_cell": cell_id(arm, bars, seed)}))
            return 1
        if run_cell(arm, bars, seed, out_root, ledger=True) != 0:
            print(json.dumps({"status": "STOPPED", "failed": cell_id(arm, bars, seed)}))
            return 1
        summary = json.loads((directory / "episode_ledger_summary.json").read_text())
        if summary["status"] != "COMPLETE":
            print(json.dumps({"status": "STOPPED", "incomplete": cell_id(arm, bars, seed)}))
            return 1
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("plan")
    r = sub.add_parser("run")
    r.add_argument("--stage", choices=("canary", "main"), required=True)
    r.add_argument("--out", type=Path, default=OUT_ROOT)
    a = sub.add_parser("analyze")
    a.add_argument("--out", type=Path, default=OUT_ROOT)
    args = parser.parse_args(argv)
    if args.command == "plan":
        listed = cells()
        print(json.dumps({"cells": len(listed), "episodes": len(listed) * EPISODES_PER_CELL, "seeds": list(SEEDS),
                          "quota_per_env": QUOTA_PER_ENV, "num_envs": NUM_ENVS,
                          "minimum_exact_two_sided_p": {n: minimum_exact_p(n) for n in (3, 5, 6, 7)},
                          "holm_floor_three_contrasts": {n: 3 * minimum_exact_p(n) for n in (3, 5, 6, 7)},
                          "order": [cell_id(*c) for c in listed]}, indent=1))
        return 0
    if args.command == "run":
        return run(args.stage, args.out)
    per_cell = {c: load_cell(args.out, *c) for c in cells()}
    report = analyze(per_cell)
    (args.out / "analysis.json").write_text(json.dumps(report, indent=1) + "\n")
    print(json.dumps({"verdict": report["verdict"]}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
