#!/usr/bin/env python3
"""B1-R analysis (docs/plans/prereg_b1r_classical_baseline_v1.md §7-§8). CPU only; reads raw cells.

Primary: per-seed unweighted mean over the four densities of the quota capture rate. Contrasts
C1 F-B-PN and C2 F-B-PP form the Holm family; C3 B-PN-B-PP is descriptive (unadjusted p). Exact
two-sided sign-flip test over all 2^n sign vectors; BCa 95 % CI over seeds. Each family contrast
reads SUPERIOR / INFERIOR / NOT_SHOWN. Secondary endpoints are descriptive; non-quota aggregates are
labelled LEGACY_STEP_WINDOW.
"""
from __future__ import annotations

import itertools
import json
import math
from pathlib import Path

import numpy as np

CELL = "E2_obstacle_aware__%dbars__seed%d"


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def quota_rows(directory):
    summary = json.loads((Path(directory) / "episode_ledger_summary.json").read_text())
    rows = [r for r in read_jsonl(Path(directory) / summary.get("rows_file", "episodes.jsonl")) if r.get("in_quota")]
    return summary, rows


def cell_metrics(directory, rl_dt_s=0.1, horizon_s=60.0):
    """Quota-set endpoints of one cell; legacy-window aggregates kept separate and labelled."""
    directory = Path(directory)
    summary, rows = quota_rows(directory)
    counts = summary["quota"]["counts"]
    n = sum(counts.values())
    recorder = read_jsonl(directory / "kroc2027_episodes.jsonl")
    rec = {(r["env_index"], r["env_episode_index"]): r for r in recorder}
    keys = [(r["env_index"], r["env_episode_index"]) for r in rows]
    if not n or len(rows) != n or len(set(keys)) != n or len(rec) != len(recorder):
        raise ValueError("duplicate/missing quota or recorder rows")
    for outcome, count in counts.items():
        if sum(r["outcome"] == outcome for r in rows) != count:
            raise ValueError("quota outcome counts mismatch")
    joined = [(r, rec.get((r["env_index"], r["env_episode_index"]))) for r in rows]
    missing = sum(1 for _, k in joined if k is None)
    if missing or any(r["outcome"] != k["outcome"] for r, k in joined):
        raise ValueError("quota join missing or outcome mismatch")
    ttc = [k["actions"] * rl_dt_s for r, k in joined if k is not None and r["outcome"] == "capture"]
    rmst = [min(k["actions"] * rl_dt_s, horizon_s) if r["outcome"] == "capture" else horizon_s
            for r, k in joined if k is not None]

    def mean_of(field):
        values = [k[field] for _, k in joined if k is not None and k.get(field) is not None]
        return float(np.mean(values)) if values else None

    cam = sum(k.get("obs_camera_valid", 0) for _, k in joined if k is not None)
    obs = sum(k.get("observations", 0) for _, k in joined if k is not None)
    result = json.loads((directory / "result.json").read_text())
    gov = result.get("speed_governor", {})
    side = json.loads((directory / "b1r_hook.json").read_text())
    telemetry = side["counts"]
    pairs = telemetry.get("course_rate_pairs", 0)
    process = dict(telemetry)
    process["course_rate_realised_mean_radps"] = telemetry.get("course_rate_realised_sum", 0) / pairs if pairs else None
    process["course_rate_commanded_mean_radps"] = telemetry.get("course_rate_commanded_sum", 0) / pairs if pairs else None
    return {
        "PROCESS_WINDOW": process,
        "steps_commanded_over_2p5": sum(k["steps_commanded_over_2p5"] for _, k in joined),
        "obs_track_active": sum(k["obs_track_active"] for _, k in joined),
        "track_losses": sum(k["track_losses"] for _, k in joined),
        "observations": obs, "obs_camera_valid": cam,
        "episodes": n, "counts": counts,
        "capture_rate": counts["capture"] / n, "crash_rate": counts["crash"] / n, "timeout_rate": counts["timeout"] / n,
        "recorder_rows_missing": missing,
        "time_to_capture_mean_s": float(np.mean(ttc)) if ttc else None,
        "rmtc60_s": float(np.mean(rmst)) if rmst else None,
        "requested_speed_mean_mps": mean_of("pursuer_requested_speed_mean_mps"),
        "executed_speed_mean_mps": mean_of("pursuer_commanded_speed_mean_mps"),
        "actual_speed_mean_mps": mean_of("pursuer_actual_speed_mean_mps"),
        "camera_detection_fraction": cam / obs if obs else None,
        "LEGACY_STEP_WINDOW": {"intervention_rate": gov.get("intervention_rate"), "mean_scale": gov.get("mean_scale"),
                          "contact_stopping_margin_requested_m": gov.get("contact", {}).get("mean_stopping_margin_requested_m"),
                          "contact_stopping_margin_executed_m": gov.get("contact", {}).get("mean_stopping_margin_executed_m")},
    }


def sign_flip_p(diffs):
    """Exact two-sided sign-flip p-value of the mean difference over all 2^n sign vectors."""
    d = np.asarray(diffs, dtype=float)
    observed = abs(d.mean())
    hits = 0
    for signs in itertools.product((1.0, -1.0), repeat=d.size):
        if abs((d * np.asarray(signs)).mean()) >= observed - 1e-12:
            hits += 1
    return hits / 2 ** d.size


def bca_interval(values, resamples, seed, alpha=0.05):
    """BCa interval of the mean over units; NOT_ESTIMABLE when the bootstrap has no spread."""
    x = np.asarray(values, dtype=float)
    rng = np.random.default_rng(seed)
    boots = x[rng.integers(0, x.size, size=(resamples, x.size))].mean(axis=1)
    theta = x.mean()
    if np.ptp(boots) == 0.0:
        return "NOT_ESTIMABLE"
    from statistics import NormalDist

    nd = NormalDist()
    prop = np.clip((boots < theta).mean() + 0.5 * (boots == theta).mean(), 1e-12, 1 - 1e-12)
    z0 = nd.inv_cdf(prop)
    jack = np.array([np.delete(x, i).mean() for i in range(x.size)])
    diff = jack.mean() - jack
    denom = 6.0 * (diff ** 2).sum() ** 1.5
    accel = (diff ** 3).sum() / denom if denom > 0 else 0.0
    out = []
    for q in (alpha / 2, 1 - alpha / 2):
        z = nd.inv_cdf(q)
        adj = nd.cdf(z0 + (z0 + z) / (1 - accel * (z0 + z)))
        out.append(float(np.quantile(boots, adj)))
    return out


def holm(pvalues, alpha=0.05):
    """Holm step-down: adjusted p-values and rejections, in input order."""
    order = sorted(range(len(pvalues)), key=lambda i: pvalues[i])
    adjusted, running = [0.0] * len(pvalues), 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (len(pvalues) - rank) * pvalues[i]))
        adjusted[i] = running
    return adjusted, [p < alpha for p in adjusted]


def reading(mean, p_holm, ci, alpha=0.05):
    """Pre-declared three-way reading of one family contrast (§8); exhaustive and exclusive."""
    if ci == "NOT_ESTIMABLE":
        return "NOT_SHOWN"
    if mean > 0 and p_holm < alpha and ci[0] > 0:
        return "SUPERIOR"
    if mean < 0 and p_holm < alpha and ci[1] < 0:
        return "INFERIOR"
    return "NOT_SHOWN"


def analyse(arm_dirs, plan):
    """``arm_dirs`` maps arm -> directory holding its evaluation cells."""
    ev, an = plan["evaluation"], plan["analysis"]
    cells, primary = {}, {}
    for arm, root in arm_dirs.items():
        for seed in ev["seeds"]:
            for bars in ev["densities"]:
                cells[(arm, seed, bars)] = cell_metrics(Path(root) / (CELL % (bars, seed)), horizon_s=an["rmtc_horizon_s"])
            primary[(arm, seed)] = float(np.mean([cells[(arm, seed, b)]["capture_rate"] for b in ev["densities"]]))
    contrasts = []
    for a, b in an["family"] + an["descriptive"]:
        diffs = [primary[(a, s)] - primary[(b, s)] for s in ev["seeds"]]
        contrasts.append({"contrast": "%s - %s" % (a, b), "seed_differences": diffs, "mean": float(np.mean(diffs)),
                          "p_sign_flip": sign_flip_p(diffs),
                          "bca95": bca_interval(diffs, an["bootstrap_resamples"], an["bootstrap_seed"])})
    family = contrasts[:len(an["family"])]
    adjusted, _ = holm([c["p_sign_flip"] for c in family], an["alpha"])
    for c, p in zip(family, adjusted):
        c["p_holm"] = p
        c["reading"] = reading(c["mean"], p, c["bca95"], an["alpha"])
    for c in contrasts[len(an["family"]):]:
        c["reading"] = "DESCRIPTIVE_ONLY"
    fields = [k for k, v in next(iter(cells.values())).items() if isinstance(v, (int, float)) or v is None]
    def summarize(records):
        return {field: {"n_nonmissing": sum(r[field] is not None for r in records),
                        "mean": float(np.mean([r[field] for r in records if r[field] is not None]))
                        if any(r[field] is not None for r in records) else None}
                for field in fields}
    per_seed = {"%s|%d" % (arm, seed): summarize([cells[(arm, seed, b)] for b in ev["densities"]])
                for arm in arm_dirs for seed in ev["seeds"]}
    per_density = {"%s|%d" % (arm, bars): summarize([cells[(arm, seed, bars)] for seed in ev["seeds"]])
                   for arm in arm_dirs for bars in ev["densities"]}
    both = all(c["reading"] == "SUPERIOR" for c in family)
    return {"schema": "motar.b1r-analysis.v1",
            "statement": "F outperforms both classical sensor-based baselines under this contract" if both else None,
            "secondary_per_seed": per_seed, "secondary_per_density": per_density,
            "primary": {"%s|%d" % k: v for k, v in primary.items()},
            "contrasts": contrasts,
            "cells": {"%s|%d|%d" % k: v for k, v in cells.items()}}
