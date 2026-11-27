#!/usr/bin/env python3
"""Frozen OCC-P2 analysis; CPU only, no simulator imports, no input writes."""
from __future__ import annotations

from collections import Counter
import importlib.util
import itertools
import json
from pathlib import Path
import sys

import numpy as np
from scipy import stats

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from motar_eval import occlusion_events as ev, occlusion_geometry as og, occlusion_trace as ot


def load(name):
    spec = importlib.util.spec_from_file_location("_occ_p2_analysis_" + name, ROOT / "tools" / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def holm(pvalues):
    values = np.asarray(pvalues, float)
    if not np.all(np.isfinite(values)):
        raise ValueError("nonfinite hypothesis p value")
    order = np.argsort(values, kind="stable")
    adjusted = np.empty(len(values))
    running = 0.
    for i, index in enumerate(order):
        running = max(running, (len(values) - i) * values[index])
        adjusted[index] = min(1., running)
    return adjusted.tolist()


def mean_interval(values, alpha=.05):
    values = np.asarray(values, dtype=float)
    if len(values) < 2 or not np.all(np.isfinite(values)):
        raise ValueError("seed summary requires finite independent seed values")
    mean, se = float(values.mean()), float(values.std(ddof=1) / np.sqrt(len(values)))
    radius = float(stats.t.ppf(1-alpha/2, len(values)-1)) * se
    return {"mean": mean, "ci": [mean-radius, mean+radius], "n_seeds": len(values), "alpha": alpha}


def contrast(values):
    values = np.asarray(values, float)
    out = mean_interval(values)
    se = values.std(ddof=1) / np.sqrt(len(values))
    statistic = float(values.mean() / se) if se > 0 else None
    p = float(stats.t.sf(statistic, len(values)-1)) if se > 0 else None
    signs = np.asarray(list(itertools.product((-1, 1), repeat=len(values))), float)
    null = (signs @ values) / len(values)
    out.update(seed_differences=values.tolist(), statistic=statistic, df=len(values)-1, p_one_sided=p,
               simultaneous_ci=mean_interval(values, .05/6)["ci"],
               sign_flip_p=float(np.mean(null >= values.mean() - 1e-12)),
               sign_flip_assumption="independent symmetric seed contrasts, not randomized treatment assignment",
               positive_seeds=int(np.sum(values > 0)))
    out["inference_status"] = "ESTIMABLE" if se > 0 else "NOT_ESTIMABLE_ZERO_SEED_VARIANCE"
    if se == 0:
        out["ci"] = out["simultaneous_ci"] = None
    return out


def duration_summary(rows, at):
    duration = np.asarray([r["duration_s"] for r in rows], float)
    ends = np.asarray([r["end"] for r in rows], dtype=str)
    restored = ends == "geometric_restored"
    quant = lambda values: ({str(q): float(np.quantile(values, q)) for q in (.25, .5, .75, .9, .95)}
                            if len(values) else None)
    bounds = {}
    for t in at:
        long = int(np.sum(duration >= t - 1e-12))
        unknown_short = int(np.sum((duration < t - 1e-12) & ~restored))
        bounds[str(t)] = {"known_at_least": long, "censored_before_threshold": unknown_short,
                          "fraction_lower": long/len(rows) if rows else None,
                          "fraction_upper": (long+unknown_short)/len(rows) if rows else None}
    km = ev.kaplan_meier(duration, restored, at)
    km["support_status_at"] = {str(t): ("SUPPORTED_RISK_SET" if np.any(duration >= t-1e-12)
                                       else "NOT_ESTIMABLE_NO_RISK_SET") for t in at}
    km["S_with_supported_risk_set_only"] = {str(t): (km["S"][str(t)] if np.any(duration >= t-1e-12)
                                                     else None) for t in at}
    return {"n": len(rows), "end_causes": dict(Counter(ends.tolist())),
            "censored_for_geometry_duration": int(np.sum(~restored)),
            "observed_elapsed_quantiles_s": quant(duration), "restored_only_quantiles_s": quant(duration[restored]),
            "threshold_bounds": bounds,
            "kaplan_meier_geometry_restoration": km,
            "competing_incidence": ev.aalen_johansen(duration, ends, at,
                        competing=("geometric_restored", "out_of_view", "capture", "crash", "timeout")),
            "note": "Elapsed/restored-only quantiles are descriptive and censor-selected; KM assumes independent censoring; competing incidence retains outcome/FOV exits. No inference beyond risk-set support."}


def extract_cell(directory, cell, plan):
    """Retain P1 event/exposure definition; augment cause and incident-onset diagnostics."""
    original = load("occlusion_analysis").analyse_cell(directory)
    if original["problems"] or len(original["_exposures"]) != 512 or len(original["_per_episode"]) != 512:
        raise ValueError("invalid complete quota trace: %s %s" % (directory, original["problems"]))
    context, identity = original["context"], original["identity"]
    if (context["seed"] != cell["seed"] or context["bars"] != cell["density"]
            or context["arm"] != cell["arm"] or context["run_id"] != plan["run_id"]):
        raise ValueError("matrix identity mismatch")
    if cell["group"] == "moving" and identity.get("distance_backend") != "direct_l2_v1":
        raise ValueError("moving lineage backend mismatch")
    trace = ot.read_trace(directory)
    meta = trace["summary"]["metadata"]
    cam, dt = meta["camera"], float(meta["dt_s"])
    layout_lookup = ot.reset_lookup(trace["layouts"])
    ledger = {(r["env_index"], r["env_episode_index"]): r for r in
              [json.loads(line) for line in (directory/"episodes.jsonl").read_text().splitlines()] if r["in_quota"]}
    records = {(r["env_index"], r["env_episode_index"]): r for r in
               [json.loads(line) for line in (directory/"kroc2027_episodes.jsonl").read_text().splitlines()]}
    rows_by_episode = {}
    for r in original["_intervals"]["center_occlusion_in_view"]:
        rows_by_episode.setdefault(r["episode"], []).append(r)
    durations, reacq, episodes = [], [], []
    for key, frames in ot.episode_frames(trace["frames"]).items():
        if key not in ledger:
            continue
        name = "%d/%d" % key
        terminal, outcome = int(records[key]["actions"]), records[key]["outcome"]
        acquired = ev.first_true(ot.flag(frames, ot.FUSED_VALID))
        exposure = ev.exposure_s(acquired, terminal, dt)
        center, half, quat = ot.layout_arrays(trace["layouts"], layout_lookup[key])
        eye = og.camera_eye(frames["p_pos"], frames["p_vehicle_quat"], cam["translation_m"])
        angular, ranged = og.in_frustum(eye, frames["p_vehicle_quat"], frames["t_pos"],
                          np.radians(cam["hfov_deg"]), np.radians(cam["vfov_deg"]), cam["max_range_m"])
        in_view = angular & ranged
        blocked = og.line_of_sight(eye, frames["t_pos"], center, half, quat,
                                   radius=meta["target_radius_m"], rim_count=0)["center_blocked"]
        incidents = 0
        onset_causes = Counter()
        interval_rows = rows_by_episode.get(name, [])
        for old in interval_rows:
            row = dict(old, episode="%s/%s" % (cell["seed"], name))
            start, end = row["start_frame"], row["end_frame"]
            if not blocked[start-1]:
                onset, incidents = "clear_to_blocked", incidents + 1
            elif start == acquired + 1:
                onset = "left_prevalent_at_post_acquisition_entry"
            else:
                onset = "in_view_reentry_while_already_blocked"
            onset_causes[onset] += 1
            row["onset_kind"] = onset
            if row["end"] == "restored":
                row["end"] = ("out_of_view" if not in_view[end] else "geometric_restored")
                if in_view[end] and blocked[end]:
                    raise ValueError("interval ended while still in-view blocked")
            row["censored"] = row["end"] != "geometric_restored"
            durations.append(row)
            if row["end"] == "geometric_restored":
                latency = ev.reacquisition(ot.flag(frames, ot.FUSED_VALID), frames["frame"],
                             dict(row, end="restored"), terminal, dt)
                latency.update(end_cause=outcome if latency["censored"] else "reacquired")
                latency["fused_invalid_before_restoration"] = not bool(ot.flag(frames, ot.FUSED_VALID)[end-1])
                reacq.append(latency)
        windows = {}
        for w in (1, 2, 5):
            time_s = min(exposure, float(w))
            count = sum((r["start_frame"]-acquired-1)*dt < time_s-1e-12 for r in interval_rows)
            windows[str(w)] = {"exposure_s": time_s, "count": int(count)}
        episodes.append({"episode": name, "outcome": outcome, "acquired": acquired >= 0,
                         "exposure_s": exposure, "intervals": len(interval_rows),
                         "strict_incidents": incidents, "onset_kinds": dict(onset_causes), "windows": windows})
    if len(episodes) != 512:
        raise ValueError("not 512 complete quota episodes")
    return original, episodes, durations, reacq


def analyse(root, plan):
    runner = load("occlusion_confirmatory")
    design, spec = plan["design"], plan["analysis"]
    seeds, densities = design["seeds"], design["densities"]
    rng = np.random.default_rng(spec["bootstrap_seed"])
    bootstrap = rng.integers(0, len(seeds), size=(spec["bootstrap_resamples"], len(seeds)))
    per_cell, groups = {}, {}
    vectors = {(g, d): [] for g in ("static", "moving") for d in densities}
    for index, cell in enumerate(runner.matrix(plan), 1):
        directory = Path(root) / cell["cell_dir"]
        original, episodes, duration, reacq = extract_cell(directory, cell, plan)
        exposure = sum(e["exposure_s"] for e in episodes)
        count = sum(e["intervals"] for e in episodes)
        if exposure <= 0:
            raise ValueError("zero exposure cannot be dropped, replaced by zero, or assigned epsilon")
        record = {"seed": cell["seed"], "group": cell["group"], "density": cell["density"],
                  "episodes": len(episodes), "acquired_episodes": sum(e["acquired"] for e in episodes),
                  "outcomes": dict(Counter(e["outcome"] for e in episodes)),
                  "exposure_s": exposure, "intervals": count, "rate_per_min": 60*count/exposure,
                  "strict_incidents": sum(e["strict_incidents"] for e in episodes),
                  "onset_kinds": dict(sum((Counter(e["onset_kinds"]) for e in episodes), Counter())),
                  "duration": duration_summary(duration, spec["duration_thresholds_s"]),
                  "windows": {str(w): {"count": sum(e["windows"][str(w)]["count"] for e in episodes),
                                         "exposure_s": sum(e["windows"][str(w)]["exposure_s"] for e in episodes)}
                              for w in (1, 2, 5)}, "parity": original["parity"],
                  "diagnostic_indicators": {k:v for k,v in original["indicators"].items()
                                             if k!="center_occlusion_in_view"}}
        per_cell[cell["cell_dir"]] = record
        vectors[cell["group"], cell["density"]].append(record)
        key = cell["group"] + "/" + str(cell["density"])
        aggregate = groups.setdefault(key, {"rows": [], "reacq": [], "legacy_intervals": [], "exposure": []})
        aggregate["rows"] += duration
        aggregate["reacq"] += reacq
        aggregate["legacy_intervals"] += [dict(r, episode="%s/%s" % (cell["seed"], r["episode"]))
                                          for r in original["_intervals"]["center_occlusion_in_view"]]
        aggregate["exposure"] += original["_exposures"]
        print("OCC-P2 CPU analysis %d/112 %s" % (index, cell["cell_dir"]), file=sys.stderr, flush=True)
    rate_vectors = {}
    for (g, d), cells in vectors.items():
        if [c["seed"] for c in cells] != seeds:
            raise ValueError("incomplete or duplicated seed vector")
        rate = np.asarray([c["rate_per_min"] for c in cells])
        rate_vectors[g, d] = rate
        key = g + "/" + str(d)
        aggregate = groups[key]
        summary = {"seeds": seeds, "episodes": 512*len(seeds),
                   "acquired_episodes": sum(c["acquired_episodes"] for c in cells),
                   "outcomes": dict(sum((Counter(c["outcomes"]) for c in cells), Counter())),
                   "exposure_s": sum(c["exposure_s"] for c in cells),
                   "intervals": sum(c["intervals"] for c in cells),
                   "seed_equal_rate_per_min": float(rate.mean()),
                   "seed_block_bootstrap_95_rate": np.quantile(rate[bootstrap].mean(axis=1), [.025,.975]).tolist(),
                   "per_seed_rates": rate.tolist(),
                   "strict_rate_per_min_seed_equal": float(np.mean([60*c["strict_incidents"]/c["exposure_s"] for c in cells])),
                   "onset_kinds": dict(sum((Counter(c["onset_kinds"]) for c in cells), Counter())),
                   "duration": duration_summary(aggregate["rows"], spec["duration_thresholds_s"]),
                   "incident_only_duration": duration_summary(
                       [r for r in aggregate["rows"] if r["onset_kind"] == "clear_to_blocked"],
                       spec["duration_thresholds_s"]),
                   "legacy_P1_observable_interval": ev.summarise(aggregate["legacy_intervals"], aggregate["exposure"]),
                   "per_seed": cells}
        summary["restricted_window_rates_seed_equal"] = {
            str(w): float(np.mean([60*c["windows"][str(w)]["count"]/c["windows"][str(w)]["exposure_s"]
                                   for c in cells])) for w in (1,2,5)}
        reacq = aggregate["reacq"]
        summary["reacquisition_after_geometry_restoration"] = {
            "eligible": len(reacq), "reacquired": sum(not r["censored"] for r in reacq),
            "censored_end_causes": dict(Counter(r["end_cause"] for r in reacq if r["censored"])),
            "fraction": sum(not r["censored"] for r in reacq)/len(reacq) if reacq else None,
            "km": ev.kaplan_meier([r["time_s"] for r in reacq], [not r["censored"] for r in reacq], (0.5,1,2)),
            "note": "fused measurement resumption, not proven persistent physical identity"}
        lost = [r for r in reacq if r["fused_invalid_before_restoration"]]
        summary["reacquisition_after_geometry_restoration"]["loss_immediately_before_restoration_subset"] = {
            "eligible":len(lost), "reacquired":sum(not r["censored"] for r in lost),
            "fraction":sum(not r["censored"] for r in lost)/len(lost) if lost else None,
            "censored_end_causes":dict(Counter(r["end_cause"] for r in lost if r["censored"]))}
        groups[key] = summary
    contrasts = {g+"_205_minus_70": contrast(rate_vectors[g,205]-rate_vectors[g,70]) for g in ("static","moving")}
    contrasts.update({"moving_minus_static_%d"%d: contrast(rate_vectors["moving",d]-rate_vectors["static",d]) for d in densities})
    adjusted = holm([c["p_one_sided"] if c["p_one_sided"] is not None else 1. for c in contrasts.values()])
    for result, p in zip(contrasts.values(), adjusted):
        result["p_holm"] = p if result["p_one_sided"] is not None else None
        differences = np.asarray(result["seed_differences"])
        result["seed_block_bootstrap_95"] = np.quantile(differences[bootstrap].mean(axis=1), [.025,.975]).tolist()
        result["decision"] = ("INCONCLUSIVE" if result["p_one_sided"] is None else
                              "SUPPORTED" if result["mean"] > 0 and p < .05 else
                              "CONTRADICTED" if result["simultaneous_ci"][1] < 0 else "INCONCLUSIVE")
    duration_changes = {}
    for g in ("static", "moving"):
        a = [c["duration"]["observed_elapsed_quantiles_s"] for c in vectors[g,70]]
        b = [c["duration"]["observed_elapsed_quantiles_s"] for c in vectors[g,205]]
        duration_changes[g] = {"seed_mean_elapsed_median_205_minus_70": mean_interval(
                    [bb["0.5"]-aa["0.5"] for aa,bb in zip(a,b)]) if all(a+b) else "NOT_ESTIMABLE",
                    "equivalence": "NOT_TESTED_NO_DEFENSIBLE_MARGIN"}
    return {"schema":"motar.occlusion-confirmatory-analysis.v1", "status":"COMPLETE", "run_id":plan["run_id"],
            "design":design, "analysis_contract":spec, "groups":groups, "contrasts":contrasts,
            "duration_density_estimation":duration_changes, "cells":per_cell,
            "RQ1":"SUPPORTED" if all(contrasts[k]["decision"]=="SUPPORTED" for k in spec["primary_contrasts"]) else "INCONCLUSIVE",
            "RQ2":"INCONCLUSIVE_EQUIVALENCE_NOT_TESTED",
            "RQ3":"SUPPORTED" if all(contrasts[k]["decision"]=="SUPPORTED" for k in spec["secondary_contrasts"]) else "INCONCLUSIVE",
            "limits":["independent confirmation relative to OCC-P1; seven seed numbers previously used for B",
                      "density endpoint inference is not proof of monotonicity at every intermediate density",
                      "closed-loop whole-system effects, not pure target-speed causality",
                      "pooled duration/CIF points are interval-weighted; seed contrasts and CIs preserve clustering",
                      "KM censoring can be informative; report competing events and threshold bounds",
                      "5 s tail not estimable where risk set is zero; no impossibility claim"]}
