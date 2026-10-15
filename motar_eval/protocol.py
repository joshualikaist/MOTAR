"""KROC2027 protocol utilities: cells, quota join, seed-level statistics, decisions, B rule and gate.

The analysis is fixed here before any KROC2027 data exist (plan v2, sections C and D):

* the inference unit is the evaluation seed (n = 7); each seed's value is the equal-weight mean of
  its four density cells, every cell being the first 16 completed episodes of each of 128 envs;
* the primary test is a two-sided paired t over seeds; the family {Delta_A, I_A} is Holm-adjusted;
  practical claims use Bonferroni 97.5 % t intervals against a 3 pp threshold (1 and 5 pp reported);
* BCa 95 % intervals and exact sign-flip p values are sensitivity analyses, computed with the
  unmodified helpers of ``tools/target_motion_replication.py``;
* Delta_A is predicted positive and I_A negative (from already-observed exploratory data); a
  significant result in the other direction is reported as OPPOSITE_DIRECTION.

Stdlib, scipy (t distribution only) and the replication tool's helpers; never ``aerial_gym``.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Dict, Iterable, List, Optional, Sequence

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL_PATH = ROOT / "docs/plans/kroc2027_protocol_v1.json"
REPLICATION_TOOL = ROOT / "tools/target_motion_replication.py"
PROTOCOL_SCHEMA = "motar.kroc2027-protocol.v1"

DECISIONS = (
    "SUPPORTED",
    "PRACTICALLY_SUPPORTED",
    "INCONCLUSIVE",
    "OPPOSITE_DIRECTION",
    "INVALID",
)


# ---------------------------------------------------------------- protocol file
def sha256_file(path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def load_protocol(path=PROTOCOL_PATH) -> Dict:
    protocol = json.loads(Path(path).read_text(encoding="utf-8"))
    if protocol.get("schema") != PROTOCOL_SCHEMA:
        raise ValueError("unexpected protocol schema %r" % protocol.get("schema"))
    for key in ("policy", "evaluation", "initial_view", "motion_gate", "experiment_A",
                "analysis_A", "experiment_B", "gpu_budget_hours"):
        if key not in protocol:
            raise ValueError("protocol is missing %r" % key)
    return protocol


def cell_label(target_model: str, density: int, seed: int) -> str:
    """The historical launcher's cell folder name."""
    return "%s__%dbars__seed%d" % (target_model, int(density), int(seed))


def a_cells(protocol: Dict, stage: str, moving_model: Optional[str]) -> List[Dict]:
    """Experiment A cells in seed-major order (an interrupted run still holds complete seeds).

    ``moving_model`` is the target model chosen by the canary motion gates; ``None`` means no
    moving model passed, so only the static arms exist (H-A2 NOT_TESTED).
    """
    exp = protocol["experiment_A"]
    models = {"static": protocol["target_models"]["static"], "moving": moving_model}
    if stage == "main":
        seeds = exp["seeds"]
    elif stage == "canary":
        seeds = [exp["canary"]["seed"]]
    else:
        raise ValueError("stage must be 'main' or 'canary'")
    out = []
    for seed in seeds:
        for density in exp["densities"]:
            for arm, spec in sorted(exp["arms"].items()):
                model = models[spec["target"]]
                if model is None:
                    continue
                out.append({
                    "experiment": "A",
                    "arm": arm,
                    "target": spec["target"],
                    "target_model": model,
                    "view": spec["view"],
                    "density": int(density),
                    "seed": int(seed),
                    "cell_dir": "%s/%s" % (arm, cell_label(model, density, seed)),
                })
    return out


def b_cells(protocol: Dict, target_model: str, densities: Sequence[int]) -> List[Dict]:
    """Experiment B cells: B0 (policy alone) and B1 (policy + supervisor) on the hidden view."""
    out = []
    for seed in protocol["experiment_B"]["seeds"]:
        for density in densities:
            for arm, search in (("B0", None), ("B1", "b1")):
                out.append({
                    "experiment": "B",
                    "arm": arm,
                    "search": search,
                    "target_model": target_model,
                    "view": "hidden",
                    "density": int(density),
                    "seed": int(seed),
                    "cell_dir": "%s/%s" % (arm, cell_label(target_model, density, seed)),
                })
    return out


# ---------------------------------------------------------------- quota and per-episode join
def quota_rows(ledger_rows: Iterable[Dict], num_envs: int, quota: int) -> List[Dict]:
    """The first ``quota`` completed episodes of each env, checked to be exactly that."""
    rows = [r for r in ledger_rows if r.get("in_quota")]
    per_env: Dict[int, List[int]] = {}
    for r in rows:
        per_env.setdefault(int(r["env_index"]), []).append(int(r["env_episode_index"]))
    if len(per_env) != num_envs:
        raise ValueError("quota covers %d envs, expected %d" % (len(per_env), num_envs))
    for env, indices in per_env.items():
        if sorted(indices) != list(range(quota)):
            raise ValueError("env %d quota episodes are %s, expected 0..%d" % (env, sorted(indices), quota - 1))
    return rows


def join_episode_records(quota: Sequence[Dict], recorder: Sequence[Dict]) -> List[Dict]:
    """Attach the KROC2027 recorder record to every quota episode; fail if any is missing."""
    index = {}
    for rec in recorder:
        key = (int(rec["env_index"]), int(rec["env_episode_index"]))
        if key in index:
            raise ValueError("duplicate recorder record for env %d episode %d" % key)
        index[key] = rec
    joined = []
    for row in quota:
        key = (int(row["env_index"]), int(row["env_episode_index"]))
        if key not in index:
            raise ValueError("no recorder record for env %d episode %d" % key)
        rec = index[key]
        if rec.get("outcome") is not None and rec["outcome"] != row["outcome"]:
            raise ValueError("recorder and ledger disagree on the outcome of env %d episode %d" % key)
        merged = dict(rec)
        merged.update({k: row[k] for k in ("outcome", "capture", "crash", "timeout") if k in row})
        joined.append(merged)
    return joined


def cell_metrics(episodes: Sequence[Dict]) -> Dict[str, float]:
    """Per-cell rates over quota episodes. Acquisition uses the fused measurement."""
    n = len(episodes)
    if n == 0:
        raise ValueError("a cell needs at least one episode")

    def rate(pred):
        return sum(1 for e in episodes if pred(e)) / n

    def never(e):
        return not e["ever_fused_valid"]

    return {
        "episodes": n,
        "capture": rate(lambda e: e["outcome"] == "capture"),
        "crash": rate(lambda e: e["outcome"] == "crash"),
        "timeout": rate(lambda e: e["outcome"] == "timeout"),
        "ever_acquired": rate(lambda e: e["ever_fused_valid"]),
        "never_acquired": rate(never),
        "never_acquired_timeout": rate(lambda e: never(e) and e["outcome"] == "timeout"),
        "never_acquired_crash": rate(lambda e: never(e) and e["outcome"] == "crash"),
        "never_acquired_failure": rate(lambda e: never(e) and e["outcome"] != "capture"),
        "acquired_failure": rate(lambda e: not never(e) and e["outcome"] != "capture"),
    }


def seed_level(per_cell: Dict, arms: Sequence[str], densities: Sequence[int],
               seeds: Sequence[int], metric: str) -> Dict:
    """values[(arm, seed)] = equal-weight mean over densities of per_cell[(arm, density, seed)][metric]."""
    out = {}
    for arm in arms:
        for seed in seeds:
            cells = [per_cell[(arm, d, seed)][metric] for d in densities]
            out[(arm, seed)] = sum(cells) / len(cells)
    return out


# ---------------------------------------------------------------- statistics
def _replication():
    spec = importlib.util.spec_from_file_location("_kroc2027_replication_helpers", REPLICATION_TOOL)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_REPLICATION = None


def replication_helpers():
    """``tools/target_motion_replication.py`` loaded once; its helpers are used unmodified."""
    global _REPLICATION
    if _REPLICATION is None:
        _REPLICATION = _replication()
    return _REPLICATION


def paired_t(diffs: Sequence[float], confidence: float = 0.95) -> Dict:
    """Two-sided one-sample t over paired seed differences."""
    from scipy import stats

    n = len(diffs)
    if n < 2:
        raise ValueError("paired t needs at least two seeds")
    mean = sum(diffs) / n
    sd = math.sqrt(sum((d - mean) ** 2 for d in diffs) / (n - 1))
    se = sd / math.sqrt(n)
    df = n - 1
    q = stats.t.ppf(0.5 + confidence / 2.0, df)
    if se == 0.0:
        t_stat = math.inf if mean != 0 else 0.0
        p = 0.0 if mean != 0 else 1.0
    else:
        t_stat = mean / se
        p = float(2.0 * stats.t.sf(abs(t_stat), df))
    return {"n": n, "mean": mean, "sd": sd, "se": se, "df": df, "t": t_stat, "p_two_sided": p,
            "confidence": confidence, "ci": [mean - q * se, mean + q * se]}


def a_contrasts(capture: Dict, seeds: Sequence[int]) -> Dict[str, List[float]]:
    """Per-seed Delta_A, I_A and the two simple visible-minus-hidden differences."""
    static = [capture[("A1", s)] - capture[("A2", s)] for s in seeds]
    moving = [capture[("A3", s)] - capture[("A4", s)] for s in seeds]
    return {
        "static_visible_minus_hidden": static,
        "moving_visible_minus_hidden": moving,
        "Delta_A": [0.5 * (a + b) for a, b in zip(static, moving)],
        "I_A": [b - a for a, b in zip(static, moving)],
    }


def decide(test: Dict, holm_p: float, predicted: str, alpha: float, practical_pp: float,
           practical_ci: Sequence[float], integrity_ok: bool = True) -> str:
    """Decision label for one confirmatory contrast. Rates are fractions; thresholds are pp."""
    if not integrity_ok:
        return "INVALID"
    sign = 1 if predicted == "positive" else -1
    if holm_p >= alpha:
        return "INCONCLUSIVE"
    if test["mean"] * sign <= 0:
        return "OPPOSITE_DIRECTION"
    threshold = practical_pp / 100.0
    near_bound = practical_ci[0] if sign > 0 else -practical_ci[1]
    return "PRACTICALLY_SUPPORTED" if near_bound > threshold else "SUPPORTED"


def analyse_a(capture: Dict, seeds: Sequence[int], analysis: Dict, integrity_ok: bool) -> Dict:
    """Confirmatory analysis of Experiment A from seed-level capture rates."""
    helpers = replication_helpers()
    contrasts = a_contrasts(capture, seeds)
    alpha = float(analysis["alpha"])
    family = analysis["family"]
    tests = {name: paired_t(contrasts[name]) for name in family}
    holm = helpers.holm({name: tests[name]["p_two_sided"] for name in family})
    bonferroni_conf = 1.0 - alpha / len(family)
    out = {"family": list(family), "alpha": alpha, "contrasts": {}}
    for name in family:
        diffs = contrasts[name]
        practical = paired_t(diffs, confidence=bonferroni_conf)
        predicted = analysis["predicted_direction"][name]
        sign = 1 if predicted == "positive" else -1
        bound = practical["ci"][0] if sign > 0 else -practical["ci"][1]
        out["contrasts"][name] = {
            "seed_diffs": dict(zip(map(str, seeds), diffs)),
            "paired_t": tests[name],
            "holm_adjusted_p": holm[name],
            "practical_interval": practical["ci"],
            "practical_interval_confidence": bonferroni_conf,
            "predicted_direction": predicted,
            "decision": decide(tests[name], holm[name], predicted, alpha,
                               float(analysis["practical_threshold_pp"]), practical["ci"],
                               integrity_ok),
            "practical_sensitivity": {
                "%gpp" % pp: bool(bound > pp / 100.0) for pp in analysis["sensitivity_thresholds_pp"]
            },
            "sensitivity": {
                "bca95": list(helpers.bca_paired(diffs, resamples=int(analysis["bca_resamples"]),
                                                 seed=int(analysis["bca_seed"]), alpha=0.05)),
                "exact_sign_flip_p": helpers.exact_sign_flip_p(diffs),
                "minimum_exact_p": helpers.minimum_exact_p(len(diffs)),
                "seeds_in_predicted_direction": sum(1 for d in diffs if d * sign > 0),
            },
        }
    for name in ("static_visible_minus_hidden", "moving_visible_minus_hidden"):
        out["contrasts"][name] = {"role": "secondary", "paired_t": paired_t(contrasts[name])}
    return out


def analyse_a_static(capture: Dict, seeds: Sequence[int], analysis: Dict, integrity_ok: bool) -> Dict:
    """Static-only Experiment A (amendment 2026-10-15): no moving model passed the canary.

    H-A is tested on A1 - A2 alone: one two-sided paired t over seeds at alpha (no family, so no
    Holm), predicted positive, practical claim from the 95 % t interval against the preregistered
    threshold. H-A2 and Delta_A / I_A are NOT_TESTED.
    """
    helpers = replication_helpers()
    alpha = float(analysis["alpha"])
    diffs = [capture[("A1", s)] - capture[("A2", s)] for s in seeds]
    test = paired_t(diffs, confidence=1.0 - alpha)
    bound = test["ci"][0]
    return {
        "family": ["static_visible_minus_hidden"],
        "alpha": alpha,
        "H-A2": "NOT_TESTED",
        "Delta_A": "NOT_TESTED",
        "I_A": "NOT_TESTED",
        "contrasts": {"static_visible_minus_hidden": {
            "seed_diffs": dict(zip(map(str, seeds), diffs)),
            "paired_t": test,
            "predicted_direction": "positive",
            "decision": decide(test, test["p_two_sided"], "positive", alpha,
                               float(analysis["practical_threshold_pp"]), test["ci"], integrity_ok),
            "practical_sensitivity": {
                "%gpp" % pp: bool(bound > pp / 100.0) for pp in analysis["sensitivity_thresholds_pp"]},
            "sensitivity": {
                "bca95": list(helpers.bca_paired(diffs, resamples=int(analysis["bca_resamples"]),
                                                 seed=int(analysis["bca_seed"]), alpha=0.05)),
                "exact_sign_flip_p": helpers.exact_sign_flip_p(diffs),
                "seeds_in_predicted_direction": sum(1 for d in diffs if d > 0),
            },
        }},
    }


B_PRACTICAL_PP = 1.0
B_SAFETY_MARGIN_PP = 0.5


def analyse_b(capture: Dict, crash: Dict, timeout: Dict, seeds: Sequence[int], integrity_ok: bool) -> Dict:
    """Experiment B (preregistration prereg_kroc2027_B_v1.md, section 4), seed-level values per arm."""
    helpers = replication_helpers()
    diff = lambda store: [store[("B1", s)] - store[("B0", s)] for s in seeds]  # noqa: E731
    primary = paired_t(diff(capture))
    safety = {name: paired_t(diff(store)) for name, store in (("crash", crash), ("timeout", timeout))}
    decision = decide(primary, primary["p_two_sided"], "positive", 0.05, B_PRACTICAL_PP, primary["ci"],
                      integrity_ok)
    non_inferior = all(t["ci"][1] <= B_SAFETY_MARGIN_PP / 100.0 for t in safety.values())
    strict = all(t["ci"][1] <= 0.0 for t in safety.values())
    return {
        "primary": {"contrast": "capture(B1) - capture(B0)", "seed_diffs": dict(zip(map(str, seeds), diff(capture))),
                    "paired_t": primary, "decision": decision,
                    "practical_sensitivity": {"%gpp" % pp: bool(primary["ci"][0] > pp / 100.0)
                                              for pp in (0.5, 1, 2, 3)},
                    "sensitivity": {"bca95": list(helpers.bca_paired(diff(capture), resamples=20000, seed=4300,
                                                                     alpha=0.05)),
                                    "exact_sign_flip_p": helpers.exact_sign_flip_p(diff(capture))}},
        "safety": {name: {"paired_t": t, "upper_bound_pp": 100 * t["ci"][1]} for name, t in safety.items()},
        "safety_non_inferior_0p5pp": non_inferior,
        "safety_upper_bounds_at_or_below_0pp": strict,
        "adopt": bool(decision in ("SUPPORTED", "PRACTICALLY_SUPPORTED") and non_inferior),
        "interpretation": "effect of the whole B1 supervisor, not of yaw scanning alone",
    }


B_SEARCH_METRICS = ("searched", "search_steps_per_episode", "search_steps_among_searched",
                    "suppressed_steps_per_episode", "researches_per_episode",
                    "search_active_at_end_crash", "search_active_at_end_timeout")


def b_secondary_cell(episodes: Sequence[Dict]) -> Dict[str, Optional[float]]:
    """Section 4 secondary quantities of one B cell over its quota episodes; search ones for B1 only."""
    n = len(episodes)
    if n == 0:
        raise ValueError("a cell needs at least one episode")
    acquired = [e for e in episodes if e["ever_fused_valid"]]
    if any(e["first_acquisition_latency_s"] is None for e in acquired):
        raise ValueError("an acquired episode has no first-acquisition latency")
    out = {"acquired": len(acquired) / n,
           "latency_s_among_acquired": (sum(e["first_acquisition_latency_s"] for e in acquired) / len(acquired)
                                        if acquired else None)}
    with_search = sum(1 for e in episodes if "search_search_steps" in e)
    if with_search == 0:
        return out
    if with_search != n:
        raise ValueError("search fields are present in %d of %d episodes" % (with_search, n))
    searched = [e for e in episodes if e["search_search_steps"] > 0]
    mean = lambda key, rows: sum(e[key] for e in rows) / len(rows) if rows else None  # noqa: E731
    at_end = lambda outcome: sum(1 for e in episodes  # noqa: E731
                                 if e["search_active_at_end"] and e["outcome"] == outcome) / n
    out.update({"searched": len(searched) / n,
                "search_steps_per_episode": mean("search_search_steps", episodes),
                "search_steps_among_searched": mean("search_search_steps", searched),
                "suppressed_steps_per_episode": mean("search_suppressed_steps", episodes),
                "researches_per_episode": mean("search_research_count", episodes),
                "search_active_at_end_crash": at_end("crash"),
                "search_active_at_end_timeout": at_end("timeout")})
    return out


def analyse_b_secondary(per_cell: Dict, densities: Sequence[int], seeds: Sequence[int],
                        capture: Dict, a_hidden_deficit: float) -> Dict:
    """Section 4 secondary outcomes, descriptive only; the confirmatory result is ``analyse_b``'s.

    ``per_cell[(arm, density, seed)]`` holds ``b_secondary_cell`` output, ``capture[(arm, seed)]`` the
    seed-level capture, and ``a_hidden_deficit`` Experiment A's A1 - A2 capture (a fraction).
    """
    def level(arm, metric):
        values = {}
        for seed in seeds:
            cells = [per_cell[(arm, d, seed)].get(metric) for d in densities]
            if any(v is None for v in cells):
                raise ValueError("%s %s is missing in a cell of seed %d" % (arm, metric, seed))
            values[seed] = sum(cells) / len(cells)
        return values

    def arms(metric):
        b0, b1 = level("B0", metric), level("B1", metric)
        diffs = [b1[s] - b0[s] for s in seeds]
        return {"B0": sum(b0.values()) / len(seeds), "B1": sum(b1.values()) / len(seeds),
                "seed_diffs": {str(s): d for s, d in zip(seeds, diffs)}, "paired_t_B1_minus_B0": paired_t(diffs)}

    gain = sum(capture[("B1", s)] - capture[("B0", s)] for s in seeds) / len(seeds)
    search = {}
    for metric in B_SEARCH_METRICS:
        values = level("B1", metric)
        search[metric] = {"mean": sum(values.values()) / len(seeds), "seeds": {str(s): v for s, v in values.items()}}
    return {
        "status": "descriptive (preregistration section 4 secondary); not confirmatory",
        "acquired": arms("acquired"),
        "latency_s_among_acquired": arms("latency_s_among_acquired"),
        "recovery_fraction": {"capture_gain": gain, "a_hidden_deficit": a_hidden_deficit,
                              "fraction": gain / a_hidden_deficit, "status": "exploratory"},
        "search_use_B1": search,
        "attribution_note": "search active at the end flags an episode; it does not say the search caused it",
    }


# ---------------------------------------------------------------- Experiment B rule and gate
def select_b_arm(never_acquired_failure: Dict, seeds: Sequence[int], tie_pp: float = 0.5) -> Dict:
    """The hidden arm with the larger never-acquired failure rate; within ``tie_pp`` -> static."""
    static = sum(never_acquired_failure[("A2", s)] for s in seeds) / len(seeds)
    moving_keys = [("A4", s) for s in seeds]
    if not all(k in never_acquired_failure for k in moving_keys):
        return {"arm": "A2", "target": "static", "static_rate": static, "moving_rate": None,
                "reason": "no moving arm (H-A2 NOT_TESTED)"}
    moving = sum(never_acquired_failure[k] for k in moving_keys) / len(seeds)
    if moving - static > tie_pp / 100.0:
        choice, target = "A4", "moving"
    else:
        choice, target = "A2", "static"
    return {"arm": choice, "target": target, "static_rate": static, "moving_rate": moving,
            "tie_pp": tie_pp}


def b_density_count(remaining_gpu_h: float, b_stress_4_h: float, smoke_h: float,
                    reserve_h: float) -> int:
    """4 densities if the stressed 4-density B plus smoke leaves ``reserve_h``; else 2 (115/205)."""
    return 4 if remaining_gpu_h - b_stress_4_h - smoke_h >= reserve_h else 2


def b_gate(integrity_ok: bool, selected: Dict, capture_diff: Sequence[float],
           ever_acquired_diff: Sequence[float], na_failure_share: float,
           remaining_gpu_h: float, b_planned_h: float, retry_h: float,
           b_preregistered: bool, b_run_approved: bool) -> Dict:
    """Experiment B go/no-go (plan v2, section D). Differences are visible minus hidden per seed."""
    cap = paired_t(capture_diff)
    acq = paired_t(ever_acquired_diff)
    criteria = {
        "1_A_integrity_pass": bool(integrity_ok),
        "2_visible_minus_hidden_capture_positive": bool(cap["ci"][0] > 0.0),
        "2_visible_minus_hidden_ever_acquired_positive": bool(acq["ci"][0] > 0.0),
        "3_never_acquired_failures_majority": bool(na_failure_share > 0.5),
        "4_budget_fits": bool(remaining_gpu_h >= b_planned_h + retry_h),
        "5_B_preregistered_and_approved": bool(b_preregistered and b_run_approved),
    }
    return {
        "selected_arm": selected,
        "capture_diff_test": cap,
        "ever_acquired_diff_test": acq,
        "never_acquired_failure_share": na_failure_share,
        "criteria": criteria,
        "verdict": "GO" if all(criteria.values()) else "NO_GO",
    }


# ---------------------------------------------------------------- exploratory pilot (existing data)
def _find(obj, key):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == key:
                return v
            found = _find(v, key)
            if found is not None:
                return found
    return None


def pilot_from_replication(tracked_dir, raw_dir=None) -> Dict:
    """Re-aggregate first acquisition from the 2026-09-25 seven-seed replication (already seen).

    ``tracked_dir`` holds the committed ``cells.csv`` and ``RAW_MANIFEST.json``; ``raw_dir`` holds
    the git-ignored cell folders (default: the same directory). Every ``result.json`` read must
    match its SHA-256 in the manifest. ``first_acquisition`` covers the legacy pooled stopping
    window, not the per-environment quota; the returned record says so and carries both timeout
    rates.
    """
    import csv

    tracked_dir = Path(tracked_dir)
    results_dir = Path(raw_dir) if raw_dir is not None else tracked_dir
    with open(tracked_dir / "cells.csv", newline="") as stream:
        cells = {r["cell_id"]: r for r in csv.DictReader(stream)}
    manifest = json.loads((tracked_dir / "RAW_MANIFEST.json").read_text())["files"]
    per_cell = {}
    files = []
    paths = sorted(results_dir.glob("*__*bars__seed*/result.json"))
    if len(paths) != len(cells):
        raise ValueError("found %d result.json files for %d cells in %s" % (len(paths), len(cells), results_dir))
    for path in paths:
        cid = path.parent.name
        relative = "%s/result.json" % cid
        if manifest.get(relative) != sha256_file(path):
            raise ValueError("%s does not match RAW_MANIFEST.json" % relative)
        arm, bars, seed = cid.split("__")
        bars = int(bars.replace("bars", ""))
        seed = int(seed.replace("seed", ""))
        fa = _find(json.loads(path.read_text()), "first_acquisition")
        outcomes = ("capture", "crash", "timeout")
        n = sum(fa[o]["episodes"] for o in outcomes)
        row = cells[cid]
        if n != int(row["legacy_episodes"]):
            raise ValueError("%s: first_acquisition does not cover the legacy window" % cid)
        na = {o: fa[o]["never_acquired"] for o in outcomes}
        ep = {o: fa[o]["episodes"] for o in outcomes}
        failures = ep["crash"] + ep["timeout"]
        per_cell[(arm, bars, seed)] = {
            "n": n,
            "na": sum(na.values()), "na_timeout": na["timeout"], "na_crash": na["crash"],
            "failures": failures,
            "never_acquired": sum(na.values()) / n,
            "never_acquired_timeout": na["timeout"] / n,
            "never_acquired_crash": na["crash"] / n,
            "na_share_of_failures": (na["crash"] + na["timeout"]) / failures if failures else 0.0,
            "timeout_legacy": ep["timeout"] / n,
            "timeout_quota": int(row["primary_timeout"]) / int(row["primary_episodes"]),
        }
        files.append((relative, manifest[relative]))
    arms = sorted({k[0] for k in per_cell})
    densities = sorted({k[1] for k in per_cell})
    seeds = sorted({k[2] for k in per_cell})
    table = {}
    for arm in arms:
        entry = {}
        for metric in ("never_acquired", "never_acquired_timeout", "never_acquired_crash",
                       "na_share_of_failures", "timeout_legacy", "timeout_quota"):
            values = seed_level(per_cell, [arm], densities, seeds, metric)
            series = [values[(arm, s)] for s in seeds]
            mean = sum(series) / len(series)
            sd = math.sqrt(sum((x - mean) ** 2 for x in series) / (len(series) - 1))
            entry[metric] = {"mean": mean, "sd_over_seeds": sd}
        cells_arm = [v for k, v in per_cell.items() if k[0] == arm]
        failures = sum(c["failures"] for c in cells_arm)
        entry["pooled_na_share_of_failures"] = (
            sum(c["na_timeout"] + c["na_crash"] for c in cells_arm) / failures if failures else 0.0
        )
        table[arm] = entry
    listing = "\n".join("%s %s" % f for f in files)
    try:
        source = str(tracked_dir.resolve().relative_to(ROOT))
    except ValueError:
        source = str(tracked_dir)
    return {
        "schema": "motar.kroc2027-pilot.v1",
        "role": "EXPLORATORY_ALREADY_OBSERVED_NOT_CONFIRMATORY",
        "source": source,
        "verified_against": "RAW_MANIFEST.json (every result.json SHA-256 matched)",
        "estimand": "legacy pooled stopping window (first_acquisition counts equal legacy_episodes)",
        "signal": "fused measurement (camera, or LiDAR associated with a camera-started track)",
        "aggregation": "seed-level mean over 7 seeds; densities equally weighted; fractions",
        "densities": densities,
        "seeds": seeds,
        "arms": table,
        "source_files": len(files),
        "source_listing_sha256": hashlib.sha256(listing.encode("utf-8")).hexdigest(),
    }
