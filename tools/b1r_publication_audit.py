#!/usr/bin/env python3
"""B1-R publication evidence and mechanism audit (read-only; CPU; no simulation).

    python -B tools/b1r_publication_audit.py --out docs/evidence/b1r_v1_audit/audit.json

Reads only the existing B1-R raw cells, their sidecars and manifests, and the frozen records. It
writes one new JSON file (exclusive create). Sections are labelled by status:

* ``verification``   -- independent recomputation of the preregistered result (no new claim).
* ``exploratory``    -- descriptive / observational diagnostics; never causal, never a new verdict.
* ``tf32``           -- technical sensitivity against the known TF32 exposure; it does not change
  the preregistered C1/C2 reading.

Nothing here edits raw data, the preregistration or existing evidence.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from motar_eval import occlusion_trace as ot  # noqa: E402

RAW = Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1")
OCC = Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2/moving/A3")
SEEDS = (4301, 4302, 4303, 4304, 4305, 4306, 4307)
DENSITIES = (70, 115, 160, 205)
ARMS = {"F": RAW / "A/F_SHADOW", "B-PN": RAW / "C-PN/B-PN", "B-PP": RAW / "C-PP/B-PP"}
CELL = "E2_obstacle_aware__%dbars__seed%d"
DT = 0.1


def load_tool(name):
    spec = importlib.util.spec_from_file_location("_audit_" + name, ROOT / "tools" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def quota_rows(cell):
    summary = json.loads((cell / "episode_ledger_summary.json").read_text())
    return summary, [r for r in jsonl(cell / summary["rows_file"]) if r["in_quota"]]


def recorder(cell, keys):
    rec = {(r["env_index"], r["env_episode_index"]): r for r in jsonl(cell / "kroc2027_episodes.jsonl")}
    return [rec[k] for k in keys]


def q(values, p):
    return float(np.quantile(values, p)) if len(values) else None


def mean(values):
    return float(np.mean(values)) if len(values) else None


# ------------------------------------------------------------------ statistics (independent code)
def sign_flip_two_sided(d):
    d = np.asarray(d, float)
    obs = abs(d.mean())
    signs = np.array(list(itertools.product((1.0, -1.0), repeat=len(d))))
    return float(np.mean(np.abs((signs * d).mean(axis=1)) >= obs - 1e-12))


def holm_adjust(p):
    order = np.argsort(p)
    adj, running = np.empty(len(p)), 0.0
    for rank, i in enumerate(order):
        running = max(running, min(1.0, (len(p) - rank) * p[i]))
        adj[i] = running
    return adj.tolist()


def scipy_bca(d, seed=20261023):
    from scipy import stats

    d = np.asarray(d, float)
    if np.ptp(d) == 0:
        return "NOT_ESTIMABLE"
    res = stats.bootstrap((d,), np.mean, n_resamples=20000, method="BCa", confidence_level=0.95,
                          random_state=np.random.default_rng(seed))
    return [float(res.confidence_interval.low), float(res.confidence_interval.high)]


def contrasts(primary, pairs):
    out = []
    for a, b in pairs:
        d = [primary[a][s] - primary[b][s] for s in SEEDS]
        out.append({"contrast": "%s - %s" % (a, b), "seed_differences": d, "mean": float(np.mean(d)),
                    "same_sign_seeds": int(max(sum(x > 0 for x in d), sum(x < 0 for x in d))),
                    "p_sign_flip": sign_flip_two_sided(d), "bca95_scipy": scipy_bca(d),
                    "bca95_scipy_other_rng_seed_7": scipy_bca(d, seed=7)})
    adj = holm_adjust([c["p_sign_flip"] for c in out[:2]])
    for c, p in zip(out[:2], adj):
        c["p_holm_family_C1_C2"] = p
    return out


# ------------------------------------------------------------------ geometry (exact, no cdist)
def surface_distance(points, center, half, quat):
    """Distance [P] from 2-D points to the nearest rotated rectangle surface (0 inside)."""
    if len(center) == 0:
        return np.full(len(points), np.inf)
    yaw = 2.0 * np.arctan2(quat[:, 2], quat[:, 3])
    cs, sn = np.cos(-yaw), np.sin(-yaw)
    rel = points[:, None, :] - center[None, :, :2]
    lx = rel[..., 0] * cs - rel[..., 1] * sn
    ly = rel[..., 0] * sn + rel[..., 1] * cs
    dx = np.maximum(np.abs(lx) - half[None, :, 0], 0.0)
    dy = np.maximum(np.abs(ly) - half[None, :, 1], 0.0)
    return np.sqrt(dx * dx + dy * dy).min(axis=1)


# ------------------------------------------------------------------ sections
def verification(analysis_tool):
    out = {"status": "VERIFICATION_OF_PREREGISTERED_RESULT"}
    run = load_tool("b1r_run")
    manifests = {}
    for stage in ("A", "B", "C-PN", "C-PP"):
        manifests[stage] = {"raw_manifest_sha256": run.verify_manifest(RAW / stage),
                            "file_count": json.loads((RAW / stage / "RAW_MANIFEST.json").read_text())["file_count"]}
    out["raw_manifests_verified"] = manifests
    summary = json.loads((ROOT / "docs/evidence/b1r_v1/analysis_summary.json").read_text())
    binding = json.loads((ROOT / "docs/evidence/b1r_v1/selection_binding.json").read_text())
    out["analysis_json_sha_matches_record"] = sha(RAW / "analysis.json") == summary["analysis_sha256"]
    out["selection_json_sha_matches_binding"] = sha(RAW / "selection.json") == binding["selection_sha256"]
    for stage, rec in (("A", "stage_A_g0.json"), ("C-PN", "stage_C-PN.json"), ("C-PP", "stage_C-PP.json"),
                       ("B", "stage_B_selection.json")):
        out["%s_manifest_matches_record" % stage] = (
            json.loads((ROOT / "docs/evidence/b1r_v1" / rec).read_text())["raw_manifest_sha256"]
            == manifests[stage]["raw_manifest_sha256"])
    # Primary endpoint recomputed from ledger ROWS (not the ledger summary counts).
    per_cell, primary, exclusive = {}, {}, 0
    for arm, root in ARMS.items():
        primary[arm] = {}
        for s in SEEDS:
            rates = []
            for b in DENSITIES:
                summary_c, rows = quota_rows(root / (CELL % (b, s)))
                flags_ok = all((r["capture"] + r["crash"] + r["timeout"]) == 1 and
                               r["outcome"] == ("capture" if r["capture"] else "crash" if r["crash"] else "timeout")
                               for r in rows)
                exclusive += 0 if flags_ok else 1
                counts = {o: sum(r["outcome"] == o for r in rows) for o in ("capture", "crash", "timeout")}
                if counts != summary_c["quota"]["counts"] or len(rows) != 512:
                    raise ValueError("ledger rows disagree with the summary: %s %d %d" % (arm, s, b))
                per_cell["%s|%d|%d" % (arm, s, b)] = counts
                rates.append(counts["capture"] / 512)
            primary[arm][s] = float(np.mean(rates))
    out["cells_with_non_exclusive_outcome_flags"] = exclusive
    recorded = json.loads((RAW / "analysis.json").read_text())
    out["primary_recomputed"] = {a: {str(s): v for s, v in d.items()} for a, d in primary.items()}
    out["primary_max_abs_diff_vs_analysis_json"] = max(abs(primary[a][s] - recorded["primary"]["%s|%d" % (
        {"F": "F_SHADOW"}.get(a, a), s)]) for a in ARMS for s in SEEDS)
    out["contrasts_recomputed"] = contrasts(primary, [("F", "B-PN"), ("F", "B-PP"), ("B-PN", "B-PP")])
    rec_c = {c["contrast"]: c for c in recorded["contrasts"]}
    out["contrast_check_vs_analysis_json"] = [
        {"contrast": c["contrast"], "mean_abs_diff": abs(c["mean"] - r["mean"]),
         "p_equal": c["p_sign_flip"] == r["p_sign_flip"],
         "holm_equal": c.get("p_holm_family_C1_C2") == r.get("p_holm"),
         "bca_recorded": r["bca95"], "bca_scipy_independent": c["bca95_scipy"], "reading_recorded": r["reading"]}
        for c, r in zip(out["contrasts_recomputed"],
                        (rec_c["F_SHADOW - B-PN"], rec_c["F_SHADOW - B-PP"], rec_c["B-PN - B-PP"]))]
    # Density table (pooled over seeds) and per-seed ranges.
    dens = {}
    for arm in ARMS:
        for b in DENSITIES:
            cs = [per_cell["%s|%d|%d" % (arm, s, b)] for s in SEEDS]
            n = 512 * len(cs)
            dens["%s|%d" % (arm, b)] = {
                "capture": sum(c["capture"] for c in cs) / n, "crash": sum(c["crash"] for c in cs) / n,
                "timeout": sum(c["timeout"] for c in cs) / n,
                "capture_seed_min": min(c["capture"] for c in cs) / 512,
                "capture_seed_max": max(c["capture"] for c in cs) / 512, "episodes": n}
    out["density_pooled"] = dens
    out["capture_crash_precedence"] = {
        "rule": "navrl_task.py:6123-6124 crashed_out = crashed_out & ~captured (capture wins a same-step contact)",
        "ledger_outcome_flags_exclusive": exclusive == 0,
        "same_step_contact_on_capture": "NOT_RECORDED (no per-episode contact record; terminal state is not in the frame trace)"}
    # Tuning limits.
    sel = json.loads((ROOT / "docs/evidence/b1r_v1/stage_B_selection.json").read_text())
    table = sel["table"]
    mono = {}
    for v in (1.5, 2.0, 2.5, 3.0):
        caps = [next(t["capture"] for t in table if t["controller"] == "B-PN" and t["N"] == n and t["v_nom"] == v)
                for n in (3.0, 5.0, 8.0)]
        mono[str(v)] = {"capture_by_N_3_5_8": caps, "increasing": caps[0] < caps[1] < caps[2]}
    pp = sorted((t for t in table if t["controller"] == "B-PP"), key=lambda t: -t["capture"])
    tune_vs_eval = {}
    for arm, key in (("B-PN", "B-PN"), ("B-PP", "B-PP")):
        sdir = RAW / "B" / ("B-PN_N8_v1.5" if arm == "B-PN" else "B-PP_v2.5")
        for b in (115, 205):
            _, rows = quota_rows(sdir / (CELL % (b, 4390)))
            tune_vs_eval["%s|%d" % (arm, b)] = {
                "tuning_seed_4390_capture": sum(r["outcome"] == "capture" for r in rows) / len(rows),
                "evaluation_pooled_capture": dens["%s|%d" % (key, b)]["capture"]}
    out["tuning_limits"] = {"B_PN_capture_increases_with_N_at_every_v_nom": all(m["increasing"] for m in mono.values()),
                            "B_PN_by_v_nom": mono, "selected_N_on_grid_edge": True,
                            "B_PP_top_two": [(t["v_nom"], t["capture"], t["crash"]) for t in pp[:2]],
                            "tuning_seed_vs_evaluation": tune_vs_eval, "tuning_seeds": 1}
    return out, primary, per_cell


def exploratory():
    out = {"status": "EXPLORATORY_DESCRIPTIVE_NOT_CAUSAL",
           "windows": {"quota": "first 4 completed episodes per environment (ledger in_quota)",
                       "LEGACY_STEP_WINDOW": "result.json speed_governor: all env-steps up to cutoff_step, incl. in-progress episodes",
                       "frames": "occlusion_frames.bin restricted to quota episodes; terminal state not recorded"}}
    rows_out = {}
    for arm, root in ARMS.items():
        for b in DENSITIES:
            acc = {k: [] for k in ("req", "exe", "act", "ratio", "ttc", "crash_t", "speed_cap", "speed_crash")}
            gov = {k: [] for k in ("intervention_rate", "mean_scale", "negative_stopping_margin_requested_rate",
                                   "negative_stopping_margin_executed_rate")}
            contact = {k: [] for k in ("count", "mean_actual_speed_mps", "mean_requested_speed_mps",
                                       "mean_executed_speed_mps", "mean_clearance_m", "mean_stopping_margin_requested_m",
                                       "mean_stopping_margin_executed_m")}
            state = {"camera": [0, 0.0, 0.0], "track_only": [0, 0.0, 0.0], "no_track": [0, 0.0, 0.0]}
            causes = {"count": 0, "bar_contact": 0, "below": 0, "above": 0, "out_of_bounds": 0}
            pre_speed, pre_clear = [], []           # last recorded frame before a quota crash
            terminal = {"camera": 0, "track_only": 0, "no_track": 0}
            split = []                     # (actual mean speed, crashed)
            for s in SEEDS:
                cell = root / (CELL % (b, s))
                _, rows = quota_rows(cell)
                keys = [(r["env_index"], r["env_episode_index"]) for r in rows]
                recs = recorder(cell, keys)
                for r, k in zip(rows, recs):
                    acc["req"].append(k["pursuer_requested_speed_mean_mps"])
                    acc["exe"].append(k["pursuer_commanded_speed_mean_mps"])
                    acc["act"].append(k["pursuer_actual_speed_mean_mps"])
                    if k["pursuer_requested_speed_mean_mps"] > 1e-6:
                        acc["ratio"].append(k["pursuer_commanded_speed_mean_mps"] / k["pursuer_requested_speed_mean_mps"])
                    if r["outcome"] == "capture":
                        acc["ttc"].append(k["actions"] * DT)
                    if r["outcome"] == "crash":
                        acc["crash_t"].append(k["actions"] * DT)
                    split.append((k["pursuer_actual_speed_mean_mps"], r["outcome"] == "crash"))
                result_json = json.loads((cell / "result.json").read_text())
                for kk in causes:
                    causes[kk] += int(result_json["crash_causes"][kk])
                res = result_json["speed_governor"]
                for kk in gov:
                    gov[kk].append(res.get(kk))
                for kk in contact:
                    v = res.get("contact", {}).get(kk)
                    if v is not None:
                        contact[kk].append(v)
                # Tracking-state occupancy from the frame trace (quota episodes only).
                trace = ot.read_trace(cell)
                frames = trace["frames"]
                look = ot.reset_lookup(trace["layouts"])
                key_arr = frames["env"].astype(np.int64) * 1000003 + frames["episode"].astype(np.int64)
                quota_keys = np.array([e * 1000003 + ep for e, ep in keys], dtype=np.int64)
                f = frames[np.isin(key_arr, quota_keys)]
                acquired = (f["flags"] & ot.ACQUIRED_BEFORE) > 0
                cam = (f["flags"] & ot.CAMERA_VALID) > 0
                trk = (f["flags"] & ot.TRACK_ACTIVE) > 0
                spd = np.linalg.norm(f["p_vel"][:, :2], axis=1)
                dist = np.linalg.norm(f["t_pos"][:, :2] - f["p_pos"][:, :2], axis=1)
                for name, m in (("camera", acquired & cam), ("track_only", acquired & ~cam & trk),
                                ("no_track", acquired & ~cam & ~trk)):
                    state[name][0] += int(m.sum())
                    state[name][1] += float(spd[m].sum())
                    state[name][2] += float(dist[m].sum())
                crashed = {k for k, r in zip(keys, rows) if r["outcome"] == "crash"}
                for k, ep in ot.episode_frames(f).items():
                    if k in crashed:
                        last = ep[np.argmax(ep["frame"])]
                        c, t = bool(last["flags"] & ot.CAMERA_VALID), bool(last["flags"] & ot.TRACK_ACTIVE)
                        terminal["camera" if c else "track_only" if t else "no_track"] += 1
                        pre_speed.append(float(np.linalg.norm(last["p_vel"][:2])))
                        if k in look:
                            cc, hh, qq = ot.layout_arrays(trace["layouts"], look[k])
                            pre_clear.append(float(surface_distance(last["p_pos"][None, :2].astype(np.float64),
                                                                    cc.astype(np.float64), hh.astype(np.float64),
                                                                    qq.astype(np.float64))[0]))
            frames_total = sum(v[0] for v in state.values())
            sp = np.array(split)
            med = float(np.median(sp[:, 0]))
            lo, hi = sp[sp[:, 0] <= med], sp[sp[:, 0] > med]
            rows_out["%s|%d" % (arm, b)] = {
                "quota_speed_mean_mps": {"requested": mean(acc["req"]), "executed": mean(acc["exe"]), "actual": mean(acc["act"])},
                "executed_over_requested_episode_mean": mean(acc["ratio"]),
                "LEGACY_STEP_WINDOW_governor": {k: mean([x for x in v if x is not None]) for k, v in gov.items()},
                "LEGACY_STEP_WINDOW_contact": {k: (sum(v) if k == "count" else mean(v)) for k, v in contact.items()},
                "time_to_capture_s": {"median": q(acc["ttc"], .5), "p90": q(acc["ttc"], .9), "n": len(acc["ttc"])},
                "crash_time_s": {"median": q(acc["crash_t"], .5), "share_within_1s": (float(np.mean(np.array(acc["crash_t"]) <= 1.0))
                                                                                       if acc["crash_t"] else None), "n": len(acc["crash_t"])},
                "post_acquisition_frame_share": {k: v[0] / frames_total for k, v in state.items()},
                "post_acquisition_frame_count": {k: v[0] for k, v in state.items()},
                "LEGACY_STEP_WINDOW_crash_causes": dict(causes, out_of_bounds_share=(
                    causes["out_of_bounds"] / causes["count"] if causes["count"] else None)),
                "quota_crash_last_frame": {"speed_mps_median": q(pre_speed, .5), "speed_mps_p90": q(pre_speed, .9),
                                           "centre_to_bar_surface_m_median": q(pre_clear, .5),
                                           "n": len(pre_speed),
                                           "note": "one action before termination; p_vel before the contact step"},
                "post_acquisition_mean_speed_mps": {k: (v[1] / v[0] if v[0] else None) for k, v in state.items()},
                "post_acquisition_mean_range_m_GT_eval_only": {k: (v[2] / v[0] if v[0] else None) for k, v in state.items()},
                "crash_last_recorded_frame_state": terminal,
                "OBSERVATIONAL_crash_rate_by_actual_speed_half": {
                    "median_split_mps": med, "slower_half": float(lo[:, 1].mean()), "faster_half": float(hi[:, 1].mean())},
            }
    out["arm_density"] = rows_out
    return out


def tf32(primary_unused, per_cell):
    out = {"status": "TECHNICAL_SENSITIVITY_DOES_NOT_CHANGE_PREREGISTERED_READING",
           "known_exposure": "docs/audits/moving_target_tf32_readonly_2026-10-16.md: torch.cdist matmul path; "
                             "RTX 3070 kernel selection erred ~2 m when the bar count is a multiple of 4 (160 here)",
           "b1r_target_motion_backend": "direct_l2_v1 (difference norms; the audited target-motion defect path is not used)",
           "tf32_scope": "TF32 matmul stays on for every float32 matmul (policy, tracker, task) in all arms: shared contract",
           "cdist_sites_active_in_b1r_placement": [
               "navrl_task.py:133 _spawn_footprint_clearance_accepted, called by kroc_eval_adapter.sample_initial_view_for_task "
               "(target, initial view) and navrl_task.py:2398 (pursuer spawn): one query point per batch row"],
           "cdist_sites_not_on_this_path_by_source_reading": [
               "navrl_task.py:2745 _sample_general_target (bypassed when the KROC initial view is set, navrl_task.py:4742)",
               "navrl_task.py:4813/4827 goal clearance and snap (inferred not to run in this branch)",
               "kroc_eval_adapter.py:1069 violation-packet diagnostics (no violation occurred)"],
           "note": "Placement precedes actions. Episode 0 of every environment is placed identically in all arms; later "
                   "reset batches differ by arm because episode lengths differ, so arm independence is supported "
                   "empirically (start-clearance quantiles below), not by construction. The minimum-clearance check can "
                   "detect false acceptances only; a false rejection would shift the distribution without lowering the "
                   "minimum. Excluding 160 bars is a partial sensitivity: the TF32 audit recorded nonzero errors at other "
                   "densities too."}
    gate = {"cells": 0, "violations": 0, "wall_violations": 0, "motion_gate_not_pass": 0, "min_clearance_m": math.inf}
    spawn = {}
    early = {}
    for arm, root in ARMS.items():
        for b in DENSITIES:
            p_clear, t_clear, crash_early, crashes = [], [], 0, 0
            for s in SEEDS:
                cell = root / (CELL % (b, s))
                summ = json.loads((cell / "kroc2027_summary.json").read_text())
                sw = summ["swept_clearance"]
                gate["cells"] += 1
                gate["violations"] += sw["violations"]
                gate["wall_violations"] += sw["wall_violations"]
                gate["min_clearance_m"] = min(gate["min_clearance_m"], sw.get("min_clearance_m", math.inf))
                gate["motion_gate_not_pass"] += summ["motion_gate"]["verdict"] != "PASS"
                _, rows = quota_rows(cell)
                keys = [(r["env_index"], r["env_episode_index"]) for r in rows]
                recs = recorder(cell, keys)
                for r, k in zip(rows, recs):
                    if r["outcome"] == "crash":
                        crashes += 1
                        crash_early += k["actions"] <= 5
                tr = ot.read_trace(cell)
                L, look = tr["layouts"], ot.reset_lookup(tr["layouts"])
                f = tr["frames"]
                f0 = f[f["frame"] == 0]
                first = {(int(x["env"]), int(x["episode"])): x for x in f0}
                for k in keys:
                    if k not in first or k not in look:
                        continue
                    c, h, qq = ot.layout_arrays(L, look[k])
                    pts = np.stack([first[k]["p_pos"][:2], first[k]["t_pos"][:2]]).astype(np.float64)
                    d = surface_distance(pts, c.astype(np.float64), h.astype(np.float64), qq.astype(np.float64))
                    p_clear.append(float(d[0]))
                    t_clear.append(float(d[1]))
            spawn["%s|%d" % (arm, b)] = {
                "pursuer_start_surface_clearance_m": {"min": min(p_clear), "p01": q(p_clear, .01), "median": q(p_clear, .5),
                                                      "share_below_0p4": float(np.mean(np.array(p_clear) < 0.4)), "n": len(p_clear)},
                "target_start_surface_clearance_m": {"min": min(t_clear), "p01": q(t_clear, .01), "median": q(t_clear, .5),
                                                     "share_below_0p4": float(np.mean(np.array(t_clear) < 0.4)), "n": len(t_clear)}}
            early["%s|%d" % (arm, b)] = {"crash_within_5_actions_share_of_episodes": crash_early / (512 * len(SEEDS)),
                                         "crash_within_5_actions_share_of_crashes": (crash_early / crashes) if crashes else None}
    out["target_motion_gate_all_84_evaluation_cells"] = gate
    out["start_clearance_exact_geometry"] = spawn
    out["early_crash"] = early
    # Sensitivity: primary over 70/115/205 only (exclude the multiple-of-4 density).
    prim = {arm: {s: float(np.mean([per_cell["%s|%d|%d" % (arm, s, b)]["capture"] / 512 for b in (70, 115, 205)]))
                  for s in SEEDS} for arm in ARMS}
    out["sensitivity_excluding_160_bars"] = contrasts(prim, [("F", "B-PN"), ("F", "B-PP"), ("B-PN", "B-PP")])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    ver, primary, per_cell = verification(None)
    result = {"schema": "motar.b1r-publication-audit.v1", "raw_root": str(RAW),
              "prereg": "docs/plans/prereg_b1r_classical_baseline_v1.md", "verification": ver,
              "exploratory": exploratory(), "tf32": tf32(primary, per_cell)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False, default=float)
        stream.write("\n")
    print(json.dumps({"written": str(args.out)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
