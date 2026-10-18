#!/usr/bin/env python3
"""P02-only CPU analysis; event intervals are conditional descriptive, never population inference."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "docs/plans/h_r3_p02.json"
PLAN = json.loads(PLAN_PATH.read_text())
ARMS = ("B0", "B1", "B2", "B3", "B4", "B5")
CONTRASTS = (("B3", "B2"), ("B3", "B4"), ("B3", "B5"))
CLASSES = ("NONE", "RANGE_EXIT", "FOV_EXIT", "SCALE_LOSS", "FULL", "CENTRE_ONLY", "PARTIAL", "VISIBLE")
PRIMARY = ("FULL", "CENTRE_ONLY", "PARTIAL")
VARIANTS = {"spatial_primary": "id_camera_identity_correct",
            "spatial_025": "id_camera_identity_correct_025", "spatial_075": "id_camera_identity_correct_075",
            "p01_literal": None, "p01_terminal_corrected": None}
CORE = ("camera", "el0_cls", "el0_visible_px", "reset", "crashed", "captured", "timeout", "before_pos",
        "actual_pos", "actual_vel", "target_pos", "action", "command", "intervening", "digest", "sensor_digest",
        "rng_probe", "raw_nearest_m", "contact_peak_n", "contact_substeps", "nominal_xy", "post_point_xy",
        "post_cap_xy", "budget_error", "shadow_offset", "shadow_clear", "sham_sign", "governor_clearance_m",
        "governor_requested_speed_mps", "governor_executed_speed_mps", "governor_scale")
IDENTITY = ("id_camera_identity_correct", "id_camera_identity_correct_025", "id_camera_identity_correct_075",
            "id_detector_px", "id_visible_px", "id_overlap_px", "id_detector_false_px", "id_detector_overlap_fraction",
            "id_target_recall", "id_mask_iou", "id_detector_bbox", "id_reported_centroid_uv", "id_centroid_near_visible",
            "id_camera_delivered")
SHADOWS = tuple("shadow_%s_%s" % (arm, key) for arm in ("B2", "B3", "B4", "B5")
                for key in ("nominal_xy", "post_point_xy", "post_cap_xy"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def exact(a, b):
    return a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes()


def terminal(tr, env, start, end):
    mask = np.logical_or.reduce([tr[k][start:end, env] for k in ("reset", "crashed", "captured", "timeout")])
    hit = np.flatnonzero(mask)
    return start + int(hit[0]) if len(hit) else None


def load_process(path, failures, identity=True):
    try:
        with np.load(path / "h_r3_p02_trace.npz", allow_pickle=False) as data:
            tr = {key: data[key] for key in data.files}
        meta = json.loads((path / "h_r3_p02_meta.json").read_text())
        done = json.loads((path / "process_done.json").read_text())
        n, steps = len(tr["t0"]), len(tr["camera"])
        for key in CORE + SHADOWS + (IDENTITY if identity else ()):
            if key not in tr or tr[key].shape[:2] != (steps, n):
                raise ValueError("missing or malformed " + key)
        for key in ("digest", "sensor_digest"):
            if tr[key].shape != (steps, n, 32) or tr[key].dtype != np.uint8:
                raise ValueError("full SHA256 required: " + key)
            init = tr["initial_" + key]
            if init.shape != (n, 32) or init.dtype != np.uint8:
                raise ValueError("full initial SHA256 required: " + key)
        if tr["controller_ms"].shape != (steps,) or meta["steps"] != steps or meta["num_envs"] != n:
            raise ValueError("trace/meta dimensions differ")
        if meta.get("nan_fields") or done.get("nan_fields"):
            raise ValueError("process recorded nonfinite telemetry")
        if any(not np.isfinite(v).all() for v in tr.values() if v.dtype.kind == "f"):
            raise ValueError("nonfinite telemetry")
        if done.get("process") != path.name or done.get("actor_obs_dim") != 898:
            raise ValueError("completion identity/actor width differs")
        if meta.get("model_training_flag") is not False or not meta.get("vision_mode"):
            raise ValueError("actor evaluation/vision guard")
        if meta.get("rms_digest_start") != meta.get("rms_digest_end"):
            raise ValueError("RMS changed")
        if identity and meta.get("identity_enabled") is not True:
            raise ValueError("identity recorder disabled")
        if identity and not exact(tr["camera"], tr["id_camera_delivered"]):
            raise ValueError("identity/camera timestamps differ")
        if meta.get("window_steps") != PLAN["intervention"]["window_steps"]:
            raise ValueError("intervention window differs")
        if meta.get("sensor_lineage") != PLAN["switch"]["value_used"]:
            raise ValueError("sensor lineage differs")
        if meta.get("step_dt") != PLAN["intervention"]["dt_s"]:
            raise ValueError("physical action interval differs")
        if {item.get("file") for item in done.get("files", [])} != {"h_r3_p02_trace.npz", "h_r3_p02_meta.json"}:
            raise ValueError("completion file set differs")
        for item in done.get("files", []):
            file = path / item["file"]
            if sha(file) != item["sha256"] or file.stat().st_size != item["bytes"]:
                raise ValueError("completion file hash/size differs: " + item["file"])
        return tr, meta
    except Exception as error:
        failures.append(path.name + ": " + str(error))
        return None


def metric_outcome(tr, env, t0, window, dt):
    end = min(t0 + window, len(tr["camera"]))
    term = terminal(tr, env, t0, end)
    valid_end = term if term is not None else end
    variants = {}
    for name, key in VARIANTS.items():
        stop = term + 1 if name == "p01_literal" and term is not None else valid_end
        flag = tr[key] if key else tr["camera"] & (tr["el0_visible_px"] >= 1)
        hit = np.flatnonzero(flag[t0:stop, env])
        step = t0 + int(hit[0]) if len(hit) else None
        variants[name] = {"reobserved": step is not None,
                          "time_to_reobs_s": (step - t0 + 1) * dt if step is not None else None}
    # Physical telemetry is captured before reset, including the terminal action itself.
    physical_end = term + 1 if term is not None else end
    ix = np.arange(t0, physical_end)
    control = ix[tr["intervening"][ix, env].astype(bool)]

    def motion(rows):
        if len(rows) == 0:
            return {"steps": 0, "exposure_s": 0., "translation_m": 0., "translation_3d_m": 0., "translation_xy_m": 0., "command_xy_sq_integral": 0.,
                    "actual_xy_sq_integral": 0., "stage_xy_sq_integral": {}, "mean_command_speed_mps": None,
                    "mean_actual_speed_mps": None, "min_post_cap_speed_mps": None, "mean_point_attenuation": None,
                    "governor_scale_mean": None, "raw_nearest_min_m": None, "governor_clearance_min_m": None}
        displacement = tr["actual_pos"][rows, env] - tr["before_pos"][rows, env]
        command_speed = np.linalg.norm(tr["command"][rows, env, :2], axis=1)
        actual_speed = np.linalg.norm(tr["actual_vel"][rows, env, :2], axis=1)
        nominal_speed = np.linalg.norm(tr["nominal_xy"][rows, env], axis=1)
        point_speed = np.linalg.norm(tr["post_point_xy"][rows, env], axis=1)
        good = nominal_speed > 0
        return {"steps": int(len(rows)), "exposure_s": float(len(rows) * dt),
                "translation_m": float(np.linalg.norm(displacement, axis=1).sum()),
                "translation_3d_m": float(np.linalg.norm(displacement, axis=1).sum()),
                "translation_xy_m": float(np.linalg.norm(displacement[:, :2], axis=1).sum()),
                "command_xy_sq_integral": float(np.square(command_speed).sum() * dt),
                "actual_xy_sq_integral": float(np.square(actual_speed).sum() * dt),
                "stage_xy_sq_integral": {key: float(np.square(tr[key][rows, env]).sum() * dt)
                                         for key in ("nominal_xy", "post_point_xy", "post_cap_xy")},
                "mean_command_speed_mps": float(command_speed.mean()), "mean_actual_speed_mps": float(actual_speed.mean()),
                "min_post_cap_speed_mps": float(np.linalg.norm(tr["post_cap_xy"][rows, env], axis=1).min()),
                "mean_point_attenuation": float((1 - point_speed[good] / nominal_speed[good]).mean()) if good.any() else None,
                "governor_scale_mean": float(tr["governor_scale"][rows, env].mean()),
                "raw_nearest_min_m": float(tr["raw_nearest_m"][rows, env].min()),
                "governor_clearance_min_m": float(tr["governor_clearance_m"][rows, env].min())}
    later_end = min(t0 + window + 30, len(tr["camera"]))
    later_term = terminal(tr, env, t0, later_end)
    later_end = later_term + 1 if later_term is not None else later_end
    later = slice(t0, later_end)
    primary = variants["spatial_primary"]
    return {"reobserved": primary["reobserved"], "time_to_reobs_s": primary["time_to_reobs_s"], "variants": variants,
            "terminal_step": term, "collision_in_window": bool(tr["crashed"][ix, env].any()),
            "collision_under_arm_control": bool(tr["crashed"][control, env].any()),
            "contact_in_window": bool((tr["contact_substeps"][ix, env] > 0).any()),
            "contact_peak_n": float(tr["contact_peak_n"][ix, env].max(initial=0)),
            "contact_substeps": int(tr["contact_substeps"][ix, env].sum()),
            "capture_in_window": bool(tr["captured"][ix, env].any()),
            "timeout_in_window": bool(tr["timeout"][ix, env].any()),
            "capture_by_w_plus_3s": bool(tr["captured"][later, env].any()),
            "crash_by_w_plus_3s": bool(tr["crashed"][later, env].any()),
            "timeout_by_w_plus_3s": bool(tr["timeout"][later, env].any()),
            "fixed_window": motion(ix), "control_only": motion(control)}


def mean(values):
    usable = [v for v in values if v is not None]
    return float(np.mean(usable)) if usable else None


def summarize(events):
    result = {"n": len(events), "label": "CONDITIONAL_DESCRIPTIVE; NO POPULATION INFERENCE", "arms": {}, "contrasts": {}}
    for arm in ARMS:
        outcomes = [event["results"][arm] for event in events]
        result["arms"][arm] = {"n": len(outcomes), "reobserved": sum(r["reobserved"] for r in outcomes),
                               "rate": mean([r["reobserved"] for r in outcomes]),
                               "variants": {v: {"reobserved": sum(r["variants"][v]["reobserved"] for r in outcomes),
                                                "rate": mean([r["variants"][v]["reobserved"] for r in outcomes])} for v in VARIANTS}}
        for flag in ("collision_in_window", "collision_under_arm_control", "contact_in_window", "capture_in_window",
                     "timeout_in_window", "capture_by_w_plus_3s", "crash_by_w_plus_3s", "timeout_by_w_plus_3s"):
            result["arms"][arm][flag] = sum(r[flag] for r in outcomes)
        for scope in ("fixed_window", "control_only"):
            keys = ("exposure_s", "translation_m", "translation_3d_m", "translation_xy_m", "command_xy_sq_integral", "actual_xy_sq_integral",
                    "mean_command_speed_mps", "mean_actual_speed_mps", "min_post_cap_speed_mps", "mean_point_attenuation",
                    "governor_scale_mean", "raw_nearest_min_m", "governor_clearance_min_m")
            result["arms"][arm][scope] = {k: mean([r[scope][k] for r in outcomes]) for k in keys}
    random = np.random.default_rng(0)
    for x, y in CONTRASTS:
        contrasts = {}
        for variant in VARIANTS:
            a = np.array([e["results"][x]["variants"][variant]["reobserved"] for e in events], dtype=int)
            b = np.array([e["results"][y]["variants"][variant]["reobserved"] for e in events], dtype=int)
            diff = a - b
            ci = None
            if len(diff):
                draws = random.integers(0, len(diff), size=(10000, len(diff)))
                ci = np.percentile(diff[draws].mean(axis=1), [2.5, 97.5]).tolist()
            contrasts[variant] = {"rate_diff": mean(diff.tolist()), "n10": int(((a == 1) & (b == 0)).sum()),
                                  "n01": int(((a == 0) & (b == 1)).sum()), "conditional_event_bootstrap95": ci}
        result["contrasts"][x + "-" + y] = dict(contrasts["spatial_primary"], variants=contrasts)
    return result


def population(events, seeds, valid):
    per_seed = {str(seed): summarize([e for e in events if e["seed"] == seed]) for seed in seeds}
    equal = {"unit": "seed cluster", "planned_seed_clusters": len(seeds), "population_inference": "NOT_SUPPORTED",
             "two_seed_minimum_two_sided_signflip_p": 0.5, "arms": {}, "contrasts": {}}
    for arm in ARMS:
        values = [r["arms"][arm]["rate"] for r in per_seed.values()]
        equal["arms"][arm] = {"mean_rate": mean(values), "contributing_seed_clusters": sum(v is not None for v in values)}
    for x, y in CONTRASTS:
        key = x + "-" + y
        values = [r["contrasts"][key]["rate_diff"] for r in per_seed.values()]
        usable = [v for v in values if v is not None]
        equal["contrasts"][key] = {"mean_rate_diff": mean(values), "observed_seed_range": [min(usable), max(usable)] if usable else None,
                                  "contributing_seed_clusters": len(usable)}
    return {"n": len(events), "integrity_valid": valid, "interpretation": "DESCRIPTIVE" if valid else "INVALID_INTEGRITY_FAILURE",
            "per_seed": per_seed, "equal_seed": equal, "conditional_descriptive": summarize(events)}


def analyse(run, smoke=False):
    run = Path(run)
    failures, inconclusive = [], []
    data, extras = {}, {}
    for path in sorted(run.glob("seed*_B*")):
        if not path.is_dir():
            continue
        match = re.fullmatch(r"seed(\d+)_(B[0-5])", path.name)
        if match:
            process = load_process(path, failures)
            if process is not None:
                data.setdefault(int(match[1]), {})[match[2]] = process
        elif smoke and re.fullmatch(r"seed\d+_B0_(forced|noeval)", path.name):
            process = load_process(path, failures, identity=not path.name.endswith("noeval"))
            if process is not None:
                extras[path.name] = process
    seeds = [PLAN["run"]["smoke"]["seed"]] if smoke else PLAN["run"]["seeds"]
    expected = {"seed%d_%s" % (seed, arm) for seed in seeds for arm in ARMS}
    if smoke:
        expected |= {"seed%d_B0_%s" % (seeds[0], suffix) for suffix in ("forced", "noeval")}
    found = {"seed%d_%s" % (seed, arm) for seed, arms in data.items() for arm in arms} | set(extras)
    if found != expected:
        failures.append("process set differs: missing=%s extra=%s" % (sorted(expected - found), sorted(found - expected)))
    provenance = {"analysis_sha256": sha(__file__), "plan_sha256": sha(PLAN_PATH), "source_commit": None}
    try:
        report = json.loads((run / "run_report.json").read_text())
        provenance.update(source_commit=report["source_commit"], run_report_sha256=sha(run / "run_report.json"),
                          source_input_sha256=report["preflight"]["hr3_input_sha256"])
        if report["status"] != "COMPLETED" or report.get("stop_reason"):
            failures.append("runner did not complete cleanly")
        if report["plan_sha256"] != provenance["plan_sha256"]:
            failures.append("runner plan SHA differs")
        binding = report["preflight"]["hr3_input_sha256"].get("tools/h_r3_p02_analysis.py")
        if binding != provenance["analysis_sha256"]:
            failures.append("analysis source differs from run source binding")
        completed = report["processes"]
        if {p["id"] for p in completed} != expected or len(completed) != len(expected) or any(p.get("returncode") != 0 for p in completed):
            failures.append("runner process completion set differs")
    except Exception as error:
        failures.append("run report: " + str(error))
    events, own_safety = [], {a: [] for a in ARMS}
    target_checks, exposure_failures, nonzero, budget_max = [], [], {"B3": 0, "B4": 0}, 0.
    shadow_proof = []
    for seed, arms in sorted(data.items()):
        for arm, (tr, meta) in arms.items():
            if meta.get("arm") != arm or meta.get("seed") != seed:
                failures.append("process meta identity differs: %s/%s" % (seed, arm))
            expected_cfg = PLAN["run"]["smoke"] if smoke else PLAN["run"]
            if meta["num_envs"] != expected_cfg["num_envs"] or meta["steps"] != expected_cfg["steps"]:
                failures.append("process dimensions differ from plan: %s/%s" % (seed, arm))
            budget_max = max(budget_max, float(tr["budget_error"].max(initial=0)))
            reference_norm = np.linalg.norm(tr["shadow_B3_nominal_xy"], axis=-1)
            independent_error = max(float(np.abs(np.linalg.norm(tr["shadow_%s_nominal_xy" % a], axis=-1) - reference_norm).max(initial=0))
                                    for a in ("B4", "B5"))
            budget_max = max(budget_max, independent_error)
            selected_ok = True
            selected_shadow = "B2" if arm == "B1" else arm
            if selected_shadow != "B0":
                mask = tr["intervening"].astype(bool)
                selected_ok = all(exact(tr[key][mask], tr["shadow_%s_%s" % (selected_shadow, key)][mask])
                                  for key in ("nominal_xy", "post_point_xy", "post_cap_xy"))
                if not selected_ok:
                    failures.append("selected stages differ from same-state shadow: %s/%s" % (seed, arm))
            shadow_proof.append({"seed": seed, "arm": arm, "max_independent_nominal_error_mps": independent_error,
                                 "selected_matches_shadow": selected_ok})
            if arm in nonzero:
                nonzero[arm] += int(((tr["shadow_offset"] != 0) & tr["intervening"]).sum())
            for env, branch in enumerate(tr["t0"]):
                t0 = int(branch)
                if t0 < 0:
                    if tr["intervening"][:, env].any():
                        exposure_failures.append("%s/%s/env%d active without trigger" % (seed, arm, env))
                    continue
                if t0 < 1 or t0 + PLAN["intervention"]["window_steps"] > len(tr["camera"]):
                    failures.append("invalid trigger boundary %s/%s/env%d" % (seed, arm, env))
                    continue
                outcome = metric_outcome(tr, env, t0, PLAN["intervention"]["window_steps"], meta["step_dt"])
                own_safety[arm].append(outcome["collision_in_window"])
                end = t0 + PLAN["intervention"]["window_steps"]
                term = terminal(tr, env, t0, end)
                expected_active = np.zeros(len(tr["camera"]), dtype=bool)
                expected_active[t0:term + 1 if term is not None else end] = True
                if not exact(tr["intervening"][:, env], expected_active):
                    exposure_failures.append("%s/%s/env%d fixed20 exposure differs" % (seed, arm, env))
        if set(arms) != set(ARMS):
            continue
        ref, meta = arms["B0"]
        if any(t[0]["camera"].shape != ref["camera"].shape for t in arms.values()):
            failures.append("arm dimensions differ for seed%d" % seed)
            continue
        for arm in ARMS[1:]:
            tr = arms[arm][0]
            for key in ("initial_digest", "initial_sensor_digest"):
                if not exact(tr[key], ref[key]):
                    failures.append("seed%d/%s %s differs" % (seed, arm, key))
        for env, branch in enumerate(ref["t0"]):
            t0 = int(branch)
            same_t0 = all(int(arms[a][0]["t0"][env]) == t0 for a in ARMS)
            stop = len(ref["camera"]) if t0 < 0 else t0
            prefix_ok = all(exact(arms[a][0][key][:stop, env], ref[key][:stop, env])
                            for a in ARMS for key in ("digest", "sensor_digest"))
            if not same_t0 or not prefix_ok:
                failures.append("seed%d/env%d %s" % (seed, env, "t0 differs" if not same_t0 else "prebranch/no-trigger state or sensor differs"))
            if t0 < 1 or t0 + PLAN["intervention"]["window_steps"] > len(ref["camera"]):
                continue
            cls_id = int(ref["el0_cls"][t0 - 1, env])
            if not 0 <= cls_id < len(CLASSES):
                failures.append("unknown event class")
                continue
            cls = CLASSES[cls_id]
            results = {a: metric_outcome(arms[a][0], env, t0, PLAN["intervention"]["window_steps"], meta["step_dt"]) for a in ARMS}
            preloss = {a: bool(arms[a][0]["id_camera_identity_correct"][t0 - 2, env]) if t0 >= 2 else None for a in ARMS}
            end = t0 + PLAN["intervention"]["window_steps"]
            terms = [terminal(arms[a][0], env, t0, end) for a in ARMS]
            shared_end = min([end] + [t for t in terms if t is not None])
            equality = {key: all(exact(arms[a][0][key][t0:shared_end, env], ref[key][t0:shared_end, env]) for a in ARMS)
                        for key in ("target_pos", "rng_probe")}
            if not all(equality.values()):
                failures.append("seed%d/env%d target/RNG noninterference differs" % (seed, env))
            target_checks.append(dict(seed=seed, env=env, shared_steps=shared_end - t0, **equality))
            events.append({"seed": seed, "env": env, "t0": t0, "el0_class": cls, "primary": cls in PRIMARY,
                           "prefix_matched": same_t0 and prefix_ok, "results": results,
                           "preloss_online_camera": bool(ref["camera"][t0 - 2, env]) if t0 >= 2 else None,
                           "preloss_spatial_identity_by_arm": preloss,
                           "confirmed_preloss_spatial_identity": preloss["B0"] is True,
                           "any_online_camera_acquisition_before_loss": bool(ref["camera"][:t0 - 1, env].any()),
                           "any_spatial_identity_before_loss": bool(ref["id_camera_identity_correct"][:t0 - 1, env].any())})
    failures.extend(exposure_failures)
    if budget_max > PLAN["controller"]["same_state_norm_tolerance_mps"]:
        failures.append("same-state nominal budget mismatch")
    safety = {}
    for arm, values in own_safety.items():
        fraction = mean(values)
        safety[arm] = {"own_triggered_windows": len(values), "collision_windows": sum(values), "collision_fraction": fraction}
        if smoke and arm != "B0" and not values:
            inconclusive.append("no scripted trigger: " + arm)
        if smoke and arm != "B0" and fraction is not None and fraction > PLAN["run"]["smoke"]["collision_stop_fraction"]:
            failures.append("own-trigger collision threshold exceeded: " + arm)
    smoke_checks = {}
    if smoke and seeds[0] in data and "B0" in data[seeds[0]]:
        ref = data[seeds[0]]["B0"][0]
        for suffix, fields in (("forced", ("digest", "sensor_digest", "rng_probe")),
                               ("noeval", ("digest", "sensor_digest", "action", "rng_probe"))):
            name = "seed%d_B0_%s" % (seeds[0], suffix)
            if name not in extras:
                smoke_checks[suffix] = False
                continue
            tr = extras[name][0]
            extra_meta = extras[name][1]
            if suffix == "forced" and (not extra_meta.get("forced_reset") or tr["reset"].shape[1] <= 15 or
                                        len(tr["reset"]) <= 40 or not tr["reset"][40, 15]):
                failures.append("forced reset was not recorded at step40/env15")
            if suffix == "noeval" and extra_meta.get("identity_enabled") is not False:
                failures.append("noeval process did not disable evaluator")
            keep = [i for i in range(ref["camera"].shape[1]) if suffix != "forced" or i != 15]
            ok = all(exact(tr[key][:, keep], ref[key][:, keep]) for key in fields)
            ok &= all(exact(tr[key][keep], ref[key][keep]) for key in ("initial_digest", "initial_sensor_digest"))
            smoke_checks[suffix] = bool(ok)
            if not ok:
                failures.append("smoke noninterference failed: " + suffix)
        for arm, count in nonzero.items():
            if not count:
                inconclusive.append("no active nonzero lateral sample: " + arm)
    valid = not failures
    primary = [e for e in events if e["primary"]]
    compute = {}
    for arm in ARMS:
        values = [tr["controller_ms"] for arms in data.values() for a, (tr, _) in arms.items() if a == arm]
        if values:
            v = np.concatenate(values)
            compute[arm] = {"mean": float(v.mean()), "p95": float(np.percentile(v, 95)), "p99": float(np.percentile(v, 99))}
    effort = {}
    for x, y in CONTRASTS:
        metrics = {}
        for key in ("translation_m", "command_xy_sq_integral", "actual_xy_sq_integral"):
            relative = []
            omitted = 0
            for e in primary:
                a, b = (e["results"][arm]["control_only"][key] for arm in (x, y))
                denominator = (abs(a) + abs(b)) / 2
                if denominator == 0:
                    omitted += 1
                else:
                    relative.append(abs(a - b) / denominator)
            value = mean(relative)
            metrics[key] = {"mean_absolute_symmetric_relative_difference": value, "zero_denominator_not_imputed": omitted,
                            "within_10_percent": value <= .1 if value is not None and not omitted else None}
        effort[x + "-" + y] = metrics
    return {"schema": "motar.h-r3-p02-analysis.v1", "run": str(run.resolve()), "smoke": smoke,
            "smoke_status": ("FAIL" if failures else "INCONCLUSIVE" if inconclusive else "PASS") if smoke else "NOT_APPLICABLE",
            "integrity": {"status": "PASS" if valid else "FAIL", "failures": failures, "inconclusive": inconclusive},
            "provenance": provenance, "counts": {"expected_processes": len(expected), "loaded_processes": len(found),
                "reference_trigger_events": len(events), "matched_events": sum(e["prefix_matched"] for e in events),
                "primary_events": len(primary), "by_class": {c: sum(e["el0_class"] == c for e in events) for c in CLASSES}},
            "primary": population(primary, seeds, valid),
            "confirmed_preloss_primary_diagnostic_only": population(
                [e for e in primary if e["confirmed_preloss_spatial_identity"]], seeds, valid),
            "separate_classes": {c: population([e for e in events if e["el0_class"] == c], seeds, valid) for c in CLASSES if c not in PRIMARY},
            "primary_strata": {c: population([e for e in primary if e["el0_class"] == c], seeds, valid) for c in PRIMARY},
            "safety_own_triggered_windows": safety, "same_state_budget_max_error_mps": budget_max,
            "same_state_shadow_proof": shadow_proof,
            "active_nonzero_lateral_samples": nonzero, "smoke_noninterference": smoke_checks,
            "target_rng_noninterference": target_checks, "compute_ms": compute, "effort_equivalence_diagnostic": effort,
            "effort_relative_definition": "abs(a-b)/((abs(a)+abs(b))/2); zero denominators not imputed",
            "mechanical_energy": "NOT_RECORDED; squared-speed integrals are kinematic proxies, units m^2/s",
            "motion_definition": "translation_m aliases 3D actual_pos-before_pos displacement; translation_xy_m is horizontal; squared-speed integrals use horizontal command/velocity",
            "events": events}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)
    result = analyse(args.run, smoke=args.smoke)
    with Path(args.out).open("x") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({k: result[k] for k in ("smoke_status", "integrity", "counts")}, indent=2))
    return 0 if result["integrity"]["status"] == "PASS" and (not args.smoke or result["smoke_status"] == "PASS") else 2


if __name__ == "__main__":
    raise SystemExit(main())
