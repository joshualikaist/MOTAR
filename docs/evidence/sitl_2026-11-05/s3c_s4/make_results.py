"""After S3c v1 and S4 v1 attempt01 ended (2026-11-07): build the compact evidence records from the raw attempts.

  nice -n 10 python3 -B ~/motar_tmp/s3c_s4_results_20261107/make_results.py --out <dir>

Writes, refusing to overwrite, into <dir> (staging: ~/motar_tmp/s3c_s4_results_20261107/staged; after the CAM-K1
training ends it is run once more with --out <worktree>/docs/evidence/sitl_2026-11-05/s3c_s4):
  execution_record.json       bindings, the smokes, per-arm descriptive numbers, the per-scene tables
  reading_s3c_attempt01.json  byte copy of ~/motar_tmp/s3c_20261106/attempt01/reading.json
  reading_s4_attempt01.json   byte copy of ~/motar_tmp/s4_20261106/attempt01/reading.json
Every number the README and WORKLOG quote must come from these files. The `descriptive` block is NOT part of either
preregistration's reported list; it is labelled exploratory and is only used to say what to look at next.
"""
import argparse
import hashlib
import json
import pathlib
import shutil
import statistics

T = pathlib.Path.home() / "motar_tmp"
UNITS = {"s3c": T / "s3c_20261106", "s4": T / "s4_20261106"}
PROTOCOL = {"s3c": "s3c-v1", "s4": "s4-v1"}
CLEAR_COL, DIST_COL = 20, 19            # steps.npy columns (tools/sitl_s3b_formal._fly)
W = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly")
S3B_REC = W / "docs/evidence/sitl_2026-11-05/s3b_formal/execution_record.json"   # committed in 0616cb9
S3B_A = [p for p in json.loads(S3B_REC.read_text())["per_scene"] if p["block"] == "A"]
# sensor_steps.npy columns (tools/sitl_s4._fly): 0 k, 1 lidar_t, 2 rgb_t, 3 depth_t, 4 target_t, 5 colour px,
# 6 shadow target px, 7 camera_visible, 8 |lidar-shadow| median, 9 |lidar-shadow| max, 10 |depth-shadow| median,
# 11 lidar ground removed, 12 depth ground removed, 13 lidar_visible, 14 Gazebo sim_now, 15 PX4 time
S_CAMVIS, S_LDMAX, S_DDMED, S_DGND, S_SIMNOW, S_PX4T = 7, 9, 10, 12, 14, 15
ROTOR_REACH_M = {   # centre-to-rotor-disc edge; rotor collision cylinders have radius 0.128 m in each SDF
    "navrl_quad": 0.31,     # S3b results review r1 MAJOR-1 (rotors at +-0.13, +-0.13)
    "navrl_quad_s4": 0.31,  # navrl_quad's collisions (only visuals removed)
    "navrl_iris": 0.384}    # navrl_iris.sdf rotors at (+-0.13, +-0.22/0.20): sqrt(0.13^2 + 0.22^2) + 0.128
IRIS_SDF = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/"
                        "sitl_gazebo-classic/models/navrl_iris/navrl_iris.sdf")
CHAIN_LOG = T / "sitl_chain_20261106" / "chain.log"
ARM_MODEL = {"iris": "navrl_iris", "delay1": "navrl_quad"}   # S3c rows carry no model field (attempt.json arms)
QUOTED_LOGS = {"s3c": [40, 66, 77], "s4": []}   # episodes whose px4.log lines the README quotes


def clopper_pearson(k, n, a=0.05):
    """Exact two-sided interval by bisection on the binomial CDF (stdlib only)."""
    from math import comb

    def cdf(x, p):
        return sum(comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(0, x + 1))

    def solve(f, lo=0.0, hi=1.0):
        for _ in range(200):
            mid = (lo + hi) / 2
            if f(mid):
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2
    low = 0.0 if k == 0 else solve(lambda p: 1 - cdf(k - 1, p) < a / 2)
    high = 1.0 if k == n else solve(lambda p: cdf(k, p) > a / 2)
    return [round(low, 4), round(high, 4)]


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def med(v):
    return statistics.median(v) if v else None


def unit_record(key):
    import numpy as np
    U = UNITS[key]
    A = U / "attempt01"
    attempt = json.loads((A / "attempt.json").read_text())
    final = json.loads((A / "final.json").read_text())
    reading = json.loads((A / "reading.json").read_text())
    smoke = json.loads((U / "smoke01" / "final.json").read_text())
    smoke_rows = [json.loads(x) for x in (U / "smoke01" / "episodes.jsonl").read_text().splitlines() if x.strip()]
    rows = [json.loads(x) for x in (A / "episodes.jsonl").read_text().splitlines() if x.strip()]
    assert reading.get("protocol") == PROTOCOL[key], reading.get("protocol")
    assert attempt.get("development_smoke") is False
    per_scene, manifest = [], {}
    for r in rows:
        d = A / ("ep%03d" % r["index"])
        s = np.load(d / "steps.npy") if (d / "steps.npy").is_file() else np.zeros((0, 27))
        manifest["ep%03d/steps.npy" % r["index"]] = sha(d / "steps.npy") if (d / "steps.npy").is_file() else None
        if key == "s4":
            f = d / "sensor_steps.npy"
            manifest["ep%03d/sensor_steps.npy" % r["index"]] = sha(f) if f.is_file() else None
        if r["index"] in QUOTED_LOGS[key]:
            manifest["ep%03d/logs/px4.log" % r["index"]] = sha(d / "logs" / "px4.log")
        p = {
            "index": r["index"], "arm": r["arm"], "env": r["env"], "bars": r["bars"], "model": r.get("model"),
            "valid": r["valid"], "invalid_reason": r["invalid_reason"], "outcome": r["outcome"],
            "isaac_outcome": r["isaac_outcome"], "crash_reason": r.get("crash_reason"), "steps": r.get("steps"),
            "min_clearance_m": float(s[:, CLEAR_COL].min()) if len(s) else None,
            "min_clearance_step": int(s[int(np.argmin(s[:, CLEAR_COL])), 0]) if len(s) else None,
            "ekf_xy_err_over_1m_first_step": (lambda e: int(s[int(np.argmax(e > 1.0)), 0]) if (e > 1.0).any()
                                              else None)(np.linalg.norm(s[:, 2:4] - s[:, 23:25], axis=1))
            if len(s) else None,
            "z_true_min_m": float(s[:, 25].min()) if len(s) else None,
            "min_target_dist_m": r.get("min_dist_m"),
            "late_max_s": r.get("late_max_s"), "rtf": r.get("real_time_factor"),
            "ekf_xy_err_max_m": (r.get("ekf_vs_truth_xy_err_m") or {}).get("max"),
            "params_ok": not r.get("param_problems"), "kernel_faults": r.get("kernel_faults"),
            "error": r.get("error")}
        if key == "s4":
            sn = r.get("sensor") or {}
            p["sensor"] = {k: sn.get(k) for k in (
                "steps_motar_visible", "steps_camera_visible_when_motar_visible",
                "steps_lidar_only_visible_when_motar_visible", "colour_px_median_when_motar_visible",
                "motar_px_median_when_motar_visible", "frame_age_max_s", "frame_age_median_s")}
            p["hover_guard"] = r.get("hover_guard")
            S = np.load(d / "sensor_steps.npy")
            last = S[-10:]
            dt = S[:, S_SIMNOW] - S[:, S_PX4T]
            p["sensor_derived"] = {
                "camera_detection_in_last10_steps": bool((last[:, S_CAMVIS] > 0.5).any()),
                "step0_lidar_absdiff_max_m": float(S[0, S_LDMAX]),
                "per_step_lidar_absdiff_max_median_m": float(np.median(S[:, S_LDMAX])),
                "steps": int(len(S)),
                "steps_zero_depth_ground_removed": int((S[:, S_DGND] == 0).sum()),
                "steps_depth_absdiff_median_over_1m": int((S[:, S_DDMED] > 1.0).sum()),
                "simnow_minus_px4_s_min_max": [float(dt.min()), float(dt.max())],
                "steps_rgb_older_than_target_after_step0": int((S[1:, 2] < S[1:, 4]).sum()),
                "step0_rgb_older_than_target": bool(S[0, 2] < S[0, 4])}
        model = r.get("model") or ARM_MODEL.get(r["arm"])
        p["model"] = model
        reach = ROTOR_REACH_M.get(model)
        p["rotor_reach_m"] = reach
        p["within_rotor_reach"] = bool(p["min_clearance_m"] is not None and reach is not None
                                       and p["min_clearance_m"] < reach)
        per_scene.append(p)
    arms = sorted({p["arm"] for p in per_scene}, key=[p["arm"] for p in per_scene].index)
    descriptive = {}
    for arm in arms:
        v = [p for p in per_scene if p["arm"] == arm and p["valid"]]
        trans = {}
        for p in v:
            k = "Isaac %s -> SITL %s%s" % (p["isaac_outcome"], p["outcome"],
                                          (" (%s)" % p["crash_reason"]) if p["crash_reason"] else "")
            trans[k] = trans.get(k, 0) + 1
        s3b_cap = {p["env"]: p["steps"] for p in S3B_A if p["valid"] and p["outcome"] == "capture"}
        both = [p for p in v if p["outcome"] == "capture" and p["env"] in s3b_cap]
        dsc = {
            "valid": len(v), "transitions": trans,
            "capture_steps_median": med([p["steps"] for p in v if p["outcome"] == "capture"]),
            "capture_steps_median_scenes_captured_in_both": {
                "n": len(both), "this_arm": med([p["steps"] for p in both]),
                "s3b_block_A": med([s3b_cap[p["env"]] for p in both])},
            "crash_steps": sorted(p["steps"] for p in v if p["outcome"] == "crash"),
            "crash_min_target_dist_m_median": med([p["min_target_dist_m"] for p in v if p["outcome"] == "crash"]),
            "ekf_xy_err_max_m_median_over_episodes": med([p["ekf_xy_err_max_m"] for p in v
                                                           if p["ekf_xy_err_max_m"] is not None]),
            "ekf_xy_err_max_over_0p5m_episodes": sum(1 for p in v if (p["ekf_xy_err_max_m"] or 0) > 0.5),
            "ekf_xy_err_max_top2": sorted(([round(p["ekf_xy_err_max_m"], 3), p["env"], p["outcome"]] for p in v
                                           if p["ekf_xy_err_max_m"] is not None), reverse=True)[:2],
            "crash_clopper_pearson_95": {"k": sum(1 for p in v if p["outcome"] == "crash"), "n": len(v),
                                         "interval": clopper_pearson(sum(1 for p in v if p["outcome"] == "crash"),
                                                                     len(v))},
            "rotor_reach_m": sorted({p["rotor_reach_m"] for p in v})[0] if v else None,
            "non_crash_within_rotor_reach": sorted([p["env"], p["outcome"], round(p["min_clearance_m"], 3)]
                                                   for p in v if p["outcome"] != "crash" and p["within_rotor_reach"]),
            "invalid_episodes": [[p["env"], p["invalid_reason"], p["outcome"], p["late_max_s"]]
                                 for p in per_scene if p["arm"] == arm and not p["valid"]],
        }
        if key == "s4":
            zero = [p for p in v if (p["sensor"]["steps_camera_visible_when_motar_visible"] or 0) == 0]
            dsc["episodes_with_zero_camera_detections"] = {
                "n": len(zero), "outcomes": {o: sum(1 for p in zero if p["outcome"] == o)
                                             for o in sorted({p["outcome"] for p in zero})},
                "motar_px_median_of_those": med([p["sensor"]["motar_px_median_when_motar_visible"] for p in zero])}
            rest = [p for p in v if p not in zero]
            dsc["episodes_with_camera_detections"] = {
                "n": len(rest), "outcomes": {o: sum(1 for p in rest if p["outcome"] == o)
                                             for o in sorted({p["outcome"] for p in rest})},
                "definition": "at least one camera_visible step among the steps the shadow shows >= 2 target px;"
                              " says nothing about detection at the crash"}
            cr = [p for p in v if p["outcome"] == "crash"]
            dsc["crashes_with_camera_detection_in_last10_steps"] = {
                "k": sum(1 for p in cr if p["sensor_derived"]["camera_detection_in_last10_steps"]), "n": len(cr)}
            s0 = [p["sensor_derived"]["step0_lidar_absdiff_max_m"] for p in v]
            dsc["step0_hover_lidar_absdiff_max"] = {
                "values_m": [round(x, 3) for x in s0],
                "episodes_in_6p0_to_6p6m": sum(1 for x in s0 if 6.0 <= x <= 6.6),
                "note": "6.0-6.6 m is the raw arm's ground-return signature at hover (results review r1 MAJOR-2)"}
            dsc["per_step_sensor_disagreement"] = {
                "median_over_episodes_of_per_step_lidar_absdiff_max_m":
                    med([p["sensor_derived"]["per_step_lidar_absdiff_max_median_m"] for p in v]),
                "steps": sum(p["sensor_derived"]["steps"] for p in v),
                "steps_zero_depth_ground_removed": sum(p["sensor_derived"]["steps_zero_depth_ground_removed"]
                                                       for p in v),
                "steps_depth_absdiff_median_over_1m": sum(p["sensor_derived"]["steps_depth_absdiff_median_over_1m"]
                                                          for p in v),
                "episodes_with_depth_absdiff_median_over_1m": sum(
                    1 for p in v if p["sensor_derived"]["steps_depth_absdiff_median_over_1m"] > 0)}
            dsc["clock_and_frame_order"] = {
                "simnow_minus_px4_s_min": min(p["sensor_derived"]["simnow_minus_px4_s_min_max"][0] for p in v),
                "simnow_minus_px4_s_max": max(p["sensor_derived"]["simnow_minus_px4_s_min_max"][1] for p in v),
                "steps_rgb_older_than_target_after_step0": sum(
                    p["sensor_derived"]["steps_rgb_older_than_target_after_step0"] for p in v),
                "episodes_whose_step0_rgb_predates_the_target_placement": sum(
                    p["sensor_derived"]["step0_rgb_older_than_target"] for p in v)}
        descriptive[arm] = dsc
    return {
        "unit": "%s v1 attempt01" % key.upper(), "prereg": attempt["prereg"], "scope": attempt["scope"],
        "protocol": attempt["protocol"], "git_head": attempt["git"]["head"], "git_dirty": attempt["git"]["dirty"],
        "approval_sha256": attempt["approval_sha256"], "f_sha256": attempt["f_sha256"],
        "environment": attempt["environment"], "arms": attempt["arms"],
        "plugin": {k: attempt.get(k) for k in ("display", "plugin_sha256", "plugin_source_sha256",
                                               "model_sdf_sha256") if k in attempt},
        "raw": {"attempt_dir": str(A), "attempt.json": sha(A / "attempt.json"), "final.json": sha(A / "final.json"),
                "episodes.jsonl": sha(A / "episodes.jsonl"), "reading.json": sha(A / "reading.json")},
        "smoke01": {"final": smoke, "episodes": [{k: r.get(k) for k in ("arm", "env", "valid", "invalid_reason",
                                                                         "outcome", "isaac_outcome", "steps")}
                                                  for r in smoke_rows],
                    "final.json": sha(U / "smoke01" / "final.json")},
        "final": final,
        "reading_integrity_problems": reading["integrity_problems"],
        "descriptive_not_preregistered": descriptive,
        "per_scene": per_scene,
        "raw_manifest_sha256": manifest,
    }


def matched_ground_raw(s4):
    g = {p["env"]: p for p in s4["per_scene"] if p["arm"] == "ground" and p["valid"]}
    r = {p["env"]: p for p in s4["per_scene"] if p["arm"] == "raw" and p["valid"]}
    envs = sorted(set(g) & set(r))
    pairs = {}
    for e in envs:
        k = "ground %s / raw %s" % (g[e]["outcome"], r[e]["outcome"])
        pairs[k] = pairs.get(k, 0) + 1
    return {"scenes_valid_in_both": len(envs), "ground_captures": sum(g[e]["outcome"] == "capture" for e in envs),
            "raw_captures": sum(r[e]["outcome"] == "capture" for e in envs), "pairs": pairs}


def chain_times():
    lines = [x for x in CHAIN_LOG.read_text().splitlines() if x.startswith("2026-11-07")]
    return {"path": str(CHAIN_LOG), "sha256": sha(CHAIN_LOG), "lines_2026_11_07": lines}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    targets = {"execution_record.json": None, "reading_s3c_attempt01.json": UNITS["s3c"] / "attempt01/reading.json",
               "reading_s4_attempt01.json": UNITS["s4"] / "attempt01/reading.json"}
    for name in targets:
        if (out / name).exists():
            raise SystemExit("refusing to overwrite %s" % (out / name))
    rec = {"written_by": "~/motar_tmp/s3c_s4_results_20261107/make_results.py",
           "script_sha256": sha(__file__),
           "s3b_reference_record": {"path": str(S3B_REC.relative_to(W)), "sha256": sha(S3B_REC)},
           "iris_sdf": {"path": str(IRIS_SDF), "sha256": sha(IRIS_SDF)},
           "chain": chain_times(),
           "s3c": unit_record("s3c"), "s4": unit_record("s4")}
    rec["s4"]["descriptive_not_preregistered"]["ground_vs_raw_matched_scenes"] = matched_ground_raw(rec["s4"])
    (out / "execution_record.json").write_text(json.dumps(rec, indent=1, sort_keys=True) + "\n")
    for name, src in targets.items():
        if src is not None:
            shutil.copyfile(src, out / name)
            assert sha(src) == sha(out / name)
    print(json.dumps({k: rec[k]["descriptive_not_preregistered"] for k in ("s3c", "s4")}, indent=1))


if __name__ == "__main__":
    main()
