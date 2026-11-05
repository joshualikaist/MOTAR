"""Summarise S3b dev07 (2026-11-05): per-episode SITL outcome against Isaac's, and step counts. Reads the two
committed reports next to this file; writes dev07_summary.json. usage: python dev07_summary.py"""
import json
import pathlib
import statistics

HERE = pathlib.Path(__file__).resolve().parent
out = {"unit": "S3b dev07", "cells": {}}
for b in (70, 205):
    rep = json.loads((HERE / ("dev07_%d_s3b_report.json" % b)).read_text())
    eps = rep["episodes"]
    agree = sum(1 for e in eps if e["outcome"] == e["isaac_outcome"])
    both_cap = [e for e in eps if e["outcome"] == "capture" and e["isaac_outcome"] == "capture"]
    d = [e["steps"] - e["isaac_steps"] for e in both_cap]
    out["cells"][str(b)] = {
        "episodes": len(eps), "start_check_valid": sum(1 for e in eps if e["start_check"].get("armed") and e["start_check"].get("offboard")),
        "outcome_agree": agree,
        "sitl": {k: sum(1 for e in eps if e["outcome"] == k) for k in ("capture", "crash", "timeout")},
        "isaac": {k: sum(1 for e in eps if e["isaac_outcome"] == k) for k in ("capture", "crash", "timeout")},
        "capture_steps_sitl_minus_isaac": {"median": statistics.median(d) if d else None, "min": min(d) if d else None,
                                           "max": max(d) if d else None, "n": len(d)},
        "rtf_min": min(e["real_time_factor"] for e in eps), "rtf_max": max(e["real_time_factor"] for e in eps),
        "ekf_xy_err_median_max_m": max(e["ekf_vs_truth_xy_err_m"]["median"] for e in eps),
        "ekf_xy_err_max_m": max(e["ekf_vs_truth_xy_err_m"]["max"] for e in eps),
        "ode_iters": rep.get("ode_iters"), "params": rep.get("params"),
    }
(HERE / "dev07_summary.json").write_text(json.dumps(out, indent=1) + "\n")
print(json.dumps(out["cells"], indent=1))
