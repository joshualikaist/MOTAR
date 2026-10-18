#!/usr/bin/env python3
"""SL1D-C02 sensor-only canary (S-L1D amendment 1): frozen preflight and fail-closed supervision.

Reuses the qualified S-L1A/S-L1B/S-L1D supervision read-only. Importing launches nothing.
Smoke runs are mechanics only, never evidence.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
import s_l1d_run as dr  # noqa: E402

a, b, base, el0 = dr.a, dr.b, dr.base, dr.el0
AMENDMENT_PATH = ROOT / "docs/plans/s_l1d_v1_amendment1.json"
LAUNCHER = ROOT / "tools/s_l1d_c02_launcher.py"
PRODUCTION_SOURCE = "76f0f4cc08ceecf9a05e628ea82d956123d29524"
SL1D_C01_ROOT = base.resolve("../aerial_gym_simulator/results/s_l1d_v1/SL1D-C01")
SL1D_C01_MANIFEST_COPY = ROOT / "docs/evidence/s_l1d_2026-10-18/run_records/RAW_MANIFEST.json"
SENSOR_LINEAGE = {"s_l1c": "s_l1c_v1", "s_l1d": "s_l1d_v1"}


def load_plan():
    p = dr.load_plan()
    amendment = a.read_json(AMENDMENT_PATH)
    run = copy.deepcopy(p["run"])
    run.update({k: v for k, v in amendment["run"].items() if k != "closed_loop"})
    p["run"] = run
    p["amendment1"] = amendment
    return p


def process_specs(p):
    run = p["run"]
    return [{"id": "sensors_" + mode, "batch": "SENSORS", "mode": mode, "config": "static", "arm": "P2",
             "num_envs": run["sensor_num_envs"], "seed": run["sensor_seed"], "cap_s": run["process_wall_cap_s"],
             "lineage": SENSOR_LINEAGE[mode]} for mode in run["sensor_modes"]]


def preflight(p, approved, run_dir, smoke=False):
    old = a.read_json(base.resolve(p["references"]["eq1d_plan"]))
    problems, checks = base.preflight(old, approved, run_dir, smoke)
    paths = (dr.PLAN_PATH, dr.PLAN_PATH.with_suffix(".md"), AMENDMENT_PATH, AMENDMENT_PATH.with_suffix(".md"),
             Path(__file__).resolve(), LAUNCHER, ROOT / "tools/s_l1d_launcher.py", ROOT / "tools/s_l1d_run.py",
             ROOT / "tools/s_l1c_run.py", ROOT / "tools/s_l1b_run.py", ROOT / "tools/s_l1a_run.py")
    checks["c02_committed_inputs"] = {str(x.relative_to(ROOT)): a.committed(x) for x in paths}
    checks["c02_input_sha256"] = {str(x.relative_to(ROOT)): base.sha256_file(x) for x in paths if x.is_file()}
    if not smoke and not all(checks["c02_committed_inputs"].values()):
        problems.append("SL1D-C02 plan/amendment/code not committed unchanged")
    diff = base.git("diff", "--quiet", PRODUCTION_SOURCE, "HEAD", "--", "aerial_gym", "motar_eval").returncode
    checks["production_identical_to_76f0f4c"] = diff == 0
    if diff != 0:
        problems.append("production source differs from the qualified S-L1D source 76f0f4c")
    if checks.get("policy_f_sha256") != p["policy_f_sha256"]:
        problems.append("Policy F binding differs")
    checks["untracked_runtime_files"] = base.git("ls-files", "--others", "--exclude-standard", "--",
                                                "aerial_gym", "motar_eval", "tools").stdout.splitlines()
    if not smoke and checks["untracked_runtime_files"]:
        problems.append("untracked runtime source would be outside the approved commit")
    try:
        manifest = SL1D_C01_ROOT / "RAW_MANIFEST.json"
        checks["sl1d_c01_manifest_sha256"] = base.sha256_file(manifest)
        if checks["sl1d_c01_manifest_sha256"] != base.sha256_file(SL1D_C01_MANIFEST_COPY):
            problems.append("SL1D-C01 manifest differs from its committed copy")
    except OSError as error:
        problems.append("SL1D-C01 binding: " + str(error))
    checks["effective_processes"] = process_specs(p)
    checks["effective_run_limits"] = p["run"]
    return problems, checks


def run(p, run_dir, specs, smoke, checks):
    run_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    deadline = started + p["run"]["total_wall_cap_s"]
    source_head = base.git("rev-parse", "HEAD").stdout.strip()
    baseline = base.git("status", "--porcelain", "--untracked-files=no").stdout
    report = {"schema": "motar.s-l1d-c02-run-report.v1", "run_dir": str(run_dir), "smoke": smoke, "preflight": checks,
              "effective_processes": specs, "processes": [], "source_commit": source_head, "status": "RUNNING",
              "stop_reason": None, "plan_sha256": base.sha256_file(dr.PLAN_PATH),
              "amendment_sha256": base.sha256_file(AMENDMENT_PATH)}
    try:
        for spec in specs:
            reason, _ = a.runtime_guard(p, run_dir, source_head, baseline, deadline)
            if reason:
                report["stop_reason"] = reason
                break
            gpu = base.gpu_state()
            cap = gpu["total_mib"] - gpu["used_mib"] - p["run"]["oom_reserve_mib"]
            if cap <= 0 or "3070" not in gpu["name"]:
                raise ValueError("invalid launch-time GPU platform or memory headroom")
            entry = {"id": spec["id"], "gpu_at_launch": gpu, "gpu_memory_cap_mib": cap, "free_disk_gb_before": base.free_gb(run_dir)}
            report["processes"].append(entry)
            out = run_dir / spec["id"]
            command = [base.PYTHON, "-B", str(LAUNCHER), "--process", spec["id"], "--out", str(out)]
            entry["command"] = command
            process_deadline = time.monotonic() + spec["cap_s"]
            guard = (lambda proc, process_deadline=process_deadline, cap=cap: a.runtime_guard(
                p, run_dir, source_head, baseline, deadline, process_deadline=process_deadline, proc=proc, cap_mib=cap))
            rc, wall, peak, reason = a.supervised_launch(command, b.process_env(p, spec, out),
                                                         run_dir / (spec["id"] + ".log"), guard, p["run"]["poll_s"])
            entry.update(returncode=rc, wall_s=wall, peak_gpu_mib=peak)
            if reason or rc != 0:
                report["stop_reason"] = reason or "NONZERO_EXIT: %s: %s" % (spec["id"], rc)
                break
            entry["process_done"] = a.validate_done(out, spec, checks)
            reason, _ = a.runtime_guard(p, run_dir, source_head, baseline, deadline)
            if reason:
                report["stop_reason"] = reason
                break
            print("[sl1d-c02] %s rc=%d wall=%.1fs peak=%dMiB" % (spec["id"], rc, wall, peak), flush=True)
    except Exception as error:
        report["stop_reason"] = "INTEGRITY_EXCEPTION: %s: %s" % (type(error).__name__, error)
    report["status"] = "STOPPED_INTEGRITY" if report["stop_reason"] else "COMPLETED"
    report["wall_s_total"] = time.monotonic() - started
    report["gpu_seconds_total"] = sum(item.get("wall_s", 0) for item in report["processes"])
    report["output_bytes_total"] = base.dir_bytes(run_dir)
    report["free_disk_gb_after"] = base.free_gb(run_dir)
    old = a.read_json(base.resolve(p["references"]["eq1d_plan"]))
    report["policy_f_sha256_after"] = base.sha256_file(base.RL_DIR / old["frozen"]["policy_f_rel_to_rl_games"])
    if report["policy_f_sha256_after"] != p["policy_f_sha256"]:
        report["status"], report["stop_reason"] = "STOPPED_INTEGRITY", "POLICY_F_CHANGED"
    with (run_dir / "run_report.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    if (run_dir / "RAW_MANIFEST.json").exists():
        raise FileExistsError("refusing to overwrite run manifest")
    report["manifest_members"] = base.write_manifest(run_dir)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("stage", choices=("preflight", "run"))
    parser.add_argument("--approved-commit", default="")
    parser.add_argument("--smoke-root", type=Path)
    args = parser.parse_args(argv)
    p = load_plan()
    smoke = args.smoke_root is not None
    run_dir = (args.smoke_root.resolve() / ("sl1d_c02_smoke_" + time.strftime("%Y%m%d_%H%M%S"))) if smoke else (
        base.resolve(p["run"]["output_root"]) / p["run"]["run_id"])
    specs = process_specs(p)
    problems, checks = preflight(p, args.approved_commit, run_dir, smoke)
    if args.stage == "preflight" or problems:
        print(json.dumps({"problems": problems, "checks": checks}, indent=2, allow_nan=False))
        return 2 if problems else 0
    report = run(p, run_dir, specs, smoke, checks)
    print(json.dumps({k: report[k] for k in ("status", "stop_reason", "run_dir", "gpu_seconds_total")}, indent=2))
    return 0 if report["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    sys.exit(main())
