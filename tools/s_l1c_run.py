#!/usr/bin/env python3
"""S-L1C fixed-matrix canary: frozen preflight, fail-closed supervision and digest gates.

Supervision is reused read-only from the qualified S-L1A/S-L1B runners. Importing this module
launches nothing. Smoke subsets are mechanics only, never evidence.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
import s_l1b_run as b  # noqa: E402

a, base, el0 = b.a, b.base, b.el0
PLAN_PATH = ROOT / "docs/plans/s_l1c_v1.json"
LAUNCHER = ROOT / "tools/s_l1c_launcher.py"
S_L1B_ROOT = base.resolve("../aerial_gym_simulator/results/s_l1b_v1/SL1B-C01")
S_L1B_MANIFEST_COPY = ROOT / "docs/evidence/s_l1b_2026-10-18/run_records/RAW_MANIFEST.json"
LINEAGE = {"off": "off", "s_l1a": "s_l1a_v1", "s_l1b": "s_l1b_v1", "s_l1c_p1": "s_l1c_v1", "s_l1c_p2": "s_l1c_v1"}
SENSOR_LINEAGE = {"legacy": None, "s_l1b": "s_l1b_v1", "s_l1c": "s_l1c_v1"}
LEVEL = {"off": 0, "s_l1a": 1, "s_l1b": 2, "s_l1c_p1": 3, "s_l1c_p2": 3}


def load_plan():
    return a.read_json(PLAN_PATH)


def process_specs(p):
    run = p["run"]
    specs = [{"id": "sensors_" + mode, "batch": "SENSORS", "mode": mode, "config": "static", "arm": "P2",
              "num_envs": run["sensor_num_envs"], "seed": run["sensor_seed"], "cap_s": run["process_wall_cap_s"],
              "lineage": SENSOR_LINEAGE[mode]} for mode in run["sensor_modes"]]
    for config in ("static", "moving"):
        for variant in run["closed_loop_variants"]:
            specs.append({"id": config + "_" + variant, "batch": "V3", "config": config, "variant": variant,
                          "arm": "P1" if variant == "s_l1c_p1" else "P2", "num_envs": run["closed_loop_num_envs"],
                          "steps": run["closed_loop_steps"], "seed": run["closed_loop_seed_by_config"][config],
                          "cap_s": run["process_wall_cap_s"], "lineage": LINEAGE[variant]})
    return specs


process_env = b.process_env


def bound_run(root, copy, expected_sha, names, p):
    manifest = root / "RAW_MANIFEST.json"
    digest = base.sha256_file(manifest)
    if digest != base.sha256_file(copy) or digest != expected_sha:
        raise ValueError("prior run manifest differs from its committed copy or plan binding: " + str(root))
    members = {m["path"]: m for m in a.read_json(manifest)["members"]}
    result = {"manifest_sha256": digest, "configs": {}}
    for config in ("static", "moving"):
        path = root / (config + "_" + names) / "v3_steps.jsonl"
        record = members[str(path.relative_to(root))]
        if path.stat().st_size != record["bytes"] or base.sha256_file(path) != record["sha256"]:
            raise ValueError("prior step log changed: " + str(path))
        done = a.read_json(root / (config + "_" + names) / "process_done.json")
        if done["process_spec"]["seed"] != p["run"]["closed_loop_seed_by_config"][config]:
            raise ValueError("prior run seed mismatch: " + config)
        result["configs"][config] = record["sha256"]
    return result


def preflight(p, approved, run_dir, smoke=False):
    old = a.read_json(base.resolve(p["references"]["eq1d_plan"]))
    problems, checks = base.preflight(old, approved, run_dir, smoke)
    paths = (PLAN_PATH, PLAN_PATH.with_suffix(".md"), Path(__file__).resolve(), LAUNCHER,
             ROOT / "tools/s_l1b_run.py", ROOT / "tools/s_l1a_run.py")
    checks["s_l1c_committed_inputs"] = {str(x.relative_to(ROOT)): a.committed(x) for x in paths}
    checks["s_l1c_input_sha256"] = {str(x.relative_to(ROOT)): base.sha256_file(x) for x in paths if x.is_file()}
    if not smoke and not all(checks["s_l1c_committed_inputs"].values()):
        problems.append("S-L1C plan/code not committed unchanged")
    if checks.get("policy_f_sha256") != p["policy_f_sha256"]:
        problems.append("S-L1C Policy F binding differs")
    checks["untracked_runtime_files"] = base.git("ls-files", "--others", "--exclude-standard", "--",
                                                "aerial_gym", "motar_eval", "tools").stdout.splitlines()
    if not smoke and checks["untracked_runtime_files"]:
        problems.append("untracked runtime source would be outside the approved commit")
    old_el0 = a.read_json(base.resolve(p["references"]["e_l0_plan"]))
    checks["configuration_receipts"] = {}
    for config in ("static", "moving"):
        receipt = el0.config_receipt(old_el0, {"config": config})
        checks["configuration_receipts"][config] = {"path": str(receipt), "sha256": base.sha256_file(receipt) if receipt.is_file() else None}
        if not receipt.is_file() or a.read_json(receipt).get("checkpoint_sha256") != p["policy_f_sha256"]:
            problems.append("configuration receipt missing or checkpoint mismatch: " + config)
    sl1a_plan = a.read_json(ROOT / "docs/plans/s_l1b_v1.json")
    for name, fn in (("historical_bindings", lambda: a.historical_bindings(p)),
                     ("s_l1a_bindings", lambda: bound_run(b.S_L1A_ROOT, b.S_L1A_MANIFEST_COPY,
                                                          sl1a_plan["references"]["s_l1a_run_manifest_sha256"], "on_p2", p)),
                     ("s_l1b_bindings", lambda: bound_run(S_L1B_ROOT, S_L1B_MANIFEST_COPY,
                                                          p["references"]["s_l1b_run_manifest_sha256"], "s_l1b_p2", p))):
        try:
            checks[name] = fn()
        except (OSError, ValueError, KeyError) as error:
            problems.append(name + " preflight: " + str(error))
    checks["effective_processes"] = process_specs(p)
    checks["effective_run_limits"] = p["run"]
    return problems, checks


def noninterference(run_dir, spec):
    config, variant, steps = spec["config"], spec["variant"], spec["steps"]
    out = run_dir / spec["id"] / "v3_steps.jsonl"
    if variant == "off":
        return {spec["id"] + "_vs_historical_el0_r1": a.compare_digest_files(
            out, a.HISTORICAL_ROOT / ("v3_" + config + "_p2") / "v3_steps.jsonl", steps)}
    if variant == "s_l1a":
        return {spec["id"] + "_vs_sl1a_c01_on_p2": a.compare_digest_files(
            out, b.S_L1A_ROOT / (config + "_on_p2") / "v3_steps.jsonl", steps)}
    if variant == "s_l1b":
        return {spec["id"] + "_vs_sl1b_c01_s_l1b_p2": a.compare_digest_files(
            out, S_L1B_ROOT / (config + "_s_l1b_p2") / "v3_steps.jsonl", steps)}
    if variant == "s_l1c_p2" and (run_dir / (config + "_s_l1c_p1")).exists():
        return {spec["id"] + "_vs_s_l1c_p1": a.compare_digest_files(
            out, run_dir / (config + "_s_l1c_p1") / "v3_steps.jsonl", steps)}
    return {}


def validate_done(out, spec, checks):
    done = a.validate_done(out, spec, checks)
    if spec["batch"] == "V3":
        level = LEVEL[spec["variant"]]
        flags = (done.get("perception_s_l1a"), done.get("perception_s_l1b"), done.get("perception_s_l1c"),
                 done.get("detector_lineage_enabled"))
        if flags != (level >= 1, level >= 2, level >= 3, level >= 1):
            raise ValueError("lineage flags differ from the process specification")
    return done


def run(p, run_dir, specs, smoke, checks):
    run_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    deadline = started + p["run"]["total_wall_cap_s"]
    source_head = base.git("rev-parse", "HEAD").stdout.strip()
    baseline = base.git("status", "--porcelain", "--untracked-files=no").stdout
    report = {"schema": "motar.s-l1c-run-report.v1", "run_dir": str(run_dir), "smoke": smoke,
              "preflight": checks, "effective_processes": specs, "processes": [], "noninterference": {},
              "source_commit": source_head, "status": "RUNNING", "stop_reason": None,
              "plan_sha256": base.sha256_file(PLAN_PATH)}
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
            entry = {"id": spec["id"], "gpu_at_launch": gpu, "gpu_memory_cap_mib": cap,
                     "free_disk_gb_before": base.free_gb(run_dir)}
            report["processes"].append(entry)
            out = run_dir / spec["id"]
            command = [base.PYTHON, "-B", str(LAUNCHER), "--process", spec["id"], "--out", str(out)]
            entry["command"] = command
            process_deadline = time.monotonic() + spec["cap_s"]
            guard = (lambda proc, process_deadline=process_deadline, cap=cap: a.runtime_guard(
                p, run_dir, source_head, baseline, deadline, process_deadline=process_deadline, proc=proc, cap_mib=cap))
            rc, wall, peak, reason = a.supervised_launch(command, process_env(p, spec, out),
                                                         run_dir / (spec["id"] + ".log"), guard, p["run"]["poll_s"])
            entry.update(returncode=rc, wall_s=wall, peak_gpu_mib=peak)
            if reason or rc != 0:
                report["stop_reason"] = reason or "NONZERO_EXIT: %s: %s" % (spec["id"], rc)
                break
            entry["process_done"] = validate_done(out, spec, checks)
            if spec["batch"] == "V3":
                pairs = noninterference(run_dir, spec)
                report["noninterference"].update(pairs)
                if any(not value["identical"] for value in pairs.values()):
                    report["stop_reason"] = "NONINTERFERENCE_FAIL: " + spec["id"]
                    break
            reason, _ = a.runtime_guard(p, run_dir, source_head, baseline, deadline)
            if reason:
                report["stop_reason"] = reason
                break
            print("[s-l1c] %s rc=%d wall=%.1fs peak=%dMiB" % (spec["id"], rc, wall, peak), flush=True)
    except Exception as error:
        report["stop_reason"] = "INTEGRITY_EXCEPTION: %s: %s" % (type(error).__name__, error)
    report["status"] = ("STOPPED_NONINTERFERENCE" if str(report["stop_reason"]).startswith("NONINTERFERENCE") else
                        "STOPPED_INTEGRITY" if report["stop_reason"] else "COMPLETED")
    report["wall_s_total"] = time.monotonic() - started
    report["gpu_seconds_total"] = sum(item.get("wall_s", 0) for item in report["processes"])
    report["output_bytes_total"] = base.dir_bytes(run_dir)
    report["free_disk_gb_after"] = base.free_gb(run_dir)
    old = a.read_json(base.resolve(p["references"]["eq1d_plan"]))
    checkpoint = base.RL_DIR / old["frozen"]["policy_f_rel_to_rl_games"]
    report["policy_f_sha256_after"] = base.sha256_file(checkpoint)
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
    parser.add_argument("--processes", help="comma-separated mechanics-only smoke subset")
    args = parser.parse_args(argv)
    p = load_plan()
    smoke = args.smoke_root is not None
    run_dir = (args.smoke_root.resolve() / ("sl1c_smoke_" + time.strftime("%Y%m%d_%H%M%S"))) if smoke else (
        base.resolve(p["run"]["output_root"]) / p["run"]["run_id"])
    specs = process_specs(p)
    if args.processes:
        wanted = args.processes.split(",")
        if not smoke or not set(wanted) <= {x["id"] for x in specs} or len(wanted) != len(set(wanted)):
            parser.error("valid unique --processes is allowed only for smoke")
        specs = [x for x in specs if x["id"] in wanted]
    problems, checks = preflight(p, args.approved_commit, run_dir, smoke)
    if args.stage == "preflight" or problems:
        print(json.dumps({"problems": problems, "checks": checks}, indent=2, allow_nan=False))
        return 2 if problems else 0
    report = run(p, run_dir, specs, smoke, checks)
    print(json.dumps({k: report[k] for k in ("status", "stop_reason", "run_dir", "gpu_seconds_total")}, indent=2))
    return 0 if report["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    sys.exit(main())
