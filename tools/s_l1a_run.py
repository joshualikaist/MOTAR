#!/usr/bin/env python3
"""S-L1A fixed-matrix canary: frozen preflight, fail-closed supervision and digest gates.

The CLI may launch GPU processes only for the explicitly approved committed plan.
Importing this module launches nothing. Smoke subsets are mechanics only, never evidence.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
import eq1d_run as base  # noqa: E402
import e_l0_run as el0  # noqa: E402

PLAN_PATH = ROOT / "docs/plans/s_l1a_v1.json"
AMENDMENT_PATH = ROOT / "docs/plans/s_l1a_v1_amendment1.json"
LAUNCHER = ROOT / "tools/s_l1a_launcher.py"
HISTORICAL_ROOT = base.resolve("../aerial_gym_simulator/results/e_l0_evaluation_v1/EL0-R1")
HISTORICAL_MANIFEST = ROOT / "docs/evidence/e_l0_evaluation_2026-10-18/run_records/RAW_MANIFEST.json"
DIGEST_KEYS = el0.DIGEST_KEYS


def read_json(path):
    def finite(value):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("nonfinite JSON number")
        return number

    def invalid(value):
        raise ValueError("nonfinite JSON constant: " + value)

    return json.loads(Path(path).read_text(), parse_float=finite, parse_constant=invalid)


def load_plan():
    p = read_json(PLAN_PATH)
    amendment = read_json(AMENDMENT_PATH)
    overrides = amendment["overrides"]
    if set(overrides) != {"run"} or set(overrides["run"]) != {"closed_loop_seed_by_config"}:
        raise ValueError("amendment 1 must only override the historical-comparison seeds")
    seeds = overrides["run"]["closed_loop_seed_by_config"]
    if set(seeds) != {"static", "moving"} or any(type(v) is not int for v in seeds.values()):
        raise ValueError("invalid amended configuration seeds")
    p["run"].update(copy.deepcopy(overrides["run"]))
    return p


def process_specs(p):
    run = p["run"]
    specs = [{"id": "sensors_" + mode, "batch": "SENSORS", "mode": mode,
              "config": "static", "arm": "P2", "num_envs": run["sensor_num_envs"],
              "seed": run["closed_loop_seed"], "cap_s": run["process_wall_cap_s"],
              "lineage": None if mode == "legacy" else p["switch"]["on"]}
             for mode in ("legacy", "depth", "full")]
    for config in ("static", "moving"):
        for variant in ("unset", "off", "on_p1", "on_p2"):
            specs.append({"id": config + "_" + variant, "batch": "V3", "config": config,
                          "variant": variant, "arm": "P1" if variant == "on_p1" else "P2",
                          "num_envs": run["closed_loop_num_envs"], "steps": run["closed_loop_steps"],
                          "seed": run["closed_loop_seed_by_config"][config],
                          "cap_s": run["process_wall_cap_s"],
                          "lineage": None if variant == "unset" else
                          (p["switch"]["off"] if variant == "off" else p["switch"]["on"])})
    return specs


def process_env(p, spec, out):
    old = read_json(base.resolve(p["references"]["e_l0_plan"]))
    env = el0.process_env(old, spec, out)
    env.pop(p["switch"]["name"], None)
    if spec["lineage"] is not None:
        env[p["switch"]["name"]] = spec["lineage"]
    env.update(NAVRL_KROC2027_RUN_ID=p["run"]["run_id"],
               NAVRL_NUM_BARS=str(p["run"]["closed_loop_density"]),
               NAVRL_DENSITY_START=str(p["run"]["closed_loop_density"]),
               NAVRL_DENSITY_FINAL=str(p["run"]["closed_loop_density"]))
    return env


def committed(path):
    relative = str(path.relative_to(ROOT))
    return (base.git("ls-files", "--error-unmatch", relative).returncode == 0 and
            base.git("diff", "--quiet", "HEAD", "--", relative).returncode == 0)


def historical_bindings(p):
    manifest = HISTORICAL_ROOT / "RAW_MANIFEST.json"
    if base.sha256_file(manifest) != base.sha256_file(HISTORICAL_MANIFEST):
        raise ValueError("historical E-L0 manifest differs from its committed evidence copy")
    members = {m["path"]: m for m in read_json(manifest)["members"]}
    result = {"manifest_sha256": base.sha256_file(manifest), "configs": {}}
    for config in ("static", "moving"):
        directory = HISTORICAL_ROOT / ("v3_" + config + "_p2")
        item = {}
        for name in ("v3_steps.jsonl", "v3_meta.json"):
            path = directory / name
            record = members[str(path.relative_to(HISTORICAL_ROOT))]
            digest = base.sha256_file(path)
            if path.stat().st_size != record["bytes"] or digest != record["sha256"]:
                raise ValueError("historical E-L0 file changed: " + str(path))
            item[name] = digest
        meta = read_json(directory / "v3_meta.json")
        expected = {"config": config, "seed": p["run"]["closed_loop_seed_by_config"][config],
                    "num_envs": p["run"]["closed_loop_num_envs"], "steps": p["run"]["closed_loop_steps"]}
        if any(meta["process"].get(k) != v for k, v in expected.items()):
            raise ValueError("historical comparison configuration mismatch: " + config)
        item["process"] = meta["process"]
        result["configs"][config] = item
    return result


def preflight(p, approved, run_dir, smoke=False):
    old = read_json(base.resolve(p["references"]["eq1d_plan"]))
    problems, checks = base.preflight(old, approved, run_dir, smoke)
    paths = (PLAN_PATH, AMENDMENT_PATH, PLAN_PATH.with_suffix(".md"), AMENDMENT_PATH.with_suffix(".md"),
             Path(__file__).resolve(), LAUNCHER)
    checks["s_l1a_committed_inputs"] = {str(path.relative_to(ROOT)): committed(path) for path in paths}
    checks["s_l1a_input_sha256"] = {str(path.relative_to(ROOT)): base.sha256_file(path)
                                      for path in paths if path.is_file()}
    if not smoke and not all(checks["s_l1a_committed_inputs"].values()):
        problems.append("S-L1A plan/amendment/code not committed unchanged")
    if checks.get("policy_f_sha256") != p["policy_f_sha256"]:
        problems.append("S-L1A Policy F binding differs")
    checks["untracked_runtime_files"] = base.git("ls-files", "--others", "--exclude-standard", "--",
                                                "aerial_gym", "motar_eval", "tools").stdout.splitlines()
    if not smoke and checks["untracked_runtime_files"]:
        problems.append("untracked runtime source would be outside the approved commit")
    old_el0 = read_json(base.resolve(p["references"]["e_l0_plan"]))
    checks["configuration_receipts"] = {}
    for config in ("static", "moving"):
        receipt = el0.config_receipt(old_el0, {"config": config})
        checks["configuration_receipts"][config] = {"path": str(receipt),
                                                   "sha256": base.sha256_file(receipt) if receipt.is_file() else None}
        if not receipt.is_file() or read_json(receipt).get("checkpoint_sha256") != p["policy_f_sha256"]:
            problems.append("configuration receipt missing or checkpoint mismatch: " + config)
    try:
        checks["historical_bindings"] = historical_bindings(p)
    except (OSError, ValueError, KeyError) as error:
        problems.append("historical OFF comparison preflight: " + str(error))
    checks["effective_processes"] = process_specs(p)
    checks["effective_run_limits"] = p["run"]
    return problems, checks


def load_digests(path, steps):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    if len(rows) != steps:
        raise ValueError("wrong number of digest steps: " + str(path))
    for i, row in enumerate(rows):
        if row.get("step") != i or any(k not in row for k in DIGEST_KEYS):
            raise ValueError("incomplete or nonconsecutive digest row")
        for key in DIGEST_KEYS:
            if key != "resets" and not re.fullmatch(r"[0-9a-f]{64}", str(row[key])):
                raise ValueError("invalid digest: " + key)
        if not isinstance(row["resets"], list):
            raise ValueError("invalid reset list")
    return rows


def compare_digest_files(left, right, steps):
    a, b = load_digests(left, steps), load_digests(right, steps)
    differences = [{"step": i, "key": key} for i, (ra, rb) in enumerate(zip(a, b))
                   for key in DIGEST_KEYS if ra[key] != rb[key]]
    return {"identical": not differences, "steps": steps, "keys": list(DIGEST_KEYS),
            "n_differences": len(differences), "differences": differences[:50],
            "left_sha256": base.sha256_file(left), "right_sha256": base.sha256_file(right)}


def validate_done(out, spec, checks):
    done = read_json(out / "process_done.json")
    if done.get("process") != spec["id"] or done.get("nan_fields") != []:
        raise ValueError("missing completion identity or nonfinite-state record")
    if not isinstance(done.get("files"), list) or not done["files"]:
        raise ValueError("completion has no bound outputs")
    seen = set()
    for item in done["files"]:
        path = out / item["file"]
        if path.resolve().parent != out.resolve() or path.is_symlink() or item["file"] in seen:
            raise ValueError("unsafe or duplicate completion output")
        seen.add(item["file"])
        if path.stat().st_size != item["bytes"] or base.sha256_file(path) != item["sha256"]:
            raise ValueError("missing or changed completion output: " + item["file"])
    if spec["batch"] == "V3":
        if done.get("actor_obs_dim") != 898:
            raise ValueError("actor observation width not recorded as 898")
        meta = read_json(out / "v3_meta.json")
        for key in ("config", "seed", "num_envs", "steps"):
            if meta["process"].get(key) != spec[key]:
                raise ValueError("completed V3 configuration mismatch: " + key)
        if (meta.get("rms_digest_start") != checks["rms_digest"] or
                meta.get("rms_digest_end") != checks["rms_digest"] or meta.get("model_training_flag") is not False):
            raise ValueError("RMS changed or actor not in evaluation mode")
        origins = meta.get("origins", {})
        if any(not origins.get(k) or ROOT.resolve() not in Path(origins[k]).resolve().parents
               for k in ("aerial_gym", "motar_eval")):
            raise ValueError("completed process import origin outside approved worktree")
        load_digests(out / "v3_steps.jsonl", spec["steps"])
    return done


def runtime_guard(p, run_dir, source_head, baseline_status, total_deadline, *,
                  process_deadline=None, proc=None, cap_mib=None):
    try:
        now = time.monotonic()
        if now >= total_deadline:
            return "TOTAL_WALL_CLOCK_CAP", 0
        if process_deadline is not None and now >= process_deadline:
            return "PROCESS_WATCHDOG", 0
        if base.free_gb(run_dir) < p["run"]["disk_min_free_gb"]:
            return "FREE_DISK_BELOW_8_GB", 0
        if base.dir_bytes(run_dir) > p["run"]["output_max_gb"] * 1e9:
            return "OUTPUT_SIZE_CAP", 0
        head = base.git("rev-parse", "HEAD")
        status = base.git("status", "--porcelain", "--untracked-files=no")
        if head.returncode or status.returncode:
            return "SOURCE_GIT_CHECK_FAILED", 0
        if head.stdout.strip() != source_head or status.stdout != baseline_status:
            return "TRACKED_TREE_OR_HEAD_CHANGED", 0
        used = 0
        if proc is not None:
            pids = base.proc_tree_pids(proc.pid)
            used = sum(v["used_mib"] for v in base.gpu_state()["compute_apps"] if v["pid"] in pids)
            if cap_mib is None or used > cap_mib:
                return "GPU_MEMORY_CAP", used
        return None, used
    except Exception as error:  # Failure to monitor is not permission to continue.
        return "GUARD_CHECK_FAILED: %s: %s" % (type(error).__name__, error), 0


def terminate_group(proc):
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    # The group leader may exit while a GPU grandchild ignores TERM.
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=10)


def supervised_launch(command, env, log_path, stop_check, poll_s):
    started, peak, reason = time.monotonic(), 0, None
    with Path(log_path).open("x") as log:
        proc = subprocess.Popen(command, cwd=str(ROOT), env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while True:
                reason, used = stop_check(proc)
                peak = max(peak, used)
                if reason:
                    terminate_group(proc)
                    break
                if proc.poll() is not None:
                    break
                time.sleep(poll_s)
        except BaseException:
            terminate_group(proc)
            raise
        finally:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return proc.returncode, time.monotonic() - started, peak, reason


def noninterference(run_dir, spec):
    config, variant, steps = spec["config"], spec["variant"], spec["steps"]
    out = run_dir / spec["id"] / "v3_steps.jsonl"
    pairs = {}
    if variant in ("unset", "off"):
        pairs[spec["id"] + "_vs_historical"] = compare_digest_files(
            out, HISTORICAL_ROOT / ("v3_" + config + "_p2") / "v3_steps.jsonl", steps)
    if variant == "off":
        pairs[config + "_off_vs_unset"] = compare_digest_files(
            out, run_dir / (config + "_unset") / "v3_steps.jsonl", steps)
    if variant == "on_p2":
        pairs[config + "_on_p2_vs_on_p1"] = compare_digest_files(
            out, run_dir / (config + "_on_p1") / "v3_steps.jsonl", steps)
    return pairs


def run(p, run_dir, specs, smoke, checks):
    run_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    deadline = started + p["run"]["total_wall_cap_s"]
    source_head = base.git("rev-parse", "HEAD").stdout.strip()
    baseline = base.git("status", "--porcelain", "--untracked-files=no").stdout
    report = {"schema": "motar.s-l1a-run-report.v1", "run_dir": str(run_dir), "smoke": smoke,
              "preflight": checks, "effective_processes": specs, "processes": [], "noninterference": {},
              "source_commit": source_head, "status": "RUNNING", "stop_reason": None,
              "plan_sha256": base.sha256_file(PLAN_PATH), "amendment_sha256": base.sha256_file(AMENDMENT_PATH)}
    try:
        for spec in specs:
            reason, _ = runtime_guard(p, run_dir, source_head, baseline, deadline)
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
            guard = lambda proc: runtime_guard(p, run_dir, source_head, baseline, deadline,
                                               process_deadline=process_deadline, proc=proc, cap_mib=cap)
            rc, wall, peak, reason = supervised_launch(command, process_env(p, spec, out),
                                                       run_dir / (spec["id"] + ".log"), guard, p["run"]["poll_s"])
            entry.update(returncode=rc, wall_s=wall, peak_gpu_mib=peak)
            if reason or rc != 0:
                report["stop_reason"] = reason or "NONZERO_EXIT: %s: %s" % (spec["id"], rc)
                break
            entry["process_done"] = validate_done(out, spec, checks)
            if spec["batch"] == "V3":
                # Smoke subsets may omit prerequisites; they never establish official gates.
                prerequisites = (spec["variant"] != "off" or (run_dir / (spec["config"] + "_unset")).exists()) and (
                    spec["variant"] != "on_p2" or (run_dir / (spec["config"] + "_on_p1")).exists())
                pairs = noninterference(run_dir, spec) if not smoke or prerequisites else {}
                report["noninterference"].update(pairs)
                if any(not value["identical"] for value in pairs.values()):
                    report["stop_reason"] = "NONINTERFERENCE_FAIL: " + spec["id"]
                    break
            reason, _ = runtime_guard(p, run_dir, source_head, baseline, deadline)
            if reason:
                report["stop_reason"] = reason
                break
            print("[s-l1a] %s rc=%d wall=%.1fs peak=%dMiB" % (spec["id"], rc, wall, peak), flush=True)
    except Exception as error:
        report["stop_reason"] = "INTEGRITY_EXCEPTION: %s: %s" % (type(error).__name__, error)
    report["status"] = ("STOPPED_NONINTERFERENCE" if str(report["stop_reason"]).startswith("NONINTERFERENCE") else
                        "STOPPED_INTEGRITY" if report["stop_reason"] else "COMPLETED")
    report["wall_s_total"] = time.monotonic() - started
    report["gpu_seconds_total"] = sum(item.get("wall_s", 0) for item in report["processes"])
    report["output_bytes_total"] = base.dir_bytes(run_dir)
    report["free_disk_gb_after"] = base.free_gb(run_dir)
    old = read_json(base.resolve(p["references"]["eq1d_plan"]))
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
    run_dir = (args.smoke_root.resolve() / ("sl1a_smoke_" + time.strftime("%Y%m%d_%H%M%S"))) if smoke else (
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
