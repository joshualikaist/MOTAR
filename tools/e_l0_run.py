#!/usr/bin/env python3
"""E-L0 orchestrator: preflight, watchdogged process sequence, in-run non-interference stop, manifest.

    python -B tools/e_l0_run.py preflight --approved-commit SHA
    python -B tools/e_l0_run.py run --approved-commit SHA
    python -B tools/e_l0_run.py run --smoke-root SCRATCH [--processes v3_static_p0,v3_static_p2]

Same safety contract as ``tools/eq1d_run.py`` (whose helpers it reuses): approved commit = HEAD on a
clean tree with the plan committed, Policy F and RMS verified, imports from this tree, fresh run
directory, >= 8 GB free disk, per-process wall caps, a GPU memory cap measured at launch, an output
limit, no retry. In addition, after the P0/P1/P2 arms of a configuration finish, their per-step
digests are compared and any difference stops the run before the next process.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import eq1d_run as base  # noqa: E402  (helpers only)

PLAN_PATH = ROOT / "docs/plans/e_l0_evaluation_lineage_v1.json"
LAUNCHER = ROOT / "tools/e_l0_launcher.py"
EQ1D_PLAN = json.loads(base.PLAN_PATH.read_text())
DIGEST_KEYS = ("obs", "action", "reward", "done", "captured", "crashed", "pursuer", "target", "resets",
               "rgb", "depth", "lidar", "semantic", "tracker")
SCRUB = ("NAVRL_KROC2027_RECORDER", "NAVRL_KROC2027_RUN_ID", "NAVRL_KROC2027_ARM", "NAVRL_KROC2027_PROTOCOL_SHA256",
         "NAVRL_KROC2027_OCCLUSION_TRACE", "NAVRL_KROC2027_RECOVERY_EVENTS", "NAVRL_KROC2027_RECOVERY_TRACE",
         "NAVRL_KROC2027_PACKETS", "NAVRL_KROC2027_PACKET_SCHEMA", "NAVRL_KROC2027_ON_VIOLATION", "NAVRL_KROC2027_E_L0",
         "NAVRL_BULK_EVAL_JSON", "NAVRL_EPISODE_LEDGER", "NAVRL_EPISODE_QUOTA_PER_ENV", "NAVRL_EVAL_RUN_NONCE",
         "NAVRL_SEED", "NAVRL_EVAL_CHECKPOINT", "NAVRL_NUM_BARS", "NAVRL_DENSITY_START", "NAVRL_DENSITY_FINAL")


def plan():
    return json.loads(PLAN_PATH.read_text())


def config_receipt(p, spec):
    key = spec.get("config", "static")
    rel = p["run"]["configs"][key].split(" with ")[0]
    return base.resolve(rel)


def process_env(p, spec, out):
    env = {k: v for k, v in os.environ.items() if not k.startswith(base.SCRUB_PREFIXES) and k not in base.SCRUB_KEYS}
    snap = json.loads(config_receipt(p, spec).read_text())["env_snapshot_navrl"]
    env.update({k: str(v) for k, v in snap.items() if k not in SCRUB})
    env["PATH"] = str(Path(base.PYTHON).resolve().parent) + os.pathsep + env.get("PATH", "")
    env.update(PYTHONPATH=str(ROOT), PYTHONNOUSERSITE="1", NAVRL_SEED=str(spec.get("seed", 4401)),
               NAVRL_NUM_BARS="160", NAVRL_DENSITY_START="160", NAVRL_DENSITY_FINAL="160",
               TASK=EQ1D_PLAN["frozen"]["task"], FILE=EQ1D_PLAN["frozen"]["player_yaml"],
               AERIAL_GYM_SIM_NAME=EQ1D_PLAN["frozen"]["sim_name"],
               NAVRL_BULK_EVAL="1", NAVRL_BULK_EVAL_JSON=str(Path(out) / "result.json"))
    arm = spec.get("arm")
    if arm in ("P1", "P2"):
        env.update(NAVRL_KROC2027_RECORDER="1", NAVRL_KROC2027_OCCLUSION_TRACE="1",
                   NAVRL_KROC2027_RUN_ID=p["run"]["run_id"], NAVRL_KROC2027_ARM=spec["id"])
    if arm == "P2":
        env["NAVRL_KROC2027_E_L0"] = "1"
    return env


def digests(path):
    with open(path) as fh:
        return [json.loads(line) for line in fh]


def compare_arms(run_dir, ids):
    """Per-step digest equality across the arms of one configuration (P0 is the reference)."""
    logs = {i: digests(Path(run_dir) / i / "v3_steps.jsonl") for i in ids}
    ref_id = ids[0]
    report = {"reference": ref_id, "pairs": {}}
    ok = True
    for other in ids[1:]:
        a, b = logs[ref_id], logs[other]
        diffs = []
        if len(a) != len(b):
            diffs.append({"step": None, "key": "length", "a": len(a), "b": len(b)})
        for ra, rb in zip(a, b):
            for key in DIGEST_KEYS:
                if ra.get(key) != rb.get(key):
                    diffs.append({"step": ra["step"], "key": key})
        report["pairs"]["%s_vs_%s" % (other, ref_id)] = {"steps": min(len(a), len(b)), "differences": diffs[:50],
                                                         "n_differences": len(diffs)}
        ok &= not diffs
    report["identical"] = ok
    return report


def run(p, run_dir, processes, smoke, checks):
    run_dir.mkdir(parents=True, exist_ok=False)
    gpu0 = checks.get("gpu") or {}
    cap_mib = (gpu0.get("total_mib", 0) - gpu0.get("used_mib", 0) - base.OOM_RESERVE_MIB) if gpu0 else None
    report = {"schema": "motar.e-l0-run-report.v1", "run_dir": str(run_dir), "smoke": smoke,
              "plan_sha256": base.sha256_file(PLAN_PATH), "preflight": checks, "gpu_memory_cap_mib": cap_mib,
              "processes": [], "noninterference": {}, "status": "RUNNING",
              "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    baseline_dirty = base.git("status", "--porcelain", "--untracked-files=no").stdout
    head0 = base.git("rev-parse", "HEAD").stdout.strip()
    stop = None
    groups = {}
    for spec in p["run"]["processes"]:
        if spec["batch"] == "V3":
            groups.setdefault("%s_n%d" % (spec["config"], spec["num_envs"]), []).append(spec["id"])
    done_ids = set()
    for spec in processes:
        entry = {"id": spec["id"], "cap_s": spec["cap_s"]}
        disk = base.free_gb(run_dir)
        entry["free_disk_gb_before"] = disk
        if disk < p["run"]["disk_min_free_gb"]:
            stop = "free disk %.2f GB below %.1f GB before %s" % (disk, p["run"]["disk_min_free_gb"], spec["id"])
            report["processes"].append(entry)
            break
        out = run_dir / spec["id"]
        log_path = run_dir / ("%s.log" % spec["id"])
        cmd = [base.PYTHON, "-B", str(LAUNCHER), spec["batch"].lower(), "--process", spec["id"], "--out", str(out)]
        entry["command"] = cmd
        t0, peak, reason = time.time(), 0, None
        with open(log_path, "w") as logf:
            proc = subprocess.Popen(cmd, cwd=str(ROOT), env=process_env(p, spec, out), stdout=logf,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            while proc.poll() is None:
                time.sleep(base.POLL_S)
                try:
                    pids = base.proc_tree_pids(proc.pid)
                    used = sum(a["used_mib"] for a in base.gpu_state()["compute_apps"] if a["pid"] in pids)
                    peak = max(peak, used)
                    if cap_mib is not None and used > cap_mib:
                        reason = "GPU memory %d MiB above the launch-time cap %d MiB" % (used, cap_mib)
                except Exception:  # noqa: BLE001
                    pass
                if time.time() - t0 > spec["cap_s"]:
                    reason = "watchdog: %s exceeded its %d s cap" % (spec["id"], spec["cap_s"])
                if reason:
                    base.kill_group(proc)
                    break
        entry.update(wall_s=time.time() - t0, returncode=proc.returncode, peak_gpu_mib=peak, log=log_path.name)
        done_path = out / "process_done.json"
        done = json.loads(done_path.read_text()) if done_path.exists() else None
        entry["process_done"] = done
        entry["output_bytes"] = base.dir_bytes(out) if out.exists() else 0
        if reason:
            stop = reason
        elif proc.returncode != 0:
            tail = log_path.read_text()[-3000:] if log_path.exists() else ""
            entry["log_tail"] = tail
            stop = "%s exited %s%s" % (spec["id"], proc.returncode, " (CUDA OOM)" if "out of memory" in tail.lower() else "")
        elif done is None or done.get("nan_fields"):
            stop = "%s: missing process_done or NaN fields" % spec["id"]
        if base.git("status", "--porcelain", "--untracked-files=no").stdout != baseline_dirty or \
                base.git("rev-parse", "HEAD").stdout.strip() != head0:
            stop = stop or "tracked tree or HEAD changed during %s" % spec["id"]
        if base.dir_bytes(run_dir) > p["run"]["output_max_gb"] * 1e9:
            stop = stop or "run output above %.1f GB" % p["run"]["output_max_gb"]
        report["processes"].append(entry)
        done_ids.add(spec["id"])
        print("[e_l0-run] %s rc=%s wall=%.1fs peak_gpu=%dMiB%s" % (spec["id"], proc.returncode, entry["wall_s"], peak,
                                                                  (" STOP: " + stop) if stop else ""), flush=True)
        if stop:
            break
        for name, ids in groups.items():
            if spec["id"] == ids[-1] and all(i in done_ids for i in ids):
                cmp = compare_arms(run_dir, ids)
                report["noninterference"][name] = cmp
                print("[e_l0-run] non-interference %s: %s" % (name, "IDENTICAL" if cmp["identical"] else "DIFFERENT"),
                      flush=True)
                if not cmp["identical"]:
                    stop = "NONINTERFERENCE_FAIL in %s; run stopped for cause analysis" % name
        if stop:
            break
    report["status"] = "STOPPED_INTEGRITY" if stop and not stop.startswith("NONINTERFERENCE") else \
        ("STOPPED_NONINTERFERENCE" if stop else "COMPLETED")
    report["stop_reason"] = stop
    report["gpu_seconds_total"] = sum(e.get("wall_s", 0.0) for e in report["processes"])
    report["output_bytes_total"] = base.dir_bytes(run_dir)
    report["free_disk_gb_after"] = base.free_gb(run_dir)
    report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (run_dir / "run_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    report["manifest_members"] = base.write_manifest(run_dir)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("stage", choices=("preflight", "run"))
    parser.add_argument("--approved-commit", default="")
    parser.add_argument("--smoke-root")
    parser.add_argument("--processes")
    args = parser.parse_args(argv)
    p = plan()
    smoke = args.smoke_root is not None
    run_dir = (Path(args.smoke_root).resolve() / ("el0_smoke_%s" % time.strftime("%H%M%S"))) if smoke else \
        base.resolve(p["run"]["output_root"]) / p["run"]["run_id"]
    procs = p["run"]["processes"]
    if args.processes:
        if not smoke:
            print(json.dumps({"problems": ["--processes is for smoke runs only"]}))
            return 2
        procs = [x for x in procs if x["id"] in args.processes.split(",")]
    problems, checks = base.preflight(EQ1D_PLAN, args.approved_commit, run_dir, smoke)
    plan_rel = str(PLAN_PATH.relative_to(ROOT))
    tracked = base.git("ls-files", "--error-unmatch", plan_rel).returncode == 0
    changed = base.git("diff", "--quiet", "HEAD", "--", plan_rel).returncode != 0
    checks["e_l0_plan_committed_unchanged"] = tracked and not changed
    if not smoke and not checks["e_l0_plan_committed_unchanged"]:
        problems.append("E-L0 plan not committed unchanged")
    for key in ("static", "moving"):
        receipt = config_receipt(p, {"config": key})
        checks["receipt_%s_sha256" % key] = base.sha256_file(receipt) if receipt.exists() else None
        if not receipt.exists():
            problems.append("receipt %s missing" % key)
    if args.stage == "preflight" or problems:
        print(json.dumps({"problems": problems, "checks": checks}, indent=1, default=str))
        return 2 if problems else 0
    report = run(p, run_dir, procs, smoke, checks)
    print(json.dumps({k: report[k] for k in ("status", "stop_reason", "gpu_seconds_total", "output_bytes_total",
                                             "manifest_members", "run_dir")}, indent=1))
    return 0 if report["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    sys.exit(main())
