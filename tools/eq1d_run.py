#!/usr/bin/env python3
"""EQ-1D orchestrator: preflight, watchdogged process sequence, manifest (preregistration v1).

    python -B tools/eq1d_run.py preflight --approved-commit SHA
    python -B tools/eq1d_run.py run --approved-commit SHA                # official run (results/)
    python -B tools/eq1d_run.py run --smoke-root SCRATCH [--processes d1_dt10,d3_n1]   # mechanics only

The official run refuses unless the approved commit is HEAD, the tracked tree is clean, the plan is
committed unchanged, Policy F and its RMS digest match, ``aerial_gym``/``motar_eval`` import from this
tree, the run directory is new and free disk is >= 8 GB. Processes run one at a time, each under its
own wall-clock watchdog and a GPU-memory cap measured at launch. An integrity failure stops the run
with no retry; qualification FAILs are not judged here (``tools/eq1d_analysis.py`` does that).
A smoke run writes only under the given scratch directory and is never evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PLAN_PATH = ROOT / "docs/plans/eq1d_qualification_v1.json"
LAUNCHER = ROOT / "tools/eq1d_fixture_launcher.py"
RL_DIR = ROOT / "aerial_gym/rl_training/rl_games"
PYTHON = os.environ.get("PYTHON", sys.executable)
OOM_RESERVE_MIB = 512
POLL_S = 5.0
SCRUB_PREFIXES = ("NAVRL_", "AERIAL_GYM_", "AERIAL_RUN", "AERIAL_TASK")
SCRUB_KEYS = ("TASK", "FILE", "NUM_ENVS", "HEADLESS", "CKPT", "PYTHONPATH")


def plan():
    return json.loads(PLAN_PATH.read_text())


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT)] + list(args), capture_output=True, text=True)


def resolve(rel):
    return (ROOT / rel).resolve()


def free_gb(path):
    return shutil.disk_usage(path).free / 1e9


def gpu_state():
    q = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total,memory.used,memory.free",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=30)
    name, total, used, free = [x.strip() for x in q.stdout.strip().splitlines()[0].split(",")]
    apps = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name,used_memory",
                           "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=30)
    procs = []
    for line in apps.stdout.strip().splitlines():
        parts = [x.strip() for x in line.split(",")]
        if len(parts) == 3 and parts[0].isdigit():
            procs.append({"pid": int(parts[0]), "name": parts[1], "used_mib": int(parts[2] or 0)})
    return {"name": name, "total_mib": int(total), "used_mib": int(used), "free_mib": int(free), "compute_apps": procs}


def receipt_env(p):
    snap = json.loads(resolve(p["frozen"]["receipt_environment"]).read_text())["env_snapshot_navrl"]
    scrub = tuple(p["frozen"]["receipt_scrubbed_prefixes"])
    return {k: str(v) for k, v in snap.items() if not k.startswith(scrub)}


def process_env(p, spec):
    env = {k: v for k, v in os.environ.items() if not k.startswith(SCRUB_PREFIXES) and k not in SCRUB_KEYS}
    env.update(receipt_env(p))
    # As play_navrl.sh: the interpreter's bin first on PATH (gymtorch builds with that env's ninja).
    env["PATH"] = str(Path(PYTHON).resolve().parent) + os.pathsep + env.get("PATH", "")
    env.update(PYTHONPATH=str(ROOT), PYTHONNOUSERSITE="1", NAVRL_SEED=str(spec.get("seed", 4401)),
               TASK=p["frozen"]["task"], FILE=p["frozen"]["player_yaml"], AERIAL_GYM_SIM_NAME=p["frozen"]["sim_name"])
    if spec["batch"] == "D1":
        env["AERIAL_GYM_SIM_NAME"] = "eq1d_" + spec["id"]
    return env


def import_origin(module, cwd, env):
    code = ("import importlib.util, sys\nspec = importlib.util.find_spec(sys.argv[1])\n"
            "print(spec.origin if spec is not None and spec.origin else '')\n")
    done = subprocess.run([PYTHON, "-B", "-c", code, module], cwd=str(cwd), env=env, capture_output=True,
                          text=True, timeout=120)
    lines = [x for x in done.stdout.strip().splitlines() if x.strip()]
    return lines[-1].strip() if lines else ""


def rms_digest_from_checkpoint(path):
    code = ("import hashlib, sys, torch\nck = torch.load(sys.argv[1], map_location='cpu')\nm = ck['model']\n"
            "h = hashlib.sha256()\nfor k in ('running_mean', 'running_var', 'count'):\n"
            "    h.update(m['_orig_mod.running_mean_std.' + k].contiguous().numpy().tobytes())\nprint(h.hexdigest())\n")
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1")
    done = subprocess.run([PYTHON, "-B", "-W", "ignore", "-c", code, str(path)], capture_output=True, text=True,
                          env=env, timeout=300)
    return done.stdout.strip().splitlines()[-1] if done.stdout.strip() else ""


def preflight(p, approved, run_dir, smoke):
    problems, checks = [], {}
    head = git("rev-parse", "HEAD").stdout.strip()
    checks["head"] = head
    tracked_dirty = git("status", "--porcelain", "--untracked-files=no").stdout.strip()
    checks["tracked_tree_clean"] = not tracked_dirty
    plan_tracked = git("ls-files", "--error-unmatch", str(PLAN_PATH.relative_to(ROOT))).returncode == 0
    plan_changed = git("diff", "--quiet", "HEAD", "--", str(PLAN_PATH.relative_to(ROOT))).returncode != 0
    checks["plan_committed_unchanged"] = plan_tracked and not plan_changed
    if not smoke:
        if approved != head:
            problems.append("approved-commit %r is not HEAD %s" % (approved, head))
        if tracked_dirty:
            problems.append("tracked tree not clean")
        if not checks["plan_committed_unchanged"]:
            problems.append("plan not committed unchanged")
    ckpt = RL_DIR / p["frozen"]["policy_f_rel_to_rl_games"]
    checks["policy_f_sha256"] = sha256_file(ckpt) if ckpt.exists() else None
    if checks["policy_f_sha256"] != p["frozen"]["policy_f_sha256"]:
        problems.append("Policy F SHA mismatch")
    checks["rms_digest"] = rms_digest_from_checkpoint(ckpt) if ckpt.exists() else None
    if checks["rms_digest"] != p["frozen"]["rms_digest_sha256"]:
        problems.append("RMS digest mismatch")
    env = process_env(p, p["run"]["processes"][0])
    checks["import_origins"] = {}
    for cwd in (ROOT, RL_DIR):
        for mod in ("aerial_gym", "motar_eval"):
            origin = import_origin(mod, cwd, env)
            checks["import_origins"]["%s@%s" % (mod, cwd.name)] = origin
            if not origin or ROOT.resolve() not in Path(origin).resolve().parents:
                problems.append("%s from %s resolves to %r" % (mod, cwd, origin))
    checks["run_dir"] = str(run_dir)
    if run_dir.exists():
        problems.append("output path exists: %s" % run_dir)
    if not smoke and "results" not in run_dir.parts:
        problems.append("official output must be under results/")
    if smoke and "results" in run_dir.resolve().parts:
        problems.append("smoke output must not be under results/")
    checks["free_disk_gb"] = free_gb(run_dir.parent if run_dir.parent.exists() else ROOT)
    if checks["free_disk_gb"] < p["run"]["disk_min_free_gb"]:
        problems.append("free disk %.2f GB < %.1f GB" % (checks["free_disk_gb"], p["run"]["disk_min_free_gb"]))
    try:
        checks["gpu"] = gpu_state()
    except Exception as error:  # noqa: BLE001
        problems.append("nvidia-smi failed: %s" % error)
        checks["gpu"] = None
    if checks["gpu"] and "3070" not in checks["gpu"]["name"]:
        problems.append("GPU is %s, not the RTX 3070" % checks["gpu"]["name"])
    receipt = resolve(p["frozen"]["receipt_environment"])
    checks["receipt_sha256"] = sha256_file(receipt) if receipt.exists() else None
    if checks["receipt_sha256"] is None:
        problems.append("receipt missing")
    return problems, checks


def proc_tree_pids(pid):
    out = subprocess.run(["ps", "-o", "pid=", "--ppid", str(pid)], capture_output=True, text=True).stdout.split()
    pids = {pid}
    for child in out:
        pids |= proc_tree_pids(int(child))
    return pids


def kill_group(proc):
    try:
        os.killpg(proc.pid, signal.SIGTERM)
        proc.wait(timeout=30)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:  # noqa: BLE001
            pass


def dir_bytes(path):
    return sum(f.stat().st_size for f in Path(path).rglob("*") if f.is_file())


def write_manifest(run_dir):
    members = []
    for f in sorted(Path(run_dir).rglob("*")):
        if f.is_file() and f.name != "RAW_MANIFEST.json":
            members.append({"path": str(f.relative_to(run_dir)), "bytes": f.stat().st_size, "sha256": sha256_file(f)})
    (Path(run_dir) / "RAW_MANIFEST.json").write_text(json.dumps({"members": members}, indent=1) + "\n")
    return len(members)


def run(p, run_dir, processes, smoke, checks):
    run_dir.mkdir(parents=True, exist_ok=False)
    gpu0 = checks.get("gpu") or {}
    cap_mib = (gpu0.get("total_mib", 0) - gpu0.get("used_mib", 0) - OOM_RESERVE_MIB) if gpu0 else None
    report = {"schema": "motar.eq1d-run-report.v1", "run_dir": str(run_dir), "smoke": smoke,
              "plan_sha256": sha256_file(PLAN_PATH), "preflight": checks, "gpu_memory_cap_mib": cap_mib,
              "gpu_memory_cap_rule": p["run"]["gpu_memory_cap"], "processes": [], "status": "RUNNING",
              "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    baseline_dirty = git("status", "--porcelain", "--untracked-files=no").stdout
    head0 = git("rev-parse", "HEAD").stdout.strip()
    stop = None
    for spec in processes:
        entry = {"id": spec["id"], "cap_s": spec["cap_s"]}
        disk = free_gb(run_dir)
        entry["free_disk_gb_before"] = disk
        if disk < p["run"]["disk_min_free_gb"]:
            stop = "free disk %.2f GB below %.1f GB before %s" % (disk, p["run"]["disk_min_free_gb"], spec["id"])
            report["processes"].append(entry)
            break
        out = run_dir / spec["id"]
        log_path = run_dir / ("%s.log" % spec["id"])
        cmd = [PYTHON, "-B", str(LAUNCHER), spec["batch"].lower(), "--process", spec["id"], "--out", str(out)]
        entry["command"] = cmd
        t0 = time.time()
        peak = 0
        with open(log_path, "w") as logf:
            proc = subprocess.Popen(cmd, cwd=str(ROOT), env=process_env(p, spec), stdout=logf, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            reason = None
            while proc.poll() is None:
                time.sleep(POLL_S)
                elapsed = time.time() - t0
                try:
                    pids = proc_tree_pids(proc.pid)
                    used = sum(a["used_mib"] for a in gpu_state()["compute_apps"] if a["pid"] in pids)
                    peak = max(peak, used)
                    if cap_mib is not None and used > cap_mib:
                        reason = "GPU memory %d MiB above the launch-time cap %d MiB" % (used, cap_mib)
                except Exception:  # noqa: BLE001
                    pass
                if elapsed > spec["cap_s"]:
                    reason = "watchdog: %s exceeded its %d s cap" % (spec["id"], spec["cap_s"])
                if reason:
                    kill_group(proc)
                    break
        entry.update(wall_s=time.time() - t0, returncode=proc.returncode, peak_gpu_mib=peak, log=log_path.name)
        done_path = out / "process_done.json"
        done = json.loads(done_path.read_text()) if done_path.exists() else None
        entry["process_done"] = done
        entry["output_bytes"] = dir_bytes(out) if out.exists() else 0
        if reason:
            stop = reason
        elif proc.returncode != 0:
            tail = log_path.read_text()[-3000:] if log_path.exists() else ""
            stop = "%s exited %s" % (spec["id"], proc.returncode)
            entry["log_tail"] = tail
            if "out of memory" in tail.lower():
                stop += " (CUDA OOM)"
        elif done is None or done.get("nan_fields"):
            stop = "%s: missing process_done or NaN fields %s" % (spec["id"], done and done.get("nan_fields"))
        if git("status", "--porcelain", "--untracked-files=no").stdout != baseline_dirty or \
                git("rev-parse", "HEAD").stdout.strip() != head0:
            stop = stop or "tracked tree or HEAD changed during %s" % spec["id"]
        if dir_bytes(run_dir) > p["run"]["output_max_gb"] * 1e9:
            stop = stop or "run output above %.1f GB" % p["run"]["output_max_gb"]
        report["processes"].append(entry)
        print("[eq1d-run] %s rc=%s wall=%.1fs peak_gpu=%dMiB%s" % (spec["id"], proc.returncode, entry["wall_s"], peak,
                                                                   (" STOP: " + stop) if stop else ""), flush=True)
        if stop:
            break
    report["status"] = "STOPPED_INTEGRITY" if stop else "COMPLETED"
    report["stop_reason"] = stop
    report["gpu_seconds_total"] = sum(e.get("wall_s", 0.0) for e in report["processes"])
    report["output_bytes_total"] = dir_bytes(run_dir)
    report["free_disk_gb_after"] = free_gb(run_dir)
    report["finished_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    (run_dir / "run_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    report["manifest_members"] = write_manifest(run_dir)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("stage", choices=("preflight", "run"))
    parser.add_argument("--approved-commit", default="")
    parser.add_argument("--smoke-root")
    parser.add_argument("--processes", help="comma-separated subset (smoke only)")
    args = parser.parse_args(argv)
    p = plan()
    smoke = args.smoke_root is not None
    if smoke:
        run_dir = Path(args.smoke_root).resolve() / ("smoke_%s" % time.strftime("%H%M%S"))
    else:
        run_dir = resolve(p["run"]["output_root"]) / p["run"]["run_id"]
    procs = p["run"]["processes"]
    if args.processes:
        if not smoke:
            print(json.dumps({"problems": ["--processes is for smoke runs only"]}))
            return 2
        wanted = args.processes.split(",")
        procs = [x for x in procs if x["id"] in wanted]
    problems, checks = preflight(p, args.approved_commit, run_dir, smoke)
    if args.stage == "preflight" or problems:
        print(json.dumps({"problems": problems, "checks": checks}, indent=1, default=str))
        return 2 if problems else 0
    report = run(p, run_dir, procs, smoke, checks)
    print(json.dumps({k: report[k] for k in ("status", "stop_reason", "gpu_seconds_total", "output_bytes_total",
                                             "manifest_members", "run_dir")}, indent=1))
    return 0 if report["status"] == "COMPLETED" else 1


if __name__ == "__main__":
    sys.exit(main())
