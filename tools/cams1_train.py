"""CAM-S1 driver (docs/plans/prereg_camonly_s1_scratch_2026-11-23.md): preflight, then one run of
aerial_gym/rl_training/rl_games/train_navrl_cams1.sh with the preregistered values, a wall cap, a run report and the
host-fault count before and after. Training only; evaluation is a separate step.

Preflight refusals: branch other than camonly/dev; preregistration not committed at HEAD; tracked or untracked
changes under aerial_gym/, tools/, resources/, docs/plans/; another CUDA process; less than MIN_FREE_GB free; an
existing unit folder for this run.

While the run is active a watcher parses the launcher log's epoch blocks ("epoch : N / M", "captured (success)",
"crash", "timeout (no capture)", "density bars"), writes progress.json every PROGRESS_EVERY_S, and applies the
preregistered stop rule once: at epoch STOP_CHECK_EPOCH, if the density is still the curriculum start and the
episode-weighted capture rate of the last STOP_WINDOW_EPOCHS epochs is below STOP_MIN_CAPTURE, the run is stopped
(SIGINT, then SIGTERM) and the verdict is STOPPED_NO_LEARNING.
The smoke runs SMOKE_EPOCHS (800 steps > the 600-step episode limit, so every environment finishes at least one
episode; with 2 epochs the trainer's final save fails when no episode has ended, smoke01 of 2026-11-23).
usage: cams1_train.py {--smoke | --run} [--max-hours H] [--max-epochs N]
"""
import argparse
import datetime
import json
import os
import pathlib
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
RL = ROOT / "aerial_gym" / "rl_training" / "rl_games"
LAUNCHER = RL / "train_navrl_cams1.sh"
PREREG = "docs/plans/prereg_camonly_s1_scratch_2026-11-23.md"
BRANCH = "camonly/dev"
UNIT = pathlib.Path.home() / "motar_tmp" / "cams1_20261123"
MIN_FREE_GB = 1.0
GUARDED = ("aerial_gym", "tools", "resources", "docs/plans")
# Camera-only gate calibration (preregistration "Curriculum"): F's fresh schedule x RATIO, where RATIO = CAM-K2 v3's
# last-window training-rollout capture at 205 bars (26.73 / 28.51 %, docs/evidence/camonly_2026-11-04/camk2/README.md
# line 96) over the F-lineage LiDAR reference in the same setting (GR-1 ctrl 79.84 %, .../camk1/README.md line 85).
F_SCHEDULE = ((70, 0.82), (85, 0.77), (100, 0.72), (115, 0.70))
F_START, F_END = 0.80, 0.70
RATIO = ((26.73 + 28.51) / 2.0) / 79.84


def gate_values():
    sched = ",".join("%d:%.3f" % (b, round(t * RATIO, 3)) for b, t in F_SCHEDULE)
    return sched, "%.3f" % round(F_START * RATIO, 3), "%.3f" % round(F_END * RATIO, 3)


SCHEDULE, START, END = gate_values()
# Preregistered values (the preregistration's table "Settings"); changing one needs an amendment.
VALUES = {"CAMS1_SEED": "1", "CAMS1_DENSITY_MIN_EPOCHS": "500", "CAMS1_DENSITY_WARMUP": "500",
          "CAMS1_THRESHOLD_SCHEDULE": SCHEDULE, "CAMS1_THRESHOLD_START": START, "CAMS1_THRESHOLD_END": END,
          "CAMS1_FREE_MPS": "2.5", "CAMS1_BLIND_MPS": "1.0", "CAMS1_BLIND_WEIGHT": "0.5",
          "CAMS1_COLLISION_PENALTY": "-40", "CAMS1_RSS_CLIP_M": "4.0", "CAMS1_P9": "1",
          "NAVRL_SAVE_FREQUENCY": "250"}
STOP_CHECK_EPOCH = 3000
STOP_WINDOW_EPOCHS = 200
STOP_MIN_CAPTURE = 0.10
START_BARS = 70
PROGRESS_EVERY_S = 300
RUN_CORES = "0-3,8-11,14-23"
BLOCK = re.compile(r"epoch\s*:\s*(\d+)\s*/\s*(\d+).*?captured \(success\)\s*:\s*[\d.]+%\s*\((\d+)/(\d+)\)"
                   r".*?crash\s*:\s*[\d.]+%\s*\((\d+)/\d+\).*?timeout \(no capture\)\s*:\s*[\d.]+%\s*\((\d+)/\d+\)"
                   r".*?density bars\s*:\s*(\d+)", re.S)


def parse_blocks(text):
    """[(epoch, max_epoch, captured, episodes, crashed, timeouts, bars)] in log order."""
    return [tuple(int(g) for g in m.groups()) for m in BLOCK.finditer(text)]


def window_summary(blocks, n):
    tail = blocks[-n:]
    eps = sum(b[3] for b in tail)
    return {"epochs": len(tail), "episodes": eps,
            "capture": (sum(b[2] for b in tail) / eps) if eps else None,
            "crash": (sum(b[4] for b in tail) / eps) if eps else None,
            "timeout": (sum(b[5] for b in tail) / eps) if eps else None}


def stop_rule(blocks):
    """True when the preregistered no-learning rule fires (evaluated once, at STOP_CHECK_EPOCH)."""
    if not blocks or blocks[-1][0] < STOP_CHECK_EPOCH:
        return False
    w = window_summary(blocks, STOP_WINDOW_EPOCHS)
    return blocks[-1][6] == START_BARS and w["capture"] is not None and w["capture"] < STOP_MIN_CAPTURE
DEFAULT_MAX_EPOCHS = 12000
SMOKE_EPOCHS = 25
DEFAULT_MAX_HOURS = 17.0


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT)] + list(args), check=True, capture_output=True,
                          text=True).stdout.strip()


def host_faults():
    out = subprocess.run("journalctl -k -b 0 --no-pager | grep -cE 'Machine check|segfault|general protection'",
                         shell=True, capture_output=True, text=True).stdout.strip()
    return int(out or 0)


def cuda_processes():
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader"],
                         capture_output=True, text=True)
    return [line for line in out.stdout.splitlines() if line.strip()]


def preflight(run_dir):
    problems = []
    if git("rev-parse", "--abbrev-ref", "HEAD") != BRANCH:
        problems.append("branch is not %s" % BRANCH)
    if subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e", "HEAD:" + PREREG]).returncode != 0:
        problems.append("preregistration %s is not committed at HEAD" % PREREG)
    dirty = git("status", "--porcelain=v1", "--untracked-files=all", "--", *GUARDED)
    if dirty:
        problems.append("uncommitted changes under %s:\n%s" % (", ".join(GUARDED), dirty[:2000]))
    procs = cuda_processes()
    if procs:
        problems.append("CUDA processes running: %s" % procs)
    free_gb = shutil.disk_usage(str(ROOT)).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("free disk %.2f GB < %.1f GB" % (free_gb, MIN_FREE_GB))
    if run_dir.exists():
        problems.append("run folder %s exists" % run_dir)
    if problems:
        raise SystemExit("[cams1] preflight refused:\n- " + "\n- ".join(problems))
    return {"head": git("rev-parse", "HEAD"), "free_gb": round(free_gb, 2)}


def main():
    ap = argparse.ArgumentParser()
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--smoke", action="store_true", help="SMOKE_EPOCHS epochs, development only, not a result")
    mode.add_argument("--run", action="store_true")
    ap.add_argument("--max-hours", type=float, default=DEFAULT_MAX_HOURS)
    ap.add_argument("--max-epochs", type=int, default=DEFAULT_MAX_EPOCHS)
    args = ap.parse_args()
    name = "train_s1_smoke02" if args.smoke else "train_s1"
    run_dir = UNIT / name
    pre = preflight(run_dir)
    run_dir.mkdir(parents=True)
    env = {k: v for k, v in os.environ.items() if not k.startswith(("NAVRL_", "CAMS1_", "CAMK", "REALVEH_"))}
    env.update(VALUES)
    env.update({"CAMS1_MAX_EPOCHS": str(SMOKE_EPOCHS) if args.smoke else str(args.max_epochs),
                "CAMS1_TAG_SUFFIX": "-smoke" if args.smoke else "",
                "NAVRL_REALVEH_RECEIPT": str(run_dir / "receipt.json"),
                "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "PYTHONNOUSERSITE": "1"})
    hours = 0.5 if args.smoke else args.max_hours
    report = {"unit": "CAM-S1", "mode": "smoke" if args.smoke else "run", "preregistration": PREREG,
              "preflight": pre, "values": VALUES, "run_cores": RUN_CORES, "max_epochs": env["CAMS1_MAX_EPOCHS"], "max_hours": hours,
              "faults_before": host_faults(), "started": datetime.datetime.now().isoformat(timespec="seconds")}
    (run_dir / "run_report.json").write_text(json.dumps(report, indent=1) + "\n")
    log_path = run_dir / "launcher.log"
    state = {"stopped": False, "checked": False}
    with open(log_path, "w") as log:
        # cores 4-5 are excluded (host MCE hazard); 6, 7, 12, 13 stay free for disjoint CPU work (D-2026-11-05-1)
        proc = subprocess.Popen(["taskset", "-c", RUN_CORES, "timeout", "-k", "120", "%ds" % int(hours * 3600),
                                 "bash", str(LAUNCHER)],
                                cwd=str(RL), env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def watch():
            t0 = time.time()
            while proc.poll() is None:
                time.sleep(30)
                try:
                    blocks = parse_blocks(log_path.read_text(errors="replace"))
                except OSError:
                    continue
                if blocks:
                    prog = {"time": datetime.datetime.now().isoformat(timespec="seconds"),
                            "elapsed_h": round((time.time() - t0) / 3600, 3), "epoch": blocks[-1][0],
                            "max_epoch": blocks[-1][1], "density_bars": blocks[-1][6],
                            "last_100": window_summary(blocks, 100), "last_10": window_summary(blocks, 10)}
                    (run_dir / "progress.json").write_text(json.dumps(prog, indent=1) + "\n")
                    if not args.smoke and not state["checked"] and blocks[-1][0] >= STOP_CHECK_EPOCH:
                        state["checked"] = True
                        state["check"] = {"epoch": blocks[-1][0], "density_bars": blocks[-1][6],
                                          "window": window_summary(blocks, STOP_WINDOW_EPOCHS)}
                        if stop_rule(blocks):
                            state["stopped"] = True
                            os.killpg(proc.pid, signal.SIGINT)
                            time.sleep(120)
                            if proc.poll() is None:
                                os.killpg(proc.pid, signal.SIGTERM)

        watcher = threading.Thread(target=watch, daemon=True)
        watcher.start()
        proc.wait()
    report.update({"exit_code": proc.returncode, "faults_after": host_faults(),
                   "ended": datetime.datetime.now().isoformat(timespec="seconds"),
                   "stop_check": state.get("check"),
                   "verdict": "STOPPED_NO_LEARNING" if state["stopped"] else
                   ("COMPLETED_PROCESS" if proc.returncode == 0 else
                    ("WALL_CAP" if proc.returncode in (124, 137) else "TRAINING_FAIL"))})
    (run_dir / "run_report.json").write_text(json.dumps(report, indent=1) + "\n")
    print("[cams1] %s exit=%d faults %d->%d report=%s" % (report["verdict"], proc.returncode, report["faults_before"],
                                                         report["faults_after"], run_dir / "run_report.json"))
    return 0 if proc.returncode == 0 and not state["stopped"] else 1


if __name__ == "__main__":
    sys.exit(main())
