"""CAM-S2b driver (docs/plans/prereg_camonly_s2b_2026-11-24.md), built on tools/cams2a_train.py: preflight, then one run
of aerial_gym/rl_training/rl_games/train_navrl_cams2b.sh with the preregistered values, a wall cap, run-time contract
checks, a run report with the preregistered verdict, and the host-fault count before and after. Training only.

  python3 tools/cams2b_train.py --run ft [--smoke]     S2b-FT: warm start from CAM-S2a ep12000, yaw 1.0472 rad/s,
                                                       epochs 12001..15000, wall cap 4.5 h
  python3 tools/cams2b_train.py --run scr [--smoke]    S2b-SCR: from scratch, yaw 1.0472 rad/s, 12,000 epochs, 17 h

Unit folder ~/motar_tmp/cams2b_20261124/{train_ft, train_scr, train_<run>_smoke_HHMMSS}.

Preflight refusals: branch other than camonly/dev; preregistration not committed at HEAD; tracked or untracked changes
under aerial_gym/, tools/, resources/, docs/plans/; a CUDA compute process other than the RustDesk display server
(CUDA_ALLOWED, exact path); free disk below MIN_FREE_GB + the run's estimate; the CAM-S2a receipt (the environment
reference) or, for ft, the source checkpoint not matching its pinned SHA-256; an existing run folder.

Run-time checks (a failure stops the run, verdict TRAINING_FAIL):
  - receipt: once tools/realveh_entry.py has written receipt.json, its env_navrl must equal CAM-S2a's receipt env_navrl
    except exactly the preregistered differences (expected_env), and it must carry no evaluation-only wrapper
    (yawclip, cams2diag, cams2_contactcode);
  - start: once the first epoch block is logged, the wrapper's yaw line must be present; for ft the checkpoint restore
    (task_steps 384000, 175 bars, k 20..28), the yaw-cap CONFIG MISMATCH warning and the density-evidence reset with the
    1000-epoch resume warm-up must be logged and the first epoch must be 12001; for scr no restore and epoch <= 2.
Stop rule (once, at STOP_CHECK_EPOCH): episode-weighted capture of the last STOP_WINDOW_EPOCHS epochs below
STOP_MIN_CAPTURE -> STOPPED_NO_LEARNING (SIGINT, then SIGTERM after 120 s).
Verdicts: TRAINED, STOPPED_NO_LEARNING, WALL_CAP, TRAINING_FAIL, VOID_HOST_FAULT (smoke: SMOKE_COMPLETED / SMOKE_FAIL /
VOID_HOST_FAULT). The report also hashes every last_gen checkpoint (the last and every 1000th listed separately, review
r1 F11), summarises each density stage and lists every promotion with its epoch.
"""
import argparse
import datetime
import hashlib
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
LAUNCHER = RL / "train_navrl_cams2b.sh"
WRAPPER = ROOT / "tools" / "cams2b_python"
PREREG = "docs/plans/prereg_camonly_s2b_2026-11-24.md"
BRANCH = "camonly/dev"
UNIT = pathlib.Path.home() / "motar_tmp" / "cams2b_20261124"
GUARDED = ("aerial_gym", "tools", "resources", "docs/plans")
MIN_FREE_GB = 1.0
RUN_CORES = "0-3,8-11,14-23"
PROGRESS_EVERY_S = 30
CUDA_ALLOWED = ("/usr/share/rustdesk/rustdesk",)      # RustDesk display/encoder process, not a training job

# The environment reference: CAM-S2a's training receipt (env_navrl = the exact NAVRL_* environment of that run).
S2A_RECEIPT = pathlib.Path.home() / "motar_tmp" / "cams2a_20261123" / "train_s1" / "receipt.json"
S2A_RECEIPT_SHA = "e3e9a995c5f00f206a9dcc5fa859b3bd25d2fdb38b8052d7390c041c782a076d"
# S2b-FT source: CAM-S2a's final checkpoint (read-only; copied into the unit folder before use because runner.py
# writes a *_rlnorm.pth next to the checkpoint it loads).
SOURCE_RUN = "ppo_261123_2213_navrl_v2-cams2a-scratch-camonly-s1"
SOURCE_FILE = "last_gen_ppo_ep_12000_rew_104.0523.pth"
SOURCE_CKPT = (pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly/aerial_gym/rl_training/rl_games/runs")
               / SOURCE_RUN / "nn" / SOURCE_FILE)
SOURCE_SHA = "9fc9dfb7edfc90db0b9a86432eb8cfb172a6e708a6d58eb1d40f15b6ed548996"
SOURCE_EPOCH = 12000
SOURCE_TASK_STEPS = 384000          # env_state num_task_steps = 12000 epochs x horizon 32
SOURCE_BARS = 175                   # env_state n_bars_active
SOURCE_LR = "7.5e-06"               # env_state current_action_learning_rate (3e-5 halved twice by the rollback)

YAW_RATE_MAX = "1.0472"             # 60 deg/s, PX4 MPC_YAWRAUTO_MAX of the real vehicle
DENSITY_CHECK_EPS = "16"            # <= the fewest episodes per epoch CAM-S2a logged per stage (17.8); see prereg
RESUME_WARMUP = "1000"
# CAM-S2a's preregistered values, unchanged (tools/cams2a_train.py VALUES).
SCHEDULE, START, END = "70:0.001,85:0.001,100:0.001,115:0.001", "0.001", "0.001"
VALUES = {"CAMS2_SEED": "1", "CAMS2_DENSITY_MIN_EPOCHS": "1000", "CAMS2_DENSITY_WARMUP": "1000",
          "CAMS2_THRESHOLD_SCHEDULE": SCHEDULE, "CAMS2_THRESHOLD_START": START, "CAMS2_THRESHOLD_END": END,
          "CAMS2_FREE_MPS": "2.5", "CAMS2_BLIND_MPS": "1.0", "CAMS2_BLIND_WEIGHT": "0.5",
          "CAMS2_COLLISION_PENALTY": "-40", "CAMS2_RSS_CLIP_M": "4.0", "CAMS2_P9": "1",
          "CAMS2_DELAY_MIN": "1", "CAMS2_DELAY_MAX": "2", "CAMS2_DETECT_WIDTH": "312", "CAMS2_DETECT_HEIGHT": "234",
          "NAVRL_SAVE_FREQUENCY": "250",
          "CAMS2B_YAW_RATE_MAX": YAW_RATE_MAX, "CAMS2B_DENSITY_CHECK_EPS": DENSITY_CHECK_EPS}
SMOKE_EPOCHS = 25        # 800 steps > the 600-step episode limit (cams2a_train.py)
RUNS = {
    "ft": {"max_epochs": SOURCE_EPOCH + 3000, "max_hours": 4.5, "smoke_max_epochs": SOURCE_EPOCH + SMOKE_EPOCHS,
           "stop_check_epoch": SOURCE_EPOCH + 1000, "stop_window": 200, "stop_min_capture": 0.30,
           "disk_estimate_gb": 0.2,
           # 175 is held by the 1000-epoch resume warm-up until 13000; 190 then dwells 1000 epochs
           "expected_promotions": [(175, 190, SOURCE_EPOCH + 1000), (190, 205, None)],
           "extra": {"CAMS2B_CKPT": None, "CAMS2B_LEARNING_RATE": SOURCE_LR}},
    "scr": {"max_epochs": 12000, "max_hours": 17.0, "smoke_max_epochs": SMOKE_EPOCHS,
            "stop_check_epoch": 3000, "stop_window": 200, "stop_min_capture": 0.05,
            "disk_estimate_gb": 0.55,
            "expected_promotions": [(70, 85, 1000)] + [(85 + 15 * j, 100 + 15 * j, None) for j in range(8)],
            "extra": {}},
}
PROMOTION_TOLERANCE_EPOCHS = 10
MIN_DWELL = 1000          # CAMS2_DENSITY_MIN_EPOCHS
EVAL_ONLY_PREFIXES = ("NAVRL_YAWCLIP", "NAVRL_CAMS2DIAG", "NAVRL_CAMS2_CONTACTCODE")
EVAL_ONLY_RECEIPT_KEYS = ("yawclip", "cams2diag", "cams2_contactcode")
STRIP_PREFIXES = ("NAVRL_", "CAMS1_", "CAMS2_", "CAMS2B_", "CAMK", "REALVEH_")
STRIP_NAMES = ("CKPT", "MAX_EPOCHS", "SEED", "NUM_ENVS", "FILE", "TASK", "GPU4GB", "AERIAL_RUN_TAG", "HEADLESS",
               "AERIAL_GYM_SIM_NAME", "PYTHON", "TRAIN_SESSION_LOG", "TRAIN_LIVE_LOG",
               # runner.py:1027 only setdefault()s these; a stale value would rename the run (review r1 F4)
               "AERIAL_RUN_NAME", "AERIAL_RUN_TITLE", "AERIAL_TASK_CONFIG_MODULE")
DRIVER_CORES = {6, 7, 12, 13}      # the driver/watcher itself (review r1 F10); the runner gets RUN_CORES

BLOCK = re.compile(r"epoch\s*:\s*(\d+)\s*/\s*(\d+).*?captured \(success\)\s*:\s*[\d.]+%\s*\((\d+)/(\d+)\)"
                   r".*?crash\s*:\s*[\d.]+%\s*\((\d+)/\d+\).*?timeout \(no capture\)\s*:\s*[\d.]+%\s*\((\d+)/\d+\)"
                   r".*?density bars\s*:\s*(\d+)", re.S)
PROMOTED = re.compile(r"density curriculum promoted \| bars (\d+) -> (\d+) after (\d+) eps, capture=([\d.]+)"
                      r" \(threshold=[\d.]+\) dwell=(\d+) epochs")
RUN_NAME = re.compile(r"^\s*run\s*:\s*(ppo_\S+)\s*$", re.M)
EXIT_REASON = re.compile(r"exit reason\s*:\s*(.+?)\s*$", re.M)


# ----------------------------------------------------------------------------------------------------- log parsing
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


def stop_window(blocks, spec):
    """The preregistered window: the last stop_window epochs up to and including stop_check_epoch (blocks logged after
    it are excluded, review r1 F5). None before the check epoch has been logged."""
    if not blocks or blocks[-1][0] < spec["stop_check_epoch"]:
        return None
    upto = [b for b in blocks if b[0] <= spec["stop_check_epoch"]]
    return window_summary(upto, spec["stop_window"])


def stop_rule(blocks, spec):
    """True when the preregistered no-learning rule fires (evaluated once, at the run's stop_check_epoch)."""
    w = stop_window(blocks, spec)
    return w is not None and w["capture"] is not None and w["capture"] < spec["stop_min_capture"]


def parse_promotions(text):
    """Every density promotion with the epoch of the last epoch block logged before it."""
    events = [(m.start(), "block", int(m.group(1))) for m in BLOCK.finditer(text)]
    events += [(m.start(), "promo", m) for m in PROMOTED.finditer(text)]
    out, epoch = [], 0
    for _, kind, value in sorted(events, key=lambda e: e[0]):
        if kind == "block":
            epoch = value
        else:
            out.append({"after_epoch": epoch, "from": int(value.group(1)), "to": int(value.group(2)),
                        "episodes": int(value.group(3)), "capture": float(value.group(4)),
                        "dwell_epochs": int(value.group(5))})
    return out


def curriculum_check(promotions, spec):
    """Compare the promotions with the preregistered time-based schedule (recorded, not a stop criterion). A row with
    an epoch must promote during epochs [epoch + 1, epoch + PROMOTION_TOLERANCE_EPOCHS]; a row without one must have
    dwelt MIN_DWELL..MIN_DWELL + PROMOTION_TOLERANCE_EPOCHS epochs at the previous density."""
    rows, ok = [], True
    for i, (frm, to, epoch) in enumerate(spec["expected_promotions"]):
        got = promotions[i] if i < len(promotions) else None
        good = bool(got and got["from"] == frm and got["to"] == to)
        if good and epoch is not None:
            good = epoch + 1 <= got["after_epoch"] + 1 <= epoch + PROMOTION_TOLERANCE_EPOCHS
        elif good:
            good = MIN_DWELL <= got["dwell_epochs"] <= MIN_DWELL + PROMOTION_TOLERANCE_EPOCHS
        rows.append({"expected": [frm, to, epoch], "logged": got, "as_preregistered": good})
        ok = ok and good
    extra = promotions[len(spec["expected_promotions"]):]
    return {"rows": rows, "unexpected_extra": extra, "as_preregistered": ok and not extra}


def stage_summary(blocks):
    stages = []
    for b in blocks:
        if not stages or stages[-1]["bars"] != b[6]:
            stages.append({"bars": b[6], "epoch_first": b[0], "epoch_last": b[0], "epochs": 0, "episodes": 0,
                           "captured": 0, "crashed": 0, "timeouts": 0})
        s = stages[-1]
        s["epoch_last"] = b[0]
        s["epochs"] += 1
        s["episodes"] += b[3]
        s["captured"] += b[2]
        s["crashed"] += b[4]
        s["timeouts"] += b[5]
    for s in stages:
        n = s["episodes"]
        s.update({k: (round(s[c] / n, 4) if n else None) for k, c in
                  (("capture", "captured"), ("crash", "crashed"), ("timeout", "timeouts"))})
    return stages


# ------------------------------------------------------------------------------------------- environment contract
def expected_env(s2a_env, run, root, receipt_path):
    """CAM-S2a's env_navrl with exactly the preregistered CAM-S2b differences."""
    env = dict(s2a_env)
    env.update({"NAVRL_YAW_RATE_MAX": YAW_RATE_MAX, "NAVRL_DENSITY_CHECK_EPS": DENSITY_CHECK_EPS,
                "NAVRL_REALVEH_RECEIPT": str(receipt_path),
                # same file (SHA pinned by the launcher), resolved inside the running tree
                "NAVRL_P9_ERROR_MODEL": str(pathlib.Path(root) / "results/perception_p9_2026-09-09/p9_error_injector.json")})
    if run == "ft":
        env.update({"NAVRL_V2_ALLOW_RESUME": "1", "NAVRL_LEARNING_RATE": SOURCE_LR,
                    "NAVRL_RESET_DENSITY_WINDOW": "1", "NAVRL_DENSITY_RESUME_WARMUP": RESUME_WARMUP})
    return env


def receipt_problems(receipt, s2a_env, run, root, receipt_path):
    problems = []
    env = receipt.get("env_navrl") or {}
    want = expected_env(s2a_env, run, root, receipt_path)
    for key in sorted(set(env) | set(want)):
        if env.get(key) != want.get(key):
            problems.append("env_navrl %s = %r, expected %r" % (key, env.get(key), want.get(key)))
    for key in EVAL_ONLY_RECEIPT_KEYS:
        if key in receipt or key in (receipt.get("settings") or {}):
            problems.append("evaluation-only wrapper %s installed in training" % key)
    if "cams2" not in receipt or "camsafe" not in receipt:
        problems.append("cams2/camsafe install record missing")
    return problems


def start_problems(text, blocks, run, max_epochs):
    """Checks on the start of the log, applied once the first epoch block exists."""
    problems = []
    if not blocks:
        return ["no epoch block logged"]
    first = blocks[0]
    if "[cams2b-python] NAVRL_YAW_RATE_MAX 3.0 -> %s rad/s" % YAW_RATE_MAX not in text:
        problems.append("yaw wrapper line missing")
    if first[1] != max_epochs:
        problems.append("first block max epoch %d != %d" % (first[1], max_epochs))
    restored = [line for line in text.splitlines() if "NavRL checkpoint state restored" in line]
    if run == "ft":
        if not any("task_steps=%d bars=" % SOURCE_TASK_STEPS in line and "->%d " % SOURCE_BARS in line
                   and "k_min=20.0 k_max=28.0" in line for line in restored):
            problems.append("checkpoint restore line (task_steps %d, %d bars, k 20..28) missing"
                            % (SOURCE_TASK_STEPS, SOURCE_BARS))
        if ("NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 3.000, running with %.3f"
                % float(YAW_RATE_MAX)) not in text:
            problems.append("yaw-cap CONFIG MISMATCH warning missing")
        gate = SOURCE_TASK_STEPS + int(RESUME_WARMUP) * 32
        if not re.search(r"density evidence RESET \| discarded=\d+/\d+ eps resume_warmup=%s epochs "
                         r"gate_not_before_step=%d" % (RESUME_WARMUP, gate), text):
            problems.append("density evidence reset with the %s-epoch resume warm-up missing" % RESUME_WARMUP)
        if first[0] != SOURCE_EPOCH + 1:
            problems.append("first epoch %d != %d" % (first[0], SOURCE_EPOCH + 1))
    else:
        if restored:
            problems.append("scr restored a checkpoint state")
        if first[0] > 2:
            problems.append("scr first epoch %d > 2" % first[0])
    return problems


def clean_env(environ):
    return {k: v for k, v in environ.items() if not k.startswith(STRIP_PREFIXES) and k not in STRIP_NAMES}


def child_env(environ, run, smoke, run_dir, ckpt_copy=None):
    env = clean_env(environ)
    env.update(VALUES)
    spec = RUNS[run]
    env.update({"CAMS2B_RUN": run,
                "CAMS2_MAX_EPOCHS": str(spec["smoke_max_epochs"] if smoke else spec["max_epochs"]),
                "CAMS2_TAG_SUFFIX": "-smoke" if smoke else "",
                "NAVRL_REALVEH_RECEIPT": str(pathlib.Path(run_dir) / "receipt.json"),
                "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "PYTHONNOUSERSITE": "1"})
    if run == "ft":
        env.update({"CAMS2B_CKPT": str(ckpt_copy), "CAMS2B_LEARNING_RATE": SOURCE_LR})
    return env


# ----------------------------------------------------------------------------------------------------- verdict
def verdict(smoke, exit_code, stopped, guard_failure, faults_before, faults_after, exit_reason, last_epoch,
            max_epochs, final_ckpt):
    if faults_after is None or faults_before is None or faults_after > faults_before:
        return "VOID_HOST_FAULT"
    if guard_failure:
        return "SMOKE_FAIL" if smoke else "TRAINING_FAIL"
    if stopped and not smoke:
        return "STOPPED_NO_LEARNING"
    if exit_code in (124, 137) and not smoke:
        return "WALL_CAP"
    ok = (exit_code == 0 and (exit_reason or "").startswith("max_epochs") and last_epoch is not None
          and last_epoch >= max_epochs and bool(final_ckpt))
    if smoke:
        return "SMOKE_COMPLETED" if ok else "SMOKE_FAIL"
    return "TRAINED" if ok else "TRAINING_FAIL"


# ------------------------------------------------------------------------------------------------- host checks
def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT)] + list(args), check=True, capture_output=True,
                          text=True).stdout.strip()


def committed(relpath):
    return subprocess.run(["git", "-C", str(ROOT), "cat-file", "-e", "HEAD:" + relpath],
                          capture_output=True).returncode == 0


def host_faults():
    out = subprocess.run("journalctl -k -b 0 --no-pager | grep -cE 'Machine check|segfault|general protection'",
                         shell=True, capture_output=True, text=True).stdout.strip()
    return int(out or 0)


def cuda_lines():
    out = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader"],
                         capture_output=True, text=True)
    if out.returncode != 0:
        return ["nvidia-smi failed (exit %d)" % out.returncode]
    return [line for line in out.stdout.splitlines() if line.strip()]


def foreign_cuda(lines):
    """Compute processes other than the allowed display server (exact process path)."""
    bad = []
    for line in lines:
        name = line.split(",", 1)[1].strip() if "," in line else line.strip()
        if name not in CUDA_ALLOWED:
            bad.append(line.strip())
    return bad


def checkpoint_hashes(nn_dir, max_epochs):
    pat = re.compile(r"^last_gen_ppo_ep_(\d+)_rew_.*\.pth$")
    files = sorted(((int(m.group(1)), p) for p in pathlib.Path(nn_dir).glob("last_gen_ppo_ep_*.pth")
                    for m in [pat.match(p.name)] if m and not p.name.endswith("_rlnorm.pth")), key=lambda t: t[0])
    all_rows = [(e, p.name, sha256(p)) for e, p in files]
    last = all_rows[-1][0] if all_rows else None
    keep = [r for r in all_rows if r[0] % 1000 == 0 or r[0] == last]
    final = [r for r in all_rows if r[0] == max_epochs]
    return all_rows, keep, (final[0] if final else None)


def preflight(run, run_dir, smoke):
    spec = RUNS[run]
    problems = []
    if git("rev-parse", "--abbrev-ref", "HEAD") != BRANCH:
        problems.append("branch is not %s" % BRANCH)
    if not committed(PREREG):
        problems.append("preregistration %s is not committed at HEAD" % PREREG)
    dirty = git("status", "--porcelain=v1", "--untracked-files=all", "--", *GUARDED)
    if dirty:
        problems.append("uncommitted changes under %s:\n%s" % (", ".join(GUARDED), dirty[:2000]))
    lines = cuda_lines()
    bad = foreign_cuda(lines)
    if bad:
        problems.append("CUDA processes running: %s" % bad)
    free_gb = shutil.disk_usage(str(ROOT)).free / 1e9
    need = MIN_FREE_GB + (0.05 if smoke else spec["disk_estimate_gb"])
    if free_gb < need:
        problems.append("free disk %.2f GB < %.2f GB (floor %.1f + estimate)" % (free_gb, need, MIN_FREE_GB))
    if not (S2A_RECEIPT.is_file() and sha256(S2A_RECEIPT) == S2A_RECEIPT_SHA):
        problems.append("CAM-S2a receipt %s missing or not the pinned SHA-256" % S2A_RECEIPT)
    if not (LAUNCHER.is_file() and WRAPPER.is_file() and os.access(str(WRAPPER), os.X_OK)):
        problems.append("launcher or executable wrapper missing")
    if run == "ft" and not (SOURCE_CKPT.is_file() and sha256(SOURCE_CKPT) == SOURCE_SHA):
        problems.append("source checkpoint %s missing or not the pinned SHA-256" % SOURCE_CKPT)
    if run_dir.exists():
        problems.append("run folder %s exists" % run_dir)
    if problems:
        raise SystemExit("[cams2b] preflight refused:\n- " + "\n- ".join(problems))
    return {"head": git("rev-parse", "HEAD"), "branch": BRANCH, "free_gb": round(free_gb, 2),
            "free_gb_required": round(need, 2), "cuda_processes_seen": lines}


def source_copy():
    """Copy the source checkpoint into the unit folder (runs/<name>/nn layout kept for the run lineage); verify it."""
    dest = UNIT / "source" / "runs" / SOURCE_RUN / "nn" / SOURCE_FILE
    if not dest.exists():
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_suffix(".partial")
        shutil.copyfile(str(SOURCE_CKPT), str(tmp))
        os.replace(str(tmp), str(dest))
    if sha256(dest) != SOURCE_SHA:
        raise SystemExit("[cams2b] source copy %s does not match the pinned SHA-256" % dest)
    return dest


# --------------------------------------------------------------------------------------------------------- main
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", choices=sorted(RUNS), required=True)
    ap.add_argument("--smoke", action="store_true", help="development smoke (%d epochs), not a result" % SMOKE_EPOCHS)
    args = ap.parse_args(argv)
    spec = RUNS[args.run]
    try:
        os.sched_setaffinity(0, DRIVER_CORES & os.sched_getaffinity(0) or DRIVER_CORES)
    except OSError as exc:
        print("[cams2b] could not pin the driver to %s: %s" % (sorted(DRIVER_CORES), exc))
    name = ("train_%s_smoke_%s" % (args.run, datetime.datetime.now().strftime("%H%M%S"))) if args.smoke \
        else "train_%s" % args.run
    run_dir = UNIT / name
    pre = preflight(args.run, run_dir, args.smoke)
    s2a_env = json.loads(S2A_RECEIPT.read_text())["env_navrl"]
    ckpt = source_copy() if args.run == "ft" else None
    run_dir.mkdir(parents=True)
    env = child_env(os.environ, args.run, args.smoke, run_dir, ckpt)
    max_epochs = int(env["CAMS2_MAX_EPOCHS"])
    hours = 0.5 if args.smoke else spec["max_hours"]
    report = {"unit": "CAM-S2b", "run": args.run, "mode": "smoke" if args.smoke else "run", "preregistration": PREREG,
              "preflight": pre, "values": VALUES, "run_cores": RUN_CORES, "max_epochs": max_epochs, "max_hours": hours,
              "stop_rule": None if args.smoke else {k: spec[k] for k in
                                                     ("stop_check_epoch", "stop_window", "stop_min_capture")},
              "source_checkpoint": None if ckpt is None else {"path": str(SOURCE_CKPT), "copy": str(ckpt),
                                                              "sha256": SOURCE_SHA, "epoch": SOURCE_EPOCH,
                                                              "learning_rate": SOURCE_LR},
              "env_reference": {"path": str(S2A_RECEIPT), "sha256": S2A_RECEIPT_SHA},
              "sha256": {"driver": sha256(__file__), "launcher": sha256(LAUNCHER), "wrapper": sha256(WRAPPER)},
              "faults_before": host_faults(), "started": datetime.datetime.now().isoformat(timespec="seconds")}
    (run_dir / "run_report.json").write_text(json.dumps(report, indent=1) + "\n")
    log_path = run_dir / "launcher.log"
    receipt_path = run_dir / "receipt.json"
    state = {"stopped": False, "checked": False, "receipt": None, "start": None, "guard_failure": None}
    with open(log_path, "w") as log:
        # cores 4-5 are excluded (host MCE hazard); 6, 7, 12, 13 stay free for disjoint CPU work (D-2026-11-05-1)
        proc = subprocess.Popen(["taskset", "-c", RUN_CORES, "timeout", "-k", "120", "%ds" % int(hours * 3600),
                                 "bash", str(LAUNCHER)],
                                cwd=str(RL), env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

        def stop(reason):
            if state["guard_failure"] is None and not state["stopped"]:
                if reason != "stop_rule":
                    state["guard_failure"] = reason
                else:
                    state["stopped"] = True
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    return
                time.sleep(120)
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGTERM)

        def watch():
            try:
                watch_loop()
            except Exception as exc:          # recorded; the stop rule is re-applied after the run (review r1 F2)
                state["watch_error"] = "%s: %s" % (type(exc).__name__, exc)

        def watch_loop():
            t0 = time.time()
            while proc.poll() is None:
                time.sleep(PROGRESS_EVERY_S)
                if state["receipt"] is None and receipt_path.is_file():
                    try:
                        receipt = json.loads(receipt_path.read_text())
                    except (OSError, ValueError):
                        continue
                    state["receipt"] = receipt_problems(receipt, s2a_env, args.run, ROOT, receipt_path)
                    if state["receipt"]:
                        stop("receipt: " + "; ".join(state["receipt"]))
                        return
                try:
                    text = log_path.read_text(errors="replace")
                except OSError:
                    continue
                blocks = parse_blocks(text)
                if not blocks:
                    continue
                if state["start"] is None:
                    state["start"] = start_problems(text, blocks, args.run, max_epochs)
                    if state["start"]:
                        stop("start: " + "; ".join(state["start"]))
                        return
                prog = {"time": datetime.datetime.now().isoformat(timespec="seconds"),
                        "elapsed_h": round((time.time() - t0) / 3600, 3), "epoch": blocks[-1][0],
                        "max_epoch": blocks[-1][1], "density_bars": blocks[-1][6],
                        "promotions": parse_promotions(text),
                        "last_100": window_summary(blocks, 100), "last_10": window_summary(blocks, 10)}
                (run_dir / "progress.json").write_text(json.dumps(prog, indent=1) + "\n")
                if not args.smoke and not state["checked"] and blocks[-1][0] >= spec["stop_check_epoch"]:
                    state["checked"] = True
                    state["check"] = {"epoch": blocks[-1][0], "density_bars": blocks[-1][6],
                                      "window": stop_window(blocks, spec)}
                    if stop_rule(blocks, spec):
                        stop("stop_rule")
                        return

        watcher = threading.Thread(target=watch, daemon=True)
        watcher.start()
        proc.wait()
        watcher.join(timeout=150)
    text = log_path.read_text(errors="replace")
    blocks = parse_blocks(text)
    # A run that ended before the watcher looked still gets both checks.
    if state["receipt"] is None:
        state["receipt"] = (receipt_problems(json.loads(receipt_path.read_text()), s2a_env, args.run, ROOT,
                                             receipt_path) if receipt_path.is_file() else ["receipt not written"])
    if state["start"] is None:
        state["start"] = start_problems(text, blocks, args.run, max_epochs)
    if state["guard_failure"] is None and (state["receipt"] or state["start"]):
        state["guard_failure"] = "; ".join(state["receipt"] + state["start"])
    # The stop rule if the watcher never applied it (it died, or the run ended between two polls).
    if not args.smoke and not state["checked"] and stop_window(blocks, spec) is not None:
        state["checked"] = True
        state["check"] = {"epoch": spec["stop_check_epoch"], "applied_after_run": True,
                          "window": stop_window(blocks, spec)}
        if stop_rule(blocks, spec) and state["guard_failure"] is None:
            state["guard_failure"] = "stop rule fired at epoch %d but was not applied during the run" \
                % spec["stop_check_epoch"]
    names = RUN_NAME.findall(text)
    reasons = EXIT_REASON.findall(text)
    run_folder = (RL / "runs" / names[0]) if names else None
    all_rows, keep, final = ([], [], None)
    if run_folder is not None and (run_folder / "nn").is_dir():
        all_rows, keep, final = checkpoint_hashes(run_folder / "nn", max_epochs)
        (run_dir / "checkpoint_sha256_all.txt").write_text("".join("%s  %s\n" % (h, n) for _, n, h in all_rows))
        (run_dir / "checkpoint_sha256.txt").write_text("".join("%s  %s\n" % (h, n) for _, n, h in keep))
    promotions = parse_promotions(text)
    report.update({"exit_code": proc.returncode, "exit_reason": reasons[-1] if reasons else None,
                   "faults_after": host_faults(), "ended": datetime.datetime.now().isoformat(timespec="seconds"),
                   "receipt_check": state["receipt"], "start_check": state["start"],
                   "guard_failure": state["guard_failure"], "stop_check": state.get("check"),
                   "watcher_error": state.get("watch_error"),
                   "last_epoch": blocks[-1][0] if blocks else None, "run_folder": str(run_folder) if run_folder else None,
                   "checkpoints_every_1000_and_last": [{"epoch": e, "file": n, "sha256": h} for e, n, h in keep],
                   "final_checkpoint": None if final is None else {"epoch": final[0], "file": final[1],
                                                                   "sha256": final[2]},
                   "stages": stage_summary(blocks), "promotions": promotions,
                   "curriculum_check": None if args.smoke else curriculum_check(promotions, spec)})
    report["verdict"] = verdict(args.smoke, proc.returncode, state["stopped"], state["guard_failure"],
                                report["faults_before"], report["faults_after"], report["exit_reason"],
                                report["last_epoch"], max_epochs, final)
    (run_dir / "run_report.json").write_text(json.dumps(report, indent=1) + "\n")
    print("[cams2b] %s %s exit=%d faults %d->%d report=%s" % (args.run, report["verdict"], proc.returncode,
                                                             report["faults_before"], report["faults_after"],
                                                             run_dir / "run_report.json"))
    return 0 if report["verdict"] in ("TRAINED", "SMOKE_COMPLETED") else 1


if __name__ == "__main__":
    sys.exit(main())
