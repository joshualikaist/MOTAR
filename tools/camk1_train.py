"""CAM-K1 training wrapper (docs/plans/prereg_camonly_k1_train_v1_2026-11-06.md): one seed of the 3,000-epoch
camera-only continuation of Policy F, via train_navrl_camk1.sh.

Fail-closed preflight, before anything is created:
  - UNIT_DIR/execution_approval.json names this file's SCOPE and the commit, which is HEAD; branch camonly/dev;
    clean tracked tree; the preregistration committed;
  - exactly one RTX 3070, no GPU compute process except the user's remote desktop, no NavRL runner, no PX4 SITL or
    Gazebo process; Policy F's SHA;
  - the seed's output directory does not exist (single use).
During the run: a wall-clock cap (timeout -k). Afterwards: kernel faults, boot id and online CPUs unchanged (else
VOID_HOST_FAULT); exactly one new run folder whose tag matches the seed; its last_gen_ppo_ep_28000_*.pth exists; the
camtrain receipt shows no_lidar and the preregistered overrides. The checkpoint is copied into the output folder with
its SHA-256 and the session log.
usage: camk1_train.py {1,2} [--max-hours H]
"""
import argparse
import json
import os
import pathlib
import shutil
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import gr_run as R                                          # noqa: E402  (host, gpu, tree helpers)

PREREG = "docs/plans/prereg_camonly_k1_train_v1_2026-11-06.md"
SCOPE = ("CAM-K1 v1: camera-only (no_lidar) continuation of Frozen Policy F ep25000 for 3,000 epochs (25001..28000), "
         "fixed 205 bars, LR 5e-6, camera FOV 66x54 deg, yaw_align_weight 0.6, visibility_bonus 0.03; a 2-epoch "
         "development smoke (seed 1, not a result) first, then seed 1 then seed 2, each once")
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camk1_20261106"
RL = ROOT / "aerial_gym" / "rl_training" / "rl_games"
LAUNCHER = RL / "train_navrl_camk1.sh"
F_REL = "runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth"
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
TAG = "v2-camk1-ep25000-205bars-main-riskcap-camonly-s%d"
FINAL_EPOCH = 28000
SMOKE_EPOCH = 25002
FOV_LOG_LINE = "hfov=66deg"            # navrl_task.py prints the built camera's FOV at task construction
MIN_FREE_GB = 2.0
ENV_STRIP = ("NAVRL_", "CAMK1_", "CAMTRAIN_", "S3B_")   # the launcher sets every one it needs
EXPECTED_AFTER = {"detector_hfov_deg": 66.0, "detector_vfov_deg": 54.0, "yaw_align_weight": 0.6,
                  "visibility_bonus": 0.03}


def approval_problems(approval, head, scope=SCOPE):
    p = []
    if not isinstance(approval, dict):
        return ["approval is not a JSON object"]
    for k in APPROVAL_KEYS:
        if not isinstance(approval.get(k), str) or not approval[k].strip():
            p.append("approval key %s missing or empty" % k)
    if approval.get("scope") != scope:
        p.append("approval scope differs from the code's SCOPE")
    if not head or approval.get("commit") not in (head, head[:7]):
        p.append("approval commit %r is not HEAD %s" % (approval.get("commit"), head))
    return p


def receipt_problems(receipt):
    if not isinstance(receipt, dict):
        return ["camtrain receipt missing or unreadable"]
    p = []
    if (receipt.get("ablation") or {}).get("condition") != "no_lidar":
        p.append("ablation condition is not no_lidar: %r" % (receipt.get("ablation") or {}).get("condition"))
    after = receipt.get("overrides_after") or {}
    for k, v in EXPECTED_AFTER.items():
        if after.get(k) != v:
            p.append("override %s is %r, preregistered %r" % (k, after.get(k), v))
    return p


def smoke_ok(smoke_dir):
    """Seeds 1 and 2 need the development smoke to have read SMOKE_OK at this same HEAD."""
    f = pathlib.Path(smoke_dir) / "train_report.json"
    if not f.is_file():
        return False
    r = json.loads(f.read_text())
    head = R.K.git("rev-parse", "HEAD", root=ROOT).strip()
    return r.get("verdict") == "SMOKE_OK" and r.get("development_smoke") is True and r.get("commit") == head


def sim_processes():
    out = subprocess.run(["pgrep", "-a", "-f", "gzserver|px4_sitl_default/bin/px4"], capture_output=True,
                         text=True).stdout
    return [ln for ln in out.splitlines() if ln.strip() and "pgrep" not in ln]


def preflight(seed, out):
    head, problems = R.tree_problems(ROOT, R.K.git("rev-parse", "HEAD", root=ROOT).strip())
    if R.K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != "camonly/dev":
        problems.append("branch is not camonly/dev")
    ap = UNIT_DIR / "execution_approval.json"
    if not ap.is_file():
        problems.append("no approval file %s" % ap)
    else:
        problems += approval_problems(json.loads(ap.read_text()), head)
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG], capture_output=True).returncode:
        problems.append("preregistration %s is not committed" % PREREG)
    if R.K.gpu_names() != [R.GPU]:
        problems.append("expected exactly one %s" % R.GPU)
    if R.gpu_apps():
        problems.append("other GPU compute processes: %s" % R.gpu_apps())
    if subprocess.run(["pgrep", "-f", "runner.py .*--task navrl_task"], capture_output=True).stdout.strip():
        problems.append("a NavRL runner process is already active")
    if sim_processes():
        problems.append("a PX4 SITL or Gazebo process is running: %s" % sim_processes()[:2])
    if R.sha(RL / F_REL) != F_SHA:
        problems.append("Policy F checkpoint SHA mismatch")
    if out.exists():
        problems.append("output exists (single use): %s" % out)
    free_gb = shutil.disk_usage(str(ROOT)).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("only %.1f GB free on disk (need %.1f)" % (free_gb, MIN_FREE_GB))
    return head, problems


def clean_env():
    """The caller's environment without any NAVRL_/CAMK1_/CAMTRAIN_/S3B_ variable (review r1 K-10)."""
    return {k: v for k, v in os.environ.items() if not k.startswith(ENV_STRIP)}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("seed", type=int, choices=(1, 2))
    ap.add_argument("--max-hours", type=float, default=5.0)
    ap.add_argument("--smoke", action="store_true", help="the 2-epoch development smoke (seed 1 only; not a result)")
    args = ap.parse_args(argv)
    if args.smoke and args.seed != 1:
        raise SystemExit("the smoke runs seed 1 only")
    out = UNIT_DIR / ("train_smoke" if args.smoke else "train_s%d" % args.seed)
    tag = TAG % args.seed + ("-smoke" if args.smoke else "")
    final_epoch = SMOKE_EPOCH if args.smoke else FINAL_EPOCH
    if not args.smoke and not smoke_ok(UNIT_DIR / "train_smoke"):
        print(json.dumps({"seed": args.seed, "verdict": "PREFLIGHT_FAIL",
                          "preflight_problems": ["the development smoke did not read SMOKE_OK"]}, indent=1))
        return 2
    head, problems = preflight(args.seed, out)
    start = R.host()
    report = {"seed": args.seed, "commit": head, "prereg": PREREG, "scope": SCOPE, "host_start": start,
              "preflight_problems": problems, "started": R.now()}
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out.mkdir(parents=True)
    receipt_path = out / "camtrain_receipt.json"
    before = {p.name for p in (RL / "runs").iterdir()}
    t0 = time.time()
    env = dict(clean_env(), NAVRL_CAMTRAIN_RECEIPT=str(receipt_path), CUDA_VISIBLE_DEVICES="0",
               CAMK1_SMOKE="1" if args.smoke else "0")
    with (out / "launcher.log").open("w") as fh:
        rc = subprocess.run(["timeout", "-k", "120", str(int(args.max_hours * 3600)), "bash", str(LAUNCHER),
                             str(args.seed)], stdout=fh, stderr=subprocess.STDOUT, cwd=str(RL), env=env).returncode
    report["exit"], report["wall_s"] = rc, time.time() - t0
    end = R.host()
    report["host_end"] = end
    host_ok = all(end[k] == start[k] for k in ("fault_count", "boot_id", "online"))
    new = sorted(p for p in (RL / "runs").iterdir() if p.name not in before)
    report["new_run_dirs"] = [p.name for p in new]
    problems = []
    if not host_ok:
        problems.append("HOST state changed during training")
    if len(new) != 1 or not new[0].name.endswith(tag):
        problems.append("expected exactly one new run folder ending in %s" % tag)
    ckpts = sorted((new[0] / "nn").glob("last_gen_ppo_ep_%d_*.pth" % final_epoch)) if len(new) == 1 else []
    if len(ckpts) != 1:                                     # the smoke too: the final save is what seeds rely on
        problems.append("missing last_gen_ppo_ep_%d_*.pth" % final_epoch)
    if rc != 0:
        problems.append("launcher exit %d" % rc)
    receipt = json.loads(receipt_path.read_text()) if receipt_path.is_file() else None
    report["camtrain_receipt"] = receipt
    problems += receipt_problems(receipt)
    if ckpts:
        dst = out / "checkpoint" / ckpts[0].name
        dst.parent.mkdir()
        shutil.copy2(ckpts[0], dst)
        report["checkpoint"] = {"run_dir": str(new[0]), "file": ckpts[0].name, "sha256": R.sha(ckpts[0]),
                                "copy": str(dst), "copy_sha256": R.sha(dst)}
        logs = sorted((RL / "train_session_logs").glob("camk1_s%d%s_*.log" % (args.seed, "-smoke" if args.smoke else "")))
        if logs:
            shutil.copy2(logs[-1], out / logs[-1].name)
            report["session_log"] = {"file": logs[-1].name, "sha256": R.sha(logs[-1])}
    texts = [(out / "launcher.log").read_text(errors="replace") if (out / "launcher.log").is_file() else ""]
    if report.get("session_log"):
        texts.append((out / report["session_log"]["file"]).read_text(errors="replace"))
    report["fov_log_line_found"] = any(FOV_LOG_LINE in x for x in texts)
    if not report["fov_log_line_found"]:
        problems.append("the task's startup line %r is in neither log: the FOV override is not shown to have reached "
                        "the task" % FOV_LOG_LINE)
    if R.K.git("status", "--porcelain", "--untracked-files=no", root=ROOT).strip():
        problems.append("tree became dirty")
    report["problems"] = problems
    ok_label = "SMOKE_OK" if args.smoke else "TRAINED"
    report["development_smoke"] = bool(args.smoke)
    report["verdict"] = "VOID_HOST_FAULT" if not host_ok else (ok_label if not problems else "TRAINING_FAIL")
    report["finished"] = R.now()
    (out / "train_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report.get(k) for k in ("seed", "verdict", "exit", "wall_s", "checkpoint", "problems")},
                     indent=1, default=str))
    return 0 if report["verdict"] == ok_label else 1


if __name__ == "__main__":
    raise SystemExit(main())
