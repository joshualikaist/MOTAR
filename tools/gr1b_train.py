"""GR-1b training wrapper (docs/plans/prereg_gain_randomization_gr1b_v1.md; a copy of gr1_train.py with the seed-2
launcher, the -s2 run tag and the gr1b_ session-log prefix): one arm of the 1,000-epoch continuation of
Policy F, with (rand40) or without (ctrl) controller-gain randomization, via train_navrl_gr1b.sh.

Fail-closed preflight: branch gainrand/dev, approved commit = HEAD, clean tracked tree, preregistration committed,
exactly one RTX 3070 and no other GPU compute or NavRL training process, Policy F SHA. During the run: wall-clock cap
(timeout -k). Afterwards: kernel fault record, boot id and online CPU list unchanged (else VOID_HOST_FAULT); exactly
one new run folder whose tag matches the arm; its last_gen_ppo_ep_26000_*.pth exists. The checkpoint is copied into
the output folder (raw results never live only in the worktree's runs/), with its SHA-256 and the session log.
usage: gr1b_train.py {rand40,ctrl} --approved-commit SHA --out DIR [--max-hours H]
"""
import argparse
import hashlib
import json
import pathlib
import shutil
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import gr_run as R                                          # noqa: E402  (host, gpu, tree helpers)

PREREG = "docs/plans/prereg_gain_randomization_gr1b_v1.md"
RL = ROOT / "aerial_gym" / "rl_training" / "rl_games"
LAUNCHER = RL / "train_navrl_gr1b.sh"
F_REL = "runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth"
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
TAG = "v2-gr1-ep25000-205bars-main-riskcap-%s-s2"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("arm", choices=("rand40", "ctrl"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=3.0)
    args = ap.parse_args()
    out = pathlib.Path(args.out).resolve()
    head, problems = R.tree_problems(ROOT, args.approved_commit)
    if R.K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != "gainrand/dev":
        problems.append("branch is not gainrand/dev")
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG], capture_output=True).returncode:
        problems.append("preregistration %s is not committed" % PREREG)
    if R.K.gpu_names() != [R.GPU]:
        problems.append("expected exactly one %s" % R.GPU)
    if R.gpu_apps():
        problems.append("other GPU compute processes: %s" % R.gpu_apps())
    if subprocess.run(["pgrep", "-f", "runner.py .*--task navrl_task"], capture_output=True).stdout.strip():
        problems.append("a NavRL runner process is already active")
    if R.sha(RL / F_REL) != F_SHA:
        problems.append("Policy F checkpoint SHA mismatch")
    if out.exists():
        problems.append("output exists: %s" % out)
    start = R.host()
    report = {"arm": args.arm, "commit": head, "prereg": PREREG, "host_start": start, "preflight_problems": problems,
              "started": R.now()}
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1))
        return 2
    out.mkdir(parents=True)
    before = {p.name for p in (RL / "runs").iterdir()}
    t0 = time.time()
    log = out / "launcher.log"
    with log.open("w") as fh:
        rc = subprocess.run(["timeout", "-k", "120", str(int(args.max_hours * 3600)), "bash", str(LAUNCHER), args.arm],
                            stdout=fh, stderr=subprocess.STDOUT, cwd=str(RL)).returncode
    report["exit"], report["wall_s"] = rc, time.time() - t0
    end = R.host()
    report["host_end"] = end
    host_ok = all(end[k] == start[k] for k in ("fault_count", "boot_id", "online"))
    new = sorted(p for p in (RL / "runs").iterdir() if p.name not in before)
    report["new_run_dirs"] = [p.name for p in new]
    problems = []
    if not host_ok:
        problems.append("HOST state changed during training")
    if len(new) != 1 or not new[0].name.endswith(TAG % args.arm):
        problems.append("expected exactly one new run folder ending in %s" % (TAG % args.arm))
    ckpts = sorted((new[0] / "nn").glob("last_gen_ppo_ep_26000_*.pth")) if len(new) == 1 else []
    if len(ckpts) != 1:
        problems.append("missing last_gen_ppo_ep_26000_*.pth")
    if rc != 0:
        problems.append("launcher exit %d" % rc)
    if ckpts:
        dst = out / "checkpoint" / ckpts[0].name
        dst.parent.mkdir()
        shutil.copy2(ckpts[0], dst)
        report["checkpoint"] = {"run_dir": str(new[0]), "file": ckpts[0].name, "sha256": R.sha(ckpts[0]),
                                "copy": str(dst), "copy_sha256": R.sha(dst)}
        logs = sorted((RL / "train_session_logs").glob("gr1b_%s_*.log" % args.arm))
        if logs:
            shutil.copy2(logs[-1], out / logs[-1].name)
            report["session_log"] = {"file": logs[-1].name, "sha256": R.sha(logs[-1])}
    if R.K.git("status", "--porcelain", root=ROOT).strip():
        problems.append("tree became dirty")
    report["problems"] = problems
    report["verdict"] = "VOID_HOST_FAULT" if not host_ok else ("TRAINED" if not problems else "TRAINING_FAIL")
    report["finished"] = R.now()
    (out / "train_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report.get(k) for k in ("arm", "verdict", "exit", "wall_s", "checkpoint", "problems")},
                     indent=1, default=str))
    return 0 if report["verdict"] == "TRAINED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
