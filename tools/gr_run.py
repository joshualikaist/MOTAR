"""GPU runner for the control-gain-randomization unit (D-2026-11-04-1; docs/plans/prereg_gain_randomization_gr0_v1.md).

Stages (each writes a fresh output folder and never overwrites):
  identity  one cell (E2_obstacle_aware, 160 bars, seed 4501, quota 2) four times:
            ref  the base tree src/MOTAR-mt-p1 at 62e59c0, OFF        (native run before this unit's change)
            off  this tree, OFF                                       -> must equal ref
            s00  this tree, NAVRL_GAIN_RAND=1, scale 0.0              -> must equal off (ON path, nominal gains)
            s40  this tree, NAVRL_GAIN_RAND=1, scale 0.4              -> must differ from off (positive control)
  gr0       Policy F, E2_obstacle_aware, 70/115/160/205 bars x seeds 4501-4503 x gain levels S00 (OFF), S20, S40,
            quota 16 (2,048 episodes per cell); requires an IDENTITY_PASS report on the same commit.
Every cell runs the KROC2027 launcher byte-for-byte through tools/kroc2027_eval.py helpers (scrubbed environment,
PYTHONPATH = the tree, quota ledger, motion recorder, NAVRL_TARGET_DISTANCE_BACKEND=direct_l2_v1). Fail-closed:
approved commit = HEAD, clean tracked tree, pinned launcher / player / checkpoint, one RTX 3070, import origins inside
the tree; after every cell the kernel fault record, boot id and online CPU list must be unchanged (else
VOID_HOST_FAULT and stop), the receipt must carry this commit and 0 dirty files, the recorded NAVRL_GAIN_RAND* and
condition cfg_gain_rand* must match the level (absent when OFF), the ledger must be complete and the recorder clean.
usage: gr_run.py {identity,gr0} --approved-commit SHA --out DIR [--max-hours H]
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import kroc2027_eval as K                                  # noqa: E402

PREREG = "docs/plans/prereg_gain_randomization_gr0_v1.md"
AMENDMENT = "docs/plans/prereg_gain_randomization_gr0_v1_amendment1_2026-11-04.md"
GR1_PREREG = "docs/plans/prereg_gain_randomization_gr1_v1.md"
GR1_LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_gr1_arms.sh"
GR1_LAUNCHER_SHA = "418433eb0f023f5b0f6db1a5d20d2f3d83cff039893654edadd3f66953d0c2cd"   # amendment 1 (review L3): the copy is pinned
GR1_AMENDMENT = "docs/plans/prereg_gain_randomization_gr1_v1_amendment1_2026-11-04.md"
F_CKPT_REL = ("aerial_gym/rl_training/rl_games/runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/"
              "last_gen_ppo_ep_25000_rew_39.742134.pth")
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
REF_TREE = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
REF_COMMIT = "62e59c031ad2"
ARM = "E2_obstacle_aware"
BACKEND = {"NAVRL_TARGET_DISTANCE_BACKEND": "direct_l2_v1"}
GPU = "NVIDIA GeForce RTX 3070"
ALLOWED_GPU_APPS = ("rustdesk",)
KST = datetime.timezone(datetime.timedelta(hours=9))
LEVELS = {"S00": None, "S20": 0.2, "S40": 0.4}
DENSITIES = (70, 115, 160, 205)
SEEDS = (4501, 4502, 4503)
CELL_TIMEOUT_S = 1800


def now():
    return datetime.datetime.now(KST).isoformat()


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def host():
    j = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, text=True).stdout
    faults = sum(1 for line in j.splitlines() if any(s in line for s in
                 ("Machine check", "segfault", "general protection", "mce: [Hardware Error]")))
    return {"fault_count": faults,
            "boot_id": pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            "online": pathlib.Path("/sys/devices/system/cpu/online").read_text().strip(),
            "time": now()}


def gpu_apps():
    done = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader"],
                          capture_output=True, text=True, timeout=30)
    return [line.strip() for line in done.stdout.splitlines() if line.strip()
            and not any(a in line for a in ALLOWED_GPU_APPS)]


def gain_env(scale, seed):
    if scale is None:
        return {}
    return {"NAVRL_GAIN_RAND": "1", "NAVRL_GAIN_RAND_SCALE": repr(float(scale)), "NAVRL_GAIN_RAND_SEED": str(int(seed))}


def tree_problems(root, commit_prefix):
    problems = []
    head = K.git("rev-parse", "HEAD", root=root).strip()
    if not head.startswith(commit_prefix):
        problems.append("%s HEAD %s is not %s" % (root, head[:12], commit_prefix))
    if K.git("status", "--porcelain", root=root).strip():
        problems.append("%s has uncommitted or untracked files" % root)
    return head, problems


def cell_env(root, protocol, density, seed, out_dir, quota, run_id, scale, extra=None):
    inherited = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")}
    # Amendment 1: the KROC2027 A3 placement (moving target, initial view visible at 10 m, spawn clearance and
    # line-of-sight rules), i.e. the B1-R Policy F condition (tools/b1r_run.py view='visible', arm='A3').
    env = K.cell_env(K.base_env(root=root, inherited=inherited), protocol, target_model=ARM, density=density,
                     seed=seed, out_dir=out_dir, quota=quota, view="visible", recorder=True, run_id=run_id,
                     arm="A3")
    env.update(BACKEND)
    env.update(gain_env(scale, seed))
    env.update(extra or {})
    return env


def check_cell(cell_dir, root, head, quota, num_envs, scale, seed, start_host, ckpt_sha=None, cond_fields=True):
    problems = []
    rec_host = host()
    for k in ("fault_count", "boot_id", "online"):
        if rec_host[k] != start_host[k]:
            problems.append("HOST " + k + " changed")
    try:
        receipt = json.loads((cell_dir / "receipt.json").read_text())
        result = json.loads((cell_dir / "result.json").read_text())
        summary = json.loads((cell_dir / "episode_ledger_summary.json").read_text())
        rows = K.read_jsonl(cell_dir / "episodes.jsonl")
        rec_summary = json.loads((cell_dir / K.RECORDER_SUMMARY).read_text())
    except (OSError, ValueError) as e:
        return problems + ["missing or unreadable output: %s" % e], rec_host, None
    if receipt.get("status") != "OK" or receipt.get("git_commit") != head or receipt.get("git_dirty_files") != 0:
        problems.append("receipt status/commit/dirty mismatch")
    if ckpt_sha is not None and receipt.get("checkpoint_sha256") != ckpt_sha:
        problems.append("receipt checkpoint %s is not %s" % (receipt.get("checkpoint_sha256"), ckpt_sha))
    snap = receipt.get("env_snapshot_navrl", {})
    want = gain_env(scale, seed)
    got = {k: v for k, v in snap.items() if k.startswith("NAVRL_GAIN_RAND")}
    if got != want:
        problems.append("recorded NAVRL_GAIN_RAND* %s != %s" % (got, want))
    cond = result.get("condition", {})
    cond_gain = {k: v for k, v in cond.items() if k.startswith("cfg_gain_rand")}
    # GR-0's commit also recorded the switch in result.json's condition; from GR-1 on navrl_task.py is back to the
    # base bytes (the H-R3 source pins), so the receipt's environment snapshot is the only record and the condition
    # must carry no cfg_gain_rand* field.
    want_cond = {} if (scale is None or not cond_fields) else {"cfg_gain_rand": True, "cfg_gain_rand_scale": float(scale),
                                                              "cfg_gain_rand_seed": int(seed)}
    if cond_gain != want_cond:
        problems.append("condition cfg_gain_rand* %s != %s" % (cond_gain, want_cond))
    if snap.get("NAVRL_TARGET_DISTANCE_BACKEND") != BACKEND["NAVRL_TARGET_DISTANCE_BACKEND"]:
        problems.append("target distance backend not direct_l2_v1")
    problems.extend(K.ledger_problems(summary, rows, num_envs, quota))
    origins = rec_summary.get("import_origins", {})
    for mod in ("aerial_gym", "motar_eval"):
        if not origins.get(mod) or not K.is_inside(origins[mod], root):
            problems.append("import origin of %s outside the tree: %s" % (mod, origins.get(mod)))
    viol = cell_dir / "kroc2027_violations.jsonl"
    if viol.exists() and viol.read_text().strip():
        problems.append("recorder violations present")
    if K.git("status", "--porcelain", root=root).strip():
        problems.append("tree became dirty")
    quota_rows = [r for r in rows if r.get("in_quota")]
    counts = {o: sum(1 for r in quota_rows if r.get("outcome") == o) for o in ("capture", "crash", "timeout")}
    return problems, rec_host, {"quota_episodes": len(quota_rows), "counts": counts,
                                "duration_s": receipt.get("duration_s"), "result_sha256": receipt.get("result_sha256")}


def run_launcher(root, protocol, env, timeout_s, launcher=None):
    """K.run_launcher, but for the launcher of ``root`` (the identity reference runs the base tree's launcher);
    ``launcher`` (relative to root) selects the GR-1 copy for a GR-1 checkpoint."""
    import signal

    path = root / launcher if launcher else K.launcher_path(protocol, root)
    proc = subprocess.Popen(["bash", str(path)], env=env, start_new_session=True)
    try:
        return proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        return K.BUDGET_EXIT


def run_cell(root, protocol, out_root, level, density, seed, quota, run_id, scale, head, start_host, records,
             timeout_s, extra=None, launcher=None, ckpt_sha=None):
    out_dir = out_root / level
    out_dir.mkdir(parents=True, exist_ok=True)
    cell_id = "%s__%dbars__seed%d" % (ARM, density, seed)
    env = cell_env(root, protocol, density, seed, out_dir, quota, run_id, scale, extra)
    print("==", level, cell_id, now(), flush=True)
    t0 = time.time()
    rc = run_launcher(root, protocol, env, timeout_s, launcher)
    problems, h, stats = check_cell(out_dir / cell_id, root, head, quota, protocol["evaluation"]["num_envs"], scale,
                                    seed, start_host, ckpt_sha, cond_fields=not run_id.startswith("GR1"))
    if rc != 0:
        problems.insert(0, "launcher exit %d" % rc)
    rec = {"level": level, "cell_id": cell_id, "tree": str(root), "commit": head, "scale": scale, "seed": seed,
           "density": density, "quota": quota, "exit": rc, "wall_s": time.time() - t0, "host_after": h,
           "stats": stats, "problems": problems,
           "verdict": "VOID_HOST_FAULT" if any(p.startswith("HOST") for p in problems) else
           ("OK" if not problems else "FAIL")}
    records.append(rec)
    print("  ", rec["verdict"], stats and stats["counts"], "%.0fs" % rec["wall_s"], problems[:3], flush=True)
    return rec


def identity_compare(a_dir, b_dir, drop_gain=False):
    """Equal outcomes: result.json minus the per-run nonce, the absolute checkpoint path and (optionally) the
    cfg_gain_rand* condition fields, plus the per-episode ledger rows."""
    def load(d):
        r = K.comparable(json.loads((d / "result.json").read_text()))
        r.pop("checkpoint", None)
        if drop_gain:
            r["condition"] = {k: v for k, v in r["condition"].items() if not k.startswith("cfg_gain_rand")}
        return r, K.read_jsonl(d / "episodes.jsonl")
    ra, ea = load(a_dir)
    rb, eb = load(b_dir)
    return {"result_equal": ra == rb, "episodes_equal": ea == eb,
            "outcome_a": ra.get("outcome"), "outcome_b": rb.get("outcome")}


def preflight(root, protocol, approved, out_root, gr1=False):
    head, problems = tree_problems(root, approved)
    for rel in ((GR1_PREREG, GR1_AMENDMENT) if gr1 else ()):
        if subprocess.run(["git", "-C", str(root), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("preregistration %s is not committed" % rel)
    if gr1 and (not (root / GR1_LAUNCHER).is_file() or sha(root / GR1_LAUNCHER) != GR1_LAUNCHER_SHA):
        problems.append("GR-1 launcher copy missing or not the pinned SHA %s" % GR1_LAUNCHER_SHA[:12])
    if K.git("rev-parse", "--abbrev-ref", "HEAD", root=root).strip() != "gainrand/dev":
        problems.append("branch is not gainrand/dev")
    for rel in (PREREG, AMENDMENT):
        if subprocess.run(["git", "-C", str(root), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("preregistration %s is not committed" % rel)
    problems += K.static_problems(protocol, root)
    if K.gpu_names() != [GPU]:
        problems.append("expected exactly one %s, found %s" % (GPU, K.gpu_names()))
    if gpu_apps():
        problems.append("other GPU compute processes: %s" % gpu_apps())
    launcher_dir = K.launcher_path(protocol, root).parent
    imp, origins = K.import_problems(K.base_env(root=root, inherited={k: v for k, v in os.environ.items()
                                                                       if not k.startswith("NAVRL_")}),
                                     launcher_dir, root)
    problems += imp
    if out_root.exists():
        problems.append("output already exists: %s" % out_root)
    return head, problems, origins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("identity", "gr0", "gr1repro", "gr1eval"))
    ap.add_argument("--train-report", default=None, help="gr1eval: train_<arm>/train_report.json (amendment 1, H1)")
    ap.add_argument("--gr0-dir", default=None)
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=None)
    args = ap.parse_args()
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = preflight(ROOT, protocol, args.approved_commit, out_root,
                                        gr1=args.stage.startswith("gr1"))
    start = host()
    if args.stage.startswith("gr1") and args.max_hours is None:
        args.max_hours = 3.0                                    # amendment 1 (review L3): default cap
    records, report = [], {"stage": args.stage, "tree": str(ROOT), "commit": head,
                           "prereg": GR1_PREREG if args.stage.startswith("gr1") else PREREG,
                           "started": now(), "host_start": start, "import_origins": origins,
                           "preflight_problems": problems}
    if args.stage == "identity":
        ref_head, ref_problems = tree_problems(REF_TREE, REF_COMMIT)
        ref_problems += K.static_problems(protocol, REF_TREE)
        problems += ref_problems
        report["ref_tree"], report["ref_commit"] = str(REF_TREE), ref_head
    if not problems and args.stage == "gr0":
        idr = None
        for cand in sorted(out_root.parent.glob("*/identity_report.json")):
            r = json.loads(cand.read_text())
            if r.get("verdict") == "IDENTITY_PASS" and r.get("commit") == head:
                idr = cand
        if idr is None:
            problems.append("gr0 needs an IDENTITY_PASS identity_report.json on this commit in %s" % out_root.parent)
        report["identity_report"] = str(idr)
    if args.stage == "gr1repro" and not (args.gr0_dir and (pathlib.Path(args.gr0_dir) / "gr0_run_report.json").is_file()):
        problems.append("gr1repro needs --gr0-dir with gr0_run_report.json")
    if args.stage == "gr1eval":
        # amendment 1 (review H1): arm label, checkpoint and SHA come from the training report, never the caller
        tr = json.loads(pathlib.Path(args.train_report).read_text()) if args.train_report and \
            pathlib.Path(args.train_report).is_file() else {}
        ck = tr.get("checkpoint") or {}
        args.policy, args.ckpt, args.ckpt_sha = tr.get("arm"), ck.get("copy"), ck.get("copy_sha256")
        if tr.get("verdict") != "TRAINED" or args.policy not in ("rand40", "ctrl") or ck.get("sha256") != args.ckpt_sha:
            problems.append("gr1eval needs --train-report of a TRAINED arm with a matching checkpoint copy")
        elif not pathlib.Path(args.ckpt).is_file() or sha(args.ckpt) != args.ckpt_sha:
            problems.append("checkpoint missing or SHA mismatch: %s" % args.ckpt)
        report["train_report"] = str(pathlib.Path(args.train_report).resolve()) if args.train_report else None
        if not (ROOT / GR1_LAUNCHER).is_file():
            problems.append("missing %s" % GR1_LAUNCHER)
        rp = None
        for cand in sorted(out_root.parent.glob("*/gr1repro_report.json")):
            r = json.loads(cand.read_text())
            if r.get("verdict") == "GR1_REPRO_PASS" and r.get("commit") == head:
                rp = cand
        if rp is None:
            problems.append("gr1eval needs a GR1_REPRO_PASS gr1repro_report.json on this commit in %s" % out_root.parent)
        report["repro_report"] = str(rp)
        report.update(policy=args.policy, checkpoint=args.ckpt, checkpoint_sha256=args.ckpt_sha)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    run_id = ("GR1-" + (args.policy or "F").upper()) if args.stage.startswith("gr1") else "GR0-" + args.stage.upper()
    void = False
    if args.stage == "identity":
        plan = [("ref", REF_TREE, ref_head, None), ("off", ROOT, head, None), ("s00", ROOT, head, 0.0),
                ("s40", ROOT, head, 0.4)]
        for level, tree, commit, scale in plan:
            rec = run_cell(tree, protocol, out_root, level, 160, 4501, 2, run_id, scale, commit, start, records,
                           CELL_TIMEOUT_S)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
        cid = "%s__160bars__seed4501" % ARM
        comp = {}
        if not void and all(r["verdict"] == "OK" for r in records) and len(records) == 4:
            comp = {"off_vs_ref": identity_compare(out_root / "ref" / cid, out_root / "off" / cid),
                    "s00_vs_off": identity_compare(out_root / "off" / cid, out_root / "s00" / cid, drop_gain=True),
                    "s40_vs_off": identity_compare(out_root / "off" / cid, out_root / "s40" / cid, drop_gain=True)}
        ok = bool(comp) and all(comp[k]["result_equal"] and comp[k]["episodes_equal"]
                                for k in ("off_vs_ref", "s00_vs_off")) \
            and not comp["s40_vs_off"]["episodes_equal"]
        report["comparisons"] = comp
        report["verdict"] = "VOID_HOST_FAULT" if void else ("IDENTITY_PASS" if ok else "IDENTITY_FAIL")
        name = "identity_report.json"
    elif args.stage == "gr1repro":
        # Policy F, the S40 / 205 bars / seed 4501 GR-0 cell, rerun on this commit twice: through the pinned launcher
        # and (amendment 1, review H2b) through the GR-1 launcher copy with GR1_CKPT = Policy F. Both must equal GR-0.
        f_extra = {"GR1_CKPT": str(ROOT / F_CKPT_REL), "GR1_CKPT_SHA": F_SHA}
        plan = [("S40", None, None), ("S40_gr1launcher", GR1_LAUNCHER, f_extra)]
        cid = "%s__205bars__seed4501" % ARM
        comp, void = {}, False
        for level, launcher, extra in plan:
            rec = run_cell(ROOT, protocol, out_root, level, 205, 4501, 16, run_id, 0.4, head, start, records,
                           CELL_TIMEOUT_S, extra, launcher, F_SHA if launcher else None)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if rec["verdict"] == "OK":
                comp[level] = identity_compare(pathlib.Path(args.gr0_dir) / "S40" / cid, out_root / level / cid,
                                               drop_gain=True)
        report["gr0_dir"] = str(pathlib.Path(args.gr0_dir).resolve())
        report["comparison"] = comp
        ok = len(comp) == 2 and all(c["result_equal"] and c["episodes_equal"] for c in comp.values())
        report["verdict"] = "VOID_HOST_FAULT" if void else ("GR1_REPRO_PASS" if ok else "GR1_REPRO_FAIL")
        name = "gr1repro_report.json"
    else:
        extra = {"GR1_CKPT": str(pathlib.Path(args.ckpt).resolve()), "GR1_CKPT_SHA": args.ckpt_sha} \
            if args.stage == "gr1eval" else None
        launcher = GR1_LAUNCHER if args.stage == "gr1eval" else None
        ckpt_sha = args.ckpt_sha if args.stage == "gr1eval" else None
        for density in DENSITIES:
            for seed in SEEDS:
                for level, scale in LEVELS.items():
                    if budget.exhausted():
                        report["budget_exhausted"] = True
                        break
                    rec = run_cell(ROOT, protocol, out_root, level, density, seed, 16, run_id, scale, head, start,
                                   records, CELL_TIMEOUT_S, extra, launcher, ckpt_sha)
                    if rec["verdict"] == "VOID_HOST_FAULT":
                        void = True
                        break
                    if rec["verdict"] != "OK":
                        break
                if void or (records and records[-1]["verdict"] != "OK") or report.get("budget_exhausted"):
                    break
            if void or (records and records[-1]["verdict"] != "OK") or report.get("budget_exhausted"):
                break
        expected = len(DENSITIES) * len(SEEDS) * len(LEVELS)
        complete = len(records) == expected and all(r["verdict"] == "OK" for r in records)
        report["verdict"] = "VOID_HOST_FAULT" if void else ("COMPLETE" if complete else "INCOMPLETE")
        report["cells_expected"] = expected
        name = "gr0_run_report.json" if args.stage == "gr0" else "gr1eval_run_report.json"
    report["records"] = records
    report["host_end"] = host()
    report["finished"] = now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("IDENTITY_PASS", "COMPLETE", "GR1_REPRO_PASS") else 1


if __name__ == "__main__":
    raise SystemExit(main())
