"""Runner of the camera-only study's GPU stages C1 and C2 (docs/plans/prereg_camonly_c1c2_v1.md).

Reuses tools/gr_run.py's cell environment (GR-0's grid condition: E2, A3 placement, quota ledger, motion recorder,
direct_l2_v1 distance backend, gains OFF = level S00), its cell checks and its launcher call, with the camera-only
launcher copy aerial_gym/rl_training/rl_games/eval_navrl_camonly_arms.sh (SHA-pinned). Every cell additionally
must carry an ablation receipt written by tools/camonly_entry.py that names the cell's condition and the committed
module's SHA-256, and its receipt's NAVRL snapshot must name the same condition; otherwise the cell FAILs (the
switch is silently ignored without the entry point; review r1 L5 / r2 L-r2-2).

  c1  identity (positive and negative control): Policy F, S00, 205 bars, seed 4501, through the camera-only launcher
      with NAVRL_SENSOR_ABLATION=full must equal GR-0's S00 cell exactly; the same cell with no_lidar must differ.
  c2  the five conditions on GR-0's S00 grid (4 densities x 3 seeds, quota 16), after a C1_IDENTITY_PASS on this
      commit in the same parent folder; order: density, seed, condition.
Fail-closed preflight: approved commit = HEAD, clean tree, branch camonly/dev, the preregistration committed, launcher
copy and ablation module at their pinned / committed bytes, Policy F checkpoint SHA, one RTX 3070, no other GPU
compute process, import origins inside the tree, output folder new. Host faults void.
Amendment 1 (docs/plans/prereg_camonly_c1c2_v1_amendment1_2026-11-04.md, after the design review AMEND_FIRST): the
reports record the allowlisted runner environment (AERIAL_*, PYTHON*, PATH, LD_*, CUDA*, NVIDIA*, OMP_*, MKL_*,
TORCH*, CONDA*, LANG, LC_*, HOME, USER, DISPLAY, VK_*, __GL*), and c2 refuses to start unless it equals C1's (M3);
the default cap is 3.5 h (L5).
usage: tools/camonly_run.py {c1,c2} --approved-commit SHA --gr0-dir GR0_MAIN --out DIR [--max-hours H]
"""
import argparse
import json
import os
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import gr_run as G                                          # noqa: E402
import kroc2027_eval as K                                   # noqa: E402

PREREG = "docs/plans/prereg_camonly_c1c2_v1.md"
AMENDMENT = "docs/plans/prereg_camonly_c1c2_v1_amendment1_2026-11-04.md"
ENV_ALLOW = re.compile(r"^(AERIAL_.*|PYTHON.*|PATH|LD_.*|CUDA.*|NVIDIA.*|OMP_.*|MKL_.*|TORCH.*|CONDA.*|LANG|LC_.*|HOME|"
                       r"USER|DISPLAY|VK_.*|__GL.*)$")
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_camonly_arms.sh"
LAUNCHER_SHA = "1eadf8fbb042c7ebe70ab29f2b1adfee7d057c75a66eaf892edf52e0b4c0ff7f"
MODULE = "aerial_gym/task/navrl_task/navrl_sensor_ablation.py"
WRAPPER = ROOT / "tools/camonly_python"
BRANCH = "camonly/dev"
C2_CONDITIONS = ("no_lidar_in_fov", "no_lidar_out_fov", "no_lidar", "no_lidar_obs_only", "no_lidar_in_fov_obs_only")
C1_PLAN = (("full", 205, 4501), ("no_lidar", 205, 4501))
QUOTA = 16


def ablation_problems(cell_dir, condition, module_sha):
    problems = []
    try:
        rec = json.loads((cell_dir / "sensor_ablation_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["ablation receipt missing or unreadable: %s" % e]
    if rec.get("condition") != condition:
        problems.append("ablation receipt condition %r != %r" % (rec.get("condition"), condition))
    if rec.get("module_sha256") != module_sha:
        problems.append("ablation module SHA %s != committed %s" % (rec.get("module_sha256"), module_sha))
    if not K.is_inside(rec.get("module", ""), ROOT) or not K.is_inside(rec.get("perception", ""), ROOT) \
            or not K.is_inside(rec.get("task", ""), ROOT):
        problems.append("ablation receipt paths outside the tree: %s" % {k: rec.get(k) for k in ("module", "task")})
    if rec.get("hbeams") != 72 or rec.get("vbeams") != 4:
        problems.append("LiDAR layout %sx%s, expected 72x4" % (rec.get("hbeams"), rec.get("vbeams")))
    snap = receipt.get("env_snapshot_navrl", {})
    if snap.get("NAVRL_SENSOR_ABLATION") != condition:
        problems.append("receipt snapshot NAVRL_SENSOR_ABLATION %r != %r" % (snap.get("NAVRL_SENSOR_ABLATION"), condition))
    return problems


def run_cell(protocol, out_root, condition, density, seed, run_id, head, start, records, module_sha):
    level_dir = out_root / condition
    extra = {"NAVRL_SENSOR_ABLATION": condition, "CAMONLY_PY": str(WRAPPER)}
    rec = G.run_cell(ROOT, protocol, out_root, condition, density, seed, QUOTA, run_id, None, head, start, records,
                     G.CELL_TIMEOUT_S, extra, LAUNCHER, G.F_SHA)
    cell_dir = level_dir / rec["cell_id"]
    extra_problems = ablation_problems(cell_dir, condition, module_sha) if rec["verdict"] == "OK" else []
    if extra_problems:
        rec["problems"].extend(extra_problems)
        rec["verdict"] = "FAIL"
    rec["condition"] = condition
    print("   ablation", condition, "OK" if not extra_problems else extra_problems[:2], flush=True)
    return rec


def preflight(protocol, approved, out_root):
    head, problems = G.tree_problems(ROOT, approved)
    if K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != BRANCH:
        problems.append("branch is not " + BRANCH)
    for rel in (PREREG, AMENDMENT, LAUNCHER, MODULE, "tools/camonly_entry.py", "tools/camonly_python"):
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or G.sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("camera-only launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(WRAPPER), os.X_OK):
        problems.append("tools/camonly_python is not executable")
    problems += K.static_problems(protocol, ROOT)
    f_ckpt = ROOT / G.F_CKPT_REL
    if not f_ckpt.is_file() or G.sha(f_ckpt) != G.F_SHA:
        problems.append("Policy F checkpoint copy missing or SHA mismatch: %s" % f_ckpt)
    if K.gpu_names() != [G.GPU]:
        problems.append("expected exactly one %s, found %s" % (G.GPU, K.gpu_names()))
    if G.gpu_apps():
        problems.append("other GPU compute processes: %s" % G.gpu_apps())
    imp, origins = K.import_problems(K.base_env(root=ROOT, inherited={k: v for k, v in os.environ.items()
                                                                       if not k.startswith("NAVRL_")}),
                                     K.launcher_path(protocol, ROOT).parent, ROOT)
    problems += imp
    if out_root.exists():
        problems.append("output already exists: %s" % out_root)
    return head, problems, origins


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("c1", "c2"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--gr0-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=3.5)
    args = ap.parse_args()
    out_root = pathlib.Path(args.out).resolve()
    gr0 = pathlib.Path(args.gr0_dir).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = preflight(protocol, args.approved_commit, out_root)
    if not (gr0 / "gr0_run_report.json").is_file():
        problems.append("--gr0-dir has no gr0_run_report.json")
    module_sha = G.sha(ROOT / MODULE)
    start = G.host()
    report = {"stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "started": G.now(),
              "host_start": start, "import_origins": origins, "module_sha256": module_sha, "launcher": LAUNCHER,
              "launcher_sha256": LAUNCHER_SHA, "gr0_dir": str(gr0), "preflight_problems": problems,
              "amendment": AMENDMENT,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if ENV_ALLOW.match(k)}}
    if args.stage == "c2":
        c1 = None
        for cand in sorted(out_root.parent.glob("*/c1_report.json")):
            r = json.loads(cand.read_text())
            if r.get("verdict") == "C1_IDENTITY_PASS" and r.get("commit") == head:
                c1 = cand
        if c1 is None:
            problems.append("c2 needs a C1_IDENTITY_PASS c1_report.json on this commit in %s" % out_root.parent)
        else:
            c1rep = json.loads(c1.read_text())
            if c1rep.get("env_allowlisted") != report["env_allowlisted"]:
                diff = sorted(k for k in set(c1rep.get("env_allowlisted") or {}) | set(report["env_allowlisted"])
                              if (c1rep.get("env_allowlisted") or {}).get(k) != report["env_allowlisted"].get(k))
                problems.append("runner environment differs from C1's in %s (amendment 1, M3)" % diff)
            if c1rep.get("gr0_dir") != str(gr0):
                problems.append("--gr0-dir differs from C1's")
        report["c1_report"] = str(c1)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    records, void = [], False
    run_id = "CAMONLY-" + args.stage.upper()
    if args.stage == "c1":
        comp = {}
        for condition, density, seed in C1_PLAN:
            rec = run_cell(protocol, out_root, condition, density, seed, run_id, head, start, records, module_sha)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if rec["verdict"] == "OK":
                comp[condition] = G.identity_compare(gr0 / "S00" / rec["cell_id"], out_root / condition / rec["cell_id"])
        ok = (set(comp) == {"full", "no_lidar"} and comp["full"]["result_equal"] and comp["full"]["episodes_equal"]
              and not comp["no_lidar"]["episodes_equal"])
        report["comparison"] = comp
        report["verdict"] = "VOID_HOST_FAULT" if void else ("C1_IDENTITY_PASS" if ok else "C1_IDENTITY_FAIL")
        name = "c1_report.json"
    else:
        stop = False
        for density in G.DENSITIES:
            for seed in G.SEEDS:
                for condition in C2_CONDITIONS:
                    if budget.exhausted():
                        report["budget_exhausted"] = True
                        stop = True
                        break
                    rec = run_cell(protocol, out_root, condition, density, seed, run_id, head, start, records,
                                   module_sha)
                    if rec["verdict"] != "OK":
                        void = rec["verdict"] == "VOID_HOST_FAULT"
                        stop = True
                        break
                if stop:
                    break
            if stop:
                break
        expected = len(G.DENSITIES) * len(G.SEEDS) * len(C2_CONDITIONS)
        complete = len(records) == expected and all(r["verdict"] == "OK" for r in records)
        report["verdict"] = "VOID_HOST_FAULT" if void else ("COMPLETE" if complete else "INCOMPLETE")
        report["cells_expected"] = expected
        name = "c2_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("C1_IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
