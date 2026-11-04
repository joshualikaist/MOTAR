"""G7 runner: Policy F with held or delayed actions (docs/plans/prereg_actionhold_g7_v1.md).

Through the unchanged camonly cell path (`full` condition = native F; GR-0's grid condition; quota 16), with the opt-in
hook tools/actionhold_hook.py loaded by tools/actionhold_site/sitecustomize.py (PYTHONPATH = tree:site). Plan:
  205/4501: k1d0 (identity control), k2, k4, d1;   70/4501: k2, k4, d1.
The identity cell runs first and must equal GR-0's S00 cell (episodes.jsonl byte-identical and result_equal) with the
hook's sidecar present (patch applied, config k=1 d=0); otherwise HOOK_IDENTITY_FAIL and the unit stops. Every other
cell must carry a sidecar whose config matches its arm, with patch applied and a non-zero latched/delayed count.
usage: tools/actionhold_run.py --approved-commit SHA --out DIR [--max-hours H]
"""
import argparse
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import camonly_run as C                                     # noqa: E402

G, K = C.G, C.K
PREREG = "docs/plans/prereg_actionhold_g7_v1.md"
SITE = ROOT / "tools/actionhold_site"
HOOK_FILES = ("tools/actionhold_hook.py", "tools/actionhold_site/sitecustomize.py")
PLAN = ((205, 4501, 1, 0), (205, 4501, 2, 0), (205, 4501, 4, 0), (205, 4501, 1, 1),
        (70, 4501, 2, 0), (70, 4501, 4, 0), (70, 4501, 1, 1))
REF = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00")


def arm_name(k, d):
    return "k%dd%d" % (k, d)


def sidecar_problems(cell_dir, k, d):
    try:
        sc = json.loads((cell_dir / "actionhold_hook.json").read_text())
    except (OSError, ValueError) as e:
        return ["actionhold sidecar missing or unreadable: %s" % e], None
    p = []
    if not sc.get("patch_applied"):
        p.append("actionhold patch not applied")
    if sc.get("config") != {"hold_k": k, "delay_d": d}:
        p.append("sidecar config %s != k=%d d=%d" % (sc.get("config"), k, d))
    if (k > 1 and not sc.get("held_env_steps")) or (d > 0 and not sc.get("delayed_zero_env_steps")):
        p.append("hook did not act (no held / delayed steps)")
    if not sc.get("calls"):
        p.append("hook was never called")
    return p, sc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=1.5)
    args = ap.parse_args()
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = C.preflight(protocol, args.approved_commit, out_root)
    for rel in (PREREG,) + HOOK_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    module_sha = G.sha(ROOT / C.MODULE)
    start = G.host()
    report = {"unit": "G7", "prereg": PREREG, "commit": head, "started": G.now(), "host_start": start,
              "import_origins": origins, "preflight_problems": problems,
              "hook_sha256": {f: G.sha(ROOT / f) for f in HOOK_FILES}}
    if problems:
        report["reading"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    records, cells, reading = [], [], None
    for d_bars, seed, k, d in PLAN:
        if budget.exhausted():
            report["budget_exhausted"] = True
            break
        arm = arm_name(k, d)
        extra = {"NAVRL_SENSOR_ABLATION": "full", "CAMONLY_PY": str(C.WRAPPER),
                 "PYTHONPATH": "%s:%s" % (ROOT, SITE), "MOTAR_ACTION_HOLD_K": str(k), "MOTAR_ACTION_DELAY_D": str(d)}
        rec = G.run_cell(ROOT, protocol, out_root, arm, d_bars, seed, C.QUOTA, "G7-" + arm.upper(), None, head, start,
                         records, G.CELL_TIMEOUT_S, extra, C.LAUNCHER, G.F_SHA)
        cell_dir = out_root / arm / rec["cell_id"]
        sc = None
        if rec["verdict"] == "OK":
            extra_problems = C.ablation_problems(cell_dir, "full", module_sha)
            sp, sc = sidecar_problems(cell_dir, k, d)
            extra_problems += sp
            if extra_problems:
                rec["problems"].extend(extra_problems)
                rec["verdict"] = "FAIL"
        entry = {"arm": arm, "k": k, "d": d, "density": d_bars, "seed": seed, "dir": str(cell_dir),
                 "verdict": rec["verdict"], "sidecar": sc}
        if rec["verdict"] == "OK" and (k, d) == (1, 0):
            comp = G.identity_compare(REF / rec["cell_id"], cell_dir)
            entry["identity"] = comp
            if not (comp["episodes_equal"] and comp["result_equal"]):
                reading = "HOOK_IDENTITY_FAIL"
        cells.append(entry)
        print("   G7", arm, d_bars, rec["verdict"], entry.get("identity"), flush=True)
        if rec["verdict"] != "OK":
            reading = "VOID_HOST_FAULT" if rec["verdict"] == "VOID_HOST_FAULT" else "INCOMPLETE"
            break
        if reading == "HOOK_IDENTITY_FAIL":
            break
    if reading is None:
        reading = "COMPLETE" if len(cells) == len(PLAN) else "INCOMPLETE"
    report.update(reading=reading, cells=cells, records=records, host_end=G.host(), finished=G.now())
    (out_root / "g7_run_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("unit", "reading", "commit")}, indent=1))
    return 0 if reading == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
