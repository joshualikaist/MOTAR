"""Runner of CAM-K1-E, the matched evaluation of the CAM-K1 camera-only lineage
(docs/plans/prereg_camonly_k1_eval_v1_2026-11-08.md).

Reuses tools/gr_run.py's cell environment (GR-0's grid condition: E2, A3 placement, quota ledger, motion recorder,
direct_l2_v1 distance backend, gains OFF = level S00), its cell checks and its launcher call, with the launcher copy
aerial_gym/rl_training/rl_games/eval_navrl_camk1e_arms.sh (SHA-pinned). Every cell runs `no_lidar` through
tools/camtrain_python, CAM-K1's committed entry point, which installs the ablation and the camera FOV / reward-weight
overrides and writes camtrain_receipt.json into the cell. A cell FAILs unless that receipt names `no_lidar`, the
committed ablation module and entry point, and exactly the arm's four values, and the receipt's NAVRL snapshot names
the same condition and values.

  identity  positive and negative control: Policy F with F's own values (87 x 58 deg, 0.3, 0.02) through this launcher
            and entry must equal C2's `no_lidar` cell (205 bars, seed 4501) exactly; F at the 66 x 54 deg camera must
            differ from it; CAM-K1 seed 1 at 66 x 54 deg (K1s1_smoke, the checkpoint path) must be valid and differ
            from the F66 cell (review r1 m1).
  eval      arms F66 (Policy F), K1s1 and K1s2 (CAM-K1 seeds 1 and 2), each at 66 x 54 deg, 0.6, 0.03, on GR-0's S00
            grid (4 densities x 3 seeds, quota 16); order density, seed, arm. Needs exactly one identity report in the
            same parent folder, an IDENTITY_PASS on this commit, no earlier eval there, and the same allowlisted runner
            environment (review r1 m11).
Review r1 (docs/evidence/camonly_2026-11-04/camk1e/REVIEW_design_r1.md) repairs: M1 receipt keys, m2 recorder FOV,
m5 pinned CAM-K1 checkpoint SHA-256, l2 free disk >= 1 GB, l4 commit length.
Fail-closed preflight: an execution approval naming SCOPE and HEAD; clean tree; branch camonly/dev; the preregistration,
launcher copy, ablation module and entry point committed; the launcher at its pinned SHA; Policy F's checkpoint SHA;
each CAM-K1 checkpoint from a TRAINED train report with matching copy SHA; one RTX 3070; no other GPU compute process;
import origins inside the tree; the output folder new. A rise in the host fault count voids the run.
usage: tools/camk1e_run.py {identity,eval} --approved-commit SHA --c2-dir C2_DIR --out DIR [--max-hours H]
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

PREREG = "docs/plans/prereg_camonly_k1_eval_v1_2026-11-08.md"
SCOPE = ("CAM-K1-E v1: matched GPU evaluation of CAM-K1 seeds 1 and 2 and of camera-only Policy F at the 66 x 54 deg "
         "camera (no_lidar) on GR-0's S00 grid (4 densities x 3 seeds, quota 16), after an identity control; "
         "about 39 cells, cap 3 h; tools/camk1e_run.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camk1e_20261108"
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit")
ENV_ALLOW = re.compile(r"^(AERIAL_.*|PYTHON.*|PATH|LD_.*|CUDA.*|NVIDIA.*|OMP_.*|MKL_.*|TORCH.*|CONDA.*|LANG|LC_.*|HOME|"
                       r"USER|DISPLAY|VK_.*|__GL.*)$")
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_camk1e_arms.sh"
LAUNCHER_SHA = "bdd0b07b0cd1b2c20e8f80d5cca70ff9410eb7cc43ed38f7ad61e67e0ef79d47"
MODULE = "aerial_gym/task/navrl_task/navrl_sensor_ablation.py"
ENTRY = "tools/camtrain_entry.py"
WRAPPER = ROOT / "tools/camtrain_python"
BRANCH = "camonly/dev"
CONDITION = "no_lidar"
QUOTA = 16
CAMK1_DIR = pathlib.Path.home() / "motar_tmp" / "camk1_20261106"
F_VALUES = {"NAVRL_CAMTRAIN_HFOV_DEG": "87.0", "NAVRL_CAMTRAIN_VFOV_DEG": "58.0",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.3", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.02"}
CAM_VALUES = {"NAVRL_CAMTRAIN_HFOV_DEG": "66.0", "NAVRL_CAMTRAIN_VFOV_DEG": "54.0",
              "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}
AFTER_KEYS = {"NAVRL_CAMTRAIN_HFOV_DEG": "detector_hfov_deg", "NAVRL_CAMTRAIN_VFOV_DEG": "detector_vfov_deg",
              "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "yaw_align_weight",
              "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "visibility_bonus"}
EVAL_ARMS = ("F66", "K1s1", "K1s2")
IDENTITY_PLAN = (("F87_identity", 205, 4501), ("F66_negative", 205, 4501), ("K1s1_smoke", 205, 4501))
K1_SHA = {1: "d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb",      # review r1 m5: pinned
          2: "5128bf2e0461fb45c6b8c4463d4d0a950bb88c0d28923c41d02d99da1102a4b7"}
MIN_FREE_GB = 1.0                                                                   # review r1 l2
C2_CELL = "no_lidar/E2_obstacle_aware__205bars__seed4501"


def arm_values(arm):
    return F_VALUES if arm == "F87_identity" else CAM_VALUES


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


def checkpoints(camk1_dir=CAMK1_DIR):
    """Policy F (worktree copy, F_SHA) and the two CAM-K1 checkpoints from their TRAINED train reports."""
    problems, out = [], {}
    f = ROOT / G.F_CKPT_REL
    if not f.is_file() or G.sha(f) != G.F_SHA:
        problems.append("Policy F checkpoint copy missing or SHA mismatch: %s" % f)
    out["F"] = (str(f), G.F_SHA)
    for seed in (1, 2):
        rp = camk1_dir / ("train_s%d" % seed) / "train_report.json"
        try:
            tr = json.loads(rp.read_text())
            ck = tr["checkpoint"]
        except (OSError, ValueError, KeyError) as e:
            problems.append("CAM-K1 seed %d train report unreadable: %s" % (seed, e))
            continue
        if tr.get("verdict") != "TRAINED" or tr.get("seed") != seed or tr.get("development_smoke"):
            problems.append("CAM-K1 seed %d is not a TRAINED non-smoke run" % seed)
        if ck.get("sha256") != K1_SHA[seed]:
            problems.append("CAM-K1 seed %d checkpoint SHA %s is not the pinned %s" % (seed, ck.get("sha256"),
                                                                                       K1_SHA[seed][:12]))
        if ck.get("sha256") != ck.get("copy_sha256") or not ck.get("copy"):
            problems.append("CAM-K1 seed %d checkpoint copy SHA differs from the run's" % seed)
        elif not pathlib.Path(ck["copy"]).is_file() or G.sha(ck["copy"]) != ck["copy_sha256"]:
            problems.append("CAM-K1 seed %d checkpoint copy missing or SHA mismatch" % seed)
        out["K1s%d" % seed] = (str(ck.get("copy")), ck.get("copy_sha256"))
    return out, problems


def arm_ckpt(arm, ckpts):
    if arm == "K1s1_smoke":
        return ckpts["K1s1"]
    return ckpts["F"] if arm in ("F87_identity", "F66_negative", "F66") else ckpts[arm]


def receipt_problems(cell_dir, values, module_sha, entry_sha):
    """The CAM-K1 entry point's receipt and the cell receipt's NAVRL snapshot must name no_lidar and the arm's values."""
    problems = []
    try:
        rec = json.loads((cell_dir / "camtrain_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["camtrain receipt missing or unreadable: %s" % e]
    abl = rec.get("ablation") or {}
    if abl.get("condition") != CONDITION:
        problems.append("camtrain receipt condition %r != %r" % (abl.get("condition"), CONDITION))
    if abl.get("hbeams") != 72 or abl.get("vbeams") != 4:
        problems.append("LiDAR layout %sx%s, expected 72x4" % (abl.get("hbeams"), abl.get("vbeams")))
    if rec.get("ablation_module_sha256") != module_sha:
        problems.append("ablation module SHA %s != committed %s" % (rec.get("ablation_module_sha256"), module_sha))
    if rec.get("entry_sha256") != entry_sha:
        problems.append("entry point SHA %s != committed %s" % (rec.get("entry_sha256"), entry_sha))
    for k in ("entry", "script"):
        if not K.is_inside(rec.get(k, ""), ROOT):
            problems.append("camtrain receipt %s outside the tree: %s" % (k, rec.get(k)))
    for k in ("task", "perception"):                    # the keys the entry point's receipt has (review r1 M1)
        v = abl.get(k)
        if not isinstance(v, str) or not v.startswith("/") or not K.is_inside(v, ROOT):
            problems.append("ablation %s missing, relative or outside the tree: %r" % (k, v))
    after = rec.get("overrides_after") or {}
    for env_key, field in AFTER_KEYS.items():
        if after.get(field) != float(values[env_key]):
            problems.append("override %s = %r, expected %s" % (field, after.get(field), values[env_key]))
    snap = receipt.get("env_snapshot_navrl", {})
    if snap.get("NAVRL_SENSOR_ABLATION") != CONDITION:
        problems.append("receipt snapshot NAVRL_SENSOR_ABLATION %r != %r" % (snap.get("NAVRL_SENSOR_ABLATION"), CONDITION))
    for env_key, v in values.items():
        if snap.get(env_key) != v:
            problems.append("receipt snapshot %s %r != %r" % (env_key, snap.get(env_key), v))
    try:                                                # review r1 m2: the FOV the simulator built
        cam = json.loads((cell_dir / K.RECORDER_SUMMARY).read_text()).get("camera", {})
        for key, env_key in (("hfov_deg", "NAVRL_CAMTRAIN_HFOV_DEG"), ("vfov_deg", "NAVRL_CAMTRAIN_VFOV_DEG")):
            if not isinstance(cam.get(key), (int, float)) or abs(cam[key] - float(values[env_key])) > 1e-6:
                problems.append("recorder camera %s = %r, expected %s" % (key, cam.get(key), values[env_key]))
    except (OSError, ValueError) as e:
        problems.append("recorder summary unreadable: %s" % e)
    log = cell_dir / "cell.log"
    want = "hfov=%ddeg" % round(float(values["NAVRL_CAMTRAIN_HFOV_DEG"]))
    if not log.is_file() or want not in log.read_text(errors="replace"):
        problems.append("cell.log lacks the built camera's FOV line %r" % want)
    return problems


def run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, module_sha, entry_sha):
    ckpt, ckpt_sha = arm_ckpt(arm, ckpts)
    values = arm_values(arm)
    extra = {"NAVRL_SENSOR_ABLATION": CONDITION, "CAMEVAL_PY": str(WRAPPER), "CAMK1E_CKPT": ckpt,
             "CAMK1E_CKPT_SHA": ckpt_sha, **values}
    rec = G.run_cell(ROOT, protocol, out_root, arm, density, seed, QUOTA, run_id, None, head, start, records,
                     G.CELL_TIMEOUT_S, extra, LAUNCHER, ckpt_sha)
    cell_dir = out_root / arm / rec["cell_id"]
    extra_problems = receipt_problems(cell_dir, values, module_sha, entry_sha) if rec["verdict"] == "OK" else []
    if extra_problems:
        rec["problems"].extend(extra_problems)
        rec["verdict"] = "FAIL"
    rec.update(arm=arm, checkpoint=ckpt, checkpoint_sha256=ckpt_sha, values=values)
    print("   camtrain receipt", arm, "OK" if not extra_problems else extra_problems[:2], flush=True)
    return rec


def preflight(protocol, approved, out_root, unit_dir=UNIT_DIR):
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:                         # review r1 l4
        problems.append("--approved-commit must name at least 7 hex digits")
    import shutil
    free_gb = shutil.disk_usage(str(pathlib.Path.home())).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("free disk %.2f GB < %.1f GB" % (free_gb, MIN_FREE_GB))
    if K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != BRANCH:
        problems.append("branch is not " + BRANCH)
    ap = unit_dir / "execution_approval.json"
    if not ap.is_file():
        problems.append("no approval file %s" % ap)
    else:
        try:
            problems += approval_problems(json.loads(ap.read_text()), head)
        except ValueError as e:
            problems.append("approval unreadable: %s" % e)
    for rel in (PREREG, LAUNCHER, MODULE, ENTRY, "tools/camtrain_python", "tools/camk1e_run.py"):
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or G.sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("CAM-K1-E launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(WRAPPER), os.X_OK):
        problems.append("tools/camtrain_python is not executable")
    problems += K.static_problems(protocol, ROOT)
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


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("identity", "eval"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--c2-dir", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=3.0)
    args = ap.parse_args(argv)
    out_root = pathlib.Path(args.out).resolve()
    c2 = pathlib.Path(args.c2_dir).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = preflight(protocol, args.approved_commit, out_root)
    ckpts, ck_problems = checkpoints()
    problems += ck_problems
    c2rep = c2 / "c2_run_report.json"
    if not c2rep.is_file() or json.loads(c2rep.read_text()).get("verdict") != "COMPLETE":
        problems.append("--c2-dir has no COMPLETE c2_run_report.json")
    if not (c2 / C2_CELL / "episodes.jsonl").is_file():
        problems.append("--c2-dir lacks %s" % C2_CELL)
    module_sha = G.sha(ROOT / MODULE)
    entry_sha = G.sha(ROOT / ENTRY)
    start = G.host()
    report = {"stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "scope": SCOPE,
              "started": G.now(), "host_start": start, "import_origins": origins, "module_sha256": module_sha,
              "entry_sha256": entry_sha, "launcher": LAUNCHER, "launcher_sha256": LAUNCHER_SHA, "c2_dir": str(c2),
              "checkpoints": ckpts, "preflight_problems": problems, "max_hours": args.max_hours,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if ENV_ALLOW.match(k)}}
    if args.stage == "eval":
        # review r1 m11: exactly one identity report in the parent folder, an IDENTITY_PASS on this commit, and no
        # earlier eval there; a failed identity or a rerun needs a new parent folder and an amendment.
        idents = sorted(out_root.parent.glob("*/identity_report.json"))
        ident = None
        if len(idents) != 1:
            problems.append("eval needs exactly one identity_report.json in %s, found %d" % (out_root.parent,
                                                                                            len(idents)))
        else:
            r = json.loads(idents[0].read_text())
            if r.get("verdict") == "IDENTITY_PASS" and r.get("commit") == head:
                ident = idents[0]
            else:
                problems.append("the identity report is not an IDENTITY_PASS on this commit")
        if list(out_root.parent.glob("*/eval_run_report.json")):
            problems.append("an eval_run_report.json already exists in %s" % out_root.parent)
        if ident is not None:
            irep = json.loads(ident.read_text())
            if irep.get("env_allowlisted") != report["env_allowlisted"]:
                diff = sorted(k for k in set(irep.get("env_allowlisted") or {}) | set(report["env_allowlisted"])
                              if (irep.get("env_allowlisted") or {}).get(k) != report["env_allowlisted"].get(k))
                problems.append("runner environment differs from the identity stage's in %s" % diff)
            if irep.get("c2_dir") != str(c2):
                problems.append("--c2-dir differs from the identity stage's")
        report["identity_report"] = str(ident)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    records, void = [], False
    run_id = "CAMK1E-" + args.stage.upper()
    if args.stage == "identity":
        comp = {}
        for arm, density, seed in IDENTITY_PLAN:
            rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, module_sha,
                           entry_sha)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if rec["verdict"] == "OK":
                comp[arm] = G.identity_compare(c2 / C2_CELL, out_root / arm / rec["cell_id"])
        if "K1s1_smoke" in comp and "F66_negative" in comp:   # review r1 m1: the CAM-K1 checkpoint path takes effect
            cid = "E2_obstacle_aware__205bars__seed4501"
            comp["K1s1_smoke"]["differs_from_F66_negative"] = (
                K.read_jsonl(out_root / "K1s1_smoke" / cid / "episodes.jsonl")
                != K.read_jsonl(out_root / "F66_negative" / cid / "episodes.jsonl"))
        ok = (set(comp) == {"F87_identity", "F66_negative", "K1s1_smoke"} and comp["F87_identity"]["result_equal"]
              and comp["F87_identity"]["episodes_equal"] and not comp["F66_negative"]["episodes_equal"]
              and comp["K1s1_smoke"].get("differs_from_F66_negative") is True)
        report["comparison"] = comp
        report["verdict"] = "VOID_HOST_FAULT" if void else ("IDENTITY_PASS" if ok else "IDENTITY_FAIL")
        name = "identity_report.json"
    else:
        stop = False
        for density in G.DENSITIES:
            for seed in G.SEEDS:
                for arm in EVAL_ARMS:
                    if budget.exhausted():
                        report["budget_exhausted"] = True
                        stop = True
                        break
                    rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts,
                                   module_sha, entry_sha)
                    if rec["verdict"] != "OK":
                        void = rec["verdict"] == "VOID_HOST_FAULT"
                        stop = True
                        break
                if stop:
                    break
            if stop:
                break
        expected = len(G.DENSITIES) * len(G.SEEDS) * len(EVAL_ARMS)
        complete = len(records) == expected and all(r["verdict"] == "OK" for r in records)
        report["verdict"] = "VOID_HOST_FAULT" if void else ("COMPLETE" if complete else "INCOMPLETE")
        report["cells_expected"] = expected
        name = "eval_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
