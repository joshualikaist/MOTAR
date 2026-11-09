"""Runner of CAM-K2-E3, the matched evaluation of CAM-K2 v3 (docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md).

CAM-K2 v3 (tools/camk2v3_train.py) is CAM-K2 v2 retrained with the invalid-depth validity rule blob_near
(aerial_gym/task/navrl_task/navrl_depth_validity.py). Every cell runs CAM-K2-E's condition: GR-0's S00 grid
(E2, A3, quota 16), `no_lidar` at 66 x 54 deg through tools/realveh_python, with CAM-K2 v2's treatments ON (per-motor
limit raised to T/W 4.0, thrust +-15 %, imbalance +-5 %, tau 0.02-0.04 s, the OAK-D Pro depth model; module seeds
10 x seed + 1 / + 2, matched in every arm). It reuses CAM-K2-E's launcher (SHA-pinned in tools/camk2e_run.py) and its
cell checks (tools/camk2e_run.receipt_problems with a treated arm), and adds the rule's checks.

Arms (all with the treatments ON):
  V3s   CAM-K2 v3 seed s, with the validity rule (as trained);
  V2Rs  CAM-K2 v2 seed s, with the validity rule (the rule at evaluation only, without retraining);
  V2s   CAM-K2 v2 seed s without the rule: NOT re-run. The reference cells are CAM-K2-E's eval K2s cells
        (~/motar_tmp/camk2e_20261108/raw/eval_01, committed SHA256_MANIFEST at d2a018b); the identity stage shows
        that this commit reproduces them exactly.
  identity  V2s1_identity / V2s2_identity: CAM-K2 v2 s1 / s2 without the rule (205 bars, seed 4501) must equal
            CAM-K2-E's eval K2s1 / K2s2 cell exactly; V2Rs1_negative: the same with the rule must differ from it; V2Rs1_repeat must equal
            V2Rs1_negative; V3s1_smoke: CAM-K2 v3 s1 with the rule must be valid and differ from V2Rs1_negative.
  eval      48 cells in order density, seed, arm (V3s1, V3s2, V2Rs1, V2Rs2); rate gate RATE_CELLS (48) x CAM-K2-E's
            mean eval cell x (identity / CAM-K2-E identity cell time) x 1.10 <= MAX_HOURS (3.5).
Cell checks: camk2e_run.receipt_problems (realveh receipt settings, committed SHA-256 of every hashed file, robot,
ablation, CAM-K1 overrides, snapshot treatment keys, camera, vehicle-randomization and depth-model telemetry and the
raised-limit record), and for the rule: the receipt's settings.depth_validity, its depth_validity install record and
depth_validity_sha256 (the module committed at HEAD), the snapshot's NAVRL_DEPTH_VALIDITY* keys, and the rule's
telemetry (one install line with the settings; a first and a summary line; 0 <= near <= invalid <= pixels;
sentinel_mismatch 0). The summary's near count is recorded, not required > 0: an eval cell's first 50 frames need not
come near a bar (the identity stage's V2Rs1_negative != V2s1_identity shows that the rule acts). Without the rule:
no depth_validity keys and no rule telemetry.
Fail-closed preflight as CAM-K2-E (approval naming SCOPE and HEAD, clean tree, branch, committed files, launcher SHA,
one RTX 3070 and no other GPU compute, import origins, free disk >= 1 GB, new output folder), plus the CAM-K2 v2 and
v3 checkpoints at their pins with TRAINED reports and sidecar manifests, and a COMPLETE CAM-K2-E eval folder.
usage: tools/camk2v3e_run.py {identity,eval} --approved-commit SHA --camk2e-raw DIR --out DIR
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
import camk2e_run as R2                                     # noqa: E402  (CAM-K2-E: cell checks, launcher, treatments)
import camk2v3_train as T3                                  # noqa: E402  (CAM-K2 v3: the rule's settings, module)

G, K, R1, T2 = R2.G, R2.K, R2.R1, R2.T2
PREREG = "docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md"
SCOPE = ("CAM-K2-E3 v1: matched GPU evaluation of CAM-K2 v3 seeds 1 and 2 (with the depth-validity rule) and of "
         "CAM-K2 v2 seeds 1 and 2 with the rule added at evaluation, against CAM-K2-E's CAM-K2 v2 cells, all under "
         "CAM-K2 v2's treatments (no_lidar, 66 x 54 deg, per-motor limit raised to T/W 4.0, thrust +-15 %, motor "
         "imbalance +-5 %, tau 0.02-0.04 s, OAK-D Pro depth model) on GR-0's S00 grid (4 densities x 3 seeds, "
         "quota 16), after a 5-cell identity control; 53 cells, cap 3.5 h per stage with a rate gate, idle host; "
         "tools/camk2v3e_run.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camk2v3e_20261110"
CAMK2V3_DIR = pathlib.Path.home() / "motar_tmp" / "camk2v3_20261108"
CAMK2V3_UNIT = "CAM-K2 v3"
V3_SHA = {1: "888e8056b2a6768318e6d9fa4136c5ed01f68b7480954f7f339801b99fafc8e1",      # CAM-K2 v3 s1, 326e307
          2: "203deea459ef1d937239281c46f07d96e797839708337666de1ecdf9492e4ea2"}      # CAM-K2 v3 s2, 326e307
COMMITTED_FILES = (PREREG, "tools/camk2v3e_run.py", "tools/camk2v3e_analyse.py", "tools/camk2v3_train.py",
                   T3.VALIDITY_REL) + tuple(R2.COMMITTED_FILES[1:])          # CAM-K2-E's list without its prereg
MIN_FREE_GB = 1.0
MAX_HOURS = 3.5
EVAL_ARMS = ("V3s1", "V3s2", "V2Rs1", "V2Rs2")
RULE_ARMS = EVAL_ARMS + ("V2Rs1_negative", "V2Rs1_repeat", "V3s1_smoke")
IDENTITY_PLAN = (("V2s1_identity", 205, 4501), ("V2s2_identity", 205, 4501), ("V2Rs1_negative", 205, 4501),
                 ("V2Rs1_repeat", 205, 4501), ("V3s1_smoke", 205, 4501))     # review r1 LOW: both v2 seeds reproduced
RATE_CELLS, RATE_MARGIN = 48, 1.10
CELL = R2.CELL
IDENTITY_REFERENCE = "eval_01/K2s1/" + CELL % (205, 4501)      # CAM-K2-E's eval K2s1 cell, inside --camk2e-raw
IDENTITY_REFERENCES = {"V2s2_identity": "eval_01/K2s2/" + CELL % (205, 4501)}   # every other identity arm: K2s1's
VALIDITY_ENV = {k: str(v) for k, v in T3.DEPTH_VALIDITY.items()}            # as train_navrl_camk2v3.sh exports them
VALIDITY_KEYS = tuple(T3.DEPTH_VALIDITY)


def rule(arm):
    return arm in RULE_ARMS


def arm_ckpt(arm, ckpts):
    base = {"V2s1_identity": "K2s1", "V2s2_identity": "K2s2", "V2Rs1_negative": "K2s1", "V2Rs1_repeat": "K2s1", "V3s1_smoke": "V3s1",
            "V2Rs1": "K2s1", "V2Rs2": "K2s2"}.get(arm, arm)
    return ckpts[base]


def checkpoints(camk2v3_dir=CAMK2V3_DIR):
    """CAM-K2 v2's checkpoints through camk2e_run.checkpoints (pins, TRAINED reports and manifests), and CAM-K2 v3's
    from their TRAINED train reports and sidecar manifests at the V3_SHA pins."""
    out, problems = R2.checkpoints()
    out = {k: v for k, v in out.items() if k in ("K2s1", "K2s2")}
    problems = [p for p in problems if p.startswith("CAM-K2")]
    for seed in (1, 2):
        pin = V3_SHA[seed]
        rp = camk2v3_dir / ("train_s%d" % seed) / "train_report.json"
        try:
            tr = json.loads(rp.read_text())
            ck = tr["checkpoint"]
        except (OSError, ValueError, KeyError, TypeError) as e:
            problems.append("CAM-K2 v3 seed %d train report unreadable: %s" % (seed, e))
            continue
        if tr.get("verdict") != "TRAINED" or tr.get("seed") != seed or tr.get("development_smoke"):
            problems.append("CAM-K2 v3 seed %d is not a TRAINED non-smoke run" % seed)
        if tr.get("prereg") != T3.PREREG or tr.get("scope") != T3.SCOPE:
            problems.append("CAM-K2 v3 seed %d train report is not CAM-K2 v3's" % seed)
        if ck.get("sha256") != pin:
            problems.append("CAM-K2 v3 seed %d checkpoint SHA %s is not the pinned %s" % (seed, ck.get("sha256"), pin))
        copy = pathlib.Path(str(ck.get("copy") or ""))
        if ck.get("sha256") != ck.get("copy_sha256") or not copy.is_file() or G.sha(copy) != ck.get("copy_sha256"):
            problems.append("CAM-K2 v3 seed %d checkpoint copy missing or SHA mismatch" % seed)
        man = copy.parent / (copy.name + ".manifest.json")
        try:
            m = json.loads(man.read_text())
            if (ck.get("manifest") or {}).get("sha256") != G.sha(man):
                problems.append("CAM-K2 v3 seed %d manifest SHA differs from the train report's" % seed)
            if (m.get("verdict") != "TRAINED" or m.get("checkpoint_sha256") != pin or m.get("robot") != R2.ROBOT
                    or m.get("treatments") != T3.expected_settings(seed) or m.get("seed") != seed
                    or m.get("unit") != CAMK2V3_UNIT or m.get("prereg") != T3.PREREG):
                problems.append("CAM-K2 v3 seed %d manifest is not a TRAINED manifest of this checkpoint" % seed)
        except (OSError, ValueError) as e:
            problems.append("CAM-K2 v3 seed %d sidecar manifest unreadable: %s" % (seed, e))
        out["V3s%d" % seed] = (str(copy), ck.get("copy_sha256"))
    return out, problems


def validity_problems(cell_dir, on, validity_sha):
    """The rule's receipt keys, snapshot keys and telemetry for one cell (on: the rule is expected)."""
    p = []
    try:
        rec = json.loads((cell_dir / "realveh_receipt.json").read_text())
        snap = json.loads((cell_dir / "receipt.json").read_text()).get("env_snapshot_navrl", {})
    except (OSError, ValueError) as e:
        return ["receipts unreadable: %s" % e]
    s = rec.get("settings") or {}
    got_env = {k: snap[k] for k in VALIDITY_KEYS if k in snap}
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    lines = T3.telemetry_lines([text], T3.DV_MOD.LOG_PREFIX)
    if not on:
        if "depth_validity" in s or "depth_validity" in rec or "depth_validity_sha256" in rec:
            p.append("rule OFF, but the receipt has depth_validity keys")
        if got_env:
            p.append("rule OFF, but the snapshot has %r" % got_env)
        if lines:
            p.append("rule OFF, but %d validity telemetry lines appear" % len(lines))
        return p
    if s.get("depth_validity") != T3.DEPTH_VALIDITY:
        p.append("receipt settings.depth_validity %r != %r" % (s.get("depth_validity"), T3.DEPTH_VALIDITY))
    if (rec.get("depth_validity") or {}).get("settings") != T3.DEPTH_VALIDITY:
        p.append("receipt depth_validity install record %r" % (rec.get("depth_validity"),))
    if not validity_sha or rec.get("depth_validity_sha256") != validity_sha:
        p.append("receipt depth_validity_sha256 %r is not the committed module's %r" % (
            rec.get("depth_validity_sha256"), validity_sha))
    if got_env != VALIDITY_ENV:
        p.append("snapshot validity keys %r != %r" % (got_env, VALIDITY_ENV))
    by = {e: [x for x in lines if x.get("event") == e] for e in ("install", "first", "summary")}
    if len(by["install"]) != 1 or by["install"][0].get("settings") != T3.DEPTH_VALIDITY:
        p.append("validity: expected one install line with the settings, got %r" % by["install"])
    if not by["first"]:
        p.append("validity: no first-frame line")
    if not by["summary"]:
        p.append("validity: no summary line")
    for x in by["first"] + by["summary"]:
        ok = (isinstance(x.get("pixels"), int) and x["pixels"] > 0 and isinstance(x.get("invalid_pixels"), int)
              and isinstance(x.get("near_pixels"), int) and 0 <= x["near_pixels"] <= x["invalid_pixels"] <= x["pixels"]
              and x.get("sentinel_mismatch") == 0
              and (x.get("event") != "summary" or x.get("calls") == T3.DV_MOD.STATS_CALLS))
        if not ok:
            p.append("validity: %s line implausible: %r" % (x.get("event"), x))
    return p


def cell_summary(cell_dir):
    """Descriptive: the rule's summary line (near / invalid / pixels), or NOT_RECORDED."""
    try:
        text = (cell_dir / "cell.log").read_text(errors="replace")
    except OSError:
        return "NOT_RECORDED"
    s = [x for x in T3.telemetry_lines([text], T3.DV_MOD.LOG_PREFIX) if x.get("event") == "summary"]
    return {k: s[-1].get(k) for k in ("near_pixels", "invalid_pixels", "pixels", "calls")} if s else "NOT_RECORDED"


def run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed, validity_sha):
    ckpt, ckpt_sha = arm_ckpt(arm, ckpts)
    on = rule(arm)
    extra = {"NAVRL_SENSOR_ABLATION": R2.CONDITION, "CAMEVAL_PY": str(R2.WRAPPER), "CAMK2E_CKPT": ckpt,
             "CAMK2E_CKPT_SHA": ckpt_sha, "CAMK2E_TREATMENTS": "on", **R1.CAM_VALUES, **R2.treatment_env(seed),
             **(VALIDITY_ENV if on else {})}
    rec = G.run_cell(ROOT, protocol, out_root, arm, density, seed, R2.QUOTA, run_id, None, head, start, records,
                     G.CELL_TIMEOUT_S, extra, R2.LAUNCHER, ckpt_sha)
    cell_dir = out_root / arm / rec["cell_id"]
    extra_problems = []
    if rec["verdict"] == "OK":
        extra_problems = R2.receipt_problems(cell_dir, "K2s1", seed, committed)     # a treated arm of CAM-K2-E
        extra_problems += validity_problems(cell_dir, on, validity_sha)
    if extra_problems:
        rec["problems"].extend(extra_problems)
        rec["verdict"] = "FAIL"
    rec.update(arm=arm, checkpoint=ckpt, checkpoint_sha256=ckpt_sha, treatments="on", validity_rule=on,
               module_seeds=list(R2.module_seeds(seed)),
               validity_summary=cell_summary(cell_dir) if on and rec["verdict"] == "OK" else None)
    print("   realveh+validity receipt", arm, "OK" if not extra_problems else extra_problems[:2], flush=True)
    return rec


def rate_problems(identity_report, camk2e_raw):
    """48 x CAM-K2-E's mean eval cell x (identity mean / CAM-K2-E identity mean) x 1.10 <= MAX_HOURS. Both identity
    stages run 205-bar, seed-4501 cells but different arm mixes (5 cells here, 4 there); the ratio is a speed proxy
    with about 10 % error, covered by the margin and the 3.5 h cap (review r2 LOW)."""
    walls = [r.get("wall_s") for r in identity_report.get("records") or []]
    if len(walls) != len(IDENTITY_PLAN) or not all(isinstance(w, (int, float)) and w > 0 for w in walls):
        return ["the identity report has no wall time for each of its %d cells" % len(IDENTITY_PLAN)]
    try:
        ki = json.loads((pathlib.Path(camk2e_raw) / "identity_01" / "identity_report.json").read_text())
        ke = json.loads((pathlib.Path(camk2e_raw) / "eval_01" / "eval_run_report.json").read_text())
        ref_i = [r["wall_s"] for r in ki["records"]]
        ref_e = [r["wall_s"] for r in ke["records"]]
        assert ref_i and ref_e
    except (OSError, ValueError, KeyError, AssertionError) as e:
        return ["rate gate: CAM-K2-E's identity or eval cell times are unreadable: %s" % e]
    ratio = (sum(walls) / len(walls)) / (sum(ref_i) / len(ref_i))
    projected = RATE_CELLS * (sum(ref_e) / len(ref_e)) * ratio * RATE_MARGIN
    if projected > MAX_HOURS * 3600:
        return ["rate gate: %d x %.0f s x ratio %.2f x %.2f = %.0f s exceeds the %.1f h cap" % (
            RATE_CELLS, sum(ref_e) / len(ref_e), ratio, RATE_MARGIN, projected, MAX_HOURS)]
    return []


def rate_projection(identity_report, camk2e_raw):
    """The projected eval seconds (recorded in the report; review lesson from CAM-K2-E)."""
    walls = [r["wall_s"] for r in identity_report["records"]]
    ki = json.loads((pathlib.Path(camk2e_raw) / "identity_01" / "identity_report.json").read_text())
    ke = json.loads((pathlib.Path(camk2e_raw) / "eval_01" / "eval_run_report.json").read_text())
    ref_i = [r["wall_s"] for r in ki["records"]]
    ref_e = [r["wall_s"] for r in ke["records"]]
    ratio = (sum(walls) / len(walls)) / (sum(ref_i) / len(ref_i))
    return {"ratio": ratio, "camk2e_mean_eval_cell_s": sum(ref_e) / len(ref_e),
            "projected_s": RATE_CELLS * (sum(ref_e) / len(ref_e)) * ratio * RATE_MARGIN}


def eval_plan():
    return [(arm, density, seed) for density in G.DENSITIES for seed in G.SEEDS for arm in EVAL_ARMS]


def preflight(protocol, approved, out_root, camk2e_raw, unit_dir=UNIT_DIR):
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:
        problems.append("--approved-commit must name at least 7 hex digits")
    import shutil
    free_gb = shutil.disk_usage(str(pathlib.Path.home())).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("free disk %.2f GB < %.1f GB" % (free_gb, MIN_FREE_GB))
    if K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip() != R2.BRANCH:
        problems.append("branch is not " + R2.BRANCH)
    ap = unit_dir / "execution_approval.json"
    if not ap.is_file():
        problems.append("no approval file %s" % ap)
    else:
        try:
            problems += R1.approval_problems(json.loads(ap.read_text()), head, scope=SCOPE)
        except ValueError as e:
            problems.append("approval unreadable: %s" % e)
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / R2.LAUNCHER).is_file() or G.sha(ROOT / R2.LAUNCHER) != R2.LAUNCHER_SHA:
        problems.append("CAM-K2-E launcher missing or not the pinned SHA %s" % R2.LAUNCHER_SHA[:12])
    if not os.access(str(R2.WRAPPER), os.X_OK):
        problems.append("tools/realveh_python is not executable")
    ev = camk2e_raw / "eval_01" / "eval_run_report.json"
    if not ev.is_file() or json.loads(ev.read_text()).get("verdict") != "COMPLETE":
        problems.append("--camk2e-raw has no COMPLETE eval_01/eval_run_report.json")
    for ref in {IDENTITY_REFERENCE, *IDENTITY_REFERENCES.values()}:
        if not (camk2e_raw / ref / "episodes.jsonl").is_file():
            problems.append("--camk2e-raw lacks %s" % ref)
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
    ap.add_argument("--camk2e-raw", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    out_root = pathlib.Path(args.out).resolve()
    k2e = pathlib.Path(args.camk2e_raw).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = preflight(protocol, args.approved_commit, out_root, k2e)
    ckpts, ck_problems = checkpoints()
    problems += ck_problems
    committed = T2.committed_sha256(ROOT)
    validity_sha = T3.committed_validity_sha256(ROOT)
    if not validity_sha:
        problems.append("the validity module is not committed at HEAD")
    start = G.host()
    report = {"stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "scope": SCOPE,
              "started": G.now(), "host_start": start, "import_origins": origins, "launcher": R2.LAUNCHER,
              "launcher_sha256": R2.LAUNCHER_SHA, "camk2e_raw": str(k2e), "checkpoints": ckpts,
              "committed_sha256": committed, "validity_sha256": validity_sha, "preflight_problems": problems,
              "max_hours": MAX_HOURS, "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ)
                                                          if R2.ENV_ALLOW.match(k)}}
    if args.stage == "eval":
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
                problems.append("runner environment differs from the identity stage's")
            if irep.get("camk2e_raw") != str(k2e):
                problems.append("--camk2e-raw differs from the identity stage's")
            problems += rate_problems(irep, k2e)
            if not rate_problems(irep, k2e):
                report["rate_gate"] = rate_projection(irep, k2e)
        report["identity_report"] = str(ident)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(MAX_HOURS)
    records, void = [], False
    run_id = "CAMK2E3-" + args.stage.upper()
    if args.stage == "identity":
        comp = {}
        for arm, density, seed in IDENTITY_PLAN:
            rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed,
                           validity_sha)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if rec["verdict"] == "OK":
                comp[arm] = G.identity_compare(k2e / IDENTITY_REFERENCES.get(arm, IDENTITY_REFERENCE),
                                               out_root / arm / rec["cell_id"])
        cid = CELL % (205, 4501)
        if "V2Rs1_repeat" in comp and "V2Rs1_negative" in comp:
            comp["V2Rs1_repeat"]["equals_V2Rs1_negative"] = (
                K.read_jsonl(out_root / "V2Rs1_repeat" / cid / "episodes.jsonl")
                == K.read_jsonl(out_root / "V2Rs1_negative" / cid / "episodes.jsonl"))
        if "V3s1_smoke" in comp and "V2Rs1_negative" in comp:
            comp["V3s1_smoke"]["differs_from_V2Rs1_negative"] = (
                K.read_jsonl(out_root / "V3s1_smoke" / cid / "episodes.jsonl")
                != K.read_jsonl(out_root / "V2Rs1_negative" / cid / "episodes.jsonl"))
        ok = (set(comp) == {a for a, _, _ in IDENTITY_PLAN}
              and all(comp[a]["result_equal"] and comp[a]["episodes_equal"] for a in ("V2s1_identity", "V2s2_identity"))
              and not comp["V2Rs1_negative"]["episodes_equal"]
              and comp["V2Rs1_repeat"].get("equals_V2Rs1_negative") is True
              and comp["V3s1_smoke"].get("differs_from_V2Rs1_negative") is True)
        report["comparison"] = comp
        report["verdict"] = "VOID_HOST_FAULT" if void else ("IDENTITY_PASS" if ok else "IDENTITY_FAIL")
        name = "identity_report.json"
    else:
        plan = eval_plan()
        for arm, density, seed in plan:
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed,
                           validity_sha)
            if rec["verdict"] != "OK":
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        complete = len(records) == len(plan) and all(r["verdict"] == "OK" for r in records)
        report["verdict"] = "VOID_HOST_FAULT" if void else ("COMPLETE" if complete else "INCOMPLETE")
        report["cells_expected"] = len(plan)
        name = "eval_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
