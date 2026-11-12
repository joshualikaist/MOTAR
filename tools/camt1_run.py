"""Runner of CAM-T1 (docs/plans/prereg_camonly_t1_2026-11-12.md): where the camera-only CAM-K2 v3 policy crashes
(Stage A, traced) and how its crash rate responds to a lower speed cap or a one-step action delay (Stage B).

Policy and condition: CAM-K2 v3 seeds 1 and 2 at their pins (tools/camk2v3e_run.checkpoints), in CAM-K2-E3's V3 arm
exactly: no_lidar, the 66 x 54 deg camera through tools/realveh_python, CAM-K2 v2's treatments ON with the per-cell
module seeds, the depth-validity rule blob_near ON, GR-0's S00 grid condition (E2, A3, quota 16, recorder,
direct_l2_v1). Every cell runs the launcher copy aerial_gym/rl_training/rl_games/eval_navrl_camt1_arms.sh (SHA-pinned
below): CAM-K2-E's launcher with one change, the governor's free-speed cap from CAMT1_GOVERNOR_FREE_MPS (3.53553390593
native, 2.5 or 2.0 only). NAVRL_MAX_VELOCITY is never changed.

Stages (each writes a new folder; run in this order by ~/motar_tmp/camt1_20261112/run_camt1.sh):
  identity  1 cell, run first (review r1 F4): native_identity (V3s1, 205 bars, seed 4501, native cap, no hook, no
            trace) must equal CAM-K2-E3's V3s1 cell: both ledgers byte for byte, equal outcome counts and result_equal
            (result.json minus the nonce and checkpoint path, as G7). Reading IDENTITY_PASS, else NOT_REPRODUCED (the
            chain stops; no trace cell runs, so a reproduction failure is never labelled TRACE_INTERFERES).
  trace     Stage A, 24 cells (needs one IDENTITY_PASS on this commit): V3s1, V3s2 x 70/115/160/205 bars x seeds 4501-4503, native cap, plus
            NAVRL_KROC2027_OCCLUSION_TRACE=1. Per cell, noninterference against CAM-K2-E3's eval cell of the same arm
            and block: episodes.jsonl and kroc2027_episodes.jsonl byte-identical and result.json outcome counts equal.
            Reading NONINTERFERENCE_PASS (all 24) or TRACE_INTERFERES (the traces then describe these runs only).
            The trace must read back through motar_eval/occlusion_trace.read_trace.
  control   3 cells, V3s1, 205 bars, seed 4501: k1d0_identity (the action hook loaded with K=1, D=0) must equal
            CAM-K2-E3's V3s1 cell as native_identity does (ledgers, outcome counts and result_equal); d1_smoke (D=1)
            and cap20_smoke (free cap 2.0) must be valid and differ from it.
            Reading CONTROL_PASS or CONTROL_FAIL. Needs one trace report on this commit (any finished reading).
  envelope  Stage B, 36 cells: cap25 (free cap 2.5), cap20 (free cap 2.0) and d1 (one-step delay, tools/actionhold_*)
            on V3s1 and V3s2 x 115/205 bars x seeds 4501-4503, in order density, seed, arm. Needs one CONTROL_PASS
            report on this commit, the same allowlisted environment, and the rate gate. The native reference is
            CAM-K2-E3's V3 cells (not re-run).
  plan      no launch, no output: prints the preflight problems and every cell's arm settings (smoke / dry run).
Cell checks (a FAIL stops the stage): gr_run.check_cell (host faults, receipt commit and clean tree, ledger,
recorder, import origins), camk2e_run.receipt_problems with a treated arm (realveh receipt, committed SHA-256,
treatment keys, camera, telemetry, vehicle record), camk2v3e_run.validity_problems (the rule ON), and this unit's
keys (camt1_problems): the NAVRL snapshot differs from the reference cell's only in the per-run keys and the arm's
own key (NAVRL_KROC2027_OCCLUSION_TRACE=1 when traced; NAVRL_SPEED_GOVERNOR_FREE_MPS = the cap for cap arms, else
the native string); result.json's condition equals the reference's except speed_governor_free_mps for cap arms
(= the cap); the action-hook sidecar is present exactly for hook arms with its config, patch applied and calls > 0
(D = 1: delayed steps > 0); the trace exists exactly for traced cells, reads back, closed complete, with the
66 x 54 deg camera; no episode's commanded horizontal speed exceeds the arm's free cap (+1e-3 m/s).
Fail-closed preflight: an approval json naming SCOPE and HEAD (~/motar_tmp/camt1_20261112/execution_approval.json);
clean tree; branch camonly/t1; COMMITTED_FILES committed (the preregistration included); the launcher at its pinned
SHA; the KROC2027 protocol's launcher and player at their pins (not Policy F's checkpoint, which CAM-T1 never
loads); tools/realveh_python executable; the CAM-K2 v3 checkpoints at their pins; CAM-K2-E3's eval report COMPLETE at its
run commit and every reference file at its committed manifest's SHA-256; one RTX 3070 and no other GPU compute;
PYTHONPATH equal to this tree and aerial_gym, motar_eval (and, for hook arms, sitecustomize from
tools/actionhold_site) resolving inside it; no inherited MOTAR_ACTION_* or CAMT1_* variable; free disk >= 1.5 GB;
the output folder new. A rise in the kernel fault count (or a boot / online-CPU change) voids the stage
(VOID_HOST_FAULT). No cell starts while another GPU compute process is present (checked before every cell; review r1
F10; review r2 N2: a failing or hanging GPU query also stops the stage, recorded as gpu_check_error). Budget: a wall-clock cap per stage (K.GpuBudget) and a rolling rate gate before every cell (stop
when elapsed + mean cell time x remaining cells x 1.10 exceeds the cap); the envelope also has a projection gate
from the control stage. A cell is not started with less than 1.0 GB free.
usage: tools/camt1_run.py {plan,identity,trace,control,envelope} --approved-commit SHA --out DIR [--e3-raw DIR]
"""
import argparse
import hashlib
import importlib.util
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import camk2v3e_run as V3                                   # noqa: E402  (CAM-K2-E3: checkpoints, validity checks)
import actionhold_run as AH                                 # noqa: E402  (G7: sidecar checks)

R2, G, K, R1, T2, T3 = V3.R2, V3.G, V3.K, V3.R1, V3.T2, V3.T3
PREREG = "docs/plans/prereg_camonly_t1_2026-11-12.md"
SCOPE = ("CAM-T1 v1: GPU evaluation of CAM-K2 v3 seeds 1 and 2 in CAM-K2-E3's V3 condition (no_lidar, 66 x 54 deg, "
         "CAM-K2 v2 treatments, blob_near rule): Stage A 24 traced cells (4 densities x 3 seeds) with a per-cell "
         "noninterference gate against CAM-K2-E3, after a 1-cell native identity; a 3-cell control (K=1/D=0 hook "
         "identity, d1 and cap20 smokes); Stage B 36 cells (free speed cap 2.5 and 2.0 m/s, one-step action delay; "
         "115 and 205 bars x 3 seeds); 64 cells, caps 0.25 / 2.0 / 0.5 / 2.5 h per stage with rate gates, idle host; tools/camt1_run.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camt1_20261112"
E3_RAW = pathlib.Path.home() / "motar_tmp" / "camk2v3e_20261110" / "raw"
E3_COMMIT = "aaca40e8dc6a3492cedd0b692427cbc796661015"
E3_MANIFEST = "docs/evidence/camonly_2026-11-04/camk2e/SHA256_MANIFEST_v3e.json"
BRANCH = "camonly/t1"
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_camt1_arms.sh"
LAUNCHER_SHA = "c166f560167b39b1ede168219f0afb0c5e1875c25642be8ea427e4cb8dbee22f"
SITE = ROOT / "tools/actionhold_site"
HOOK_FILES = AH.HOOK_FILES
TRACE_FILES = ("motar_eval/occlusion_trace.py", "aerial_gym/task/navrl_task/kroc_eval_adapter.py")
COMMITTED_FILES = ((PREREG, LAUNCHER, "tools/camt1_run.py", "tools/camt1_analyse.py", "tests/test_camt1.py", E3_MANIFEST)
                   + HOOK_FILES + TRACE_FILES + tuple(V3.COMMITTED_FILES))
MIN_FREE_GB = 1.5
CELL_MIN_FREE_GB = 1.0
MAX_HOURS = {"identity": 0.25, "trace": 2.0, "control": 0.5, "envelope": 2.5}
RATE_MARGIN = 1.10
CELL = V3.CELL
NATIVE_FREE = "3.53553390593"
CAPS = {"cap25": "2.5", "cap20": "2.0"}
CAP_TOLERANCE_MPS = 1e-3
CAP_CONDITION_KEY = "speed_governor_free_mps"
TRACE_ENV = {"NAVRL_KROC2027_OCCLUSION_TRACE": "1"}
SIDECAR = "actionhold_hook.json"                           # tools/actionhold_hook.SIDECAR
CAMERA = {"hfov_deg": 66.0, "vfov_deg": 54.0, "translation_m": [0.1, 0.0, 0.03]}
TRACE_DENSITIES = (70, 115, 160, 205)
ENVELOPE_DENSITIES = (115, 205)
SEEDS = (4501, 4502, 4503)
TRACE_ARMS = ("V3s1", "V3s2")
ENVELOPE_ARMS = ("cap25s1", "cap25s2", "cap20s1", "cap20s2", "d1s1", "d1s2")
IDENTITY_PLAN = (("native_identity", 205, 4501),)       # review r1 F4: the cheapest discriminating check runs first
CONTROL_PLAN = (("k1d0_identity", 205, 4501), ("d1_smoke", 205, 4501), ("cap20_smoke", 205, 4501))
CONTROL_EQUAL = ("k1d0_identity",)
CONTROL_DIFFER = ("d1_smoke", "cap20_smoke")
# The NAVRL snapshot keys that may differ from the reference cell's in every arm (paths, nonce, run id).
RUN_KEYS = {"NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_RUN_NONCE", "NAVRL_KROC2027_RUN_ID", "NAVRL_REALVEH_RECEIPT"}
INHERITED_REFUSED = re.compile(r"^(MOTAR_ACTION_|CAMT1_)")
REPORT = {"identity": "identity_report.json", "trace": "trace_run_report.json", "control": "control_report.json",
          "envelope": "envelope_run_report.json"}
FINISHED_TRACE = ("NONINTERFERENCE_PASS", "TRACE_INTERFERES")


def arm_spec(arm):
    """The settings of an arm: policy seed, free cap string, traced, action hook (k, d) or None, reference arm."""
    base = {"V3s1": (1, NATIVE_FREE, True, None), "V3s2": (2, NATIVE_FREE, True, None),
            "native_identity": (1, NATIVE_FREE, False, None), "k1d0_identity": (1, NATIVE_FREE, False, (1, 0)),
            "d1_smoke": (1, NATIVE_FREE, False, (1, 1)), "cap20_smoke": (1, CAPS["cap20"], False, None)}
    if arm in base:
        seed, free, trace, hook = base[arm]
    else:
        m = re.fullmatch(r"(cap25|cap20|d1)s([12])", arm)
        if not m:
            raise KeyError("unknown CAM-T1 arm %r" % arm)
        kind, seed = m.group(1), int(m.group(2))
        free, trace, hook = (CAPS.get(kind, NATIVE_FREE), False, (1, 1) if kind == "d1" else None)
    return {"policy_seed": seed, "free": free, "trace": trace, "hook": hook, "ref_arm": "V3s%d" % seed}


def trace_plan():
    return [(arm, d, s) for d in TRACE_DENSITIES for s in SEEDS for arm in TRACE_ARMS]


def identity_plan():
    return list(IDENTITY_PLAN)


def control_plan():
    return list(CONTROL_PLAN)


def envelope_plan():
    return [(arm, d, s) for d in ENVELOPE_DENSITIES for s in SEEDS for arm in ENVELOPE_ARMS]


PLANS = {"identity": identity_plan, "trace": trace_plan, "control": control_plan, "envelope": envelope_plan}


def inherited_problems(environ):
    """Refuse inherited action-hook or CAM-T1 variables (the cell environment is built from the runner's), and a
    runner PYTHONPATH other than this tree (the editable install would otherwise import the main checkout)."""
    p = ["inherited %s=%r is refused" % (k, environ[k]) for k in sorted(environ) if INHERITED_REFUSED.match(k)]
    if environ.get("PYTHONPATH") != str(ROOT):
        p.append("PYTHONPATH must be exactly %s, found %r" % (ROOT, environ.get("PYTHONPATH")))
    return p


def cell_extra(arm, seed, ckpts):
    """The variables a cell adds to gr_run's cell environment (CAM-K2-E3's V3 arm plus this arm's own)."""
    spec = arm_spec(arm)
    ckpt, ckpt_sha = ckpts[spec["ref_arm"]]
    extra = {"NAVRL_SENSOR_ABLATION": R2.CONDITION, "CAMEVAL_PY": str(R2.WRAPPER), "CAMK2E_CKPT": ckpt,
             "CAMK2E_CKPT_SHA": ckpt_sha, "CAMK2E_TREATMENTS": "on", **R1.CAM_VALUES, **R2.treatment_env(seed),
             **V3.VALIDITY_ENV, "CAMT1_GOVERNOR_FREE_MPS": spec["free"]}
    if spec["trace"]:
        extra.update(TRACE_ENV)
    if spec["hook"] is not None:
        k, d = spec["hook"]
        extra.update({"PYTHONPATH": "%s:%s" % (ROOT, SITE), "MOTAR_ACTION_HOLD_K": str(k),
                      "MOTAR_ACTION_DELAY_D": str(d)})
    return extra


def load_ot():
    """motar_eval/occlusion_trace.py of this tree, by path, under a private name (not registered in sys.modules)."""
    spec = importlib.util.spec_from_file_location("occlusion_trace_camt1", str(ROOT / "motar_eval/occlusion_trace.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def snapshot(cell_dir):
    return json.loads((pathlib.Path(cell_dir) / "receipt.json").read_text()).get("env_snapshot_navrl", {})


def trace_problems(cell_dir, ot):
    """The trace of a traced cell reads back (both SHA-256), has frames and layouts, closed complete, and records the
    66 x 54 deg camera at its mount; returns (problems, frames, layouts)."""
    try:
        tr = ot.read_trace(cell_dir)
    except (OSError, ValueError, KeyError) as e:
        return ["trace unreadable: %s" % e], None, None
    s = tr["summary"]
    p = []
    if s.get("frames", 0) <= 0 or s.get("layouts", 0) <= 0:
        p.append("trace has no frames or layouts")
    if s.get("reason") not in ("complete", "ledger complete", "quota complete"):
        p.append("trace closed for %r" % s.get("reason"))
    cam = (s.get("metadata") or {}).get("camera") or {}
    for key in ("hfov_deg", "vfov_deg"):
        if not isinstance(cam.get(key), (int, float)) or abs(cam[key] - CAMERA[key]) > 1e-6:
            p.append("trace camera %s = %r" % (key, cam.get(key)))
    if [round(float(x), 6) for x in cam.get("translation_m") or []] != CAMERA["translation_m"]:
        p.append("trace camera translation %r" % (cam.get("translation_m"),))
    return p, int(s.get("frames", 0)), int(s.get("layouts", 0))


def camt1_problems(cell_dir, arm, ref_dir, ot=None):
    """This unit's checks of one finished cell against the CAM-K2-E3 reference cell of the same arm seed and block."""
    spec = arm_spec(arm)
    cell_dir, ref_dir = pathlib.Path(cell_dir), pathlib.Path(ref_dir)
    p = []
    try:
        snap, ref_snap = snapshot(cell_dir), snapshot(ref_dir)
        result = K.comparable(json.loads((cell_dir / "result.json").read_text()))
        ref_result = K.comparable(json.loads((ref_dir / "result.json").read_text()))
    except (OSError, ValueError) as e:
        return ["cell or reference receipt / result unreadable: %s" % e]
    allowed = set(RUN_KEYS)
    if spec["trace"]:
        allowed.add("NAVRL_KROC2027_OCCLUSION_TRACE")
        if snap.get("NAVRL_KROC2027_OCCLUSION_TRACE") != "1":
            p.append("traced arm, but snapshot NAVRL_KROC2027_OCCLUSION_TRACE is %r" %
                     snap.get("NAVRL_KROC2027_OCCLUSION_TRACE"))
    elif "NAVRL_KROC2027_OCCLUSION_TRACE" in snap:
        p.append("untraced arm, but the snapshot has NAVRL_KROC2027_OCCLUSION_TRACE")
    if spec["free"] != NATIVE_FREE:
        allowed.add("NAVRL_SPEED_GOVERNOR_FREE_MPS")
    if snap.get("NAVRL_SPEED_GOVERNOR_FREE_MPS") != spec["free"]:
        p.append("snapshot NAVRL_SPEED_GOVERNOR_FREE_MPS %r is not the arm's %r" % (
            snap.get("NAVRL_SPEED_GOVERNOR_FREE_MPS"), spec["free"]))
    for key, want in (("NAVRL_SPEED_GOVERNOR_FIXED_MPS", "2.0"), ("NAVRL_MAX_VELOCITY", "2.5"),
                      ("NAVRL_SPEED_GOVERNOR", "riskcap")):
        if snap.get(key) != want:
            p.append("snapshot %s %r != %r" % (key, snap.get(key), want))
    diff = sorted(k for k in set(snap) | set(ref_snap) if snap.get(k) != ref_snap.get(k))
    extra = [k for k in diff if k not in allowed]
    if extra:
        p.append("snapshot differs from the reference cell in %s" % extra)
    cond, ref_cond = result.get("condition") or {}, ref_result.get("condition") or {}
    cdiff = sorted(k for k in set(cond) | set(ref_cond) if cond.get(k) != ref_cond.get(k))
    want_cdiff = [] if spec["free"] == NATIVE_FREE else [CAP_CONDITION_KEY]
    if cdiff != want_cdiff:
        p.append("result condition differs from the reference in %s, expected %s" % (cdiff, want_cdiff))
    if cond.get(CAP_CONDITION_KEY) != float(spec["free"]):
        p.append("result condition %s %r is not %s" % (CAP_CONDITION_KEY, cond.get(CAP_CONDITION_KEY), spec["free"]))
    side = cell_dir / SIDECAR
    if spec["hook"] is not None:
        sp, _sc = AH.sidecar_problems(cell_dir, *spec["hook"])
        p += sp
    elif side.exists():
        p.append("hook not expected, but %s exists" % side.name)
    trace_names = ("occlusion_frames.bin", "occlusion_layouts.npz", "occlusion_trace_summary.json")
    if spec["trace"]:
        tp, _f, _l = trace_problems(cell_dir, ot or load_ot())
        p += tp
    elif any((cell_dir / n).exists() for n in trace_names):
        p.append("untraced arm, but trace files exist")
    try:
        rows = K.read_jsonl(cell_dir / "kroc2027_episodes.jsonl")
        top = max(float(r["pursuer_commanded_speed_max_mps"]) for r in rows)
        if top > float(spec["free"]) + CAP_TOLERANCE_MPS:
            p.append("commanded horizontal speed %.4f m/s exceeds the free cap %s" % (top, spec["free"]))
    except (OSError, ValueError, KeyError, TypeError) as e:
        p.append("kroc2027 episodes unreadable for the cap check: %s" % e)
    return p


def ledgers_equal(cell_dir, ref_dir):
    """Byte equality of both episode ledgers and equal result.json outcome counts (the noninterference and identity
    rule), plus result_equal (G.identity_compare, recorded only)."""
    cell_dir, ref_dir = pathlib.Path(cell_dir), pathlib.Path(ref_dir)
    out = {"ref": str(ref_dir)}
    try:
        out["episodes_equal"] = sha(cell_dir / "episodes.jsonl") == sha(ref_dir / "episodes.jsonl")
        out["kroc_equal"] = sha(cell_dir / "kroc2027_episodes.jsonl") == sha(ref_dir / "kroc2027_episodes.jsonl")
        a = json.loads((cell_dir / "result.json").read_text())["outcome"]
        b = json.loads((ref_dir / "result.json").read_text())["outcome"]
        out["outcome_equal"] = all(a[k] == b[k] for k in ("captured", "crash", "timeout"))
        out["result_equal"] = G.identity_compare(ref_dir, cell_dir)["result_equal"]
    except (OSError, ValueError, KeyError) as e:
        out["error"] = str(e)
        out.update(episodes_equal=False, kroc_equal=False, outcome_equal=False, result_equal=False)
    out["pass"] = bool(out["episodes_equal"] and out["kroc_equal"] and out["outcome_equal"])
    out["identity_pass"] = bool(out["pass"] and out["result_equal"])      # review r1 F3: result_equal required (G7)
    return out


def noninterference(cell_dir, ref_dir):
    return ledgers_equal(cell_dir, ref_dir)


def identity_reading(comparisons):
    """IDENTITY_PASS iff native_identity ran and equals CAM-K2-E3's cell (both ledgers byte for byte, outcome counts
    and result.json minus the nonce and checkpoint path); otherwise NOT_REPRODUCED (review r1 F3, F4)."""
    ok = set(comparisons) == {a for a, _, _ in IDENTITY_PLAN} and all(c["identity_pass"] for c in comparisons.values())
    return "IDENTITY_PASS" if ok else "NOT_REPRODUCED"


def control_reading(comparisons):
    """CONTROL_PASS iff every control cell ran, k1d0_identity equals the reference (identity_pass, review r1 F3) and both
    smokes differ from it."""
    ok = (set(comparisons) == {a for a, _, _ in CONTROL_PLAN}
          and all(comparisons[a]["identity_pass"] for a in CONTROL_EQUAL)
          and all(not comparisons[a]["episodes_equal"] for a in CONTROL_DIFFER))
    return "CONTROL_PASS" if ok else "CONTROL_FAIL"


def trace_reading(cells, void):
    complete = len(cells) == len(trace_plan()) and all(c["verdict"] == "OK" for c in cells)
    if void:
        return "VOID_HOST_FAULT"
    if not complete:
        return "INCOMPLETE"
    return "NONINTERFERENCE_PASS" if all(c["noninterference"]["pass"] for c in cells) else "TRACE_INTERFERES"


def reference_dir(e3_raw, arm, density, seed):
    return pathlib.Path(e3_raw) / "eval_01" / arm_spec(arm)["ref_arm"] / (CELL % (density, seed))


def reference_files(arms_blocks):
    rels = set()
    for arm, d, s in arms_blocks:
        for f in ("episodes.jsonl", "kroc2027_episodes.jsonl", "result.json", "receipt.json"):
            rels.add("raw/eval_01/%s/%s/%s" % (arm_spec(arm)["ref_arm"], CELL % (d, s), f))
    return sorted(rels)


def e3_problems(e3_raw, root=ROOT):
    """CAM-K2-E3's eval report COMPLETE at its run commit, and every reference file this unit reads at the SHA-256 of
    the committed manifest (whose external root must be e3_raw's parent)."""
    p = []
    e3_raw = pathlib.Path(e3_raw)
    try:
        ev = json.loads((e3_raw / "eval_01" / "eval_run_report.json").read_text())
        if ev.get("verdict") != "COMPLETE" or ev.get("commit") != E3_COMMIT:
            p.append("CAM-K2-E3 eval report is not COMPLETE at %s" % E3_COMMIT[:7])
        man = json.loads((root / E3_MANIFEST).read_text())
    except (OSError, ValueError) as e:
        return ["CAM-K2-E3 eval report or manifest unreadable: %s" % e]
    if pathlib.Path(man.get("external_root", "")).resolve() != e3_raw.parent.resolve():
        p.append("manifest external_root %r is not %s" % (man.get("external_root"), e3_raw.parent))
    files = man.get("files") or {}
    blocks = identity_plan() + trace_plan() + envelope_plan() + control_plan()
    for rel in reference_files(blocks):
        want = (files.get(rel) or {}).get("sha256")
        path = e3_raw.parent / rel
        if not want or not path.is_file() or sha(path) != want:
            p.append("reference %s missing or not at the manifest SHA-256" % rel)
    return p


def mean_wall(records):
    walls = [r.get("wall_s") for r in records]
    return sum(walls) / len(walls) if walls and all(isinstance(w, (int, float)) for w in walls) else None


def rate_stop(elapsed_s, records, remaining, max_hours):
    """Rolling gate before a cell: stop when elapsed + mean cell time x remaining cells x margin exceeds the cap."""
    m = mean_wall(records)
    if m is None or not records:
        return False
    return elapsed_s + m * remaining * RATE_MARGIN > max_hours * 3600.0


def envelope_projection(control_report, e3_raw):
    """36 x CAM-K2-E3's mean V3 cell time at 115/205 bars x (this commit's control cells / CAM-K2-E3's V3s1 205-bar
    seed-4501 cell) x 1.10, in seconds; None when unreadable."""
    try:
        ev = json.loads((pathlib.Path(e3_raw) / "eval_01" / "eval_run_report.json").read_text())
        recs = [r for r in ev["records"] if r["arm"] in ("V3s1", "V3s2") and r["density"] in ENVELOPE_DENSITIES]
        ref = [r["wall_s"] for r in ev["records"] if (r["arm"], r["density"], r["seed"]) == ("V3s1", 205, 4501)]
        walls = [r["wall_s"] for r in control_report["records"]]
        assert recs and len(ref) == 1 and len(walls) == len(CONTROL_PLAN) and all(w > 0 for w in walls)
    except (OSError, ValueError, KeyError, TypeError, AssertionError):
        return None
    ratio = (sum(walls) / len(walls)) / ref[0]
    mean_e3 = sum(r["wall_s"] for r in recs) / len(recs)
    return {"ratio": ratio, "e3_mean_v3_cell_s": mean_e3,
            "projected_s": len(envelope_plan()) * mean_e3 * ratio * RATE_MARGIN}


def import_origin_problems():
    """aerial_gym / motar_eval resolve inside the tree for both cell environments, and sitecustomize resolves to
    tools/actionhold_site for hook cells (and is absent for the others)."""
    env = K.base_env(root=ROOT, inherited={k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")})
    cwd = ROOT / pathlib.Path(LAUNCHER).parent
    problems, origins = K.import_problems(env, cwd, ROOT)
    hook_env = dict(env, PYTHONPATH="%s:%s" % (ROOT, SITE))
    hp, hook_origins = K.import_problems(hook_env, cwd, ROOT)
    problems += ["(hook PYTHONPATH) " + x for x in hp]
    site = K.import_origin("sitecustomize", cwd, hook_env)
    if not site or not K.is_inside(site, SITE):
        problems.append("hook cells: sitecustomize resolves to %r, not inside %s" % (site, SITE))
    plain = K.import_origin("sitecustomize", cwd, env)
    if plain and K.is_inside(plain, SITE):
        problems.append("plain cells resolve the action-hook sitecustomize")
    origins.update({"hook_" + k: v for k, v in hook_origins.items()}, sitecustomize_hook=site, sitecustomize_plain=plain)
    return problems, origins


def stage_dependency(stage, out_root, head):
    """trace needs one IDENTITY_PASS, control one finished trace report and envelope one CONTROL_PASS, each on this
    commit and in the same parent folder; no stage may already have a report there. Returns (problems, previous
    report or None)."""
    parent = pathlib.Path(out_root).parent
    p, prev = [], None
    if list(parent.glob("*/" + REPORT[stage])):
        p.append("a %s already exists in %s" % (REPORT[stage], parent))
    need = {"trace": ("identity", ("IDENTITY_PASS",)), "control": ("trace", FINISHED_TRACE),
            "envelope": ("control", ("CONTROL_PASS",))}.get(stage)
    if need is None:
        return p, None
    found = sorted(parent.glob("*/" + REPORT[need[0]]))
    if len(found) != 1:
        return p + ["%s needs exactly one %s in %s, found %d" % (stage, REPORT[need[0]], parent, len(found))], None
    try:
        prev = json.loads(found[0].read_text())
    except ValueError as e:
        return p + ["%s unreadable: %s" % (found[0], e)], None
    if prev.get("reading") not in need[1] or prev.get("commit") != head:
        p.append("%s reads %r on %r, needs %s on %s" % (found[0], prev.get("reading"), prev.get("commit"), need[1],
                                                        head))
    prev["_path"] = str(found[0])
    return p, prev


def preflight(protocol, approved, out_root, e3_raw, unit_dir=UNIT_DIR, check_gpu=True):
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:
        problems.append("--approved-commit must name at least 7 hex digits")
    problems += inherited_problems(os.environ)
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
            problems += R1.approval_problems(json.loads(ap.read_text()), head, scope=SCOPE)
        except ValueError as e:
            problems.append("approval unreadable: %s" % e)
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or G.sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("CAM-T1 launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(R2.WRAPPER), os.X_OK):
        problems.append("tools/realveh_python is not executable")
    if not (SITE / "sitecustomize.py").is_file():
        problems.append("tools/actionhold_site/sitecustomize.py missing")
    problems += e3_problems(e3_raw)
    problems += protocol_problems(protocol)
    if check_gpu:
        if K.gpu_names() != [G.GPU]:
            problems.append("expected exactly one %s, found %s" % (G.GPU, K.gpu_names()))
        if G.gpu_apps():
            problems.append("other GPU compute processes: %s" % G.gpu_apps())
    imp, origins = import_origin_problems()
    problems += imp
    for name, mod in list(sys.modules.items()):
        f = getattr(mod, "__file__", None)
        if name.split(".")[0] in ("aerial_gym", "motar_eval") and f and not K.is_inside(f, ROOT):
            problems.append("runner process imported %s from outside the tree: %s" % (name, f))
    if out_root.exists():
        problems.append("output already exists: %s" % out_root)
    return head, problems, origins


def protocol_problems(protocol, root=ROOT):
    """kroc2027_eval.static_problems without Policy F's checkpoint: the protocol's launcher and player (play_navrl.sh,
    which every cell runs) at their pins. CAM-T1 never loads Policy F, and this worktree holds no copy of it (runs/ is
    ignored), so requiring that file would add nothing but a refusal."""
    p = []
    pol = protocol["policy"]
    for key, rel in (("launcher_sha256", pol["launcher"]), ("player_sha256", pol["player"])):
        path = root / rel
        if not path.is_file():
            p.append("missing %s" % rel)
        elif G.sha(path) != pol[key]:
            p.append("%s changed (pinned %s)" % (rel, pol[key][:12]))
    return p


def checkpoints():
    """CAM-K2 v3's checkpoints at their pins (camk2v3e_run.checkpoints); CAM-K2 v2's are not used."""
    ck, problems = V3.checkpoints()
    return ({k: v for k, v in ck.items() if k in ("V3s1", "V3s2")},
            [x for x in problems if x.startswith("CAM-K2 v3")])


def run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed, validity_sha,
             e3_raw, ot, budget):
    spec = arm_spec(arm)
    ckpt, ckpt_sha = ckpts[spec["ref_arm"]]
    timeout_s = G.CELL_TIMEOUT_S
    left = budget.remaining_s()
    if left is not None:
        timeout_s = max(60, min(timeout_s, int(left)))
    rec = G.run_cell(ROOT, protocol, out_root, arm, density, seed, R2.QUOTA, run_id, None, head, start, records,
                     timeout_s, cell_extra(arm, seed, ckpts), LAUNCHER, ckpt_sha)
    cell_dir = out_root / arm / rec["cell_id"]
    ref = reference_dir(e3_raw, arm, density, seed)
    extra = []
    if rec["verdict"] == "OK":
        extra = R2.receipt_problems(cell_dir, "K2s1", seed, committed)      # a treated arm of CAM-K2-E
        extra += V3.validity_problems(cell_dir, True, validity_sha)
        extra += camt1_problems(cell_dir, arm, ref, ot)
    if extra:
        rec["problems"].extend(extra)
        rec["verdict"] = "FAIL"
    rec.update(arm=arm, checkpoint=ckpt, checkpoint_sha256=ckpt_sha, free_mps=spec["free"], traced=spec["trace"],
               hook=list(spec["hook"]) if spec["hook"] else None, reference=str(ref),
               module_seeds=list(R2.module_seeds(seed)))
    if rec["verdict"] == "OK" and spec["trace"]:
        _p, rec["trace_frames"], rec["trace_layouts"] = trace_problems(cell_dir, ot)
    print("   camt1 checks", arm, density, seed, "OK" if not extra else extra[:2], flush=True)
    return rec


def run_stage(stage, protocol, out_root, head, start, ckpts, committed, validity_sha, e3_raw, report):
    ot = load_ot()
    budget = K.GpuBudget(MAX_HOURS[stage])
    records, cells, void = [], [], False
    run_id = "CAMT1-" + stage.upper()
    plan = PLANS[stage]()
    for i, (arm, density, seed) in enumerate(plan):
        if budget.exhausted():
            report["budget_exhausted"] = True
            break
        try:
            busy = G.gpu_apps()                              # review r1 F10: GPU idle before every cell
        except (subprocess.TimeoutExpired, OSError) as e:    # review r2 N2: a wedged nvidia-smi stops the stage
            report["gpu_check_error"] = {"before_cell": i, "error": "%s: %s" % (type(e).__name__, e)}
            break
        if busy:
            report["gpu_busy_stop"] = {"before_cell": i, "apps": busy}
            break
        if rate_stop(time.time() - budget.started, records, len(plan) - i, MAX_HOURS[stage]):
            report["rate_gate_stop"] = {"before_cell": i, "mean_cell_s": mean_wall(records)}
            break
        if shutil.disk_usage(str(out_root)).free / 1e9 < CELL_MIN_FREE_GB:
            report["disk_low_stop"] = {"before_cell": i}
            break
        rec = run_cell(protocol, out_root, arm, density, seed, run_id, head, start, records, ckpts, committed,
                       validity_sha, e3_raw, ot, budget)
        entry = {"arm": arm, "density": density, "seed": seed, "dir": str(out_root / arm / rec["cell_id"]),
                 "verdict": rec["verdict"]}
        if rec["verdict"] == "OK" and stage in ("identity", "trace", "control"):
            entry["comparison"] = ledgers_equal(out_root / arm / rec["cell_id"], reference_dir(e3_raw, arm, density,
                                                                                                seed))
            if stage == "trace":
                entry["noninterference"] = entry["comparison"]
        cells.append(entry)
        if rec["verdict"] != "OK":
            void = rec["verdict"] == "VOID_HOST_FAULT"
            break
    end = G.host()
    if any(end[k] != start[k] for k in ("fault_count", "boot_id", "online")):
        void = True
    report["host_end"] = end
    complete = len(cells) == len(plan) and all(c["verdict"] == "OK" for c in cells)
    if stage == "identity":
        comp = {c["arm"]: c["comparison"] for c in cells if "comparison" in c}
        reading = "VOID_HOST_FAULT" if void else (identity_reading(comp) if complete else "INCOMPLETE")
    elif stage == "trace":
        reading = trace_reading(cells, void)
    elif stage == "control":
        comp = {c["arm"]: c["comparison"] for c in cells if "comparison" in c}
        reading = "VOID_HOST_FAULT" if void else (control_reading(comp) if len(cells) == len(plan) and all(
            c["verdict"] == "OK" for c in cells) else "INCOMPLETE")
    else:
        complete = len(cells) == len(plan) and all(c["verdict"] == "OK" for c in cells)
        reading = "VOID_HOST_FAULT" if void else ("COMPLETE" if complete else "INCOMPLETE")
    report.update(reading=reading, cells=cells, records=records, cells_expected=len(plan))
    return reading


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("plan", "identity", "trace", "control", "envelope"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--e3-raw", default=str(E3_RAW))
    args = ap.parse_args(argv)
    out_root = pathlib.Path(args.out).resolve()
    e3_raw = pathlib.Path(args.e3_raw).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = preflight(protocol, args.approved_commit, out_root, e3_raw,
                                        check_gpu=args.stage != "plan")
    ckpts, ck_problems = checkpoints()
    problems += ck_problems
    committed = T2.committed_sha256(ROOT)
    validity_sha = T3.committed_validity_sha256(ROOT)
    if not validity_sha:
        problems.append("the validity module is not committed at HEAD")
    if args.stage == "plan":
        plan = {st: [{"arm": a, "density": d, "seed": s, **arm_spec(a),
                      "extra": {k: v for k, v in cell_extra(a, s, ckpts).items()
                                if k.startswith(("CAMT1_", "MOTAR_", "NAVRL_KROC2027_", "PYTHONPATH", "CAMK2E_CKPT"))}}
                     for a, d, s in PLANS[st]()] if len(ckpts) == 2 else "checkpoints unavailable"
                for st in PLANS}
        print(json.dumps({"stage": "plan", "commit": head, "preflight_problems": problems, "import_origins": origins,
                          "cells": {k: len(f()) for k, f in PLANS.items()}, "plan": plan},
                         indent=1, default=str))
        return 0
    start = G.host()
    report = {"unit": "CAM-T1", "stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG,
              "scope": SCOPE, "started": G.now(), "host_start": start, "import_origins": origins,
              "launcher": LAUNCHER, "launcher_sha256": LAUNCHER_SHA, "e3_raw": str(e3_raw), "checkpoints": ckpts,
              "committed_sha256": committed, "validity_sha256": validity_sha,
              "hook_sha256": {f: G.sha(ROOT / f) for f in HOOK_FILES},
              "max_hours": MAX_HOURS[args.stage], "preflight_problems": problems,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if R2.ENV_ALLOW.match(k)}}
    dep, prev = stage_dependency(args.stage, out_root, head)
    problems += dep
    if prev is not None:
        report["previous_report"] = prev["_path"]
        if prev.get("env_allowlisted") != report["env_allowlisted"]:
            problems.append("runner environment differs from the previous stage's")
        if prev.get("e3_raw") != str(e3_raw):
            problems.append("--e3-raw differs from the previous stage's")
        if args.stage == "envelope":
            proj = envelope_projection(prev, e3_raw)
            if proj is None:
                problems.append("envelope rate gate: control or CAM-K2-E3 cell times unreadable")
            elif proj["projected_s"] > MAX_HOURS["envelope"] * 3600:
                problems.append("envelope rate gate: projected %.0f s exceeds %.1f h" % (proj["projected_s"],
                                                                                         MAX_HOURS["envelope"]))
            report["rate_gate"] = proj
    if problems:
        report["reading"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    reading = run_stage(args.stage, protocol, out_root, head, start, ckpts, committed, validity_sha, e3_raw, report)
    report["finished"] = G.now()
    with (out_root / REPORT[args.stage]).open("x") as fh:
        fh.write(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("unit", "stage", "reading", "commit")}, indent=1))
    return 0 if reading in FINISHED_TRACE + ("IDENTITY_PASS", "CONTROL_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
