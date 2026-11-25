"""Runner of CAM-DX1, the 2 x 2 diagnostic evaluation of the CAM-S2b-FT final checkpoint at 205 bars: start condition
(random vs visible) x detection resolution (312 x 234 vs 624 x 468) (docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md).

It follows the committed CAM-S2b-FT-E runner (tools/cams2b_eval_run.py, not edited) and imports its helpers and
CAM-S2a-E's (tools/cams2a_eval_run.py, not edited): the FT training-run and checkpoint checks, the FT cell environment
(arm FT: no yaw clip, recorder navrl_cams2diag at the 0.7854 rad/s reference, the contact code), the tree-relative
ablation check, the telemetry parser, the recorder-row check and the launcher pin. The launcher is the committed CAM-S2a-E
copy aerial_gym/rl_training/rl_games/eval_navrl_cams2a_arms.sh at its pinned SHA: it passes every NAVRL_* variable of the
caller through unchanged, so the per-arm keys below need no new launcher.

Environment of every cell: the NAVRL_* keys of S2b-FT's training receipt (~/motar_tmp/cams2b_20261124/train_ft/
receipt.json, env_navrl) with CAM-S2b-FT-E's per-cell replacements for its FT arm (fixed density, module seeds 10 x seed + k,
ledger, quota, contact code, recorder), plus, in every cell, the CAM-DX1 recorder (NAVRL_INVIEW_DIAG=1, rows in the
cell directory), plus the arm's keys:
  R312  nothing else (camera = detection = 312 x 234, target 6-28 m, random yaw: as trained);
  R624  NAVRL_CAMERA_WIDTH/HEIGHT = NAVRL_DETECT_WIDTH/HEIGHT = 624/468;
  V312  NAVRL_GENERAL_GOAL_DIST_MAX=12, NAVRL_INVIEW_START=1, NAVRL_INVIEW_START_HALF_DEG=28.35,
        NAVRL_INVIEW_START_SEED=<cell seed> (aerial_gym/task/navrl_task/navrl_inview_start.py);
  V624  both.
Every other difference from the training environment refuses the cell before launch (env_problems) and fails it after
(snapshot_problems on the launcher's and the entry's NAVRL snapshots).
GPU memory fallback (preregistered): a 624 cell that fails with CUDA out-of-memory at 128 envs switches the 624 arms to
64 envs x quota 16 (1,024 episodes; the smoke at 64 x 2); the failed cell is rerun at 64 envs in its own folder
(<arm>_e64), and 624 cells already run at 128 are rerun at 64 at the end of the plan, so each 624 arm is homogeneous. A
second out-of-memory at 64 envs sets the 624 arms NOT_RUN_OOM; the 312 cells continue. The switch is recorded in the
stage report (mem_624) and carried from stage to stage. Every 312 cell stays at 128 x 8.
Stages (each needs the approval file and the stage before it, on the same commit, in the same parent folder):
  check     CPU only, writes nothing, needs no approval: prints the preflight problems.
  scope     prints SCOPE and its SHA-256 for the approval file.
  smoke     R312_smoke, V624_smoke at 205 / 4501, quota 1, never counted (V312_smoke only if V624 reads NOT_RUN_OOM, to
            carry the visible-start control); cap 0.5 h. SMOKE_OK needs every cell OK with >= 1 finished episode, the
            visible-start positive control on the RENDERED step-0 pose (every V bearing within +-28.35 deg; >= 1 R312
            bearing outside +-33.35 deg), the step-0 detection control (V share detected on the first observation > 0
            and >= 2 x R312's) and the 624 pixel control PASS (step-0 frames, else every detected frame by its own
            distance; no matched bin in either = SMOKE_FAIL), unless the 624 arms read NOT_RUN_OOM (review r1 P1-1/P1-2).
  identity  R312 at 205 / 4501 twice (R312_identity_a, R312_identity_b): result.json without nonce and checkpoint path
            and every ledger row equal; the recorders' rows are compared and reported. Cap 0.5 h. IDENTITY_FAIL stops.
  eval      12 cells, block-major by seed (R312, R624, V312, V624 per seed), quota 8 (1,024 episodes); cap 3.5 h. A cell
            whose projected time (projected_cell_s) exceeds the remaining cap is not started: BUDGET_EXHAUSTED, the
            stage reads INCOMPLETE and the analysis reads complete seed blocks only (review r1 P2-4).
A cell's verdict (cell_verdict, review r1 P2-3): a kernel fault-count rise voids the stage whatever the log says; else a
624 cell with a FAILED receipt and CUDA out-of-memory text is OOM (fallback); anything else that fails is FAIL.
Under the fallback only the 624 arms move to 64 envs; the 312 arms stay at 128 envs (a stated limit of DET under E64).
Cell checks (every cell; a FAIL stops the stage, an out-of-memory 624 cell triggers the fallback): launcher receipt (OK,
this commit, 0 dirty files, the checkpoint SHA, the training-receipt SHA, base_sim, the interpreter, GPU4GB unset);
ledger complete for the cell's envs x quota; result condition (CAM-S2a-E's expectation with the arm's goal maximum and
the cell's env count); realveh receipt against the FT training receipt (module settings with the cell's seeds, hashed
modules byte-identical except realveh_entry = HEAD's, tree-relative ablation paths, no yaw clip, contact code, cams2diag,
the CAM-DX1 recorder, the visible start iff V with its half-angle and seed); both NAVRL snapshots; telemetry ([camsafe],
[cams2contact], [cams2diag], [inviewdiag] with the arm's camera and detection resolution, [inview] iff V); recorder rows
of both recorders aligned with the ledger; in V cells every step-0 bearing within +-HALF_DEG and every step-0 horizontal
distance within 6-12 m; every MISMATCH line inside the arm's allowlist. Host fault count, boot id and online CPU list
unchanged.
Fail-closed preflight: approval naming SCOPE (and its SHA-256) and HEAD; clean tree; branch camonly/s2b; committed files;
launcher and play script at their pins; `git diff --name-only <FT training commit 335a1fa> HEAD` inside CODE_ALLOWLIST;
S2b-FT TRAINED (CAM-S2b-FT-E's check) with its final checkpoint and training receipt at the preregistered SHA-256; the
training environment as preregistered (camera = detection = 312 x 234, goals 6-28 m, FOV 66.7 deg); every planned cell
environment passes env_problems; one RTX 3070; no other GPU compute; import origins inside the tree; free disk >= 1 GB;
new output folder.
usage: tools/camdx1_eval_run.py {smoke,identity,eval} --approved-commit SHA --out DIR
       tools/camdx1_eval_run.py check --out DIR
       tools/camdx1_eval_run.py scope
"""
import argparse
import fnmatch
import hashlib
import json
import math
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
import gr_run as G                                          # noqa: E402  (host, tree_problems, run_launcher, identity_compare)
import kroc2027_eval as K                                   # noqa: E402  (base_env, ledger checks, GPU, imports, budget)
import camk1e_run as R1                                     # noqa: E402  (approval format, ENV_ALLOW)
import cams2a_eval_run as S2A                               # noqa: E402  (CAM-S2a-E's committed helpers, unchanged)
import cams2b_eval_run as W                                 # noqa: E402  (CAM-S2b-FT-E's committed helpers, unchanged)

PREREG = "docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md"
UNIT = "CAM-DX1"
SCOPE = ("CAM-DX1 v1: GPU diagnostic evaluation of the CAM-S2b-FT final checkpoint (last_gen_ppo_ep_15000_rew_51.018074.pth, "
         "sha256 f2c80fcc2b2e...) in its own training environment (~/motar_tmp/cams2b_20261124/train_ft/receipt.json) at 205 "
         "bars x seeds 4501-4503, quota 8 per env at 128 envs (1,024 episodes per cell), 2 x 2 arms: start random (R) vs "
         "visible start (V: target 6-12 m, initial target bearing within +-28.35 deg, navrl_inview_start) x camera = detection "
         "312 x 234 vs 624 x 468; stages smoke (R312 and V624 at quota 1, 0.5 h cap), identity (R312 twice, 0.5 h cap) and "
         "eval (12 cells block-major by seed, 3.5 h cap); 624 arms at 64 envs x quota 16 only after a recorded CUDA "
         "out-of-memory at 128 envs; idle host; tools/camdx1_eval_run.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camdx1_20261125"
BRANCHES = ("camonly/s2b",)

# ---- the policy and its training run (S2b-FT) ------------------------------------------------------------------------
FT_TRAIN_RECEIPT = W.FT_TRAIN_RECEIPT
FT_TRAIN_REPORT = W.FT_TRAIN_REPORT
FT_TRAIN_HEAD = "335a1fa6672feefa38103c3cf80d24053275370f"        # S2b-FT trained here (camonly/dev); the code diff base
FT_CKPT_SHA = "f2c80fcc2b2eaeb24e0b05d2284e74594a7ed8e7953f6b17b7dce49b09f2bbab"   # the preregistration's pin
FT_CKPT_FILE = "last_gen_ppo_ep_15000_rew_51.018074.pth"
FT_RECEIPT_SHA = "3411d59cc1656a2b94cae97843ecd22c8d5d2d75ebe362def0b3887132f8365f"  # the preregistration's pin (3411d59c...)
TRAINED_ENV = {"NAVRL_CAMERA_WIDTH": "312", "NAVRL_CAMERA_HEIGHT": "234", "NAVRL_DETECT_WIDTH": "312",
               "NAVRL_DETECT_HEIGHT": "234", "NAVRL_GENERAL_GOAL_DIST_MIN": "6", "NAVRL_GENERAL_GOAL_DIST_MAX": "28",
               "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6"}

# ---- the arms --------------------------------------------------------------------------------------------------------
HALF_DEG = "28.35"                     # half FOV (66.7 / 2 = 33.35) - 5 deg
R_OUTSIDE_DEG = 33.35                  # the R312 positive control: >= 1 step-0 bearing outside half the FOV
V_GOAL_DIST_MAX = "12"
V_GOAL_RANGE_M = (6.0, 12.0)
BEARING_TOL_DEG = 0.01                 # float32 pose round trip
DIST_TOL_M = 0.01
RES = {312: ("312", "234"), 624: ("624", "468")}
ARMS = {"R312": {"start": "random", "res": 312, "label": "DX1_R312"},
        "R624": {"start": "random", "res": 624, "label": "DX1_R624"},
        "V312": {"start": "visible", "res": 312, "label": "DX1_V312"},
        "V624": {"start": "visible", "res": 624, "label": "DX1_V624"}}
ARM_BASE = {"R312_smoke": "R312", "V624_smoke": "V624", "V312_smoke": "V312",
            "R312_identity_a": "R312", "R312_identity_b": "R312"}
DENSITY = 205
SEEDS = (4501, 4502, 4503)
EVAL_ARMS = ("R312", "R624", "V312", "V624")
SMOKE_PLAN = (("R312_smoke", DENSITY, 4501), ("V624_smoke", DENSITY, 4501))
SMOKE_V312 = ("V312_smoke", DENSITY, 4501)     # only when V624 reads NOT_RUN_OOM (carries the visible-start control)
IDENTITY_PLAN = (("R312_identity_a", DENSITY, 4501), ("R312_identity_b", DENSITY, 4501))
NUM_ENVS, QUOTA = 128, 8
FALLBACK_NUM_ENVS, FALLBACK_QUOTA = 64, 16
SMOKE_QUOTA = 1                        # at 128 envs; 2 at 64 envs (128 episodes either way)
MAX_HOURS = {"smoke": 0.5, "identity": 0.5, "eval": 3.5}
CELL_TIMEOUT_S = {312: 2400, 624: 3600}
MIN_FREE_GB = 1.0
YAW_REF = W.FT45_RADPS                 # the recorder reference of CAM-S2b-FT-E's FT arm (counts only)
LAUNCHER = S2A.LAUNCHER
LAUNCHER_SHA = S2A.LAUNCHER_SHA
DIAG_FILE = S2A.DIAG_FILE
INVIEW_REL = "aerial_gym/task/navrl_task/navrl_inview_start.py"
INVIEW_FILE = "inview_episodes.jsonl"
INVIEW_PREFIX, INVIEWDIAG_PREFIX = "[inview]", "[inviewdiag]"
NEW_FILES = ("tools/camdx1_eval_run.py", "tools/camdx1_eval_analyse.py", "tests/test_camdx1.py", INVIEW_REL)
COMMITTED_FILES = (PREREG, W.FT_TRAIN_PREREG, S2A.ENTRY, "tools/realveh_python", "tools/cams2a_eval_run.py",
                   "tools/cams2b_eval_run.py", S2A.CAMS2DIAG_REL, S2A.CONTACTCODE_REL, LAUNCHER) + NEW_FILES
# Paths that may change after S2b-FT's training commit: records, tests, this package and the realveh entry (additive hook,
# OFF bit-identical: tests/test_camdx1.py). Everything else refuses every stage (an amendment decides).
CODE_ALLOWLIST = ("docs/*", "WORKLOG.md", "tests/*", "tools/camdx1_*", S2A.ENTRY, INVIEW_REL)
ARM_KEYS = {"NAVRL_CAMERA_WIDTH", "NAVRL_CAMERA_HEIGHT", "NAVRL_DETECT_WIDTH", "NAVRL_DETECT_HEIGHT",
            "NAVRL_GENERAL_GOAL_DIST_MAX", "NAVRL_INVIEW_START", "NAVRL_INVIEW_START_HALF_DEG", "NAVRL_INVIEW_START_SEED"}
RECORDER_KEYS = {"NAVRL_INVIEW_DIAG", "NAVRL_INVIEW_DIAG_JSONL"}
SNAPSHOT_EXPECTED = S2A.SNAPSHOT_EXPECTED | ARM_KEYS | RECORDER_KEYS
RUNNER_SETS = (S2A.RUNNER_SETS | ARM_KEYS | RECORDER_KEYS) - {"NAVRL_YAWCLIP", "NAVRL_YAWCLIP_RADPS"}
UNCONSTRAINED_IN_SNAPSHOT = S2A.UNCONSTRAINED_IN_SNAPSHOT
RECEIPT_SHA_MAY_DIFFER = S2A.RECEIPT_SHA_MAY_DIFFER
# The checkpoint's own contract warnings: CAM-S2a-E's allowlist (training seed, curriculum keys, P9 module seed), plus,
# per arm, exactly the lines its own keys cause (the 624 detection size; the V goal maximum). Anything else fails the cell.
MISMATCH_624 = (r"CONFIG MISMATCH \| NAVRL_DETECT_WIDTH: checkpoint trained with 312\.000, running with 624\.000\.",
                r"CONFIG MISMATCH \| NAVRL_DETECT_HEIGHT: checkpoint trained with 234\.000, running with 468\.000\.")
MISMATCH_V = (r"TASK-DISTRIBUTION MISMATCH \| NAVRL_GENERAL_GOAL_DIST_MAX: checkpoint used 28\.000, running with 12\.000\.",)
OOM_RE = re.compile(r"out of memory|OutOfMemoryError|cudaErrorMemoryAllocation|CUDA_ERROR_OUT_OF_MEMORY", re.I)
# Amendment 1 (2026-11-25, after smoke raw/): at the 64-env fallback only, after a recorded out-of-memory at 128 envs,
# a cuDNN allocation/internal failure counts as the second out-of-memory (prereg lines 40-41: "if 64 envs also fail").
OOM_FALLBACK_RE = re.compile(r"CUDNN_STATUS_INTERNAL_ERROR|CUDNN_STATUS_ALLOC_FAILED", re.I)
STEP0_BIN_M = 1.0                      # the 624 control's step-0 distance bins (1 m, fixed before any run)
PIXEL_RATIO_MIN = 3.0
STEP0_DETECT_FACTOR = 2.0              # review r1 P1-1: V step-0 detected share >= 2 x R312's (and > 0)
RENDER_BEARING, RENDER_DIST = "step0_render_bearing_deg", "step0_render_dist_xy_m"
PROJECTION_MARGIN = 1.5                # review r1 P2-4: smoke-scaled cell-time estimate x 1.5


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def cfg(arm):
    return ARMS[ARM_BASE.get(arm, arm)]


def visible(arm):
    return cfg(arm)["start"] == "visible"


def res(arm):
    return cfg(arm)["res"]


def cell_id(arm, density, seed):
    return "%s__%dbars__seed%d" % (cfg(arm)["label"], int(density), int(seed))


def scope_digest():
    return hashlib.sha256(SCOPE.encode("utf-8")).hexdigest()


def mem_for(arm, mem_624, stage):
    """(num_envs, quota) of a cell: 128 x 8 (smoke 128 x 1) unless a 624 arm runs in the fallback mode."""
    fallback = res(arm) == 624 and mem_624 == "E64"
    if stage == "smoke":
        return (FALLBACK_NUM_ENVS, SMOKE_QUOTA * 2) if fallback else (NUM_ENVS, SMOKE_QUOTA)
    return (FALLBACK_NUM_ENVS, FALLBACK_QUOTA) if fallback else (NUM_ENVS, QUOTA)


def arm_dir(arm, num_envs):
    return arm if num_envs == NUM_ENVS else "%s_e%d" % (arm, num_envs)


# ---------------------------------------------------------------- environments
def training_env_problems(train_env):
    """The FT training environment carries the cams2 rules (CAM-S2a-E's F2) and the preregistered trained values."""
    p = list(S2A.training_env_problems(train_env))
    for k, v in TRAINED_ENV.items():
        if train_env.get(k) != v:
            p.append("training environment %s is %r, expected %r" % (k, train_env.get(k), v))
    stray = sorted(k for k in train_env if k.startswith("NAVRL_INVIEW_") or k.startswith("NAVRL_YAWCLIP"))
    if stray:
        p.append("training environment sets evaluation-only keys %s" % stray)
    return p


def arm_values(arm, seed):
    """The arm's keys (camera / detection resolution, goal maximum, visible start) as strings."""
    w, h = RES[res(arm)]
    out = {"NAVRL_CAMERA_WIDTH": w, "NAVRL_CAMERA_HEIGHT": h, "NAVRL_DETECT_WIDTH": w, "NAVRL_DETECT_HEIGHT": h,
           "NAVRL_GENERAL_GOAL_DIST_MAX": V_GOAL_DIST_MAX if visible(arm) else TRAINED_ENV["NAVRL_GENERAL_GOAL_DIST_MAX"]}
    if visible(arm):
        out.update(NAVRL_INVIEW_START="1", NAVRL_INVIEW_START_HALF_DEG=HALF_DEG, NAVRL_INVIEW_START_SEED=str(int(seed)))
    return out


def cell_navrl_env(train_env, seed, arm, cell_dir, quota=QUOTA, player_games_cap=None):
    """The NAVRL_* variables of one cell: CAM-S2b-FT-E's FT cell (no clip) + the CAM-DX1 recorder + the arm's keys.
    Raises ValueError when the training environment is not the preregistered one."""
    problems = training_env_problems(train_env)
    if problems:
        raise ValueError("; ".join(problems))
    env = W.cell_navrl_env(train_env, seed, "FT", quota, player_games_cap)
    env.update(NAVRL_INVIEW_DIAG="1", NAVRL_INVIEW_DIAG_JSONL=str(pathlib.Path(cell_dir) / INVIEW_FILE))
    env.update(arm_values(arm, seed))
    return env


def expected_cell_values(arm, seed, cell_dir, quota=QUOTA):
    """The runner-set values of one cell (before the launcher adds its per-cell keys)."""
    want = {"NAVRL_DENSITY_CURRICULUM": "0", "NAVRL_EPISODE_LEDGER": "1", "NAVRL_EPISODE_QUOTA_PER_ENV": str(int(quota)),
            "NAVRL_PLAYER_GAMES_CAP": str(10 ** 9), "NAVRL_CAMS2_CONTACTCODE": "1", "NAVRL_CAMS2DIAG": "1",
            "NAVRL_CAMS2DIAG_YAW_REF_RADPS": YAW_REF, "NAVRL_INVIEW_DIAG": "1",
            "NAVRL_INVIEW_DIAG_JSONL": str(pathlib.Path(cell_dir) / INVIEW_FILE), **S2A.module_seeds(seed),
            **arm_values(arm, seed)}
    return want


def env_problems(env, train_env, arm, seed, cell_dir, quota=QUOTA):
    """Before launch: the cell's NAVRL_* variables differ from the training environment only in the keys the runner sets
    (RUNNER_SETS), each with the cell's value, and none the arm must not set is present."""
    env = {k: v for k, v in env.items() if k.startswith("NAVRL_")}
    p = []
    removed = sorted(k for k in train_env if k not in env and k not in S2A.REMOVED_FROM_RECEIPT)
    if removed:
        p.append("cell environment drops training keys %s" % removed)
    diff = {k for k in set(env) | set(train_env) if env.get(k) != train_env.get(k)} - set(S2A.REMOVED_FROM_RECEIPT)
    extra = sorted(diff - RUNNER_SETS)
    if extra:
        p.append("cell environment differs from the training environment in %s" % extra)
    want = expected_cell_values(arm, seed, cell_dir, quota)
    for k, v in want.items():
        if env.get(k) != v:
            p.append("cell %s is %r, expected %r" % (k, env.get(k), v))
    for k in sorted(RUNNER_SETS - set(want)):
        if k in env and env.get(k) != train_env.get(k):
            p.append("cell sets %s=%r, which arm %s must not set" % (k, env[k], arm))
    for k in S2A.REMOVED_FROM_RECEIPT:
        if k in env:
            p.append("cell sets %s, which the launcher sets per cell" % k)
    return p


def cell_env(out_dir, arm, density, seed, ckpt, ckpt_sha, train_env, train_receipt_sha, protocol, num_envs, quota):
    cell_dir = out_dir / cell_id(arm, density, seed)
    inherited = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_") and k != "GPU4GB"}
    env = K.base_env(root=ROOT, inherited=inherited)
    env.update(cell_navrl_env(train_env, seed, arm, cell_dir, quota, protocol["evaluation"]["player_games_cap"]))
    env.update(ARMS=cfg(arm)["label"], DENSITIES=str(int(density)), SEEDS=str(int(seed)), GAMES=str(quota * num_envs),
               NUM_ENVS=str(num_envs), OUT_ROOT=str(out_dir), CAMEVAL_PY=str(S2A.WRAPPER), CAMS2A_CKPT=str(ckpt),
               CAMS2A_CKPT_SHA=ckpt_sha, CAMS2A_TRAIN_RECEIPT_SHA256=train_receipt_sha,
               AERIAL_GYM_SIM_NAME=S2A.SIM_NAME, PYTHON=S2A.REAL_PYTHON)                 # pinned, not inherited
    return env


def expected_snapshot_values(arm, density, seed, cell_dir, ckpt_sha, quota=QUOTA):
    want = expected_cell_values(arm, seed, cell_dir, quota)
    want.update({"NAVRL_NUM_BARS": str(density), "NAVRL_DENSITY_START": str(density), "NAVRL_DENSITY_FINAL": str(density),
                 "NAVRL_SEED": str(seed), "NAVRL_BULK_EVAL": "1", "NAVRL_EVAL_CHECKPOINT": ckpt_sha,
                 "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"),
                 "NAVRL_REALVEH_RECEIPT": str(cell_dir / "realveh_receipt.json"),
                 "NAVRL_CAMS2DIAG_JSONL": str(cell_dir / DIAG_FILE)})
    return want


def snapshot_problems(snap, train_env, arm, density, seed, cell_dir, ckpt_sha, label="snapshot", quota=QUOTA):
    """`snap` equals the training environment except SNAPSHOT_EXPECTED; every expected key has the cell's value; no
    expected key the arm does not use is present."""
    p = []
    diff = {k for k in set(snap) | set(train_env) if snap.get(k) != train_env.get(k)}
    extra = sorted(diff - SNAPSHOT_EXPECTED)
    if extra:
        p.append("%s differs from the training environment in %s" % (label, extra))
    want = expected_snapshot_values(arm, density, seed, cell_dir, ckpt_sha, quota)
    for k, v in want.items():
        if snap.get(k) != v:
            p.append("%s %s is %r, expected %r" % (label, k, snap.get(k), v))
    for k in sorted(SNAPSHOT_EXPECTED - set(want) - UNCONSTRAINED_IN_SNAPSHOT):
        if k in snap:
            p.append("%s has %s=%r, which arm %s must not set" % (label, k, snap[k], arm))
    if not snap.get("NAVRL_EVAL_RUN_NONCE"):
        p.append("%s lacks NAVRL_EVAL_RUN_NONCE" % label)
    return p


# ---------------------------------------------------------------- per-cell checks
def condition_problems(cond, train_env, arm, num_envs):
    """CAM-S2a-E's expected condition with the arm's goal maximum and the cell's env count."""
    want = dict(S2A.CONDITION_EXPECTED, num_envs=int(num_envs),
                goal_dist_max_m=float(V_GOAL_DIST_MAX) if visible(arm) else S2A.CONDITION_EXPECTED["goal_dist_max_m"])
    p = []
    for k, v in want.items():
        if cond.get(k) != v:
            p.append("result condition %s is %r, expected %r" % (k, cond.get(k), v))
    if cond.get("p9_empirical_error_sha256") != train_env.get("NAVRL_P9_EXPECTED_SHA256"):
        p.append("result condition P9 SHA %r is not training's" % cond.get("p9_empirical_error_sha256"))
    contract = cond.get("d8_dynamic_mesh_contract")
    if not isinstance(contract, dict) or contract.get("mode") != S2A.D8_MODE:
        p.append("result condition d8_dynamic_mesh_contract %r is not mode %s" % (contract, S2A.D8_MODE))
    return p


def is_oom(cell_dir, fallback=False):
    """The launcher's FAILED receipt and a CUDA out-of-memory message in the cell log; at the 64-env fallback
    (fallback=True, amendment 1) a cuDNN allocation/internal failure also counts."""
    try:
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError):
        receipt = {}
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    hit = OOM_RE.search(text) or (fallback and OOM_FALLBACK_RE.search(text))
    return receipt.get("status") != "OK" and bool(hit)


def output_problems(cell_dir, arm, commit, ckpt_sha, train_receipt_sha, train_env, num_envs, quota):
    """The launcher receipt, the result condition and the ledger of one cell (pure on the files)."""
    try:
        receipt = json.loads((cell_dir / "receipt.json").read_text())
        result = json.loads((cell_dir / "result.json").read_text())
        summary = json.loads((cell_dir / "episode_ledger_summary.json").read_text())
        rows = K.read_jsonl(cell_dir / "episodes.jsonl")
    except (OSError, ValueError) as e:
        return ["missing or unreadable output: %s" % e], None
    p = []
    if receipt.get("status") != "OK" or receipt.get("git_commit") != commit or receipt.get("git_dirty_files") != 0:
        p.append("receipt status/commit/dirty mismatch")
    if receipt.get("checkpoint_sha256") != ckpt_sha or result.get("checkpoint") != ckpt_sha:
        p.append("receipt/result checkpoint %s / %s is not %s" % (receipt.get("checkpoint_sha256"), result.get("checkpoint"),
                                                                  ckpt_sha))
    if receipt.get("train_receipt_sha256") != train_receipt_sha:
        p.append("receipt training-receipt SHA mismatch")
    if receipt.get("sim_name") != S2A.SIM_NAME or receipt.get("python") != S2A.REAL_PYTHON or receipt.get("gpu4gb", "") != "":
        p.append("receipt runtime pins (sim %r, python %r, gpu4gb %r) are not the preregistered ones" % (
            receipt.get("sim_name"), receipt.get("python"), receipt.get("gpu4gb")))
    p += condition_problems(result.get("condition") or {}, train_env, arm, num_envs)
    p += K.ledger_problems(summary, rows, num_envs, quota)
    quota_rows = [r for r in rows if r.get("in_quota")]
    counts = {o: sum(1 for r in quota_rows if r.get("outcome") == o) for o in ("capture", "crash", "timeout")}
    return p, {"quota_episodes": len(quota_rows), "finished_episodes": len(rows), "counts": counts,
               "duration_s": receipt.get("duration_s"), "result_sha256": receipt.get("result_sha256")}


def inview_rows_problems(cell_dir, arm, seed):
    """The CAM-DX1 recorder's rows: aligned with the ledger, every start recorded, and in a V cell every step-0 bearing
    within +-HALF_DEG and every step-0 horizontal distance within the 6-12 m range."""
    try:
        ledger = K.read_jsonl(cell_dir / "episodes.jsonl")
        rows = K.read_jsonl(cell_dir / INVIEW_FILE)
    except (OSError, ValueError) as e:
        return ["inview recorder or ledger rows unreadable: %s" % e]
    p = []
    if not rows or len(rows) != len(ledger):
        p.append("inview recorder rows %d != ledger rows %d" % (len(rows), len(ledger)))
    by = {(r.get("env_index"), r.get("env_episode_index")): r for r in rows}
    mism = sum(1 for r in ledger if (by.get((r.get("env_index"), r.get("env_episode_index"))) or {}).get("outcome")
               != r.get("outcome"))
    if mism:
        p.append("%d ledger episodes have no inview recorder row with the same outcome" % mism)
    if any(not r.get("start_recorded") or r.get("start_bearing_deg") is None or r.get(RENDER_BEARING) is None
           for r in rows):
        p.append("inview recorder rows without a recorded start or rendered step-0 pose")
    if visible(arm):                         # review r1 P1-1: on the pose the first observation was rendered with
        half = float(HALF_DEG) + BEARING_TOL_DEG
        out = [r for r in rows if r.get(RENDER_BEARING) is None or abs(r[RENDER_BEARING]) > half]
        if out:
            p.append("visible start: %d of %d rendered step-0 bearings outside +-%s deg" % (len(out), len(rows), HALF_DEG))
        lo, hi = V_GOAL_RANGE_M
        far = [r for r in rows if r.get(RENDER_DIST) is None
               or not lo - DIST_TOL_M <= r[RENDER_DIST] <= hi + DIST_TOL_M]
        if far:
            p.append("visible start: %d step-0 distances outside %g-%g m" % (len(far), lo, hi))
    return p


def inviewdiag_line(text):
    lines = [json.loads(line[len(INVIEWDIAG_PREFIX) + 1:]) for line in text.splitlines()
             if line.startswith(INVIEWDIAG_PREFIX + " {")]
    return lines[-1] if lines else None


def mismatch_problems(text, arm):
    allow = S2A.MISMATCH_ALLOWLIST + (MISMATCH_624 if res(arm) == 624 else ()) + (MISMATCH_V if visible(arm) else ())
    bad = [line for line in S2A.mismatch_lines(text) if not any(re.search(pat, line) for pat in allow)]
    return ["unexpected checkpoint-contract line: %s" % line[:200] for line in bad]


def receipt_problems(cell_dir, arm, density, seed, train_receipt, train_receipt_sha, committed, ckpt_sha, quota=QUOTA,
                     root=ROOT):
    """The realveh receipt, both NAVRL snapshots, the telemetry and both recorders of one cell against the FT training
    receipt. `committed` maps realveh_entry / cams2diag / contactcode / inview to their SHA-256 at HEAD."""
    try:
        rec = json.loads((cell_dir / "realveh_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["realveh receipt or cell receipt missing or unreadable: %s" % e]
    train_env = train_receipt.get("env_navrl") or {}
    p = []
    ts, s = train_receipt.get("settings") or {}, rec.get("settings") or {}
    seeds = S2A.module_seeds(seed)
    for key, seed_key in W.FT_SETTINGS_SEED_KEYS.items():
        want = dict(ts.get(key) or {})
        if not want:
            p.append("training receipt has no %s settings" % key)
            continue
        want[seed_key] = int(seeds[seed_key])
        if s.get(key) != want:
            p.append("realveh setting %s is %r, expected %r (F2)" % (key, s.get(key), want))
    for key in ("camsafe", "camtrain", "worst_case_thrust_to_weight"):
        if s.get(key) != ts.get(key):
            p.append("realveh setting %s is %r, expected %r" % (key, s.get(key), ts.get(key)))
    if "cams2" not in rec or (rec.get("cams2") or {}).get("config_after") != (train_receipt.get("cams2") or {}).get("config_after"):
        p.append("cams2 install record missing or its config_after differs from training (F2)")
    for key in ("cams2_sha256", "camsafe_sha256", "task_robot_name", "overrides_after"):
        if rec.get(key) != train_receipt.get(key):
            p.append("receipt %s %r != training %r" % (key, rec.get(key), train_receipt.get(key)))
    p += W.ablation_problems(rec, train_receipt, root)
    shas, tshas = rec.get("sha256") or {}, train_receipt.get("sha256") or {}
    if not tshas or set(shas) != set(tshas) or any(shas[k] != v for k, v in tshas.items() if k not in RECEIPT_SHA_MAY_DIFFER):
        p.append("receipt SHA-256 of the hashed modules differ from the training receipt")
    if shas.get("realveh_entry") != committed.get("realveh_entry"):
        p.append("receipt realveh_entry SHA is not the cell commit's")
    for k in ("entry", "script"):
        if not K.is_inside(rec.get(k, ""), root):
            p.append("realveh receipt %s outside the tree %s: %r" % (k, root, rec.get(k)))
    if "yawclip" in s or "yawclip" in rec:
        p.append("a yaw clip is installed (CAM-DX1 runs unclipped)")
    if ((rec.get("cams2_contactcode") or {}).get("contact_code") != 0 or s.get("cams2_contactcode") != {}
            or rec.get("cams2_contactcode_sha256") != committed.get("contactcode")):
        p.append("cams2 contact-code install record missing or not the committed module")
    d = rec.get("cams2diag") or {}
    if (d.get("jsonl") != str(cell_dir / DIAG_FILE) or d.get("yaw_ref_radps") != float(YAW_REF)
            or rec.get("cams2diag_sha256") != committed.get("cams2diag")):
        p.append("cams2diag install record missing, elsewhere, wrong yaw reference or not the committed module")
    inview_path = str(cell_dir / INVIEW_FILE)
    if (s.get("inview_diag") != {"NAVRL_INVIEW_DIAG_JSONL": inview_path}
            or (rec.get("inview_diag") or {}).get("jsonl") != inview_path or rec.get("inview_sha256") != committed.get("inview")):
        p.append("inview recorder install record missing, elsewhere or not the committed module")
    want_start = ({"NAVRL_INVIEW_START_HALF_DEG": float(HALF_DEG), "NAVRL_INVIEW_START_SEED": int(seed)}
                  if visible(arm) else None)
    istart = rec.get("inview_start") or {}
    if (s.get("inview_start") != want_start or ("inview_start" in rec) != visible(arm)
            or (visible(arm) and (istart.get("half_deg") != float(HALF_DEG) or istart.get("seed") != int(seed)))):
        p.append("visible-start receipt state %r / %r does not match arm %s" % (s.get("inview_start"), rec.get("inview_start"), arm))
    p += snapshot_problems(receipt.get("env_snapshot_navrl", {}), train_env, arm, density, seed, cell_dir, ckpt_sha,
                           label="launcher snapshot", quota=quota)
    p += snapshot_problems(rec.get("env_navrl") or {}, train_env, arm, density, seed, cell_dir, ckpt_sha,
                           label="entry env_navrl", quota=quota)
    if receipt.get("train_receipt_sha256") != train_receipt_sha:
        p.append("cell receipt does not name the training receipt SHA")
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    if S2A.parse_kv_line(text, S2A.CAMSAFE_PREFIX) is None:
        p.append("no [camsafe] telemetry line")
    if S2A.CONTACT_PREFIX + " {" not in text:
        p.append("no [cams2contact] telemetry line")
    if S2A.parse_kv_line(text, S2A.YAWCLIP_PREFIX) is not None:
        p.append("[yawclip] telemetry appears (CAM-DX1 runs unclipped)")
    if S2A.DIAG_PREFIX + " {" not in text:
        p.append("no [cams2diag] summary line")
    line = inviewdiag_line(text)
    w, h = (int(x) for x in RES[res(arm)])
    if line is None or line.get("camera_wh") != [w, h] or line.get("detect_wh") != [w, h]:
        p.append("no [inviewdiag] line with camera = detection = %d x %d (got %r)" % (
            w, h, line and (line.get("camera_wh"), line.get("detect_wh"))))
    iline = S2A.parse_kv_line(text, INVIEW_PREFIX)
    if visible(arm):
        if iline is None or iline.get("half_deg") != float(HALF_DEG) or iline.get("seed") != float(seed) \
                or not iline.get("envs") or iline.get("max_abs_rel_deg", 1e9) > float(HALF_DEG) + BEARING_TOL_DEG:
            p.append("visible start ON, but no [inview] line with half %s, seed %d and every bearing inside" % (HALF_DEG, seed))
    elif iline is not None:
        p.append("visible start OFF, but [inview] telemetry appears")
    p += S2A.diag_rows_problems(cell_dir)
    p += inview_rows_problems(cell_dir, arm, seed)
    p += mismatch_problems(text, arm)
    return p


def check_cell(cell_dir, arm, head, start_host, ckpt_sha, train_receipt_sha, train_env, num_envs, quota, root=ROOT):
    problems = []
    rec_host = G.host()
    for k in ("fault_count", "boot_id", "online"):
        if rec_host[k] != start_host[k]:
            problems.append("HOST " + k + " changed")
    p, stats = output_problems(cell_dir, arm, head, ckpt_sha, train_receipt_sha, train_env, num_envs, quota)
    problems += p
    if K.git("status", "--porcelain", root=root).strip():
        problems.append("tree became dirty")
    return problems, rec_host, stats


# ---------------------------------------------------------------- smoke controls (pure on recorder rows)
def visible_start_control(v_rows, r_rows):
    """The visible-start positive control: every V step-0 bearing within +-HALF_DEG (and >= 1 row), and >= 1 R312 step-0
    bearing outside +-33.35 deg (the random start is not accidentally aligned). Read on the rendered step-0 pose
    (step0_render_bearing_deg, robot_vehicle_orientation when the first observation is built; review r1 P1-1)."""
    vb = [r.get(RENDER_BEARING) for r in v_rows]
    rb = [r.get(RENDER_BEARING) for r in r_rows]
    v_in = sum(1 for b in vb if b is not None and abs(b) <= float(HALF_DEG) + BEARING_TOL_DEG)
    r_out = sum(1 for b in rb if b is not None and abs(b) > R_OUTSIDE_DEG)
    ok = bool(vb) and v_in == len(vb) and r_out >= 1
    return {"ok": ok, "v_rows": len(vb), "v_within": v_in, "v_max_abs_deg": max((abs(b) for b in vb if b is not None), default=None),
            "r_rows": len(rb), "r_outside_33_35": r_out}


def step0_detection_control(v_rows, r_rows):
    """Review r1 P1-1 (downstream of the renderer): the V cell's share of episodes detected on their first observation
    must be > 0 and >= STEP0_DETECT_FACTOR x the R312 cell's share."""
    def share(rows):
        known = [r for r in rows if r.get("start_detected") is not None]
        return (sum(1 for r in known if r["start_detected"]) / len(known)) if known else None

    v, r = share(v_rows), share(r_rows)
    ok = v is not None and r is not None and v > 0 and v >= STEP0_DETECT_FACTOR * r
    return {"ok": ok, "v_share": v, "r312_share": r, "factor": STEP0_DETECT_FACTOR}


def _bins(pairs):
    """{1 m distance bin: [frames, pixel sum]} of (distance, pixels) pairs."""
    out = {}
    for d, px in pairs:
        acc = out.setdefault(int(math.floor(float(d) / STEP0_BIN_M)), [0, 0.0])
        acc[0] += 1
        acc[1] += float(px)
    return out


def _standardised(hi, lo):
    """sum_b w_b A_hi_b / sum_b w_b A_lo_b over the bins with frames in both, w_b = hi frames in b; (ratio, bins)."""
    matched = sorted(b for b in hi if b in lo and hi[b][0] > 0 and lo[b][0] > 0)
    if not matched:
        return None, {}
    num = sum(hi[b][1] for b in matched)
    den = sum(hi[b][0] * lo[b][1] / lo[b][0] for b in matched)
    per_bin = {str(b): {"range_m": [b * STEP0_BIN_M, (b + 1) * STEP0_BIN_M], "frames_624": hi[b][0], "frames_312": lo[b][0],
                        "area_624": hi[b][1] / hi[b][0], "area_312": lo[b][1] / lo[b][0]} for b in matched}
    return (num / den if den > 0 else None), per_bin


def pixel_control(hi_rows, lo_rows):
    """The 624 control. Gated: the rendered target pixel area of the DETECTED STEP-0 FRAMES (start_detected, its pixel
    count start_target_pixels, its distance start_dist_xy_m), 624 cell over 312 cell, matched on 1 m bins of that step-0
    distance (bins with >= 1 detected step-0 frame in both cells) and standardised to the 624 cell's frame mix:
    ratio = sum_b w_b A624_b / sum_b w_b A312_b, w_b = the 624 cell's frames in b. PASS iff ratio >= 3;
    NO_MATCHED_FRAMES when no bin matches (reported, not failed). At step 0 both cells see the target from the same
    start geometry, so the trajectory after it cannot confound the ratio. Reported, not gated: the same ratio on every
    detected frame of the episodes binned by their step-0 distance, on every detected frame binned by its own distance
    (2 m recorder bins), and the d^2-normalised step-0 ratio (pixels x distance^2 does not depend on distance)."""
    def step0(rows):
        return [(r["start_dist_xy_m"], r.get("start_target_pixels") or 0) for r in rows
                if r.get("start_detected") and r.get("start_dist_xy_m") is not None]

    h0, l0 = step0(hi_rows), step0(lo_rows)
    ratio, per_bin = _standardised(_bins(h0), _bins(l0))

    def episode_bins(rows):
        out = {}
        for r in rows:
            if r.get("start_dist_xy_m") is None:
                continue
            acc = out.setdefault(int(math.floor(float(r["start_dist_xy_m"]) / STEP0_BIN_M)), [0, 0.0])
            acc[0] += int(r.get("detected_frames") or 0)
            acc[1] += float(r.get("detected_pixel_sum") or 0.0)
        return out

    def frame_bins(rows):
        out = {}
        for r in rows:
            for k, (f, px) in enumerate(zip(r.get("det_frames_by_dist_bin") or [], r.get("det_pixels_by_dist_bin") or [])):
                acc = out.setdefault(k, [0, 0.0])
                acc[0] += int(f)
                acc[1] += float(px)
        return out

    def d2(pairs):
        return (sum(px * d * d for d, px in pairs) / len(pairs)) if pairs else None

    ep_ratio, _ = _standardised(episode_bins(hi_rows), episode_bins(lo_rows))
    fr_ratio, fr_bins = _standardised(frame_bins(hi_rows), frame_bins(lo_rows))
    if per_bin:                              # the step-0 frames decide
        basis, gated = "step0", ratio
    elif fr_bins:                            # review r1 P1-2: no matched step-0 frame -> every detected frame by its own distance
        basis, gated = "own_distance", fr_ratio
    else:
        basis, gated = None, None
    verdict = ("NO_MATCHED_FRAMES" if basis is None
               else "PASS" if gated is not None and gated >= PIXEL_RATIO_MIN else "FAIL")
    a, b = d2(h0), d2(l0)
    return {"verdict": verdict, "basis": basis, "gated_ratio": gated, "ratio": ratio, "min_ratio": PIXEL_RATIO_MIN,
            "bin_m": STEP0_BIN_M, "matched_bins": per_bin,
            "step0_detected_frames": {"624": len(h0), "312": len(l0)},
            "reported_episode_frames_by_step0_bin_ratio": ep_ratio, "reported_frames_by_own_distance_ratio": fr_ratio,
            "reported_step0_d2_normalised_ratio": (a / b) if a and b else None}


def smoke_verdict(records, vs_control, px_control, mem_624, det_control=None):
    if any(r["verdict"] == "VOID_HOST_FAULT" for r in records):
        return "VOID_HOST_FAULT"
    ok_arms = {ARM_BASE.get(r["arm"], r["arm"]) for r in records if r["verdict"] == "OK"
               and ((r.get("stats") or {}).get("finished_episodes") or 0) >= 1}
    need = {"R312", "V624"} if mem_624 != "NOT_RUN_OOM" else {"R312", "V312"}
    if not need <= ok_arms or not (vs_control or {}).get("ok") or not (det_control or {}).get("ok"):
        return "SMOKE_FAIL"
    if mem_624 != "NOT_RUN_OOM" and (px_control or {}).get("verdict") != "PASS":     # r1 P1-2: no pass on no match
        return "SMOKE_FAIL"
    return "SMOKE_OK"


def identity_verdict(records, comparison):
    by = {r["arm"]: r["verdict"] for r in records}
    ok = all(by.get(a) == "OK" for a, _, _ in IDENTITY_PLAN) and len(records) == len(IDENTITY_PLAN)
    eq = (isinstance(comparison, dict) and comparison.get("result_equal") is True
          and comparison.get("episodes_equal") is True)
    return "IDENTITY_PASS" if ok and eq else "IDENTITY_FAIL"


def eval_plan():
    """Block-major by seed: R312, R624, V312, V624 per seed."""
    return [(arm, DENSITY, s) for s in SEEDS for arm in EVAL_ARMS]


def eval_verdict(records, mem_624):
    """COMPLETE iff every planned cell has an OK record in the arm's final memory mode; COMPLETE_624_NOT_RUN_OOM iff the
    six 312 cells are OK and the 624 arms read NOT_RUN_OOM; INCOMPLETE otherwise (a void is handled by the caller)."""
    if any(r["verdict"] == "FAIL" for r in records):
        return "INCOMPLETE"
    final_envs = FALLBACK_NUM_ENVS if mem_624 == "E64" else NUM_ENVS
    done = {(r["arm"], r["density"], r["seed"]) for r in records if r["verdict"] == "OK"
            and (res(r["arm"]) == 312 or r.get("num_envs") == final_envs)}
    want312 = {c for c in eval_plan() if res(c[0]) == 312}
    want624 = {c for c in eval_plan() if res(c[0]) == 624}
    if mem_624 == "NOT_RUN_OOM":
        return "COMPLETE_624_NOT_RUN_OOM" if want312 <= done else "INCOMPLETE"
    return "COMPLETE" if (want312 | want624) <= done else "INCOMPLETE"


def cell_verdict(problems, oom, arm):
    """Review r1 P2-3, fixed before any run: a rise of the kernel fault count (machine check, segfault, general
    protection; G.host) voids the stage whatever the cell log says, so an out-of-memory that ends in a counted segfault
    is VOID_HOST_FAULT, not OOM; otherwise a 624 cell whose launcher receipt is FAILED and whose log carries CUDA
    out-of-memory text is OOM (the fallback); every other problem is FAIL."""
    if any(p.startswith("HOST") for p in problems):
        return "VOID_HOST_FAULT"
    if not problems:
        return "OK"
    return "OOM" if oom and res(arm) == 624 else "FAIL"


def projected_cell_s(arm, num_envs, records, smoke_rep, ident_rep):
    """Review r1 P2-4: the projected wall time of a cell: the longest OK cell of the same arm and env count in this
    stage; else PROJECTION_MARGIN x the identity stage's R312 wall time x (the arm's smoke wall / the R312 smoke wall;
    624 arms use the V624 smoke cell, 312 arms ratio 1). None when nothing is known (the cell runs)."""
    base = ARM_BASE.get(arm, arm)
    own = [r["wall_s"] for r in records if r.get("base_arm") == base and r.get("verdict") == "OK"
           and r.get("num_envs") == num_envs and isinstance(r.get("wall_s"), (int, float))]
    if own:
        return max(own)
    ident = [r["wall_s"] for r in (ident_rep or {}).get("records") or [] if r.get("verdict") == "OK"]
    if not ident:
        return None
    smoke = {r.get("base_arm"): r for r in (smoke_rep or {}).get("records") or [] if r.get("verdict") == "OK"}
    ratio = 1.0
    if res(arm) == 624:
        v, r = smoke.get("V624"), smoke.get("R312")
        if not v or not r or v.get("num_envs") != num_envs or not r.get("wall_s"):
            return None
        ratio = v["wall_s"] / r["wall_s"]
    return PROJECTION_MARGIN * max(ident) * ratio


# ---------------------------------------------------------------- one cell
def run_cell(ctx, out_root, arm, density, seed, records, timeout_s, stage, num_envs, quota):
    ckpt, ckpt_sha = ctx["ckpt"]
    train_receipt, train_receipt_sha = ctx["train"]
    out_dir = out_root / arm_dir(arm, num_envs)
    out_dir.mkdir(parents=True, exist_ok=True)
    cid = cell_id(arm, density, seed)
    cell_dir = out_dir / cid
    env = cell_env(out_dir, arm, density, seed, ckpt, ckpt_sha, train_receipt["env_navrl"], train_receipt_sha,
                   ctx["protocol"], num_envs, quota)
    pre = env_problems(env, train_receipt["env_navrl"], arm, seed, cell_dir, quota)
    print("==", arm, cid, "envs %d quota %d" % (num_envs, quota), G.now(), "timeout %.0fs" % timeout_s, flush=True)
    t0 = time.time()
    rc = G.run_launcher(ROOT, ctx["protocol"], env, max(1.0, timeout_s), LAUNCHER) if not pre else None
    oom = (not pre) and is_oom(cell_dir, fallback=(res(arm) == 624 and num_envs != NUM_ENVS))
    problems, h, stats = check_cell(cell_dir, arm, ctx["head"], ctx["start"], ckpt_sha, train_receipt_sha,
                                    train_receipt["env_navrl"], num_envs, quota)
    problems = pre + problems
    if rc not in (0, None):
        problems.insert(0, "launcher exit %d" % rc)
    if not problems:
        problems += receipt_problems(cell_dir, arm, density, seed, train_receipt, train_receipt_sha, ctx["committed"],
                                     ckpt_sha, quota)
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    verdict = cell_verdict(problems, oom, arm)
    rec = {"arm": arm, "base_arm": ARM_BASE.get(arm, arm), "stage": stage, "cell_id": cid, "cell_dir": str(cell_dir),
           "tree": str(ROOT), "commit": ctx["head"], "seed": seed, "density": density, "num_envs": num_envs, "quota": quota,
           "start": cfg(arm)["start"], "resolution": list(RES[res(arm)]), "checkpoint": ckpt, "checkpoint_sha256": ckpt_sha,
           "train_receipt_sha256": train_receipt_sha, "module_seeds": S2A.module_seeds(seed), "exit": rc, "oom": oom,
           "wall_s": time.time() - t0, "timeout_s": timeout_s, "host_after": h, "stats": stats,
           "camsafe_telemetry_last_line": S2A.parse_kv_line(text, S2A.CAMSAFE_PREFIX),
           "inview_telemetry_last_line": S2A.parse_kv_line(text, INVIEW_PREFIX),
           "inviewdiag_last_line": inviewdiag_line(text), "mismatch_lines": S2A.mismatch_lines(text),
           "problems": problems, "verdict": verdict}
    records.append(rec)
    print("  ", rec["verdict"], stats and stats["counts"], "%.0fs" % rec["wall_s"], problems[:3], flush=True)
    return rec


# ---------------------------------------------------------------- preflight
def changed_since(base, root=ROOT):
    out = subprocess.run(["git", "-C", str(root), "diff", "--name-only", base, "HEAD"], capture_output=True, text=True)
    if out.returncode:
        return None
    return [line.strip() for line in out.stdout.splitlines() if line.strip()]


def allowlist_problems(paths, patterns, base):
    if paths is None:
        return ["git diff %s..HEAD failed" % base[:7]]
    bad = [p for p in paths if p and not any(fnmatch.fnmatch(p, pat) for pat in patterns)]
    return ["path changed since %s outside the allowlist: %s" % (base[:7], p) for p in bad]


def training_inputs():
    """(s2a_receipt, ft_receipt, ft_report, problems); unreadable files are problems, never defaults."""
    out, p = [], []
    for path in (W.S2A_TRAIN_RECEIPT, FT_TRAIN_RECEIPT, FT_TRAIN_REPORT):
        try:
            out.append(json.loads(path.read_text()))
        except (OSError, ValueError) as e:
            out.append({})
            p.append("unreadable %s: %s" % (path, e))
    return out[0], out[1], out[2], p


def policy_problems(s2a_receipt, ft_receipt, ft_report):
    """S2b-FT TRAINED (CAM-S2b-FT-E's check), its final checkpoint and training receipt at the preregistered pins, the
    training commit, and the preregistered training environment. Returns ((ckpt_path, ckpt_sha), problems)."""
    p = list(W.ft_training_problems(ft_receipt, ft_report, s2a_receipt))
    ck, cp = W.ft_checkpoint(ft_report)
    p += cp
    if ck[1] != FT_CKPT_SHA or pathlib.Path(ck[0] or "").name != FT_CKPT_FILE:
        p.append("FT final checkpoint %r / %r is not the preregistered %s (%s...)" % (ck[0], ck[1], FT_CKPT_FILE, FT_CKPT_SHA[:12]))
    if not FT_TRAIN_RECEIPT.is_file() or sha(FT_TRAIN_RECEIPT) != FT_RECEIPT_SHA:
        p.append("FT training receipt missing or not the preregistered SHA %s..." % FT_RECEIPT_SHA[:16])
    if (ft_report.get("preflight") or {}).get("head") != FT_TRAIN_HEAD:
        p.append("FT training head %r is not %s" % ((ft_report.get("preflight") or {}).get("head"), FT_TRAIN_HEAD[:7]))
    p += training_env_problems(ft_receipt.get("env_navrl") or {})
    return ck, p


def plan_env_problems(train_env, protocol, out_root):
    """Every planned cell environment (smoke, identity, eval, both memory modes) passes env_problems."""
    p = []
    cells = [(a, s, "smoke") for a, _, s in SMOKE_PLAN + (SMOKE_V312,)] + [(a, s, "identity") for a, _, s in IDENTITY_PLAN]
    cells += [(a, s, "eval") for a, _, s in eval_plan()]
    for arm, seed, stage in cells:
        for mode in (("E128", "E64") if res(arm) == 624 else ("E128",)):
            n, q = mem_for(arm, mode, stage)
            try:
                env = cell_env(out_root / arm_dir(arm, n), arm, DENSITY, seed, "/x/last_gen_ppo_ep_1_rew_1.pth", FT_CKPT_SHA,
                               train_env, FT_RECEIPT_SHA, protocol, n, q)
            except ValueError as e:
                return ["cell environment refused: %s" % e]
            cell_dir = out_root / arm_dir(arm, n) / cell_id(arm, DENSITY, seed)
            p += ["%s/%d/%s: %s" % (arm, seed, mode, x) for x in env_problems(env, train_env, arm, seed, cell_dir, q)]
    return p


def preflight(protocol, approved, out_root, stage, unit_dir=UNIT_DIR):
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:
        problems.append("--approved-commit must name at least 7 hex digits")
    free_gb = shutil.disk_usage(str(pathlib.Path.home())).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("free disk %.2f GB < %.1f GB" % (free_gb, MIN_FREE_GB))
    branch = K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip()
    if branch not in BRANCHES:
        problems.append("branch %s is not one of %s" % (branch, BRANCHES))
    ap = unit_dir / "execution_approval.json"
    approval_state = []
    if not ap.is_file():
        approval_state.append("no approval file %s" % ap)
    else:
        try:
            approval = json.loads(ap.read_text())
            approval_state += R1.approval_problems(approval, head, scope=SCOPE)
            if approval.get("scope_sha256") != scope_digest():
                approval_state.append("approval scope_sha256 is not the SHA-256 of SCOPE (`camdx1_eval_run.py scope`)")
        except ValueError as e:
            approval_state.append("approval unreadable: %s" % e)
    if stage != "check":                          # the check stage needs no approval file (preregistration)
        problems += approval_state
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(S2A.WRAPPER), os.X_OK):
        problems.append("tools/realveh_python is not executable")
    if not pathlib.Path(S2A.REAL_PYTHON).is_file():
        problems.append("interpreter %s missing" % S2A.REAL_PYTHON)
    problems += W.player_pin_problems(protocol)
    if K.gpu_names() != [G.GPU]:
        problems.append("expected exactly one %s, found %s" % (G.GPU, K.gpu_names()))
    if G.gpu_apps():
        problems.append("other GPU compute processes: %s" % G.gpu_apps())
    imp, origins = K.import_problems(K.base_env(root=ROOT, inherited={k: v for k, v in os.environ.items()
                                                                       if not k.startswith("NAVRL_")}),
                                     (ROOT / LAUNCHER).parent, ROOT)
    problems += imp
    if out_root.exists():
        problems.append("output already exists: %s" % out_root)
    return head, problems, origins, approval_state


def stage_link_problems(stage, out_root, head, ckpt_sha):
    """Each stage needs the earlier ones in the same parent, on this commit, with the same checkpoint (pure on files)."""
    parent, p, found = out_root.parent, [], {}

    def one(name, verdicts):
        hits = sorted(parent.glob("*/" + name))
        if len(hits) != 1:
            p.append("%s needs exactly one %s in %s, found %d" % (stage, name, parent, len(hits)))
            return None
        r = json.loads(hits[0].read_text())
        if r.get("verdict") not in verdicts or r.get("commit") != head or r.get("ft_checkpoint_sha256") != ckpt_sha:
            p.append("%s is not %s on this commit with this checkpoint" % (hits[0], "/".join(verdicts)))
            return None
        found[name] = (str(hits[0]), r)
        return r

    if stage == "smoke" and list(parent.glob("*/smoke_report.json")):
        p.append("a smoke_report.json already exists in %s" % parent)
    if stage in ("identity", "eval"):
        one("smoke_report.json", ("SMOKE_OK",))
    if stage == "identity" and list(parent.glob("*/identity_report.json")):
        p.append("an identity_report.json already exists in %s" % parent)
    if stage == "eval":
        one("identity_report.json", ("IDENTITY_PASS",))
        if list(parent.glob("*/eval_run_report.json")):
            p.append("an eval_run_report.json already exists in %s" % parent)
    return p, found


def committed_sha256(root=ROOT):
    return {"realveh_entry": sha(root / S2A.ENTRY), "cams2diag": sha(root / S2A.CAMS2DIAG_REL),
            "contactcode": sha(root / S2A.CONTACTCODE_REL), "inview": sha(root / INVIEW_REL)}


def recorder_equal(a_dir, b_dir):
    out = {}
    for name in (DIAG_FILE, INVIEW_FILE):
        try:
            out[name] = K.read_jsonl(a_dir / name) == K.read_jsonl(b_dir / name)
        except (OSError, ValueError):
            out[name] = False
    return out


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv == ["scope"]:
        print(json.dumps({"scope": SCOPE, "scope_sha256": scope_digest()}, indent=1))
        return 0
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("smoke", "identity", "eval", "check"))
    ap.add_argument("--approved-commit", default=None, help="required for every stage but check")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    if args.stage != "check" and not args.approved_commit:
        ap.error("--approved-commit is required for stage %s" % args.stage)
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    approved = args.approved_commit or K.git("rev-parse", "HEAD", root=ROOT).strip()
    head, problems, origins, approval_state = preflight(protocol, approved, out_root, args.stage)
    s2a_receipt, ft_receipt, ft_report, tp = training_inputs()
    problems += tp
    ck, pp = policy_problems(s2a_receipt, ft_receipt, ft_report)
    problems += pp
    changed = changed_since(FT_TRAIN_HEAD)
    problems += allowlist_problems(changed, CODE_ALLOWLIST, FT_TRAIN_HEAD)
    train_env = ft_receipt.get("env_navrl") or {}
    if not training_env_problems(train_env):
        problems += plan_env_problems(train_env, protocol, out_root)
    link, found = stage_link_problems(args.stage if args.stage != "check" else "smoke", out_root, head, ck[1])
    problems += link
    if args.stage == "check":                      # CPU only, read only: nothing is written
        print(json.dumps({"stage": "check", "unit": UNIT, "commit": head, "preflight_problems": problems,
                          "approval_state_for_gpu_stages": approval_state, "ft_checkpoint": ck,
                          "changed_since_ft_training": changed, "branch": K.git("rev-parse", "--abbrev-ref", "HEAD",
                                                                                 root=ROOT).strip()},
                         indent=1, default=str))
        return 0 if not problems else 2
    train_sha = sha(FT_TRAIN_RECEIPT) if FT_TRAIN_RECEIPT.is_file() else None
    mem_624 = "E128"
    if args.stage in ("identity", "eval") and "smoke_report.json" in found:
        mem_624 = found["smoke_report.json"][1].get("mem_624", "E128")
    start = G.host()
    report = {"unit": UNIT, "stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "scope": SCOPE,
              "scope_sha256": scope_digest(), "started": G.now(), "host_start": start, "import_origins": origins,
              "launcher": LAUNCHER, "launcher_sha256": LAUNCHER_SHA, "ft_train_head": FT_TRAIN_HEAD,
              "changed_since_ft_training": changed, "ft_train_receipt": str(FT_TRAIN_RECEIPT),
              "ft_train_receipt_sha256": train_sha,
              "ft_train_report_sha256": sha(FT_TRAIN_REPORT) if FT_TRAIN_REPORT.is_file() else None,
              "ft_checkpoint": ck[0], "ft_checkpoint_sha256": ck[1], "arms": ARMS, "density": DENSITY,
              "seeds": list(SEEDS), "max_hours": MAX_HOURS[args.stage], "cell_timeout_s": CELL_TIMEOUT_S,
              "mem_624_at_start": mem_624, "preflight_problems": problems,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if R1.ENV_ALLOW.match(k)},
              "linked_reports": {k: v[0] for k, v in found.items()}}
    if args.stage == "eval" and "identity_report.json" in found:
        irep = found["identity_report.json"][1]
        if irep.get("env_allowlisted") != report["env_allowlisted"]:
            problems.append("runner environment differs from the identity stage's")
        for key in ("ft_train_receipt_sha256", "ft_train_report_sha256"):
            if irep.get(key) != report[key]:
                problems.append("%s changed since the identity stage" % key)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    committed = committed_sha256()
    report["committed_sha256"] = committed
    ctx = {"protocol": protocol, "head": head, "start": start, "committed": committed, "train": (ft_receipt, train_sha),
           "ckpt": ck}
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(MAX_HOURS[args.stage])
    records, void, oom_events = [], False, []

    def timeout(arm):
        return min(float(CELL_TIMEOUT_S[res(arm)]), max(0.0, budget.remaining_s()))

    def run_one(arm, d, s, stage):
        """One cell with the memory fallback; returns the final record or None when the 624 arms are NOT_RUN_OOM."""
        nonlocal mem_624
        if res(arm) == 624 and mem_624 == "NOT_RUN_OOM":
            return None
        n, q = mem_for(arm, mem_624, stage)
        rec = run_cell(ctx, out_root, arm, d, s, records, timeout(arm), stage, n, q)
        if rec["verdict"] == "OOM":
            oom_events.append({"arm": arm, "seed": s, "num_envs": n, "cell_dir": rec["cell_dir"], "time": G.now()})
            if n != NUM_ENVS:                                  # out of memory at 64 envs too
                mem_624 = "NOT_RUN_OOM"
                return None
            mem_624 = "E64"
            if budget.exhausted():
                return None
            n, q = mem_for(arm, mem_624, stage)
            rec = run_cell(ctx, out_root, arm, d, s, records, timeout(arm), stage, n, q)
            if rec["verdict"] == "OOM":
                oom_events.append({"arm": arm, "seed": s, "num_envs": n, "cell_dir": rec["cell_dir"], "time": G.now()})
                mem_624 = "NOT_RUN_OOM"
                return None
        return rec

    def stopped(rec):
        return rec is not None and rec["verdict"] in ("FAIL", "VOID_HOST_FAULT")

    if args.stage == "smoke":
        for arm, d, s in SMOKE_PLAN:
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            rec = run_one(arm, d, s, "SMOKE")
            if stopped(rec):
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        if mem_624 == "NOT_RUN_OOM" and not void and not budget.exhausted():
            rec = run_one(*SMOKE_V312, "SMOKE")
            void = bool(rec and rec["verdict"] == "VOID_HOST_FAULT")
        rows = {}
        for r in records:
            if r["verdict"] == "OK":
                rows[r["base_arm"]] = K.read_jsonl(pathlib.Path(r["cell_dir"]) / INVIEW_FILE)
        v_rows = rows.get("V624") if mem_624 != "NOT_RUN_OOM" else rows.get("V312")
        vs = visible_start_control(v_rows or [], rows.get("R312") or [])
        dc = step0_detection_control(v_rows or [], rows.get("R312") or [])
        px = (pixel_control(rows.get("V624") or [], rows.get("R312") or []) if mem_624 != "NOT_RUN_OOM"
              else {"verdict": "NOT_RUN_OOM"})
        report.update(quota=SMOKE_QUOTA, visible_start_control=vs, step0_detection_control=dc, smoke_624_control=px,
                      mem_624=mem_624, oom_events=oom_events)
        report["verdict"] = "VOID_HOST_FAULT" if void else smoke_verdict(records, vs, px, mem_624, dc)
        name = "smoke_report.json"
    elif args.stage == "identity":
        for arm, d, s in IDENTITY_PLAN:
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            rec = run_one(arm, d, s, "IDENTITY")
            if stopped(rec):
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        comp = None
        if len(records) == 2 and all(r["verdict"] == "OK" for r in records):
            a, b = (pathlib.Path(r["cell_dir"]) for r in records)
            comp = dict(G.identity_compare(a, b), recorders_equal=recorder_equal(a, b), a=str(a), b=str(b))
        report.update(identity_comparison=comp, mem_624=mem_624)
        report["verdict"] = "VOID_HOST_FAULT" if void else identity_verdict(records, comp)
        name = "identity_report.json"
    else:
        plan = list(eval_plan())
        done624 = []
        i = 0
        while i < len(plan):
            arm, d, s = plan[i]
            i += 1
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            n_cell, _ = mem_for(arm, mem_624, "eval")
            proj = projected_cell_s(arm, n_cell, records, (found.get("smoke_report.json") or (None, {}))[1],
                                    (found.get("identity_report.json") or (None, {}))[1])
            if res(arm) == 624 and mem_624 == "NOT_RUN_OOM":
                pass
            elif proj is not None and proj > budget.remaining_s():          # review r1 P2-4: never start a cell the cap cuts
                records.append({"arm": arm, "base_arm": arm, "stage": "EVAL", "density": d, "seed": s, "num_envs": n_cell,
                                "verdict": "BUDGET_EXHAUSTED", "projected_s": proj, "remaining_s": budget.remaining_s()})
                report["budget_exhausted"] = True
                break
            before = mem_624
            rec = run_one(arm, d, s, "EVAL")
            if before == "E128" and mem_624 == "E64":             # switched: rerun the 624 cells already run at 128
                plan += [c for c in done624]
                done624 = []
            if rec is not None and res(arm) == 624 and rec["num_envs"] == NUM_ENVS and rec["verdict"] == "OK":
                done624.append((arm, d, s))
            if stopped(rec):
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        report.update(mem_624=mem_624, oom_events=oom_events, cells_planned=len(eval_plan()))
        ident = found.get("identity_report.json")
        ev = next((r for r in records if r["arm"] == "R312" and r["seed"] == 4501 and r["verdict"] == "OK"), None)
        if ident and ev and (ident[1].get("identity_comparison") or {}).get("a"):
            a = pathlib.Path(ident[1]["identity_comparison"]["a"])          # reported, not gated
            report["eval_r312_4501_vs_identity"] = dict(G.identity_compare(a, pathlib.Path(ev["cell_dir"])),
                                                        recorders_equal=recorder_equal(a, pathlib.Path(ev["cell_dir"])))
        report["verdict"] = "VOID_HOST_FAULT" if void else eval_verdict(records, mem_624)
        name = "eval_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    if any(report["host_end"][k] != start[k] for k in ("fault_count", "boot_id", "online")):
        report["verdict"] = "VOID_HOST_FAULT"
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit", "mem_624")}, indent=1))
    return 0 if report["verdict"] in ("SMOKE_OK", "IDENTITY_PASS", "COMPLETE", "COMPLETE_624_NOT_RUN_OOM") else 1


if __name__ == "__main__":
    raise SystemExit(main())
