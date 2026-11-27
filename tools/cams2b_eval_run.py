"""Runner of CAM-S2b-FT-E, the evaluation of the warm-started CAM-S2b policy "S2b-FT" (trained at the real vehicle's
60 deg/s yaw-rate limit, NAVRL_YAW_RATE_MAX = 1.0472 rad/s) against the CAM-S2a ep12000 policy clipped at 1.047 rad/s
(docs/plans/prereg_camonly_s2b_ft_eval_2026-11-24.md).

It generalises the committed CAM-S2a-E runner (tools/cams2a_eval_run.py, bound by SHA in the CAM-S2a-E evaluation and
therefore NOT edited) by a per-arm configuration and imports its pure helpers (module seeds, the F2 training-environment
check, the result-condition check, the MISMATCH scan, the telemetry parser, the recorder checks, the launcher pin). The
launcher is the committed CAM-S2a-E copy, aerial_gym/rl_training/rl_games/eval_navrl_cams2a_arms.sh, at its pinned SHA:
it is generic in the checkpoint and the training-receipt SHA, so no new launcher is needed.

Arms (ARMS; each arm = policy x environment x yaw clip x recorder yaw reference):
  FT    S2b-FT's final checkpoint (path and SHA from the training run report) in ITS training environment (the NAVRL_*
        keys of ~/motar_tmp/cams2b_20261124/train_ft/receipt.json; yaw scale 1.0472), no clip; recorder reference 0.7854.
  FT45  the same with the yaw command clipped at 0.7854 rad/s (45 deg/s; navrl_yawclip, action scale 1.0472 unchanged).
  A60   CAM-S2a ep12000 (9fc9dfb7...) in the CAM-S2a training environment (yaw scale 3.0) with the yaw command clipped at
        1.047 rad/s; recorder reference 1.047: byte-for-byte CAM-S2a-E's YC12k arm.
A60 source: CAM-S2a-E's YC12k cells at 175/205 bars x seeds 4501-4503 (~/motar_tmp/cams2a_eval_20261124/raw_a1/eval_01/
YC12k/) are REUSED, bound by path and SHA-256, iff (a) CAM-S2a-E's identity report is IDENTITY_PASS and its eval report is
not VOID_HOST_FAULT / PREFLIGHT_FAIL with those six records OK at commit S2A_E_COMMIT, (b) every reused cell passes this
runner's checks again (as an A60 cell of tree S2A_E_TREE at S2A_E_COMMIT), and (c) the identity cell A60_repro, run here at
175 bars / seed 4501, equals the reused cell exactly (result.json without nonce and checkpoint path, every ledger row and
every recorder row). Otherwise the identity report reads a60_source = RERUN_REQUIRED and the optional stage `a60` reruns
the six A60 cells here; until it has run, gate G1 is not answered.
Stages (each needs the approval file and the stage before it, on the same commit, in the same parent folder):
  smoke     FT_smoke, FT45_smoke at 205 bars / seed 4501, quota 1, never counted; cap 0.25 h.
  identity  FT_identity (205/4501; must equal eval FT there: determinism of the FT path; its wall time sets the rate gate),
            then A60_repro (175/4501; the reuse proof); cap 0.4 h. IDENTITY_PASS needs FT_identity OK and A60_repro run;
            a FAILed A60_repro only sets a60_source = A60_PATH_FAIL (G1 not answerable, G2 unaffected). The clip acting is
            shown by the smoke (FT45_smoke's whole-cell clipped share > 0) and in every FT45 eval cell (requested yaw over
            the clip > 0, executed = 0).
  eval      FT, FT45 x densities 175/205 x seeds 4501-4503, block-major, quota 8 (12 cells); cap 2.5 h with a rate gate
            (1.10 x (6 x 0.90 + 6) x the FT_identity wall at 205 bars must fit the cap).
  a60       only when the identity report reads a60_source RERUN_REQUIRED (the reuse inputs were refused, the A60 path
            ran OK here) and the eval is COMPLETE: A60 on the 6 blocks; cap 1.25 h. Not when the reproduction differed
            (A60_NOT_REPRODUCED: the determinism premise failed; an amendment decides).
  check     CPU only, writes nothing, needs no approval: prints the preflight problems and the reuse problems (run it
            before the night and again before the identity stage).
Cell checks (every cell; a FAIL stops the stage): launcher receipt (OK, this commit, 0 dirty files, the checkpoint SHA, the
arm's training-receipt SHA, base_sim, the interpreter, GPU4GB unset); ledger complete; result condition (CAM-S2a-E's
expectation, P9 SHA of the arm's training environment); realveh receipt against the arm's training receipt (module
settings with the cell's seeds, hashed modules byte-identical except realveh_entry = HEAD's, tree-relative ablation paths,
yaw clip iff the arm clips with limit and scale, recorder with the arm's reference, contact code); both NAVRL snapshots
equal to the arm's training environment except the per-cell keys; telemetry; recorder rows aligned with the ledger;
executed yaw never over the clip; every MISMATCH line inside CAM-S2a-E's allowlist. Host fault count, boot id and online
CPU list unchanged.
Fail-closed preflight: approval naming SCOPE (and its SHA-256) and HEAD; clean tree; the branch S2b-FT trained on
(camonly/dev); committed files;
the launcher and play script at their pins; `git diff --name-only S2A_E_COMMIT HEAD` inside S2A_CODE_ALLOWLIST (binds what
the A60 cells execute to CAM-S2a-E's commit, itself bound to CAM-S2a's training commit); `git diff --name-only <FT training
commit> HEAD` inside FT_CODE_ALLOWLIST; CAM-S2a COMPLETED_PROCESS and S2b-FT TRAINED (warm-started from ep12000) with
faults unchanged; the FT training
environment differs from CAM-S2a's only in FT_ENV_MAY_DIFFER with NAVRL_YAW_RATE_MAX = 1.0472; both checkpoints at their
SHA; one RTX 3070; no other GPU compute; import origins inside the tree; free disk >= 1 GB; new output folder.
usage: tools/cams2b_eval_run.py {smoke,identity,eval,a60} --approved-commit SHA --out DIR
       tools/cams2b_eval_run.py check --out DIR    (CPU-only dry run: preflight + reuse problems; writes nothing)
       tools/cams2b_eval_run.py scope          (prints SCOPE and its SHA-256 for the approval file)
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

PREREG = "docs/plans/prereg_camonly_s2b_ft_eval_2026-11-24.md"
UNIT = "CAM-S2b-FT-E"
SCOPE = ("CAM-S2b-FT-E v1: GPU evaluation of the warm-started CAM-S2b policy S2b-FT (final checkpoint of the training run "
         "~/motar_tmp/cams2b_20261124/train_ft, NAVRL_YAW_RATE_MAX 1.0472 rad/s) in its own training environment, unclipped "
         "(FT) and with the yaw command clipped at 0.7854 rad/s (FT45), at fixed densities 175 and 205 bars x seeds "
         "4501-4503, quota 8 per env (1,024 episodes per cell), against CAM-S2a ep12000 clipped at 1.047 rad/s in the CAM-S2a "
         "training environment (A60 = CAM-S2a-E's YC12k cells, reused by path and SHA-256 after one reproduction cell); "
         "stages smoke (2 cells, quota 1, 0.25 h cap), identity (2 cells, 0.4 h cap), eval (12 cells, 2.5 h cap with a "
         "rate gate) and, only if the reuse is refused, a60 (6 cells, 1.25 h cap); idle host; tools/cams2b_eval_run.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "cams2b_eval_20261124"
BRANCHES = ("camonly/dev",)        # tools/cams2b_train.py trains only on camonly/dev; the eval runs on S2b-FT's branch

# ---- CAM-S2a (the A60 arm's policy and environment) and CAM-S2a-E (the reused cells) --------------------------------
S2A_TRAIN_RECEIPT = S2A.TRAIN_RECEIPT
S2A_TRAIN_REPORT = S2A.TRAIN_REPORT
S2A_CKPT_PATH = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly/aerial_gym/rl_training/rl_games/runs/"
                             "ppo_261123_2213_navrl_v2-cams2a-scratch-camonly-s1/nn/last_gen_ppo_ep_12000_rew_104.0523.pth")
S2A_CKPT_SHA = S2A.CKPT["S12k"][1]                          # 9fc9dfb7..., also the warm-start source of S2b-FT
S2A_E_COMMIT = "3a453e69811005aaf2a5bf0ff4849bc62ac47d77"   # CAM-S2a-E amendment 1: the commit its cells ran at
S2A_E_TREE = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly")   # read only, never written
S2A_E_RAW = pathlib.Path.home() / "motar_tmp" / "cams2a_eval_20261124" / "raw_a1"
S2A_E_ARM = "YC12k"
S2A_E_CELL = S2A.CELL                                       # "S2A_trained__%dbars__seed%d"
REUSED_FILES = ("result.json", "episodes.jsonl", "episode_ledger_summary.json", S2A.DIAG_FILE, "receipt.json",
                "realveh_receipt.json", "realveh_receipt.json.veh_rand_init.json", "cell.log")
S2A_E_REPORTS = ("identity_01/identity_report.json", "eval_01/eval_run_report.json")      # bound with the cells (P3-1)

# ---- S2b-FT (the FT arms) --------------------------------------------------------------------------------------------
FT_TRAIN_DIR = pathlib.Path.home() / "motar_tmp" / "cams2b_20261124" / "train_ft"
FT_TRAIN_RECEIPT = FT_TRAIN_DIR / "receipt.json"
FT_TRAIN_REPORT = FT_TRAIN_DIR / "run_report.json"
FT_TRAIN_PREREG = "docs/plans/prereg_camonly_s2b_2026-11-24.md"
FT_TRAIN_UNIT, FT_TRAIN_RUN, FT_TRAIN_VERDICT = "CAM-S2b", "ft", "TRAINED"      # tools/cams2b_train.py's run report
FT_YAW_RATE_MAX = 1.0472                                    # 60 deg/s, the real vehicle's limit; FT's action scale
FT_CKPT_NAME = re.compile(r"last_gen_ppo_ep_\d+_rew_[-0-9.]+\.pth")      # never gen_ppo.pth (best-reward, low density)
# The FT training environment may differ from CAM-S2a's only in these keys (tools/cams2b_train.expected_env: the yaw
# scale; the warm-start keys NAVRL_V2_ALLOW_RESUME (read by the training launcher only), NAVRL_LEARNING_RATE (PPO only)
# and NAVRL_RESET_DENSITY_WINDOW / NAVRL_DENSITY_RESUME_WARMUP (density-curriculum evidence; the evaluation runs with the
# curriculum off); the curriculum keys NAVRL_DENSITY_*; the receipt path; the P9 model path, which may move with the tree
# but must name the same tree-relative file (its SHA, NAVRL_P9_EXPECTED_SHA256, must be equal); the seeds, which every
# cell replaces). Anything else refuses the run until a dated amendment lists it and says why the cells are unchanged.
FT_ENV_MAY_DIFFER_KEYS = ({"NAVRL_YAW_RATE_MAX", "NAVRL_SEED", "NAVRL_REALVEH_RECEIPT", "NAVRL_SAVE_FREQUENCY",
                           "NAVRL_V2_ALLOW_RESUME", "NAVRL_LEARNING_RATE", "NAVRL_RESET_DENSITY_WINDOW", "NAVRL_P9_ERROR_MODEL"}
                          | set(S2A.MODULE_SEED_KEYS))
FT_ENV_MAY_DIFFER_PREFIXES = ("NAVRL_DENSITY_",)            # the density curriculum (175 -> 205); evaluation fixes the density
P9_MODEL_REL = "results/perception_p9_2026-09-09/p9_error_injector.json"
FT_SETTINGS_SEED_KEYS = {"cams2": "NAVRL_CAMS2_DELAY_SEED", "vehicle_rand": "NAVRL_VEH_RAND_SEED", "depth_model": "NAVRL_DEPTH_SEED"}

# ---- the arms --------------------------------------------------------------------------------------------------------
FT45_RADPS = "0.7854"                                       # 45 deg/s
A60_RADPS = S2A.YAWCLIP_RADPS                               # "1.047", byte-for-byte CAM-S2a-E's YC12k
ARMS = {"FT": {"policy": "FT", "env": "FT", "yawclip": None, "diag_ref": FT45_RADPS, "label": "S2B_FT"},
        "FT45": {"policy": "FT", "env": "FT", "yawclip": FT45_RADPS, "diag_ref": FT45_RADPS, "label": "S2B_FT"},
        "A60": {"policy": "S12k", "env": "S2A", "yawclip": A60_RADPS, "diag_ref": A60_RADPS, "label": "S2A_trained"}}
ARM_BASE = {"FT_smoke": "FT", "FT45_smoke": "FT45", "FT_identity": "FT", "A60_repro": "A60"}
SMOKE_PLAN = (("FT_smoke", 205, 4501), ("FT45_smoke", 205, 4501))
SMOKE_QUOTA = 1
REPRO_BLOCK = (175, 4501)                                   # the reused CAM-S2a-E YC12k cell the identity stage reproduces
IDENTITY_PLAN = (("FT_identity", 205, 4501), ("A60_repro",) + REPRO_BLOCK)   # FT first: G2 never waits on the A60 path
EVAL_ARMS = ("FT", "FT45")
DENSITIES = (175, 205)                                      # 175 = CAM-S2a's final density; 205 = S2b-FT's final density
SEEDS = S2A.SEEDS                                           # 4501-4503
QUOTA = S2A.QUOTA                                           # 8 per env, 1,024 per cell
NUM_ENVS = S2A.NUM_ENVS
MAX_HOURS = {"smoke": 0.25, "identity": 0.4, "eval": 2.5, "a60": 1.25}
# Rate-gate density factors fixed from CAM-S2a-E's cells (receipts' duration_s, review r1 P2-2 / P3-5): unclipped ep12000
# 175 vs 205 bars (374, 411 s vs 434 s) -> 0.90; clipped YC12k 205 vs 175 bars (579 s vs 332, 362 s) -> 1.67.
RATE_175_OVER_205 = 0.90
RATE_205_OVER_175_CLIPPED = 1.67
RATE_MARGIN = 1.10
CELL_TIMEOUT_S = 2400
MIN_FREE_GB = 1.0
LAUNCHER = S2A.LAUNCHER                                     # the committed CAM-S2a-E launcher copy, generic in the checkpoint
LAUNCHER_SHA = S2A.LAUNCHER_SHA
NEW_FILES = ("tools/cams2b_eval_run.py", "tools/cams2b_eval_analyse.py", "tests/test_cams2b_eval.py")
COMMITTED_FILES = (PREREG, FT_TRAIN_PREREG, S2A.PREREG, S2A.AMENDMENT, S2A.ENTRY, "tools/realveh_python", "tools/cams2a_eval_run.py",
                   S2A.YAWCLIP_REL, S2A.CAMS2DIAG_REL, S2A.CONTACTCODE_REL, LAUNCHER) + NEW_FILES
# Paths that may change after CAM-S2a-E's commit without changing what an A60 cell executes: records, tests, this
# package, the S2b training package (its launcher is a training script no cell runs) and the flight runtime (never
# imported by a cell). Everything else refuses every stage (an amendment decides).
S2A_CODE_ALLOWLIST = ("docs/*", "WORKLOG.md", "tests/*", "tools/cams2b_*", "tools/camera_policy_*",
                      "aerial_gym/rl_training/rl_games/train_navrl_cams2b.sh")
FT_CODE_ALLOWLIST = ("docs/*", "WORKLOG.md", "tests/*", "tools/cams2b_eval_*", "tools/camera_policy_*")
MISMATCH_ALLOWLIST = S2A.MISMATCH_ALLOWLIST
DIAG_FILE = S2A.DIAG_FILE
SNAPSHOT_EXPECTED = S2A.SNAPSHOT_EXPECTED
UNCONSTRAINED_IN_SNAPSHOT = S2A.UNCONSTRAINED_IN_SNAPSHOT
RECEIPT_SHA_MAY_DIFFER = S2A.RECEIPT_SHA_MAY_DIFFER


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def cfg(arm):
    return ARMS[ARM_BASE.get(arm, arm)]


def cell_id(arm, density, seed):
    return "%s__%dbars__seed%d" % (cfg(arm)["label"], int(density), int(seed))


def scope_digest():
    return hashlib.sha256(SCOPE.encode("utf-8")).hexdigest()


# ---------------------------------------------------------------- environments
def ft_env_problems(ft_env, s2a_env, ft_tree=None, s2a_tree=None):
    """The FT training environment is CAM-S2a's except FT_ENV_MAY_DIFFER and carries the 1.0472 rad/s yaw scale; the P9
    model is the same tree-relative file."""
    p = list(S2A.training_env_problems(ft_env))
    for env, tree, name in ((ft_env, ft_tree, "FT"), (s2a_env, s2a_tree, "CAM-S2a")):
        if _rel(env.get("NAVRL_P9_ERROR_MODEL"), tree) != P9_MODEL_REL:
            p.append("%s training NAVRL_P9_ERROR_MODEL %r is not %s inside %s" % (name, env.get("NAVRL_P9_ERROR_MODEL"),
                                                                                P9_MODEL_REL, tree))
    diff = sorted(k for k in set(ft_env) | set(s2a_env) if ft_env.get(k) != s2a_env.get(k)
                  and k not in FT_ENV_MAY_DIFFER_KEYS and not k.startswith(FT_ENV_MAY_DIFFER_PREFIXES))
    if diff:
        p.append("FT training environment differs from CAM-S2a's outside the allowlist: %s" % diff)
    try:
        yaw = float(ft_env.get("NAVRL_YAW_RATE_MAX", "nan"))
    except ValueError:
        yaw = float("nan")
    if not (math.isfinite(yaw) and abs(yaw - FT_YAW_RATE_MAX) < 1e-9):
        p.append("FT training NAVRL_YAW_RATE_MAX is %r, expected %s" % (ft_env.get("NAVRL_YAW_RATE_MAX"), FT_YAW_RATE_MAX))
    if s2a_env.get("NAVRL_YAW_RATE_MAX") != "3.0":
        p.append("CAM-S2a training NAVRL_YAW_RATE_MAX is %r, expected '3.0'" % s2a_env.get("NAVRL_YAW_RATE_MAX"))
    return p


def cell_navrl_env(train_env, seed, arm, quota=QUOTA, player_games_cap=None):
    """The NAVRL_* variables of one cell: the arm's training environment with CAM-S2a-E's per-cell replacements, the
    recorder at the arm's yaw reference and the arm's yaw clip. Raises ValueError when the environment lacks the cams2
    rules (training review F2)."""
    problems = S2A.training_env_problems(train_env)
    if problems:
        raise ValueError("; ".join(problems))
    c = cfg(arm)
    env = {k: v for k, v in train_env.items() if k.startswith("NAVRL_") and k not in S2A.REMOVED_FROM_RECEIPT}
    env.update(S2A.EVAL_OVERRIDES)
    env.update(S2A.module_seeds(seed))
    env.update(NAVRL_EPISODE_LEDGER="1", NAVRL_EPISODE_QUOTA_PER_ENV=str(int(quota)),
               NAVRL_PLAYER_GAMES_CAP=str(int(player_games_cap if player_games_cap is not None else 10 ** 9)),
               NAVRL_CAMS2_CONTACTCODE="1", NAVRL_CAMS2DIAG="1", NAVRL_CAMS2DIAG_YAW_REF_RADPS=c["diag_ref"])
    if c["yawclip"]:
        env.update(NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS=c["yawclip"])
    return env


def cell_env(root, out_dir, arm, density, seed, ckpt, ckpt_sha, train_env, train_receipt_sha, protocol, quota=QUOTA):
    inherited = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_") and k != "GPU4GB"}
    env = K.base_env(root=root, inherited=inherited)
    env.update(cell_navrl_env(train_env, seed, arm, quota, protocol["evaluation"]["player_games_cap"]))
    env.update(ARMS=cfg(arm)["label"], DENSITIES=str(int(density)), SEEDS=str(int(seed)), GAMES=str(quota * NUM_ENVS),
               NUM_ENVS=str(NUM_ENVS), OUT_ROOT=str(out_dir), CAMEVAL_PY=str(S2A.WRAPPER), CAMS2A_CKPT=str(ckpt),
               CAMS2A_CKPT_SHA=ckpt_sha, CAMS2A_TRAIN_RECEIPT_SHA256=train_receipt_sha,
               AERIAL_GYM_SIM_NAME=S2A.SIM_NAME, PYTHON=S2A.REAL_PYTHON)                 # pinned, not inherited
    return env


def expected_snapshot_values(arm, density, seed, cell_dir, ckpt_sha, quota=QUOTA):
    c = cfg(arm)
    want = {"NAVRL_DENSITY_CURRICULUM": "0", "NAVRL_NUM_BARS": str(density), "NAVRL_DENSITY_START": str(density),
            "NAVRL_DENSITY_FINAL": str(density), "NAVRL_SEED": str(seed), "NAVRL_BULK_EVAL": "1", "NAVRL_EPISODE_LEDGER": "1",
            "NAVRL_EPISODE_QUOTA_PER_ENV": str(quota), "NAVRL_PLAYER_GAMES_CAP": str(10 ** 9), "NAVRL_CAMS2_CONTACTCODE": "1",
            "NAVRL_EVAL_CHECKPOINT": ckpt_sha, "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"),
            "NAVRL_REALVEH_RECEIPT": str(cell_dir / "realveh_receipt.json"), "NAVRL_CAMS2DIAG": "1",
            "NAVRL_CAMS2DIAG_YAW_REF_RADPS": c["diag_ref"], "NAVRL_CAMS2DIAG_JSONL": str(cell_dir / DIAG_FILE),
            **S2A.module_seeds(seed)}
    if c["yawclip"]:
        want.update(NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS=c["yawclip"])
    return want


def snapshot_problems(snap, train_env, arm, density, seed, cell_dir, ckpt_sha, label="snapshot", quota=QUOTA):
    """`snap` equals the arm's training environment except SNAPSHOT_EXPECTED; every expected key has the cell's value; no
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
def tree_of(receipt):
    """The tree a realveh receipt was written from (its entry is <tree>/tools/realveh_entry.py)."""
    entry = (receipt or {}).get("entry") or ""
    return pathlib.Path(entry).resolve().parents[1] if entry else None


def _rel(path, root):
    if not path or not root or not os.path.isabs(str(path)):
        return None
    try:
        return str(pathlib.Path(path).resolve().relative_to(pathlib.Path(root).resolve()))
    except (ValueError, TypeError):
        return None


def ablation_problems(rec, train_receipt, root):
    """The ablation record with its module paths made tree-relative (a cell of this worktree against a receipt written in
    another worktree), the paths inside the cell's tree."""
    a, t = dict(rec.get("ablation") or {}), dict(train_receipt.get("ablation") or {})
    p = []
    troot = tree_of(train_receipt)
    for k in ("perception", "task"):
        here, there = _rel(a.get(k, ""), root), (_rel(t.get(k, ""), troot) if troot else None)
        if here is None or here != there:
            p.append("ablation %s %r is not the training module %r inside %s" % (k, a.get(k), t.get(k), root))
        a.pop(k, None)
        t.pop(k, None)
    if a != t:
        p.append("ablation %r != training %r" % (a, t))
    return p


def receipt_problems(cell_dir, arm, density, seed, train_receipt, train_receipt_sha, committed, ckpt_sha, quota=QUOTA,
                     root=ROOT):
    """The realveh receipt, both NAVRL snapshots and the telemetry of one cell against its arm's training receipt.
    `committed` maps realveh_entry / yawclip / cams2diag / contactcode to their SHA-256 at the cell's commit; `root` is
    the cell's tree (this worktree, or CAM-S2a-E's for a reused cell)."""
    try:
        rec = json.loads((cell_dir / "realveh_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["realveh receipt or cell receipt missing or unreadable: %s" % e]
    c = cfg(arm)
    train_env = train_receipt.get("env_navrl") or {}
    p = []
    ts, s = train_receipt.get("settings") or {}, rec.get("settings") or {}
    seeds = S2A.module_seeds(seed)
    for key, seed_key in FT_SETTINGS_SEED_KEYS.items():
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
    p += ablation_problems(rec, train_receipt, root)
    shas, tshas = rec.get("sha256") or {}, train_receipt.get("sha256") or {}
    if not tshas or set(shas) != set(tshas) or any(shas[k] != v for k, v in tshas.items() if k not in RECEIPT_SHA_MAY_DIFFER):
        p.append("receipt SHA-256 of the hashed modules differ from the training receipt")
    if shas.get("realveh_entry") != committed.get("realveh_entry"):
        p.append("receipt realveh_entry SHA is not the cell commit's")
    for k in ("entry", "script"):
        if not K.is_inside(rec.get(k, ""), root):
            p.append("realveh receipt %s outside the tree %s: %r" % (k, root, rec.get(k)))
    clip = c["yawclip"]
    scale = float(train_env.get("NAVRL_YAW_RATE_MAX", "nan"))
    want_y = {"NAVRL_YAWCLIP_RADPS": float(clip)} if clip else None
    yrec = rec.get("yawclip") or {}
    if (s.get("yawclip") != want_y or ("yawclip" in rec) != bool(clip)
            or (clip and (rec.get("yawclip_sha256") != committed.get("yawclip") or yrec.get("limit_radps") != float(clip)
                          or yrec.get("scale_radps") != scale))):
        p.append("yaw-clip receipt state %r / %r does not match arm %s" % (s.get("yawclip"), rec.get("yawclip"), arm))
    if ((rec.get("cams2_contactcode") or {}).get("contact_code") != 0 or s.get("cams2_contactcode") != {}
            or rec.get("cams2_contactcode_sha256") != committed.get("contactcode")):
        p.append("cams2 contact-code install record missing or not the committed module")
    d = rec.get("cams2diag") or {}
    if (d.get("jsonl") != str(cell_dir / DIAG_FILE) or d.get("yaw_ref_radps") != float(c["diag_ref"])
            or rec.get("cams2diag_sha256") != committed.get("cams2diag")):
        p.append("cams2diag install record missing, elsewhere, wrong yaw reference or not the committed module")
    p += snapshot_problems(receipt.get("env_snapshot_navrl", {}), train_env, arm, density, seed, cell_dir, ckpt_sha,
                           label="launcher snapshot", quota=quota)
    p += snapshot_problems(rec.get("env_navrl") or {}, train_env, arm, density, seed, cell_dir, ckpt_sha,
                           label="entry env_navrl", quota=quota)
    if receipt.get("train_receipt_sha256") != train_receipt_sha:
        p.append("cell receipt does not name the arm's training receipt SHA")
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    if S2A.parse_kv_line(text, S2A.CAMSAFE_PREFIX) is None:
        p.append("no [camsafe] telemetry line")
    if S2A.CONTACT_PREFIX + " {" not in text:
        p.append("no [cams2contact] telemetry line")
    yline = S2A.parse_kv_line(text, S2A.YAWCLIP_PREFIX)
    if clip:
        if yline is None or yline.get("limit_radps") != float(clip) or yline.get("scale_radps") != scale:
            p.append("yaw clip ON, but no [yawclip] line with limit %s and scale %s" % (clip, scale))
    elif yline is not None:
        p.append("yaw clip OFF, but [yawclip] telemetry appears")
    if S2A.DIAG_PREFIX + " {" not in text:
        p.append("no [cams2diag] summary line")
    p += S2A.diag_rows_problems(cell_dir)
    if clip:
        p += S2A.yc_exec_yaw_problems(cell_dir)
    p += S2A.mismatch_problems(text)
    return p


def output_problems(cell_dir, commit, ckpt_sha, train_receipt_sha, train_env, quota=QUOTA):
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
    p += S2A.condition_problems(result.get("condition") or {}, train_env)
    p += K.ledger_problems(summary, rows, NUM_ENVS, quota)
    quota_rows = [r for r in rows if r.get("in_quota")]
    counts = {o: sum(1 for r in quota_rows if r.get("outcome") == o) for o in ("capture", "crash", "timeout")}
    return p, {"quota_episodes": len(quota_rows), "counts": counts, "duration_s": receipt.get("duration_s"),
               "result_sha256": receipt.get("result_sha256")}


def check_cell(cell_dir, head, start_host, ckpt_sha, train_receipt_sha, train_env, quota=QUOTA, root=ROOT):
    problems = []
    rec_host = G.host()
    for k in ("fault_count", "boot_id", "online"):
        if rec_host[k] != start_host[k]:
            problems.append("HOST " + k + " changed")
    p, stats = output_problems(cell_dir, head, ckpt_sha, train_receipt_sha, train_env, quota)
    problems += p
    if K.git("status", "--porcelain", root=root).strip():
        problems.append("tree became dirty")
    return problems, rec_host, stats


# ---------------------------------------------------------------- checkpoints and training runs
def ft_checkpoint(report):
    """S2b-FT's final checkpoint from its training run report (tools/cams2b_train.py): run_folder/nn/<final_checkpoint.file>
    at final_checkpoint.sha256, epoch = max_epochs. Returns ((path, sha), problems)."""
    report = report if isinstance(report, dict) else {}
    fc = report.get("final_checkpoint")
    if not isinstance(fc, dict) or not fc.get("file") or not fc.get("sha256") or not report.get("run_folder"):
        return (None, None), ["training run report has no final_checkpoint {epoch, file, sha256} and run_folder"]
    path, pin = pathlib.Path(report["run_folder"]) / "nn" / str(fc["file"]), str(fc["sha256"])
    p = []
    if fc.get("epoch") is None or fc.get("epoch") != report.get("max_epochs"):
        p.append("FT final checkpoint epoch %r is not the run's max_epochs %r" % (fc.get("epoch"), report.get("max_epochs")))
    if not re.fullmatch(r"[0-9a-f]{64}", pin):
        p.append("FT checkpoint pin is not a SHA-256")
    if not FT_CKPT_NAME.fullmatch(path.name):
        p.append("FT checkpoint %s is not a last_gen_ppo_ep_*_rew_*.pth" % path.name)
    if not path.is_absolute() or not path.is_file() or sha(path) != pin:
        p.append("FT checkpoint missing or SHA mismatch: %s" % path)
    if pin == S2A_CKPT_SHA:
        p.append("FT checkpoint is the warm-start source itself")
    return (str(path), pin), p


def s2a_checkpoint():
    p = []
    if not S2A_CKPT_PATH.is_file() or sha(S2A_CKPT_PATH) != S2A_CKPT_SHA:
        p.append("CAM-S2a ep12000 missing or SHA mismatch: %s" % S2A_CKPT_PATH)
    if S2A_CKPT_PATH.name != S2A.CKPT["S12k"][0]:
        p.append("CAM-S2a ep12000 file name is not %s" % S2A.CKPT["S12k"][0])
    return (str(S2A_CKPT_PATH), S2A_CKPT_SHA), p


def ft_training_problems(ft_receipt, ft_report, s2a_receipt):
    """S2b-FT's training run: completed, fault-free, its own preregistration and commit, its receipt naming itself, an
    environment that is CAM-S2a's except the allowlist, the same module settings (seeds aside) and module bytes."""
    p = []
    if (ft_report.get("verdict") != FT_TRAIN_VERDICT or ft_report.get("mode") != "run" or ft_report.get("unit") != FT_TRAIN_UNIT
            or ft_report.get("run") != FT_TRAIN_RUN):
        p.append("FT training run_report unit/run/mode/verdict %r/%r/%r/%r, expected %s/%s/run/%s" % (
            ft_report.get("unit"), ft_report.get("run"), ft_report.get("mode"), ft_report.get("verdict"), FT_TRAIN_UNIT,
            FT_TRAIN_RUN, FT_TRAIN_VERDICT))
    if (ft_report.get("source_checkpoint") or {}).get("sha256") != S2A_CKPT_SHA:
        p.append("FT training did not warm-start from CAM-S2a ep12000 (%s)" % S2A_CKPT_SHA[:12])
    if (ft_report.get("preflight") or {}).get("branch") not in BRANCHES:
        p.append("FT training branch %r is not one of %s" % ((ft_report.get("preflight") or {}).get("branch"), BRANCHES))
    if ft_report.get("faults_before") is None or ft_report.get("faults_before") != ft_report.get("faults_after"):
        p.append("FT training host fault count missing or changed during the run")
    if ft_report.get("preregistration") != FT_TRAIN_PREREG:
        p.append("FT training run_report names preregistration %r, expected %s" % (ft_report.get("preregistration"), FT_TRAIN_PREREG))
    if not re.fullmatch(r"[0-9a-f]{40}", str((ft_report.get("preflight") or {}).get("head") or "")):
        p.append("FT training run_report has no 40-hex preflight.head")
    if (ft_receipt.get("settings") or {}).get("receipt") != str(FT_TRAIN_RECEIPT):
        p.append("FT training receipt does not name itself")
    p += ft_env_problems(ft_receipt.get("env_navrl") or {}, s2a_receipt.get("env_navrl") or {}, tree_of(ft_receipt),
                         tree_of(s2a_receipt))
    p += ["FT training " + x for x in ablation_problems(ft_receipt, s2a_receipt, tree_of(ft_receipt))]
    fs, ss = ft_receipt.get("settings") or {}, s2a_receipt.get("settings") or {}
    for key, seed_key in FT_SETTINGS_SEED_KEYS.items():
        a = {k: v for k, v in (fs.get(key) or {}).items() if k != seed_key}
        b = {k: v for k, v in (ss.get(key) or {}).items() if k != seed_key}
        if not b or a != b:
            p.append("FT training setting %s differs from CAM-S2a's (seed aside)" % key)
    for key in ("camsafe", "camtrain", "worst_case_thrust_to_weight"):
        if fs.get(key) != ss.get(key):
            p.append("FT training setting %s differs from CAM-S2a's" % key)
    for key in ("cams2_sha256", "camsafe_sha256", "task_robot_name", "overrides_after"):
        if ft_receipt.get(key) != s2a_receipt.get(key):
            p.append("FT training receipt %s differs from CAM-S2a's" % key)
    if (ft_receipt.get("cams2") or {}).get("config_after") != (s2a_receipt.get("cams2") or {}).get("config_after"):
        p.append("FT training cams2 config_after differs from CAM-S2a's")
    fsh, ssh = ft_receipt.get("sha256") or {}, s2a_receipt.get("sha256") or {}
    if set(fsh) != set(ssh) or any(fsh[k] != v for k, v in ssh.items() if k not in RECEIPT_SHA_MAY_DIFFER):
        p.append("FT training hashed modules differ from CAM-S2a's: %s"
                 % sorted(k for k in set(fsh) | set(ssh) if fsh.get(k) != ssh.get(k)))
    return p


def committed_sha256_at(commit, root=ROOT):
    """SHA-256 of the four eval-side modules at a commit, read from the object store (no worktree is touched)."""
    out = {}
    for key, rel in (("realveh_entry", S2A.ENTRY), ("yawclip", S2A.YAWCLIP_REL), ("cams2diag", S2A.CAMS2DIAG_REL),
                     ("contactcode", S2A.CONTACTCODE_REL)):
        done = subprocess.run(["git", "-C", str(root), "show", "%s:%s" % (commit, rel)], capture_output=True)
        out[key] = hashlib.sha256(done.stdout).hexdigest() if done.returncode == 0 else None
    return out


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


# ---------------------------------------------------------------- the reused A60 cells
def reuse_blocks():
    return [(d, s) for d in DENSITIES for s in SEEDS]


def reused_cell_dir(density, seed, raw=None):
    return (raw or S2A_E_RAW) / "eval_01" / S2A_E_ARM / (S2A_E_CELL % (density, seed))


def s2a_e_report_problems(ev, ident):
    """CAM-S2a-E's own records for the six reused cells (pure)."""
    p = []
    if ident.get("verdict") != "IDENTITY_PASS" or ident.get("commit") != S2A_E_COMMIT:
        p.append("CAM-S2a-E identity report is not IDENTITY_PASS at %s" % S2A_E_COMMIT[:7])
    if ev.get("unit") != S2A.UNIT or ev.get("commit") != S2A_E_COMMIT or ev.get("tree") != str(S2A_E_TREE):
        p.append("CAM-S2a-E eval report is not CAM-S2a-E at %s in %s" % (S2A_E_COMMIT[:7], S2A_E_TREE))
    if ev.get("verdict") not in ("COMPLETE", "INCOMPLETE"):
        p.append("CAM-S2a-E eval verdict %r (VOID_HOST_FAULT or PREFLIGHT_FAIL refuse the reuse)" % ev.get("verdict"))
    hs, he = ev.get("host_start") or {}, ev.get("host_end") or {}
    if not hs or any(hs.get(k) != he.get(k) for k in ("fault_count", "boot_id", "online")):
        p.append("CAM-S2a-E eval host fault count, boot id or CPU list changed (or unrecorded)")
    for d, s in reuse_blocks():
        recs = [r for r in ev.get("records") or [] if (r.get("arm"), r.get("density"), r.get("seed")) == (S2A_E_ARM, d, s)]
        if len(recs) != 1 or recs[0].get("verdict") != "OK" or recs[0].get("stage") != "A" \
                or recs[0].get("checkpoint_sha256") != S2A_CKPT_SHA or recs[0].get("commit") != S2A_E_COMMIT:
            p.append("CAM-S2a-E has no single OK stage-A YC12k record at %d bars / seed %d" % (d, s))
    return p


def reused_cell_problems(cell_dir, density, seed, s2a_receipt, committed_at_s2a_e):
    """A reused CAM-S2a-E YC12k cell passes this runner's checks as an A60 cell of CAM-S2a-E's tree and commit."""
    p, _ = output_problems(cell_dir, S2A_E_COMMIT, S2A_CKPT_SHA, sha(S2A_TRAIN_RECEIPT), s2a_receipt.get("env_navrl") or {})
    if not p:
        p += receipt_problems(cell_dir, "A60", density, seed, s2a_receipt, sha(S2A_TRAIN_RECEIPT), committed_at_s2a_e,
                              S2A_CKPT_SHA, root=S2A_E_TREE)
    return p


def reuse_manifest(raw=None):
    """{path: sha256} of every reused file and CAM-S2a-E's two reports (bound by path and SHA in the identity report and
    re-checked by the analysis)."""
    raw = raw or S2A_E_RAW
    out = {str(raw / rel): (sha(raw / rel) if (raw / rel).is_file() else None) for rel in S2A_E_REPORTS}
    for d, s in reuse_blocks():
        for f in REUSED_FILES:
            path = reused_cell_dir(d, s, raw) / f
            out[str(path)] = sha(path) if path.is_file() else None
    return out


def manifest_problems(manifest):
    p = []
    if not manifest or len(manifest) != len(reuse_blocks()) * len(REUSED_FILES) + len(S2A_E_REPORTS):
        p.append("reuse manifest incomplete")
    for path, want in (manifest or {}).items():
        if not want or not pathlib.Path(path).is_file() or sha(path) != want:
            p.append("reused file missing or changed: %s" % path)
    return p


def reuse_problems(s2a_receipt, raw=None, root=ROOT):
    raw = raw or S2A_E_RAW
    p = []
    try:
        ev = json.loads((raw / "eval_01" / "eval_run_report.json").read_text())
        ident = json.loads((raw / "identity_01" / "identity_report.json").read_text())
    except (OSError, ValueError) as e:
        return ["CAM-S2a-E reports unreadable: %s" % e]
    p += s2a_e_report_problems(ev, ident)
    committed = committed_sha256_at(S2A_E_COMMIT, root)
    for d, s in reuse_blocks():
        cell = reused_cell_dir(d, s, raw)
        p += ["%d/%d: %s" % (d, s, x) for x in reused_cell_problems(cell, d, s, s2a_receipt, committed)]
        rec = [r for r in ev.get("records") or [] if (r.get("arm"), r.get("density"), r.get("seed")) == (S2A_E_ARM, d, s)]
        want = ((rec[0].get("stats") or {}).get("result_sha256")) if len(rec) == 1 else None
        if not want or not (cell / "result.json").is_file() or sha(cell / "result.json") != want:
            p.append("%d/%d: result.json is not the one CAM-S2a-E's record names (result_sha256)" % (d, s))
    return p


def diag_equal(a_dir, b_dir):
    try:
        return K.read_jsonl(a_dir / DIAG_FILE) == K.read_jsonl(b_dir / DIAG_FILE)
    except (OSError, ValueError):
        return False


def reuse_decision(problems, repro, a60_repro_verdict="OK"):
    """REUSE_OK iff the reused cells pass every check and the reproduction equals the reused cell exactly;
    A60_NOT_RUN if A60_repro did not run (FT_identity failed first, or the identity cap); A60_PATH_FAIL if A60_repro
    itself failed here (the A60 path does not run under this runner: no rerun either);
    RERUN_REQUIRED if the reuse inputs were refused (stage a60 may rerun A60 here);
    A60_NOT_REPRODUCED if the inputs passed but the reproduction differs (the determinism premise failed: no rerun,
    an amendment decides). Only REUSE_OK, or RERUN_REQUIRED followed by stage a60, answers G1."""
    if a60_repro_verdict is None:
        return "A60_NOT_RUN"
    if a60_repro_verdict != "OK":
        return "A60_PATH_FAIL"
    if problems:
        return "RERUN_REQUIRED"
    ok = (isinstance(repro, dict) and repro.get("result_equal") is True and repro.get("episodes_equal") is True
          and repro.get("diag_equal") is True)
    return "REUSE_OK" if ok else "A60_NOT_REPRODUCED"


# ---------------------------------------------------------------- plans and verdicts
def eval_plan():
    """Block-major: for every (density, seed) block FT then FT45."""
    return [(arm, d, s) for d in DENSITIES for s in SEEDS for arm in EVAL_ARMS]


def a60_plan():
    return [("A60", d, s) for d in DENSITIES for s in SEEDS]


def smoke_verdict(records, acted):
    if any(r["verdict"] == "VOID_HOST_FAULT" for r in records):
        return "VOID_HOST_FAULT"
    ok = {r["arm"] for r in records if r["verdict"] == "OK"} == {a for a, _, _ in SMOKE_PLAN}
    return "SMOKE_OK" if ok and acted else "SMOKE_FAIL"


def identity_verdict(records):
    """IDENTITY_PASS iff FT_identity is OK and A60_repro ran (OK or FAIL; a FAIL only makes G1 unanswerable through
    a60_source = A60_PATH_FAIL, so G2 never waits on the A60 path). A void cell is handled by the caller."""
    by = {r["arm"]: r["verdict"] for r in records}
    return ("IDENTITY_PASS" if by.get("FT_identity") == "OK" and by.get("A60_repro") in ("OK", "FAIL")
            and len(records) == len(IDENTITY_PLAN) else "IDENTITY_FAIL")


def plan_verdict(records, plan):
    """COMPLETE iff every planned cell is OK (no optional cells in this unit)."""
    done = {(r["arm"], r["density"], r["seed"]) for r in records if r["verdict"] == "OK"}
    if any(r["verdict"] != "OK" for r in records):
        return "INCOMPLETE"
    return "COMPLETE" if set(plan) <= done else "INCOMPLETE"


def rate_projection(identity_report, stage):
    """Projected seconds of a stage from one identity cell's wall time and CAM-S2a-E's density factors (review r1 P2-2,
    P3-5): eval = (6 x RATE_175_OVER_205 + 6) x FT_identity (205 bars); a60 = (3 + 3 x RATE_205_OVER_175_CLIPPED) x
    A60_repro (175 bars); each x RATE_MARGIN against MAX_HOURS[stage]. The FT45 clip's own slowdown is not projected
    (no FT45 identity cell); the cap absorbs it, and a cut by the cap reads INCOMPLETE."""
    walls = {r.get("arm"): r.get("wall_s") for r in identity_report.get("records") or []}
    arm, units = (("FT_identity", 6 * RATE_175_OVER_205 + 6) if stage == "eval" else ("A60_repro", 3 + 3 * RATE_205_OVER_175_CLIPPED))
    w = walls.get(arm)
    if not isinstance(w, (int, float)) or w <= 0:
        return None
    n = len(eval_plan() if stage == "eval" else a60_plan())
    return {"cells": n, "cell_s": w, "from": arm, "cell_units": units, "margin": RATE_MARGIN,
            "projected_s": units * w * RATE_MARGIN, "cap_s": MAX_HOURS[stage] * 3600}


def rate_problems(identity_report, stage):
    proj = rate_projection(identity_report, stage)
    if proj is None:
        return ["the identity report lacks the wall times the %s rate gate needs" % stage]
    if proj["projected_s"] > proj["cap_s"]:
        return ["rate gate: %s projects to %.0f s (%.2f x %.0f s x %.2f), above the %.2f h cap"
                % (stage, proj["projected_s"], proj["cell_units"], proj["cell_s"], RATE_MARGIN, MAX_HOURS[stage])]
    return []


# ---------------------------------------------------------------- one cell
def run_cell(ctx, out_root, arm, density, seed, records, timeout_s, stage, quota=QUOTA):
    c = cfg(arm)
    ckpt, ckpt_sha = ctx["ckpt"][c["policy"]]
    train_receipt, train_receipt_sha = ctx["train"][c["env"]]
    out_dir = out_root / arm
    out_dir.mkdir(parents=True, exist_ok=True)
    cid = cell_id(arm, density, seed)
    env = cell_env(ROOT, out_dir, arm, density, seed, ckpt, ckpt_sha, train_receipt["env_navrl"], train_receipt_sha,
                   ctx["protocol"], quota)
    print("==", arm, cid, G.now(), "timeout %.0fs" % timeout_s, flush=True)
    t0 = time.time()
    rc = G.run_launcher(ROOT, ctx["protocol"], env, max(1.0, timeout_s), LAUNCHER)
    cell_dir = out_dir / cid
    problems, h, stats = check_cell(cell_dir, ctx["head"], ctx["start"], ckpt_sha, train_receipt_sha,
                                    train_receipt["env_navrl"], quota)
    if rc != 0:
        problems.insert(0, "launcher exit %d" % rc)
    if not problems:
        problems += receipt_problems(cell_dir, arm, density, seed, train_receipt, train_receipt_sha, ctx["committed"],
                                     ckpt_sha, quota)
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    rec = {"arm": arm, "stage": stage, "cell_id": cid, "tree": str(ROOT), "commit": ctx["head"], "seed": seed,
           "density": density, "quota": quota, "policy": c["policy"], "environment": c["env"], "checkpoint": ckpt,
           "checkpoint_sha256": ckpt_sha, "train_receipt_sha256": train_receipt_sha, "yawclip_radps": c["yawclip"],
           "recorder_yaw_ref_radps": c["diag_ref"], "module_seeds": S2A.module_seeds(seed), "exit": rc,
           "wall_s": time.time() - t0, "timeout_s": timeout_s, "host_after": h, "stats": stats,
           "yawclip_telemetry_last_line": S2A.parse_kv_line(text, S2A.YAWCLIP_PREFIX),
           "camsafe_telemetry_last_line": S2A.parse_kv_line(text, S2A.CAMSAFE_PREFIX),
           "mismatch_lines": S2A.mismatch_lines(text), "problems": problems,
           "verdict": "VOID_HOST_FAULT" if any(p.startswith("HOST") for p in problems) else ("OK" if not problems else "FAIL")}
    records.append(rec)
    print("  ", rec["verdict"], stats and stats["counts"], "%.0fs" % rec["wall_s"], problems[:3], flush=True)
    return rec


# ---------------------------------------------------------------- preflight
def player_pin_problems(protocol, root=ROOT):
    """The pinned play script (and the protocol launcher) bytes; Policy F's checkpoint is not needed here."""
    pol, p = protocol["policy"], []
    for key, rel in (("launcher_sha256", pol["launcher"]), ("player_sha256", pol["player"])):
        path = root / rel
        if not path.is_file() or sha(path) != pol[key]:
            p.append("%s missing or changed (pinned %s)" % (rel, pol[key][:12]))
    return p


def training_inputs():
    """(s2a_receipt, s2a_report, ft_receipt, ft_report, problems); unreadable files are problems, never defaults."""
    out, p = [], []
    for path in (S2A_TRAIN_RECEIPT, S2A_TRAIN_REPORT, FT_TRAIN_RECEIPT, FT_TRAIN_REPORT):
        try:
            out.append(json.loads(path.read_text()))
        except (OSError, ValueError) as e:
            out.append({})
            p.append("unreadable %s: %s" % (path, e))
    return out[0], out[1], out[2], out[3], p


def preflight(protocol, approved, out_root, unit_dir=UNIT_DIR):
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
    if not ap.is_file():
        problems.append("no approval file %s" % ap)
    else:
        try:
            approval = json.loads(ap.read_text())
            problems += R1.approval_problems(approval, head, scope=SCOPE)
            if approval.get("scope_sha256") != scope_digest():
                problems.append("approval scope_sha256 is not the SHA-256 of SCOPE (generate it with `cams2b_eval_run.py scope`)")
        except ValueError as e:
            problems.append("approval unreadable: %s" % e)
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(S2A.WRAPPER), os.X_OK):
        problems.append("tools/realveh_python is not executable")
    if not pathlib.Path(S2A.REAL_PYTHON).is_file():
        problems.append("interpreter %s missing" % S2A.REAL_PYTHON)
    problems += player_pin_problems(protocol)
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
    return head, problems, origins


def stage_link_problems(stage, out_root, head, ft_pin):
    """Each stage needs the earlier ones in the same parent, on this commit, with the same FT checkpoint (pure on files)."""
    parent, p, found = out_root.parent, [], {}

    def one(name, verdicts):
        hits = sorted(parent.glob("*/" + name))
        if len(hits) != 1:
            p.append("%s needs exactly one %s in %s, found %d" % (stage, name, parent, len(hits)))
            return None
        r = json.loads(hits[0].read_text())
        if r.get("verdict") not in verdicts or r.get("commit") != head or r.get("ft_checkpoint_sha256") != ft_pin:
            p.append("%s is not %s on this commit with this FT checkpoint" % (hits[0], "/".join(verdicts)))
            return None
        found[name] = (str(hits[0]), r)
        return r

    if stage in ("identity", "eval", "a60"):
        one("smoke_report.json", ("SMOKE_OK",))
    if stage == "identity" and list(parent.glob("*/identity_report.json")):
        p.append("an identity_report.json already exists in %s" % parent)
    if stage in ("eval", "a60"):
        one("identity_report.json", ("IDENTITY_PASS",))
    if stage == "eval" and list(parent.glob("*/eval_run_report.json")):
        p.append("an eval_run_report.json already exists in %s" % parent)
    if stage == "a60":
        one("eval_run_report.json", ("COMPLETE",))
        ident = found.get("identity_report.json")
        if ident and ident[1].get("a60_source") != "RERUN_REQUIRED":
            p.append("stage a60 runs only when the identity report reads a60_source RERUN_REQUIRED (it reads %r)"
                     % ident[1].get("a60_source"))
        if list(parent.glob("*/a60_run_report.json")):
            p.append("an a60_run_report.json already exists in %s" % parent)
    return p, found


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv == ["scope"]:
        print(json.dumps({"scope": SCOPE, "scope_sha256": scope_digest()}, indent=1))
        return 0
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("smoke", "identity", "eval", "a60", "check"))
    ap.add_argument("--approved-commit", default=None, help="required for every stage but check")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    if args.stage != "check" and not args.approved_commit:
        ap.error("--approved-commit is required for stage %s" % args.stage)
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    approved = args.approved_commit or K.git("rev-parse", "HEAD", root=ROOT).strip()
    head, problems, origins = preflight(protocol, approved, out_root)
    s2a_receipt, s2a_report, ft_receipt, ft_report, tp = training_inputs()
    problems += tp
    problems += S2A.training_problems(s2a_receipt, s2a_report)
    problems += ft_training_problems(ft_receipt, ft_report, s2a_receipt)
    branch = K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip()
    if branch != (ft_report.get("preflight") or {}).get("branch"):
        problems.append("eval branch %s is not the branch S2b-FT trained on (%r)" % (branch, (ft_report.get("preflight") or {}).get("branch")))
    ft_ck, ft_ck_problems = ft_checkpoint(ft_report)
    s2a_ck, s2a_ck_problems = s2a_checkpoint()
    problems += ft_ck_problems + s2a_ck_problems
    ft_head = str((ft_report.get("preflight") or {}).get("head") or "")
    changed_s2a, changed_ft = changed_since(S2A_E_COMMIT), (changed_since(ft_head) if ft_head else None)
    problems += allowlist_problems(changed_s2a, S2A_CODE_ALLOWLIST, S2A_E_COMMIT)
    problems += allowlist_problems(changed_ft, FT_CODE_ALLOWLIST, ft_head or "<no FT head>")
    link, found = stage_link_problems(args.stage if args.stage != "check" else "smoke", out_root, head, ft_ck[1])
    problems += link
    if args.stage == "check":                      # CPU only, read only (review r1 P2-6, P3-9): nothing is written
        print(json.dumps({"stage": "check", "commit": head, "preflight_problems": problems,
                          "reuse_problems": reuse_problems(s2a_receipt), "ft_checkpoint": ft_ck,
                          "changed_since_s2a_e": changed_s2a, "changed_since_ft_training": changed_ft}, indent=1, default=str))
        return 0 if not problems else 2
    train = {"S2A": (s2a_receipt, sha(S2A_TRAIN_RECEIPT) if S2A_TRAIN_RECEIPT.is_file() else None),
             "FT": (ft_receipt, sha(FT_TRAIN_RECEIPT) if FT_TRAIN_RECEIPT.is_file() else None)}
    start = G.host()
    report = {"unit": UNIT, "stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "scope": SCOPE,
              "scope_sha256": scope_digest(), "started": G.now(), "host_start": start, "import_origins": origins,
              "launcher": LAUNCHER, "launcher_sha256": LAUNCHER_SHA, "s2a_e_commit": S2A_E_COMMIT,
              "changed_since_s2a_e": changed_s2a, "ft_train_head": ft_head, "changed_since_ft_training": changed_ft,
              "s2a_train_receipt": str(S2A_TRAIN_RECEIPT), "s2a_train_receipt_sha256": train["S2A"][1],
              "ft_train_receipt": str(FT_TRAIN_RECEIPT), "ft_train_receipt_sha256": train["FT"][1],
              "ft_train_report_sha256": sha(FT_TRAIN_REPORT) if FT_TRAIN_REPORT.is_file() else None,
              "ft_checkpoint": ft_ck[0], "ft_checkpoint_sha256": ft_ck[1], "s2a_checkpoint": s2a_ck[0],
              "s2a_checkpoint_sha256": s2a_ck[1], "arms": ARMS, "quota": QUOTA, "num_envs": NUM_ENVS,
              "densities": list(DENSITIES), "seeds": list(SEEDS), "max_hours": MAX_HOURS[args.stage],
              "cell_timeout_s": CELL_TIMEOUT_S, "preflight_problems": problems,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if R1.ENV_ALLOW.match(k)},
              "linked_reports": {k: v[0] for k, v in found.items()}}
    if args.stage in ("eval", "a60") and "identity_report.json" in found:
        irep = found["identity_report.json"][1]
        if irep.get("env_allowlisted") != report["env_allowlisted"]:
            problems.append("runner environment differs from the identity stage's")
        for key in ("s2a_train_receipt_sha256", "ft_train_receipt_sha256", "ft_train_report_sha256"):
            if irep.get(key) != report[key]:
                problems.append("%s changed since the identity stage" % key)
        problems += rate_problems(irep, args.stage)
        report["rate_gate"] = rate_projection(irep, args.stage)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    committed = {k: sha(ROOT / rel) for k, rel in (("realveh_entry", S2A.ENTRY), ("yawclip", S2A.YAWCLIP_REL),
                                                   ("cams2diag", S2A.CAMS2DIAG_REL), ("contactcode", S2A.CONTACTCODE_REL))}
    report["committed_sha256"] = committed
    ctx = {"protocol": protocol, "head": head, "start": start, "committed": committed, "train": train,
           "ckpt": {"FT": ft_ck, "S12k": s2a_ck}}
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(MAX_HOURS[args.stage])
    records, void = [], False

    def timeout():
        return min(float(CELL_TIMEOUT_S), max(0.0, budget.remaining_s()))

    def run_plan(plan, stage, quota=QUOTA):
        for arm, density, seed in plan:
            if budget.exhausted():
                report["budget_exhausted"] = True
                return False
            rec = run_cell(ctx, out_root, arm, density, seed, records, timeout(), stage, quota)
            if rec["verdict"] != "OK":
                return rec["verdict"] == "VOID_HOST_FAULT"
        return False

    if args.stage == "smoke":
        void = run_plan(SMOKE_PLAN, "SMOKE", SMOKE_QUOTA)
        rows = {}
        for arm, d, s in SMOKE_PLAN:
            f = out_root / arm / cell_id(arm, d, s) / DIAG_FILE
            rows[arm] = K.read_jsonl(f) if f.is_file() else []
        yrec = next((r for r in records if r["arm"] == "FT45_smoke"), {})
        clipped = (yrec.get("yawclip_telemetry_last_line") or {}).get("clipped_frac")
        acted = (all(any(r.get("outcome") == "capture" for r in v) for v in rows.values())
                 and isinstance(clipped, float) and clipped > 0.0)
        report.update(smoke_rows={a: len(v) for a, v in rows.items()}, smoke_paths_acted=acted, quota=SMOKE_QUOTA,
                      smoke_crash_classes={a: sorted({str(r.get("crash_class")) for r in v}) for a, v in rows.items()},
                      smoke_ft45_clipped_frac=clipped)
        report["verdict"] = "VOID_HOST_FAULT" if void else smoke_verdict(records, acted)
        name = "smoke_report.json"
    elif args.stage == "identity":
        void = False
        for arm, density, seed in IDENTITY_PLAN:              # FT_identity first; A60_repro runs only after it is OK
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            rec = run_cell(ctx, out_root, arm, density, seed, records, timeout(), "IDENTITY")
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if arm == "FT_identity" and rec["verdict"] != "OK":
                break
        ok = {r["arm"] for r in records if r["verdict"] == "OK"}
        rp = reuse_problems(s2a_receipt)
        repro = None
        if "A60_repro" in ok and not rp:
            a = reused_cell_dir(*REPRO_BLOCK)
            b = out_root / "A60_repro" / cell_id("A60_repro", *REPRO_BLOCK)
            repro = dict(G.identity_compare(a, b), diag_equal=diag_equal(a, b), reused=str(a), reproduced=str(b))
        report.update(reuse_problems=rp, a60_reproduction=repro,
                      reuse_manifest=reuse_manifest() if not rp else None)
        report["a60_source"] = reuse_decision(rp, repro, next((r["verdict"] for r in records if r["arm"] == "A60_repro"), None))
        report["verdict"] = "VOID_HOST_FAULT" if void else identity_verdict(records)
        name = "identity_report.json"
    elif args.stage == "eval":
        void = run_plan(eval_plan(), "EVAL")
        report["verdict"] = "VOID_HOST_FAULT" if void else plan_verdict(records, eval_plan())
        report["cells_planned"] = len(eval_plan())
        name = "eval_run_report.json"
    else:
        void = run_plan(a60_plan(), "A60")
        report["verdict"] = "VOID_HOST_FAULT" if void else plan_verdict(records, a60_plan())
        report["cells_planned"] = len(a60_plan())
        name = "a60_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("SMOKE_OK", "IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
