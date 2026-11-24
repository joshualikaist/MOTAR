"""Runner of CAM-S2a-E, the evaluation of the CAM-S2a camera-only from-scratch policy in its own training environment
(docs/plans/prereg_camonly_s2a_eval_2026-11-24.md and its amendment 1; design reviews r1/r2 of the draft applied:
~/motar_tmp/cams2a_eval_20261124/review_r1/REVIEW_r1.md, review_r2/REVIEW_r2.md).

Amendment 1 (2026-11-24, after identity_01 read IDENTITY_FAIL with every cell aborting at the result export): every cell
also installs navrl_cams2_contactcode (NAVRL_CAMS2_CONTACTCODE=1, between camsafe and cams2), which gives the cams2
converted target contacts the crash-cause code 0 so the task's strata validator holds; the recorder classifies them
from the exposed mask. The P9 module seed's CONFIG MISMATCH line (the per-cell seed, by design) joins the allowlist.
A mandatory GPU smoke (`smoke`: one S12k cell, quota 1, 205 bars, never counted) precedes the identity stage; the rerun
writes under a new parent (raw_a1/{smoke,identity_01,eval_01}); raw/identity_01 stays as recorded.

The environment of every cell is the TRAINING environment: the NAVRL_* keys of ~/motar_tmp/cams2a_20261123/train_s1/
receipt.json (env_navrl, 161 keys), minus the keys this runner replaces (REMOVED_FROM_RECEIPT: the launcher sets them per
cell), plus EVAL_OVERRIDES (fixed density), per-cell module seeds (MODULE_SEED_KEYS: 10 x seed + k, matched across arms
of a block and distinct from training's 49xx seeds), the episode ledger, the per-episode recorder navrl_cams2diag (OFF in
one identity control) and, for the YC arms, the yaw-rate command clip navrl_yawclip (1.047 rad/s, action scale 3.0
unchanged). The launcher copy aerial_gym/rl_training/rl_games/eval_navrl_cams2a_arms.sh (SHA-pinned) adds only evaluation
mode and the per-cell keys; the simulator name, interpreter and GPU4GB are pinned by this runner, never inherited (P2-2).
A cell whose training environment lacks the cams2 rules is refused before launch (training review F2); a cell whose
realveh receipt does not show them installed FAILs and stops the stage.

Arms (checkpoints are last_gen_ppo_ep_*.pth of run ppo_261123_2213_navrl_v2-cams2a-scratch-camonly-s1, at their pins):
  S1k, S3k, S6k, S12k   epochs 1000, 3000, 6000, 12000 (trained at 70, 85, 100, 175 bars at those epochs);
  YC12k                 ep12000 with the yaw-rate command clip.
Identity stage (205 bars, seed 4501, quota 8; 5 cells; IDENTITY_FAIL stops the unit):
  S12k_identity   the reference; S12k_repeat must equal it exactly (determinism); S12k_nodiag (recorder OFF) must equal
  it exactly (the recorder is passive, P1-1); S1k_smoke must differ (the checkpoint path acts); YC12k_negative must
  differ and its whole-cell [yawclip] clipped share must be > 0 (the clip acts). No unexpected checkpoint-contract
  MISMATCH line may appear (P1-4).
Eval stage, one invocation, block-major so every block completes before the next (P1-5):
  stage A  gated arms S1k, S12k, YC12k x densities 115/175/205 x seeds 4501-4503 = 27 cells;
  stage B  S3k, S6k at 175 bars x 3 seeds = 6 cells;
  stage C  S3k, S6k at 115 and 205 bars x 3 seeds = 12 cells, optional: a C cell starts only when the remaining budget
           is >= RATE_MARGIN x the ep12000 identity mean cell time (review r2 P0-3).
  COMPLETE needs A and B; a FAIL cell in A or B stops the stage (INCOMPLETE); a timed-out or FAILed C cell is recorded and
  stops C, but A+B stay COMPLETE; a VOID_HOST_FAULT anywhere voids the run.
  Rate gate: projected A+B seconds from the identity cells' wall times per arm (S1k cells from S1k_smoke; every other
  cell from the mean of the four ep12000 cells) x RATE_MARGIN must fit MAX_HOURS["eval"]. Caps: identity 1.0 h, eval
  5.0 h; each cell's timeout is min(CELL_TIMEOUT_S, the remaining budget) (P2-7).
Cell checks (every cell; a FAIL stops the stage): launcher receipt (status OK, this commit, 0 dirty files, the checkpoint
SHA, the training-receipt SHA, base_sim, the interpreter, GPU4GB unset); ledger complete and clean; result.json condition
(goal range 6-28 m, full distribution, FOV curriculum saturated, riskcap FREE 2.5, P9 ON at the pinned SHA, 600 steps,
128 envs; P2-3); realveh receipt (cams2 / camsafe / depth / vehicle / camtrain settings equal to training's with the
cell's seeds; cams2 config_after; hashed modules byte-identical to training's except realveh_entry = HEAD's; yawclip iff
YC; cams2diag iff not a nodiag arm); the launcher-shell snapshot AND the entry's process env_navrl equal to training's
env_navrl except SNAPSHOT_EXPECTED, whose values are the cell's (P2-1); telemetry; recorder rows aligned with the
ledger; every "MISMATCH |" line of cell.log inside MISMATCH_ALLOWLIST (P1-4). Host fault count, boot id and online CPU
list unchanged.
Fail-closed preflight: approval naming SCOPE (and its SHA-256) and HEAD; clean tree; branch camonly/dev; committed files;
launcher at its pinned SHA; `git diff --name-only TRAIN_HEAD HEAD` inside DIFF_ALLOWLIST (P1-3); training run
COMPLETED_PROCESS with faults unchanged and its receipt at the recorded SHA; checkpoints at their pins; one RTX 3070; no
other GPU compute; import origins inside the tree; free disk >= 1 GB; new output folder.
usage: tools/cams2a_eval_run.py {smoke,identity,eval} --approved-commit SHA --out DIR
       tools/cams2a_eval_run.py scope          (prints SCOPE and its SHA-256 for the approval file)
"""
import argparse
import fnmatch
import hashlib
import json
import os
import pathlib
import re
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import gr_run as G                                          # noqa: E402  (host, tree_problems, run_launcher, identity_compare)
import kroc2027_eval as K                                   # noqa: E402  (base_env, ledger checks, GPU, imports, budget)
import camk1e_run as R1                                     # noqa: E402  (approval format, ENV_ALLOW)

PREREG = "docs/plans/prereg_camonly_s2a_eval_2026-11-24.md"
AMENDMENT = "docs/plans/prereg_camonly_s2a_eval_2026-11-24_amendment1.md"
UNIT = "CAM-S2a-E"
SCOPE = ("CAM-S2a-E v1: GPU evaluation of the CAM-S2a camera-only from-scratch checkpoints ep1000, ep3000, ep6000 and "
         "ep12000 in the CAM-S2a training environment (training receipt env_navrl: no_lidar, 312 x 234 camera at "
         "66.7 x 52.6 deg, D8 quadrotor target, legacy mixed target motion, riskcap FREE 2.5 m/s, camsafe, cams2 delay "
         "1-2 steps / capture 1.0 m 1.0 m/s detected / contact 0.40 m, altitude 1.5 m, T/W 3.5 vehicle randomisation, "
         "OAK-D depth model, gain randomisation) at fixed densities 115, 175, 205 bars x seeds 4501-4503, quota 8 per env "
         "(1,024 episodes per cell), plus ep12000 with the yaw-rate command clipped at 1.047 rad/s; 5 identity cells, then "
         "stage A (ep1000, ep12000, clipped ep12000 on all 9 blocks, 27 cells), stage B (ep3000, ep6000 at 175 bars, 6 "
         "cells) and optional stage C (ep3000, ep6000 at 115 and 205 bars, 12 cells, only within the cap); caps 1.0 h + "
         "5.0 h with a rate gate, idle host; tools/cams2a_eval_run.py; amendment 1: navrl_cams2_contactcode in every cell, "
         "one GPU smoke cell (quota 1) before the identity stage, output under raw_a1/")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "cams2a_eval_20261124"
TRAIN_DIR = pathlib.Path.home() / "motar_tmp" / "cams2a_20261123" / "train_s1"
TRAIN_RECEIPT = TRAIN_DIR / "receipt.json"
TRAIN_REPORT = TRAIN_DIR / "run_report.json"
TRAIN_HASHES = TRAIN_DIR / "checkpoint_sha256.txt"
TRAIN_HEAD = "22ed16f97fdc73d3457bc132fb458a3dc96a93af"   # the commit CAM-S2a run 1 trained at; the diff base (P1-3)
TRAIN_PREREG = "docs/plans/prereg_camonly_s2a_2026-11-23.md"
RUN_NN_REL = "aerial_gym/rl_training/rl_games/runs/ppo_261123_2213_navrl_v2-cams2a-scratch-camonly-s1/nn"
# Pins from ~/motar_tmp/cams2a_20261123/train_s1/checkpoint_sha256.txt (last_gen_ppo_ep_*.pth, never gen_ppo.pth).
CKPT = {"S1k": ("last_gen_ppo_ep_1000_rew_139.00893.pth",
                "1bf008994af2b46cbb492ad9d39374ba2ff6e04937750fe5dcb46cc17ee444b9"),
        "S3k": ("last_gen_ppo_ep_3000_rew_160.77231.pth",
                "22a20ddda5a3d12a64204f7f72d07b1c8e322095ab82a036a2c91198e3af9940"),
        "S6k": ("last_gen_ppo_ep_6000_rew_136.10913.pth",
                "bb956b55f616b1f556d7b07b22252b04bba08991ddc5f2ab18610d6a87b53885"),
        "S12k": ("last_gen_ppo_ep_12000_rew_104.0523.pth",
                 "9fc9dfb7edfc90db0b9a86432eb8cfb172a6e708a6d58eb1d40f15b6ed548996")}
ARM_BASE = {"S12k_identity": "S12k", "S12k_repeat": "S12k", "S12k_nodiag": "S12k", "S1k_smoke": "S1k",
            "YC12k_negative": "S12k", "YC12k": "S12k", "S12k_smoke": "S12k"}
YAWCLIP_ARMS = ("YC12k", "YC12k_negative")
NODIAG_ARMS = ("S12k_nodiag",)                            # review r1 P1-1: the recorder-OFF passivity control
YAWCLIP_RADPS = "1.047"                                   # 60 deg/s, the real vehicle's yaw-rate limit
IDENTITY_PLAN = (("S12k_identity", 205, 4501), ("S12k_repeat", 205, 4501), ("S12k_nodiag", 205, 4501),
                 ("S1k_smoke", 205, 4501), ("YC12k_negative", 205, 4501))
GATED_ARMS = ("S1k", "S12k", "YC12k")                      # stage A
DESCRIPTIVE_ARMS = ("S3k", "S6k")                          # stages B and C
EVAL_ARMS = GATED_ARMS + DESCRIPTIVE_ARMS
DENSITIES = (115, 175, 205)                               # none divisible by 4 (TF32 cdist note); 175 = final trained
STAGE_B_DENSITIES = (175,)
STAGE_C_DENSITIES = (115, 205)
SEEDS = (4501, 4502, 4503)                                # the family's grid seeds; training used NAVRL_SEED=1
QUOTA = 8                                                 # episodes per env; 1,024 per cell (CAM-K2-E: 16, 2,048)
NUM_ENVS = 128
MAX_HOURS = {"identity": 1.0, "eval": 5.0}
RATE_MARGIN = 1.10
CELL_TIMEOUT_S = 2400
MIN_FREE_GB = 1.0
BRANCH = "camonly/dev"
ARM_LABEL = "S2A_trained"
CELL = ARM_LABEL + "__%dbars__seed%d"
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_cams2a_arms.sh"
LAUNCHER_SHA = "0ad30292a584a9d95a03021bc771b5baf3d939634fcf3100ce9811668794e309"                           # amendment 1 changes the launcher; fill from the committed copy; a placeholder refuses
ENTRY = "tools/realveh_entry.py"
WRAPPER = ROOT / "tools/realveh_python"
REAL_PYTHON = "/home/fair/miniconda3/envs/aerialgym/bin/python"
SIM_NAME = "base_sim"
YAWCLIP_REL = "aerial_gym/task/navrl_task/navrl_yawclip.py"
CAMS2DIAG_REL = "aerial_gym/task/navrl_task/navrl_cams2diag.py"
CONTACTCODE_REL = "aerial_gym/task/navrl_task/navrl_cams2_contactcode.py"     # amendment 1
NEW_FILES = ("tools/cams2a_eval_run.py", "tools/cams2a_eval_analyse.py", LAUNCHER, YAWCLIP_REL, CAMS2DIAG_REL, CONTACTCODE_REL)
COMMITTED_FILES = (PREREG, AMENDMENT, ENTRY, "tools/realveh_python", TRAIN_PREREG, "aerial_gym/task/navrl_task/navrl_cams2.py",
                   "aerial_gym/task/navrl_task/navrl_camsafe.py") + NEW_FILES
# Review r1 P1-3: between the training commit and the eval commit only these paths may differ; everything else that the
# cells execute (runner.py, target_motion.py, the yaml, the ledger, configs, assets) is thereby bound to TRAIN_HEAD.
DIFF_ALLOWLIST = ("docs/*", "WORKLOG.md", "tests/*", ENTRY) + NEW_FILES
# Review r1 P1-4: the checkpoint's own contract warnings. Expected: the training seed (1 vs 4501-4503); the density
# curriculum keys (curriculum 0, START = FINAL = N against the checkpoint's 70 / 205). Anything else fails the cell.
MISMATCH_ALLOWLIST = (r"TRAINING SEED MISMATCH \|",
                      r"CURRICULUM CONFIG MISMATCH \| (NAVRL_DENSITY_\w+|cfg_density_\w+|n_bars\w*|density\w*)[: ]",
                      r"CONFIG MISMATCH \| NAVRL_P9_SEED:")       # amendment 1: the per-cell module seed (10 x seed + 3), by design
DIAG_FILE = "cams2diag_episodes.jsonl"
# Environment construction (see the module docstring).
REMOVED_FROM_RECEIPT = ("NAVRL_REALVEH_RECEIPT", "NAVRL_SEED", "NAVRL_DENSITY_START", "NAVRL_DENSITY_FINAL")
EVAL_OVERRIDES = {"NAVRL_DENSITY_CURRICULUM": "0"}
MODULE_SEED_KEYS = {"NAVRL_VEH_RAND_SEED": 1, "NAVRL_DEPTH_SEED": 2, "NAVRL_P9_SEED": 3, "NAVRL_CAMS2_DELAY_SEED": 4,
                    "NAVRL_GAIN_RAND_SEED": 5}
REQUIRED_IN_TRAINING_ENV = {"NAVRL_CAMS2": "1", "NAVRL_CAMSAFE": "1", "NAVRL_SENSOR_ABLATION": "no_lidar",
                            "NAVRL_VEH_RAND": "1", "NAVRL_DEPTH_MODEL": "oakd", "NAVRL_DYNAMIC_MESH_TREATMENT": "mesh_flat",
                            "NAVRL_VISION": "1", "NAVRL_PERCEPTION": "1"}
CAMS2_PARAMETERS = ("NAVRL_CAMS2_DELAY_MIN", "NAVRL_CAMS2_DELAY_MAX", "NAVRL_CAMS2_DELAY_SEED", "NAVRL_CAMS2_CAPTURE_RADIUS_M",
                    "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS", "NAVRL_CAMS2_CONTACT_RADIUS_M", "NAVRL_CAMS2_FLIGHT_ALTITUDE_M",
                    "NAVRL_CAMS2_CAMERA_X_M", "NAVRL_ACTION_DELAY_STEPS_MAX")
LAUNCHER_SETS = {"NAVRL_BULK_EVAL", "NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_CHECKPOINT", "NAVRL_EVAL_RUN_NONCE", "NAVRL_NUM_BARS",
                 "NAVRL_REALVEH_RECEIPT", "NAVRL_SEED", "NAVRL_DENSITY_START", "NAVRL_DENSITY_FINAL", "NAVRL_CAMS2DIAG_JSONL"}
RUNNER_SETS = ({"NAVRL_EPISODE_LEDGER", "NAVRL_EPISODE_QUOTA_PER_ENV", "NAVRL_PLAYER_GAMES_CAP", "NAVRL_CAMS2DIAG",
                "NAVRL_CAMS2DIAG_YAW_REF_RADPS", "NAVRL_YAWCLIP", "NAVRL_YAWCLIP_RADPS", "NAVRL_CAMS2_CONTACTCODE"}
               | set(EVAL_OVERRIDES) | set(MODULE_SEED_KEYS))
SNAPSHOT_EXPECTED = LAUNCHER_SETS | RUNNER_SETS
UNCONSTRAINED_IN_SNAPSHOT = {"NAVRL_EVAL_RUN_NONCE"}       # every other expected key has a required value (P2-1)
RECEIPT_SHA_MAY_DIFFER = ("realveh_entry",)
CAMSAFE_PREFIX, YAWCLIP_PREFIX, DIAG_PREFIX, CONTACT_PREFIX = "[camsafe]", "[yawclip]", "[cams2diag]", "[cams2contact]"
SMOKE = {"arm": "S12k_smoke", "density": 205, "seed": 4501, "quota": 1, "max_hours": 0.5}   # amendment 1: never counted
CONDITION_EXPECTED = {"goal_dist_min_m": 6.0, "goal_dist_max_m": 28.0, "full_goal_distribution": True,
                      "fov_curriculum_saturated": True, "speed_governor_mode": "riskcap", "speed_governor_free_mps": 2.5,
                      "p9_empirical_error_enabled": True, "episode_len_steps": 600, "num_envs": 128,
                      "d8_dynamic_mesh_treatment": True, "action_selection": "configured"}   # the task writes a bool (P0-1)
D8_MODE = "mesh_flat"                                     # condition["d8_dynamic_mesh_contract"]["mode"]


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def yawclip(arm):
    return arm in YAWCLIP_ARMS


def diag(arm):
    return arm not in NODIAG_ARMS


def arm_ckpt(arm, ckpts):
    return ckpts[ARM_BASE.get(arm, arm)]


def module_seeds(seed):
    """Per-cell seeds of the randomised modules, matched across the arms of a block."""
    return {k: str(10 * int(seed) + n) for k, n in MODULE_SEED_KEYS.items()}


def training_env_problems(train_env):
    """Training review F2: the environment must carry the cams2 rules (and the other opt-in modules) or nothing is launched."""
    p = []
    for k, v in REQUIRED_IN_TRAINING_ENV.items():
        if train_env.get(k) != v:
            p.append("training environment %s is %r, expected %r" % (k, train_env.get(k), v))
    for k in CAMS2_PARAMETERS:
        if not str(train_env.get(k, "")).strip():
            p.append("training environment lacks %s" % k)
    for k in MODULE_SEED_KEYS:
        if not str(train_env.get(k, "")).strip():
            p.append("training environment lacks %s (its module must be on)" % k)
    return p


def cell_navrl_env(train_env, seed, arm, quota=QUOTA, player_games_cap=None):
    """The NAVRL_* variables of one cell: the training environment with the per-cell replacements. Raises ValueError
    when the training environment does not define the CAM-S2a task."""
    problems = training_env_problems(train_env)
    if problems:
        raise ValueError("; ".join(problems))
    env = {k: v for k, v in train_env.items() if k.startswith("NAVRL_") and k not in REMOVED_FROM_RECEIPT}
    env.update(EVAL_OVERRIDES)
    env.update(module_seeds(seed))
    env.update(NAVRL_EPISODE_LEDGER="1", NAVRL_EPISODE_QUOTA_PER_ENV=str(int(quota)),
               NAVRL_PLAYER_GAMES_CAP=str(int(player_games_cap if player_games_cap is not None else 10 ** 9)),
               NAVRL_CAMS2_CONTACTCODE="1")                                                 # amendment 1: every cell
    if diag(arm):
        env.update(NAVRL_CAMS2DIAG="1", NAVRL_CAMS2DIAG_YAW_REF_RADPS=YAWCLIP_RADPS)
    if yawclip(arm):
        env.update(NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS=YAWCLIP_RADPS)
    return env


def cell_env(root, out_dir, arm, density, seed, ckpt, ckpt_sha, train_env, train_receipt_sha, protocol, quota=QUOTA):
    inherited = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_") and k != "GPU4GB"}
    env = K.base_env(root=root, inherited=inherited)
    cap = protocol["evaluation"]["player_games_cap"]
    env.update(cell_navrl_env(train_env, seed, arm, quota, cap))
    env.update(ARMS=ARM_LABEL, DENSITIES=str(int(density)), SEEDS=str(int(seed)), GAMES=str(quota * NUM_ENVS),
               NUM_ENVS=str(NUM_ENVS), OUT_ROOT=str(out_dir), CAMEVAL_PY=str(WRAPPER), CAMS2A_CKPT=str(ckpt),
               CAMS2A_CKPT_SHA=ckpt_sha, CAMS2A_TRAIN_RECEIPT_SHA256=train_receipt_sha,
               AERIAL_GYM_SIM_NAME=SIM_NAME, PYTHON=REAL_PYTHON)                        # P2-2: pinned, not inherited
    return env


def expected_snapshot_values(arm, density, seed, cell_dir, ckpt_sha, quota=QUOTA):
    want = {"NAVRL_DENSITY_CURRICULUM": "0", "NAVRL_NUM_BARS": str(density), "NAVRL_DENSITY_START": str(density),
            "NAVRL_DENSITY_FINAL": str(density), "NAVRL_SEED": str(seed), "NAVRL_BULK_EVAL": "1", "NAVRL_EPISODE_LEDGER": "1",
            "NAVRL_EPISODE_QUOTA_PER_ENV": str(quota), "NAVRL_PLAYER_GAMES_CAP": str(10 ** 9), "NAVRL_CAMS2_CONTACTCODE": "1",
            "NAVRL_EVAL_CHECKPOINT": ckpt_sha, "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"),
            "NAVRL_REALVEH_RECEIPT": str(cell_dir / "realveh_receipt.json"), **module_seeds(seed)}
    if diag(arm):
        want.update(NAVRL_CAMS2DIAG="1", NAVRL_CAMS2DIAG_YAW_REF_RADPS=YAWCLIP_RADPS, NAVRL_CAMS2DIAG_JSONL=str(cell_dir / DIAG_FILE))
    if yawclip(arm):
        want.update(NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS=YAWCLIP_RADPS)
    return want


def snapshot_problems(snap, train_env, arm, density, seed, cell_dir, ckpt_sha, label="snapshot", quota=QUOTA):
    """`snap` equals the training environment except SNAPSHOT_EXPECTED; every expected key present has the cell's value
    and no expected key that the arm does not use is present (P2-1)."""
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


def parse_kv_line(text, prefix):
    """The last 'prefix k=v k=v' telemetry line as a dict of floats (camsafe / yawclip format), or None."""
    last = None
    for line in text.splitlines():
        if line.startswith(prefix):
            last = line
    if last is None:
        return None
    out = {}
    for m in re.finditer(r"(\w+)=([-+0-9.eE]+)", last[len(prefix):]):
        out[m.group(1)] = float(m.group(2))
    return out


def mismatch_lines(text):
    return [line.strip() for line in text.splitlines() if "MISMATCH |" in line]


def mismatch_problems(text):
    """P1-4: every checkpoint-contract MISMATCH line must match a preregistered pattern."""
    bad = [line for line in mismatch_lines(text) if not any(re.search(pat, line) for pat in MISMATCH_ALLOWLIST)]
    return ["unexpected checkpoint-contract line: %s" % line[:200] for line in bad]


def diff_allowlist_problems(paths):
    """P1-3: the paths changed since TRAIN_HEAD must all be allow-listed."""
    bad = [p for p in paths if p and not any(fnmatch.fnmatch(p, pat) for pat in DIFF_ALLOWLIST)]
    return ["path changed since %s outside the allowlist: %s" % (TRAIN_HEAD[:7], p) for p in bad]


def condition_problems(cond, train_env):
    """P2-3: the result's condition shows the saturated evaluation distribution and the trained contract."""
    p = []
    for k, v in CONDITION_EXPECTED.items():
        if cond.get(k) != v:
            p.append("result condition %s is %r, expected %r" % (k, cond.get(k), v))
    if cond.get("p9_empirical_error_sha256") != train_env.get("NAVRL_P9_EXPECTED_SHA256"):
        p.append("result condition P9 SHA %r is not training's" % cond.get("p9_empirical_error_sha256"))
    contract = cond.get("d8_dynamic_mesh_contract")
    if not isinstance(contract, dict) or contract.get("mode") != D8_MODE:
        p.append("result condition d8_dynamic_mesh_contract %r is not mode %s" % (contract, D8_MODE))
    return p


def yc_exec_yaw_problems(cell_dir):
    """P0-2 / P2-13: in a clipped cell the recorder's executed-yaw-over-limit count must be 0 in every row, and the
    requested-over-limit count must be > 0 somewhere (the clip bound)."""
    try:
        rows = K.read_jsonl(cell_dir / DIAG_FILE)
    except (OSError, ValueError) as e:
        return ["recorder rows unreadable: %s" % e]
    over = sum(int(r.get("yaw_exec_over_ref_steps") or 0) for r in rows)
    requested = sum(int(r.get("yaw_request_over_ref_steps") or 0) for r in rows)
    p = []
    if over:
        p.append("yaw clip ON, but the recorder counts %d executed steps over the limit" % over)
    if not requested:
        p.append("yaw clip ON, but no step requested a yaw rate over the limit (the clip never bound)")
    return p


def diag_rows_problems(cell_dir):
    """The recorder's rows align with the ledger: same count, same outcome for every (env, episode)."""
    try:
        ledger = K.read_jsonl(cell_dir / "episodes.jsonl")
        rows = K.read_jsonl(cell_dir / DIAG_FILE)
    except (OSError, ValueError) as e:
        return ["recorder or ledger rows unreadable: %s" % e]
    p = []
    if len(rows) != len(ledger):
        p.append("recorder rows %d != ledger rows %d" % (len(rows), len(ledger)))
    by = {(r.get("env_index"), r.get("env_episode_index")): r for r in rows}
    mism = sum(1 for r in ledger if (by.get((r.get("env_index"), r.get("env_episode_index"))) or {}).get("outcome")
               != r.get("outcome"))
    if mism:
        p.append("%d ledger episodes have no recorder row with the same outcome" % mism)
    if not any(r.get("outcome") in ("capture", "crash", "timeout") for r in rows):
        p.append("recorder rows carry no outcome")
    return p


def receipt_problems(cell_dir, arm, density, seed, train_receipt, committed, ckpt_sha, quota=QUOTA):
    """The realveh receipt, both NAVRL snapshots and the telemetry of one cell against the training receipt.
    `committed` maps realveh_entry / yawclip / cams2diag / contactcode to their SHA-256 at HEAD."""
    try:
        rec = json.loads((cell_dir / "realveh_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["realveh receipt or cell receipt missing or unreadable: %s" % e]
    p = []
    ts, s = train_receipt.get("settings") or {}, rec.get("settings") or {}
    seeds = module_seeds(seed)
    for key, seed_key in (("cams2", "NAVRL_CAMS2_DELAY_SEED"), ("vehicle_rand", "NAVRL_VEH_RAND_SEED"),
                          ("depth_model", "NAVRL_DEPTH_SEED")):
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
    for key in ("cams2_sha256", "camsafe_sha256", "task_robot_name", "overrides_after", "ablation"):
        if rec.get(key) != train_receipt.get(key):
            p.append("receipt %s %r != training %r" % (key, rec.get(key), train_receipt.get(key)))
    shas, tshas = rec.get("sha256") or {}, train_receipt.get("sha256") or {}
    if set(shas) != set(tshas) or any(shas[k] != v for k, v in tshas.items() if k not in RECEIPT_SHA_MAY_DIFFER):
        p.append("receipt SHA-256 of the hashed modules differ from the training receipt")
    if shas.get("realveh_entry") != committed.get("realveh_entry"):
        p.append("receipt realveh_entry SHA is not HEAD's")
    for k in ("entry", "script"):
        if not K.is_inside(rec.get(k, ""), ROOT):
            p.append("realveh receipt %s outside the tree: %r" % (k, rec.get(k)))
    want_y = {"NAVRL_YAWCLIP_RADPS": float(YAWCLIP_RADPS)} if yawclip(arm) else None
    if (s.get("yawclip") != want_y or ("yawclip" in rec) != yawclip(arm)
            or (yawclip(arm) and (rec.get("yawclip_sha256") != committed.get("yawclip")
                                  or (rec.get("yawclip") or {}).get("limit_radps") != float(YAWCLIP_RADPS)
                                  or (rec.get("yawclip") or {}).get("scale_radps") != 3.0))):
        p.append("yaw-clip receipt state %r / %r does not match arm %s" % (s.get("yawclip"), rec.get("yawclip"), arm))
    if ((rec.get("cams2_contactcode") or {}).get("contact_code") != 0 or s.get("cams2_contactcode") != {}
            or rec.get("cams2_contactcode_sha256") != committed.get("contactcode")):          # amendment 1: every cell
        p.append("cams2 contact-code install record missing or not the committed module (amendment 1)")
    if diag(arm):
        if ((rec.get("cams2diag") or {}).get("jsonl") != str(cell_dir / DIAG_FILE)
                or (rec.get("cams2diag") or {}).get("yaw_ref_radps") != float(YAWCLIP_RADPS)
                or rec.get("cams2diag_sha256") != committed.get("cams2diag")):
            p.append("cams2diag install record missing, elsewhere, wrong yaw reference or not the committed module")
    elif "cams2diag" in rec or "cams2diag" in s or (cell_dir / DIAG_FILE).exists():
        p.append("recorder OFF, but a cams2diag record or file exists")
    p += snapshot_problems(receipt.get("env_snapshot_navrl", {}), train_receipt.get("env_navrl") or {}, arm, density, seed,
                           cell_dir, ckpt_sha, label="launcher snapshot", quota=quota)
    p += snapshot_problems(rec.get("env_navrl") or {}, train_receipt.get("env_navrl") or {}, arm, density, seed, cell_dir,
                           ckpt_sha, label="entry env_navrl", quota=quota)                                 # P2-1
    if receipt.get("train_receipt_sha256") != sha(TRAIN_RECEIPT):
        p.append("cell receipt does not name the training receipt's SHA")
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    if parse_kv_line(text, CAMSAFE_PREFIX) is None:
        p.append("no [camsafe] telemetry line")
    if CONTACT_PREFIX + " {" not in text:
        p.append("no [cams2contact] telemetry line (amendment 1)")
    yline = parse_kv_line(text, YAWCLIP_PREFIX)
    if yawclip(arm):
        if yline is None or yline.get("limit_radps") != float(YAWCLIP_RADPS) or yline.get("scale_radps") != 3.0:
            p.append("yaw clip ON, but no [yawclip] line with limit %s and scale 3.0" % YAWCLIP_RADPS)
    elif yline is not None:
        p.append("yaw clip OFF, but [yawclip] telemetry appears")
    if diag(arm):
        if DIAG_PREFIX + " {" not in text:
            p.append("no [cams2diag] summary line")
        p += diag_rows_problems(cell_dir)
        if yawclip(arm):
            p += yc_exec_yaw_problems(cell_dir)
    elif DIAG_PREFIX in text:
        p.append("recorder OFF, but [cams2diag] telemetry appears")
    p += mismatch_problems(text)                                                                           # P1-4
    return p


def check_cell(cell_dir, root, head, start_host, ckpt_sha, train_receipt_sha, train_env, quota=QUOTA):
    """gr_run.check_cell without the gain-randomisation and recorder checks (the training environment keeps gains ON
    and the KROC2027 recorder is off), with the training-receipt, pinned-runtime and condition bindings."""
    problems = []
    rec_host = G.host()
    for k in ("fault_count", "boot_id", "online"):
        if rec_host[k] != start_host[k]:
            problems.append("HOST " + k + " changed")
    try:
        receipt = json.loads((cell_dir / "receipt.json").read_text())
        result = json.loads((cell_dir / "result.json").read_text())
        summary = json.loads((cell_dir / "episode_ledger_summary.json").read_text())
        rows = K.read_jsonl(cell_dir / "episodes.jsonl")
    except (OSError, ValueError) as e:
        return problems + ["missing or unreadable output: %s" % e], rec_host, None
    if receipt.get("status") != "OK" or receipt.get("git_commit") != head or receipt.get("git_dirty_files") != 0:
        problems.append("receipt status/commit/dirty mismatch")
    if receipt.get("checkpoint_sha256") != ckpt_sha:
        problems.append("receipt checkpoint %s is not %s" % (receipt.get("checkpoint_sha256"), ckpt_sha))
    if receipt.get("train_receipt_sha256") != train_receipt_sha:
        problems.append("receipt training-receipt SHA mismatch")
    if receipt.get("sim_name") != SIM_NAME or receipt.get("python") != REAL_PYTHON or receipt.get("gpu4gb", "") != "":
        problems.append("receipt runtime pins (sim %r, python %r, gpu4gb %r) are not the preregistered ones" % (
            receipt.get("sim_name"), receipt.get("python"), receipt.get("gpu4gb")))
    problems.extend(condition_problems(result.get("condition") or {}, train_env))
    problems.extend(K.ledger_problems(summary, rows, NUM_ENVS, quota))
    if K.git("status", "--porcelain", root=root).strip():
        problems.append("tree became dirty")
    quota_rows = [r for r in rows if r.get("in_quota")]
    counts = {o: sum(1 for r in quota_rows if r.get("outcome") == o) for o in ("capture", "crash", "timeout")}
    return problems, rec_host, {"quota_episodes": len(quota_rows), "counts": counts,
                                "duration_s": receipt.get("duration_s"), "result_sha256": receipt.get("result_sha256")}


def run_cell(protocol, out_root, arm, density, seed, head, start, records, ckpts, train_receipt, train_receipt_sha,
             committed, timeout_s, stage=None, quota=QUOTA):
    ckpt, ckpt_sha = arm_ckpt(arm, ckpts)
    out_dir = out_root / arm
    out_dir.mkdir(parents=True, exist_ok=True)
    cell_id = CELL % (density, seed)
    env = cell_env(ROOT, out_dir, arm, density, seed, ckpt, ckpt_sha, train_receipt["env_navrl"], train_receipt_sha,
                   protocol, quota)
    print("==", arm, cell_id, G.now(), "timeout %.0fs" % timeout_s, flush=True)
    t0 = time.time()
    rc = G.run_launcher(ROOT, protocol, env, max(1.0, timeout_s), LAUNCHER)
    cell_dir = out_dir / cell_id
    problems, h, stats = check_cell(cell_dir, ROOT, head, start, ckpt_sha, train_receipt_sha, train_receipt["env_navrl"], quota)
    if rc != 0:
        problems.insert(0, "launcher exit %d" % rc)
    if not problems:
        problems += receipt_problems(cell_dir, arm, density, seed, train_receipt, committed, ckpt_sha, quota)
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    rec = {"arm": arm, "stage": stage, "cell_id": cell_id, "tree": str(ROOT), "commit": head, "seed": seed, "density": density,
           "quota": quota, "checkpoint": ckpt, "checkpoint_sha256": ckpt_sha, "yawclip": yawclip(arm), "recorder": diag(arm),
           "module_seeds": module_seeds(seed), "exit": rc, "wall_s": time.time() - t0, "timeout_s": timeout_s,
           "host_after": h, "stats": stats,
           "yawclip_telemetry_last_line": parse_kv_line(text, YAWCLIP_PREFIX),
           "camsafe_telemetry_last_line": parse_kv_line(text, CAMSAFE_PREFIX), "mismatch_lines": mismatch_lines(text),
           "problems": problems,
           "verdict": "VOID_HOST_FAULT" if any(p.startswith("HOST") for p in problems) else ("OK" if not problems else "FAIL")}
    records.append(rec)
    print("  ", rec["verdict"], stats and stats["counts"], "%.0fs" % rec["wall_s"], problems[:3], flush=True)
    return rec


def eval_plan():
    """Block-major: for every (density, seed) block the gated arms, then stage B, then stage C (P1-5)."""
    plan = [(arm, d, s, "A") for d in DENSITIES for s in SEEDS for arm in GATED_ARMS]
    plan += [(arm, d, s, "B") for d in STAGE_B_DENSITIES for s in SEEDS for arm in DESCRIPTIVE_ARMS]
    plan += [(arm, d, s, "C") for d in STAGE_C_DENSITIES for s in SEEDS for arm in DESCRIPTIVE_ARMS]
    return plan


def required_cells():
    return [x for x in eval_plan() if x[3] in ("A", "B")]


def rate_projection(identity_report):
    """Projected seconds of stages A + B from the identity cells' wall times per arm (P1-5): S1k cells from S1k_smoke,
    every other cell from the mean of the four ep12000 cells (identity, repeat, nodiag, YC12k_negative)."""
    walls = {r.get("arm"): r.get("wall_s") for r in identity_report.get("records") or []}
    if set(walls) != {a for a, _, _ in IDENTITY_PLAN} or not all(isinstance(w, (int, float)) and w > 0 for w in walls.values()):
        return None
    ep12k = [walls[a] for a in ("S12k_identity", "S12k_repeat", "S12k_nodiag", "YC12k_negative")]
    mean12k = sum(ep12k) / len(ep12k)
    cells = required_cells()
    n_s1k = sum(1 for c in cells if c[0] == "S1k")
    projected = (n_s1k * walls["S1k_smoke"] + (len(cells) - n_s1k) * mean12k) * RATE_MARGIN
    return {"cells_required": len(cells), "s1k_cells": n_s1k, "s1k_cell_s": walls["S1k_smoke"], "ep12k_mean_cell_s": mean12k,
            "margin": RATE_MARGIN, "projected_s": projected, "cap_s": MAX_HOURS["eval"] * 3600,
            "stage_c_fits": projected + 12 * mean12k * RATE_MARGIN <= MAX_HOURS["eval"] * 3600}


def rate_problems(identity_report):
    proj = rate_projection(identity_report)
    if proj is None:
        return ["the identity report has no wall time for each of its %d cells" % len(IDENTITY_PLAN)]
    if proj["projected_s"] > proj["cap_s"]:
        return ["rate gate: stages A+B project to %.0f s (%d x %.0f s S1k + %d x %.0f s ep12000, x %.2f), above the %.1f h cap"
                % (proj["projected_s"], proj["s1k_cells"], proj["s1k_cell_s"], proj["cells_required"] - proj["s1k_cells"],
                   proj["ep12k_mean_cell_s"], RATE_MARGIN, MAX_HOURS["eval"])]
    return []


def identity_verdict(records, comparisons):
    """IDENTITY_PASS iff every planned cell is OK; S12k_repeat and S12k_nodiag equal S12k_identity (result and episodes);
    S1k_smoke and YC12k_negative differ in their episodes; YC12k_negative's whole-cell clipped share > 0."""
    ok_cells = {r["arm"] for r in records if r["verdict"] == "OK"}
    if ok_cells != {a for a, _, _ in IDENTITY_PLAN}:
        return "IDENTITY_FAIL"
    c = comparisons
    equal = all(c.get(a, {}).get("result_equal") is True and c.get(a, {}).get("episodes_equal") is True
                for a in ("S12k_repeat", "S12k_nodiag"))
    differ = all(c.get(a, {}).get("episodes_equal") is False for a in ("S1k_smoke", "YC12k_negative"))
    clipped = c.get("YC12k_negative_clipped_frac")
    return "IDENTITY_PASS" if equal and differ and isinstance(clipped, float) and clipped > 0.0 else "IDENTITY_FAIL"


def ab_records(records):
    """The stage A and B records; stage C is optional and never changes the verdict (review r2 P0-3)."""
    return [r for r in records if r.get("stage") in ("A", "B")]


def eval_verdict(records, plan=None):
    """COMPLETE iff every stage A and B cell is OK; INCOMPLETE otherwise. Stage C records are reported, never judged
    (a timed-out or FAILed C cell leaves A+B COMPLETE; a VOID_HOST_FAULT anywhere is handled by the caller)."""
    plan = plan or eval_plan()
    ab = ab_records(records)
    if any(r["verdict"] != "OK" for r in ab):
        return "INCOMPLETE"
    done = {(r["arm"], r["density"], r["seed"]) for r in ab if r["verdict"] == "OK"}
    need = {(a, d, s) for a, d, s, st in plan if st in ("A", "B")}
    return "COMPLETE" if need <= done else "INCOMPLETE"


def stage_c_may_start(remaining_s, ep12k_mean_cell_s):
    """P0-3: a stage C cell starts only when the remaining budget covers a full ep12000-like cell with the margin."""
    return remaining_s is not None and ep12k_mean_cell_s is not None and remaining_s >= RATE_MARGIN * float(ep12k_mean_cell_s)


def checkpoints(root=ROOT, hashes_path=TRAIN_HASHES):
    """The four checkpoints at their pins, which must also appear in the training unit's hash list."""
    out, problems = {}, []
    try:
        listed = dict(reversed(line.split()) for line in hashes_path.read_text().splitlines() if line.strip())
    except OSError as e:
        listed, problems = {}, ["checkpoint hash list unreadable: %s" % e]
    for arm, (name, pin) in CKPT.items():
        path = root / RUN_NN_REL / name
        if not re.fullmatch(r"[0-9a-f]{64}", pin):
            problems.append("%s pin is not a SHA-256" % arm)
        if listed.get(name) != pin:
            problems.append("%s: the training hash list does not list %s at the pin" % (arm, name))
        if not path.is_file() or sha(path) != pin:
            problems.append("%s checkpoint missing or SHA mismatch: %s" % (arm, path))
        out[arm] = (str(path), pin)
    return out, problems


def training_problems(train_receipt, train_report):
    p = []
    if train_report.get("verdict") != "COMPLETED_PROCESS" or train_report.get("mode") != "run":
        p.append("training run_report verdict %r / mode %r" % (train_report.get("verdict"), train_report.get("mode")))
    if train_report.get("faults_before") != train_report.get("faults_after"):
        p.append("training host fault count changed during the run")
    if (train_report.get("preflight") or {}).get("head") != TRAIN_HEAD or train_report.get("preregistration") != TRAIN_PREREG:
        p.append("training run_report is not CAM-S2a run 1 at %s" % TRAIN_HEAD[:7])
    p += training_env_problems(train_receipt.get("env_navrl") or {})
    if (train_receipt.get("settings") or {}).get("receipt") != str(TRAIN_RECEIPT):
        p.append("training receipt does not name itself")
    return p


def committed_sha256(root=ROOT):
    return {"realveh_entry": sha(root / ENTRY), "yawclip": sha(root / YAWCLIP_REL), "cams2diag": sha(root / CAMS2DIAG_REL),
            "contactcode": sha(root / CONTACTCODE_REL)}


def changed_since_training(root=ROOT):
    out = subprocess.run(["git", "-C", str(root), "diff", "--name-only", TRAIN_HEAD, "HEAD"], capture_output=True, text=True)
    if out.returncode:
        return None
    return [line.strip() for line in out.stdout.splitlines() if line.strip()]


def scope_digest():
    return hashlib.sha256(SCOPE.encode("utf-8")).hexdigest()


def preflight(protocol, approved, out_root, unit_dir=UNIT_DIR):
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:
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
            approval = json.loads(ap.read_text())
            problems += R1.approval_problems(approval, head, scope=SCOPE)
            if approval.get("scope_sha256") != scope_digest():
                problems.append("approval scope_sha256 is not the SHA-256 of SCOPE (generate it with `cams2a_eval_run.py scope`)")
        except ValueError as e:
            problems.append("approval unreadable: %s" % e)
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    changed = changed_since_training()
    if changed is None:
        problems.append("git diff %s..HEAD failed" % TRAIN_HEAD[:7])
    else:
        problems += diff_allowlist_problems(changed)
    if not re.fullmatch(r"[0-9a-f]{64}", LAUNCHER_SHA):
        problems.append("LAUNCHER_SHA is a placeholder: fill it from the committed launcher copy before the commit")
    if not (ROOT / LAUNCHER).is_file() or sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(WRAPPER), os.X_OK):
        problems.append("tools/realveh_python is not executable")
    if not pathlib.Path(REAL_PYTHON).is_file():
        problems.append("interpreter %s missing" % REAL_PYTHON)
    problems += K.static_problems(protocol, ROOT)
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
    return head, problems, origins, changed


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv == ["scope"]:
        print(json.dumps({"scope": SCOPE, "scope_sha256": scope_digest()}, indent=1))
        return 0
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=("smoke", "identity", "eval"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins, changed = preflight(protocol, args.approved_commit, out_root)
    try:
        train_receipt = json.loads(TRAIN_RECEIPT.read_text())
        train_report = json.loads(TRAIN_REPORT.read_text())
    except (OSError, ValueError) as e:
        train_receipt, train_report = {}, {}
        problems.append("training receipt or run report unreadable: %s" % e)
    problems += training_problems(train_receipt, train_report)
    train_receipt_sha = sha(TRAIN_RECEIPT) if TRAIN_RECEIPT.is_file() else None
    ckpts, ck_problems = checkpoints()
    problems += ck_problems
    committed = committed_sha256() if not problems else {}
    start = G.host()
    report = {"unit": UNIT, "stage": args.stage, "tree": str(ROOT), "commit": head, "train_head": TRAIN_HEAD,
              "changed_since_training": changed, "prereg": PREREG, "scope": SCOPE, "scope_sha256": scope_digest(),
              "started": G.now(), "host_start": start, "import_origins": origins, "launcher": LAUNCHER,
              "launcher_sha256": LAUNCHER_SHA, "train_receipt": str(TRAIN_RECEIPT), "train_receipt_sha256": train_receipt_sha,
              "train_env_keys": len(train_receipt.get("env_navrl") or {}), "checkpoints": ckpts, "committed_sha256": committed,
              "quota": QUOTA, "num_envs": NUM_ENVS, "densities": list(DENSITIES), "seeds": list(SEEDS),
              "yawclip_radps": YAWCLIP_RADPS, "sim_name": SIM_NAME, "python": REAL_PYTHON, "preflight_problems": problems,
              "max_hours": MAX_HOURS.get(args.stage, SMOKE["max_hours"]), "cell_timeout_s": CELL_TIMEOUT_S,
              "amendment": AMENDMENT,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if R1.ENV_ALLOW.match(k)}}
    if args.stage in ("identity", "eval"):                       # amendment 1: one SMOKE_OK on this commit in the same parent
        smokes = sorted(out_root.parent.glob("*/smoke_report.json"))
        if len(smokes) != 1 or json.loads(smokes[0].read_text()).get("verdict") != "SMOKE_OK" \
                or json.loads(smokes[0].read_text()).get("commit") != head:
            problems.append("%s needs exactly one SMOKE_OK smoke_report.json on this commit in %s, found %d"
                            % (args.stage, out_root.parent, len(smokes)))
        else:
            report["smoke_report"] = str(smokes[0])
    if args.stage == "identity" and list(out_root.parent.glob("*/identity_report.json")):
        problems.append("an identity_report.json already exists in %s (the rerun needs a new parent folder)" % out_root.parent)
    if args.stage == "eval":
        idents = sorted(out_root.parent.glob("*/identity_report.json"))
        ident = None
        if len(idents) != 1:
            problems.append("eval needs exactly one identity_report.json in %s, found %d" % (out_root.parent, len(idents)))
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
            if irep.get("train_receipt_sha256") != train_receipt_sha:
                problems.append("the training receipt changed since the identity stage")
            problems += rate_problems(irep)
            report["rate_gate"] = rate_projection(irep)
        report["identity_report"] = str(ident)
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(MAX_HOURS.get(args.stage, SMOKE["max_hours"]))
    records, void = [], False

    def timeout():
        return min(float(CELL_TIMEOUT_S), max(0.0, budget.remaining_s()))

    if args.stage == "smoke":
        # Amendment 1: one short cell through every changed runner path (contact code, recorder, launcher, checks); it is
        # never counted in any rate or result. SMOKE_OK needs the cell OK, a result.json and >= 1 crash-cause-coded row.
        rec = run_cell(protocol, out_root, SMOKE["arm"], SMOKE["density"], SMOKE["seed"], head, start, records, ckpts,
                       train_receipt, train_receipt_sha, committed, timeout(), "SMOKE", SMOKE["quota"])
        cell_dir = out_root / SMOKE["arm"] / rec["cell_id"]
        rows = K.read_jsonl(cell_dir / DIAG_FILE) if (cell_dir / DIAG_FILE).is_file() else []
        report["smoke_rows"] = len(rows)
        report["smoke_crash_classes"] = sorted({r.get("crash_class") for r in rows})
        report["smoke_outcomes"] = sorted({r.get("outcome") for r in rows})
        # Review a1 P2-1: the smoke must show the fix acting, not only a cell that ran: >= 1 converted target contact
        # (recorder class target_contact_swept), >= 1 capture row, and a [cams2contact] running count > 0 in cell.log.
        log_text = (cell_dir / "cell.log").read_text(errors="replace") if (cell_dir / "cell.log").is_file() else ""
        contact_lines = [json.loads(line.split(CONTACT_PREFIX, 1)[1]) for line in log_text.splitlines()
                         if CONTACT_PREFIX + " {" in line]
        report["smoke_cams2contact_last"] = contact_lines[-1] if contact_lines else None
        acted = (any(r.get("crash_class") == "target_contact_swept" for r in rows)
                 and any(r.get("outcome") == "capture" for r in rows)
                 and bool(contact_lines) and int(contact_lines[-1].get("contacts", 0)) > 0)
        report["smoke_fix_acted"] = acted
        report["quota"] = SMOKE["quota"]                                                      # review a1 P3-5
        ok = rec["verdict"] == "OK" and (cell_dir / "result.json").is_file() and len(rows) > 0 and acted
        report["verdict"] = "VOID_HOST_FAULT" if rec["verdict"] == "VOID_HOST_FAULT" else ("SMOKE_OK" if ok else "SMOKE_FAIL")
        name = "smoke_report.json"
    elif args.stage == "identity":
        comp = {}
        for arm, density, seed in IDENTITY_PLAN:
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            rec = run_cell(protocol, out_root, arm, density, seed, head, start, records, ckpts, train_receipt,
                           train_receipt_sha, committed, timeout())
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
        cid = CELL % (205, 4501)
        ok_cells = {r["arm"] for r in records if r["verdict"] == "OK"}
        if "S12k_identity" in ok_cells:
            for other in ("S12k_repeat", "S12k_nodiag", "S1k_smoke", "YC12k_negative"):
                if other in ok_cells:
                    comp[other] = G.identity_compare(out_root / "S12k_identity" / cid, out_root / other / cid)
        yneg = next((r for r in records if r["arm"] == "YC12k_negative"), None)
        comp["YC12k_negative_clipped_frac"] = ((yneg or {}).get("yawclip_telemetry_last_line") or {}).get("clipped_frac")
        report["comparison"] = comp
        report["verdict"] = "VOID_HOST_FAULT" if void else identity_verdict(records, comp)
        name = "identity_report.json"
    else:
        plan = eval_plan()
        ep12k_mean = (report.get("rate_gate") or {}).get("ep12k_mean_cell_s")
        for arm, density, seed, stage in plan:
            if budget.exhausted():
                report["budget_exhausted"] = True
                break
            if stage == "C" and not stage_c_may_start(budget.remaining_s(), ep12k_mean):
                report["stage_c_stopped_by_budget"] = {"remaining_s": budget.remaining_s(), "ep12k_mean_cell_s": ep12k_mean}
                break
            rec = run_cell(protocol, out_root, arm, density, seed, head, start, records, ckpts, train_receipt,
                           train_receipt_sha, committed, timeout(), stage)
            if rec["verdict"] != "OK":
                void = rec["verdict"] == "VOID_HOST_FAULT"
                if stage == "C" and not void:
                    report["stage_c_stopped_by_cell"] = rec["cell_id"]       # recorded; A+B stay judged on their own
                break
        report["verdict"] = "VOID_HOST_FAULT" if void else eval_verdict(records, plan)
        report["cells_planned"] = len(plan)
        report["cells_required"] = len(required_cells())
        report["stage_c_cells_done"] = sum(1 for r in records if r.get("stage") == "C" and r["verdict"] == "OK")
        name = "eval_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("SMOKE_OK", "IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
