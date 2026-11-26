"""Runner of CAM-REF stage 1 (stage 2, docs/plans/prereg_camonly_ref_stage2_2026-11-26.md, runs this module through
tools/camref_stage2_run.py, which replaces the unit constants; its rungs F4D..F4A come from the factor table F_FACTORS): Policy F, CAM-K1 (seed 1) and CAM-S2b-FT on one protocol ladder at 205 bars
(preregistration docs/plans/prereg_camonly_ref_matched_protocol_2026-11-26.md, draft in ~/motar_tmp/camref_20261126/draft/).

Protocol points: P_K (CAM-K1-E / GR-0: A3 start, E2 target, 0.5 m swept capture, no delay, nominal vehicle and gains,
altitude 1.0 m, mount 0.10 m, analytic target), P_S (CAM-S2 / CAM-DX1 V312) and P_T (P_S task rules and the D8 target with
P_K's vehicle and environment). Every policy keeps its own perception and trained control stack.
Cells (stage 1 table of the preregistration; 205 bars, quota 8 x 128 envs = 1,024 episodes per cell):
  F0ID  F at P_K, seed 4501, idle host: identity against GR-0 S00 205 / 4501 (first 8 episodes per env).
  K1ID  CAM-K1 at its native P_K through the new path, seed 4501: identity against CAM-K1-E K1s1 205 / 4501 (first 8).
  S0ID  S2b-FT at P_S, seed 4501: identity against CAM-DX1 V312 205 / 4501 (per episode).
  F3    F: P_K vehicle / environment + V312 start + legacy mixed motion + the CAM-S2 capture / contact rule
        (navrl_camref_rules, detection = F's fused _visible_now).
  F4    F3 + the D8 target (mesh_flat) = F at P_T.
  F8    F4 + delay U{1,2} + vehicle (T/W 3.5 raise, +-15 %, +-5 %, tau 0.02-0.04 s) and gain (+-20 %) randomisation +
        yaw clip 1.047 rad/s (scale kept 3.0) + governor FREE 2.5 + altitude 1.5 m / mount 0.08 m = F at P_S (no camsafe).
  S3    S2b-FT at P_S minus altitude / mount (1.0 / 0.10), vehicle and gain randomisation and delay (0) = S2b-FT at P_T.
  S6    S3 minus the CAM-S2 capture rule (cams2 off), E2 target, A3 start = S2b-FT at P_K (D8 target, camsafe stay).
  K1T   F4's settings with CAM-K1's perception (no_lidar, 66 x 54 deg).        K1S  F8's settings with CAM-K1's perception.
  Eval: the seven rung arms x seeds 4501-4503, block-major by seed (21 cells).
Environments: F and K1 start from F_BASE_ENV, the NAVRL_* contract GR-0's KROC2027 launcher held fixed (bound against the
recorded GR-0 S00 205 / 4501-4503 snapshots in preflight); S2b-FT starts from CAM-DX1's V312 cell construction
(tools/camdx1_eval_run.cell_navrl_env, bound against the recorded DX1 V312 205 / 4501-4503 snapshots); CAM-K1's native
environment (F_BASE_ENV + its no_lidar / FOV keys) is bound against the recorded CAM-K1-E K1s1 snapshots. A rung's keys are
RUNG_DELTA; every other difference refuses the cell before launch and fails it after (launcher and entry snapshots).
Launcher: the pinned copy aerial_gym/rl_training/rl_games/eval_navrl_camref_arms.sh through tools/camref_python
(tools/camref_entry.py; K1 and S2b-FT continue through tools/realveh_entry.py, unchanged).
Stages (each needs the approval file and the stage before it, on the same commit, in the same parent folder; the three
stages share the preregistered 2.5 h wall cap, each stage's own cap being min(its sub-cap, 2.5 h - the earlier stages'
wall time); a cell whose projected wall time does not fit the remaining cap is skipped, not started (BUDGET_SKIPPED), and
its stage reads incomplete):
  check     CPU only, writes nothing, needs no approval: prints the preflight problems (approval and idle-host state for
            the GPU stages reported separately).
  scope     prints SCOPE and its SHA-256 for the approval file.
  smoke     F8_smoke, K1S_smoke, S6_smoke at 205 / 4501, quota 1, never counted (the three entry paths S0ID does not
            exercise). SMOKE_OK needs every cell OK with >= 1 finished episode and the module controls (SMOKE_CONTROLS).
  identity  F0ID, K1ID, then S0ID (quota 8). Any per-episode outcome difference is IDENTITY_FAIL and stops the unit.
  eval      21 cells; INCOMPLETE when any cell is skipped or fails (the analysis reads complete seed blocks).
Fail-closed preflight: approval naming SCOPE (and its SHA-256) and HEAD; clean tree; branch camonly/s2b; committed files;
launcher and play script at their pins; `git diff --name-only 335a1fa HEAD` inside CODE_ALLOWLIST (the simulation bytes
S2b-FT trained and CAM-DX1 ran with); the three checkpoints at their pins; the reference cells at their SHA-256 pins; base
environments equal to the recorded snapshots; every planned cell environment passes env_problems; one RTX 3070, no other
GPU compute, an idle host (GPU stages); import origins inside the tree; free disk >= 1 GB; new output folder. Host fault
count, boot id and online CPU list unchanged after every cell and at the end of each stage.
usage: tools/camref_eval_run.py {smoke,identity,eval} --approved-commit SHA --out DIR
       tools/camref_eval_run.py check --out DIR
       tools/camref_eval_run.py scope
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
import gr_run as G                                          # noqa: E402  (host, tree_problems, run_launcher)
import kroc2027_eval as K                                   # noqa: E402  (base_env, ledger checks, GPU, imports, budget)
import camk1e_run as R1                                     # noqa: E402  (approval format, ENV_ALLOW)
import cams2a_eval_run as S2A                               # noqa: E402  (module seeds, telemetry parsers, pins)
import cams2b_eval_run as W                                 # noqa: E402  (player pin)
import camdx1_eval_run as D                                 # noqa: E402  (S2b-FT's V312 cell, its training checks)

PREREG = "docs/plans/prereg_camonly_ref_matched_protocol_2026-11-26.md"
UNIT = "CAM-REF"
SCOPE = ("CAM-REF stage 1 v1: GPU evaluation of Policy F (ep25000 riskcap, f7022139...), CAM-K1 seed 1 (d7bf51a8...) and "
         "CAM-S2b-FT (f2c80fcc...) on one protocol ladder at 205 bars, quota 8 per env at 128 envs (1,024 episodes per "
         "cell): smoke F8, K1S and S6 at quota 1 (205 / 4501); identity F0ID (F at P_K, against GR-0 S00 205 / 4501, first "
         "8 episodes per env), K1ID (CAM-K1 at P_K, against CAM-K1-E K1s1 205 / 4501, first 8 per env) and S0ID (S2b-FT at "
         "P_S, against CAM-DX1 V312 205 / 4501), any difference IDENTITY_FAIL; eval F3, F4, F8, S3, S6, K1T, K1S x seeds "
         "4501-4503 (21 cells, block-major by seed); CAM-S2 capture / contact, delay and altitude / mount for F and K1 "
         "through navrl_camref_rules (detection = the policy's own _visible_now, no camsafe); wall cap 2.5 h shared by the "
         "three stages, cells that cannot finish inside it skipped; idle host; tools/camref_eval_run.py with "
         "eval_navrl_camref_arms.sh and tools/camref_entry.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "camref_20261126"
BRANCHES = ("camonly/s2b",)

# ---- policies and checkpoints ----------------------------------------------------------------------------------------
F_SHA = G.F_SHA
F_CKPT = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly") / G.F_CKPT_REL   # read only, SHA-checked
K1_SEED = 1
K1_SHA = R1.K1_SHA[K1_SEED]
K1_TRAIN_REPORT = R1.CAMK1_DIR / ("train_s%d" % K1_SEED) / "train_report.json"
FT_SHA = D.FT_CKPT_SHA
POLICY_SHA = {"F": F_SHA, "K1": K1_SHA, "S2B": FT_SHA}

# ---- reference cells (read only) and their SHA-256 pins ----------------------------------------------------------------
MAIN_TREE = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator")
REF_DIRS = {"GR0_S00": MAIN_TREE / "results/gain_randomization_gr0_v1/gr0_main/S00/E2_obstacle_aware__205bars__seed%d",
            "DX1_V312": pathlib.Path.home() / "motar_tmp/camdx1_20261125/a1/eval/V312/DX1_V312__205bars__seed%d",
            "K1E_K1s1": pathlib.Path.home() / "motar_tmp/camk1e_20261108/raw/eval_01/K1s1/E2_obstacle_aware__205bars__seed%d"}
REF_SHA = {
    ("GR0_S00", 4501): {"episodes.jsonl": "8899856375b19c6d1865e0f807827e479077325045bf82f1c44ab3113f77e6ad",
                        "receipt.json": "5efb24b110e60e03285110c81119e14782d394813667d42f18320460b7ebc860"},
    ("GR0_S00", 4502): {"episodes.jsonl": "d0d2269f3541517cdbbb54b073010f2f372959cbe1f415c7862e38c5f5a5167e",
                        "receipt.json": "bd31ae4b329d11131f55e6184306e3ea79ee1a50b38ddfebd9007d70e29846ec"},
    ("GR0_S00", 4503): {"episodes.jsonl": "fc59ff6356a246c4004ac703a0bfbd2069a956b724b7751c860adf9023c8931c",
                        "receipt.json": "c994042cea6eafe569f07c0b47cf8103888517e695968cb210b4b16c7f7ef025"},
    ("DX1_V312", 4501): {"episodes.jsonl": "dfbe4b9950b2bb81aa459c27b2eddb8f4b6109448030e55ce033a637e5a212fd",
                         "receipt.json": "1c46cddfe1cd4d46ae3e8fdeba26e3376635bdd26012ecd71bdf0fd20fed62c9",
                         "cams2diag_episodes.jsonl": "f359329eced69b62c32ee57ff7eb2ea090b4eb98fadc27f5581cc899f6cef68e"},
    ("DX1_V312", 4502): {"episodes.jsonl": "52033a6e017e6d2169b431fa7316aba321d7cfacfeb1eef48bd060b8d810c5ac",
                         "receipt.json": "8c1592ce30fb5318fa90deded0965eef37e9772bf464c3434e7f091926268bef",
                         "cams2diag_episodes.jsonl": "b51bffde090b8faac4756f79cad02c0c42816f67f1e9e10972fb05a4e7436814"},
    ("DX1_V312", 4503): {"episodes.jsonl": "0e515d99084f2ddf573efbc6b6df4b8ee430e98c752de225cf7feea3853cf50b",
                         "receipt.json": "e33808ff97ccd7f16e7d85b49d88eb996809642ede84ee93ae3de5be3e413443",
                         "cams2diag_episodes.jsonl": "1542ff3bb9792abf55091e46d0266b6cb8f07e987c7cd9b17aeb95c08ce8c699"},
    ("K1E_K1s1", 4501): {"episodes.jsonl": "f1868fa2fe8770c1b1cef5898cbcc856ba418e9a9b236981fa946441fbadfe10",
                         "receipt.json": "def209c371761b24b34e74c6f36b64ab511df45204af9b65cfaedccafced814b"},
    ("K1E_K1s1", 4502): {"episodes.jsonl": "62c9f65b97e35d018867a672f23f6fd135b8ea4da04ecc003d4aecfb9037a76b",
                         "receipt.json": "024439a0144d25688b35ad3065c7c009a8a12cb5c555af11710315fb036e0a6e"},
    ("K1E_K1s1", 4503): {"episodes.jsonl": "d407e34ccb2cdc9c9ad350de98354f28a97ab4094a3cb9de2a16bc861d5a546b",
                         "receipt.json": "c5d41ac9f1dd0ae69a2f3f24439a233b8b0f42c06df6c7e77f51cc706fd4d7b6"},
}
# GR-0 S00 205 / 4501 overlapped CPU jobs (gainrand/dev 352b919, load_overlap/overlap_table.md) but is byte-identical to
# its idle twins C1 / C1_r2 `full` (EXPERIMENTS, T-LOAD: "The loaded GR-0 cells of 2026-11-04 (205/4501 and 70/4501) equal
# the idle runs"), so a difference is not load: every identity difference is IDENTITY_FAIL (preregistration).
FIRST_N = 8                                  # episodes counted per env everywhere (the DX1 estimand)

# ---- environments ----------------------------------------------------------------------------------------------------
# The keys a cell sets per cell (runner or launcher); never part of a base environment.
PER_CELL = {"NAVRL_BULK_EVAL", "NAVRL_BULK_EVAL_JSON", "NAVRL_DENSITY_START", "NAVRL_DENSITY_FINAL", "NAVRL_NUM_BARS",
            "NAVRL_SEED", "NAVRL_EVAL_CHECKPOINT", "NAVRL_EVAL_RUN_NONCE", "NAVRL_EPISODE_LEDGER",
            "NAVRL_EPISODE_QUOTA_PER_ENV", "NAVRL_PLAYER_GAMES_CAP", "NAVRL_KROC2027_INITIAL_VIEW",
            "NAVRL_KROC2027_INITIAL_DISTANCE_M", "NAVRL_KROC2027_RECORDER", "NAVRL_KROC2027_RUN_ID", "NAVRL_KROC2027_ARM",
            "NAVRL_KROC2027_PROTOCOL_SHA256", "NAVRL_REALVEH_RECEIPT", "NAVRL_CAMTRAIN_RECEIPT", "NAVRL_CAMREF_RECEIPT",
            "NAVRL_CAMREF_RECORD_JSONL", "NAVRL_CAMS2DIAG_JSONL", "NAVRL_INVIEW_DIAG_JSONL"} | set(S2A.MODULE_SEED_KEYS) \
    | {"NAVRL_INVIEW_START_SEED"}
# Policy F's (and CAM-K1's) contract as GR-0's KROC2027 launcher held it fixed (eval_navrl_target_behavior_arms.sh HELD
# FIXED block + apply_arm E2_obstacle_aware + tools/gr_run.py's direct_l2_v1 backend); preflight binds it to the recorded
# GR-0 S00 205 / 4501-4503 snapshots minus PER_CELL.
F_BASE_ENV = {
    "NAVRL_ALT_HOLD_VMAX": "2.5", "NAVRL_ARENA_XY": "40", "NAVRL_ARENA_Z": "3", "NAVRL_BAR_POOL": "bars_h3",
    "NAVRL_BAR_X_MAX": "1.0", "NAVRL_BAR_X_MIN": "0.0", "NAVRL_DENSITY_CURRICULUM": "0", "NAVRL_DEPTH_NOISE_STD": "0.02",
    "NAVRL_DETECTION_DROPOUT": "0.3", "NAVRL_DETECTOR_THRESHOLD": "0.55", "NAVRL_EPISODE_LEN_STEPS": "600",
    "NAVRL_EVAL_CV_INITIAL_HEADING": "random", "NAVRL_GENERAL_GOAL_DIST_MAX": "28", "NAVRL_GENERAL_GOAL_DIST_MIN": "6",
    "NAVRL_GENERAL_TRAIN": "1", "NAVRL_LIDAR_HBEAMS": "72", "NAVRL_LIDAR_RANGE": "12", "NAVRL_LIDAR_VBEAMS": "4",
    "NAVRL_MAX_BARS": "300", "NAVRL_MAX_OBSTACLES": "8", "NAVRL_MAX_TILT_DEG": "45", "NAVRL_MAX_VELOCITY": "2.5",
    "NAVRL_OBSTACLE_CLUSTER_GAP_M": "0.45", "NAVRL_OBSTACLE_FOV_DEG": "240", "NAVRL_OBSTACLE_SECTORS": "8",
    "NAVRL_OBSTACLE_SELECTOR": "cluster_sector", "NAVRL_OBSTACLE_SUPPRESS_DEG": "10", "NAVRL_OBSTACLE_TTC_IDLE_S": "30",
    "NAVRL_OBSTACLE_TTC_MIN_SPEED": "0.15", "NAVRL_OOB_MARGIN": "1.0", "NAVRL_PERCEPTION": "1",
    "NAVRL_PLACEMENT_GAP_M": "1.6", "NAVRL_PLACEMENT_MODE": "navrl_band", "NAVRL_PLACEMENT_TOUCH_M": "0.4",
    "NAVRL_RGB_NOISE_STD": "0.015", "NAVRL_SPEED_GOVERNOR": "riskcap", "NAVRL_SPEED_GOVERNOR_BRAKE_MPS2": "2.9608856678",
    "NAVRL_SPEED_GOVERNOR_FIXED_MPS": "2.0", "NAVRL_SPEED_GOVERNOR_FREE_MPS": "3.53553390593",
    "NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M": "0.45", "NAVRL_SPEED_GOVERNOR_MARGIN_M": "0.45",
    "NAVRL_SPEED_GOVERNOR_REACTION_S": "0.1", "NAVRL_SPEED_GOVERNOR_RELEASE_M": "5.0", "NAVRL_SPEED_GOVERNOR_SLOW_M": "3.0",
    "NAVRL_SPEED_GOVERNOR_TTC_S": "1.2", "NAVRL_TARGET_BEHAVIOR_LEVEL": "e2_obstacle_aware",
    "NAVRL_TARGET_DISTANCE_BACKEND": "direct_l2_v1", "NAVRL_TARGET_DYNAMICS": "bounded", "NAVRL_TARGET_PATTERN": "waypoint",
    "NAVRL_TARGET_ROUTE_MODE": "off", "NAVRL_TARGET_SPEED_FINAL": "1.5", "NAVRL_TARGET_SPEED_MIN": "0.3",
    "NAVRL_TARGET_SPEED_RAMP_EPOCHS": "300", "NAVRL_TILT_COMP": "1", "NAVRL_V2_PROFILE": "main", "NAVRL_VISION": "1",
    "NAVRL_YAW_RATE_MAX": "3.0"}
E2_KEYS = {"NAVRL_TARGET_BEHAVIOR_LEVEL": "e2_obstacle_aware", "NAVRL_TARGET_DISTANCE_BACKEND": "direct_l2_v1",
           "NAVRL_TARGET_DYNAMICS": "bounded", "NAVRL_TARGET_PATTERN": "waypoint"}
LEGACY_MOTION = {"NAVRL_TARGET_BEHAVIOR_LEVEL": None, "NAVRL_TARGET_DISTANCE_BACKEND": None,
                 "NAVRL_TARGET_DYNAMICS": "legacy", "NAVRL_TARGET_PATTERN": "mixed"}       # S2b-FT's (DX1's) form
A3_VIEW = {"NAVRL_KROC2027_INITIAL_VIEW": "visible", "NAVRL_KROC2027_INITIAL_DISTANCE_M": "10.0"}
HALF_DEG = D.HALF_DEG                                   # "28.35"
V_GOAL_DIST_MAX = D.V_GOAL_DIST_MAX                     # "12"
CAPTURE = {"NAVRL_CAMREF_CAPTURE": "1", "NAVRL_CAMREF_CAPTURE_RADIUS_M": "1.0",
           "NAVRL_CAMREF_CAPTURE_MAX_CLOSING_MPS": "1.0", "NAVRL_CAMREF_CONTACT_RADIUS_M": "0.40",
           "NAVRL_CAMREF_DETECT_SOURCE": "visible_now"}             # the preregistration's fixed choice (fused for F)
DELAY = {"NAVRL_CAMREF_DELAY": "1", "NAVRL_CAMREF_DELAY_MIN": "1", "NAVRL_CAMREF_DELAY_MAX": "2"}   # seed per cell
GEOMETRY = {"NAVRL_CAMREF_GEOMETRY": "1", "NAVRL_CAMREF_FLIGHT_ALTITUDE_M": "1.5", "NAVRL_CAMREF_CAMERA_X_M": "0.08"}
VEH_RAND = {"NAVRL_VEH_RAND": "1", "NAVRL_VEH_RAND_LIMIT_MODE": "raise", "NAVRL_VEH_RAND_MOTOR_IMBAL": "0.05",
            "NAVRL_VEH_RAND_TAU_MAX": "0.04", "NAVRL_VEH_RAND_TAU_MIN": "0.02", "NAVRL_VEH_RAND_TW_NOMINAL": "3.5",
            "NAVRL_VEH_RAND_TW_SCALE": "0.15"}                         # S2b-FT's training values (seed per cell)
GAIN_RAND = {"NAVRL_GAIN_RAND": "1", "NAVRL_GAIN_RAND_SCALE": "0.2"}  # seed per cell
YAWCLIP = {"NAVRL_YAWCLIP": "1", "NAVRL_YAWCLIP_RADPS": S2A.YAWCLIP_RADPS}       # "1.047"; action scale stays 3.0
K1_PERCEPTION = {"NAVRL_SENSOR_ABLATION": "no_lidar", **R1.CAM_VALUES}         # 66.0 / 54.0 / 0.6 / 0.03 (K1-E)
RECORD = {"NAVRL_CAMREF_RECORD": "1"}
INVIEW_DIAG = {"NAVRL_INVIEW_DIAG": "1"}                                       # path per cell (runner, as DX1)
INVIEW_FILE, RECORD_FILE, DIAG_FILE = D.INVIEW_FILE, "camref_episodes.jsonl", S2A.DIAG_FILE


def module_seed(key, seed):
    return S2A.module_seeds(seed)[key]


def f_task_rules(seed):
    """P_T task rules for F / K1 on top of F_BASE_ENV: V312 start, legacy mixed motion, CAM-S2 capture / contact."""
    d = dict(LEGACY_MOTION)
    d.update(NAVRL_GENERAL_GOAL_DIST_MAX=V_GOAL_DIST_MAX, NAVRL_INVIEW_START="1", NAVRL_INVIEW_START_HALF_DEG=HALF_DEG,
             NAVRL_INVIEW_START_SEED=str(int(seed)), **INVIEW_DIAG, **CAPTURE)
    return d


# The P_T -> P_S factors of Policy F (stage 2 adds each one alone to F4; F8 has all five):
#   delay = U{1,2} steps; rand = vehicle + gain randomisation; yaw = clip 1.047 rad/s (scale kept 3.0);
#   free = governor FREE 2.5 m/s; geom = altitude 1.5 m / camera mount 0.08 m.
F_FACTOR_KEYS = ("delay", "rand", "yaw", "free", "geom")
F_FACTORS = {"F4": (), "F4D": ("delay",), "F4R": ("rand",), "F4Y": ("yaw",), "F4V": ("free",), "F4A": ("geom",),
             "F8": F_FACTOR_KEYS}


def f_factor_delta(factors, seed):
    """The keys of the named P_T -> P_S factors (key order as F8 had it in stage 1)."""
    unknown = set(factors) - set(F_FACTOR_KEYS)
    if unknown:
        raise ValueError("unknown factor(s) %s" % sorted(unknown))
    d = {}
    if "delay" in factors:
        d.update(DELAY, NAVRL_CAMREF_DELAY_SEED=module_seed("NAVRL_CAMS2_DELAY_SEED", seed))
    if "rand" in factors:
        d.update(VEH_RAND, NAVRL_VEH_RAND_SEED=module_seed("NAVRL_VEH_RAND_SEED", seed), **GAIN_RAND)
        d["NAVRL_GAIN_RAND_SEED"] = module_seed("NAVRL_GAIN_RAND_SEED", seed)
    if "yaw" in factors:
        d.update(YAWCLIP)
    if "geom" in factors:
        d.update(GEOMETRY)
    if "free" in factors:
        d["NAVRL_SPEED_GOVERNOR_FREE_MPS"] = "2.5"
    return d


def f8_extra(seed):
    return f_factor_delta(F_FACTOR_KEYS, seed)


S2_FACTOR_RUNGS = ("F4D", "F4R", "F4Y", "F4V", "F4A")


def factors(arm_or_rung):
    """The P_T -> P_S factors of a rung (K1S has F8's; every other rung none)."""
    r = arm_or_rung if arm_or_rung in F_FACTORS or arm_or_rung in RUNG_PROTOCOL else rung(arm_or_rung)
    return F_FACTOR_KEYS if r == "K1S" else F_FACTORS.get(r, ())


def rung_delta(rung, seed):
    """The keys a rung sets (value) or removes (None) relative to its policy's base environment."""
    if rung == "F0":
        return dict(A3_VIEW, **RECORD)
    if rung == "K1PK":
        return dict(A3_VIEW, **RECORD, **K1_PERCEPTION)
    if rung in ("F3", "K1T"):
        d = f_task_rules(seed)
    elif rung in F_FACTORS:                                      # F4, the stage-2 single-factor rungs, F8
        d = dict(f_task_rules(seed), NAVRL_DYNAMIC_MESH_TREATMENT="mesh_flat", **f_factor_delta(F_FACTORS[rung], seed))
    elif rung == "K1S":
        d = dict(f_task_rules(seed), NAVRL_DYNAMIC_MESH_TREATMENT="mesh_flat", **f8_extra(seed))
    elif rung == "S0":
        return {}
    elif rung == "S3":
        return s3_delta()
    elif rung == "S6":
        return s6_delta()
    else:
        raise ValueError("unknown rung %r" % rung)
    if rung == "K1T":
        d["NAVRL_DYNAMIC_MESH_TREATMENT"] = "mesh_flat"            # K1T = F4's settings
    if rung in ("K1T", "K1S"):
        d.update(K1_PERCEPTION)
    d.update(RECORD)
    return d


def s3_delta():
    """S2b-FT at P_T: altitude 1.0 / mount 0.10, delay 0 (cams2 parameters), vehicle and gain randomisation off."""
    d = {"NAVRL_CAMS2_FLIGHT_ALTITUDE_M": "1.0", "NAVRL_CAMS2_CAMERA_X_M": "0.10", "NAVRL_CAMS2_DELAY_MIN": "0",
         "NAVRL_CAMS2_DELAY_MAX": "0", "NAVRL_ACTION_DELAY_STEPS_MAX": "0"}
    for k in VEH_RAND:
        d[k] = None
    d.update(NAVRL_VEH_RAND_SEED=None, NAVRL_GAIN_RAND=None, NAVRL_GAIN_RAND_SCALE=None, NAVRL_GAIN_RAND_SEED=None)
    return d


S2_CAMS2_KEYS = ("NAVRL_CAMS2", "NAVRL_CAMS2_DELAY_MIN", "NAVRL_CAMS2_DELAY_MAX", "NAVRL_CAMS2_DELAY_SEED",
                 "NAVRL_CAMS2_CAPTURE_RADIUS_M", "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS", "NAVRL_CAMS2_CONTACT_RADIUS_M",
                 "NAVRL_CAMS2_FLIGHT_ALTITUDE_M", "NAVRL_CAMS2_CAMERA_X_M", "NAVRL_ACTION_DELAY_STEPS_MAX",
                 "NAVRL_CAMS2_CONTACTCODE", "NAVRL_CAMS2DIAG", "NAVRL_CAMS2DIAG_YAW_REF_RADPS")


def s6_delta():
    """S2b-FT at P_K: S3 minus the CAM-S2 capture rule (cams2, its contact code and recorder off; 1.0 m / 0.10 m / no delay
    are then the task defaults, as in S3), E2 target, A3 start (goal maximum back to the trained 28 m)."""
    d = s3_delta()
    for k in S2_CAMS2_KEYS:
        d[k] = None
    d.update(E2_KEYS)
    d.update(A3_VIEW)
    d.update(NAVRL_INVIEW_START=None, NAVRL_INVIEW_START_HALF_DEG=None, NAVRL_INVIEW_START_SEED=None,
             NAVRL_GENERAL_GOAL_DIST_MAX=D.TRAINED_ENV["NAVRL_GENERAL_GOAL_DIST_MAX"], **RECORD)
    return d


# ---- arms ------------------------------------------------------------------------------------------------------------
ARMS = {"F0ID": {"policy": "F", "rung": "F0", "label": "CAMREF_F0ID"},
        "S0ID": {"policy": "S2B", "rung": "S0", "label": "CAMREF_S0ID"},
        "F3": {"policy": "F", "rung": "F3", "label": "CAMREF_F3"},
        "F4": {"policy": "F", "rung": "F4", "label": "CAMREF_F4"},
        "F8": {"policy": "F", "rung": "F8", "label": "CAMREF_F8"},
        "S3": {"policy": "S2B", "rung": "S3", "label": "CAMREF_S3"},
        "S6": {"policy": "S2B", "rung": "S6", "label": "CAMREF_S6"},
        "K1ID": {"policy": "K1", "rung": "K1PK", "label": "CAMREF_K1ID"},
        "K1T": {"policy": "K1", "rung": "K1T", "label": "CAMREF_K1T"},
        "K1S": {"policy": "K1", "rung": "K1S", "label": "CAMREF_K1S"},
        # stage 2 (docs/plans/prereg_camonly_ref_stage2_2026-11-26.md): F4 plus one factor; F4 / F8 identity reruns
        "F4D": {"policy": "F", "rung": "F4D", "label": "CAMREF_F4D"},
        "F4R": {"policy": "F", "rung": "F4R", "label": "CAMREF_F4R"},
        "F4Y": {"policy": "F", "rung": "F4Y", "label": "CAMREF_F4Y"},
        "F4V": {"policy": "F", "rung": "F4V", "label": "CAMREF_F4V"},
        "F4A": {"policy": "F", "rung": "F4A", "label": "CAMREF_F4A"},
        "F4ID": {"policy": "F", "rung": "F4", "label": "CAMREF_F4ID"},
        "F8ID": {"policy": "F", "rung": "F8", "label": "CAMREF_F8ID"}}
ARM_BASE = {"F8_smoke": "F8", "K1S_smoke": "K1S", "S6_smoke": "S6", "F4D_smoke": "F4D", "F4R_smoke": "F4R",
            "F4Y_smoke": "F4Y", "F4V_smoke": "F4V", "F4A_smoke": "F4A"}
DENSITY = 205
SEEDS = (4501, 4502, 4503)
NUM_ENVS, QUOTA, SMOKE_QUOTA = 128, 8, 1
SMOKE_PLAN = (("F8_smoke", DENSITY, 4501), ("K1S_smoke", DENSITY, 4501), ("S6_smoke", DENSITY, 4501))
IDENTITY_PLAN = (("F0ID", DENSITY, 4501), ("K1ID", DENSITY, 4501), ("S0ID", DENSITY, 4501))
IDENTITY_REF = {"F0ID": ("GR0_S00", 4501), "K1ID": ("K1E_K1s1", 4501), "S0ID": ("DX1_V312", 4501)}
EVAL_ARMS = ("F3", "F4", "F8", "S3", "S6", "K1T", "K1S")
MAX_HOURS_TOTAL = 2.5                         # the preregistered wall cap of stage 1 (all three stages)
STAGE_SUB_CAP_H = {"smoke": 0.5, "identity": 0.75, "eval": 2.5}
CELL_TIMEOUT_S = {"F": 1800, "K1": 1800, "S2B": 2400}
PROJECTION_MARGIN = 1.5
MIN_FREE_GB = 1.0
IDLE_MAX_BUSY = 0.15                           # an idle host: < 15 % of the online CPUs busy over a 2 s sample
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_camref_arms.sh"
LAUNCHER_SHA = "51f949d08ec70935aeca13fe3b28ce289ff208edeccc526d4b0410662573518a"   # the committed copy
WRAPPER = ROOT / "tools/camref_python"
ENTRY = "tools/camref_entry.py"
RULES_REL = "aerial_gym/task/navrl_task/navrl_camref_rules.py"
YAWCLIP_REL = S2A.YAWCLIP_REL
REAL_PYTHON, SIM_NAME = S2A.REAL_PYTHON, S2A.SIM_NAME
NEW_FILES = ("tools/camref_eval_run.py", "tools/camref_eval_analyse.py", "tests/test_camref.py", ENTRY,
             "tools/camref_python", RULES_REL, LAUNCHER)
COMMITTED_FILES = (PREREG, "tools/realveh_entry.py", "tools/realveh_python", D.INVIEW_REL, YAWCLIP_REL,
                   "tools/camdx1_eval_run.py") + NEW_FILES
FT_TRAIN_HEAD = D.FT_TRAIN_HEAD               # 335a1fa: S2b-FT trained here; CAM-DX1 ran on the same simulation bytes
CODE_ALLOWLIST = ("docs/*", "WORKLOG.md", "tests/*", "tools/camdx1_*", "tools/camref_*", "tools/realveh_entry.py",
                  D.INVIEW_REL, RULES_REL, LAUNCHER)
LAUNCHER_SETS = {"NAVRL_BULK_EVAL", "NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_CHECKPOINT", "NAVRL_EVAL_RUN_NONCE",
                 "NAVRL_NUM_BARS", "NAVRL_SEED", "NAVRL_DENSITY_START", "NAVRL_DENSITY_FINAL", "NAVRL_CAMREF_RECEIPT",
                 "NAVRL_REALVEH_RECEIPT", "NAVRL_CAMREF_RECORD_JSONL", "NAVRL_CAMS2DIAG_JSONL"}
# Checkpoint-contract warnings each cell may print (anything else fails the cell): the common ones (training seed,
# density-curriculum keys), the P9 module seed for S2b-FT, the E2 axis for a policy trained on the legacy motion (F, K1 at
# P_K; S2b-FT at S6), and the visible start's goal maximum (28 -> 12 m).
MISMATCH_COMMON = (r"TRAINING SEED MISMATCH \|",
                   r"CURRICULUM CONFIG MISMATCH \| (NAVRL_DENSITY_\w+|cfg_density_\w+|n_bars\w*|density\w*)[: ]")
MISMATCH_P9 = (r"CONFIG MISMATCH \| NAVRL_P9_SEED:",)
MISMATCH_E2 = (r"TARGET BEHAVIOR MISMATCH \| checkpoint=historical running=e2_obstacle_aware\.",
               r"TARGET DISTANCE BACKEND MISMATCH \| checkpoint=legacy_cdist running=direct_l2_v1\.",
               r"TARGET MOTION MISMATCH \| checkpoint=symmetric_local_steer_v2_heading_continuity90 "
               r"running=bounded_planar_drone_v1_rollout_direct_l2_v1\.",
               r"TASK CONTRACT MISMATCH \| NAVRL_TARGET_PATTERN: checkpoint=mixed running=waypoint\.")
MISMATCH_V = D.MISMATCH_V
SMOKE_CONTROLS = ("every smoke cell OK with >= 1 finished episode",
                  "F8_smoke and K1S_smoke: [camref] rules telemetry with delay draws of both 1 and 2 steps and >= 1 capture "
                  "or target contact; [veh-rand] init line; [yawclip] line with commands > 0",
                  "F8_smoke and K1S_smoke: every rendered step-0 bearing within +-28.35 deg (visible start)",
                  "S6_smoke: every rendered step-0 bearing within +-2 deg (A3 boresight start), no [cams2diag] / "
                  "[cams2contact] line, a [camref] recorder line")
A3_BEARING_TOL_DEG = 2.0


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def cfg(arm):
    return ARMS[ARM_BASE.get(arm, arm)]


def policy(arm):
    return cfg(arm)["policy"]


def rung(arm):
    return cfg(arm)["rung"]


def camref_rules_on(arm):
    """The CAM-REF rule module (capture / contact, and any delay / geometry) is installed: F and K1 at P_T / P_S."""
    return rung(arm) in ("F3", "K1T", "K1S") or rung(arm) in F_FACTORS


def visible_start(arm):
    return camref_rules_on(arm) or rung(arm) in ("S0", "S3")


def cams2_on(arm):
    return rung(arm) in ("S0", "S3")


def camref_record(arm):
    return rung(arm) not in ("S0", "S3")


def cell_id(arm, density, seed):
    return "%s__%dbars__seed%d" % (cfg(arm)["label"], int(density), int(seed))


def scope_digest():
    return hashlib.sha256(SCOPE.encode("utf-8")).hexdigest()


def strip_per_cell(env):
    return {k: v for k, v in env.items() if k.startswith("NAVRL_") and k not in PER_CELL}


def apply_delta(base, delta):
    out = dict(base)
    for k, v in delta.items():
        if v is None:
            out.pop(k, None)
        else:
            out[k] = v
    return out


# ---------------------------------------------------------------- environments
def s2b_base_env(train_env, seed, cell_dir, quota, cap):
    """CAM-DX1's V312 cell (FT training environment + CAM-S2b-FT-E's FT replacements + DX1's recorder and arm keys)."""
    return D.cell_navrl_env(train_env, seed, "V312", cell_dir, quota, cap)


def base_env(arm, seed, cell_dir, quota, cap, train_env):
    """The NAVRL_* base of a cell (with the runner's per-cell keys): F / K1 from F_BASE_ENV, S2b-FT from CAM-DX1's V312."""
    if policy(arm) == "S2B":
        env = s2b_base_env(train_env, seed, cell_dir, quota, cap)
    else:
        env = dict(F_BASE_ENV, NAVRL_EPISODE_LEDGER="1", NAVRL_EPISODE_QUOTA_PER_ENV=str(int(quota)),
                   NAVRL_PLAYER_GAMES_CAP=str(int(cap)))
    return env


def cell_navrl_env(arm, seed, cell_dir, quota, cap, train_env, protocol_sha=None):
    """Every NAVRL_* key the runner exports for one cell (the launcher adds LAUNCHER_SETS)."""
    env = apply_delta(base_env(arm, seed, cell_dir, quota, cap, train_env), rung_delta(rung(arm), seed))
    env["NAVRL_CAMREF_POLICY"] = policy(arm)
    if "NAVRL_INVIEW_DIAG" in env:
        env["NAVRL_INVIEW_DIAG_JSONL"] = str(pathlib.Path(cell_dir) / INVIEW_FILE)
    if rung(arm) in ("F0", "K1PK"):                        # the KROC2027 recorder, as GR-0 S00 and CAM-K1-E ran
        env.update(NAVRL_KROC2027_RECORDER="1", NAVRL_KROC2027_RUN_ID="CAMREF-" + ARM_BASE.get(arm, arm),
                   NAVRL_KROC2027_ARM="A3", NAVRL_KROC2027_PROTOCOL_SHA256=protocol_sha or "")
    return env


# The preregistration's protocol points, written out per rung (independent of the RUNG_DELTA construction above):
# start, target motion, capture rule, target appearance, delay steps, vehicle / gain randomisation, yaw, governor FREE,
# altitude / camera x, camera-blind cap, sensor. env_problems derives the same fields from each cell's environment.
P_K = {"start": "A3", "motion": "E2", "capture": "swept_0.5", "delay": (0, 0), "veh_rand": False, "gain_rand": False,
       "altitude_mount": (1.0, 0.10)}
P_T = {"start": "V312", "motion": "legacy_mixed", "capture": "cs2", "target": "D8", "delay": (0, 0), "veh_rand": False,
       "gain_rand": False, "altitude_mount": (1.0, 0.10)}
P_S = {"start": "V312", "motion": "legacy_mixed", "capture": "cs2", "target": "D8", "delay": (1, 2), "veh_rand": True,
       "gain_rand": True, "altitude_mount": (1.5, 0.08)}
F_STACK = {"yaw": "scale_3.0", "free_mps": 3.53553390593, "camsafe": False, "sensor": "camera+lidar"}
K1_STACK = dict(F_STACK, sensor="camera_66x54")
S2B_STACK = {"yaw": "scale_1.0472", "free_mps": 2.5, "camsafe": True, "sensor": "camera_66.7x52.6_312x234"}
RUNG_PROTOCOL = {
    "F0": dict(P_K, target="analytic", **F_STACK),
    "F3": dict(P_T, target="analytic", **F_STACK),
    "F4": dict(P_T, **F_STACK),
    "F8": dict(P_S, **dict(F_STACK, yaw="clip_1.047", free_mps=2.5)),
    "F4D": dict(P_T, **dict(F_STACK, delay=(1, 2))),                 # stage 2: F4 plus one factor each
    "F4R": dict(P_T, **dict(F_STACK, veh_rand=True, gain_rand=True)),
    "F4Y": dict(P_T, **dict(F_STACK, yaw="clip_1.047")),
    "F4V": dict(P_T, **dict(F_STACK, free_mps=2.5)),
    "F4A": dict(P_T, **dict(F_STACK, altitude_mount=(1.5, 0.08))),
    "K1PK": dict(P_K, target="analytic", **K1_STACK),
    "K1T": dict(P_T, **K1_STACK),
    "K1S": dict(P_S, **dict(K1_STACK, yaw="clip_1.047", free_mps=2.5)),
    "S0": dict(P_S, **S2B_STACK),
    "S3": dict(P_T, **S2B_STACK),
    "S6": dict(P_K, target="D8", **S2B_STACK)}


def _f(env, key, default=None):
    try:
        return float(env[key])
    except (KeyError, ValueError):
        return default


def protocol_point(env):
    """The protocol fields of a cell environment (see RUNG_PROTOCOL)."""
    view = env.get("NAVRL_KROC2027_INITIAL_VIEW") == "visible" and _f(env, "NAVRL_KROC2027_INITIAL_DISTANCE_M") == 10.0
    inview = (env.get("NAVRL_INVIEW_START") == "1" and _f(env, "NAVRL_INVIEW_START_HALF_DEG") == float(HALF_DEG)
              and _f(env, "NAVRL_GENERAL_GOAL_DIST_MAX") == float(V_GOAL_DIST_MAX))
    start = "A3" if view and not inview else ("V312" if inview and not view else "other")
    e2 = all(env.get(k) == v for k, v in E2_KEYS.items())
    legacy = (env.get("NAVRL_TARGET_DYNAMICS") == "legacy" and env.get("NAVRL_TARGET_PATTERN") == "mixed"
              and env.get("NAVRL_TARGET_BEHAVIOR_LEVEL", "historical") == "historical"
              and env.get("NAVRL_TARGET_DISTANCE_BACKEND", "legacy_cdist") == "legacy_cdist")
    motion = "E2" if e2 else ("legacy_mixed" if legacy else "other")
    cams2 = env.get("NAVRL_CAMS2") == "1"
    camref = env.get("NAVRL_CAMREF_CAPTURE") == "1"
    pre = "NAVRL_CAMS2_" if cams2 else "NAVRL_CAMREF_"
    rule_ok = (_f(env, pre + "CAPTURE_RADIUS_M") == 1.0 and _f(env, pre + "CAPTURE_MAX_CLOSING_MPS") == 1.0
               and _f(env, pre + "CONTACT_RADIUS_M") == 0.4
               and (cams2 or env.get("NAVRL_CAMREF_DETECT_SOURCE") == "visible_now"))
    capture = ("swept_0.5" if not cams2 and not camref else ("cs2" if (cams2 != camref) and rule_ok else "other"))
    if cams2:
        delay = (int(_f(env, "NAVRL_CAMS2_DELAY_MIN", -1)), int(_f(env, "NAVRL_CAMS2_DELAY_MAX", -1)))
        geo = (_f(env, "NAVRL_CAMS2_FLIGHT_ALTITUDE_M"), _f(env, "NAVRL_CAMS2_CAMERA_X_M"))
    else:
        delay = ((int(_f(env, "NAVRL_CAMREF_DELAY_MIN", -1)), int(_f(env, "NAVRL_CAMREF_DELAY_MAX", -1)))
                 if env.get("NAVRL_CAMREF_DELAY") == "1" else (0, 0))
        geo = ((_f(env, "NAVRL_CAMREF_FLIGHT_ALTITUDE_M"), _f(env, "NAVRL_CAMREF_CAMERA_X_M"))
               if env.get("NAVRL_CAMREF_GEOMETRY") == "1" else (1.0, 0.10))
    veh = env.get("NAVRL_VEH_RAND") == "1" and all(env.get(k) == v for k, v in VEH_RAND.items())
    gain = env.get("NAVRL_GAIN_RAND") == "1" and env.get("NAVRL_GAIN_RAND_SCALE") == "0.2"
    if env.get("NAVRL_YAWCLIP") == "1":
        yaw = "clip_%s" % env.get("NAVRL_YAWCLIP_RADPS") if env.get("NAVRL_YAW_RATE_MAX") == "3.0" else "other"
    else:
        yaw = "scale_%s" % env.get("NAVRL_YAW_RATE_MAX")
    abl = env.get("NAVRL_SENSOR_ABLATION")
    if abl is None:
        sensor = "camera+lidar" if not any(k.startswith("NAVRL_CAMTRAIN_") for k in env) else "other"
    elif abl == "no_lidar" and all(env.get(k) == v for k, v in (("NAVRL_CAMERA_WIDTH", "312"), ("NAVRL_CAMERA_HEIGHT", "234"),
                                                                    ("NAVRL_DETECT_WIDTH", "312"), ("NAVRL_DETECT_HEIGHT", "234"))):
        sensor = "camera_%gx%g_312x234" % (_f(env, "NAVRL_CAMTRAIN_HFOV_DEG", 0.0), _f(env, "NAVRL_CAMTRAIN_VFOV_DEG", 0.0))
    elif abl == "no_lidar" and "NAVRL_CAMERA_WIDTH" not in env:
        sensor = "camera_%gx%g" % (_f(env, "NAVRL_CAMTRAIN_HFOV_DEG", 0.0), _f(env, "NAVRL_CAMTRAIN_VFOV_DEG", 0.0))
    else:
        sensor = "other"
    return {"start": start, "motion": motion, "capture": capture,
            "target": "D8" if env.get("NAVRL_DYNAMIC_MESH_TREATMENT") == "mesh_flat" else "analytic", "delay": delay,
            "veh_rand": veh, "gain_rand": gain, "altitude_mount": geo, "yaw": yaw,
            "free_mps": _f(env, "NAVRL_SPEED_GOVERNOR_FREE_MPS"), "camsafe": env.get("NAVRL_CAMSAFE") == "1",
            "sensor": sensor}


def protocol_problems(env, arm):
    got, want = protocol_point(env), RUNG_PROTOCOL[rung(arm)]
    return ["protocol %s of rung %s is %r, preregistered %r" % (k, rung(arm), got.get(k), v)
            for k, v in want.items() if got.get(k) != v]


def expected_cell_env(arm, seed, cell_dir, quota, cap, train_env, protocol_sha=None):
    """The environment the runner must export for the arm (env_problems compares it key by key and, independently,
    derives the rung's protocol fields from it against RUNG_PROTOCOL)."""
    return cell_navrl_env(arm, seed, cell_dir, quota, cap, train_env, protocol_sha)


def env_problems(env, arm, seed, cell_dir, quota, cap, train_env, protocol_sha=None):
    """Before launch: the cell's NAVRL_* variables are exactly the policy base plus the rung's keys plus the per-cell keys;
    no launcher-set key is preset; the policy names the arm's checkpoint; rules / modules the arm must not have are absent."""
    env = {k: v for k, v in env.items() if k.startswith("NAVRL_")}
    p = []
    want = expected_cell_env(arm, seed, cell_dir, quota, cap, train_env, protocol_sha)
    diff = sorted(k for k in set(env) | set(want) if env.get(k) != want.get(k))
    if diff:
        p.append("cell environment differs from the arm's construction in %s" % diff)
    p += protocol_problems(env, arm)
    preset = sorted(k for k in LAUNCHER_SETS if k in env)
    if preset:
        p.append("cell environment presets launcher keys %s" % preset)
    if env.get("NAVRL_CAMREF_POLICY") != policy(arm):
        p.append("NAVRL_CAMREF_POLICY %r is not %s" % (env.get("NAVRL_CAMREF_POLICY"), policy(arm)))
    pol, r = policy(arm), rung(arm)
    if pol in ("F", "K1") and any(k.startswith(("NAVRL_CAMS2", "NAVRL_CAMSAFE", "NAVRL_DEPTH_MODEL", "NAVRL_P9_"))
                                  for k in env):
        p.append("policy %s cell carries S2b-FT's modules" % pol)
    if pol == "F" and "NAVRL_SENSOR_ABLATION" in env:
        p.append("Policy F cell carries a sensor ablation")
    if pol == "K1" and env.get("NAVRL_SENSOR_ABLATION") != "no_lidar":
        p.append("CAM-K1 cell is not no_lidar")
    if pol == "S2B" and any(k in env for k in ("NAVRL_CAMREF_CAPTURE", "NAVRL_CAMREF_DELAY", "NAVRL_CAMREF_GEOMETRY",
                                                "NAVRL_YAWCLIP")):
        p.append("S2b-FT cell carries CAM-REF rules or a yaw clip")
    if ("NAVRL_CAMS2" in env) != cams2_on(arm) or ("NAVRL_CAMREF_RECORD" in env) != camref_record(arm):
        p.append("cams2 / CAM-REF recorder state does not match rung %s" % r)
    if ("NAVRL_INVIEW_START" in env) != visible_start(arm):
        p.append("visible-start state does not match rung %s" % r)
    if ("NAVRL_KROC2027_INITIAL_VIEW" in env) == visible_start(arm):
        p.append("A3-start state does not match rung %s" % r)
    return p


def cell_env(out_dir, arm, density, seed, ckpt, ckpt_sha, train_env, protocol, num_envs, quota):
    cell_dir = out_dir / cell_id(arm, density, seed)
    inherited = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_") and k != "GPU4GB"}
    env = K.base_env(root=ROOT, inherited=inherited)
    env.update(cell_navrl_env(arm, seed, cell_dir, quota, protocol["evaluation"]["player_games_cap"], train_env,
                              protocol_sha()))
    env.update(ARMS=cfg(arm)["label"], DENSITIES=str(int(density)), SEEDS=str(int(seed)), GAMES=str(quota * num_envs),
               NUM_ENVS=str(num_envs), OUT_ROOT=str(out_dir), CAMEVAL_PY=str(WRAPPER), CAMREF_CKPT=str(ckpt),
               CAMREF_CKPT_SHA=ckpt_sha, AERIAL_GYM_SIM_NAME=SIM_NAME, PYTHON=REAL_PYTHON)   # pinned, not inherited
    return env


def protocol_sha():
    return K.kp.sha256_file(K.PROTOCOL)


def launcher_values(arm, density, seed, cell_dir, ckpt_sha):
    """The keys the launcher adds to a cell (its snapshot)."""
    want = {"NAVRL_BULK_EVAL": "1", "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"),
            "NAVRL_EVAL_CHECKPOINT": ckpt_sha, "NAVRL_NUM_BARS": str(density), "NAVRL_DENSITY_START": str(density),
            "NAVRL_DENSITY_FINAL": str(density), "NAVRL_SEED": str(seed),
            "NAVRL_CAMREF_RECEIPT": str(cell_dir / "camref_receipt.json")}
    if policy(arm) != "F":
        want["NAVRL_REALVEH_RECEIPT"] = str(cell_dir / "realveh_receipt.json")
    if camref_record(arm):
        want["NAVRL_CAMREF_RECORD_JSONL"] = str(cell_dir / RECORD_FILE)
    if cams2_on(arm):
        want["NAVRL_CAMS2DIAG_JSONL"] = str(cell_dir / DIAG_FILE)
    return want


def snapshot_problems(snap, arm, density, seed, cell_dir, ckpt_sha, quota, train_env, label="snapshot"):
    """A NAVRL snapshot equals the runner's cell environment plus the launcher's keys, except the per-run nonce."""
    want = dict(cell_navrl_env(arm, seed, cell_dir, quota, 10 ** 9, train_env, protocol_sha()))
    want.update(launcher_values(arm, density, seed, cell_dir, ckpt_sha))
    got = {k: v for k, v in (snap or {}).items() if k != "NAVRL_EVAL_RUN_NONCE"}
    diff = sorted(k for k in set(got) | set(want) if got.get(k) != want.get(k))
    p = ["%s differs from the cell's environment in %s" % (label, diff)] if diff else []
    if not (snap or {}).get("NAVRL_EVAL_RUN_NONCE"):
        p.append("%s lacks NAVRL_EVAL_RUN_NONCE" % label)
    return p


# ---------------------------------------------------------------- base-environment provenance (preflight)
def reference_dir(ref, seed):
    return pathlib.Path(str(REF_DIRS[ref]) % int(seed))


def reference_problems():
    """Every reference file at its SHA-256 pin."""
    p = []
    for (ref, seed), files in sorted(REF_SHA.items()):
        for name, pin in files.items():
            path = reference_dir(ref, seed) / name
            if not path.is_file() or sha(path) != pin:
                p.append("reference %s missing or not its pin %s..." % (path, pin[:12]))
    return p


def base_provenance_problems(train_env, cap):
    """F_BASE_ENV equals each recorded GR-0 S00 205 snapshot minus PER_CELL; CAM-DX1's V312 construction equals each
    recorded DX1 V312 205 snapshot minus PER_CELL (and the receipt-path keys, which differ by entry)."""
    p = []
    for seed in SEEDS:
        try:
            gr = json.loads((reference_dir("GR0_S00", seed) / "receipt.json").read_text())["env_snapshot_navrl"]
            dx = json.loads((reference_dir("DX1_V312", seed) / "receipt.json").read_text())["env_snapshot_navrl"]
        except (OSError, ValueError, KeyError) as e:
            p.append("reference snapshot unreadable for seed %d: %s" % (seed, e))
            continue
        a, b = strip_per_cell(gr), F_BASE_ENV
        diff = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
        if diff:
            p.append("F_BASE_ENV differs from GR-0 S00 205 / %d in %s" % (seed, diff))
        if gr.get("NAVRL_KROC2027_INITIAL_VIEW") != "visible" or gr.get("NAVRL_KROC2027_INITIAL_DISTANCE_M") != "10.0":
            p.append("GR-0 S00 205 / %d is not the A3 start" % seed)
        try:
            k1 = json.loads((reference_dir("K1E_K1s1", seed) / "receipt.json").read_text())["env_snapshot_navrl"]
        except (OSError, ValueError, KeyError) as e:
            p.append("CAM-K1-E snapshot unreadable for seed %d: %s" % (seed, e))
            k1 = None
        if k1 is not None:
            a, b = strip_per_cell(k1), dict(F_BASE_ENV, **K1_PERCEPTION)
            diff = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
            if diff:
                p.append("F_BASE_ENV + CAM-K1's perception differs from CAM-K1-E K1s1 205 / %d in %s" % (seed, diff))
            if k1.get("NAVRL_KROC2027_INITIAL_VIEW") != "visible":
                p.append("CAM-K1-E K1s1 205 / %d is not the A3 start" % seed)
        try:
            built = strip_per_cell(s2b_base_env(train_env, seed, reference_dir("DX1_V312", seed), QUOTA, cap))
        except ValueError as e:
            p.append("CAM-DX1 V312 construction refused: %s" % e)
            continue
        a = strip_per_cell(dx)
        diff = sorted(k for k in set(a) | set(built) if a.get(k) != built.get(k))
        if diff:
            p.append("CAM-DX1 V312 construction differs from DX1 V312 205 / %d in %s" % (seed, diff))
    return p


# ---------------------------------------------------------------- per-cell checks
def condition_problems(cond, arm, train_env):
    want = {"num_envs": NUM_ENVS, "episode_len_steps": 600, "full_goal_distribution": True,
            "fov_curriculum_saturated": True, "speed_governor_mode": "riskcap",
            "goal_dist_max_m": 12.0 if visible_start(arm) else 28.0,
            "speed_governor_free_mps": 2.5 if (policy(arm) == "S2B" or "free" in factors(arm)) else 3.53553390593,
            "p9_empirical_error_enabled": policy(arm) == "S2B",
            "d8_dynamic_mesh_treatment": rung(arm) not in ("F0", "F3", "K1PK")}
    p = ["result condition %s is %r, expected %r" % (k, cond.get(k), v) for k, v in want.items() if cond.get(k) != v]
    contract = cond.get("d8_dynamic_mesh_contract")
    if want["d8_dynamic_mesh_treatment"]:
        if not isinstance(contract, dict) or contract.get("mode") != S2A.D8_MODE:
            p.append("result condition d8_dynamic_mesh_contract %r is not mode %s" % (contract, S2A.D8_MODE))
    elif contract is not None:
        p.append("result condition carries a D8 contract in an analytic-target rung")
    if policy(arm) == "S2B" and cond.get("p9_empirical_error_sha256") != train_env.get("NAVRL_P9_EXPECTED_SHA256"):
        p.append("result condition P9 SHA %r is not training's" % cond.get("p9_empirical_error_sha256"))
    return p


def output_problems(cell_dir, arm, commit, ckpt_sha, train_env, num_envs, quota):
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
        p.append("receipt/result checkpoint is not %s" % ckpt_sha[:12])
    if receipt.get("camref_policy") != policy(arm):
        p.append("receipt camref_policy %r is not %s" % (receipt.get("camref_policy"), policy(arm)))
    if receipt.get("sim_name") != SIM_NAME or receipt.get("python") != REAL_PYTHON or receipt.get("gpu4gb", "") != "":
        p.append("receipt runtime pins are not the preregistered ones")
    p += condition_problems(result.get("condition") or {}, arm, train_env)
    p += K.ledger_problems(summary, rows, num_envs, quota)
    quota_rows = [r for r in rows if r.get("in_quota")]
    counts = {o: sum(1 for r in quota_rows if r.get("outcome") == o) for o in ("capture", "crash", "timeout")}
    return p, {"quota_episodes": len(quota_rows), "finished_episodes": len(rows), "counts": counts,
               "duration_s": receipt.get("duration_s"), "result_sha256": receipt.get("result_sha256")}


def rows_aligned_problems(cell_dir, name):
    """Recorder rows align with the ledger: same count, same outcome for every (env, episode)."""
    try:
        ledger = K.read_jsonl(cell_dir / "episodes.jsonl")
        rows = K.read_jsonl(cell_dir / name)
    except (OSError, ValueError) as e:
        return ["%s or ledger rows unreadable: %s" % (name, e)]
    p = []
    if not rows or len(rows) != len(ledger):
        p.append("%s rows %d != ledger rows %d" % (name, len(rows), len(ledger)))
    by = {(r.get("env_index"), r.get("env_episode_index")): r for r in rows}
    mism = sum(1 for r in ledger if (by.get((r.get("env_index"), r.get("env_episode_index"))) or {}).get("outcome")
               != r.get("outcome"))
    if mism:
        p.append("%d ledger episodes have no %s row with the same outcome" % (mism, name))
    return p


def start_rows_problems(cell_dir, arm):
    """The rendered step-0 bearings: within +-HALF_DEG for a visible start, within +-A3_BEARING_TOL_DEG for A3."""
    try:
        rows = K.read_jsonl(cell_dir / INVIEW_FILE)
    except (OSError, ValueError) as e:
        return ["inview recorder rows unreadable: %s" % e]
    b = [r.get(D.RENDER_BEARING) for r in rows]
    if not b or any(x is None for x in b):
        return ["inview recorder rows without a rendered step-0 bearing"]
    lim = float(HALF_DEG) + D.BEARING_TOL_DEG if visible_start(arm) else A3_BEARING_TOL_DEG
    out = sum(1 for x in b if abs(x) > lim)
    return ["%d of %d rendered step-0 bearings outside +-%g deg" % (out, len(b), lim)] if out else []


def camref_line(text):
    lines = [json.loads(line[len("[camref] "):]) for line in text.splitlines() if line.startswith("[camref] {")]
    return lines[-1] if lines else None


def mismatch_allow(arm):
    allow = MISMATCH_COMMON
    if policy(arm) == "S2B":
        allow += MISMATCH_P9
    if rung(arm) in ("F0", "K1PK", "S6"):
        allow += MISMATCH_E2
    if visible_start(arm):
        allow += MISMATCH_V
    return allow


def mismatch_problems(text, arm):
    bad = [line for line in S2A.mismatch_lines(text) if not any(re.search(pat, line) for pat in mismatch_allow(arm))]
    return ["unexpected checkpoint-contract line: %s" % line[:200] for line in bad]


def expected_installed(arm):
    """What camref_entry itself installs for the arm (the delegate's installs are in realveh_receipt.json)."""
    r, pol = rung(arm), policy(arm)
    out = []
    if pol == "F" and "rand" in factors(arm):
        out.append("vehicle_rand")
    if camref_rules_on(arm):
        out.append("camref_rules")
    if camref_record(arm):
        out.append("camref_record")
    if pol == "F" and "yaw" in factors(arm):
        out.append("yawclip")
    if pol == "F" and visible_start(arm):
        out += ["inview_start", "inview_diag"]
    return out


def geometry_problems(rec, arm):
    """Stage-2 review r1 P2-1: the rule module's own record of the geometry it applied (camref_receipt camref_rules
    config_after): altitude 1.5 m / camera x 0.08 m with the geometry factor, the task's 1.0 m / 0.10 m without."""
    if not camref_rules_on(arm):
        return []
    after = ((rec.get("camref_rules") or {}).get("config_after")) or {}
    want = (1.5, 0.08) if "geom" in factors(arm) else (1.0, 0.10)
    got = (after.get("flight_altitude"), (after.get("camera_translation") or [None])[0])
    return [] if got == want else ["camref rules applied altitude / camera x %r, rung %s needs %r" % (got, rung(arm), want)]


def receipt_problems(cell_dir, arm, density, seed, ckpt_sha, quota, committed, train_env, root=ROOT):
    """The CAM-REF receipt (installs, module bytes, policy), the delegate's receipt (K1, S2B), both NAVRL snapshots, the
    telemetry of every installed module and the absence of every other, recorder rows and the MISMATCH lines."""
    try:
        rec = json.loads((cell_dir / "camref_receipt.json").read_text())
        receipt = json.loads((cell_dir / "receipt.json").read_text())
    except (OSError, ValueError) as e:
        return ["camref receipt or cell receipt missing or unreadable: %s" % e]
    p = []
    if rec.get("policy") != policy(arm) or rec.get("installed") != expected_installed(arm):
        p.append("camref receipt policy / installs %r / %r, expected %s / %r" % (rec.get("policy"), rec.get("installed"),
                                                                            policy(arm), expected_installed(arm)))
    shas = rec.get("sha256") or {}
    for key, rel in (("camref_entry", ENTRY), ("camref_rules", RULES_REL), ("realveh_entry", "tools/realveh_entry.py"),
                     ("yawclip", YAWCLIP_REL), ("inview", D.INVIEW_REL), ("camref_python", "tools/camref_python")):
        if shas.get(key) != committed.get(key):
            p.append("camref receipt %s SHA is not the committed file's" % key)
    for k in ("entry", "script", "aerial_gym"):
        if not K.is_inside(rec.get(k, ""), root):
            p.append("camref receipt %s outside the tree: %r" % (k, rec.get(k)))
    p += snapshot_problems(receipt.get("env_snapshot_navrl"), arm, density, seed, cell_dir, ckpt_sha, quota, train_env,
                           "launcher snapshot")
    p += snapshot_problems(rec.get("env_navrl"), arm, density, seed, cell_dir, ckpt_sha, quota, train_env,
                           "entry env_navrl")
    p += geometry_problems(rec, arm)
    if policy(arm) != "F":
        try:
            rv = json.loads((cell_dir / "realveh_receipt.json").read_text())
        except (OSError, ValueError) as e:
            return p + ["realveh receipt missing or unreadable: %s" % e]
        s = rv.get("settings") or {}
        want_veh = rung(arm) in ("K1S", "S0")
        if (s.get("vehicle_rand") is not None) != want_veh:
            p.append("realveh vehicle randomisation state does not match rung %s" % rung(arm))
        if ("cams2" in s) != cams2_on(arm) or ("camsafe" in s) != (policy(arm) == "S2B"):
            p.append("realveh cams2 / camsafe state does not match rung %s" % rung(arm))
        if ("yawclip" in s) != (rung(arm) == "K1S"):
            p.append("realveh yaw-clip state does not match rung %s" % rung(arm))
        if ("inview_start" in s) != visible_start(arm):
            p.append("realveh visible-start state does not match rung %s" % rung(arm))
        if cams2_on(arm):
            after = (rv.get("cams2") or {}).get("config_after") or {}
            alt = 1.5 if rung(arm) == "S0" else 1.0
            if after.get("flight_altitude") != alt or (after.get("camera_translation") or [None])[0] != \
                    (0.08 if rung(arm) == "S0" else 0.10) or after.get("success_radius") != 0.4:
                p.append("realveh cams2 config_after %r is not rung %s's" % (after, rung(arm)))
        if ((rv.get("ablation") or {}).get("condition")) != "no_lidar":
            p.append("realveh ablation is not no_lidar")
        if shas.get("realveh_entry") != (rv.get("sha256") or {}).get("realveh_entry"):
            p.append("realveh receipt names another realveh_entry")
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    have = {"[camref] {": camref_record(arm) or camref_rules_on(arm),
            "[veh-rand] ": "rand" in factors(arm) or rung(arm) == "S0", "[yawclip] ": "yaw" in factors(arm),
            "[inview] ": visible_start(arm), "[inviewdiag] {": "NAVRL_INVIEW_DIAG" in (rec.get("env_navrl") or {}),
            "[camsafe] ": policy(arm) == "S2B", "[cams2contact] {": cams2_on(arm), "[cams2diag] {": cams2_on(arm)}
    for prefix, want in have.items():
        if (prefix in text) != want:
            p.append("telemetry %s %s" % (prefix.strip(), "missing" if want else "present but the module is off"))
    if camref_record(arm):
        p += rows_aligned_problems(cell_dir, RECORD_FILE)
    if cams2_on(arm):
        p += S2A.diag_rows_problems(cell_dir)
    if "NAVRL_INVIEW_DIAG" in (rec.get("env_navrl") or {}):
        p += rows_aligned_problems(cell_dir, INVIEW_FILE)
        p += start_rows_problems(cell_dir, arm)
    if rung(arm) in ("F0", "K1PK"):
        try:
            rs = json.loads((cell_dir / K.RECORDER_SUMMARY).read_text())
            for mod in ("aerial_gym", "motar_eval"):
                if not K.is_inside((rs.get("import_origins") or {}).get(mod, ""), root):
                    p.append("KROC2027 recorder import origin of %s outside the tree" % mod)
        except (OSError, ValueError) as e:
            p.append("KROC2027 recorder summary unreadable: %s" % e)
    p += mismatch_problems(text, arm)
    return p


def check_cell(cell_dir, arm, head, start_host, ckpt_sha, train_env, num_envs, quota, root=ROOT):
    problems = []
    rec_host = G.host()
    for k in ("fault_count", "boot_id", "online"):
        if rec_host[k] != start_host[k]:
            problems.append("HOST " + k + " changed")
    p, stats = output_problems(cell_dir, arm, head, ckpt_sha, train_env, num_envs, quota)
    problems += p
    if K.git("status", "--porcelain", root=root).strip():
        problems.append("tree became dirty")
    return problems, rec_host, stats


def cell_verdict(problems):
    if any(p.startswith("HOST") for p in problems):
        return "VOID_HOST_FAULT"
    return "OK" if not problems else "FAIL"


# ---------------------------------------------------------------- smoke controls (pure on the cell files)
def smoke_cell_controls(cell_dir, arm):
    """The module controls of one smoke cell: {name: bool} (SMOKE_CONTROLS)."""
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    line = camref_line(text) or {}
    out = {}
    f = factors(arm)
    if camref_rules_on(arm) and rung(arm) not in ("F3", "F4", "K1T"):   # stage 1: F8 / K1S; stage 2: F4D .. F4A
        rules = line.get("rules") or {}
        draws = rules.get("delay_draws") or {}
        if "delay" in f:
            out["delay_draws_1_and_2"] = draws.get("1", 0) > 0 and draws.get("2", 0) > 0 and set(draws) <= {"1", "2"}
        else:
            out["no_delay_draws"] = not any(v for k, v in draws.items() if k != "0")
        out["capture_rule_acted"] = (rules.get("captures", 0) + rules.get("contacts", 0)) > 0
        veh_init = any(x.startswith("[veh-rand] {") and '"event": "init"' in x for x in text.splitlines())
        out["veh_rand_init" if "rand" in f else "no_veh_rand"] = veh_init if "rand" in f else not veh_init
        clip = S2A.parse_kv_line(text, S2A.YAWCLIP_PREFIX)
        if "yaw" in f:
            out["yawclip_commands"] = bool(clip) and clip.get("commands", 0) > 0
        else:
            out["no_yawclip"] = not clip
        if rung(arm) in S2_FACTOR_RUNGS:                     # stage 2 only (stage-1 F8 / K1S controls unchanged)
            try:
                rec = json.loads((cell_dir / "camref_receipt.json").read_text())
            except (OSError, ValueError):
                rec = {}
            out["geometry_applied" if "geom" in f else "no_geometry"] = not geometry_problems(rec, arm)
    if rung(arm) == "S6":
        out["no_cams2_telemetry"] = "[cams2diag] {" not in text and "[cams2contact] {" not in text
        out["camref_recorder_line"] = bool(line)
    out["start_bearings"] = not start_rows_problems(cell_dir, arm)
    return out


def smoke_verdict(records, controls):
    if any(r["verdict"] == "VOID_HOST_FAULT" for r in records):
        return "VOID_HOST_FAULT"
    if any(r["verdict"] == "FAIL" for r in records):
        return "SMOKE_FAIL"
    if any(r["verdict"] == "BUDGET_SKIPPED" for r in records):
        return "SMOKE_INCOMPLETE"
    ok = {r["arm"] for r in records if r["verdict"] == "OK" and ((r.get("stats") or {}).get("finished_episodes") or 0) >= 1}
    if ok != {a for a, _, _ in SMOKE_PLAN} or len(records) != len(SMOKE_PLAN):
        return "SMOKE_FAIL"
    if set(controls) != {a for a, _, _ in SMOKE_PLAN} or not all(c and all(c.values()) for c in controls.values()):
        return "SMOKE_FAIL"
    return "SMOKE_OK"


# ---------------------------------------------------------------- identity (pure on the ledgers)
def first_n(rows, n=FIRST_N):
    """{(env_index, env_episode_index): row} of the first n episodes per env."""
    return {(r["env_index"], r["env_episode_index"]): r for r in rows if int(r["env_episode_index"]) < n}


ROW_LABEL_FIELDS = ("cell_id", "episode_id", "arm", "in_quota", "in_legacy_window")


def identity_compare(new_rows, ref_rows, n=FIRST_N):
    """Per-episode outcomes of the first n episodes per env (the gate), plus the full rows without the label fields
    (reported)."""
    a, b = first_n(new_rows, n), first_n(ref_rows, n)
    keys_equal = set(a) == set(b)
    common = sorted(set(a) & set(b))
    differ = [k for k in common if a[k].get("outcome") != b[k].get("outcome")]
    strip = (lambda r: {k: v for k, v in r.items() if k not in ROW_LABEL_FIELDS})
    rows_equal = keys_equal and all(strip(a[k]) == strip(b[k]) for k in common)

    def rates(rows):
        out = {o: sum(1 for r in rows.values() if r.get("outcome") == o) for o in ("capture", "crash", "timeout")}
        out["n"] = len(rows)
        return out

    per_env = {}
    for env, _ in a:
        per_env[env] = per_env.get(env, 0) + 1
    complete = bool(a) and len(per_env) == NUM_ENVS and all(v == n for v in per_env.values())
    return {"episodes_new": len(a), "episodes_ref": len(b), "new_complete": complete, "keys_equal": keys_equal,
            "outcomes_differ": len(differ), "outcomes_equal": keys_equal and complete and not differ,
            "rows_equal_without_labels": rows_equal, "counts_new": rates(a), "counts_ref": rates(b),
            "first_differences": [list(k) for k in differ[:10]]}


def identity_cell_verdict(arm, record, comparison):
    """IDENTITY_PASS iff the cell is OK and every per-episode outcome of the first 8 per env equals the reference's;
    IDENTITY_FAIL on any difference (no loaded-reference branch: GR-0 S00 205 / 4501 equals its idle twins)."""
    if not record or record.get("verdict") != "OK" or not isinstance(comparison, dict):
        return "IDENTITY_FAIL"
    return "IDENTITY_PASS" if comparison.get("outcomes_equal") is True else "IDENTITY_FAIL"


def identity_verdict(cells, skipped=()):
    """Stage verdict from the per-cell verdicts {arm: verdict}: IDENTITY_PASS iff every planned cell passed; IDENTITY_FAIL
    if any cell failed; IDENTITY_INCOMPLETE if none failed but a cell was skipped for the wall cap."""
    want = {a for a, _, _ in IDENTITY_PLAN}
    if any(v != "IDENTITY_PASS" for v in cells.values()):
        return "IDENTITY_FAIL"
    if set(cells) == want:
        return "IDENTITY_PASS"
    return "IDENTITY_INCOMPLETE" if set(skipped) and set(cells) | set(skipped) >= want else "IDENTITY_FAIL"


def eval_plan():
    return [(arm, DENSITY, s) for s in SEEDS for arm in EVAL_ARMS]


def eval_verdict(records):
    if any(r["verdict"] in ("FAIL", "VOID_HOST_FAULT", "BUDGET_SKIPPED") for r in records):
        return "INCOMPLETE"
    done = {(r["arm"], r["density"], r["seed"]) for r in records if r["verdict"] == "OK"}
    return "COMPLETE" if set(eval_plan()) <= done else "INCOMPLETE"


def projected_cell_s(arm, quota, records, earlier_reports=()):
    """The projected wall time of a cell: the longest OK cell of the same policy and quota in this stage; else
    PROJECTION_MARGIN x the longest OK cell of the same policy and quota in an earlier stage (eval: the identity cells
    F0ID, K1ID, S0ID); None when nothing is known (the cell runs, its timeout bounded by the remaining cap)."""
    def walls(recs):
        return [r["wall_s"] for r in recs if r.get("policy") == policy(arm) and r.get("verdict") == "OK"
                and r.get("quota") == quota and isinstance(r.get("wall_s"), (int, float))]

    own = walls(records)
    if own:
        return max(own)
    prior = [w for rep in earlier_reports for w in walls((rep or {}).get("records") or [])]
    return PROJECTION_MARGIN * max(prior) if prior else None


def fits_budget(arm, quota, records, earlier_reports, remaining_s):
    """(fits, projection): a cell is started only if its projected wall time fits the remaining cap."""
    proj = projected_cell_s(arm, quota, records, earlier_reports)
    return (proj is None or proj <= remaining_s), proj


# ---------------------------------------------------------------- one cell
def run_cell(ctx, out_root, arm, density, seed, records, timeout_s, stage, quota):
    ckpt, ckpt_sha = ctx["ckpts"][policy(arm)]
    train_env = ctx["train_env"]
    out_dir = out_root / ARM_BASE.get(arm, arm)
    out_dir.mkdir(parents=True, exist_ok=True)
    cid = cell_id(arm, density, seed)
    cell_dir = out_dir / cid
    env = cell_env(out_dir, arm, density, seed, ckpt, ckpt_sha, train_env, ctx["protocol"], NUM_ENVS, quota)
    pre = env_problems(env, arm, seed, cell_dir, quota, ctx["protocol"]["evaluation"]["player_games_cap"], train_env,
                       protocol_sha())
    print("==", arm, cid, "quota %d" % quota, G.now(), "timeout %.0fs" % timeout_s, flush=True)
    t0 = time.time()
    loadavg = pathlib.Path("/proc/loadavg").read_text().split()[:3]
    rc = G.run_launcher(ROOT, ctx["protocol"], env, max(1.0, timeout_s), LAUNCHER) if not pre else None
    problems, h, stats = check_cell(cell_dir, arm, ctx["head"], ctx["start"], ckpt_sha, train_env, NUM_ENVS, quota)
    problems = pre + problems
    if rc not in (0, None):
        problems.insert(0, "launcher exit %d" % rc)
    if not problems:
        problems += receipt_problems(cell_dir, arm, density, seed, ckpt_sha, quota, ctx["committed"], train_env)
    log = cell_dir / "cell.log"
    text = log.read_text(errors="replace") if log.is_file() else ""
    rec = {"arm": arm, "base_arm": ARM_BASE.get(arm, arm), "policy": policy(arm), "rung": rung(arm), "stage": stage,
           "cell_id": cid, "cell_dir": str(cell_dir), "tree": str(ROOT), "commit": ctx["head"], "seed": seed,
           "density": density, "num_envs": NUM_ENVS, "quota": quota, "checkpoint": ckpt, "checkpoint_sha256": ckpt_sha,
           "exit": rc, "wall_s": time.time() - t0, "timeout_s": timeout_s, "loadavg_at_start": loadavg,
           "host_after": h, "stats": stats, "camref_last_line": camref_line(text),
           "mismatch_lines": S2A.mismatch_lines(text), "problems": problems, "verdict": cell_verdict(problems)}
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


def k1_checkpoint(report_path=K1_TRAIN_REPORT):
    """CAM-K1 seed 1 from its TRAINED train report (copy at the pinned SHA)."""
    try:
        tr = json.loads(pathlib.Path(report_path).read_text())
        ck = tr["checkpoint"]
    except (OSError, ValueError, KeyError) as e:
        return (None, None), ["CAM-K1 seed %d train report unreadable: %s" % (K1_SEED, e)]
    p = []
    if tr.get("verdict") != "TRAINED" or tr.get("seed") != K1_SEED or tr.get("development_smoke"):
        p.append("CAM-K1 seed %d is not a TRAINED non-smoke run" % K1_SEED)
    if ck.get("sha256") != K1_SHA or ck.get("copy_sha256") != K1_SHA or not ck.get("copy"):
        p.append("CAM-K1 seed %d checkpoint SHA is not the pinned %s" % (K1_SEED, K1_SHA[:12]))
    elif not pathlib.Path(ck["copy"]).is_file() or sha(ck["copy"]) != K1_SHA:
        p.append("CAM-K1 seed %d checkpoint copy missing or SHA mismatch" % K1_SEED)
    return (str(ck.get("copy")), K1_SHA), p


def checkpoints():
    """{policy: (path, sha)} and problems: F (read-only copy), K1 seed 1, S2b-FT (CAM-DX1's checks)."""
    p = []
    if not F_CKPT.is_file() or sha(F_CKPT) != F_SHA:
        p.append("Policy F checkpoint missing or SHA mismatch: %s" % F_CKPT)
    k1, kp = k1_checkpoint()
    p += kp
    s2a_receipt, ft_receipt, ft_report, tp = D.training_inputs()
    p += tp
    ft, fp = D.policy_problems(s2a_receipt, ft_receipt, ft_report)
    p += fp
    return {"F": (str(F_CKPT), F_SHA), "K1": k1, "S2B": (ft[0], ft[1])}, ft_receipt, p


def busy_share(seconds=2.0):
    """Share of the online CPUs busy over `seconds` (from /proc/stat)."""
    def snap():
        f = [float(x) for x in pathlib.Path("/proc/stat").read_text().splitlines()[0].split()[1:]]
        idle = f[3] + (f[4] if len(f) > 4 else 0.0)
        return sum(f), idle

    t0, i0 = snap()
    time.sleep(seconds)
    t1, i1 = snap()
    return 1.0 - (i1 - i0) / max(1e-9, t1 - t0)


def plan_env_problems(train_env, protocol, out_root):
    """Every planned cell environment (smoke, identity, eval) passes env_problems."""
    p = []
    cap = protocol["evaluation"]["player_games_cap"]
    cells = [(a, s, SMOKE_QUOTA) for a, _, s in SMOKE_PLAN] + [(a, s, QUOTA) for a, _, s in IDENTITY_PLAN]
    cells += [(a, s, QUOTA) for a, _, s in eval_plan()]
    for arm, seed, quota in cells:
        out_dir = out_root / ARM_BASE.get(arm, arm)
        try:
            env = cell_env(out_dir, arm, DENSITY, seed, "/x/last_gen_ppo_ep_1_rew_1.pth", POLICY_SHA[policy(arm)],
                           train_env, protocol, NUM_ENVS, quota)
        except ValueError as e:
            return ["cell environment refused: %s" % e]
        cell_dir = out_dir / cell_id(arm, DENSITY, seed)
        p += ["%s/%d: %s" % (arm, seed, x) for x in env_problems(env, arm, seed, cell_dir, quota, cap, train_env,
                                                                  protocol_sha())]
    return p


def stage_link_problems(stage, out_root, head):
    """Each stage needs the earlier ones in the same parent, on this commit (pure on files)."""
    parent, p, found = out_root.parent, [], {}

    def one(name, verdicts):
        hits = sorted(parent.glob("*/" + name))
        if len(hits) != 1:
            p.append("%s needs exactly one %s in %s, found %d" % (stage, name, parent, len(hits)))
            return None
        r = json.loads(hits[0].read_text())
        if r.get("verdict") not in verdicts or r.get("commit") != head:
            p.append("%s is not %s on this commit" % (hits[0], "/".join(verdicts)))
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


def stage_budget_h(stage, found):
    """min(the stage's sub-cap, MAX_HOURS_TOTAL - the wall time of the earlier stages' reports)."""
    used = sum(float(r.get("wall_s") or 0.0) for _, r in found.values()) / 3600.0
    return max(0.0, min(STAGE_SUB_CAP_H[stage], MAX_HOURS_TOTAL - used)), used


def committed_sha256(root=ROOT):
    return {"camref_entry": sha(root / ENTRY), "camref_rules": sha(root / RULES_REL),
            "realveh_entry": sha(root / "tools/realveh_entry.py"), "yawclip": sha(root / YAWCLIP_REL),
            "inview": sha(root / D.INVIEW_REL), "camref_python": sha(WRAPPER)}


def preflight(protocol, approved, out_root, stage, unit_dir=UNIT_DIR):
    """(head, problems, origins, gpu_stage_state). gpu_stage_state carries the approval and idle-host problems, which the
    check stage reports without counting (it runs before approval, possibly on a busy host)."""
    head, problems = G.tree_problems(ROOT, approved)
    if len(approved or "") < 7:
        problems.append("--approved-commit must name at least 7 hex digits")
    free_gb = shutil.disk_usage(str(pathlib.Path.home())).free / 1e9
    if free_gb < MIN_FREE_GB:
        problems.append("free disk %.2f GB < %.1f GB" % (free_gb, MIN_FREE_GB))
    branch = K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip()
    if branch not in BRANCHES:
        problems.append("branch %s is not one of %s" % (branch, BRANCHES))
    gpu_state = []
    ap = unit_dir / "execution_approval.json"
    if not ap.is_file():
        gpu_state.append("no approval file %s" % ap)
    else:
        try:
            approval = json.loads(ap.read_text())
            gpu_state += R1.approval_problems(approval, head, scope=SCOPE)
            if approval.get("scope_sha256") != scope_digest():
                gpu_state.append("approval scope_sha256 is not the SHA-256 of SCOPE (`camref_eval_run.py scope`)")
        except ValueError as e:
            gpu_state.append("approval unreadable: %s" % e)
    busy = busy_share()
    if busy > IDLE_MAX_BUSY:
        gpu_state.append("host not idle: %.0f %% of the online CPUs busy (limit %.0f %%)" % (100 * busy,
                                                                                            100 * IDLE_MAX_BUSY))
    if stage != "check":
        problems += gpu_state
    for rel in COMMITTED_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not (ROOT / LAUNCHER).is_file() or sha(ROOT / LAUNCHER) != LAUNCHER_SHA:
        problems.append("launcher missing or not the pinned SHA %s" % LAUNCHER_SHA[:12])
    if not os.access(str(WRAPPER), os.X_OK):
        problems.append("tools/camref_python is not executable")
    if not pathlib.Path(REAL_PYTHON).is_file():
        problems.append("interpreter %s missing" % REAL_PYTHON)
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
    return head, problems, origins, gpu_state


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
    head, problems, origins, gpu_state = preflight(protocol, approved, out_root, args.stage, UNIT_DIR)
    ckpts, ft_receipt, cp = checkpoints()
    problems += cp
    changed = changed_since(FT_TRAIN_HEAD)
    problems += allowlist_problems(changed, CODE_ALLOWLIST, FT_TRAIN_HEAD)
    problems += reference_problems()
    train_env = ft_receipt.get("env_navrl") or {}
    cap = protocol["evaluation"]["player_games_cap"]
    if not D.training_env_problems(train_env):
        problems += base_provenance_problems(train_env, cap)
        problems += plan_env_problems(train_env, protocol, out_root)
    link, found = stage_link_problems(args.stage if args.stage != "check" else "smoke", out_root, head)
    problems += link
    budget_h, used_h = stage_budget_h(args.stage if args.stage != "check" else "smoke", found)
    if args.stage != "check" and budget_h <= 0:
        problems.append("no wall time left of the %.1f h %s cap (%.2f h used)" % (MAX_HOURS_TOTAL, UNIT, used_h))
    if args.stage == "check":                      # CPU only, read only: nothing is written
        print(json.dumps({"stage": "check", "unit": UNIT, "commit": head, "preflight_problems": problems,
                          "gpu_stage_state": gpu_state, "checkpoints": ckpts, "changed_since_ft_training": changed,
                          "branch": K.git("rev-parse", "--abbrev-ref", "HEAD", root=ROOT).strip(),
                          "import_origins": origins}, indent=1, default=str))
        return 0 if not problems else 2
    start = G.host()
    report = {"unit": UNIT, "stage": args.stage, "tree": str(ROOT), "commit": head, "prereg": PREREG, "scope": SCOPE,
              "scope_sha256": scope_digest(), "started": G.now(), "host_start": start, "import_origins": origins,
              "launcher": LAUNCHER, "launcher_sha256": LAUNCHER_SHA, "checkpoints": ckpts, "arms": ARMS,
              "density": DENSITY, "seeds": list(SEEDS), "budget_h": budget_h, "earlier_stages_h": used_h,
              "preflight_problems": problems, "linked_reports": {k: v[0] for k, v in found.items()},
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if R1.ENV_ALLOW.match(k)}}
    if args.stage == "eval" and "identity_report.json" in found:
        if found["identity_report.json"][1].get("env_allowlisted") != report["env_allowlisted"]:
            problems.append("runner environment differs from the identity stage's")
    if problems:
        report["verdict"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    committed = committed_sha256()
    report["committed_sha256"] = committed
    ctx = {"protocol": protocol, "head": head, "start": start, "committed": committed, "train_env": train_env,
           "ckpts": ckpts}
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(budget_h)
    t_stage = time.time()
    records, void = [], False

    def timeout(arm):
        return min(float(CELL_TIMEOUT_S[policy(arm)]), max(0.0, budget.remaining_s()))

    def stopped(rec):
        return rec["verdict"] in ("FAIL", "VOID_HOST_FAULT")

    earlier = [r for _, r in found.values()]

    def skip(arm, d, s, stage, quota):
        """Skip (never start) a cell whose projected time does not fit the remaining cap."""
        ok, proj = fits_budget(arm, quota, records, earlier, budget.remaining_s())
        if ok and not budget.exhausted():
            return False
        records.append({"arm": arm, "base_arm": ARM_BASE.get(arm, arm), "policy": policy(arm), "stage": stage,
                        "density": d, "seed": s, "quota": quota, "verdict": "BUDGET_SKIPPED", "projected_s": proj,
                        "remaining_s": budget.remaining_s()})
        report["budget_skipped"] = report.get("budget_skipped", 0) + 1
        return True

    if args.stage == "smoke":
        controls = {}
        for arm, d, s in SMOKE_PLAN:
            if skip(arm, d, s, "SMOKE", SMOKE_QUOTA):
                continue
            rec = run_cell(ctx, out_root, arm, d, s, records, timeout(arm), "SMOKE", SMOKE_QUOTA)
            if rec["verdict"] == "OK":
                controls[arm] = smoke_cell_controls(pathlib.Path(rec["cell_dir"]), arm)
            if stopped(rec):
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        report.update(quota=SMOKE_QUOTA, smoke_controls=controls, smoke_control_definitions=SMOKE_CONTROLS)
        report["verdict"] = "VOID_HOST_FAULT" if void else smoke_verdict(records, controls)
        name = "smoke_report.json"
    elif args.stage == "identity":
        cells, comps = {}, {}
        for arm, d, s in IDENTITY_PLAN:
            if skip(arm, d, s, "IDENTITY", QUOTA):
                continue
            rec = run_cell(ctx, out_root, arm, d, s, records, timeout(arm), "IDENTITY", QUOTA)
            comp = None
            if rec["verdict"] == "OK":
                ref = reference_dir(*IDENTITY_REF[arm])
                comp = dict(identity_compare(K.read_jsonl(pathlib.Path(rec["cell_dir"]) / "episodes.jsonl"),
                                             K.read_jsonl(ref / "episodes.jsonl")),
                            reference=str(ref))
            comps[arm] = comp
            cells[arm] = identity_cell_verdict(arm, rec, comp)
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
            if cells[arm] == "IDENTITY_FAIL":                 # stop on IDENTITY_FAIL (preregistration)
                break
        report.update(identity_comparisons=comps, identity_cells=cells)
        skipped = [r["arm"] for r in records if r["verdict"] == "BUDGET_SKIPPED"]
        report["verdict"] = "VOID_HOST_FAULT" if void else identity_verdict(cells, skipped)
        name = "identity_report.json"
    else:
        ident = (found.get("identity_report.json") or (None, {}))[1]
        for arm, d, s in eval_plan():
            if skip(arm, d, s, "EVAL", QUOTA):
                continue
            rec = run_cell(ctx, out_root, arm, d, s, records, timeout(arm), "EVAL", QUOTA)
            if stopped(rec):
                void = rec["verdict"] == "VOID_HOST_FAULT"
                break
        report.update(cells_planned=len(eval_plan()), identity_cells=ident.get("identity_cells"))
        report["verdict"] = "VOID_HOST_FAULT" if void else eval_verdict(records)
        name = "eval_run_report.json"
    report["records"] = records
    report["host_end"] = G.host()
    if any(report["host_end"][k] != start[k] for k in ("fault_count", "boot_id", "online")):
        report["verdict"] = "VOID_HOST_FAULT"
    report["wall_s"] = time.time() - t_stage
    report["finished"] = G.now()
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("stage", "verdict", "commit")}, indent=1))
    return 0 if report["verdict"] in ("SMOKE_OK", "IDENTITY_PASS", "COMPLETE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
