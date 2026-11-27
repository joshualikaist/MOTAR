"""Runner of CAM-S3-E, stage 3 of the CAM-REF runner (docs/plans/prereg_camonly_s3w_eval_2026-11-27.md): CAM-S3 S3-W
ep18000 against CAM-S2b-FT ep15000 at P_S (the CAM-DX1 V312 construction, stage 1's S0), 205 bars, each policy in its own
trained runtime.

  S0   reference, not rerun: CAM-DX1 V312 205 / 4501-4503 (SHA-256 pinned in camref_eval_run.REF_SHA); S0ID reruns its
       seed-4501 cell on this commit and must reproduce it episode for episode (else IDENTITY_FAIL, stop).
  S0T  S2b-FT (1.0472 rad/s, iid 2 % holes, sampled columns) on bars_h3_thin.
  S0M  S2b-FT with navrl_cams3's measured masks + 10-row top band (false near 0, holes 0), no sector rule, bars_h3.
  W    S3-W in its trained runtime (1.5708 rad/s, measured masks, sector k = 24) on bars_h3.
  WT   W on bars_h3_thin (S3-W's training arena).

Stage 1's runner (tools/camref_eval_run.py) builds, launches and checks every cell; its stage-3 rungs (S3_RUNGS,
s3_rung_delta, CAMS3_POINT, cams3_receipt_problems, s3_smoke_controls) are defined there. This module replaces its unit
constants (scope, preregistration, approval folder, plans, wall cap, launcher copy, code allowlist) and adds S3-W to
checkpoints() with its training checks, then calls its main().
Stages (same chain and guards as stage 1): smoke (the four arms at quota 1, 205 / 4501, never counted), identity (S0ID),
eval (the four arms x seeds 4501-4503, block-major by seed: 12 cells). Wall cap 2.0 h shared by the three stages
(sub-caps 0.4 / 0.3 / 2.0 h).
usage: tools/camref_stage3_run.py scope | {check,smoke,identity,eval} [--approved-commit SHA] --out DIR
"""
import contextlib
import hashlib
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import camref_eval_run as R                                 # noqa: E402

PREREG = "docs/plans/prereg_camonly_s3w_eval_2026-11-27.md"
TRAIN_PREREG = "docs/plans/prereg_camonly_s3w_2026-11-27.md"
UNIT = "CAM-S3-E"
SCOPE = ("CAM-S3-E v1 (CAM-REF stage 3): GPU evaluation at P_S (the CAM-DX1 V312 construction) at 205 bars, quota 8 per "
         "env at 128 envs (1,024 episodes per cell), of CAM-S2b-FT (f2c80fcc...) and CAM-S3 S3-W ep18000 (2626a4b2...), "
         "each in its own trained runtime: S0T S2b-FT on bars_h3_thin; S0M S2b-FT with the measured OAK-D masks and top "
         "band (false near 0, no sector rule); W S3-W (1.5708 rad/s, measured masks, sector k 24) on bars_h3; WT W on "
         "bars_h3_thin; reference S0 = CAM-DX1 V312 205 / 4501-4503 (SHA-256 pinned); smoke S0T, S0M, W, WT at quota 1 "
         "(205 / 4501); identity S0ID (S2b-FT at P_S, against CAM-DX1 V312 205 / 4501, first 8 episodes per env), any "
         "difference IDENTITY_FAIL; eval S0T, S0M, W, WT x seeds 4501-4503 (12 cells, block-major by seed); wall cap "
         "2.0 h shared by the three stages, cells that cannot finish inside it skipped; idle host; "
         "tools/camref_stage3_run.py over tools/camref_eval_run.py with eval_navrl_cams3e_arms.sh and "
         "tools/camref_entry.py")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "cams3e_20261127"
NEW_ARMS = R.S3_RUNGS                                       # S0T, S0M, W, WT (arm name = rung name)
SMOKE_PLAN = tuple((a + "_smoke", R.DENSITY, 4501) for a in NEW_ARMS)
IDENTITY_PLAN = (("S0ID", R.DENSITY, 4501),)
IDENTITY_REF = {"S0ID": ("DX1_V312", 4501)}
EVAL_ARMS = NEW_ARMS
MAX_HOURS_TOTAL = 2.0
STAGE_SUB_CAP_H = {"smoke": 0.4, "identity": 0.3, "eval": 2.0}
SMOKE_CONTROLS = ("every smoke cell OK with >= 1 finished episode (cams2 telemetry, realveh receipt, MISMATCH lines and "
                  "snapshots checked as for every cell)",
                  "every smoke cell: every rendered step-0 bearing within +-28.35 deg (visible start)",
                  "S0M_smoke, W_smoke, WT_smoke: one [cams3] install_measured (false near off) and one meas_first over "
                  "the 1,849-frame library with an invalid share in (0, 1); S0T_smoke: no measured-depth event",
                  "W_smoke, WT_smoke: one [cams3] install_sector and one sector_first with k 24 over 13 bins; S0T_smoke, "
                  "S0M_smoke: no sector event",
                  "W_smoke, WT_smoke: snapshot NAVRL_YAW_RATE_MAX 1.5708; S0T_smoke, S0M_smoke: 1.0472; none with a "
                  "NAVRL_YAW_RATE_MAX CONFIG MISMATCH line (the task compares the running scale with the checkpoint's)",
                  "S0T_smoke: the task's ARENA line checkpoint=bars_h3 running=bars_h3_thin; W_smoke: checkpoint="
                  "bars_h3_thin running=bars_h3; S0M_smoke, WT_smoke: no ARENA line; snapshot pool = the rung's")
LAUNCHER = "aerial_gym/rl_training/rl_games/eval_navrl_cams3e_arms.sh"
LAUNCHER_SHA = "561ea75ca9db5d0d50d4387622aa086b69084d40c5853852b1b9fbf6e3d87750"   # the committed copy
NEW_FILES = ("tools/camref_stage3_run.py", "tools/camref_stage3_analyse.py", "tests/test_cams3_eval.py", LAUNCHER)
# Paths changed since S2b-FT's training commit (335a1fa) outside stage 1's allowlist: the CAM-S3 simulation files (the
# module, the thin-bar assets and the object config's height ratio for the new pool; S0ID shows they leave S0 unchanged),
# the stage-3 launcher copy, CAM-S3's training-only files and two flight-runtime files and the test runner, none of which
# the evaluation path imports.
CODE_ALLOWLIST = R.CODE_ALLOWLIST + (R.CAMS3_REL, "aerial_gym/config/asset_config/env_object_config.py",
                                     "resources/models/environment_assets/bars_h3_thin/*", LAUNCHER,
                                     "aerial_gym/rl_training/rl_games/train_navrl_cams3.sh", "tools/cams3_python",
                                     "tools/cams3_train.py", "tools/cams3_build_mask_library.py",
                                     "tools/camera_policy_runtime.py", "tools/camera_ros2_node.py", "tools/run_tests.py")

# ---- S3-W (read only, SHA-checked) ----------------------------------------------------------------------------------
S2B_TREE = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b")
S3W_RUN_REL = "aerial_gym/rl_training/rl_games/runs/ppo_261127_1746_navrl_v2-cams3w-yaw90-camonly-s1"
S3W_FILE = "last_gen_ppo_ep_18000_rew_93.111465.pth"
S3W_CKPT = S2B_TREE / S3W_RUN_REL / "nn" / S3W_FILE
S3W_SHA = R.S3W_SHA
S3W_UNIT = pathlib.Path.home() / "motar_tmp" / "cams3_20261127" / "train_w"
S3W_REPORT, S3W_REPORT_SHA = S3W_UNIT / "run_report.json", \
    "1c8ae51bf091a33655832ffb4c223de7f9a91da40f718daf771f3c82c1c89ffe"   # = docs/evidence/.../cams3/train_w_run_report.json
S3W_RECEIPT, S3W_RECEIPT_SHA = S3W_UNIT / "receipt.json", \
    "3ae1448904391b1ca5b037ac3990b6fc02836a0fcf1b9db3627280bfb0574c05"
S3W_TRAIN_HEAD = "cf1184555ebc8fd85efb9208afda3692c0811cfb"
# The keys in which S3-W's training environment differs from S2b-FT's (the CAM-S3 changes, plus the receipt path and the
# P9 file path of another worktree; the P9 SHA-256 key is equal): nothing else may differ.
S3W_TRAIN_DIFF = frozenset({"NAVRL_BAR_POOL", "NAVRL_CAMS3_SECTOR_K", "NAVRL_DEPTH_HOLE_P", "NAVRL_DEPTH_MEAS",
                            "NAVRL_DEPTH_MEAS_ADVANCE_P", "NAVRL_DEPTH_MEAS_FALSE_NEAR", "NAVRL_DEPTH_MEAS_JUMP_P",
                            "NAVRL_DEPTH_MEAS_LIBRARY", "NAVRL_DEPTH_MEAS_SEED", "NAVRL_DEPTH_MEAS_SHA256",
                            "NAVRL_DEPTH_MEAS_TOP_ROWS", "NAVRL_P9_ERROR_MODEL", "NAVRL_REALVEH_RECEIPT",
                            "NAVRL_YAW_RATE_MAX"})
PER_CELL_S3W = ("NAVRL_DEPTH_MEAS_SEED",)                  # a per-cell module seed in evaluation


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def s3w_problems(ft_env, report_path=S3W_REPORT, receipt_path=S3W_RECEIPT, ckpt=S3W_CKPT, root=R.ROOT):
    """S3-W TRAINED with its final checkpoint at the pin; its training receipt at the pin, differing from S2b-FT's
    training environment in exactly S3W_TRAIN_DIFF; W's stage-3 keys equal to the trained values (the pool per arm, the
    mask seed per cell); the mask library and the navrl_cams3 bytes those of training. Pure on the files."""
    p = []
    try:
        if sha(report_path) != S3W_REPORT_SHA:
            p.append("S3-W run report %s is not its pin" % report_path)
        rep = json.loads(pathlib.Path(report_path).read_text())
        rec_text = pathlib.Path(receipt_path).read_bytes()
        rec = json.loads(rec_text)
    except (OSError, ValueError) as e:
        return ["S3-W run report or receipt unreadable: %s" % e]
    if hashlib.sha256(rec_text).hexdigest() != S3W_RECEIPT_SHA:
        p.append("S3-W training receipt %s is not its pin" % receipt_path)
    fin = rep.get("final_checkpoint") or {}
    if (rep.get("verdict") != "TRAINED" or rep.get("mode") != "run" or rep.get("last_epoch") != 18000
            or fin.get("sha256") != S3W_SHA or fin.get("file") != S3W_FILE
            or rep.get("faults_before") != rep.get("faults_after")
            or (rep.get("preflight") or {}).get("head") != S3W_TRAIN_HEAD):
        p.append("S3-W run report is not TRAINED at epoch 18000 with the pinned final checkpoint on %s"
                 % S3W_TRAIN_HEAD[:7])
    if not pathlib.Path(ckpt).is_file() or sha(ckpt) != S3W_SHA:
        p.append("S3-W checkpoint %s missing or not its pin %s..." % (ckpt, S3W_SHA[:12]))
    env = rec.get("env_navrl") or {}
    diff = {k for k in set(env) | set(ft_env) if env.get(k) != ft_env.get(k)}
    if diff != set(S3W_TRAIN_DIFF):
        p.append("S3-W training environment differs from S2b-FT's in %s, expected %s"
                 % (sorted(diff), sorted(S3W_TRAIN_DIFF)))
    for k, v in R.s3_rung_delta("WT", 4501).items():
        if k not in PER_CELL_S3W and env.get(k) != v:
            p.append("stage-3 key %s=%r is not S3-W's trained %r" % (k, v, env.get(k)))
    w, wt = R.s3_rung_delta("W", 4501), R.s3_rung_delta("WT", 4501)
    if {k: v for k, v in wt.items() if k != "NAVRL_BAR_POOL"} != {k: v for k, v in w.items() if k != "NAVRL_BAR_POOL"} \
            or w.get("NAVRL_BAR_POOL") is not None or env.get("NAVRL_BAR_POOL") != "bars_h3_thin":
        p.append("W is not WT on bars_h3 (S3-W trained on bars_h3_thin)")
    lib = pathlib.Path(R.CAMS3_LIBRARY)
    if not lib.is_file() or sha(lib) != R.CAMS3_LIBRARY_SHA:
        p.append("mask library %s missing or not its pin" % lib)
    if rec.get("cams3_sha256") != sha(pathlib.Path(root) / R.CAMS3_REL):
        p.append("navrl_cams3.py differs from the bytes S3-W trained with")
    return p


_STAGE1_CHECKPOINTS = R.checkpoints


def checkpoints():
    """Stage 1's checkpoints (F, K1, S2b-FT) plus S3-W with s3w_problems."""
    ckpts, ft_receipt, p = _STAGE1_CHECKPOINTS()
    ckpts = dict(ckpts, S3W=(str(S3W_CKPT), S3W_SHA))
    p = list(p) + s3w_problems((ft_receipt or {}).get("env_navrl") or {})
    return ckpts, ft_receipt, p


PATCHED = ("PREREG", "UNIT", "SCOPE", "UNIT_DIR", "SMOKE_PLAN", "IDENTITY_PLAN", "IDENTITY_REF", "EVAL_ARMS",
           "MAX_HOURS_TOTAL", "STAGE_SUB_CAP_H", "SMOKE_CONTROLS", "COMMITTED_FILES", "NEW_FILES", "LAUNCHER",
           "LAUNCHER_SHA", "CODE_ALLOWLIST", "checkpoints")


def install():
    """Replace stage 1's unit constants in camref_eval_run (its functions read them at call time)."""
    R.PREREG, R.UNIT, R.SCOPE, R.UNIT_DIR = PREREG, UNIT, SCOPE, UNIT_DIR
    R.SMOKE_PLAN, R.IDENTITY_PLAN, R.IDENTITY_REF, R.EVAL_ARMS = SMOKE_PLAN, IDENTITY_PLAN, IDENTITY_REF, EVAL_ARMS
    R.MAX_HOURS_TOTAL, R.STAGE_SUB_CAP_H, R.SMOKE_CONTROLS = MAX_HOURS_TOTAL, STAGE_SUB_CAP_H, SMOKE_CONTROLS
    R.COMMITTED_FILES = tuple(dict.fromkeys((PREREG, TRAIN_PREREG, R.CAMS3_REL) + R.COMMITTED_FILES + NEW_FILES))
    R.NEW_FILES = tuple(dict.fromkeys(R.NEW_FILES + NEW_FILES))
    R.LAUNCHER, R.LAUNCHER_SHA, R.CODE_ALLOWLIST = LAUNCHER, LAUNCHER_SHA, CODE_ALLOWLIST
    R.checkpoints = checkpoints


@contextlib.contextmanager
def installed():
    """install() for the duration of a block, then stage 1's constants back (tests share one process)."""
    saved = {k: getattr(R, k) for k in PATCHED}
    try:
        install()
        yield R
    finally:
        for k, v in saved.items():
            setattr(R, k, v)


def main(argv=None):
    install()
    return R.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
