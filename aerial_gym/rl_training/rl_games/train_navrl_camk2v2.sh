#!/usr/bin/env bash
# CAM-K2 v2: the camera-only lineage (CAM-K1 v1) continued with the real vehicle's thrust-to-weight modelled on
# navrl_quad (the user, 2026-11-08: 1.1 kg, 5x4.3x3 props, Velox 2550 kV motors; T/W 4.0 nominal, INFERRED from the
# LIGPOWER V2306 V2 2400KV / T5143S / 4S bench point, review r1 m-2): the motor model's per-motor limit is raised at run
# time (NAVRL_VEH_RAND_LIMIT_MODE=raise, opt-in) to 4.0 x m x g / 4, with vehicle (mass / motor)
# randomization and the OAK-D Pro depth model; a new policy lineage (docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md).
# A copy of train_navrl_camk2.sh (CAM-K2 v1, WITHDRAWN_BEFORE_RUN, unchanged) with the v2 values. Neither Policy F nor
# CAM-K1 is modified: the run starts from a copy of CAM-K1 seed SEED's ep28000 checkpoint (tools/camk2v2_train.py makes
# it and passes CAMK2V2_SOURCE_CKPT) and branches into a new runs/ folder.
#
#   usage: train_navrl_camk2v2.sh SEED          (SEED 1 or 2)
#
# Everything is CAM-K1's contract (train_navrl_camk1.sh: GR-1 ctrl's fixed 205 bars, curriculum off, LR 5e-6, the
# riskcap governor parameters from the same validated screen summary, main profile, 128 envs, no_lidar, FOV 66 x 54,
# yaw_align_weight 0.6, visibility_bonus 0.03), except:
#   - the source: CAM-K1 v1 seed SEED ep28000 (SHA-256 pinned per seed), 1,000 epochs (28001..29000);
#   - PYTHON=tools/realveh_python (tools/realveh_entry.py) instead of tools/camtrain_python. It installs, before the
#     runner builds the task: vehicle randomization with the per-motor limit raised to TW 4.0 x m x g / 4 (2.4525 N at
#     0.25 kg, above navrl_quad's own 2 N; the thrust-to-weight of a 1.1 kg vehicle at T/W 4.0, rotational dynamics
#     and drag navrl_quad's) and TW_SCALE 0.15, MOTOR_IMBAL 0.05, tau 0.02..0.04 s around it (worst-case T/W
#     3.23 >= 1.8); the
#     OAK-D Pro depth model (MinZ 0.4 m, MaxZ 20 m, sigma_d 0.15 px, f*B 66.1875 px m = 882.5 px x 0.075 m for 800P
#     extended disparity + subpixel, holes 0.02; invalid depth = no return), as CAM-K2 v1; the camera-only ablation and
#     CAM-K1's overrides (camtrain_entry, unchanged). The robot stays navrl_quad: URDF, config, Lee gains and
#     checkpoint lineage are CAM-K1's;
#   - one receipt, NAVRL_REALVEH_RECEIPT (no camtrain receipt).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
HERE="$(pwd)"
ROOT="$(cd ../../.. && pwd)"
if (( $# != 1 )) || [[ "$1" != "1" && "$1" != "2" ]]; then
    echo "[camk2v2-train] usage: train_navrl_camk2v2.sh {1|2}" >&2
    exit 2
fi
TSEED="$1"
REAL_PYTHON="/home/fair/miniconda3/envs/aerialgym/bin/python"
export PATH="$(dirname "${REAL_PYTHON}"):${PATH}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${ROOT}"
ORIGIN="$("${REAL_PYTHON}" -c 'import importlib.util; print(importlib.util.find_spec("aerial_gym").origin)')"
case "${ORIGIN}" in
    "${ROOT}"/*) ;;
    *) echo "[camk2v2-train] aerial_gym resolves outside this tree: ${ORIGIN}" >&2; exit 2 ;;
esac
# CAM-K1 v1 seed TSEED, ep28000 (tools/camk2_train.py SOURCES), and this seed's own randomization seeds.
case "${TSEED}" in
    1) SOURCE_SHA="d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb"; VEH_SEED=4911; DEPTH_SEED=4921 ;;
    2) SOURCE_SHA="5128bf2e0461fb45c6b8c4463d4d0a950bb88c0d28923c41d02d99da1102a4b7"; VEH_SEED=4912; DEPTH_SEED=4922 ;;
esac
GOV_SOURCE_SHA="82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"   # ep24000, the screen's source
SUMMARY="../../../results/navrl_v2_ep24000_riskcap_seed44_screen/summary.json"
export CKPT="${CAMK2V2_SOURCE_CKPT:?CAMK2V2_SOURCE_CKPT is required (tools/camk2v2_train.py sets it)}"
[[ -f "${CKPT}" && -f "${SUMMARY}" ]] || { echo "[camk2v2-train] missing source checkpoint or screen summary" >&2; exit 2; }
[[ "$(sha256sum "${CKPT}" | cut -d' ' -f1)" == "${SOURCE_SHA}" ]] || { echo "[camk2v2-train] source checkpoint SHA mismatch" >&2; exit 2; }
# Governor parameters of Policy F, from the same validated summary its own training used (as train_navrl_gr1.sh).
read -r SELECTED_MODE FIXED_MPS FREE_MPS HALF_WIDTH_M MARGIN_M SLOW_M RELEASE_M TTC_S BRAKE_MPS2 REACTION_S <<< "$(${REAL_PYTHON} - "${SUMMARY}" "${GOV_SOURCE_SHA}" <<'PY'
import json, sys
from pathlib import Path
payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
problems = []
if payload.get("source_checkpoint_sha256") != sys.argv[2]:
    problems.append("summary source SHA mismatch")
if payload.get("adaptive_go") is not True or payload.get("selected_tag") != "riskcap":
    problems.append("summary does not select the preregistered riskcap")
row = {r.get("tag"): r for r in payload.get("rows", [])}.get("riskcap") or {}
params = row.get("parameters") or {}
keys = ("speed_governor_fixed_mps", "speed_governor_free_mps", "speed_governor_half_width_m", "speed_governor_margin_m",
        "speed_governor_slow_m", "speed_governor_release_m", "speed_governor_ttc_s", "speed_governor_brake_mps2",
        "speed_governor_reaction_s")
if row.get("screen_pass") is not True or any(k not in params for k in keys):
    problems.append("riskcap row incomplete")
if problems:
    print("[camk2v2-train] refusing:", "; ".join(problems), file=sys.stderr)
    raise SystemExit(2)
print("riskcap", *(params[k] for k in keys))
PY
)"
export NAVRL_SPEED_GOVERNOR="${SELECTED_MODE}"
export NAVRL_SPEED_GOVERNOR_FIXED_MPS="${FIXED_MPS}"
export NAVRL_SPEED_GOVERNOR_FREE_MPS="${FREE_MPS}"
export NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M="${HALF_WIDTH_M}"
export NAVRL_SPEED_GOVERNOR_MARGIN_M="${MARGIN_M}"
export NAVRL_SPEED_GOVERNOR_SLOW_M="${SLOW_M}"
export NAVRL_SPEED_GOVERNOR_RELEASE_M="${RELEASE_M}"
export NAVRL_SPEED_GOVERNOR_TTC_S="${TTC_S}"
export NAVRL_SPEED_GOVERNOR_BRAKE_MPS2="${BRAKE_MPS2}"
export NAVRL_SPEED_GOVERNOR_REACTION_S="${REACTION_S}"
export NAVRL_SPEED_GOVERNOR_DIAG=1

# CAM-K1's fixed-density continuation contract, now 28001..29000 with the given training seed.
export NAVRL_V2_ALLOW_RESUME=1
export NAVRL_V2_PROFILE=main
export NAVRL_OBSTACLE_SELECTOR=cluster_sector
export SEED="${TSEED}"
export NAVRL_DENSITY_CURRICULUM=0
export NAVRL_NUM_BARS=205
export NAVRL_DENSITY_START=205
export NAVRL_DENSITY_FINAL=205
export NAVRL_DENSITY_WARMUP=0
export NAVRL_DENSITY_MIN_EPOCHS=0
export NAVRL_RESET_DENSITY_WINDOW=1
export NAVRL_LEARNING_RATE=5e-6
export NAVRL_RESET_ACTOR_OPTIMIZER=0
export MAX_EPOCHS=29000
TAG_SUFFIX=""
if [[ "${CAMK2V2_SMOKE:-0}" == "1" ]]; then    # the unit's development smoke: two epochs, its own run tag
    export MAX_EPOCHS=28002
    TAG_SUFFIX="-smoke"
fi
unset NAVRL_GAIN_RAND NAVRL_GAIN_RAND_SCALE NAVRL_GAIN_RAND_SEED

# CAM-K1: camera only, OAK-D Pro RGB field of view, FOV-keeping reward weights (tools/camtrain_entry.py).
export NAVRL_SENSOR_ABLATION=no_lidar
export NAVRL_CAMTRAIN_HFOV_DEG=66
export NAVRL_CAMTRAIN_VFOV_DEG=54
export NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT=0.6
export NAVRL_CAMTRAIN_VISIBILITY_BONUS=0.03
unset NAVRL_CAMTRAIN_RECEIPT CAMTRAIN_REAL_PYTHON

# CAM-K2 v2: the raised per-motor limit (T/W 4.0, opt-in raise), vehicle randomization and the OAK-D Pro depth model
# (tools/realveh_entry.py). The robot is the task default navrl_quad, the lineage of F and CAM-K1.
unset NAVRL_ROBOT NAVRL_REALVEH_ROBOT NAVRL_REALVEH_WARMSTART_FROM
export NAVRL_REALVEH_RECEIPT="${NAVRL_REALVEH_RECEIPT:?NAVRL_REALVEH_RECEIPT is required (tools/camk2v2_train.py sets it)}"
export NAVRL_VEH_RAND=1
export NAVRL_VEH_RAND_SEED="${VEH_SEED}"
export NAVRL_VEH_RAND_TW_NOMINAL=4.0
export NAVRL_VEH_RAND_LIMIT_MODE=raise
export NAVRL_VEH_RAND_TW_SCALE=0.15
export NAVRL_VEH_RAND_MOTOR_IMBAL=0.05
export NAVRL_VEH_RAND_TAU_MIN=0.02
export NAVRL_VEH_RAND_TAU_MAX=0.04
export NAVRL_DEPTH_MODEL=oakd
export NAVRL_DEPTH_SEED="${DEPTH_SEED}"
export NAVRL_DEPTH_MINZ_M=0.4
export NAVRL_DEPTH_MAXZ_M=20
export NAVRL_DEPTH_SIGMA_D_PX=0.15
# f*B: OAK-D Pro 800P, extended disparity + subpixel, 882.5 px x 0.075 m.
export NAVRL_DEPTH_FB_PXM=66.1875
export NAVRL_DEPTH_HOLE_P=0.02
export REALVEH_REAL_PYTHON="${REAL_PYTHON}"
export PYTHON="${ROOT}/tools/realveh_python"

export AERIAL_RUN_TAG="v2-camk2v2-ep28000-205bars-main-riskcap-camonly-tw40-s${TSEED}${TAG_SUFFIX}"
export TRAIN_SESSION_LOG="train_session_logs/camk2v2_s${TSEED}${TAG_SUFFIX}_$(date +%y%m%d_%H%M%S)_$$.log"
export TRAIN_LIVE_LOG="train_session_logs/current_training.log"

unset NAVRL_RECOVERY_STAGE NAVRL_RECOVERY_SOURCE_EPOCH
unset NAVRL_RECOVERY_SOURCE_SHA256 NAVRL_RECOVERY_SMOKE_REQUIRED_EPOCHS
unset NAVRL_RECOVERY_SMOKE_BARS NAVRL_RECOVERY_EVAL_ATTESTATION_SHA256
unset NAVRL_RECOVERY_EVAL_ATTESTATION_B64
unset NAVRL_GENERAL_EVAL NAVRL_INTERACTIVE NAVRL_BULK_EVAL NAVRL_BULK_EVAL_JSON
unset NAVRL_GENERAL_RESULTS_JSON NAVRL_EVAL_CHECKPOINT NAVRL_EVAL_TARGET_SPEED_FINAL
unset NAVRL_EVAL_FULL_DISTRIBUTION NAVRL_EVAL_ACTION_MODE NAVRL_V2_ACTION_MODE
unset NAVRL_EVAL_RUN_NONCE NAVRL_EVAL_PROFILE NAVRL_SIM_PHYSICS_CONTRACT
unset ALLOW_CONCURRENT NAVRL_V2_FORCE

echo "[camk2v2-train] seed=${TSEED} source=CAM-K1 v1 s${TSEED} ep28000 (${SOURCE_SHA:0:12}) epochs 28001..${MAX_EPOCHS} fixed 205 bars lr 5e-6${TAG_SUFFIX:+ (development smoke)}"
echo "[camk2v2-train] camera only (${NAVRL_SENSOR_ABLATION}); FOV ${NAVRL_CAMTRAIN_HFOV_DEG}x${NAVRL_CAMTRAIN_VFOV_DEG} deg; yaw_align ${NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT}; visibility ${NAVRL_CAMTRAIN_VISIBILITY_BONUS}"
echo "[camk2v2-train] vehicle navrl_quad, per-motor limit at T/W ${NAVRL_VEH_RAND_TW_NOMINAL} (${NAVRL_VEH_RAND_LIMIT_MODE}); veh-rand tw ${NAVRL_VEH_RAND_TW_SCALE} imbal ${NAVRL_VEH_RAND_MOTOR_IMBAL} tau ${NAVRL_VEH_RAND_TAU_MIN}..${NAVRL_VEH_RAND_TAU_MAX} seed ${NAVRL_VEH_RAND_SEED}; depth ${NAVRL_DEPTH_MODEL} minz ${NAVRL_DEPTH_MINZ_M} maxz ${NAVRL_DEPTH_MAXZ_M} sigma_d ${NAVRL_DEPTH_SIGMA_D_PX} fb ${NAVRL_DEPTH_FB_PXM} holes ${NAVRL_DEPTH_HOLE_P} seed ${NAVRL_DEPTH_SEED}"
echo "[camk2v2-train] aerial_gym=${ORIGIN} runner python=${PYTHON}"
if [[ "${NAVRL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    export NAVRL_V2_CONTRACT_PREFLIGHT_ONLY=1
fi
exec ./train_navrl_v2_search.sh --checkpoint "${CKPT}" --branch_run
