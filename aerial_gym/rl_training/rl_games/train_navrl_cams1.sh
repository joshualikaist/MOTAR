#!/usr/bin/env bash
# CAM-S1 (docs/plans/prereg_camonly_s1_scratch_2026-11-23.md; D-2026-11-23-1): a camera-only policy trained FROM
# SCRATCH (no checkpoint) with Policy F's fresh-training launcher contract (train_navrl_v2_search.sh fresh mode),
# except the settings below. The LiDAR is never an input: NAVRL_SENSOR_ABLATION=no_lidar from epoch 0.
#
#   usage: train_navrl_cams1.sh          (all values from the environment set by tools/cams1_train.py)
#
# Differences from F's fresh contract:
#   - camera only (no_lidar), camera 66 x 54 deg, yaw_align_weight 0.6, visibility_bonus 0.03 (CAM-K1's overrides);
#   - CAM-K2 v3's real-vehicle treatments: per-motor limit at T/W 4.0 (INFERRED), thrust +-15 %, imbalance +-5 %,
#     tau 0.02-0.04 s; OAK-D Pro depth model; depth-validity rule blob_near;
#   - the riskcap governor with F's screened parameters, in the loop from epoch 0 (F trained without it until ep24000);
#   - the P9 empirical detector-error injector from epoch 0. On this 160 x 90 image it acts as a detection-loss
#     placeholder (design review r1 P1-3: frames beyond 2.31 m fall into its smallest supported size bin, latency rounds
#     to zero steps, no false detections out of view); it is not a model of recognising a drone;
#   - navrl_camsafe: blind-direction speed cap, blind-motion penalty, collision penalty;
#   - density-curriculum stage minimum and warm-up from CAMS1_DENSITY_MIN_EPOCHS / CAMS1_DENSITY_WARMUP (budget) and a
#     camera-only gate schedule (CAMS1_THRESHOLD_*);
#   - governor FREE speed 2.5 m/s instead of 3.5355 (CAMS1_FREE_MPS);
#   - static-safety reward range 4 m (NAVRL_CAMSAFE_RSS_CLIP_M) instead of 12 m;
#   - checkpoints every NAVRL_SAVE_FREQUENCY epochs.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
ROOT="$(cd ../../.. && pwd)"
REAL_PYTHON="/home/fair/miniconda3/envs/aerialgym/bin/python"
export PATH="$(dirname "${REAL_PYTHON}"):${PATH}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${ROOT}"
ORIGIN="$("${REAL_PYTHON}" -c 'import importlib.util; print(importlib.util.find_spec("aerial_gym").origin)')"
case "${ORIGIN}" in
    "${ROOT}"/*) ;;
    *) echo "[cams1-train] aerial_gym resolves outside this tree: ${ORIGIN}" >&2; exit 2 ;;
esac
: "${CAMS1_MAX_EPOCHS:?}" "${CAMS1_SEED:?}" "${CAMS1_DENSITY_MIN_EPOCHS:?}" "${CAMS1_DENSITY_WARMUP:?}"
: "${NAVRL_REALVEH_RECEIPT:?NAVRL_REALVEH_RECEIPT is required (tools/cams1_train.py sets it)}"
unset CKPT

GOV_SOURCE_SHA="82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"   # ep24000, the screen's source
SUMMARY="../../../results/navrl_v2_ep24000_riskcap_seed44_screen/summary.json"
P9_MODEL="${ROOT}/results/perception_p9_2026-09-09/p9_error_injector.json"
P9_SHA="89879d798a5b10cfb2c0de785dbcfb1754815630b0cd4f91a453de28e728eda9"
P9_FIT="${ROOT}/results/perception_p9_2026-09-09/goodness_of_fit.json"
[[ -f "${SUMMARY}" && -f "${P9_MODEL}" && -f "${P9_FIT}" ]] || { echo "[cams1-train] missing governor summary or P9 model" >&2; exit 2; }
[[ "$(sha256sum "${P9_MODEL}" | cut -d' ' -f1)" == "${P9_SHA}" ]] || { echo "[cams1-train] P9 model SHA mismatch" >&2; exit 2; }
"${REAL_PYTHON}" - "${P9_FIT}" <<'PY'
import json, sys
report = json.loads(open(sys.argv[1]).read())
if report.get("passed") is not True or report.get("test_used"):
    raise SystemExit("[cams1-train] P9 goodness-of-fit not passed or test used; refusing")
PY
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
    print("[cams1-train] refusing:", "; ".join(problems), file=sys.stderr)
    raise SystemExit(2)
print("riskcap", *(params[k] for k in keys))
PY
)"
export NAVRL_SPEED_GOVERNOR="${SELECTED_MODE}"
export NAVRL_SPEED_GOVERNOR_FIXED_MPS="${FIXED_MPS}"
# Reward review r1 (Fable) recommendation 2: no direction faster than forward. F's screened FREE is 3.5355 m/s, which
# the per-axis 2.5 m/s action box reaches only off-axis (2.98 m/s at the 33-degree FOV edge). CAM-S1 sets FREE to
# CAMS1_FREE_MPS (2.5); every other riskcap parameter is the screened one.
echo "[cams1-train] governor FREE ${FREE_MPS} (screened) -> ${CAMS1_FREE_MPS:?} (CAM-S1)"
export NAVRL_SPEED_GOVERNOR_FREE_MPS="${CAMS1_FREE_MPS}"
export NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M="${HALF_WIDTH_M}"
export NAVRL_SPEED_GOVERNOR_MARGIN_M="${MARGIN_M}"
export NAVRL_SPEED_GOVERNOR_SLOW_M="${SLOW_M}"
export NAVRL_SPEED_GOVERNOR_RELEASE_M="${RELEASE_M}"
export NAVRL_SPEED_GOVERNOR_TTC_S="${TTC_S}"
export NAVRL_SPEED_GOVERNOR_BRAKE_MPS2="${BRAKE_MPS2}"
export NAVRL_SPEED_GOVERNOR_REACTION_S="${REACTION_S}"
export NAVRL_SPEED_GOVERNOR_DIAG=1

export NAVRL_V2_ALLOW_RESUME=0
export NAVRL_V2_PROFILE=main
export NAVRL_OBSTACLE_SELECTOR=cluster_sector
export SEED="${CAMS1_SEED}"
export MAX_EPOCHS="${CAMS1_MAX_EPOCHS}"
export NAVRL_SAVE_FREQUENCY="${NAVRL_SAVE_FREQUENCY:-250}"
export NAVRL_DENSITY_CURRICULUM=1
export NAVRL_DENSITY_START=70
export NAVRL_DENSITY_FINAL=205
export NAVRL_DENSITY_MIN_EPOCHS="${CAMS1_DENSITY_MIN_EPOCHS}"
export NAVRL_DENSITY_WARMUP="${CAMS1_DENSITY_WARMUP}"
# Camera-only gate calibration (preregistration "Curriculum"): F's schedule x CAMS1 ratio, set by the driver.
export NAVRL_DENSITY_THRESHOLD_SCHEDULE="${CAMS1_THRESHOLD_SCHEDULE:?}"
export NAVRL_DENSITY_THRESHOLD_START="${CAMS1_THRESHOLD_START:?}"
export NAVRL_DENSITY_THRESHOLD_END="${CAMS1_THRESHOLD_END:?}"

# Camera only + CAM-K1 overrides (tools/camtrain_entry.py through tools/realveh_entry.py).
export NAVRL_SENSOR_ABLATION=no_lidar
export NAVRL_CAMTRAIN_HFOV_DEG=66
export NAVRL_CAMTRAIN_VFOV_DEG=54
export NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT=0.6
export NAVRL_CAMTRAIN_VISIBILITY_BONUS=0.03
# CAM-K2 v3's real-vehicle treatments.
export NAVRL_VEH_RAND=1
export NAVRL_VEH_RAND_SEED="${CAMS1_VEH_SEED:-4931}"
export NAVRL_VEH_RAND_TW_NOMINAL=4.0
export NAVRL_VEH_RAND_LIMIT_MODE=raise
export NAVRL_VEH_RAND_TW_SCALE=0.15
export NAVRL_VEH_RAND_MOTOR_IMBAL=0.05
export NAVRL_VEH_RAND_TAU_MIN=0.02
export NAVRL_VEH_RAND_TAU_MAX=0.04
export NAVRL_DEPTH_MODEL=oakd
export NAVRL_DEPTH_SEED="${CAMS1_DEPTH_SEED:-4941}"
export NAVRL_DEPTH_MINZ_M=0.4
export NAVRL_DEPTH_MAXZ_M=20
export NAVRL_DEPTH_SIGMA_D_PX=0.15
export NAVRL_DEPTH_FB_PXM=66.1875
export NAVRL_DEPTH_HOLE_P=0.02
export NAVRL_DEPTH_VALIDITY=blob_near
export NAVRL_DEPTH_VALIDITY_WINDOW=5
export NAVRL_DEPTH_VALIDITY_DENSITY=0.5
# P9 detector-error injector.
if [[ "${CAMS1_P9:-1}" == "1" ]]; then
    export NAVRL_P9_ERROR_MODEL="${P9_MODEL}"
    export NAVRL_P9_EXPECTED_SHA256="${P9_SHA}"
    export NAVRL_P9_SEED="${CAMS1_P9_SEED:-4951}"
else
    unset NAVRL_P9_ERROR_MODEL NAVRL_P9_EXPECTED_SHA256 NAVRL_P9_SEED
fi
export NAVRL_PERCEPTION_PERTURB=0
# Camera-safety rule and reward terms (aerial_gym/task/navrl_task/navrl_camsafe.py).
export NAVRL_CAMSAFE=1
export NAVRL_CAMSAFE_BLIND_MPS="${CAMS1_BLIND_MPS:?}"
export NAVRL_CAMSAFE_BLIND_WEIGHT="${CAMS1_BLIND_WEIGHT:?}"
export NAVRL_CAMSAFE_COLLISION_PENALTY="${CAMS1_COLLISION_PENALTY:?}"
export NAVRL_CAMSAFE_RSS_CLIP_M="${CAMS1_RSS_CLIP_M:?}"

export REALVEH_REAL_PYTHON="${REAL_PYTHON}"
export PYTHON="${ROOT}/tools/realveh_python"
export AERIAL_RUN_TAG="v2-cams1-scratch-camonly-s${SEED}${CAMS1_TAG_SUFFIX:-}"
export TRAIN_SESSION_LOG="train_session_logs/cams1_s${SEED}${CAMS1_TAG_SUFFIX:-}_$(date +%y%m%d_%H%M%S)_$$.log"
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

echo "[cams1-train] FROM SCRATCH seed=${SEED} max_epochs=${MAX_EPOCHS} density 70->205 min_epochs ${NAVRL_DENSITY_MIN_EPOCHS} warmup ${NAVRL_DENSITY_WARMUP} save every ${NAVRL_SAVE_FREQUENCY}"
echo "[cams1-train] camera only (${NAVRL_SENSOR_ABLATION}); FOV 66x54; yaw_align 0.6; visibility 0.03; P9 ${NAVRL_P9_ERROR_MODEL:+on}; camsafe blind ${NAVRL_CAMSAFE_BLIND_MPS} m/s weight ${NAVRL_CAMSAFE_BLIND_WEIGHT} collision ${NAVRL_CAMSAFE_COLLISION_PENALTY} rss_clip ${NAVRL_CAMSAFE_RSS_CLIP_M} m; gates ${NAVRL_DENSITY_THRESHOLD_SCHEDULE} (start ${NAVRL_DENSITY_THRESHOLD_START} end ${NAVRL_DENSITY_THRESHOLD_END})"
echo "[cams1-train] aerial_gym=${ORIGIN} runner python=${PYTHON}"
if [[ "${NAVRL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    export NAVRL_V2_CONTRACT_PREFLIGHT_ONLY=1
fi
exec ./train_navrl_v2_search.sh
