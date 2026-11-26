#!/usr/bin/env bash
# CAM-S3 S3-W (docs/plans/prereg_camonly_s3w_2026-11-27.md): S2b-FT's camera-only training contract
# (train_navrl_cams2b.sh run ft, receipt ~/motar_tmp/cams2b_20261124/train_ft/receipt.json) continued from the S2b-FT
# ep15000 checkpoint with: yaw-rate ceiling CAMS3_YAW_RATE_MAX (1.5708 rad/s = 90 deg/s), bar pool CAMS3_BAR_POOL
# (bars_h3_thin: 14 of 54 footprints 0.20-0.30 m), both set through PYTHON=tools/cams3_python because
# train_navrl_v2_search.sh pins 3.0 / bars_h3; the measured depth (navrl_cams3 rule 1: recorded OAK-D invalid masks
# and top band; its false near is OFF, review r1 P1-1) replacing the 2 % iid holes; the camera sector reduction (navrl_cams3 rule 2, k-th smallest
# per 5-degree bin).
#
#   usage: train_navrl_cams3.sh          (all values from the environment set by tools/cams3_train.py)
#
# Warm start through the repository's continuation path (NAVRL_V2_ALLOW_RESUME=1, `--checkpoint CKPT --branch_run`,
# absolute MAX_EPOCHS > 15000). The checkpoint restores the epoch clock (num_task_steps 480000) and 205 bars (final);
# the yaw-cap and bar-pool changes reset the density evidence. LR = the checkpoint's current_action_learning_rate.
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
    *) echo "[cams3-train] aerial_gym resolves outside this tree: ${ORIGIN}" >&2; exit 2 ;;
esac
: "${CAMS2_MAX_EPOCHS:?}" "${CAMS2_SEED:?}" "${CAMS2_DENSITY_MIN_EPOCHS:?}" "${CAMS2_DENSITY_WARMUP:?}"
: "${CAMS3_YAW_RATE_MAX:?}" "${CAMS3_BAR_POOL:?}" "${CAMS3_DENSITY_CHECK_EPS:?}" "${CAMS3_SECTOR_K:?}"
: "${CAMS3_CKPT:?CAMS3_CKPT is required for the warm start}" "${CAMS3_LEARNING_RATE:?}"
: "${NAVRL_REALVEH_RECEIPT:?NAVRL_REALVEH_RECEIPT is required (tools/cams3_train.py sets it)}"
for name in $(compgen -e); do
    case "${name}" in
        NAVRL_YAWCLIP*|NAVRL_CAMS2DIAG*|NAVRL_CAMS2_CONTACTCODE*|NAVRL_INVIEW_*)
            echo "[cams3-train] evaluation-only switch ${name} is set; training refuses it" >&2; exit 2 ;;
    esac
done

SOURCE_SHA="f2c80fcc2b2eaeb24e0b05d2284e74594a7ed8e7953f6b17b7dce49b09f2bbab"   # S2b-FT last_gen_ppo_ep_15000
SOURCE_EPOCH=15000
[[ -f "${CAMS3_CKPT}" ]] || { echo "[cams3-train] checkpoint not found: ${CAMS3_CKPT}" >&2; exit 2; }
[[ "$(sha256sum "${CAMS3_CKPT}" | cut -d' ' -f1)" == "${SOURCE_SHA}" ]] \
    || { echo "[cams3-train] checkpoint SHA-256 is not S2b-FT ep15000" >&2; exit 2; }
if (( CAMS2_MAX_EPOCHS <= SOURCE_EPOCH )); then
    echo "[cams3-train] MAX_EPOCHS ${CAMS2_MAX_EPOCHS} must exceed the checkpoint epoch ${SOURCE_EPOCH}" >&2; exit 2
fi
export CKPT="${CAMS3_CKPT}"
export NAVRL_V2_ALLOW_RESUME=1
export NAVRL_RESET_DENSITY_WINDOW=1
export NAVRL_DENSITY_RESUME_WARMUP=1000
export NAVRL_LEARNING_RATE="${CAMS3_LEARNING_RATE}"
RUN_ARGS=(--checkpoint "${CKPT}" --branch_run)
RUN_LABEL="WARM START from S2b-FT ep${SOURCE_EPOCH} (${SOURCE_SHA:0:12}) lr ${NAVRL_LEARNING_RATE}"
TAG="v2-cams3w-yaw90-camonly-s${CAMS2_SEED}"
MASK_LIBRARY="${HOME}/motar_tmp/cams3_20261127/mask_library_v1.npz"
MASK_SHA="55c781136c3ce26635cab61d11c2e5fe4cbdcd28a7433cb6c315f858c603a351"
[[ -f "${MASK_LIBRARY}" && "$(sha256sum "${MASK_LIBRARY}" | cut -d' ' -f1)" == "${MASK_SHA}" ]] \
    || { echo "[cams3-train] mask library missing or not the pinned SHA-256" >&2; exit 2; }

GOV_SOURCE_SHA="82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"   # ep24000, the screen's source
SUMMARY="../../../results/navrl_v2_ep24000_riskcap_seed44_screen/summary.json"
P9_MODEL="${ROOT}/results/perception_p9_2026-09-09/p9_error_injector.json"
P9_SHA="89879d798a5b10cfb2c0de785dbcfb1754815630b0cd4f91a453de28e728eda9"
P9_FIT="${ROOT}/results/perception_p9_2026-09-09/goodness_of_fit.json"
[[ -f "${SUMMARY}" && -f "${P9_MODEL}" && -f "${P9_FIT}" ]] || { echo "[cams3-train] missing governor summary or P9 model" >&2; exit 2; }
[[ "$(sha256sum "${P9_MODEL}" | cut -d' ' -f1)" == "${P9_SHA}" ]] || { echo "[cams3-train] P9 model SHA mismatch" >&2; exit 2; }
"${REAL_PYTHON}" - "${P9_FIT}" <<'PY'
import json, sys
report = json.loads(open(sys.argv[1]).read())
if report.get("passed") is not True or report.get("test_used"):
    raise SystemExit("[cams3-train] P9 goodness-of-fit not passed or test used; refusing")
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
    print("[cams3-train] refusing:", "; ".join(problems), file=sys.stderr)
    raise SystemExit(2)
print("riskcap", *(params[k] for k in keys))
PY
)"
export NAVRL_SPEED_GOVERNOR="${SELECTED_MODE}"
export NAVRL_SPEED_GOVERNOR_FIXED_MPS="${FIXED_MPS}"
echo "[cams3-train] governor FREE ${FREE_MPS} (screened) -> ${CAMS2_FREE_MPS:?} (CAM-S1)"
export NAVRL_SPEED_GOVERNOR_FREE_MPS="${CAMS2_FREE_MPS}"
export NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M="${HALF_WIDTH_M}"
export NAVRL_SPEED_GOVERNOR_MARGIN_M="${MARGIN_M}"
export NAVRL_SPEED_GOVERNOR_SLOW_M="${SLOW_M}"
export NAVRL_SPEED_GOVERNOR_RELEASE_M="${RELEASE_M}"
export NAVRL_SPEED_GOVERNOR_TTC_S="${TTC_S}"
export NAVRL_SPEED_GOVERNOR_BRAKE_MPS2="${BRAKE_MPS2}"
export NAVRL_SPEED_GOVERNOR_REACTION_S="${REACTION_S}"
export NAVRL_SPEED_GOVERNOR_DIAG=1

export NAVRL_V2_PROFILE=main
export NAVRL_OBSTACLE_SELECTOR=cluster_sector
export SEED="${CAMS2_SEED}"
export MAX_EPOCHS="${CAMS2_MAX_EPOCHS}"
export NAVRL_SAVE_FREQUENCY="${NAVRL_SAVE_FREQUENCY:-250}"
export NAVRL_DENSITY_CURRICULUM=1
export NAVRL_DENSITY_START=70
export NAVRL_DENSITY_FINAL=205
export NAVRL_DENSITY_MIN_EPOCHS="${CAMS2_DENSITY_MIN_EPOCHS}"
export NAVRL_DENSITY_WARMUP="${CAMS2_DENSITY_WARMUP}"
# CAM-S2b: the evidence window no longer binds (CAM-S2a review r1 F3; arithmetic in the preregistration).
export NAVRL_DENSITY_CHECK_EPS="${CAMS3_DENSITY_CHECK_EPS}"
export NAVRL_DENSITY_THRESHOLD_SCHEDULE="${CAMS2_THRESHOLD_SCHEDULE:?}"
export NAVRL_DENSITY_THRESHOLD_START="${CAMS2_THRESHOLD_START:?}"
export NAVRL_DENSITY_THRESHOLD_END="${CAMS2_THRESHOLD_END:?}"

# Camera only + CAM-K1 overrides (tools/camtrain_entry.py through tools/realveh_entry.py), as CAM-S2a.
export NAVRL_SENSOR_ABLATION=no_lidar
export NAVRL_CAMTRAIN_HFOV_DEG=66.7
export NAVRL_CAMTRAIN_VFOV_DEG=52.6
export NAVRL_DYNAMIC_MESH_TREATMENT=mesh_flat
export NAVRL_CAMERA_WIDTH="${CAMS2_DETECT_WIDTH:?}"
export NAVRL_CAMERA_HEIGHT="${CAMS2_DETECT_HEIGHT:?}"
export NAVRL_DETECT_WIDTH="${CAMS2_DETECT_WIDTH:?}"
export NAVRL_DETECT_HEIGHT="${CAMS2_DETECT_HEIGHT:?}"
export NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT=0.6
export NAVRL_CAMTRAIN_VISIBILITY_BONUS=0.03
# Vehicle treatments, as CAM-S2a (T/W 3.5 measured).
export NAVRL_VEH_RAND=1
export NAVRL_VEH_RAND_SEED="${CAMS2_VEH_SEED:-4932}"
export NAVRL_VEH_RAND_TW_NOMINAL=3.5
export NAVRL_VEH_RAND_LIMIT_MODE=raise
export NAVRL_VEH_RAND_TW_SCALE=0.15
export NAVRL_VEH_RAND_MOTOR_IMBAL=0.05
export NAVRL_VEH_RAND_TAU_MIN=0.02
export NAVRL_VEH_RAND_TAU_MAX=0.04
export NAVRL_DEPTH_MODEL=oakd
export NAVRL_DEPTH_SEED="${CAMS2_DEPTH_SEED:-4942}"
export NAVRL_DEPTH_MINZ_M=0.315
export NAVRL_DEPTH_MAXZ_M=20
export NAVRL_DEPTH_SIGMA_D_PX=0.125
export NAVRL_DEPTH_FB_PXM=29.88
# CAM-S3: the 2 % iid holes are replaced by the recorded invalid masks (navrl_cams3 rule 1)
export NAVRL_DEPTH_HOLE_P=0
export NAVRL_DEPTH_MEAS=1
export NAVRL_DEPTH_MEAS_LIBRARY="${MASK_LIBRARY}"
export NAVRL_DEPTH_MEAS_SHA256="${MASK_SHA}"
export NAVRL_DEPTH_MEAS_SEED="${CAMS3_MEAS_SEED:-4943}"
export NAVRL_DEPTH_MEAS_TOP_ROWS=10
export NAVRL_DEPTH_MEAS_ADVANCE_P=0.5
export NAVRL_DEPTH_MEAS_JUMP_P=0.01
export NAVRL_DEPTH_MEAS_FALSE_NEAR=0     # review r1 P1-1: the iid false-near model compresses every range; OFF
export NAVRL_CAMS3_SECTOR_K="${CAMS3_SECTOR_K}"
unset NAVRL_DEPTH_VALIDITY NAVRL_DEPTH_VALIDITY_WINDOW NAVRL_DEPTH_VALIDITY_DENSITY
export NAVRL_GAIN_RAND=1
export NAVRL_GAIN_RAND_SCALE=0.2
export NAVRL_GAIN_RAND_SEED="${CAMS2_GAIN_SEED:-4971}"
if [[ "${CAMS2_P9:-1}" == "1" ]]; then
    export NAVRL_P9_ERROR_MODEL="${P9_MODEL}"
    export NAVRL_P9_EXPECTED_SHA256="${P9_SHA}"
    export NAVRL_P9_SEED="${CAMS2_P9_SEED:-4952}"
else
    unset NAVRL_P9_ERROR_MODEL NAVRL_P9_EXPECTED_SHA256 NAVRL_P9_SEED
fi
export NAVRL_PERCEPTION_PERTURB=0
export NAVRL_CAMSAFE=1
export NAVRL_CAMSAFE_BLIND_MPS="${CAMS2_BLIND_MPS:?}"
export NAVRL_CAMSAFE_BLIND_WEIGHT="${CAMS2_BLIND_WEIGHT:?}"
export NAVRL_CAMSAFE_COLLISION_PENALTY="${CAMS2_COLLISION_PENALTY:?}"
export NAVRL_CAMSAFE_RSS_CLIP_M="${CAMS2_RSS_CLIP_M:?}"
export NAVRL_CAMS2=1
export NAVRL_CAMS2_DELAY_MIN="${CAMS2_DELAY_MIN:?}"
export NAVRL_CAMS2_DELAY_MAX="${CAMS2_DELAY_MAX:?}"
export NAVRL_ACTION_DELAY_STEPS_MAX="${CAMS2_DELAY_MAX}"
export NAVRL_CAMS2_DELAY_SEED="${CAMS2_DELAY_SEED:-4961}"
export NAVRL_CAMS2_CAPTURE_RADIUS_M=1.0
export NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS=1.0
export NAVRL_CAMS2_CONTACT_RADIUS_M=0.40
export NAVRL_CAMS2_FLIGHT_ALTITUDE_M=1.5
export NAVRL_CAMS2_CAMERA_X_M=0.08

# Yaw ceiling and bar pool: applied by tools/cams3_python after train_navrl_v2_search.sh has pinned 3.0 / bars_h3.
export CAMS3_YAW_RATE_MAX CAMS3_BAR_POOL
export REALVEH_REAL_PYTHON="${REAL_PYTHON}"
export PYTHON="${ROOT}/tools/cams3_python"
export AERIAL_RUN_TAG="${TAG}${CAMS2_TAG_SUFFIX:-}"
export TRAIN_SESSION_LOG="train_session_logs/cams3_w_s${SEED}${CAMS2_TAG_SUFFIX:-}_$(date +%y%m%d_%H%M%S)_$$.log"
export TRAIN_LIVE_LOG="train_session_logs/current_training.log"

unset NAVRL_RECOVERY_STAGE NAVRL_RECOVERY_SOURCE_EPOCH
unset NAVRL_RECOVERY_SOURCE_SHA256 NAVRL_RECOVERY_SMOKE_REQUIRED_EPOCHS
unset NAVRL_RECOVERY_SMOKE_BARS NAVRL_RECOVERY_EVAL_ATTESTATION_SHA256
unset NAVRL_RECOVERY_EVAL_ATTESTATION_B64
unset NAVRL_GENERAL_EVAL NAVRL_INTERACTIVE NAVRL_BULK_EVAL NAVRL_BULK_EVAL_JSON
unset NAVRL_GENERAL_RESULTS_JSON NAVRL_EVAL_CHECKPOINT NAVRL_EVAL_TARGET_SPEED_FINAL
unset NAVRL_EVAL_FULL_DISTRIBUTION NAVRL_EVAL_ACTION_MODE NAVRL_V2_ACTION_MODE
unset NAVRL_EVAL_RUN_NONCE NAVRL_EVAL_PROFILE NAVRL_SIM_PHYSICS_CONTRACT
unset ALLOW_CONCURRENT NAVRL_V2_FORCE NAVRL_NUM_BARS

echo "[cams3-train] w: ${RUN_LABEL} seed=${SEED} max_epochs=${MAX_EPOCHS} density 70->205 min_epochs ${NAVRL_DENSITY_MIN_EPOCHS} warmup ${NAVRL_DENSITY_WARMUP} check_eps ${NAVRL_DENSITY_CHECK_EPS} save every ${NAVRL_SAVE_FREQUENCY}"
echo "[cams3-train] yaw ceiling ${CAMS3_YAW_RATE_MAX} rad/s, bar pool ${CAMS3_BAR_POOL}, sector k ${NAVRL_CAMS3_SECTOR_K} (via ${PYTHON}); camera only (${NAVRL_SENSOR_ABLATION}); FOV ${NAVRL_CAMTRAIN_HFOV_DEG}x${NAVRL_CAMTRAIN_VFOV_DEG}; camera/detect ${NAVRL_CAMERA_WIDTH}x${NAVRL_CAMERA_HEIGHT}"
echo "[cams3-train] aerial_gym=${ORIGIN}"
if [[ "${NAVRL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    export NAVRL_V2_CONTRACT_PREFLIGHT_ONLY=1
fi
exec ./train_navrl_v2_search.sh ${RUN_ARGS[@]+"${RUN_ARGS[@]}"}
