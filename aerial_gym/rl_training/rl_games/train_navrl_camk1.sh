#!/usr/bin/env bash
# CAM-K1: camera-only continuation of Frozen Policy F (ep25000 riskcap), a new policy lineage
# (D-2026-11-04-1, D-2026-11-04-3; docs/plans/prereg_camonly_k1_train_v1_2026-11-06.md). Policy F itself is never
# modified: the run branches into a new runs/ folder and writes a new checkpoint lineage.
#
#   usage: train_navrl_camk1.sh SEED          (SEED 1 or 2)
#
# Everything is GR-1 ctrl's contract (train_navrl_gr1.sh: fixed 205 bars, curriculum off, LR 5e-6, the riskcap
# governor parameters from the same validated screen summary, main profile, 128 envs), except:
#   - the camera-only ablation (NAVRL_SENSOR_ABLATION=no_lidar) and the CAM-K1 overrides, installed by
#     tools/camtrain_entry.py through PYTHON=tools/camtrain_python for the runner only:
#     camera FOV 66 x 54 deg (OAK-D Pro autofocus RGB), yaw_align_weight 0.6 (F: 0.3), visibility_bonus 0.03 (F: 0.02);
#   - 3,000 epochs (25001..28000) and the training seed given here.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
HERE="$(pwd)"
ROOT="$(cd ../../.. && pwd)"
if (( $# != 1 )) || [[ "$1" != "1" && "$1" != "2" ]]; then
    echo "[camk1-train] usage: train_navrl_camk1.sh {1|2}" >&2
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
    *) echo "[camk1-train] aerial_gym resolves outside this tree: ${ORIGIN}" >&2; exit 2 ;;
esac
SOURCE_REL="runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth"
SOURCE_SHA="f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
GOV_SOURCE_SHA="82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"   # ep24000, the screen's source
SUMMARY="../../../results/navrl_v2_ep24000_riskcap_seed44_screen/summary.json"
export CKPT="${SOURCE_REL}"
[[ -f "${CKPT}" && -f "${SUMMARY}" ]] || { echo "[camk1-train] missing Policy F checkpoint or screen summary" >&2; exit 2; }
[[ "$(sha256sum "${CKPT}" | cut -d' ' -f1)" == "${SOURCE_SHA}" ]] || { echo "[camk1-train] Policy F SHA mismatch" >&2; exit 2; }
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
    print("[camk1-train] refusing:", "; ".join(problems), file=sys.stderr)
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

# GR-1 ctrl's fixed-density continuation contract, now 25001..28000 with the given training seed.
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
export MAX_EPOCHS=28000
TAG_SUFFIX=""
if [[ "${CAMK1_SMOKE:-0}" == "1" ]]; then    # the unit's development smoke: two epochs, its own run tag
    export MAX_EPOCHS=25002
    TAG_SUFFIX="-smoke"
fi
unset NAVRL_GAIN_RAND NAVRL_GAIN_RAND_SCALE NAVRL_GAIN_RAND_SEED

# CAM-K1: camera only, OAK-D Pro RGB field of view, FOV-keeping reward weights (tools/camtrain_entry.py).
export NAVRL_SENSOR_ABLATION=no_lidar
export NAVRL_CAMTRAIN_HFOV_DEG=66
export NAVRL_CAMTRAIN_VFOV_DEG=54
export NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT=0.6
export NAVRL_CAMTRAIN_VISIBILITY_BONUS=0.03
export NAVRL_CAMTRAIN_RECEIPT="${NAVRL_CAMTRAIN_RECEIPT:?NAVRL_CAMTRAIN_RECEIPT is required (tools/camk1_train.py sets it)}"
export CAMTRAIN_REAL_PYTHON="${REAL_PYTHON}"
export PYTHON="${ROOT}/tools/camtrain_python"

export AERIAL_RUN_TAG="v2-camk1-ep25000-205bars-main-riskcap-camonly-s${TSEED}${TAG_SUFFIX}"
export TRAIN_SESSION_LOG="train_session_logs/camk1_s${TSEED}${TAG_SUFFIX}_$(date +%y%m%d_%H%M%S)_$$.log"
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

echo "[camk1-train] seed=${TSEED} source=Policy F (${SOURCE_SHA:0:12}) epochs 25001..${MAX_EPOCHS} fixed 205 bars lr 5e-6${TAG_SUFFIX:+ (development smoke)}"
echo "[camk1-train] camera only (${NAVRL_SENSOR_ABLATION}); FOV ${NAVRL_CAMTRAIN_HFOV_DEG}x${NAVRL_CAMTRAIN_VFOV_DEG} deg; yaw_align ${NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT}; visibility ${NAVRL_CAMTRAIN_VISIBILITY_BONUS}"
echo "[camk1-train] aerial_gym=${ORIGIN} runner python=${PYTHON}"
if [[ "${NAVRL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    export NAVRL_V2_CONTRACT_PREFLIGHT_ONLY=1
fi
exec ./train_navrl_v2_search.sh --checkpoint "${CKPT}" --branch_run
