#!/usr/bin/env bash
# CAM-K2-E (docs/plans/prereg_camonly_k2_eval_v1_2026-11-08.md): a copy of CAM-K1-E's launcher
# eval_navrl_camk1e_arms.sh, itself a copy of the pinned eval_navrl_target_behavior_arms.sh, with five changes to it,
# all marked [camk2e]:
#   1. CAMEVAL_PY is the absolute path of tools/realveh_python (CAM-K2 v2's entry point: the camera-only ablation, the
#      CAM-K1 FOV / reward-weight overrides and, when switched on, the vehicle randomization with the per-motor limit
#      raised to T/W 4.2 and the OAK-D Pro depth model); the real interpreter goes to REALVEH_REAL_PYTHON;
#   2. CAMK2E_TREATMENTS is required: "on" needs NAVRL_VEH_RAND=1 and NAVRL_DEPTH_MODEL=oakd from the caller
#      (tools/camk2e_run.py sets every value and the per-cell seeds), "off" needs both unset (the nominal arms and the
#      identity cell);
#   3. the checkpoint and its expected SHA-256 come from the caller as CAMK2E_CKPT, CAMK2E_CKPT_SHA;
#   4. each cell gets its own NAVRL_REALVEH_RECEIPT inside the cell directory (the entry also writes the
#      vehicle-randomization record next to it);
#   5. the per-cell NAVRL_CAMTRAIN_RECEIPT of CAM-K1-E's copy is not set (the realveh entry writes one receipt).
# Everything else is byte-for-byte CAM-K1-E's launcher after its own header, which follows:
#
# Frozen-policy H/E0/E1/E2 target-behaviour evaluation (preregistered).
#
#   docs/prereg_2026-09-17_target_motion_complexity_e0_e1_e2.md  (Amendment 1)
#   subject commit      a44c8d32622b97a0f7a49ecb0d4255cdae8653c8
#   attestation commit  096eee5e7067f1eea8de40180c5edcce93760521
#
# Why this is a separate launcher and not eval_navrl_v2_density_sweep.sh: that
# sweep unconditionally exports NAVRL_TARGET_SPEED_MIN=0.3 / _FINAL=1.5 and has
# no behaviour-level axis, so it would silently clobber the E0 zero-speed arm.
# Existing evaluation machinery is left byte-identical.
#
# The held-fixed environment below is REPLAYED FROM THE FROZEN CHECKPOINT'S OWN
# env_state, so the policy is evaluated under the contract it was trained in.
# HEAD defaults differ on 14 knobs; see
# docs/audits/training_contract_vs_current_runtime_2026-09-17.json.
#
# This script evaluates a frozen checkpoint. It never trains.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../../.." && pwd)"
cd "${HERE}"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PYTHONNOUSERSITE=1
# [camk1e] 1: the condition and the entry-point wrapper come from the caller; both interpreters are absolute paths
: "${NAVRL_SENSOR_ABLATION:?NAVRL_SENSOR_ABLATION is required}"
: "${CAMEVAL_PY:?CAMEVAL_PY (absolute path of tools/realveh_python) is required}"   # [camk2e] 1
[[ "${CAMEVAL_PY}" == /* && -x "${CAMEVAL_PY}" ]] || { echo "[camk1e] CAMEVAL_PY must be an absolute executable path" >&2; exit 2; }
[[ "${PYTHON}" == /* ]] || { echo "[camk1e] PYTHON must be an absolute path" >&2; exit 2; }
export REALVEH_REAL_PYTHON="${PYTHON}"   # [camk2e] 1
# [camk2e] 2: the treatment switch; every value and the per-cell seeds come from tools/camk2e_run.py
case "${CAMK2E_TREATMENTS:?CAMK2E_TREATMENTS (on or off) is required}" in
    on)  [[ "${NAVRL_VEH_RAND:-}" == "1" && "${NAVRL_DEPTH_MODEL:-}" == "oakd" ]] || { echo "[camk2e] treatments on needs NAVRL_VEH_RAND=1 and NAVRL_DEPTH_MODEL=oakd" >&2; exit 2; } ;;
    off) [[ -z "${NAVRL_VEH_RAND:-}" && -z "${NAVRL_DEPTH_MODEL:-}" ]] || { echo "[camk2e] treatments off needs NAVRL_VEH_RAND and NAVRL_DEPTH_MODEL unset" >&2; exit 2; } ;;
    *)   echo "[camk2e] CAMK2E_TREATMENTS must be on or off" >&2; exit 2 ;;
esac

# [camk2e] 3: the checkpoint and its expected SHA-256 come from the caller (tools/camk2e_run.py); both are required
CKPT="${CAMK2E_CKPT:?CAMK2E_CKPT is required}"
EXPECT_SHA="${CAMK2E_CKPT_SHA:?CAMK2E_CKPT_SHA is required}"

ARMS_TEXT="${ARMS:-H_historical E0_static E1_cv E2_obstacle_aware}"
DENS_TEXT="${DENSITIES:-70 115 160 205}"
SEEDS_TEXT="${SEEDS:-4101 4102 4103}"
GAMES="${GAMES:-2048}"
NUM_ENVS="${NUM_ENVS:-128}"
OUT_ROOT="${OUT_ROOT:-${ROOT}/results/target_motion_e0_e2_evaluation_2026-09-18}"

read -r -a ARMS_ARR <<< "${ARMS_TEXT}"
read -r -a DENS_ARR <<< "${DENS_TEXT}"
read -r -a SEEDS_ARR <<< "${SEEDS_TEXT}"

sha256_file() { sha256sum "$1" | cut -d' ' -f1; }

# ---- fail closed on the checkpoint ---------------------------------------
[[ -f "${CKPT}" ]] || { echo "[arms] missing checkpoint: ${CKPT}" >&2; exit 2; }
ACTUAL_SHA="$(sha256_file "${CKPT}")"
if [[ "${ACTUAL_SHA}" != "${EXPECT_SHA}" ]]; then
    echo "[arms] checkpoint SHA mismatch. expected ${EXPECT_SHA} got ${ACTUAL_SHA}" >&2
    exit 2
fi

GIT_COMMIT="$(git -C "${ROOT}" rev-parse HEAD)"
GIT_DIRTY="$(git -C "${ROOT}" status --porcelain | wc -l)"

# ================= HELD FIXED: the checkpoint's attested contract ==========
export NAVRL_VISION=1 NAVRL_PERCEPTION=1
export TASK=navrl_task
export FILE=ppo_navrl_perception_transformer.yaml
export AERIAL_GYM_SIM_NAME="${AERIAL_GYM_SIM_NAME:-base_sim}"

export NAVRL_ARENA_XY=40 NAVRL_ARENA_Z=3
export NAVRL_BAR_POOL=bars_h3
export NAVRL_PLACEMENT_MODE=navrl_band
export NAVRL_PLACEMENT_TOUCH_M=0.4 NAVRL_PLACEMENT_GAP_M=1.6
export NAVRL_BAR_X_MIN=0.0 NAVRL_BAR_X_MAX=1.0
export NAVRL_MAX_BARS=300
export NAVRL_EPISODE_LEN_STEPS=600
export NAVRL_OOB_MARGIN=1.0

export NAVRL_LIDAR_HBEAMS=72 NAVRL_LIDAR_VBEAMS=4 NAVRL_LIDAR_RANGE=12
export NAVRL_MAX_OBSTACLES=8
export NAVRL_OBSTACLE_SELECTOR=cluster_sector
export NAVRL_OBSTACLE_FOV_DEG=240
export NAVRL_OBSTACLE_SUPPRESS_DEG=10
export NAVRL_OBSTACLE_SECTORS=8
export NAVRL_OBSTACLE_CLUSTER_GAP_M=0.45
export NAVRL_OBSTACLE_TTC_IDLE_S=30 NAVRL_OBSTACLE_TTC_MIN_SPEED=0.15

export NAVRL_MAX_VELOCITY=2.5 NAVRL_YAW_RATE_MAX=3.0
export NAVRL_MAX_TILT_DEG=45 NAVRL_ALT_HOLD_VMAX=2.5
export NAVRL_TILT_COMP=1

export NAVRL_SPEED_GOVERNOR=riskcap
export NAVRL_SPEED_GOVERNOR_FIXED_MPS=2.0
export NAVRL_SPEED_GOVERNOR_FREE_MPS=3.53553390593
export NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M=0.45
export NAVRL_SPEED_GOVERNOR_MARGIN_M=0.45
export NAVRL_SPEED_GOVERNOR_SLOW_M=3.0
export NAVRL_SPEED_GOVERNOR_RELEASE_M=5.0
export NAVRL_SPEED_GOVERNOR_TTC_S=1.2
export NAVRL_SPEED_GOVERNOR_BRAKE_MPS2=2.9608856678
export NAVRL_SPEED_GOVERNOR_REACTION_S=0.1

export NAVRL_DETECTOR_THRESHOLD=0.55
export NAVRL_DETECTION_DROPOUT=0.3
export NAVRL_DEPTH_NOISE_STD=0.02
export NAVRL_RGB_NOISE_STD=0.015

export NAVRL_GENERAL_TRAIN=1
export NAVRL_GENERAL_GOAL_DIST_MIN=6 NAVRL_GENERAL_GOAL_DIST_MAX=28
export NAVRL_DENSITY_CURRICULUM=0
export NAVRL_TARGET_ROUTE_MODE=off
export NAVRL_EVAL_CV_INITIAL_HEADING=random
export NAVRL_V2_PROFILE=main

export NAVRL_BULK_EVAL=1
export NAVRL_EVAL_CHECKPOINT="${EXPECT_SHA}"
export HEADLESS=True
# =========================================================================

# ---- the ONLY axis that varies between arms ------------------------------
apply_arm() {
    case "$1" in
      H_historical)
        export NAVRL_TARGET_BEHAVIOR_LEVEL=historical
        export NAVRL_TARGET_DYNAMICS=legacy
        export NAVRL_TARGET_PATTERN=mixed
        export NAVRL_TARGET_SPEED_MIN=0.3 NAVRL_TARGET_SPEED_FINAL=1.5 ;;
      E0_static)
        export NAVRL_TARGET_BEHAVIOR_LEVEL=e0_static
        export NAVRL_TARGET_DYNAMICS=bounded
        export NAVRL_TARGET_PATTERN=cv
        export NAVRL_TARGET_SPEED_MIN=0.0 NAVRL_TARGET_SPEED_FINAL=0.0 ;;
      E1_cv)
        export NAVRL_TARGET_BEHAVIOR_LEVEL=e1_cv
        export NAVRL_TARGET_DYNAMICS=bounded
        export NAVRL_TARGET_PATTERN=cv
        export NAVRL_TARGET_SPEED_MIN=0.3 NAVRL_TARGET_SPEED_FINAL=1.5 ;;
      E2_obstacle_aware)
        export NAVRL_TARGET_BEHAVIOR_LEVEL=e2_obstacle_aware
        export NAVRL_TARGET_DYNAMICS=bounded
        export NAVRL_TARGET_PATTERN=waypoint
        export NAVRL_TARGET_SPEED_MIN=0.3 NAVRL_TARGET_SPEED_FINAL=1.5 ;;
      *) echo "[arms] unknown arm: $1" >&2; exit 2 ;;
    esac
    export NAVRL_TARGET_SPEED_RAMP_EPOCHS=300
}

mkdir -p "${OUT_ROOT}"
echo "[arms] commit=${GIT_COMMIT} dirty=${GIT_DIRTY} ckpt=${ACTUAL_SHA:0:16}..."
echo "[arms] arms=${ARMS_TEXT}"
echo "[arms] densities=${DENS_TEXT} seeds=${SEEDS_TEXT} games/cell=${GAMES} envs=${NUM_ENVS}"
echo "[arms] out=${OUT_ROOT}"

TOTAL=$(( ${#ARMS_ARR[@]} * ${#DENS_ARR[@]} * ${#SEEDS_ARR[@]} ))
INDEX=0
for ARM in "${ARMS_ARR[@]}"; do
  for N in "${DENS_ARR[@]}"; do
    for SEED in "${SEEDS_ARR[@]}"; do
      INDEX=$(( INDEX + 1 ))
      CELL_ID="${ARM}__${N}bars__seed${SEED}"
      CELL_DIR="${OUT_ROOT}/${CELL_ID}"
      RESULT_JSON="${CELL_DIR}/result.json"
      RECEIPT_JSON="${CELL_DIR}/receipt.json"
      CELL_LOG="${CELL_DIR}/cell.log"

      # never overwrite a completed cell; resume skips it
      if [[ -s "${RESULT_JSON}" && -s "${RECEIPT_JSON}" ]]; then
        echo "[arms] (${INDEX}/${TOTAL}) SKIP completed ${CELL_ID}"
        continue
      fi
      mkdir -p "${CELL_DIR}"
      export NAVRL_REALVEH_RECEIPT="${CELL_DIR}/realveh_receipt.json"   # [camk2e] 4, 5
      apply_arm "${ARM}"
      export NAVRL_NUM_BARS="${N}"
      export NAVRL_DENSITY_START="${N}" NAVRL_DENSITY_FINAL="${N}"
      export NAVRL_SEED="${SEED}"
      export NAVRL_BULK_EVAL_JSON="${RESULT_JSON}"
      NONCE="$(${PYTHON} -c 'import secrets;print(secrets.token_hex(32))')"
      export NAVRL_EVAL_RUN_NONCE="${NONCE}"
      STARTED="$(${PYTHON} -c 'from datetime import datetime,timezone;print(datetime.now(timezone.utc).isoformat())')"
      SECONDS=0

      echo "======== (${INDEX}/${TOTAL}) ${CELL_ID} ========"
      set +e
      NUM_ENVS="${NUM_ENVS}" PLAY_GAMES_NUM="${GAMES}" PYTHON="${CAMEVAL_PY}" \
        ./play_navrl.sh "${CKPT}" > "${CELL_LOG}" 2>&1   # [camk1e] 4
      RC=$?
      set -e
      DURATION=${SECONDS}
      FINISHED="$(${PYTHON} -c 'from datetime import datetime,timezone;print(datetime.now(timezone.utc).isoformat())')"

      if [[ ${RC} -ne 0 || ! -s "${RESULT_JSON}" ]]; then
        echo "[arms] CELL FAILED rc=${RC}: ${CELL_ID} (log kept: ${CELL_LOG})" >&2
        ${PYTHON} - "${RECEIPT_JSON}" "${CELL_ID}" "${ARM}" "${N}" "${SEED}" "${GAMES}" \
          "${GIT_COMMIT}" "${GIT_DIRTY}" "${EXPECT_SHA}" "${NONCE}" "${STARTED}" "${FINISHED}" \
          "${DURATION}" "${RC}" "FAILED" <<'PY'
import json, sys
(out, cid, arm, bars, seed, games, commit, dirty, ckpt, nonce,
 started, finished, duration, rc, status) = sys.argv[1:16]
json.dump({"cell_id": cid, "arm": arm, "density_bars": int(bars), "seed": int(seed),
           "requested_episodes": int(games), "git_commit": commit,
           "git_dirty_files": int(dirty), "checkpoint_sha256": ckpt,
           "evaluation_nonce": nonce, "started_at": started, "finished_at": finished,
           "duration_s": int(duration), "return_code": int(rc), "status": status},
          open(out, "w"), indent=1, sort_keys=True)
PY
        continue
      fi

      RESULT_SHA="$(sha256_file "${RESULT_JSON}")"
      ${PYTHON} - "${RECEIPT_JSON}" "${CELL_ID}" "${ARM}" "${N}" "${SEED}" "${GAMES}" \
        "${GIT_COMMIT}" "${GIT_DIRTY}" "${EXPECT_SHA}" "${NONCE}" "${STARTED}" "${FINISHED}" \
        "${DURATION}" "${RESULT_SHA}" "${RESULT_JSON}" <<'PY'
import json, os, sys
(out, cid, arm, bars, seed, games, commit, dirty, ckpt, nonce,
 started, finished, duration, result_sha, result_path) = sys.argv[1:16]
payload = json.load(open(result_path))
cond = payload.get("condition", {})
env_snapshot = {k: v for k, v in os.environ.items() if k.startswith("NAVRL_")}
json.dump({
    "cell_id": cid, "arm": arm, "density_bars": int(bars), "seed": int(seed),
    "requested_episodes": int(games),
    "actual_episodes": payload.get("actual_episodes"),
    "git_commit": commit, "git_dirty_files": int(dirty),
    "checkpoint_sha256": ckpt, "evaluation_nonce": nonce,
    "started_at": started, "finished_at": finished, "duration_s": int(duration),
    "result_sha256": result_sha, "status": "OK",
    "condition_echo": {k: cond.get(k) for k in (
        "bars", "seed", "target_pattern", "episode_len_steps",
        "cfg_target_pattern", "cfg_target_speed_min", "cfg_target_speed_final",
        "cfg_max_velocity", "cfg_lidar_hbeams", "cfg_lidar_max_range")},
    "env_snapshot_navrl": env_snapshot,
}, open(out, "w"), indent=1, sort_keys=True)
PY
      echo "[arms] (${INDEX}/${TOTAL}) done ${CELL_ID} in ${DURATION}s"
    done
  done
done
echo "[arms] ALL_CELLS_ATTEMPTED"
