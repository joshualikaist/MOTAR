#!/usr/bin/env bash
# CAM-S2a-E (docs/plans/prereg_camonly_s2a_eval_2026-11-24.md): a copy of CAM-K2-E's launcher eval_navrl_camk2e_arms.sh
# (itself CAM-K1-E's copy of the pinned eval_navrl_target_behavior_arms.sh) with these changes, all marked [cams2a]:
#   1. the HELD-FIXED block of Policy F's contract is REMOVED. The CAM-S2a policy trained under its own contract
#      (~/motar_tmp/cams2a_20261123/train_s1/receipt.json, key env_navrl: governor FREE 2.5 m/s, legacy mixed target,
#      navrl_band placement without an initial-view rule, gain randomisation ON, camera 312 x 234 at 66.7 x 52.6 deg,
#      the D8 quadrotor target, cams2 delay / capture / altitude / mount). tools/cams2a_eval_run.py exports that
#      environment, minus the training-only keys it replaces, into this launcher; the launcher refuses to run unless the
#      keys that define the task are present (fail closed, review r1 F2) and adds only the evaluation-mode and per-cell
#      variables. apply_arm is gone: the target is the trained one, and the arm label is a cell-id prefix only;
#   2. CAMS2A_CKPT / CAMS2A_CKPT_SHA name the checkpoint (a last_gen_ppo_ep_*.pth, never gen_ppo.pth);
#   3. CAMS2A_TRAIN_RECEIPT_SHA256 is recorded in every cell receipt (the environment's provenance);
#   4. per cell: NAVRL_REALVEH_RECEIPT and NAVRL_CAMS2DIAG_JSONL inside the cell directory;
#   5. the yaw-clip condition (NAVRL_YAWCLIP*) and the recorder switch come from the caller unchanged.
# Everything else (checkpoint SHA check, cell loop, receipt writer, resume rule) is CAM-K2-E's launcher after its header.
# This script evaluates a frozen checkpoint. It never trains.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../../.." && pwd)"
cd "${HERE}"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PYTHONNOUSERSITE=1
: "${CAMEVAL_PY:?CAMEVAL_PY (absolute path of tools/realveh_python) is required}"
[[ "${CAMEVAL_PY}" == /* && -x "${CAMEVAL_PY}" ]] || { echo "[cams2a] CAMEVAL_PY must be an absolute executable path" >&2; exit 2; }
[[ "${PYTHON}" == /* ]] || { echo "[cams2a] PYTHON must be an absolute path" >&2; exit 2; }
export REALVEH_REAL_PYTHON="${PYTHON}"

# [cams2a] 1: the task-defining keys of the training environment must be present (tools/cams2a_eval_run.py exports them)
for K in NAVRL_VISION NAVRL_PERCEPTION NAVRL_SENSOR_ABLATION NAVRL_CAMSAFE NAVRL_CAMS2 NAVRL_CAMS2_DELAY_MIN NAVRL_CAMS2_DELAY_MAX \
         NAVRL_CAMS2_DELAY_SEED NAVRL_CAMS2_CAPTURE_RADIUS_M NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS NAVRL_CAMS2_CONTACT_RADIUS_M \
         NAVRL_CAMS2_FLIGHT_ALTITUDE_M NAVRL_CAMS2_CAMERA_X_M NAVRL_ACTION_DELAY_STEPS_MAX NAVRL_VEH_RAND NAVRL_DEPTH_MODEL \
         NAVRL_DYNAMIC_MESH_TREATMENT NAVRL_CAMERA_WIDTH NAVRL_CAMERA_HEIGHT NAVRL_DETECT_WIDTH NAVRL_DETECT_HEIGHT \
         NAVRL_CAMTRAIN_HFOV_DEG NAVRL_CAMTRAIN_VFOV_DEG NAVRL_SPEED_GOVERNOR NAVRL_SPEED_GOVERNOR_FREE_MPS NAVRL_MAX_VELOCITY \
         NAVRL_YAW_RATE_MAX NAVRL_TARGET_DYNAMICS NAVRL_TARGET_PATTERN NAVRL_PLACEMENT_MODE NAVRL_EPISODE_LEN_STEPS \
         NAVRL_GAIN_RAND NAVRL_EPISODE_LEDGER NAVRL_EPISODE_QUOTA_PER_ENV NAVRL_PLAYER_GAMES_CAP \
         CAMS2A_TRAIN_RECEIPT_SHA256 AERIAL_GYM_SIM_NAME PYTHON; do
    [[ -n "${!K:-}" ]] || { echo "[cams2a] ${K} is required (the training environment is exported by tools/cams2a_eval_run.py)" >&2; exit 2; }
done
[[ "${NAVRL_CAMS2}" == "1" && "${NAVRL_CAMSAFE}" == "1" && "${NAVRL_SENSOR_ABLATION}" == "no_lidar" ]] \
    || { echo "[cams2a] NAVRL_CAMS2=1, NAVRL_CAMSAFE=1 and NAVRL_SENSOR_ABLATION=no_lidar are required" >&2; exit 2; }
# [cams2a] the recorder is ON in every cell except the identity stage's recorder-OFF control (review r1 P1-1)
[[ "${NAVRL_CAMS2DIAG:-}" == "1" || -z "${NAVRL_CAMS2DIAG:-}" ]] || { echo "[cams2a] NAVRL_CAMS2DIAG must be 1 or unset" >&2; exit 2; }
# [cams2a] review r1 P2-2: the simulator and interpreter are pinned by the caller, never inherited from a shell
[[ "${AERIAL_GYM_SIM_NAME}" == "base_sim" && "${PYTHON}" == "/home/fair/miniconda3/envs/aerialgym/bin/python" && -z "${GPU4GB:-}" ]] \
    || { echo "[cams2a] AERIAL_GYM_SIM_NAME must be base_sim, PYTHON the aerialgym interpreter, GPU4GB unset" >&2; exit 2; }
[[ "${NAVRL_DENSITY_CURRICULUM:-}" == "0" ]] || { echo "[cams2a] NAVRL_DENSITY_CURRICULUM must be 0 (fixed density)" >&2; exit 2; }
[[ -z "${NAVRL_REALVEH_RECEIPT:-}" && -z "${NAVRL_CAMS2DIAG_JSONL:-}" && -z "${NAVRL_SEED:-}" && -z "${NAVRL_NUM_BARS:-}" ]] \
    || { echo "[cams2a] NAVRL_REALVEH_RECEIPT, NAVRL_CAMS2DIAG_JSONL, NAVRL_SEED and NAVRL_NUM_BARS are set per cell here" >&2; exit 2; }

# [cams2a] 2: the checkpoint and its expected SHA-256 come from the caller; gen_ppo.pth is refused
CKPT="${CAMS2A_CKPT:?CAMS2A_CKPT is required}"
EXPECT_SHA="${CAMS2A_CKPT_SHA:?CAMS2A_CKPT_SHA is required}"
[[ "$(basename "${CKPT}")" == last_gen_ppo_ep_*.pth ]] || { echo "[cams2a] the checkpoint must be a last_gen_ppo_ep_*.pth, got ${CKPT}" >&2; exit 2; }

ARMS_TEXT="${ARMS:-S2A_trained}"
DENS_TEXT="${DENSITIES:?DENSITIES is required}"
SEEDS_TEXT="${SEEDS:?SEEDS is required}"
GAMES="${GAMES:?GAMES is required}"
NUM_ENVS="${NUM_ENVS:-128}"
OUT_ROOT="${OUT_ROOT:?OUT_ROOT is required}"

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

# ================= evaluation mode only; the task contract is the caller's (training receipt) ==========
export TASK=navrl_task
export FILE=ppo_navrl_perception_transformer.yaml
export AERIAL_GYM_SIM_NAME="${AERIAL_GYM_SIM_NAME}"
export NAVRL_BULK_EVAL=1
export NAVRL_EVAL_CHECKPOINT="${EXPECT_SHA}"
export HEADLESS=True
# =========================================================================

mkdir -p "${OUT_ROOT}"
echo "[arms] commit=${GIT_COMMIT} dirty=${GIT_DIRTY} ckpt=${ACTUAL_SHA:0:16}... train_receipt=${CAMS2A_TRAIN_RECEIPT_SHA256:0:16}..."
echo "[arms] arms=${ARMS_TEXT} (label only; the target is the trained one: ${NAVRL_TARGET_DYNAMICS}/${NAVRL_TARGET_PATTERN})"
echo "[arms] densities=${DENS_TEXT} seeds=${SEEDS_TEXT} games/cell=${GAMES} envs=${NUM_ENVS} yawclip=${NAVRL_YAWCLIP:-off}"
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
      export NAVRL_REALVEH_RECEIPT="${CELL_DIR}/realveh_receipt.json"          # [cams2a] 4
      if [[ "${NAVRL_CAMS2DIAG:-}" == "1" ]]; then
        export NAVRL_CAMS2DIAG_JSONL="${CELL_DIR}/cams2diag_episodes.jsonl"     # [cams2a] 4
      else
        unset NAVRL_CAMS2DIAG_JSONL NAVRL_CAMS2DIAG_YAW_REF_RADPS
      fi
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
        ./play_navrl.sh "${CKPT}" > "${CELL_LOG}" 2>&1
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
        "${DURATION}" "${RESULT_SHA}" "${RESULT_JSON}" "${CAMS2A_TRAIN_RECEIPT_SHA256}" "${AERIAL_GYM_SIM_NAME}" "${PYTHON}" "${GPU4GB:-}" <<'PY'
import json, os, sys
(out, cid, arm, bars, seed, games, commit, dirty, ckpt, nonce,
 started, finished, duration, result_sha, result_path, train_receipt_sha, sim_name, python, gpu4gb) = sys.argv[1:20]
payload = json.load(open(result_path))
cond = payload.get("condition", {})
env_snapshot = {k: v for k, v in os.environ.items() if k.startswith("NAVRL_")}
json.dump({
    "cell_id": cid, "arm": arm, "density_bars": int(bars), "seed": int(seed),
    "requested_episodes": int(games),
    "actual_episodes": payload.get("actual_episodes"),
    "git_commit": commit, "git_dirty_files": int(dirty),
    "checkpoint_sha256": ckpt, "evaluation_nonce": nonce,
    "train_receipt_sha256": train_receipt_sha,                                  # [cams2a] 3
    "sim_name": sim_name, "python": python, "gpu4gb": gpu4gb,                 # [cams2a] review r1 P2-2
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
