#!/usr/bin/env bash
# CAM-S3-E (docs/plans/prereg_camonly_s3w_eval_2026-11-27.md, stage 3 of the CAM-REF runner): a copy of
# eval_navrl_camref_arms.sh (sha256 51f949d0..., pinned for stages 1 and 2 and left unchanged) whose only change, marked
# [cams3e], is that NAVRL_CAMREF_POLICY may also be S3W (CAM-S3 S3-W, which tools/camref_entry.py runs through
# tools/realveh_entry.py like S2B). Everything below is the stage-1 copy.
# CAM-REF (draft ~/motar_tmp/camref_20261126/draft/prereg_camonly_ref_matched_protocol_2026-11-26.md): a copy of CAM-S2a-E's
# launcher eval_navrl_cams2a_arms.sh (sha256 0ad30292...; itself a descendant of the pinned
# eval_navrl_target_behavior_arms.sh) with these changes, all marked [camref]:
#   1. the caller exports the WHOLE task environment of the cell (tools/camref_eval_run.py: Policy F's and CAM-K1's from
#      GR-0's KROC2027 contract, CAM-S2b-FT's from its training receipt via CAM-DX1's cell construction, plus the rung's
#      keys). The launcher requires only the keys every policy's contract defines, NAVRL_CAMREF_POLICY (F | K1 | S2B) and
#      the pinned interpreter / simulator; the cams2 / camsafe / no_lidar requirements of the CAM-S2a copy are gone
#      (Policy F has the LiDAR; S2b-FT's S6 rung runs with cams2 off). camref_entry.py validates the policy's stack;
#   2. CAMREF_CKPT / CAMREF_CKPT_SHA name the checkpoint (a last_gen_ppo_ep_*.pth, never gen_ppo.pth);
#   3. NAVRL_CAMREF_POLICY is recorded in every cell receipt (camref_policy);
#   4. per cell: NAVRL_CAMREF_RECEIPT; NAVRL_REALVEH_RECEIPT for the policies that run through realveh_entry (K1, S2B);
#      NAVRL_CAMREF_RECORD_JSONL when NAVRL_CAMREF_RECORD=1; NAVRL_CAMS2DIAG_JSONL when NAVRL_CAMS2DIAG=1;
#   5. CAMEVAL_PY is tools/camref_python (exported interpreter CAMREF_REAL_PYTHON).
# Everything else (checkpoint SHA check, evaluation mode, cell loop, receipt writer, resume rule) is the CAM-S2a copy.
# This script evaluates a frozen checkpoint. It never trains.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/../../.." && pwd)"
cd "${HERE}"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PYTHONNOUSERSITE=1
: "${CAMEVAL_PY:?CAMEVAL_PY (absolute path of tools/camref_python) is required}"
[[ "${CAMEVAL_PY}" == /* && -x "${CAMEVAL_PY}" ]] || { echo "[camref] CAMEVAL_PY must be an absolute executable path" >&2; exit 2; }
[[ "${PYTHON}" == /* ]] || { echo "[camref] PYTHON must be an absolute path" >&2; exit 2; }
export CAMREF_REAL_PYTHON="${PYTHON}"                                                          # [camref] 5

# [camref] 1: the keys every policy's contract defines must be present (tools/camref_eval_run.py exports the cell)
for K in NAVRL_CAMREF_POLICY NAVRL_VISION NAVRL_PERCEPTION NAVRL_SPEED_GOVERNOR NAVRL_SPEED_GOVERNOR_FREE_MPS NAVRL_MAX_VELOCITY \
         NAVRL_YAW_RATE_MAX NAVRL_TARGET_DYNAMICS NAVRL_TARGET_PATTERN NAVRL_PLACEMENT_MODE NAVRL_EPISODE_LEN_STEPS \
         NAVRL_GENERAL_TRAIN NAVRL_GENERAL_GOAL_DIST_MIN NAVRL_GENERAL_GOAL_DIST_MAX NAVRL_LIDAR_HBEAMS NAVRL_ARENA_XY \
         NAVRL_EPISODE_LEDGER NAVRL_EPISODE_QUOTA_PER_ENV NAVRL_PLAYER_GAMES_CAP AERIAL_GYM_SIM_NAME PYTHON; do
    [[ -n "${!K:-}" ]] || { echo "[camref] ${K} is required (the cell environment is exported by tools/camref_eval_run.py)" >&2; exit 2; }
done
[[ "${NAVRL_CAMREF_POLICY}" == "F" || "${NAVRL_CAMREF_POLICY}" == "K1" || "${NAVRL_CAMREF_POLICY}" == "S2B" \
   || "${NAVRL_CAMREF_POLICY}" == "S3W" ]] \
    || { echo "[camref] NAVRL_CAMREF_POLICY must be F, K1, S2B or S3W" >&2; exit 2; }                  # [cams3e]
[[ "${NAVRL_VISION}" == "1" && "${NAVRL_PERCEPTION}" == "1" ]] || { echo "[camref] NAVRL_VISION=1 and NAVRL_PERCEPTION=1 are required" >&2; exit 2; }
[[ "${NAVRL_CAMREF_RECORD:-}" == "1" || -z "${NAVRL_CAMREF_RECORD:-}" ]] || { echo "[camref] NAVRL_CAMREF_RECORD must be 1 or unset" >&2; exit 2; }
[[ "${NAVRL_CAMS2DIAG:-}" == "1" || -z "${NAVRL_CAMS2DIAG:-}" ]] || { echo "[camref] NAVRL_CAMS2DIAG must be 1 or unset" >&2; exit 2; }
# review r1 P2-2 of CAM-S2a-E: the simulator and interpreter are pinned by the caller, never inherited from a shell
[[ "${AERIAL_GYM_SIM_NAME}" == "base_sim" && "${PYTHON}" == "/home/fair/miniconda3/envs/aerialgym/bin/python" && -z "${GPU4GB:-}" ]] \
    || { echo "[camref] AERIAL_GYM_SIM_NAME must be base_sim, PYTHON the aerialgym interpreter, GPU4GB unset" >&2; exit 2; }
[[ "${NAVRL_DENSITY_CURRICULUM:-}" == "0" ]] || { echo "[camref] NAVRL_DENSITY_CURRICULUM must be 0 (fixed density)" >&2; exit 2; }
[[ -z "${NAVRL_CAMREF_RECEIPT:-}" && -z "${NAVRL_REALVEH_RECEIPT:-}" && -z "${NAVRL_CAMTRAIN_RECEIPT:-}" && -z "${NAVRL_CAMREF_RECORD_JSONL:-}" \
   && -z "${NAVRL_CAMS2DIAG_JSONL:-}" && -z "${NAVRL_SEED:-}" && -z "${NAVRL_NUM_BARS:-}" ]] \
    || { echo "[camref] the receipts, recorder paths, NAVRL_SEED and NAVRL_NUM_BARS are set per cell here" >&2; exit 2; }

# [camref] 2: the checkpoint and its expected SHA-256 come from the caller; gen_ppo.pth is refused
CKPT="${CAMREF_CKPT:?CAMREF_CKPT is required}"
EXPECT_SHA="${CAMREF_CKPT_SHA:?CAMREF_CKPT_SHA is required}"
[[ "$(basename "${CKPT}")" == last_gen_ppo_ep_*.pth ]] || { echo "[camref] the checkpoint must be a last_gen_ppo_ep_*.pth, got ${CKPT}" >&2; exit 2; }

ARMS_TEXT="${ARMS:?ARMS is required}"
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

# ================= evaluation mode only; the task contract is the caller's (the cell environment) ==========
export TASK=navrl_task
export FILE=ppo_navrl_perception_transformer.yaml
export AERIAL_GYM_SIM_NAME="${AERIAL_GYM_SIM_NAME}"
export NAVRL_BULK_EVAL=1
export NAVRL_EVAL_CHECKPOINT="${EXPECT_SHA}"
export HEADLESS=True
# =========================================================================

mkdir -p "${OUT_ROOT}"
echo "[arms] commit=${GIT_COMMIT} dirty=${GIT_DIRTY} ckpt=${ACTUAL_SHA:0:16}... policy=${NAVRL_CAMREF_POLICY}"
echo "[arms] arms=${ARMS_TEXT} (label only; target ${NAVRL_TARGET_DYNAMICS}/${NAVRL_TARGET_PATTERN})"
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
      export NAVRL_CAMREF_RECEIPT="${CELL_DIR}/camref_receipt.json"            # [camref] 4
      if [[ "${NAVRL_CAMREF_POLICY}" != "F" ]]; then
        export NAVRL_REALVEH_RECEIPT="${CELL_DIR}/realveh_receipt.json"        # [camref] 4 (K1, S2B: realveh_entry)
      fi
      if [[ "${NAVRL_CAMREF_RECORD:-}" == "1" ]]; then
        export NAVRL_CAMREF_RECORD_JSONL="${CELL_DIR}/camref_episodes.jsonl"   # [camref] 4
      fi
      if [[ "${NAVRL_CAMS2DIAG:-}" == "1" ]]; then
        export NAVRL_CAMS2DIAG_JSONL="${CELL_DIR}/cams2diag_episodes.jsonl"     # [camref] 4
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
        "${DURATION}" "${RESULT_SHA}" "${RESULT_JSON}" "${NAVRL_CAMREF_POLICY}" "${AERIAL_GYM_SIM_NAME}" "${PYTHON}" "${GPU4GB:-}" <<'PY'
import json, os, sys
(out, cid, arm, bars, seed, games, commit, dirty, ckpt, nonce,
 started, finished, duration, result_sha, result_path, camref_policy, sim_name, python, gpu4gb) = sys.argv[1:20]
payload = json.load(open(result_path))
cond = payload.get("condition", {})
env_snapshot = {k: v for k, v in os.environ.items() if k.startswith("NAVRL_")}
json.dump({
    "cell_id": cid, "arm": arm, "density_bars": int(bars), "seed": int(seed),
    "requested_episodes": int(games),
    "actual_episodes": payload.get("actual_episodes"),
    "git_commit": commit, "git_dirty_files": int(dirty),
    "checkpoint_sha256": ckpt, "evaluation_nonce": nonce,
    "camref_policy": camref_policy,                                            # [camref] 3
    "sim_name": sim_name, "python": python, "gpu4gb": gpu4gb,                 # CAM-S2a-E review r1 P2-2
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
