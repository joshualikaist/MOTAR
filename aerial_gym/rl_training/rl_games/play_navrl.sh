#!/usr/bin/env bash
# === NavRL 재생 / 평가 (짧은 실행 래퍼) ===
#   뷰어(16 envs):     ./play_navrl.sh runs/ppo_XXXX_navrl/nn/gen_ppo.pth
#   대량 통계(창 없음): NUM_ENVS=512 HEADLESS=True PLAY_GAMES_NUM=8000 \
#                        ./play_navrl.sh runs/ppo_XXXX_navrl/nn/gen_ppo.pth
#   4GB 보조 머신:      GPU4GB=1 ./play_navrl.sh CKPT   (PhysX 버퍼 축소 sim 사용)
#   Phase 3 평가 셀:    NAVRL_NUM_BARS=110 NAVRL_TARGET_SPEED=1.0 NAVRL_TARGET_PATTERN=cv \
#                        NUM_ENVS=512 HEADLESS=True ./play_navrl.sh CKPT
#   다른 파이썬:        PYTHON=/path/to/python ./play_navrl.sh CKPT
#
# NOTE: --seed 는 rl_games 플레이어가 버리므로 평가 재현성은 NAVRL_SEED=<n> 으로 제어한다.
# NOTE: NAVRL_NUM_BARS 를 명시하면 체크포인트에 저장된 밀도 대신 그 값이 쓰인다(교차 평가용).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
export PYTHONNOUSERSITE=1

# 4 GB VRAM 보조 머신 프리셋: PhysX 버퍼를 줄인 sim 으로 빌드 (train_navrl.sh 와 동일)
if [ "${GPU4GB:-0}" = "1" ]; then
    export AERIAL_GYM_SIM_NAME="${AERIAL_GYM_SIM_NAME:-base_sim_4gb}"
fi

PY="${PYTHON:-python}"
FILE="${FILE:-ppo_navrl_cnn.yaml}"     # CNN 체크포인트는 반드시 CNN yaml 로 (state_dict 불일치 방지)
TASK="${TASK:-navrl_task}"
NUM_ENVS="${NUM_ENVS:-16}"             # 뷰어는 적게, 통계는 크게
HEADLESS="${HEADLESS:-False}"

CKPT="${1:-}"
if [[ -z "${CKPT}" ]]; then
  echo "usage: ./play_navrl.sh <checkpoint.pth> [extra runner args]"
  exit 1
fi
shift || true

echo "[play_navrl] py=${PY} envs=${NUM_ENVS} headless=${HEADLESS} ckpt=${CKPT}"
"${PY}" runner.py --file "${FILE}" --task "${TASK}" \
    --num_envs "${NUM_ENVS}" --headless "${HEADLESS}" --play --checkpoint "${CKPT}" "$@"
