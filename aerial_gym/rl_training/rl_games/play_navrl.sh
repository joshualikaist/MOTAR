#!/usr/bin/env bash
# === NavRL Phase-1 재생 / 평가 (짧은 실행 래퍼) ===
#   뷰어(16 envs):     ./play_navrl.sh runs/ppo_XXXX_navrl/nn/gen_ppo.pth
#   대량 통계(창 없음): NUM_ENVS=512 HEADLESS=True PLAY_GAMES_NUM=8000 \
#                        ./play_navrl.sh runs/ppo_XXXX_navrl/nn/gen_ppo.pth
#   다른 파이썬:        PYTHON=/path/to/python ./play_navrl.sh CKPT
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
export PYTHONNOUSERSITE=1

PY="${PYTHON:-python}"
FILE="${FILE:-ppo_navrl_cnn.yaml}"     # CNN 체크포인트는 반드시 CNN yaml 로 (state_dict 불일치 방지)
TASK="${TASK:-navrl_task}"
NUM_ENVS="${NUM_ENVS:-16}"             # 뷰어는 적게, 통계는 크게
HEADLESS="${HEADLESS:-False}"

CKPT="${1:-}"
if [[ -z "${CKPT}" ]]; then
  echo "usage: ./play_navrl.sh <checkpoint.pth>   (예: runs/ppo_260714_1904_navrl/nn/gen_ppo.pth)"
  exit 1
fi

echo "[play_navrl] py=${PY} envs=${NUM_ENVS} headless=${HEADLESS} ckpt=${CKPT}"
"${PY}" runner.py --file "${FILE}" --task "${TASK}" \
    --num_envs "${NUM_ENVS}" --headless "${HEADLESS}" --play --checkpoint "${CKPT}"
