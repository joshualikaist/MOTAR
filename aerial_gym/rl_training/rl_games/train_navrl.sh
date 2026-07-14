#!/usr/bin/env bash
# === NavRL Phase-1 학습 (짧은 실행 래퍼) ===
# 기존의 긴 명령
#   PYTHONNOUSERSITE=1 python runner.py --file ppo_navrl_cnn.yaml --task navrl_task \
#       --num_envs 256 --headless True --use_warp True --train 2>&1 | tee train_....log
# 을 이 한 줄로:
#
#   처음부터:     ./train_navrl.sh
#   env 수 변경:  NUM_ENVS=512 ./train_navrl.sh          # RTX 3070 8GB 기본은 256
#   이어하기:     ./train_navrl.sh --checkpoint runs/ppo_XXXX_navrl/nn/gen_ppo.pth
#   MLP 베이스라인: FILE=ppo_navrl.yaml ./train_navrl.sh
#   다른 파이썬:  PYTHON=/path/to/conda/envs/aerialgym/bin/python ./train_navrl.sh
#
# 추가 인자(예: --checkpoint, --seed)는 그대로 runner.py 로 전달된다("$@").
set -euo pipefail

# runs/·nn/ 출력이 이 폴더에 생기도록 cwd 를 rl_games 로 고정 (1904 등 기존 run 위치와 동일).
cd "$(dirname "${BASH_SOURCE[0]}")"

# AirSim user-site numpy 가 conda numpy 를 가리는 문제 차단 (activate.d 가 이미 설정하지만 이중 안전).
export PYTHONNOUSERSITE=1

# 4 GB VRAM 보조 머신 프리셋(GPU4GB=1): 세 조각을 한 번에 켠다 —
#   (1) base_sim_4gb 로 PhysX GPU 버퍼 축소(navrl_task_config 가 AERIAL_GYM_SIM_NAME 을 읽음),
#   (2) minibatch=512 인 4gb yaml, (3) 낮은 NUM_ENVS. 이 셋이 다 있어야 4GB 에서 OOM 없이 학습된다.
# GPU4GB 를 세팅하지 않으면 8 GB 메인 머신 동작은 기존 그대로. (개별 변수로 오버라이드 가능.)
if [ "${GPU4GB:-0}" = "1" ]; then
    export AERIAL_GYM_SIM_NAME="${AERIAL_GYM_SIM_NAME:-base_sim_4gb}"
    FILE="${FILE:-ppo_navrl_cnn_4gb.yaml}"
    NUM_ENVS="${NUM_ENVS:-32}"
fi

PY="${PYTHON:-python}"                 # 활성화된 conda env 의 python (2번째 머신 이식용, 경로 하드코딩 X)
FILE="${FILE:-ppo_navrl_cnn.yaml}"     # NavRL LiDAR CNN (권장). MLP 는 ppo_navrl.yaml
TASK="${TASK:-navrl_task}"
NUM_ENVS="${NUM_ENVS:-256}"            # 256 envs + 48 bars ≈ 6.8GB (8GB 안전선). 4GB 는 GPU4GB=1 (→32)

mkdir -p runs train_session_logs
LOG="train_session_logs/train_$(date +%y%m%d_%H%M).log"
echo "[train_navrl] py=${PY} file=${FILE} task=${TASK} envs=${NUM_ENVS}"
echo "[train_navrl] log → ${LOG}"
echo "[train_navrl] TensorBoard: tensorboard --logdir $(pwd)/runs"

"${PY}" runner.py --file "${FILE}" --task "${TASK}" \
    --num_envs "${NUM_ENVS}" --headless True --use_warp True --train "$@" \
    2>&1 | tee "${LOG}"
