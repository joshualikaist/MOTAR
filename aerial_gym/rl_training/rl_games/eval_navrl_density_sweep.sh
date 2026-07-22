#!/usr/bin/env bash
# Held-out density sweep for a vision checkpoint (논문 밀도–성능 곡선).
# ALWAYS pass last_gen_ppo_ep_* — NOT gen_ppo.pth (best-reward = low-density policy).
#
# Usage:
#   ./eval_navrl_density_sweep.sh runs/ppo_XXXX_navrl/nn/last_gen_ppo_ep_9000.pth
#   ./eval_navrl_density_sweep.sh runs/ppo_XXXX_navrl/nn/last_gen_ppo_ep_9000.pth 2500
#
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
export PYTHONNOUSERSITE=1
export NAVRL_VISION=1
export NAVRL_K_FINAL=16
export NAVRL_K_MIN_FINAL=10

CKPT="${1:?usage: $0 <last_gen_ppo_ep_XXXX.pth> [games_per_cell]}"
GAMES="${2:-2500}"
DENSITIES=(25 50 75 110 130)

echo "[eval_sweep] ckpt=${CKPT} games/cell=${GAMES}"
for N in "${DENSITIES[@]}"; do
  echo "======== density ${N} bars ========"
  NAVRL_NUM_BARS="${N}" NUM_ENVS=512 HEADLESS=True PLAY_GAMES_NUM="${GAMES}" \
    ./play_navrl.sh "${CKPT}" 2>&1 | tee "train_session_logs/eval_${N}bars_$(date +%y%m%d_%H%M).log" \
    | grep -E "NavRL progress|captured|crash|timeout|games" || true
done
echo "[eval_sweep] done. logs in train_session_logs/eval_*bars_*.log"
