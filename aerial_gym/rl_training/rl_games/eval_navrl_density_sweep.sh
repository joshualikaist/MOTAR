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
# Perception (574-dim Transformer) is the current policy family. Without this, play_navrl.sh picks
# ppo_navrl_vision.yaml (1265-dim CNN) and the checkpoint cannot load at all.
export NAVRL_PERCEPTION="${NAVRL_PERCEPTION:-1}"
export NAVRL_K_FINAL=16
export NAVRL_K_MIN_FINAL=10

# SCALING-CRITICAL: these must match the values the checkpoint was TRAINED with. lidar range is both
# the sensor horizon and the observation divisor (scan/range); max_velocity and yaw_rate_max scale
# both observations and action limits. Leaving them unset silently reverts to the config defaults
# (4.0 / 2.0 / 2.5) and produces a wrong density curve that looks like a bad policy, not a bad eval.
# Defaults below match the current general-spawn recipe; override per checkpoint as needed.
export NAVRL_LIDAR_RANGE="${NAVRL_LIDAR_RANGE:-8}"
export NAVRL_MAX_VELOCITY="${NAVRL_MAX_VELOCITY:-2.5}"
export NAVRL_YAW_RATE_MAX="${NAVRL_YAW_RATE_MAX:-3.0}"
export NAVRL_TILT_COMP="${NAVRL_TILT_COMP:-1}"
# NOTE: spawns are the DEFAULT (non-general) distribution unless NAVRL_GENERAL_TRAIN=1 is exported.
# For a checkpoint trained with random spawns this is a held-out distribution shift -- intentional,
# but do not compare those numbers against general-spawn evals.
echo "[eval_sweep] lidar=${NAVRL_LIDAR_RANGE}m vmax=${NAVRL_MAX_VELOCITY} yaw=${NAVRL_YAW_RATE_MAX} \
general_spawn=${NAVRL_GENERAL_TRAIN:-0} (must match training; task warns on checkpoint mismatch)"

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
