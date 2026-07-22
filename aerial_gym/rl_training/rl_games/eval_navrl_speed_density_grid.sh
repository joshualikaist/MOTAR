#!/usr/bin/env bash
# Evaluate whether low target/pursuer speed removes high-density crashes.
# Usage: NAVRL_VISION=1 NAVRL_PERCEPTION=1 ./eval_navrl_speed_density_grid.sh CKPT
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

CKPT="${1:?usage: $0 CHECKPOINT}"
GAMES="${PLAY_GAMES_NUM:-1000}"
DENSITIES="${DENSITIES:-110 130 150}"
TARGET_SPEEDS="${TARGET_SPEEDS:-0.0 0.5}"
PURSUER_SPEEDS="${PURSUER_SPEEDS:-0.75 1.0 1.5 2.0}"
EPISODE_LEN_STEPS="${EPISODE_LEN_STEPS:-300}"

export NAVRL_VISION=1
export NAVRL_PERCEPTION=1
export NAVRL_TARGET_PATTERN="${NAVRL_TARGET_PATTERN:-cv}"
export NAVRL_MAX_BARS="${NAVRL_MAX_BARS:-150}"

for density in ${DENSITIES}; do
  for target_speed in ${TARGET_SPEEDS}; do
    for pursuer_speed in ${PURSUER_SPEEDS}; do
      echo "[speed-density] bars=${density} target=${target_speed} pursuer_limit=${pursuer_speed}"
      NAVRL_NUM_BARS="${density}" NAVRL_TARGET_SPEED="${target_speed}" \
      NAVRL_MAX_VELOCITY="${pursuer_speed}" NAVRL_EPISODE_LEN_STEPS="${EPISODE_LEN_STEPS}" \
      PLAY_GAMES_NUM="${GAMES}" \
        ./play_navrl.sh "${CKPT}"
    done
  done
done
