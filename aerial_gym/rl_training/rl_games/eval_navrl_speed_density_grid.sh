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
export NAVRL_GENERAL_TRAIN="${NAVRL_GENERAL_TRAIN:-1}"

# SCALING-CRITICAL: must match the checkpoint's training values (lidar range is also the scan
# normalizer). Unset -> config defaults (4.0 / 2.5) -> wrong curve that looks like a bad policy.
export NAVRL_LIDAR_RANGE="${NAVRL_LIDAR_RANGE:-12}"
export NAVRL_YAW_RATE_MAX="${NAVRL_YAW_RATE_MAX:-3.0}"
export NAVRL_TILT_COMP="${NAVRL_TILT_COMP:-1}"
export NAVRL_MAX_OBSTACLES="${NAVRL_MAX_OBSTACLES:-8}"
export NAVRL_LIDAR_HBEAMS="${NAVRL_LIDAR_HBEAMS:-72}"
export NAVRL_LIDAR_VBEAMS="${NAVRL_LIDAR_VBEAMS:-4}"
export NAVRL_OBSTACLE_SUPPRESS_DEG="${NAVRL_OBSTACLE_SUPPRESS_DEG:-10}"
export NAVRL_OBSTACLE_FOV_DEG="${NAVRL_OBSTACLE_FOV_DEG:-240}"
# The pursuer-speed axis below sweeps NAVRL_MAX_VELOCITY, which ALSO rescales the ego-velocity
# observation. Pin the altitude-hold authority so it does NOT shrink with the swept speed, otherwise
# "slow pursuer crashes less" is confounded with "slow pursuer cannot hold altitude".
export NAVRL_ALT_HOLD_VMAX="${NAVRL_ALT_HOLD_VMAX:-2.5}"

echo "[speed-density] representation scan=${NAVRL_LIDAR_VBEAMS}x${NAVRL_LIDAR_HBEAMS} \
tokens=${NAVRL_MAX_OBSTACLES} fov=${NAVRL_OBSTACLE_FOV_DEG} \
suppress=+-${NAVRL_OBSTACLE_SUPPRESS_DEG} lidar=${NAVRL_LIDAR_RANGE}m"

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
