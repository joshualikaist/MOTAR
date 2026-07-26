#!/usr/bin/env bash
# Candidate #4: fix the OBSTACLE REPRESENTATION -- the measured cause of the bar_contact floor.
#
# Three changes, applied together because the bar_contact probe showed they are one problem:
#   1. NAVRL_MAX_OBSTACLES 5 -> 8        H1: 15.8 bars sat inside the horizon against a capacity of
#                                        5, and 35% of contact deaths were with a bar that was
#                                        never in the policy's input at all.
#   2. suppression +-20 -> +-10 deg      H1: each token blanks a wedge around itself, so at +-20 deg
#                                        no more than ~7 tokens could EVER fill -- raising the
#                                        capacity alone would have wasted the extra slots.
#   3. NAVRL_LIDAR_HBEAMS 36 -> 72       H2: token positions come from range/angle geometry, so
#                                        their lateral error is ~half a bin (~0.44 m at 5 m with
#                                        10 deg bins). The probe measured 0.57 m, matching. 5 deg
#                                        bins halve it.
#
# All three change the observation layout (574 -> 898), so this is a FRESH run: no warm-start is
# possible from any existing checkpoint. Everything else matches train_navrl_general_12m_lookahead.sh
# (the current best recipe, capture 0.856), so the representation is the only changed variable.
#
# Usage:
#   ./train_navrl_general_repr.sh
#   MAX_EPOCHS=8000 NUM_ENVS=128 ./train_navrl_general_repr.sh
#
# Watch (crashdiag prints every ~2048 episodes):
#   NavRL barprobe hit_in_tokens  -- was 0.647; H1 is fixed if this climbs toward ~0.9
#   NavRL barprobe token_err      -- was 0.57 m; H2 is fixed if this drops toward ~0.3 m
#   NavRL crashdiag bar_contact   -- the actual goal: was ~13% of all episodes
# If hit_in_tokens rises but bar_contact does not fall, the limit is control/planning, not sensing.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PATH="$(dirname "${PYTHON}"):${PATH}"
export PYTHONNOUSERSITE=1

export NAVRL_VISION=1
export NAVRL_PERCEPTION=1
export NAVRL_GENERAL_TRAIN=1
export NAVRL_PERCEPTION_PERTURB=0
export NAVRL_TILT_COMP=1

# --- the three representation changes (the ONLY difference from the 12 m recipe) ---
export NAVRL_MAX_OBSTACLES="${NAVRL_MAX_OBSTACLES:-8}"
export NAVRL_OBSTACLE_SUPPRESS_DEG="${NAVRL_OBSTACLE_SUPPRESS_DEG:-10}"
export NAVRL_LIDAR_HBEAMS="${NAVRL_LIDAR_HBEAMS:-72}"

export NAVRL_LIDAR_RANGE=12
export NAVRL_MAX_BARS=150
export NAVRL_NUM_BARS=25
export NAVRL_DENSITY_CURRICULUM=0
export NAVRL_K_COMPETENCE=1
export NAVRL_K_FINAL=16
export NAVRL_K_MIN_FINAL=10
export NAVRL_FOV_CURRICULUM_EPOCHS=1000000
export NAVRL_MAX_VELOCITY=2.5
export NAVRL_ALT_HOLD_VMAX=2.5
export NAVRL_YAW_RATE_MAX=3.0
export NAVRL_TARGET_SPEED_MIN=0.0
export NAVRL_TARGET_SPEED_FINAL=1.5
export NAVRL_TARGET_PATTERN=mixed
unset NAVRL_TARGET_SPEED

export NAVRL_OOB_MARGIN=1.0
export NAVRL_CRASH_DIAG=1
export NAVRL_BAR_PROBE=1   # so the token/crowding forensics are logged throughout training
export NAVRL_OOB_PROBE=0

MAX_EPOCHS="${MAX_EPOCHS:-8000}"
NUM_ENVS="${NUM_ENVS:-128}"
SEED="${SEED:-1}"

echo "[general_repr] FRESH run (obs 574 -> 898; no warm-start possible)"
echo "[general_repr] tokens=${NAVRL_MAX_OBSTACLES} suppress=${NAVRL_OBSTACLE_SUPPRESS_DEG}deg \
hbeams=${NAVRL_LIDAR_HBEAMS} lidar=${NAVRL_LIDAR_RANGE}m"
echo "[general_repr] max_epochs=${MAX_EPOCHS} envs=${NUM_ENVS} seed=${SEED}"

exec env NUM_ENVS="${NUM_ENVS}" ./train_navrl.sh --max_epochs "${MAX_EPOCHS}" --seed "${SEED}"
