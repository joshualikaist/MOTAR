#!/usr/bin/env bash
# Stage C: density curriculum from the validated general-spawn FOV-240 representation policy.
#
# This is deliberately separate from the older train_navrl_vision_seq_density.sh.  The old launcher
# selects a different observation schema and did not pin the representation settings, so using it
# with the 898-D checkpoint either fails shape loading or silently evaluates the wrong policy input.
#
# Usage:
#   ./train_navrl_general_repr_density.sh
#   CKPT=runs/.../nn/gen_ppo.pth MAX_EPOCHS=15000 ./train_navrl_general_repr_density.sh
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

# Policy-input contract of ppo_260727_0930. Keep all of these pinned together.
export NAVRL_MAX_OBSTACLES=8
export NAVRL_OBSTACLE_FOV_DEG=240
export NAVRL_OBSTACLE_SUPPRESS_DEG=10
export NAVRL_LIDAR_HBEAMS=72
export NAVRL_LIDAR_VBEAMS=4
export NAVRL_LIDAR_RANGE=12

export NAVRL_MAX_BARS=150
unset NAVRL_NUM_BARS  # an explicit value disables promotion even when the curriculum flag is on
export NAVRL_DENSITY_CURRICULUM=1
export NAVRL_DENSITY_START=25
export NAVRL_DENSITY_FINAL="${NAVRL_DENSITY_FINAL:-110}"
export NAVRL_DENSITY_STEP="${NAVRL_DENSITY_STEP:-5}"
export NAVRL_DENSITY_THRESHOLD="${NAVRL_DENSITY_THRESHOLD:-0.55}"
export NAVRL_DENSITY_WARMUP="${NAVRL_DENSITY_WARMUP:-1000}"
export NAVRL_DENSITY_CHECK_EPS="${NAVRL_DENSITY_CHECK_EPS:-4096}"

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
export NAVRL_BAR_PROBE=1
export NAVRL_OOB_PROBE=0

CKPT="${CKPT:-runs/ppo_260727_0930_navrl/nn/gen_ppo.pth}"
MAX_EPOCHS="${MAX_EPOCHS:-15000}"
NUM_ENVS="${NUM_ENVS:-128}"
SEED="${SEED:-1}"

if [[ ! -f "${CKPT}" ]]; then
    echo "checkpoint not found: ${CKPT}" >&2
    exit 1
fi

echo "[general_repr_density] Stage C | 25 -> ${NAVRL_DENSITY_FINAL} bars, step=${NAVRL_DENSITY_STEP}"
echo "[general_repr_density] representation | tokens=${NAVRL_MAX_OBSTACLES} \
fov=${NAVRL_OBSTACLE_FOV_DEG}deg suppress=+-${NAVRL_OBSTACLE_SUPPRESS_DEG}deg \
scan=${NAVRL_LIDAR_VBEAMS}x${NAVRL_LIDAR_HBEAMS} lidar=${NAVRL_LIDAR_RANGE}m"
echo "[general_repr_density] checkpoint=${CKPT} max_epochs=${MAX_EPOCHS} envs=${NUM_ENVS} seed=${SEED}"

exec env NUM_ENVS="${NUM_ENVS}" ./train_navrl.sh \
    --checkpoint "${CKPT}" --branch_run --max_epochs "${MAX_EPOCHS}" --seed "${SEED}" "$@"
