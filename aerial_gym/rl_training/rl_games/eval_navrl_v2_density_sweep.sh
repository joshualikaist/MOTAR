#!/usr/bin/env bash
# Held-out density sweep for a TASK-V2 (search arena) checkpoint.
#
# v2 differs from v1 in the TASK, not the observation width (both 898-D), so a v2 checkpoint
# loads without error in the v1 arena and would be scored on a completely different problem.
# This script pins the whole v2 contract and refuses to run against a checkpoint whose
# recorded arena provenance disagrees.
#
# ALWAYS pass last_gen_ppo_ep_* -- gen_ppo.pth is the best-reward (low-density) policy.
#
# Usage:
#   ./eval_navrl_v2_density_sweep.sh runs/ppo_XXXX_v2/nn/last_gen_ppo_ep_9000.pth
#   ./eval_navrl_v2_density_sweep.sh <ckpt> 2049                 # episodes per cell
#   NUM_ENVS=64 GPU4GB=1 ./eval_navrl_v2_density_sweep.sh <ckpt> # 4 GB machine (1650 Ti)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PATH="$(dirname "${PYTHON}"):${PATH}"
export PYTHONNOUSERSITE=1

# ---- v2 ARENA / TASK contract (must match train_navrl_v2_search.sh exactly) ----
export NAVRL_ARENA_XY=40
export NAVRL_ARENA_Z=3
export NAVRL_BAR_POOL=bars_h3
export NAVRL_PLACEMENT_MODE=navrl_band
export NAVRL_PLACEMENT_TOUCH_M=0.4
export NAVRL_PLACEMENT_GAP_M=1.6
export NAVRL_EPISODE_LEN_STEPS=600
export NAVRL_MAX_BARS="${NAVRL_MAX_BARS:-300}"
export NAVRL_BAR_X_MIN=0.0
export NAVRL_BAR_X_MAX=1.0
export NAVRL_GENERAL_GOAL_DIST_MIN=6
export NAVRL_GENERAL_GOAL_DIST_MAX=28
export NAVRL_K_FINAL=28
export NAVRL_K_MIN_FINAL=20

# ---- representation contract (unchanged from v1: same policy family) ----
export NAVRL_VISION=1
export NAVRL_PERCEPTION="${NAVRL_PERCEPTION:-1}"
export NAVRL_GENERAL_TRAIN=1
export NAVRL_TILT_COMP=1
export NAVRL_LIDAR_RANGE=12
export NAVRL_LIDAR_HBEAMS=72
export NAVRL_LIDAR_VBEAMS=4
export NAVRL_MAX_OBSTACLES=8
export NAVRL_OBSTACLE_SELECTOR="${NAVRL_OBSTACLE_SELECTOR:-cluster_sector}"
export NAVRL_OBSTACLE_CLUSTER_GAP_M="${NAVRL_OBSTACLE_CLUSTER_GAP_M:-0.45}"
export NAVRL_OBSTACLE_SECTORS="${NAVRL_OBSTACLE_SECTORS:-8}"
export NAVRL_OBSTACLE_SUPPRESS_DEG=10
export NAVRL_OBSTACLE_FOV_DEG=240
export NAVRL_CORRIDOR_TOKENS="${NAVRL_CORRIDOR_TOKENS:-0}"
export NAVRL_MAX_VELOCITY=2.5
export NAVRL_ALT_HOLD_VMAX=2.5
export NAVRL_YAW_RATE_MAX=3.0

CKPT="${1:?usage: $0 <last_gen_ppo_ep_XXXX.pth> [games_per_cell]}"
GAMES="${2:-2049}"
# Same per-area density schedule as the v1 sweep, over the full 1600 m^2 v2 arena:
# 70/150/210/280 bars = 4.4 / 9.4 / 13.1 / 17.5 per 100 m^2.
DENSITIES="${NAVRL_V2_DENSITIES:-70 150 210 280}"

# ---- provenance gate: refuse a checkpoint that was not trained in this arena ----
# NAVRL_V2_FORCE is honored INSIDE the gate: it used to be checked only after the Python block had
# already exited 2 under `set -e`, so the documented override never worked.
NAVRL_V2_FORCE="${NAVRL_V2_FORCE:-0}" "${PYTHON}" - "${CKPT}" <<'PY'
import os
import sys
import torch

force = os.environ.get("NAVRL_V2_FORCE", "0") == "1"
ckpt = torch.load(sys.argv[1], map_location="cpu", weights_only=False)
state = ckpt.get("env_state") or {}
# Every field the environment is actually built from. Anything checked by the preflight contract
# but not here would let a silently different task through the evaluator.
want = {
    "cfg_arena_xy": 40.0,
    "cfg_arena_z": 3.0,
    "cfg_bar_pool": "bars_h3",
    "cfg_placement_mode": "navrl_band",
    "cfg_placement_gap_m": 1.6,
    "cfg_placement_touch_m": 0.4,
    "cfg_episode_len_steps": 600.0,
    "cfg_bar_x_min": 0.0,
    "cfg_bar_x_max": 1.0,
}
verb = "WARNING (forced)" if force else "REFUSING"
missing = [k for k in want if state.get(k) is None]
if missing:
    print(
        "[eval_v2] %s: checkpoint has no v2 arena provenance (%s).\n"
        "          It was trained before arena provenance existed, or in the v1 arena.\n"
        "          Re-check the run, or set NAVRL_V2_FORCE=1 to override deliberately."
        % (verb, ", ".join(sorted(missing))),
        file=sys.stderr,
    )
    if not force:
        sys.exit(2)
bad = []
for key, expected in want.items():
    got = state.get(key)
    if got is None:
        continue
    ok = (
        str(got).strip() == expected
        if isinstance(expected, str)
        else abs(float(got) - expected) <= 1e-6
    )
    if not ok:
        bad.append(f"{key}: checkpoint={got} expected={expected}")
if bad:
    print("[eval_v2] %s: arena contract mismatch:\n  " % verb + "\n  ".join(bad), file=sys.stderr)
    if not force:
        sys.exit(2)
if missing or bad:
    sys.exit(0)
print(
    "[eval_v2] arena provenance OK | %.0fx%.0f m, pool=%s, placement=%s, %.0f steps"
    % (
        state["cfg_arena_xy"],
        state["cfg_arena_xy"],
        state["cfg_bar_pool"],
        state["cfg_placement_mode"],
        state["cfg_episode_len_steps"],
    )
)
PY

echo "[eval_v2] arena=${NAVRL_ARENA_XY}m pool=${NAVRL_BAR_POOL} placement=${NAVRL_PLACEMENT_MODE} \
episode=${NAVRL_EPISODE_LEN_STEPS} goal=${NAVRL_GENERAL_GOAL_DIST_MIN}..${NAVRL_GENERAL_GOAL_DIST_MAX}m"
echo "[eval_v2] lidar=${NAVRL_LIDAR_RANGE}m scan=${NAVRL_LIDAR_VBEAMS}x${NAVRL_LIDAR_HBEAMS} \
tokens=${NAVRL_MAX_OBSTACLES} selector=${NAVRL_OBSTACLE_SELECTOR} corridor=${NAVRL_CORRIDOR_TOKENS}"
echo "[eval_v2] ckpt=${CKPT} games/cell=${GAMES} densities=${DENSITIES}"

mkdir -p train_session_logs
for N in ${DENSITIES}; do
    echo "======== v2 density ${N} bars ($(python3 -c "print(f'{${N}/1600*100:.1f}')")/100m2) ========"
    NAVRL_NUM_BARS="${N}" NUM_ENVS="${NUM_ENVS:-128}" HEADLESS=True PLAY_GAMES_NUM="${GAMES}" \
        ./play_navrl.sh "${CKPT}" 2>&1 \
        | tee "train_session_logs/eval_v2_${N}bars_$(date +%y%m%d_%H%M).log"
done
echo "[eval_v2] done. logs in train_session_logs/eval_v2_*bars_*.log"
