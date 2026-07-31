#!/usr/bin/env bash
# Task v2 "search arena": NavRL-scale environment + interception objective. FRESH training.
#
# Why v2 (WORKLOG 2026-07-31): in the v1 24 x 24 arena the goal (4..16 m) was almost always
# inside the sensor horizon (LiDAR 12 m, forward camera 20 m), so the task degenerated into
# visible-target pursuit -- no search. v2 matches the reference NavRL scale (40 x 40 map,
# ~22 obstacles/100 m^2; reference env.py map_range [20,20], 350 obstacles) and pushes the
# goal-distance curriculum past the camera horizon so the drone must FIND the target with
# LiDAR + a 87-degree forward camera before it can intercept it.
#
# v2 vs v1 deltas (each env-gated; defaults preserve v1 exactly):
#   arena          24x24x3        -> 40x40x3          (NAVRL_ARENA_XY=40)
#   bar pool       2.0 m bars     -> 3.0 m full height (NAVRL_BAR_POOL=bars_h3; no fly-over,
#                                     like NavRL's mostly 4-6 m obstacles in a 4.5 m map)
#   placement      random+relax   -> navrl_band       (slit-free: touching-or->=0.8 m-gap;
#                                     the legacy relax made ~2.2 impassable slits/layout at 150 bars)
#   goal distance  4..16 m        -> 6..28 m           (28 m > camera 20 m -> search regime;
#                                     competence curriculum ramps k_max 10->28 as before)
#   episode        300 steps/30 s -> 600 steps/60 s    (search + longer traversals need time;
#                                     NavRL uses 2200 x 0.016 s = 35 s for navigation alone)
#   density        25..150 bars   -> 70..300 bars      (4.4..18.8/100m^2 over the full 1600 m^2
#                                     arena; step 15 = v1's 5-bar step scaled)
#   bar x band     0.13..0.96     -> 0.0..1.0          (the legacy window kept v1's left-to-right
#                                     spawn strip clear; with uniform v2 spawns it only left 17%
#                                     of the arena obstacle-free and inflated reported density)
#   promote gate   flat 0.70      -> 0.85 @70 -> 0.70 @300 (linear in bar count: demand mastery
#                                     at the easy end, avoid a permanent stall at the hard end)
#   target speed   U[0,1.5] / 3000-> U[0.3,vmax] over 300 epochs (always moving; the short ramp
#                                     recovers early survival learning and finishes 700 epochs
#                                     before density evidence starts, so the two gates do not overlap)
# Deliberately UNCHANGED (variable control): observation contract 898-D, LiDAR 12 m 72x4,
# 8 cluster-sector tokens, camera 87 deg @20 m, v_max 2.5, yaw 3.0, mixed pattern,
# squashed-Gaussian action policy, PPO hyperparameters, seed.
#
# VRAM: measured 6314 MiB / 8192 at 128 envs, 40x40, 300 full-height bars (2026-07-31).
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PATH="$(dirname "${PYTHON}"):${PATH}"
export PYTHONNOUSERSITE=1

export MAX_EPOCHS="${MAX_EPOCHS:-30000}"
export SEED="${SEED:-1}"
export NUM_ENVS="${NUM_ENVS:-128}"
export AERIAL_RUN_TAG="${AERIAL_RUN_TAG:-v2-search-fresh-s${SEED}}"
export TRAIN_SESSION_LOG="${TRAIN_SESSION_LOG:-train_session_logs/v2_search_fresh_$(date +%y%m%d_%H%M%S).log}"

# ---- v2 environment ----
export NAVRL_ARENA_XY=40
export NAVRL_ARENA_Z=3
export NAVRL_BAR_POOL=bars_h3
export NAVRL_PLACEMENT_MODE=navrl_band
export NAVRL_PLACEMENT_TOUCH_M=0.4
export NAVRL_PLACEMENT_GAP_M=1.6
export NAVRL_EPISODE_LEN_STEPS=600
export NAVRL_MAX_BARS=300
# Full-width obstacle band. The legacy 0.13..0.96 window kept the v1 left-to-right spawn strip
# clear; v2 spawns drone AND target uniformly (NAVRL_GENERAL_TRAIN=1), so that window only left
# 17% of the arena permanently empty -- episodes spawning there were straight-line pursuit at any
# density -- and inflated reported density (1328 m^2 band vs 1600 m^2 flyable).
export NAVRL_BAR_X_MIN=0.0
export NAVRL_BAR_X_MAX=1.0

# ---- v2 objective: goal beyond the sensor horizon ----
export NAVRL_GENERAL_GOAL_DIST_MIN=6
export NAVRL_GENERAL_GOAL_DIST_MAX=28
export NAVRL_K_COMPETENCE=1
export NAVRL_K_FINAL=28
export NAVRL_K_MIN_FINAL=20

# ---- density curriculum, v1-equivalent per-area schedule ----
# Overridable so a fixed-density probe (train_navrl_v2_ceiling_probe.sh) can freeze the curriculum
# while inheriting the rest of this contract verbatim. Unset => the normal curriculum run.
export NAVRL_DENSITY_CURRICULUM="${NAVRL_DENSITY_CURRICULUM:-1}"
export NAVRL_DENSITY_START="${NAVRL_DENSITY_START:-70}"
export NAVRL_DENSITY_FINAL="${NAVRL_DENSITY_FINAL:-300}"
export NAVRL_DENSITY_STEP=15
# Per-density threshold ramp (2026-07-31): flat 0.70 risks the curriculum stalling forever if the
# achievable capture ceiling keeps falling with density (the same failure mode behind v1's 100-bar
# plateau). Require MORE capture to leave the easy end, LESS to leave the hard end. Unset either
# var to fall back to the flat NAVRL_DENSITY_THRESHOLD.
#
# START was 0.85 (a design guess) and got MEASURED at 70 bars on 2026-07-31 (run ppo_260731_1722,
# 2300 epochs): two full 16,384-episode gate windows scored 0.816 and 0.837, crash decayed to an
# extrapolated floor of 13.1% (fit tau~650 epochs, already converged), and with the ~2.6% timeout
# base that puts the achievable capture ceiling at ~0.843 -- structurally BELOW 0.85. Root cause is
# representation, not geometry: 23.6% of crashed bars were outside the 240-degree token window and
# 11.7% of in-window hits had no token (capacity 8 vs ~12 bars in FOV). 0.80 sits under the
# measured ceiling with margin while still demanding near-ceiling mastery before promotion.
export NAVRL_DENSITY_THRESHOLD_START="${NAVRL_DENSITY_THRESHOLD_START:-0.80}"
export NAVRL_DENSITY_THRESHOLD_END="${NAVRL_DENSITY_THRESHOLD_END:-0.70}"
# Explicit per-density gate (overrides the ramp above). The achievable ceiling is not linear in
# density, so a two-point ramp cannot express it: 70 bars measured 0.843, and the early levels
# should still demand near-ceiling mastery while the dense end settles at the 0.70 floor. Every
# density the curriculum can occupy (70, 85, 100, 115, ...) is a knot, and the schedule holds the
# last knot beyond 115.
export NAVRL_DENSITY_THRESHOLD_SCHEDULE="${NAVRL_DENSITY_THRESHOLD_SCHEDULE:-70:0.82,85:0.77,100:0.72,115:0.70}"
export NAVRL_DENSITY_WARMUP="${NAVRL_DENSITY_WARMUP:-1000}"
export NAVRL_DENSITY_CHECK_EPS="${NAVRL_DENSITY_CHECK_EPS:-16384}"
# Dwell at each density for at least this many epochs before promoting, even when the capture gate
# already passes. Without it the curriculum chains promotions as fast as evidence windows fill, so
# no level ever converges and every metric only ever tracks rising difficulty.
export NAVRL_DENSITY_MIN_EPOCHS="${NAVRL_DENSITY_MIN_EPOCHS:-1000}"
# NAVRL_NUM_BARS pins the active density; clearing it is what lets the curriculum own the value.
# A fixed-density probe sets it deliberately, so only clear it when the curriculum is actually on --
# otherwise the probe's density silently reverts to the config default.
if [[ "${NAVRL_DENSITY_CURRICULUM}" == "1" ]]; then
    unset NAVRL_NUM_BARS
fi
unset NAVRL_FIXED_BARS NAVRL_CONTROLLED_ABLATION

# ---- UNCHANGED sensor/representation/action contract (variable control) ----
export NAVRL_VISION=1
export NAVRL_PERCEPTION=1
export NAVRL_GENERAL_TRAIN=1
export NAVRL_PERCEPTION_PERTURB=0
export NAVRL_TILT_COMP=1
export NAVRL_MAX_OBSTACLES=8
export NAVRL_OBSTACLE_FOV_DEG=240
# Overridable so a representation A/B (train_navrl_v2_ttc_ab.sh) can swap the selector while
# inheriting the rest of this contract. Hardcoding it silently discarded the experimental variable
# and made both arms run the SAME condition -- an A/B failure that is invisible because both arms
# still train normally. Unset => cluster_sector, the v2 default.
export NAVRL_OBSTACLE_SELECTOR="${NAVRL_OBSTACLE_SELECTOR:-cluster_sector}"
export NAVRL_OBSTACLE_CLUSTER_GAP_M=0.45
export NAVRL_OBSTACLE_SECTORS=8
export NAVRL_OBSTACLE_SUPPRESS_DEG=10
export NAVRL_CORRIDOR_TOKENS=0
export NAVRL_LIDAR_HBEAMS=72
export NAVRL_LIDAR_VBEAMS=4
export NAVRL_LIDAR_RANGE=12
export NAVRL_MAX_VELOCITY=2.5
export NAVRL_ALT_HOLD_VMAX=2.5
export NAVRL_YAW_RATE_MAX=3.0
# Target speed: always moving at >=0.3 m/s, with the upper support increasing to 1.5 m/s over a
# short 300-epoch ramp. The measured no-ramp v3 pilot learned, but reached 10-epoch rolling capture
# 0.50 at epoch 140 versus 53 for the old ramped run; the comparison is confounded by the simultaneous
# full-width bar-band change, so keep only a short scaffold. It ends at epoch 300, well before density
# evidence starts at epoch 1000, avoiding the old 3000-epoch overlap with the density curriculum.
export NAVRL_TARGET_SPEED_MIN=0.3
export NAVRL_TARGET_SPEED_FINAL=1.5
export NAVRL_TARGET_SPEED_RAMP_EPOCHS="${NAVRL_TARGET_SPEED_RAMP_EPOCHS:-300}"
export NAVRL_TARGET_PATTERN=mixed
unset NAVRL_TARGET_SPEED
export NAVRL_ACTION_POLICY=squashed_gaussian
export NAVRL_ACTION_STD=0.35,0.35,0.05,0.08
export NAVRL_ACTION_MU_SCALE=1.0,0.4,1.0,1.0
export NAVRL_ENTROPY_COEF=0.0
export NAVRL_PPO_LOG_RATIO_CLAMP=10.0
export NAVRL_PPO_KL_STOP=0.04
export NAVRL_LATENT_MARGIN_Y=1.25
export NAVRL_OOB_MARGIN=1.0
export NAVRL_CRASH_DIAG=1
export NAVRL_BAR_PROBE=1

mkdir -p train_session_logs
if [[ "${ALLOW_CONCURRENT:-0}" != "1" ]]; then
    ACTIVE_PIDS="$(pgrep -f '[r]unner.py .*--task navrl_task .*--train' | tr '\n' ' ' || true)"
    if [[ -n "${ACTIVE_PIDS// }" ]]; then
        echo "[v2-search] refusing duplicate NavRL training; active PID(s): ${ACTIVE_PIDS}" >&2
        exit 3
    fi
fi

echo "[v2-search] FRESH | arena=${NAVRL_ARENA_XY}m pool=${NAVRL_BAR_POOL} placement=${NAVRL_PLACEMENT_MODE}"
echo "[v2-search] goal ${NAVRL_GENERAL_GOAL_DIST_MIN}..${NAVRL_GENERAL_GOAL_DIST_MAX}m (camera 20m -> search) episode=${NAVRL_EPISODE_LEN_STEPS} steps"
echo "[v2-search] density ${NAVRL_DENSITY_START}->${NAVRL_DENSITY_FINAL} bars step=${NAVRL_DENSITY_STEP} (4.4->18.8 /100m2 over 1600m2) threshold ${NAVRL_DENSITY_THRESHOLD_SCHEDULE:-${NAVRL_DENSITY_THRESHOLD_START}->${NAVRL_DENSITY_THRESHOLD_END}}"
echo "[v2-search] target speed U[${NAVRL_TARGET_SPEED_MIN}, vmax] m/s, vmax->${NAVRL_TARGET_SPEED_FINAL} by epoch ${NAVRL_TARGET_SPEED_RAMP_EPOCHS} | bar band x=[${NAVRL_BAR_X_MIN}, ${NAVRL_BAR_X_MAX}]"
exec ./train_navrl.sh --seed "${SEED}" --max_epochs "${MAX_EPOCHS}" --disable_collapse_early_stop "$@"
