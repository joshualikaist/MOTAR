#!/usr/bin/env bash
# TTC selector A/B -- both arms, one script, on the 4 GB card.
#
#   ARM=baseline ./train_navrl_v2_ttc_ab.sh     # cluster_sector (bearing-ranked tokens)
#   ARM=ttc      ./train_navrl_v2_ttc_ab.sh     # ttc_sector     (threat-ranked tokens)
#
# WHY ONE SCRIPT FOR BOTH ARMS
# ----------------------------
# Everything except NAVRL_OBSTACLE_SELECTOR is set below the ARM switch, so the two arms cannot
# drift apart. This project has already lost hours twice to a launcher that silently changed a
# second variable (a hardcoded NAVRL_CONTROLLED_ABLATION that blocked density promotion; an
# unconditional `unset NAVRL_NUM_BARS` that reverted a fixed-density probe), and an A/B is exactly
# where that class of bug is invisible: both arms still run, they just answer different questions.
#
# WHAT IS BEING TESTED
# --------------------
# cluster_sector allocates the 8 token slots by BEARING (one cluster per forward sector). The crash
# probe on run ppo_260731_1722 (2300 epochs, 70 bars) measured where that proxy breaks:
#   * 23.6% of the bars actually STRUCK were outside the 240-degree token window entirely
#     (hit_fov = 0.764) -- while searching, the drone yaws hard, so a bar leaves the window while
#     the velocity vector still points at it;
#   * of the hits inside the window, 11.7% still had no token (hit_token_given_fov = 0.883),
#     because a sector reserves only its single nearest cluster.
# ttc_sector ranks by closing time instead (range / closing speed), so a bar is tokenized because
# the drone is moving into it, not because of where it sits. Clustering is shared verbatim, so this
# isolates the RANKING, not the grouping. Observation stays 8x12 -> both arms warm-start from the
# same checkpoint with no surgery.
#
# HONEST PRIOR: representation changes have a poor track record here. Token capacity 5->8 was
# REJECTED (coverage fell 0.647 -> 0.40-0.53), beams 36->72 was REJECTED (token error rose
# 0.57 -> 0.72-1.13 m), and corridor tokens missed their gate (+1.57pp vs +3pp required). The one
# intervention that did work was narrowing the selection FOV 360->240, which is also a "what do we
# pick" change -- the same family as this one. Expect a small effect and measure it coldly.
#
# PREREGISTERED GATE (decide BEFORE looking at results)
# -----------------------------------------------------
# Compare the two arms by HELD-OUT evaluation at the same fixed density, not by training curves.
# ttc_sector advances only if BOTH hold:
#     capture(ttc) - capture(baseline) >= +2.0 pp
#     crash(ttc)   - crash(baseline)   <= -2.0 pp
# Ties or a capture gain paid for with more crashes = rejected, recorded, and the arm is frozen.
# Judge at the FINAL checkpoint. The corridor A/B dipped mid-run (0.6420 -> 0.5993 -> 0.6610)
# before recovering, and ttc_sector additionally has to re-adapt to a changed input distribution
# while the baseline arm does not -- an asymmetry that penalises it early by construction.
#
# EVAL (per arm, after both finish)
#   NAVRL_OBSTACLE_SELECTOR=<the arm's selector> NAVRL_V2_DENSITIES=70 NUM_ENVS=64 \
#     ./eval_navrl_v2_density_sweep.sh runs/<arm run>/nn/last_gen_ppo_ep_XXXX.pth
#   The selector MUST match the arm or the numbers are meaningless.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

ARM="${ARM:-}"
case "${ARM}" in
    baseline) SELECTOR=cluster_sector ;;
    ttc)      SELECTOR=ttc_sector ;;
    *)
        echo "usage: ARM=baseline|ttc $0 [extra runner args]" >&2
        echo "  baseline = cluster_sector (bearing-ranked)   ttc = ttc_sector (threat-ranked)" >&2
        exit 2
        ;;
esac

# ---- THE experimental variable, and the only difference between the arms ----
export NAVRL_OBSTACLE_SELECTOR="${SELECTOR}"

# ---- shared: fixed density, so no promotion can occur mid-comparison ----
# 70 bars because the warm-start checkpoint is a converged 70-bar policy and the crash floor was
# characterised there (13.1% extrapolated). A promotion during either arm would change the task
# under one arm and not the other.
export NAVRL_DENSITY_CURRICULUM=0
export NAVRL_NUM_BARS="${AB_BARS:-70}"
export NAVRL_MAX_BARS=300          # keep the PhysX actor count at the validated 4 GB profile
export NAVRL_DENSITY_MIN_EPOCHS=0  # inert with the curriculum off; pinned so a stray export cannot revive it

# ---- shared: identical start point and budget ----
export CKPT="${CKPT:-runs/ppo_260731_1940_navrl_v2-search-thr80-s1/nn/last_gen_ppo_ep_3250_rew_128.5965.pth}"
export SEED="${SEED:-1}"
# Budget matched in SAMPLES, not in epoch count. The warm-start checkpoint comes off the 3070 at
# 128 envs (32x128 = 4096 samples/epoch); this card runs 64 envs (32x64 = 2048), so an epoch here
# is worth half of one there. "+1000 epochs" would have delivered 2.05M steps -- half the intended
# adaptation. +2000 epochs = 4.1M steps = the 1000 3070-equivalent epochs actually meant.
# This is not a neutral shortfall: the baseline arm starts already converged for its selector while
# the ttc arm starts off-distribution, so a short budget penalises only the arm that needs to adapt.
export MAX_EPOCHS="${MAX_EPOCHS:-5250}"   # 3250 + 2000 @ 2048 samples/ep = 4.1M steps of adaptation
export AERIAL_RUN_TAG="${AERIAL_RUN_TAG:-v2-ttc-${ARM}-s${SEED}}"
export TRAIN_SESSION_LOG="${TRAIN_SESSION_LOG:-train_session_logs/v2_ttc_${ARM}_$(date +%y%m%d_%H%M%S).log}"

if [[ ! -f "${CKPT}" ]]; then
    echo "[ttc-ab] checkpoint not found: ${CKPT}" >&2
    echo "[ttc-ab] pass CKPT=<path to a 70-bar v2 checkpoint>" >&2
    exit 2
fi

echo "[ttc-ab] ARM=${ARM}  selector=${NAVRL_OBSTACLE_SELECTOR}  bars=${NAVRL_NUM_BARS} (fixed)"
echo "[ttc-ab] warm-start ${CKPT}"
echo "[ttc-ab] gate: capture >= +2.0pp AND crash <= -2.0pp vs the baseline arm, held-out, at the FINAL ckpt"

# 4 GB preset -> v2 contract. Both arms inherit arena, sensors, reward and target motion from
# train_navrl_v2_search.sh rather than restating them here.
exec ./train_navrl_v2_search_4gb.sh --checkpoint "${CKPT}" --branch_run "$@"
