#!/usr/bin/env bash
# GR-1: 1,000-epoch continuation of Frozen Policy F (ep25000 riskcap) with or without controller-gain randomization
# (D-2026-11-04-1; docs/plans/prereg_gain_randomization_gr1_v1.md). Policy F itself is never modified: the run
# branches into a new runs/ folder and writes a new checkpoint lineage.
#
#   usage: train_navrl_gr1.sh {rand40|ctrl}
#
# Everything else is the contract of the run that produced F (train_navrl_v2_ep24000_speed_governor.sh): fixed
# 205 bars, curriculum off, seed 1, LR 5e-6, the riskcap governor parameters re-read from the same validated screen
# summary, main profile (128 envs). The only difference between the two arms is NAVRL_GAIN_RAND (rand40: scale 0.4,
# gain seed 1). PYTHONPATH is pinned to this tree and the launcher refuses unless aerial_gym resolves inside it.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
HERE="$(pwd)"
ROOT="$(cd ../../.. && pwd)"

if (( $# != 1 )) || [[ "$1" != "rand40" && "$1" != "ctrl" ]]; then
    echo "[gr1-train] usage: train_navrl_gr1.sh {rand40|ctrl}" >&2
    exit 2
fi
ARM="$1"

export PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PATH="$(dirname "${PYTHON}"):${PATH}"
export PYTHONNOUSERSITE=1
export PYTHONPATH="${ROOT}"
ORIGIN="$("${PYTHON}" -c 'import importlib.util; print(importlib.util.find_spec("aerial_gym").origin)')"
case "${ORIGIN}" in
    "${ROOT}"/*) ;;
    *) echo "[gr1-train] aerial_gym resolves outside this tree: ${ORIGIN}" >&2; exit 2 ;;
esac

SOURCE_REL="runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth"
SOURCE_SHA="f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
GOV_SOURCE_SHA="82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"   # ep24000, the screen's source
SUMMARY="../../../results/navrl_v2_ep24000_riskcap_seed44_screen/summary.json"
export CKPT="${SOURCE_REL}"
[[ -f "${CKPT}" && -f "${SUMMARY}" ]] || { echo "[gr1-train] missing Policy F checkpoint or screen summary" >&2; exit 2; }
[[ "$(sha256sum "${CKPT}" | cut -d' ' -f1)" == "${SOURCE_SHA}" ]] || { echo "[gr1-train] Policy F SHA mismatch" >&2; exit 2; }

# Governor parameters of Policy F, from the same validated summary its own training used.
read -r SELECTED_MODE FIXED_MPS FREE_MPS HALF_WIDTH_M MARGIN_M SLOW_M RELEASE_M TTC_S BRAKE_MPS2 REACTION_S <<< "$(${PYTHON} - "${SUMMARY}" "${GOV_SOURCE_SHA}" <<'PY'
import json, sys
from pathlib import Path
payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
problems = []
if payload.get("source_checkpoint_sha256") != sys.argv[2]:
    problems.append("summary source SHA mismatch")
if payload.get("adaptive_go") is not True or payload.get("selected_tag") != "riskcap":
    problems.append("summary does not select the preregistered riskcap")
row = {r.get("tag"): r for r in payload.get("rows", [])}.get("riskcap") or {}
params = row.get("parameters") or {}
keys = ("speed_governor_fixed_mps", "speed_governor_free_mps", "speed_governor_half_width_m", "speed_governor_margin_m",
        "speed_governor_slow_m", "speed_governor_release_m", "speed_governor_ttc_s", "speed_governor_brake_mps2",
        "speed_governor_reaction_s")
if row.get("screen_pass") is not True or any(k not in params for k in keys):
    problems.append("riskcap row incomplete")
if problems:
    print("[gr1-train] refusing:", "; ".join(problems), file=sys.stderr)
    raise SystemExit(2)
print("riskcap", *(params[k] for k in keys))
PY
)"
export NAVRL_SPEED_GOVERNOR="${SELECTED_MODE}"
export NAVRL_SPEED_GOVERNOR_FIXED_MPS="${FIXED_MPS}"
export NAVRL_SPEED_GOVERNOR_FREE_MPS="${FREE_MPS}"
export NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M="${HALF_WIDTH_M}"
export NAVRL_SPEED_GOVERNOR_MARGIN_M="${MARGIN_M}"
export NAVRL_SPEED_GOVERNOR_SLOW_M="${SLOW_M}"
export NAVRL_SPEED_GOVERNOR_RELEASE_M="${RELEASE_M}"
export NAVRL_SPEED_GOVERNOR_TTC_S="${TTC_S}"
export NAVRL_SPEED_GOVERNOR_BRAKE_MPS2="${BRAKE_MPS2}"
export NAVRL_SPEED_GOVERNOR_REACTION_S="${REACTION_S}"
export NAVRL_SPEED_GOVERNOR_DIAG=1

# Same fixed-density continuation contract as Policy F's own 24001..25000 run, now 25001..26000.
export NAVRL_V2_ALLOW_RESUME=1
export NAVRL_V2_PROFILE=main
export NAVRL_OBSTACLE_SELECTOR=cluster_sector
export SEED=1
export NAVRL_DENSITY_CURRICULUM=0
export NAVRL_NUM_BARS=205
export NAVRL_DENSITY_START=205
export NAVRL_DENSITY_FINAL=205
export NAVRL_DENSITY_WARMUP=0
export NAVRL_DENSITY_MIN_EPOCHS=0
export NAVRL_RESET_DENSITY_WINDOW=1
export NAVRL_LEARNING_RATE=5e-6
export NAVRL_RESET_ACTOR_OPTIMIZER=0
export MAX_EPOCHS=26000

unset NAVRL_GAIN_RAND NAVRL_GAIN_RAND_SCALE NAVRL_GAIN_RAND_SEED
if [[ "${ARM}" == "rand40" ]]; then
    export NAVRL_GAIN_RAND=1 NAVRL_GAIN_RAND_SCALE=0.4 NAVRL_GAIN_RAND_SEED=1
fi
export AERIAL_RUN_TAG="v2-gr1-ep25000-205bars-main-riskcap-${ARM}-s1"
export TRAIN_SESSION_LOG="train_session_logs/gr1_${ARM}_$(date +%y%m%d_%H%M%S)_$$.log"
export TRAIN_LIVE_LOG="train_session_logs/current_training.log"

unset NAVRL_RECOVERY_STAGE NAVRL_RECOVERY_SOURCE_EPOCH
unset NAVRL_RECOVERY_SOURCE_SHA256 NAVRL_RECOVERY_SMOKE_REQUIRED_EPOCHS
unset NAVRL_RECOVERY_SMOKE_BARS NAVRL_RECOVERY_EVAL_ATTESTATION_SHA256
unset NAVRL_RECOVERY_EVAL_ATTESTATION_B64
unset NAVRL_GENERAL_EVAL NAVRL_INTERACTIVE NAVRL_BULK_EVAL NAVRL_BULK_EVAL_JSON
unset NAVRL_GENERAL_RESULTS_JSON NAVRL_EVAL_CHECKPOINT NAVRL_EVAL_TARGET_SPEED_FINAL
unset NAVRL_EVAL_FULL_DISTRIBUTION NAVRL_EVAL_ACTION_MODE NAVRL_V2_ACTION_MODE
unset NAVRL_EVAL_RUN_NONCE NAVRL_EVAL_PROFILE NAVRL_SIM_PHYSICS_CONTRACT
unset ALLOW_CONCURRENT NAVRL_V2_FORCE

echo "[gr1-train] arm=${ARM} source=Policy F (${SOURCE_SHA:0:12}) epochs 25001..26000 fixed 205 bars lr 5e-6 seed 1"
echo "[gr1-train] gain randomization: ${NAVRL_GAIN_RAND:-off} scale=${NAVRL_GAIN_RAND_SCALE:-} seed=${NAVRL_GAIN_RAND_SEED:-}"
echo "[gr1-train] aerial_gym=${ORIGIN}"
if [[ "${NAVRL_PREFLIGHT_ONLY:-0}" == "1" ]]; then
    export NAVRL_V2_CONTRACT_PREFLIGHT_ONLY=1
fi
exec ./train_navrl_v2_search.sh --checkpoint "${CKPT}" --branch_run
