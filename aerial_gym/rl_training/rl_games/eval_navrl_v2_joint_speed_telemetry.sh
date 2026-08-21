#!/usr/bin/env bash
# Frozen-policy, one-cell descriptive diagnosis of speed allocation before bar contact.
# This is evaluation only.  It does not compare or tune riskcap parameters.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

PYTHON="${PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
export PYTHON PYTHONNOUSERSITE=1
POLICY="${POLICY:-runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/nn/last_gen_ppo_ep_25000_rew_39.742134.pth}"
POLICY_SHA="f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
RESULT_ROOT="${RESULT_ROOT:-../../../results/navrl_v2_joint_speed_allocation_seed379}"
EPISODES="${EPISODES:-4097}"
PREFLIGHT="${PREFLIGHT:-0}"

[[ -f "${POLICY}" ]] || { echo "[joint-speed] missing policy: ${POLICY}" >&2; exit 2; }
[[ "${EPISODES}" =~ ^[1-9][0-9]*$ ]] || { echo "[joint-speed] invalid EPISODES" >&2; exit 2; }
if (( EPISODES < 2049 )); then
    echo "[joint-speed] preregistration requires EPISODES>=2049" >&2; exit 2
fi
if [[ -e "${RESULT_ROOT}" && "${PREFLIGHT}" != "1" ]]; then
    echo "[joint-speed] refusing to overwrite ${RESULT_ROOT}" >&2; exit 2
fi

ACTUAL_SHA="$(${PYTHON} - "${POLICY}" <<'PY'
import hashlib, sys
from pathlib import Path
print(hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)"
[[ "${ACTUAL_SHA}" == "${POLICY_SHA}" ]] || {
    echo "[joint-speed] policy SHA mismatch: ${ACTUAL_SHA}" >&2; exit 3;
}

# Exact frozen navigation/control candidate.  These constants come from the preregistered braking
# probe and existing riskcap campaign; changing any one creates a different experiment.
export NAVRL_V2_DENSITIES=205
export NAVRL_V2_ACTION_MODE=deterministic
export NAVRL_EVAL_REFLECTION_MODE=original
export NAVRL_SPEED_GOVERNOR=riskcap
export NAVRL_SEED=379
export NAVRL_SPEED_GOVERNOR_FIXED_MPS=2.0
export NAVRL_SPEED_GOVERNOR_FREE_MPS=3.53553390593
export NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M=0.45
export NAVRL_SPEED_GOVERNOR_MARGIN_M=0.45
export NAVRL_SPEED_GOVERNOR_SLOW_M=3.0
export NAVRL_SPEED_GOVERNOR_RELEASE_M=5.0
export NAVRL_SPEED_GOVERNOR_TTC_S=1.2
export NAVRL_SPEED_GOVERNOR_BRAKE_MPS2=2.9608856678
export NAVRL_SPEED_GOVERNOR_REACTION_S=0.1
export NAVRL_JOINT_SPEED_TELEMETRY=1
unset NAVRL_DETECTOR_CHECKPOINT

if [[ "${PREFLIGHT}" == "1" ]]; then
    echo "[joint-speed] PREFLIGHT PASS | ep25000+riskcap bars=205 seed=379 episodes=${EPISODES}"
    exit 0
fi

NAVRL_V2_RESULT_DIR="${RESULT_ROOT}" \
    ./eval_navrl_v2_density_sweep.sh "${POLICY}" "${EPISODES}"

PYTHONPATH="../../../:${PYTHONPATH:-}" "${PYTHON}" \
    ../../../tools/analyze_navrl_joint_speed.py \
    "${RESULT_ROOT}/205bars.json" "${RESULT_ROOT}/205bars.receipt.json" \
    --output "${RESULT_ROOT}/assessment.json"
echo "[joint-speed] done -> ${RESULT_ROOT}/assessment.json"
