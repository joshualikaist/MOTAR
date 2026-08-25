#!/usr/bin/env bash
set -euo pipefail

# New physical-platform lineage. Deliberately fresh-only: legacy/bounded checkpoints encode a
# different target state transition and cannot be resumed merely because tensor shapes match.
if [[ -n "${CKPT:-}" || -n "${CHECKPOINT:-}" ]]; then
  echo "[physical-fresh] refusing CKPT/CHECKPOINT: physical 6-DoF target requires fresh PPO" >&2
  exit 4
fi
for arg in "$@"; do
  case "${arg}" in
    --checkpoint|--checkpoint=*|--resume_in_place|--branch_run)
      echo "[physical-fresh] refusing resume/checkpoint flag: ${arg}" >&2
      exit 4
      ;;
  esac
done

export NAVRL_ROBOT=navrl_ref5in_quad
export NAVRL_TARGET_DYNAMICS=physical
# Explicit child-entry marker. The base v2 launcher otherwise forces legacy dynamics so a stale
# interactive-shell NAVRL_TARGET_DYNAMICS cannot silently switch the canonical training lineage.
export NAVRL_V2_PHYSICAL_FRESH_CHILD=1
export NAVRL_VISION="${NAVRL_VISION:-1}"
export NAVRL_PERCEPTION="${NAVRL_PERCEPTION:-1}"
export NAVRL_GENERAL_TRAIN="${NAVRL_GENERAL_TRAIN:-1}"
export NAVRL_ARENA_XY="${NAVRL_ARENA_XY:-40}"
export NAVRL_ARENA_Z="${NAVRL_ARENA_Z:-3}"
export NAVRL_BAR_POOL="${NAVRL_BAR_POOL:-bars_h3}"
export NAVRL_BAR_X_MIN="${NAVRL_BAR_X_MIN:-0}"
export NAVRL_BAR_X_MAX="${NAVRL_BAR_X_MAX:-1}"
export NAVRL_PLACEMENT_MODE="${NAVRL_PLACEMENT_MODE:-navrl_band}"
export NAVRL_MAX_BARS="${NAVRL_MAX_BARS:-300}"
export NAVRL_TARGET_SPEED_FINAL="${NAVRL_TARGET_SPEED_FINAL:-1.5}"
export NAVRL_TARGET_SPEED_RAMP_EPOCHS="${NAVRL_TARGET_SPEED_RAMP_EPOCHS:-1}"

echo "[physical-fresh] robot=$NAVRL_ROBOT target=$NAVRL_TARGET_DYNAMICS fresh=1"
echo "[physical-fresh] WARNING: ref5in is an internally consistent simulation design point; hardware BOM/ID is pending"
exec ./train_navrl_v2_search.sh "$@"
