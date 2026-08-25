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
if [[ "${NAVRL_PHYSICAL_ROUTED_CHILD:-0}" == "1" ]]; then
  if [[ "${NAVRL_TARGET_ROUTE_MODE:-}" != "global_astar_v1" || "${NAVRL_TARGET_PATTERN:-}" != "waypoint" ]]; then
    echo "[physical-fresh] routed child requires global_astar_v1 + waypoint" >&2
    exit 4
  fi
else
  if [[ "${NAVRL_TARGET_ROUTE_MODE:-off}" != "off" ]]; then
    echo "[physical-fresh] canonical physical lineage refuses target route; use train_navrl_physical_routed_fresh.sh" >&2
    exit 4
  fi
  export NAVRL_TARGET_ROUTE_MODE=off
fi
unset NAVRL_PHYSICAL_ROUTED_CHILD
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
