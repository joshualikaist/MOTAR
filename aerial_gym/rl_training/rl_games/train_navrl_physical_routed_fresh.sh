#!/usr/bin/env bash
set -euo pipefail

# Fresh-only physical+waypoint lineage. The parent physical launcher and base v2 launcher each
# repeat the checkpoint/dynamics guards; this wrapper adds the exact route contract.
if [[ -n "${CKPT:-}" || -n "${CHECKPOINT:-}" ]]; then
  echo "[physical-routed] refusing CKPT/CHECKPOINT: global target routes require fresh PPO" >&2
  exit 4
fi
for arg in "$@"; do
  case "${arg}" in
    --checkpoint|--checkpoint=*|--resume_in_place|--branch_run)
      echo "[physical-routed] refusing resume/checkpoint flag: ${arg}" >&2
      exit 4
      ;;
  esac
done

export NAVRL_TARGET_ROUTE_MODE=global_astar_v1
export NAVRL_TARGET_PATTERN=waypoint
export NAVRL_PHYSICAL_ROUTED_CHILD=1
export NAVRL_TARGET_ROUTE_RESOLUTION_M=0.25
export NAVRL_TARGET_ROUTE_MAX_EXPANSIONS=50000
export NAVRL_TARGET_ROUTE_MAX_WAYPOINTS=128
export NAVRL_TARGET_ROUTE_REPLAN_COOLDOWN_STEPS=10
export NAVRL_TARGET_ROUTE_GOAL_TOLERANCE_M=0.05
export NAVRL_TARGET_ROUTE_MIN_GOAL_DISTANCE_M=6.0

echo "[physical-routed] model=physx_ref5in_6dof_global_astar_aabb_v1 fresh=1"
echo "[physical-routed] route=global_astar_v1 pattern=waypoint grid=0.25m fail_closed=1"
exec ./train_navrl_physical_fresh.sh "$@"
