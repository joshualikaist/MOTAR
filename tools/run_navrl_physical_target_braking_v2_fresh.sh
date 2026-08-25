#!/usr/bin/env bash
set -euo pipefail

# Fresh-only physical-target braking probe.  It accepts only --output and --preflight; any
# checkpoint/resume/continuation argument is rejected by the Python entry point.
root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export AERIAL_GYM_SIM_NAME=base_sim
export NAVRL_ROBOT=navrl_ref5in_quad
export NAVRL_TARGET_DYNAMICS=physical
export NAVRL_TARGET_PATTERN=waypoint
export NAVRL_TARGET_ROUTE_MODE=off
export NAVRL_NUM_BARS=70
export NAVRL_MAX_BARS=300
export NAVRL_REQUIRE_SOURCE_ROOT="$root_dir"
export PYTHONPATH="$root_dir:$root_dir/tools${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$root_dir/tools/run_navrl_physical_target_braking_v2_fresh.py" "$@"
