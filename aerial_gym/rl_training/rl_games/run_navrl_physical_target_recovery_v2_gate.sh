#!/usr/bin/env bash
set -euo pipefail

if (($# != 0)); then
  echo "[recovery-v2-gate] no positional arguments; use the documented environment only" >&2
  exit 2
fi

ROOT="$(git -C "$(dirname "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)"
PYTHON="${NAVRL_PYTHON:-/home/fair/miniconda3/envs/aerialgym/bin/python}"
: "${NAVRL_TARGET_RECOVERY_BRAKE_P05:?set from the verified physical-target braking receipt}"
: "${NAVRL_TARGET_RECOVERY_STOP_TIME_P95_S:?set from the verified physical-target braking receipt}"
: "${NAVRL_TARGET_RECOVERY_BRAKE_PROBE_RECEIPT:?set the finalized receipt.json path}"
: "${NAVRL_TARGET_RECOVERY_BRAKE_PROBE_RECEIPT_SHA256:?set the exact receipt.json SHA256}"

test -x "${PYTHON}"
cd "${ROOT}"
if [[ "${RECOVERY_V2_GATE_PREFLIGHT:-0}" == "1" ]]; then
  exec "${PYTHON}" tools/verify_navrl_physical_target_recovery_v2_gate.py --preflight
fi
exec "${PYTHON}" tools/verify_navrl_physical_target_recovery_v2_gate.py
