#!/usr/bin/env bash
set -euo pipefail
: "${NAVRL_V3_OUTPUT_ROOT:?set a new, unique pilot output root}"
: "${NAVRL_V3_CELL_RUNNER:?set the tracked executable v3 cell adapter}"
: "${NAVRL_V3_BRAKING_RECEIPT:?set the canonical raw braking receipt.json}"
: "${NAVRL_V3_BRAKING_RECEIPT_SHA256:?set its exact SHA256}"
exec python3 tools/run_navrl_braking_route_v3_gate.py pilot \
  --output-root "${NAVRL_V3_OUTPUT_ROOT}" \
  --cell-runner "${NAVRL_V3_CELL_RUNNER}" \
  --braking-receipt "${NAVRL_V3_BRAKING_RECEIPT}" \
  --braking-receipt-sha256 "${NAVRL_V3_BRAKING_RECEIPT_SHA256}"
