#!/usr/bin/env python3
"""CPU-frozen contract for recovery-v2 NO_CONNECTOR geometry forensics.

This module locks the 2026-08-26 preregistration before any GPU child. The observer
``--run`` path is not part of this freeze. Evaluation-only: it must not change target
commands, planner decisions, the 32-cell evaluator, gain, ``0.45 m``, or env count.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional


ROOT = Path(__file__).resolve().parents[1]
SEED = 827
ENVS = 32
STEPS = 300
WARMUP_STEPS = 20
DENSITIES = (70,)
SPEEDS = (0.6, 0.9, 1.2, 1.25)
ROUTE_MODE = "global_astar_recovery_v2"
CONTRACT_VARIANT = "baseline_1p25"
GRID_RESOLUTION_M = 0.25
TRACKING_MARGIN_M = 0.45
ANCHOR_RADIUS_CELLS = 3
SOFT_HYSTERESIS_M = 0.25
HARD_EPSILON_M = 1e-4
REACHABLE_TUBE_MARGIN_M = 0.0123
RECOVERY_HARD_EPSILON_M = HARD_EPSILON_M + REACHABLE_TUBE_MARGIN_M
RUNTIME_WALL_MARGIN_M = 0.50
ROUTE_BOUNDARY_MARGIN_M = 1.25
VEL_KP = 2.5
GATE_SOURCE_COMMIT = "2b151d9a4c4fe078ecc027152e5642fa857a2e2f"
GATE_SUMMARY = ROOT / "results/navrl_physical_target_recovery_v2_gate_lower1p25_seed827/summary.json"
GATE_RECEIPT = ROOT / "results/navrl_physical_target_recovery_v2_gate_lower1p25_seed827/receipt.json"
GATE_SUMMARY_SHA256 = "a85e95764061b7b20cacaa622efc44e2d7e31e054e398f57cfdf48ec98e6c04f"
GATE_RECEIPT_SHA256 = "707636fcbcfe0c855267b39e307af7ac133a0feabbf25d2e7feba726465f1f96"
BRAKING_RECEIPT = ROOT / "results/navrl_physical_target_braking_lower1p25_headingrest_seed827/receipt.json"
BRAKING_RECEIPT_SHA256 = "4e87eb9ddf5dd9cea1fc0354d272a5d18ec6a05427e0f41e672749a57df9047a"
OUTPUT_ROOT = ROOT / "results/navrl_physical_target_recovery_v2_no_connector_forensics_seed827"
PREREGISTRATION = "docs/preregistration_physical_target_recovery_v2_no_connector_forensics_2026-08-26.md"
SCHEMA = "navrl_physical_target_recovery_v2_no_connector_forensics_v1"
PRIMARY_PACKED_CLASSES = (
    "brake_no_anchor_likely",
    "same_interval_brake_no_anchor_likely",
)
WILSON_N_MIN = 20
PLANNER_PATH = ROOT / "aerial_gym/task/navrl_task/target_route_planner.py"

FROZEN_CHILD_ENV = {
    "AERIAL_GYM_SIM_NAME": "base_sim",
    "NAVRL_ROBOT": "navrl_ref5in_quad",
    "NAVRL_TARGET_DYNAMICS": "physical",
    "NAVRL_TARGET_ROUTE_MODE": ROUTE_MODE,
    "NAVRL_TARGET_BRAKING_CONTRACT_VARIANT": CONTRACT_VARIANT,
    "NAVRL_TARGET_PATTERN": "waypoint",
    "NAVRL_TARGET_MAX_ACCEL": "4.0",
    "NAVRL_TARGET_MAX_TURN_RATE_DEG": "150.0",
    "NAVRL_TARGET_LOOKAHEAD_S": "1.0",
    "NAVRL_TARGET_VEL_KP": "2.5",
    "NAVRL_TARGET_TRACKING_MARGIN_M": "0.45",
    "NAVRL_TARGET_ROUTE_RESOLUTION_M": "0.25",
    "NAVRL_NUM_BARS": "70",
    "NAVRL_MAX_BARS": "300",
    "NAVRL_PLACEMENT_MODE": "navrl_band",
    "NAVRL_PLACEMENT_TOUCH_M": "0.4",
    "NAVRL_PLACEMENT_GAP_M": "1.6",
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


_PLANNER = None


def _load_planner():
    global _PLANNER
    if _PLANNER is not None:
        return _PLANNER
    spec = importlib.util.spec_from_file_location(
        "navrl_target_route_planner_no_connector_forensics", PLANNER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    _PLANNER = module
    return module


def probe_contract() -> dict[str, Any]:
    """Constants frozen before GPU. Does not read local result files."""
    return {
        "schema": SCHEMA,
        "seed": SEED,
        "envs": ENVS,
        "steps": STEPS,
        "warmup_steps": WARMUP_STEPS,
        "route_mode": ROUTE_MODE,
        "contract_variant": CONTRACT_VARIANT,
        "target_dynamics": "physical",
        "pattern": "waypoint",
        "densities": list(DENSITIES),
        "speeds_mps": list(SPEEDS),
        "grid_resolution_m": GRID_RESOLUTION_M,
        "tracking_margin_m": TRACKING_MARGIN_M,
        "anchor_radius_cells": ANCHOR_RADIUS_CELLS,
        "soft_hysteresis_m": SOFT_HYSTERESIS_M,
        "hard_epsilon_m": HARD_EPSILON_M,
        "reachable_tube_margin_m": REACHABLE_TUBE_MARGIN_M,
        "recovery_hard_epsilon_m": RECOVERY_HARD_EPSILON_M,
        "runtime_wall_margin_m": RUNTIME_WALL_MARGIN_M,
        "route_boundary_margin_m": ROUTE_BOUNDARY_MARGIN_M,
        "boundary_soft_minus_hard_m": ROUTE_BOUNDARY_MARGIN_M - RUNTIME_WALL_MARGIN_M,
        "vel_kp": VEL_KP,
        "gate_source_commit": GATE_SOURCE_COMMIT,
        "gate_summary_sha256": GATE_SUMMARY_SHA256,
        "gate_receipt_sha256": GATE_RECEIPT_SHA256,
        "braking_receipt_sha256": BRAKING_RECEIPT_SHA256,
        "primary_packed_classes": list(PRIMARY_PACKED_CLASSES),
        "wilson_n_min": WILSON_N_MIN,
        "preregistration": PREREGISTRATION,
        "output_root": str(OUTPUT_ROOT.relative_to(ROOT)),
        "gate_artifacts_read_only": True,
        "original_evaluator_unchanged": True,
    }


def frozen_contract() -> dict[str, Any]:
    """Fail-closed provenance check used by --check-contract and the future GPU child."""
    if not GATE_SUMMARY.is_file() or sha256(GATE_SUMMARY) != GATE_SUMMARY_SHA256:
        raise RuntimeError("lower-1.25 gate summary provenance mismatch; refusing diagnostic")
    if not GATE_RECEIPT.is_file() or sha256(GATE_RECEIPT) != GATE_RECEIPT_SHA256:
        raise RuntimeError("lower-1.25 gate receipt provenance mismatch; refusing diagnostic")
    if not BRAKING_RECEIPT.is_file() or sha256(BRAKING_RECEIPT) != BRAKING_RECEIPT_SHA256:
        raise RuntimeError("heading-rest braking receipt provenance mismatch; refusing diagnostic")
    contract = probe_contract()
    contract["gate_summary_path"] = str(GATE_SUMMARY)
    contract["gate_receipt_path"] = str(GATE_RECEIPT)
    contract["braking_receipt_path"] = str(BRAKING_RECEIPT)
    return contract


def wilson_interval(success: int, total: int, z: float = 1.96) -> Optional[tuple[float, float]]:
    if total < 1:
        return None
    p = float(success) / float(total)
    denominator = 1.0 + z * z / float(total)
    center = p + z * z / (2.0 * float(total))
    spread = z * math.sqrt(p * (1.0 - p) / float(total) + z * z / (4.0 * float(total) * float(total)))
    return float((center - spread) / denominator), float((center + spread) / denominator)


def descriptive_verdict(anchor_present: int, primary_n: int) -> dict[str, Any]:
    """Frozen label. Cannot pass the 32-cell mechanism gate."""
    interval = wilson_interval(int(anchor_present), int(primary_n)) if int(primary_n) else None
    absent_interval = (
        wilson_interval(int(primary_n) - int(anchor_present), int(primary_n))
        if int(primary_n) else None
    )
    if int(primary_n) < WILSON_N_MIN or interval is None or absent_interval is None:
        label = "INCONCLUSIVE"
    elif interval[0] > 0.5:
        label = "ANCHOR_PRESENT_LATCH"
    elif absent_interval[0] > 0.5:
        label = "ANCHOR_ABSENT_AT_LATCH"
    else:
        label = "INCONCLUSIVE"
    return {
        "label": label,
        "primary_n": int(primary_n),
        "anchor_present": int(anchor_present),
        "wilson_present": None if interval is None else {"lower": interval[0], "upper": interval[1]},
        "wilson_absent": (
            None if absent_interval is None
            else {"lower": absent_interval[0], "upper": absent_interval[1]}
        ),
        "passes_32_cell_mechanism": False,
        "authorizes_retune_or_ppo": False,
    }


def recovery_v2_anchor_kwargs() -> dict[str, Any]:
    return {
        "resolution_m": GRID_RESOLUTION_M,
        "radius_cells": ANCHOR_RADIUS_CELLS,
        "tracking_margin_m": TRACKING_MARGIN_M,
        "soft_hysteresis_m": SOFT_HYSTERESIS_M,
        "hard_epsilon_m": RECOVERY_HARD_EPSILON_M,
    }


def v1_forensic_anchor_kwargs() -> dict[str, Any]:
    """The 2026-08-25 search. Using these on recovery-v2 latches is a contract failure."""
    return {
        "resolution_m": GRID_RESOLUTION_M,
        "radius_cells": ANCHOR_RADIUS_CELLS,
        "tracking_margin_m": TRACKING_MARGIN_M,
        "soft_hysteresis_m": 0.0,
        "hard_epsilon_m": HARD_EPSILON_M,
    }


def recovery_anchor_query(
    point,
    bars,
    bar_half,
    bounds_lo,
    bounds_hi,
    support,
    *,
    variant: str = "recovery_v2",
) -> dict[str, Any]:
    planner = _load_planner()
    kwargs = recovery_v2_anchor_kwargs() if variant == "recovery_v2" else v1_forensic_anchor_kwargs()
    return planner.nearest_soft_free_anchor(
        point, bars, bar_half, bounds_lo, bounds_hi, support,
        RUNTIME_WALL_MARGIN_M, ROUTE_BOUNDARY_MARGIN_M, **kwargs,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-contract", action="store_true")
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if args.run:
        raise SystemExit(
            "GPU --run is not part of this CPU freeze; implement the observer in a later commit"
        )
    if args.check_contract:
        print(json.dumps(frozen_contract(), indent=2, sort_keys=True))
        return 0
    print(json.dumps(probe_contract(), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
