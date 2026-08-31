"""CPU-only contract tests for the braking-route-v3 verifier and frozen grid."""

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "braking_v3_gate_standalone",
    ROOT / "tools/verify_navrl_braking_route_v3_gate.py",
)
GATE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GATE
SPEC.loader.exec_module(GATE)


def _identity():
    hashes = {name: "a" * 64 for name in GATE.IDENTITY_HASH_FIELDS}
    hashes.update({
        "runtime_clean": True,
        "physical_geometry_version": "v2",
        "placement_mode": "footprint_clearance",
        "preregistration_sha256": GATE.PREREG_SHA256,
        "robot_name": "navrl_ref5in_v2_quad",
        "target_dynamics": "physical",
        "target_pattern": "waypoint",
    })
    return hashes


def _diagnostics(**overrides):
    payload = {
        "runtime_replan_unsafe_start_count": 0,
        "soft_envelope_exit_count": 0,
        "accepted_terminal_stop_certificate_count": 100,
        "accepted_command_count": 100,
        "plan_attempt_count": 100,
        "plan_success_count": 100,
        "fallback_interval_count": 0,
        "goal_completion_count": 32,
    }
    payload.update(overrides)
    return payload


def _cell(route, speed, bars, seed, identity, **overrides):
    local_name = (
        "off_bounded_local_step_infeasible_fraction"
        if route == "off"
        else "routed_local_step_invalidation_fraction"
    )
    absent = (
        "routed_local_step_invalidation_fraction"
        if route == "off"
        else "off_bounded_local_step_infeasible_fraction"
    )
    row = {
        "record_id": GATE.record_id(route, speed, bars),
        "seed": seed, "envs": GATE.ENVS, "steps": GATE.STEPS,
        "warmup_steps": GATE.WARMUP_STEPS,
        "route_mode": route, "speed_mps": speed, "bars": bars,
        "tracking_rmse_mps": 0.1, "mean_speed_ratio": 0.9,
        "contact_step_fraction": 0.0, "motor_saturation_fraction": 0.0,
        "max_tilt_deg": 1.0, "invalid_state_fraction": 0.0,
        local_name: 0.0, absent: None,
        "identity": identity,
        "initial_layout_sha256": "b" * 64,
        "initial_robot_pose_sha256": "c" * 64,
        "initial_target_pose_sha256": "d" * 64,
        "v3_diagnostics": None if route == "off" else _diagnostics(),
    }
    row.update(overrides)
    row["physical_gates"] = GATE.physical_gate_metrics(row)
    row["physical_pass"] = all(row["physical_gates"].values())
    return row


def _summary(stage, identity=None, cells=None, **overrides):
    config = GATE.STAGES[stage]
    identity = identity or _identity()
    if cells is None:
        cells = [
            _cell(route, speed, bars, config["seed"], identity)
            for route in GATE.ROUTE_ARMS
            for speed in GATE.SPEEDS
            for bars in config["densities"]
        ]
    payload = {
        "schema": GATE.SCHEMA, "stage": stage, "seed": config["seed"],
        "route_arms": list(GATE.ROUTE_ARMS), "speeds_mps": list(GATE.SPEEDS),
        "densities": list(config["densities"]),
        "envs": GATE.ENVS, "steps": GATE.STEPS, "warmup_steps": GATE.WARMUP_STEPS,
        "physical_gates_preregistered": GATE.PHYSICAL_GATES,
        "identity": identity, "pilot_authorization": None, "cells": cells,
    }
    payload.update(overrides)
    return payload


class BrakingRouteV3GateContractTest(unittest.TestCase):
    def test_preregistration_bytes_and_grid_are_frozen(self):
        self.assertEqual(GATE.sha256_file(GATE.PREREG), GATE.PREREG_SHA256)
        self.assertEqual(GATE.STAGES["pilot"]["seed"], 829)
        self.assertEqual(GATE.STAGES["confirmatory"]["seed"], 839)
        self.assertEqual(list(GATE.STAGES["pilot"]["densities"]), [70])
        self.assertEqual(list(GATE.STAGES["confirmatory"]["densities"]), [70, 115, 160, 205])
        self.assertNotIn(300, GATE.STAGES["confirmatory"]["densities"])
        self.assertEqual(GATE.ROUTE_ARMS, ("off", "global_astar_braking_v3"))
        self.assertEqual(GATE.ENVS, 32)
        self.assertEqual(GATE.STEPS, 300)

    def test_pilot_pass_does_not_authorize_long_training(self):
        payload = _summary("pilot")
        verdict = GATE.derive_verdict(payload)
        self.assertEqual(verdict["execution_integrity"], "PASS_8_CELL_INTEGRITY")
        self.assertEqual(verdict["gate"], "PASS_PILOT_AUTHORIZES_CONFIRMATORY")
        self.assertFalse(verdict["long_training_authorized"])

    def test_pilot_mechanism_fail_blocks_confirmatory(self):
        identity = _identity()
        cells = []
        for route in GATE.ROUTE_ARMS:
            for speed in GATE.SPEEDS:
                extras = {}
                if route != "off":
                    extras["v3_diagnostics"] = _diagnostics(runtime_replan_unsafe_start_count=1)
                cells.append(_cell(route, speed, 70, 829, identity, **extras))
        verdict = GATE.derive_verdict(_summary("pilot", identity=identity, cells=cells))
        self.assertEqual(verdict["gate"], "FAIL_BLOCKS_CONFIRMATORY")
        self.assertFalse(verdict["mechanism_pass"])

    def test_confirmatory_without_pilot_pass_is_void(self):
        verdict = GATE.derive_verdict(_summary("confirmatory"))
        self.assertEqual(verdict["execution_integrity"], "VOID_EXECUTION")
        self.assertEqual(verdict["gate"], "NOT_INTERPRETED")

    def test_prereg_sha_drift_is_void(self):
        identity = _identity()
        identity["preregistration_sha256"] = "0" * 64
        verdict = GATE.derive_verdict(_summary("pilot", identity=identity))
        self.assertEqual(verdict["execution_integrity"], "VOID_EXECUTION")

    def test_threshold_override_is_void(self):
        gates = dict(GATE.PHYSICAL_GATES)
        gates["mean_speed_ratio_min"] = 0.1
        verdict = GATE.derive_verdict(_summary("pilot", physical_gates_preregistered=gates))
        self.assertEqual(verdict["execution_integrity"], "VOID_EXECUTION")


if __name__ == "__main__":
    unittest.main()
