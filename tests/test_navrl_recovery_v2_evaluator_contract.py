"""CPU-only contract tests for the recovery-v2 32-cell evaluator."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "recovery_v2_gate_test_module",
    ROOT / "tools/verify_navrl_physical_target_recovery_v2_gate.py",
)
GATE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = GATE
SPEC.loader.exec_module(GATE)


def rows(throughput=100.0):
    output = []
    for route in GATE.ROUTE_ARMS:
        for speed in GATE.SPEEDS:
            for bars in GATE.DENSITIES:
                telemetry = {
                    "watchdog_breach_substeps": 0,
                    "direct_position_writes": 0,
                    "reset_calls_during_advance": 0,
                }
                row = {
                    "record_id": GATE.record_id(route, speed, bars),
                    "seed": GATE.SEED, "envs": GATE.ENVS, "steps": GATE.STEPS,
                    "warmup_steps": GATE.WARMUP_STEPS,
                    "route_mode": route, "speed_mps": speed, "bars": bars,
                    "tracking_rmse_mps": 0.1, "mean_speed_ratio": 0.9,
                    "contact_step_fraction": 0.0,
                    "off_rounded_local_infeasible_fraction": 0.0,
                    "recovery_normal_route_rounded_invalidation_fraction": 0.0,
                    "motor_saturation_fraction": 0.0, "max_tilt_deg": 1.0,
                    "invalid_state_fraction": 0.0, "telemetry_summary": telemetry,
                    "throughput": {"env_intervals_per_s": throughput},
                    "measurement_denominators": {
                        "tracking_env_intervals": GATE.ENVS * (GATE.STEPS-GATE.WARMUP_STEPS),
                        "safety_env_intervals": GATE.ENVS * GATE.STEPS,
                    },
                    "local_metric_contract": {
                        "numerator": 0,
                        "denominator": GATE.ENVS * GATE.STEPS,
                        "geometry": (
                            "bounded rounded Euclidean surface clearance"
                            if route == "off" else
                            "normal-route rounded invalidation only; exact CONNECT failures are in recovery reasons"
                        ),
                    },
                    "task_clock": {"increments": GATE.STEPS},
                    "initial_layout_sha256": "a" * 64,
                    "initial_robot_pose_sha256": "b" * 64,
                    "initial_target_pose_sha256": "c" * 64,
                    "initial_task_waypoint_sha256": "d" * 64,
                }
                row["gates"] = GATE.row_gates(row)
                row["pass"] = all(row["gates"].values())
                output.append(row)
    return output


class RecoveryV2EvaluatorContractTest(unittest.TestCase):
    def test_exact_32_cell_grid_and_denominators(self):
        records = rows()
        GATE.validate_grid(records)
        self.assertEqual(len(records), 32)

    def test_missing_cell_and_matched_throughput_regression_are_refused(self):
        with self.assertRaisesRegex(GATE.IntegrityError, "32-cell"):
            GATE.validate_grid(rows()[:-1])
        records = rows()
        for row in records:
            if row["route_mode"] != "off" and row["speed_mps"] == 0.6 and row["bars"] == 70:
                row["throughput"]["env_intervals_per_s"] = 49.0
        with self.assertRaisesRegex(GATE.IntegrityError, "throughput"):
            GATE.validate_grid(records)

    def test_atomic_json_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "receipt.json"
            GATE.atomic_json(path, {"x": 1})
            with self.assertRaisesRegex(GATE.IntegrityError, "overwrite"):
                GATE.atomic_json(path, {"x": 2})

    def test_probe_contract_uses_raw_verified_lookup_and_exact_schema_pair(self):
        with tempfile.TemporaryDirectory() as directory:
            receipt = Path(directory) / "receipt.json"
            core = {"certified_lateral_tube_p95_m": 0.02}
            payload = {
                "schema": "navrl_target_recovery_braking_receipt_v1",
                "probe_schema": "navrl_target_recovery_braking_probe_v1",
                "decel_p05_mps2": 2.0, "stop_time_p95_s": 0.8,
                "core_integration": core,
            }
            receipt.write_text(json.dumps(payload), encoding="utf-8")
            lookup = {
                format(speed, ".1f"): {"p95_stop_distance_m": speed * speed / 4.0}
                for speed in GATE.SPEEDS
            }
            values = {
                "NAVRL_TARGET_RECOVERY_BRAKE_P05": "2.0",
                "NAVRL_TARGET_RECOVERY_STOP_TIME_P95_S": "0.8",
                "NAVRL_TARGET_RECOVERY_BRAKE_PROBE_RECEIPT": str(receipt),
                "NAVRL_TARGET_RECOVERY_BRAKE_PROBE_RECEIPT_SHA256": GATE.sha256_file(receipt),
            }
            summary = {
                "measured_speed_to_p95_lookup": lookup,
                "certified_monotone_speed_to_p95_lookup": lookup,
            }
            with mock.patch.object(GATE.BRAKE_VERIFY, "verify_receipt",
                                   return_value={"summary": summary}), mock.patch.object(
                GATE.BRAKE_VERIFY, "core_integration_object", return_value=core
            ):
                validated = GATE.validate_braking_probe_values(values)
            self.assertIn("__MEASURED_LOOKUP_JSON", validated)
            self.assertIn("__CERTIFIED_LOOKUP_JSON", validated)
            self.assertEqual(validated["__LATERAL_TUBE_P95_M"], "0.02")
            payload["schema"] = "navrl_target_recovery_braking_probe_v1"
            receipt.write_text(json.dumps(payload), encoding="utf-8")
            values["NAVRL_TARGET_RECOVERY_BRAKE_PROBE_RECEIPT_SHA256"] = GATE.sha256_file(receipt)
            with mock.patch.object(GATE.BRAKE_VERIFY, "verify_receipt",
                                   return_value={"summary": summary}), mock.patch.object(
                GATE.BRAKE_VERIFY, "core_integration_object", return_value=core
            ):
                with self.assertRaisesRegex(GATE.IntegrityError, "schema pair"):
                    GATE.validate_braking_probe_values(values)

    def test_v1_evaluator_bytes_remain_unchanged(self):
        expected = GATE.git("show", "afb48c4:%s" % GATE.BASE_PATH.relative_to(ROOT))
        self.assertEqual(GATE.BASE_PATH.read_text(encoding="utf-8").rstrip(), expected.rstrip())


if __name__ == "__main__":
    unittest.main()
