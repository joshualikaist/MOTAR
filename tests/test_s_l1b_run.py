"""CPU-only checks of the S-L1B canary contract and its independent analysis helpers."""
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for extra in ("tools", "tools/audits"):
    if str(ROOT / extra) not in sys.path:
        sys.path.insert(0, str(ROOT / extra))
import s_l1b_run as runner  # noqa: E402
import s_l1b_analysis as analysis  # noqa: E402

PLAN = runner.load_plan()


class ContractTest(unittest.TestCase):
    def test_plan_amendment_and_matrix(self):
        self.assertIn("B5_carve_common_frame_elevation", PLAN["corrections"])
        self.assertEqual(PLAN["base_sha"], "2d65f1df40f05eeeadb4b1510a09d4b80fcfdf84")
        specs = runner.process_specs(PLAN)
        self.assertEqual([s["id"] for s in specs],
                         ["sensors_legacy", "sensors_s_l1a", "sensors_s_l1b"] +
                         [c + "_" + v for c in ("static", "moving") for v in ("off", "s_l1a", "s_l1b_p1", "s_l1b_p2")])
        by_id = {s["id"]: s for s in specs}
        self.assertEqual(by_id["static_off"]["seed"], 4501)
        self.assertEqual(by_id["moving_s_l1b_p2"]["seed"], 4502)
        self.assertEqual(by_id["sensors_s_l1b"]["num_envs"], 43)
        self.assertEqual(by_id["static_s_l1b_p1"]["arm"], "P1")
        self.assertEqual([by_id[k]["lineage"] for k in ("sensors_legacy", "sensors_s_l1a", "sensors_s_l1b",
                                                        "static_off", "static_s_l1a", "static_s_l1b_p1")],
                         [None, "s_l1a_v1", "s_l1b_v1", "off", "s_l1a_v1", "s_l1b_v1"])

    def test_amendment_cannot_override_gates(self):
        bad = {"overrides": {"gates": {"B-CARVE": "relaxed"}}}
        with mock.patch.object(runner.a, "read_json", side_effect=[runner.a.read_json(runner.PLAN_PATH), bad]):
            with self.assertRaises(ValueError):
                runner.load_plan()

    def test_inherited_lineage_is_scrubbed(self):
        specs = {s["id"]: s for s in runner.process_specs(PLAN)}
        with mock.patch.dict("os.environ", {"NAVRL_SENSOR_LINEAGE": "s_l1b_v1"}):
            legacy = runner.process_env(PLAN, specs["sensors_legacy"], Path("/tmp/unused"))
            off = runner.process_env(PLAN, specs["static_off"], Path("/tmp/unused"))
        self.assertNotIn("NAVRL_SENSOR_LINEAGE", legacy)
        self.assertEqual(off["NAVRL_SENSOR_LINEAGE"], "off")
        self.assertEqual(off["NAVRL_KROC2027_RUN_ID"], "SL1B-C01")

    def test_noninterference_pairs(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp)
            calls = []
            with mock.patch.object(runner.a, "compare_digest_files",
                                   side_effect=lambda l, r, s: calls.append((Path(l).parent.name, str(r))) or {"identical": True}):
                for variant in ("off", "s_l1a", "s_l1b_p1", "s_l1b_p2"):
                    if variant == "s_l1b_p2":
                        (run_dir / "static_s_l1b_p1").mkdir()
                    runner.noninterference(run_dir, {"id": "static_" + variant, "config": "static",
                                                     "variant": variant, "steps": 600})
        self.assertIn("EL0-R1/v3_static_p2", calls[0][1])
        self.assertIn("SL1A-C01/static_on_p2", calls[1][1])
        self.assertIn("static_s_l1b_p1", calls[2][1])
        self.assertEqual(len(calls), 3)

    def test_completion_flags_must_match_specification(self):
        spec = {"id": "static_s_l1b_p2", "batch": "V3", "variant": "s_l1b_p2"}
        good = {"perception_s_l1a": True, "perception_s_l1b": True, "detector_lineage_enabled": True}
        with mock.patch.object(runner.a, "validate_done", return_value=good):
            runner.validate_done(Path("."), spec, {})
        for broken in ({**good, "perception_s_l1b": False}, {**good, "detector_lineage_enabled": False}):
            with mock.patch.object(runner.a, "validate_done", return_value=broken), self.assertRaises(ValueError):
                runner.validate_done(Path("."), spec, {})

    def test_uncommitted_contract_refuses_official_preflight(self):
        with tempfile.TemporaryDirectory() as temp:
            receipt = Path(temp) / "receipt.json"
            receipt.write_text(json.dumps({"checkpoint_sha256": PLAN["policy_f_sha256"]}))
            with mock.patch.object(runner.base, "preflight", return_value=([], {"policy_f_sha256": PLAN["policy_f_sha256"]})), \
                    mock.patch.object(runner.a, "committed", return_value=False), \
                    mock.patch.object(runner.el0, "config_receipt", return_value=receipt), \
                    mock.patch.object(runner.a, "historical_bindings", return_value={}), \
                    mock.patch.object(runner, "s_l1a_bindings", return_value={}), \
                    mock.patch.object(runner.base, "git", return_value=SimpleNamespace(returncode=0, stdout="")):
                problems, _ = runner.preflight(PLAN, "a" * 40, Path(temp) / "new", False)
        self.assertTrue(any("not committed" in problem for problem in problems))


class AnalysisHelperTest(unittest.TestCase):
    def test_angle_error_wraps(self):
        self.assertAlmostEqual(float(analysis.angle_error(math.pi - 1e-6, -math.pi + 1e-6)), 2e-6, places=9)

    def test_firewall_ast_on_current_helper(self):
        fw = analysis.firewall_ast()
        self.assertEqual(fw["forbidden_parameters"], [])
        self.assertEqual(fw["imports"], ["math", "os", "torch"])
        self.assertFalse(fw["import_from"])

    def test_lidar_truth_beam_convention(self):
        angles = np.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72)
        hits = analysis.lidar_truth([[5.1, 0, 0]], [[.1, 20, 20]], None, angles, .2)
        self.assertAlmostEqual(float(hits["depth_m"][2, 36]), 5.0, places=9)
        self.assertAlmostEqual(float(hits["depth_m"][0, 36]), 5.0 / math.cos(math.radians(20)), places=9)
        self.assertGreaterEqual(float(hits["depth_m"][2, 0]), 12.0)


if __name__ == "__main__":
    unittest.main()
