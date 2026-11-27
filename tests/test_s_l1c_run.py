"""CPU-only checks of the S-L1C canary contract and its independent analysis helpers."""
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
import s_l1c_run as runner  # noqa: E402
import s_l1c_analysis as analysis  # noqa: E402

PLAN = runner.load_plan()


class ContractTest(unittest.TestCase):
    def test_matrix_seeds_and_lineages(self):
        specs = runner.process_specs(PLAN)
        self.assertEqual([s["id"] for s in specs],
                         ["sensors_legacy", "sensors_s_l1b", "sensors_s_l1c"] +
                         [c + "_" + v for c in ("static", "moving") for v in ("off", "s_l1a", "s_l1b", "s_l1c_p1", "s_l1c_p2")])
        by_id = {s["id"]: s for s in specs}
        self.assertEqual((by_id["static_off"]["seed"], by_id["moving_s_l1c_p2"]["seed"]), (4501, 4502))
        self.assertEqual(by_id["static_s_l1c_p1"]["arm"], "P1")
        self.assertEqual([by_id[k]["lineage"] for k in ("sensors_legacy", "sensors_s_l1b", "sensors_s_l1c", "static_off",
                                                        "static_s_l1a", "static_s_l1b", "static_s_l1c_p2")],
                         [None, "s_l1b_v1", "s_l1c_v1", "off", "s_l1a_v1", "s_l1b_v1", "s_l1c_v1"])
        self.assertNotEqual(PLAN["base_sha"], "TBD_AT_COMMIT")
        self.assertEqual(len(PLAN["references"]["s_l1b_run_manifest_sha256"]), 64)

    def test_inherited_lineage_scrubbed(self):
        specs = {s["id"]: s for s in runner.process_specs(PLAN)}
        with mock.patch.dict("os.environ", {"NAVRL_SENSOR_LINEAGE": "s_l1c_v1"}):
            legacy = runner.process_env(PLAN, specs["sensors_legacy"], Path("/tmp/unused"))
            off = runner.process_env(PLAN, specs["static_off"], Path("/tmp/unused"))
        self.assertNotIn("NAVRL_SENSOR_LINEAGE", legacy)
        self.assertEqual(off["NAVRL_SENSOR_LINEAGE"], "off")
        self.assertEqual(off["NAVRL_KROC2027_RUN_ID"], "SL1C-C01")

    def test_noninterference_pairs(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp)
            calls = []
            with mock.patch.object(runner.a, "compare_digest_files",
                                   side_effect=lambda l, r, s: calls.append(str(r)) or {"identical": True}):
                for variant in ("off", "s_l1a", "s_l1b", "s_l1c_p1", "s_l1c_p2"):
                    if variant == "s_l1c_p2":
                        (run_dir / "moving_s_l1c_p1").mkdir()
                    runner.noninterference(run_dir, {"id": "moving_" + variant, "config": "moving", "variant": variant, "steps": 600})
        self.assertEqual(len(calls), 4)
        self.assertIn("EL0-R1/v3_moving_p2", calls[0])
        self.assertIn("SL1A-C01/moving_on_p2", calls[1])
        self.assertIn("SL1B-C01/moving_s_l1b_p2", calls[2])
        self.assertIn("moving_s_l1c_p1", calls[3])

    def test_completion_flags(self):
        good = {"perception_s_l1a": True, "perception_s_l1b": True, "perception_s_l1c": False, "detector_lineage_enabled": True}
        with mock.patch.object(runner.a, "validate_done", return_value=good):
            runner.validate_done(Path("."), {"id": "static_s_l1b", "batch": "V3", "variant": "s_l1b"}, {})
            with self.assertRaises(ValueError):
                runner.validate_done(Path("."), {"id": "static_s_l1c_p2", "batch": "V3", "variant": "s_l1c_p2"}, {})

    def test_uncommitted_contract_refuses_official_preflight(self):
        with tempfile.TemporaryDirectory() as temp:
            receipt = Path(temp) / "receipt.json"
            receipt.write_text(json.dumps({"checkpoint_sha256": PLAN["policy_f_sha256"]}))
            with mock.patch.object(runner.base, "preflight", return_value=([], {"policy_f_sha256": PLAN["policy_f_sha256"]})), \
                    mock.patch.object(runner.a, "committed", return_value=False), \
                    mock.patch.object(runner.el0, "config_receipt", return_value=receipt), \
                    mock.patch.object(runner.a, "historical_bindings", return_value={}), \
                    mock.patch.object(runner, "bound_run", return_value={}), \
                    mock.patch.object(runner.base, "git", return_value=SimpleNamespace(returncode=0, stdout="")):
                problems, _ = runner.preflight(PLAN, "a" * 40, Path(temp) / "new", False)
        self.assertTrue(any("not committed" in x for x in problems))


class AnalysisHelperTest(unittest.TestCase):
    def test_eligibility_uses_lidar_truth_only(self):
        ok, d = analysis.eligible_positive([], [], np.array([5., 0., 0.]))
        self.assertTrue(ok)
        self.assertAlmostEqual(d, 5.0)
        hidden, _ = analysis.eligible_positive([[3.3, 0., .5]], [[.3, .3, 1.5]], np.array([6., 0., 0.]))
        self.assertFalse(hidden)
        high, _ = analysis.eligible_positive([], [], np.array([4., 0., .5]))
        self.assertFalse(high)
        near, _ = analysis.eligible_positive([], [], np.array([1., 0., 0.]))
        self.assertFalse(near)

    def test_firewall_ast(self):
        fw = analysis.firewall_ast()
        self.assertEqual(fw["forbidden_parameters"], [])
        self.assertEqual(fw["imports"], ["math", "os", "torch"])


if __name__ == "__main__":
    unittest.main()
