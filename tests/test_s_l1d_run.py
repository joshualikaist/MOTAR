"""CPU-only checks of the S-L1D canary contract and its independent analysis helpers."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
for extra in ("tools", "tools/audits"):
    if str(ROOT / extra) not in sys.path:
        sys.path.insert(0, str(ROOT / extra))
import s_l1d_run as runner  # noqa: E402
import s_l1d_analysis as analysis  # noqa: E402

PLAN = runner.load_plan()


class ContractTest(unittest.TestCase):
    def test_matrix_and_lineages(self):
        specs = runner.process_specs(PLAN)
        self.assertEqual([s["id"] for s in specs], ["sensors_s_l1c", "sensors_s_l1d"] +
                         [c + "_" + v for c in ("static", "moving") for v in ("off", "s_l1c", "s_l1d_p1", "s_l1d_p2")])
        by_id = {s["id"]: s for s in specs}
        self.assertEqual((by_id["static_off"]["seed"], by_id["moving_s_l1d_p2"]["seed"]), (4501, 4502))
        self.assertEqual(by_id["static_s_l1d_p1"]["arm"], "P1")
        self.assertEqual([by_id[k]["lineage"] for k in ("sensors_s_l1c", "sensors_s_l1d", "static_off", "static_s_l1c", "static_s_l1d_p2")],
                         ["s_l1c_v1", "s_l1d_v1", "off", "s_l1c_v1", "s_l1d_v1"])
        self.assertNotEqual(PLAN["base_sha"], "TBD_AT_COMMIT")

    def test_noninterference_pairs(self):
        with tempfile.TemporaryDirectory() as temp:
            run_dir = Path(temp)
            calls = []
            with mock.patch.object(runner.a, "compare_digest_files",
                                   side_effect=lambda l, r, s: calls.append(str(r)) or {"identical": True}):
                (run_dir / "static_s_l1d_p1").mkdir()
                (run_dir / "static_s_l1c").mkdir()
                for variant in ("off", "s_l1c", "s_l1d_p1", "s_l1d_p2"):
                    runner.noninterference(run_dir, {"id": "static_" + variant, "config": "static", "variant": variant, "steps": 600})
        self.assertEqual(len(calls), 4)
        self.assertIn("EL0-R1/v3_static_p2", calls[0])
        self.assertIn("SL1C-C01/static_s_l1c_p2", calls[1])
        self.assertTrue(calls[2].endswith("static_s_l1c/v3_steps.jsonl"))
        self.assertIn("static_s_l1d_p1", calls[3])

    def test_completion_flags(self):
        good = {"perception_s_l1a": True, "perception_s_l1b": True, "perception_s_l1c": True, "perception_s_l1d": True,
                "detector_lineage_enabled": True}
        with mock.patch.object(runner.a, "validate_done", return_value=good):
            runner.validate_done(Path("."), {"id": "static_s_l1d_p2", "batch": "V3", "variant": "s_l1d_p2"}, {})
            with self.assertRaises(ValueError):
                runner.validate_done(Path("."), {"id": "static_s_l1c", "batch": "V3", "variant": "s_l1c"}, {})

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
    def test_required_mapping(self):
        self.assertEqual(analysis.required("FULL", 30, 2, True, True), ("OCCLUSION_FULL_EVIDENCE",))
        self.assertEqual(analysis.required("FULL", 1, 2, True, True), ("PRED_SCALE_LOSS",))
        self.assertEqual(analysis.required("CENTRE_ONLY", 30, 2, True, True), ("OCCLUSION_CENTRE_EVIDENCE",))
        self.assertEqual(analysis.required("PARTIAL", 30, 2, False, False), analysis.PARTIAL_TYPES)
        self.assertEqual(analysis.required("FOV_EXIT", 0, 2, True, False), ("PRED_FOV_EXIT",))
        self.assertIsNone(analysis.required("VISIBLE", 30, 2, True, False))

    def test_firewall_ast(self):
        fw = analysis.firewall_ast()
        self.assertEqual(fw["forbidden_parameters"], [])
        self.assertEqual(fw["imports"], ["math", "os", "torch"])


if __name__ == "__main__":
    unittest.main()
