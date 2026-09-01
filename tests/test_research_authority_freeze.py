import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "check_research_authority", ROOT / "tools" / "check_research_authority.py"
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class TestResearchAuthorityFreeze(unittest.TestCase):
    def test_frozen_authority_matches_result_summaries(self):
        receipt = MODULE.verify_authority()
        self.assertFalse(receipt["track_a"]["stage2_authorised"])
        self.assertFalse(receipt["track_b"]["long_training_authorized"])
        self.assertEqual(receipt["hardware_state"]["real_flights"], 0)
        geometry = receipt["corrected_environment_v2_2026_08_27"]["density_geometry_audit"]
        self.assertTrue(geometry["training_cap_passes"])
        self.assertEqual(geometry["training_cap_bars"], 205)
        self.assertEqual(geometry["disconnected_at_bars"], 300)
        self.assertFalse(geometry["authorizes_ppo"])
        gate = receipt["corrected_environment_v2_2026_08_27"]["route_physical_gate_r2"]
        self.assertEqual(gate["execution_integrity"], "PASS_32_CELL_INTEGRITY")
        self.assertEqual(gate["route_mechanism"], "FAIL_ROUTE_MECHANISM")
        self.assertFalse(gate["authorizes_ppo"])
        self.assertEqual(gate["fresh_ppo_epochs_run"], 0)
        lower_v3 = receipt["corrected_environment_v2_2026_08_27"][
            "braking_aware_route_v3_lower1p25"
        ]
        self.assertTrue(lower_v3["gpu_authority"])
        self.assertFalse(lower_v3["authorizes_ppo"])
        self.assertEqual(
            lower_v3["gpu_authority_scope"],
            "one fresh baseline_1p25 receipt and one seed-829 70-bar 8-cell pilot only",
        )

    def test_track_b_prohibits_training_and_retuning(self):
        receipt = MODULE.verify_authority()
        prohibited = set(receipt["track_b"]["prohibited"])
        self.assertTrue({"ppo_training", "gain_or_margin_retune", "cell_or_grid_rerun"} <= prohibited)

    def test_evidence_sha_drift_fails_closed(self):
        receipt = json.loads(MODULE.DEFAULT_RECEIPT.read_text(encoding="utf-8"))
        receipt["evidence"][0]["sha256"] = "0" * 64
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "authority.json"
            path.write_text(json.dumps(receipt), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "SHA drift"):
                MODULE.verify_authority(path)


if __name__ == "__main__":
    unittest.main()
