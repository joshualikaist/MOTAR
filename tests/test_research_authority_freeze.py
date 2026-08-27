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
