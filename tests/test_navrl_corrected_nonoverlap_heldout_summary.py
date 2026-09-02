from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "heldout_summary",
    ROOT / "tools/summarize_navrl_corrected_nonoverlap_heldout.py",
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
RESULT = ROOT / "results/navrl_corrected_nonoverlap_physical_off_heldout_seed313"


class HeldoutSummaryTest(unittest.TestCase):
    def test_wilson_interval_contains_observed_fraction(self):
        lo, hi = MODULE.wilson95(1715, 2049)
        self.assertLess(lo, 1715 / 2049)
        self.assertGreater(hi, 1715 / 2049)

    def test_frozen_results_validate_with_explicit_metadata_erratum(self):
        result = MODULE.summarize(RESULT)
        self.assertEqual(result["status"], "COMPLETE_VALID_WITH_METADATA_ERRATUM")
        self.assertEqual([cell["bars"] for cell in result["cells"]], [70, 85, 100, 115, 130, 145])
        self.assertFalse(result["metadata_erratum"]["outcome_affecting"])
        self.assertFalse(result["metadata_erratum"]["raw_evidence_modified"])
        self.assertFalse(result["interpretation"]["resume_authorized"])
        self.assertFalse(result["interpretation"]["routed_ppo_authorized"])
        self.assertLess(result["density_trend"]["capture_delta_70_to_145_pp"], -18.0)


if __name__ == "__main__":
    unittest.main()
