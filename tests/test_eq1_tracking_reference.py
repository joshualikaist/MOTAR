"""EQ-1C tracker (CPU): hand-derived CV Kalman reference, 5 s expiry, identity clock, range-only information."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import eq1_tracking_reference as trk  # noqa: E402


class ReferenceSelfTest(unittest.TestCase):
    def test_reference_filter_algebra(self):
        ref = trk.ReferenceCV()
        ref.step(np.array([1.0, 2.0, 1.0]), True, trk.CAMERA_VAR)
        ref.step(np.zeros(3), False, trk.CAMERA_VAR)
        p = ref.P
        self.assertTrue(np.allclose(p, p.T))
        self.assertTrue(np.all(np.linalg.eigvalsh(p) > 0))
        self.assertAlmostEqual(p[0, 0], trk.CAMERA_VAR[0] + 0.01 * 1.0 + 0.025 * 0.01, places=12)
        post = trk.range_only_posterior(np.eye(3), np.array([1.0, 0, 0]), 0.08)
        self.assertAlmostEqual(post[1, 1], 1.0, places=12)
        self.assertAlmostEqual(post[0, 0], 0.08 ** 2 / (1 + 0.08 ** 2), places=12)


class ProductionTrackerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = {c["name"][:2]: c for c in trk.run()["cases"]}

    def test_t1_kalman(self):
        c = self.cases["T1"]
        self.assertEqual(c["production"]["status"], "PASS")
        self.assertLess(c["worst_rel_err"], trk.TOL)

    def test_t2_expiry_51_misses(self):
        rows = {r["misses"]: r for r in self.cases["T2"]["rows"]}
        self.assertEqual([rows[k]["active"] for k in (0, 49, 50, 51, 60)], [True, True, True, False, False])
        self.assertLess(rows[50]["age_float32"], 5.0)

    def test_t3_identity_clock(self):
        c = self.cases["T3"]
        self.assertEqual(c["lidar_accepts"], 100)
        self.assertTrue(c["track_active_after_10s"])
        self.assertEqual(c["literal_contract"]["status"], "PASS")
        self.assertEqual(c["production_identity_clock"]["status"], "FAIL")

    def test_t4_range_information(self):
        c = self.cases["T4"]
        self.assertEqual(c["production_policy_f"]["status"], "FAIL")
        self.assertEqual(c["existing_opt_in_flag"]["status"], "PASS")
        f0 = next(r for r in c["rows"] if not r["lidar_range_only_update"] and r["bearing_deg"] == 0.0)
        self.assertLess(f0["tangential_var_prod"], 0.05)
        self.assertAlmostEqual(f0["tangential_var_ref"], 1.0, places=9)


if __name__ == "__main__":
    unittest.main()
