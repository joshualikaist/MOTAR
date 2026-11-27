"""EQ-1A sensor geometry (CPU): independent references and the preserved production counterexamples.

The production verdicts asserted here are FAILs on purpose: these tests lock in that the frozen
sensor code still reproduces the EQ-0 counterexamples, and that each proposed correction (reference
code only) passes the same unrelaxed gate. A source change that alters a production verdict makes
these tests fail, which is the point: it needs its own lineage and a new validation record.
"""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import eq1_geometry_reference as geo  # noqa: E402

_RECORD = None


def record():
    global _RECORD
    if _RECORD is None:
        _RECORD = geo.run()
    return _RECORD


def case(prefix):
    return next(c for c in record()["cases"] if c["name"].startswith(prefix))


class ReferenceSelfTest(unittest.TestCase):
    def test_pinhole_and_intersections(self):
        self.assertAlmostEqual(geo.FX, 160 / (2 * math.tan(math.radians(43.5))), places=12)
        self.assertEqual(geo.pinhole_column(0.0), 79.5)
        self.assertAlmostEqual(geo.column_bearing(geo.pinhole_column(0.3)), 0.3, places=12)
        o, d = np.zeros(3), np.array([1.0, 0, 0])
        self.assertAlmostEqual(geo.ray_sphere(o, d, np.array([5.0, 0, 0]), 0.15), 4.85, places=12)
        self.assertTrue(math.isinf(geo.ray_sphere(o, d, np.array([5.0, 0.2, 0]), 0.15)))
        lo, hi = np.array([2.0, -1, -1]), np.array([3.0, 1, 1])
        self.assertAlmostEqual(geo.ray_box(o, d, lo, hi), 2.0, places=12)
        self.assertTrue(math.isinf(geo.ray_box(o, -d, lo, hi)))

    # The live "HEAD equals measurement source" check moved to tests/test_s_l1a_eq1_provenance.py:
    # the approved opt-in S-L1A lineage edits navrl_detector.py and navrl_perception.py. That test
    # binds this frozen record to immutable measurement-source blobs. The live counterexamples below
    # still run the default-OFF production path.


class GeometryCounterexampleTest(unittest.TestCase):
    def test_a1_column_mapping_20deg(self):
        c = case("A1")
        r = c["case_20deg"]
        self.assertAlmostEqual(r["u_pinhole"], 48.8164, places=3)
        self.assertEqual(r["u_production"], 43)
        self.assertAlmostEqual(r["pixel_error"], 5.8164, places=3)
        self.assertAlmostEqual(r["bearing_error_deg"], 3.4109, places=3)
        self.assertGreater(c["worst"]["pixel_error"], 6.0)
        self.assertEqual(c["production"]["status"], "FAIL")
        self.assertEqual(c["proposed_correction"]["status"], "PASS")
        self.assertTrue(all(r["proposed_pixel_error"] <= 0.5 for r in c["rows"]))

    def test_a2_common_origin(self):
        c = case("A2")
        self.assertAlmostEqual(c["on_axis"]["error_m"], -0.10, delta=0.001)
        self.assertEqual(c["production"]["status"], "FAIL")
        self.assertLessEqual(c["proposed_worst_error_m"], geo.RANGE_GATE_M)
        self.assertEqual(c["proposed_correction"]["status"], "PASS")

    def test_a3_vertical_collapse_is_conservative_not_metric(self):
        c = case("A3")
        self.assertEqual(c["rows_fused_m"], [3.0] * 4)
        self.assertEqual(c["production_conservative"]["status"], "PASS")
        self.assertEqual(c["production_metric_per_elevation"]["status"], "FAIL")

    def test_a4_radius_conventions(self):
        c = case("A4")
        self.assertAlmostEqual(c["bias_m"], -0.05, places=9)
        self.assertEqual(c["production"]["status"], "FAIL")
        self.assertEqual(c["proposed_correction"]["status"], "PASS")
        self.assertTrue(all(0 < r["bias_m"] <= c["camera_bias_limit_m"] for r in c["camera_centre_range"]))
        self.assertEqual(c["production_camera_estimator"]["status"], "FAIL")

    def test_a5_occlusion_classes(self):
        c = case("A5")
        self.assertEqual(set(c["disagreements"]), {
            "clear 20 m, centre on a pixel corner (scale)",
            "partial: bar over one side, centre clear",
            "rim-proxy gap: two bars block all 9 rays, a strip stays visible"})
        rows = {r["case"]: r for r in c["rows"]}
        gap = rows["rim-proxy gap: two bars block all 9 rays, a strip stays visible"]
        self.assertTrue(gap["rim_full"] and gap["camera_detectable"])
        self.assertEqual(rows["clear 20 m, centre on a pixel corner (scale)"]["alone_pixels"], 0)
        self.assertEqual(rows["full: wide bar at 5 m, target 10 m"]["reference"], "FULL")
        self.assertEqual(rows["FOV exit 50 deg"]["current_indicators"], "FOV_EXIT")
        self.assertEqual(c["production_indicators"]["status"], "FAIL")
        band = c["scale_loss"]
        self.assertLess(band["first_range_with_possible_scale_loss_m"], band["first_range_with_certain_scale_loss_m"])
        self.assertGreater(band["first_range_with_possible_scale_loss_m"], 10.0)

    def test_a7_camera_measurement(self):
        c = case("A7")
        self.assertEqual(c["production_direction"]["status"], "PASS")
        self.assertEqual(c["production_range"]["status"], "FAIL")
        self.assertTrue(all(r["visible"] and r["direction_error_px"] < 0.2 for r in c["rows"]))
        self.assertTrue(all(0.03 < r["range_error_m"] < 0.07 for r in c["rows"]))
        below = next(r for r in c["rows"] if r["case"] == "0.5 m below, 3 m")
        self.assertGreater(below["direction_error_px_if_offset_ignored"], 0.5)  # the test can see the offset

    def test_a6_false_association(self):
        c = case("A6")
        fx = c["fixtures"]
        self.assertTrue(fx["bar at predicted range (no target)"]["accepted_as_target"])
        self.assertEqual(fx["bar at predicted range (no target)"]["age_after"], 0.0)
        self.assertTrue(c["negative_control_rejected"])
        self.assertEqual(c["production_identity"]["status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
