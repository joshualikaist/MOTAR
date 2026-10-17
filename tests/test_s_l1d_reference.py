"""Self-checks of the independent S-L1D float64 oracle against hand-derived geometry (CPU)."""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import s_l1d_reference as ref  # noqa: E402

TIGHT = np.eye(3) * 1e-6


def case(boxes, halves, target, detected=False, cov=TIGHT, primed=None):
    ev = ref.evaluator_class(boxes, halves, target)
    depth = ref.delivered_depth(ev)
    online = ref.online_class(depth, detected, True, target if primed is None else primed, cov)
    return ev, online


class EvaluatorAndOnlineTest(unittest.TestCase):
    def test_classes_and_mapping(self):
        cases = {
            "clear": (([], [], [5., 0., 0.]), "VISIBLE", "UNEXPLAINED_LOSS"),
            "full": (([[3.3, 0., .5]], [[.3, .6, 1.5]], [6., 0., 0.]), "FULL", "OCCLUSION_FULL_EVIDENCE"),
            "centre_only": (([[2.3, .7, .5]], [[.3, .7, 1.5]], [4., .05, 0.]), "CENTRE_ONLY", "OCCLUSION_CENTRE_EVIDENCE"),
            "partial": (([[2.3, .74, .5]], [[.3, .7, 1.5]], [4., 0., 0.]), "PARTIAL", "OCCLUSION_PARTIAL_EVIDENCE"),
            "fov_exit": (([], [], [5. * math.cos(math.radians(50)), 5. * math.sin(math.radians(50)), 0.]), "FOV_EXIT", "PRED_FOV_EXIT"),
            "range_exit": (([], [], [24., 0., 0.]), "RANGE_EXIT", "PRED_RANGE_EXIT"),
        }
        for name, (args, el0, online) in cases.items():
            with self.subTest(case=name):
                ev, on = case(*args)
                self.assertEqual(ev["class"], el0, ev["class"])
                self.assertEqual(on["class"], online, on)
                if el0 in ("FULL", "CENTRE_ONLY", "PARTIAL"):
                    self.assertTrue(ev["centre_robust"])

    def test_scale_loss_band(self):
        far = ref.evaluator_class([], [], [23.5 - .1, 0., 0.])
        self.assertIn(far["class"], ("SCALE_LOSS", "RANGE_EXIT", "VISIBLE"))
        for d in (14., 18.):
            ev, on = case([], [], [d, 0., 0.])
            if ev["alone_px"] < 2:
                self.assertEqual(on["class"], "PRED_SCALE_LOSS")
        ev, on = case([[3.3, 0., .5]], [[.3, .6, 1.5]], [16.5, 0., 0.])
        if 0 < ev["alone_px"] < 2:
            self.assertEqual(on["class"], "PRED_SCALE_LOSS")  # occluded but too small to detect anyway

    def test_detected_and_inactive_take_precedence(self):
        ev, on = case([], [], [5., 0., 0.], detected=True)
        self.assertEqual(on["class"], "DETECTED")
        self.assertEqual(ref.online_class(np.full((90, 160), 10.), False, False, [5., 0., 0.], TIGHT)["class"], "NO_TRACK")

    def test_tau_from_track_covariance(self):
        ev = ref.evaluator_class([[4.0, 0., .5]], [[.05, .6, 1.5]], [4.4, 0., 0.])  # occluder ~0.2 m in front
        depth = ref.delivered_depth(ev)
        tight = ref.online_class(depth, False, True, [4.4, 0., 0.], TIGHT)
        loose = ref.online_class(depth, False, True, [4.4, 0., 0.], np.eye(3) * .04)
        self.assertAlmostEqual(tight["tau_m"], .05 + 2e-3, places=9)
        self.assertAlmostEqual(loose["tau_m"], .45, places=9)
        self.assertGreater(tight["occluded"], 0)
        self.assertEqual(loose["occluded"], 0)

    def test_alone_pass_never_occlusion_and_zero_support_never_full(self):
        for target in ([3., 0., 0.], [12., 1., 0.], [0., 5., 0.], [24., 0., 0.]):
            ev, on = case([], [], target)
            self.assertNotIn(on["class"], ("OCCLUSION_FULL_EVIDENCE", "OCCLUSION_CENTRE_EVIDENCE", "OCCLUSION_PARTIAL_EVIDENCE"))

    def test_wrong_prediction_changes_support(self):
        ev = ref.evaluator_class([], [], [5., 0., 0.])
        shifted = ref.online_class(ref.delivered_depth(ev), False, True, [5., .3, 0.], TIGHT)
        self.assertNotEqual(shifted["support"], ev["alone_px"])

    def test_ambiguous_pixel_count(self):
        self.assertEqual(ref.ambiguous_support_pixels([5., 0., 0.]), 0)
        rays = ref.rays()
        d = rays[44, 79]
        # Place the sphere so pixel (44, 79) grazes it exactly: centre at distance r from the ray.
        normal = np.cross(d, [0., 0., 1.])
        normal /= np.linalg.norm(normal)
        centre = ref.CAMERA_TRANSLATION + 5. * d + ref.RADIUS * normal
        self.assertGreaterEqual(ref.ambiguous_support_pixels(centre), 1)

    def test_self_test_and_no_production_import(self):
        self.assertTrue(all(ref.self_test().values()))
        tree = ast.parse((ROOT / "tools/audits/s_l1d_reference.py").read_text())
        names = {a.name for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom)) for a in n.names}
        self.assertFalse(any("aerial_gym" in x or "torch" in x for x in names))


if __name__ == "__main__":
    unittest.main()
