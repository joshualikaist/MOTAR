"""S-L1D preregistered CPU fixtures that the first commit left unasserted (review 2026-10-18).

`docs/plans/s_l1d_v1.json` lists these CPU cases: clear targets 2-24 m, FOV edge 40/44/50 deg,
scale loss at 12.75-23 m, full occlusion, centre-only, partial, an S14-like rim gap and a primed
centre offset of 0.2 m. Each case checks production == independent oracle (class, support,
occluded count) and the preregistered evaluator-to-online mapping where it constrains the case.
Production code is the qualified 76f0f4c source; nothing here changes a gate.
"""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
for extra in ("tools/audits", "tools"):
    if str(ROOT / extra) not in sys.path:
        sys.path.insert(0, str(ROOT / extra))
import s_l1d_reference as oracle  # noqa: E402
import s_l1d_analysis as analysis  # noqa: E402

FX, FY = 160 / (2 * math.tan(math.radians(43.5))), 90 / (2 * math.tan(math.radians(29.)))
TIGHT = np.eye(3) * 1e-6


def helper():
    spec = importlib.util.spec_from_file_location("_s_l1d_fixture_helper", ROOT / "aerial_gym/task/navrl_task/navrl_sensor_lineage.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor = helper()
RAYS = sensor.camera_unit_rays(160, 90, FX, FY, 79.5, 44.5)


def production(depth, centre, cov=TIGHT):
    out = sensor.measurement_validity(
        torch.from_numpy(np.asarray(depth, dtype=np.float32)).view(1, 90, 160), torch.tensor([False]), torch.tensor([True]),
        torch.from_numpy(np.asarray([centre], dtype=np.float32)), torch.from_numpy(np.asarray(cov, dtype=np.float32)).view(1, 3, 3),
        RAYS, torch.tensor([.10, 0., .03]), target_radius=.15, camera_far_m=20., min_target_pixels=2,
        hfov=math.radians(87.), vfov=math.radians(58.), fx=FX, fy=FY, cx=79.5, cy=44.5)
    return oracle.ONLINE[int(out["code"][0])], int(out["support_px"][0]), int(out["occluded_px"][0])


def check(test, boxes, halves, target, primed=None):
    """Production equals the oracle; the preregistered mapping holds where it constrains the case."""
    ev = oracle.evaluator_class(boxes, halves, target)
    depth = oracle.delivered_depth(ev).astype(np.float32).astype(np.float64)
    centre = target if primed is None else primed
    expected = oracle.online_class(depth, False, True, centre, TIGHT)
    actual = production(depth, centre)
    test.assertEqual(actual, (expected["class"], expected["support"], expected["occluded"]), (ev["class"], target))
    if primed is None:
        allowed = analysis.required(ev["class"], ev["alone_px"], 2, ev["centre_robust"], ev["centre_blocked"])
        if allowed is not None:
            test.assertIn(actual[0], allowed, (ev["class"], target))
    return ev, actual


def one_pixel_target():
    """Deterministic search for a clear in-frustum target whose support is exactly one pixel."""
    for dist in np.arange(12.75, 23.0, 0.25):
        for y in np.arange(-0.5, 0.51, 0.05):
            for z in np.arange(-0.3, 0.31, 0.05):
                if int(oracle.predicted_support([dist, y, z])["mask"].sum()) == 1:
                    return [float(dist), float(y), float(z)]
    raise AssertionError("no one-pixel target in the preregistered 12.75-23 m band")


class PreregisteredCpuFixtureTest(unittest.TestCase):
    def test_clear_targets_2_to_24_m(self):
        classes = {}
        for d in (2., 5., 10., 16., 20.05, 24.):
            with self.subTest(d=d):
                ev, actual = check(self, [], [], [d, 0., 0.])
                classes[d] = actual[0]
        self.assertEqual(classes[2.], "UNEXPLAINED_LOSS")
        self.assertEqual(classes[24.], "PRED_RANGE_EXIT")

    def test_fov_edge_40_44_50_deg(self):
        seen = {}
        for ang in (40., 44., 50.):
            with self.subTest(angle=ang):
                ev, actual = check(self, [], [], [5 * math.cos(math.radians(ang)), 5 * math.sin(math.radians(ang)), 0.])
                seen[ang] = (ev["class"], actual[0])
        self.assertEqual(seen[50.], ("FOV_EXIT", "PRED_FOV_EXIT"))
        self.assertEqual(seen[40.][1], "UNEXPLAINED_LOSS")

    def test_scale_loss_band_and_precedence_over_occlusion(self):
        target = one_pixel_target()
        _, clear = check(self, [], [], target)
        self.assertEqual(clear[:2], ("PRED_SCALE_LOSS", 1))
        ev, occluded = check(self, [[3.3, target[1], .5]], [[.3, .6, 1.5]], target)
        self.assertEqual(ev["class"], "FULL")
        self.assertEqual(ev["alone_px"], 1)
        self.assertEqual(occluded[0], "PRED_SCALE_LOSS")  # the detector cannot detect one pixel anyway

    def test_full_centre_only_partial_and_rim_gap(self):
        cases = (("FULL", [[3.3, 0., .5]], [[.3, .6, 1.5]], [6., 0., 0.]),
                 ("CENTRE_ONLY", [[2.3, .7, .5]], [[.3, .7, 1.5]], [4., .05, 0.]),
                 ("PARTIAL", [[2.3, .74, .5]], [[.3, .7, 1.5]], [4., 0., 0.]))
        for name, boxes, halves, target in cases:
            with self.subTest(case=name):
                ev, actual = check(self, boxes, halves, target)
                self.assertEqual(ev["class"], name)
        # S14-like rim gap: two bars leave a vertical gap over the target; the evaluator decides
        # the class, the mapping fixes the allowed online classes and production must match.
        ev, actual = check(self, [[2.6 + .3, -.08 - .3, .5], [2.6 + .3, -.01 + .3, .5]], [[.3, .3, 1.5], [.3, .3, 1.5]], [3., 0., 0.])
        self.assertIn(ev["class"], ("CENTRE_ONLY", "PARTIAL"))
        self.assertIn(actual[0], analysis.PARTIAL_TYPES)

    def test_primed_offset_0p2_m(self):
        for boxes, halves, target in (([], [], [5., 0., 0.]), ([[3.3, 0., .5]], [[.3, .6, 1.5]], [6., 0., 0.])):
            with self.subTest(target=target):
                primed = [target[0], .2, 0.]
                check(self, boxes, halves, target, primed=primed)  # production follows the prediction
                moved = oracle.predicted_support(primed)["mask"]
                true = oracle.predicted_support(target)["mask"]
                self.assertTrue(bool((moved != true).any()))  # the support moves with the belief


if __name__ == "__main__":
    unittest.main()
