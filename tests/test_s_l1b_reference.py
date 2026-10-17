"""Self-checks of the independent S-L1B float64 oracle against hand-derived geometry (CPU)."""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import s_l1b_reference as ref  # noqa: E402

STEP = 2 * math.pi / 72


class ReprojectionTest(unittest.TestCase):
    def test_wall_points_lie_on_the_wall_for_any_mount(self):
        for rotation, translation in ((np.eye(3), ref.CAMERA_TRANSLATION),
                                      (ref.rotation_matrix(pitch=math.radians(10), yaw=math.radians(5)), (.2, .05, -.04))):
            with self.subTest(translation=translation):
                surf = ref.true_surface([[5.1, 0, 0]], [[.1, 20, 20]], rotation=rotation, translation=translation)
                cam = ref.camera_points(surf["depth_m"], rotation=rotation, translation=translation)
                valid = cam["valid"][0]
                self.assertGreater(valid.sum(), 10000)
                np.testing.assert_allclose(cam["points"][0][valid][:, 0], 5.0, atol=1e-12, rtol=0)
                np.testing.assert_allclose(cam["rho"][0][valid], surf["rho"][valid], atol=1e-12, rtol=0)

    def test_hand_point_range_bearing_elevation(self):
        depth = np.full((90, 160), 10.0)
        depth[44, 79] = 2.0  # one pixel just above-left of the optical axis
        cam = ref.camera_points(depth)
        fx, fy = 160 / (2 * math.tan(math.radians(43.5))), 90 / (2 * math.tan(math.radians(29)))
        n = np.array([1, -(79 - 79.5) / fx, -(44 - 44.5) / fy])
        p = np.array([.10, 0, .03]) + 2.0 * n / np.linalg.norm(n)
        self.assertEqual(int(cam["valid"].sum()), 1)
        self.assertAlmostEqual(cam["rho"][0, 44, 79], np.linalg.norm(p), places=12)
        self.assertAlmostEqual(cam["az"][0, 44, 79], math.atan2(p[1], p[0]), places=12)
        self.assertAlmostEqual(cam["el"][0, 44, 79], math.atan2(p[2], math.hypot(p[0], p[1])), places=12)

    def test_validity_conventions(self):
        depth = np.array([[[9.9989, 9.9991, 0., -1., np.nan, np.inf, 3.0, 3.0]]])
        blank = np.array([[[False] * 7 + [True]]])
        cam = ref.camera_points(depth, blank=blank, hfov_deg=87., vfov_deg=58.)
        self.assertEqual(cam["valid"][0, 0].tolist(), [True, False, False, False, False, False, True, False])


class BinConventionTest(unittest.TestCase):
    def test_azimuth_and_row_indices(self):
        az = np.array([math.pi, math.pi - STEP, 0., -math.pi + STEP, -math.pi])
        j, _ = ref.bin_index(az, np.zeros(5))
        self.assertEqual(j.tolist(), [0, 1, 36, 71, 0])
        _, e = ref.bin_index(np.zeros(6), np.radians([20, 10, 0, -10, 30, -20]))
        self.assertEqual(e.tolist(), [0, 1, 2, 3, -1, 4])

    def test_strict_and_clamped_bins(self):
        cam = {"rho": np.array([[3., 2., 4., 5.]]), "valid": np.array([[True, True, True, True]]),
               "az": np.zeros((1, 4)), "el": np.radians([[0., 30., -20., 1.]])}
        strict = ref.lidar_bins(cam, clamp=False)
        clamped = ref.lidar_bins(cam, clamp=True)
        self.assertEqual(strict[0, :, 36].tolist(), [12., 12., 3., 12.])
        self.assertEqual(clamped[0, :, 36].tolist(), [2., 12., 3., 4.])
        self.assertTrue((strict[0, :, :36] == 12).all())

    def test_boundary_point_is_not_robust(self):
        cam = {"rho": np.array([[3., 4.]]), "valid": np.array([[True, True]]),
               "az": np.array([[STEP / 2 + 1e-9, -STEP / 2 * .5]]), "el": np.zeros((1, 2))}
        base, robust = ref.robust_bins(cam, clamp=True)
        self.assertFalse(robust[0, 2, 35] and robust[0, 2, 36])
        self.assertTrue(robust[0, 0, 10])


class CarveTest(unittest.TestCase):
    def inputs(self, bar_range):
        surf = ref.true_surface(sphere_center=[3.0, 0, 0], sphere_radius=.15, far_m=20.)
        depth = surf["depth_m"][None]
        candidates = (surf["hit_class"] == 2)[None]
        angles = np.linspace(math.pi, -math.pi + STEP, 72)
        scan = np.full((1, 4, 72), 12.0)
        scan[0, 2, 36] = bar_range
        return scan, angles, depth, candidates

    def test_b_tight_bar_retained_in_common_frame_but_inside_s_l1a_window(self):
        scan, angles, depth, candidates = self.inputs(2.77)
        visible, bearing = np.array([True]), np.array([0.])
        new = ref.common_frame_carve_masks(scan, angles, depth, candidates, visible, bearing)
        old = ref.ref.carve_masks(scan, angles, depth, candidates, visible, bearing, target_radius=.15, camera_far_m=20.)
        self.assertTrue(new["supported"][0])
        self.assertAlmostEqual(new["lidar_surface_lower_m"][0], 2.80, delta=3e-3)
        self.assertGreaterEqual(new["lidar_surface_lower_m"][0], 2.80 - 1e-9)
        self.assertFalse(new["lidar_carve_mask"][0, 2, 36])
        self.assertTrue(old["lidar_carve_mask"][0, 2, 36])  # S-L1A camera-frame window starts near 2.75 m

    def test_proxy_surface_return_is_carved_and_nearer_never(self):
        for value, carved in ((2.805, True), (2.79, False), (3.34, True), (3.36, False)):
            with self.subTest(value=value):
                scan, angles, depth, candidates = self.inputs(value)
                masks = ref.common_frame_carve_masks(scan, angles, depth, candidates, np.array([True]), np.array([0.]))
                self.assertEqual(bool(masks["lidar_carve_mask"][0, 2, 36]), carved)

    def test_amendment1_rows_outside_camera_target_elevations_are_never_carved(self):
        scan, angles, depth, candidates = self.inputs(2.77)
        scan[0, 1, 36] = scan[0, 3, 36] = 2.813  # tall bar seen by the +-10 deg rows
        scan[0, 0, 36] = 2.948
        visible, bearing = np.array([True]), np.array([0.])
        new = ref.common_frame_carve_masks(scan, angles, depth, candidates, visible, bearing)
        old = ref.ref.carve_masks(scan, angles, depth, candidates, visible, bearing, target_radius=.15, camera_far_m=20.)
        self.assertFalse(new["lidar_carve_mask"][0, :, 36].any())
        self.assertTrue(old["lidar_carve_mask"][0, [0, 1, 3], 36].all())  # recorded S-L1A limitation
        scan[0, 2, 36] = 2.81  # the on-axis target proxy return remains carve-eligible
        self.assertTrue(ref.common_frame_carve_masks(scan, angles, depth, candidates, visible, bearing)["lidar_carve_mask"][0, 2, 36])

    def test_unsupported_evidence_never_carves(self):
        scan, angles, depth, candidates = self.inputs(2.81)
        for visible, cand in ((False, candidates), (True, np.zeros_like(candidates))):
            masks = ref.common_frame_carve_masks(scan, angles, depth, cand, np.array([visible]), np.array([0.]))
            self.assertFalse(masks["lidar_carve_mask"].any())
            self.assertFalse(masks["camera_blank_mask"].any())


class FixtureTest(unittest.TestCase):
    def test_pinhole_20_degree_counterexample_value(self):
        self.assertAlmostEqual(ref.pinhole_column(math.radians(20)), 48.8164, places=4)

    def test_grazing_flag_on_oblique_wall(self):
        # A wall almost parallel to the optical axis is grazing at the image edge only.
        surf = ref.true_surface([[10.0, 1.0, 0]], [[10.0, .01, 5]])
        self.assertTrue(surf["grazing"].any())
        front = ref.true_surface([[5.1, 0, 0]], [[.1, 20, 20]])
        self.assertFalse(front["grazing"][45, 80])

    def test_self_test_and_no_production_import(self):
        self.assertTrue(all(ref.self_test().values()))
        tree = ast.parse((ROOT / "tools/audits/s_l1b_reference.py").read_text())
        names = {a.name for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom)) for a in n.names}
        modules = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        self.assertFalse(any("aerial_gym" in x or "torch" in x for x in names | {m for m in modules if m}))


if __name__ == "__main__":
    unittest.main()
