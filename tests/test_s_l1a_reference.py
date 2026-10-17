"""Independent analytic fixtures for the preregistered S-L1A CPU oracle."""
import ast
import importlib.util
import math
from pathlib import Path
import unittest
import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'tools/audits/s_l1a_reference.py'
SPEC = importlib.util.spec_from_file_location('_s_l1a_reference_test', PATH)
ref = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ref)


class PixelRayTest(unittest.TestCase):
    def test_axis_symmetry_and_independent_oblique_plane(self):
        rays = ref.normalized_pixel_rays(5, 3, 90, 60)
        self.assertEqual(rays.dtype, np.float64)
        np.testing.assert_array_equal(rays[1, 2], [1, 0, 0])
        np.testing.assert_allclose(np.linalg.norm(rays, axis=2), 1, atol=2e-16)
        self.assertGreater(rays[0, 0, 1], 0)
        self.assertGreater(rays[0, 0, 2], 0)
        np.testing.assert_allclose(rays[0, 0], rays[-1, -1]*[1, -1, -1])
        direction = np.array([[math.sqrt(.5), math.sqrt(.5), 0]])
        hit = ref.first_hits([0, 0, 0], direction, [[3, 0, 0]], [[1, 9, 9]])
        self.assertAlmostEqual(hit['depth_m'][0], 2*math.sqrt(2))
        self.assertGreater(hit['depth_m'][0], 2)  # slant, not forward-axis range

    def test_origin_rotation_and_single_pixel_camera(self):
        rays = ref.normalized_pixel_rays(1, 1)
        np.testing.assert_array_equal(rays, [[[1, 0, 0]]])
        result = ref.render_reference([[1, 5, 2]], [[1, 1, 1]], origin=[1, 1, 2],
                                      orientation_xyzw=[0, 0, math.sqrt(.5), math.sqrt(.5)],
                                      width=1, height=1)
        self.assertAlmostEqual(result['depth_m'][0, 0], 3)
        self.assertTrue(result['robust_mask'][0, 0])

    def test_pixel_boundary_robustness_excludes_label_change(self):
        result = ref.render_reference([[3, 1, 0]], [[1, 1, 10]], width=5, height=3)
        self.assertEqual(result['hit_class'][1, 2], ref.BOX)  # closed edge at y=0
        self.assertFalse(result['robust_mask'][1, 2])
        self.assertTrue(result['robust_mask'][1, 0])
        self.assertTrue(result['robust_mask'][1, 4])

    def test_invalid_camera_or_rays_fail(self):
        for kwargs in [dict(width=0), dict(height=2.5), dict(hfov_deg=180), dict(pixel_offset_xy=[np.nan, 0])]:
            with self.assertRaises(ValueError): ref.normalized_pixel_rays(**kwargs)
        with self.assertRaises(ValueError): ref.first_hits([0, 0, 0], [[2, 0, 0]])
        with self.assertRaises(ValueError): ref.rotate_rays([[1, 0, 0]], [0, 0, 0, 2])


class SurfaceIntersectionTest(unittest.TestCase):
    def test_exact_axis_inside_parallel_miss_and_negative(self):
        directions = np.array([[1., 0, 0], [-1., 0, 0], [0, 1., 0], [0, -1., 0]])
        hit = ref.first_hits([0, 0, 0], directions, [[3, 0, 0]], [[1, 1, 1]])
        np.testing.assert_array_equal(hit['depth_m'], [2, 10, 10, 10])
        np.testing.assert_array_equal(hit['hit_index'], [0, -1, -1, -1])
        hit = ref.first_hits([3, 0, 0], directions, [[3, 0, 0]], [[1, 1, 1]])
        np.testing.assert_array_equal(hit['depth_m'], [1, 1, 1, 1])
        miss = ref.first_hits([0, 2, 0], [[1., 0, 0]], [[3, 0, 0]], [[1, 1, 1]])
        self.assertEqual(miss['hit_class'].item(), ref.MISS)

    def test_first_hit_object_order_chunk_and_far(self):
        c = [[6, 0, 0], [3, 0, 0]]
        h = [[1, 1, 1], [.5, 1, 1]]
        hit = ref.first_hits([0, 0, 0], [[1., 0, 0]], c, h, box_chunk=1)
        self.assertEqual(hit['depth_m'].item(), 2.5)
        self.assertEqual(hit['hit_index'].item(), 1)
        miss = ref.first_hits([0, 0, 0], [[1., 0, 0]], c, h, far_m=2.5)
        self.assertEqual(miss['depth_m'].item(), 2.5)
        self.assertEqual(miss['hit_class'].item(), ref.MISS)
        empty = ref.first_hits([0, 0, 0], [[1., 0, 0]])
        self.assertEqual(empty['hit_class'].item(), ref.MISS)

    def test_sphere_roots_inside_tangent_behind_and_box_occluder(self):
        for origin, expected in [([0, 0, 0], 2), ([3, 0, 0], 1), ([3, 1, 0], 0)]:
            hit = ref.first_hits(origin, [[1., 0, 0]], sphere_center=[3, 0, 0], sphere_radius=1)
            self.assertAlmostEqual(hit['depth_m'].item(), expected)
            self.assertEqual(hit['hit_class'].item(), ref.SPHERE)
        hit = ref.first_hits([0, 1, 0], [[1., 0, 0]], sphere_center=[3, 0, 0], sphere_radius=1)
        self.assertAlmostEqual(hit['depth_m'].item(), 3)
        behind = ref.first_hits([0, 0, 0], [[-1., 0, 0]], sphere_center=[3, 0, 0], sphere_radius=1)
        self.assertEqual(behind['hit_class'].item(), ref.MISS)
        front = ref.first_hits([0, 0, 0], [[1., 0, 0]], [[1, 0, 0]], [[.1, 1, 1]], sphere_center=[3, 0, 0])
        self.assertEqual(front['hit_class'].item(), ref.BOX)
        self.assertAlmostEqual(front['depth_m'].item(), .9)


class InterpolationCounterexampleTest(unittest.TestCase):
    def test_bilinear_hand_values(self):
        out = ref.bilinear_resize([[2., 6.]], 1, 4)
        np.testing.assert_array_equal(out, [[2, 3, 5, 6]])

    def test_two_depth_surfaces_create_phantom_and_false_clear(self):
        centers = [[2.1, 5, 0], [6.1, -5, 0]]
        half = [[.1, 5, 10], [.1, 5, 10]]
        exact = ref.render_reference(centers, half)
        coarse = ref.render_reference(centers, half, width=40, height=24)
        interpolated = ref.bilinear_resize(coarse['depth_m'], 90, 160)
        hit = exact['hit_class'] == ref.BOX
        phantom = hit & (abs(interpolated-exact['depth_m']) > .05)
        false_clear = hit & (interpolated > exact['depth_m']+.05)
        self.assertTrue(phantom.any())
        self.assertTrue(false_clear.any())
        # Axis-near foreground/background surfaces are around 2 and 6 m;
        # the interpolated centre pixels belong to neither true ray surface.
        self.assertTrue(((interpolated[45] > 2.2) & (interpolated[45] < 5.8)).any())

    def test_boundary_creates_no_hit_false_positive(self):
        exact = ref.render_reference([[3.1, 5, 0]], [[.1, 5, 10]])
        coarse = ref.render_reference([[3.1, 5, 0]], [[.1, 5, 10]], width=40, height=24)
        interpolated = ref.bilinear_resize(coarse['depth_m'], 90, 160)
        self.assertTrue(((exact['hit_class'] == ref.MISS) & (interpolated < 9.95)).any())

    def test_thin_obstacle_absent_on_coarse_grid_cannot_be_recovered(self):
        exact = ref.render_reference([[8.01, .04745, 0]], [[.01, .01, 1]])
        coarse = ref.render_reference([[8.01, .04745, 0]], [[.01, .01, 1]], width=40, height=24)
        self.assertTrue((exact['hit_class'] == ref.BOX).any())
        self.assertFalse((coarse['hit_class'] == ref.BOX).any())
        interpolated = ref.bilinear_resize(coarse['depth_m'], 90, 160)
        self.assertTrue(((exact['hit_class'] == ref.BOX) & (interpolated > exact['depth_m']+.05)).any())


class ConservativeCarveTest(unittest.TestCase):
    def call(self, scan=None, depth=None, mask=None, visible=True, bearing=0., angles=None, **kwargs):
        depth = np.array([[[3., 3.1, 3.6]]]) if depth is None else np.asarray(depth, dtype=float)
        mask = np.ones_like(depth, bool) if mask is None else np.asarray(mask, dtype=bool)
        scan = np.array([[[2.47, 2.999999, 3., 3.1, 3.55, 12.]]]) if scan is None else np.asarray(scan, dtype=float)
        angles = np.zeros(scan.shape[-1]) if angles is None else np.asarray(angles)
        return ref.carve_masks(scan, angles, depth, mask, np.array([visible]), np.array([bearing]), **kwargs)

    def test_s14_foreground_zero_inward_and_strict_upper_window(self):
        out = self.call()
        self.assertEqual(out['anchor_m'].item(), 3)
        np.testing.assert_array_equal(out['lidar_carve_mask'], [[[False, False, True, True, False, False]]])
        np.testing.assert_array_equal(out['camera_blank_mask'], [[[True, True, False]]])
        self.assertAlmostEqual(out['angular_limit_rad'].item(), math.asin(.15/3.15))
        # Legacy would remove the 0.53 m foreground return; independent new rule retains it.
        self.assertLess(abs(2.47-3.), .55)
        self.assertFalse(out['lidar_carve_mask'][0, 0, 0])

    def test_angular_extent_and_wrap_at_pi(self):
        extent = math.asin(.15/3.15)
        out = self.call(scan=[[[3.1]*4]], angles=[0, extent-.0001, extent+.0001, math.radians(14)])
        np.testing.assert_array_equal(out['lidar_carve_mask'], [[[True, True, False, False]]])
        out = self.call(scan=[[[3.1]]], angles=[-math.pi+.01], bearing=math.pi-.01)
        self.assertTrue(out['lidar_carve_mask'].item())

    def test_insufficient_or_invalid_camera_and_fused_only_cannot_authorize(self):
        cases = [dict(visible=False), dict(mask=[[[True, False, False]]]),
                 dict(depth=[[[0, np.nan, np.inf]]]), dict(depth=[[[20, 21, -1]]]),
                 dict(bearing=np.nan)]
        for case in cases:
            with self.subTest(case=case):
                out = self.call(**case)
                self.assertFalse(out['supported'].item())
                self.assertEqual(out['anchor_m'].item(), 0)
                self.assertFalse(out['lidar_carve_mask'].any())
                self.assertFalse(out['camera_blank_mask'].any())
                self.assertTrue(np.isfinite(out['angular_limit_rad']).all())

    def test_invalid_lidar_and_no_return_never_removed(self):
        out = self.call(scan=[[[0, -1, np.nan, np.inf, 12, 13, 3.1]]])
        np.testing.assert_array_equal(out['lidar_carve_mask'], [[[False]*6+[True]]])

    def test_anchor_is_valid_camera_minimum_not_mean_or_lidar_estimate(self):
        out = self.call(depth=[[[np.nan, 3.8, 2.8, 0, 20]]], scan=[[[2.79, 2.8, 3.34, 3.35, 3.8]]])
        self.assertTrue(out['supported'].item())
        self.assertAlmostEqual(out['anchor_m'].item(), 2.8)
        np.testing.assert_array_equal(out['camera_blank_mask'], [[[False, False, True, False, False]]])
        np.testing.assert_array_equal(out['lidar_carve_mask'], [[[False, True, True, False, False]]])

    def test_batch_inputs_unchanged_and_no_rng_or_production_imports(self):
        state = np.random.get_state()
        self.call()
        after = np.random.get_state()
        self.assertEqual(state[0], after[0])
        np.testing.assert_array_equal(state[1], after[1])
        self.assertEqual(state[2:], after[2:])
        source = ast.parse(PATH.read_text())
        modules = [n.module for n in ast.walk(source) if isinstance(n, ast.ImportFrom)]
        modules += [alias.name for n in ast.walk(source) if isinstance(n, ast.Import) for alias in n.names]
        self.assertEqual(set(modules), {'argparse', 'json', 'math', 'numpy'})
        scan = np.array([[[3.1]], [[3.1]]]);copy = scan.copy()
        out = ref.carve_masks(scan, [0], np.array([[[3, 3]], [[3, 3]]]), np.ones((2,1,2),bool), np.array([True, False]), [0,0])
        np.testing.assert_array_equal(scan, copy)
        np.testing.assert_array_equal(out['supported'], [True,False])


if __name__ == '__main__':
    unittest.main()
