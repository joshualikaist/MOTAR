"""P0 independent geometric counterexamples; no simulator, GT online input or raw reads.

Evaluation fixtures may name true object roles. The online functions receive only
the same indistinguishable measured points/belief, so cannot prove their identities.
"""
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import geometry as geom  # noqa: E402
from motar_eval.boia import visibility as vis  # noqa: E402
from motar_eval.boia.types import BoiaParams, CameraParams, FreeContext  # noqa: E402

for module in (geom, vis):
    assert str(Path(module.__file__).resolve()).startswith(str(ROOT) + "/")

DT = torch.float64


def angles():
    return math.pi - torch.arange(72, dtype=DT) * (2 * math.pi / 72)


def scalar_distance(a, b, point):
    """Independent scalar projection oracle, with ordinary Python arithmetic."""
    vx, vy = b[0] - a[0], b[1] - a[1]
    dx, dy = point[0] - a[0], point[1] - a[1]
    length2 = vx * vx + vy * vy
    fraction = max(0., min(1., (dx * vx + dy * vy) / length2)) if length2 else 0.
    return math.hypot(point[0] - a[0] - fraction * vx, point[1] - a[1] - fraction * vy)


def retained(point, mean=(6., 0.), covariance=0.):
    pts = torch.tensor([[point]], dtype=DT)
    valid = torch.ones(1, 1, dtype=torch.bool)
    mu = torch.tensor([mean], dtype=DT)
    sigma = torch.eye(2, dtype=DT)[None] * covariance
    return geom.exclude_target(pts, valid, mu, sigma, .15)


class ExclusionCounterexamples(unittest.TestCase):
    def test_positive_actual_target_is_retained_with_abstention_tradeoff(self):
        self.assertTrue(bool(retained((5.85, 0.))))

    def test_near_foreground_bar_is_not_erased_by_zero_covariance_gate(self):
        # Published gate radius is sqrt(9.21)*.15=.45522m even at Sigma=0.
        self.assertLess(.2, math.sqrt(9.21) * .15)
        self.assertTrue(bool(retained((5.8, 0.))))

    def test_eq1d_style_bar_point53m_before_belief_is_preserved(self):
        self.assertLess(.53 ** 2 / (.01 + .15 ** 2), 9.21)
        self.assertTrue(bool(retained((5.47, 0.), covariance=.01)))

    def test_large_belief_covariance_cannot_erase_far_foreground(self):
        self.assertTrue(bool(retained((3., 0.), covariance=4.)))

    def test_false_association_on_bar_has_no_identity_privilege(self):
        self.assertTrue(bool(retained((3., .2), mean=(3., .2), covariance=.01)))

    def test_camera_lidar_depth_mismatch_is_conservatively_preserved(self):
        self.assertLess(2. ** 2 / (.5 + .15 ** 2), 9.21)
        self.assertTrue(bool(retained((4., 0.), mean=(6., 0.), covariance=.5)))

    def test_return_order_invalid_mask_and_source_arrays_are_preserved(self):
        points = torch.tensor([[[5.8, 0.], [5.85, 0.], [6., 0.], [3., 1.]]], dtype=DT)
        valid = torch.tensor([[True, True, False, True]])
        saved = points.clone(), valid.clone()
        result = geom.exclude_target(points, valid, torch.tensor([[6., 0.]], dtype=DT),
                                     torch.eye(2, dtype=DT)[None] * .1, .15)
        self.assertTrue(torch.equal(result, valid))
        result.zero_()
        self.assertTrue(torch.equal(points, saved[0]))
        self.assertTrue(torch.equal(valid, saved[1]))


class HorizontalFreeCounterexamples(unittest.TestCase):
    def test_default_declared_four_rows_use_horizontal_first_hit(self):
        rho = torch.full((1, 288), 12., dtype=DT)
        rho[0, 36] = 5.
        radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),
                                 torch.zeros(1, dtype=DT), 12., 0.)
        expected = 5 * math.cos(math.radians(20))
        self.assertAlmostEqual(float(radius[0, 36]), expected, places=12)
        query = torch.tensor([[[4.85, 0.]]], dtype=DT)
        self.assertFalse(bool(geom.point_in_known_free(query, torch.zeros(1, 3, dtype=DT),
                              torch.zeros(1, dtype=DT), radius, angles())))

    def test_row_minimum_is_taken_after_projection_not_before(self):
        rho = torch.full((1, 288), 12., dtype=DT)
        rho[0, 36], rho[0, 2 * 72 + 36] = 5., 4.9
        radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),
                                 torch.zeros(1, dtype=DT), 12., .1)
        self.assertAlmostEqual(float(radius[0, 36]), 5 * math.cos(math.radians(20)) - .1, places=12)

    def test_explicit_zero_and_positive_negative_elevations(self):
        for alpha in (0., 10., 20., -10., -20.):
            with self.subTest(alpha=alpha):
                rho = torch.full((1, 72), 5., dtype=DT)
                radius = geom.known_free(rho, angles(), torch.zeros(1, 3, dtype=DT),
                    torch.zeros(1, dtype=DT), 12., .1,
                    elevations=torch.tensor([math.radians(alpha)], dtype=DT))
                self.assertAlmostEqual(float(radius[0, 36]), 5 * math.cos(math.radians(alpha)) - .1, places=12)

    def test_undeclared_nonstandard_rows_fail_closed(self):
        with self.assertRaises(ValueError):
            geom.known_free(torch.ones(1, 72, dtype=DT), angles(), torch.zeros(1, 3, dtype=DT),
                            torch.zeros(1, dtype=DT), 12., .1)

    def test_invalid_and_nearzero_ranges_cannot_expand_free_space(self):
        rho = torch.full((1, 288), 12., dtype=DT)
        rho[0, :4] = torch.tensor([float("nan"), 1e-12, 0., -1.], dtype=DT)
        position = torch.tensor([[2., 3., 1.]], dtype=DT)
        elevation = torch.tensor([20., 10., 0., -10.], dtype=DT) * (math.pi / 180)
        points, valid = geom.lidar_points_xy(rho, angles(), elevation, position,
                                            torch.zeros(1, dtype=DT), 12.)
        self.assertEqual(valid[0, :4].tolist(), [False, True, False, False])
        self.assertTrue(bool(torch.isfinite(points).all()))
        radius = geom.known_free(rho, angles(), position, torch.zeros(1, dtype=DT), 12., .1)
        self.assertTrue(bool((radius[0, :5] == 0).all()))


class ContextInputCounterexamples(unittest.TestCase):
    def test_invalid_context_row_never_certifies_point_or_segment(self):
        # Row 1 is an independent valid control; a bad sector invalidates row 0.
        cases = [("radius", x) for x in (float("nan"), float("inf"), -1., 12.01)]
        cases += [("yaw", x) for x in (float("nan"), float("inf"))]
        cases += [("position_x", float("nan")), ("position_z", float("nan")),
                  ("position_z", float("inf"))]
        for field, value in cases:
            with self.subTest(field=field, value=value):
                pos = torch.zeros(2, 3, dtype=DT)
                yaw = torch.zeros(2, dtype=DT)
                radius = torch.full((2, 72), 3., dtype=DT)
                if field == "radius":
                    radius[0, 0] = value  # Not necessarily the queried sector.
                elif field == "yaw":
                    yaw[0] = value
                else:
                    pos[0, 0 if field == "position_x" else 2] = value
                context = FreeContext(pos, yaw, radius, angles(), 12.)
                q = torch.zeros(2, 2, 2, dtype=DT)
                q[:, 1, 0] = 1.
                expected = torch.tensor([[False, False], [True, True]])
                results = {"point": geom.point_in_known_free(q, pos, yaw, radius, angles(), 12.),
                           "segment": geom.segment_in_known_free(torch.zeros_like(q), q, context)}
                for method, actual in results.items():
                    with self.subTest(method=method):
                        self.assertTrue(torch.equal(actual, expected), str(actual))

    def test_nonfinite_or_nonpositive_far_never_certifies_free(self):
        pos, yaw = torch.zeros(1, 3, dtype=DT), torch.zeros(1, dtype=DT)
        radius, q = torch.ones(1, 72, dtype=DT), torch.zeros(1, 1, 2, dtype=DT)
        for far in (float("nan"), float("inf"), -1., 0.):
            with self.subTest(far=far):
                context = FreeContext(pos, yaw, radius, angles(), far)
                results = {"point": geom.point_in_known_free(q, pos, yaw, radius, angles(), far),
                           "segment": geom.segment_in_known_free(q, q, context)}
                for method, actual in results.items():
                    with self.subTest(method=method):
                        self.assertFalse(bool(actual))


class IndependentHelperChecks(unittest.TestCase):
    def test_fov_and_camera_transform_use_declared_yaw_frame(self):
        # Stay 0.01m from the exact floating-point tan(pi/4) boundary.
        rel = torch.tensor([[1., .99, 0.], [1., 1.01, 0.], [-1., 0., 0.],
                            [1., 0., float("nan")]], dtype=DT)
        self.assertEqual(vis.fov_mask(rel, math.pi / 2, math.pi / 2, 10.).tolist(),
                         [True, False, False, False])
        camera = CameraParams(math.pi / 2, math.pi / 2, 80., 80., 0., 0., (.1, .2, .03))
        transformed = vis._to_camera(torch.tensor([[[0., 2., 1.]]], dtype=DT),
                                     torch.tensor([math.pi / 2], dtype=DT), camera)
        torch.testing.assert_close(transformed, torch.tensor([[[1.9, -.2, .97]]], dtype=DT),
                                   atol=1e-14, rtol=0)

    def test_union_is_computed_per_joint_path_before_averaging(self):
        p = torch.tensor([[[[.2, .3], [1., 0.]]]], dtype=DT)
        expected = ((1 - .8 * .7) + 1) / 2
        self.assertAlmostEqual(float(vis.p_any(p)), expected, places=14)
        marginal_union = 1 - (1 - .6) * (1 - .15)
        self.assertNotAlmostEqual(expected, marginal_union)

    def test_normal_clearance_matches_scalar_gaussian_and_strict_zero(self):
        g = torch.tensor([-1., 0., 1.], dtype=DT)
        grad = torch.tensor([[1., 0., 0.]] * 3, dtype=DT)
        cov = torch.eye(3, dtype=DT).expand(3, 3, 3) * 4
        actual = vis._normal_clearance(g, grad, cov)
        expected = torch.tensor([(1 + math.erf(x / (2 * math.sqrt(2)))) / 2
                                 for x in (-1., 0., 1.)], dtype=DT)
        torch.testing.assert_close(actual, expected, atol=1e-14, rtol=0)
        actual = vis._normal_clearance(torch.zeros(2, dtype=DT), grad[:2],
                                      torch.zeros(2, 3, 3, dtype=DT),
                                      torch.tensor([False, True]))
        self.assertEqual(actual.tolist(), [1., 0.])

    def test_sector_helper_uses_minimum_on_boundary(self):
        a = torch.tensor([math.pi, math.pi / 2, 0., -math.pi / 2], dtype=DT)
        radius = torch.tensor([[1., 2., 3., 4.]], dtype=DT)
        q = torch.tensor([[[1., 0.], [1., 1.], [0., 1.], [0., -1.]]], dtype=DT)
        actual = geom._radii(q, torch.zeros(1, dtype=DT), radius, a)
        self.assertEqual(actual.tolist(), [[3., 2., 2., 4.]])


class VisibilityOriginsAndModelLimits(unittest.TestCase):
    def test_camera_offset_los_counterexample_forecast_and_bpod(self):
        ego, target, obstacle = (0., 0.), (4., 2.), (2., .6)
        self.assertGreater(scalar_distance(ego, target, obstacle), .35)
        self.assertLess(scalar_distance((.1, 0.), target, obstacle), .35)
        points = torch.tensor([[obstacle]], dtype=DT)
        valid = torch.ones(1, 1, dtype=torch.bool)
        camera = CameraParams(math.radians(87), math.radians(58), 80., 80., 79.5, 44.5, (.1, 0., .03))
        pos = torch.zeros(1, 1, 1, 3, dtype=DT)
        yaw = torch.zeros(1, 1, 1, dtype=DT)
        sample = torch.tensor([[[[4., 2., 0., 0., 0., 0.]]]], dtype=DT)
        params = BoiaParams(tau_on=.1, r_bar_est=.2, unknown_blocks=False)
        values = {"forecast": vis.forecast(pos, yaw, sample, points, valid, None, camera, params),
                  "bpod": vis.bpod_analytic(pos[:, :, 0], yaw[:, :, 0], sample[:, 0, 0, :3],
                    torch.zeros(1, 3, 3, dtype=DT), points, valid, .35, camera)}
        for method, value in values.items():
            with self.subTest(method=method):
                self.assertEqual(float(value), 0.)

    def test_yaw_rotated_camera_origin_and_no_double_offset(self):
        # Rotate the preceding independent world fixture by +90 degrees.
        points = torch.tensor([[[-.6, 2.]]], dtype=DT)
        valid = torch.ones(1, 1, dtype=torch.bool)
        camera = CameraParams(math.radians(87), math.radians(58), 80., 80., 79.5, 44.5, (.1, 0., .03))
        pos = torch.zeros(1, 1, 1, 3, dtype=DT)
        yaw = torch.full((1, 1, 1), math.pi / 2, dtype=DT)
        sample = torch.tensor([[[[-2., 4., 0., 0., 0., 0.]]]], dtype=DT)
        params = BoiaParams(tau_on=.1, r_bar_est=.2, unknown_blocks=False)
        self.assertEqual(float(vis.forecast(pos, yaw, sample, points, valid, None, camera, params)), 0.)
        # In the absence of obstacle evidence the FOV transform remains correct.
        self.assertEqual(float(vis.forecast(pos, yaw, sample, points, valid & False, None, camera, params)), 1.)

    def test_six_metre_target_with_first_hit5p8_remains_blocked(self):
        position = torch.zeros(1, 3, dtype=DT)
        yaw = torch.zeros(1, dtype=DT)
        rho = torch.full((1, 288), 12., dtype=DT)
        rho[0, 2 * 72 + 36] = 5.8
        elevation = torch.tensor([20., 10., 0., -10.], dtype=DT) * (math.pi / 180)
        points, valid = geom.lidar_points_xy(rho, angles(), elevation, position, yaw, 12.)
        valid = geom.exclude_target(points, valid, torch.tensor([[6., 0.]], dtype=DT),
                                    torch.zeros(1, 2, 2, dtype=DT), .15)
        radius = geom.known_free(rho, angles(), position, yaw, 12., .1)
        context = FreeContext(position, yaw, radius, angles(), 12.)
        result = vis.los_clear(position[:, :2], torch.tensor([[6., 0.]], dtype=DT),
                               points, valid, .35, context, True)
        self.assertFalse(bool(result))
        self.assertLess(float(radius[0, 36]), 6.)

    def test_sparse_scan_sectors_are_not_a_physical_free_certificate(self):
        # An unseen 0.2m-radius obstacle between two 5deg horizontal rays.
        # This checks a retained model LIMITATION, not a qualified free-space claim.
        centre = (5 * math.cos(math.radians(2.5)), 5 * math.sin(math.radians(2.5)))
        self.assertGreater(5 * math.sin(math.radians(2.5)), .2)
        rho = torch.full((1, 288), 12., dtype=DT)
        position, yaw = torch.zeros(1, 3, dtype=DT), torch.zeros(1, dtype=DT)
        radius = geom.known_free(rho, angles(), position, yaw, 12., .1)
        self.assertTrue(bool(geom.point_in_known_free(torch.tensor([[centre]], dtype=DT),
                                                      position, yaw, radius, angles())))

    def test_segment_distance_zero_length_and_partition_independent_examples(self):
        points = torch.tensor([[[1., 1.], [-2., 3.]]], dtype=DT)
        a = torch.tensor([[[0., 0.], [1., 2.]]], dtype=DT)
        b = torch.tensor([[[4., 0.], [1., 2.]]], dtype=DT)
        result = geom.seg_point_dist(a, b, points)
        for i in range(2):
            for j in range(2):
                self.assertAlmostEqual(float(result[0, i, j]), scalar_distance(a[0, i].tolist(),
                                       b[0, i].tolist(), points[0, j].tolist()), places=12)
        context = FreeContext(torch.zeros(1, 3, dtype=DT), torch.zeros(1, dtype=DT),
                              torch.full((1, 72), 3., dtype=DT), angles(), 12.)
        self.assertTrue(bool(geom.segment_in_known_free(torch.tensor([[[-2., 0.]]], dtype=DT),
                              torch.tensor([[[2., 0.]]], dtype=DT), context)))
        self.assertFalse(bool(geom.segment_in_known_free(torch.tensor([[[-2., 0.]]], dtype=DT),
                               torch.tensor([[[4., 0.]]], dtype=DT), context)))


if __name__ == "__main__":
    unittest.main()
