"""Independent analytic C02 planar-model checks; no simulator or physical claim."""
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import geometry as geom
from motar_eval.boia.types import ContainmentReason, FreeBoundary2D, FreeContext, SafetyStatus

assert str(Path(geom.__file__).resolve()).startswith(str(ROOT) + "/")
DT = torch.float64


def boundary(n=1, radius=3., bins=72, dtype=DT):
    angles = math.pi - torch.arange(bins, dtype=dtype) * (2 * math.pi / bins)
    return FreeBoundary2D(FreeContext(torch.zeros(n, 3, dtype=dtype), torch.zeros(n, dtype=dtype),
                          torch.full((n, bins), radius, dtype=dtype), angles, 12.))


def point(x, y=0., dtype=DT):
    return torch.tensor([[x, y]], dtype=dtype)


def notch_boundary():
    model = boundary()
    model.context.free_radius[0, 35] = .5  # Missing wedge +2.5..+7.5 degrees.
    return model


class CapsuleGeometryTest(unittest.TestCase):
    def test_center_only_false_safe_and_exact_radial_step_distance(self):
        model = notch_boundary()
        centre = point(1.)
        # Independent geometry: centre bearing0 is in the3m sector, but body
        # point(1,.1) has bearing5.7106deg and radius>the0.5m wedge extent.
        self.assertGreater(math.degrees(math.atan2(.1, 1.)), 2.5)
        self.assertLess(math.degrees(math.atan2(.1, 1.)), 7.5)
        self.assertGreater(math.hypot(1., .1), .5)
        result = geom.capsule_in_free_model(centre, centre, model, .201990099)
        self.assertEqual(int(result.status), int(SafetyStatus.UNKNOWN))
        self.assertEqual(int(result.reason), int(ContainmentReason.UNKNOWN_SPACE))
        self.assertAlmostEqual(float(result.min_clearance_m), math.sin(math.radians(2.5)) - .201990099, places=12)

    def test_body_radius_subtraction_does_not_handle_radial_steps(self):
        model = notch_boundary()
        a, b = point(.8), point(1.2)
        result = geom.capsule_in_free_model(a, b, model, .1)
        # Point radius<3-.1 everywhere on this line, yet the adjacent unknown
        # wedge is only .8*sin(2.5deg) from the closest segment endpoint.
        expected = .8 * math.sin(math.radians(2.5)) - .1
        self.assertAlmostEqual(float(result.min_clearance_m), expected, places=12)
        self.assertFalse(bool(result.model_safe))

    def test_nonconvex_notch_all_box_corners_pass_but_face_fails(self):
        # Four corners of this box avoid the notch. Its right face crosses it.
        corners = ((.8, -.2), (.8, .2), (1.2, -.2), (1.2, .2))
        for x, y in corners:
            self.assertGreater(abs(math.degrees(math.atan2(y, x))), 7.5)
            self.assertLess(math.hypot(x, y), 3.)
        face_angle = math.degrees(math.atan2(.1, 1.2))
        self.assertTrue(2.5 < face_angle < 7.5)
        self.assertGreater(math.hypot(1.2, .1), .5)
        result = geom.capsule_in_free_model(point(1.), point(1.), notch_boundary(), math.hypot(.2, .2))
        self.assertFalse(bool(result.model_safe))

    def test_uniform_disk_has_no_artificial_internal_ray_boundary(self):
        for degrees in (0., 2.5, 5., 90., 177.5):
            with self.subTest(degrees=degrees):
                a = point(1.5 * math.cos(math.radians(degrees)), 1.5 * math.sin(math.radians(degrees)))
                result = geom.capsule_in_free_model(a, a, boundary(radius=2.), .2)
                self.assertTrue(bool(result.model_safe))
                self.assertAlmostEqual(float(result.min_clearance_m), .3, places=12)
        tangent = geom.capsule_in_free_model(point(1.), point(1.), boundary(radius=2.), 1.)
        self.assertEqual(int(tangent.status), int(SafetyStatus.UNKNOWN))
        self.assertEqual(float(tangent.min_clearance_m), 0.)  # Numerical ambiguity abstains.

    def test_disk_capsule_against_independent_convex_endpoint_theorem(self):
        g = torch.Generator().manual_seed(20261020)
        a = torch.rand(2, 31, 2, generator=g, dtype=DT) * 5 - 2.5
        b = torch.rand(2, 31, 2, generator=g, dtype=DT) * 5 - 2.5
        radii = torch.rand(2, 31, generator=g, dtype=DT) * .3
        result = geom.capsule_in_free_model(a, b, boundary(2, radius=3.), radii)
        for e in range(2):
            for j in range(31):
                max_norm = max(math.hypot(*a[e, j].tolist()), math.hypot(*b[e, j].tolist()))
                expected = max_norm + float(radii[e, j]) <= 3.
                self.assertEqual(bool(result.model_safe[e, j]), expected)
                if max_norm <= 3.:
                    self.assertAlmostEqual(float(result.min_clearance_m[e, j]), 3 - max_norm - float(radii[e, j]), places=11)

    def test_roundoff_scale_positive_margin_and_step_tangency_abstain(self):
        for radius, safe in ((1. - 1e-14, False), (1. - 1e-8, True), (1. + 1e-8, False)):
            result = geom.capsule_in_free_model(point(1.), point(1.), boundary(radius=2.), radius)
            self.assertEqual(bool(result.model_safe), safe)
        radius = math.sin(math.radians(2.5))
        result = geom.capsule_in_free_model(point(1.), point(1.), notch_boundary(), radius)
        self.assertEqual(int(result.status), int(SafetyStatus.UNKNOWN))

    def test_sector_crossing_between_free_endpoints_is_rejected(self):
        # Endpoints at +/-45deg are outside the missing5deg notch; the line
        # crosses it at y=tan(5deg), x=1. This is not an endpoint proof.
        result = geom.capsule_in_free_model(point(1., -1.), point(1., 1.), notch_boundary(), 0.)
        self.assertEqual(int(result.status), int(SafetyStatus.UNKNOWN))

    def test_zero_radius_zero_length_and_unknown_only_model(self):
        model = boundary(radius=0.)
        for r, expected in ((0., SafetyStatus.MODEL_SAFE), (.2, SafetyStatus.UNKNOWN)):
            result = geom.capsule_in_free_model(point(0.), point(0.), model, r)
            self.assertEqual(int(result.status), int(expected))
        self.assertTrue(bool(geom.capsule_in_free_model(point(0.), point(1.), boundary(), 0.).model_safe))
        self.assertFalse(bool(geom.capsule_in_free_model(point(0.), point(4.), boundary(), 0.).model_safe))

    def test_narrow_gap_model_exact_origin_disk_and_radius_tradeoff(self):
        model = boundary()
        half_step = math.pi / 72
        phi = model.context.angles
        maximum_sine = torch.maximum((phi - half_step).sin().abs(), (phi + half_step).sin().abs())
        maximum_sine[18] = maximum_sine[54] = 1.
        model.context.free_radius[0] = (.275 / maximum_sine).clamp(max=12.)
        # The radial minimum is .275, so the largest origin-centred contained
        # disk has radius .275 independent of all internal sector boundaries.
        for body_radius, safe in ((.201990099, True), (.28, False)):
            result = geom.capsule_in_free_model(point(0.), point(0.), model, body_radius)
            self.assertEqual(bool(result.model_safe), safe)
            self.assertAlmostEqual(float(result.min_clearance_m), .275 - body_radius, places=12)

    def test_yaw_translation_reversal_and_per_env_radius_broadcast(self):
        model = boundary(2)
        model.context.free_radius[1, 35] = .5
        model.context.pos[1, :2] = torch.tensor([2., -3.], dtype=DT)
        model.context.yaw[1] = math.pi / 2
        a = torch.tensor([[[.8, 0.], [0., 0.]], [[2., -2.2], [2., -3.]]], dtype=DT)
        b = torch.tensor([[[1.2, 0.], [1., 0.]], [[2., -1.8], [2., -2.]]], dtype=DT)
        r = torch.tensor([.1, .1], dtype=DT)
        forward = geom.capsule_in_free_model(a, b, model, r)
        reverse = geom.capsule_in_free_model(b, a, model, r)
        self.assertTrue(torch.equal(forward.status, reverse.status))
        torch.testing.assert_close(forward.min_clearance_m, reverse.min_clearance_m, atol=1e-12, rtol=0)
        self.assertTrue(bool(forward.model_safe[0].all()))
        self.assertFalse(bool(forward.model_safe[1, 0]))

    def test_nonfinite_query_and_context_isolation(self):
        model = boundary(3)
        model.context.free_radius[0, 0] = float("inf")
        model.context.yaw[1] = float("nan")
        a = torch.zeros(3, 2, 2, dtype=DT)
        a[2, 0, 0] = float("nan")
        r = torch.tensor([[.2, .2], [.2, .2], [.2, .2]], dtype=DT)
        result = geom.capsule_in_free_model(a, torch.zeros_like(a), model, r)
        self.assertEqual(result.status.tolist(), [[3, 3], [3, 3], [3, 0]])
        self.assertTrue(bool(torch.isnan(result.min_clearance_m[:2]).all()))
        self.assertAlmostEqual(float(result.min_clearance_m[2, 1]), 2.8, places=12)
        for invalid in (float("nan"), float("inf"), -.1):
            self.assertEqual(int(geom.capsule_in_free_model(point(0.), point(0.), boundary(), invalid).status), 3)

    def test_invalid_grid_is_invalid_not_model_safe(self):
        model = boundary()
        model.context.angles[0] += 1.
        result = geom.capsule_in_free_model(point(0.), point(0.), model, .2)
        self.assertEqual(int(result.status), int(SafetyStatus.INVALID))
        self.assertTrue(bool(torch.isnan(result.min_clearance_m)))

    def test_arc_interior_stationary_distance_and_segment_intersection(self):
        # Independent circle tangent geometry: the segment y=2 spans the top
        # of a unit circle's90deg arc; its closest points are(0,2),(0,1).
        a, b = point(-1., 2.)[:, None], point(1., 2.)[:, None]
        result = geom._c02_arc_distance(a, b, torch.ones(1, 1, dtype=DT),
                                        torch.full((1, 1), math.pi / 2, dtype=DT), .2)
        self.assertAlmostEqual(float(result), 1., places=12)
        result = geom._c02_segment_distance(point(-1.), point(1.), point(0., -1.), point(0., 1.))
        self.assertEqual(float(result), 0.)

    def test_dtype_purity_rng_and_physical_status(self):
        rng = torch.get_rng_state().clone()
        default = torch.get_default_dtype()
        flags = torch.backends.cuda.matmul.allow_tf32, torch.backends.cudnn.allow_tf32
        for dtype in (torch.float32, torch.float64):
            model = boundary(dtype=dtype)
            a, b = point(.5, dtype=dtype), point(1., dtype=dtype)
            saved = [t.clone() for t in (a, b, model.context.pos, model.context.yaw, model.context.free_radius, model.context.angles)]
            result = geom.capsule_in_free_model(a, b, model, .2)
            self.assertEqual(result.min_clearance_m.dtype, dtype)
            self.assertTrue(bool(result.model_safe))
            self.assertEqual(result.physical_qualification, "PHYSICAL_QUALIFICATION_NOT_AVAILABLE")
            for before, after in zip(saved, (a, b, model.context.pos, model.context.yaw, model.context.free_radius, model.context.angles)):
                self.assertTrue(torch.equal(before, after))
        self.assertTrue(torch.equal(rng, torch.get_rng_state()))
        self.assertEqual(default, torch.get_default_dtype())
        self.assertEqual(flags, (torch.backends.cuda.matmul.allow_tf32, torch.backends.cudnn.allow_tf32))
        self.assertNotIn("isaacgym", sys.modules)


if __name__ == "__main__":
    unittest.main()
