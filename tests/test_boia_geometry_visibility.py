"""CPU-only synthetic U1 checks; no simulator, external planner or recorded experiment."""
import ast
from dataclasses import FrozenInstanceError
import inspect
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import geometry as geom  # noqa: E402
from motar_eval.boia import types as records  # noqa: E402
from motar_eval.boia import visibility as vis  # noqa: E402

for module in (geom, records, vis):
    assert Path(module.__file__).resolve().is_relative_to(ROOT) if sys.version_info >= (3, 9) else str(Path(module.__file__).resolve()).startswith(str(ROOT) + "/")


def generator(seed=20261019):
    result = torch.Generator(device="cpu")
    result.manual_seed(seed)
    return result


def angles(dtype=torch.float64):
    return torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72, dtype=dtype)


def camera():
    return records.CameraParams(math.pi / 2, math.pi / 3, 80., 80., 80., 45., (0., 0., 0.))


def context(n=1, dtype=torch.float64):
    return records.FreeContext(torch.zeros(n, 3, dtype=dtype), torch.zeros(n, dtype=dtype),
                               torch.full((n, 72), 12., dtype=dtype), angles(dtype), 12.)


def params(**extra):
    return records.BoiaParams(tau_on=.1, r_bar_est=.2, **extra)


def scene(pts=None, samples=None, n=1, k=1, h=1):
    roll = torch.zeros(n, k, h, 3, dtype=torch.float64)
    yaw = torch.zeros(n, k, h, dtype=torch.float64)
    if samples is None:
        samples = torch.zeros(n, 4, h, 6, dtype=torch.float64)
        samples[..., 0] = 6.
    if pts is None:
        pts = torch.empty(n, 0, 2, dtype=torch.float64)
    return roll, yaw, samples, pts, torch.ones(pts.shape[:2], dtype=torch.bool)


class GeometryTest(unittest.TestCase):
    def test_lidar_rows_angles_world_rotation_and_validity(self):
        rho = torch.full((1, 288), 2., dtype=torch.float64)
        rho[0, :4] = torch.tensor([0., -1., 12., float("nan")])
        elev = torch.tensor([20., 10., 0., -10.], dtype=torch.float64) * math.pi / 180
        pos = torch.tensor([[3., 4., 1.]], dtype=torch.float64)
        yaw = torch.tensor([math.pi / 2], dtype=torch.float64)
        pts, valid = geom.lidar_points_xy(rho, angles(), elev, pos, yaw, 12.)
        self.assertEqual(tuple(pts.shape), (1, 288, 2))
        self.assertFalse(bool(valid[0, :4].any()))
        self.assertTrue(bool(torch.isfinite(pts).all()))
        # Row2 has elevation zero; bearing pi plus yaw pi/2 points down world-y.
        torch.testing.assert_close(pts[0, 144], torch.tensor([3., 2.], dtype=torch.float64))
        self.assertAlmostEqual(float(((pts[0, 72] - pos[0, :2]) ** 2).sum().sqrt()), 2 * math.cos(math.pi / 18))

    def test_segment_dist_against_independent_dense_brute_force(self):
        g = generator()
        a = 10 * torch.rand(2, 2, 2, generator=g, dtype=torch.float64) - 5
        b = a + .1 * (2 * torch.rand(2, 2, 2, generator=g, dtype=torch.float64) - 1)
        b[0, 0] = a[0, 0]
        pts = 10 * torch.rand(2, 3, 2, generator=g, dtype=torch.float64) - 5
        result = geom.seg_point_dist(a, b, pts)
        # <=sqrt(2)*0.1 length and 131072 intervals bound distance-grid error <5.4e-7.
        t = torch.linspace(0, 1, 131073, dtype=torch.float64)
        for e in range(2):
            for k in range(2):
                grid = (1 - t[:, None]) * a[e, k] + t[:, None] * b[e, k]
                for j in range(3):
                    ref = ((grid - pts[e, j]) ** 2).sum(-1).sqrt().min()
                    self.assertLessEqual(abs(float(result[e, k, j] - ref)), 1e-6)

    def test_distance_zero_length_and_query_dimensions(self):
        a = torch.zeros(2, 3, 4, 2, dtype=torch.float64)
        pts = torch.tensor([[[3., 4.]], [[0., 2.]]], dtype=torch.float64)
        out = geom.seg_point_dist(a, a, pts)
        self.assertEqual(tuple(out.shape), (2, 3, 4, 1))
        self.assertTrue(bool((out[0] == 5.).all()))
        self.assertTrue(bool((out[1] == 2.).all()))

    def test_belief_gate_excludes_self_and_preserves_distant_bar(self):
        pts = torch.tensor([[[6., 0.], [6.1, 0.], [3., 0.]]], dtype=torch.float64)
        before = pts.clone()
        keep = geom.exclude_target(pts, torch.ones(1, 3, dtype=torch.bool),
                                   torch.tensor([[6., 0.]], dtype=torch.float64),
                                   torch.eye(2, dtype=torch.float64)[None] * .01, .15)
        self.assertEqual(keep.tolist(), [[False, False, True]])
        self.assertTrue(torch.equal(pts, before))

    def test_known_free_row_and_neighbor_minimum(self):
        rho = torch.full((1, 288), 12., dtype=torch.float64)
        rho[0, 72 + 10] = 2.
        ctx = context()
        radius = geom.known_free(rho, ctx.angles, ctx.pos, ctx.yaw, 12., .1)
        torch.testing.assert_close(radius[0, 9:12], torch.full((3,), 1.9, dtype=torch.float64))
        self.assertAlmostEqual(float(radius[0, 8]), 11.9)
        rho[0, 20] = float("nan")
        self.assertEqual(float(geom.known_free(rho, ctx.angles, ctx.pos, ctx.yaw, 12., .1)[0, 20]), 0.)

    def test_known_free_far_bound_translation_and_yaw_wrap(self):
        ctx = context()
        q = torch.tensor([[[11.99, 0.], [12.001, 0.], [0., 0.]]], dtype=torch.float64)
        self.assertEqual(geom.point_in_known_free(q, ctx.pos, ctx.yaw, ctx.free_radius, ctx.angles).tolist(), [[True, False, True]])
        pos = torch.tensor([[3., -2., 1.]], dtype=torch.float64)
        yaw = torch.tensor([2 * math.pi], dtype=torch.float64)
        self.assertTrue(torch.equal(geom.point_in_known_free(q + pos[:, None, :2], pos, yaw, ctx.free_radius, ctx.angles),
                                    geom.point_in_known_free(q, ctx.pos, ctx.yaw, ctx.free_radius, ctx.angles)))

    def test_exact_segment_detects_short_unknown_sector_missed_by_coarse_checks(self):
        ctx = context()
        radius = ctx.free_radius.clone()
        radius[0, 35] = .5  # Five-degree sector around +5deg, crossed near t=.504.
        ctx = records.FreeContext(ctx.pos, ctx.yaw, radius, ctx.angles, 12.)
        a = torch.tensor([[[1., -10.]]], dtype=torch.float64)
        b = torch.tensor([[[1., 10.]]], dtype=torch.float64)
        coarse_t = torch.tensor([0., .25, .5, .75, 1.], dtype=torch.float64)
        q = a[:, :, None] + coarse_t[None, None, :, None] * (b - a)[:, :, None]
        self.assertTrue(bool(geom.point_in_known_free(q, ctx.pos, ctx.yaw, radius, ctx.angles).all()))
        self.assertFalse(bool(geom.segment_in_known_free(a, b, ctx)[0, 0]))
        empty = torch.empty(1, 0, 2, dtype=torch.float64)
        valid = torch.empty(1, 0, dtype=torch.bool)
        self.assertTrue(bool(vis.los_clear(a, b, empty, valid, .1, ctx, False)[0, 0]))
        self.assertFalse(bool(vis.los_clear(a, b, empty, valid, .1, ctx, True)[0, 0]))

    def test_known_free_zero_length_reversal_and_boundary_ray(self):
        ctx = context(2)
        a = torch.tensor([[[1., 1.], [0., 0.]], [[-1., 1.], [0., 0.]]], dtype=torch.float64)
        b = -a
        self.assertTrue(bool(geom.segment_in_known_free(a, b, ctx).all()))
        self.assertTrue(bool(geom.segment_in_known_free(a, a, ctx).all()))
        self.assertTrue(torch.equal(geom.segment_in_known_free(a, b, ctx), geom.segment_in_known_free(b, a, ctx)))

    def test_float32_boundary_below_rounding_point_checks_both_neighbor_sectors(self):
        step = 2 * math.pi / 72
        ang = -torch.arange(72, dtype=torch.float32) * step
        radius = torch.full((1, 72), 12.)
        radius[0, 1] = .5
        phase = .5 - 4 * torch.finfo(torch.float32).eps
        phi = torch.tensor(-phase * step)
        q = (2 * torch.stack((phi.cos(), phi.sin()))).reshape(1, 1, 2)
        actual_phase = float(torch.remainder(-torch.atan2(q[0, 0, 1], q[0, 0, 0]) / step, 72))
        self.assertLess(actual_phase, .5)
        self.assertLess(.5 - actual_phase, 8 * torch.finfo(torch.float32).eps)
        self.assertFalse(bool(geom.point_in_known_free(q, torch.zeros(1, 3), torch.zeros(1), radius, ang)[0, 0]))


class VisibilityTest(unittest.TestCase):
    def test_clear_and_blocked_forecast_and_invalid_point(self):
        inputs = scene()
        out = vis.forecast(*inputs, context(), camera(), params())
        self.assertEqual(tuple(out.shape), (1, 1, 4, 1))
        self.assertTrue(bool((out == 1).all()))
        inputs = scene(torch.tensor([[[3., 0.]]], dtype=torch.float64))
        self.assertTrue(bool((vis.forecast(*inputs, context(), camera(), params()) == 0).all()))
        inputs = (*inputs[:-1], torch.zeros(1, 1, dtype=torch.bool))
        self.assertTrue(bool((vis.forecast(*inputs, context(), camera(), params()) == 1).all()))

    def test_outside_fov_and_range_are_zero(self):
        q = torch.tensor([[1., 0., 0.], [-1., 0., 0.], [1., 2., 0.], [1., 0., 2.], [21., 0., 0.]], dtype=torch.float64)
        self.assertEqual(vis.fov_mask(q, math.pi / 2, math.pi / 3, 20.).tolist(), [True, False, False, False, False])
        inputs = scene()
        inputs[2][..., 0] = -6.
        self.assertTrue(bool((vis.forecast(*inputs, context(), camera(), params()) == 0).all()))

    def test_same_path_horizon_union_not_product_of_marginals(self):
        p = torch.tensor([[[[1., 0.], [0., 1.]]]], dtype=torch.float64)
        self.assertEqual(float(vis.p_any(p)), 1.)
        self.assertEqual(float(1 - (1 - p.mean(2)).prod(-1)), .75)

    def test_invalid_sample_is_blocked_without_nan_in_both_modes(self):
        inp = scene()
        inp[2][0, 0, 0, 0] = float("nan")
        for mode in ("oracle", "stress"):
            out = vis.forecast(*inp, context(), camera(), params(p_miss=.3), mode=mode)
            self.assertEqual(float(out[0, 0, 0, 0]), 0.)
            self.assertTrue(bool(torch.isfinite(out).all()))

    def test_single_occluder_1024_antithetic_mc_agrees_with_linearized_analytic(self):
        g = generator()
        z = torch.randn(512, generator=g, dtype=torch.float64)
        z = torch.cat((z, -z))
        samples = torch.zeros(1, 1024, 1, 6, dtype=torch.float64)
        samples[0, :, 0, 0] = 4.5 + .1 * z
        pts = torch.tensor([[[5., 0.]]], dtype=torch.float64)
        inp = scene(pts, samples)
        p = vis.forecast(*inp, context(), camera(), records.BoiaParams(tau_on=.1, r_bar_est=.35))
        mu = torch.tensor([[4.5, 0., 0.]], dtype=torch.float64)
        cov = torch.diag(torch.tensor([.01, 0., 0.], dtype=torch.float64))[None]
        analytic = vis.bpod_analytic(inp[0][:, :, 0], inp[1][:, :, 0], mu, cov, pts, inp[4], .5, camera())
        self.assertLess(abs(float(vis.p_any(p) - analytic)), .02)
        self.assertAlmostEqual(float(analytic), .5)

    def test_adjacent_nested_clear_events_independence_underestimates_visibility(self):
        z = torch.randn(512, generator=generator(), dtype=torch.float64)
        z = torch.cat((z, -z))
        samples = torch.zeros(1, 1024, 1, 6, dtype=torch.float64)
        samples[0, :, 0, 0] = 4.5 + .1 * z
        pts = torch.tensor([[[5., 0.], [5.02, 0.]]], dtype=torch.float64)
        inp = scene(pts, samples)
        p = vis.forecast(*inp, context(), camera(), records.BoiaParams(tau_on=.1, r_bar_est=.35))
        mu = torch.tensor([[4.5, 0., 0.]], dtype=torch.float64)
        cov = torch.diag(torch.tensor([.01, 0., 0.], dtype=torch.float64))[None]
        product = vis.bpod_analytic(inp[0][:, :, 0], inp[1][:, :, 0], mu, cov, pts, inp[4], .5, camera())
        # The two clear events are nested functions of the SAME uncertain x: positive correlation.
        self.assertGreater(float(vis.p_any(p) - product), .15)
        self.assertAlmostEqual(float(product), .5 * .5 * (1 + math.erf(.2 / math.sqrt(2))), places=12)

    def test_degenerate_covariance_and_endpoint_gradients_are_finite(self):
        ego = torch.zeros(1, 2, 3, dtype=torch.float64)
        mu = torch.tensor([[6., 0., 0.]], dtype=torch.float64)
        pts = torch.tensor([[[3., 0.], [-2., 0.]]], dtype=torch.float64)
        out = vis.bpod_analytic(ego, torch.zeros(1, 2), mu, torch.zeros(1, 3, 3), pts,
                                torch.ones(1, 2, dtype=torch.bool), .3, camera())
        self.assertTrue(bool(torch.isfinite(out).all()))
        self.assertTrue(bool((out == 0).all()))
        at_origin = vis.bpod_analytic(ego, torch.zeros(1, 2), torch.zeros(1, 3),
                                      torch.zeros(1, 3, 3), torch.empty(1, 0, 2),
                                      torch.empty(1, 0, dtype=torch.bool), .3, camera())
        self.assertTrue(bool((at_origin == 0).all()))

    def test_parameters_are_explicit_and_frozen(self):
        with self.assertRaises(ValueError):
            records.BoiaParams()
        for bad in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                records.BoiaParams(tau_on=bad)
            with self.assertRaises(ValueError):
                records.BoiaParams(tau_on=.1, r_bar_est=bad)
        with self.assertRaises(FrozenInstanceError):
            params().H = 5
        self.assertIsNone(records.BoiaParams(tau_on=.1).r_bar_est)
        self.assertIsNone(params().R_camera_diag)
        inp = scene()
        with self.assertRaises(ValueError):
            vis.forecast(*inp, context(), camera(), records.BoiaParams(tau_on=.1))
        with self.assertRaises(ValueError):
            vis.forecast(*inp, context(), camera(), params(), mode="stress")
        out = vis.forecast(*inp, context(), camera(), params(p_miss=.3), mode="stress")
        self.assertTrue(bool(((out >= 0) & (out <= .7)).all()))
        self.assertTrue(torch.equal(out, vis.forecast(*inp, context(), camera(), params(p_miss=.3), mode="stressed")))

    def test_batch_chunking_matches_individual_environments_and_is_pure(self):
        inp = scene(n=2, k=2, h=3)
        saved = [x.clone() for x in inp]
        out = vis.forecast(*inp, context(2), camera(), params())
        for e in range(2):
            sub = tuple(x[e:e + 1] for x in inp)
            torch.testing.assert_close(out[e:e + 1], vis.forecast(*sub, context(), camera(), params()), rtol=0, atol=0)
        for before, after in zip(saved, inp):
            self.assertTrue(torch.equal(before, after))

    def test_fixed_generator_repeatability_and_global_rng_preservation(self):
        before = torch.get_rng_state().clone()
        a = torch.randn(1, 8, 2, 6, generator=generator(), dtype=torch.float64)
        b = torch.randn(1, 8, 2, 6, generator=generator(), dtype=torch.float64)
        a[..., 0] = a[..., 0].abs() + 3
        b[..., 0] = b[..., 0].abs() + 3
        left = vis.forecast(*scene(samples=a, h=2), context(), camera(), params())
        right = vis.forecast(*scene(samples=b, h=2), context(), camera(), params())
        self.assertTrue(torch.equal(left, right))
        self.assertTrue(torch.equal(before, torch.get_rng_state()))
        self.assertTrue(bool(torch.isfinite(left).all()))

    def test_tf32_backend_flag_does_not_change_cpu_distance_result(self):
        old = torch.backends.cuda.matmul.allow_tf32
        try:
            a = torch.tensor([[[1000., 1000.]]], dtype=torch.float32)
            b = a + torch.tensor([[[.01, .01]]])
            pts = a[:, 0:1] + torch.tensor([[[.005, .003]]])
            torch.backends.cuda.matmul.allow_tf32 = False
            x = geom.seg_point_dist(a, b, pts)
            torch.backends.cuda.matmul.allow_tf32 = True
            y = geom.seg_point_dist(a, b, pts)
            self.assertTrue(torch.equal(x, y))
        finally:
            torch.backends.cuda.matmul.allow_tf32 = old

    def test_static_import_information_and_rng_boundaries(self):
        allowed = {"math", "dataclasses", "typing", "torch"}
        owned = {"__init__.py", "types.py", "geometry.py", "visibility.py"}
        forbidden = {"target_position", "target_vel_w", "target_orientation", "navrl_target_position",
                     "segmentation_pixels", "target_mask", "target_depth", "_targets", "_targets_wp",
                     "DETECT_CHANNEL", "obstacle_position", 'task_obs["states"]'}
        for path in (ROOT / "motar_eval" / "boia").glob("*.py"):
            source = path.read_text()
            tree = ast.parse(source)
            if path.name in owned:
                for name in forbidden:
                    self.assertNotIn(name, source)
            self.assertNotIn("cdist", source)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertTrue(all(x.name.split(".")[0] in allowed for x in node.names))
                if isinstance(node, ast.ImportFrom):
                    if node.level:
                        self.assertEqual(node.level, 1)  # Same-package relative imports only.
                    elif not (node.module == "motar_eval.boia" or node.module.startswith("motar_eval.boia.")):
                        self.assertIn(node.module.split(".")[0], allowed)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, {"manual_seed", "seed"})
                    if node.func.attr in {"rand", "randn", "randint", "randperm", "normal"}:
                        declared = [kw.value for kw in node.keywords if kw.arg == "generator"]
                        self.assertEqual(len(declared), 1)
                        self.assertFalse(isinstance(declared[0], ast.Constant) and declared[0].value is None)


if __name__ == "__main__":
    unittest.main()
