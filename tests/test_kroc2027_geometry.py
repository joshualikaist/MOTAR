"""KROC2027 ground-truth geometry and the Experiment A initial condition (CPU only).

Line of sight, the level yaw-only camera frustum, exact segment-to-footprint distance (checked
against dense sampling, including the diagonal corner crossing an endpoint test misses), and the
initial-view sampler: fixed range, same altitude, clear line of sight, acceptance that does not
depend on the arm, and yaw 0 / 180 degrees.
"""
import math
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import geometry, initial_view  # noqa: E402

HFOV = math.radians(87.0)
VFOV = math.radians(58.0)
CAMERA_T = (0.10, 0.0, 0.03)


def _brute_segment_rect_distance(p0, p1, center, half, samples=4001):
    t = torch.linspace(0.0, 1.0, samples, dtype=torch.float64).unsqueeze(-1)
    points = p0 + t * (p1 - p0)
    return float(((points - center).abs() - half).clamp(min=0.0).norm(dim=-1).min())


class SlabTest(unittest.TestCase):
    def test_crossing_missing_touching(self):
        lo = torch.tensor([[1.0, 1.0]])
        hi = torch.tensor([[2.0, 2.0]])
        hit = geometry.segment_hits_aabb
        self.assertTrue(bool(hit(torch.tensor([0.0, 1.5]), torch.tensor([3.0, 1.5]), lo, hi)[0]))
        self.assertFalse(bool(hit(torch.tensor([0.0, 2.5]), torch.tensor([3.0, 2.5]), lo, hi)[0]))
        # Grazing the top face counts as blocked (closed boxes).
        self.assertTrue(bool(hit(torch.tensor([0.0, 2.0]), torch.tensor([3.0, 2.0]), lo, hi)[0]))
        # Ends before the box.
        self.assertFalse(bool(hit(torch.tensor([0.0, 1.5]), torch.tensor([0.9, 1.5]), lo, hi)[0]))
        # Axis-parallel outside and inside the slab.
        self.assertFalse(bool(hit(torch.tensor([0.5, 0.0]), torch.tensor([0.5, 3.0]), lo, hi)[0]))
        self.assertTrue(bool(hit(torch.tensor([1.5, 0.0]), torch.tensor([1.5, 3.0]), lo, hi)[0]))

    def test_three_dimensions(self):
        lo = torch.tensor([[1.0, -0.5, 0.0]])
        hi = torch.tensor([[2.0, 0.5, 3.0]])
        over = geometry.segment_hits_aabb(torch.tensor([0.0, 0.0, 3.5]),
                                          torch.tensor([3.0, 0.0, 3.5]), lo, hi)
        through = geometry.segment_hits_aabb(torch.tensor([0.0, 0.0, 1.0]),
                                             torch.tensor([3.0, 0.0, 1.0]), lo, hi)
        self.assertFalse(bool(over[0]))
        self.assertTrue(bool(through[0]))


class LineOfSightTest(unittest.TestCase):
    def setUp(self):
        self.center = torch.tensor([[5.0, 0.0, 1.5]])
        self.half = torch.tensor([[0.3, 0.3, 1.5]])
        self.eye = torch.tensor([0.0, 0.0, 1.0])

    def test_blocked_and_clear(self):
        blocked = geometry.los_clear(self.eye, torch.tensor([10.0, 0.0, 1.0]), self.center, self.half)
        beside = geometry.los_clear(self.eye, torch.tensor([10.0, 3.0, 1.0]), self.center, self.half)
        self.assertFalse(bool(blocked))
        self.assertTrue(bool(beside))

    def test_valid_mask_and_empty(self):
        valid = torch.tensor([False])
        self.assertTrue(bool(geometry.los_clear(
            self.eye, torch.tensor([10.0, 0.0, 1.0]), self.center, self.half, valid)))
        empty = torch.zeros(0, 3)
        self.assertTrue(bool(geometry.los_clear(self.eye, torch.tensor([10.0, 0.0, 1.0]), empty, empty)))

    def test_ray_over_a_short_obstacle(self):
        low_half = torch.tensor([[0.3, 0.3, 0.4]])
        low_center = torch.tensor([[5.0, 0.0, 0.4]])
        self.assertTrue(bool(geometry.los_clear(self.eye, torch.tensor([10.0, 0.0, 1.0]),
                                                low_center, low_half)))


class FrustumTest(unittest.TestCase):
    def _visible(self, target, yaw=0.0, max_range=20.0):
        eye = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64)
        quat = geometry.yaw_quat_xyzw(torch.tensor(yaw, dtype=torch.float64))
        return bool(geometry.in_camera_frustum(
            eye, quat, torch.tensor(target, dtype=torch.float64), HFOV, VFOV, max_range))

    def test_boresight_edges_and_range(self):
        self.assertTrue(self._visible([10.0, 0.0, 1.0]))
        self.assertFalse(self._visible([-10.0, 0.0, 1.0]))
        inside = math.radians(43.0)
        outside = math.radians(44.0)
        self.assertTrue(self._visible([10 * math.cos(inside), 10 * math.sin(inside), 1.0]))
        self.assertFalse(self._visible([10 * math.cos(outside), 10 * math.sin(outside), 1.0]))
        self.assertFalse(self._visible([5.0, 0.0, 1.0 + 5.0 * math.tan(math.radians(30.0))]))
        self.assertFalse(self._visible([25.0, 0.0, 1.0]))

    def test_yaw_is_respected(self):
        self.assertTrue(self._visible([0.0, 10.0, 1.0], yaw=math.pi / 2))
        self.assertFalse(self._visible([10.0, 0.0, 1.0], yaw=math.pi))

    def test_relative_bearing_and_rotation_round_trip(self):
        position = torch.tensor([1.0, 2.0, 1.0], dtype=torch.float64)
        quat = geometry.yaw_quat_xyzw(torch.tensor(0.7, dtype=torch.float64))
        target = torch.tensor([1.0 + math.cos(1.2), 2.0 + math.sin(1.2), 1.0], dtype=torch.float64)
        self.assertAlmostEqual(float(geometry.relative_bearing(position, quat, target)), 0.5, places=9)
        vec = torch.tensor([0.3, -0.2, 0.9], dtype=torch.float64)
        back = geometry.quat_rotate_inverse_xyzw(quat, geometry.quat_rotate_xyzw(quat, vec))
        self.assertTrue(torch.allclose(back, vec, atol=1e-12))

    def test_camera_eye_rotates_with_yaw(self):
        position = torch.tensor([0.0, 0.0, 1.0], dtype=torch.float64)
        quat = geometry.yaw_quat_xyzw(torch.tensor(math.pi / 2, dtype=torch.float64))
        eye = geometry.camera_eye(position, quat, CAMERA_T)
        self.assertTrue(torch.allclose(eye, torch.tensor([0.0, 0.10, 1.03], dtype=torch.float64),
                                       atol=1e-12))


class SweptClearanceTest(unittest.TestCase):
    def test_matches_dense_sampling(self):
        gen = torch.Generator().manual_seed(11)
        for _ in range(300):
            p0 = torch.rand(2, generator=gen, dtype=torch.float64) * 6.0
            p1 = p0 + (torch.rand(2, generator=gen, dtype=torch.float64) - 0.5) * 2.0
            center = torch.rand(1, 2, generator=gen, dtype=torch.float64) * 6.0
            half = 0.2 + 0.3 * torch.rand(1, 2, generator=gen, dtype=torch.float64)
            exact = float(geometry.segment_rect_distance_2d(p0, p1, center, half)[0])
            brute = _brute_segment_rect_distance(p0, p1, center[0], half[0])
            self.assertLessEqual(exact, brute + 1e-12)
            self.assertAlmostEqual(exact, brute, delta=2e-3)

    def test_diagonal_corner_crossing(self):
        center = torch.tensor([[0.0, 0.0]], dtype=torch.float64)
        half = torch.tensor([[0.3, 0.3]], dtype=torch.float64)
        # Both endpoints are 0.35 m from the box, but the segment cuts its corner.
        p0 = torch.tensor([0.0, 0.65], dtype=torch.float64)
        p1 = torch.tensor([0.65, 0.0], dtype=torch.float64)
        endpoint_only = min(
            float(geometry.point_rect_distance_2d(p0, center[0], half[0])),
            float(geometry.point_rect_distance_2d(p1, center[0], half[0])),
        )
        self.assertGreater(endpoint_only, 0.3)
        swept = float(geometry.swept_proxy_clearance(p0, p1, center, half))
        self.assertLess(swept, 0.05)

    def test_static_segment_and_masks(self):
        center = torch.tensor([[0.0, 0.0], [5.0, 0.0]], dtype=torch.float64)
        half = torch.tensor([[0.3, 0.3], [0.3, 0.3]], dtype=torch.float64)
        point = torch.tensor([1.0, 0.0], dtype=torch.float64)
        self.assertAlmostEqual(float(geometry.swept_proxy_clearance(point, point, center, half)), 0.7)
        valid = torch.tensor([False, True])
        self.assertAlmostEqual(
            float(geometry.swept_proxy_clearance(point, point, center, half, valid)), 3.7)
        empty = torch.zeros(0, 2, dtype=torch.float64)
        self.assertEqual(float(geometry.swept_proxy_clearance(point, point, empty, empty)), math.inf)


def _footprint_ok(margin, centers_xy, halves_xy):
    """The NavRL drone-spawn footprint rule: centre distance minus ||half_xy|| >= margin."""
    def clearance_ok(candidates, ids):
        c = centers_xy[ids].unsqueeze(1)
        h = halves_xy[ids].unsqueeze(1)
        dist = (candidates.unsqueeze(2) - c).norm(dim=-1) - h.norm(dim=-1)
        return dist.amin(dim=-1) >= margin
    return clearance_ok


class InitialViewTest(unittest.TestCase):
    def setUp(self):
        gen = torch.Generator().manual_seed(5)
        self.n = 64
        n_bars = 60
        self.lo = torch.zeros(self.n, 2)
        self.hi = torch.full((self.n, 2), 40.0)
        centers = torch.rand(self.n, n_bars, 2, generator=gen) * 40.0
        self.half3 = torch.cat((0.2 + 0.2 * torch.rand(self.n, n_bars, 2, generator=gen),
                                torch.full((self.n, n_bars, 1), 1.5)), dim=-1)
        self.center3 = torch.cat((centers, torch.full((self.n, n_bars, 1), 1.5)), dim=-1)
        pursuer_xy = 12.0 + 16.0 * torch.rand(self.n, 2, generator=gen)
        self.pursuer = torch.cat((pursuer_xy, torch.ones(self.n, 1)), dim=-1)
        self.clearance_ok = _footprint_ok(0.6, self.center3[..., 0:2], self.half3[..., 0:2])

    def _sample(self, view, seed=99, **spec_kwargs):
        gen = torch.Generator().manual_seed(seed)
        spec = initial_view.InitialViewSpec(view=view, **spec_kwargs)
        return initial_view.sample_initial_view(
            self.pursuer, self.center3, self.half3, self.lo, self.hi, CAMERA_T, spec,
            self.clearance_ok, lambda shape: torch.rand(shape, generator=gen),
        )

    def test_contract_of_accepted_rows(self):
        out = self._sample("visible")
        ok = out["accepted"]
        self.assertGreater(int(ok.sum()), self.n // 2)
        target = out["target_xyz"][ok]
        pursuer = self.pursuer[ok]
        rng = (target[:, 0:2] - pursuer[:, 0:2]).norm(dim=-1)
        self.assertTrue(torch.allclose(rng, torch.full_like(rng, 10.0), atol=1e-4))
        self.assertTrue(torch.equal(target[:, 2], pursuer[:, 2]))
        self.assertTrue(bool(((target[:, 0:2] >= 1.0) & (target[:, 0:2] <= 39.0)).all()))
        ids = ok.nonzero(as_tuple=False).squeeze(-1)
        self.assertTrue(bool(self.clearance_ok(target[:, None, 0:2], ids).all()))

    def test_visible_sees_and_hidden_faces_away(self):
        visible = self._sample("visible")
        hidden = self._sample("hidden")
        ok = visible["accepted"]
        for out, expect in ((visible, True), (hidden, False)):
            eye = geometry.camera_eye(self.pursuer[ok], out["quat_xyzw"][ok], CAMERA_T)
            in_view = geometry.in_camera_frustum(eye, out["quat_xyzw"][ok], out["target_xyz"][ok],
                                                 HFOV, VFOV, 20.0)
            self.assertTrue(bool((in_view == expect).all()))
        rel = geometry.relative_bearing(self.pursuer[ok], hidden["quat_xyzw"][ok],
                                        hidden["target_xyz"][ok])
        self.assertTrue(bool((rel.abs() > math.pi - 1e-4).all()))
        eye = geometry.camera_eye(self.pursuer[ok], visible["quat_xyzw"][ok], CAMERA_T)
        self.assertTrue(bool(geometry.los_clear(eye, visible["target_xyz"][ok],
                                                self.center3[ok], self.half3[ok]).all()))

    def test_acceptance_does_not_depend_on_the_arm(self):
        visible = self._sample("visible")
        hidden = self._sample("hidden")
        self.assertTrue(torch.equal(visible["accepted"], hidden["accepted"]))
        self.assertTrue(torch.equal(visible["rounds_used"], hidden["rounds_used"]))
        ok = visible["accepted"]
        self.assertTrue(torch.equal(visible["target_xyz"][ok], hidden["target_xyz"][ok]))
        self.assertTrue(torch.equal(visible["bearing"][ok], hidden["bearing"][ok]))
        dyaw = geometry.wrap_angle(hidden["yaw"][ok] - visible["yaw"][ok])
        self.assertTrue(torch.allclose(dyaw.abs(), torch.full_like(dyaw, math.pi), atol=1e-5))

    def test_impossible_rows_fail_closed(self):
        # Surround pursuer 0 with a wall of boxes at 5 m: no clear 10 m ray exists.
        ring = 72
        angles = torch.linspace(0.0, 2 * math.pi, ring + 1)[:-1]
        p = self.pursuer[0, 0:2]
        wall = torch.stack((p[0] + 5.0 * torch.cos(angles), p[1] + 5.0 * torch.sin(angles)), dim=-1)
        center3 = self.center3.clone()
        half3 = self.half3.clone()
        extra_c = torch.zeros(self.n, ring, 3)
        extra_h = torch.zeros(self.n, ring, 3)
        extra_c[:, :, 0:2] = -100.0  # parked far away for every other env
        extra_c[0, :, 0:2] = wall
        extra_c[:, :, 2] = 1.5
        extra_h[:, :, 0:2] = 0.3
        extra_h[:, :, 2] = 1.5
        self.center3 = torch.cat((center3, extra_c), dim=1)
        self.half3 = torch.cat((half3, extra_h), dim=1)
        out = self._sample("visible", rounds=4)
        self.assertFalse(bool(out["accepted"][0]))
        self.assertTrue(bool(torch.isnan(out["target_xyz"][0]).all()))
        self.assertEqual(int(out["rounds_used"][0]), 4)

    def test_spec_validation_and_yaw(self):
        with self.assertRaises(ValueError):
            initial_view.InitialViewSpec(view="occluded")
        with self.assertRaises(ValueError):
            initial_view.InitialViewSpec(view="visible", distance_m=0.0)
        bearing = torch.tensor([0.0, 3.0, -3.0])
        self.assertTrue(torch.allclose(initial_view.view_yaw(bearing, "visible"), bearing))
        hidden = initial_view.view_yaw(bearing, "hidden")
        diff = geometry.wrap_angle(hidden - bearing).abs()
        self.assertTrue(torch.allclose(diff, torch.full_like(diff, math.pi), atol=1e-6))


if __name__ == "__main__":
    unittest.main()
