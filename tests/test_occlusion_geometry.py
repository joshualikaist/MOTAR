"""Offline 3D line of sight (numpy) against hand cases and the recorder's torch geometry (CPU)."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import geometry as tg  # noqa: E402
from motar_eval import occlusion_geometry as og  # noqa: E402


def yaw_quat(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


class HandCaseTest(unittest.TestCase):
    def test_blocked_clear_grazing_and_above(self):
        center = np.array([[5.0, 0.0, 1.5]])
        half = np.array([[0.3, 0.3, 1.5]])
        eye = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 1.0], [0.0, 0.3, 1.0], [0.0, 0.0, 3.5]])
        target = np.array([[10.0, 0.0, 1.0], [10.0, 1.0, 1.0], [10.0, 0.3, 1.0], [10.0, 0.0, 3.5]])
        hits = og.segment_hits_boxes(eye, target, center, half)[:, 0]
        self.assertEqual(hits.tolist(), [True, False, True, False])  # grazing counts; above a 3 m bar is clear

    def test_full_versus_partial_occlusion(self):
        eye, target = np.array([[0.0, 0.0, 1.0]]), np.array([[10.0, 0.0, 1.0]])
        wide = og.line_of_sight(eye, target, np.array([[8.0, 0.0, 1.5]]), np.array([[0.2, 0.5, 1.5]]), radius=0.15)
        narrow = og.line_of_sight(eye, target, np.array([[8.0, 0.0, 1.5]]), np.array([[0.2, 0.05, 1.5]]), radius=0.15)
        self.assertTrue(wide["center_blocked"][0] and wide["geometric_full_occlusion"][0])
        self.assertTrue(narrow["center_blocked"][0])
        self.assertFalse(narrow["geometric_full_occlusion"][0])
        self.assertLess(narrow["rim_blocked_fraction"][0], 1.0)

    def test_frustum_and_range(self):
        eye = np.zeros((3, 3))
        quat = np.tile(yaw_quat(0.0), (3, 1))
        target = np.array([[10.0, 0.0, 0.0], [10.0, 20.0, 0.0], [25.0, 0.0, 0.0]])
        angular, ranged = og.in_frustum(eye, quat, target, math.radians(87), math.radians(58), 20.0)
        self.assertEqual(angular.tolist(), [True, False, True])
        self.assertEqual(ranged.tolist(), [True, False, False])


class CrossCheckTest(unittest.TestCase):
    def test_agrees_with_torch_recorder_geometry(self):
        rng = np.random.default_rng(3)
        n, b = 400, 60
        center = np.column_stack([rng.uniform(0, 40, b), rng.uniform(0, 40, b), np.full(b, 1.5)])
        half = np.column_stack([rng.uniform(0.2, 0.4, b), rng.uniform(0.2, 0.4, b), np.full(b, 1.5)])
        pos = np.column_stack([rng.uniform(0, 40, n), rng.uniform(0, 40, n), rng.uniform(0.5, 2.0, n)])
        yaw = rng.uniform(-math.pi, math.pi, n)
        quat = np.stack([yaw_quat(y) for y in yaw])
        target = np.column_stack([rng.uniform(0, 40, n), rng.uniform(0, 40, n), rng.uniform(0.5, 2.0, n)])
        eye = og.camera_eye(pos, quat, (0.10, 0.0, 0.03))
        mine = og.line_of_sight(eye, target, center, half, rim_count=0)["center_blocked"]
        t = lambda a: torch.as_tensor(a, dtype=torch.float64)  # noqa: E731
        t_eye = tg.camera_eye(t(pos), t(quat), (0.10, 0.0, 0.03))
        np.testing.assert_allclose(eye, t_eye.numpy(), atol=1e-12)
        theirs = ~tg.los_clear(t_eye, t(target), t(center).expand(n, b, 3), t(half).expand(n, b, 3)).numpy()
        self.assertEqual(mine.tolist(), theirs.tolist())
        self.assertTrue(0 < mine.sum() < n)  # the fixture exercises both outcomes
        angular, ranged = og.in_frustum(eye, quat, target, math.radians(87), math.radians(58), 20.0)
        theirs_fov = tg.in_camera_frustum(t_eye, t(quat), t(target), math.radians(87), math.radians(58), 20.0)
        self.assertEqual((angular & ranged).tolist(), theirs_fov.numpy().tolist())

    def test_rotated_box_equals_rotated_world(self):
        rng = np.random.default_rng(5)
        center = np.array([[4.0, 1.0, 1.5]])
        half = np.array([[0.2, 1.0, 1.5]])
        yaw = 0.6
        p0 = np.column_stack([rng.uniform(-5, 0, 200), rng.uniform(-3, 5, 200), np.ones(200)])
        p1 = np.column_stack([rng.uniform(6, 10, 200), rng.uniform(-3, 5, 200), np.ones(200)])
        rotated = og.segment_hits_boxes(p0, p1, center, half, yaw_quat(yaw)[None])[:, 0]
        # Same test in the box frame with an axis-aligned box.
        q = np.tile(yaw_quat(yaw), (200, 1))
        local0 = og.quat_rotate_inverse(q, p0 - center)
        local1 = og.quat_rotate_inverse(q, p1 - center)
        aligned = og.segment_hits_boxes(local0, local1, np.zeros((1, 3)), half)[:, 0]
        self.assertEqual(rotated.tolist(), aligned.tolist())
        self.assertTrue(0 < rotated.sum() < 200)


if __name__ == "__main__":
    unittest.main()
