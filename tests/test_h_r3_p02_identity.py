"""CPU-only P02 pixel association oracles and call-through information-boundary checks."""
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval import h_r3_p02_identity as identity  # noqa: E402


def oracle(detected, visible, uv, delivered=True):
    """Independent set/count arithmetic; no production torch geometry/association helpers."""
    overlap = len(detected.intersection(visible))
    dp, vp = len(detected), len(visible)
    ratio = overlap / dp if dp else 0.0
    near = all(math.isfinite(x) for x in uv) and any(
        max(abs(uv[0] - col), abs(uv[1] - row)) <= 1.00001 for row, col in visible)
    eligible = delivered and overlap >= 1 and near
    bbox = ([min(c for r, c in detected), min(r for r, c in detected),
             max(c for r, c in detected), max(r for r, c in detected)] if detected else [-1] * 4)
    return {"detector_px": dp, "visible_px": vp, "overlap_px": overlap,
            "detector_false_px": dp - overlap, "detector_overlap_fraction": ratio,
            "target_recall": overlap / vp if vp else 0.0,
            "mask_iou": overlap / len(detected.union(visible)) if detected.union(visible) else 0.0,
            "detector_bbox": bbox, "centroid_near_visible": near,
            "camera_delivered": delivered, "camera_identity_correct": eligible and ratio >= .5,
            "camera_identity_correct_025": eligible and ratio >= .25,
            "camera_identity_correct_075": eligible and ratio >= .75}


def score(detected, visible, uv=(3., 2.), delivered=True, measurement=None):
    dm, vm = np.zeros((1, 6, 9), bool), np.zeros((1, 6, 9), bool)
    for row, col in detected:
        dm[0, row, col] = True
    for row, col in visible:
        vm[0, row, col] = True
    # Deliberately asymmetric intrinsics/translation detect swaps, omitted offsets and sign errors.
    offset = np.array([[.13, -.07, .03]])
    if measurement is None:
        measurement = offset + np.array([[2., (4. - uv[0]) / 7. * 2., (2.5 - uv[1]) / 5. * 2.]])
    return identity.score_detection(torch.from_numpy(dm), torch.from_numpy(vm),
                                    torch.as_tensor(measurement, dtype=torch.float64),
                                    torch.tensor([delivered]), torch.from_numpy(offset), 7., 5., 4., 2.5)


class SpatialOracleTest(unittest.TestCase):
    def test_manual_pixel_sets_match_independent_oracle(self):
        target = {(2, 3), (2, 4), (3, 3), (3, 4)}
        cases = [
            (target, target, (3.5, 2.5), True),              # exact target
            ({(0, 0), (0, 1)}, target, (.5, 0.), True),    # target visible elsewhere
            ({(2, 3), (2, 4)}, target, (3.5, 2.), True),   # partial support
            ({(2, 3), (0, 0)}, target, (3., 2.), True),    # exact .5 dominance
            ({(2, 3), (0, 0), (0, 1), (0, 2)}, target, (3., 2.), True),
            ({(2, 3), (2, 4), (3, 3), (0, 0)}, target, (3., 2.), True),
            (set(), target, (3., 2.), False),
            ({(2, 3)}, set(), (3., 2.), True),
            (set(), set(), (3., 2.), False),
            (target, target, (3.5, 2.5), False),
            # Bbox encloses target; actual selected pixels are disjoint.
            ({(0, 0), (0, 8), (5, 0), (5, 8)}, target, (4., 2.5), True),
            # Dominant support still cannot excuse a centroid on a different object.
            (target, target, (8., 0.), True),
        ]
        for detected, visible, uv, delivered in cases:
            with self.subTest(detected=detected, uv=uv, delivered=delivered):
                actual, expected = score(detected, visible, uv, delivered), oracle(detected, visible, uv, delivered)
                for key, value in expected.items():
                    got = actual[key][0].tolist()
                    if isinstance(value, float):
                        self.assertAlmostEqual(got, value, places=6, msg=key)
                    else:
                        self.assertEqual(got, value, key)
                np.testing.assert_allclose(actual["reported_centroid_uv"].numpy()[0], uv, atol=1e-12)

    def test_continuous_distance_boundary_and_not_raster_rounding(self):
        pixel = {(2, 3)}
        for uv, expected in [((4., 3.), True), ((4.000009, 3.000009), True),
                             ((4.000011, 2.), False), ((4.49, 2.), False), ((3., .99998), False)]:
            with self.subTest(uv=uv):
                self.assertEqual(bool(score(pixel, pixel, uv)["centroid_near_visible"][0]), expected)

    def test_invalid_forward_or_nonfinite_measurement_cannot_pass(self):
        pixel = {(2, 3)}
        for m in ([.13, -.07, .03], [-1., 0., 0.], [float("nan"), 0., 0.],
                  [1., float("inf"), 0.]):
            out = score(pixel, pixel, measurement=[m])
            self.assertFalse(bool(out["centroid_near_visible"][0]))
            self.assertFalse(bool(out["camera_identity_correct"][0]))

    def test_batches_keep_environments_separate(self):
        masks = torch.zeros(2, 4, 7, dtype=torch.bool)
        target = masks.clone()
        masks[0, 1, 2], masks[1, 2, 5] = True, True
        target[0, 1, 2], target[1, 1, 2] = True, True
        out = identity.score_detection(masks, target, torch.tensor([[1., 1., 1.], [1., -2., 0.]]),
                                       torch.ones(2, dtype=torch.bool), torch.zeros(1, 3), 1., 1., 3., 2.)
        self.assertEqual(out["camera_identity_correct"].tolist(), [True, False])
        self.assertEqual(out["detector_bbox"].tolist(), [[2, 1, 2, 1], [5, 2, 5, 2]])


class FakePerception:
    def __init__(self):
        self.width, self.height = 7, 4
        self.detect_width, self.detect_height, self.detect_decoupled = 7, 4, False
        self.detection_latency_s, self._latency_steps = 0., 0
        self.empirical_error, self._detector_noise_active, self.range_error_m = None, False, 0.
        self.camera_offset = torch.tensor([[.1, -.2, .3]])
        self.detect_fx, self.detect_fy, self.detect_cx, self.detect_cy = 2., 3., 3., 1.5
        self.calls = 0
        self.mask = torch.zeros(1, 4, 7, dtype=torch.bool)
        self.mask[0, 1, 2] = True
        self.output = (torch.tensor([[1.1, .3, .466666667]]), torch.tensor([.85]), torch.tensor([.2]),
                       torch.tensor([True]), torch.tensor([.9]), self.mask)

    def _detect_rgbd(self, rgb, depth, training, **kwargs):
        self.calls += 1
        return self.output


class WrapperBoundaryTest(unittest.TestCase):
    def setUp(self):
        self.perception = FakePerception()
        self.detector = SimpleNamespace(width=7, height=4, target_mask=self.perception.mask.clone().int())

    def test_original_return_identity_nonmutation_rng_and_stable_snapshots(self):
        p, d = self.perception, self.detector
        originals = [x.clone() for x in p.output]
        rec = identity.attach_camera_identity(p, d)
        state = torch.random.get_rng_state().clone()
        np_state = np.random.get_state()
        py_state = random.getstate()
        rgb, depth = torch.zeros(1, 3, 4, 7), torch.ones(1, 4, 7)
        result = p._detect_rgbd(rgb, depth, False)
        self.assertIs(result, p.output)
        self.assertEqual((p.calls, rec.call_count), (1, 1))
        self.assertTrue(torch.equal(state, torch.random.get_rng_state()))
        self.assertEqual(py_state, random.getstate())
        self.assertEqual(np_state[0], np.random.get_state()[0])
        np.testing.assert_array_equal(np_state[1], np.random.get_state()[1])
        self.assertEqual(np_state[2:], np.random.get_state()[2:])
        for value, before in zip(result, originals):
            self.assertTrue(torch.equal(value, before))
        self.assertTrue(bool(rec.latest["camera_identity_correct"][0]))
        first = rec.latest
        p.mask.zero_()
        p.output[3].zero_()
        d.target_mask.zero_()
        self.assertTrue(bool(first["camera_delivered"][0]))
        self.assertEqual(int(first["detector_px"][0]), 1)
        p._detect_rgbd(rgb, depth, training=False)
        self.assertEqual(rec.call_count, 2)
        self.assertEqual(int(rec.latest["detector_px"][0]), 0)
        self.assertFalse(bool(rec.latest["camera_identity_correct"][0]))
        rec.close()
        rec.close()
        self.assertNotIn("_detect_rgbd", vars(p))
        self.assertIs(p._detect_rgbd.__func__, FakePerception._detect_rgbd)

    def test_duplicate_and_external_replacement_are_not_silently_overwritten(self):
        p = self.perception
        rec = identity.attach_camera_identity(p, self.detector)
        with self.assertRaisesRegex(ValueError, "already attached"):
            identity.attach_camera_identity(p, self.detector)
        replacement = lambda *a, **k: p.output
        p._detect_rgbd = replacement
        with self.assertRaisesRegex(RuntimeError, "replaced externally"):
            rec.close()
        self.assertIs(p._detect_rgbd, replacement)

    def test_unsupported_configurations_fail_before_original_call(self):
        for key, value in [("detect_decoupled", True), ("detect_width", 8),
                           ("detection_latency_s", .01), ("_latency_steps", 1),
                           ("empirical_error", object()), ("_detector_noise_active", True),
                           ("range_error_m", .1), ("_pose_premise_active", True),
                           ("target_mask_backfill", True), ("latency_obstacle_fix", "predict")]:
            with self.subTest(key=key):
                p = FakePerception()
                setattr(p, key, value)
                with self.assertRaises(ValueError):
                    identity.attach_camera_identity(p, self.detector)
                self.assertEqual(p.calls, 0)
        p = self.perception
        rec = identity.attach_camera_identity(p, self.detector)
        p.rgb_noise_std = .01
        with self.assertRaisesRegex(ValueError, "training perturbations"):
            p._detect_rgbd(None, None, True)
        self.assertEqual((p.calls, rec.call_count), (0, 0))

    def test_runtime_guard_invalidates_previous_snapshot(self):
        p = self.perception
        rec = identity.attach_camera_identity(p, self.detector)
        p._detect_rgbd(None, None, False)
        self.assertIsNotNone(rec.latest)
        p._latency_steps = 1
        with self.assertRaisesRegex(ValueError, "zero detection latency"):
            p._detect_rgbd(None, None, False)
        self.assertIsNone(rec.latest)
        self.assertEqual((p.calls, rec.call_count), (1, 1))

    def test_failed_invocation_cannot_publish_old_frame(self):
        p = self.perception
        rec = identity.attach_camera_identity(p, self.detector)
        p._detect_rgbd(None, None, False)
        p.output = (torch.zeros(1, 3),)  # malformed original output
        with self.assertRaises(IndexError):
            p._detect_rgbd(None, None, False)
        self.assertIsNone(rec.latest)
        self.assertEqual(rec.call_count, 1)


if __name__ == "__main__":
    unittest.main()
