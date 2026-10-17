"""CPU-only analytic checks of S-L1A analysis populations and digest identity."""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'tools/s_l1a_analysis.py'
SPEC = importlib.util.spec_from_file_location('_s_l1a_analysis_test', PATH)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


class PixelSummaryTest(unittest.TestCase):
    def test_hand_error_statistics_and_threshold_counts(self):
        out = analysis.pixel_summary(np.array([0., .1, .2, .3]), np.zeros(4), np.ones(4, bool))
        self.assertEqual(out['n'], 4)
        self.assertAlmostEqual(out['max_m'], .3)
        self.assertAlmostEqual(out['mean_m'], .15)
        self.assertAlmostEqual(out['p50_m'], .15)
        self.assertAlmostEqual(out['p95_m'], .285)
        self.assertAlmostEqual(out['p99_m'], .297)
        self.assertEqual(out['over_1e_4'], 3)
        self.assertEqual(out['over_0p05'], 3)

    def test_empty_population_is_explicitly_not_a_pass(self):
        out = analysis.pixel_summary(np.array([4., 5.]), np.zeros(2), np.zeros(2, bool))
        self.assertEqual(out['n'], 0)
        for key in ('max_m', 'mean_m', 'p50_m', 'p95_m', 'p99_m'):
            self.assertIsNone(out[key])
        self.assertEqual(out['over_1e_4'], 0)
        # The qualification criterion must include n>0, rather than using zero violations alone.
        self.assertFalse(out['n'] > 0 and out['over_1e_4'] == 0)
        self.assertNotEqual(out.get('status'), 'PASS')

    def test_nonfinite_anywhere_fails_even_if_mask_would_hide_it(self):
        for value in (np.nan, np.inf, -np.inf):
            with self.subTest(value=value):
                with self.assertRaisesRegex(ValueError, 'Nonfinite'):
                    analysis.pixel_summary([1., value], [1., 2.], [True, False])
                with self.assertRaisesRegex(ValueError, 'Nonfinite'):
                    analysis.pixel_summary([1., 2.], [1., value], [True, False])
        with self.assertRaisesRegex(ValueError, 'shapes'):
            analysis.pixel_summary([1., 2.], [[1., 2.]], [True, True])

    def test_target_residual_stays_separate_from_obstacle_gate(self):
        reference = np.array([[2., 3., 10.]])
        observed = np.array([[2., 3.00033, 10.]])
        target = np.array([[False, True, False]])
        non_target = analysis.pixel_summary(observed, reference, ~target)
        target_only = analysis.pixel_summary(observed, reference, target)
        self.assertEqual(non_target['over_1e_4'], 0)
        self.assertEqual(non_target['max_m'], 0)
        self.assertEqual(target_only['over_1e_4'], 1)
        self.assertAlmostEqual(target_only['max_m'], .00033)
        self.assertEqual(analysis.pixel_summary(observed, reference, np.ones_like(target))['over_1e_4'], 1)


class MixedFarPlaneTest(unittest.TestCase):
    def test_visible_far_target_overlays_nearer_no_return_sentinel(self):
        out = analysis.mixed_reference(origin=(0, 0, 0), sphere_center=(15.15, 0, 0),
                                       sphere_radius=.15, width=3, height=3)
        self.assertAlmostEqual(out['depth_m'][1, 1], 15.)
        self.assertTrue(out['target'][1, 1])
        self.assertFalse(out['obstacle'][1, 1])
        self.assertFalse(out['miss'][1, 1])
        self.assertTrue(out['robust'][1, 1])
        self.assertGreater(out['depth_m'][1, 1], 10.)

    def test_beyond_obstacle_far_box_occludes_target_without_becoming_depth_return(self):
        out = analysis.mixed_reference([[12.1, 0, 0]], [[.1, 2, 2]], origin=(0, 0, 0),
                                       sphere_center=(15.15, 0, 0), width=1, height=1)
        self.assertEqual(out['depth_m'].item(), 10.)
        self.assertFalse(out['target'].item())
        self.assertFalse(out['obstacle'].item())
        self.assertTrue(out['miss'].item())
        self.assertTrue(out['robust'].item())

    def test_near_box_is_first_surface_and_oblique_depth_is_slant(self):
        out = analysis.mixed_reference([[4, 0, 0]], [[1, 100, 100]], origin=(0, 0, 0),
                                       sphere_center=(15.15, 0, 0), width=3, height=1)
        self.assertAlmostEqual(out['depth_m'][0, 1], 3.)
        fx = 3 / (2 * math.tan(math.radians(87)/2))
        expected_oblique = 3 * math.sqrt(1 + 1/fx**2)
        self.assertAlmostEqual(out['depth_m'][0, 0], expected_oblique)
        self.assertTrue(out['obstacle'].all())
        self.assertFalse(out['target'].any())

    def test_robust_edge_exclusion_does_not_silently_erase_all_pixel_error(self):
        out = analysis.mixed_reference([[3, 1, 0]], [[1, 1, 10]], origin=(0, 0, 0), width=5, height=3)
        self.assertFalse(out['robust'][1, 2])
        self.assertTrue(out['robust'][1, 0])
        measured = out['depth_m'].copy()
        measured[1, 2] += 1
        all_pixels = analysis.pixel_summary(measured, out['depth_m'], np.ones((3, 5), bool))
        robust = analysis.pixel_summary(measured, out['depth_m'], out['robust'])
        self.assertEqual(all_pixels['over_1e_4'], 1)
        self.assertEqual(robust['over_1e_4'], 0)
        self.assertLess(robust['n'], all_pixels['n'])
        # Target/obstacle/miss are a partition, regardless of the robust subset.
        np.testing.assert_array_equal(out['target'].astype(int)+out['obstacle']+out['miss'], np.ones((3, 5), int))


class DigestIdentityTest(unittest.TestCase):
    def rows(self, count=2):
        return [{'step': i, **{key: '%s-%d' % (key, i) for key in analysis.DIGEST_KEYS}}
                for i in range(count)]

    def compare(self, left, right):
        with tempfile.TemporaryDirectory() as temporary:
            a, b = Path(temporary)/'a.jsonl', Path(temporary)/'b.jsonl'
            a.write_text(''.join(json.dumps(row)+'\n' for row in left))
            b.write_text(''.join(json.dumps(row)+'\n' for row in right))
            original_a, original_b = a.read_bytes(), b.read_bytes()
            result = analysis.compare_steps(a, b)
            self.assertEqual(a.read_bytes(), original_a)
            self.assertEqual(b.read_bytes(), original_b)
            return result

    def test_complete_fourteen_key_records_match(self):
        self.assertEqual(len(analysis.DIGEST_KEYS), 14)
        out = self.compare(self.rows(), self.rows())
        self.assertTrue(out['identical'])
        self.assertEqual(out['steps_a'], 2)
        self.assertFalse(any(out['differences'].values()))

    def test_missing_key_on_either_or_both_sides_is_not_identity(self):
        for key in analysis.DIGEST_KEYS:
            for both in (False, True):
                left, right = self.rows(), self.rows()
                del right[0][key]
                if both:
                    del left[0][key]
                with self.subTest(key=key, both=both):
                    out = self.compare(left, right)
                    self.assertFalse(out['identical'])
                    self.assertEqual(out['differences'][key], 1)

    def test_length_empty_and_step_sequence_mismatches_fail(self):
        for left, right in ((self.rows(2), self.rows(1)), ([], []),
                            (self.rows(1), self.rows(2))):
            self.assertFalse(self.compare(left, right)['identical'])
        for steps in ((1, 2), (0, 0), (0, 2)):
            left, right = self.rows(), self.rows()
            for row, step in zip(right, steps):row['step'] = step
            self.assertFalse(self.compare(left, right)['identical'])
        left, right = self.rows(), self.rows()
        del right[0]['step']
        self.assertFalse(self.compare(left, right)['identical'])

    def test_mismatched_single_digest_is_reported_without_ignoring_tail(self):
        left, right = self.rows(), self.rows()
        right[1]['tracker'] = 'different'
        out = self.compare(left, right)
        self.assertFalse(out['identical'])
        self.assertEqual(out['differences']['tracker'], 1)
        self.assertEqual(sum(out['differences'].values()), 1)


if __name__ == '__main__':
    unittest.main()
