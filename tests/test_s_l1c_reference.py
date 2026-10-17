"""Self-checks of the independent S-L1C float64 oracle against hand-derived geometry (CPU)."""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import s_l1c_reference as ref  # noqa: E402

GATE = 0.35 + 2 * math.sqrt(0.03)  # D2-N2 primed covariance 0.01 I


def bar(x, y, half=(.3, .3, 1.5), z_centre=.5):
    """Tall bar (3 m, bottom at the vehicle's -1 m) in the vehicle frame."""
    return [[x, y, z_centre]], [list(half)]


def primed_in_front(scan_centres, halves, beam_deg):
    a = math.radians(beam_deg)
    d = np.array([math.cos(a), math.sin(a), 0.])
    t = min(ref.ref.first_hits(np.zeros(3), d[None], [c], [h], far_m=50.)["depth_m"][0]
            for c, h in zip(scan_centres, halves))
    return (t + ref.CAMERA_TARGET_RADIUS_M) * d


class NegativeTest(unittest.TestCase):
    def test_tall_bars_rejected_where_legacy_accepts(self):
        for x, y, beam in ((5.3, 0, 0), (2.82 + .3, 1.03, 20), (8.3, 0, 0), (0, 5.3, 90), (-5.3, 0, 180),
                           (2.3, 0, 0), (10.3, 0, 0), (4.3, 0, 0), (6.8, 0, 0)):
            with self.subTest(x=x, y=y):
                centres, halves = bar(x, y)
                scan, _ = ref.lidar_scan(centres, halves)
                primed = primed_in_front(centres, halves, beam)
                self.assertTrue(ref.associate(scan, primed, gate_m=GATE, mode="legacy")["valid"])
                out = ref.associate(scan, primed, gate_m=GATE)
                self.assertFalse(out["valid"])
                self.assertTrue(out["extended"])

    def test_declared_residual_band_beyond_11_m(self):
        # A 3 m bar near the LiDAR limit returns one row only: the shape test cannot reject it.
        centres, halves = bar(11.8, 0)  # front face 11.5 m: the +10 deg row passes above (z 2.03 > 2)
        scan, _ = ref.lidar_scan(centres, halves)
        out = ref.associate(scan, primed_in_front(centres, halves, 0), gate_m=GATE)
        self.assertFalse(out["extended"])


class PositiveTest(unittest.TestCase):
    def test_clear_targets_accepted_with_exact_centre(self):
        for d in (1.5, 3., 6., 9., 11.5):
            for bearing in (0., 20., -45.):
                with self.subTest(d=d, bearing=bearing):
                    a = math.radians(bearing)
                    centre = np.array([d * math.cos(a), d * math.sin(a), 0.])
                    scan, cls = ref.lidar_scan(sphere_center=centre)
                    out = ref.associate(scan, centre, gate_m=GATE)
                    self.assertTrue(out["valid"])
                    self.assertAlmostEqual(out["centre_range"], d, delta=1e-9)

    def test_target_in_front_of_bar_and_between_bars(self):
        cases = (([[5.8, 0, .5]], [[.3, .3, 1.5]]),
                 ([[5.0, 1.0, .5], [5.0, -1.0, .5]], [[.3, .3, 1.5], [.3, .3, 1.5]]))
        for centres, halves in cases:
            with self.subTest(n=len(centres)):
                scan, _ = ref.lidar_scan(centres, halves, sphere_center=[5., 0., 0.])
                self.assertTrue(ref.associate(scan, [5., 0., 0.], gate_m=GATE)["valid"])

    def test_short_range_inconclusive_and_ineligible_rows(self):
        scan, _ = ref.lidar_scan(sphere_center=[1.0, 0., 0.])
        self.assertFalse(ref.associate(scan, [1.0, 0., 0.], gate_m=GATE)["valid"])
        # Predicted 0.5 m above the drone at 4 m: no row passes within the proxy radius.
        out = ref.associate(np.full((4, 72), 12.), [4., 0., .5], gate_m=GATE)
        self.assertEqual(out.get("reason"), "no_eligible_row")

    def test_camera_visible_or_inactive_track_never_associates(self):
        scan, _ = ref.lidar_scan(sphere_center=[4., 0., 0.])
        self.assertFalse(ref.associate(scan, [4., 0., 0.], gate_m=GATE, camera_visible=True)["valid"])
        self.assertFalse(ref.associate(scan, [4., 0., 0.], gate_m=GATE, active=False)["valid"])


class CovarianceTest(unittest.TestCase):
    def test_range_only_update_keeps_unobserved_directions(self):
        for az, el in ((0., 0.), (45., 0.), (90., 8.)):
            with self.subTest(az=az, el=el):
                a, e = math.radians(az), math.radians(el)
                u, t1, t2 = ref.orthonormal_basis([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
                post = ref.range_only_posterior(np.eye(3), u)
                self.assertGreaterEqual(t1 @ post @ t1, .99)
                self.assertGreaterEqual(t2 @ post @ t2, .99)
                self.assertLess(u @ post @ u, .01)

    def test_legacy_diagonal_update_shrinks_tangential_variance(self):
        p = np.eye(3)
        r = np.diag([.08 ** 2, .15 ** 2, .20 ** 2])
        k = p @ np.linalg.inv(p + r)
        post = (np.eye(3) - k) @ p
        self.assertLess(post[1, 1], .03)  # EQ-1C T4: 1.0 -> 0.022


class ContractTest(unittest.TestCase):
    def test_self_test_constants_and_no_production_import(self):
        self.assertTrue(all(ref.self_test().values()))
        self.assertAlmostEqual(ref.MIN_CONCLUSIVE_M, .2 / math.sin(math.radians(10)), places=12)
        tree = ast.parse((ROOT / "tools/audits/s_l1c_reference.py").read_text())
        names = {a.name for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom)) for a in n.names}
        self.assertFalse(any("aerial_gym" in x or "torch" in x for x in names))


if __name__ == "__main__":
    unittest.main()
