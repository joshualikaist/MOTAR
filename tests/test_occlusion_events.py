"""Occlusion intervals, censoring, Kaplan-Meier, Aalen-Johansen and the geometry-only sweep (CPU)."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import occlusion_events as ev  # noqa: E402
from motar_eval import occlusion_sweep as osw  # noqa: E402


class IntervalTest(unittest.TestCase):
    def test_after_acquisition_restored_and_censored(self):
        #          frame: 0  1  2  3  4  5  6  7
        indicator = [1, 0, 1, 1, 0, 0, 1, 1]
        rows = ev.episode_intervals(indicator, range(8), acquisition_frame=1, terminal_frame=8, dt=0.1,
                                    outcome="crash")
        self.assertEqual([(r["start_frame"], r["end_frame"], r["end"]) for r in rows],
                         [(2, 4, "restored"), (6, 8, "crash")])
        self.assertAlmostEqual(rows[0]["duration_s"], 0.2)
        self.assertTrue(rows[1]["censored"])
        self.assertAlmostEqual(ev.exposure_s(1, 8, 0.1), 0.6)
        self.assertEqual(ev.episode_intervals(indicator, range(8), -1, 8, 0.1, "capture"), [])
        shutdown = ev.episode_intervals([0, 1], [0, 1], 0, 2, 0.1, None)
        self.assertEqual(shutdown[0]["end"], "unknown")

    def test_gaps_and_bad_terminal_are_refused(self):
        with self.assertRaisesRegex(ValueError, "contiguous"):
            ev.episode_intervals([1, 1], [0, 2], 0, 3, 0.1, "capture")
        with self.assertRaisesRegex(ValueError, "terminal"):
            ev.episode_intervals([1, 1], [0, 1], 0, 1, 0.1, "capture")

    def test_reacquisition(self):
        interval = {"end": "restored", "end_frame": 4}
        self.assertAlmostEqual(ev.reacquisition([0, 0, 0, 0, 0, 1], range(6), interval, 6, 0.1)["time_s"], 0.1)
        censored = ev.reacquisition([0] * 6, range(6), interval, 6, 0.1)
        self.assertTrue(censored["censored"])
        self.assertFalse(ev.reacquisition([1], [0], {"end": "crash"}, 1, 0.1)["eligible"])


class EstimatorTest(unittest.TestCase):
    def test_kaplan_meier_hand_values(self):
        # durations 1,2,3(censored),4 ; events at 1,2,4.
        km = ev.kaplan_meier([1, 2, 3, 4], [True, True, False, True], (0.5, 1.0, 2.5, 4.0))
        self.assertAlmostEqual(km["S"]["0.5"], 1.0)
        self.assertAlmostEqual(km["S"]["1.0"], 0.75)
        self.assertAlmostEqual(km["S"]["2.5"], 0.5)
        self.assertAlmostEqual(km["S"]["4.0"], 0.0)
        self.assertEqual(km["median_s"], 2.0)

    def test_cumulative_incidence_partitions_ends(self):
        d = [1, 1, 2, 3, 3, 4]
        c = ["restored", "capture", "restored", "crash", "restored", "timeout"]
        cif = ev.aalen_johansen(d, c, (10.0,))
        total = sum(cif[k]["10.0"] for k in ev.CAUSES)
        self.assertAlmostEqual(total, 1.0)
        self.assertAlmostEqual(cif["restored"]["10.0"], 0.5)
        unknown = ev.aalen_johansen([1, 2], ["unknown", "restored"], (5.0,))
        self.assertAlmostEqual(unknown["restored"]["5.0"], 1.0)  # unknown end is censoring

    def test_summary_and_bootstrap(self):
        rows = [{"episode": 0, "duration_s": 0.6, "end": "restored"},
                {"episode": 0, "duration_s": 2.5, "end": "capture"},
                {"episode": 1, "duration_s": 1.1, "end": "restored"}]
        s = ev.summarise(rows, [2.0, 3.0, 5.0])
        self.assertEqual((s["intervals"], s["episodes_with_interval"], s["censored_intervals"]), (3, 2, 1))
        self.assertAlmostEqual(s["rate_per_exposure_s"], 0.3)
        self.assertAlmostEqual(s["observed_share_at_least"]["2.0"], 1 / 3)
        clusters = {0: (2, 2.0), 1: (1, 3.0), 2: (0, 5.0)}
        boot = ev.cluster_bootstrap(clusters, lambda p: sum(c for c, _ in p) / sum(x for _, x in p), 200, 1)
        lo, hi = boot["interval"]
        self.assertLessEqual(lo, 0.3 + 1e-9)
        self.assertGreaterEqual(hi, 0.3 - 1e-9)
        self.assertIsNone(ev.cluster_bootstrap({0: (1, 1.0)}, lambda p: 0.0)["interval"])


class SweepTest(unittest.TestCase):
    def test_deterministic_and_labelled(self):
        rng = np.random.default_rng(0)
        sizes = [(0.45, 0.6), (0.5, 0.8)]
        layouts = {70: [osw.synthetic_layout(70, rng, sizes)], 205: [osw.synthetic_layout(205, rng, sizes)]}
        a = osw.sweep(layouts, paths_per_layout=3, seed=7, horizon_s=10.0)
        b = osw.sweep(layouts, paths_per_layout=3, seed=7, horizon_s=10.0)
        self.assertEqual(a, b)
        self.assertEqual(a["label"], osw.LABEL)
        self.assertEqual(set(a["densities"]), {"70", "205"})
        self.assertEqual(a["densities"]["70"]["episodes"], 3)

    def test_static_pursuit_never_reoccludes_but_lateral_motion_can(self):
        rng = np.random.default_rng(2)
        layouts = {205: [osw.synthetic_layout(205, rng, [(0.45, 0.6), (0.5, 0.8)]) for _ in range(2)]}
        out = osw.sweep_families(layouts, paths_per_layout=15, seed=3)
        fam = out["families"]
        self.assertEqual(fam["static_pursuit"]["densities"]["205"]["intervals"], 0)
        self.assertGreater(fam["static_crossing"]["densities"]["205"]["intervals"]
                           + fam["moving_pursuit"]["densities"]["205"]["intervals"], 0)

    def test_empty_layout_never_blocks(self):
        rng = np.random.default_rng(1)
        empty = (np.zeros((0, 3)), np.zeros((0, 3)))
        out = osw.sweep({0: [empty]}, paths_per_layout=2, seed=1, horizon_s=10.0)
        self.assertEqual(out["densities"]["0"]["intervals"], 0)
        del rng


if __name__ == "__main__":
    unittest.main()
