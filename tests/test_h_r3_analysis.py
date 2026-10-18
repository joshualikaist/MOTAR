"""CPU checks of tools/h_r3_analysis.py on synthetic four-arm traces."""
import ast
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import h_r3_analysis as ha  # noqa: E402

CLASSES = ["NONE", "RANGE_EXIT", "FOV_EXIT", "SCALE_LOSS", "FULL", "CENTRE_ONLY", "PARTIAL", "VISIBLE"]
STEPS, N, W = 60, 4, ha.PLAN["event"]["window_steps"]


def base_trace():
    z = lambda dt=bool: np.zeros((STEPS, N), dtype=dt)
    tr = {"camera": z(), "lidar_only": z(), "el0_cls": np.full((STEPS, N), 7, dtype=np.int64),
          "el0_visible_px": np.full((STEPS, N), 50, dtype=np.int64), "el0_alone_px": np.full((STEPS, N), 50, dtype=np.int64),
          "el0_in_frustum": np.ones((STEPS, N), dtype=bool), "crashed": z(), "captured": z(), "reset": z(),
          "robot_pos": np.zeros((STEPS, N, 3), dtype=np.float32), "target_pos": np.zeros((STEPS, N, 3), dtype=np.float32),
          "belief_pos": np.zeros((STEPS, N, 3), dtype=np.float32), "action": np.zeros((STEPS, N, 4), dtype=np.float32),
          "intervening": z(), "shadow_offset": np.zeros((STEPS, N), dtype=np.float32), "shadow_clear": np.ones((STEPS, N), dtype=bool),
          "digest": np.arange(STEPS * N, dtype=np.uint64).reshape(STEPS, N), "controller_ms": np.ones(STEPS),
          "t0": np.array([10, 10, 10, -1])}
    tr["camera"][:10] = True
    tr["target_pos"][:, :, 0] = 5.0
    tr["el0_cls"][9, 0] = 4      # FULL (primary) for env 0 at the branch observation
    tr["el0_cls"][9, 1] = 2      # FOV_EXIT for env 1
    tr["el0_cls"][9, 2] = 5      # CENTRE_ONLY for env 2
    tr["el0_visible_px"][10:, :3] = 0
    return tr


def write_run(root, traces):
    for arm, tr in traces.items():
        d = Path(root) / ("seed4502_" + arm)
        d.mkdir(parents=True)
        np.savez(d / "h_r3_trace.npz", **tr)
        (d / "h_r3_meta.json").write_text(json.dumps({"el0_classes": CLASSES, "end_reason": ["window"] * N}))


def arms_from(edit):
    traces = {}
    for arm in ha.ARMS:
        tr = base_trace()
        edit(arm, tr)
        traces[arm] = tr
    return traces


class AnalysisTest(unittest.TestCase):
    def run_analysis(self, traces):
        with tempfile.TemporaryDirectory() as tmp:
            write_run(tmp, traces)
            return ha.analyse(tmp)

    def test_primary_denominator_counts_crash_as_failure(self):
        def edit(arm, tr):
            if arm == "B3":                      # reobserves env 0 after 3 steps
                tr["camera"][12, 0], tr["el0_visible_px"][12, 0] = True, 40
            if arm == "B2":                      # crashes before reobserving
                tr["crashed"][11, 0] = tr["reset"][11, 0] = True
                tr["camera"][13, 0], tr["el0_visible_px"][13, 0] = True, 40
        r = self.run_analysis(arms_from(edit))
        self.assertEqual(r["counts"]["matched_events"], 3)
        self.assertEqual(r["counts"]["primary_events"], 2)              # FULL + CENTRE_ONLY, not FOV_EXIT
        arms = r["primary"]["arms"]
        self.assertEqual([arms[a]["n"] for a in ha.ARMS], [2, 2, 2, 2])
        self.assertEqual(arms["B3"]["reobserved"], 1)
        self.assertAlmostEqual(arms["B3"]["median_time_s"], 0.3)
        self.assertEqual(arms["B2"]["reobserved"], 0)
        self.assertEqual(arms["B2"]["collision_in_window"], 1)
        c = r["primary"]["contrasts"]["B3-B2"]
        self.assertEqual((c["n10"], c["n01"]), (1, 0))
        self.assertAlmostEqual(c["rate_diff"], 0.5)
        self.assertIn("FOV_EXIT", r["separate_classes"])

    def test_camera_without_target_pixels_is_not_reobservation(self):
        def edit(arm, tr):
            if arm == "B3":
                tr["camera"][12, 0] = True       # detection but zero renderer pixels
        r = self.run_analysis(arms_from(edit))
        b3 = r["primary"]["arms"]["B3"]
        self.assertEqual(b3["reobserved"], 0)
        self.assertEqual(b3["camera_detection_without_target_pixels"], 1)

    def test_prefix_mismatch_and_t0_mismatch_are_unmatched(self):
        def edit(arm, tr):
            if arm == "B1":
                tr["digest"][5, 0] += np.uint64(1)
                tr["t0"][1] = 11
        r = self.run_analysis(arms_from(edit))
        reasons = {u["env"]: u["reason"] for u in r["unmatched"]}
        self.assertEqual(reasons, {0: "prefix digest differs", 1: "t0 differs"})
        self.assertEqual(r["counts"]["matched_events"], 1)
        self.assertEqual(r["coupling_onset_step_vs_B0"][4502]["B1"], 5)

    def test_kinematics_stop_before_terminal_and_target_diff(self):
        def edit(arm, tr):
            tr["robot_pos"][10:, 0, 0] = np.arange(1, STEPS - 9)
            if arm == "B1":
                tr["captured"][12, 0] = tr["reset"][12, 0] = True
                tr["robot_pos"][12:, 0] = 99.0   # post-reset state must be ignored
                tr["target_pos"][12:, 0] = -50.0
        r = self.run_analysis(arms_from(edit))
        ev = [e for e in r["events"] if e["env"] == 0][0]
        b1 = ev["results"]["B1"]
        self.assertAlmostEqual(b1["translation_m"], 2.0)
        self.assertAlmostEqual(b1["distance_change_m"], -2.0)
        self.assertTrue(b1["capture_in_window"])
        self.assertEqual(ev["target_max_diff_pre_terminal_m"], 0.0)

    def test_statistics(self):
        self.assertAlmostEqual(ha.mcnemar_exact(4, 0), 0.125)
        self.assertEqual(ha.mcnemar_exact(0, 0), 1.0)
        lo, hi = ha.wilson(4, 5)
        self.assertAlmostEqual(lo, 0.3755, places=3)
        self.assertAlmostEqual(hi, 0.9638, places=3)

    def test_imports_no_production_code(self):
        tree = ast.parse((ROOT / "tools/h_r3_analysis.py").read_text())
        mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        mods |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
        self.assertEqual(mods, {"__future__", "argparse", "json", "math", "pathlib", "numpy"})


if __name__ == "__main__":
    unittest.main()
