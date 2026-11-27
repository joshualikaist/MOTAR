"""SL1D-C02 fixture design: every amendment scene is robustly what the amendment says (CPU)."""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import s_l1d_c02_scene_search as search  # noqa: E402

AMENDMENT = json.loads((ROOT / "docs/plans/s_l1d_v1_amendment1.json").read_text())


class SceneDesignTest(unittest.TestCase):
    def test_every_scene_stable_and_as_designed(self):
        expected = AMENDMENT["scene_design"]["expected_from_design"]
        classes = {}
        for scene in AMENDMENT["scenes"]:
            with self.subTest(scene=scene["id"]):
                got = search.verify_scene(scene)
                self.assertTrue(got["stable"], got["variants"])
                self.assertEqual(got["class"], expected[scene["id"]]["design_evaluator_class"])
                self.assertEqual(got["online"], expected[scene["id"]]["design_online_class"])
                self.assertEqual(got["ambiguous_px"], 0)
                self.assertGreaterEqual(got["margin_m"], 0.003)
                classes.setdefault(got["class"], []).append(scene["id"])
        # Each required state has at least two distinct geometries (non-vacuous design).
        self.assertGreaterEqual(len(classes.get("PARTIAL", [])), 2)
        self.assertGreaterEqual(len(classes.get("CENTRE_ONLY", [])), 2)
        self.assertGreaterEqual(len(classes.get("SCALE_LOSS", [])) + len(classes.get("VISIBLE", [])), 2)
        for sid in classes["PARTIAL"] + classes["CENTRE_ONLY"]:
            got = search.verify_scene(next(s for s in AMENDMENT["scenes"] if s["id"] == sid))
            self.assertEqual((got["alone_px"], got["visible_px"]), (2, 1))

    def test_amendment_keeps_rules_and_counts_unique_cases(self):
        self.assertEqual(AMENDMENT["run"]["sensor_num_envs"], len(AMENDMENT["scenes"]))
        self.assertIn("unchanged", AMENDMENT)
        self.assertIn("level scene pass", AMENDMENT["gates"]["C02-UNIQUE"])


if __name__ == "__main__":
    unittest.main()
