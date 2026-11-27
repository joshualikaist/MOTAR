"""EQ-1C information flow (CPU): actor firewall, checkpoint/RMS invariance, loss separation, populations.

The dynamic firewall runs the production perception module on CPU. The checkpoint check needs
Policy F in the main checkout, and the population/loss checks read the committed evidence record,
so a clean clone without raw results or checkpoints skips only those parts.
"""
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import eq1_common as common  # noqa: E402
import eq1_information_flow as flow  # noqa: E402

EVIDENCE = ROOT / "docs/evidence/eq1_validation_2026-10-18/eq1c_information_flow.json"


class IsolationTest(unittest.TestCase):
    """Production loading must not leak stubs, 72-beam modules or receipt variables into other tests."""

    def test_context_restores_modules_and_environment(self):
        import os
        modules, environ = set(sys.modules), dict(os.environ)
        with common.production({"NAVRL_EQ1_PROBE": "1"}):
            p = common.load_production("aerial_gym.task.navrl_task.navrl_perception")
            self.assertEqual(p.HBEAMS, 72)
            self.assertEqual(os.environ["NAVRL_EQ1_PROBE"], "1")
        self.assertEqual(set(sys.modules) - modules, set())
        self.assertEqual(dict(os.environ), environ)
        with self.assertRaises(RuntimeError):
            common.load_production("aerial_gym.task.navrl_task.navrl_perception")


class StaticFirewallTest(unittest.TestCase):
    def test_actor_written_only_from_observe(self):
        s = flow.static_firewall()
        self.assertEqual(s["status"], "PASS", s)
        self.assertEqual(s["observe_calls"], 1)
        self.assertEqual(s["forbidden_argument_reads"], {})
        self.assertIn("rgb", s["observe_keywords"])


@unittest.skipUnless(flow.RECEIPT.exists(), "OCC-P2 receipt not present")
class DynamicFirewallTest(unittest.TestCase):
    def test_held_fixed_input_invariance(self):
        d = flow.dynamic_firewall()
        self.assertEqual(d["actor_width"], 898)
        for key in ("fresh_instances_bitwise_equal", "rng_state_changed_bitwise_equal",
                    "permutation_undone_bitwise_equal", "positive_control_training_rng_differs"):
            self.assertTrue(d[key], key)
        self.assertEqual(d["status"], "PASS")
        self.assertEqual(set(d["batch_size_env0"]), {"1", "2", "4", "128"})


@unittest.skipUnless((common.MAIN_CHECKOUT / common.POLICY_F).exists(), "Policy F checkpoint not present")
class CheckpointTest(unittest.TestCase):
    def test_identity_and_rms_eval_invariance(self):
        c = flow.checkpoint_rms()
        self.assertEqual(c["sha256"], common.POLICY_F_SHA256)
        self.assertEqual(c["rms_input_width"], 898)
        self.assertTrue(c["eval_forward_leaves_state_unchanged"])
        self.assertTrue(c["train_forward_changes_state"])
        self.assertEqual(c["status"], "PASS")


@unittest.skipUnless(EVIDENCE.exists(), "EQ-1C evidence not present")
class RecordedEvidenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = json.loads(EVIDENCE.read_text())

    def test_populations(self):
        p = self.r["C5_populations"]
        self.assertEqual(p["cells"], 112)
        self.assertTrue(p["quota_rule_reproduced"])
        self.assertTrue(p["legacy_rule_reproduced"])
        self.assertEqual(p["legacy_rule_checked_cells"], 112)
        self.assertEqual(p["cells_outcome_counts_differ"], 96)
        self.assertEqual(p["same_population"]["status"], "FAIL")

    def test_loss_separation_partitions_camera_loss(self):
        t = self.r["C4_loss_separation"]["by_group_density"]["all"]
        self.assertEqual(t["fov_exit"] + t["geometric_centre_occlusion_in_view"] + t["in_view_clear_camera_loss"],
                         t["camera_lost_frames"])
        self.assertEqual(t["in_view_clear_camera_loss_beyond_scale_band"] + t["in_view_clear_camera_loss_within_scale_band"],
                         t["in_view_clear_camera_loss"])
        self.assertEqual(self.r["C4_loss_separation"]["identity_loss"]["status"], "NOT_RECORDED")


if __name__ == "__main__":
    unittest.main()
