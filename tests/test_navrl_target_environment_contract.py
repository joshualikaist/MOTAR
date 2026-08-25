"""CPU-only guards for target-motion training-environment provenance.

These tests intentionally inspect source rather than importing NavRLTask, whose module import
requires Isaac Gym.  They protect fail-loud checkpoint metadata and fresh-lineage launcher rules;
they do not claim dynamic feasibility or policy performance.
"""

from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
TASK_SOURCE = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text(encoding="utf-8")
PHYSICAL_LAUNCHER = (
    ROOT / "aerial_gym/rl_training/rl_games/train_navrl_physical_fresh.sh"
).read_text(encoding="utf-8")
V2_LAUNCHER = (
    ROOT / "aerial_gym/rl_training/rl_games/train_navrl_v2_search.sh"
).read_text(encoding="utf-8")


class TargetCheckpointContractTest(unittest.TestCase):
    def test_all_motion_authority_fields_are_saved_and_checked_on_restore(self):
        keys = (
            "cfg_target_max_accel_mps2",
            "cfg_target_max_turn_rate_degps",
            "cfg_target_lookahead_s",
            "cfg_target_obstacle_clearance_m",
            "cfg_target_physical_motor_arm_xy_m",
            "cfg_target_physical_yaw_torque_ratio_m",
            "cfg_target_physical_max_tilt_deg",
            "cfg_target_physical_tracking_margin_m",
            "cfg_target_physical_boundary_margin_m",
        )
        # One occurrence serializes the field and a second occurrence compares it during restore.
        for key in keys:
            self.assertGreaterEqual(TASK_SOURCE.count(f'"{key}"'), 2, key)

    def test_old_checkpoints_remain_compatible(self):
        # Missing provenance is tolerated; present mismatches are loud and discard curriculum
        # evidence. This is the compatibility rule that keeps legacy checkpoints loadable.
        self.assertRegex(TASK_SOURCE, r"saved_value = state\.get\(key\)\s+if saved_value is None:\s+continue")


class TargetLauncherContractTest(unittest.TestCase):
    def test_physical_lineage_is_fresh_only_and_forces_matching_airframe(self):
        self.assertIn("refusing CKPT/CHECKPOINT", PHYSICAL_LAUNCHER)
        self.assertIn("export NAVRL_ROBOT=navrl_ref5in_quad", PHYSICAL_LAUNCHER)
        self.assertIn("export NAVRL_TARGET_DYNAMICS=physical", PHYSICAL_LAUNCHER)
        self.assertIn("export NAVRL_V2_PHYSICAL_FRESH_CHILD=1", PHYSICAL_LAUNCHER)

    def test_base_v2_refuses_stale_nonlegacy_target_dynamics(self):
        self.assertIn("refusing inherited target dynamics", V2_LAUNCHER)
        self.assertIn('export NAVRL_TARGET_DYNAMICS="${_TARGET_DYNAMICS_REQUESTED}"', V2_LAUNCHER)
        self.assertIn('if [[ "${NAVRL_V2_PHYSICAL_FRESH_CHILD:-0}" == "1" ]]', V2_LAUNCHER)

    def test_v2_pins_timing_geometry_and_motion_distribution(self):
        required = {
            "NAVRL_ARENA_XY": "40",
            "NAVRL_ARENA_Z": "3",
            "NAVRL_BAR_POOL": "bars_h3",
            "NAVRL_PLACEMENT_MODE": "navrl_band",
            "NAVRL_EPISODE_LEN_STEPS": "600",
            "NAVRL_MAX_BARS": "300",
            "NAVRL_TARGET_SPEED_MIN": "0.3",
            "NAVRL_TARGET_SPEED_FINAL": "1.5",
            "NAVRL_TARGET_PATTERN": "mixed",
        }
        for name, value in required.items():
            self.assertRegex(V2_LAUNCHER, rf"export {name}=(?:\"\$\{{[^\n]+:-)?{re.escape(value)}")


if __name__ == "__main__":
    unittest.main()
