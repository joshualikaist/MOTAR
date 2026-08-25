"""CPU-only guards for target-motion training-environment provenance.

These tests intentionally inspect source rather than importing NavRLTask, whose module import
requires Isaac Gym.  They protect fail-loud checkpoint metadata and fresh-lineage launcher rules;
they do not claim dynamic feasibility or policy performance.
"""

from pathlib import Path
import os
import re
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
TASK_SOURCE = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text(encoding="utf-8")
PHYSICAL_LAUNCHER = (
    ROOT / "aerial_gym/rl_training/rl_games/train_navrl_physical_fresh.sh"
).read_text(encoding="utf-8")
V2_LAUNCHER = (
    ROOT / "aerial_gym/rl_training/rl_games/train_navrl_v2_search.sh"
).read_text(encoding="utf-8")
ROUTED_LAUNCHER_PATH = (
    ROOT / "aerial_gym/rl_training/rl_games/train_navrl_physical_routed_fresh.sh"
)
ROUTED_LAUNCHER = ROUTED_LAUNCHER_PATH.read_text(encoding="utf-8")
RL = ROOT / "aerial_gym/rl_training/rl_games"


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
            "cfg_target_route_mode",
            "cfg_target_route_resolution_m",
            "cfg_target_route_max_expansions",
            "cfg_target_route_max_waypoints",
            "cfg_target_route_replan_cooldown_steps",
            "cfg_target_route_goal_tolerance_m",
            "cfg_target_route_min_goal_distance_m",
        )
        # One occurrence serializes the field and a second occurrence compares it during restore.
        for key in keys:
            self.assertGreaterEqual(TASK_SOURCE.count(f'"{key}"'), 2, key)

    def test_old_checkpoints_remain_compatible(self):
        # Missing provenance is tolerated; present mismatches are loud and discard curriculum
        # evidence. This is the compatibility rule that keeps legacy checkpoints loadable.
        self.assertRegex(TASK_SOURCE, r"saved_value = state\.get\(key\)\s+if saved_value is None:\s+continue")

    def test_local_route_failure_replaces_goal_after_cooldown(self):
        self.assertIn('STATUS_CODES["local_step_infeasible"]', TASK_SOURCE)
        local_branch = TASK_SOURCE.index("if bool(local_failure.any()):")
        ordinary_branch = TASK_SOURCE.index("ordinary_replan =", local_branch)
        self.assertIn(
            "connected_goal=True",
            TASK_SOURCE[local_branch:ordinary_branch],
        )
        invalidation = TASK_SOURCE.index("local_invalid =")
        self.assertIn(
            "torch.rand_like",
            TASK_SOURCE[invalidation:invalidation + 700],
        )


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

    def test_route_lineage_has_distinct_marker_and_fresh_guard(self):
        self.assertIn("refusing CKPT/CHECKPOINT", ROUTED_LAUNCHER)
        self.assertIn("export NAVRL_PHYSICAL_ROUTED_CHILD=1", ROUTED_LAUNCHER)
        self.assertIn("export NAVRL_TARGET_ROUTE_MODE=global_astar_v1", ROUTED_LAUNCHER)
        self.assertIn("export NAVRL_TARGET_PATTERN=waypoint", ROUTED_LAUNCHER)
        self.assertIn("canonical physical lineage refuses target route", PHYSICAL_LAUNCHER)

    def _preflight(self, launcher, **updates):
        env = {
            **os.environ,
            "NAVRL_TARGET_CONTRACT_PREFLIGHT_ONLY": "1",
            "PYTHONNOUSERSITE": "1",
            **updates,
        }
        return subprocess.run(
            [str(launcher)], cwd=RL, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False,
        )

    def test_base_physical_and_routed_preflight_contracts(self):
        base = self._preflight(
            RL / "train_navrl_v2_search.sh",
            NAVRL_TARGET_DYNAMICS="legacy", NAVRL_TARGET_ROUTE_MODE="off",
        )
        self.assertEqual(base.returncode, 0, base.stdout)
        self.assertIn("route=off pattern=mixed", base.stdout)

        stale = self._preflight(
            RL / "train_navrl_physical_fresh.sh",
            NAVRL_TARGET_ROUTE_MODE="global_astar_v1", NAVRL_TARGET_PATTERN="waypoint",
        )
        self.assertNotEqual(stale.returncode, 0, stale.stdout)
        self.assertIn("canonical physical lineage refuses target route", stale.stdout)

        routed = self._preflight(ROUTED_LAUNCHER_PATH)
        self.assertEqual(routed.returncode, 0, routed.stdout)
        self.assertIn("route=global_astar_v1 pattern=waypoint", routed.stdout)

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
