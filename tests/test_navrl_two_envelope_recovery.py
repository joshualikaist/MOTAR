"""CPU-only contracts for the fresh two-envelope route recovery lineage."""

import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


ROUTE = _load("route_recovery_route", "aerial_gym/task/navrl_task/target_route_planner.py")
MOTION = _load("route_recovery_motion", "aerial_gym/task/navrl_task/target_motion.py")


class TwoEnvelopeRecoveryTest(unittest.TestCase):
    def test_support_and_soft_inflation_are_frozen(self):
        support = ROUTE.conservative_xy_support_from_box([0.28, 0.28, 0.12])
        self.assertAlmostEqual(float(support[0]), 0.2068816087, places=9)
        self.assertAlmostEqual(float(support[0] + 0.45), 0.6568816087, places=9)

    def test_anchor_is_nearest_deterministic_and_hard_segment_safe(self):
        anchor = ROUTE.nearest_soft_free_anchor(
            [4.0, 5.0],
            np.asarray([[5.0, 5.0]], dtype=np.float64),
            np.asarray([[0.5, 0.5]], dtype=np.float64),
            [0.0, 0.0], [10.0, 10.0], [0.2, 0.2],
            0.5, 1.25, resolution_m=0.25, radius_cells=3, tracking_margin_m=0.45,
        )
        self.assertTrue(anchor["exists"])
        self.assertTrue(anchor["hard_connector_safe"])
        self.assertEqual(anchor["cell_ij"], [13, 19])
        # The candidate itself carries the release hysteresis, not merely soft-free > 0.
        self.assertGreaterEqual(anchor["xy_m"][0], 0.5 + 0.2 + 0.45 + 0.25)

    def test_anchor_fails_closed_for_nan_and_blocked_neighbourhood(self):
        bad = ROUTE.nearest_soft_free_anchor(
            [float("nan"), 0.0], [], np.empty((0, 2)), [0.0, 0.0], [10.0, 10.0],
            [0.2, 0.2], 0.5, 1.25,
        )
        self.assertIsNone(bad["exists"])
        bars = np.asarray([[5.0, y] for y in np.linspace(0.0, 10.0, 20)])
        blocked = ROUTE.nearest_soft_free_anchor(
            [4.0, 5.0], bars, np.full((20, 2), 1.0), [0.0, 0.0], [10.0, 10.0],
            [0.2, 0.2], 0.5, 1.25,
        )
        self.assertFalse(blocked["exists"])

    def test_exact_aabb_rollout_rejects_corner_that_rounded_test_accepts(self):
        kwargs = dict(
            old_xy=torch.tensor([[6.6, 6.6]]),
            current_velocity=torch.zeros((1, 2)),
            desired_velocity=torch.tensor([[1.0, 0.0]]),
            speed_limit=torch.tensor([0.0]), dt=0.1,
            bars_xy=torch.tensor([[[5.0, 5.0]]]),
            lo=torch.tensor([[0.0, 0.0]]), hi=torch.tensor([[10.0, 10.0]]),
            clearance=0.45, turn_sign=torch.ones(1),
            max_accel=torch.tensor([4.0]), max_turn_rate=torch.tensor([2.6179939]),
            lookahead_s=0.1,
            bars_half_extents_xy=torch.tensor([[[1.2, 1.2]]]),
        )
        _, _, _, rounded = MOTION.bounded_drone_target_step(**kwargs)
        soft_kwargs = dict(kwargs)
        soft_kwargs["bars_half_extents_xy"] = torch.tensor([[[1.65, 1.65]]])
        soft_kwargs["clearance"] = 0.0
        _, _, _, exact = MOTION.bounded_drone_target_step(
            **soft_kwargs, exact_aabb_clearance=True
        )
        self.assertTrue(bool(rounded.item()))
        self.assertFalse(bool(exact.item()))

    def test_no_teleport_or_reset_in_recovery_sources(self):
        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        self.assertIn("RECOVERY_NO_CONNECTOR", task)
        self.assertIn("recovery_anchor_idx", task)
        self.assertNotIn("self.target_position[recovery", task)
        self.assertIn("cfg_target_route_recovery_schema", task)

    def test_local_infeasible_soft_free_is_latched_separate_reason(self):
        manager = ROUTE.BatchedTargetRouteManager(
            1, torch.device("cpu"), ROUTE.RoutePlannerConfig(), recovery_enabled=True
        )
        mask = torch.tensor([True])
        manager.mark_local_infeasible_soft_free(mask)
        self.assertEqual(int(manager.recovery_state[0]), ROUTE.RECOVERY_NO_CONNECTOR)
        self.assertEqual(
            int(manager.status_code[0]),
            manager.STATUS_CODES["recovery_local_infeasible_soft_free"],
        )
        manager.recovery_state[0] = ROUTE.RECOVERY_ROUTE
        manager.enter_recovery(mask, 10)
        self.assertEqual(int(manager.recovery_state[0]), ROUTE.RECOVERY_BRAKE)

    def test_watchdog_source_has_continuous_segment_certificate(self):
        source = (ROOT / "aerial_gym/task/navrl_task/physical_target.py").read_text()
        self.assertIn("watchdog_prev_xy", source)
        self.assertIn("segment_hits_bar", source)
        self.assertIn("t_enter <= t_exit", source)
        # Both endpoints miss the closed obstacle, while the diagonal crosses its corner.
        self.assertFalse(
            ROUTE.segment_is_safe(
                [-1.0, -1.0], [1.0, 1.0], [-2.0, -2.0], [2.0, 2.0],
                np.asarray([[0.0, 0.0]]), np.asarray([[0.5, 0.5]]),
            )
        )

    def test_recovery_mode_is_not_v1_alias(self):
        self.assertEqual(ROUTE.TARGET_ROUTE_MODE_GLOBAL_ASTAR, "global_astar_v1")
        self.assertEqual(ROUTE.TARGET_ROUTE_MODE_RECOVERY, "global_astar_recovery_v2")
        self.assertNotEqual(ROUTE.TARGET_ROUTE_MODEL, ROUTE.TARGET_ROUTE_RECOVERY_MODEL)

    def test_v1_diagnostics_shape_is_unchanged(self):
        manager = ROUTE.BatchedTargetRouteManager(
            1, torch.device("cpu"), ROUTE.RoutePlannerConfig(), recovery_enabled=False
        )
        self.assertEqual(
            set(manager.diagnostics()),
            {
                "mode", "plan_attempts", "plan_successes", "replan_attempts",
                "connected_goal_replans", "same_goal_reselection_count", "no_path_count",
                "invalid_count", "local_step_invalidations", "fallback_intervals",
                "goal_completions", "invalidation_counts", "planning_batches", "planning_envs",
                "total_planning_wall_s", "max_batch_wall_s", "max_batch_size",
                "planning_wall_ms_per_env", "expanded_nodes", "raw_waypoints",
                "smoothed_waypoints", "currently_valid", "status_counts",
            },
        )

    def test_recovery_geometry_malformed_rows_fail_closed(self):
        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        self.assertIn("geometry_valid", task)
        self.assertIn("torch.isfinite(bars_xy)", task)
        self.assertIn('float("-inf")', task)

    def test_timeout_and_tube_contract_are_explicit(self):
        source = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        self.assertIn("recovery_brake_age_steps", source)
        self.assertIn("recovery_connect_age_steps", source)
        self.assertIn("max_anchor_distance", source)
        self.assertIn("TARGET_ROUTE_REACHABLE_TUBE_MARGIN_M", source)


if __name__ == "__main__":
    unittest.main()
