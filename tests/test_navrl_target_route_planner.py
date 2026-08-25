"""CPU contracts for the opt-in physical-target global route planner."""

import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "aerial_gym/task/navrl_task/target_route_planner.py"
SPEC = importlib.util.spec_from_file_location("navrl_target_route_planner_standalone", PATH)
ROUTE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ROUTE
SPEC.loader.exec_module(ROUTE)


def planner(resolution=0.20, tracking=0.10, boundary=0.0):
    return ROUTE.DeterministicAStarRoutePlanner(
        ROUTE.RoutePlannerConfig(
            resolution_m=resolution,
            tracking_margin_m=tracking,
            boundary_margin_m=boundary,
            max_expansions=100000,
            max_waypoints=128,
        )
    )


def plan(p, bars, half, start=(1.0, 5.0), goal=(9.0, 5.0), support=(0.2, 0.2)):
    return p.plan(
        np.asarray(start), np.asarray(goal), np.asarray(bars).reshape((-1, 2)),
        np.asarray(half).reshape((-1, 2)), np.array([0.0, 0.0]), np.array([10.0, 10.0]),
        np.asarray(support),
    )


class TargetRoutePlannerTest(unittest.TestCase):
    def test_support_envelope_is_full_orientation_half_diagonal(self):
        support = ROUTE.conservative_xy_support_from_box((0.28, 0.28, 0.12))
        expected = 0.5 * np.linalg.norm([0.28, 0.28, 0.12])
        np.testing.assert_allclose(support, [expected, expected])

    def test_blocked_straight_line_routes_around_exact_aabb(self):
        p = planner()
        result = plan(p, [(5.0, 5.0)], [(0.8, 2.4)])
        self.assertTrue(result.valid, result.status)
        self.assertGreater(result.smoothed_nodes, 2)
        inflated = np.array([[1.1, 2.7]])
        for a, b in zip(result.waypoints_xy[:-1], result.waypoints_xy[1:]):
            self.assertTrue(
                ROUTE.segment_is_safe(
                    a, b, np.array([0.2, 0.2]), np.array([9.8, 9.8]),
                    np.array([[5.0, 5.0]]), inflated,
                )
            )

    def test_full_height_wall_fails_closed(self):
        result = plan(planner(), [(5.0, 5.0)], [(0.3, 5.0)])
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "no_path")
        self.assertEqual(result.waypoints_xy.shape, (0, 2))

    def test_narrow_corridor_passes_only_above_inflated_width(self):
        # Two long horizontal AABBs. With support+tracking=0.30 m per wall, a 1.2 m raw opening
        # leaves 0.6 m for the target centre; a 0.5 m opening closes completely.
        centers = [(5.0, 3.4), (5.0, 6.6)]
        wide = plan(planner(), centers, [(5.0, 1.0), (5.0, 1.0)])
        self.assertTrue(wide.valid, wide.status)
        narrow_centers = [(5.0, 4.25), (5.0, 5.75)]
        narrow = plan(planner(), narrow_centers, [(5.0, 0.5), (5.0, 0.5)])
        self.assertFalse(narrow.valid)
        self.assertIn(narrow.status, ("unsafe_start", "unsafe_start_cell", "no_path"))

    def test_reflection_and_repeat_are_deterministic(self):
        p = planner(resolution=0.25)
        bars = np.array([[4.0, 3.0], [6.0, 3.0]])
        half = np.array([[0.8, 3.0], [0.8, 3.0]])
        first = plan(p, bars, half, start=(1.0, 5.0), goal=(9.0, 5.0))
        second = plan(p, bars, half, start=(1.0, 5.0), goal=(9.0, 5.0))
        self.assertTrue(first.valid, first.status)
        np.testing.assert_array_equal(first.waypoints_xy, second.waypoints_xy)

        reflected_bars = bars.copy()
        reflected_bars[:, 1] = 10.0 - reflected_bars[:, 1]
        reflected = plan(
            p, reflected_bars, half, start=(1.0, 5.0), goal=(9.0, 5.0)
        )
        self.assertTrue(reflected.valid, reflected.status)
        expected = first.waypoints_xy.copy()
        expected[:, 1] = 10.0 - expected[:, 1]
        np.testing.assert_allclose(reflected.waypoints_xy, expected, atol=0.26)

    def test_line_of_sight_rejects_aabb_corner_cut(self):
        lo, hi = np.array([0.0, 0.0]), np.array([3.0, 3.0])
        bars = np.array([[1.0, 1.0]])
        half = np.array([[0.2, 0.2]])
        self.assertFalse(
            ROUTE.segment_is_safe(
                np.array([0.2, 0.2]), np.array([1.8, 1.8]), lo, hi, bars, half
            )
        )
        self.assertTrue(
            ROUTE.segment_is_safe(
                np.array([0.2, 0.2]), np.array([1.8, 0.5]), lo, hi, bars, half
            )
        )

    def test_nonfinite_geometry_is_invalid_not_optimistically_empty(self):
        result = plan(planner(), [(float("nan"), 5.0)], [(0.5, 0.5)])
        self.assertFalse(result.valid)
        self.assertEqual(result.status, "invalid_input")

    def test_connected_goal_sampling_returns_proved_distant_route(self):
        p = planner()
        result = p.plan_to_connected_goal(
            np.array([1.0, 5.0]), np.array([[5.0, 5.0]]), np.array([[0.8, 2.4]]),
            np.array([0.0, 0.0]), np.array([10.0, 10.0]), np.array([0.2, 0.2]),
            min_goal_distance_m=4.0, selector=0.37,
        )
        self.assertTrue(result.valid, result.status)
        self.assertGreaterEqual(np.linalg.norm(result.waypoints_xy[-1] - result.waypoints_xy[0]), 4.0)

    def test_waypoint_overshoot_advances_without_exact_radius_hit(self):
        config = ROUTE.RoutePlannerConfig(
            resolution_m=0.25, tracking_margin_m=0.0, boundary_margin_m=0.0,
            max_waypoints=8,
        )
        manager = ROUTE.BatchedTargetRouteManager(1, torch.device("cpu"), config)
        manager.valid[0] = True
        manager.length[0] = 2
        manager.cursor[0] = 0
        manager.segment_start[0] = torch.tensor([0.0, 0.0])
        manager.waypoints[0, 0] = torch.tensor([1.0, 0.0])
        manager.waypoints[0, 1] = torch.tensor([2.0, 0.0])
        velocity, active, complete = manager.velocity_reference(
            torch.tensor([[1.2, 0.0]]), torch.tensor([1.0]), reach_m=0.05
        )
        self.assertEqual(int(manager.cursor[0]), 1)
        self.assertTrue(bool(active[0]))
        self.assertFalse(bool(complete[0]))
        self.assertGreater(float(velocity[0, 0]), 0.0)

        # Crossing the waypoint plane far away from the segment is not progress along the route.
        manager.cursor[0] = 0
        manager.segment_start[0] = torch.tensor([0.0, 0.0])
        manager.velocity_reference(
            torch.tensor([[1.2, 2.0]]), torch.tensor([1.0]), reach_m=0.5
        )
        self.assertEqual(int(manager.cursor[0]), 0)

        manager.cursor[0] = 1
        manager.segment_start[0] = torch.tensor([1.0, 0.0])
        _, _, complete = manager.velocity_reference(
            torch.tensor([[2.2, 0.0]]), torch.tensor([1.0]), reach_m=0.5
        )
        self.assertTrue(bool(complete[0]))
        manager.velocity_reference(
            torch.tensor([[2.2, 0.0]]), torch.tensor([1.0]), reach_m=0.5
        )
        self.assertEqual(manager.diagnostics()["goal_completions"], 1)

    def test_local_failure_is_distinct_and_requires_connected_goal_replacement(self):
        config = ROUTE.RoutePlannerConfig(replan_cooldown_steps=3)
        manager = ROUTE.BatchedTargetRouteManager(1, torch.device("cpu"), config)
        ids = torch.tensor([0])
        start = torch.tensor([[5.0, 5.0]])
        arbitrary_goal = torch.tensor([[8.0, 5.0]])
        bars = torch.empty((1, 0, 2))
        bounds_lo = torch.tensor([[0.0, 0.0]])
        bounds_hi = torch.tensor([[10.0, 10.0]])
        support = torch.tensor([[0.1, 0.1]])
        manager.plan_idx(
            ids, start, arbitrary_goal, bars, bars, bounds_lo, bounds_hi, support,
            current_step=0, connected_goal_selector=torch.tensor([0.1]),
            min_goal_distance_m=2.0,
        )
        first_goal = manager.goal.clone()
        manager.invalidate(
            torch.tensor([True]), "local_step_infeasible", current_step=10
        )
        self.assertEqual(
            int(manager.status_code[0]), manager.STATUS_CODES["local_step_infeasible"]
        )
        self.assertFalse(bool(manager.needs_replan(manager.goal, manager.planned_support, 12)[0]))
        self.assertTrue(bool(manager.needs_replan(manager.goal, manager.planned_support, 13)[0]))
        manager.plan_idx(
            ids, start, arbitrary_goal, bars, bars, bounds_lo, bounds_hi, support,
            current_step=13, is_replan=True,
            connected_goal_selector=torch.tensor([0.8]), min_goal_distance_m=2.0,
        )
        self.assertFalse(torch.equal(first_goal, manager.goal))
        self.assertEqual(manager.diagnostics()["local_step_invalidations"], 1)
        self.assertEqual(manager.diagnostics()["connected_goal_replans"], 1)
        diagnostics = manager.diagnostics()
        self.assertEqual(diagnostics["invalidation_counts"]["local_step_infeasible"], 1)
        self.assertGreaterEqual(diagnostics["planning_batches"], 2)
        self.assertGreaterEqual(diagnostics["planning_envs"], 2)
        self.assertGreaterEqual(diagnostics["total_planning_wall_s"], 0.0)
        self.assertGreaterEqual(diagnostics["max_batch_wall_s"], 0.0)
        self.assertGreaterEqual(diagnostics["planning_wall_ms_per_env"], 0.0)


if __name__ == "__main__":
    unittest.main()
