"""Q03 reacquisition candidate planner v0 CPU tests (synthetic ray-cast scenes; contract checks, not performance)."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q03_local_map as t3
from motar_eval.q02 import belief as bl, obstacle_map as om, reacq_planner as rp


def belief(xy=(8.0, 0.0), cov=0.01, valid=True):
    return bl.BeliefSnapshot((xy[0], xy[1], 1.5, 0.0, 0.0, 0.0), tuple(tuple(r) for r in np.diag([cov] * 3 + [0.1] * 3)), 0.0,
                             "TRACKER_DELIVERED", valid, () if valid else ("POSITION_UNCERTAINTY_TOO_LARGE",))


def plan(bars, b=None, vel=(0.0, 0.0, 0.0), yaw=0.0, now=0.0, limits=rp.MotionLimits(), empty_map=False):
    m = om.LocalObstacleMap()
    if not empty_map:
        inp, rep = t3.frame_at(bars, yaw=yaw); m.integrate(inp, rep)
    return rp.plan(m, (0.0, 0.0, 1.5), yaw, vel, b or belief(), now, limits=limits)


def by_kind(cands, kind):
    return [c for c in cands if c.kind == kind]


class PlannerTests(unittest.TestCase):
    def test_target_behind_bar_detours_are_proposed_with_separate_levels(self):
        cands, status, _ = plan(((4.0, 0.0, 0.3),))
        self.assertIsNone(status)
        for side in ("LEFT", "RIGHT"):
            ok = [c for c in by_kind(cands, side) if c.l2_kinematic_check == rp.PASS and "NO_VISIBILITY_GAIN" not in c.reasons]
            self.assertTrue(ok, side)
            self.assertTrue(all(c.visibility_blocked_share < c.current_blocked_share for c in ok))
        for c in cands:
            self.assertEqual((c.l3_dynamic_feasibility, c.l4_closed_loop_safety), (rp.NOT_VALIDATED, rp.NOT_VALIDATED))
            self.assertEqual(c.score_semantics, rp.SCORE_SEMANTICS)
            self.assertTrue(c.trajectory[0][0] == 0.0 and c.trajectory[-1][1:3] == c.position_xy)

    def test_only_left_or_only_right_detour(self):
        for blocked, free in (("LEFT", "RIGHT"), ("RIGHT", "LEFT")):
            y = 1.0 if blocked == "LEFT" else -1.0
            cands, status, _ = plan(((4.0, 0.0, 0.3), (0.3, y, 0.3), (1.2, 1.2 * y, 0.3)))
            with self.subTest(blocked=blocked):
                self.assertIsNone(status)
                self.assertTrue(all(c.l2_kinematic_check == rp.FAIL for c in by_kind(cands, blocked)))
                self.assertTrue(any(c.l2_kinematic_check == rp.PASS and "NO_VISIBILITY_GAIN" not in c.reasons
                                    for c in by_kind(cands, free)))

    def test_both_detours_possible(self):
        cands, status, _ = plan(((4.0, 0.0, 0.3),))
        self.assertTrue(all(any(c.l2_kinematic_check == rp.PASS for c in by_kind(cands, k)) for k in ("LEFT", "RIGHT")))

    def test_all_unknown_surroundings_give_no_plan(self):
        cands, status, why = plan((), empty_map=True)
        self.assertEqual((status, why), (rp.NO_FEASIBLE_PLAN, ("LOCAL_MAP_EMPTY",)))
        cands, status, why = plan(((4.0, 0.0, 0.3),), now=2.0)          # stale map: free space aged out
        self.assertEqual(status, rp.NO_FEASIBLE_PLAN); self.assertIn("LOCAL_MAP_STALE", why)
        self.assertTrue(all(c.l2_kinematic_check == rp.FAIL for c in cands))

    def test_path_through_an_obstacle_fails_L1_and_L2(self):
        cands, _, _ = plan(((4.0, 0.0, 0.3), (0.3, 1.0, 0.3), (1.2, 1.2, 0.3)))
        bad = by_kind(cands, "LEFT")[0]
        self.assertEqual((bad.l1_observed_direction_open, bad.l2_kinematic_check), (rp.FAIL, rp.FAIL))
        self.assertIn("PATH_INTERSECTS_OBSERVED_OCCUPANCY", bad.reasons)

    def test_candidates_that_do_not_help_fov_recovery_are_not_a_plan(self):
        cands, status, _ = plan(((4.0, 0.0, 0.3),))
        for kind in ("ROTATE", "ADVANCE", "RETREAT"):
            self.assertTrue(all("NO_VISIBILITY_GAIN" in c.reasons for c in by_kind(cands, kind)), kind)
        cands, status, why = plan(())                                      # target already visible: nothing to gain
        self.assertEqual(status, rp.NO_FEASIBLE_PLAN)

    def test_excessive_velocity_change_or_yaw(self):
        cands, _, _ = plan(((4.0, 0.0, 0.3),), vel=(0.0, -2.5, 0.0))       # moving fast to the right
        self.assertTrue(all("EXCESSIVE_VELOCITY_CHANGE" in c.reasons for c in by_kind(cands, "LEFT")))
        self.assertTrue(any(c.l2_kinematic_check == rp.PASS for c in by_kind(cands, "RIGHT")))
        slow_yaw = rp.MotionLimits(yaw_rate_cap=0.5)
        cands, _, _ = plan(((4.0, 0.0, 0.3),), b=belief((-8.0, 0.5)), limits=slow_yaw)   # belief behind: ~180 deg turn
        self.assertTrue(all("TRAVEL_TIME_ABOVE_LIMIT" in c.reasons for c in cands))

    def test_large_belief_covariance(self):
        cands, status, why = plan(((4.0, 0.0, 0.3),), b=belief(valid=False, cov=9.0))
        self.assertEqual((status, why[0]), (rp.NO_FEASIBLE_PLAN, "BELIEF_INVALID"))
        tight, _, _ = plan(((4.0, 0.0, 0.3),), b=belief(cov=0.01))
        wide, _, _ = plan(((4.0, 0.0, 0.3),), b=belief(cov=1.0))
        self.assertNotEqual([c.visibility_unknown_share for c in tight], [c.visibility_unknown_share for c in wide])

    def test_no_candidate_at_all(self):
        ring = tuple((0.9 * math.cos(a), 0.9 * math.sin(a), 0.25) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False))
        cands, status, _ = plan(ring + ((4.0, 0.0, 0.3),))
        self.assertEqual(status, rp.NO_FEASIBLE_PLAN)
        self.assertFalse(any(c.l2_kinematic_check == rp.PASS and c.kind != "ROTATE" for c in cands))

    def test_score_is_separate_from_hard_feasibility(self):
        cands, _, _ = plan(((4.0, 0.0, 0.3), (0.3, 1.0, 0.3), (1.2, 1.2, 0.3)))
        gained_but_infeasible = [c for c in cands if c.visibility_blocked_share < c.current_blocked_share and c.l2_kinematic_check == rp.FAIL]
        for c in gained_but_infeasible:
            self.assertIn("PATH_NOT_IN_OBSERVED_FREE", c.reasons)

    def test_numpy_ports_equal_the_boia_originals(self):
        import torch
        from motar_eval.boia.dynamics import braking_distance
        from motar_eval.boia.visibility import fov_mask
        rng = np.random.default_rng(3)
        rel = rng.uniform(-15, 15, (500, 3))
        ref = fov_mask(torch.tensor(rel), math.radians(87.0), math.radians(58.0), 20.0).numpy()
        self.assertTrue(np.array_equal(rp.fov_mask(rel, math.radians(87.0), math.radians(58.0), 20.0), ref))
        v = np.linspace(0.0, 5.0, 101)
        d_ref, x_ref = braking_distance(torch.tensor(v), t_react=0.1)
        d, x = rp.braking_distance(v, rp.MotionLimits().braking_table, 0.1)
        self.assertTrue(np.allclose(d, d_ref.numpy()) and np.array_equal(x, x_ref.numpy()))

    def test_deterministic(self):
        a = plan(((4.0, 0.0, 0.3),)); b = plan(((4.0, 0.0, 0.3),))
        self.assertEqual(a, b)


class PipelineIntegrationTests(unittest.TestCase):
    def test_shadow_record_carries_map_and_candidate_summaries(self):
        import test_q02b_timing as qb
        from motar_eval.q02 import shadow as sh
        out = qb.run([qb.SEEN] * 4 + [qb.OCC] * 12, policy="H3")
        props = [d for d in out if d.decision == sh.PROPOSE_REACQUIRE_SHADOW]
        self.assertTrue(props)
        p = props[0]
        self.assertGreater(p.candidate_summary["l2_pass_with_view_gain"], 0)
        self.assertEqual((p.candidate_summary["l3_dynamic_feasibility"], p.candidate_summary["l4_closed_loop_safety"]),
                         (rp.NOT_VALIDATED, rp.NOT_VALIDATED))
        self.assertTrue(p.local_map_status["accepted"]); self.assertFalse(p.local_map_status["ground_truth_used"])
        self.assertIn("EXECUTOR_NOT_AVAILABLE", p.reasons)
        self.assertTrue(all(d.owner == "F" and d.action is None and not d.action_applied for d in out))
        self.assertTrue(all(d.runtime_s is not None and d.runtime_s > 0 for d in out))


if __name__ == "__main__":
    unittest.main()
