"""Focused CPU checks of P03's prefix-budget and direction controls."""
import ast
import hashlib
import inspect
import math
from pathlib import Path
import random
import sys
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval import h_r3_arms as frozen
from motar_eval import h_r3_p03_arms as new

ANGLES = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72)
ELEV = torch.deg2rad(torch.tensor([20., 10., 0., -10.]))


def wall(x, low, high):
    """Independent intersection with a vertical finite wall x=constant."""
    ranges = torch.full((4, 72), 12.)
    for col, angle in enumerate(ANGLES.tolist()):
        if math.cos(angle) > 0 and low <= x * math.tan(angle) <= high:
            for row, elevation in enumerate(ELEV.tolist()):
                ranges[row, col] = x / (math.cos(angle) * math.cos(elevation))
    return ranges.reshape(1, -1)


def inputs(scan=None, yaw=0., belief=(6., 0., 1.)):
    return (torch.tensor([[0., 0., 1.]]),
            torch.tensor([[0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]]),
            torch.tensor([belief]), torch.full((1, 288), 12.) if scan is None else scan,
            ANGLES.clone(), ELEV.clone())


def freeze(args, cap=2.5):
    return new.freeze_schedule(*args, max_velocity=cap, yaw_rate_max=3., lidar_far_m=12.)


def evaluate(args, schedule=None, decision=0, sign=1., cap=2.5, **kwargs):
    return new.evaluate_actions(*args, schedule=freeze(args, cap) if schedule is None else schedule,
                                decision_index=decision, sham_sign=torch.tensor([sign]),
                                max_velocity=cap, yaw_rate_max=3., lidar_far_m=12., **kwargs)


class PrefixBudgetTest(unittest.TestCase):
    def test_freeze_has_fixed_detached_20_step_window(self):
        args = inputs(wall(3., -.3, .3))
        schedule = freeze(args)
        self.assertEqual(schedule["window_steps"], 20)
        self.assertEqual(schedule["dt_s"], .1)
        lateral = schedule["lateral_abs_mps"]
        self.assertEqual(tuple(lateral.shape), (1, 20))
        self.assertGreater(float(lateral[0, 0]), 0)
        self.assertTrue(torch.equal(lateral, lateral[:, :1].expand(-1, 20)))
        self.assertFalse(lateral.requires_grad)
        expected = math.sqrt(1. + float(lateral[0, 0]) ** 2)
        self.assertAlmostEqual(float(schedule["reference_norm_mps"][0, 0]), expected, places=6)

    def test_clear_centre_keeps_feasible_prefix_lateral_budget(self):
        schedule = freeze(inputs())
        self.assertEqual(schedule["prefix_chosen_offset_m"].tolist(), [0.])
        self.assertEqual(schedule["prefix_budget_offset_abs_m"].tolist(), [2.])
        self.assertEqual(schedule["lateral_abs_mps"][:, 0].tolist(), [1.5])
        self.assertEqual(schedule["zero_lateral"].tolist(), [False])

    def test_online_changes_cannot_reestimate_magnitude(self):
        schedule = freeze(inputs(wall(3., -.3, .3)))
        before = {key: value.clone() if isinstance(value, torch.Tensor) else value for key, value in schedule.items()}
        for args in (inputs(), inputs(wall(.8, -2., 2.)), inputs(wall(3., -.8, .2), yaw=.4)):
            _, info = evaluate(args, schedule, decision=19)
            self.assertTrue(torch.equal(info["lateral_abs_mps"], schedule["lateral_abs_mps"][:, 19]))
        for key, value in before.items():
            if isinstance(value, torch.Tensor):
                self.assertTrue(torch.equal(value, schedule[key]), key)

    def test_nominal_common_norm_and_sham_side_across_future_states(self):
        schedule = freeze(inputs(wall(3., -.3, .3)))
        for args in (inputs(), inputs(wall(.8, -.2, .2)), inputs(yaw=.4, belief=(4., 2., 1.))):
            for sign in (-1., 1.):
                actions, info = evaluate(args, schedule, sign=sign)
                reference = info["reference_norm_mps"]
                for arm in ("B3", "B4", "B5"):
                    torch.testing.assert_close(info["arms"][arm]["nominal_norm_mps"], reference, rtol=0, atol=3e-7)
                    self.assertTrue(torch.equal(actions[arm][:, 3], actions["B2"][:, 3]))
                rel = frozen._quat_rotate_inverse(args[1], args[2] - args[0])[:, :2]
                u = rel / rel.norm(dim=1, keepdim=True)
                normal = torch.stack([-u[:, 1], u[:, 0]], dim=1)
                lateral_component = (info["arms"]["B4"]["nominal_xy"] * normal).sum(dim=1)
                torch.testing.assert_close(lateral_component, sign * info["lateral_abs_mps"], rtol=0, atol=2e-7)
                self.assertAlmostEqual(float((info["arms"]["B5"]["nominal_xy"] * normal).sum()), 0., places=6)

    def test_informed_side_responds_online_but_sham_and_magnitude_do_not(self):
        schedule = freeze(inputs(wall(3., -.3, .3)))
        _, left = evaluate(inputs(wall(3., -.8, .2)), schedule, sign=-1.)
        _, right = evaluate(inputs(wall(3., -.2, .8)), schedule, sign=-1.)
        self.assertEqual(left["selected_side"].tolist(), [1.])
        self.assertEqual(right["selected_side"].tolist(), [-1.])
        self.assertTrue(torch.equal(left["arms"]["B4"]["nominal_xy"], right["arms"]["B4"]["nominal_xy"]))
        self.assertTrue(torch.equal(left["lateral_abs_mps"], right["lateral_abs_mps"]))
        self.assertGreater(float(left["arms"]["B3"]["nominal_xy"][0, 1]), 0.)
        self.assertLess(float(right["arms"]["B3"]["nominal_xy"][0, 1]), 0.)

    def test_frozen_b1_b2_and_b0_passthrough(self):
        for args in (inputs(), inputs(wall(.6, -1., 1.)), inputs(yaw=.4, belief=(5., 2., 1.))):
            actor = torch.tensor([[.21, -.32, .43, -.54]])
            actions, _ = evaluate(args, frozen_action=actor)
            self.assertTrue(torch.equal(actions["B0"], actor))
            self.assertNotEqual(actions["B0"].data_ptr(), actor.data_ptr())
            for arm in ("B1", "B2"):
                expected, _ = frozen.scripted_action(arm, *args, max_velocity=2.5, yaw_rate_max=3., lidar_far_m=12.)
                self.assertTrue(torch.equal(actions[arm], expected), arm)

    def test_zero_lateral_and_no_candidate_keep_denominators(self):
        for args in (inputs(wall(.6, -10., 10.)), inputs(belief=(0., 0., 1.))):
            schedule = freeze(args)
            actions, info = evaluate(args, schedule)
            self.assertEqual(schedule["zero_lateral"].tolist(), [True])
            self.assertEqual(info["zero_lateral"].tolist(), [True])
            self.assertEqual(info["lateral_abs_mps"].numel(), 1)
            for arm in ("B3", "B4", "B5"):
                self.assertTrue(torch.equal(actions[arm], actions["B2"]))

    def test_coincident_belief_reports_abstention_without_fabricated_direction(self):
        schedule = freeze(inputs(wall(3., -.3, .3)))
        actions, info = evaluate(inputs(belief=(0., 0., 1.)), schedule)
        self.assertEqual(info["direction_valid"].tolist(), [False])
        self.assertEqual(info["zero_lateral"].tolist(), [False])
        for arm in new.SCRIPTED_ARMS:
            self.assertTrue(torch.equal(actions[arm], torch.zeros_like(actions[arm])))

    def test_near_zero_belief_direction_abstains_but_threshold_is_valid(self):
        schedule = freeze(inputs(wall(3., -.3, .3)))
        for distance in (1e-8, 5e-7):
            args = inputs(belief=(distance, 0., 1.))
            actions, info = evaluate(args, schedule)
            self.assertEqual(info["direction_valid"].tolist(), [False])
            for arm in new.SCRIPTED_ARMS:
                self.assertTrue(torch.equal(info["arms"][arm]["nominal_xy"], torch.zeros(1, 2)),
                                (arm, distance))
                self.assertTrue(torch.equal(actions[arm], torch.zeros(1, 4)), (arm, distance))
            self.assertTrue(bool(freeze(args)["zero_lateral"][0]))
        _, info = evaluate(inputs(belief=(1e-6, 0., 1.)), schedule)
        self.assertEqual(info["direction_valid"].tolist(), [True])
        for arm in ("B3", "B4", "B5"):
            torch.testing.assert_close(info["arms"][arm]["nominal_norm_mps"],
                                       info["reference_norm_mps"], rtol=0, atol=3e-7)

    def test_reference_is_bounded_without_reversing_safety(self):
        args = inputs(wall(3., -.3, .3))
        schedule = freeze(args, cap=1.2)
        self.assertLessEqual(float(schedule["reference_norm_mps"].max()), 1.2)
        _, info = evaluate(inputs(wall(.8, -.2, .2)), schedule, cap=1.2)
        for stage in info["arms"].values():
            self.assertTrue(bool((stage["post_point_norm_mps"] <= stage["nominal_norm_mps"] + 1e-6).all()))
            self.assertTrue(bool((stage["post_cap_norm_mps"] <= stage["post_point_norm_mps"] + 1e-6).all()))
            self.assertGreaterEqual(float(stage["safety_attenuation_mps"][0]), -1e-6)
        self.assertGreater(abs(float(info["arms"]["B3"]["post_cap_norm_mps"][0] -
                                    info["arms"]["B5"]["post_cap_norm_mps"][0])), .1)

    def test_budget_metadata_uses_full_registered_window(self):
        _, info = evaluate(inputs(wall(3., -.3, .3)), decision=19)
        ref = info["reference_norm_mps"]
        torch.testing.assert_close(info["reference_path_m"], 2. * ref)
        torch.testing.assert_close(info["reference_squared_speed_integral_m2_s"], 2. * ref.square())

    def test_invalid_schedule_decision_and_inputs_fail_closed(self):
        args, schedule = inputs(), freeze(inputs())
        for index in (-1, 20, .5, float("nan"), [0, 1]):
            with self.assertRaises(ValueError):
                evaluate(args, schedule, decision=index)
        for key, value in (("dt_s", .2), ("window_steps", 21),
                           ("lateral_abs_mps", torch.zeros(1, 19)),
                           ("reference_norm_mps", torch.zeros(1, 20))):
            with self.assertRaises(ValueError):
                evaluate(args, dict(schedule, **{key: value}))
        for sign in (0, float("nan")):
            with self.assertRaises(ValueError):
                evaluate(args, sign=sign)
        for cap in (0., -1., .7, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                freeze(args, cap)
        for index in range(6):
            bad = [x.clone() for x in args]
            bad[index].reshape(-1)[0] = float("nan")
            with self.assertRaises(ValueError):
                freeze(bad)


class AssignmentAndFirewallTest(unittest.TestCase):
    def test_hash_matches_full_registered_domain_and_key(self):
        actual = new.sham_signs(4701, [12], [3], [5], [9]).item()
        expected = 1. if hashlib.sha256(b"HR3-P03|4701|12|3|5|9").digest()[0] & 1 else -1.
        self.assertEqual(actual, expected)

    def test_assignment_is_batch_order_and_reset_order_independent(self):
        keys = ([7, 2, 100, 0], [1, 3, 0, 9], [0, 1, 2, 3], [0, 8, 19, 5])
        batch = new.sham_signs(4701, *keys)
        reversed_batch = new.sham_signs(4701, *[values[::-1] for values in keys])
        self.assertTrue(torch.equal(batch.flip(0), reversed_batch))
        for row in (2, 0, 3, 1):
            solo = new.sham_signs(4701, *[[values[row]] for values in keys])
            self.assertEqual(batch[row].item(), solo.item())

    def test_episode_event_and_decision_each_affect_assignment_sequence(self):
        n = 128
        zeros = [0] * n
        baseline = new.sham_signs(4701, list(range(n)), zeros, zeros, zeros)
        self.assertEqual(set(baseline.tolist()), {-1., 1.})
        for index in (1, 2, 3):
            keys = [list(range(n)), zeros, zeros, zeros]
            keys[index] = [1] * n
            self.assertFalse(torch.equal(baseline, new.sham_signs(4701, *keys)))

    def test_assignment_rejects_invalid_identity(self):
        for value in (-1, .5, float("nan"), True):
            with self.assertRaises(ValueError):
                new.sham_signs(4701, [value], [0], [0], [0])
        with self.assertRaises(ValueError):
            new.sham_signs(4701, [0, 1], [0], [0, 0], [0, 0])

    def test_batched_control_preserves_environment_identity(self):
        cases = (inputs(wall(3., -.3, .3)), inputs(wall(3., -.8, .2), yaw=.3, belief=(5., 2., 1.)))
        args = tuple(torch.cat([case[i] for case in cases]) if i < 4 else cases[0][i] for i in range(6))
        schedule = freeze(args)
        signs = new.sham_signs(4701, [12, 3], [0, 2], [1, 4], [0, 19])
        batch, info = new.evaluate_actions(*args, schedule=schedule, decision_index=torch.tensor([0, 19]),
                                           sham_sign=signs, max_velocity=2.5, yaw_rate_max=3., lidar_far_m=12.)
        for row, case in enumerate(cases):
            solo_schedule = {key: value[row:row + 1] if isinstance(value, torch.Tensor) else value
                             for key, value in schedule.items()}
            solo, solo_info = evaluate(case, solo_schedule, decision=(0, 19)[row], sign=signs[row].item())
            for arm in new.SCRIPTED_ARMS:
                self.assertTrue(torch.equal(batch[arm][row], solo[arm][0]), arm)
                self.assertTrue(torch.equal(info["arms"][arm]["nominal_xy"][row], solo_info["arms"][arm]["nominal_xy"][0]))

    def test_online_boundary_contains_only_permitted_inputs(self):
        forbidden = ("target_position", "semantic", "segmentation", "occlusion", "el0", "truth", "mask")
        for function in (new.freeze_schedule, new.evaluate_actions):
            for name in inspect.signature(function).parameters:
                self.assertFalse(any(word in name.lower() for word in forbidden), name)
        source = ast.parse((ROOT / "motar_eval/h_r3_p03_arms.py").read_text())
        imports = {alias.name for node in ast.walk(source) if isinstance(node, ast.Import) for alias in node.names}
        self.assertEqual(imports, {"hashlib", "math", "torch"})
        froms = {node.module for node in ast.walk(source) if isinstance(node, ast.ImportFrom)}
        self.assertEqual(froms, {"__future__", "motar_eval"})

    def test_inputs_schedule_and_global_rng_are_unchanged(self):
        args = inputs(wall(3., -.3, .3))
        copies = [value.clone() for value in args]
        schedule = freeze(args)
        py_state, np_state, th_state = random.getstate(), np.random.get_state(), torch.random.get_rng_state().clone()
        new.sham_signs(4701, [1], [2], [3], [4])
        evaluate(args, schedule)
        self.assertEqual(random.getstate(), py_state)
        np_after = np.random.get_state()
        self.assertTrue(np.array_equal(np_state[1], np_after[1]))
        self.assertEqual(np_state[2:], np_after[2:])
        self.assertTrue(torch.equal(torch.random.get_rng_state(), th_state))
        for before, after in zip(copies, args):
            self.assertTrue(torch.equal(before, after))


if __name__ == "__main__":
    unittest.main()
