"""CPU checks of P02 nominal controls and their frozen-P01 compatibility."""
import ast
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
from motar_eval import h_r3_arms as old
from motar_eval import h_r3_p02_arms as new

ANGLES = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72)
ELEV = torch.deg2rad(torch.tensor([20., 10., 0., -10.]))


def wall(x, low, high):
    scan = torch.full((4, 72), 12.0)
    for j, angle in enumerate(ANGLES.tolist()):
        if math.cos(angle) > 0 and low <= x * math.tan(angle) <= high:
            scan[:, j] = x / math.cos(angle) / torch.cos(ELEV)
    return scan.reshape(1, -1)


def inputs(scan=None, yaw=0.0, belief=(6., 0., 1.)):
    return (torch.tensor([[0., 0., 1.]]),
            torch.tensor([[0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]]),
            torch.tensor([belief]), torch.full((1, 288), 12.0) if scan is None else scan,
            ANGLES.clone(), ELEV.clone())


def evaluate(args, sign=1.0, max_velocity=2.5):
    return new.evaluate_actions(*args, sham_sign=torch.tensor([sign]), max_velocity=max_velocity,
                                yaw_rate_max=3.0, lidar_far_m=12.0)


class ControlsTest(unittest.TestCase):
    def test_frozen_actions_match_across_geometry_and_cap_cases(self):
        cases = (inputs(), inputs(wall(3., -.3, .3)), inputs(wall(.6, -1., 1.)),
                 inputs(wall(3., -.8, .2), yaw=.4, belief=(6., 2., 1.)),
                 inputs(wall(3., -.2, .8), yaw=-.5, belief=(4., -2., 1.)))
        for args in cases:
            for cap in (2.5, .7):
                actions, info = evaluate(args, max_velocity=cap)
                for arm in old.ARMS:
                    frozen, _ = old.scripted_action(arm, *args, max_velocity=cap, yaw_rate_max=3., lidar_far_m=12.)
                    self.assertTrue(torch.equal(actions[arm], frozen), (arm, cap))
                self.assertTrue(torch.equal(actions["B3"], info["shadow_B3_reference_action"]))

    def test_nominal_budgets_and_sham_side(self):
        args = inputs(wall(3., -.3, .3))
        for sign in (-1., 1.):
            actions, info = evaluate(args, sign)
            stages = info["arms"]
            self.assertGreater(abs(float(info["lateral_mps"][0])), 0.)
            for arm in ("B4", "B5"):
                torch.testing.assert_close(stages[arm]["nominal_norm_mps"], stages["B3"]["nominal_norm_mps"], rtol=0, atol=1e-6)
                self.assertTrue(torch.equal(actions[arm][:, 3], actions["B3"][:, 3]))
            self.assertAlmostEqual(float(stages["B4"]["nominal_xy"][0, 1]), sign * abs(float(info["lateral_mps"][0])), places=6)
            self.assertEqual(float(stages["B5"]["nominal_xy"][0, 1]), 0.)
            self.assertTrue(torch.equal(info["shadow_B2"]["post_cap_xy"], stages["B2"]["post_cap_xy"]))
            self.assertTrue(torch.equal(info["shadow_B3"]["nominal_xy"], stages["B3"]["nominal_xy"]))

    def test_zero_offset_and_no_candidate_cases(self):
        for args in (inputs(), inputs(wall(.6, -10., 10.)), inputs(belief=(0., 0., 1.))):
            actions, info = evaluate(args)
            self.assertEqual(float(info["chosen_offset_m"][0]), 0.)
            for arm in ("B3", "B4", "B5"):
                self.assertTrue(torch.equal(actions["B2"], actions[arm]))

    def test_attenuation_stages_never_restore_speed(self):
        args = inputs(wall(.6, -1., 1.))
        actions, info = evaluate(args, max_velocity=.7)
        for arm, stage in info["arms"].items():
            self.assertTrue(bool((stage["post_point_norm_mps"] <= stage["nominal_norm_mps"] + 1e-6).all()))
            self.assertTrue(bool((stage["post_cap_norm_mps"] <= stage["post_point_norm_mps"] + 1e-6).all()))
            self.assertTrue(bool((stage["post_cap_norm_mps"] <= .7 + 1e-6).all()))
            torch.testing.assert_close(actions[arm][:, :2] * .7, stage["post_cap_xy"], rtol=0, atol=1e-6)
            self.assertTrue(bool(torch.isfinite(actions[arm]).all()))
            self.assertLessEqual(float(actions[arm].abs().max()), 1.)
        self.assertLess(float(info["shadow_B2"]["post_point_norm_mps"][0]), .2)

    def test_directional_safety_can_break_nominal_matching(self):
        _, info = evaluate(inputs(wall(.8, -.2, .2)))
        b3, b5 = info["arms"]["B3"], info["arms"]["B5"]
        torch.testing.assert_close(b3["nominal_norm_mps"], b5["nominal_norm_mps"], rtol=0, atol=1e-6)
        self.assertGreater(abs(float(b3["post_cap_norm_mps"][0] - b5["post_cap_norm_mps"][0])), .1)

    def test_batched_actions_and_shadows_preserve_row_identity(self):
        cases = (inputs(wall(3., -.3, .3)), inputs(wall(.8, -.2, .2), yaw=.3, belief=(5., 2., 1.)))
        args = tuple(torch.cat([case[i] for case in cases]) if i < 4 else cases[0][i] for i in range(6))
        signs = new.sham_signs(4601, [12, 3])
        batch, info = new.evaluate_actions(*args, sham_sign=signs, max_velocity=2.5, yaw_rate_max=3., lidar_far_m=12.)
        for row, case in enumerate(cases):
            solo, solo_info = evaluate(case, float(signs[row]))
            for arm in new.ARMS:
                self.assertTrue(torch.equal(batch[arm][row], solo[arm][0]))
                for key in ("nominal_xy", "post_point_xy", "post_cap_xy"):
                    self.assertTrue(torch.equal(info["arms"][arm][key][row], solo_info["arms"][arm][key][0]))

    def test_sham_assignment_is_fixed_and_batch_order_independent(self):
        ids = [7, 2, 100, 0]
        a = new.sham_signs(4601, ids)
        self.assertTrue(torch.equal(a, new.sham_signs(4601, ids)))
        self.assertTrue(torch.equal(a.flip(0), new.sham_signs(4601, ids[::-1])))
        self.assertTrue(torch.equal(a[:2], new.sham_signs(4601, ids[:2])))
        self.assertEqual(set(new.sham_signs(4601, range(128)).tolist()), {-1., 1.})
        with self.assertRaises(ValueError):
            new.sham_signs(4601, [-1])

    def test_inputs_and_global_rng_states_are_unchanged(self):
        args = inputs(wall(3., -.3, .3))
        copies = [a.clone() for a in args]
        sign = torch.tensor([-1.])
        py_state, np_state, th_state = random.getstate(), np.random.get_state(), torch.random.get_rng_state().clone()
        new.sham_signs(4601, [0, 1, 2])
        new.evaluate_actions(*args, sham_sign=sign, max_velocity=2.5, yaw_rate_max=3., lidar_far_m=12.)
        for before, after in zip(copies, args):
            self.assertTrue(torch.equal(before, after))
        self.assertEqual(sign.tolist(), [-1.])
        self.assertEqual(random.getstate(), py_state)
        after_np = np.random.get_state()
        self.assertEqual(after_np[0], np_state[0])
        self.assertTrue(np.array_equal(after_np[1], np_state[1]))
        self.assertEqual(after_np[2:], np_state[2:])
        self.assertTrue(torch.equal(torch.random.get_rng_state(), th_state))

    def test_invalid_scalars_signs_shapes_and_nonfinite_inputs_fail(self):
        args = inputs()
        kwargs = dict(sham_sign=torch.tensor([1.]), max_velocity=2.5, yaw_rate_max=3., lidar_far_m=12.)
        for name in ("max_velocity", "yaw_rate_max", "lidar_far_m"):
            for value in (0., -1., float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    new.evaluate_actions(*args, **dict(kwargs, **{name: value}))
        for sign in (torch.tensor([0.]), torch.tensor([float("nan")]), torch.tensor([1., -1.])):
            with self.assertRaises(ValueError):
                new.evaluate_actions(*args, **dict(kwargs, sham_sign=sign))
        for index in range(len(args)):
            invalid = [x.clone() for x in args]
            invalid[index].reshape(-1)[0] = float("nan")
            with self.assertRaises(ValueError):
                new.evaluate_actions(*invalid, **kwargs)
        invalid = list(args)
        invalid[3] = torch.ones(1, 72)
        with self.assertRaises(ValueError):
            new.evaluate_actions(*invalid, **kwargs)

    def test_online_input_and_import_firewall(self):
        forbidden = ("target_position", "semantic", "segmentation", "occlusion", "el0", "truth", "mask")
        for name in inspect.signature(new.evaluate_actions).parameters:
            self.assertFalse(any(word in name.lower() for word in forbidden), name)
        tree = ast.parse((ROOT / "motar_eval/h_r3_p02_arms.py").read_text())
        imports = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import) for alias in node.names}
        self.assertEqual(imports, {"hashlib", "math", "torch"})
        froms = {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        self.assertEqual(froms, {"__future__", "motar_eval"})


if __name__ == "__main__":
    unittest.main()
