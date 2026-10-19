"""U6 synthetic CPU tests: mode precedence, safety, hysteresis and independent cost oracle."""
import ast
from dataclasses import FrozenInstanceError, fields, replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import arbitration as arb, cost, fallback, terminal  # noqa: E402
from motar_eval.boia.types import BoiaParams, Rollout  # noqa: E402

for module in (arb, cost, fallback, terminal):
    assert str(Path(module.__file__).resolve()).startswith(str(ROOT) + "/")


def params():
    return BoiaParams(tau_on=.1, r_bar_est=.2)


def online(n=1):
    mu = torch.zeros(n, 6, dtype=torch.float64)
    mu[:, 0] = 4
    return SimpleNamespace(pos=torch.zeros(n, 3, dtype=torch.float64), mu=mu,
                           Sigma=.1 * torch.eye(6, dtype=torch.float64)[None].expand(n, 6, 6).clone(),
                           yaw=torch.zeros(n, dtype=torch.float64), active=torch.ones(n, dtype=torch.bool),
                           age=torch.zeros(n, dtype=torch.float64), camera_visible=torch.ones(n, dtype=torch.bool))


def fixture(n=1):
    command = torch.zeros(n, 3, 4, dtype=torch.float64)
    command[:, 0, 0], command[:, 2, 1] = .4, .4
    state = arb.initial_state(n, command)
    mode = torch.full((n,), arb.Mode.PRE_OCCLUSION, dtype=torch.int64)
    return command, state, mode


def decide(values, state, mode=None, hard=None, stop=None):
    command, _, default_mode = fixture(state.mode.shape[0])
    J = torch.tensor(values, dtype=command.dtype).reshape(state.mode.shape[0], 3)
    good = torch.ones_like(J, dtype=torch.bool)
    return arb.decide(J, good if hard is None else hard, good if stop is None else stop,
                      command, command[:, 0], default_mode if mode is None else mode, state, params())


class ModeTest(unittest.TestCase):
    def test_intercept_precedence_strict_forecast_threshold_and_loss_boundaries(self):
        inp = online(6)
        inp.camera_visible[:] = torch.tensor([1, 1, 1, 0, 0, 0], dtype=torch.bool)
        inp.mu[:, 0] = torch.tensor([2.99, 3., 4., 4., 4., 4.])
        inp.Sigma[4, :3, :3] = .4 * torch.eye(3)
        _, state, _ = fixture(6)
        state = replace(state, ever_detected=torch.tensor([0, 0, 0, 1, 1, 0], dtype=torch.bool),
                        miss_streak=torch.tensor([2, 2, 2, 19, 0, 0]))
        forecast = torch.ones(6, 2, 7, dtype=torch.float64)
        forecast[0] = 0
        forecast[1, :, 4] = .49
        forecast[2, :, :5] = .5
        forecast[2, :, 5:] = 0  # Beyond H_pre must not create PRE_OCCLUSION.
        out = arb.update_mode(state, inp, forecast, params())
        self.assertEqual(out.mode.tolist(), [arb.Mode.INTERCEPT, arb.Mode.PRE_OCCLUSION,
                         arb.Mode.TRACK_VISIBLE, arb.Mode.LOST_PREDICTABLE,
                         arb.Mode.LOST_UNLOCALIZED, arb.Mode.NEVER_ACQUIRED])
        self.assertEqual(out.miss_streak.tolist(), [0, 0, 0, 20, 1, 1])
        self.assertEqual(out.ever_detected.tolist(), [True, True, True, True, True, False])

    def test_short_forecast_and_camera_miss_clock_ignore_lidar_recency(self):
        inp = online()
        _, state, _ = fixture()
        state = replace(state, ever_detected=torch.ones(1, dtype=torch.bool), miss_streak=torch.tensor([4]))
        forecast = torch.ones(1, 2, 3, dtype=torch.float64)
        forecast[:, :, -1] = .49
        self.assertEqual(int(arb.update_mode(state, inp, forecast, params()).mode), arb.Mode.PRE_OCCLUSION)
        inp.camera_visible[:] = False
        a = arb.update_mode(state, inp, forecast, params())
        inp.age[:] = 4.99
        b = arb.update_mode(state, inp, forecast, params())
        self.assertEqual(a.miss_streak.tolist(), [5])
        self.assertTrue(torch.equal(a.miss_streak, b.miss_streak))
        self.assertTrue(torch.equal(a.mode, b.mode))
        inp.camera_visible[:] = True
        self.assertEqual(int(arb.update_mode(a, inp, forecast, params()).miss_streak), 0)

    def test_terminal_reset_ignores_postreset_camera_and_does_not_latch_next_live_step(self):
        command, state, _ = fixture(2)
        state = replace(state, engaged=torch.ones(2, dtype=torch.bool), dwell=torch.tensor([9, 7]),
                        off_count=torch.tensor([3, 2]), miss_streak=torch.tensor([8, 6]),
                        a_prev=torch.ones_like(command[:, 0]), ever_detected=torch.ones(2, dtype=torch.bool))
        before = {f.name: getattr(state, f.name).clone() for f in fields(state)}
        mask = torch.tensor([True, False])
        reset = terminal.apply(mask, state)
        self.assertEqual(reset.mode[0], arb.Mode.TERMINAL)
        self.assertFalse(bool(reset.engaged[0]) or bool(reset.ever_detected[0]))
        self.assertEqual(int(reset.dwell[0]) + int(reset.off_count[0]) + int(reset.miss_streak[0]), 0)
        self.assertTrue(bool((reset.a_prev[0] == 0).all()))
        inp = online(2)
        forecast = torch.ones(2, 2, 3, dtype=torch.float64)
        skipped = arb.update_mode(reset, inp, forecast, params(), terminal_prev=mask)
        self.assertEqual(int(skipped.mode[0]), arb.Mode.TERMINAL)
        self.assertFalse(bool(skipped.ever_detected[0]))
        inp.camera_visible[0] = False
        live = arb.update_mode(skipped, inp, forecast, params(), terminal_prev=torch.zeros(2, dtype=torch.bool))
        self.assertEqual(int(live.mode[0]), arb.Mode.NEVER_ACQUIRED)
        for key, value in before.items():
            self.assertTrue(torch.equal(value, getattr(state, key)), key)


class ArbitrationTest(unittest.TestCase):
    def test_below_on_threshold_does_not_intervene(self):
        command, state, _ = fixture()
        action, engaged, reason, new = decide([1., .96, .93], state)
        self.assertFalse(bool(engaged))
        self.assertEqual(int(reason), 0)
        self.assertTrue(torch.equal(action, command[:, 0]))
        self.assertEqual(int(new.dwell), 0)

    def test_safety_override_bypasses_dwell_and_checks_stop_feasibility(self):
        command, state, _ = fixture()
        state = replace(state, engaged=torch.ones(1, dtype=torch.bool), dwell=torch.tensor([1]))
        for mode in (arb.Mode.PRE_OCCLUSION, arb.Mode.INTERCEPT, arb.Mode.NEVER_ACQUIRED):
            for hard_fails in (True, False):
                hard = torch.ones(1, 3, dtype=torch.bool)
                stop = hard.clone()
                (hard if hard_fails else stop)[0, 0] = False
                action, engaged, reason, _ = decide([0., .8, .5], state,
                    torch.tensor([mode]), hard=hard, stop=stop)
                self.assertTrue(bool(engaged))
                self.assertEqual(int(reason), 1)
                self.assertTrue(torch.equal(action, command[:, 2]))

    def test_visible_intercept_never_and_unlocalized_allow_only_safety(self):
        command, state, _ = fixture()
        state = replace(state, engaged=torch.ones(1, dtype=torch.bool), dwell=torch.tensor([1]))
        for mode in (arb.Mode.TRACK_VISIBLE, arb.Mode.INTERCEPT, arb.Mode.NEVER_ACQUIRED, arb.Mode.LOST_UNLOCALIZED):
            action, engaged, reason, new = decide([100., 20., 0.], state, torch.tensor([mode]))
            self.assertFalse(bool(engaged))
            self.assertTrue(torch.equal(action, command[:, 0]))
            self.assertEqual(int(new.dwell), 0)
            self.assertEqual(int(reason), 5 if mode == arb.Mode.NEVER_ACQUIRED else 0)

    def test_alternating_advantage_does_not_flap(self):
        _, state, _ = fixture()
        statuses = []
        for delta in (.11, .08, .11, .08, .11, .08, .11, .08):
            _, engaged, _, state = decide([1., 1., 1. - delta], state)
            statuses.append(bool(engaged))
        self.assertEqual(statuses, [True] * 8)

    def test_minimum_five_engaged_decisions_release_on_sixth_not_fifth(self):
        _, state, _ = fixture()
        statuses, dwell = [], []
        for i in range(6):
            # After engagement the advantage remains below off; counter may accrue during hold.
            _, engaged, _, state = decide([1., 1., .8] if i == 0 else [1., 1., .98], state)
            statuses.append(bool(engaged))
            dwell.append(int(state.dwell))
        self.assertEqual(statuses, [True, True, True, True, True, False])
        self.assertEqual(dwell, [1, 2, 3, 4, 5, 0])

    def test_receding_argmin_may_return_f_during_hold_without_forcing_worse_action(self):
        command, state, _ = fixture()
        _, _, _, state = decide([1., 1., .8], state)
        for i in range(2, 6):
            action, engaged, _, state = decide([0., .5, .5], state)
            self.assertTrue(bool(engaged))
            self.assertEqual(int(state.dwell), i)
            self.assertTrue(torch.equal(action, command[:, 0]))
        _, engaged, _, state = decide([0., .5, .5], state)
        self.assertFalse(bool(engaged))

    def test_all_infeasible_has_zero_advantage_and_actual_fallback_previous_action(self):
        command, state, _ = fixture()
        values = torch.full((1, 3), float("inf"), dtype=command.dtype)
        feasible = torch.zeros_like(values, dtype=torch.bool)
        self.assertEqual(float(arb.advantage(values, feasible)), 0.)
        action, engaged, reason, new = arb.decide(values, feasible, feasible, command, command[:, 0],
                                               torch.tensor([arb.Mode.PRE_OCCLUSION]), state, params())
        self.assertTrue(bool(engaged))
        self.assertEqual(int(reason), 3)
        self.assertEqual(int(new.mode), arb.Mode.SAFE_HOLD)
        self.assertTrue(torch.equal(action, command[:, 1]))
        self.assertTrue(torch.equal(new.a_prev, action))
        self.assertFalse(bool(torch.isnan(action).any()))
        alternative = torch.tensor([[False, True, False]])
        values[0, 1] = .5
        self.assertTrue(bool(torch.isposinf(arb.advantage(values, alternative))))
        # The public helper must honor infeasibility even if its caller supplied raw finite J.
        raw = torch.tensor([[-5., 0.]], dtype=command.dtype)
        self.assertTrue(bool(torch.isposinf(arb.advantage(raw, torch.tensor([[False, True]])))))

    def test_terminal_step_never_intervenes_even_when_all_candidates_fail(self):
        command, state, _ = fixture()
        state = replace(state, engaged=torch.ones(1, dtype=torch.bool), dwell=torch.tensor([10]),
                        ever_detected=torch.ones(1, dtype=torch.bool), a_prev=torch.ones(1, 4, dtype=command.dtype))
        bad = torch.zeros(1, 3, dtype=torch.bool)
        action, engaged, reason, new = decide([float("inf")] * 3, state,
                                             torch.tensor([arb.Mode.TERMINAL]), hard=bad, stop=bad)
        self.assertFalse(bool(engaged))
        self.assertEqual(int(reason), 4)
        self.assertTrue(torch.equal(action, command[:, 0]))
        self.assertTrue(bool((new.a_prev == 0).all()))
        self.assertFalse(bool(new.ever_detected))

    def test_explicit_threshold_frozen_state_purity_and_missing_threshold(self):
        _, state, mode = fixture()
        with self.assertRaises(FrozenInstanceError):
            state.dwell = torch.tensor([5])
        with self.assertRaises(ValueError):
            BoiaParams()
        command, _, _ = fixture()
        with self.assertRaises(ValueError):
            arb.decide(torch.ones(1, 3), torch.ones(1, 3, dtype=torch.bool), torch.ones(1, 3, dtype=torch.bool),
                       command, command[:, 0], mode, state, SimpleNamespace(tau_on=None))
        before = {f.name: getattr(state, f.name).clone() for f in fields(state)}
        rng = torch.get_rng_state().clone()
        decide([1., .8, .5], state)
        self.assertTrue(torch.equal(rng, torch.get_rng_state()))
        for key, value in before.items():
            self.assertTrue(torch.equal(value, getattr(state, key)), key)

    def test_fallback_brakes_and_uses_only_active_belief_yaw(self):
        inp = online(2)
        inp.mu[:, :2] = torch.tensor([0., 4.])
        inp.active[1] = False
        inp.mu[1] = float("nan")
        action = fallback.safe_hold(inp)
        self.assertTrue(bool((action[:, :3] == 0).all()))
        self.assertEqual(action[:, 3].tolist(), [1., 0.])


def cost_fixture():
    inp = online()
    inp.Sigma[0, :3, :3] = torch.diag(torch.tensor([1., 4., 9.], dtype=torch.float64))
    pos = torch.zeros(1, 2, 2, 1, 3, dtype=torch.float64)
    pos[0, 0, -1, 0, 0], pos[0, 1, -1, 0, 0] = 1., 2.
    vel, yaw = torch.zeros_like(pos), torch.zeros(pos.shape[:-1], dtype=pos.dtype)
    roll = Rollout(pos, vel, yaw, pos[:, :, :, -1], vel[:, :, :, -1], yaw[:, :, :, -1],
                   torch.zeros(1, 2, 2, 4, dtype=pos.dtype))
    samples = torch.zeros(1, 2, 2, 6, dtype=pos.dtype)
    samples[0, :, -1, 0] = torch.tensor([4., 6.])
    pdet = torch.tensor([[[[.2, .3], [.4, .5]], [[.8, .9], [.1, .2]]]], dtype=pos.dtype)
    sigma = inp.Sigma[:, None, None].expand(1, 2, 2, 6, 6).clone()
    sigma[0, 0, -1, :3, :3] *= .5
    sigma[0, 1, -1, :3, :3] *= 2.
    cmd = torch.tensor([[[.4, 0., 0., 0.], [0., .2, 0., .3]]], dtype=pos.dtype)
    previous = torch.tensor([[.1, .2, 0., -.1]], dtype=pos.dtype)
    mode = torch.tensor([arb.Mode.TRACK_VISIBLE])
    good = torch.ones(1, 2, dtype=torch.bool)
    clearance = torch.tensor([[[.5, 1.5], [.2, .4]]], dtype=pos.dtype)
    return [inp, roll, samples, pdet, sigma, cmd, previous, mode, good, good.clone(), clearance, params()]


class CostTest(unittest.TestCase):
    def test_terms_and_mode_weights_match_independent_scalar_oracle(self):
        args = cost_fixture()
        # Independent arithmetic for two final targets at4,6 and candidate positions1,2.
        expected = [[0., .43, -1.5 * math.log(2), .035, .25],
                    [-2.5, .37, 1.5 * math.log(2), .0425, .7]]
        for mode, weights in ((arb.Mode.TRACK_VISIBLE, (1., .2, .1, .05, .5)),
                              (arb.Mode.LOST_PREDICTABLE, (1., 1., .3, .05, 1.))):
            args[7] = torch.tensor([mode])
            score, terms = cost.evaluate(*args)
            for k in range(2):
                for j, key in enumerate(("Phi", "P_loss", "U", "S", "C")):
                    self.assertAlmostEqual(float(terms[key][0, k]), expected[k][j], places=13)
                self.assertAlmostEqual(float(score[0, k]), sum(a * b for a, b in zip(expected[k], weights)), places=13)

    def test_infeasible_is_inf_and_empty_obstacle_clearance_cost_is_zero(self):
        args = cost_fixture()
        args[8][0, 0] = False
        args[10] = torch.full_like(args[10], float("inf"))
        score, terms = cost.evaluate(*args)
        self.assertTrue(bool(torch.isposinf(score[0, 0])))
        self.assertTrue(bool(torch.isfinite(score[0, 1])))
        self.assertTrue(bool((terms["C"] == 0).all()))
        self.assertFalse(bool(torch.isnan(score).any()))

    def test_singular_and_non_pd_covariances_fail_explicitly_without_hidden_floor(self):
        for final, diagonal in ((False, [0., 1., 1.]), (True, [0., 1., 1.]),
                                (False, [-1., -1., 1.])):
            args = cost_fixture()
            diag = torch.diag(torch.tensor(diagonal, dtype=torch.float64))
            if final:
                args[4][0, 0, -1, :3, :3] = diag
            else:
                args[0].Sigma[0, :3, :3] = diag
            with self.assertRaisesRegex(ValueError, "positive definite"):
                cost.evaluate(*args)

    def test_source_imports_no_mutable_runtime_globals_or_implicit_rng(self):
        for module in (arb, cost, fallback, terminal):
            source = Path(module.__file__).read_text()
            tree = ast.parse(source)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertTrue(all(x.name == "torch" for x in node.names))
                if isinstance(node, ast.ImportFrom):
                    self.assertTrue((node.level == 0 and node.module == "dataclasses") or
                                    (node.level == 1 and node.module in {"arbitration", "visibility"}))
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, {"rand", "randn", "manual_seed", "cdist"})
            self.assertFalse(any(isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)) for node in tree.body))


if __name__ == "__main__":
    unittest.main()
