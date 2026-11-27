"""G7 action hold / delay hook (tools/actionhold_hook.py): CPU checks of the controller logic, no simulator.

Covers: the identity control (k=1, d=0 returns the policy tensor itself), hold latching every k steps and at
episode starts, delay by d steps with a zero (hover) pipeline at episode starts, per-env independence, config
validation, and the post-import patch on a stand-in module. Every negative check is paired with a positive one.
"""
import importlib.util
from pathlib import Path
import sys
import types
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]


def load_hook():
    spec = importlib.util.spec_from_file_location("actionhold_hook_test", ROOT / "tools/actionhold_hook.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


H = load_hook()


class Steps:
    """A stand-in step source: per-env episode steps that the test advances and resets."""

    def __init__(self, n):
        self.t = torch.zeros(n, dtype=torch.long)

    def __call__(self, player):
        return self.t

    def advance(self, reset=()):
        self.t += 1
        for i in reset:
            self.t[i] = 0


def actions(n, step):
    return torch.arange(n * 4, dtype=torch.float32).reshape(n, 4) + 100.0 * step


class HoldDelayTests(unittest.TestCase):
    def test_identity_returns_policy_action(self):
        c = H.Controller({"hold_k": 1, "delay_d": 0}, Steps(3))
        a = actions(3, 0)
        self.assertIs(c.act(None, a), a)

    def test_hold_latches_every_k_steps(self):
        src = Steps(2)
        c = H.Controller({"hold_k": 2, "delay_d": 0}, src)
        outs = []
        for s in range(5):
            outs.append(c.act(None, actions(2, s)))
            src.advance()
        expect = [0, 0, 2, 2, 4]                               # latched step for each call
        for s, e in enumerate(expect):
            self.assertTrue(torch.equal(outs[s], actions(2, e)), (s, e))
        self.assertFalse(torch.equal(outs[1], actions(2, 1)))   # positive control: the stream changes
        self.assertEqual(c.latched, 6)
        self.assertEqual(c.held, 4)

    def test_hold_relatches_at_episode_start(self):
        src = Steps(2)
        c = H.Controller({"hold_k": 4, "delay_d": 0}, src)
        c.act(None, actions(2, 0))
        src.advance()
        c.act(None, actions(2, 1))
        src.advance(reset=(1,))                                # env 1 starts a new episode at call 2
        out = c.act(None, actions(2, 2))
        self.assertTrue(torch.equal(out[0], actions(2, 0)[0]))  # env 0 still holds step 0
        self.assertTrue(torch.equal(out[1], actions(2, 2)[1]))  # env 1 latched its new episode's first action

    def test_delay_by_d_steps_with_hover_start(self):
        src = Steps(2)
        c = H.Controller({"hold_k": 1, "delay_d": 2}, src)
        outs = []
        for s in range(5):
            outs.append(c.act(None, actions(2, s)))
            src.advance()
        self.assertTrue(torch.equal(outs[0], torch.zeros(2, 4)))
        self.assertTrue(torch.equal(outs[1], torch.zeros(2, 4)))
        for s in range(2, 5):
            self.assertTrue(torch.equal(outs[s], actions(2, s - 2)), s)
        self.assertEqual(c.delayed_zero, 4)

    def test_delay_pipeline_cleared_at_episode_start(self):
        src = Steps(2)
        c = H.Controller({"hold_k": 1, "delay_d": 1}, src)
        c.act(None, actions(2, 0))
        src.advance()
        c.act(None, actions(2, 1))
        src.advance(reset=(0,))
        out = c.act(None, actions(2, 2))
        self.assertTrue(torch.equal(out[0], torch.zeros(4)))        # env 0: new episode, hover
        self.assertTrue(torch.equal(out[1], actions(2, 1)[1]))       # env 1: one step old

    def test_config_validation(self):
        self.assertEqual(H.read_config({}), {"hold_k": 1, "delay_d": 0})
        self.assertEqual(H.read_config({"MOTAR_ACTION_HOLD_K": "4"}), {"hold_k": 4, "delay_d": 0})
        for bad in ({"MOTAR_ACTION_HOLD_K": "0"}, {"MOTAR_ACTION_DELAY_D": "-1"},
                    {"MOTAR_ACTION_HOLD_K": "2", "MOTAR_ACTION_DELAY_D": "1"}):
            with self.assertRaises(ValueError):
                H.read_config(bad)

    def test_patch_wraps_get_action_and_keeps_original(self):
        class Player:
            def get_action(self, obs, is_deterministic=False):
                return obs * 2

            def run(self):
                return "ran"
        mod = types.SimpleNamespace(NavRLPpoPlayerContinuous=Player)
        seen = []

        class Ctl:
            def act(self, player, action):
                seen.append(action)
                return action + 1
        H.patch_player_module(mod, lambda: Ctl())
        p = Player()
        self.assertTrue(torch.equal(p.get_action(torch.ones(1)), torch.full((1,), 3.0)))
        self.assertEqual(len(seen), 1)
        self.assertTrue(hasattr(Player.get_action, "__actionhold_original__"))

    def test_sitecustomize_is_inert_without_the_variables(self):
        src = (ROOT / "tools/actionhold_site/sitecustomize.py").read_text()
        self.assertIn('os.environ.get("MOTAR_ACTION_HOLD_K", "").strip()', src)
        self.assertIn("os._exit(3)", src)


if __name__ == "__main__":
    unittest.main()
