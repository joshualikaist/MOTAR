"""G8 zero-action hook (tools/zeroaction_hook.py): CPU checks with a stand-in player, no simulator."""
import importlib.util
from pathlib import Path
import types
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("zeroaction_hook_test", ROOT / "tools/zeroaction_hook.py")
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)


class ZeroActionTests(unittest.TestCase):
    def test_patch_returns_zeros_of_the_same_shape_and_calls_the_original(self):
        calls = []

        class Player:
            def get_action(self, obs, is_deterministic=False):
                calls.append(1)
                return obs + 1.0
        H.patch_player_module(types.SimpleNamespace(NavRLPpoPlayerContinuous=Player))
        out = Player().get_action(torch.ones(3, 4))
        self.assertTrue(torch.equal(out, torch.zeros(3, 4)))
        self.assertEqual(len(calls), 1)                     # the policy still ran (same RNG-free path)

    def test_install_requires_the_flag(self):
        with self.assertRaises(ValueError):
            H.install({})
        import sys
        finder = H.install({"MOTAR_ACTION_ZERO": "1"})
        self.assertIn(finder, sys.meta_path)
        sys.meta_path.remove(finder)


if __name__ == "__main__":
    unittest.main()
