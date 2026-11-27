"""G8 state recorder hook (tools/staterec_hook.py): CPU checks with stand-in player and task objects, no simulator.

Checks that the patched get_action returns the policy action unchanged, that every recorded array has one row per
call with the env axis intact and the recorded values are copies (later in-place changes do not leak in), that the
npz is written exactly once and refuses to overwrite, that install validates its path, and that an existing
file is refused at the first record, not at install (G8 attempt 1). Negative checks are
paired with positive ones.
"""
import importlib.util
from pathlib import Path
import tempfile
import types
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]


def load_hook():
    spec = importlib.util.spec_from_file_location("staterec_hook_test", ROOT / "tools/staterec_hook.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


H = load_hook()


def fake_task(n):
    t = types.SimpleNamespace()
    t.obs_dict = {"robot_position": torch.zeros(n, 3), "robot_linvel": torch.zeros(n, 3),
                  "robot_vehicle_orientation": torch.tensor([[0.0, 0.0, 0.0, 1.0]] * n),
                  "robot_orientation": torch.tensor([[0.0, 0.0, 0.0, 1.0]] * n), "robot_body_angvel": torch.zeros(n, 3),
                  "env_bounds_min": torch.zeros(n, 3), "env_bounds_max": torch.full((n, 3), 40.0)}
    t.target_orientation = torch.tensor([[0.0, 0.0, 0.0, 1.0]] * n)
    t.prev_action = torch.zeros(n, 4)
    t.target_position = torch.ones(n, 3)
    t.sim_env = types.SimpleNamespace(sim_steps=torch.zeros(n, dtype=torch.long))
    t.perception = types.SimpleNamespace(step_count=0)
    return t


class StateRecTests(unittest.TestCase):
    def test_records_copies_and_returns_action_unchanged(self):
        task = fake_task(3)
        player = types.SimpleNamespace(env=task)
        with tempfile.TemporaryDirectory() as d:
            rec = H.Recorder(Path(d) / "r.npz")
            for step in range(4):
                task.obs_dict["robot_position"][:, 0] = step
                task.prev_action[:, 1] = -step
                task.sim_env.sim_steps[:] = step
                task.perception.step_count = step + 1
                a = torch.full((3, 4), float(step))
                rec.record(player, a)
                a.add_(100.0)                                    # an in-place change after recording must not leak
            rec.save()
            z = np.load(Path(d) / "r.npz")
            self.assertEqual(z["robot_position"].shape, (4, 3, 3))
            self.assertEqual(list(z["robot_position"][:, 0, 0]), [0, 1, 2, 3])
            self.assertEqual(list(z["prev_action"][:, 2, 1]), [0, -1, -2, -3])
            self.assertEqual(list(z["perception_step_count"]), [1, 2, 3, 4])
            self.assertEqual(list(z["action"][:, 0, 0]), [0, 1, 2, 3])
            self.assertEqual(list(z["sim_steps"][:, 1]), [0, 1, 2, 3])
            self.assertEqual(z["env_bounds_max"].shape, (4, 3, 3))
            self.assertEqual(z["target_orientation"].shape, (4, 3, 4))
            with self.assertRaises(FileExistsError):
                rec.save()                                       # never overwrites

    def test_patch_returns_original_action(self):
        class Player:
            def __init__(self):
                self.env = fake_task(2)

            def get_action(self, obs, is_deterministic=False):
                return obs * 3

            def run(self):
                return "ran"
        mod = types.SimpleNamespace(NavRLPpoPlayerContinuous=Player)
        seen = []

        class Rec:
            def record(self, player, action):
                seen.append(action.clone())
        H.patch_player_module(mod, lambda: Rec())
        out = Player().get_action(torch.ones(2, 4))
        self.assertTrue(torch.equal(out, torch.full((2, 4), 3.0)))
        self.assertEqual(len(seen), 1)

    def test_install_validates_path(self):
        import sys
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(ValueError):
                H.install({"MOTAR_STATE_RECORD": str(Path(d) / "x.txt")})
            finder = H.install({"MOTAR_STATE_RECORD": str(Path(d) / "new.npz")})
            try:
                self.assertIn(finder, sys.meta_path)
                self.assertIsInstance(H.recorder(), H.Recorder)  # positive: a new path gives a recorder
                self.assertFalse((Path(d) / "new.npz").exists())  # nothing written before the save
            finally:
                sys.meta_path.remove(finder)                     # leave the shared process as it was
                H.STATE.update(recorder=None, path=None)

    def test_existing_file_refused_at_first_record_not_at_install(self):
        """G8 attempt 1: a later process of the same launcher (the receipt writer) inherits MOTAR_STATE_RECORD after
        the recording process has written the file. Its install must succeed and touch nothing; only a process that
        would record refuses the existing file."""
        import sys
        with tempfile.TemporaryDirectory() as d:
            existing = Path(d) / "x.npz"
            existing.write_bytes(b"kept")
            finder = H.install({"MOTAR_STATE_RECORD": str(existing)})
            try:
                self.assertIn(finder, sys.meta_path)
                with self.assertRaises(FileExistsError):
                    H.recorder()
                self.assertIsNone(H.STATE["recorder"])
                H._save()                                        # the atexit path with no recorder writes nothing
                self.assertEqual(existing.read_bytes(), b"kept")
            finally:
                sys.meta_path.remove(finder)
                H.STATE.update(recorder=None, path=None)


if __name__ == "__main__":
    unittest.main()
