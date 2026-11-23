"""CPU tests of aerial_gym/task/navrl_task/navrl_cams2.py (CAM-S2a action delay, capture/contact rule, altitude and
camera mount) and of its opt-in install through tools/realveh_entry.py. Same fake-task style as test_camsafe.py;
every refusal has a positive control."""
import importlib.util
import pathlib
import unittest

import torch

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


C2 = _load("cams2_under_test", ROOT / "aerial_gym/task/navrl_task/navrl_cams2.py")
E = _load("cams2_test_realveh_entry", ROOT / "tools/realveh_entry.py")

ON = {"NAVRL_CAMS2": "1", "NAVRL_CAMS2_DELAY_MIN": "1", "NAVRL_CAMS2_DELAY_MAX": "2", "NAVRL_CAMS2_DELAY_SEED": "4961",
      "NAVRL_CAMS2_CAPTURE_RADIUS_M": "1.0", "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS": "1.0",
      "NAVRL_CAMS2_CONTACT_RADIUS_M": "0.40", "NAVRL_CAMS2_FLIGHT_ALTITUDE_M": "1.5", "NAVRL_CAMS2_CAMERA_X_M": "0.08",
      "NAVRL_ACTION_DELAY_STEPS_MAX": "2"}
CAMSAFE_ON = {"NAVRL_CAMSAFE": "1", "NAVRL_CAMSAFE_BLIND_MPS": "1.0", "NAVRL_CAMSAFE_BLIND_WEIGHT": "0.5",
              "NAVRL_CAMSAFE_COLLISION_PENALTY": "-40", "NAVRL_CAMSAFE_RSS_CLIP_M": "4.0"}


class Settings(unittest.TestCase):
    def test_off_and_on(self):
        self.assertIsNone(C2.settings_from_env({}))
        s = C2.settings_from_env(ON)
        self.assertEqual((s["NAVRL_CAMS2_DELAY_MIN"], s["NAVRL_CAMS2_DELAY_MAX"]), (1, 2))
        self.assertEqual(s["NAVRL_CAMS2_CAPTURE_RADIUS_M"], 1.0)

    def test_refusals(self):
        bad = [dict(ON, NAVRL_CAMS2="yes"), {"NAVRL_CAMS2_DELAY_MAX": "2"},
               {k: v for k, v in ON.items() if k != "NAVRL_CAMS2_DELAY_SEED"},
               dict(ON, NAVRL_CAMS2_DELAY_MIN="3"), dict(ON, NAVRL_CAMS2_DELAY_MAX="4", NAVRL_ACTION_DELAY_STEPS_MAX="4"),
               dict(ON, NAVRL_CAMS2_DELAY_MIN="1.5"), dict(ON, NAVRL_CAMS2_CONTACT_RADIUS_M="1.0"),
               dict(ON, NAVRL_ACTION_DELAY_STEPS_MAX="1"), {k: v for k, v in ON.items()
                                                            if k != "NAVRL_ACTION_DELAY_STEPS_MAX"},
               dict(ON, NAVRL_CAMS2_FLIGHT_ALTITUDE_M="nan")]
        for env in bad:
            with self.subTest(env=env), self.assertRaises(ValueError):
                C2.settings_from_env(env)
        self.assertIsNotNone(C2.settings_from_env(ON))


class Pure(unittest.TestCase):
    def test_delayed_selects_per_env(self):
        hist = torch.tensor([[[1.], [2.], [3.]], [[4.], [5.], [6.]]])
        self.assertEqual(C2.delayed(hist, torch.tensor([1, 2])).tolist(), [[2.], [6.]])

    def test_capture_mask_requires_all_three_and_alive(self):
        rel = torch.tensor([[0.9, 0., 0.], [0.9, 0., 0.], [0.9, 0., 0.], [1.2, 0., 0.], [0.9, 0., 0.]])
        vel = torch.tensor([[-0.5, 0., 0.], [-1.5, 0., 0.], [-0.5, 0., 0.], [-0.5, 0., 0.], [-0.5, 0., 0.]])
        visible = torch.tensor([True, True, False, True, True])
        alive = torch.tensor([True, True, True, True, False])
        self.assertEqual(C2.capture_mask(rel, vel, visible, alive, 1.0, 1.0).tolist(), [True, False, False, False, False])


def _fake_task_cls():
    class Rw(dict):
        pass

    class Task:
        _camsafe_settings = {"x": 1}

        def __init__(self, n):
            self.num_envs = n
            self.rw = {"collision_penalty": -40.0, "capture_bonus": 30.0}
            self.rewards = torch.zeros(n)
            self.terminations = torch.zeros(n)
            self.truncations = torch.zeros(n)
            self.captured_now = torch.zeros(n, dtype=torch.bool)
            self.crashed_now = torch.zeros(n, dtype=torch.bool)
            self._visible_now = torch.ones(n, dtype=torch.bool)
            self.target_position = torch.zeros(n, 3)
            self.target_vel_w = torch.zeros(n, 3)
            self.obs_dict = {"robot_position": torch.zeros(n, 3), "robot_linvel": torch.zeros(n, 3)}
            self.applied = []
            self.orig_captured = torch.zeros(n, dtype=torch.bool)

        def transform_action_to_command(self, actions):
            self.applied.append(actions.clone())
            return actions

        def reset_idx(self, env_ids):
            return None

        def compute_state_reward_and_terminations(self):
            self.rewards[:] = 1.0
            self.captured_now = self.orig_captured.clone()
            self.rewards[self.captured_now] = 31.0
            self.terminations[:] = self.captured_now.float()

    return Task


class Install(unittest.TestCase):
    def tearDown(self):
        C2.uninstall()

    def _install(self):
        Task = _fake_task_cls()

        class Cfg:
            success_radius = 0.5
            flight_altitude = 1.0

            class vision:
                camera_translation = [0.10, 0.0, 0.03]
        info = C2.install_task(Task, C2.settings_from_env(ON), Cfg)
        return Task, Cfg, info

    def test_refuses_without_camsafe_and_twice(self):
        Task = _fake_task_cls()
        Task._camsafe_settings = None

        class Cfg:
            success_radius = 0.5
            flight_altitude = 1.0

            class vision:
                camera_translation = [0.10, 0.0, 0.03]
        with self.assertRaises(RuntimeError):
            C2.install_task(Task, C2.settings_from_env(ON), Cfg)
        Task._camsafe_settings = {"x": 1}
        C2.install_task(Task, C2.settings_from_env(ON), Cfg)          # positive control
        with self.assertRaises(RuntimeError):
            C2.install_task(Task, C2.settings_from_env(ON), Cfg)

    def test_config_overrides(self):
        _, Cfg, info = self._install()
        self.assertEqual((Cfg.success_radius, Cfg.flight_altitude), (0.40, 1.5))
        self.assertEqual(Cfg.vision.camera_translation, [0.08, 0.0, 0.03])
        self.assertEqual(info["delay_steps"], [1, 2])

    def test_actions_are_delayed_and_reset_clears_history(self):
        Task, _, _ = self._install()
        task = Task(2)
        task._cams2_delay = None
        outs = [task.transform_action_to_command(torch.full((2, 4), float(i + 1))) for i in range(4)]
        delay = task._cams2_delay.tolist()
        self.assertTrue(all(d in (1, 2) for d in delay))
        for env, d in enumerate(delay):
            seq = [float(o[env, 0]) for o in outs]
            expected = [0.0] * d + [float(i + 1) for i in range(4 - d)]
            self.assertEqual(seq, expected, (env, d))
        task.reset_idx([0])
        out = task.transform_action_to_command(torch.full((2, 4), 9.0))
        if int(task._cams2_delay[0]) > 0:
            self.assertEqual(float(out[0, 0]), 0.0)                   # history of env 0 cleared

    def test_contact_becomes_collision_and_new_capture_rule(self):
        Task, _, _ = self._install()
        task = Task(4)
        task.orig_captured = torch.tensor([True, False, False, False])          # env 0: inside the contact radius
        task.obs_dict["robot_position"] = torch.tensor([[0.3, 0., 1.5], [0.9, 0., 1.5], [0.9, 0., 1.5], [2.0, 0., 1.5]])
        task.obs_dict["robot_linvel"] = torch.tensor([[0., 0., 0.], [-0.5, 0., 0.], [-2.0, 0., 0.], [0., 0., 0.]])
        task.target_position[:, 2] = 1.5
        task.compute_state_reward_and_terminations()
        self.assertEqual(task.captured_now.tolist(), [False, True, False, False])
        self.assertEqual(task.crashed_now.tolist(), [True, False, False, False])
        self.assertEqual(task.rewards.tolist(), [-40.0, 31.0, 1.0, 1.0])
        self.assertEqual(task.terminations.tolist(), [1.0, 1.0, 0.0, 0.0])
        self.assertEqual(Task._cams2_stats["contacts"], 1)
        self.assertEqual(Task._cams2_stats["captures"], 1)

    def test_not_detected_means_no_capture(self):
        Task, _, _ = self._install()
        task = Task(1)
        task.obs_dict["robot_position"] = torch.tensor([[0.8, 0., 1.5]])
        task.target_position[:, 2] = 1.5
        task._visible_now = torch.tensor([False])
        task.compute_state_reward_and_terminations()
        self.assertFalse(bool(task.captured_now[0]))
        task._visible_now = torch.tensor([True])                                 # positive control
        task.compute_state_reward_and_terminations()
        self.assertTrue(bool(task.captured_now[0]))


class RealvehEntry(unittest.TestCase):
    BASE = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/receipt.json", "NAVRL_SENSOR_ABLATION": "no_lidar",
            "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}

    def test_off_on_and_needs_camsafe(self):
        self.assertNotIn("cams2", E.settings(dict(self.BASE, **CAMSAFE_ON)))
        self.assertEqual(E.settings(dict(self.BASE, **CAMSAFE_ON, **ON))["cams2"]["NAVRL_CAMS2_DELAY_MAX"], 2)
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **ON))                                    # without camsafe
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **CAMSAFE_ON, NAVRL_CAMS2_DELAY_MAX="2"))  # stray while off

    def test_cams2_not_in_hashed(self):
        self.assertNotIn("cams2", E.HASHED)
        self.assertTrue(E.CAMS2.is_file())


if __name__ == "__main__":
    unittest.main()
