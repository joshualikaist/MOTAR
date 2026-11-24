"""CPU tests of navrl_yawclip.py (draft) and of its opt-in hook in the patched tools/realveh_entry.py copy. Same
fake-task style as tests/test_cams2.py; every refusal has a positive control and every count is asserted non-zero.
Run from tests/: python -B -m unittest test_yawclip"""
import importlib.util
import io
import pathlib
import unittest

import torch

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


Y = _load("yawclip_under_test", ROOT / "aerial_gym/task/navrl_task/navrl_yawclip.py")
E = _load("yawclip_test_realveh_entry", ROOT / "tools/realveh_entry.py")

ON = {"NAVRL_YAWCLIP": "1", "NAVRL_YAWCLIP_RADPS": "1.047"}


class Settings(unittest.TestCase):
    def test_off_and_on(self):
        self.assertIsNone(Y.settings_from_env({}))
        self.assertIsNone(Y.settings_from_env({"NAVRL_YAWCLIP": "0"}))
        self.assertEqual(Y.settings_from_env(ON), {"NAVRL_YAWCLIP_RADPS": 1.047})

    def test_refusals(self):
        bad = [{"NAVRL_YAWCLIP_RADPS": "1.047"}, {"NAVRL_YAWCLIP": "yes", "NAVRL_YAWCLIP_RADPS": "1.047"},
               {"NAVRL_YAWCLIP": "1"}, dict(ON, NAVRL_YAWCLIP_RADPS="abc"), dict(ON, NAVRL_YAWCLIP_RADPS="0.0"),
               dict(ON, NAVRL_YAWCLIP_RADPS="3.5"), dict(ON, NAVRL_YAWCLIP_RADPS="nan")]
        for env in bad:
            with self.subTest(env=env), self.assertRaises(ValueError):
                Y.settings_from_env(env)
        self.assertIsNotNone(Y.settings_from_env(ON))                                   # positive control


class Pure(unittest.TestCase):
    def test_clip_yaw_counts_and_clamps(self):
        cmd = torch.tensor([[0., 0., 0., 2.5], [0., 0., 0., -3.0], [0., 0., 0., 0.5], [0., 0., 0., -1.047]])
        n = Y.clip_yaw(cmd, 1.047)
        self.assertEqual(n, 2)
        for got, want in zip(cmd[:, 3].tolist(), [1.047, -1.047, 0.5, -1.047]):
            self.assertAlmostEqual(got, want, places=6)
        cmd2 = torch.tensor([[0., 0., 0., 0.9]])
        self.assertEqual(Y.clip_yaw(cmd2, 1.047), 0)                                    # nothing to clip
        self.assertAlmostEqual(cmd2[0, 3].item(), 0.9, places=6)


def _fake_task_cls(yaw_scale=3.0):
    class Task:
        _cams2_settings = {"x": 1}

        class task_config:
            yaw_rate_max = yaw_scale
            max_velocity = 2.5

        def __init__(self, n):
            self.command = torch.zeros(n, 4)
            self._visible_now = torch.ones(n, dtype=torch.bool)

        def transform_action_to_command(self, actions):
            # the task's mapping (navrl_task.py:4961, 5006-5008) without governor and delay
            self.command[:, 0:3] = actions[:, 0:3].clamp(-1, 1) * self.task_config.max_velocity
            self.command[:, 3] = actions[:, 3].clamp(-1, 1) * self.task_config.yaw_rate_max
            return self.command

    return Task


class Install(unittest.TestCase):
    def tearDown(self):
        Y.uninstall()

    def test_refuses_twice_and_a_non_binding_clip(self):
        Task = _fake_task_cls()
        with self.assertRaises(RuntimeError):
            Y.install_task(Task, {"NAVRL_YAWCLIP_RADPS": 3.0}, Task.task_config)      # == scale: never binds
        info = Y.install_task(Task, Y.settings_from_env(ON), Task.task_config)        # positive control
        self.assertEqual(info["wrapped"], ["transform_action_to_command"])
        self.assertEqual((info["limit_radps"], info["scale_radps"]), (1.047, 3.0))
        with self.assertRaises(RuntimeError):
            Y.install_task(Task, Y.settings_from_env(ON), Task.task_config)

    def test_clips_the_command_not_the_action_scale(self):
        Task = _fake_task_cls()
        Y.install_task(Task, Y.settings_from_env(ON), Task.task_config)
        task = Task(3)
        actions = torch.tensor([[1., 0., 0., 1.0], [0.5, 0., 0., -0.2], [0., 0., 0., -0.5]])
        cmd = task.transform_action_to_command(actions)
        self.assertAlmostEqual(cmd[0, 3].item(), 1.047, places=6)          # 3.0 -> clipped
        self.assertAlmostEqual(cmd[1, 3].item(), -0.6, places=6)           # inside: untouched
        self.assertAlmostEqual(cmd[2, 3].item(), -1.047, places=6)         # -1.5 -> clipped
        self.assertEqual(cmd[0, 0].item(), 2.5)                            # velocity channels untouched
        self.assertEqual(Task.task_config.yaw_rate_max, 3.0)               # the action scale stays 3.0
        st = Task._yawclip_stats
        self.assertEqual((st["calls"], st["commands"], st["clipped"]), (1, 3, 2))
        self.assertAlmostEqual(st["abs_before"], 3.0 + 0.6 + 1.5, places=6)
        self.assertAlmostEqual(st["abs_after"], 1.047 + 0.6 + 1.047, places=6)

    def test_negative_control_small_actions_are_never_clipped(self):
        Task = _fake_task_cls()
        Y.install_task(Task, Y.settings_from_env(ON), Task.task_config)
        task = Task(2)
        for _ in range(5):
            task.transform_action_to_command(torch.tensor([[0., 0., 0., 0.3], [0., 0., 0., -0.3]]))
        self.assertEqual(Task._yawclip_stats["clipped"], 0)
        self.assertEqual(Task._yawclip_stats["commands"], 10)
        task.transform_action_to_command(torch.tensor([[0., 0., 0., 1.0], [0., 0., 0., 0.0]]))   # positive control
        self.assertEqual(Task._yawclip_stats["clipped"], 1)

    def test_report_line_at_the_report_calls(self):
        Task = _fake_task_cls()
        Y.install_task(Task, Y.settings_from_env(ON), Task.task_config)
        task = Task(1)
        buf = io.StringIO()
        import sys
        real = sys.stderr
        # the module bound sys.stderr at install; rebind through the closure by re-installing with the buffer active
        Y.uninstall()
        sys.stderr = buf
        try:
            Y.install_task(Task, Y.settings_from_env(ON), Task.task_config)
            for _ in range(Y.REPORT_AT_CALLS[0]):
                task.transform_action_to_command(torch.tensor([[0., 0., 0., 1.0]]))
        finally:
            sys.stderr = real
        lines = [x for x in buf.getvalue().splitlines() if x.startswith(Y.LOG_PREFIX)]
        self.assertEqual(len(lines), 1)
        self.assertIn("clipped_frac=1.0000", lines[0])
        self.assertIn("limit_radps=1.047", lines[0])


class RealvehEntryHook(unittest.TestCase):
    BASE = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/receipt.json", "NAVRL_SENSOR_ABLATION": "no_lidar",
            "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}

    def test_off_on_and_refusals(self):
        self.assertNotIn("yawclip", E.settings(dict(self.BASE)))
        self.assertEqual(E.settings(dict(self.BASE, **ON))["yawclip"], {"NAVRL_YAWCLIP_RADPS": 1.047})
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, NAVRL_YAWCLIP_RADPS="1.047"))                 # stray while off
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS="9"))   # out of range

    def test_install_order_in_the_entry_source(self):
        """ablation -> camsafe -> cams2 -> yawclip -> cams2diag (the clip acts on the delayed command; the recorder sees the
        clipped command)."""
        src = (ROOT / "tools/realveh_entry.py").read_text()
        order = [src.index(m) for m in ("ablation.install(", "camsafe.install(", "contactcode.install(", "cams2.install(",
                                        "yawclip.install(", "cams2diag.install(")]
        self.assertEqual(order, sorted(order))

    def test_report_every_320_calls(self):
        self.assertEqual(Y.REPORT_EVERY_CALLS, 320)                                      # review P2-10

    def test_module_paths_and_not_in_hashed(self):
        self.assertNotIn("yawclip", E.HASHED)
        self.assertNotIn("cams2diag", E.HASHED)
        src = (ROOT / "tools/realveh_entry.py").read_text()          # the committed paths
        self.assertIn('YAWCLIP = ROOT / "aerial_gym/task/navrl_task/navrl_yawclip.py"', src)
        self.assertIn('CAMS2DIAG = ROOT / "aerial_gym/task/navrl_task/navrl_cams2diag.py"', src)
        self.assertTrue(E.YAWCLIP.is_file() and E.CAMS2DIAG.is_file())


if __name__ == "__main__":
    unittest.main()
