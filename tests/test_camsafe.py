"""CPU tests of aerial_gym/task/navrl_task/navrl_camsafe.py (CAM-S1 blind-direction speed cap and reward terms) and
of its opt-in install through tools/realveh_entry.py. Every refusal test has a positive control."""
import importlib.util
import math
import pathlib
import sys
import unittest

import torch

ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


C = _load("camsafe_under_test", ROOT / "aerial_gym/task/navrl_task/navrl_camsafe.py")
E = _load("camsafe_test_realveh_entry", ROOT / "tools/realveh_entry.py")

HALF = math.radians(33.0)
ON = {"NAVRL_CAMSAFE": "1", "NAVRL_CAMSAFE_BLIND_MPS": "1.0", "NAVRL_CAMSAFE_BLIND_WEIGHT": "0.5",
      "NAVRL_CAMSAFE_COLLISION_PENALTY": "-40", "NAVRL_CAMSAFE_RSS_CLIP_M": "4.0"}


def _cmd(angle_deg, speed):
    a = math.radians(angle_deg)
    return torch.tensor([[speed * math.cos(a), speed * math.sin(a)]], dtype=torch.float32)


class Settings(unittest.TestCase):
    def test_off_is_none_and_on_is_parsed(self):
        self.assertIsNone(C.settings_from_env({}))
        self.assertIsNone(C.settings_from_env({"NAVRL_CAMSAFE": "0"}))
        s = C.settings_from_env(ON)
        self.assertEqual(s, {"NAVRL_CAMSAFE_BLIND_MPS": 1.0, "NAVRL_CAMSAFE_BLIND_WEIGHT": 0.5,
                             "NAVRL_CAMSAFE_COLLISION_PENALTY": -40.0, "NAVRL_CAMSAFE_RSS_CLIP_M": 4.0})

    def test_refusals(self):
        bad = [dict(ON, NAVRL_CAMSAFE="yes"),
               {"NAVRL_CAMSAFE_BLIND_MPS": "1.0"},                       # parameter while off
               {k: v for k, v in ON.items() if k != "NAVRL_CAMSAFE_BLIND_WEIGHT"},
               dict(ON, NAVRL_CAMSAFE_BLIND_MPS="fast"),
               dict(ON, NAVRL_CAMSAFE_BLIND_MPS="0.1"),
               dict(ON, NAVRL_CAMSAFE_COLLISION_PENALTY="-10"),
               dict(ON, NAVRL_CAMSAFE_BLIND_WEIGHT="nan"),
               dict(ON, NAVRL_CAMSAFE_RSS_CLIP_M="20")]
        for env in bad:
            with self.subTest(env=env), self.assertRaises(ValueError):
                C.settings_from_env(env)
        self.assertIsNotNone(C.settings_from_env(ON))                    # positive control


class Cap(unittest.TestCase):
    def test_in_fov_unchanged_out_of_fov_capped_direction_kept(self):
        inside = _cmd(30.0, 3.0)
        out, mask = C.cap_blind(inside, HALF, 1.0)
        self.assertFalse(bool(mask.any()))
        self.assertTrue(torch.equal(out, inside))
        for angle in (45.0, 90.0, -120.0, 180.0):
            cmd = _cmd(angle, 3.0)
            out, mask = C.cap_blind(cmd, HALF, 1.0)
            self.assertTrue(bool(mask.all()), angle)
            self.assertAlmostEqual(float(out.norm()), 1.0, places=5)
            self.assertAlmostEqual(math.degrees(math.atan2(float(out[0, 1]), float(out[0, 0]))),
                                   math.degrees(math.atan2(float(cmd[0, 1]), float(cmd[0, 0]))), places=4)

    def test_slow_and_zero_out_of_fov_commands_unchanged(self):
        slow = _cmd(90.0, 0.5)
        out, mask = C.cap_blind(slow, HALF, 1.0)
        self.assertTrue(bool(mask.all()))
        self.assertTrue(torch.allclose(out, slow))
        zero = torch.zeros(1, 2)
        out, mask = C.cap_blind(zero, HALF, 1.0)
        self.assertFalse(bool(mask.any()))
        self.assertTrue(torch.equal(out, zero))

    def test_batch_mixed(self):
        cmd = torch.cat([_cmd(0.0, 3.0), _cmd(60.0, 3.0), _cmd(-20.0, 2.0)])
        out, mask = C.cap_blind(cmd, HALF, 1.0)
        self.assertEqual(mask.tolist(), [False, True, False])
        self.assertTrue(torch.equal(out[0], cmd[0]) and torch.equal(out[2], cmd[2]))


class Penalty(unittest.TestCase):
    def test_zero_inside_positive_outside_monotone_and_gated(self):
        inside = C.blind_penalty(_cmd(30.0, 2.0), HALF, 0.5, 1.0)
        self.assertEqual(float(inside), 0.0)
        values = [float(C.blind_penalty(_cmd(a, 2.0), HALF, 0.5, 1.0)) for a in (40.0, 90.0, 150.0, 180.0)]
        self.assertTrue(all(b > a for a, b in zip(values, values[1:])), values)
        self.assertAlmostEqual(values[-1], 0.5, places=5)                 # backwards at full gate = weight
        self.assertEqual(float(C.blind_penalty(torch.zeros(1, 2), HALF, 0.5, 1.0)), 0.0)
        half_speed = float(C.blind_penalty(_cmd(180.0, 0.5), HALF, 0.5, 1.0))
        self.assertAlmostEqual(half_speed, 0.25, places=5)


class _Perception:
    hfov = 2 * HALF


class _Cfg:
    yaw_align_speed_ref = 1.0


RANGE_M = 12.0


def original_r_ss(dist_m, seg, range_m=RANGE_M):
    """Verbatim arithmetic of NavRLTask.add_static_safety_reward (navrl_task.py), for the equivalence test."""
    d = dist_m.clamp(min=1e-6, max=range_m)
    d = torch.where(seg == 50, torch.full_like(d, range_m), d)
    return (torch.log(d) - math.log(range_m)).mean(dim=1)


def _fake_task_cls():
    class Task:
        _sensor_ablation_condition = "no_lidar"
        vision_mode = True

        def __init__(self, vel, terminated, pix=None, seg=None):
            n = vel.shape[0]
            self.num_envs = n
            self.perception = _Perception()
            self.task_config = _Cfg()
            self.rw = {"safety_static_weight": 1.5}
            pix = torch.full((n, 1, 4, 72), 1.0) if pix is None else pix
            seg = torch.zeros((n, 1, 4, 72)) if seg is None else seg
            self.obs_dict = {"robot_vehicle_linvel": torch.cat([vel, torch.zeros(n, 1)], dim=1),
                             "depth_range_pixels": pix, "segmentation_pixels": seg}
            self.rewards = torch.zeros(n)
            self.terminations = torch.tensor(terminated, dtype=torch.float32)
            self.truncations = torch.zeros(n)

        def _lidar_distance_m(self):
            pix = self.obs_dict["depth_range_pixels"].squeeze(1).clamp(0.0, 1.0)
            return (pix * RANGE_M).reshape(self.num_envs, -1)

        def add_static_safety_reward(self):
            raise AssertionError("the original must be replaced, not called")

        def _apply_sensor_speed_governor(self, command_xy):
            return command_xy * 1.0

        def compute_state_reward_and_terminations(self):
            self.rewards[:] = 1.0
            self.rewards[self.terminations > 0] = -20.0
            return "orig-return"

    return Task


class Install(unittest.TestCase):
    def tearDown(self):
        C.uninstall()

    def test_refuses_without_ablation_and_twice(self):
        Task = _fake_task_cls()
        Task._sensor_ablation_condition = None
        with self.assertRaises(RuntimeError):
            C.install_task(Task, C.settings_from_env(ON))
        Task._sensor_ablation_condition = "no_lidar"
        C.install_task(Task, C.settings_from_env(ON))                    # positive control
        with self.assertRaises(RuntimeError):
            C.install_task(Task, C.settings_from_env(ON))

    def test_wrapped_governor_and_reward(self):
        Task = _fake_task_cls()
        C.install_task(Task, C.settings_from_env(ON))
        vel = torch.cat([_cmd(0.0, 2.0), _cmd(90.0, 2.0), _cmd(180.0, 2.0)])
        task = Task(vel, [0.0, 0.0, 1.0])
        governed = task._apply_sensor_speed_governor(torch.cat([_cmd(0.0, 3.0), _cmd(90.0, 3.0)]))
        self.assertAlmostEqual(float(governed[0].norm()), 3.0, places=5)
        self.assertAlmostEqual(float(governed[1].norm()), 1.0, places=5)
        self.assertEqual(task.compute_state_reward_and_terminations(), "orig-return")
        self.assertEqual(float(task.rewards[0]), 1.0)                     # in view: unchanged
        expected = 1.0 - 0.5 * (math.pi / 2 - HALF) / (math.pi - HALF)
        self.assertAlmostEqual(float(task.rewards[1]), expected, places=5)
        self.assertEqual(float(task.rewards[2]), -20.0)                   # terminal: untouched
        acc = Task._camsafe_stats["acc"].tolist()
        self.assertEqual(acc[:4], [2.0, 1.0, 2.0, 1.0])                   # cmds, capped, alive, penalised

    def test_static_safety_clip_equals_original_at_full_range_and_differs_at_4m(self):
        g = torch.Generator().manual_seed(7)
        pix = torch.rand((5, 1, 4, 72), generator=g)
        seg = (torch.rand((5, 1, 4, 72), generator=g) < 0.05).float() * 50.0
        dist = (pix.squeeze(1) * RANGE_M).reshape(5, -1)
        flat_seg = seg.squeeze(1).reshape(5, -1)
        self.assertTrue(torch.equal(C.static_safety_r_ss(dist, flat_seg, RANGE_M), original_r_ss(dist, flat_seg)))
        clipped = C.static_safety_r_ss(dist, flat_seg, 4.0)
        self.assertTrue(bool((clipped > original_r_ss(dist, flat_seg)).all()))   # smaller standing cost
        open_space = torch.full((1, 288), RANGE_M)
        self.assertEqual(float(C.static_safety_r_ss(open_space, None, 4.0)), 0.0)
        near = torch.full((1, 288), 0.5)                                  # gradient below 4 m unchanged
        self.assertAlmostEqual(float(C.static_safety_r_ss(near, None, 4.0) - C.static_safety_r_ss(near * 4, None, 4.0)),
                               float(original_r_ss(near, torch.zeros(1, 288)) - original_r_ss(near * 4,
                                                                                               torch.zeros(1, 288))),
                               places=5)

    def test_replaced_static_safety_reward_alive_only(self):
        Task = _fake_task_cls()
        C.install_task(Task, C.settings_from_env(ON))
        pix = torch.full((2, 1, 4, 72), 1.0 / 12.0)                       # every ray at 1 m
        task = Task(torch.zeros(2, 2), [0.0, 1.0], pix=pix)
        task.add_static_safety_reward()
        self.assertAlmostEqual(float(task.rewards[0]), 1.5 * math.log(1.0 / 4.0), places=5)
        self.assertEqual(float(task.rewards[1]), 0.0)

    def test_reward_override(self):
        class Config:
            reward_parameters = {"collision_penalty": -20.0}

        info = C.apply_reward_override(Config, C.settings_from_env(ON))
        self.assertEqual(info, {"collision_penalty_before": -20.0, "collision_penalty_after": -40.0})


class OrderWithRealAblation(unittest.TestCase):
    """The production order (ablation first, then camsafe) with the real navrl_sensor_ablation.install_governor on a
    fake task module (design review r1 P3)."""

    def test_order(self):
        import types

        A = _load("camsafe_test_ablation", ROOT / "aerial_gym/task/navrl_task/navrl_sensor_ablation.py")
        mod = types.ModuleType("camsafe_fake_task_mod")
        exec(compile(
            "import torch\n"
            "def directional_lidar_clearance(scan_m, bearings_rad, command_xy, vertical_fov_deg=(-20.0, 20.0), **kw):\n"
            "    return torch.full((command_xy.shape[0],), 99.0)\n"
            "class Cfg:\n"
            "    mode = 'riskcap'; open_width_enabled = False; lateral_channel_enabled = False; yaw_cap_enabled = False\n"
            "class Task:\n"
            "    speed_governor_cfg = Cfg()\n"
            "    def _apply_sensor_speed_governor(self, command_xy):\n"
            "        return command_xy * 1.0\n"
            "    def compute_state_reward_and_terminations(self):\n"
            "        return None\n"
            "    def add_static_safety_reward(self):\n"
            "        return None\n", "camsafe_fake_task_mod", "exec"), mod.__dict__)
        A.install_governor(mod.Task, "no_lidar")
        try:
            C.install_task(mod.Task, C.settings_from_env(ON))

            class P:
                hfov = math.radians(66.0)
                _ablation_removed = torch.ones(72, dtype=torch.bool)

            t = mod.Task()
            t.perception = P()
            out = t._apply_sensor_speed_governor(torch.tensor([[3.0, 0.0], [0.0, 3.0], [2.5, 2.5]]))
            self.assertEqual([round(float(x), 3) for x in out.norm(dim=1)], [3.0, 1.0, 1.0])
            self.assertIs(mod.__dict__["directional_lidar_clearance"].__module__, A.__name__)   # ablation's binding
        finally:
            C.uninstall()


class RealvehEntry(unittest.TestCase):
    BASE = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/receipt.json", "NAVRL_SENSOR_ABLATION": "no_lidar",
            "NAVRL_CAMTRAIN_HFOV_DEG": "66", "NAVRL_CAMTRAIN_VFOV_DEG": "54",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}

    def test_off_leaves_settings_without_camsafe_key(self):
        self.assertNotIn("camsafe", E.settings(dict(self.BASE)))
        self.assertNotIn("camsafe", E.settings(dict(self.BASE, NAVRL_CAMSAFE="0")))

    def test_on_parses_and_stray_is_refused(self):
        self.assertEqual(E.settings(dict(self.BASE, **ON))["camsafe"]["NAVRL_CAMSAFE_BLIND_MPS"], 1.0)
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, NAVRL_CAMSAFE_BLIND_MPS="1.0"))
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **dict(ON, NAVRL_CAMSAFE_BLIND_MPS="9")))

    def test_camsafe_not_in_hashed_so_earlier_receipts_are_unchanged(self):
        self.assertNotIn("camsafe", E.HASHED)
        self.assertTrue(E.CAMSAFE.is_file())


if __name__ == "__main__":
    unittest.main()
