"""CPU tests of navrl_cams2diag.py (draft): the per-episode recorder of the CAM-S2a evaluation, installed over a fake
task that carries navrl_cams2's real wrapper (loaded from the repository, read-only) so that the recorder sees exactly
what cams2 leaves behind: a contact converted to a crash with cause code -1, a wrapper capture, a task crash with its
cause code, and a timeout. Every refusal has a positive control; every count is asserted non-zero.
Run from tests/: python -B -m unittest test_cams2diag"""
import importlib.util
import json
import pathlib
import sys
import tempfile
import types
import unittest

import torch

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


D = _load("cams2diag_under_test", ROOT / "aerial_gym/task/navrl_task/navrl_cams2diag.py")
C2 = _load("cams2diag_test_cams2", ROOT / "aerial_gym/task/navrl_task/navrl_cams2.py")
E = _load("cams2diag_test_realveh_entry", ROOT / "tools/realveh_entry.py")

CAMS2_ON = {"NAVRL_CAMS2": "1", "NAVRL_CAMS2_DELAY_MIN": "1", "NAVRL_CAMS2_DELAY_MAX": "2", "NAVRL_CAMS2_DELAY_SEED": "4961",
            "NAVRL_CAMS2_CAPTURE_RADIUS_M": "1.0", "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS": "1.0",
            "NAVRL_CAMS2_CONTACT_RADIUS_M": "0.40", "NAVRL_CAMS2_FLIGHT_ALTITUDE_M": "1.5", "NAVRL_CAMS2_CAMERA_X_M": "0.08",
            "NAVRL_ACTION_DELAY_STEPS_MAX": "2"}
CAMSAFE_ON = {"NAVRL_CAMSAFE": "1", "NAVRL_CAMSAFE_BLIND_MPS": "1.0", "NAVRL_CAMSAFE_BLIND_WEIGHT": "0.5",
              "NAVRL_CAMSAFE_COLLISION_PENALTY": "-40", "NAVRL_CAMSAFE_RSS_CLIP_M": "4.0"}

# A stand-in for navrl_task's module-level _episode_limit_reached, looked up through sys.modules[task.__module__].
FAKE_MOD = types.ModuleType("cams2diag_fake_task_module")
FAKE_MOD._episode_limit_reached = lambda sim_steps, limit: sim_steps >= int(limit)
sys.modules[FAKE_MOD.__name__] = FAKE_MOD


def _fake_task_cls():
    class Task:
        __module__ = FAKE_MOD.__name__
        _camsafe_settings = {"x": 1}

        class task_config:
            yaw_rate_max = 3.0
            max_velocity = 2.5
            episode_len_steps = 600
            success_radius = 0.5
            flight_altitude = 1.0

            class vision:
                camera_translation = [0.10, 0.0, 0.03]

        def __init__(self, n):
            self.num_envs = n
            self.rw = {"collision_penalty": -40.0, "capture_bonus": 30.0}
            self.rewards = torch.zeros(n)
            self.terminations = torch.zeros(n)
            self.truncations = torch.zeros(n)
            self.captured_now = torch.zeros(n, dtype=torch.bool)
            self.crashed_now = torch.zeros(n, dtype=torch.bool)
            self._visible_now = torch.ones(n, dtype=torch.bool)
            self._crash_cause_code = torch.full((n,), -1, dtype=torch.int8)
            self.target_position = torch.zeros(n, 3)
            self.target_vel_w = torch.zeros(n, 3)
            self.obs_dict = {"robot_position": torch.zeros(n, 3), "robot_linvel": torch.zeros(n, 3)}
            self.command = torch.zeros(n, 4)
            self.sim_env = types.SimpleNamespace(sim_steps=torch.zeros(n, dtype=torch.long))
            # what the task's own swept test / crash code will report this step
            self.orig_captured = torch.zeros(n, dtype=torch.bool)
            self.orig_crashed = torch.zeros(n, dtype=torch.bool)
            self.orig_cause = torch.full((n,), -1, dtype=torch.int8)

        def transform_action_to_command(self, actions):
            self.command[:, 0:3] = actions[:, 0:3].clamp(-1, 1) * self.task_config.max_velocity
            self.command[:, 3] = actions[:, 3].clamp(-1, 1) * self.task_config.yaw_rate_max
            return self.command

        def reset_idx(self, env_ids):
            return None

        def compute_state_reward_and_terminations(self):
            # navrl_task.py:6118-6141 in miniature: captured beats crashed; cause codes for crashes
            self.rewards[:] = 1.0
            self.captured_now = self.orig_captured.clone()
            self.crashed_now = self.orig_crashed.clone() & ~self.captured_now
            self._crash_cause_code = torch.where(self.crashed_now, self.orig_cause, torch.full_like(self.orig_cause, -1))
            self.terminations[:] = (self.captured_now | self.crashed_now).float()

    return Task


class Settings(unittest.TestCase):
    def test_off_on_and_refusals(self):
        self.assertIsNone(D.settings_from_env({}))
        with tempfile.TemporaryDirectory() as tmp:
            path = str(pathlib.Path(tmp) / "rows.jsonl")
            on = {"NAVRL_CAMS2DIAG": "1", "NAVRL_CAMS2DIAG_JSONL": path, "NAVRL_CAMS2DIAG_YAW_REF_RADPS": "1.047"}
            self.assertEqual(D.settings_from_env(on), {"NAVRL_CAMS2DIAG_JSONL": path, "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 1.047})
            bad = [dict(on, NAVRL_CAMS2DIAG_YAW_REF_RADPS=""), dict(on, NAVRL_CAMS2DIAG_YAW_REF_RADPS="abc"),
                   dict(on, NAVRL_CAMS2DIAG_YAW_REF_RADPS="5.0"), {"NAVRL_CAMS2DIAG_YAW_REF_RADPS": "1.047"}]
            pathlib.Path(path).write_text("")
            bad += [{"NAVRL_CAMS2DIAG_JSONL": path}, {"NAVRL_CAMS2DIAG": "on", "NAVRL_CAMS2DIAG_JSONL": path},
                    {"NAVRL_CAMS2DIAG": "1"}, dict(on, NAVRL_CAMS2DIAG_JSONL="relative.jsonl"),
                    on]                                                                   # the last: file exists
            for env in bad:
                with self.subTest(env=env), self.assertRaises(ValueError):
                    D.settings_from_env(env)


class Pure(unittest.TestCase):
    def test_capture_kinematics_split_radial_and_lateral(self):
        rel = torch.tensor([[0.8, 0.0, 0.0], [0.8, 0.0, 0.0]], dtype=torch.float64)
        vel = torch.tensor([[-0.6, 0.0, 0.0], [0.0, 2.5, 0.0]], dtype=torch.float64)   # head-on; pure fly-by
        dist, closing, speed, lateral = D.capture_kinematics(rel, vel)
        self.assertEqual(dist.tolist(), [0.8, 0.8])
        self.assertAlmostEqual(closing[0].item(), 0.6)
        self.assertAlmostEqual(lateral[0].item(), 0.0)
        self.assertAlmostEqual(closing[1].item(), 0.0)
        self.assertAlmostEqual(lateral[1].item(), 2.5)
        self.assertAlmostEqual(speed[1].item(), 2.5)

    def test_classify(self):
        crashed = torch.tensor([True, True, False, False, True])
        captured = torch.tensor([False, False, True, False, False])
        code = torch.tensor([-1, 0, -1, -1, 3])
        self.assertEqual(D.classify(crashed, captured, code),
                         ["crash:target_contact_swept", "crash:contact", "capture", "", "crash:out_of_bounds"])

    def test_telemetry_lines(self):
        text = "x\n[cams2diag] {\"event\": \"summary\", \"rows\": 3}\n[camsafe] calls=1\n[cams2diag] not json\n"
        self.assertEqual(D.telemetry_lines([text]), [{"event": "summary", "rows": 3}])


class Install(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = str(pathlib.Path(self.tmp.name) / "cams2diag_episodes.jsonl")

    def tearDown(self):
        D.uninstall()
        C2.uninstall()
        self.tmp.cleanup()

    SETTINGS_KEY = "NAVRL_CAMS2DIAG_YAW_REF_RADPS"

    def _settings(self, path=None):
        return {"NAVRL_CAMS2DIAG_JSONL": path or self.path, self.SETTINGS_KEY: 1.047}

    def _install(self):
        Task = _fake_task_cls()
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
        info = D.install_task(Task, self._settings(), 1.0)
        return Task, info

    def test_refuses_without_cams2_and_twice(self):
        Task = _fake_task_cls()
        with self.assertRaises(RuntimeError):
            D.install_task(Task, self._settings(), 1.0)
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
        info = D.install_task(Task, self._settings(), 1.0)                              # positive control
        self.assertEqual(info["yaw_ref_radps"], 1.047)
        with self.assertRaises(RuntimeError):
            D.install_task(Task, self._settings(), 1.0)

    def test_rows_separate_target_contact_capture_obstacle_crash_and_timeout(self):
        Task, info = self._install()
        self.assertEqual(info["wrapped"], ["transform_action_to_command", "compute_state_reward_and_terminations"])
        task = Task(5)
        task.target_position[:, 2] = 1.5
        # two steps: step 1 nobody finishes (hidden for envs 0, 1), step 2 outcomes
        task._visible_now = torch.tensor([False, False, True, True, True])
        act = torch.tensor([[0.8, 0.2, 0., 1.0], [0.4, -0.4, 0., -0.5], [1.0, 0., 0., 0.], [0., 0., 0., 0.], [0.2, 0., 0., 0.1]])
        task.transform_action_to_command(act)
        task.compute_state_reward_and_terminations()
        self.assertFalse(pathlib.Path(self.path).read_text().strip())                  # no episode finished yet
        task._visible_now = torch.tensor([True, False, True, True, True])
        task.transform_action_to_command(act)
        # env 0: task swept test inside 0.40 m -> cams2 converts to a crash (cause stays -1)
        # env 1: cams2 capture (0.9 m, closing 0.5 m/s, visible last step? no: hidden -> no capture; make it visible)
        task._visible_now = torch.tensor([True, True, True, True, True])
        task.orig_captured = torch.tensor([True, False, False, False, False])
        task.orig_crashed = torch.tensor([False, False, True, False, False])
        task.orig_cause = torch.tensor([-1, -1, 0, -1, -1], dtype=torch.int8)
        task.obs_dict["robot_position"] = torch.tensor([[0.3, 0., 1.5], [0.9, 0., 1.5], [5., 0., 1.5], [6., 0., 1.5], [0.8, 0., 1.5]])
        task.obs_dict["robot_linvel"] = torch.tensor([[0., 0., 0.], [-0.5, 0., 0.], [1., 0., 0.], [0., 0., 0.], [0., 2.0, 0.]])
        task.sim_env.sim_steps = torch.tensor([2, 2, 2, 600, 2])                       # env 3 hits the step limit
        task.compute_state_reward_and_terminations()
        rows = [json.loads(l) for l in pathlib.Path(self.path).read_text().splitlines()]
        by = {r["env_index"]: r for r in rows}
        self.assertEqual(sorted(by), [0, 1, 2, 3, 4])                                   # env 4 captured too (fly-by)
        self.assertEqual((by[0]["outcome"], by[0]["crash_class"]), ("crash", "target_contact_swept"))
        self.assertEqual((by[1]["outcome"], by[1]["crash_class"]), ("capture", "none"))
        self.assertEqual((by[2]["outcome"], by[2]["crash_class"]), ("crash", "contact"))
        self.assertEqual((by[3]["outcome"], by[3]["crash_class"]), ("timeout", "none"))
        self.assertEqual(by[4]["outcome"], "capture")
        self.assertAlmostEqual(by[1]["capture_closing_mps"], 0.5, places=6)
        self.assertAlmostEqual(by[1]["capture_lateral_mps"], 0.0, places=6)
        self.assertAlmostEqual(by[4]["capture_rel_speed_mps"], 2.0, places=6)          # the fly-by is visible (F7)
        self.assertAlmostEqual(by[4]["capture_lateral_mps"], 2.0, places=6)
        self.assertIsNone(by[0]["capture_dist_m"])
        for r in rows:
            self.assertEqual(r["episode_steps"], 2)
            self.assertEqual(r["completion_step"], 2)
            self.assertEqual(r["env_episode_index"], 0)
            self.assertIn(r["delay_k"], (1, 2))
        # blind behaviour: env 0 hidden in step 1 only; env 1 hidden in both steps
        self.assertEqual((by[0]["hidden_steps"], by[0]["visible_steps"]), (1, 1))
        self.assertEqual((by[1]["hidden_steps"], by[1]["visible_steps"]), (2, 0))
        self.assertAlmostEqual(by[0]["hidden_raw_vx_mps"], 2.0, places=6)              # 0.8 * 2.5
        self.assertAlmostEqual(by[0]["hidden_raw_abs_yaw_radps"], 3.0, places=6)       # 1.0 * 3.0
        self.assertAlmostEqual(by[1]["hidden_raw_yaw_radps"], -1.5, places=6)
        self.assertAlmostEqual(by[1]["hidden_raw_speed_mps"], (0.4 ** 2 + 0.4 ** 2) ** 0.5 * 2.5, places=6)
        self.assertEqual(by[2]["hidden_steps"], 0)
        self.assertIsNone(by[2]["hidden_raw_vx_mps"])
        # executed (delayed) command: zero in step 1 for every env (history empty), so over-cap counts come from step 2:
        # env 0 was hidden only in step 1 (exec 0) -> 0; env 1 hidden in both: step 2 executes act[1] of step 1 when k=1
        # (speed 0.4*sqrt2*2.5 = 1.41 > 1.0 -> 1) or the zero history slot when k=2 -> 0 (review P3-1: exact counts)
        self.assertEqual(by[0]["hidden_exec_over_blind_cap"], 0)
        self.assertEqual(by[1]["hidden_exec_over_blind_cap"], 1 if by[1]["delay_k"] == 1 else 0)
        # yaw reference counters (review P1-2): env 0 requests 3.0 rad/s in both steps (> 1.047), env 1 -1.5 in both,
        # env 4 0.3 in both; executed yaw is the delayed command: step 1 zero, step 2 act of step 1 when k=1
        self.assertEqual((by[0]["yaw_request_over_ref_steps"], by[1]["yaw_request_over_ref_steps"], by[4]["yaw_request_over_ref_steps"]), (2, 2, 0))
        self.assertEqual(by[0]["yaw_exec_over_ref_steps"], 1 if by[0]["delay_k"] == 1 else 0)
        self.assertEqual(by[0]["yaw_ref_radps"], 1.047)
        st = Task._cams2diag_stats
        self.assertEqual(st["totals"], {"capture": 2, "crash": 2, "timeout": 1, "hidden_steps": 3, "visible_steps": 7})
        self.assertEqual(st["classes"], {"target_contact_swept": 1, "none": 3, "contact": 1})
        self.assertEqual(Task._cams2_stats["contacts"], 1)
        self.assertEqual(Task._cams2_stats["captures"], 2)

    def test_episode_index_advances_and_accumulators_reset(self):
        Task, _ = self._install()
        task = Task(2)
        task.target_position[:, 2] = 1.5
        task.obs_dict["robot_position"] = torch.tensor([[5., 0., 1.5], [5., 0., 1.5]])
        act = torch.zeros(2, 4)
        task._visible_now = torch.tensor([False, True])
        task.transform_action_to_command(act)
        task.orig_crashed = torch.tensor([True, False])
        task.orig_cause = torch.tensor([3, -1], dtype=torch.int8)
        task.compute_state_reward_and_terminations()                                  # env 0 ends (oob)
        task.orig_crashed = torch.tensor([False, False])
        task._visible_now = torch.tensor([True, True])
        task.transform_action_to_command(act)
        task.orig_crashed = torch.tensor([True, True])
        task.orig_cause = torch.tensor([1, 0], dtype=torch.int8)
        task.compute_state_reward_and_terminations()                                  # both end
        rows = [json.loads(l) for l in pathlib.Path(self.path).read_text().splitlines()]
        self.assertEqual([(r["env_index"], r["env_episode_index"], r["episode_steps"], r["hidden_steps"], r["crash_class"])
                          for r in rows],
                         [(0, 0, 1, 1, "out_of_bounds"), (0, 1, 1, 0, "below"), (1, 0, 2, 0, "contact")])
        self.assertEqual([r["completion_step"] for r in rows], [1, 2, 2])

    def test_summary_line_carries_cams2_counters(self):
        Task, _ = self._install()
        task = Task(1)
        task.target_position[:, 2] = 1.5
        task.obs_dict["robot_position"] = torch.tensor([[0.3, 0., 1.5]])
        import io
        buf = io.StringIO()
        real = sys.stderr
        D.uninstall()
        sys.stderr = buf
        try:
            D.install_task(Task, self._settings(), 1.0)
            for i in range(D.REPORT_AT_CALLS[0]):
                task.orig_captured = torch.tensor([i == 0])                               # one contact, then idle
                task.compute_state_reward_and_terminations()
                task.obs_dict["robot_position"] = torch.tensor([[5.0, 0., 1.5]])         # out of the capture radius
        finally:
            sys.stderr = real
        lines = D.telemetry_lines([buf.getvalue()])
        self.assertEqual(len(lines), 1)
        self.assertEqual(lines[0]["crash_classes"], {"target_contact_swept": 1})
        self.assertEqual(lines[0]["cams2_stats"]["contacts"], 1)
        self.assertEqual(lines[0]["rows"], 1)
        self.assertIn("snapshot", lines[0])                                               # review P2-10: labelled as running totals

    def test_recorder_is_passive(self):
        """Review r1 P1-1: the same action sequence with and without the recorder gives identical rewards, terminations,
        captured / crashed flags and commands (the CPU side of the S12k_nodiag identity control)."""
        import copy

        def run(with_recorder, path):
            torch.manual_seed(7)
            Task = _fake_task_cls()
            C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
            if with_recorder:
                D.install_task(Task, self._settings(path), 1.0)
            task = Task(6)
            task.target_position[:, 2] = 1.5
            g = torch.Generator().manual_seed(11)
            out = []
            for step in range(12):
                acts = torch.rand((6, 4), generator=g) * 2 - 1
                task._visible_now = torch.rand(6, generator=g) > 0.4
                task.obs_dict["robot_position"] = torch.cat([torch.rand((6, 2), generator=g) * 3, torch.full((6, 1), 1.5)], dim=1)
                task.obs_dict["robot_linvel"] = torch.rand((6, 3), generator=g) * 2 - 1
                task.orig_captured = torch.rand(6, generator=g) > 0.9
                task.orig_crashed = torch.rand(6, generator=g) > 0.85
                task.orig_cause = (torch.rand(6, generator=g) * 4).to(torch.int8)
                task.sim_env.sim_steps = torch.full((6,), 600 if step == 9 else step)
                cmd = task.transform_action_to_command(acts).clone()
                task.compute_state_reward_and_terminations()
                out.append((cmd, task.rewards.clone(), task.terminations.clone(), task.captured_now.clone(), task.crashed_now.clone()))
            stats = copy.deepcopy(Task._cams2_stats)
            D.uninstall()
            C2.uninstall()
            return out, stats

        with tempfile.TemporaryDirectory() as tmp:
            on, stats_on = run(True, str(pathlib.Path(tmp) / "rows.jsonl"))
            rows = pathlib.Path(tmp, "rows.jsonl").read_text().splitlines()
        off, stats_off = run(False, None)
        self.assertGreater(len(rows), 0)                                                  # the recorder did record
        self.assertEqual(stats_on, stats_off)
        self.assertGreater(stats_on["captures"] + stats_on["contacts"], 0)
        for a, b in zip(on, off):
            for x, y in zip(a, b):
                self.assertTrue(torch.equal(x, y))


class Float32Integration(unittest.TestCase):
    """Review r2 P0-2 / P2-13: cams2 -> yawclip -> cams2diag on float32 command tensors, as in the task. A clipped float32
    command reads 1.0470000505; the recorder must count no executed step over the limit, while the request counter sees
    the raw |yaw| = 3.0 rad/s."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.Y = _load("cams2diag_test_yawclip", ROOT / "aerial_gym/task/navrl_task/navrl_yawclip.py")

    def tearDown(self):
        D.uninstall()
        self.Y.uninstall()
        C2.uninstall()
        self.tmp.cleanup()

    def test_clipped_float32_command_counts_zero_executed_over_limit(self):
        Task = _fake_task_cls()
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
        self.Y.install_task(Task, self.Y.settings_from_env({"NAVRL_YAWCLIP": "1", "NAVRL_YAWCLIP_RADPS": "1.047"}), Task.task_config)
        path = str(pathlib.Path(self.tmp.name) / "rows.jsonl")
        D.install_task(Task, {"NAVRL_CAMS2DIAG_JSONL": path, "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 1.047}, 1.0)
        task = Task(3)
        self.assertIs(task.command.dtype, torch.float32)
        task.target_position[:, 2] = 1.5
        task.obs_dict["robot_position"] = torch.tensor([[5., 0., 1.5]] * 3)
        acts = torch.tensor([[0.5, 0., 0., 1.0], [0.5, 0., 0., -1.0], [0.5, 0., 0., 0.2]], dtype=torch.float32)
        for step in range(4):                                                                 # delay 1-2: the clip binds from step 2
            cmd = task.transform_action_to_command(acts)
            self.assertLessEqual(cmd[:, 3].abs().max().item(), 1.0470000505447388 + 1e-9)
            task.compute_state_reward_and_terminations()
        task.orig_crashed = torch.tensor([True, True, True])
        task.orig_cause = torch.tensor([3, 3, 3], dtype=torch.int8)
        task.compute_state_reward_and_terminations()                                         # finish all three episodes
        rows = {r["env_index"]: r for r in (json.loads(l) for l in pathlib.Path(path).read_text().splitlines())}
        self.assertEqual(len(rows), 3)
        self.assertEqual([rows[i]["yaw_exec_over_ref_steps"] for i in range(3)], [0, 0, 0])
        self.assertEqual([rows[i]["yaw_request_over_ref_steps"] for i in range(3)], [4, 4, 0])      # 4 transform calls
        self.assertGreater(Task._yawclip_stats["clipped"], 0)                                 # the clip did bind
        # negative control: without the tolerance the float32 clip would count every clipped step
        clipped = torch.tensor([2.9, -2.9], dtype=torch.float32).clamp(-1.047, 1.047).double().abs()
        self.assertTrue(bool((clipped > 1.047).all()))
        self.assertFalse(bool((clipped > 1.047 + D.YAW_TOL).any()))


class RealvehEntryHook(unittest.TestCase):
    BASE = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/receipt.json", "NAVRL_SENSOR_ABLATION": "no_lidar",
            "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}

    def test_needs_cams2_and_parses(self):
        on = {"NAVRL_CAMS2DIAG": "1", "NAVRL_CAMS2DIAG_JSONL": "/nonexistent/dir/rows.jsonl", "NAVRL_CAMS2DIAG_YAW_REF_RADPS": "1.047"}
        s = E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON, **on))
        self.assertEqual(s["cams2diag"], {"NAVRL_CAMS2DIAG_JSONL": "/nonexistent/dir/rows.jsonl", "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 1.047})
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON, NAVRL_CAMS2DIAG_YAW_REF_RADPS="1.047"))   # stray while off
        self.assertNotIn("cams2diag", E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON)))
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **CAMSAFE_ON, **on))                              # without cams2
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON, NAVRL_CAMS2DIAG_JSONL="/x.jsonl"))   # stray while off


if __name__ == "__main__":
    unittest.main()
