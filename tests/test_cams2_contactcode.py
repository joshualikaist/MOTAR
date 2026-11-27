"""CPU tests of navrl_cams2_contactcode.py (amendment 1 of the CAM-S2a-E preregistration): the eval-only crash-cause code
for the cams2 converted target contacts, over the REAL navrl_cams2 wrapper on a fake task that reproduces the task's
cause-code, diag-tally and strata-validator arithmetic (navrl_task.py:6137-6141, 6160-6166, 8514-8520, 9045-9050) with
float32 tensors. Every refusal has a positive control; the validator's arithmetic is shown to hold WITH the fix and to fail
WITHOUT it (the 2026-11-24 identity_01 failure).
Run from tests/: python -B -m unittest test_cams2_contactcode"""
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


CC = _load("contactcode_under_test", ROOT / "aerial_gym/task/navrl_task/navrl_cams2_contactcode.py")
C2 = _load("contactcode_test_cams2", ROOT / "aerial_gym/task/navrl_task/navrl_cams2.py")
D = _load("contactcode_test_cams2diag", ROOT / "aerial_gym/task/navrl_task/navrl_cams2diag.py")
E = _load("contactcode_test_realveh_entry", ROOT / "tools/realveh_entry.py")

CAMS2_ON = {"NAVRL_CAMS2": "1", "NAVRL_CAMS2_DELAY_MIN": "1", "NAVRL_CAMS2_DELAY_MAX": "2", "NAVRL_CAMS2_DELAY_SEED": "4961",
            "NAVRL_CAMS2_CAPTURE_RADIUS_M": "1.0", "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS": "1.0",
            "NAVRL_CAMS2_CONTACT_RADIUS_M": "0.40", "NAVRL_CAMS2_FLIGHT_ALTITUDE_M": "1.5", "NAVRL_CAMS2_CAMERA_X_M": "0.08",
            "NAVRL_ACTION_DELAY_STEPS_MAX": "2"}
CAMSAFE_ON = {"NAVRL_CAMSAFE": "1", "NAVRL_CAMSAFE_BLIND_MPS": "1.0", "NAVRL_CAMSAFE_BLIND_WEIGHT": "0.5",
              "NAVRL_CAMSAFE_COLLISION_PENALTY": "-40", "NAVRL_CAMSAFE_RSS_CLIP_M": "4.0"}
FAKE_MOD = types.ModuleType("contactcode_fake_task_module")
FAKE_MOD._episode_limit_reached = lambda sim_steps, limit: sim_steps >= int(limit)
sys.modules[FAKE_MOD.__name__] = FAKE_MOD


def _fake_task_cls():
    """The task's outcome resolution in miniature (navrl_task.py:6118-6166): the swept test at success_radius wins over a
    same-step crash; cause codes -1 then 0..3; the _diag tallies; float32 command."""
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
            self._crash_diag = True
            self._diag = {k: 0 for k in ("contact", "below", "above", "oob")}
            self._diag_steps = {"contact": 0.0, "oob": 0.0, "below": 0.0}
            self._diag_x_sum = 0.0
            self.target_position = torch.zeros(n, 3)
            self.target_vel_w = torch.zeros(n, 3)
            self.obs_dict = {"robot_position": torch.zeros(n, 3), "robot_linvel": torch.zeros(n, 3)}
            self.command = torch.zeros(n, 4, dtype=torch.float32)
            self.sim_env = types.SimpleNamespace(sim_steps=torch.zeros(n, dtype=torch.long))
            self.swept_contact = torch.zeros(n, dtype=torch.bool)      # the task's swept test at the contact radius
            self.task_crash = torch.zeros(n, dtype=torch.bool)
            self.task_cause = torch.full((n,), 0, dtype=torch.int8)

        def transform_action_to_command(self, actions):
            self.command[:, 0:3] = actions[:, 0:3].clamp(-1, 1) * self.task_config.max_velocity
            self.command[:, 3] = actions[:, 3].clamp(-1, 1) * self.task_config.yaw_rate_max
            return self.command

        def reset_idx(self, env_ids):
            return None

        def compute_state_reward_and_terminations(self):
            self.rewards[:] = 1.0
            captured = self.swept_contact.clone()
            crashed_out = self.task_crash.clone() & ~captured
            self.captured_now = captured
            self.crashed_now = crashed_out
            self._crash_cause_code.fill_(-1)
            self._crash_cause_code[crashed_out] = self.task_cause[crashed_out]
            for code, key in ((0, "contact"), (1, "below"), (2, "above"), (3, "oob")):
                self._diag[key] += int((crashed_out & (self.task_cause == code)).sum().item())
            self.terminations[:] = (captured | crashed_out).float()

    return Task


def strata_cause_sum(task):
    """The validator's left-hand side (navrl_task.py:8514-8520, 9045-9050): crashed envs whose code is in 0..3."""
    crashed = task.crashed_now.bool()
    code = task._crash_cause_code.long()
    return int((crashed & (code >= 0) & (code < 4)).sum().item()), int(crashed.sum().item())


class Settings(unittest.TestCase):
    def test_on_off_refusals(self):
        self.assertIsNone(CC.settings_from_env({}))
        self.assertIsNone(CC.settings_from_env({"NAVRL_CAMS2_CONTACTCODE": "0"}))
        self.assertEqual(CC.settings_from_env({"NAVRL_CAMS2_CONTACTCODE": "1"}), {})
        with self.assertRaises(ValueError):
            CC.settings_from_env({"NAVRL_CAMS2_CONTACTCODE": "yes"})


class Install(unittest.TestCase):
    def tearDown(self):
        D.uninstall()
        C2.uninstall()
        CC.uninstall()

    def test_install_order_refusals_with_positive_control(self):
        Task = _fake_task_cls()
        Task._camsafe_settings = None
        with self.assertRaises(RuntimeError):
            CC.install_task(Task)                                                        # before camsafe
        Task._camsafe_settings = {"x": 1}
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
        with self.assertRaises(RuntimeError):
            CC.install_task(Task)                                                        # after cams2: wrong layer
        C2.uninstall()
        info = CC.install_task(Task)                                                     # positive control
        self.assertEqual(info["contact_code"], 0)
        with self.assertRaises(RuntimeError):
            CC.install_task(Task)                                                        # twice
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)          # cams2 installs over it

    def _scenario(self, with_fix):
        """env 0: swept contact (converted by cams2); env 1: bar contact (code 0); env 2: oob (code 3); env 3: swept
        contact AND same-step bar contact (the task lets the capture win); env 4: alive."""
        Task = _fake_task_cls()
        if with_fix:
            CC.install_task(Task)
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
        task = Task(5)
        task.target_position[:, 2] = 1.5
        task.obs_dict["robot_position"] = torch.tensor([[0.3, 0., 1.5], [5., 0., 1.5], [30., 0., 1.5], [0.35, 0., 1.5], [6., 0., 1.5]],
                                                       dtype=torch.float32)
        task.sim_env.sim_steps = torch.tensor([12, 40, 7, 9, 3])
        task.swept_contact = torch.tensor([True, False, False, True, False])
        task.task_crash = torch.tensor([False, True, True, True, False])
        task.task_cause = torch.tensor([0, 0, 3, 0, 0], dtype=torch.int8)
        task.compute_state_reward_and_terminations()
        return Task, task

    def test_validator_arithmetic_fails_without_the_fix_and_holds_with_it(self):
        _, task = self._scenario(with_fix=False)
        coded, crashes = strata_cause_sum(task)
        self.assertEqual(crashes, 4)                                                     # 2 converted + bar + oob
        self.assertEqual(coded, 2)                                                       # the 2026-11-24 failure mode
        self.assertNotEqual(coded, crashes)
        self.assertEqual(task._crash_cause_code.tolist(), [-1, 0, 3, -1, -1])
        self.assertEqual(task._diag["contact"], 1)
        D.uninstall(); C2.uninstall(); CC.uninstall()
        Task, task = self._scenario(with_fix=True)
        coded, crashes = strata_cause_sum(task)
        self.assertEqual((coded, crashes), (4, 4))                                       # the validator's equality holds
        self.assertEqual(task._crash_cause_code.tolist(), [0, 0, 3, 0, -1])
        self.assertEqual(task.crashed_now.tolist(), [True, True, True, True, False])      # cams2's conversion unchanged
        self.assertEqual(task.captured_now.tolist(), [False] * 5)
        self.assertEqual(task.rewards.tolist(), [-40.0, 1.0, 1.0, -40.0, 1.0])            # rewards as cams2 leaves them
        self.assertEqual(task._cams2_contact_now.tolist(), [True, False, False, True, False])
        self.assertEqual(task._diag["contact"], 3)                                        # bar contact + 2 target contacts
        self.assertEqual(task._diag["oob"], 1)
        self.assertAlmostEqual(task._diag_steps["contact"], 12.0 + 9.0)
        self.assertAlmostEqual(task._diag_x_sum, 0.3 + 0.35, places=5)
        self.assertEqual(Task._cams2_contactcode_stats, {"steps": 1, "contacts": 2, "contact_steps": 1})
        self.assertEqual(Task._cams2_stats["contacts"], 2)

    def test_recorder_classifies_from_the_mask_and_refuses_without_the_fix(self):
        Task = _fake_task_cls()
        C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
        with tempfile.TemporaryDirectory() as tmp:
            path = str(pathlib.Path(tmp) / "rows.jsonl")
            with self.assertRaises(RuntimeError):
                D.install_task(Task, {"NAVRL_CAMS2DIAG_JSONL": path, "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 1.047}, 1.0)   # no fix
            C2.uninstall()
            Task, task = self._scenario(with_fix=True)                                   # fix inside cams2
            D.install_task(Task, {"NAVRL_CAMS2DIAG_JSONL": path, "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 1.047}, 1.0)
            task.transform_action_to_command(torch.zeros(5, 4, dtype=torch.float32))
            task.compute_state_reward_and_terminations()                                 # same scenario, recorded
            rows = {r["env_index"]: r for r in (json.loads(l) for l in pathlib.Path(path).read_text().splitlines())}
        self.assertEqual(sorted(rows), [0, 1, 2, 3])
        self.assertEqual([rows[i]["crash_class"] for i in (0, 1, 2, 3)],
                         ["target_contact_swept", "contact", "out_of_bounds", "target_contact_swept"])
        self.assertTrue(all(rows[i]["outcome"] == "crash" for i in (0, 1, 2, 3)))
        coded, crashes = strata_cause_sum(task)
        self.assertEqual((coded, crashes), (4, 4))

    def test_nodiag_path_holds(self):
        """S12k_nodiag: the fix without the recorder."""
        Task, task = self._scenario(with_fix=True)
        self.assertIsNone(getattr(Task, "_cams2diag_settings", None))
        coded, crashes = strata_cause_sum(task)
        self.assertEqual((coded, crashes), (4, 4))
        self.assertTrue(hasattr(task, "_cams2_contact_now"))

    def test_telemetry_line_and_parser(self):
        import io
        Task = _fake_task_cls()
        buf = io.StringIO()
        real = sys.stderr
        sys.stderr = buf
        try:
            CC.install_task(Task)
            C2.install_task(Task, C2.settings_from_env(CAMS2_ON), Task.task_config)
            task = Task(1)
            task.target_position[:, 2] = 1.5
            task.obs_dict["robot_position"] = torch.tensor([[5., 0., 1.5]])
            for i in range(CC.REPORT_AT_CALLS[0]):
                task.swept_contact = torch.tensor([i == 0])
                task.compute_state_reward_and_terminations()
        finally:
            sys.stderr = real
        lines = CC.telemetry_lines([buf.getvalue()])
        self.assertEqual(len(lines), 1)
        self.assertEqual((lines[0]["steps"], lines[0]["contacts"], lines[0]["contact_steps"]), (32, 1, 1))


class RealvehEntryHook(unittest.TestCase):
    BASE = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/receipt.json", "NAVRL_SENSOR_ABLATION": "no_lidar",
            "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}

    def test_needs_cams2_and_parses(self):
        on = {"NAVRL_CAMS2_CONTACTCODE": "1"}
        self.assertEqual(E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON, **on))["cams2_contactcode"], {})
        self.assertNotIn("cams2_contactcode", E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON)))
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **CAMSAFE_ON, **on))                              # without cams2
        with self.assertRaises(SystemExit):
            E.settings(dict(self.BASE, **CAMSAFE_ON, **CAMS2_ON, NAVRL_CAMS2_CONTACTCODE="x"))

    def test_install_order_in_the_entry_source(self):
        src = (ROOT / "tools/realveh_entry.py").read_text()
        order = [src.index(m) for m in ("ablation.install(", "camsafe.install(", "contactcode.install(", "cams2.install(",
                                        "yawclip.install(", "cams2diag.install(")]
        self.assertEqual(order, sorted(order))
        self.assertNotIn("cams2_contactcode", E.HASHED)


if __name__ == "__main__":
    unittest.main()
