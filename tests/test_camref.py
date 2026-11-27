"""CPU tests of the CAM-REF package (draft preregistration ~/motar_tmp/camref_20261126/draft/
prereg_camonly_ref_matched_protocol_2026-11-26.md): aerial_gym/task/navrl_task/navrl_camref_rules.py, tools/camref_entry.py,
tools/camref_eval_run.py and tools/camref_eval_analyse.py. Every refusal is paired with a positive control and every
equality test asserts a non-zero count of the events it compares. Tests that read recorded cells or training receipts in
~/motar_tmp or the main checkout are skipped (not passed) when those files are absent. Nothing here touches os.environ
or registers an aerial_gym module in sys.modules (production modules are loaded by path under private names)."""
import hashlib
import importlib.util
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

import torch

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


R = _load("camref_rules_under_test", ROOT / "aerial_gym/task/navrl_task/navrl_camref_rules.py")
C2 = _load("camref_test_cams2", ROOT / "aerial_gym/task/navrl_task/navrl_cams2.py")
CC = _load("camref_test_contactcode", ROOT / "aerial_gym/task/navrl_task/navrl_cams2_contactcode.py")
E = _load("camref_test_entry", TOOLS / "camref_entry.py")
import camref_eval_run as X                                 # noqa: E402
import camref_eval_analyse as A                             # noqa: E402

HAVE_FT = X.D.FT_TRAIN_RECEIPT.is_file()
HAVE_REFS = all((X.reference_dir(ref, s) / "episodes.jsonl").is_file() for ref, s in X.REF_SHA)
needs_ft = unittest.skipUnless(HAVE_FT, "S2b-FT training receipt %s is required (skipped, not passed)" % X.D.FT_TRAIN_RECEIPT)
needs_refs = unittest.skipUnless(HAVE_REFS, "the recorded reference cells are required (skipped, not passed)")

CAPTURE_ON = {"NAVRL_CAMREF_CAPTURE": "1", "NAVRL_CAMREF_CAPTURE_RADIUS_M": "1.0",
              "NAVRL_CAMREF_CAPTURE_MAX_CLOSING_MPS": "1.0", "NAVRL_CAMREF_CONTACT_RADIUS_M": "0.40",
              "NAVRL_CAMREF_DETECT_SOURCE": "visible_now"}
DELAY_ON = {"NAVRL_CAMREF_DELAY": "1", "NAVRL_CAMREF_DELAY_MIN": "1", "NAVRL_CAMREF_DELAY_MAX": "2",
            "NAVRL_CAMREF_DELAY_SEED": "45014"}
GEOM_ON = {"NAVRL_CAMREF_GEOMETRY": "1", "NAVRL_CAMREF_FLIGHT_ALTITUDE_M": "1.5", "NAVRL_CAMREF_CAMERA_X_M": "0.08"}
CAMS2_ON = {"NAVRL_CAMS2": "1", "NAVRL_CAMS2_DELAY_MIN": "1", "NAVRL_CAMS2_DELAY_MAX": "2", "NAVRL_CAMS2_DELAY_SEED": "45014",
            "NAVRL_CAMS2_CAPTURE_RADIUS_M": "1.0", "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS": "1.0",
            "NAVRL_CAMS2_CONTACT_RADIUS_M": "0.40", "NAVRL_CAMS2_FLIGHT_ALTITUDE_M": "1.5", "NAVRL_CAMS2_CAMERA_X_M": "0.08",
            "NAVRL_ACTION_DELAY_STEPS_MAX": "2"}


def _episode_limit_reached(sim_steps, limit):          # read by the recorder through sys.modules[type(task).__module__]
    return sim_steps >= limit


def _cfg():
    class Cfg:
        success_radius = 0.5
        flight_altitude = 1.0
        episode_len_steps = 600

        class vision:
            camera_translation = [0.10, 0.0, 0.03]
            detector_hfov_deg = 87.0
    return Cfg


def _task_cls():
    """A fake NavRLTask whose reward method replays a scripted swept test / crash / cause code."""
    class Task:
        def __init__(self, n):
            self.num_envs = n
            self.rw = {"collision_penalty": -40.0, "capture_bonus": 30.0}
            self.rewards = torch.zeros(n)
            self.terminations = torch.zeros(n)
            self.captured_now = torch.zeros(n, dtype=torch.bool)
            self.crashed_now = torch.zeros(n, dtype=torch.bool)
            self._crash_cause_code = torch.full((n,), -1, dtype=torch.long)
            self._crash_diag = True
            self._diag = {"contact": 0}
            self._diag_steps = {"contact": 0.0}
            self._diag_x_sum = 0.0
            self._visible_now = torch.zeros(n, dtype=torch.bool)
            self._camera_visible_now = torch.zeros(n, dtype=torch.bool)
            self.target_position = torch.zeros(n, 3)
            self.target_vel_w = torch.zeros(n, 3)
            self.obs_dict = {"robot_position": torch.zeros(n, 3), "robot_linvel": torch.zeros(n, 3)}

            class Sim:
                pass
            self.sim_env = Sim()
            self.sim_env.sim_steps = torch.zeros(n, dtype=torch.long)
            self.task_config = _cfg()
            self.applied = []
            self.script = None

        def transform_action_to_command(self, actions):
            self.applied.append(actions.clone())
            return actions.clone()

        def reset_idx(self, env_ids):
            self.sim_env.sim_steps[torch.as_tensor(env_ids).long()] = 0
            return None

        def compute_state_reward_and_terminations(self):
            swept, crash, code, pos, vel, vis = self.script
            self.sim_env.sim_steps += 1
            self.rewards[:] = 1.0
            self.captured_now = swept.clone()
            self.crashed_now = crash.clone()
            self._crash_cause_code.fill_(-1)
            self._crash_cause_code[crash] = code[crash]
            self.terminations[:] = (swept | crash).float()
            self.obs_dict["robot_position"] = pos.clone()
            self.obs_dict["robot_linvel"] = vel.clone()
            self._visible_now = vis.clone()
            self._camera_visible_now = vis.clone() & (torch.arange(self.num_envs) % 2 == 0)
    return Task


def _script(gen, n):
    """One step's scripted task output: ~10 % swept tests, ~10 % crashes (codes 0 / 3), robots within ~1.5 m."""
    swept = torch.rand(n, generator=gen) < 0.10
    crash = (torch.rand(n, generator=gen) < 0.10) & ~swept
    code = torch.where(torch.rand(n, generator=gen) < 0.7, torch.zeros(n, dtype=torch.long), torch.full((n,), 3))
    pos = (torch.rand(n, 3, generator=gen) - 0.5) * 3.0
    vel = (torch.rand(n, 3, generator=gen) - 0.5) * 3.0
    vis = torch.rand(n, generator=gen) < 0.6
    return swept, crash, code, pos, vel, vis


# ================================================================ the rule module
class RuleSettings(unittest.TestCase):
    def test_off_is_none_and_each_rule_switches_alone(self):
        self.assertIsNone(R.settings_from_env({}))
        self.assertIsNone(R.settings_from_env({"NAVRL_CAMREF_CAPTURE": "0", "NAVRL_CAMREF_DELAY": "0"}))
        for env, key in ((CAPTURE_ON, "capture"), (DELAY_ON, "delay"), (GEOM_ON, "geometry")):
            s = R.settings_from_env(env)
            self.assertIsNotNone(s[key])
            self.assertEqual([k for k in ("capture", "delay", "geometry") if s[k] is not None], [key])
            self.assertTrue(R.rules_on(s))
        s = R.settings_from_env({"NAVRL_CAMREF_RECORD": "1", "NAVRL_CAMREF_RECORD_JSONL": "/nonexistent/camref.jsonl"})
        self.assertFalse(R.rules_on(s))                      # the recorder alone is not a rule
        self.assertIsNotNone(s["record"])

    def test_refusals_with_positive_controls(self):
        with tempfile.NamedTemporaryFile() as existing:
            bad = [dict(CAPTURE_ON, NAVRL_CAMREF_CAPTURE="yes"),
                   {"NAVRL_CAMREF_CAPTURE_RADIUS_M": "1.0"},                        # parameter while off
                   {k: v for k, v in CAPTURE_ON.items() if k != "NAVRL_CAMREF_CONTACT_RADIUS_M"},
                   dict(CAPTURE_ON, NAVRL_CAMREF_CONTACT_RADIUS_M="1.0"),           # contact not below capture
                   dict(CAPTURE_ON, NAVRL_CAMREF_DETECT_SOURCE="lidar"),
                   {k: v for k, v in CAPTURE_ON.items() if k != "NAVRL_CAMREF_DETECT_SOURCE"},
                   dict(DELAY_ON, NAVRL_CAMREF_DELAY_MIN="3"), dict(DELAY_ON, NAVRL_CAMREF_DELAY_MIN="1.5"),
                   dict(DELAY_ON, NAVRL_CAMREF_DELAY_MAX="4"), dict(GEOM_ON, NAVRL_CAMREF_FLIGHT_ALTITUDE_M="nan"),
                   dict(GEOM_ON, NAVRL_CAMREF_CAMERA_X_M="0.5"), {"NAVRL_CAMREF_SOMETHING": "1"},
                   {"NAVRL_CAMREF_RECORD": "1"}, {"NAVRL_CAMREF_RECORD": "1", "NAVRL_CAMREF_RECORD_JSONL": "rel.jsonl"},
                   {"NAVRL_CAMREF_RECORD": "1", "NAVRL_CAMREF_RECORD_JSONL": existing.name},
                   {"NAVRL_CAMREF_RECORD_JSONL": "/x/y.jsonl"}]
            for env in bad:
                with self.subTest(env=env), self.assertRaises(ValueError):
                    R.settings_from_env(env)
        good = [CAPTURE_ON, DELAY_ON, GEOM_ON, dict(CAPTURE_ON, NAVRL_CAMREF_DETECT_SOURCE="camera_visible_now"),
                dict(DELAY_ON, NAVRL_CAMREF_DELAY_MIN="0", NAVRL_CAMREF_DELAY_MAX="0"), {**CAPTURE_ON, **DELAY_ON, **GEOM_ON}]
        self.assertEqual(sum(R.settings_from_env(env) is not None for env in good), len(good))

    def test_entry_keys_are_known(self):
        s = R.settings_from_env(dict(CAPTURE_ON, NAVRL_CAMREF_POLICY="F", NAVRL_CAMREF_RECEIPT="/x/r.json"))
        self.assertIsNotNone(s["capture"])


class RulePureEqualsCams2(unittest.TestCase):
    def test_capture_mask_and_delay_equal_cams2_on_random_tensors(self):
        gen = torch.Generator().manual_seed(4611)
        hits = 0
        for _ in range(50):
            n = 64
            rel = (torch.rand(n, 3, generator=gen) - 0.5) * 3.0
            vel = (torch.rand(n, 3, generator=gen) - 0.5) * 3.0
            vis = torch.rand(n, generator=gen) < 0.6
            alive = torch.rand(n, generator=gen) < 0.8
            a = R.capture_mask(rel, vel, vis, alive, 1.0, 1.0)
            b = C2.capture_mask(rel, vel, vis, alive, 1.0, 1.0)
            self.assertTrue(torch.equal(a, b))
            hits += int(a.sum())
            hist = torch.rand(n, 3, 4, generator=gen)
            delay = torch.randint(0, 3, (n,), generator=gen)
            self.assertTrue(torch.equal(R.delayed(hist, delay), C2.delayed(hist, delay)))
        self.assertGreater(hits, 0)                           # the masks were not trivially empty

    def test_config_equals_cams2_apply_config(self):
        a, b = _cfg(), _cfg()
        R.apply_config(a, R.settings_from_env({**CAPTURE_ON, **GEOM_ON}))
        C2.apply_config(b, C2.settings_from_env(CAMS2_ON))
        self.assertEqual((a.success_radius, a.flight_altitude, a.vision.camera_translation),
                         (b.success_radius, b.flight_altitude, b.vision.camera_translation))
        self.assertEqual((a.success_radius, a.flight_altitude, a.vision.camera_translation[0]), (0.40, 1.5, 0.08))
        c = _cfg()
        R.apply_config(c, R.settings_from_env(DELAY_ON))       # a delay-only rule leaves the configuration alone
        self.assertEqual((c.success_radius, c.flight_altitude, c.vision.camera_translation), (0.5, 1.0, [0.10, 0.0, 0.03]))


class RuleInstallEqualsCams2(unittest.TestCase):
    """navrl_camref_rules (capture + delay) against navrl_cams2_contactcode + navrl_cams2 on the same scripted task."""

    def tearDown(self):
        R.uninstall()
        C2.uninstall()
        CC.uninstall()

    def test_step_by_step_equal(self):
        n, steps = 48, 80
        TA, TB = _task_cls(), _task_cls()
        TA._camsafe_settings = {"stub": 1}                    # cams2's install-order guard (camsafe is not exercised)
        CC.install_task(TA)
        C2.install_task(TA, C2.settings_from_env(CAMS2_ON), _cfg())
        info = R.install_rules_task(TB, R.settings_from_env({**CAPTURE_ON, **DELAY_ON}), _cfg())
        self.assertEqual(info["delay_steps"], [1, 2])
        a, b = TA(n), TB(n)
        gen, act_gen = torch.Generator().manual_seed(1), torch.Generator().manual_seed(2)
        totals = {"contacts": 0, "captures": 0, "resets": 0}
        for _ in range(steps):
            actions = torch.rand(n, 4, generator=act_gen)
            ca, cb = a.transform_action_to_command(actions), b.transform_action_to_command(actions)
            self.assertTrue(torch.equal(ca, cb))
            script = _script(gen, n)
            a.script = b.script = script
            a.compute_state_reward_and_terminations()
            b.compute_state_reward_and_terminations()
            for name in ("rewards", "terminations", "captured_now", "crashed_now", "_crash_cause_code"):
                self.assertTrue(torch.equal(getattr(a, name), getattr(b, name)), name)
            self.assertEqual((a._diag, a._diag_steps, a._diag_x_sum), (b._diag, b._diag_steps, b._diag_x_sum))
            self.assertTrue(torch.equal(a._cams2_contact_now, b._camref_contact_now))
            totals["contacts"] += int(b._camref_contact_now.sum())
            totals["captures"] += int((b.captured_now & ~script[0]).sum())
            done = torch.nonzero(b.captured_now | b.crashed_now).flatten().tolist()
            if done:
                a.reset_idx(done)
                b.reset_idx(done)
                totals["resets"] += len(done)
            self.assertTrue(torch.equal(a._cams2_delay, b._camref_delay))
        self.assertGreater(totals["contacts"], 0)
        self.assertGreater(totals["captures"], 0)
        self.assertGreater(totals["resets"], 0)
        self.assertEqual(set(TB._camref_stats["delay_draws"]), {"1", "2"})      # both delays drawn
        self.assertEqual(TA._cams2_stats["contacts"], TB._camref_stats["contacts"])
        self.assertEqual(TA._cams2_stats["captures"], TB._camref_stats["captures"])

    def test_camera_source_differs_from_fused(self):
        """Positive control of the detection setting: the camera-only flag captures a strict subset here."""
        n = 64
        TA, TB = _task_cls(), _task_cls()
        R.install_rules_task(TA, R.settings_from_env(CAPTURE_ON), _cfg())
        R._INSTALLED.pop("rules")                              # two classes; restore by hand below
        R.install_rules_task(TB, R.settings_from_env(dict(CAPTURE_ON, NAVRL_CAMREF_DETECT_SOURCE="camera_visible_now")),
                             _cfg())
        a, b = TA(n), TB(n)
        gen = torch.Generator().manual_seed(7)
        caps_a = caps_b = 0
        for _ in range(40):
            a.script = b.script = _script(gen, n)
            a.compute_state_reward_and_terminations()
            b.compute_state_reward_and_terminations()
            self.assertFalse(bool((b.captured_now & ~a.captured_now).any()))
            caps_a += int(a.captured_now.sum())
            caps_b += int(b.captured_now.sum())
        self.assertGreater(caps_a, caps_b)
        self.assertGreater(caps_b, 0)

    def test_refusals(self):
        T = _task_cls()
        with self.assertRaises(RuntimeError):
            R.install_rules_task(T, R.settings_from_env({"NAVRL_CAMREF_RECORD": "1",
                                                         "NAVRL_CAMREF_RECORD_JSONL": "/nonexistent/x.jsonl"}), _cfg())
        T._cams2_settings = {"x": 1}
        with self.assertRaises(RuntimeError):
            R.install_rules_task(T, R.settings_from_env(CAPTURE_ON), _cfg())
        del T._cams2_settings
        R.install_rules_task(T, R.settings_from_env(CAPTURE_ON), _cfg())          # positive control
        with self.assertRaises(RuntimeError):
            R.install_rules_task(T, R.settings_from_env(CAPTURE_ON), _cfg())      # second install

    def test_uninstall_restores_the_class(self):
        T = _task_cls()
        before = {k: T.__dict__[k] for k in ("transform_action_to_command", "reset_idx",
                                             "compute_state_reward_and_terminations")}
        R.install_rules_task(T, R.settings_from_env({**CAPTURE_ON, **DELAY_ON}), _cfg())
        self.assertEqual(sum(T.__dict__[k] is not v for k, v in before.items()), 3)
        R.uninstall()
        self.assertEqual({k: T.__dict__[k] for k in before}, before)
        self.assertNotIn("_camref_settings", T.__dict__)


class RuleNeutralSettings(unittest.TestCase):
    def tearDown(self):
        R.uninstall()

    def test_delay_zero_applies_the_newest_action(self):
        T = _task_cls()
        R.install_rules_task(T, R.settings_from_env(dict(DELAY_ON, NAVRL_CAMREF_DELAY_MIN="0", NAVRL_CAMREF_DELAY_MAX="0")),
                             _cfg())
        t = T(8)
        gen = torch.Generator().manual_seed(3)
        for _ in range(5):
            act = torch.rand(8, 4, generator=gen)
            self.assertTrue(torch.equal(t.transform_action_to_command(act), act))
        self.assertEqual(len(t.applied), 5)

    def test_delay_one_two_shifts_the_action(self):
        """Positive control of the delay: with k >= 1 the first command is zero and later ones are older actions."""
        T = _task_cls()
        R.install_rules_task(T, R.settings_from_env(DELAY_ON), _cfg())
        t = T(8)
        acts = [torch.full((8, 4), float(i + 1)) for i in range(4)]
        cmds = [t.transform_action_to_command(a) for a in acts]
        self.assertTrue(torch.equal(cmds[0], torch.zeros(8, 4)))
        k = t._camref_delay
        self.assertTrue(bool(((k == 1) | (k == 2)).all()))
        self.assertTrue(torch.equal(cmds[3][:, 0], (4 - k).float()))


class Recorder(unittest.TestCase):
    def tearDown(self):
        R.uninstall()

    def test_rows_classes_and_alignment(self):
        n = 6
        T = _task_cls()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "camref_episodes.jsonl")
            s = R.settings_from_env({**CAPTURE_ON, "NAVRL_CAMREF_RECORD": "1", "NAVRL_CAMREF_RECORD_JSONL": path})
            R.install_rules_task(T, s, _cfg())
            R.install_record_task(T, s)
            t = T(n)
            t.task_config.episode_len_steps = 3
            vis = torch.tensor([True, True, True, True, False, False])
            t._visible_now = vis.clone()                                             # the flag the action was computed on
            t.transform_action_to_command(torch.zeros(n, 4))
            swept = torch.tensor([True, False, False, False, False, False])           # env 0: target contact
            crash = torch.tensor([False, True, True, False, False, False])            # env 1 bar, env 2 out of bounds
            code = torch.tensor([0, 0, 3, -1, -1, -1])
            pos = torch.zeros(n, 3)
            pos[3] = torch.tensor([0.5, 0.0, 0.0])                                    # env 3: slow and detected
            pos[4:] = 5.0                                                             # envs 4, 5: far
            t.script = (swept, crash, code, pos, torch.zeros(n, 3), vis)
            t.compute_state_reward_and_terminations()
            rows = [json.loads(x) for x in pathlib.Path(path).read_text().splitlines()]
            self.assertEqual([(r["env_index"], r["outcome"], r["crash_class"]) for r in rows],
                             [(0, "crash", "target_contact_swept"), (1, "crash", "contact"), (2, "crash", "out_of_bounds"),
                              (3, "capture", "none")])
            self.assertEqual(rows[3]["detected_steps"], 1)
            self.assertAlmostEqual(rows[3]["capture_dist_m"], 0.5, places=6)
            t.reset_idx([0, 1, 2, 3])
            for _ in range(2):                                                       # envs 4, 5 time out at step 3
                t.transform_action_to_command(torch.zeros(n, 4))
                t.script = (torch.zeros(n, dtype=torch.bool), torch.zeros(n, dtype=torch.bool), code, pos,
                            torch.zeros(n, 3), torch.zeros(n, dtype=torch.bool))
                t.compute_state_reward_and_terminations()
            rows = [json.loads(x) for x in pathlib.Path(path).read_text().splitlines()]
            self.assertEqual([(r["env_index"], r["outcome"], r["env_episode_index"]) for r in rows[4:]],
                             [(4, "timeout", 0), (5, "timeout", 0)])

    def test_refuses_inside_cams2(self):
        T = _task_cls()
        s = R.settings_from_env({"NAVRL_CAMREF_RECORD": "1", "NAVRL_CAMREF_RECORD_JSONL": "/nonexistent/r.jsonl"})
        T._cams2_settings = {"x": 1}
        with self.assertRaises(RuntimeError):
            R.install_record_task(T, s)
        del T._cams2_settings
        self.assertEqual(R.install_record_task(T, s)["detect_attribute"], "_visible_now")     # positive control


# ================================================================ the entry
def _f_env(**extra):
    env = {"NAVRL_CAMREF_POLICY": "F", "NAVRL_CAMREF_RECEIPT": "/nonexistent/camref_receipt.json",
           "NAVRL_EVAL_CHECKPOINT": E.POLICY_CKPT["F"]}
    env.update(extra)
    return env


class EntrySettings(unittest.TestCase):
    def test_policy_f_refusals_with_positive_controls(self):
        self.assertIsNone(E.settings(_f_env())["camref"])                        # all off
        good = [_f_env(), _f_env(**CAPTURE_ON), _f_env(NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS="1.047")]
        self.assertEqual(sum(E.settings(env)["policy"] == "F" for env in good), 3)
        bad = [dict(_f_env(), NAVRL_CAMREF_POLICY="G"), dict(_f_env(), NAVRL_EVAL_CHECKPOINT=E.POLICY_CKPT["K1"]),
               dict(_f_env(), NAVRL_CAMREF_RECEIPT="relative.json"), _f_env(NAVRL_SENSOR_ABLATION="full"),
               _f_env(NAVRL_CAMSAFE="1"), _f_env(NAVRL_CAMS2="1"), _f_env(NAVRL_DEPTH_MODEL="oakd"),
               _f_env(NAVRL_P9_ERROR_MODEL="/x"), _f_env(NAVRL_CAMERA_WIDTH="312"), _f_env(NAVRL_CAMTRAIN_HFOV_DEG="66"),
               _f_env(NAVRL_REALVEH_RECEIPT="/x/r.json"), _f_env(NAVRL_CAMTRAIN_RECEIPT="/x/c.json"),
               _f_env(NAVRL_VEH_RAND_SEED="1"), _f_env(NAVRL_CAMREF_FOO="1"), _f_env(NAVRL_ROBOT="navrl_rv05"),
               _f_env(NAVRL_ACTION_DELAY_STEPS_MAX="2")]
        for env in bad:
            with self.subTest(env={k: v for k, v in env.items() if k not in _f_env()} or env), \
                    self.assertRaises(SystemExit):
                E.settings(env)

    @needs_ft
    def test_every_planned_cell_is_accepted_and_bad_combinations_refused(self):
        te = json.loads(X.D.FT_TRAIN_RECEIPT.read_text())["env_navrl"]
        accepted = 0
        with tempfile.TemporaryDirectory() as d:
            envs = {}
            for arm in list(X.ARMS) + list(X.ARM_BASE):
                cd = pathlib.Path(d) / X.cell_id(arm, 205, 4501)
                env = X.cell_navrl_env(arm, 4501, cd, 8, 10 ** 9, te, "sha")
                env.update(X.launcher_values(arm, 205, 4501, cd, X.POLICY_SHA[X.policy(arm)]))
                s = E.settings(env)
                self.assertEqual((s["policy"], s["delegate"] is None), (X.policy(arm), X.policy(arm) == "F"))
                envs[arm] = env
                accepted += 1
            self.assertEqual(accepted, len(X.ARMS) + len(X.ARM_BASE))
            bad = {"S2B with a CAM-REF rule": dict(envs["S3"], **CAPTURE_ON),
                   "S2B with a yaw clip": dict(envs["S6"], NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS="1.047"),
                   "S2B recorder inside cams2": dict(envs["S0ID"], NAVRL_CAMREF_RECORD="1",
                                                     NAVRL_CAMREF_RECORD_JSONL=str(pathlib.Path(d) / "x.jsonl")),
                   "S2B without camsafe": {k: v for k, v in envs["S6"].items() if not k.startswith("NAVRL_CAMSAFE")},
                   "S2B at 624": dict(envs["S3"], NAVRL_CAMERA_WIDTH="624"),
                   "S2B under K1's checkpoint": dict(envs["S3"], NAVRL_EVAL_CHECKPOINT=E.POLICY_CKPT["K1"]),
                   "K1 with camsafe": dict(envs["K1T"], NAVRL_CAMSAFE="1"),
                   "K1 at 87 deg": dict(envs["K1T"], NAVRL_CAMTRAIN_HFOV_DEG="87.0"),
                   "K1 with the LiDAR": {k: v for k, v in envs["K1T"].items() if k != "NAVRL_SENSOR_ABLATION"},
                   "K1 without the delegate's receipt": {k: v for k, v in envs["K1S"].items()
                                                         if k != "NAVRL_REALVEH_RECEIPT"},
                   "F with no_lidar": dict(envs["F3"], NAVRL_SENSOR_ABLATION="no_lidar")}
            for name, env in bad.items():
                with self.subTest(name), self.assertRaises(SystemExit):
                    E.settings(env)


class EntryChildProcess(unittest.TestCase):
    """tools/camref_entry.py run for real in a child process, from a copy of the tree's entry files with stub isaacgym /
    aerial_gym packages (the real ones need Isaac Gym). All switches off: nothing is installed and the stub task class is
    unchanged when runner.py runs; with the F3 switches the rules, the recorder and the visible start are installed."""

    TASK = ("import builtins\nclass NavRLTask:\n"
            "    def transform_action_to_command(self, a):\n        return a\n"
            "    def reset_idx(self, ids):\n        return None\n"
            "    def compute_state_reward_and_terminations(self):\n        return None\n"
            "    def _align_general_spawn_yaw_to_target(self, ids, s, g):\n        return None\n"
            "    def _process_obs_perception(self):\n        return None\n"
            "ORIGINAL = {k: v for k, v in NavRLTask.__dict__.items() if callable(v)}\n")
    CFG = ("class task_config:\n    robot_name = 'navrl_quad'\n    success_radius = 0.5\n    flight_altitude = 1.0\n"
           "    class vision:\n        camera_translation = [0.10, 0.0, 0.03]\n        detector_hfov_deg = 87.0\n")
    RUNNER = ("import json, sys\nfrom aerial_gym.task.navrl_task import navrl_task as t\n"
              "changed = sorted(k for k, v in t.NavRLTask.__dict__.items() if callable(v) and t.ORIGINAL.get(k) is not v)\n"
              "print('RUNNER ' + json.dumps({'argv': sys.argv, 'changed': changed}))\n")

    def _tree(self, root):
        root = pathlib.Path(root)
        for rel in ("tools/camref_entry.py", "tools/camref_python", "tools/realveh_entry.py", "tools/camtrain_entry.py",
                    "aerial_gym/task/navrl_task/navrl_camref_rules.py", "aerial_gym/task/navrl_task/navrl_inview_start.py",
                    "aerial_gym/task/navrl_task/navrl_yawclip.py", "aerial_gym/task/navrl_task/navrl_vehicle_rand.py"):
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(ROOT / rel, root / rel)
        for rel in E.HASHED.values():                        # the entry hashes these; stubs suffice here
            p = root / pathlib.Path(rel).relative_to(ROOT)
            if not p.exists():
                p.parent.mkdir(parents=True, exist_ok=True)
                p.write_text("# stub\n")
        stubs = {"isaacgym/__init__.py": "", "aerial_gym/__init__.py": "", "aerial_gym/rl_training/__init__.py": "",
                 "aerial_gym/rl_training/rl_games/__init__.py": "",
                 "aerial_gym/rl_training/rl_games/navrl_import_origin.py": "def assert_runtime_origin():\n    return {}\n",
                 "aerial_gym/rl_training/rl_games/quiet_rl_io.py":
                     "import contextlib\ndef quiet_startup_enabled():\n    return False\n"
                     "@contextlib.contextmanager\ndef devnull_stdio():\n    yield\n",
                 "aerial_gym/config/__init__.py": "", "aerial_gym/config/task_config/__init__.py": "",
                 "aerial_gym/config/task_config/navrl_task_config.py": self.CFG,
                 "aerial_gym/task/__init__.py": "", "aerial_gym/task/navrl_task/__init__.py": "",
                 "aerial_gym/task/navrl_task/navrl_task.py": self.TASK, "rl/runner.py": self.RUNNER}
        for rel, text in stubs.items():
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            (root / rel).write_text(text)
        return root

    def _run(self, root, extra):
        env = {"PATH": "/usr/bin:/bin", "PYTHONPATH": str(root), "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1",
               "CUDA_VISIBLE_DEVICES": "", "CAMREF_REAL_PYTHON": sys.executable}
        env.update(_f_env(NAVRL_CAMREF_RECEIPT=str(pathlib.Path(root) / "camref_receipt.json")))
        env.update(extra)
        return subprocess.run(["bash", str(pathlib.Path(root) / "tools/camref_python"), str(pathlib.Path(root) / "rl/runner.py"),
                               "--play"], env=env, capture_output=True, text=True, timeout=300)

    def _got(self, r):
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        line = [x for x in r.stdout.splitlines() if x.startswith("RUNNER ")][0]
        return json.loads(line[len("RUNNER "):])

    def test_all_off_installs_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            root = self._tree(d)
            got = self._got(self._run(root, {}))
            self.assertEqual(got["changed"], [])
            self.assertEqual(got["argv"], [str(root / "rl/runner.py"), "--play"])
            rec = json.loads((root / "camref_receipt.json").read_text())
            self.assertEqual((rec["policy"], rec["installed"], rec["delegate"]), ("F", [], None))
            again = self._run(root, {})                       # the receipt is never overwritten
            self.assertNotEqual(again.returncode, 0)

    def test_f3_switches_install_rules_recorder_and_start(self):
        """Positive control of the child-process probe: the same stub class is wrapped when the switches are on."""
        with tempfile.TemporaryDirectory() as d:
            root = self._tree(d)
            extra = dict(CAPTURE_ON, NAVRL_CAMREF_RECORD="1", NAVRL_CAMREF_RECORD_JSONL=str(root / "rec.jsonl"),
                         NAVRL_INVIEW_START="1", NAVRL_INVIEW_START_HALF_DEG="28.35", NAVRL_INVIEW_START_SEED="4501",
                         NAVRL_INVIEW_DIAG="1", NAVRL_INVIEW_DIAG_JSONL=str(root / "inview.jsonl"))
            got = self._got(self._run(root, extra))
            self.assertEqual(got["changed"], ["_align_general_spawn_yaw_to_target", "_process_obs_perception",
                                              "compute_state_reward_and_terminations", "reset_idx",
                                              "transform_action_to_command"])
            rec = json.loads((root / "camref_receipt.json").read_text())
            self.assertEqual(rec["installed"], X.expected_installed("F3"))
            self.assertEqual(rec["camref_rules"]["config_after"]["success_radius"], 0.4)


class IsolationNote(unittest.TestCase):
    def test_entry_is_the_only_new_importer_of_vehicle_rand(self):
        """camref_entry installs navrl_vehicle_rand for Policy F (realveh_entry refuses the LiDAR). It imports neither the
        depth model nor the sensor ablation: K1 and S2b-FT reach those through realveh_entry, unchanged."""
        text = (TOOLS / "camref_entry.py").read_text()
        self.assertIn("from aerial_gym.task.navrl_task import navrl_vehicle_rand as veh", text)    # positive control
        for name in ("navrl_depth_model", "navrl_sensor_ablation", "navrl_depth_validity", "navrl_camsafe", "navrl_cams2 "):
            self.assertNotIn("import " + name, text)
        self.assertNotIn("navrl_task import navrl_cams2", text)


# ================================================================ the runner
class RunnerEnvironments(unittest.TestCase):
    def test_rung_protocol_matched_points(self):
        p = X.RUNG_PROTOCOL
        task = ("start", "motion", "capture", "target", "delay", "veh_rand", "gain_rand", "altitude_mount")
        for point, members in (("P_T", ("F4", "K1T", "S3")), ("P_S", ("F8", "K1S", "S0"))):
            for k in task:
                self.assertEqual(len({str(p[m][k]) for m in members}), 1, (point, k))
        for k in task:
            self.assertEqual(p["F0"][k], p["K1PK"][k], k)
            if k != "target":
                self.assertEqual(p["F0"][k], p["S6"][k], k)
        self.assertNotEqual(p["F0"]["target"], p["S6"]["target"])      # the stated residual (D8 at S6)
        self.assertNotEqual(p["F3"]["target"], p["F4"]["target"])
        self.assertEqual(len({p[r]["sensor"] for r in p}), 3)

    @needs_ft
    def test_every_planned_cell_matches_its_protocol_and_env_problems_bite(self):
        te = json.loads(X.D.FT_TRAIN_RECEIPT.read_text())["env_navrl"]
        cap, checked = 10 ** 9, 0
        for arm, seed in [(a, s) for a, _, s in X.SMOKE_PLAN + X.IDENTITY_PLAN] + [(a, s) for a, _, s in X.eval_plan()]:
            cd = pathlib.Path("/nonexistent/out") / X.cell_id(arm, 205, seed)
            env = X.cell_navrl_env(arm, seed, cd, 8, cap, te, "sha")
            self.assertEqual(X.env_problems(env, arm, seed, cd, 8, cap, te, "sha"), [], arm)
            checked += 1
        self.assertEqual(checked, 3 + 3 + 21)
        cd = pathlib.Path("/nonexistent/out/c")
        f4 = X.cell_navrl_env("F4", 4501, cd, 8, cap, te, "sha")
        bad = {"stray key": dict(f4, NAVRL_GAIN_RAND="1"),
               "no capture rule": {k: v for k, v in f4.items() if not k.startswith("NAVRL_CAMREF_CAPTURE")},
               "preset launcher key": dict(f4, NAVRL_SEED="4501"),
               "camsafe on F": dict(f4, NAVRL_CAMSAFE="1")}
        for name, env in bad.items():
            with self.subTest(name):
                self.assertTrue(X.env_problems(env, "F4", 4501, cd, 8, cap, te, "sha"))
        s3 = X.cell_navrl_env("S3", 4501, cd, 8, cap, te, "sha")
        self.assertTrue(X.env_problems(dict(s3, NAVRL_CAMS2_FLIGHT_ALTITUDE_M="1.5"), "S3", 4501, cd, 8, cap, te, "sha"))
        # the protocol table catches a change the construction would also make (independent derivation)
        self.assertTrue(X.protocol_problems(dict(s3, NAVRL_CAMS2_DELAY_MAX="2"), "S3"))
        self.assertEqual(X.protocol_problems(s3, "S3"), [])

    @needs_ft
    @needs_refs
    def test_base_environments_equal_the_recorded_reference_snapshots(self):
        te = json.loads(X.D.FT_TRAIN_RECEIPT.read_text())["env_navrl"]
        self.assertEqual(X.base_provenance_problems(te, 10 ** 9), [])
        self.assertEqual(X.reference_problems(), [])
        saved = dict(X.F_BASE_ENV)
        try:
            X.F_BASE_ENV["NAVRL_YAW_RATE_MAX"] = "1.0472"   # negative control: caught per seed against GR-0 and CAM-K1-E
            self.assertEqual(len(X.base_provenance_problems(te, 10 ** 9)), 6)
        finally:
            X.F_BASE_ENV.clear()
            X.F_BASE_ENV.update(saved)

    def test_launcher_pin_and_marks(self):
        text = (ROOT / X.LAUNCHER).read_text()
        self.assertEqual(hashlib.sha256(text.encode()).hexdigest(), X.LAUNCHER_SHA)
        self.assertEqual(sum(1 for line in text.splitlines() if "[camref]" in line and not line.startswith("#")) > 5, True)
        self.assertNotIn("CAMS2A_", text)
        r = subprocess.run(["bash", "-n", str(ROOT / X.LAUNCHER)], capture_output=True)
        self.assertEqual(r.returncode, 0)


class RunnerPreflightPieces(unittest.TestCase):
    def _report(self, d, name, verdict, commit, wall=600.0):
        p = pathlib.Path(d) / name.split(".")[0] / name
        p.parent.mkdir(parents=True)
        p.write_text(json.dumps({"verdict": verdict, "commit": commit, "wall_s": wall}))

    def test_stage_links_and_wall_budget(self):
        with tempfile.TemporaryDirectory() as d:
            out = pathlib.Path(d) / "identity_01"
            p, _ = X.stage_link_problems("identity", out, "abc")
            self.assertTrue(p)                                           # no smoke report
            self._report(d, "smoke_report.json", "SMOKE_OK", "abc", wall=1800.0)
            p, found = X.stage_link_problems("identity", out, "abc")
            self.assertEqual(p, [])                                      # positive control
            self.assertEqual(X.stage_budget_h("identity", found)[0], 0.75)
            p, _ = X.stage_link_problems("identity", out, "other")
            self.assertTrue(p)                                           # another commit
            p, _ = X.stage_link_problems("eval", pathlib.Path(d) / "eval_01", "abc")
            self.assertTrue(any("identity_report.json" in x for x in p))
            self._report(d, "identity_report.json", "IDENTITY_INCOMPLETE", "abc", wall=1800.0)
            p, _ = X.stage_link_problems("eval", pathlib.Path(d) / "eval_01", "abc")
            self.assertTrue(p)                                           # an incomplete identity stage links nothing
            (pathlib.Path(d) / "identity_report" / "identity_report.json").write_text(
                json.dumps({"verdict": "IDENTITY_PASS", "commit": "abc", "wall_s": 1800.0}))
            p, found = X.stage_link_problems("eval", pathlib.Path(d) / "eval_01", "abc")
            self.assertEqual(p, [])
            self.assertAlmostEqual(X.stage_budget_h("eval", found)[0], 1.5)        # 2.5 h - 0.5 h - 0.5 h
            self._report(d, "eval_run_report.json", "COMPLETE", "abc")
            p, _ = X.stage_link_problems("eval", pathlib.Path(d) / "eval_02", "abc")
            self.assertTrue(any("already exists" in x for x in p))
        with tempfile.TemporaryDirectory() as d:
            self._report(d, "smoke_report.json", "SMOKE_OK", "abc", wall=9100.0)
            _, found = X.stage_link_problems("identity", pathlib.Path(d) / "identity_01", "abc")
            self.assertEqual(X.stage_budget_h("identity", found)[0], 0.0)     # the 2.5 h cap is spent

    def test_cells_that_cannot_finish_are_skipped(self):
        """fits_budget: a cell runs only if its projection fits the remaining cap; unknown projections run."""
        prior = [{"records": [{"policy": "S2B", "verdict": "OK", "quota": 8, "wall_s": 400.0},
                              {"policy": "F", "verdict": "OK", "quota": 8, "wall_s": 200.0}]}]
        self.assertEqual(X.fits_budget("S3", 8, [], prior, 700.0), (True, 600.0))
        self.assertEqual(X.fits_budget("S3", 8, [], prior, 500.0), (False, 600.0))
        self.assertEqual(X.fits_budget("F8", 8, [], prior, 500.0), (True, 300.0))     # a faster policy still fits
        self.assertEqual(X.fits_budget("K1T", 8, [], prior, 10.0), (True, None))       # nothing known: runs
        own = [{"policy": "S2B", "verdict": "OK", "quota": 8, "wall_s": 450.0}]
        self.assertEqual(X.fits_budget("S6", 8, own, prior, 449.0), (False, 450.0))
        recs = [{"arm": a, "density": d, "seed": s, "verdict": "OK"} for a, d, s in X.eval_plan()]
        self.assertEqual(X.eval_verdict(recs), "COMPLETE")
        self.assertEqual(X.eval_verdict(recs + [{"arm": "S6", "verdict": "BUDGET_SKIPPED"}]), "INCOMPLETE")
        smoke = [{"arm": a, "verdict": "OK", "stats": {"finished_episodes": 128}} for a, _, _ in X.SMOKE_PLAN[:2]]
        ctl = {a: {"x": True} for a, _, _ in X.SMOKE_PLAN}
        self.assertEqual(X.smoke_verdict(smoke + [{"arm": "S6_smoke", "verdict": "BUDGET_SKIPPED"}], ctl),
                         "SMOKE_INCOMPLETE")
        self.assertEqual(X.identity_verdict({"F0ID": "IDENTITY_PASS", "K1ID": "IDENTITY_PASS"}, ["S0ID"]),
                         "IDENTITY_INCOMPLETE")

    def test_code_allowlist(self):
        self.assertEqual(X.allowlist_problems(["WORKLOG.md", "tools/camref_entry.py", X.RULES_REL, X.LAUNCHER,
                                               "tests/test_camref.py", "docs/plans/x.md"], X.CODE_ALLOWLIST, "335a1fa"), [])
        bad = ["aerial_gym/task/navrl_task/navrl_task.py", "tools/cams2a_eval_run.py",
               "aerial_gym/rl_training/rl_games/play_navrl.sh"]
        self.assertEqual(len(X.allowlist_problems(bad, X.CODE_ALLOWLIST, "335a1fa")), 3)
        self.assertTrue(X.allowlist_problems(None, X.CODE_ALLOWLIST, "335a1fa"))

    def test_reference_pins_bite(self):
        with tempfile.TemporaryDirectory() as d:
            f = pathlib.Path(d) / "ref_4501"
            f.mkdir()
            (f / "episodes.jsonl").write_text("{}\n")
            saved_dirs, saved_sha = dict(X.REF_DIRS), dict(X.REF_SHA)
            try:
                X.REF_DIRS.clear()
                X.REF_DIRS["T"] = pathlib.Path(d) / "ref_%d"
                X.REF_SHA.clear()
                X.REF_SHA[("T", 4501)] = {"episodes.jsonl": hashlib.sha256(b"{}\n").hexdigest()}
                self.assertEqual(X.reference_problems(), [])
                (f / "episodes.jsonl").write_text("{} \n")
                self.assertEqual(len(X.reference_problems()), 1)
            finally:
                X.REF_DIRS.clear()
                X.REF_DIRS.update(saved_dirs)
                X.REF_SHA.clear()
                X.REF_SHA.update(saved_sha)

    def test_approval_scope_digest_and_format(self):
        good = {"approved_by": "user", "approved_at": "2026-11-26", "scope": X.SCOPE, "commit": "abcdef0"}
        self.assertEqual(X.R1.approval_problems(good, "abcdef0123", scope=X.SCOPE), [])
        self.assertTrue(X.R1.approval_problems(dict(good, scope="CAM-DX1"), "abcdef0123", scope=X.SCOPE))
        self.assertTrue(X.R1.approval_problems(dict(good, commit="1234567"), "abcdef0123", scope=X.SCOPE))
        self.assertEqual(X.scope_digest(), hashlib.sha256(X.SCOPE.encode()).hexdigest())
        self.assertIn("2.5 h", X.SCOPE)
        self.assertEqual(X.MAX_HOURS_TOTAL, 2.5)


class RunnerVerdicts(unittest.TestCase):
    @staticmethod
    def _rows(outcomes, n_env=X.NUM_ENVS, per_env=8):
        return [{"env_index": e, "env_episode_index": k, "outcome": outcomes(e, k), "cell_id": "c", "episode_id": "c/%d/%d" % (e, k),
                 "in_quota": True} for e in range(n_env) for k in range(per_env)]

    def test_identity_compare_and_verdicts(self):
        ref = self._rows(lambda e, k: "crash" if (e + k) % 7 == 0 else "capture", per_env=16)
        new = self._rows(lambda e, k: "crash" if (e + k) % 7 == 0 else "capture")
        c = X.identity_compare(new, ref)
        self.assertEqual((c["episodes_new"], c["outcomes_equal"], c["outcomes_differ"]), (1024, True, 0))
        self.assertGreater(c["counts_new"]["crash"], 0)
        ok = {"verdict": "OK"}
        self.assertEqual(X.identity_cell_verdict("F0ID", ok, c), "IDENTITY_PASS")
        flipped = [dict(r, outcome="timeout") if (r["env_index"], r["env_episode_index"]) == (3, 2) else r for r in new]
        d = X.identity_compare(flipped, ref)
        self.assertEqual((d["outcomes_equal"], d["outcomes_differ"]), (False, 1))
        for arm in ("F0ID", "K1ID", "S0ID"):                                    # any difference fails, every cell
            self.assertEqual(X.identity_cell_verdict(arm, ok, d), "IDENTITY_FAIL")
            self.assertEqual(X.identity_cell_verdict(arm, ok, c), "IDENTITY_PASS")
        short = [r for r in new if not (r["env_index"] == 5 and r["env_episode_index"] == 7)]
        self.assertEqual(X.identity_cell_verdict("F0ID", ok, X.identity_compare(short, ref)), "IDENTITY_FAIL")
        self.assertEqual(X.identity_cell_verdict("F0ID", {"verdict": "FAIL"}, c), "IDENTITY_FAIL")
        allp = {"F0ID": "IDENTITY_PASS", "K1ID": "IDENTITY_PASS", "S0ID": "IDENTITY_PASS"}
        self.assertEqual(X.identity_verdict(allp), "IDENTITY_PASS")
        self.assertEqual(X.identity_verdict({"F0ID": "IDENTITY_PASS", "S0ID": "IDENTITY_PASS"}), "IDENTITY_FAIL")
        self.assertEqual(X.identity_verdict(dict(allp, K1ID="IDENTITY_FAIL")), "IDENTITY_FAIL")
        self.assertEqual(X.identity_verdict({"F0ID": "IDENTITY_FAIL"}), "IDENTITY_FAIL")         # stopped after F0ID

    @needs_refs
    def test_identity_on_recorded_cells(self):
        """A reference against itself passes with 1,024 compared episodes; GR-0 against DX1 (another policy) does not."""
        gr = X.K.read_jsonl(X.reference_dir("GR0_S00", 4501) / "episodes.jsonl")
        dx = X.K.read_jsonl(X.reference_dir("DX1_V312", 4501) / "episodes.jsonl")
        c = X.identity_compare(gr, gr)
        self.assertEqual((c["episodes_new"], c["outcomes_equal"]), (1024, True))
        self.assertTrue(X.identity_compare(dx, dx)["outcomes_equal"])
        k1 = X.K.read_jsonl(X.reference_dir("K1E_K1s1", 4501) / "episodes.jsonl")
        self.assertTrue(X.identity_compare(k1, k1)["outcomes_equal"])
        self.assertFalse(X.identity_compare(k1, gr)["outcomes_equal"])
        self.assertFalse(X.identity_compare(dx, gr)["outcomes_equal"])
        self.assertGreater(X.identity_compare(dx, gr)["outcomes_differ"], 0)

    def test_cell_eval_and_smoke_verdicts(self):
        self.assertEqual(X.cell_verdict([]), "OK")
        self.assertEqual(X.cell_verdict(["x"]), "FAIL")
        self.assertEqual(X.cell_verdict(["x", "HOST fault_count changed"]), "VOID_HOST_FAULT")
        recs = [{"arm": a, "density": d, "seed": s, "verdict": "OK"} for a, d, s in X.eval_plan()]
        self.assertEqual(len(recs), 21)
        self.assertEqual(X.eval_verdict(recs), "COMPLETE")
        self.assertEqual(X.eval_verdict(recs[:-1]), "INCOMPLETE")
        self.assertEqual(X.eval_verdict(recs[:-1] + [dict(recs[-1], verdict="FAIL")]), "INCOMPLETE")
        smoke = [{"arm": a, "verdict": "OK", "stats": {"finished_episodes": 128}} for a, _, _ in X.SMOKE_PLAN]
        ctl = {a: {"x": True} for a, _, _ in X.SMOKE_PLAN}
        self.assertEqual(X.smoke_verdict(smoke, ctl), "SMOKE_OK")
        self.assertEqual(X.smoke_verdict(smoke, dict(ctl, S6_smoke={"x": False})), "SMOKE_FAIL")
        self.assertEqual(X.smoke_verdict(smoke[:2], ctl), "SMOKE_FAIL")
        self.assertEqual(X.smoke_verdict([dict(smoke[0], verdict="VOID_HOST_FAULT")] + smoke[1:], ctl), "VOID_HOST_FAULT")

    def test_expected_installs_and_mismatch_allowlists(self):
        self.assertEqual(X.expected_installed("F0ID"), ["camref_record"])
        self.assertEqual(X.expected_installed("F8"), ["vehicle_rand", "camref_rules", "camref_record", "yawclip",
                                                      "inview_start", "inview_diag"])
        self.assertEqual(X.expected_installed("K1S"), ["camref_rules", "camref_record"])
        self.assertEqual(X.expected_installed("K1ID"), ["camref_record"])
        self.assertEqual((X.expected_installed("S0ID"), X.expected_installed("S6")), ([], ["camref_record"]))
        e2 = ("WARNING : NavRL TARGET BEHAVIOR MISMATCH | checkpoint=historical running=e2_obstacle_aware. This is ...")
        v = "WARNING : NavRL TASK-DISTRIBUTION MISMATCH | NAVRL_GENERAL_GOAL_DIST_MAX: checkpoint used 28.000, running with 12.000."
        self.assertEqual(X.mismatch_problems(e2, "F0ID"), [])
        self.assertEqual(len(X.mismatch_problems(e2, "F3")), 1)               # F at V312 runs the legacy motion
        self.assertEqual(X.mismatch_problems(v, "F3"), [])
        self.assertEqual(len(X.mismatch_problems(v, "S6")), 1)
        self.assertEqual(len(X.mismatch_problems("NavRL ACTION STD MISMATCH | checkpoint=a running=b", "S0ID")), 1)


# ================================================================ the analysis
class AnalysisPieces(unittest.TestCase):
    def test_labels_wilson_and_bar_labels(self):
        self.assertEqual(A.label(8.0, (5.7, 10.4)), "MATERIAL")
        self.assertEqual(A.label(-8.0, (-10.4, -5.7)), "MATERIAL")
        self.assertEqual(A.label(2.0, (0.5, 3.5)), "SMALL")
        self.assertEqual(A.label(6.0, (-0.5, 12.0)), "NO_DETECTED_CHANGE")
        lo, hi = A.wilson(553, 3072)
        self.assertLess(lo, 553 / 3072)
        self.assertGreater(hi, 553 / 3072)
        self.assertEqual(A.bar_label(553, 3072), "BELOW_20")                  # 18.0 %
        self.assertEqual(A.bar_label(707, 3072), "ABOVE_20")                  # 23.0 %
        self.assertEqual(A.bar_label(614, 3072), "UNRESOLVED")                # 20.0 %
        self.assertEqual(A.bar_label(0, 0), "NOT_RUN")

    def test_counts_with_classes_and_refusal(self):
        ledger = [{"env_index": e, "env_episode_index": k, "outcome": "crash" if k == 0 else "capture"}
                  for e in range(4) for k in range(10)]
        classes = [{"env_index": e, "env_episode_index": 0, "outcome": "crash",
                    "crash_class": ("contact", "target_contact_swept", "out_of_bounds", "below")[e]} for e in range(4)]
        c = A.counts(ledger, classes, n_first=8)
        self.assertEqual((c["n"], c["crash"], c["crash_bar"], c["crash_target"], c["crash_oob"], c["crash_other"]),
                         (32, 4, 1, 1, 1, 1))
        self.assertIsNone(A.rate(A.counts(ledger, None, n_first=8), "crash_bar"))      # NOT_RECORDED without a recorder
        with self.assertRaises(ValueError):
            A.counts(ledger, classes[:3], n_first=8)                                  # a crash without a recorder row

    def test_bootstrap_is_deterministic_and_effects_use_complete_blocks(self):
        cells = A.synthetic_cells(1024, -8.0, 8.0)
        r1, r2 = A.readings(cells), A.readings(cells)
        self.assertEqual(r1["R1"]["S0-S6"]["crash"]["ci95_pp"], r2["R1"]["S0-S6"]["crash"]["ci95_pp"])
        self.assertEqual(r1["R1"]["S0-S6"]["crash"]["label"], "MATERIAL")
        self.assertEqual(r1["R2"]["P_S"]["F-S2B"], "NOT_RUN")                         # no F8 cell here
        del cells["S6"][4503]
        self.assertEqual(A.readings(cells)["R1"]["S0-S6"]["blocks"], [4501, 4502])
        cells["S6"] = {}
        self.assertEqual(A.readings(cells)["R1"]["S0-S6"], "NOT_RUN")

    def test_main_reads_nothing_without_a_complete_seed_block(self):
        """Review r1 P1-2: no complete block -> NOT_RUN (no fallback to all seeds); one complete block -> READ."""
        cells = A.synthetic_cells(1024, -8.0, 8.0)
        orig = (A.load_eval, A.controls)
        try:
            A.controls = lambda: (True, {})
            with tempfile.TemporaryDirectory() as d:
                out = pathlib.Path(d) / "a.json"
                A.load_eval = lambda i, e: ({"ok": True, "complete_blocks": []}, cells)
                self.assertEqual(A.main(["eval", "i", "e", str(out)]), 1)
                self.assertEqual(json.loads(out.read_text())["verdict"], "NOT_RUN")
                A.load_eval = lambda i, e: ({"ok": True, "complete_blocks": [4501]}, cells)
                self.assertEqual(A.main(["eval", "i", "e", str(out)]), 0)
                rep = json.loads(out.read_text())
                self.assertEqual(rep["verdict"], "READ")
                self.assertEqual(rep["R1"]["S0-S6"]["blocks"], [4501])
        finally:
            A.load_eval, A.controls = orig

    def test_r3_reads_pooled_seeds(self):
        cells = {"F8": {s: A.synthetic_counts(1024, 0.6, 0.18, bar_share=0.5) for s in A.SEEDS}}
        r = A.r3_reading(cells, "F8", A.SEEDS)
        self.assertEqual(r["n"], 3072)
        self.assertEqual(r["all_crashes"]["label"], "BELOW_20")
        self.assertEqual(r["bar_contact_crashes"]["label"], "BELOW_20")
        cells = {"F8": {s: A.synthetic_counts(1024, 0.5, 0.40, bar_share=0.6) for s in A.SEEDS}}
        r = A.r3_reading(cells, "F8", A.SEEDS)
        self.assertEqual((r["all_crashes"]["label"], r["bar_contact_crashes"]["label"]), ("ABOVE_20", "ABOVE_20"))
        self.assertEqual(A.r3_reading({"F8": {s: A.synthetic_counts(1024, 0.6, 0.3, split=False) for s in A.SEEDS}},
                                      "F8", A.SEEDS)["bar_contact_crashes"], "NOT_RECORDED")

    def test_use_of_result_pointers(self):
        r = {"R3": {"F8": {"bar_contact_crashes": {"label": "BELOW_20"}}, "S0": {"bar_contact_crashes": {"label": "ABOVE_20"}}},
             "R1": {"S0-S6": {"crash": {"label": "MATERIAL"}}}}
        out = A.use_of_result(r)
        self.assertEqual(len(out), 2)
        r["R3"]["F8"]["bar_contact_crashes"]["label"] = "ABOVE_20"
        self.assertTrue(any(x.startswith("R3(ii) ABOVE_20") for x in A.use_of_result(r)))
        r["R3"]["F8"]["bar_contact_crashes"]["label"] = "UNRESOLVED"
        r["R1"]["S0-S6"]["crash"]["label"] = "SMALL"
        self.assertTrue(A.use_of_result(r)[0].startswith("no preregistered pointer"))

    def test_preregistered_controls(self):
        ok, res = A.controls()
        names = {"synthetic_identical", "synthetic_plus8pp", "synthetic_18pct_n3072", "synthetic_23pct_n3072",
                 "dx1_r312_minus_v312_205"}
        self.assertEqual(set(res), names)
        for name in names - {"dx1_r312_minus_v312_205"}:
            self.assertTrue(res[name]["ok"], name)
        if HAVE_REFS and (pathlib.Path(str(A.DX1_R312) % 4501) / "episodes.jsonl").is_file():
            self.assertTrue(ok, res["dx1_r312_minus_v312_205"])
            self.assertGreater(res["dx1_r312_minus_v312_205"]["crash"]["mean_pp"], 10.0)


if __name__ == "__main__":
    unittest.main()
