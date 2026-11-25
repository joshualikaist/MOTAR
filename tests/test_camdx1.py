"""CPU tests of CAM-DX1 (docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md): the evaluation-only module
navrl_inview_start.py (visible start and recorder) over fake task classes, its additive hook in tools/realveh_entry.py (OFF
settings identical to the entry at S2b-FT's training commit 335a1fa, the module not loaded), the runner
tools/camdx1_eval_run.py (cell environments of the four arms accepted, foreign or missing keys refused; receipt, snapshot,
recorder and telemetry checks on faithful and deviating synthetic cells; smoke controls; verdicts; memory fallback) and
the analysis tools/camdx1_eval_analyse.py (the preregistered controls, labels, per-cell measures, an end-to-end run on
synthetic cells and a binding failure). Every refusal has a positive control; counts are asserted non-zero. Tests that
need files absent on this host are skipped with a reason (a skip is never a pass).
Run from tests/: python -B -m unittest test_camdx1"""
import contextlib
import hashlib
import importlib.util
import io
import json
import math
import pathlib
import subprocess
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


def _load_tool(name):
    """tools/<name>.py under its production module name (the analysis imports the runner), tools/ importable."""
    if str(ROOT / "tools") not in sys.path:
        sys.path.insert(0, str(ROOT / "tools"))
    spec = importlib.util.spec_from_file_location(name, str(ROOT / "tools" / (name + ".py")))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


M = _load("camdx1_test_inview", ROOT / "aerial_gym/task/navrl_task/navrl_inview_start.py")
E = _load("camdx1_test_realveh_entry", ROOT / "tools/realveh_entry.py")
X = _load_tool("camdx1_eval_run")
AN = _load_tool("camdx1_eval_analyse")
S2A, W = X.S2A, X.W
HAVE_FT = X.FT_TRAIN_RECEIPT.is_file() and X.FT_TRAIN_REPORT.is_file() and W.S2A_TRAIN_RECEIPT.is_file()
FT_RECEIPT = json.loads(X.FT_TRAIN_RECEIPT.read_text()) if HAVE_FT else None
FT_ENV = FT_RECEIPT["env_navrl"] if FT_RECEIPT else None
needs_ft = unittest.skipUnless(HAVE_FT, "S2b-FT training receipt/report %s are required (skipped, not passed)"
                               % X.FT_TRAIN_RECEIPT)
FT_TRAIN_COMMIT = X.FT_TRAIN_HEAD
FAKE_MOD = types.ModuleType("camdx1_fake_task_module")
FAKE_MOD._episode_limit_reached = lambda sim_steps, limit: sim_steps >= int(limit)
sys.modules[FAKE_MOD.__name__] = FAKE_MOD


# ================================================================ the module: settings and geometry
class Settings(unittest.TestCase):
    def test_start_off_on_and_refusals(self):
        self.assertIsNone(M.start_settings_from_env({}))
        self.assertIsNone(M.start_settings_from_env({"NAVRL_INVIEW_START": "0"}))
        on = {"NAVRL_INVIEW_START": "1", "NAVRL_INVIEW_START_HALF_DEG": "28.35", "NAVRL_INVIEW_START_SEED": "4501"}
        self.assertEqual(M.start_settings_from_env(on), {"NAVRL_INVIEW_START_HALF_DEG": 28.35, "NAVRL_INVIEW_START_SEED": 4501})
        bad = [dict(on, NAVRL_INVIEW_START_HALF_DEG=""), dict(on, NAVRL_INVIEW_START_HALF_DEG="abc"),
               dict(on, NAVRL_INVIEW_START_HALF_DEG="nan"), dict(on, NAVRL_INVIEW_START_HALF_DEG="0.1"),
               dict(on, NAVRL_INVIEW_START_HALF_DEG="95"), dict(on, NAVRL_INVIEW_START_SEED=""),
               dict(on, NAVRL_INVIEW_START_SEED="-1"), dict(on, NAVRL_INVIEW_START_SEED="45.1"),
               dict(on, NAVRL_INVIEW_START_SEED=str(2 ** 31)), dict(on, NAVRL_INVIEW_START="on"),
               {"NAVRL_INVIEW_START_HALF_DEG": "28.35"}, {"NAVRL_INVIEW_START": "0", "NAVRL_INVIEW_START_SEED": "1"}]
        for env in bad:
            with self.subTest(env=env), self.assertRaises(ValueError):
                M.start_settings_from_env(env)

    def test_diag_off_on_and_refusals(self):
        self.assertIsNone(M.diag_settings_from_env({}))
        with tempfile.TemporaryDirectory() as tmp:
            path = str(pathlib.Path(tmp) / "rows.jsonl")
            on = {"NAVRL_INVIEW_DIAG": "1", "NAVRL_INVIEW_DIAG_JSONL": path}
            self.assertEqual(M.diag_settings_from_env(on), {"NAVRL_INVIEW_DIAG_JSONL": path})
            bad = [{"NAVRL_INVIEW_DIAG": "1"}, dict(on, NAVRL_INVIEW_DIAG_JSONL="rel.jsonl"), {"NAVRL_INVIEW_DIAG_JSONL": path},
                   dict(on, NAVRL_INVIEW_DIAG="yes")]
            pathlib.Path(path).write_text("")
            bad.append(on)                                                            # the file exists
            for env in bad:
                with self.subTest(env=env), self.assertRaises(ValueError):
                    M.diag_settings_from_env(env)


class Geometry(unittest.TestCase):
    def test_quaternion_round_trip_and_bins(self):
        yaw = torch.linspace(-3.1, 3.1, 41, dtype=torch.float64)
        self.assertTrue(torch.allclose(M.yaw_from_quat_xyzw(M.quat_from_yaw(yaw)), yaw, atol=1e-12))
        self.assertEqual(M.dist_bin(torch.tensor([0.0, 1.99, 2.0, 11.9, 29.9, 30.0, 300.0])).tolist(),
                         [0, 0, 1, 5, 14, 15, 15])

    def test_inview_yaw_puts_the_target_at_the_drawn_bearing(self):
        g = torch.Generator().manual_seed(1)
        start = torch.rand((200, 3), generator=g, dtype=torch.float64) * 20
        goal = torch.rand((200, 3), generator=g, dtype=torch.float64) * 20
        u = torch.rand(200, generator=g, dtype=torch.float64)
        half = math.radians(28.35)
        yaw, rel = M.inview_yaw(start[:, 0:2], goal[:, 0:2], u, half)
        bearing, dxy, d3 = M.start_geometry(start, M.quat_from_yaw(yaw), goal)
        self.assertTrue(torch.allclose(bearing, rel, atol=1e-9))
        self.assertLessEqual(float(bearing.abs().max()), half + 1e-12)
        self.assertTrue(torch.allclose(dxy, (goal - start)[:, 0:2].norm(dim=1)))
        self.assertTrue(bool((d3 >= dxy).all()))


# ================================================================ the module: visible start over a fake task
def _start_task(saturated=True, general=True):
    class Task:
        general_spawn_mode = general
        vision_mode = True

        def __init__(self, n):
            self.obs_dict = {"robot_orientation": torch.zeros((n, 4))}
            self.obs_dict["robot_orientation"][:, 3] = 1.0
            yaw = -math.pi + 2 * math.pi * torch.rand(n)                      # _randomize_general_drone_spawn's draw
            self.obs_dict["robot_orientation"][:] = M.quat_from_yaw(yaw).float()

        def _align_general_spawn_yaw_to_target(self, env_ids, start_pos, goal):
            if saturated:                                                         # evaluation: a no-op
                return
            delta = goal[:, 0:2] - start_pos[:, 0:2]
            yaw = torch.atan2(delta[:, 1], delta[:, 0]) - (2.0 * torch.rand(len(env_ids)) - 1.0) * 0.2
            self.obs_dict["robot_orientation"][env_ids] = M.quat_from_yaw(yaw)

    return Task


class TaskConfig:
    class vision:
        detector_hfov_deg = 66.7


def _scene(n, seed=3):
    g = torch.Generator().manual_seed(seed)
    start = torch.rand((n, 3), generator=g) * 30
    ang = torch.rand(n, generator=g) * 2 * math.pi
    dist = 6 + 6 * torch.rand(n, generator=g)
    goal = start.clone()
    goal[:, 0] += dist * torch.cos(ang)
    goal[:, 1] += dist * torch.sin(ang)
    return start, goal


class VisibleStart(unittest.TestCase):
    S = {"NAVRL_INVIEW_START_HALF_DEG": 28.35, "NAVRL_INVIEW_START_SEED": 4501}

    def tearDown(self):
        M.uninstall()

    def _bearings(self, task, ids, start, goal):
        b, _, _ = M.start_geometry(start[ids].double(), task.obs_dict["robot_orientation"][ids].double(), goal[ids].double())
        return [math.degrees(x) for x in b.tolist()]

    def test_off_leaves_random_bearings_positive_control(self):
        Task = _start_task()
        n = 400
        start, goal = _scene(n)
        task = Task(n)
        ids = torch.arange(n)
        task._align_general_spawn_yaw_to_target(ids, start[ids], goal[ids])
        b = self._bearings(task, ids, start, goal)
        self.assertGreater(sum(1 for x in b if abs(x) > 33.35), 200)            # OFF: the target is mostly out of view

    def test_on_puts_every_bearing_inside_and_spreads_them(self):
        Task = _start_task()
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            info = M.install_start_task(Task, self.S, TaskConfig)
        self.assertEqual(info["wrapped"], ["_align_general_spawn_yaw_to_target"])
        n = 600
        start, goal = _scene(n)
        task = Task(n)
        before = task.obs_dict["robot_orientation"].clone()
        ids = torch.arange(0, n, 2)                                               # only half the envs reset
        rng_before = torch.get_rng_state()
        task._align_general_spawn_yaw_to_target(ids, start[ids], goal[ids])
        self.assertTrue(torch.equal(rng_before, torch.get_rng_state()))           # the global RNG is not touched
        b = self._bearings(task, ids, start, goal)
        self.assertEqual(len(b), 300)
        self.assertTrue(all(abs(x) <= 28.35 + 1e-3 for x in b))
        self.assertLess(min(b), -20.0)
        self.assertGreater(max(b), 20.0)
        others = torch.arange(1, n, 2)
        self.assertTrue(torch.equal(task.obs_dict["robot_orientation"][others], before[others]))
        self.assertEqual(Task._inview_start_stats["envs"], 300)
        line = S2A.parse_kv_line(err.getvalue(), "[inview]")
        self.assertEqual((line["envs"], line["half_deg"], line["seed"]), (300.0, 28.35, 4501.0))

    def test_overrides_the_unsaturated_curriculum_and_is_seeded(self):
        out = []
        for seed in (4501, 4501, 4502):
            Task = _start_task(saturated=False)
            with contextlib.redirect_stderr(io.StringIO()):
                M.install_start_task(Task, dict(self.S, NAVRL_INVIEW_START_SEED=seed), TaskConfig)
            start, goal = _scene(64)
            task = Task(64)
            ids = torch.arange(64)
            task._align_general_spawn_yaw_to_target(ids, start, goal)
            out.append(task.obs_dict["robot_orientation"].clone())
            b = self._bearings(task, ids, start, goal)
            self.assertGreater(max(abs(x) for x in b), math.degrees(0.2) + 1.0)   # not the curriculum's +-0.2 rad
            M.uninstall()
        self.assertTrue(torch.equal(out[0], out[1]))
        self.assertFalse(torch.equal(out[0], out[2]))

    def test_refusals(self):
        Task = _start_task()
        M.install_start_task(Task, self.S, TaskConfig)
        with self.assertRaises(RuntimeError):
            M.install_start_task(Task, self.S, TaskConfig)
        M.uninstall()
        with self.assertRaises(RuntimeError):
            M.install_start_task(_start_task(), dict(self.S, NAVRL_INVIEW_START_HALF_DEG=33.35), TaskConfig)
        Legacy = _start_task(general=False)
        M.install_start_task(Legacy, self.S, TaskConfig)
        start, goal = _scene(4)
        with self.assertRaises(RuntimeError):
            Legacy(4)._align_general_spawn_yaw_to_target(torch.arange(4), start, goal)


# ================================================================ the module: recorder over a fake task
def _diag_task(n, h=6, w=8):
    class Task:
        __module__ = FAKE_MOD.__name__

        class task_config:
            episode_len_steps = 50

        def __init__(self):
            self.num_envs = n
            self.obs_dict = {"robot_position": torch.zeros((n, 3)), "robot_orientation": torch.zeros((n, 4)),
                             "robot_vehicle_orientation": torch.zeros((n, 4)), "robot_linvel": torch.zeros((n, 3))}
            self.obs_dict["robot_orientation"][:, 3] = 1.0
            self.obs_dict["robot_vehicle_orientation"][:, 3] = 1.0
            self.renders = 0
            self.target_position = torch.zeros((n, 3))
            self.captured_now = torch.zeros(n, dtype=torch.bool)
            self.crashed_now = torch.zeros(n, dtype=torch.bool)
            self._visible_now = torch.zeros(n, dtype=torch.bool)
            self.detector = types.SimpleNamespace(width=w, height=h, detect_width=w, detect_height=h,
                                                  target_mask=torch.zeros((n, h, w), dtype=torch.int32))
            self.sim_env = types.SimpleNamespace(sim_steps=torch.zeros(n, dtype=torch.long))
            self.next_capture = torch.zeros(n, dtype=torch.bool)
            self.next_crash = torch.zeros(n, dtype=torch.bool)
            self.rewards = torch.zeros(n)

        def reset_idx(self, env_ids):
            return None

        def _process_obs_perception(self):
            self.renders += 1

        def transform_action_to_command(self, actions):
            return actions * 2.0

        def compute_state_reward_and_terminations(self):
            self.captured_now = self.next_capture.clone()
            self.crashed_now = self.next_crash.clone() & ~self.captured_now
            self.rewards[:] = 1.0

    return Task


class Recorder(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = pathlib.Path(self.tmp.name) / "rows.jsonl"

    def tearDown(self):
        M.uninstall()
        self.tmp.cleanup()

    def test_rows_align_and_record_start_detection_pixels_and_speed(self):
        Task = _diag_task(3)
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            info = M.install_diag_task(Task, {"NAVRL_INVIEW_DIAG_JSONL": str(self.path)})
        self.assertEqual(info["n_dist_bins"], 16)
        t = Task()
        # starts: env 0 target 10 m ahead (bearing 0), env 1 target 7 m to the left (+90 deg), env 2 25 m behind
        t.target_position[:] = torch.tensor([[10.0, 0.0, 0.0], [0.0, 7.0, 0.0], [-25.0, 0.0, 0.0]])
        # the RENDERED pose of env 1 faces +90 deg (the reset-time tensor does not): the step-0 check must follow the
        # rendered pose (review r1 P1-1), so env 1 reads 90 deg at reset and 0 deg as rendered
        t.obs_dict["robot_vehicle_orientation"][1] = M.quat_from_yaw(torch.tensor([math.pi / 2]))[0]
        t.reset_idx(torch.arange(3))
        t._process_obs_perception()                                               # the first observation
        actions = torch.tensor([[0.5, 0.0, 0.0, 0.0]] * 3)
        for step in range(3):
            t._visible_now[:] = torch.tensor([True, step >= 1, False])
            t.detector.target_mask.zero_()
            t.detector.target_mask[0, 0, 0:4] = 1                                  # env 0: 4 pixels
            t.detector.target_mask[1, 0:2, 0:3] = 1                                # env 1: 6 pixels
            t.detector.target_mask[2, 0, 0] = 1                                    # env 2: 1 pixel, never detected
            t.target_position[0, 0] = 13.0                                         # the target advances after the render
            cmd = t.transform_action_to_command(actions)
            t.target_position[0, 0] = 10.0
            self.assertTrue(torch.equal(cmd, actions * 2.0))                       # passive
            t.obs_dict["robot_linvel"][:, 0] = 1.5
            t.next_capture[:] = torch.tensor([step == 2, False, False])
            t.next_crash[:] = torch.tensor([False, step == 1, False])
            t.sim_env.sim_steps += 1
            t.compute_state_reward_and_terminations()
            self.assertTrue(torch.equal(t.rewards, torch.ones(3)))
            fin = t.captured_now | t.crashed_now
            if bool(fin.any()):
                t.reset_idx(torch.nonzero(fin).flatten())
            t._process_obs_perception()
        rows = [json.loads(x) for x in self.path.read_text().splitlines()]
        self.assertEqual([(r["env_index"], r["outcome"]) for r in rows], [(1, "crash"), (0, "capture")])
        crash, cap = rows
        self.assertAlmostEqual(cap["start_bearing_deg"], 0.0)
        self.assertAlmostEqual(crash["start_bearing_deg"], 90.0)
        self.assertAlmostEqual(cap["start_dist_xy_m"], 10.0)
        self.assertAlmostEqual(cap["step0_render_bearing_deg"], 0.0)
        self.assertAlmostEqual(crash["step0_render_bearing_deg"], 0.0, places=5)  # the rendered pose, not the reset tensor
        self.assertAlmostEqual(crash["step0_render_dist_xy_m"], 7.0, places=5)
        self.assertEqual(t.renders, 4)
        self.assertEqual((cap["start_detected"], cap["start_target_pixels"]), (True, 4))
        self.assertEqual((crash["start_detected"], crash["start_target_pixels"]), (False, 6))
        self.assertEqual((cap["frames"], cap["detected_frames"], cap["detected_pixel_sum"]), (3, 3, 12.0))
        self.assertEqual((crash["frames"], crash["detected_frames"], crash["detected_pixel_sum"], crash["first_detected_step"]),
                         (2, 1, 6.0, 1))
        self.assertEqual(cap["det_frames_by_dist_bin"][5], 3)                    # rendered at 10 m -> bin [10, 12)
        self.assertEqual(cap["det_frames_by_dist_bin"][6], 0)                    # not the advanced 13 m
        self.assertEqual(cap["det_pixels_by_dist_bin"][5], 12.0)
        self.assertAlmostEqual(cap["exec_speed_mean_mps"], 1.0)
        self.assertAlmostEqual(cap["robot_speed_mean_mps"], 1.5)
        self.assertEqual((cap["env_episode_index"], crash["env_episode_index"]), (0, 0))
        # the next episode of env 1 starts fresh
        for step in range(2):
            t._visible_now[:] = True
            t.transform_action_to_command(actions)
            t.next_capture[:] = False
            t.next_crash[:] = torch.tensor([False, step == 1, False])
            t.compute_state_reward_and_terminations()
            t._process_obs_perception()
        last = json.loads(self.path.read_text().splitlines()[-1])
        self.assertEqual((last["env_index"], last["env_episode_index"], last["frames"], last["start_detected"]),
                         (1, 1, 3, True))
        lines = M.telemetry_lines([err.getvalue()])
        self.assertTrue(lines and lines[0]["camera_wh"] == [8, 6] and lines[0]["detect_wh"] == [8, 6])

    def test_refuses_twice_and_without_a_detector(self):
        Task = _diag_task(2)
        M.install_diag_task(Task, {"NAVRL_INVIEW_DIAG_JSONL": str(self.path)})
        with self.assertRaises(RuntimeError):
            M.install_diag_task(Task, {"NAVRL_INVIEW_DIAG_JSONL": str(self.path)})
        t = Task()
        t.detector = None
        with self.assertRaises(RuntimeError):
            t.transform_action_to_command(torch.zeros((2, 4)))


# ================================================================ the realveh entry hook
def _entry_at(commit):
    """tools/realveh_entry.py as committed at `commit`, executed with this tree's __file__ (its ROOT is this tree)."""
    src = subprocess.run(["git", "-C", str(ROOT), "show", "%s:tools/realveh_entry.py" % commit], capture_output=True)
    if src.returncode:
        return None
    module = types.ModuleType("camdx1_test_realveh_entry_%s" % commit[:7])
    module.__file__ = str(ROOT / "tools/realveh_entry.py")
    exec(compile(src.stdout, module.__file__, "exec"), module.__dict__)
    return module


@needs_ft
class EntryHook(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.env = dict(FT_ENV, NAVRL_REALVEH_RECEIPT=str(pathlib.Path(self.tmp.name) / "receipt.json"))

    def tearDown(self):
        self.tmp.cleanup()

    def test_off_settings_equal_the_training_commit_entry(self):
        old = _entry_at(FT_TRAIN_COMMIT)
        if old is None:
            self.skipTest("git object %s unavailable (skipped, not passed)" % FT_TRAIN_COMMIT)
        new = E.settings(dict(self.env))
        self.assertEqual(new, old.settings(dict(self.env)))
        self.assertIn("cams2", new)                                               # non-vacuous: the real FT settings
        self.assertNotIn("inview_start", new)
        self.assertNotIn("inview_diag", new)

    def test_off_does_not_load_the_module_and_on_does(self):
        loaded = []
        orig = E._load_private

        def spy(name, path):
            loaded.append(name)
            return orig(name, path)

        E._load_private = spy
        try:
            E.settings(dict(self.env))
            self.assertNotIn("realveh_inview_settings", loaded)
            self.assertIn("realveh_cams2_settings", loaded)                     # positive control: the spy sees loads
            path = str(pathlib.Path(self.tmp.name) / "rows.jsonl")
            on = dict(self.env, NAVRL_INVIEW_START="1", NAVRL_INVIEW_START_HALF_DEG="28.35", NAVRL_INVIEW_START_SEED="4501",
                      NAVRL_INVIEW_DIAG="1", NAVRL_INVIEW_DIAG_JSONL=path)
            s = E.settings(on)
            self.assertIn("realveh_inview_settings", loaded)
        finally:
            E._load_private = orig
        self.assertEqual(s["inview_start"], {"NAVRL_INVIEW_START_HALF_DEG": 28.35, "NAVRL_INVIEW_START_SEED": 4501})
        self.assertEqual(s["inview_diag"], {"NAVRL_INVIEW_DIAG_JSONL": path})
        self.assertEqual({k: v for k, v in s.items() if not k.startswith("inview_")}, E.settings(dict(self.env)))
        for bad in (dict(self.env, NAVRL_INVIEW_START_SEED="1"), dict(on, NAVRL_INVIEW_START_HALF_DEG="99"),
                    dict(self.env, NAVRL_INVIEW_DIAG="1")):
            with self.subTest(bad={k: v for k, v in bad.items() if "INVIEW" in k}), self.assertRaises(SystemExit):
                E.settings(bad)

    def test_install_order_and_not_hashed(self):
        src = (ROOT / "tools/realveh_entry.py").read_text()
        order = [src.index(m) for m in ("ablation.install(", "camsafe.install(", "cams2.install(", "yawclip.install(",
                                        "cams2diag.install(", "inview.install_start(", "inview.install_diag(")]
        self.assertEqual(order, sorted(order))
        self.assertNotIn("inview", E.HASHED)
        self.assertTrue(E.INVIEW.is_file())
        diff = subprocess.run(["git", "-C", str(ROOT), "diff", FT_TRAIN_COMMIT, "--", "tools/realveh_entry.py"],
                              capture_output=True, text=True)
        if diff.returncode == 0:
            removed = [line for line in diff.stdout.splitlines() if line.startswith("-") and not line.startswith("---")]
            self.assertEqual(removed, [])                                         # additive only


# ================================================================ the runner
class Plans(unittest.TestCase):
    def test_plans_and_memory_modes(self):
        plan = X.eval_plan()
        self.assertEqual(len(plan), 12)
        self.assertEqual(plan[:4], [("R312", 205, 4501), ("R624", 205, 4501), ("V312", 205, 4501), ("V624", 205, 4501)])
        self.assertEqual([s for _, _, s in plan], [4501] * 4 + [4502] * 4 + [4503] * 4)
        self.assertEqual(X.mem_for("R624", "E128", "eval"), (128, 8))
        self.assertEqual(X.mem_for("R624", "E64", "eval"), (64, 16))
        self.assertEqual(X.mem_for("V312", "E64", "eval"), (128, 8))                # 312 cells stay at 128 x 8
        self.assertEqual(X.mem_for("V624_smoke", "E64", "smoke"), (64, 2))
        self.assertEqual(X.arm_dir("R624", 64), "R624_e64")
        self.assertEqual(X.cell_id("V624_smoke", 205, 4501), "DX1_V624__205bars__seed4501")
        self.assertEqual(json.loads(json.dumps(X.MAX_HOURS)), {"smoke": 0.5, "identity": 0.5, "eval": 3.5})
        self.assertEqual(X.scope_digest(), hashlib.sha256(X.SCOPE.encode()).hexdigest())

    def test_code_allowlist(self):
        ok = ["docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md", "WORKLOG.md", "docs/EXPERIMENTS.md",
              "tests/test_camdx1.py", "tools/camdx1_eval_run.py", "tools/realveh_entry.py", X.INVIEW_REL]
        self.assertEqual(X.allowlist_problems(ok, X.CODE_ALLOWLIST, X.FT_TRAIN_HEAD), [])
        for bad in ("aerial_gym/task/navrl_task/navrl_task.py", "tools/cams2b_eval_run.py", X.LAUNCHER):
            self.assertTrue(X.allowlist_problems([bad], X.CODE_ALLOWLIST, X.FT_TRAIN_HEAD), bad)

    def test_oom_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = pathlib.Path(tmp)
            (d / "receipt.json").write_text(json.dumps({"status": "FAILED"}))
            (d / "cell.log").write_text("RuntimeError: CUDA out of memory. Tried to allocate 2.00 GiB\n")
            self.assertTrue(X.is_oom(d))
            (d / "cell.log").write_text("RuntimeError: something else\n")
            self.assertFalse(X.is_oom(d))
            (d / "receipt.json").write_text(json.dumps({"status": "OK"}))
            (d / "cell.log").write_text("warn: out of memory once\n")
            self.assertFalse(X.is_oom(d))


@needs_ft
class Environments(unittest.TestCase):
    def test_training_environment_as_preregistered(self):
        self.assertEqual(X.training_env_problems(FT_ENV), [])
        self.assertTrue(X.training_env_problems(dict(FT_ENV, NAVRL_CAMERA_WIDTH="624")))
        self.assertTrue(X.training_env_problems(dict(FT_ENV, NAVRL_INVIEW_START="1")))
        with self.assertRaises(ValueError):
            X.cell_navrl_env(dict(FT_ENV, NAVRL_GENERAL_GOAL_DIST_MAX="12"), 4501, "R312", "/c")

    def test_every_planned_cell_environment_is_accepted(self):
        protocol = json.loads(X.K.PROTOCOL.read_text())
        self.assertEqual(X.plan_env_problems(FT_ENV, protocol, pathlib.Path("/tmp/camdx1_test_out")), [])
        n = 0
        for arm in X.EVAL_ARMS:
            for seed in X.SEEDS:
                env = X.cell_navrl_env(FT_ENV, seed, arm, "/c/%s" % arm)
                self.assertEqual(X.env_problems(env, FT_ENV, arm, seed, "/c/%s" % arm), [])
                n += 1
        self.assertEqual(n, 12)

    def test_arm_values(self):
        r = X.cell_navrl_env(FT_ENV, 4502, "R312", "/c")
        v = X.cell_navrl_env(FT_ENV, 4502, "V624", "/c")
        self.assertNotIn("NAVRL_INVIEW_START", r)
        self.assertEqual((r["NAVRL_CAMERA_WIDTH"], r["NAVRL_DETECT_HEIGHT"], r["NAVRL_GENERAL_GOAL_DIST_MAX"]), ("312", "234", "28"))
        self.assertEqual((v["NAVRL_CAMERA_WIDTH"], v["NAVRL_CAMERA_HEIGHT"], v["NAVRL_DETECT_WIDTH"], v["NAVRL_DETECT_HEIGHT"]),
                         ("624", "468", "624", "468"))
        self.assertEqual((v["NAVRL_GENERAL_GOAL_DIST_MAX"], v["NAVRL_INVIEW_START"], v["NAVRL_INVIEW_START_HALF_DEG"],
                          v["NAVRL_INVIEW_START_SEED"]), ("12", "1", "28.35", "4502"))
        self.assertEqual((r["NAVRL_INVIEW_DIAG"], r["NAVRL_INVIEW_DIAG_JSONL"]), ("1", "/c/" + X.INVIEW_FILE))
        self.assertEqual(r["NAVRL_YAW_RATE_MAX"], FT_ENV["NAVRL_YAW_RATE_MAX"])
        self.assertNotIn("NAVRL_YAWCLIP", r)
        diff = sorted(k for k in set(r) | set(v) if r.get(k) != v.get(k))
        self.assertEqual(diff, sorted(X.ARM_KEYS))                               # R312 vs V624: exactly the arm keys

    def test_foreign_and_missing_keys_refused(self):
        good = X.cell_navrl_env(FT_ENV, 4501, "V624", "/c")
        self.assertEqual(X.env_problems(good, FT_ENV, "V624", 4501, "/c"), [])
        r312 = X.cell_navrl_env(FT_ENV, 4501, "R312", "/c")
        bad = {"foreign key": (dict(good, NAVRL_FOO="1"), "V624"),
               "training key changed": (dict(good, NAVRL_YAW_RATE_MAX="3.0"), "V624"),
               "training key dropped": ({k: v for k, v in good.items() if k != "NAVRL_CAMSAFE_BLIND_MPS"}, "V624"),
               "yaw clip": (dict(good, NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS="0.7854"), "V624"),
               "V half missing": ({k: v for k, v in good.items() if k != "NAVRL_INVIEW_START_HALF_DEG"}, "V624"),
               "V seed of another cell": (dict(good, NAVRL_INVIEW_START_SEED="4502"), "V624"),
               "624 detection at 312": (dict(good, NAVRL_DETECT_WIDTH="312"), "V624"),
               "R with visible start": (dict(r312, NAVRL_INVIEW_START="1", NAVRL_INVIEW_START_HALF_DEG="28.35",
                                             NAVRL_INVIEW_START_SEED="4501"), "R312"),
               "R with 12 m goals": (dict(r312, NAVRL_GENERAL_GOAL_DIST_MAX="12"), "R312"),
               "recorder elsewhere": (dict(r312, NAVRL_INVIEW_DIAG_JSONL="/elsewhere.jsonl"), "R312"),
               "launcher key preset": (dict(r312, NAVRL_SEED="4501"), "R312"),
               "V624 env used as R624": (good, "R624")}
        for name, (env, arm) in bad.items():
            with self.subTest(name):
                self.assertTrue(X.env_problems(env, FT_ENV, arm, 4501, "/c"))


def _reroot(obj, old, new):
    if isinstance(obj, dict):
        return {k: _reroot(v, old, new) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_reroot(v, old, new) for v in obj]
    if isinstance(obj, str) and obj.startswith(str(old) + "/"):
        return str(new) + obj[len(str(old)):]
    return obj


def _snapshot(cell_dir, arm, seed, ckpt_sha, quota=8, **changes):
    snap = dict(X.cell_navrl_env(FT_ENV, seed, arm, cell_dir, quota))
    snap.update({"NAVRL_BULK_EVAL": "1", "NAVRL_BULK_EVAL_JSON": str(cell_dir / "result.json"), "NAVRL_EVAL_CHECKPOINT": ckpt_sha,
                 "NAVRL_EVAL_RUN_NONCE": "n", "NAVRL_NUM_BARS": "205", "NAVRL_DENSITY_START": "205",
                 "NAVRL_DENSITY_FINAL": "205", "NAVRL_SEED": str(seed),
                 "NAVRL_REALVEH_RECEIPT": str(cell_dir / "realveh_receipt.json"),
                 "NAVRL_CAMS2DIAG_JSONL": str(cell_dir / X.DIAG_FILE)})
    snap.update(changes)
    return snap


def _write_cell(cell_dir, arm, seed, committed, ckpt_sha, train_sha, n_env=4, quota=2, tw=None, outcomes=None):
    """A synthetic, faithful CAM-DX1 cell (tw: deviations)."""
    tw = tw or {}
    cell_dir.mkdir(parents=True)
    vis = X.visible(arm)
    seeds = S2A.module_seeds(seed)
    rec = _reroot(json.loads(json.dumps(FT_RECEIPT)), W.tree_of(FT_RECEIPT), ROOT)
    rec["settings"]["cams2"]["NAVRL_CAMS2_DELAY_SEED"] = int(seeds["NAVRL_CAMS2_DELAY_SEED"])
    rec["settings"]["vehicle_rand"]["NAVRL_VEH_RAND_SEED"] = int(seeds["NAVRL_VEH_RAND_SEED"])
    rec["settings"]["depth_model"]["NAVRL_DEPTH_SEED"] = int(seeds["NAVRL_DEPTH_SEED"])
    rec["sha256"]["realveh_entry"] = committed["realveh_entry"]
    rec["settings"]["cams2_contactcode"] = {}
    rec["cams2_contactcode"] = {"contact_code": 0}
    rec["cams2_contactcode_sha256"] = committed["contactcode"]
    rec["settings"]["cams2diag"] = {"NAVRL_CAMS2DIAG_JSONL": str(cell_dir / X.DIAG_FILE), "NAVRL_CAMS2DIAG_YAW_REF_RADPS": 0.7854}
    rec["cams2diag"] = {"jsonl": str(cell_dir / X.DIAG_FILE), "yaw_ref_radps": 0.7854}
    rec["cams2diag_sha256"] = committed["cams2diag"]
    rec["settings"]["inview_diag"] = {"NAVRL_INVIEW_DIAG_JSONL": str(cell_dir / X.INVIEW_FILE)}
    rec["inview_diag"] = {"jsonl": str(cell_dir / X.INVIEW_FILE)}
    rec["inview_sha256"] = committed["inview"]
    if vis or tw.get("start_in_r"):
        rec["settings"]["inview_start"] = {"NAVRL_INVIEW_START_HALF_DEG": 28.35, "NAVRL_INVIEW_START_SEED": seed}
        rec["inview_start"] = {"half_deg": 28.35, "seed": tw.get("start_seed", seed)}
    rec["env_navrl"] = _snapshot(cell_dir, arm, seed, ckpt_sha, quota, **tw.get("entry_env", {}))
    (cell_dir / "realveh_receipt.json").write_text(json.dumps(rec))
    (cell_dir / "receipt.json").write_text(json.dumps({
        "status": "OK", "train_receipt_sha256": train_sha, "git_commit": "h" * 40, "git_dirty_files": 0,
        "checkpoint_sha256": ckpt_sha, "env_snapshot_navrl": _snapshot(cell_dir, arm, seed, ckpt_sha, quota),
        "duration_s": 300, "sim_name": "base_sim", "python": S2A.REAL_PYTHON, "gpu4gb": "", "result_sha256": None}))
    ledger, drows, irows, step = [], [], [], 0
    outs = outcomes or ["capture", "crash", "crash", "timeout"]
    for ep in range(quota):
        for env in range(n_env):
            step += 7
            o = outs[(env + ep) % len(outs)]
            ledger.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o, "in_quota": True,
                           "episode_id": "%d-%d" % (env, ep), "min_relative_distance_m": 0.8 if o == "capture" else 4.0})
            drows.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o,
                          "crash_class": "contact" if o == "crash" else "none", "episode_steps": 7,
                          "hidden_steps": 3 if env else 7, "visible_steps": 4 if env else 0,
                          "yaw_request_over_ref_steps": 1, "yaw_exec_over_ref_steps": 1, "yaw_ref_radps": 0.7854})
            k = env % 4
            bearing = tw.get("bearing", (-20.0 + 13.0 * k) if vis else (-150.0, 10.0, 30.0, 120.0)[k])
            dist = tw.get("dist", 6.5 + k) if vis else 6.5 + 5 * k
            area = tw.get("area624", 16.0) if X.res(arm) == 624 else 4.0
            irows.append({"env_index": env, "env_episode_index": ep, "completion_step": step, "outcome": o, "episode_steps": 7,
                          "start_recorded": True, "start_bearing_deg": tw.get("reset_bearing", bearing),
                          "start_dist_xy_m": dist, "start_dist_m": 7.0,
                          "step0_render_bearing_deg": bearing, "step0_render_dist_xy_m": dist,
                          "start_detected": env % 2 == 0, "start_target_pixels": 4, "first_detected_step": 0,
                          "frames": 7, "detected_frames": 4, "detected_pixel_sum": 4.0 * area,
                          "det_frames_by_dist_bin": [0, 0, 0, 4] + [0] * 12,
                          "det_pixels_by_dist_bin": [0, 0, 0, 4.0 * area] + [0.0] * 12,
                          "exec_speed_mean_mps": 1.0, "robot_speed_mean_mps": 0.9})
    (cell_dir / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in ledger))
    (cell_dir / X.DIAG_FILE).write_text("".join(json.dumps(r) + "\n" for r in drows))
    (cell_dir / X.INVIEW_FILE).write_text("".join(json.dumps(r) + "\n" for r in irows))
    (cell_dir / "episode_ledger_summary.json").write_text(json.dumps({"status": "COMPLETE", "partition_violations": 0}))
    cond = dict(S2A.CONDITION_EXPECTED, p9_empirical_error_sha256=FT_ENV["NAVRL_P9_EXPECTED_SHA256"],
                d8_dynamic_mesh_contract={"mode": S2A.D8_MODE}, evaluation_nonce="n", num_envs=n_env, rl_step_dt_s=0.1,
                goal_dist_max_m=12.0 if vis else 28.0)
    (cell_dir / "result.json").write_text(json.dumps({"checkpoint": ckpt_sha, "outcome": {"n": len(ledger)}, "condition": cond}))
    w, h = X.RES[X.res(arm)]
    log = "WARNING NavRL TRAINING SEED MISMATCH | checkpoint=1 running=%d. x\n" % seed
    log += "[camsafe] calls=320 capped_frac=0.0100 penalised_frac=0.0200 mean_penalty=0.00100 blind_mps=1 weight=0.5\n"
    log += "[cams2contact] {\"event\": \"summary\", \"steps\": 320, \"contacts\": 4}\n"
    log += "[cams2diag] {\"event\": \"summary\", \"rows\": %d}\n" % len(drows)
    cam = tw.get("camera_wh", [int(w), int(h)])
    log += "[inviewdiag] %s\n" % json.dumps({"event": "summary", "camera_wh": cam, "detect_wh": cam, "rows": len(irows)})
    if vis or tw.get("start_in_r"):
        log += "[inview] calls=40 envs=%d max_abs_rel_deg=27.9000 half_deg=28.35 seed=%d\n" % (len(irows), seed)
    if X.res(arm) == 624:
        log += ("NavRL CONFIG MISMATCH | NAVRL_DETECT_WIDTH: checkpoint trained with 312.000, running with 624.000. This "
                "changes policy inputs\n")
    if vis:
        log += "NavRL TASK-DISTRIBUTION MISMATCH | NAVRL_GENERAL_GOAL_DIST_MAX: checkpoint used 28.000, running with 12.000. x\n"
    log += tw.get("extra_log", "")
    (cell_dir / "cell.log").write_text(log)
    return cell_dir


@needs_ft
class CellChecks(unittest.TestCase):
    COMMITTED = {"realveh_entry": "e" * 64, "cams2diag": "d" * 64, "contactcode": "k" * 64, "inview": "i" * 64}
    CK, TSHA = "c" * 64, "t" * 64

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _check(self, arm, tag="", seed=4502, **tw):
        d = _write_cell(self.base / (arm + tag) / X.cell_id(arm, 205, seed), arm, seed, self.COMMITTED, self.CK, self.TSHA,
                        tw=tw)
        return X.receipt_problems(d, arm, 205, seed, FT_RECEIPT, self.TSHA, self.COMMITTED, self.CK, quota=2)

    def test_faithful_cells_of_every_arm_pass(self):
        for arm in X.EVAL_ARMS:
            with self.subTest(arm):
                self.assertEqual(self._check(arm), [])

    def test_deviations_fail(self):
        cases = {"V bearing outside": ("V312", {"bearing": 29.0}, "bearings outside"),
                 "V rendered outside, reset tensor inside": ("V624", {"bearing": 29.0, "reset_bearing": 0.0},
                                                             "rendered step-0 bearings outside"),         # r1 P1-1
                 "V distance outside": ("V624", {"dist": 12.5}, "distances outside"),
                 "624 rendered at 312": ("R624", {"camera_wh": [312, 234]}, "[inviewdiag]"),
                 "312 rendered at 624": ("R312", {"camera_wh": [624, 468]}, "[inviewdiag]"),
                 "R with a visible start": ("R312", {"start_in_r": True}, "visible-start receipt"),
                 "V with another seed": ("V312", {"start_seed": 1}, "visible-start receipt"),
                 "R312 with a 624 mismatch line": ("R312", {"extra_log": "NavRL CONFIG MISMATCH | NAVRL_DETECT_WIDTH: "
                                                            "checkpoint trained with 312.000, running with 624.000. x\n"},
                                                   "checkpoint-contract"),
                 "unexpected mismatch": ("V624", {"extra_log": "NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint "
                                                               "trained with 1.047, running with 3.000\n"}, "checkpoint-contract"),
                 "entry env foreign key": ("R624", {"entry_env": {"NAVRL_FOO": "1"}}, "entry env_navrl differs"),
                 "entry env 312 detection": ("V624", {"entry_env": {"NAVRL_DETECT_WIDTH": "312"}}, "NAVRL_DETECT_WIDTH")}
        for name, (arm, tw, needle) in cases.items():
            with self.subTest(name):
                p = self._check(arm, name.replace(" ", "_"), **tw)
                self.assertTrue(any(needle in x for x in p), (name, p))

    def test_output_problems_with_the_cell_env_count(self):
        d = _write_cell(self.base / "o" / X.cell_id("V624", 205, 4501), "V624", 4501, self.COMMITTED, self.CK, self.TSHA)
        p, stats = X.output_problems(d, "V624", "h" * 40, self.CK, self.TSHA, FT_ENV, 4, 2)
        self.assertEqual(p, [])
        self.assertEqual(stats["quota_episodes"], 8)
        self.assertTrue(X.output_problems(d, "R624", "h" * 40, self.CK, self.TSHA, FT_ENV, 4, 2)[0])   # goal max 12 != 28
        self.assertTrue(X.output_problems(d, "V624", "h" * 40, self.CK, self.TSHA, FT_ENV, 8, 1)[0])   # env count


class SmokeControls(unittest.TestCase):
    @staticmethod
    def _rows(bearings, dists=None, area=None, det=4):
        dists = dists or [7.0] * len(bearings)
        return [{"start_bearing_deg": b, "start_dist_xy_m": d, "detected_frames": det, "start_detected": True,
                 X.RENDER_BEARING: b, X.RENDER_DIST: d,
                 "detected_pixel_sum": det * area if area is not None else 0.0, "start_target_pixels": area or 0}
                for b, d in zip(bearings, dists)]

    def test_visible_start_control(self):
        ok = X.visible_start_control(self._rows([-28.3, 0.0, 28.35]), self._rows([10.0, 120.0]))
        self.assertTrue(ok["ok"])
        self.assertEqual((ok["v_within"], ok["r_outside_33_35"]), (3, 1))
        self.assertFalse(X.visible_start_control(self._rows([-28.3, 30.0]), self._rows([120.0]))["ok"])
        self.assertFalse(X.visible_start_control(self._rows([0.0]), self._rows([10.0, -30.0]))["ok"])   # R never outside
        self.assertFalse(X.visible_start_control([], self._rows([120.0]))["ok"])                         # no V rows
        # review r1 P1-1: the RENDERED bearing decides, not the reset-time tensor the wrapper wrote
        rendered_out = [dict(r, **{X.RENDER_BEARING: 40.0}) for r in self._rows([0.0, 5.0])]
        self.assertFalse(X.visible_start_control(rendered_out, self._rows([120.0]))["ok"])

    def test_step0_detection_control(self):
        v = [dict(r, start_detected=i < 3) for i, r in enumerate(self._rows([0.0] * 6))]     # 3 / 6
        r = [dict(x, start_detected=i < 1) for i, x in enumerate(self._rows([90.0] * 8))]    # 1 / 8
        c = X.step0_detection_control(v, r)
        self.assertTrue(c["ok"])
        self.assertAlmostEqual(c["v_share"], 0.5)
        self.assertFalse(X.step0_detection_control(v, [dict(x, start_detected=i < 3) for i, x in
                                                       enumerate(self._rows([90.0] * 8))])["ok"])    # 0.5 < 2 x 0.375
        self.assertFalse(X.step0_detection_control([dict(x, start_detected=False) for x in v], [])["ok"])
        self.assertTrue(X.step0_detection_control(v, [dict(x, start_detected=False) for x in r])["ok"])   # R 0, V > 0

    @staticmethod
    def _framed(rows, k, area):
        return [dict(r, det_frames_by_dist_bin=[0] * k + [4] + [0] * (15 - k),
                     det_pixels_by_dist_bin=[0.0] * k + [4 * area] + [0.0] * (15 - k)) for r in rows]

    def test_pixel_control(self):
        hi = self._rows([0.0] * 4, [6.5, 7.5, 9.0, 11.0], area=16.0)
        lo = self._rows([0.0] * 4, [6.5, 9.2, 15.0, 25.0], area=4.0)
        c = X.pixel_control(hi, lo)
        self.assertEqual((c["verdict"], c["basis"]), ("PASS", "step0"))
        self.assertAlmostEqual(c["ratio"], 4.0)
        self.assertEqual(sorted(c["matched_bins"]), ["6", "9"])                   # 1 m step-0 distance bins
        self.assertEqual(c["step0_detected_frames"], {"624": 4, "312": 4})
        self.assertAlmostEqual(c["reported_episode_frames_by_step0_bin_ratio"], 4.0)
        self.assertEqual(X.pixel_control(self._rows([0.0], [6.6], area=8.0), lo)["verdict"], "FAIL")      # ratio 2
        # review r1 P1-2: no matched step-0 frame -> every detected frame by its own distance decides
        far = self._rows([0.0], [25.0], area=4.0)
        self.assertEqual(X.pixel_control(hi, far)["verdict"], "NO_MATCHED_FRAMES")                         # nothing at all
        fb = X.pixel_control(self._framed(hi, 4, 16.0), self._framed(far, 4, 4.0))
        self.assertEqual((fb["verdict"], fb["basis"]), ("PASS", "own_distance"))
        self.assertAlmostEqual(fb["gated_ratio"], 4.0)
        fb2 = X.pixel_control(self._framed(hi, 4, 8.0), self._framed(far, 4, 4.0))
        self.assertEqual((fb2["verdict"], fb2["basis"]), ("FAIL", "own_distance"))
        undetected = [dict(r, start_detected=False) for r in lo]
        self.assertEqual(X.pixel_control(hi, undetected)["verdict"], "NO_MATCHED_FRAMES")   # only detected step-0 frames


class Verdicts(unittest.TestCase):
    @staticmethod
    def _rec(arm, verdict="OK", seed=4501, n=128, finished=10, wall=None):
        return {"arm": arm, "base_arm": X.ARM_BASE.get(arm, arm), "verdict": verdict, "seed": seed, "density": 205,
                "num_envs": n, "stats": {"finished_episodes": finished}, "wall_s": wall}

    def test_smoke(self):
        recs = [self._rec("R312_smoke"), self._rec("V624_smoke")]
        good, px, dc = {"ok": True}, {"verdict": "PASS"}, {"ok": True}
        self.assertEqual(X.smoke_verdict(recs, good, px, "E128", dc), "SMOKE_OK")
        self.assertEqual(X.smoke_verdict(recs, good, {"verdict": "NO_MATCHED_FRAMES"}, "E128", dc), "SMOKE_FAIL")  # r1 P1-2
        self.assertEqual(X.smoke_verdict(recs, good, px, "E128", {"ok": False}), "SMOKE_FAIL")                   # r1 P1-1
        self.assertEqual(X.smoke_verdict(recs, good, px, "E128"), "SMOKE_FAIL")
        self.assertEqual(X.smoke_verdict(recs, {"ok": False}, px, "E128", dc), "SMOKE_FAIL")
        self.assertEqual(X.smoke_verdict(recs, good, {"verdict": "FAIL"}, "E128", dc), "SMOKE_FAIL")
        self.assertEqual(X.smoke_verdict([recs[0], self._rec("V624_smoke", finished=0)], good, px, "E128", dc), "SMOKE_FAIL")
        oom = [recs[0], self._rec("V624_smoke", "OOM"), self._rec("V624_smoke", "OOM", n=64), self._rec("V312_smoke")]
        self.assertEqual(X.smoke_verdict(oom, good, {"verdict": "NOT_RUN_OOM"}, "NOT_RUN_OOM", dc), "SMOKE_OK")
        self.assertEqual(X.smoke_verdict(recs[:1], good, px, "E128", dc), "SMOKE_FAIL")
        self.assertEqual(X.smoke_verdict(recs + [self._rec("R312_smoke", "VOID_HOST_FAULT")], good, px, "E128", dc),
                         "VOID_HOST_FAULT")

    def test_cell_verdict_oom_versus_host_fault(self):                         # review r1 P2-3
        self.assertEqual(X.cell_verdict([], False, "R624"), "OK")
        self.assertEqual(X.cell_verdict(["missing or unreadable output"], True, "R624"), "OOM")
        self.assertEqual(X.cell_verdict(["HOST fault_count changed", "missing output"], True, "V624"), "VOID_HOST_FAULT")
        self.assertEqual(X.cell_verdict(["HOST fault_count changed", "missing output"], False, "V624"), "VOID_HOST_FAULT")
        self.assertEqual(X.cell_verdict(["missing or unreadable output"], True, "R312"), "FAIL")         # 312: no fallback
        self.assertEqual(X.cell_verdict(["missing or unreadable output"], False, "R624"), "FAIL")

    def test_projected_cell_time(self):                                         # review r1 P2-4
        smoke = {"records": [self._rec("R312_smoke", wall=100.0), self._rec("V624_smoke", wall=250.0)]}
        ident = {"records": [self._rec("R312_identity_a", wall=400.0), self._rec("R312_identity_b", wall=420.0)]}
        self.assertAlmostEqual(X.projected_cell_s("V312", 128, [], smoke, ident), 1.5 * 420.0)
        self.assertAlmostEqual(X.projected_cell_s("R624", 128, [], smoke, ident), 1.5 * 420.0 * 2.5)
        seen = [self._rec("R624", wall=900.0), self._rec("R624", wall=1000.0, seed=4502)]
        self.assertEqual(X.projected_cell_s("R624", 128, seen, smoke, ident), 1000.0)               # observed max
        self.assertIsNone(X.projected_cell_s("R624", 64, [], smoke, ident))                         # smoke at 128 only
        self.assertIsNone(X.projected_cell_s("R312", 128, [], smoke, {}))

    def test_identity(self):
        recs = [self._rec("R312_identity_a"), self._rec("R312_identity_b")]
        eq = {"result_equal": True, "episodes_equal": True}
        self.assertEqual(X.identity_verdict(recs, eq), "IDENTITY_PASS")
        self.assertEqual(X.identity_verdict(recs, dict(eq, episodes_equal=False)), "IDENTITY_FAIL")
        self.assertEqual(X.identity_verdict(recs[:1], eq), "IDENTITY_FAIL")
        self.assertEqual(X.identity_verdict(recs, None), "IDENTITY_FAIL")

    def test_eval_with_memory_modes(self):
        full = [self._rec(a, seed=s) for a, _, s in X.eval_plan()]
        self.assertEqual(X.eval_verdict(full, "E128"), "COMPLETE")
        self.assertEqual(X.eval_verdict(full, "E64"), "INCOMPLETE")                 # 624 cells must be at 64 envs
        e64 = [r if X.res(r["arm"]) == 312 else dict(r, num_envs=64) for r in full]
        self.assertEqual(X.eval_verdict(e64 + [self._rec("R624", "OOM")], "E64"), "COMPLETE")
        only312 = [r for r in full if X.res(r["arm"]) == 312]
        self.assertEqual(X.eval_verdict(only312, "NOT_RUN_OOM"), "COMPLETE_624_NOT_RUN_OOM")
        self.assertEqual(X.eval_verdict(only312[:-1], "NOT_RUN_OOM"), "INCOMPLETE")
        self.assertEqual(X.eval_verdict(full[:-1], "E128"), "INCOMPLETE")
        self.assertEqual(X.eval_verdict(full + [self._rec("V312", "FAIL")], "E128"), "INCOMPLETE")
        self.assertEqual(X.eval_verdict(full[:-1] + [self._rec("V624", "BUDGET_EXHAUSTED", seed=4503)], "E128"),
                         "INCOMPLETE")

    def test_stage_links(self):
        with tempfile.TemporaryDirectory() as tmp:
            parent = pathlib.Path(tmp)
            p, _ = X.stage_link_problems("identity", parent / "identity_01", "h" * 40, X.FT_CKPT_SHA)
            self.assertTrue(any("smoke_report.json" in x for x in p))
            (parent / "smoke").mkdir()
            (parent / "smoke" / "smoke_report.json").write_text(json.dumps(
                {"verdict": "SMOKE_OK", "commit": "h" * 40, "ft_checkpoint_sha256": X.FT_CKPT_SHA}))
            p, found = X.stage_link_problems("identity", parent / "identity_01", "h" * 40, X.FT_CKPT_SHA)
            self.assertEqual(p, [])
            self.assertIn("smoke_report.json", found)
            self.assertTrue(X.stage_link_problems("identity", parent / "identity_01", "g" * 40, X.FT_CKPT_SHA)[0])
            self.assertTrue(X.stage_link_problems("eval", parent / "eval_01", "h" * 40, X.FT_CKPT_SHA)[0])
            self.assertTrue(X.stage_link_problems("smoke", parent / "smoke_02", "h" * 40, X.FT_CKPT_SHA)[0])


@needs_ft
class Policy(unittest.TestCase):
    def test_policy_accepted_and_altered_refused(self):
        s2a = json.loads(W.S2A_TRAIN_RECEIPT.read_text())
        report = json.loads(X.FT_TRAIN_REPORT.read_text())
        ck, p = X.policy_problems(s2a, FT_RECEIPT, report)
        self.assertEqual(p, [])
        self.assertEqual(ck[1], X.FT_CKPT_SHA)
        alt = json.loads(json.dumps(report))
        alt["preflight"]["head"] = "0" * 40
        self.assertTrue(X.policy_problems(s2a, FT_RECEIPT, alt)[1])
        alt = json.loads(json.dumps(report))
        alt["final_checkpoint"]["sha256"] = "0" * 64
        self.assertTrue(X.policy_problems(s2a, FT_RECEIPT, alt)[1])


# ================================================================ the analysis
class AnalysisRules(unittest.TestCase):
    def test_labels(self):
        L = AN.label
        self.assertEqual(L(8.0, (6.0, 10.0), "capture"), "MATERIAL_GAIN")
        self.assertEqual(L(5.0, (0.1, 9.0), "capture"), "MATERIAL_GAIN")
        self.assertEqual(L(8.0, (-1.0, 16.0), "capture"), "NO_DETECTED_CHANGE")
        self.assertEqual(L(2.0, (1.0, 3.0), "capture"), "SMALL")
        self.assertEqual(L(-8.0, (-10.0, -6.0), "capture"), "MATERIAL_LOSS")
        self.assertEqual(L(-8.0, (-10.0, -6.0), "crash"), "MATERIAL_GAIN")             # fewer crashes
        self.assertEqual(L(16.0, (13.0, 19.0), "crash"), "MATERIAL_LOSS")
        self.assertEqual(L(-2.0, (-3.0, -1.0), "crash"), "SMALL")

    def test_preregistered_controls(self):
        ok, res = AN.controls()
        self.assertEqual(set(res), {"synthetic_identical", "synthetic_plus8pp", "synthetic_plus2pp_tight",
                                    "synthetic_minus8pp", "synthetic_det_plus8pp", "cams2a_e_yc12k_minus_s12k_205"})
        self.assertEqual(res["synthetic_det_plus8pp"]["labels"]["DET"], ("MATERIAL_GAIN", "MATERIAL_GAIN"))   # r1 P2-6
        for name, r in res.items():
            if name == "cams2a_e_yc12k_minus_s12k_205" and "error" in r:
                self.skipTest("CAM-S2a-E cells unavailable (skipped, not passed): %s" % r["error"])
            self.assertTrue(r["ok"], (name, r.get("labels")))
        self.assertTrue(ok)
        s = res["cams2a_e_yc12k_minus_s12k_205"]
        self.assertLess(s["effect"]["capture"]["mean_pp"], -5.0)
        self.assertGreater(s["effect"]["crash"]["mean_pp"], 5.0)
        self.assertTrue(all(v for v in s["manifest"].values()))
        self.assertEqual(sum(sum(v.values()) for v in s["episodes_per_cell"].values()), 6 * 1024)

    def test_bootstrap_is_seeded_and_readings_without_624(self):
        cells = AN.synthetic_cells(1024, 8.0, -8.0)
        a = AN.readings(cells, list(X.SEEDS), AN.ARM_ORDER)
        b = AN.readings(cells, list(X.SEEDS), AN.ARM_ORDER)
        self.assertEqual(a["START"]["capture"]["ci95_pp"], b["START"]["capture"]["ci95_pp"])
        self.assertIsNone(a["INTERACTION"]["capture"]["label"])                      # reported only
        self.assertAlmostEqual(a["INTERACTION"]["capture"]["mean_pp"], 0.0)
        r = AN.readings({k: v for k, v in cells.items() if k in ("R312", "V312")}, list(X.SEEDS), {"R312", "V312"})
        self.assertEqual((r["DET"], r["INTERACTION"], r["START"]["flag"]), ("NOT_RUN_OOM", "NOT_RUN_OOM", "START_312_ONLY"))
        self.assertEqual(r["START"]["capture"]["label"], "MATERIAL_GAIN")
        # review r1 P1-3: without the distance-matched contrast no 'visible start' pointer is given
        self.assertIn("no preregistered pointer", AN.use_of_result(a)[0])
        near_same = {k: {s: dict(c) for s, c in cells[k].items()} for k in ("R312", "R624")}
        a2 = AN.readings(cells, list(X.SEEDS), AN.ARM_ORDER, near_same)
        self.assertEqual(a2["START_6_12"]["capture"]["label"], "MATERIAL_GAIN")
        self.assertEqual(a2["START"]["capture"], a["START"]["capture"])            # the restricted cells draw last
        self.assertIn("shorter range", a2["START"]["reading"])
        self.assertEqual(len(AN.use_of_result(a2)), 1)
        self.assertIn("visible-start", AN.use_of_result(a2)[0])
        near_v = {k: {s: dict(cells["V312"][s]) for s in X.SEEDS} for k in ("R312", "R624")}   # yaw explains nothing
        a3 = AN.readings(cells, list(X.SEEDS), AN.ARM_ORDER, near_v)
        self.assertEqual(a3["START_6_12"]["capture"]["label"], "NO_DETECTED_CHANGE")
        self.assertEqual(a3["START"]["capture"]["label"], "MATERIAL_GAIN")

    def test_incomplete_blocks(self):                                            # review r1 P2-4
        cells = AN.synthetic_cells(1024, 8.0, -8.0)
        part = {a: dict(v) for a, v in cells.items()}
        del part["V624"][4503]
        r = AN.readings(part, list(X.SEEDS), AN.ARM_ORDER)
        self.assertEqual(r["START"]["blocks"], [4501, 4502])
        self.assertEqual(r["DET"]["blocks"], [4501, 4502])
        none = {a: dict(v) for a, v in cells.items()}
        none["V624"] = {}
        r = AN.readings(none, list(X.SEEDS), AN.ARM_ORDER)
        self.assertEqual((r["START"], r["DET"], r["INTERACTION"]), ("NOT_RUN", "NOT_RUN", "NOT_RUN"))

    def test_start_class_and_det_gate(self):
        self.assertEqual(AN.start_class(20.0, 9.0), "in_view_6_12")
        self.assertEqual(AN.start_class(-40.0, 9.0), "out_of_view_6_12")
        self.assertEqual(AN.start_class(0.0, 12.5), "beyond_12")
        pooled = {a: {"bin_frames": [0, 0, 0, 10, 10, 0] + [0] * 10,
                      "bin_pixels": [0, 0, 0, 10 * px, 10 * px, 0] + [0.0] * 10}
                  for a, px in (("R312", 4.0), ("R624", 16.0), ("V312", 4.0), ("V624", 16.0))}
        g = AN.det_pixel_gate(pooled)
        self.assertTrue(g["interpretable"])
        self.assertAlmostEqual(g["R"], 4.0)
        pooled["V624"]["bin_pixels"] = [0, 0, 0, 80.0, 80.0, 0] + [0.0] * 10                    # ratio 2
        self.assertFalse(AN.det_pixel_gate(pooled)["interpretable"])
        del pooled["R624"]
        self.assertFalse(AN.det_pixel_gate(pooled)["interpretable"])


@needs_ft
class AnalysisEndToEnd(unittest.TestCase):
    COMMITTED = CellChecks.COMMITTED

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _stage(self, name, verdict, records=(), **extra):
        d = self.base / name
        d.mkdir(parents=True, exist_ok=True)
        rep = {"unit": X.UNIT, "verdict": verdict, "commit": "h" * 40, "ft_checkpoint_sha256": X.FT_CKPT_SHA,
               "records": list(records), **extra}
        (d / {"smoke": "smoke_report.json", "identity": "identity_report.json", "eval": "eval_run_report.json"}[name]
         ).write_text(json.dumps(rep))
        return d

    def _eval(self, sub="eval", **tw):
        recs = []
        for arm, d, s in X.eval_plan():
            outs = (["capture"] * 6 + ["crash"] * 3 + ["timeout"]) if X.visible(arm) else (["capture"] * 4 + ["crash"] * 5
                                                                                              + ["timeout"])
            cell = _write_cell(self.base / sub / arm / X.cell_id(arm, d, s), arm, s, self.COMMITTED, X.FT_CKPT_SHA, "t" * 64,
                               n_env=128, quota=8, outcomes=outs, tw=tw)
            recs.append({"arm": arm, "density": d, "seed": s, "verdict": "OK", "num_envs": 128, "commit": "h" * 40,
                         "cell_dir": str(cell), "stats": {"result_sha256": AN.sha(cell / "result.json")}})
        return recs

    def test_main_eval_analysed_then_binding_fail(self):
        recs = self._eval()
        smoke = self._stage("smoke", "SMOKE_OK", visible_start_control={"ok": True}, smoke_624_control={"verdict": "PASS"})
        ident = self._stage("identity", "IDENTITY_PASS")
        ev = self._stage("eval", "COMPLETE", recs, mem_624="E128")
        out = self.base / "analysis.json"
        with contextlib.redirect_stdout(io.StringIO()):
            rc = AN.main_eval(str(smoke), str(ident), str(ev), str(out))
        res = json.loads(out.read_text())
        if not res["controls_ok"] and "error" in res["controls"]["cams2a_e_yc12k_minus_s12k_205"]:
            self.skipTest("CAM-S2a-E cells unavailable (skipped, not passed)")
        self.assertEqual((rc, res["verdict"]), (0, "ANALYSED"), res.get("binding_problems"))
        self.assertEqual(res["readings"]["START"]["capture"]["label"], "MATERIAL_GAIN")      # +20 pp
        self.assertEqual(res["readings"]["START"]["crash"]["label"], "MATERIAL_GAIN")        # -20 pp crash
        self.assertEqual(res["readings"]["DET"]["capture"]["label"], "NO_DETECTED_CHANGE")
        p = res["pooled"]["V624"]
        self.assertEqual(p["n"], 3 * 1024)
        self.assertGreater(p["never_detected_share"], 0.0)
        self.assertAlmostEqual(p["mean_target_pixel_area_per_detected_frame"], 16.0)
        self.assertAlmostEqual(res["pooled"]["V312"]["mean_target_pixel_area_per_detected_frame"], 4.0)
        self.assertAlmostEqual(res["pixel_ratio_624_over_312"]["V"][3]["ratio"], 4.0)
        self.assertAlmostEqual(p["capture_time_s"]["median"], 0.7)
        self.assertEqual(p["step0_bearing_within_28_35_share"], 1.0)
        self.assertLess(res["pooled"]["R312"]["step0_bearing_within_33_35_share"], 1.0)
        self.assertGreater(p["min_dist_bins_by_outcome"]["capture"]["<1"], 0)
        self.assertEqual(res["best_cell_vs_user_bar"]["arm"], "V312")
        self.assertTrue(res["det_pixel_gate"]["interpretable"])
        self.assertEqual(res["readings"]["START_6_12"]["capture"]["label"], "MATERIAL_GAIN")
        sc = res["pooled"]["R312"]["start_classes"]
        self.assertTrue(all(sc[c]["n"] > 0 for c in AN.START_CLASSES))
        self.assertEqual(sum(sc[c]["n"] for c in AN.START_CLASSES), 3 * 1024)
        self.assertEqual(res["pooled"]["V312"]["start_classes"]["in_view_6_12"]["n"], 3 * 1024)
        self.assertTrue(any("visible-start" in x for x in res["use_of_result"]))
        # a changed result.json no longer binds
        cell = pathlib.Path(recs[5]["cell_dir"]) / "result.json"
        cell.write_text(cell.read_text() + " ")
        out2 = self.base / "analysis2.json"
        with contextlib.redirect_stdout(io.StringIO()):
            rc2 = AN.main_eval(str(smoke), str(ident), str(ev), str(out2))
        self.assertEqual(rc2, 1)
        self.assertEqual(json.loads(out2.read_text())["verdict"], "EVAL_BINDING_FAIL")

    def test_det_not_interpretable_and_incomplete(self):
        recs = self._eval("eval_px2", area624=8.0)                                # 624 / 312 pixel area only 2x
        smoke = self._stage("smoke", "SMOKE_OK")
        ident = self._stage("identity", "IDENTITY_PASS")
        drop = [r for r in recs if (r["arm"], r["seed"]) != ("V624", 4503)]
        ev = self._stage("eval", "INCOMPLETE", drop, mem_624="E128")
        out = self.base / "analysis_px2.json"
        with contextlib.redirect_stdout(io.StringIO()):
            AN.main_eval(str(smoke), str(ident), str(ev), str(out))
        res = json.loads(out.read_text())
        if not res["controls_ok"]:
            self.skipTest("controls unavailable on this host (skipped, not passed)")
        self.assertEqual(res["verdict"], "ANALYSED", res.get("binding_problems"))
        self.assertFalse(res["det_pixel_gate"]["interpretable"])
        self.assertEqual(res["readings"]["DET"]["capture"]["label"], "NOT_INTERPRETABLE")
        self.assertEqual(res["readings"]["START"]["blocks"], [4501, 4502])
        self.assertEqual(res["eval_verdict"], "INCOMPLETE")
        ev2 = self._stage("eval", "INCOMPLETE", drop[:-2], mem_624="E128")      # a missing cell is no binding failure ...
        self.assertEqual(AN.eval_cells(json.loads((ev2 / "eval_run_report.json").read_text()))[1], [])
        ev3 = self._stage("eval", "COMPLETE", drop, mem_624="E128")             # ... unless the eval claims COMPLETE
        self.assertTrue(AN.eval_cells(json.loads((ev3 / "eval_run_report.json").read_text()))[1])


if __name__ == "__main__":
    unittest.main()
