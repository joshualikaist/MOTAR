"""CPU tests for the CAM-K2 v1 training unit (tools/camk2_train.py, aerial_gym/rl_training/rl_games/train_navrl_camk2.sh,
through tools/realveh_entry.py / tools/realveh_python). Nothing trains; no GPU, Isaac Gym or simulator is started.

Safe inside the single-process PUBLIC_CPU suite: the wrapper is loaded by file path under a private name (its
gr_run / kroc2027_eval imports are real modules, as in tests/test_camk1_train.py; sys.path is restored afterwards),
the environment is changed only through unittest.mock.patch, module constants only through mock.patch.object, and
fixture files live in temporary directories. Every negative check is paired with a positive control.
Run: PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B tests/test_camk2_train.py
"""
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import pathlib
import pickle
import re
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import zipfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
RL = ROOT / "aerial_gym/rl_training/rl_games"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


_saved_path = list(sys.path)
try:
    C = _load("camk2_train_under_test", TOOLS / "camk2_train.py")
finally:
    sys.path[:] = _saved_path
_PATH_RESTORED = sys.path == _saved_path                   # checked by TestIsolation (other modules may add later)
E = C.E                                                    # the wrapper's private copy of tools/realveh_entry.py
VR = _load("navrl_vehicle_rand_camk2_test", ROOT / "aerial_gym/task/navrl_task/navrl_vehicle_rand.py")   # stdlib at import
DM = _load("navrl_depth_model_camk2_test", ROOT / "aerial_gym/task/navrl_task/navrl_depth_model.py")    # stdlib at import
Q = _load("quiet_rl_io_camk2_test", RL / "quiet_rl_io.py")      # the real devnull_stdio of rl_games' quiet startup
HEAD = "c" * 40


def exports(text):
    return [ln.strip() for ln in text.splitlines() if re.match(r"\s*(export|unset)\s", ln)]


def literal_exports(text):
    """NAME -> value of every `export NAME=literal` line (no expansion)."""
    out = {}
    for ln in exports(text):
        m = re.match(r"export ([A-Z0-9_]+)=([^\s$\"]+)$", ln)
        if m:
            out[m.group(1)] = m.group(2)
    return out


def launcher_env(seed, text=None):
    """The environment the entry sees for `seed`: the launcher's literal exports, its per-seed case values, what
    train_navrl_v2_search.sh adds that the entry reads (NAVRL_PERCEPTION_PERTURB=0) and the wrapper's receipt."""
    text = text if text is not None else (RL / "train_navrl_camk2.sh").read_text()
    env = literal_exports(text)
    case = re.search(r"^\s*%d\) SOURCE_SHA=\"([0-9a-f]{64})\"; VEH_SEED=(\d+); DEPTH_SEED=(\d+) ;;" % seed, text, re.M)
    env.update(NAVRL_VEH_RAND_SEED=case.group(2), NAVRL_DEPTH_SEED=case.group(3),
               NAVRL_PERCEPTION_PERTURB="0", NAVRL_REALVEH_RECEIPT="/nonexistent/camk2/realveh_receipt.json")
    return env, case.group(1)


# ---------------------------------------------------------------------------------------------------------------------
class TestFixedDesign(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(C.SOURCES[1]["sha256"], "d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb")
        self.assertEqual(C.SOURCES[2]["sha256"], "5128bf2e0461fb45c6b8c4463d4d0a950bb88c0d28923c41d02d99da1102a4b7")
        self.assertEqual((C.SOURCE_EPOCH, C.FINAL_EPOCH, C.SMOKE_EPOCH), (28000, 29000, 28002))
        self.assertEqual(C.TAG % 2, "v2-camk2-ep28000-205bars-main-riskcap-camonly-rv05-s2")
        self.assertEqual((C.MAX_HOURS, C.MIN_FREE_GB, C.BRANCH), (2.5, 1.0, "camonly/dev"))
        self.assertEqual((C.SMOKE_RATE_MARGIN, C.SEED_EPOCHS), (1.25, 1000))
        self.assertEqual(C.ROOT, ROOT)                                                # paths relative to its own tree
        for rel in (C.PREREG, C.ROBOT_CONFIG_REL, C.ROBOT_URDF_REL):
            self.assertFalse(pathlib.Path(rel).is_absolute(), rel)
        self.assertEqual(C.UNIT_DIR, pathlib.Path.home() / "motar_tmp" / "camk2_20261108")
        self.assertEqual(C.VEH_RAND, {"NAVRL_VEH_RAND_TW_NOMINAL": 2.25, "NAVRL_VEH_RAND_TW_SCALE": 0.15, "NAVRL_VEH_RAND_MOTOR_IMBAL": 0.05,
                                      "NAVRL_VEH_RAND_TAU_MIN": 0.03, "NAVRL_VEH_RAND_TAU_MAX": 0.06})
        self.assertEqual(C.DEPTH_MODEL, {"NAVRL_DEPTH_MINZ_M": 0.4, "NAVRL_DEPTH_MAXZ_M": 20.0,
                                         "NAVRL_DEPTH_SIGMA_D_PX": 0.15, "NAVRL_DEPTH_FB_PXM": 66.1875,
                                         "NAVRL_DEPTH_HOLE_P": 0.02})
        for d in (C.VEH_RAND_SEED, C.DEPTH_SEED):
            self.assertNotEqual(d[1], d[2])                                           # its own seed per training seed
        self.assertEqual(len(set(C.VEH_RAND_SEED.values()) | set(C.DEPTH_SEED.values())), 4)
        for k in ("1,000 epochs", "28001..29000", "navrl_quad (robot identity and lineage unchanged)",
                  "thrust-to-weight 2.25", "TW_SCALE 0.15", "MaxZ 20 m", "smoke"):
            self.assertIn(k, C.SCOPE)
        self.assertEqual((C.ROBOT, C.ROBOT_URDF_FILE), ("navrl_quad", "quad_navrl_collide.urdf"))
        self.assertNotIn("rv05_quad", C.ROBOT_CONFIG_REL + C.ROBOT_URDF_REL)

    def test_warm_start_pins_match_the_camk1_copies(self):
        """The pinned SHA-256 equal the files on this host (skipped where the CAM-K1 copies are absent)."""
        for seed, src in C.SOURCES.items():
            if not src["path"].is_file():
                self.skipTest("CAM-K1 checkpoint copies not on this host")
            self.assertEqual(C.source_problems(seed), [])
            self.assertEqual(C.checkpoint_metadata(src["path"])["epoch"], C.SOURCE_EPOCH)

    def test_source_problems_refuse_a_wrong_or_missing_file(self):
        with tempfile.TemporaryDirectory() as d:
            f = pathlib.Path(d) / "ckpt.pth"
            f.write_bytes(b"camk1")
            good = {1: {"path": f, "sha256": hashlib.sha256(b"camk1").hexdigest()}}
            with mock.patch.object(C, "SOURCES", good):
                self.assertEqual(C.source_problems(1), [])                            # positive control
            with mock.patch.object(C, "SOURCES", {1: dict(good[1], sha256="0" * 64)}):
                self.assertIn("SHA mismatch", C.source_problems(1)[0])
            with mock.patch.object(C, "SOURCES", {1: dict(good[1], path=pathlib.Path(d) / "absent.pth")}):
                self.assertIn("missing", C.source_problems(1)[0])


class TestLauncherContract(unittest.TestCase):
    def setUp(self):
        self.k1 = (RL / "train_navrl_camk1.sh").read_text()
        self.k2 = (RL / "train_navrl_camk2.sh").read_text()

    def test_syntax(self):
        for f in (RL / "train_navrl_camk2.sh", TOOLS / "realveh_python"):
            self.assertEqual(subprocess.run(["bash", "-n", str(f)]).returncode, 0, f)

    def test_camk1_contract_kept(self):
        """Every export/unset of CAM-K1's launcher is in CAM-K2's, apart from the lines this lineage changes."""
        changed = ("export MAX_EPOCHS=", "export AERIAL_RUN_TAG=", "export TRAIN_SESSION_LOG=", "export PYTHON=",
                   "export CKPT=", "export NAVRL_CAMTRAIN_RECEIPT=", "export CAMTRAIN_REAL_PYTHON=")
        k2 = set(exports(self.k2))
        missing = [ln for ln in exports(self.k1) if not ln.startswith(changed) and ln not in k2]
        self.assertEqual(missing, [])
        # positive control: the comparator sees a dropped line
        dropped = self.k2.replace("export NAVRL_LEARNING_RATE=5e-6\n", "")
        self.assertEqual([ln for ln in exports(self.k1) if not ln.startswith(changed) and ln not in set(exports(dropped))],
                         ["export NAVRL_LEARNING_RATE=5e-6"])
        self.assertIn("unset NAVRL_GAIN_RAND NAVRL_GAIN_RAND_SCALE NAVRL_GAIN_RAND_SEED", k2)
        self.assertIn("unset NAVRL_CAMTRAIN_RECEIPT CAMTRAIN_REAL_PYTHON", k2)
        self.assertIn("unset NAVRL_ROBOT", " ".join(k2))
        self.assertEqual(self.k1.count("Governor parameters of Policy F"), self.k2.count("Governor parameters of Policy F"))

    def test_camk2_adds_only_its_own_lines(self):
        """The other direction: every export/unset of CAM-K2's launcher is CAM-K1's or one of this unit's."""
        own = ("export MAX_EPOCHS=", "export AERIAL_RUN_TAG=", "export TRAIN_SESSION_LOG=", "export PYTHON=",
               "export CKPT=", "export NAVRL_VEH_RAND", "export NAVRL_DEPTH_", "export NAVRL_REALVEH_RECEIPT=",
               "export REALVEH_REAL_PYTHON=", "unset NAVRL_CAMTRAIN_RECEIPT CAMTRAIN_REAL_PYTHON",
               "unset NAVRL_ROBOT NAVRL_REALVEH_ROBOT NAVRL_REALVEH_WARMSTART_FROM")
        k1 = set(exports(self.k1))
        extra = [ln for ln in exports(self.k2) if ln not in k1 and not ln.startswith(own)]
        self.assertEqual(extra, [])
        added = self.k2.replace("export NAVRL_NUM_BARS=205\n", "export NAVRL_NUM_BARS=205\nexport NAVRL_STRAY=1\n")
        self.assertEqual([ln for ln in exports(added) if ln not in k1 and not ln.startswith(own)],
                         ["export NAVRL_STRAY=1"])                                    # positive control

    def test_lineage_lines_match_the_wrapper(self):
        lit = literal_exports(self.k2)
        full = self.k2.index("export MAX_EPOCHS=%d\n" % C.FINAL_EPOCH)
        gate = self.k2.index('if [[ "${CAMK2_SMOKE:-0}" == "1" ]]; then')
        smoke = self.k2.index("    export MAX_EPOCHS=%d\n" % C.SMOKE_EPOCH)
        self.assertLess(full, gate)                                                   # 29000 unless the smoke gate
        self.assertLess(gate, smoke)
        self.assertEqual(self.k2.count("export MAX_EPOCHS="), 2)
        self.assertIn('export AERIAL_RUN_TAG="v2-camk2-ep28000-205bars-main-riskcap-camonly-rv05-s${TSEED}${TAG_SUFFIX}"',
                      self.k2)
        self.assertIn('export TRAIN_SESSION_LOG="train_session_logs/camk2_s${TSEED}${TAG_SUFFIX}_', self.k2)
        self.assertIn('export PYTHON="${ROOT}/tools/realveh_python"', self.k2)
        self.assertIn('export REALVEH_REAL_PYTHON="${REAL_PYTHON}"', self.k2)
        self.assertIn('export CKPT="${CAMK2_SOURCE_CKPT:?', self.k2)
        self.assertIn('exec ./train_navrl_v2_search.sh --checkpoint "${CKPT}" --branch_run', self.k2)
        for k, v in list(C.VEH_RAND.items()) + list(C.DEPTH_MODEL.items()):
            self.assertEqual(float(lit[k]), v, k)
        for gone in ("NAVRL_REALVEH_ROBOT", "NAVRL_REALVEH_WARMSTART_FROM", "NAVRL_ROBOT"):
            self.assertNotIn(gone, lit)                                               # the robot stays navrl_quad
        self.assertIn("unset NAVRL_ROBOT NAVRL_REALVEH_ROBOT NAVRL_REALVEH_WARMSTART_FROM", self.k2)
        for k, v in (("NAVRL_VEH_RAND", "1"), ("NAVRL_DEPTH_MODEL", "oakd"), ("NAVRL_SENSOR_ABLATION", "no_lidar"),
                     ("NAVRL_NUM_BARS", "205"), ("NAVRL_LEARNING_RATE", "5e-6"), ("NAVRL_V2_PROFILE", "main")):
            self.assertEqual(lit[k], v, k)
        for seed in (1, 2):
            env, sha = launcher_env(seed, self.k2)
            self.assertEqual(sha, C.SOURCES[seed]["sha256"])
            self.assertEqual(int(env["NAVRL_VEH_RAND_SEED"]), C.VEH_RAND_SEED[seed])
            self.assertEqual(int(env["NAVRL_DEPTH_SEED"]), C.DEPTH_SEED[seed])
        self.assertEqual(C.TAG % 1 + "-smoke", "v2-camk2-ep28000-205bars-main-riskcap-camonly-rv05-s1-smoke")

    def test_launcher_settings_are_accepted_by_the_entry_and_equal_the_preregistered_ones(self):
        for seed in (1, 2):
            env, _ = launcher_env(seed)
            s = E.settings(env)
            for key, want in C.expected_settings(seed).items():
                self.assertEqual(s[key], want, (seed, key))
            self.assertAlmostEqual(s["worst_case_thrust_to_weight"], 2.25 * 0.85 * 0.95, places=12)
            self.assertGreaterEqual(s["worst_case_thrust_to_weight"], 1.8)

    def test_thrust_to_weight_floor_and_perturb_refusal_on_the_launcher_env(self):
        env, _ = launcher_env(1)
        self.assertIsNotNone(E.settings(env)["vehicle_rand"])                         # positive control: 1.82
        for tw, im in (("0.2", "0.05"), ("0.15", "0.1")):
            with self.assertRaises(SystemExit, msg=(tw, im)):
                E.settings(dict(env, NAVRL_VEH_RAND_TW_SCALE=tw, NAVRL_VEH_RAND_MOTOR_IMBAL=im))
        with self.assertRaises(SystemExit):
            E.settings(dict(env, NAVRL_PERCEPTION_PERTURB="1"))
        self.assertIsNotNone(E.settings(dict(env, NAVRL_PERCEPTION_PERTURB="0"))["depth_model"])

    def test_launcher_refuses_without_or_with_a_wrong_source(self):
        base = {"PATH": "/usr/bin:/bin", "HOME": os.environ.get("HOME", "/tmp")}
        r = subprocess.run(["bash", str(RL / "train_navrl_camk2.sh"), "3"], capture_output=True, text=True, env=base)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("usage", r.stderr)
        r = subprocess.run(["bash", str(RL / "train_navrl_camk2.sh"), "1"], capture_output=True, text=True, env=base)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("CAMK2_SOURCE_CKPT is required", r.stderr)
        with tempfile.TemporaryDirectory() as d:
            fake = pathlib.Path(d) / "fake.pth"
            fake.write_bytes(b"not camk1")
            r = subprocess.run(["bash", str(RL / "train_navrl_camk2.sh"), "1"], capture_output=True, text=True,
                               env=dict(base, CAMK2_SOURCE_CKPT=str(fake)))
            self.assertNotEqual(r.returncode, 0)
            self.assertIn("source checkpoint SHA mismatch", r.stderr)                 # got past the first guard
            self.assertNotIn("CAMK2_SOURCE_CKPT is required", r.stderr)


# ---------------------------------------------------------------------------------------------------------------------
class TestApprovalAndGates(unittest.TestCase):
    def test_approval(self):
        ok = {"approved_by": "u", "approved_at": "t", "scope": C.SCOPE, "commit": HEAD}
        self.assertEqual(C.approval_problems(ok, HEAD), [])
        self.assertEqual(C.approval_problems(dict(ok, commit=HEAD[:7]), HEAD), [])
        for bad in (dict(ok, commit="d" * 40), dict(ok, scope="CAM-K1 scope"), dict(ok, approved_by=""),
                    {k: v for k, v in ok.items() if k != "approved_at"}):
            self.assertTrue(C.approval_problems(bad, HEAD), bad)
        self.assertTrue(C.approval_problems(None, HEAD))
        self.assertTrue(C.approval_problems(ok, ""))

    def test_smoke_gate(self):
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            self.assertFalse(C.smoke_ok(d, head=HEAD))                                # no record
            for rec, want in (({"verdict": "SMOKE_OK", "development_smoke": True, "commit": HEAD}, True),
                              ({"verdict": "SMOKE_OK", "development_smoke": True, "commit": "0" * 40}, False),
                              ({"verdict": "TRAINING_FAIL", "development_smoke": True, "commit": HEAD}, False),
                              ({"verdict": "SMOKE_OK", "development_smoke": False, "commit": HEAD}, False)):
                (d / "train_report.json").write_text(json.dumps(rec))
                self.assertEqual(C.smoke_ok(d, head=HEAD), want, rec)

    def test_seed_runs_refuse_without_smoke_ok_and_create_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            unit = pathlib.Path(d) / "camk2"
            with mock.patch.object(C, "UNIT_DIR", unit), mock.patch("sys.stdout", new_callable=io.StringIO) as out:
                self.assertEqual(C.main(["1"]), 2)
            self.assertIn("PREFLIGHT_FAIL", out.getvalue())
            self.assertFalse(unit.exists())

    def test_smoke_rate_gate(self):
        self.assertEqual(C.smoke_rate_problems({"epoch_step_times_s": [5.93, 4.77]}), [])   # CAM-K1's smoke: 5.96 ks
        self.assertEqual(C.smoke_rate_problems({"epoch_step_times_s": [9.0, 7.2]}), [])     # exactly 9,000 s
        self.assertTrue(C.smoke_rate_problems({"epoch_step_times_s": [4.0, 7.3]}))          # 9,125 s > 9,000 s
        self.assertTrue(C.smoke_rate_problems({"epoch_step_times_s": []}))
        self.assertTrue(C.smoke_rate_problems(None))
        head = C.R.K.git("rev-parse", "HEAD", root=C.ROOT).strip()
        with tempfile.TemporaryDirectory() as d:
            unit = pathlib.Path(d)
            (unit / "train_smoke").mkdir()
            (unit / "train_smoke" / "train_report.json").write_text(json.dumps(
                {"verdict": "SMOKE_OK", "development_smoke": True, "commit": head, "epoch_step_times_s": [5.0, 7.5]}))
            with mock.patch.object(C, "UNIT_DIR", unit), mock.patch("sys.stdout", new_callable=io.StringIO) as out:
                self.assertEqual(C.main(["1"]), 2)                                    # refused before any preflight
            self.assertIn("exceeds the 2.5 h cap", out.getvalue())
            self.assertFalse((unit / "train_s1").exists())

    def test_argument_refusals(self):
        with self.assertRaises(SystemExit) as cm:
            C.main(["2", "--smoke"])                                                  # the smoke is seed 1 only
        self.assertIn("seed 1 only", str(cm.exception))
        with self.assertRaises(SystemExit) as cm:
            C.main(["1", "--max-hours", "5"])
        self.assertIn("fixed at 2.5", str(cm.exception))
        with mock.patch("sys.stderr", new_callable=io.StringIO) as err, self.assertRaises(SystemExit):
            C.main(["3"])
        self.assertIn("invalid choice", err.getvalue())
        with tempfile.TemporaryDirectory() as d:                                      # positive control: 2.5 parses
            with mock.patch.object(C, "UNIT_DIR", pathlib.Path(d)), mock.patch("sys.stdout", new_callable=io.StringIO):
                self.assertEqual(C.main(["2", "--max-hours", "2.5"]), 2)              # then stops at the smoke gate

    def test_clean_env_strips_treatment_variables(self):
        stray = {"NAVRL_STRAY": "1", "CAMK1_SMOKE": "1", "CAMK2_SMOKE": "1", "CAMTRAIN_X": "1", "REALVEH_X": "1",
                 "S3B_COMMAND_YAW": "ekf", "KEEP_ME": "1"}
        with mock.patch.dict(os.environ, stray):
            env = C.clean_env()
        self.assertFalse([k for k in env if k.startswith(C.ENV_STRIP)])
        self.assertEqual(env.get("KEEP_ME"), "1")
        self.assertNotIn("KEEP_ME", os.environ)                                       # patch restored

    def test_disk_floor_and_fov_line(self):
        self.assertGreaterEqual(C.MIN_FREE_GB, 1.0)
        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        self.assertIn("hfov=%.0fdeg", task)
        self.assertEqual("hfov=%.0fdeg" % 66.0, C.FOV_LOG_LINE)
        self.assertNotEqual("hfov=%.0fdeg" % 87.0, C.FOV_LOG_LINE)


# ---------------------------------------------------------------------------------------------------------------------
# Fixtures for the post-run checks
COMMITTED = {name: hashlib.sha256(name.encode()).hexdigest() for name in E.HASHED}


def quad_identity(committed=COMMITTED):
    """navrl_quad's cfg_robot_* fields as NavRLTask records them (CAM-K1's checkpoints carry these, with the real SHAs)."""
    return {"cfg_robot_name": "navrl_quad", "cfg_robot_config_class": "NavRLQuadCfg",
            "cfg_robot_config_module": "aerial_gym.config.robot_config.navrl_quad_config",
            "cfg_robot_config_file": "navrl_quad_config.py", "cfg_robot_config_path": C.ROBOT_CONFIG_REL,
            "cfg_robot_config_sha256": committed["robot_config"], "cfg_robot_asset_file": "quad_navrl_collide.urdf",
            "cfg_robot_asset_path": C.ROBOT_URDF_REL, "cfg_robot_asset_sha256": committed["robot_urdf"]}


def good_receipt(seed):
    """Built from the real entry's settings(), so its keys are the ones main() writes."""
    env, _ = launcher_env(seed)
    s = E.settings(env)
    return {"settings": s, "task_robot_name": "navrl_quad", "sha256": dict(COMMITTED),
            "vehicle_rand": {"settings": dict(s["vehicle_rand"])}, "depth_model": {"settings": dict(s["depth_model"])},
            "ablation": {"condition": "no_lidar"}, "overrides_after": dict(C.EXPECTED_AFTER),
            "env_navrl": {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_VEH_RAND": "1", "NAVRL_DEPTH_MODEL": "oakd",
                          "NAVRL_PERCEPTION_PERTURB": "0", "NAVRL_REALVEH_RECEIPT": "/x/realveh_receipt.json"}}


def good_record(seed, mass=0.25):
    """The record the real navrl_vehicle_rand.initialize writes, for a stand-in robot with navrl_quad's motor model
    values (4 x 2 N), the controller's mass and the sim gravity."""
    import types
    import torch
    n = 3
    mm = types.SimpleNamespace(num_envs=n, num_motors_per_robot=4, max_thrust=torch.full((n, 4), 2.0),
                               min_thrust=torch.zeros(n, 4), current_motor_thrust=torch.full((n, 4), 0.5),
                               motor_time_constants_increasing=torch.full((n, 4), 0.04),
                               motor_time_constants_decreasing=torch.full((n, 4), 0.04))
    robot = types.SimpleNamespace(num_envs=n, control_allocator=types.SimpleNamespace(motor_model=mm),
                                  controller=types.SimpleNamespace(mass=torch.full((n, 1), mass),
                                                                   gravity=torch.tensor([0.0, 0.0, -9.81]).expand(n, -1)))
    state = torch.get_rng_state()
    with contextlib.redirect_stderr(io.StringIO()):
        record = VR.initialize(robot, C.expected_settings(seed)["vehicle_rand"])
    assert torch.equal(state, torch.get_rng_state())
    return json.loads(json.dumps(record))


def real_telemetry(seed, frames=None, hole_p=None, quiet=False, late_binding=False):
    """The lines the real modules log, produced in-process by their own functions on stand-ins: vehicle-rand install,
    init and three resets for a 128-env navrl_quad-like robot; depth-model install, first and summary over
    STATS_CALLS frames of a bar scene (one near column, so MinZ triggers).
    quiet: run initialize() and the first 3 depth frames inside quiet_rl_io.devnull_stdio(), as rl_games' quiet
    construction does (review r2 N-1). late_binding: drop the streams bound at install, i.e. the pre-r2 pattern."""
    import types
    import torch
    err = io.StringIO()
    vs = C.expected_settings(seed)["vehicle_rand"]
    ds = dict(C.expected_settings(seed)["depth_model"])
    if hole_p is not None:
        ds["NAVRL_DEPTH_HOLE_P"] = hole_p
    n = 128
    rng = torch.get_rng_state()
    with contextlib.redirect_stderr(err):
        MM = type("MM", (), {"update_motor_thrusts": lambda self, r: r})
        VR.install(vs, multirotor_cls=type("MR", (), {"init_tensors": lambda self: None, "reset_idx": lambda self, e: None}),
                   motor_model_cls=MM)
        mm = types.SimpleNamespace(num_envs=n, num_motors_per_robot=4, max_thrust=torch.full((n, 4), 2.0),
                                   min_thrust=torch.zeros(n, 4), current_motor_thrust=torch.full((n, 4), 0.5),
                                   motor_time_constants_increasing=torch.full((n, 4), 0.04),
                                   motor_time_constants_decreasing=torch.full((n, 4), 0.04))
        robot = types.SimpleNamespace(num_envs=n, control_allocator=types.SimpleNamespace(motor_model=mm),
                                      controller=types.SimpleNamespace(mass=torch.full((n, 1), 0.25),
                                                                       gravity=torch.tensor([0.0, 0.0, -9.81]).expand(n, -1)))
        saved = (VR._STREAM["stream"], DM._STREAM["stream"])
        quiet_ctx = Q.devnull_stdio if quiet else contextlib.nullcontext
        try:
            if late_binding:
                VR._STREAM["stream"] = None
            with quiet_ctx():
                VR.initialize(robot, vs)
            for ids in (torch.arange(n), torch.tensor([3, 7]), torch.tensor([1])):
                VR.redraw(robot, ids, vs)
                for _ in range(5):                                                    # motor updates between resets
                    MM.update_motor_thrusts(mm, torch.full((n, 4), 1.0))
        finally:
            VR._STREAM["stream"] = saved[0]
        mod = types.SimpleNamespace(NavRLPerceptionModule=type("P", (), {
            "observe": lambda self, rgb, depth, lidar_m: None}))
        DM.install_perception(mod, ds)
        saved_dm = DM._STREAM["stream"]
        if late_binding:
            DM._STREAM["stream"] = None
        h, w, ne = 90, 160, 4
        perc = types.SimpleNamespace(num_envs=ne, height=h, width=w, cx=(w - 1) / 2, cy=(h - 1) / 2,
                                     fx=w / (2 * 0.6494075932), fy=h / (2 * 0.5095254495), max_camera_range=20.0,
                                     camera_obstacle_max_range=10.0, detect_decoupled=False)
        g = torch.Generator().manual_seed(seed)
        for i in range(frames or DM.STATS_CALLS):
            depth = torch.full((ne, h, w), 10.0)
            for e in range(ne):
                c = int(torch.randint(0, w - 8, (1,), generator=g))
                depth[e, :, c:c + 8] = float(torch.empty(1).uniform_(1.0, 9.0, generator=g))
            depth[:, :, 0:3] = 0.3                                                    # a bar under MinZ
            depth[:, 40:44, 80:84] = 15.0                                             # a far target blob
            with (quiet_ctx() if i < 3 else contextlib.nullcontext()):
                DM.apply(perc, depth, ds)
        DM._STREAM["stream"] = saved_dm
    assert torch.equal(rng, torch.get_rng_state())
    return err.getvalue()


def write_checkpoint(path, epoch, identity, torch_save=True):
    """A torch zip checkpoint like rl_games writes (tensors, an OrderedDict, env_state)."""
    import collections
    import torch
    state = {"epoch": epoch, "model": collections.OrderedDict(w=torch.zeros(3)), "optimizer": {"lr": 5e-6},
             "env_state": dict(identity, n_bars_active=205, k_max_cur=torch.tensor(1.0), cfg_lidar_hbeams=72)}
    torch.save(state, str(path))


class TestReceiptAndEventChecks(unittest.TestCase):
    def test_good_receipt_passes(self):
        for seed in (1, 2):
            self.assertEqual(C.receipt_problems(good_receipt(seed), seed, COMMITTED), [])

    def test_each_deviation_is_refused(self):
        r = good_receipt(1)
        self.assertTrue(C.receipt_problems(good_receipt(2), 1, COMMITTED))           # other seed's settings
        self.assertTrue(C.receipt_problems(None, 1, COMMITTED))
        mutations = []
        for key, value in (("NAVRL_VEH_RAND_TW_SCALE", 0.2), ("NAVRL_VEH_RAND_SEED", 1),
                           ("NAVRL_VEH_RAND_TW_NOMINAL", 3.262)):
            m = json.loads(json.dumps(r))
            m["settings"]["vehicle_rand"][key] = value
            mutations.append(m)
        for key, value in (("NAVRL_DEPTH_MAXZ_M", 10.0), ("NAVRL_DEPTH_HOLE_P", 0.0)):
            m = json.loads(json.dumps(r))
            m["settings"]["depth_model"][key] = value
            mutations.append(m)
        for path, value in ((("settings", "camtrain", "NAVRL_CAMTRAIN_HFOV_DEG"), 87.0),
                            (("settings", "worst_case_thrust_to_weight"), 1.7), (("task_robot_name",), "navrl_rv05_quad"),
                            (("vehicle_rand",), None), (("depth_model", "settings", "NAVRL_DEPTH_SEED"), 1),
                            (("ablation", "condition"), "full"), (("overrides_after", "yaw_align_weight"), 0.3),
                            (("sha256", "depth_model"), "0" * 64), (("env_navrl", "NAVRL_ROBOT"), "navrl_rv05_quad"),
                            (("env_navrl", "NAVRL_REALVEH_ROBOT"), "navrl_rv05_quad"),
                            (("env_navrl", "NAVRL_PERCEPTION_PERTURB"), "1"), (("env_navrl", "NAVRL_VEH_RAND"), "0")):
            m = json.loads(json.dumps(r))
            node = m
            for k in path[:-1]:
                node = node[k]
            node[path[-1]] = value
            mutations.append(m)
        m = json.loads(json.dumps(r))
        del m["sha256"]["navrl_task"]
        mutations.append(m)
        for i, m in enumerate(mutations):
            self.assertTrue(C.receipt_problems(m, 1, COMMITTED), i)
        self.assertEqual(len(mutations), 18)
        self.assertEqual(C.receipt_problems(dict(r, env_navrl=dict(r["env_navrl"], NAVRL_ROBOT="navrl_quad")), 1,
                                            COMMITTED), [])                          # control: explicit navrl_quad

    def test_committed_sha256_reads_head(self):
        got = C.committed_sha256()
        self.assertEqual(set(got), set(E.HASHED))
        rel = C.hashed_relpaths()["camtrain_entry"]                                  # committed and unchanged
        self.assertEqual(got["camtrain_entry"], hashlib.sha256((ROOT / rel).read_bytes()).hexdigest())
        self.assertIsNotNone(got["robot_config"])                                    # navrl_quad's config is tracked
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", C.hashed_relpaths()["vehicle_rand"]],
                          capture_output=True).returncode:
            self.assertIsNone(got["vehicle_rand"])                                    # not committed -> no value
            self.assertTrue(C.receipt_problems(dict(good_receipt(1), sha256=got), 1, got))

    def test_vehicle_rand_record_checks(self):
        for seed in (1, 2):
            rec = good_record(seed)
            self.assertEqual(C.veh_rand_record_problems(rec, seed), [])
            self.assertAlmostEqual(rec["cap_per_motor_n"][0], 2.25 * 0.25 * 9.81 / 4, places=6)   # 1.3795 N
            self.assertTrue(rec["cap_applied"])
        self.assertTrue(C.veh_rand_record_problems(None, 1))
        self.assertTrue(C.veh_rand_record_problems(good_record(2), 1))               # other seed
        self.assertTrue(C.veh_rand_record_problems(dict(good_record(1), mass_kg=[0.5, 0.5]), 1))   # not navrl_quad's
        with self.assertRaises(RuntimeError):                                         # 4 x 2 N at 0.5 kg is T/W 1.63
            good_record(1, mass=0.5)
        self.assertEqual(C.veh_rand_record_problems(good_record(1, mass=0.2505), 1), [])   # control: within range
        for key, value in (("cap_applied", False), ("tw_nominal", 3.262), ("cap_per_motor_n", [1.3, 1.3]),
                           ("gravity_mps2", [9.0, 9.0]), ("max_thrust_per_motor_n", [2.759, 2.759]),
                           ("num_motors", 6)):
            bad = dict(good_record(1), **{key: value})
            self.assertTrue(C.veh_rand_record_problems(bad, 1), key)
        bad = good_record(1)
        del bad["mass_kg"]
        self.assertTrue(C.veh_rand_record_problems(bad, 1))


class TestCheckpointReader(unittest.TestCase):
    def test_reads_epoch_and_robot_without_torch(self):
        with tempfile.TemporaryDirectory() as d:
            ck = pathlib.Path(d) / "last_gen_ppo_ep_29000_rew_1.0.pth"
            write_checkpoint(ck, 29000, quad_identity())
            code = ("import importlib.util, json, sys\n"
                    "sys.path.insert(0, sys.argv[3])\n"
                    "spec = importlib.util.spec_from_file_location('c', sys.argv[1])\n"
                    "c = importlib.util.module_from_spec(spec); spec.loader.exec_module(c)\n"
                    "m = c.checkpoint_metadata(sys.argv[2])\n"
                    "print(json.dumps({'torch': 'torch' in sys.modules, 'meta': m}))\n")
            r = subprocess.run([sys.executable, "-B", "-c", code, str(TOOLS / "camk2_train.py"), str(ck), str(TOOLS)],
                               capture_output=True, text=True, timeout=120,
                               env={"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1", "CUDA_VISIBLE_DEVICES": ""})
            self.assertEqual(r.returncode, 0, r.stderr[-2000:])
            out = json.loads(r.stdout.strip().splitlines()[-1])
            self.assertFalse(out["torch"])
            self.assertEqual(out["meta"]["epoch"], 29000)
            self.assertEqual(out["meta"]["env_state"]["cfg_robot_name"], "navrl_quad")
            self.assertEqual(out["meta"]["env_state"]["n_bars_active"], 205)
            self.assertNotIn("k_max_cur", out["meta"]["env_state"])                   # a tensor: not a scalar field
            meta = C.checkpoint_metadata(ck)
            self.assertEqual(C.checkpoint_problems(meta, 29000, COMMITTED), [])
            self.assertTrue(C.checkpoint_problems(meta, 28002, COMMITTED))            # wrong epoch
            ck2 = pathlib.Path(d) / "quad.pth"
            rv05 = dict(quad_identity(), cfg_robot_name="navrl_rv05_quad")
            write_checkpoint(ck2, 29000, rv05)
            self.assertTrue(C.checkpoint_problems(C.checkpoint_metadata(ck2), 29000, COMMITTED))
            ck3 = pathlib.Path(d) / "sha.pth"
            write_checkpoint(ck3, 29000, dict(quad_identity(), cfg_robot_asset_sha256="0" * 64))
            self.assertTrue(C.checkpoint_problems(C.checkpoint_metadata(ck3), 29000, COMMITTED))

    def test_real_camk1_source_reads_as_navrl_quad(self):
        src = C.SOURCES[1]["path"]
        if not src.is_file():
            self.skipTest("CAM-K1 checkpoint copies not on this host")
        meta = C.checkpoint_metadata(src)
        self.assertEqual(meta["env_state"]["cfg_robot_name"], "navrl_quad")
        # real-data positive control: CAM-K1's source passes the robot check against this tree's committed navrl_quad
        # files, which is why the restore needs no lineage bypass; only the epoch differs from CAM-K2's final one
        committed = C.committed_sha256()
        self.assertEqual(C.checkpoint_problems(meta, 28000, committed), [])
        self.assertTrue(C.checkpoint_problems(meta, 29000, committed))

    def test_reader_runs_no_code_from_the_file(self):
        with tempfile.TemporaryDirectory() as d:
            marker = pathlib.Path(d) / "executed"
            payload = (b"cos\nsystem\n(S'touch " + str(marker).encode() + b"'\ntR.")
            ck = pathlib.Path(d) / "evil.pth"
            with zipfile.ZipFile(str(ck), "w") as z:
                z.writestr("evil/data.pkl", payload)
            with self.assertRaises(ValueError):                                       # root is not a dict
                C.checkpoint_metadata(ck)
            self.assertFalse(marker.exists())
            # control: the payload does name a real callable, which a plain Unpickler would resolve (not called)
            self.assertIs(pickle.Unpickler(io.BytesIO(payload)).find_class("os", "system"), os.system)


TELEMETRY = {1: real_telemetry(1), 2: real_telemetry(2)}


class TestQuietStartup(unittest.TestCase):
    """Review r2 N-1: rl_games builds the env (init_tensors -> initialize) inside quiet_rl_io.devnull_stdio()."""

    def test_lines_survive_the_real_devnull_stdio(self):
        for seed in (1, 2):
            text = real_telemetry(seed, quiet=True)
            self.assertEqual(C.telemetry_problems([text], seed)[0], [], seed)
            events = [json.loads(ln[len(VR.LOG_PREFIX):])["event"] for ln in text.splitlines()
                      if ln.startswith(VR.LOG_PREFIX)]
            self.assertIn("init", events)
            self.assertIn('"event": "first"', text)

    def test_negative_control_late_binding_loses_them(self):
        problems = C.telemetry_problems([real_telemetry(1, quiet=True, late_binding=True)], 1)[0]
        self.assertIn("veh-rand: no init line", problems)
        self.assertIn("depth-model: no first-frame line", problems)
        self.assertEqual(C.telemetry_problems([real_telemetry(1, late_binding=True)], 1)[0], [])   # not quiet: fine
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            with Q.devnull_stdio():
                sys.stderr.write("lost\n")                                           # the mechanism itself
            sys.stderr.write("kept\n")
        self.assertEqual(buf.getvalue(), "kept\n")
        self.assertIsNot(sys.stderr, None)


class TestTelemetryChecks(unittest.TestCase):
    """telemetry_problems() on the lines the real module functions log (positive), then one deviation at a time."""

    def test_real_lines_pass(self):
        for seed in (1, 2):
            problems, details = C.telemetry_problems([TELEMETRY[seed]], seed)
            self.assertEqual(problems, [], seed)
            s = details["depth_model"]["summary"][0]
            self.assertGreater(s["invalid_minz"], 0.0)                                # the near column triggered MinZ
            self.assertLess(abs(s["invalid_hole"] - 0.02), 0.005)
            self.assertGreater(s["mean_abs_noise_m"], 0.0)
            self.assertEqual(details["veh_rand"]["reset_lines"], 3)
            self.assertEqual(details["veh_rand"]["motor_updates_max"], 10)                # 5 updates after resets 1, 2
        self.assertTrue(C.telemetry_problems([TELEMETRY[1]], 2)[0])                    # seeds differ -> settings
        # N-7: without a motor-update count > 0 (no update line, counters 0) the check fails
        text = "\n".join(ln for ln in TELEMETRY[1].splitlines() if '"event": "update"' not in ln)
        text = text.replace('"motor_updates": 5', '"motor_updates": 0').replace('"motor_updates": 10', '"motor_updates": 0')
        self.assertIn("veh-rand: no motor-update count > 0: the cap and factors are not shown to have been applied",
                      C.telemetry_problems([text], 1)[0])

    def test_each_missing_or_implausible_line_fails(self):
        text = TELEMETRY[1]
        lines = text.splitlines()

        def without(pred):
            return "\n".join(ln for ln in lines if not pred(ln)) + "\n"

        cases = {
            "no veh install": without(lambda ln: '"event": "install"' in ln and ln.startswith(VR.LOG_PREFIX)),
            "no veh init": without(lambda ln: '"event": "init"' in ln),
            "no veh reset": without(lambda ln: '"event": "reset"' in ln),
            "no depth install": without(lambda ln: '"event": "install"' in ln and ln.startswith(DM.LOG_PREFIX)),
            "no depth first": without(lambda ln: '"event": "first"' in ln),
            "no depth summary": without(lambda ln: '"event": "summary"' in ln),
            "too few frames": real_telemetry(1, frames=DM.STATS_CALLS - 1),
            "hole share off": real_telemetry(1, hole_p=0.1),
            "nothing": "",
        }
        for name, bad in cases.items():
            self.assertTrue(C.telemetry_problems([bad], 1)[0], name)
        # a factor outside the bounds and a noise-free summary are implausible
        vline = [ln for ln in lines if '"event": "reset"' in ln][0]
        obj = json.loads(vline[len(VR.LOG_PREFIX):])
        obj["factor_max"] = 1.5
        self.assertTrue(C.telemetry_problems([text.replace(vline, VR.LOG_PREFIX + json.dumps(obj))], 1)[0])
        sline = [ln for ln in lines if '"event": "summary"' in ln][0]
        obj = json.loads(sline[len(DM.LOG_PREFIX):])
        obj["mean_abs_noise_m"] = 0.0
        self.assertTrue(C.telemetry_problems([text.replace(sline, DM.LOG_PREFIX + json.dumps(obj))], 1)[0])

    def test_crash_causes_and_step_times(self):
        k1 = pathlib.Path.home() / "motar_tmp/camk1_20261106/train_s1/launcher.log"
        if k1.is_file():                                                              # real-data positive control
            text = k1.read_text(errors="replace")
            cc = C.crash_causes([text])
            self.assertEqual(cc["windows"], 115)
            self.assertAlmostEqual(cc["last"]["bar_contact"], 0.999)
            self.assertEqual(len(C.epoch_step_times([text])), 3000)
        self.assertEqual(C.crash_causes(["no such lines"]), "NOT_RECORDED")
        self.assertEqual(C.epoch_step_times(["  step time (s)  : 4.7705\nx\n  step time (s)  : 5.0\n"]), [4.7705, 5.0])


class TestPostRunChecks(unittest.TestCase):
    """post_run_problems() on a fixture tree: one passing case, then one deviation at a time."""

    def build(self, d, seed=1, smoke=False, **change):
        d = pathlib.Path(d)
        rl, out = d / "rl", d / "out"
        tag = C.TAG % seed + ("-smoke" if smoke else "")
        epoch = C.SMOKE_EPOCH if smoke else C.FINAL_EPOCH
        old = rl / "runs" / "ppo_261107_1346_navrl_older"
        old.mkdir(parents=True)
        new = rl / "runs" / ("ppo_261108_1400_navrl_" + change.get("tag", tag))
        (new / "nn").mkdir(parents=True)
        ident = change.get("identity", quad_identity())
        write_checkpoint(new / "nn" / ("last_gen_ppo_ep_%d_rew_1.0.pth" % change.get("epoch", epoch)),
                         change.get("epoch", epoch), ident)
        if change.get("second_run"):
            (rl / "runs" / "ppo_261108_1401_navrl_other").mkdir()
        out.mkdir()
        (out / "source_checkpoint").mkdir()
        src_bytes = b"camk1 source"
        copy = out / "source_checkpoint" / C.SOURCES[seed]["path"].name
        copy.write_bytes(change.get("copy_bytes", src_bytes))
        sources = {seed: {"path": C.SOURCES[seed]["path"], "sha256": hashlib.sha256(src_bytes).hexdigest()}}
        receipt = change.get("receipt", good_receipt(seed))
        (out / "realveh_receipt.json").write_text(json.dumps(receipt))
        if not change.get("no_record"):
            record = change.get("record", good_record(seed))
            (out / ("realveh_receipt.json" + E.VEH_RAND_RECORD_SUFFIX)).write_text(json.dumps(record))
        steps = "".join("  step time (s)  : %.4f\n" % x for x in change.get("step_times", [5.9, 4.8]))
        crash = change.get("crash", "[33;20m[1 ms][navrl_task] - WARNING : NavRL crashdiag | bar_contact=0.900 "
                                    "(mean_x=20.0m steps=50) below=0.080 (steps=9 tilt=30deg) above=0.000 oob=0.020 "
                                    "[W=0 E=1 S=0 N=0 steps=40] (n_crash=1500) (navrl_task.py:10015)\n")
        tele = change.get("telemetry", TELEMETRY[seed])
        (out / "launcher.log").write_text("x\n" + change.get("log", "NavRL perception mode | hfov=66deg\n") + tele + steps
                                          + crash)
        (rl / "train_session_logs").mkdir()
        (rl / "train_session_logs" / ("camk2_s%d%s_261108_140000_1.log" % (seed, "-smoke" if smoke else ""))).write_text("log\n")
        return rl, out, sources, {"old": {old.name}}

    def check(self, seed=1, smoke=False, rc=0, end=None, **change):
        start = {"fault_count": 0, "boot_id": "b", "online": "0-3,6-23"}
        with tempfile.TemporaryDirectory() as d:
            rl, out, sources, before = self.build(d, seed, smoke, **change)
            with mock.patch.object(C, "RL", rl), mock.patch.object(C, "SOURCES", sources):
                problems, host_ok, details = C.post_run_problems(out, seed, smoke, before["old"], COMMITTED, rc, start,
                                                                 end or dict(start), head=HEAD)
                copied = (out / "checkpoint").is_dir() and len(list((out / "checkpoint").glob("*.pth")))
                self.assertEqual(list((out / "checkpoint").glob("*.manifest.json")) if copied else [], [])   # not yet
                if details.get("checkpoint"):                                         # main() writes it after the verdict
                    C.write_manifest(pathlib.Path(details["checkpoint"]["copy"]), seed, HEAD, details, out,
                                     "TRAINED" if not problems else "TRAINING_FAIL")
                manifests = sorted((out / "checkpoint").glob("*.manifest.json")) if copied else []
                details["_manifest"] = json.loads(manifests[0].read_text()) if manifests else None
        return problems, host_ok, details, copied

    def test_passing_fixture(self):
        for seed, smoke in ((1, False), (2, False), (1, True)):
            problems, host_ok, details, copied = self.check(seed, smoke)
            self.assertEqual(problems, [], (seed, smoke))
            self.assertTrue(host_ok)
            self.assertEqual(copied, 1)
            self.assertEqual(details["checkpoint_robot"]["cfg_robot_name"], "navrl_quad")
            self.assertTrue(details["fov_log_line_found"])
            self.assertEqual(details["epoch_step_times_s"], [5.9, 4.8])
            self.assertEqual(details["crash_causes"]["last"], {"bar_contact": 0.9, "below": 0.08, "above": 0.0,
                                                               "oob": 0.02, "n_crash": 1500})
            m = details["_manifest"]
            self.assertEqual(m["treatments"], C.expected_settings(seed))
            self.assertEqual(m["checkpoint_sha256"], details["checkpoint"]["copy_sha256"])
            self.assertEqual(m["robot"], "navrl_quad")
            self.assertEqual(m["commit"], HEAD)
            self.assertEqual(m["verdict"], "TRAINED")
            self.assertEqual(len(m["realveh_receipt_sha256"]), 64)
            self.assertEqual(details["telemetry"]["veh_rand"]["reset_lines"], 3)
        problems, _, details, _ = self.check(crash="")                                 # crash lines are descriptive
        self.assertEqual(problems, [])
        self.assertEqual(details["crash_causes"], "NOT_RECORDED")

    def test_each_deviation_fails(self):
        cases = {
            "host": dict(end={"fault_count": 1, "boot_id": "b", "online": "0-3,6-23"}),
            "exit": dict(rc=124),
            "two_runs": dict(second_run=True),
            "wrong_tag": dict(tag="v2-camk1-ep25000-205bars-main-riskcap-camonly-s1"),
            "wrong_epoch": dict(epoch=28500),
            "robot_rv05": dict(identity=dict(quad_identity(), cfg_robot_name="navrl_rv05_quad")),
            "robot_sha": dict(identity=dict(quad_identity(), cfg_robot_config_sha256="0" * 64)),
            "no_fov": dict(log="hfov=87deg\n"),
            "no_record": dict(no_record=True),
            "record_uncapped": dict(record=dict(good_record(1), cap_applied=False)),
            "copy_changed": dict(copy_bytes=b"modified"),
            "receipt": dict(receipt=dict(good_receipt(1), task_robot_name="navrl_rv05_quad")),
            "no_telemetry": dict(telemetry=""),
            "other_seed_telemetry": dict(telemetry=TELEMETRY[2]),
            "no_step_time": dict(step_times=[]),
        }
        for name, change in cases.items():
            problems, host_ok, _, _ = self.check(**change)
            self.assertTrue(problems, name)
            self.assertEqual(host_ok, name != "host", name)
        problems, _, _, _ = self.check(seed=1, smoke=True, epoch=C.FINAL_EPOCH)       # smoke needs 28002
        self.assertTrue(any("28002" in p for p in problems))


class TestIsolation(unittest.TestCase):
    def test_new_files_only_and_camk1_untouched(self):
        for rel in ("tools/camk1_train.py", "aerial_gym/rl_training/rl_games/train_navrl_camk1.sh",
                    "tests/test_camk1_train.py", "tools/camtrain_entry.py", "tools/camtrain_python"):
            r = subprocess.run(["git", "-C", str(ROOT), "diff", "--quiet", "HEAD", "--", rel])
            self.assertEqual(r.returncode, 0, rel)

    def test_no_lasting_path_or_env_change(self):
        self.assertTrue(_PATH_RESTORED)
        self.assertNotIn("CAMK2_SOURCE_CKPT", os.environ)
        self.assertNotIn("camk2_train_under_test", sys.modules)


if __name__ == "__main__":
    unittest.main()
