"""CPU tests for the CAM-K1 camera-only continuation training (tools/camtrain_entry.py, tools/camtrain_python,
tools/camk1_train.py, aerial_gym/rl_training/rl_games/train_navrl_camk1.sh). Nothing trains; no GPU, Isaac Gym or
simulator is started, and the real task configuration is never modified."""
import importlib.util
import pathlib
import re
import subprocess
import sys
import types
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
RL = ROOT / "aerial_gym/rl_training/rl_games"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


E = _load("camtrain_entry_under_test", TOOLS / "camtrain_entry.py")
sys.path.insert(0, str(TOOLS))
T = _load("camk1_train_under_test", TOOLS / "camk1_train.py")

GOOD = {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_CAMTRAIN_HFOV_DEG": "66", "NAVRL_CAMTRAIN_VFOV_DEG": "54",
        "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03",
        "NAVRL_CAMTRAIN_RECEIPT": "/tmp/x.json"}


class TestSettings(unittest.TestCase):
    def test_valid(self):
        s = E.settings(dict(GOOD))
        self.assertEqual((s["NAVRL_CAMTRAIN_HFOV_DEG"], s["NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT"]), (66.0, 0.6))

    def test_refusals(self):
        for change in ({"NAVRL_CAMTRAIN_HFOV_DEG": ""}, {"NAVRL_CAMTRAIN_RECEIPT": ""},
                       {"NAVRL_CAMTRAIN_VFOV_DEG": "wide"}, {"NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.5"},
                       {"NAVRL_CAMTRAIN_HFOV_DEG": "150"}, {"NAVRL_SENSOR_ABLATION": "full"},
                       {"NAVRL_SENSOR_ABLATION": ""}):
            with self.assertRaises(SystemExit, msg=change):
                E.settings(dict(GOOD, **change))


def fake_config():
    return types.SimpleNamespace(
        vision=types.SimpleNamespace(detector_hfov_deg=87.0, detector_vfov_deg=58.0, visibility_bonus=0.02),
        reward_parameters={"yaw_align_weight": 0.3, "capture_bonus": 30.0})


class TestOverrides(unittest.TestCase):
    def test_apply(self):
        c = fake_config()
        before, after = E.apply_overrides(c, E.settings(dict(GOOD)))
        self.assertEqual(before, {"detector_hfov_deg": 87.0, "detector_vfov_deg": 58.0, "yaw_align_weight": 0.3,
                                  "visibility_bonus": 0.02})
        self.assertEqual(after, T.EXPECTED_AFTER)
        self.assertEqual(c.reward_parameters["capture_bonus"], 30.0)            # nothing else changes

    def test_real_config_fields_hold_f_values(self):
        """The overridden names exist in the real configuration with Policy F's values. Read as text: importing the
        module in the shared PUBLIC_CPU process could pick up another test's stubs (AGENTS.md known pitfalls)."""
        src = (ROOT / "aerial_gym/config/task_config/navrl_task_config.py").read_text()
        for needle in ("detector_hfov_deg = 87.0", "detector_vfov_deg = 58.0", "visibility_bonus = 0.02",
                       '"yaw_align_weight": 0.3,'):
            self.assertEqual(src.count(needle), 1, needle)
        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        self.assertIn('self.rw["yaw_align_weight"] * misalign * align_gate', task)     # read from reward_parameters
        self.assertIn("self.vis_cfg.visibility_bonus", task)                           # read from task_config.vision


class TestReceiptAndApproval(unittest.TestCase):
    def test_receipt(self):
        ok = {"ablation": {"condition": "no_lidar"}, "overrides_after": dict(T.EXPECTED_AFTER)}
        self.assertEqual(T.receipt_problems(ok), [])
        self.assertTrue(T.receipt_problems(None))
        self.assertTrue(T.receipt_problems(dict(ok, ablation={"condition": "full"})))
        self.assertTrue(T.receipt_problems(dict(ok, overrides_after=dict(T.EXPECTED_AFTER, yaw_align_weight=0.3))))

    def test_approval(self):
        head = "c" * 40
        ok = {"approved_by": "u", "approved_at": "t", "scope": T.SCOPE, "commit": head}
        self.assertEqual(T.approval_problems(ok, head), [])
        self.assertTrue(T.approval_problems(dict(ok, commit="d" * 40), head))
        self.assertTrue(T.approval_problems(dict(ok, scope="x"), head))
        self.assertTrue(T.approval_problems(dict(ok, approved_by=""), head))


class TestSmokeGate(unittest.TestCase):
    def test_smoke_ok(self):
        import json
        import tempfile
        head = T.R.K.git("rev-parse", "HEAD", root=T.ROOT).strip()
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            self.assertFalse(T.smoke_ok(d))                                     # no record
            for rec, want in (({"verdict": "SMOKE_OK", "development_smoke": True, "commit": head}, True),
                              ({"verdict": "SMOKE_OK", "development_smoke": True, "commit": "0" * 40}, False),
                              ({"verdict": "TRAINING_FAIL", "development_smoke": True, "commit": head}, False),
                              ({"verdict": "SMOKE_OK", "development_smoke": False, "commit": head}, False)):
                (d / "train_report.json").write_text(json.dumps(rec))
                self.assertEqual(T.smoke_ok(d), want, rec)

    def test_smoke_runs_seed_one_only(self):
        with self.assertRaises(SystemExit):
            T.main(["2", "--smoke"])


def exports(text):
    return [ln.strip() for ln in text.splitlines() if re.match(r"\s*(export|unset)\s", ln)]


class TestLauncherContract(unittest.TestCase):
    def setUp(self):
        self.cam = (RL / "train_navrl_camk1.sh").read_text()
        self.gr1 = (RL / "train_navrl_gr1.sh").read_text()

    def test_syntax(self):
        for f in (RL / "train_navrl_camk1.sh", TOOLS / "camtrain_python"):
            self.assertEqual(subprocess.run(["bash", "-n", str(f)]).returncode, 0, f)

    def test_gr1_ctrl_contract_kept(self):
        """Every export/unset of GR-1 ctrl's launcher is in CAM-K1's, apart from the lines this lineage changes."""
        changed = ("export SEED=", "export MAX_EPOCHS=", "export AERIAL_RUN_TAG=", "export TRAIN_SESSION_LOG=",
                   "export PYTHON=", "export PATH=", "export NAVRL_GAIN_RAND=1")
        cam = set(exports(self.cam))
        missing = [ln for ln in exports(self.gr1) if not ln.startswith(changed) and ln not in cam]
        self.assertEqual(missing, [])
        self.assertIn("unset NAVRL_GAIN_RAND NAVRL_GAIN_RAND_SCALE NAVRL_GAIN_RAND_SEED", cam)
        self.assertNotIn("export NAVRL_GAIN_RAND=1", " ".join(cam))

    def test_lineage_lines_match_the_wrapper(self):
        self.assertIn("export MAX_EPOCHS=%d" % T.FINAL_EPOCH, self.cam)
        self.assertIn('export AERIAL_RUN_TAG="v2-camk1-ep25000-205bars-main-riskcap-camonly-s${TSEED}${TAG_SUFFIX}"',
                      self.cam)
        self.assertIn('if [[ "${CAMK1_SMOKE:-0}" == "1" ]]; then', self.cam)
        self.assertIn("export MAX_EPOCHS=%d" % T.SMOKE_EPOCH, self.cam)
        self.assertEqual(T.TAG % 1, "v2-camk1-ep25000-205bars-main-riskcap-camonly-s1")
        for k, v in (("NAVRL_SENSOR_ABLATION", "no_lidar"), ("NAVRL_CAMTRAIN_HFOV_DEG", "66"),
                     ("NAVRL_CAMTRAIN_VFOV_DEG", "54"), ("NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT", "0.6"),
                     ("NAVRL_CAMTRAIN_VISIBILITY_BONUS", "0.03")):
            self.assertIn("export %s=%s" % (k, v), self.cam)
            if k in E.LIMITS:
                self.assertEqual(E.settings(dict(GOOD))[k], float(v))
        self.assertIn('export PYTHON="${ROOT}/tools/camtrain_python"', self.cam)
        self.assertIn("F_SHA", T.__dict__)
        self.assertIn(T.F_SHA, self.cam)

    def test_wrapper_requires_real_python(self):
        r = subprocess.run(["bash", str(TOOLS / "camtrain_python"), "runner.py"], capture_output=True, text=True,
                           env={"PATH": "/usr/bin:/bin"})
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("CAMTRAIN_REAL_PYTHON", r.stderr)

    def test_entry_refuses_other_scripts(self):
        import os
        old = {k: os.environ.get(k) for k in GOOD}
        os.environ.update(GOOD)
        saved = sys.argv
        try:
            sys.argv = ["camtrain_entry.py", "/tmp/evil.py"]
            with self.assertRaises(SystemExit) as cm:
                E.main()
            self.assertIn("only runner.py", str(cm.exception))
        finally:
            sys.argv = saved
            for k, v in old.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v


class TestPostRunGuards(unittest.TestCase):
    def test_clean_env_strips_treatment_variables(self):
        import os
        saved = dict(os.environ)
        try:
            os.environ.update({"NAVRL_STRAY": "1", "CAMK1_SMOKE": "1", "CAMTRAIN_X": "1", "S3B_COMMAND_YAW": "ekf",
                               "KEEP_ME": "1"})
            env = T.clean_env()
            self.assertFalse([k for k in env if k.startswith(T.ENV_STRIP)])
            self.assertEqual(env.get("KEEP_ME"), "1")
        finally:
            os.environ.clear()
            os.environ.update(saved)

    def test_fov_line_matches_the_task_print(self):
        task = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        self.assertIn("hfov=%.0fdeg", task)
        self.assertEqual("hfov=%.0fdeg" % 66.0, T.FOV_LOG_LINE)
        self.assertNotEqual("hfov=%.0fdeg" % 87.0, T.FOV_LOG_LINE)           # F's camera would not pass

    def test_smoke_needs_its_checkpoint_and_disk_guard_exists(self):
        src = (TOOLS / "camk1_train.py").read_text()
        self.assertIn("if len(ckpts) != 1:", src)
        self.assertNotIn("and not args.smoke", src)
        self.assertGreaterEqual(T.MIN_FREE_GB, 1.0)


if __name__ == "__main__":
    unittest.main()
