"""CPU tests for the CAM-S2b training unit (docs/plans/prereg_camonly_s2b_2026-11-24.md): tools/cams2b_train.py,
aerial_gym/rl_training/rl_games/train_navrl_cams2b.sh and tools/cams2b_python. Nothing trains; no GPU, Isaac Gym or
simulator is started.

Safe inside the single-process PUBLIC_CPU suite: the driver and tools/cams2a_train.py are loaded by file path under
private names; os.environ is never written (the driver's helpers take explicit dicts, subprocesses get explicit env
dicts); module constants change only through unittest.mock.patch.object; fixtures live in temporary directories. The
launcher runs only inside a temporary tree whose train_navrl_v2_search.sh is a stub that records its environment, so the
real search launcher (global training lock, duplicate-run check) is never started. Every negative check has a positive
control. Real CAM-S2a artefacts (receipt, launcher log, source checkpoint) are read when present, else those tests skip.
Run: PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B tests/test_cams2b_train.py
"""
import importlib.util
import json
import math
import os
import pathlib
import shutil
import stat
import subprocess
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
RL = ROOT / "aerial_gym/rl_training/rl_games"
LAUNCHER = RL / "train_navrl_cams2b.sh"
WRAPPER = ROOT / "tools/cams2b_python"
PREREG = ROOT / "docs/plans/prereg_camonly_s2b_2026-11-24.md"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


D = _load("cams2b_train_under_test", ROOT / "tools/cams2b_train.py")
A = _load("cams2b_test_cams2a_train", ROOT / "tools/cams2a_train.py")
BASE_ENV = {"PATH": "/usr/bin:/bin", "HOME": str(pathlib.Path.home()), "LC_ALL": "C"}


def _block(epoch, max_epoch, captured, episodes, crashed, timeouts, bars):
    return ("\n  epoch          : %d / %d\n  captured (success)   :  50.0%% (%d/%d)\n  crash                :  10.0%% (%d/%d)\n"
            "  timeout (no capture) :   0.0%% (%d/%d)\n  density bars         : %d\n"
            % (epoch, max_epoch, captured, episodes, crashed, episodes, timeouts, episodes, bars))


def _promo(frm, to, dwell):
    return ("WARNING : NavRL density curriculum promoted | bars %d -> %d after 16 eps, capture=0.700 (threshold=0.001) "
            "dwell=%d epochs (navrl_task.py:8942)\n" % (frm, to, dwell))


FT_START = ("[cams2b-python] NAVRL_YAW_RATE_MAX 3.0 -> 1.0472 rad/s (action scale and Lee yaw clamp)\n"
            "WARNING : NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 3.000, running with 1.047. "
            "This changes policy inputs -- results are NOT comparable unless intentional.\n"
            "WARNING : NavRL density evidence RESET | discarded=2010/3256 eps resume_warmup=1000 epochs "
            "gate_not_before_step=416000 (navrl_task.py:4640)\n"
            "WARNING : NavRL checkpoint state restored | task_steps=384000 bars=70->175 k_min=20.0 k_max=28.0 "
            "density_window=0/16 eps (navrl_task.py:4664)\n")
SCR_START = "[cams2b-python] NAVRL_YAW_RATE_MAX 3.0 -> 1.0472 rad/s (action scale and Lee yaw clamp)\n"


class FixedDesign(unittest.TestCase):
    def test_yaw_cap_is_60_degrees_per_second(self):
        self.assertAlmostEqual(float(D.YAW_RATE_MAX), math.radians(60.0), places=4)
        self.assertLess(math.radians(45.0), float(D.YAW_RATE_MAX))      # the 45 deg/s clip binds below the cap

    def test_values_are_cams2a_plus_two_keys(self):
        extra = {k: v for k, v in D.VALUES.items() if k not in A.VALUES}
        self.assertEqual(extra, {"CAMS2B_YAW_RATE_MAX": "1.0472", "CAMS2B_DENSITY_CHECK_EPS": "16"})
        self.assertEqual({k: D.VALUES[k] for k in A.VALUES}, A.VALUES)

    def test_run_specs(self):
        ft, scr = D.RUNS["ft"], D.RUNS["scr"]
        self.assertEqual((ft["max_epochs"], ft["max_hours"], ft["stop_check_epoch"], ft["stop_min_capture"]),
                         (15000, 4.5, 13000, 0.30))
        self.assertEqual((scr["max_epochs"], scr["max_hours"], scr["stop_check_epoch"], scr["stop_min_capture"]),
                         (12000, 17.0, 3000, 0.05))
        self.assertEqual((scr["stop_window"], ft["stop_window"]), (A.STOP_WINDOW_EPOCHS, 200))
        self.assertEqual(scr["stop_min_capture"], A.STOP_MIN_CAPTURE)
        self.assertEqual(D.SOURCE_TASK_STEPS, D.SOURCE_EPOCH * 32)
        self.assertEqual(D.RUN_CORES, A.RUN_CORES)
        # scr reaches 205 bars: 9 promotions of +15 from 70
        last = scr["expected_promotions"][-1]
        self.assertEqual((len(scr["expected_promotions"]), last[1]), (9, 205))
        self.assertEqual(ft["expected_promotions"][-1][1], 205)

    def test_check_window_cannot_bind_beyond_three_epochs(self):
        # 128 envs x 32 steps per epoch; an episode lasts at most 600 steps -> at least 6.83 episodes per epoch.
        per_epoch_min = 128 * 32 / 600.0
        self.assertLessEqual(math.ceil(int(D.DENSITY_CHECK_EPS) / per_epoch_min), 3)
        self.assertLess(int(D.DENSITY_CHECK_EPS), 16384)

    def test_preregistration_carries_the_driver_numbers(self):
        if not PREREG.is_file():
            self.skipTest("preregistration not written yet")
        text = PREREG.read_text()
        for needle in (D.YAW_RATE_MAX, D.SOURCE_SHA, D.S2A_RECEIPT_SHA, "7.5e-6", "NAVRL_DENSITY_CHECK_EPS", "16",
                       "13,000", "15,000", "0.30", "0.05", "TRAINED", "STOPPED_NO_LEARNING", "WALL_CAP",
                       "TRAINING_FAIL", "VOID_HOST_FAULT", "0.785"):
            self.assertIn(needle, text, needle)


class LogParsing(unittest.TestCase):
    def test_stop_rule_ft_and_scr(self):
        ft, scr = D.RUNS["ft"], D.RUNS["scr"]
        low = [(12801 + i, 15000, 29, 100, 10, 61, 175) for i in range(200)]       # capture 0.29
        high = [(12801 + i, 15000, 31, 100, 10, 59, 175) for i in range(200)]      # capture 0.31
        self.assertTrue(D.stop_rule(low, ft))
        self.assertFalse(D.stop_rule(high, ft))                                     # positive control
        self.assertFalse(D.stop_rule(low[:-1], ft))                                 # before 13000: not evaluated
        s_low = [(2801 + i, 12000, 4, 100, 90, 6, 85) for i in range(200)]
        s_ok = [(2801 + i, 12000, 6, 100, 90, 4, 85) for i in range(200)]
        self.assertTrue(D.stop_rule(s_low, scr))
        self.assertFalse(D.stop_rule(s_ok, scr))
        # The window ends at the check epoch: epochs logged after it (the watcher polls every 30 s) do not count.
        later_good = low + [(13001 + i, 15000, 90, 100, 5, 5, 175) for i in range(20)]
        self.assertTrue(D.stop_rule(later_good, ft))
        self.assertEqual(D.stop_window(later_good, ft)["epochs"], 200)
        self.assertEqual(D.stop_window(later_good, ft)["capture"], 0.29)
        self.assertIsNone(D.stop_window(low[:-1], ft))

    def test_blocks_promotions_and_stages(self):
        text = (_block(999, 12000, 10, 20, 5, 5, 70) + _block(1000, 12000, 12, 20, 4, 4, 70) + _promo(70, 85, 1000)
                + _block(1001, 12000, 11, 20, 5, 4, 85))
        blocks = D.parse_blocks(text)
        self.assertEqual([b[0] for b in blocks], [999, 1000, 1001])
        promos = D.parse_promotions(text)
        self.assertEqual(len(promos), 1)
        self.assertEqual((promos[0]["after_epoch"], promos[0]["from"], promos[0]["to"], promos[0]["dwell_epochs"]),
                         (1000, 70, 85, 1000))
        stages = D.stage_summary(blocks)
        self.assertEqual([(s["bars"], s["epochs"], s["episodes"]) for s in stages], [(70, 2, 40), (85, 1, 20)])
        self.assertAlmostEqual(stages[0]["capture"], 22 / 40)

    def test_curriculum_check(self):
        scr = [{"after_epoch": 1000, "from": 70, "to": 85, "dwell_epochs": 1000}]
        epoch = 1001
        for j in range(8):
            epoch += 1001
            scr.append({"after_epoch": epoch, "from": 85 + 15 * j, "to": 100 + 15 * j, "dwell_epochs": 1001})
        self.assertTrue(D.curriculum_check(scr, D.RUNS["scr"])["as_preregistered"])          # positive control
        late = [dict(p) for p in scr]
        late[3]["dwell_epochs"] = 1700
        self.assertFalse(D.curriculum_check(late, D.RUNS["scr"])["as_preregistered"])
        self.assertFalse(D.curriculum_check(scr[:8], D.RUNS["scr"])["as_preregistered"])      # 205 never reached
        ft = [{"after_epoch": 13000, "from": 175, "to": 190, "dwell_epochs": 1183},
              {"after_epoch": 14001, "from": 190, "to": 205, "dwell_epochs": 1000}]
        self.assertTrue(D.curriculum_check(ft, D.RUNS["ft"])["as_preregistered"])
        early = [dict(ft[0], after_epoch=12300)] + ft[1:]
        self.assertFalse(D.curriculum_check(early, D.RUNS["ft"])["as_preregistered"])

    def test_real_cams2a_log_reproduces_f3(self):
        log = D.S2A_RECEIPT.parent / "launcher.log"
        if not log.is_file():
            self.skipTest("CAM-S2a launcher log not present")
        text = log.read_text(errors="replace")
        promos = D.parse_promotions(text)
        self.assertEqual(promos[0]["after_epoch"], 1749)                 # review r1 F3: stage 70 lasted 1,749 epochs
        self.assertEqual(promos[-1]["to"], 175)
        self.assertFalse(D.curriculum_check(promos, D.RUNS["scr"])["as_preregistered"])
        stages = D.stage_summary(D.parse_blocks(text))
        self.assertEqual((stages[0]["episodes"], stages[0]["capture"]), (48148, 0.6838))     # = stage_summary.json

    def test_start_checks_ft(self):
        good = FT_START + _block(12001, 15000, 10, 20, 5, 5, 175)
        self.assertEqual(D.start_problems(good, D.parse_blocks(good), "ft", 15000), [])      # positive control
        for line in FT_START.splitlines():
            bad = good.replace(line + "\n", "")
            self.assertTrue(D.start_problems(bad, D.parse_blocks(bad), "ft", 15000), line)
        wrong_yaw = good.replace("running with 1.047", "running with 3.000")
        self.assertTrue(D.start_problems(wrong_yaw, D.parse_blocks(wrong_yaw), "ft", 15000))
        wrong_warmup = good.replace("resume_warmup=1000", "resume_warmup=250")
        self.assertTrue(D.start_problems(wrong_warmup, D.parse_blocks(wrong_warmup), "ft", 15000))
        fresh = FT_START + _block(1, 15000, 10, 20, 5, 5, 175)
        self.assertTrue(D.start_problems(fresh, D.parse_blocks(fresh), "ft", 15000))
        self.assertTrue(D.start_problems(good, D.parse_blocks(good), "ft", 12025))           # wrong max epoch
        self.assertEqual(D.start_problems("", [], "ft", 15000), ["no epoch block logged"])

    def test_start_checks_scr(self):
        good = SCR_START + _block(1, 12000, 0, 1, 1, 0, 70)
        self.assertEqual(D.start_problems(good, D.parse_blocks(good), "scr", 12000), [])
        restored = FT_START + _block(1, 12000, 0, 1, 1, 0, 70)
        self.assertTrue(D.start_problems(restored, D.parse_blocks(restored), "scr", 12000))
        no_yaw = _block(1, 12000, 0, 1, 1, 0, 70)
        self.assertTrue(D.start_problems(no_yaw, D.parse_blocks(no_yaw), "scr", 12000))


class ReceiptContract(unittest.TestCase):
    def setUp(self):
        if not D.S2A_RECEIPT.is_file():
            self.skipTest("CAM-S2a receipt not present")
        self.s2a = json.loads(D.S2A_RECEIPT.read_text())
        self.root = "/some/worktree"
        self.path = "/x/train_ft/receipt.json"

    def _receipt(self, run):
        r = {k: v for k, v in self.s2a.items()}
        r["env_navrl"] = D.expected_env(self.s2a["env_navrl"], run, self.root, self.path)
        return r

    def test_reference_receipt_is_pinned(self):
        self.assertEqual(D.sha256(D.S2A_RECEIPT), D.S2A_RECEIPT_SHA)
        env = self.s2a["env_navrl"]
        self.assertEqual((env["NAVRL_YAW_RATE_MAX"], env["NAVRL_DENSITY_CHECK_EPS"], env["NAVRL_LEARNING_RATE"]),
                         ("3.0", "16384", "3e-5"))

    def test_expected_differences_only(self):
        s2a = self.s2a["env_navrl"]
        for run, n in (("scr", 4), ("ft", 8)):
            want = D.expected_env(s2a, run, self.root, self.path)
            diff = sorted(k for k in set(want) | set(s2a) if want.get(k) != s2a.get(k))
            self.assertEqual(len(diff), n, (run, diff))
            self.assertIn("NAVRL_YAW_RATE_MAX", diff)
            self.assertEqual(self.s2a.get("yawclip"), None)
        ft = D.expected_env(s2a, "ft", self.root, self.path)
        self.assertEqual((ft["NAVRL_LEARNING_RATE"], ft["NAVRL_DENSITY_RESUME_WARMUP"], ft["NAVRL_V2_ALLOW_RESUME"]),
                         ("7.5e-06", "1000", "1"))

    def test_good_receipts_pass_and_each_deviation_fails(self):
        for run in ("ft", "scr"):
            good = self._receipt(run)
            self.assertEqual(D.receipt_problems(good, self.s2a["env_navrl"], run, self.root, self.path), [])
            cases = {"yaw left at 3.0": ("NAVRL_YAW_RATE_MAX", "3.0"),
                     "check window 16384": ("NAVRL_DENSITY_CHECK_EPS", "16384"),
                     "stray variable": ("NAVRL_NUM_BARS", "205"),
                     "eval switch": ("NAVRL_YAWCLIP", "1"),
                     "other treatment": ("NAVRL_CAMS2_DELAY_MAX", "3")}
            for label, (key, value) in cases.items():
                bad = json.loads(json.dumps(good))
                bad["env_navrl"][key] = value
                got = D.receipt_problems(bad, self.s2a["env_navrl"], run, self.root, self.path)
                self.assertEqual(len(got), 1, (run, label, got))
            for key in D.EVAL_ONLY_RECEIPT_KEYS:
                bad = json.loads(json.dumps(good))
                bad[key] = {"installed": True}
                self.assertTrue(D.receipt_problems(bad, self.s2a["env_navrl"], run, self.root, self.path), key)
        swapped = self._receipt("scr")
        self.assertTrue(D.receipt_problems(swapped, self.s2a["env_navrl"], "ft", self.root, self.path))


class Verdicts(unittest.TestCase):
    def test_table(self):
        v = D.verdict
        ck = (15000, "f.pth", "0" * 64)
        self.assertEqual(v(False, 0, False, None, 1, 1, "max_epochs 도달", 15000, 15000, ck), "TRAINED")
        self.assertEqual(v(False, 0, False, None, 1, 2, "max_epochs 도달", 15000, 15000, ck), "VOID_HOST_FAULT")
        self.assertEqual(v(False, 1, True, None, 1, 1, None, 13000, 15000, None), "STOPPED_NO_LEARNING")
        self.assertEqual(v(False, 124, False, None, 1, 1, None, 14000, 15000, None), "WALL_CAP")
        self.assertEqual(v(False, 0, False, "receipt: x", 1, 1, "max_epochs 도달", 15000, 15000, ck), "TRAINING_FAIL")
        self.assertEqual(v(False, 0, False, None, 1, 1, "early_stop_density_capture_collapse", 13500, 15000, None),
                         "TRAINING_FAIL")
        self.assertEqual(v(False, 0, False, None, 1, 1, "max_epochs 도달", 15000, 15000, None), "TRAINING_FAIL")
        self.assertEqual(v(False, 2, False, None, 1, 1, None, 50, 15000, None), "TRAINING_FAIL")
        self.assertEqual(v(True, 0, False, None, 1, 1, "max_epochs 도달", 12025, 12025, ck), "SMOKE_COMPLETED")
        self.assertEqual(v(True, 0, False, "start: x", 1, 1, "max_epochs 도달", 12025, 12025, ck), "SMOKE_FAIL")
        self.assertEqual(v(True, 0, False, None, 0, 1, "max_epochs 도달", 12025, 12025, ck), "VOID_HOST_FAULT")


class HostAndEnv(unittest.TestCase):
    def test_cuda_allow_list_is_exact(self):
        self.assertEqual(D.foreign_cuda(["69473, /usr/share/rustdesk/rustdesk"]), [])           # positive control
        self.assertEqual(len(D.foreign_cuda(["1, /home/fair/miniconda3/envs/aerialgym/bin/python",
                                             "2, /tmp/rustdesk", "3, /usr/share/rustdesk/rustdesk-x"])), 3)
        self.assertEqual(D.foreign_cuda(["nvidia-smi failed (exit 9)"]), ["nvidia-smi failed (exit 9)"])

    def test_child_env_strips_inherited_switches(self):
        parent = dict(BASE_ENV, NAVRL_YAWCLIP="1", NAVRL_CAMS2DIAG="1", NAVRL_CAMS2_CONTACTCODE="1", CKPT="/x.pth",
                      NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="9", MAX_EPOCHS="99", KEEP="yes",
                      AERIAL_RUN_NAME="ppo_stale", AERIAL_RUN_TITLE="stale")
        for run in ("ft", "scr"):
            env = D.child_env(parent, run, False, "/u/train_%s" % run, "/u/source/ck.pth" if run == "ft" else None)
            self.assertFalse([k for k in env if k.startswith(D.EVAL_ONLY_PREFIXES)])
            self.assertNotIn("CKPT", env)
            self.assertNotIn("NAVRL_YAW_RATE_MAX", env)
            self.assertNotIn("AERIAL_RUN_NAME", env)
            self.assertNotIn("AERIAL_RUN_TITLE", env)
            self.assertEqual(env["KEEP"], "yes")                                                  # positive control
            self.assertEqual(env["CAMS2B_YAW_RATE_MAX"], "1.0472")
            self.assertEqual(env["NAVRL_REALVEH_RECEIPT"], "/u/train_%s/receipt.json" % run)
            self.assertEqual(env["CAMS2_MAX_EPOCHS"], str(D.RUNS[run]["max_epochs"]))
            self.assertEqual("CAMS2B_CKPT" in env, run == "ft")
        smoke = D.child_env(parent, "ft", True, "/u/s", "/u/ck.pth")
        self.assertEqual((smoke["CAMS2_MAX_EPOCHS"], smoke["CAMS2_TAG_SUFFIX"]), ("12025", "-smoke"))
        self.assertEqual(D.child_env(parent, "scr", True, "/u/s")["CAMS2_MAX_EPOCHS"], "25")
        self.assertEqual(parent["NAVRL_YAWCLIP"], "1")                                            # input untouched

    def test_checkpoint_hashes_every_1000_and_last(self):
        with tempfile.TemporaryDirectory() as tmp:
            nn = pathlib.Path(tmp)
            for e in range(12250, 15001, 250):
                (nn / ("last_gen_ppo_ep_%d_rew_1.0.pth" % e)).write_bytes(str(e).encode())
            (nn / "last_gen_ppo_ep_15000_rew_1.0_rlnorm.pth").write_bytes(b"x")
            (nn / "gen_ppo.pth").write_bytes(b"y")
            all_rows, keep, final = D.checkpoint_hashes(nn, 15000)
            self.assertEqual(len(all_rows), 12)
            self.assertEqual([r[0] for r in keep], [13000, 14000, 15000])
            self.assertEqual(final[0], 15000)
            self.assertEqual(final[2], D.sha256(nn / "last_gen_ppo_ep_15000_rew_1.0.pth"))
            self.assertIsNone(D.checkpoint_hashes(nn, 15250)[2])


class Preflight(unittest.TestCase):
    def _patched(self, tmp, **over):
        ref = pathlib.Path(tmp) / "receipt.json"
        ref.write_text("{}")
        ck = pathlib.Path(tmp) / "src.pth"
        ck.write_bytes(b"checkpoint")
        cfg = {"branch": "camonly/dev", "committed": True, "dirty": "", "cuda": ["1, /usr/share/rustdesk/rustdesk"],
               "free": 5e9, "ref_sha": D.sha256(ref), "src_sha": D.sha256(ck)}
        cfg.update(over)

        def git(*args):
            if args[:2] == ("rev-parse", "--abbrev-ref"):
                return cfg["branch"]
            if args[0] == "status":
                return cfg["dirty"]
            return "0" * 40
        usage = mock.Mock(free=cfg["free"])
        return [mock.patch.object(D, "git", git), mock.patch.object(D, "committed", lambda p: cfg["committed"]),
                mock.patch.object(D, "cuda_lines", lambda: cfg["cuda"]),
                mock.patch.object(D.shutil, "disk_usage", lambda p: usage),
                mock.patch.object(D, "S2A_RECEIPT", ref), mock.patch.object(D, "S2A_RECEIPT_SHA", cfg["ref_sha"]),
                mock.patch.object(D, "SOURCE_CKPT", ck), mock.patch.object(D, "SOURCE_SHA", cfg["src_sha"])]

    def _run(self, run, **over):
        with tempfile.TemporaryDirectory() as tmp:
            patches = self._patched(tmp, **over)
            for p in patches:
                p.start()
            try:
                return D.preflight(run, pathlib.Path(tmp) / "train_x", False)
            finally:
                for p in reversed(patches):
                    p.stop()

    def test_passes_when_everything_holds(self):
        for run in ("ft", "scr"):
            out = self._run(run)
            self.assertEqual(out["branch"], "camonly/dev")
            self.assertGreater(out["free_gb_required"], D.MIN_FREE_GB)

    def test_each_problem_refuses(self):
        cases = {"branch": {"branch": "camonly/s2b"}, "committed": {"committed": False}, "dirty": {"dirty": "?? x"},
                 "cuda": {"cuda": ["1, /usr/share/rustdesk/rustdesk", "2, /bin/python"]},
                 "disk": {"free": 1.1e9}, "reference": {"ref_sha": "0" * 64}}
        for label, over in cases.items():
            for run in ("ft", "scr"):
                with self.subTest(label=label, run=run), self.assertRaises(SystemExit):
                    self._run(run, **over)
        with self.assertRaises(SystemExit):
            self._run("ft", src_sha="0" * 64)
        self._run("scr", src_sha="0" * 64)                       # scr does not need the source checkpoint

    def test_disk_estimate_counts(self):
        free = (D.MIN_FREE_GB + 0.3) * 1e9                           # enough for ft (0.2), not for scr (0.55)
        self._run("ft", free=free)
        with self.assertRaises(SystemExit):
            self._run("scr", free=free)


def _stub_tree(tmp):
    """A copy of the launcher whose train_navrl_v2_search.sh is a stub that records its environment and arguments."""
    t = pathlib.Path(tmp) / "tree"
    rl = t / "aerial_gym/rl_training/rl_games"
    rl.mkdir(parents=True)
    (t / "aerial_gym/__init__.py").write_text("")
    os.symlink(str(ROOT / "results"), str(t / "results"))
    shutil.copy(str(LAUNCHER), str(rl / LAUNCHER.name))
    stub = rl / "train_navrl_v2_search.sh"
    stub.write_text("#!/usr/bin/env bash\nenv > \"${STUB_OUT}.env\"\nprintf '%s\\n' \"$@\" > \"${STUB_OUT}.args\"\n")
    stub.chmod(stub.stat().st_mode | stat.S_IEXEC)
    return t, rl / LAUNCHER.name


def _read_env(path):
    out = {}
    for line in pathlib.Path(path).read_text().splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


class Launcher(unittest.TestCase):
    def test_syntax(self):
        for f in (LAUNCHER, WRAPPER):
            self.assertEqual(subprocess.run(["bash", "-n", str(f)]).returncode, 0, f)
        self.assertTrue(os.access(str(WRAPPER), os.X_OK))
        self.assertTrue(os.access(str(LAUNCHER), os.X_OK))

    def test_cams2a_exports_kept(self):
        """Every NAVRL_* export of CAM-S2a's launcher is in CAM-S2b's, character for character."""
        def code(line):
            return line.split(" #", 1)[0].strip()
        a = [code(l) for l in (RL / "train_navrl_cams2a.sh").read_text().splitlines()
             if l.strip().startswith("export NAVRL_")]
        b = {code(l) for l in LAUNCHER.read_text().splitlines()}
        missing = [l for l in a if l not in b]
        self.assertEqual(missing, [])
        self.assertGreater(len(a), 50)                                                # non-vacuous
        self.assertIn("export NAVRL_DENSITY_CHECK_EPS=\"${CAMS2B_DENSITY_CHECK_EPS}\"", b)

    def _env(self, run, out, ckpt=None, **extra):
        env = D.child_env(BASE_ENV, run, False, str(pathlib.Path(out).parent / ("train_" + run)), ckpt)
        env.update({"STUB_OUT": str(out)})
        env.update(extra)
        return env

    def test_scr_and_ft_reach_the_search_launcher_with_the_contract(self):
        if not (ROOT / "results/perception_p9_2026-09-09/p9_error_injector.json").is_file():
            self.skipTest("results/ not linked in this tree")
        with tempfile.TemporaryDirectory() as tmp:
            tree, launcher = _stub_tree(tmp)
            out = pathlib.Path(tmp) / "scr"
            r = subprocess.run(["bash", str(launcher)], env=self._env("scr", out), capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            env = _read_env(str(out) + ".env")
            self.assertEqual(pathlib.Path(str(out) + ".args").read_text().strip(), "")
            self.assertEqual((env["NAVRL_V2_ALLOW_RESUME"], env["NAVRL_DENSITY_CHECK_EPS"], env["CAMS2B_YAW_RATE_MAX"]),
                             ("0", "16", "1.0472"))
            self.assertEqual(env["PYTHON"], str(tree / "tools/cams2b_python"))
            self.assertEqual(env["NAVRL_DENSITY_MIN_EPOCHS"], "1000")
            for k in ("CKPT", "NAVRL_LEARNING_RATE", "NAVRL_DENSITY_RESUME_WARMUP", "NAVRL_RESET_DENSITY_WINDOW"):
                self.assertNotIn(k, env)
            if not D.SOURCE_CKPT.is_file():
                self.skipTest("CAM-S2a source checkpoint not present")
            out = pathlib.Path(tmp) / "ft"
            r = subprocess.run(["bash", str(launcher)], env=self._env("ft", out, str(D.SOURCE_CKPT)),
                               capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            env = _read_env(str(out) + ".env")
            self.assertEqual(pathlib.Path(str(out) + ".args").read_text().split(),
                             ["--checkpoint", str(D.SOURCE_CKPT), "--branch_run"])
            self.assertEqual((env["NAVRL_V2_ALLOW_RESUME"], env["NAVRL_LEARNING_RATE"], env["NAVRL_RESET_DENSITY_WINDOW"],
                              env["NAVRL_DENSITY_RESUME_WARMUP"], env["MAX_EPOCHS"], env["CKPT"]),
                             ("1", "7.5e-06", "1", "1000", "15000", str(D.SOURCE_CKPT)))

    def test_refusals(self):
        with tempfile.TemporaryDirectory() as tmp:
            _, launcher = _stub_tree(tmp)
            out = pathlib.Path(tmp) / "o"
            bad_ck = pathlib.Path(tmp) / "wrong.pth"
            bad_ck.write_bytes(b"not the checkpoint")
            cases = {"bad run": self._env("scr", out, CAMS2B_RUN="xx"),
                     "eval switch": self._env("scr", out, NAVRL_YAWCLIP="1"),
                     "eval prefix": self._env("scr", out, NAVRL_CAMS2DIAG_OUT="/x"),
                     "contactcode": self._env("scr", out, NAVRL_CAMS2_CONTACTCODE="1"),
                     "scr with ckpt": self._env("scr", out, CAMS2B_CKPT=str(bad_ck)),
                     "ft wrong sha": self._env("ft", out, str(bad_ck)),
                     "ft missing ckpt": self._env("ft", out, str(pathlib.Path(tmp) / "none.pth")),
                     "ft epochs": self._env("ft", out, str(D.SOURCE_CKPT), CAMS2_MAX_EPOCHS="12000"),
                     "no yaw": {k: v for k, v in self._env("scr", out).items() if k != "CAMS2B_YAW_RATE_MAX"}}
            for label, env in cases.items():
                r = subprocess.run(["bash", str(launcher)], env=env, capture_output=True, text=True)
                self.assertNotEqual(r.returncode, 0, label)
                self.assertFalse(pathlib.Path(str(out) + ".env").exists(), label)    # never reached the search launcher


class Wrapper(unittest.TestCase):
    """tools/cams2b_python -> tools/realveh_python -> REALVEH_REAL_PYTHON (a stub recording what it was given)."""

    def _run(self, **env):
        with tempfile.TemporaryDirectory() as tmp:
            stub = pathlib.Path(tmp) / "python"
            out = pathlib.Path(tmp) / "out"
            stub.write_text("#!/usr/bin/env bash\necho \"yaw=${NAVRL_YAW_RATE_MAX}\" > %s\nprintf '%%s\\n' \"$@\" >> %s\n"
                            % (out, out))
            stub.chmod(stub.stat().st_mode | stat.S_IEXEC)
            full = dict(BASE_ENV, REALVEH_REAL_PYTHON=str(stub))
            full.update(env)
            r = subprocess.run([str(WRAPPER), "runner.py", "--train"], env=full, capture_output=True, text=True)
            return r, (out.read_text() if out.exists() else None)

    def test_sets_the_yaw_ceiling_and_runs_the_entry(self):
        r, out = self._run(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="1.0472")
        self.assertEqual(r.returncode, 0, r.stderr)
        lines = out.splitlines()
        self.assertEqual(lines[0], "yaw=1.0472")
        self.assertEqual(lines[1:], [str(ROOT / "tools/realveh_entry.py"), "runner.py", "--train"])
        self.assertIn("NAVRL_YAW_RATE_MAX 3.0 -> 1.0472 rad/s", r.stdout)

    def test_refusals(self):
        cases = {"inherited not 3.0": dict(NAVRL_YAW_RATE_MAX="2.5", CAMS2B_YAW_RATE_MAX="1.0472"),
                 "inherited unset": dict(CAMS2B_YAW_RATE_MAX="1.0472"),
                 "missing": dict(NAVRL_YAW_RATE_MAX="3.0"),
                 "text": dict(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="abc"),
                 "zero": dict(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="0.0"),
                 "negative": dict(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="-1"),
                 "yawclip": dict(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="1.0472", NAVRL_YAWCLIP="1"),
                 "diag": dict(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="1.0472", NAVRL_CAMS2DIAG="1"),
                 "contact": dict(NAVRL_YAW_RATE_MAX="3.0", CAMS2B_YAW_RATE_MAX="1.0472",
                                 NAVRL_CAMS2_CONTACTCODE="1")}
        for label, env in cases.items():
            r, out = self._run(**env)
            self.assertNotEqual(r.returncode, 0, label)
            self.assertIsNone(out, label)                                 # the interpreter was never started


class Isolation(unittest.TestCase):
    def test_no_environment_leak(self):
        self.assertNotIn("CAMS2B_YAW_RATE_MAX", os.environ)
        self.assertNotIn("CAMS2B_RUN", os.environ)


if __name__ == "__main__":
    unittest.main()
