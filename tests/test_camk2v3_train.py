"""CPU tests for the CAM-K2 v3 training wrapper (tools/camk2v3_train.py) and launcher (train_navrl_camk2v3.sh).

v3 is v2 (tools/camk2v2_train.py, train_navrl_camk2v2.sh, both unchanged) plus the invalid-depth validity rule
(navrl_depth_validity.py, blob_near, window 5, density 0.5) and a labelled hit-token training diagnostic. These tests
check what v3 adds or changes; the v2 helpers (receipt, record, real module telemetry, checkpoint fixture) come from
tests/test_camk2v2_train.py, loaded by path under a private name. No GPU, Isaac Gym or simulator; nothing is
registered in sys.modules and the environment is changed only through unittest.mock.patch. Every negative check is
paired with a positive control.
Run: PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B tests/test_camk2v3_train.py
"""
import contextlib
import hashlib
import importlib.util
import io
import json
import pathlib
import re
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

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
    C = _load("camk2v3_train_under_test", TOOLS / "camk2v3_train.py")
    V2T = _load("camk2v2_test_helpers_for_v3", ROOT / "tests/test_camk2v2_train.py")
finally:
    sys.path[:] = _saved_path
_PATH_RESTORED = sys.path == _saved_path                   # checked by TestIsolation (other modules may add later)
C2 = V2T.C                                                  # the v2 wrapper (unchanged)
E = C.E
DM = V2T.DM
DV = _load("navrl_depth_validity_camk2v3_test", ROOT / "aerial_gym/task/navrl_task/navrl_depth_validity.py")
HEAD = "d" * 40
V3_SH = RL / "train_navrl_camk2v3.sh"
V2_SH = RL / "train_navrl_camk2v2.sh"
VALIDITY_SHA = hashlib.sha256(b"validity").hexdigest()


def launcher_env(seed):
    env, sha = V2T.launcher_env(seed, V3_SH.read_text())
    env["NAVRL_REALVEH_RECEIPT"] = "/nonexistent/camk2v3/realveh_receipt.json"
    return env, sha


def good_receipt(seed):
    env, _ = launcher_env(seed)
    s = E.settings(env)
    r = {"settings": s, "task_robot_name": "navrl_quad", "sha256": dict(V2T.COMMITTED),
         "vehicle_rand": {"settings": dict(s["vehicle_rand"])}, "depth_model": {"settings": dict(s["depth_model"])},
         "depth_validity": {"settings": dict(s["depth_validity"])}, "depth_validity_sha256": VALIDITY_SHA,
         "ablation": {"condition": "no_lidar"}, "overrides_after": dict(C.EXPECTED_AFTER),
         "env_navrl": {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_VEH_RAND": "1", "NAVRL_DEPTH_MODEL": "oakd",
                       "NAVRL_DEPTH_VALIDITY": "blob_near", "NAVRL_DEPTH_VALIDITY_WINDOW": "5",
                       "NAVRL_DEPTH_VALIDITY_DENSITY": "0.5", "NAVRL_PERCEPTION_PERTURB": "0",
                       "NAVRL_REALVEH_RECEIPT": "/x/realveh_receipt.json"}}
    return r


def validity_telemetry(seed, frames=None, late_binding=False, settings=None):
    """The lines the real depth model + validity rule log on a stand-in perception, in the production nesting
    (depth model, then the rule on its output), with the first 3 frames inside rl_games' real devnull_stdio."""
    import torch
    err = io.StringIO()
    ds = dict(C.expected_settings(seed)["depth_model"])
    vs = dict(settings or C.expected_settings(seed)["depth_validity"])
    rng = torch.get_rng_state()
    with contextlib.redirect_stderr(err):
        cls = type("P", (), {"observe": lambda self, rgb, depth, lidar_m: None})
        mod = types.SimpleNamespace(NavRLPerceptionModule=cls)
        DV.install_perception(mod, vs)
        saved = (DV._STREAM["stream"], DM._STREAM["stream"])
        DM._STREAM["stream"] = sys.stderr
        if late_binding:
            DV._STREAM["stream"] = None
        cls._depth_model_settings = ds
        h, w, ne = 90, 160, 4
        perc = cls()
        perc.__dict__.update(num_envs=ne, height=h, width=w, cx=(w - 1) / 2, cy=(h - 1) / 2,
                             fx=w / (2 * 0.6494075932), fy=h / (2 * 0.5095254495), max_camera_range=20.0,
                             camera_obstacle_max_range=10.0, detect_decoupled=False)
        try:
            for i in range(frames or DV.STATS_CALLS):
                depth = torch.full((ne, h, w), 6.0)
                depth[:, :, 0:6] = 0.3                                                # a bar under MinZ
                with (V2T.Q.devnull_stdio() if i < 3 else contextlib.nullcontext()):
                    DV.apply(perc, DM.apply(perc, depth, ds), vs)
        finally:
            DV._STREAM["stream"], DM._STREAM["stream"] = saved
    assert torch.equal(rng, torch.get_rng_state())
    return "\n".join(ln for ln in err.getvalue().splitlines() if ln.startswith(DV.LOG_PREFIX)) + "\n"


VTELE = {1: validity_telemetry(1), 2: validity_telemetry(2)}


def barprobe(hits):
    return "".join("[navrl_task] - WARNING : NavRL barprobe v2 | n=100 bars_range=20.0 bars_fov=5.0 occupied_bins=4.0 "
                   "hit_dist=0.54m hit_fov=0.960 hit_token=%.3f hit_token_given_fov=0.200 tokens=4.0 associated=2.0 "
                   "unique=2.0 duplicate=0.0 center_offset=0.30m cross_track=0.20m radial_gap=0.10m rank=1.0 "
                   "(capacity=8)\n" % h for h in hits)


class TestFixedDesign(unittest.TestCase):
    def test_v3_constants(self):
        self.assertEqual(C.PREREG, "docs/plans/prereg_camonly_k2_train_v3_2026-11-08.md")
        self.assertTrue(C.SCOPE.startswith("CAM-K2 v3:"))
        self.assertIn("blob_near (window 5 px, density 0.5", C.SCOPE)
        self.assertEqual(C.UNIT_DIR.name, "camk2v3_20261108")
        self.assertEqual(C.LAUNCHER.name, "train_navrl_camk2v3.sh")
        self.assertEqual(C.TAG % 1, "v2-camk2v3-ep28000-205bars-main-riskcap-camonly-tw40-s1")
        self.assertEqual(C.DEPTH_VALIDITY, {"NAVRL_DEPTH_VALIDITY": "blob_near", "NAVRL_DEPTH_VALIDITY_WINDOW": 5,
                                            "NAVRL_DEPTH_VALIDITY_DENSITY": 0.5})
        self.assertEqual((C.DIAG_WINDOWS, C.DIAG_THRESHOLD), (5, 0.5))
        self.assertEqual(C.DIAG_REFERENCE["camk2v2_end"], {"s1": 0.17, "s2": 0.19})
        self.assertEqual(C.DIAG_REFERENCE["camk1v1_end"], {"s1": 0.68, "s2": 0.72})
        self.assertIn("CAMK2V3_", C.ENV_STRIP)

    def test_everything_else_is_v2(self):
        for name in ("SOURCES", "SOURCE_EPOCH", "FINAL_EPOCH", "SMOKE_EPOCH", "MAX_HOURS", "VEH_RAND",
                     "VEH_RAND_LIMIT_MODE", "DEPTH_MODEL", "VEH_RAND_SEED", "DEPTH_SEED", "EXPECTED_AFTER",
                     "EXPECTED_CAMTRAIN", "ROBOT", "ROBOT_CONFIG_REL", "ROBOT_URDF_REL", "BRANCH",
                     "HOLE_SHARE_TOLERANCE", "MASS_RANGE_KG", "SMOKE_RATE_MARGIN", "FOV_LOG_LINE"):
            self.assertEqual(getattr(C, name), getattr(C2, name), name)
        for seed in (1, 2):
            e3, e2 = C.expected_settings(seed), C2.expected_settings(seed)
            self.assertEqual({k: v for k, v in e3.items() if k != "depth_validity"}, e2)
            self.assertEqual(e3["depth_validity"], C.DEPTH_VALIDITY)
        self.assertEqual((C.MIN_FREE_GB, C2.MIN_FREE_GB), (0.6, 1.0))                 # review r1 M-2: lowered
        self.assertNotEqual(C.SCOPE, C2.SCOPE)                                       # control: SCOPE does differ
        self.assertNotEqual(C.UNIT_DIR, C2.UNIT_DIR)


class TestLauncher(unittest.TestCase):
    def test_syntax_and_mode(self):
        self.assertEqual(subprocess.run(["bash", "-n", str(V3_SH)]).returncode, 0)
        self.assertTrue(V3_SH.stat().st_mode & 0o111)

    def test_v3_adds_only_its_own_lines(self):
        def body(path):
            return [ln for ln in path.read_text().splitlines() if ln.strip() and not ln.lstrip().startswith("#")]
        norm = [ln.replace("camk2v2", "camk2v3").replace("CAMK2V2", "CAMK2V3") for ln in body(V2_SH)]
        v3 = body(V3_SH)
        added = [ln for ln in v3 if ln not in norm]
        removed = [ln for ln in norm if ln not in v3]
        self.assertEqual(sorted(ln for ln in added if ln.startswith("export")),
                         ["export NAVRL_DEPTH_VALIDITY=blob_near", "export NAVRL_DEPTH_VALIDITY_DENSITY=0.5",
                          "export NAVRL_DEPTH_VALIDITY_WINDOW=5"])
        self.assertEqual(len(added), 4)                                               # + the echo line
        self.assertTrue(any("validity ${NAVRL_DEPTH_VALIDITY}" in ln for ln in added))
        self.assertEqual(len(removed), 1)                                             # the echo line it replaces
        self.assertIn("holes ${NAVRL_DEPTH_HOLE_P} seed ${NAVRL_DEPTH_SEED}\"", removed[0])

    def test_launcher_env_is_accepted_by_the_entry_and_equals_the_preregistered_settings(self):
        for seed in (1, 2):
            env, sha = launcher_env(seed)
            self.assertEqual(sha, C.SOURCES[seed]["sha256"])
            s = E.settings(env)
            for key, want in C.expected_settings(seed).items():
                self.assertEqual(s[key], want, (seed, key))
        env2, _ = V2T.launcher_env(1)
        self.assertNotIn("depth_validity", E.settings(env2))                          # control: v2's env has none

    def test_receipt_tag_and_source_variables(self):
        text = V3_SH.read_text()
        self.assertIn('AERIAL_RUN_TAG="v2-camk2v3-ep28000-205bars-main-riskcap-camonly-tw40-s${TSEED}${TAG_SUFFIX}"',
                      text)
        self.assertIn("CAMK2V3_SOURCE_CKPT:?", text)
        self.assertIn('"${CAMK2V3_SMOKE:-0}" == "1"', text)
        self.assertNotIn("CAMK2V2_", text)
        self.assertIn("CAMK2V2_", V2_SH.read_text())                                  # control


class TestReceipt(unittest.TestCase):
    def test_good_receipt_passes(self):
        for seed in (1, 2):
            self.assertEqual(C.receipt_problems(good_receipt(seed), seed, V2T.COMMITTED, VALIDITY_SHA), [])

    def test_validity_deviations_fail(self):
        def without(d, *keys):
            d = json.loads(json.dumps(d))
            x = d
            for k in keys[:-1]:
                x = x[k]
            x.pop(keys[-1])
            return d
        g = good_receipt(1)
        cases = {
            "no_validity_record": without(g, "depth_validity"),
            "no_validity_sha": without(g, "depth_validity_sha256"),
            "no_validity_setting": without(g, "settings", "depth_validity"),
            "no_validity_env": without(g, "env_navrl", "NAVRL_DEPTH_VALIDITY"),
            "density_env": dict(g, env_navrl=dict(g["env_navrl"], NAVRL_DEPTH_VALIDITY_DENSITY="0.6")),
            "window": dict(g, depth_validity={"settings": dict(C.DEPTH_VALIDITY, NAVRL_DEPTH_VALIDITY_WINDOW=7)}),
            "sha": dict(g, depth_validity_sha256="0" * 64),
        }
        for name, r in cases.items():
            self.assertTrue(C.receipt_problems(r, 1, V2T.COMMITTED, VALIDITY_SHA), name)
        self.assertTrue(C.receipt_problems(g, 1, V2T.COMMITTED, None))                # uncommitted module fails
        self.assertTrue(C.receipt_problems(V2T.good_receipt(1), 1, V2T.COMMITTED, VALIDITY_SHA))   # a v2 receipt

    def test_committed_validity_sha_reads_head(self):
        with tempfile.TemporaryDirectory() as d:
            subprocess.run(["git", "init", "-q", d], check=True)
            p = pathlib.Path(d) / C.VALIDITY_REL
            p.parent.mkdir(parents=True)
            p.write_bytes(b"committed")
            subprocess.run(["git", "-C", d, "add", C.VALIDITY_REL], check=True)
            subprocess.run(["git", "-C", d, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x"],
                           check=True)
            p.write_bytes(b"working copy")
            self.assertEqual(C.committed_validity_sha256(pathlib.Path(d)), hashlib.sha256(b"committed").hexdigest())
        with tempfile.TemporaryDirectory() as d:
            subprocess.run(["git", "init", "-q", d], check=True)
            self.assertIsNone(C.committed_validity_sha256(pathlib.Path(d)))


class TestTelemetry(unittest.TestCase):
    def texts(self, seed=1, vtele=None):
        return [V2T.TELEMETRY[seed] + (VTELE[seed] if vtele is None else vtele)]

    def test_real_lines_pass(self):
        for seed in (1, 2):
            p, details = C.telemetry_problems(self.texts(seed), seed)
            self.assertEqual(p, [], seed)
            s = details["depth_validity"]["summary"][0]
            self.assertGreater(s["near_pixels"], 0)
            self.assertLess(s["near_pixels"], s["invalid_pixels"])                    # holes stay no return
            self.assertEqual(s["sentinel_mismatch"], 0)

    def test_missing_or_wrong_lines_fail(self):
        ok = VTELE[1]
        lines = ok.splitlines()
        cases = {
            "none": "",
            "no_summary": "\n".join(x for x in lines if '"summary"' not in x) + "\n",
            "no_first": "\n".join(x for x in lines if '"first"' not in x) + "\n",
            "two_installs": ok + lines[0] + "\n",
            "short": validity_telemetry(1, frames=10),
            "late_binding": validity_telemetry(1, late_binding=True),
            "other_settings": validity_telemetry(1, settings=dict(C.DEPTH_VALIDITY, NAVRL_DEPTH_VALIDITY_WINDOW=7)),
            "mismatch": ok.replace('"sentinel_mismatch": 0', '"sentinel_mismatch": 3'),
            "rule_never_acted": "\n".join(re.sub(r'"near_pixels": \d+', '"near_pixels": 0', x) if '"summary"' in x
                                          else x for x in lines) + "\n",                  # review r1 M-1
        }
        for name, v in cases.items():
            p, _ = C.telemetry_problems(self.texts(1, v), 1)
            self.assertTrue(any(x.startswith("depth-validity") for x in p), name)
        self.assertEqual(C.telemetry_problems(self.texts(1, ok), 1)[0], [])           # control


class TestDiagnostic(unittest.TestCase):
    def test_labels(self):
        ok = C.hit_token_diagnostic(["x\n" + barprobe([0.1, 0.1, 0.6, 0.5, 0.5, 0.5, 0.4])])
        self.assertEqual(ok["windows"], 7)
        self.assertAlmostEqual(ok["mean_last"], 0.5)                                 # the last 5: equal counts as OK
        self.assertEqual(ok["label"], "DIAG_OK")
        low = C.hit_token_diagnostic([barprobe([0.9, 0.9, 0.19, 0.17, 0.2, 0.18, 0.19])])
        self.assertEqual(low["label"], "DIAG_LOW")
        self.assertAlmostEqual(low["mean_last"], 0.186)
        few = C.hit_token_diagnostic([barprobe([0.9, 0.9, 0.9, 0.9])])
        self.assertEqual((few["label"], few["mean_last"]), ("DIAG_NOT_RECORDED", None))
        self.assertEqual(C.hit_token_diagnostic([""])["label"], "DIAG_NOT_RECORDED")
        self.assertEqual(ok["reference"], C.DIAG_REFERENCE)

    def test_reads_the_runners_own_format(self):
        """The regex matches the logger format string in navrl_task.py (positive control: the analyzer's)."""
        src = (ROOT / "aerial_gym/task/navrl_task/navrl_task.py").read_text()
        i = src.index('"NavRL barprobe v2 | n=%d')
        fmt = "".join(re.findall(r'"([^"]*)"', src[i:src.index("% (", i)]))
        line = fmt % (100, 20.0, 5.0, 4.0, 0.54, 0.96, 0.3125, 0.2, 4.0, 2.0, 2.0, 0.0, 0.3, 0.2, 0.1, 1.0, 8)
        d = C.hit_token_diagnostic([line + "\n"] * 1)
        self.assertEqual(d["windows"], 1)
        self.assertEqual(C._HIT_TOKEN.findall(line), ["0.312"])

    def test_diagnostic_does_not_change_the_verdict(self):
        src = (TOOLS / "camk2v3_train.py").read_text()
        verdict = [ln for ln in src.splitlines() if 'report["verdict"] =' in ln and "VOID_HOST_FAULT" in ln]
        self.assertEqual(len(verdict), 1)
        self.assertNotIn("DIAG", verdict[0])
        self.assertIn("ok_label if not problems else \"TRAINING_FAIL\"", verdict[0])   # control: the v2 rule


class TestPostRun(unittest.TestCase):
    def check(self, seed=1, smoke=False, **change):
        start = {"fault_count": 0, "boot_id": "b", "online": "0-3,6-23"}
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            rl, out = d / "rl", d / "out"
            tag = C.TAG % seed + ("-smoke" if smoke else "")
            epoch = C.SMOKE_EPOCH if smoke else C.FINAL_EPOCH
            new = rl / "runs" / ("ppo_261109_0100_navrl_" + tag)
            (new / "nn").mkdir(parents=True)
            V2T.write_checkpoint(new / "nn" / ("last_gen_ppo_ep_%d_rew_1.0.pth" % epoch), epoch, V2T.quad_identity())
            out.mkdir()
            (out / "source_checkpoint").mkdir()
            copy = out / "source_checkpoint" / C.SOURCES[seed]["path"].name
            copy.write_bytes(b"camk1 source")
            sources = {seed: {"path": C.SOURCES[seed]["path"], "sha256": hashlib.sha256(b"camk1 source").hexdigest()}}
            (out / "realveh_receipt.json").write_text(json.dumps(change.get("receipt", good_receipt(seed))))
            (out / ("realveh_receipt.json" + E.VEH_RAND_RECORD_SUFFIX)).write_text(json.dumps(V2T.good_record(seed)))
            (out / "launcher.log").write_text("NavRL perception mode | hfov=66deg\n" + V2T.TELEMETRY[seed]
                                              + change.get("vtele", VTELE[seed]) + change.get("probe", "")
                                              + "  step time (s)  : 5.9\n  step time (s)  : 4.8\n")
            with mock.patch.object(C, "RL", rl), mock.patch.object(C, "SOURCES", sources):
                return C.post_run_problems(out, seed, smoke, set(), V2T.COMMITTED, 0, start, dict(start), head=HEAD,
                                           committed_validity=VALIDITY_SHA)

    def test_passing_fixture_and_diagnostic(self):
        problems, host_ok, details = self.check(probe=barprobe([0.6] * 5))
        self.assertEqual(problems, [])
        self.assertTrue(host_ok)
        self.assertEqual(details["hit_token_diagnostic"]["label"], "DIAG_OK")
        problems, _, details = self.check(probe=barprobe([0.2] * 5))
        self.assertEqual(problems, [])                                                # DIAG_LOW is not a problem
        self.assertEqual(details["hit_token_diagnostic"]["label"], "DIAG_LOW")
        problems, _, details = self.check(seed=1, smoke=True)
        self.assertEqual(problems, [])
        self.assertEqual(details["hit_token_diagnostic"]["label"], "DIAG_NOT_RECORDED")

    def test_validity_deviations_fail(self):
        g = good_receipt(1)
        for name, change in {"no_validity_telemetry": {"vtele": ""},
                             "v2_receipt": {"receipt": V2T.good_receipt(1)},
                             "validity_sha": {"receipt": dict(g, depth_validity_sha256="1" * 64)}}.items():
            problems, _, _ = self.check(**change)
            self.assertTrue(problems, name)


class TestIsolation(unittest.TestCase):
    def test_v2_files_are_not_the_v3_files(self):
        self.assertNotEqual(C2.LAUNCHER, C.LAUNCHER)
        self.assertEqual(C2.PREREG, "docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md")
        self.assertEqual(C2.UNIT_DIR.name, "camk2v2_20261108")
        self.assertNotIn("validity", (TOOLS / "camk2v2_train.py").read_text())
        self.assertNotIn("VALIDITY", V2_SH.read_text())
        self.assertIn("VALIDITY", V3_SH.read_text())                                  # control

    def test_no_lasting_path_change(self):
        self.assertTrue(_PATH_RESTORED)


if __name__ == "__main__":
    unittest.main()
