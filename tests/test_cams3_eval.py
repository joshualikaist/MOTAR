"""CPU tests of CAM-S3-E (docs/plans/prereg_camonly_s3w_eval_2026-11-27.md): the stage-3 rungs of tools/camref_eval_run.py
(every earlier arm unchanged), the S3W policy of tools/camref_entry.py, the launcher copy, tools/camref_stage3_run.py and
tools/camref_stage3_analyse.py. Earlier arms are compared with the runner as committed at 7a2e9c8 (before stage 3), loaded
from git under a private name; skipped (not passed) when git cannot show it. Stage 3's constants are installed only inside
camref_stage3_run.installed(), which restores stage 1's (PUBLIC_CPU runs every test in one process)."""
import importlib.util
import json
import os
import pathlib
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))
import camref_eval_run as X                                 # noqa: E402
import camref_stage3_run as S                               # noqa: E402
import camref_stage3_analyse as B                           # noqa: E402

PRE_STAGE3 = "7a2e9c8"
SEEDS = (4501, 4502, 4503)
NEW = ("S0T", "S0M", "W", "WT")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


E = _load("cams3e_test_entry", TOOLS / "camref_entry.py")


def _old_runner():
    done = subprocess.run(["git", "-C", str(ROOT), "show", "%s:tools/camref_eval_run.py" % PRE_STAGE3],
                          capture_output=True)
    if done.returncode:
        return None
    d = pathlib.Path(tempfile.mkdtemp(prefix="camref_pre_stage3_"))
    path = d / "camref_eval_run_pre_stage3.py"
    path.write_bytes(done.stdout)
    try:
        return _load("camref_eval_run_pre_stage3_under_test", path)
    finally:
        shutil.rmtree(str(d), ignore_errors=True)            # review r1 P3-5: __pycache__ without -B


OLD = _old_runner()
TRAIN_ENV = (json.loads(X.D.FT_TRAIN_RECEIPT.read_text())["env_navrl"] if X.D.FT_TRAIN_RECEIPT.is_file() else None)
needs_ft = unittest.skipIf(TRAIN_ENV is None, "S2b-FT training receipt absent")
S3W_RECEIPT = json.loads(S.S3W_RECEIPT.read_text()) if S.S3W_RECEIPT.is_file() else None
needs_s3w = unittest.skipIf(S3W_RECEIPT is None or TRAIN_ENV is None, "S3-W training receipt absent")


def _env(arm, seed=4501, d="/x/cells"):
    cd = pathlib.Path(d) / X.cell_id(arm, 205, seed)
    return cd, X.cell_navrl_env(arm, seed, cd, 8, 10 ** 9, TRAIN_ENV, "sha")


@unittest.skipIf(OLD is None, "pre-stage-3 runner not readable from git")
class EarlierArmsUnchanged(unittest.TestCase):
    @needs_ft
    def test_every_earlier_arm_builds_and_checks_as_before(self):
        n = 0
        for arm in list(OLD.ARMS) + list(OLD.ARM_BASE):
            for s in SEEDS:
                self.assertEqual(X.rung_delta(X.rung(arm), s), OLD.rung_delta(OLD.rung(arm), s), arm)
                cd, env = _env(arm, s)
                self.assertEqual(env, OLD.cell_navrl_env(arm, s, cd, 8, 10 ** 9, TRAIN_ENV, "sha"), arm)
                self.assertEqual(X.env_problems(env, arm, s, cd, 8, 10 ** 9, TRAIN_ENV, "sha"),
                                 OLD.env_problems(env, arm, s, cd, 8, 10 ** 9, TRAIN_ENV, "sha"), arm)
                n += 1
            for f in ("expected_installed", "visible_start", "cams2_on", "camref_record", "mismatch_allow",
                      "camref_rules_on", "policy", "rung"):
                self.assertEqual(getattr(X, f)(arm), getattr(OLD, f)(arm), (f, arm))
            self.assertEqual(X.launcher_values(arm, 205, 4501, pathlib.Path("/x"), "c"),
                             OLD.launcher_values(arm, 205, 4501, pathlib.Path("/x"), "c"), arm)
        self.assertGreater(n, 60)
        for r in OLD.RUNG_PROTOCOL:
            self.assertEqual(X.RUNG_PROTOCOL[r], OLD.RUNG_PROTOCOL[r], r)

    def test_unit_constants_unchanged_outside_install(self):
        for k in S.PATCHED:
            if k not in ("checkpoints", "stage_budget_h"):          # functions; replaced only inside install()
                self.assertEqual(getattr(X, k), getattr(OLD, k), k)
        self.assertEqual(X.LAUNCHER_SHA, "51f949d08ec70935aeca13fe3b28ce289ff208edeccc526d4b0410662573518a")
        self.assertEqual(X.sha(ROOT / X.LAUNCHER), X.LAUNCHER_SHA)       # the stage-1 launcher is untouched


@needs_ft
class Stage3Rungs(unittest.TestCase):
    def test_each_rung_is_s0_plus_exactly_its_keys(self):
        for s in SEEDS:
            _, s0 = _env("S0ID", s)
            seed = str(10 * s + 6)
            meas = {"NAVRL_DEPTH_HOLE_P": "0", "NAVRL_DEPTH_MEAS": "1", "NAVRL_DEPTH_MEAS_LIBRARY": X.CAMS3_LIBRARY,
                    "NAVRL_DEPTH_MEAS_SHA256": X.CAMS3_LIBRARY_SHA, "NAVRL_DEPTH_MEAS_SEED": seed,
                    "NAVRL_DEPTH_MEAS_TOP_ROWS": "10", "NAVRL_DEPTH_MEAS_ADVANCE_P": "0.5",
                    "NAVRL_DEPTH_MEAS_JUMP_P": "0.01", "NAVRL_DEPTH_MEAS_FALSE_NEAR": "0"}
            want = {"S0T": {"NAVRL_BAR_POOL": "bars_h3_thin"}, "S0M": meas,
                    "W": dict(meas, NAVRL_YAW_RATE_MAX="1.5708", NAVRL_CAMS3_SECTOR_K="24"),
                    "WT": dict(meas, NAVRL_YAW_RATE_MAX="1.5708", NAVRL_CAMS3_SECTOR_K="24",
                               NAVRL_BAR_POOL="bars_h3_thin")}
            for arm in NEW:
                cd, env = _env(arm, s)
                _, base = _env("S0ID", s)
                base = {k: v for k, v in base.items() if k not in ("NAVRL_INVIEW_DIAG_JSONL",)}
                got = {k: v for k, v in env.items() if k != "NAVRL_INVIEW_DIAG_JSONL" and base.get(k) != v}
                got.pop("NAVRL_CAMREF_POLICY", None)
                self.assertEqual(got, want[arm], (arm, s))
                self.assertEqual(set(base) - set(env), set(), arm)
                self.assertEqual(env["NAVRL_CAMREF_POLICY"], "S3W" if arm in ("W", "WT") else "S2B")
            self.assertEqual(s0["NAVRL_DEPTH_HOLE_P"], "0.02")

    def test_every_rung_passes_its_own_checks_and_fails_every_other(self):
        for arm in NEW + ("S0ID",):
            cd, env = _env(arm)
            self.assertEqual(X.env_problems(env, arm, 4501, cd, 8, 10 ** 9, TRAIN_ENV, "sha"), [], arm)
            for other in NEW + ("S0ID",):
                if other == arm:
                    continue
                with self.subTest(arm=arm, other=other):
                    self.assertTrue(X.env_problems(env, other, 4501, cd, 8, 10 ** 9, TRAIN_ENV, "sha"))
            self.assertEqual(X.protocol_problems(env, arm), [], arm)

    def test_cams3_point_refuses_a_changed_key(self):
        cd, env = _env("W")
        for k, v in (("NAVRL_CAMS3_SECTOR_K", "1"), ("NAVRL_DEPTH_MEAS_FALSE_NEAR", "1"), ("NAVRL_BAR_POOL", "bars_h3_thin"),
                     ("NAVRL_YAW_RATE_MAX", "1.0472")):
            with self.subTest(k=k):
                self.assertTrue(X.env_problems(dict(env, **{k: v}), "W", 4501, cd, 8, 10 ** 9, TRAIN_ENV, "sha"))
        self.assertEqual(X.env_problems(env, "W", 4501, cd, 8, 10 ** 9, TRAIN_ENV, "sha"), [])

    def test_f_and_k1_refuse_the_cams3_keys(self):
        for arm in ("F4", "K1T"):
            cd, env = _env(arm)
            self.assertEqual(X.env_problems(env, arm, 4501, cd, 8, 10 ** 9, TRAIN_ENV, "sha"), [], arm)
            bad = dict(env, NAVRL_DEPTH_MEAS="1")
            self.assertIn("policy %s cell carries the CAM-S3 rules" % X.policy(arm),
                          X.env_problems(bad, arm, 4501, cd, 8, 10 ** 9, TRAIN_ENV, "sha"))

    def test_mismatch_allowance_names_the_arena_line_per_rung(self):
        thin_run = "NavRL ARENA MISMATCH | NAVRL_BAR_POOL: checkpoint=bars_h3 running=bars_h3_thin. The observation"
        h3_run = "NavRL ARENA MISMATCH | NAVRL_BAR_POOL: checkpoint=bars_h3_thin running=bars_h3. The observation"
        for arm, ok in (("S0T", (thin_run,)), ("W", (h3_run,)), ("WT", ()), ("S0M", ()), ("S0ID", ())):
            for line in (thin_run, h3_run):
                with self.subTest(arm=arm, line=line[40:80]):
                    self.assertEqual(X.mismatch_problems(line, arm) == [], line in ok)
        p9 = "NavRL CONFIG MISMATCH | NAVRL_P9_SEED: checkpoint trained with 4952.000, running with 45013.000."
        for arm in NEW:
            self.assertEqual(X.mismatch_problems(p9, arm), [], arm)
        yaw = "NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 1.571, running with 1.047."
        self.assertTrue(X.mismatch_problems(yaw, "W"))

    def test_condition_and_installs_of_new_arms(self):
        for arm in NEW:
            self.assertEqual(X.expected_installed(arm), [])
            self.assertTrue(X.cams2_on(arm) and X.visible_start(arm) and not X.camref_record(arm))
            cond = {"num_envs": 128, "episode_len_steps": 600, "full_goal_distribution": True,
                    "fov_curriculum_saturated": True, "speed_governor_mode": "riskcap", "goal_dist_max_m": 12.0,
                    "speed_governor_free_mps": 2.5, "p9_empirical_error_enabled": True,
                    "d8_dynamic_mesh_treatment": True, "d8_dynamic_mesh_contract": {"mode": X.S2A.D8_MODE},
                    "p9_empirical_error_sha256": TRAIN_ENV["NAVRL_P9_EXPECTED_SHA256"]}
            self.assertEqual(X.condition_problems(cond, arm, TRAIN_ENV), [], arm)
            self.assertTrue(X.condition_problems(dict(cond, p9_empirical_error_enabled=False), arm, TRAIN_ENV))


@needs_ft
class Entry(unittest.TestCase):
    def _settings_env(self, arm):
        cd, env = _env(arm)
        env.update(X.launcher_values(arm, 205, 4501, cd, X.POLICY_SHA[X.policy(arm)]))
        return env

    def test_new_arms_accepted_with_their_rules(self):
        for arm in NEW + tuple(a + "_smoke" for a in NEW):
            s = E.settings(self._settings_env(arm))
            self.assertEqual(s["policy"], X.policy(arm))
            self.assertEqual(s["realveh"].get("cams3_meas") is not None, X.rung(arm) in X.MEAS_RUNGS, arm)
            self.assertEqual(s["realveh"].get("cams3_sector") is not None, X.rung(arm) in X.SECTOR_RUNGS, arm)

    def test_bad_combinations_refused(self):
        w, s0 = self._settings_env("W"), self._settings_env("S0ID")
        bad = {"S3W without the sector rule": {k: v for k, v in w.items() if k != "NAVRL_CAMS3_SECTOR_K"},
               "S3W at 60 deg/s": dict(w, NAVRL_YAW_RATE_MAX="1.0472"),
               "S3W with false near": dict(w, NAVRL_DEPTH_MEAS_FALSE_NEAR="1"),
               "S3W with holes": dict(w, NAVRL_DEPTH_HOLE_P="0.02"),
               "S3W with a yaw clip": dict(w, NAVRL_YAWCLIP="1", NAVRL_YAWCLIP_RADPS="1.047"),
               "S3W under S2b-FT's checkpoint": dict(w, NAVRL_EVAL_CHECKPOINT=E.POLICY_CKPT["S2B"]),
               "W's runtime named S2B": dict(w, NAVRL_CAMREF_POLICY="S2B", NAVRL_EVAL_CHECKPOINT=E.POLICY_CKPT["S2B"]),
               "S2B with the sector rule": dict(s0, NAVRL_CAMS3_SECTOR_K="24"),
               "S2B under S3-W's checkpoint": dict(s0, NAVRL_EVAL_CHECKPOINT=E.POLICY_CKPT["S3W"]),
               "S2B with a wrong library SHA": dict(self._settings_env("S0M"), NAVRL_DEPTH_MEAS_SHA256="0" * 64)}
        for name, env in bad.items():
            with self.subTest(name), self.assertRaises(SystemExit):
                E.settings(env)
        self.assertEqual(E.settings(w)["policy"], "S3W")                # positive control of the same base
        for arm in ("F4", "K1T"):
            env = self._settings_env(arm)
            self.assertTrue(E.settings(env))
            with self.subTest(arm), self.assertRaises(SystemExit):
                E.settings(dict(env, NAVRL_DEPTH_MEAS="1"))
            with self.subTest(arm), self.assertRaises(SystemExit):
                E.settings(dict(env, NAVRL_CAMS3_SECTOR_K="24"))


class Launcher(unittest.TestCase):
    def test_copy_pinned_and_differs_only_in_the_policy_line(self):
        new = (ROOT / S.LAUNCHER).read_text().splitlines()
        old = (ROOT / X.LAUNCHER).read_text().splitlines()
        self.assertEqual(X.sha(ROOT / S.LAUNCHER), S.LAUNCHER_SHA)
        added = [line for line in new if line not in old]
        removed = [line for line in old if line not in new]
        header = added[:4]
        marked = added[4:]
        self.assertEqual(new[1:5], header)
        self.assertTrue(all(line.startswith("# ") for line in header))
        self.assertEqual(len(header), 4)                                 # the header naming the copy and its change
        self.assertTrue(all("[cams3e]" in line or "S3W" in line or "NAVRL_CAMREF_POLICY" in line for line in marked),
                        marked)
        self.assertEqual(len(marked), 6)                                 # 3 policy-check lines, 3 comments (P3-10)
        self.assertTrue(all("S2B" in line for line in removed))
        self.assertEqual(len(removed), 5)
        code = [line for line in marked if not line.lstrip().startswith("#")]
        self.assertTrue(any('== "S3W"' in line for line in code))
        self.assertFalse(any('== "S3W"' in line for line in old))
        self.assertEqual([line for line in new if line not in added], [line for line in old if line not in removed])


class Stage3Install(unittest.TestCase):
    def test_installed_patches_and_restores(self):
        before = {k: getattr(X, k) for k in S.PATCHED}
        with S.installed() as R:
            self.assertEqual(R.SCOPE, S.SCOPE)
            self.assertEqual(R.eval_plan(), [(a, 205, s) for s in SEEDS for a in NEW])
            self.assertEqual(R.LAUNCHER, S.LAUNCHER)
            self.assertIn(S.PREREG, R.COMMITTED_FILES)
            self.assertIn(X.CAMS3_REL, R.COMMITTED_FILES)
            self.assertIs(R.checkpoints, S.checkpoints)
            self.assertEqual([a for a, _, _ in R.SMOKE_PLAN], [a + "_smoke" for a in NEW])
            self.assertEqual(R.IDENTITY_REF, {"S0ID": ("DX1_V312", 4501)})
            self.assertIn(("DX1_V312", 4501), R.REF_SHA)
        self.assertEqual({k: getattr(X, k) for k in S.PATCHED}, before)

    def test_allowlist_admits_the_stage3_paths_and_nothing_else(self):
        changed = X.changed_since(X.FT_TRAIN_HEAD)
        if changed is None:
            self.skipTest("git diff unavailable")
        self.assertEqual(X.allowlist_problems(changed + list(S.NEW_FILES), S.CODE_ALLOWLIST, X.FT_TRAIN_HEAD), [])
        self.assertTrue(X.allowlist_problems(changed, X.CODE_ALLOWLIST, X.FT_TRAIN_HEAD))    # stage 1's refuses them
        for bad in ("aerial_gym/task/navrl_task/navrl_task.py", "aerial_gym/task/navrl_task/navrl_perception.py",
                    "resources/models/environment_assets/bars_h3/bar_000.urdf", "tools/cams2b_train.py"):
            self.assertTrue(X.allowlist_problems([bad], S.CODE_ALLOWLIST, X.FT_TRAIN_HEAD), bad)

    def test_timeouts_and_pins_name_s3w(self):
        self.assertEqual(X.POLICY_SHA["S3W"], E.POLICY_CKPT["S3W"])
        self.assertEqual(X.CELL_TIMEOUT_S["S3W"], X.CELL_TIMEOUT_S["S2B"])
        self.assertEqual(X.POLICY_SHA["S2B"], E.POLICY_CKPT["S2B"])


@needs_s3w
class S3WTraining(unittest.TestCase):
    def test_recorded_training_passes(self):
        self.assertEqual(S.s3w_problems(TRAIN_ENV), [])

    def test_tampered_inputs_refused(self):
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            rec = json.loads(S.S3W_RECEIPT.read_text())
            rec["env_navrl"]["NAVRL_CAMS3_SECTOR_K"] = "1"
            (d / "receipt.json").write_text(json.dumps(rec))
            p = S.s3w_problems(TRAIN_ENV, receipt_path=d / "receipt.json")
            self.assertTrue(any("not its pin" in x for x in p))
            self.assertTrue(any("NAVRL_CAMS3_SECTOR_K" in x for x in p))
            rep = json.loads(S.S3W_REPORT.read_text())
            rep["verdict"] = "WALL_CAP"
            (d / "rep.json").write_text(json.dumps(rep))
            self.assertTrue(any("TRAINED" in x for x in S.s3w_problems(TRAIN_ENV, report_path=d / "rep.json")))
            self.assertTrue(S.s3w_problems(TRAIN_ENV, ckpt=d / "missing.pth"))
            ft = dict(TRAIN_ENV, NAVRL_MAX_VELOCITY="3.0")
            self.assertTrue(any("differs from S2b-FT" in x for x in S.s3w_problems(ft)))


def _rv(meas=None, sector=None, sha="x"):
    s = {"cams2": {}, "camsafe": {}, "vehicle_rand": {}, "inview_start": {}}
    rv = {"settings": s, "cams3_sha256": sha}
    if meas is not None:
        s["cams3_meas"] = meas
        rv["cams3_meas"] = {"installed": True}
    if sector is not None:
        s["cams3_sector"] = sector
        rv["cams3_sector"] = {"installed": True}
    return rv


MEAS_OK = {"NAVRL_DEPTH_MEAS_ADVANCE_P": 0.5, "NAVRL_DEPTH_MEAS_FALSE_NEAR": 0, "NAVRL_DEPTH_MEAS_JUMP_P": 0.01,
           "NAVRL_DEPTH_MEAS_LIBRARY": X.CAMS3_LIBRARY, "NAVRL_DEPTH_MEAS_SEED": 45016,
           "NAVRL_DEPTH_MEAS_SHA256": X.CAMS3_LIBRARY_SHA, "NAVRL_DEPTH_MEAS_TOP_ROWS": 10}


class ReceiptChecks(unittest.TestCase):
    def test_each_rung_accepts_its_installs_and_refuses_the_others(self):
        sha = X.sha(ROOT / X.CAMS3_REL)
        recs = {"S0T": _rv(sha=sha), "S0M": _rv(MEAS_OK, sha=sha), "W": _rv(MEAS_OK, {"NAVRL_CAMS3_SECTOR_K": 24}, sha),
                "WT": _rv(MEAS_OK, {"NAVRL_CAMS3_SECTOR_K": 24}, sha), "S0ID": _rv(sha=sha)}
        for arm, rv in recs.items():
            self.assertEqual(X.cams3_receipt_problems(rv, arm), [], arm)
        for arm in recs:
            for other, rv in recs.items():
                same = (X.rung(arm) in X.MEAS_RUNGS) == (X.rung(other) in X.MEAS_RUNGS) and \
                       (X.rung(arm) in X.SECTOR_RUNGS) == (X.rung(other) in X.SECTOR_RUNGS)
                with self.subTest(arm=arm, receipt=other):
                    self.assertEqual(X.cams3_receipt_problems(rv, arm) == [], same)

    def test_wrong_settings_or_bytes_refused(self):
        sha = X.sha(ROOT / X.CAMS3_REL)
        for k, v in (("NAVRL_DEPTH_MEAS_FALSE_NEAR", 1), ("NAVRL_DEPTH_MEAS_TOP_ROWS", 20),
                     ("NAVRL_DEPTH_MEAS_SHA256", "0" * 64)):
            with self.subTest(k=k):
                self.assertTrue(X.cams3_receipt_problems(_rv(dict(MEAS_OK, **{k: v}), sha=sha), "S0M"))
        self.assertTrue(X.cams3_receipt_problems(_rv(MEAS_OK, {"NAVRL_CAMS3_SECTOR_K": 1}, sha), "W"))
        self.assertTrue(X.cams3_receipt_problems(_rv(MEAS_OK, {"NAVRL_CAMS3_SECTOR_K": 24}, "0" * 64), "W"))

    @needs_s3w
    def test_s3w_training_receipt_reads_as_wt(self):
        self.assertEqual(X.cams3_receipt_problems(S3W_RECEIPT, "WT"), [])
        self.assertTrue(X.cams3_receipt_problems(S3W_RECEIPT, "S0T"))


# --------------------------------------------------------------------------- smoke controls on synthetic cells
H3_LINE = ("\x1b[33;20m[1 ms][navrl_task] - WARNING : NavRL ARENA MISMATCH | NAVRL_BAR_POOL: checkpoint=bars_h3_thin "
           "running=bars_h3. The observation width is unchanged\x1b[0m")
THIN_LINE = ("\x1b[33;20m[1 ms][navrl_task] - WARNING : NavRL ARENA MISMATCH | NAVRL_BAR_POOL: checkpoint=bars_h3 "
             "running=bars_h3_thin. The observation width is unchanged\x1b[0m")
YAW_LINE = "NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 1.571, running with 1.047."


def _events(meas, sector):
    out = []
    if meas:
        out += ['[cams3] {"event": "install_measured", "false_near": "off", "settings": {}}',
                '[cams3] {"calls": 1, "event": "meas_first", "library_frames": 1849, "mask_invalid_share": 0.10}']
    if sector:
        out += ['[cams3] {"event": "install_sector", "settings": {"NAVRL_CAMS3_SECTOR_K": 24}}',
                '[cams3] {"bins": 13, "event": "sector_first", "k": 24}']
    return out


def _cell(d, arm, lines, yaw, pool):
    cd = pathlib.Path(d) / arm
    cd.mkdir()
    (cd / "cell.log").write_text("\n".join(lines) + "\n")
    (cd / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": {"NAVRL_YAW_RATE_MAX": yaw,
                                                                        "NAVRL_BAR_POOL": pool}}))
    return cd


GOOD = {"S0T": (_events(False, False) + [THIN_LINE], "1.0472", "bars_h3_thin"),
        "S0M": (_events(True, False), "1.0472", "bars_h3"),
        "W": (_events(True, True) + [H3_LINE], "1.5708", "bars_h3"),
        "WT": (_events(True, True), "1.5708", "bars_h3_thin")}


class SmokeControls(unittest.TestCase):
    def test_each_arm_passes_on_its_own_cell_and_fails_on_every_other(self):
        with tempfile.TemporaryDirectory() as d:
            cells = {a: _cell(d, a, *GOOD[a]) for a in NEW}
            for arm in NEW:
                c = X.s3_smoke_controls(cells[arm], arm)
                self.assertTrue(c and all(c.values()), (arm, c))
                self.assertEqual(len(c), 4)
                for other in NEW:
                    if other != arm:
                        with self.subTest(arm=arm, cell=other):
                            c2 = X.s3_smoke_controls(cells[other], arm)
                            self.assertFalse(all(c2.values()), c2)

    def test_single_faults_fail(self):
        with tempfile.TemporaryDirectory() as d:
            lines, yaw, pool = GOOD["W"]
            faults = {"yaw mismatch line": (lines + [YAW_LINE], yaw, pool),
                      "yaw 1.0472": (lines, "1.0472", pool),
                      "no sector_first": ([x for x in lines if "sector_first" not in x], yaw, pool),
                      "k 1": ([x.replace('"k": 24', '"k": 1') for x in lines], yaw, pool),
                      "false near on": ([x.replace('"off"', '"on"') for x in lines], yaw, pool),
                      "no arena line": ([x for x in lines if "ARENA" not in x], yaw, pool),
                      "thin snapshot": (lines, yaw, "bars_h3_thin")}
            for i, (name, (ls, y, p)) in enumerate(faults.items()):
                with self.subTest(name):
                    c = X.s3_smoke_controls(_cell(d, "W%d" % i, ls, y, p), "W")
                    self.assertFalse(all(c.values()), c)

    def test_full_smoke_cell_controls_include_stage3_and_bearings(self):
        with tempfile.TemporaryDirectory() as d:
            cd = _cell(d, "W", *GOOD["W"])
            c = X.smoke_cell_controls(cd, "W_smoke")
            self.assertIn("sector_k24_live", c)
            self.assertIn("start_bearings", c)
            self.assertFalse(c["start_bearings"])                       # no inview rows in this synthetic cell
            self.assertNotIn("sector_k24_live", X.smoke_cell_controls(cd, "S6_smoke"))


class Analysis(unittest.TestCase):
    def test_controls_pass(self):
        ok, res = B.controls()
        self.assertTrue(ok, res)
        self.assertGreaterEqual(len(res), 9)

    def test_bar_rule_is_inclusive_at_the_limits_and_strict_beyond(self):
        r = B.readings(B.synthetic(n=1000, w=(0.70, 0.20, 0.05)))
        self.assertEqual(r["Q1"]["W"]["label"], "BAR_MET_POINT")
        self.assertEqual(r["Q1"]["W"]["all_crashes"]["wilson_label"], "UNRESOLVED")
        self.assertEqual(B.use_of_result(r)["pointer"], "RUNTIME_NEXT_QUALIFIED")
        r = B.readings(B.synthetic(n=1000, w=(0.699, 0.20, 0.05)))
        self.assertEqual(r["Q1"]["W"]["label"], "BAR_NOT_MET_POINT")

    def test_incomplete_blocks_read_not_run(self):
        cells = B.synthetic()
        cells["W"] = {}
        r = B.readings(cells)
        self.assertEqual(r["Q1"]["W"], "NOT_RUN")
        self.assertEqual(r["Q2"]["W-S0"], "NOT_RUN")
        self.assertEqual(B.use_of_result(r)["pointer"], "NOT_RUN")      # review r1 P3-3
        self.assertNotIn("W", r["cells"])

    def test_per_cell_summaries_use_only_the_given_blocks(self):
        cells = B.synthetic()
        r = B.readings(cells, blocks=(4501, 4502))
        self.assertEqual(set(r["cells"]["W"]["episodes"]), {"4501", "4502"})     # review r1 P3-2

    def test_w_minus_s0m_is_labelled_and_s0m_minus_s0_is_not(self):
        r = B.readings(B.synthetic(w=(0.73, 0.21, 0.03)))
        self.assertEqual(r["Q2"]["W-S0M"]["crash"]["label"], "MATERIAL")
        self.assertEqual(r["Q2"]["W-S0M"]["status"], "IMPROVES")
        self.assertIsNone(r["Q3"]["S0M-S0"]["crash"]["label"])
        self.assertEqual(set(r["Q3"]["bar"]), {"WT", "S0", "S0T", "S0M"})

    def test_comparison_status_both_ways(self):
        mk = (lambda c, k: {"crash": {"label": c[0], "mean_pp": c[1]}, "capture": {"label": k[0], "mean_pp": k[1]}})
        self.assertEqual(B.comparison_status(mk(("MATERIAL", -8), ("SMALL", -2))), "IMPROVES")
        self.assertEqual(B.comparison_status(mk(("MATERIAL", -8), ("MATERIAL", -12))), "CRASH_DOWN_CAPTURE_DOWN")
        self.assertEqual(B.comparison_status(mk(("MATERIAL", 8), ("MATERIAL", 12))), "NO_IMPROVEMENT")
        self.assertEqual(B.comparison_status(mk(("SMALL", -4), ("NO_DETECTED_CHANGE", 0))), "NO_IMPROVEMENT")
        self.assertEqual(B.comparison_status("NOT_RUN"), "NOT_RUN")

    def test_interventions_parsed_or_not_recorded(self):
        text = ("[camsafe] calls=32 capped_frac=0.1 penalised_frac=0.2 x\n[camsafe] calls=320 capped_frac=0.1576 "
                "penalised_frac=0.1324 mean_penalty=0.01\n\x1b[33;20m[1 ms][navrl_task] - WARNING : NavRL speedgov | "
                "mode=riskcap intervene=0.326 stop=0.000 requested=2.0m/s\n")
        got = B.interventions(text)
        self.assertEqual(got["camsafe"], {"calls": 320, "capped_frac": 0.1576, "penalised_frac": 0.1324})
        self.assertEqual(got["speed_governor"], {"mode": "riskcap", "intervene": 0.326, "stop": 0.0})
        self.assertEqual(B.interventions("nothing here"), {"camsafe": "NOT_RECORDED", "speed_governor": "NOT_RECORDED"})
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(B.intervention_table({"W": {4501: d}})["W"]["4501"]["camsafe"], "NOT_RECORDED")

    @unittest.skipUnless(X.reference_dir("DX1_V312", 4501).is_dir(), "CAM-DX1 cells absent")
    def test_binding_refuses_void_and_foreign_reports(self):
        with tempfile.TemporaryDirectory() as d:
            d = pathlib.Path(d)
            (d / "i").mkdir()
            (d / "e").mkdir()
            head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True,
                                  text=True).stdout.strip()
            (d / "i" / "identity_report.json").write_text(json.dumps({"verdict": "IDENTITY_PASS", "commit": head}))
            ev = {"verdict": "VOID_HOST_FAULT", "commit": head, "unit": S.UNIT, "scope": S.SCOPE, "records": [],
                  "linked_reports": {"identity_report.json": str((d / "i" / "identity_report.json").resolve())}}
            (d / "e" / "eval_run_report.json").write_text(json.dumps(ev))
            b, _, _ = B.load_eval(d / "i", d / "e")
            self.assertFalse(b["eval_not_void"])                           # review r1 P3-1
            self.assertFalse(b["ok"])
            self.assertEqual(set(b["scripts_match_eval_commit"]), set(B.BOUND_FILES))     # review r1 P3-9
            ev["verdict"] = "COMPLETE"
            (d / "e" / "eval_run_report.json").write_text(json.dumps(ev))
            b, _, _ = B.load_eval(d / "i", d / "e")
            self.assertTrue(b["eval_not_void"])


class UnitLedger(unittest.TestCase):
    """Amendment 1 (review r1 P2-4): every report and every unlisted cell directory under the unit folder counts."""
    def _report(self, path, wall, cells=()):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"wall_s": wall, "verdict": "SMOKE_FAIL",
                                    "records": [{"cell_dir": str(c)} for c in cells]}))

    def _cell(self, d, duration=None):
        d.mkdir(parents=True)
        (d / "cell.log").write_text("x")
        if duration is not None:
            (d / "receipt.json").write_text(json.dumps({"duration_s": duration}))
        return d

    def test_reports_and_orphans_are_summed(self):
        with tempfile.TemporaryDirectory() as u:
            u = pathlib.Path(u)
            self.assertEqual(S.unit_ledger(u)["used_s"], 0.0)               # empty unit: nothing used
            listed = self._cell(u / "gpu1/smoke/W/c1", 100)
            self._report(u / "gpu1/smoke/smoke_report.json", 900.0, [listed])
            self._report(u / "gpu2/identity/identity_report.json", 300.0)
            orphan = self._cell(u / "gpu3/eval/W/c2", 250)
            killed = self._cell(u / "gpu3/eval/W/c3")
            (killed / "later.txt").write_text("y")
            t0 = (killed / "cell.log").stat().st_mtime
            os.utime(str(killed / "later.txt"), (t0 + 40, t0 + 40))
            led = S.unit_ledger(u)
            self.assertAlmostEqual(led["used_s"], 900 + 300 + 250 + 40, delta=1.0)
            self.assertEqual({pathlib.Path(o["cell_dir"]).name for o in led["orphan_cells"]}, {"c2", "c3"})
            self.assertNotIn(str(listed), [o["cell_dir"] for o in led["orphan_cells"]])
            (u / "gpu4").mkdir()
            (u / "gpu4" / "eval_run_report.json").write_text("{broken")
            led = S.unit_ledger(u)
            self.assertEqual(len(led["reports"]), 3)
            self.assertTrue(any("error" in r for r in led["reports"]))      # unreadable report recorded, not skipped
            self.assertTrue(orphan.is_dir())

    def test_stage_budget_is_the_smaller_of_chain_and_unit(self):
        with tempfile.TemporaryDirectory() as u:
            u = pathlib.Path(u)
            saved = S.UNIT_DIR
            try:
                S.unit_ledger.__defaults__ = (u,)
                with S.installed():
                    self._budgets(u)
            finally:
                S.unit_ledger.__defaults__ = (saved,)

    def _budgets(self, u):
        if True:
            if True:
                b, used = S.stage_budget_h("eval", {})
                self.assertAlmostEqual(b, 2.0)                               # empty unit: the 2.0 h sub-cap rules
                self._report(u / "gpu1/eval/eval_run_report.json", 3600 * 1.0)
                b, used = S.stage_budget_h("eval", {})
                self.assertAlmostEqual(used, 1.0)
                self.assertAlmostEqual(b, 1.4)                               # 2.4 h - 1.0 h, below the sub-cap
                self._report(u / "gpu2/eval/eval_run_report.json", 3600 * 1.5)
                self.assertEqual(S.stage_budget_h("eval", {})[0], 0.0)        # spent: no stage starts
                self.assertAlmostEqual(S.stage_budget_h("smoke", {})[0], 0.0)

    def test_install_routes_the_runner_through_the_ledger(self):
        with S.installed() as R:
            self.assertIs(R.stage_budget_h, S.stage_budget_h)
        self.assertIsNot(X.stage_budget_h, S.stage_budget_h)


class PerCellLiveness(unittest.TestCase):
    """Review r1 P3-4: receipt_problems checks the stage-3 controls on every stage-3 cell, not only in smoke."""
    def _cell(self, d, lines):
        d.mkdir()
        (d / "cell.log").write_text("\n".join(lines) + "\n")
        (d / "camref_receipt.json").write_text(json.dumps({"policy": "S3W", "installed": [], "sha256": {},
                                                           "env_navrl": {}}))
        (d / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": {"NAVRL_YAW_RATE_MAX": "1.5708",
                                                                           "NAVRL_BAR_POOL": "bars_h3"}}))
        (d / "realveh_receipt.json").write_text("{}")
        return d

    @needs_ft
    def test_missing_sector_event_fails_an_eval_cell(self):
        with tempfile.TemporaryDirectory() as d:
            good = self._cell(pathlib.Path(d) / "good", GOOD["W"][0])
            bad = self._cell(pathlib.Path(d) / "bad", [x for x in GOOD["W"][0] if "sector" not in x])
            p_good = X.receipt_problems(good, "W", 205, 4501, "c", 8, {}, TRAIN_ENV)
            p_bad = X.receipt_problems(bad, "W", 205, 4501, "c", 8, {}, TRAIN_ENV)
            self.assertFalse([x for x in p_good if x.startswith("stage-3 control")], p_good)
            self.assertIn("stage-3 control sector_k24_live failed", p_bad)
            p_s0 = X.receipt_problems(good, "S0ID", 205, 4501, "c", 8, {}, TRAIN_ENV)
            self.assertFalse([x for x in p_s0 if x.startswith("stage-3 control")])      # earlier arms unchanged


if __name__ == "__main__":
    unittest.main()
