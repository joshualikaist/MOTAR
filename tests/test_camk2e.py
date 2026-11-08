"""CPU tests of CAM-K2-E's launcher copy, runner checks and analysis (docs/plans/prereg_camonly_k2_eval_v1_2026-11-08.md;
draft ~/motar_tmp/camk2e_20261108/PREREG_DRAFT.md), following tests/test_camk1e.py.

No simulator and no GPU. Every negative case has a positive control. The end-to-end smokes and the controls read
CAM-K1-E's raw cells (read-only) and are skipped where they are absent. `test_camk2_pins_are_filled` FAILS until the
CAM-K2 checkpoint SHA-256 pins in tools/camk2e_run.py are filled from the TRAINED train reports (by design).
CAM-K2-E evaluates CAM-K2 v2 (tools/camk2v2_train.py, the per-motor limit raised to T/W 4.0, opt-in raise); CAM-K2 v1 (T/W 2.25 cap)
is WITHDRAWN_BEFORE_RUN, and the fixtures below carry v2's values (TestV2Only checks that v1's are refused).
"""
import hashlib
import io
import json
import pathlib
import random
import shutil
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import camk1e_analyse as E1                                   # noqa: E402
import camk2e_analyse as E2                                   # noqa: E402
import camk2e_run as R                                        # noqa: E402

RG = ROOT / "aerial_gym/rl_training/rl_games"
K1E = pathlib.Path.home() / "motar_tmp/camk1e_20261108/raw"
CELL = "E2_obstacle_aware__%dbars__seed%d"
T2 = R.T2
COMMITTED = {name: hashlib.sha256(name.encode()).hexdigest() for name in T2.E.HASHED}
HAVE_K1E = (K1E / "eval_01/eval_run_report.json").is_file()


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


LIMIT = 4.0 * 0.25 * 9.81 / 4                                 # CAM-K2 v2's raised per-motor limit, 2.4525 N


def telemetry_text(seed, motor_updates=10, hole=0.02, drop=(), init_mode="raise", tau=(0.021, 0.039), live=None,
                   counters=True):
    """Plausible module lines (as tools/camk2v2_train's tests produce them) for a treated cell at evaluation seed seed."""
    exp = R.expected_settings(seed, True)
    vr, dm = exp["vehicle_rand"], exp["depth_model"]
    k = dm["NAVRL_DEPTH_SIGMA_D_PX"] / dm["NAVRL_DEPTH_FB_PXM"]
    v = T2.VR_MOD.LOG_PREFIX
    d = T2.DM_MOD.LOG_PREFIX
    lines = {"veh_install": v + json.dumps({"event": "install", "settings": vr}),
             "veh_init": v + json.dumps({"event": "init", "cap_applied": True, "mode": init_mode, "factor_min": 0.82,
                                         "factor_max": 1.19, "tau_min": 0.021, "tau_max": 0.039,
                                         "cap_per_motor_n": [LIMIT, LIMIT], "live_max_thrust_n": [LIMIT, LIMIT]}),
             "veh_reset": v + json.dumps({"event": "reset", "calls": 2, "envs": 3, "envs_total": 131,
                                          "factor_min": 0.81, "factor_max": 1.2, "tau_min": tau[0], "tau_max": tau[1],
                                          "motor_updates": motor_updates, **raise_fields(motor_updates, live, counters)}),
             "veh_update": v + json.dumps({"event": "update", "motor_updates": 1 if motor_updates else 0,
                                           "cap_applied": True, **raise_fields(1 if motor_updates else 0, live, counters)}),
             "depth_install": d + json.dumps({"event": "install", "settings": dm}),
             "depth_first": d + json.dumps({"event": "first", "calls": 1}),
             "depth_summary": d + json.dumps({"event": "summary", "calls": T2.DM_MOD.STATS_CALLS, "eligible_pixels": 9000,
                                              "invalid_minz": 0.01, "invalid_maxz": 0.0, "invalid_hole": hole,
                                              "mean_abs_noise_m": 0.05, "median_valid_z_m": 4.0,
                                              "sigma_z_at_median_m": k * 16.0})}
    return "".join(line + "\n" for name, line in lines.items() if name not in drop)


def raise_fields(updates, live=None, counters=True):
    """The raise counters and live read-back the module adds to reset / update lines (review r1 M-1)."""
    if not counters:
        return {}
    return {"cmd_elements": 512 * updates, "cmd_above_own_max": 3 * updates, "cmd_at_limit": updates,
            "cmd_max_n": 3.1 if updates else None, "live_max_thrust_n": live or [LIMIT, LIMIT]}


def record(seed):
    """CAM-K2 v2's record (navrl_vehicle_rand.initialize in mode "raise"): limit 4.0 x m x g / 4 = 2.4525 N > 2 N."""
    vr = R.expected_settings(seed, True)["vehicle_rand"]
    cap = LIMIT
    own = 8.0 / (0.25 * 9.81)
    return {"tw_nominal": 4.0, "cap_applied": True, "mode": "raise", "num_motors": 4, "cap_per_motor_n": [cap, cap],
            "live_max_thrust_per_motor_n": [cap, cap],
            "mass_kg": [0.25, 0.25], "gravity_mps2": [9.81, 9.81], "own_thrust_to_weight": [own, own],
            "max_thrust_per_motor_n": [2.0, 2.0], "thrust_limit_per_motor_n": [cap, cap], "settings": vr}


def make_cell(d, arm, seed, base_snapshot=None, ckpt_sha="c" * 64, change=None):
    """Write the realveh receipt, cell receipt, recorder summary, cell log and (treated) record of a valid cell into d;
    `change` names one deviation."""
    change = change or ""
    d.mkdir(parents=True, exist_ok=True)
    on = R.treated(arm)
    receipt_path = d / "realveh_receipt.json"
    env = {"NAVRL_REALVEH_RECEIPT": str(receipt_path), "NAVRL_SENSOR_ABLATION": "no_lidar", **R1_CAM(),
           **(R.treatment_env(seed) if on else {})}
    settings = T2.E.settings(env)                                # the real entry's parser: the keys main() writes
    if change == "veh_seed":
        settings["vehicle_rand"] = dict(settings["vehicle_rand"], NAVRL_VEH_RAND_SEED=1)
    if change == "depth_value":
        settings["depth_model"] = dict(settings["depth_model"], NAVRL_DEPTH_MAXZ_M=10.0)
    rec = {"settings": settings, "task_robot_name": "navrl_rv05_quad" if change == "robot" else "navrl_quad",
           "ablation": {"condition": "full" if change == "ablation" else "no_lidar", "hbeams": 72, "vbeams": 4,
                        "task": str(ROOT / "aerial_gym/task/navrl_task/navrl_task.py"),
                        "perception": str(ROOT / "aerial_gym/task/navrl_task/navrl_perception.py")},
           "entry": str(ROOT / R.ENTRY), "script": str(RG / "runner.py"), "overrides_after": dict(T2.EXPECTED_AFTER),
           "sha256": dict(COMMITTED, depth_model="0" * 64) if change == "sha" else dict(COMMITTED)}
    if on:
        rec["vehicle_rand"] = {"settings": settings["vehicle_rand"]}
        rec["depth_model"] = {"settings": settings["depth_model"]}
    receipt_path.write_text(json.dumps(rec))
    snap = dict(base_snapshot or {})
    snap.pop("NAVRL_CAMTRAIN_RECEIPT", None)
    snap.update({"NAVRL_SENSOR_ABLATION": "no_lidar", **R1_CAM(), "NAVRL_REALVEH_RECEIPT": str(receipt_path),
                 "NAVRL_EVAL_CHECKPOINT": ckpt_sha})
    if on:
        snap.update(R.treatment_env(seed))
    if change == "off_with_treatment_key":
        snap["NAVRL_VEH_RAND"] = "1"
    if change == "camtrain_receipt_var":
        snap["NAVRL_CAMTRAIN_RECEIPT"] = str(d / "camtrain_receipt.json")
    (d / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": snap, "checkpoint_sha256": ckpt_sha,
                                                "status": "OK"}))
    cam = {"hfov_deg": 87.0 if change == "fov" else 66.0, "vfov_deg": 54.0}
    (d / "kroc2027_summary.json").write_text(json.dumps({"camera": cam}))
    log = "[navrl] camera hfov=66deg\n"
    if on and change != "no_telemetry":
        log += telemetry_text(seed, motor_updates=0 if change == "no_motor_updates" else 10,
                              hole=0.1 if change == "hole_share" else 0.02,
                              init_mode="lower" if change == "telemetry_v1_mode" else "raise",
                              tau=(0.03, 0.06) if change == "telemetry_v1_tau" else (0.021, 0.039))
    if not on and change == "telemetry_when_off":
        log += telemetry_text(seed)
    (d / "cell.log").write_text(log)
    rp = d / ("realveh_receipt.json" + T2.E.VEH_RAND_RECORD_SUFFIX)
    if on and change != "no_record":
        bad = {"record_uncapped": {"cap_applied": False},
               "record_v1_lower": {"mode": "lower", "thrust_limit_per_motor_n": [2.0, 2.0]},   # v1's mode
               "record_limit_not_raised": {"thrust_limit_per_motor_n": [2.0, 2.0]}}
        rp.write_text(json.dumps(dict(record(seed), **bad.get(change, {}))))
    return d


def R1_CAM():
    return dict(R.R1.CAM_VALUES)


class LauncherCopy(unittest.TestCase):
    def test_copy_differs_from_camk1e_only_in_marked_changes(self):
        k1 = (RG / "eval_navrl_camk1e_arms.sh").read_text()
        k2 = (RG / "eval_navrl_camk2e_arms.sh").read_text()
        k1_lines, k2_lines = k1.splitlines(keepends=True), k2.splitlines(keepends=True)
        k1_head = k1_lines[:11]                                   # its header, through the blank comment line
        start = k2_lines.index("# Everything else is byte-for-byte CAM-K1-E's launcher after its own header, "
                               "which follows:\n")
        body = "".join(k2_lines[start + 2:])
        rev = (
            ('"${CAMEVAL_PY:?CAMEVAL_PY (absolute path of tools/realveh_python) is required}"   # [camk2e] 1\n',
             '"${CAMEVAL_PY:?CAMEVAL_PY (absolute path of tools/camtrain_python) is required}"\n'),
            ('CKPT="${CAMK2E_CKPT:?CAMK2E_CKPT is required}"\n', 'CKPT="${CAMK1E_CKPT:?CAMK1E_CKPT is required}"\n'),
            ('EXPECT_SHA="${CAMK2E_CKPT_SHA:?CAMK2E_CKPT_SHA is required}"\n',
             'EXPECT_SHA="${CAMK1E_CKPT_SHA:?CAMK1E_CKPT_SHA is required}"\n'),
            ('# [camk2e] 3: the checkpoint and its expected SHA-256 come from the caller (tools/camk2e_run.py); both are '
             'required\n',
             '# [camk1e] 2: the checkpoint and its expected SHA-256 come from the caller (tools/camk1e_run.py); both are '
             'required\n'),
            ('      export NAVRL_REALVEH_RECEIPT="${CELL_DIR}/realveh_receipt.json"   # [camk2e] 4, 5\n',
             '      export NAVRL_CAMTRAIN_RECEIPT="${CELL_DIR}/camtrain_receipt.json"   # [camk1e] 3\n'))
        for new, old in rev:
            self.assertEqual(body.count(new), 1, new)
            body = body.replace(new, old)
        i = body.index('export REALVEH_REAL_PYTHON="${PYTHON}"   # [camk2e] 1\n')
        j = body.index("esac\n", i) + len("esac\n")
        body = body[:i] + 'export CAMTRAIN_REAL_PYTHON="${PYTHON}"\n' + body[j:]
        self.assertEqual("".join(k1_head) + body, k1)
        self.assertEqual(k2.count("[camk2e]"), 9)               # header 1; change 1: 2; change 2: 4; change 3: 1; 4-5: 1

    def test_runner_pins_the_launcher_sha(self):
        self.assertEqual(R.LAUNCHER_SHA, sha(ROOT / R.LAUNCHER))

    def test_launcher_refuses_a_wrong_treatment_switch(self):
        import subprocess
        base = {"PATH": "/usr/bin:/bin", "NAVRL_SENSOR_ABLATION": "no_lidar", "CAMEVAL_PY": str(R.WRAPPER),
                "PYTHON": "/usr/bin/python3"}
        for env, msg in (({}, "CAMK2E_TREATMENTS"), ({"CAMK2E_TREATMENTS": "maybe"}, "must be on or off"),
                         ({"CAMK2E_TREATMENTS": "on"}, "treatments on needs"),
                         ({"CAMK2E_TREATMENTS": "off", "NAVRL_VEH_RAND": "1"}, "treatments off needs")):
            r = subprocess.run(["bash", str(ROOT / R.LAUNCHER)], capture_output=True, text=True, env=dict(base, **env))
            self.assertNotEqual(r.returncode, 0, env)
            self.assertIn(msg, r.stderr, env)
        # positive control: a valid switch gets past it and stops at the next guard, the missing checkpoint
        for env in ({"CAMK2E_TREATMENTS": "off"}, {"CAMK2E_TREATMENTS": "on", "NAVRL_VEH_RAND": "1",
                                                   "NAVRL_DEPTH_MODEL": "oakd"}):
            r = subprocess.run(["bash", str(ROOT / R.LAUNCHER)], capture_output=True, text=True, env=dict(base, **env))
            self.assertIn("CAMK2E_CKPT is required", r.stderr, env)


class Approval(unittest.TestCase):
    def good(self):
        return {"approved_by": "user", "approved_at": "2026-11-08T21:00:00+09:00", "scope": R.SCOPE,
                "commit": "abcdef1234567"}

    def test_good_approval_passes_and_bad_ones_fail(self):
        self.assertEqual(R.R1.approval_problems(self.good(), "abcdef1234567", scope=R.SCOPE), [])
        self.assertTrue(R.R1.approval_problems(self.good(), "abcdef1234567"))     # CAM-K1-E's scope: refused
        self.assertTrue(R.R1.approval_problems(dict(self.good(), commit="1234567"), "abcdef1234567", scope=R.SCOPE))
        self.assertTrue(R.R1.approval_problems(dict(self.good(), approved_by=""), "abcdef1234567", scope=R.SCOPE))


class Design(unittest.TestCase):
    def test_plan_arms_and_seeds(self):
        plan = R.eval_plan()
        self.assertEqual(len(plan), 60)
        self.assertEqual(sum(1 for a, _, _ in plan if a in R.MAIN_ARMS), 48)
        self.assertEqual(sorted({(d, s) for a, d, s in plan if a in R.NOMINAL_ARMS}),
                         [(d, s) for d in (70, 205) for s in (4501, 4502, 4503)])
        self.assertEqual(plan[:6], [("K1s1", 70, 4501), ("K1s2", 70, 4501), ("K2s1", 70, 4501), ("K2s2", 70, 4501),
                                    ("K2s1_nom", 70, 4501), ("K2s2_nom", 70, 4501)])
        self.assertEqual(plan[6][0], "K1s1")
        self.assertEqual(len(R.IDENTITY_PLAN), 4)                    # with the ON repeat (review r1 E-L2)
        self.assertEqual(R.IDENTITY_PLAN[2], ("K1s1_on_repeat", 205, 4501))
        self.assertTrue(R.treated("K1s1_on_repeat"))
        self.assertEqual((R.RATE_CELLS, R.RATE_MARGIN), (60, 1.10))
        self.assertEqual(R.MAX_HOURS, 3.5)
        self.assertEqual(R.MIN_FREE_GB, 1.0)
        self.assertEqual(R.BRANCH, "camonly/dev")

    def test_treatments_match_camk2_and_seeds_are_matched_per_block(self):
        self.assertEqual(R.module_seeds(4501), (45011, 45012))
        env = R.treatment_env(4502)
        for arm in ("K1s1", "K1s2", "K2s1", "K2s2"):                # matched: a function of the cell seed only
            self.assertEqual(R.expected_settings(4502, R.treated(arm)), R.expected_settings(4502, True))
        self.assertEqual(env["NAVRL_VEH_RAND_SEED"], "45021")
        self.assertEqual(env["NAVRL_DEPTH_FB_PXM"], "66.1875")
        self.assertEqual(env["NAVRL_DEPTH_MAXZ_M"], "20.0")
        self.assertEqual(env["NAVRL_VEH_RAND_TW_NOMINAL"], "4.0")                   # CAM-K2 v2, not v1's 2.25
        self.assertEqual(env["NAVRL_VEH_RAND_LIMIT_MODE"], "raise")                  # the opt-in raise
        self.assertEqual((env["NAVRL_VEH_RAND_TAU_MIN"], env["NAVRL_VEH_RAND_TAU_MAX"]), ("0.02", "0.04"))
        exp = R.expected_settings(4501, True)
        camk2 = T2.expected_settings(1)
        for part in ("vehicle_rand", "depth_model"):                 # CAM-K2's values, other seeds
            self.assertEqual({k: v for k, v in exp[part].items() if not k.endswith("_SEED")},
                             {k: v for k, v in camk2[part].items() if not k.endswith("_SEED")})
        self.assertEqual(exp["camtrain"], camk2["camtrain"])
        self.assertIsNone(R.expected_settings(4501, False)["vehicle_rand"])
        self.assertFalse(R.treated("K2s1_nom") or R.treated("K1s1_off_identity"))
        self.assertTrue(R.treated("K1s1_on_negative") and R.treated("K2s1_smoke"))
        # the entry accepts the runner's environment, ON and OFF (positive control of the settings path)
        for on in (True, False):
            s = T2.E.settings({"NAVRL_REALVEH_RECEIPT": "/nonexistent/r.json", "NAVRL_SENSOR_ABLATION": "no_lidar",
                               **R1_CAM(), **(R.treatment_env(4501) if on else {})})
            self.assertEqual({k: s[k] for k in ("vehicle_rand", "depth_model", "camtrain")},
                             R.expected_settings(4501, on))


class Receipt(unittest.TestCase):
    def test_valid_cells_pass_on_and_off(self):
        with tempfile.TemporaryDirectory() as t:
            for arm in ("K2s1", "K1s2", "K2s1_nom", "K1s1_off_identity"):
                d = make_cell(pathlib.Path(t) / arm, arm, 4502)
                self.assertEqual(R.receipt_problems(d, arm, 4502, COMMITTED), [], arm)

    def test_known_bad_cells_fail(self):
        on_changes = ("veh_seed", "depth_value", "robot", "ablation", "sha", "fov", "no_telemetry", "no_motor_updates",
                      "hole_share", "no_record", "record_uncapped", "camtrain_receipt_var", "telemetry_v1_mode",
                      "telemetry_v1_tau", "record_v1_lower", "record_limit_not_raised")
        off_changes = ("off_with_treatment_key", "telemetry_when_off")
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            for change in on_changes:
                d = make_cell(t / ("on_" + change), "K2s2", 4503, change=change)
                self.assertTrue(R.receipt_problems(d, "K2s2", 4503, COMMITTED), change)
            for change in off_changes:
                d = make_cell(t / ("off_" + change), "K2s2_nom", 4503, change=change)
                self.assertTrue(R.receipt_problems(d, "K2s2_nom", 4503, COMMITTED), change)
            d = make_cell(t / "seed", "K2s2", 4503)
            self.assertEqual(R.receipt_problems(d, "K2s2", 4503, COMMITTED), [])          # control
            self.assertTrue(R.receipt_problems(d, "K2s2", 4501, COMMITTED))               # another cell seed
            self.assertTrue(R.receipt_problems(d, "K2s2_nom", 4503, COMMITTED))           # read as an OFF cell
            (t / "empty").mkdir()
            self.assertTrue(R.receipt_problems(t / "empty", "K2s2", 4503, COMMITTED))


class Checkpoints(unittest.TestCase):
    def test_camk2_pins_are_filled(self):
        """Fails until the CAM-K2 pins are filled from the TRAINED train reports (review R3-1), by design."""
        for seed in (1, 2):
            self.assertRegex(R.K2_SHA[seed], r"^[0-9a-f]{64}$", "CAM-K2 seed %d pin is still a placeholder" % seed)

    def write_camk2(self, d, seed, data, verdict="TRAINED", manifest_verdict="TRAINED", treatments=None, smoke=False,
                    manifest=True, unit="CAM-K2 v2", prereg=None):
        prereg = prereg or T2.PREREG
        ck_dir = d / ("train_s%d" % seed) / "checkpoint"
        ck_dir.mkdir(parents=True)
        copy = ck_dir / ("last_gen_ppo_ep_29000_rew_%d.pth" % seed)
        copy.write_bytes(data)
        s = hashlib.sha256(data).hexdigest()
        man = ck_dir / (copy.name + ".manifest.json")
        if manifest:
            man.write_text(json.dumps({"verdict": manifest_verdict, "checkpoint_sha256": s, "robot": "navrl_quad",
                                       "seed": seed, "unit": unit, "prereg": prereg,
                                       "treatments": treatments if treatments is not None else T2.expected_settings(seed)}))
        rep = {"seed": seed, "verdict": verdict, "development_smoke": smoke, "prereg": prereg,
               "scope": T2.SCOPE if prereg == T2.PREREG else "CAM-K2 v1: ...",
               "checkpoint": {"sha256": s, "copy": str(copy), "copy_sha256": s,
                              "manifest": {"file": man.name, "sha256": sha(man) if manifest else None}}}
        (d / ("train_s%d" % seed) / "train_report.json").write_text(json.dumps(rep))
        return s

    def run_case(self, pins=None, **kw):
        k1 = {"K1s1": ("k1", "1" * 64), "K1s2": ("k2", "2" * 64)}
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            s1 = self.write_camk2(t, 1, b"one", **kw)
            s2 = self.write_camk2(t, 2, b"two")
            with mock.patch.dict(R.K2_SHA, pins or {1: s1, 2: s2}), \
                    mock.patch.object(R.R1, "checkpoints", lambda d=None: (dict(k1, F=("f", "F" * 64)),
                                                                           ["Policy F missing"])):
                return R.checkpoints(camk2_dir=t)

    def test_trained_reports_with_manifests_pass(self):
        out, problems = self.run_case()
        self.assertEqual(problems, [])
        self.assertEqual(sorted(out), ["K1s1", "K1s2", "K2s1", "K2s2"])        # Policy F is not an arm here
        self.assertEqual(out["K2s1"][1], hashlib.sha256(b"one").hexdigest())

    def test_bad_reports_fail(self):
        self.assertTrue(self.run_case(pins={1: R.PLACEHOLDER, 2: hashlib.sha256(b"two").hexdigest()})[1])
        self.assertTrue(self.run_case(pins={1: "0" * 64, 2: hashlib.sha256(b"two").hexdigest()})[1])
        self.assertTrue(self.run_case(verdict="TRAINING_FAIL")[1])
        self.assertTrue(self.run_case(smoke=True)[1])
        self.assertTrue(self.run_case(manifest_verdict="TRAINING_FAIL")[1])
        self.assertTrue(self.run_case(treatments={"vehicle_rand": None})[1])
        self.assertTrue(self.run_case(manifest=False)[1])
        v1 = "docs/plans/prereg_camonly_k2_train_v1_2026-11-08.md"                  # CAM-K2 v1, withdrawn
        self.assertTrue(self.run_case(unit="CAM-K2 v1")[1])
        self.assertTrue(self.run_case(prereg=v1)[1])
        v1_treat = json.loads(json.dumps(T2.expected_settings(1)))
        v1_treat["vehicle_rand"].update(NAVRL_VEH_RAND_TW_NOMINAL=2.25, NAVRL_VEH_RAND_TAU_MIN=0.03,
                                        NAVRL_VEH_RAND_TAU_MAX=0.06)
        self.assertTrue(self.run_case(treatments=v1_treat)[1])

    def test_placeholders_are_refused_as_they_stand(self):
        with mock.patch.dict(R.K2_SHA, {1: R.PLACEHOLDER, 2: R.PLACEHOLDER}):
            _out, problems = R.checkpoints(camk2_dir=pathlib.Path("/nonexistent"))
        self.assertTrue(any("placeholder" in p for p in problems))


class TestV2Only(unittest.TestCase):
    """CAM-K2-E evaluates CAM-K2 v2; the withdrawn v1's constants, folder and wrapper are neither read nor accepted."""

    def test_runner_binds_v2(self):
        self.assertEqual(T2.__name__, "camk2v2_train")
        self.assertEqual(T2.PREREG, "docs/plans/prereg_camonly_k2_train_v2_2026-11-08.md")
        self.assertEqual(R.CAMK2_DIR, pathlib.Path.home() / "motar_tmp" / "camk2v2_20261108")
        self.assertEqual(R.CAMK2_UNIT, "CAM-K2 v2")
        self.assertIn("tools/camk2v2_train.py", R.COMMITTED_FILES)
        self.assertNotIn("tools/camk2_train.py", R.COMMITTED_FILES)
        src = (ROOT / "tools/camk2e_run.py").read_text()
        self.assertNotIn("import camk2_train", src)
        self.assertNotIn("camk2_20261108", src)
        self.assertIn("import camk2v2_train as T2", src)                          # positive control of the search
        for k in ("CAM-K2 v2 seeds 1 and 2", "T/W 4.0", "tau 0.02-0.04 s", "F's vehicle", "idle host"):
            self.assertIn(k, R.SCOPE)
        for gone in ("2.25", "0.03-0.06", "4.2"):
            self.assertNotIn(gone, R.SCOPE)
        self.assertNotIn("2.25", (ROOT / R.LAUNCHER).read_text())

    def test_treated_cells_carry_v2_values(self):
        vr = R.expected_settings(4501, True)["vehicle_rand"]
        self.assertEqual({k: vr[k] for k in ("NAVRL_VEH_RAND_TW_NOMINAL", "NAVRL_VEH_RAND_TW_SCALE",
                                             "NAVRL_VEH_RAND_MOTOR_IMBAL", "NAVRL_VEH_RAND_TAU_MIN",
                                             "NAVRL_VEH_RAND_TAU_MAX")},
                         {"NAVRL_VEH_RAND_TW_NOMINAL": 4.0, "NAVRL_VEH_RAND_TW_SCALE": 0.15,
                          "NAVRL_VEH_RAND_MOTOR_IMBAL": 0.05, "NAVRL_VEH_RAND_TAU_MIN": 0.02,
                          "NAVRL_VEH_RAND_TAU_MAX": 0.04})
        self.assertAlmostEqual(T2.E.settings({"NAVRL_REALVEH_RECEIPT": "/nonexistent/r.json",
                                              "NAVRL_SENSOR_ABLATION": "no_lidar", **R1_CAM(),
                                              **R.treatment_env(4501)})["worst_case_thrust_to_weight"],
                               4.0 * 0.85 * 0.95, places=12)
        self.assertEqual(vr["NAVRL_VEH_RAND_LIMIT_MODE"], "raise")

    def test_record_and_telemetry_rules_are_v2s(self):
        exp = R.expected_settings(4502, True)
        self.assertEqual(R.record_problems(record(4502), exp), [])                 # positive control
        self.assertEqual(R.telemetry_problems(telemetry_text(4502), exp), [])
        v1 = dict(record(4502), tw_nominal=2.25, mode="lower", cap_per_motor_n=[1.3795, 1.3795],
                  thrust_limit_per_motor_n=[1.3795, 1.3795])
        bad = {"v1 record": v1, "mode none": dict(record(4502), mode="none"),
               "limit at 2 N": dict(record(4502), thrust_limit_per_motor_n=[2.0, 2.0]),
               "own T/W off": dict(record(4502), own_thrust_to_weight=[4.0, 4.0]),
               "max thrust not 2 N": dict(record(4502), max_thrust_per_motor_n=[LIMIT, LIMIT]),
               "live max at 2 N": dict(record(4502), live_max_thrust_per_motor_n=[2.0, 2.0]),
               "no live read-back": {k: v for k, v in record(4502).items() if k != "live_max_thrust_per_motor_n"},
               "no mode": {k: v for k, v in record(4502).items() if k != "mode"},
               "no limit": {k: v for k, v in record(4502).items() if k != "thrust_limit_per_motor_n"}}
        for name, rec in bad.items():
            self.assertTrue(R.record_problems(rec, exp), name)
        for kw in ({"init_mode": "lower"}, {"init_mode": "none"}, {"tau": (0.03, 0.06)}):
            self.assertTrue(R.telemetry_problems(telemetry_text(4502, **kw), exp), kw)
        # review r1 M-1 negatives: the live read-back at navrl_quad's own 2 N, and no raise counters
        self.assertTrue(any("live max_thrust" in x for x in R.telemetry_problems(telemetry_text(4502, live=[2.0, 2.0]),
                                                                                  exp)))
        self.assertTrue(any("raise counters" in x for x in R.telemetry_problems(telemetry_text(4502, counters=False),
                                                                                 exp)))


class Gates(unittest.TestCase):
    def test_gate_a(self):
        self.assertEqual(E2.gate_a([-12, -6], [-13, -5.5], -9), "MATERIAL_ADAPTATION")
        self.assertEqual(E2.gate_a([-12, -6], [-13, -5.5], -9, timeout_mean=3.0), "CRASH_TO_TIMEOUT")
        self.assertEqual(E2.gate_a([-6, -1], [-7, -0.5], -3.5), "SMALL_ADAPTATION")
        self.assertEqual(E2.gate_a([-6, -1], [-7, -0.5], -3.5, timeout_mean=2.0), "CRASH_TO_TIMEOUT")
        self.assertEqual(E2.gate_a([-1.5, -0.3], [-1.8, -0.1], -0.9), "NEGLIGIBLE")     # the 2 pp floor
        self.assertEqual(E2.gate_a([2, 9], [1, 10], 5), "WORSE")
        self.assertEqual(E2.gate_a([0.5, 1.5], [0.3, 1.7], 1.0), "NEGLIGIBLE")
        self.assertEqual(E2.gate_a([-4, 3], [-5, 4], -1), "NO_DETECTED_ADAPTATION")
        self.assertEqual(E2.gate_a([-5, -1], [-6, 1], -3), "UNRESOLVED")
        self.assertEqual(E2.gate_a([0, 0], [0, 0], 0), "NEGLIGIBLE")                    # a null contrast
        self.assertEqual(E2.gate_a([-7, -5.5], [-8, -4.9], -6.5), "SMALL_ADAPTATION")    # t end above -5

    def test_gate_n_and_lineage(self):
        self.assertEqual(E2.gate_n([0.3, 2.0], [0.1, 2.5]), "NO_MATERIAL_LOSS")
        self.assertEqual(E2.gate_n([3.4, 6.0], [3.1, 6.5]), "MATERIAL_LOSS")
        self.assertEqual(E2.gate_n([1, 5], [0.5, 5.5]), "LOSS_INCONCLUSIVE")
        self.assertEqual(E1.lineage("NEGLIGIBLE", "NEGLIGIBLE"), "NEGLIGIBLE")
        self.assertEqual(E1.lineage("NEGLIGIBLE", "WORSE"), "SEEDS_DISAGREE")

    def test_gate_i_and_the_adaptation_rule(self):
        self.assertEqual(E2.gate_i([-9, -3], [-10, -2.5]), "INTERACTION_NEGATIVE")
        self.assertEqual(E2.gate_i([2, 6], [1.5, 6.5]), "INTERACTION_POSITIVE")
        self.assertEqual(E2.gate_i([-3, 2], [-4, 3]), "INTERACTION_NOT_DETECTED")
        self.assertEqual(E2.gate_i([-5, -1], [-6, 1]), "INTERACTION_UNRESOLVED")
        self.assertTrue(E2.adaptation_supported("MATERIAL_ADAPTATION", "INTERACTION_NEGATIVE"))
        self.assertTrue(E2.adaptation_supported("SMALL_ADAPTATION", "INTERACTION_NEGATIVE"))
        for a, i in (("MATERIAL_ADAPTATION", "INTERACTION_NOT_DETECTED"), ("NEGLIGIBLE", "INTERACTION_NEGATIVE"),
                     ("CRASH_TO_TIMEOUT", "INTERACTION_NEGATIVE"), ("MATERIAL_ADAPTATION", "INTERACTION_UNRESOLVED")):
            self.assertFalse(E2.adaptation_supported(a, i), (a, i))

    def test_t_interval_df(self):
        main = [(d, s) for d in (70, 115, 160, 205) for s in (1, 2, 3)]
        nominal = [(d, s) for d in (70, 205) for s in (1, 2, 3)]
        xs = [float(i % 5) for i in range(12)]
        t12, df12 = E2.strat_t(xs, main)
        self.assertEqual(df12, 8)
        self.assertEqual([round(v, 9) for v in t12], [round(v, 9) for v in E1.strat_t(xs, main)])  # = CAM-K1-E's
        t6, df6 = E2.strat_t(xs[:6], nominal)                 # xs[:6] = 0 1 2 | 3 4 0
        self.assertEqual(df6, 4)
        m = sum(xs[:6]) / 6
        se = ((1.0 / 3 + (13.0 / 3) / 3) / 4) ** 0.5             # var(0,1,2) = 1, var(3,4,0) = 13/3; 2 strata
        self.assertAlmostEqual(t6[1] - m, 2.776445105 * se, places=9)
        self.assertAlmostEqual(m - t6[0], 2.776445105 * se, places=9)
        self.assertGreater(E2.T_CRIT[4], E2.T_CRIT[8])            # control: 4 df is wider than 8 df
        blocks = main
        dens = [10.0 * (i // 3) + (i % 3) * 0.1 for i in range(12)]   # a fixed density effect does not widen it
        ci = E1.strat_ci(dens, blocks, random.Random(1))
        self.assertLess(ci[1] - ci[0], 1.0)
        self.assertLess(E2.strat_t(dens, blocks)[0][1] - E2.strat_t(dens, blocks)[0][0], 1.0)


def fake_rows(rows, arm, block, share=None):
    """Deterministic, arm-specific outcome changes of a CAM-K1-E cell's rows (each arm differs from every other)."""
    rng = random.Random("%s|%d|%d" % (arm, block[0], block[1]))
    share = share if share is not None else {"K1s1": 0.02, "K1s2": 0.03, "K2s1": 0.8, "K2s2": 0.8, "K2s1_nom": 0.04, "K2s2_nom": 0.05,
             "K1s1_on_negative": 0.02, "K2s1_smoke": 0.8, "K1s1_off_identity": 0.02}[arm]
    out = []
    for r in rows:
        if r.get("outcome") == "crash" and rng.random() < share:
            out.append(dict(r, outcome="capture"))
        elif r.get("outcome") == "capture" and rng.random() < 0.01:
            out.append(dict(r, outcome="crash"))
        else:
            out.append(dict(r))
    return out


def ref_dir(arm, block):
    return K1E / "eval_01" / ("K1s1" if "s1" in arm else "K1s2") / (CELL % block)


def write_fake_cell(out, arm, block, ckpt_sha, rows=None, change=None):
    """A cell as the launcher and the realveh entry would write it, from CAM-K1-E's cell of the same block."""
    ref = ref_dir(arm, block)
    out.mkdir(parents=True, exist_ok=True)
    shutil.copy(ref / "result.json", out / "result.json")
    src = [json.loads(x) for x in (ref / "episodes.jsonl").read_text().splitlines() if x.strip()]
    rows = rows if rows is not None else fake_rows(src, arm, block)
    (out / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    base = json.loads((ref / "receipt.json").read_text())["env_snapshot_navrl"]
    make_cell(out, arm, block[1], base_snapshot=base, ckpt_sha=ckpt_sha, change=change)
    if change == "bad_condition":
        res = json.loads((out / "result.json").read_text())
        res["condition"]["bars"] = -1
        (out / "result.json").write_text(json.dumps(res))
    if change == "extra_snap_key":
        rec = json.loads((out / "receipt.json").read_text())
        rec["env_snapshot_navrl"]["NAVRL_GAIN_RAND"] = "1"
        (out / "receipt.json").write_text(json.dumps(rec))


CK = {"K1s1": ["k1", "1" * 64], "K1s2": ["k2", "2" * 64], "K2s1": ["c1", "3" * 64], "K2s2": ["c2", "4" * 64]}


@unittest.skipUnless(HAVE_K1E, "CAM-K1-E raw cells not on this host")
class RunnerSmoke(unittest.TestCase):
    """tools/camk2e_run.py identity and eval end to end with the preflight and gr_run.run_cell patched."""

    def fake_run_cell(self, mode):
        def run_cell(root, protocol, out_root, level, density, seed, quota, run_id, scale, head, start, records,
                     timeout_s, extra=None, launcher=None, ckpt_sha=None):
            cid = CELL % (density, seed)
            out = out_root / level / cid
            self.assertEqual(launcher, R.LAUNCHER)
            self.assertEqual(extra["CAMK2E_CKPT_SHA"], ckpt_sha)
            self.assertEqual(extra["CAMK2E_TREATMENTS"], "on" if R.treated(level) else "off")
            self.assertEqual({k: extra[k] for k in R.TREATMENT_KEYS if k in extra},
                             R.treatment_env(seed) if R.treated(level) else {})
            ref = ref_dir(level, (density, seed))
            src = [json.loads(x) for x in (ref / "episodes.jsonl").read_text().splitlines() if x.strip()]
            if level == "K1s1_off_identity" and mode != "off_differs":
                rows = src                                            # equal to CAM-K1-E's cell
            elif level in ("K1s1_on_negative", "K1s1_on_repeat") and mode == "on_same":
                rows = src
            elif level == "K1s1_on_repeat" and mode != "repeat_differs":
                rows = fake_rows(src, "K1s1_on_negative", (density, seed))   # deterministic: equals the negative
            else:
                rows = fake_rows(src, "K1s1" if level == "K1s1_on_repeat" else level, (density, seed))
            change = "fov" if (mode == "wrong_fov" and level == "K2s2") else None
            write_fake_cell(out, level, (density, seed), ckpt_sha, rows=rows, change=change)
            rec = {"level": level, "cell_id": cid, "density": density, "seed": seed, "problems": [], "verdict": "OK",
                   "wall_s": 300.0 if mode == "slow" else 150.0}
            records.append(rec)
            return rec
        return run_cell

    def run_stage(self, t, stage, mode=""):
        ck = {k: tuple(v) for k, v in CK.items()}
        with mock.patch.object(R, "preflight", lambda p, a, o, k: ("c0ffee", ["output already exists"] if o.exists()
                                                                   else [], {})), \
                mock.patch.object(R, "checkpoints", lambda: (ck, [])), \
                mock.patch.object(R.T2, "committed_sha256", lambda root=None: dict(COMMITTED)), \
                mock.patch.object(R.G, "run_cell", self.fake_run_cell(mode)), \
                mock.patch.object(R.G, "host", lambda: {"fault_count": 0}), \
                mock.patch.object(R.K, "GpuBudget", lambda h: type("B", (), {"exhausted": lambda s: False})()):
            return R.main([stage, "--approved-commit", "c0ffee", "--camk1e-raw", str(K1E), "--out", str(t / stage)])

    def test_identity_then_eval_complete(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity"), 0)
            rep = json.loads((t / "identity" / "identity_report.json").read_text())
            self.assertEqual(rep["verdict"], "IDENTITY_PASS")
            self.assertTrue(rep["comparison"]["K1s1_off_identity"]["episodes_equal"])
            self.assertTrue(rep["comparison"]["K1s1_on_repeat"]["equals_K1s1_on_negative"])
            self.assertEqual(R.rate_problems(rep, K1E), [])                    # ratio 150/170 x 60 x 158 s x 1.1
            real = dict(rep, records=[dict(r, wall_s=175.0) for r in rep["records"]])
            self.assertEqual(R.rate_problems(real, K1E), [])                   # review r2 R2-1: realistic 205-bar cells pass
            slow = dict(rep, records=[dict(r, wall_s=300.0) for r in rep["records"]])
            self.assertTrue(R.rate_problems(slow, K1E))                        # a 1.8x slower path refuses
            self.assertEqual(self.run_stage(t, "eval"), 0)
            ev = json.loads((t / "eval" / "eval_run_report.json").read_text())
            self.assertEqual((ev["verdict"], len(ev["records"]), ev["cells_expected"]), ("COMPLETE", 60, 60))
            self.assertEqual([r["arm"] for r in ev["records"][:6]], list(R.MAIN_ARMS + R.NOMINAL_ARMS))
            self.assertEqual(ev["records"][0]["module_seeds"], [45011, 45012])
            self.assertIsNone(ev["records"][4]["module_seeds"])            # nominal: treatments off

    def test_known_bad_runs(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "eval"), 2)                    # no identity pass yet
            self.assertEqual(self.run_stage(t, "identity", "off_differs"), 1)
            self.assertEqual(json.loads((t / "identity" / "identity_report.json").read_text())["verdict"],
                             "IDENTITY_FAIL")
            self.assertEqual(self.run_stage(t, "eval"), 2)                    # a failed identity in the parent refuses
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity", "on_same"), 1)     # the treatments had no effect
        with tempfile.TemporaryDirectory() as t:                              # review r1 E-L2: ON not repeatable
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity", "repeat_differs"), 1)
            rep = json.loads((t / "identity" / "identity_report.json").read_text())
            self.assertFalse(rep["comparison"]["K1s1_on_repeat"]["equals_K1s1_on_negative"])
        with tempfile.TemporaryDirectory() as t:                              # review r1 E-m2: the rate gate
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity", "slow"), 0)        # 60 x 158 s x (300/170) x 1.1 > 3.5 h
            with mock.patch("sys.stdout", new_callable=io.StringIO) as out:
                self.assertEqual(self.run_stage(t, "eval"), 2)
            self.assertIn("rate gate", out.getvalue())
            self.assertFalse((t / "eval").exists())
        with self.assertRaises(SystemExit) as cm:                             # --max-hours is fixed
            R.main(["eval", "--approved-commit", "c0ffee", "--camk1e-raw", str(K1E), "--out", "/nonexistent/x",
                    "--max-hours", "5"])
        self.assertIn("fixed at 3.5", str(cm.exception))
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            self.assertEqual(self.run_stage(t, "identity"), 0)
            self.assertEqual(self.run_stage(t, "eval", "wrong_fov"), 1)        # stops at the first bad K2s2 cell
            ev = json.loads((t / "eval" / "eval_run_report.json").read_text())
            self.assertEqual(ev["verdict"], "INCOMPLETE")
            self.assertEqual(ev["records"][-1]["arm"], "K2s2")
            self.assertTrue(any("recorder camera hfov_deg" in p for p in ev["records"][-1]["problems"]))


@unittest.skipUnless(HAVE_K1E, "CAM-K1-E raw cells not on this host")
class AnalysisSmoke(unittest.TestCase):
    """Synthetic identity and eval folders built from CAM-K1-E's cells, with K2 arms that crash much less."""

    def build(self, t, k2s2_unadapted=False, k2s1_equals_k1s1=False, change_arm=None, change=None, uniform=False):
        ident, ev = t / "identity_01", t / "eval_01"
        records = []
        made = {}
        for arm, d, s in R.eval_plan():
            ref = ref_dir(arm, (d, s))
            src = [json.loads(x) for x in (ref / "episodes.jsonl").read_text().splitlines() if x.strip()]
            rows = made[("K1s1", d, s)] if (arm == "K2s1" and k2s1_equals_k1s1) else fake_rows(src, arm, (d, s))
            if arm == "K2s2" and k2s2_unadapted:
                rows = fake_rows(src, "K2s2_nom", (d, s))
            if uniform and arm in R.NOMINAL_ARMS:                     # the same improvement ON and OFF
                rows = fake_rows(src, arm, (d, s), share=0.8)
            made[(arm, d, s)] = rows
            ck = CK[R.arm_ckpt(arm, {k: k for k in CK})]
            write_fake_cell(ev / arm / (CELL % (d, s)), arm, (d, s), ck[1], rows=rows,
                            change=change if arm == change_arm else None)
            records.append({"arm": arm, "cell_id": CELL % (d, s), "density": d, "seed": s, "checkpoint_sha256": ck[1]})
            if (arm, d, s) in (("K1s1", 205, 4501), ("K2s1", 205, 4501)):
                name = "K1s1_on_negative" if arm == "K1s1" else "K2s1_smoke"
                (ident / name / (CELL % (d, s))).mkdir(parents=True)
                shutil.copy(ev / arm / (CELL % (d, s)) / "episodes.jsonl", ident / name / (CELL % (d, s)) /
                            "episodes.jsonl")
        env = {"PATH": "/x"}
        (ident / "identity_report.json").write_text(json.dumps(
            {"verdict": "IDENTITY_PASS", "commit": "c0ffee", "env_allowlisted": env, "camk1e_raw": str(K1E.resolve())}))
        (ev / "eval_run_report.json").write_text(json.dumps(
            {"verdict": "COMPLETE", "cells_expected": 60, "commit": "c0ffee", "env_allowlisted": env,
             "camk1e_raw": str(K1E.resolve()), "checkpoints": CK, "committed_sha256": COMMITTED,
             "identity_report": str(ident / "identity_report.json"), "records": records}))
        return ident, ev

    def analyse(self, **kw):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            ident, ev = self.build(t, **kw)
            with mock.patch.object(E2.E1, "file_matches_commit", lambda *a, **k: True), \
                    mock.patch.object(E2.R2, "checkpoints", lambda: ({}, [])):
                rc = E2.main(["eval", str(K1E), str(ident), str(ev), str(t / "out.json")])
            return rc, json.loads((t / "out.json").read_text())

    def test_known_adaptation_is_labelled_and_controls_pass(self):
        rc, res = self.analyse()
        self.assertEqual(rc, 0, res.get("binding"))
        self.assertEqual(res["reading"], "REPORTED")
        self.assertTrue(all(c["ok"] for c in res["controls"].values()), res["controls"])
        self.assertEqual(res["controls"]["A_on_K1E_F66_minus_K1s1"]["label"], "WORSE")
        self.assertEqual(res["gate_A"], "MATERIAL_ADAPTATION")
        c = res["contrasts"]["A_K2s1"]["crash"]
        self.assertAlmostEqual(c["mean_pp"], sum(c["blocks_pp"]) / 12, places=9)
        self.assertEqual(res["contrasts"]["N_K2s1"]["crash"]["t_df"], 4)
        self.assertEqual(len(res["contrasts"]["N_K2s1"]["crash"]["blocks_pp"]), 6)
        self.assertIn(res["gate_N"], ("NO_MATERIAL_LOSS", "LOSS_INCONCLUSIVE", "MATERIAL_LOSS", "SEEDS_DISAGREE"))
        self.assertEqual(res["gate_I"], "INTERACTION_NEGATIVE")                  # K2 gains far more ON than OFF
        self.assertTrue(res["adaptation_supported"])
        self.assertEqual(res["contrasts"]["I_K2s1"]["crash"]["t_df"], 4)
        self.assertEqual(len(res["contrasts"]["I_K2s1"]["crash"]["blocks_pp"]), 6)
        shares = res["raise_commands_per_cell"]                                  # review r1 M-1, descriptive
        self.assertEqual(len(shares), 48)
        self.assertTrue(all(isinstance(v, dict) and v["share_at_limit"] <= v["share_above_own_max"]
                            for v in shares.values()))

    def test_uniform_improvement_is_not_adaptation(self):
        """Review r1 E-M1: equal gains ON and OFF read gate A MATERIAL_ADAPTATION but not adaptation."""
        rc, res = self.analyse(uniform=True)
        self.assertEqual(rc, 0, res.get("binding"))
        self.assertEqual(res["gates_per_seed"]["A_K2s1"], "MATERIAL_ADAPTATION")
        self.assertNotEqual(res["gates_per_seed"]["I_K2s1"], "INTERACTION_NEGATIVE")
        self.assertFalse(res["adaptation_supported_per_seed"]["K2s1"])
        self.assertFalse(res["adaptation_supported"])

    def test_unadapted_seed_disagrees(self):
        rc, res = self.analyse(k2s2_unadapted=True)
        self.assertEqual(rc, 0, res.get("binding"))
        self.assertEqual(res["gates_per_seed"]["A_K2s1"], "MATERIAL_ADAPTATION")
        self.assertNotEqual(res["gates_per_seed"]["A_K2s2"], "MATERIAL_ADAPTATION")
        self.assertEqual(res["gate_A"], "SEEDS_DISAGREE")

    def test_known_bad_inputs_fail_binding(self):
        for kw in ({"k2s1_equals_k1s1": True}, {"change_arm": "K2s2", "change": "veh_seed"},
                   {"change_arm": "K1s1", "change": "extra_snap_key"},
                   {"change_arm": "K2s1_nom", "change": "bad_condition"},
                   {"change_arm": "K2s1", "change": "no_telemetry"}):
            rc, res = self.analyse(**kw)
            self.assertEqual((rc, res["reading"]), (1, "EVAL_BINDING_FAIL"), kw)


@unittest.skipUnless(HAVE_K1E, "CAM-K1-E raw cells not on this host")
class Controls(unittest.TestCase):
    def test_controls_on_camk1e_data(self):
        """Gate A on CAM-K1-E's F66 - K1s1 reads WORSE (known bad); on K1s1 - K1s2 it does not read
        MATERIAL_ADAPTATION; CAM-K1-E's cells match their committed manifest."""
        with tempfile.TemporaryDirectory() as t:
            out = pathlib.Path(t) / "controls.json"
            rc = E2.main(["controls", str(K1E), str(out)])
            res = json.loads(out.read_text())
        self.assertEqual(rc, 0, res)
        self.assertEqual(res["reading"], "CONTROLS_OK")
        self.assertTrue(res["camk1e_manifest_ok"])
        self.assertEqual(res["controls"]["A_on_K1E_F66_minus_K1s1"]["label"], "WORSE")
        self.assertGreater(res["controls"]["A_on_K1E_F66_minus_K1s1"]["crash"]["mean_pp"], 30)
        self.assertNotEqual(res["controls"]["A_on_K1E_K1s1_minus_K1s2"]["label"], "MATERIAL_ADAPTATION")
        # review r1 E-M2: the passing labels are reachable on these block counts
        self.assertEqual(res["controls"]["A_on_K1E_K1s1_minus_F66"]["label"], "MATERIAL_ADAPTATION")
        self.assertEqual(res["controls"]["N_on_K1E_K1s2_minus_K1s1"]["label"], "NO_MATERIAL_LOSS")
        self.assertEqual(res["controls"]["N_on_K1E_F66_minus_K1s1"]["label"], "MATERIAL_LOSS")
        self.assertEqual(res["controls"]["N_on_K1E_K1s2_minus_K1s1"]["crash"]["t_df"], 4)
        self.assertIn("half_width_pp", res["controls"]["A_on_K1E_K1s1_minus_K1s2"])

    def test_controls_mode_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as t:
            out = pathlib.Path(t) / "x.json"
            out.write_text("{}")
            with self.assertRaises(SystemExit):
                E2.main(["controls", str(K1E), str(out)])


if __name__ == "__main__":
    unittest.main()
