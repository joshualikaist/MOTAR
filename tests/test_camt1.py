"""CPU tests of CAM-T1 (docs/plans/prereg_camonly_t1_2026-11-12.md): tools/camt1_run.py, tools/camt1_analyse.py and the
launcher copy eval_navrl_camt1_arms.sh. No GPU, no simulator. Tests that need CAM-K2-E3 / CAM-K2-E raw cells or the
G3 traces skip when those folders are absent (another machine). Every rejection test has a passing twin."""
import contextlib
import io
import json
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
_saved_path = list(sys.path)
try:
    sys.path.insert(0, str(TOOLS))
    import camt1_run as R                                   # noqa: E402
    import camt1_analyse as X                               # noqa: E402
finally:
    sys.path[:] = _saved_path
_PATH_RESTORED = sys.path == _saved_path

HOME = pathlib.Path.home()
E3 = HOME / "motar_tmp/camk2v3e_20261110/raw"
E3_CELL = E3 / "eval_01/V3s1/E2_obstacle_aware__205bars__seed4501"
K2E = HOME / "motar_tmp/camk2e_20261108/raw"
G3_RUN = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/camonly_c2trace_v1")
OT = R.load_ot()
LAUNCHER = ROOT / R.LAUNCHER
CAMK2E_LAUNCHER = ROOT / "aerial_gym/rl_training/rl_games/eval_navrl_camk2e_arms.sh"
CKPTS = {"V3s1": ("/ck/s1.pth", "1" * 64), "V3s2": ("/ck/s2.pth", "2" * 64)}
NATIVE_SNAP = {"NAVRL_SPEED_GOVERNOR": "riskcap", "NAVRL_SPEED_GOVERNOR_FIXED_MPS": "2.0",
               "NAVRL_SPEED_GOVERNOR_FREE_MPS": R.NATIVE_FREE, "NAVRL_MAX_VELOCITY": "2.5",
               "NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_KROC2027_RUN_ID": "REF", "NAVRL_EVAL_RUN_NONCE": "a"}


def write_cell(d, snap=None, cond=None, outcome=(10, 5, 0), commanded=(2.9, 3.1), episodes=b'{"a": 1}\n',
               kroc_extra=b""):
    """A minimal finished cell: receipt snapshot, result (condition, outcome), both ledgers."""
    d = pathlib.Path(d)
    d.mkdir(parents=True, exist_ok=True)
    (d / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": dict(NATIVE_SNAP if snap is None else snap)}))
    cap, crash, timeout = outcome
    (d / "result.json").write_text(json.dumps({
        "condition": dict({"speed_governor_free_mps": 3.53553390593, "bars": 205} if cond is None else cond),
        "outcome": {"captured": cap, "crash": crash, "timeout": timeout}}))
    (d / "episodes.jsonl").write_bytes(episodes)
    rows = "".join(json.dumps({"env_index": i, "env_episode_index": 0, "pursuer_commanded_speed_max_mps": c}) + "\n"
                   for i, c in enumerate(commanded))
    (d / "kroc2027_episodes.jsonl").write_bytes(rows.encode() + kroc_extra)
    return d


def write_trace(d, frames, layouts, hfov=66.0, vfov=54.0, translation=(0.1, 0.0, 0.03), dt=0.1):
    """A trace through the production writer: layouts = {(env, episode): (center [B, 3], half [B, 3])}."""
    w = OT.TraceWriter(d, {"dt_s": dt, "camera": {"hfov_deg": hfov, "vfov_deg": vfov, "max_range_m": 20.0,
                                                   "translation_m": list(translation)}})
    for (env, ep), (c, h) in layouts.items():
        w.add_reset(env, ep, "scene-%d-%d" % (env, ep), c, h)
    w.write_frames(frames)
    return w.close("complete")


def episode(env, ep, xy, yaw, vel=None, z=1.0, flags=0):
    """FRAME_DTYPE rows of one episode: positions xy [n, 2], yaw [n] (vehicle and body), velocity [n, 2]."""
    n = len(xy)
    rec = np.zeros(n, dtype=OT.FRAME_DTYPE)
    rec["env"], rec["episode"], rec["frame"] = env, ep, np.arange(n)
    rec["p_pos"][:, :2] = xy
    rec["p_pos"][:, 2] = z
    v = np.zeros((n, 2)) if vel is None else np.asarray(vel, float)
    rec["p_vel"][:, :2] = v
    q = np.stack([np.zeros(n), np.zeros(n), np.sin(np.asarray(yaw) / 2), np.cos(np.asarray(yaw) / 2)], axis=1)
    rec["p_quat"], rec["p_vehicle_quat"] = q, q
    rec["flags"] = flags
    return rec


def bar(x, y, hx=0.25, hy=0.25):
    return np.array([[x, y, 1.5]]), np.array([[hx, hy, 1.5]])


PRIMARY = dict(X.PROFILES["primary"])
G3P = dict(X.PROFILES["g3"])


class TestDesign(unittest.TestCase):
    def test_constants_and_plans(self):
        self.assertTrue(_PATH_RESTORED)
        self.assertEqual(R.PREREG, "docs/plans/prereg_camonly_t1_2026-11-12.md")
        for f in (R.PREREG, R.LAUNCHER, "tools/camt1_run.py", "tools/camt1_analyse.py", R.E3_MANIFEST,
                  "tools/actionhold_hook.py", "tools/actionhold_site/sitecustomize.py", "motar_eval/occlusion_trace.py"):
            self.assertIn(f, R.COMMITTED_FILES)
        self.assertEqual(R.BRANCH, "camonly/t1")
        self.assertEqual((R.MIN_FREE_GB, R.MAX_HOURS), (1.5, {"identity": 0.25, "trace": 2.0, "control": 0.5,
                                                              "envelope": 2.5}))
        self.assertEqual(R.identity_plan(), [("native_identity", 205, 4501)])          # review r1 F4: first, alone
        self.assertEqual([a for a, _, _ in R.control_plan()], ["k1d0_identity", "d1_smoke", "cap20_smoke"])
        self.assertEqual(list(R.PLANS), ["identity", "trace", "control", "envelope"])
        self.assertEqual(R.G.sha(LAUNCHER), R.LAUNCHER_SHA)
        tp, cp, ep = R.trace_plan(), R.control_plan(), R.envelope_plan()
        self.assertEqual((len(tp), len(cp), len(ep)), (24, 3, 36))
        self.assertEqual(tp[:3], [("V3s1", 70, 4501), ("V3s2", 70, 4501), ("V3s1", 70, 4502)])
        self.assertEqual(ep[:7], [("cap25s1", 115, 4501), ("cap25s2", 115, 4501), ("cap20s1", 115, 4501),
                                  ("cap20s2", 115, 4501), ("d1s1", 115, 4501), ("d1s2", 115, 4501),
                                  ("cap25s1", 115, 4502)])
        self.assertEqual(sorted({(d, s) for _, d, s in ep}), [(d, s) for d in (115, 205) for s in (4501, 4502, 4503)])
        self.assertEqual(ep[-1], ("d1s2", 205, 4503))
        self.assertEqual(R.V3.V3_SHA[1][:8] + R.V3.V3_SHA[2][:8], "888e8056203deea4")

    def test_arm_specs(self):
        want = {"V3s1": (1, R.NATIVE_FREE, True, None), "V3s2": (2, R.NATIVE_FREE, True, None),
                "native_identity": (1, R.NATIVE_FREE, False, None), "k1d0_identity": (1, R.NATIVE_FREE, False, (1, 0)),
                "d1_smoke": (1, R.NATIVE_FREE, False, (1, 1)), "cap20_smoke": (1, "2.0", False, None),
                "cap25s1": (1, "2.5", False, None), "cap25s2": (2, "2.5", False, None),
                "cap20s2": (2, "2.0", False, None), "d1s1": (1, R.NATIVE_FREE, False, (1, 1)),
                "d1s2": (2, R.NATIVE_FREE, False, (1, 1))}
        for arm, (seed, free, trace, hook) in want.items():
            s = R.arm_spec(arm)
            self.assertEqual((s["policy_seed"], s["free"], s["trace"], s["hook"], s["ref_arm"]),
                             (seed, free, trace, hook, "V3s%d" % seed), arm)
        for bad in ("cap30s1", "d2s1", "V3s3", "cap25s3"):
            with self.assertRaises(KeyError):
                R.arm_spec(bad)

    def test_cell_env_construction(self):
        base_keys = None
        for arm, d, s in R.identity_plan() + R.trace_plan() + R.control_plan() + R.envelope_plan():
            e = R.cell_extra(arm, s, CKPTS)
            spec = R.arm_spec(arm)
            self.assertEqual(e["CAMT1_GOVERNOR_FREE_MPS"], spec["free"])
            self.assertEqual(e["CAMK2E_CKPT_SHA"], CKPTS[spec["ref_arm"]][1])
            self.assertEqual(e["CAMK2E_TREATMENTS"], "on")
            self.assertEqual(e["NAVRL_SENSOR_ABLATION"], "no_lidar")
            for k, v in {**R.R2.treatment_env(s), **R.V3.VALIDITY_ENV, **R.R1.CAM_VALUES}.items():
                self.assertEqual(e[k], v, (arm, k))
            self.assertNotIn("NAVRL_MAX_VELOCITY", e)
            self.assertNotIn("NAVRL_SPEED_GOVERNOR_FREE_MPS", e)            # only through the launcher's check
            self.assertEqual("NAVRL_KROC2027_OCCLUSION_TRACE" in e, spec["trace"], arm)
            if spec["hook"]:
                self.assertEqual(e["PYTHONPATH"], "%s:%s" % (R.ROOT, R.SITE))
                self.assertEqual((e["MOTAR_ACTION_HOLD_K"], e["MOTAR_ACTION_DELAY_D"]),
                                 tuple(str(x) for x in spec["hook"]))
            else:
                self.assertFalse(any(k.startswith("MOTAR_") or k == "PYTHONPATH" for k in e), arm)
            keys = set(e) - {"NAVRL_KROC2027_OCCLUSION_TRACE", "PYTHONPATH", "MOTAR_ACTION_HOLD_K",
                             "MOTAR_ACTION_DELAY_D"}
            base_keys = keys if base_keys is None else base_keys
            self.assertEqual(keys, base_keys, arm)                          # one key set for every arm
        self.assertEqual(R.cell_extra("d1s2", 4502, CKPTS)["MOTAR_ACTION_DELAY_D"], "1")
        self.assertEqual(R.cell_extra("k1d0_identity", 4501, CKPTS)["MOTAR_ACTION_DELAY_D"], "0")

    def test_inherited_environment(self):
        good = {"PYTHONPATH": str(R.ROOT), "HOME": "/h", "MOTARX": "1"}
        self.assertEqual(R.inherited_problems(good), [])                    # positive control
        for k in ("MOTAR_ACTION_HOLD_K", "MOTAR_ACTION_DELAY_D", "CAMT1_GOVERNOR_FREE_MPS", "CAMT1_ANYTHING"):
            self.assertTrue(R.inherited_problems(dict(good, **{k: "1"})), k)
        self.assertTrue(R.inherited_problems({"HOME": "/h"}))                # PYTHONPATH unset
        self.assertTrue(R.inherited_problems({"PYTHONPATH": "/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator"}))
        self.assertTrue(R.inherited_problems({"PYTHONPATH": "%s:/x" % R.ROOT}))

    def test_protocol_problems_keep_the_pins(self):
        protocol = json.loads(R.K.PROTOCOL.read_text())
        self.assertEqual(R.protocol_problems(protocol), [])
        bad = json.loads(json.dumps(protocol))
        bad["policy"]["player_sha256"] = "0" * 64
        self.assertTrue(R.protocol_problems(bad))
        bad = json.loads(json.dumps(protocol))
        bad["policy"]["launcher"] = "aerial_gym/none.sh"
        self.assertTrue(R.protocol_problems(bad))

    def test_rate_gates(self):
        recs = [{"wall_s": 100.0}] * 4
        self.assertFalse(R.rate_stop(0.0, [], 24, 2.0))
        self.assertFalse(R.rate_stop(400.0, recs, 20, 2.0))                 # 400 + 100 x 20 x 1.1 = 2600 s
        self.assertTrue(R.rate_stop(400.0, recs, 70, 2.0))                  # 400 + 7700 > 7200 s
        self.assertIsNone(R.envelope_projection({"records": []}, pathlib.Path("/nonexistent")))

    @unittest.skipUnless((E3 / "eval_01/eval_run_report.json").is_file(), "CAM-K2-E3 raw absent")
    def test_envelope_projection_real(self):
        ev = json.loads((E3 / "eval_01/eval_run_report.json").read_text())
        ref = [r["wall_s"] for r in ev["records"] if (r["arm"], r["density"], r["seed"]) == ("V3s1", 205, 4501)][0]
        p = R.envelope_projection({"records": [{"wall_s": ref}] * 3}, E3)
        self.assertAlmostEqual(p["ratio"], 1.0)
        self.assertLess(p["projected_s"], R.MAX_HOURS["envelope"] * 3600)
        slow = R.envelope_projection({"records": [{"wall_s": 3 * ref}] * 3}, E3)
        self.assertGreater(slow["projected_s"], R.MAX_HOURS["envelope"] * 3600)

    @unittest.skipUnless(E3_CELL.is_dir(), "CAM-K2-E3 raw absent")
    def test_e3_references_bound_by_manifest(self):
        self.assertEqual(R.e3_problems(E3), [])
        self.assertTrue(R.e3_problems(pathlib.Path("/nonexistent/raw")))
        self.assertEqual(len(R.reference_files(R.trace_plan())), 24 * 4)
        self.assertEqual(set(R.reference_files(R.envelope_plan() + R.control_plan() + R.identity_plan()))
                         - set(R.reference_files(R.trace_plan())),
                         set())


class TestLauncher(unittest.TestCase):
    def run_launcher(self, free):
        env = {"PATH": os.environ.get("PATH", "/usr/bin:/bin"), "HOME": str(HOME),
               "PYTHON": sys.executable, "NAVRL_SENSOR_ABLATION": "no_lidar", "CAMEVAL_PY": "/bin/true",
               "CAMK2E_TREATMENTS": "on", "NAVRL_VEH_RAND": "1", "NAVRL_DEPTH_MODEL": "oakd",
               "CAMK2E_CKPT": "/nonexistent/ckpt.pth", "CAMK2E_CKPT_SHA": "0" * 64}
        if free is not None:
            env["CAMT1_GOVERNOR_FREE_MPS"] = free
        return subprocess.run(["bash", str(LAUNCHER)], env=env, capture_output=True, text=True, timeout=60)

    def test_cap_validation(self):
        for free in ("3.53553390593", "2.5", "2.0"):                        # pass the check, stop at the checkpoint
            r = self.run_launcher(free)
            self.assertEqual(r.returncode, 2, free)
            self.assertIn("[arms] missing checkpoint", r.stderr, free)
            self.assertNotIn("[camt1]", r.stderr, free)
        for free in (None, "", "2.50", "3.5355", "3.0", "2", "1.0", " 2.5", "2.5;id"):
            r = self.run_launcher(free)
            self.assertEqual(r.returncode, 2, free)
            self.assertIn("[camt1] CAMT1_GOVERNOR_FREE_MPS must be exactly", r.stderr, free)
            self.assertNotIn("missing checkpoint", r.stderr, free)

    def test_only_marked_changes(self):
        a = CAMK2E_LAUNCHER.read_text().splitlines()
        b = LAUNCHER.read_text().splitlines()
        self.assertEqual(b[0], "#!/usr/bin/env bash")
        header = b[1:b.index(a[1])]
        self.assertTrue(all(x.startswith("#") for x in header))
        body = b[b.index(a[1]):]
        added = [x for x in body if x not in a]
        removed = [x for x in a[1:] if x not in body]
        self.assertTrue(added and all("[camt1]" in x or x.startswith(("case ", "    3.5", "    *)", "esac"))
                                      for x in added), added)
        self.assertEqual(removed, ["export NAVRL_SPEED_GOVERNOR_FREE_MPS=3.53553390593"])
        self.assertIn('export NAVRL_SPEED_GOVERNOR_FREE_MPS="${CAMT1_GOVERNOR_FREE_MPS}"   # [camt1] 1', body)
        self.assertIn("export NAVRL_MAX_VELOCITY=2.5 NAVRL_YAW_RATE_MAX=3.0", body)
        self.assertIn("export NAVRL_SPEED_GOVERNOR_FIXED_MPS=2.0", body)
        self.assertEqual(R.G.sha(CAMK2E_LAUNCHER), R.R2.LAUNCHER_SHA)       # the source copy is unchanged


class TestCellChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = pathlib.Path(self.tmp.name)
        self.ref = write_cell(self.d / "ref")

    def tearDown(self):
        self.tmp.cleanup()

    def cell(self, name, **kw):
        snap = dict(NATIVE_SNAP, NAVRL_KROC2027_RUN_ID="CAMT1-X", NAVRL_EVAL_RUN_NONCE="b")
        snap.update(kw.pop("snap_update", {}))
        for k in kw.pop("snap_drop", ()):
            snap.pop(k)
        return write_cell(self.d / name, snap=snap, **kw)

    def test_native_passes_and_deviations_fail(self):
        self.assertEqual(R.camt1_problems(self.cell("ok"), "native_identity", self.ref), [])
        bad = {"maxvel": self.cell("mv", snap_update={"NAVRL_MAX_VELOCITY": "2.0"}),
               "fixed": self.cell("fx", snap_update={"NAVRL_SPEED_GOVERNOR_FIXED_MPS": "1.5"}),
               "free": self.cell("fr", snap_update={"NAVRL_SPEED_GOVERNOR_FREE_MPS": "2.5"}),
               "other_key": self.cell("ok2", snap_update={"NAVRL_SEED": "9"}),
               "trace_key": self.cell("tk", snap_update={"NAVRL_KROC2027_OCCLUSION_TRACE": "1"}),
               "condition": self.cell("cd", cond={"speed_governor_free_mps": 3.53553390593, "bars": 115}),
               "too_fast": self.cell("tf", commanded=(2.9, 3.6))}
        for name, c in bad.items():
            self.assertTrue(R.camt1_problems(c, "native_identity", self.ref), name)

    def test_cap_arm_keys(self):
        good = self.cell("cap", snap_update={"NAVRL_SPEED_GOVERNOR_FREE_MPS": "2.0"},
                         cond={"speed_governor_free_mps": 2.0, "bars": 205}, commanded=(1.99, 2.0005))
        self.assertEqual(R.camt1_problems(good, "cap20s1", self.ref), [])  # positive control
        self.assertTrue(R.camt1_problems(good, "cap25s1", self.ref))       # another cap
        self.assertTrue(R.camt1_problems(good, "native_identity", self.ref))
        cases = {"fast": dict(commanded=(1.9, 2.01)),
                 "cond_not_changed": dict(cond={"speed_governor_free_mps": 3.53553390593, "bars": 205}),
                 "cond_other": dict(cond={"speed_governor_free_mps": 2.0, "bars": 115})}
        for name, kw in cases.items():
            c = self.cell("c_" + name, snap_update={"NAVRL_SPEED_GOVERNOR_FREE_MPS": "2.0"},
                          **dict({"cond": {"speed_governor_free_mps": 2.0, "bars": 205}, "commanded": (1.9,)}, **kw))
            self.assertTrue(R.camt1_problems(c, "cap20s1", self.ref), name)
        snap_native = self.cell("c_snap", cond={"speed_governor_free_mps": 2.0, "bars": 205}, commanded=(1.9,))
        self.assertTrue(R.camt1_problems(snap_native, "cap20s1", self.ref))

    def test_hook_sidecar(self):
        def sidecar(c, **kw):
            sc = dict({"config": {"hold_k": 1, "delay_d": 1}, "patch_applied": True, "calls": 600,
                       "delayed_zero_env_steps": 128, "held_env_steps": 0, "latched_env_steps": 0}, **kw)
            (c / "actionhold_hook.json").write_text(json.dumps(sc))
            return c
        self.assertEqual(R.camt1_problems(sidecar(self.cell("d1")), "d1s1", self.ref), [])
        self.assertTrue(R.camt1_problems(self.cell("d1_none"), "d1s1", self.ref))
        self.assertTrue(R.camt1_problems(sidecar(self.cell("d1_cfg"), config={"hold_k": 1, "delay_d": 0}), "d1s1",
                                         self.ref))
        self.assertTrue(R.camt1_problems(sidecar(self.cell("d1_np"), patch_applied=False), "d1s1", self.ref))
        self.assertTrue(R.camt1_problems(sidecar(self.cell("d1_idle"), delayed_zero_env_steps=0), "d1s1", self.ref))
        k1 = sidecar(self.cell("k1d0"), config={"hold_k": 1, "delay_d": 0}, delayed_zero_env_steps=0)
        self.assertEqual(R.camt1_problems(k1, "k1d0_identity", self.ref), [])
        self.assertTrue(R.camt1_problems(sidecar(self.cell("nat_sc")), "native_identity", self.ref))

    def test_trace_checks(self):
        frames = episode(0, 0, np.zeros((3, 2)), np.zeros(3))
        lay = {(0, 0): bar(3.0, 0.0)}

        def traced(name, **kw):
            c = self.cell(name, snap_update={"NAVRL_KROC2027_OCCLUSION_TRACE": "1"})
            write_trace(c, frames, lay, **kw)
            return c
        self.assertEqual(R.camt1_problems(traced("t"), "V3s1", self.ref), [])          # positive control
        self.assertTrue(R.camt1_problems(traced("t_fov", hfov=87.0), "V3s1", self.ref))
        self.assertTrue(R.camt1_problems(traced("t_mount", translation=(0.0, 0.0, 0.0)), "V3s1", self.ref))
        t_bad = traced("t_sha")
        with open(t_bad / OT.FRAMES_FILE, "ab") as fh:
            fh.write(b"\0")
        self.assertTrue(R.camt1_problems(t_bad, "V3s1", self.ref))
        no_trace = self.cell("t_none", snap_update={"NAVRL_KROC2027_OCCLUSION_TRACE": "1"})
        self.assertTrue(R.camt1_problems(no_trace, "V3s1", self.ref))
        self.assertTrue(R.camt1_problems(traced("t_untraced"), "native_identity", self.ref))
        p, n, l = R.trace_problems(traced("t_counts"), OT)
        self.assertEqual((p, n, l), ([], 3, 1))

    @unittest.skipUnless(E3_CELL.is_dir(), "CAM-K2-E3 raw absent")
    def test_real_e3_cell(self):
        self.assertEqual(R.camt1_problems(E3_CELL, "native_identity", E3_CELL), [])     # the reference itself
        self.assertTrue(R.camt1_problems(E3_CELL, "cap20_smoke", E3_CELL))
        self.assertTrue(R.camt1_problems(E3_CELL, "cap25s1", E3_CELL))
        self.assertTrue(R.camt1_problems(E3_CELL, "V3s1", E3_CELL))                     # no trace
        self.assertTrue(R.camt1_problems(E3_CELL, "d1s1", E3_CELL))                      # no sidecar
        other = E3 / "eval_01/V3s1/E2_obstacle_aware__115bars__seed4501"
        self.assertTrue(R.camt1_problems(E3_CELL, "native_identity", other))             # another block


class TestComparisons(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = pathlib.Path(self.tmp.name)
        self.ref = write_cell(self.d / "ref")

    def tearDown(self):
        self.tmp.cleanup()

    def test_noninterference(self):
        same = write_cell(self.d / "same")
        r = R.noninterference(same, self.ref)
        self.assertTrue(r["pass"] and r["episodes_equal"] and r["kroc_equal"] and r["outcome_equal"])
        cases = {"episodes": write_cell(self.d / "e", episodes=b'{"a": 2}\n'),
                 "episodes_whitespace": write_cell(self.d / "w", episodes=b'{"a":1}\n'),
                 "kroc": write_cell(self.d / "k", kroc_extra=b"\n"),
                 "outcome": write_cell(self.d / "o", outcome=(9, 6, 0))}
        for name, c in cases.items():
            self.assertFalse(R.noninterference(c, self.ref)["pass"], name)
        missing = write_cell(self.d / "m")
        (missing / "kroc2027_episodes.jsonl").unlink()
        r = R.noninterference(missing, self.ref)
        self.assertFalse(r["pass"])
        self.assertIn("error", r)

    def test_result_equal_is_required_for_identity(self):
        """Review r1 F3: byte-equal ledgers with a different result.json are not an identity."""
        same = write_cell(self.d / "same")
        r = R.ledgers_equal(same, self.ref)
        self.assertTrue(r["pass"] and r["result_equal"] and r["identity_pass"])          # positive control
        other = write_cell(self.d / "other")
        res = json.loads((other / "result.json").read_text())
        res["crash_causes"] = {"bar": 5}
        (other / "result.json").write_text(json.dumps(res))
        r = R.ledgers_equal(other, self.ref)
        self.assertTrue(r["pass"])                                    # ledgers and counts still equal
        self.assertFalse(r["result_equal"] or r["identity_pass"])
        self.assertEqual(R.identity_reading({"native_identity": R.ledgers_equal(same, self.ref)}), "IDENTITY_PASS")
        self.assertEqual(R.identity_reading({"native_identity": r}), "NOT_REPRODUCED")
        self.assertEqual(R.identity_reading({}), "NOT_REPRODUCED")
        nonce = write_cell(self.d / "nonce")                          # review r2 N7: the per-run nonce differs
        res = json.loads((nonce / "result.json").read_text())
        res["condition"]["evaluation_nonce"] = "b" * 64
        (nonce / "result.json").write_text(json.dumps(res))
        self.assertTrue(R.ledgers_equal(nonce, self.ref)["identity_pass"])
        ckpt = write_cell(self.d / "ckpt")                            # the absolute checkpoint path differs
        res = json.loads((ckpt / "result.json").read_text())
        res["checkpoint"] = "/elsewhere/ck.pth"
        (ckpt / "result.json").write_text(json.dumps(res))
        self.assertTrue(R.ledgers_equal(ckpt, self.ref)["identity_pass"])
        other_cond = write_cell(self.d / "cond")                      # any other condition key is a difference
        res = json.loads((other_cond / "result.json").read_text())
        res["condition"]["bars"] = 115
        (other_cond / "result.json").write_text(json.dumps(res))
        self.assertFalse(R.ledgers_equal(other_cond, self.ref)["identity_pass"])

    def test_control_reading(self):
        eq = {"pass": True, "episodes_equal": True, "identity_pass": True}
        ne = {"pass": False, "episodes_equal": False, "identity_pass": False}
        good = {"k1d0_identity": eq, "d1_smoke": ne, "cap20_smoke": ne}
        self.assertEqual(R.control_reading(good), "CONTROL_PASS")
        self.assertEqual(R.control_reading(dict(good, k1d0_identity=ne)), "CONTROL_FAIL")
        self.assertEqual(R.control_reading(dict(good, k1d0_identity=dict(eq, identity_pass=False))), "CONTROL_FAIL")
        self.assertEqual(R.control_reading(dict(good, d1_smoke=eq)), "CONTROL_FAIL")
        self.assertEqual(R.control_reading(dict(good, cap20_smoke=eq)), "CONTROL_FAIL")
        self.assertEqual(R.control_reading({k: v for k, v in good.items() if k != "d1_smoke"}), "CONTROL_FAIL")

    def test_trace_reading(self):
        ok = [{"verdict": "OK", "noninterference": {"pass": True}}] * 24
        self.assertEqual(R.trace_reading(ok, False), "NONINTERFERENCE_PASS")
        self.assertEqual(R.trace_reading(ok[:-1] + [{"verdict": "OK", "noninterference": {"pass": False}}], False),
                         "TRACE_INTERFERES")
        self.assertEqual(R.trace_reading(ok[:20], False), "INCOMPLETE")
        self.assertEqual(R.trace_reading(ok, True), "VOID_HOST_FAULT")
        self.assertEqual(R.trace_reading(ok[:-1] + [{"verdict": "FAIL"}], False), "INCOMPLETE")

    def test_stage_dependency(self):
        parent = self.d / "raw"
        parent.mkdir()
        head = "a" * 40
        p, prev = R.stage_dependency("identity", parent / "identity_01", head)
        self.assertEqual((p, prev), ([], None))
        p, _ = R.stage_dependency("trace", parent / "trace_01", head)
        self.assertTrue(p)                                                  # review r1 F4: no identity yet
        (parent / "identity_01").mkdir()
        (parent / "identity_01" / R.REPORT["identity"]).write_text(json.dumps({"reading": "NOT_REPRODUCED",
                                                                                "commit": head}))
        p, _ = R.stage_dependency("trace", parent / "trace_01", head)
        self.assertTrue(p)                                                  # NOT_REPRODUCED stops the trace
        (parent / "identity_01" / R.REPORT["identity"]).write_text(json.dumps({"reading": "IDENTITY_PASS",
                                                                                "commit": head}))
        p, prev = R.stage_dependency("trace", parent / "trace_01", head)
        self.assertEqual(p, [])
        self.assertEqual(prev["reading"], "IDENTITY_PASS")
        p, _ = R.stage_dependency("control", parent / "control_01", head)
        self.assertTrue(p)                                                  # no trace report yet
        (parent / "trace_01").mkdir()
        (parent / "trace_01" / R.REPORT["trace"]).write_text(json.dumps({"reading": "TRACE_INTERFERES",
                                                                          "commit": head}))
        p, prev = R.stage_dependency("control", parent / "control_01", head)
        self.assertEqual(p, [])
        self.assertEqual(prev["reading"], "TRACE_INTERFERES")
        p, _ = R.stage_dependency("control", parent / "control_01", "b" * 40)
        self.assertTrue(p)                                                  # another commit
        p, _ = R.stage_dependency("envelope", parent / "envelope_01", head)
        self.assertTrue(p)                                                  # no control report
        (parent / "control_01").mkdir()
        (parent / "control_01" / R.REPORT["control"]).write_text(json.dumps({"reading": "CONTROL_FAIL",
                                                                              "commit": head}))
        p, _ = R.stage_dependency("envelope", parent / "envelope_01", head)
        self.assertTrue(p)                                                  # CONTROL_FAIL
        (parent / "control_01" / R.REPORT["control"]).write_text(json.dumps({"reading": "CONTROL_PASS",
                                                                              "commit": head}))
        p, prev = R.stage_dependency("envelope", parent / "envelope_01", head)
        self.assertEqual(p, [])
        p, _ = R.stage_dependency("control", parent / "control_02", head)
        self.assertTrue(p)                                                  # a control report already exists


class TestGeometry(unittest.TestCase):
    def view(self, xy, yaw, c, h, **prof):
        p = dict(PRIMARY, **prof)
        o, oz = X.origins(np.array([[xy[0], xy[1], 1.0]]), np.array([yaw]), (0.1, 0.0, 0.03), p["origin"])
        return bool(X.bar_view(o, oz, np.array([yaw]), np.asarray(c, float), np.asarray(h, float), p)[0][0])

    def test_fov_positive_and_negative(self):
        h = [0.2, 0.2, 1.5]
        self.assertTrue(self.view((0, 0), 0.0, [3, 0, 1.5], h))                  # straight ahead
        self.assertFalse(self.view((0, 0), 0.0, [0, 3, 1.5], h))                 # beside, 90 deg
        self.assertFalse(self.view((0, 0), 0.0, [-3, 0, 1.5], h))                # behind
        self.assertTrue(self.view((0, 0), math.pi / 2, [0, 3, 1.5], h))          # the heading turns to it
        self.assertFalse(self.view((0, 0), -math.pi / 2, [0, 3, 1.5], h))

    def test_half_fov_parameter(self):
        h = [0.01, 0.01, 1.5]
        c = [3 * math.cos(math.radians(40)), 3 * math.sin(math.radians(40)) + 0.0, 1.5]
        for test in ("footprint", "nearest_point"):
            self.assertFalse(self.view((0, 0), 0.0, c, h, half_fov_deg=33.0, origin="centre", fov_test=test), test)
            self.assertTrue(self.view((0, 0), 0.0, c, h, half_fov_deg=43.5, origin="centre", fov_test=test), test)
            c2 = [c[0], -c[1], 1.5]                                              # the mirror bearing, right side
            self.assertFalse(self.view((0, 0), 0.0, c2, h, half_fov_deg=33.0, origin="centre", fov_test=test))
            self.assertTrue(self.view((0, 0), 0.0, c2, h, half_fov_deg=43.5, origin="centre", fov_test=test))

    def test_footprint_versus_nearest_point(self):
        # a long bar to the front-left: its nearest point (0.2, 0.6) is at 71.6 deg, its far corner (2.2, 0.6) at 15.3
        c, h = [1.2, 0.8, 1.5], [1.0, 0.2, 1.5]
        self.assertTrue(self.view((0, 0), 0.0, c, h, origin="centre", fov_test="footprint"))
        self.assertFalse(self.view((0, 0), 0.0, c, h, origin="centre", fov_test="nearest_point"))

    def test_eye_versus_centre(self):
        # nearest point at (0.3, 0.2): 33.7 deg from the centre, 45 deg from the eye 0.1 m ahead
        c, h = [0.3, 0.2 + 0.05, 1.5], [0.001, 0.05, 1.5]
        self.assertTrue(self.view((0, 0), 0.0, c, h, half_fov_deg=40.0, origin="centre", fov_test="nearest_point"))
        self.assertFalse(self.view((0, 0), 0.0, c, h, half_fov_deg=40.0, origin="eye", fov_test="nearest_point"))
        with self.assertRaises(ValueError):
            self.view((0, 0), 0.0, c, h, origin="nose")
        with self.assertRaises(ValueError):
            self.view((0, 0), 0.0, c, h, fov_test="cone")

    def test_vertical_fov_and_range(self):
        high = [2.0, 0.0, 5.5]                                                   # z 5-6 m, 1 m below the camera
        hh = [0.2, 0.2, 0.5]
        self.assertFalse(self.view((0, 0), 0.0, high, hh))                       # 67+ deg above the 27 deg half
        self.assertTrue(self.view((0, 0), 0.0, high, hh, vhalf_deg=None))
        self.assertTrue(self.view((0, 0), 0.0, [9.5, 0, 1.5], [0.2, 0.2, 1.5]))
        self.assertFalse(self.view((0, 0), 0.0, [12.0, 0, 1.5], [0.2, 0.2, 1.5]))   # beyond 10 m
        self.assertTrue(self.view((0, 0), 0.0, [12.0, 0, 1.5], [0.2, 0.2, 1.5], max_range_m=math.inf))

    def test_inside_footprint_counts_in_view(self):
        self.assertTrue(self.view((0, 0), 0.0, [0.1, 0.0, 1.5], [0.3, 0.3, 1.5]))


class TestCrashEpisodes(unittest.TestCase):
    """Synthetic traces with known categories, through the production trace writer and reader."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def build(self):
        n = 25
        eps, lays, rows = [], {}, []
        # env 0: flies straight at a bar ahead -> IN_VIEW, contact bearing 0
        xy = np.stack([np.linspace(0, 2.4, n), np.zeros(n)], 1)
        eps.append(episode(0, 0, xy, np.zeros(n), vel=np.tile([1.0, 0.0], (n, 1))))
        lays[(0, 0)] = bar(2.9, 0.0)
        # env 1: heading +x, drifts left (+y) into a bar beside it -> BLIND, LEFT side
        xy = np.stack([np.zeros(n), np.linspace(0, 2.4, n)], 1)
        eps.append(episode(1, 0, xy, np.zeros(n), vel=np.tile([0.0, 1.0], (n, 1))))
        lays[(1, 0)] = bar(0.0, 2.9)
        # env 2: the bar ahead for the first 20 frames, then a yaw of 90 deg to the right; contact beside -> LEFT_FOV
        xy = np.tile([0.0, 0.0], (n, 1)).astype(float)
        xy[:, 0] = np.linspace(0, 2.4, n)
        yaw = np.where(np.arange(n) < 20, 0.0, -math.pi / 2)
        eps.append(episode(2, 0, xy, yaw, vel=np.tile([1.0, 0.0], (n, 1))))
        lays[(2, 0)] = bar(2.9, 0.0)
        # env 3: 5 frames only (contact at 0.5 s) -> EARLY, IN_VIEW
        xy = np.stack([np.linspace(0, 0.4, 5), np.zeros(5)], 1)
        eps.append(episode(3, 0, xy, np.zeros(5), vel=np.tile([1.0, 0.0], (5, 1))))
        lays[(3, 0)] = bar(0.9, 0.0)
        # env 4: two bars equally near ahead-left and ahead-right -> ambiguous
        xy = np.stack([np.linspace(0, 2.4, n), np.zeros(n)], 1)
        eps.append(episode(4, 0, xy, np.zeros(n), vel=np.tile([1.0, 0.0], (n, 1))))
        lays[(4, 0)] = (np.array([[2.9, 0.4, 1.5], [2.9, -0.4, 1.5]]), np.array([[0.25, 0.15, 1.5]] * 2))
        # env 5: a capture (not a crash) and env 6: a crash outside the quota
        eps.append(episode(5, 0, np.zeros((n, 2)), np.zeros(n)))
        lays[(5, 0)] = bar(5.0, 5.0)
        eps.append(episode(6, 0, np.stack([np.zeros(n), np.linspace(0, 2.4, n)], 1), np.zeros(n)))
        lays[(6, 0)] = bar(0.0, 2.9)
        for env, outcome, quota in ((0, "crash", True), (1, "crash", True), (2, "crash", True), (3, "crash", True),
                                    (4, "crash", True), (5, "capture", True), (6, "crash", False)):
            rows.append({"env_index": env, "env_episode_index": 0, "outcome": outcome, "in_quota": quota})
        cell = self.d / "cell"
        cell.mkdir()
        write_trace(cell, np.concatenate(eps), {k: (np.asarray(c, float), np.asarray(h, float))
                                               for k, (c, h) in lays.items()})
        (cell / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
        (cell / "kroc2027_episodes.jsonl").write_text("".join(
            json.dumps({"env_index": i, "env_episode_index": 0, "actions": 5 if i == 3 else 25}) + "\n"
            for i in range(7)))
        return cell

    def test_known_categories(self):
        cell = self.build()
        eps, cov = X.cell_episodes(cell, PRIMARY, OT)
        by = {e["env"]: e for e in eps}
        self.assertEqual(sorted(by), [0, 1, 2, 3, 4])                    # quota crashes only
        self.assertEqual(cov["crash_rows_without_trace"], 0)
        self.assertEqual(by[0]["category"], "IN_VIEW")
        self.assertFalse(by[0]["EARLY"])
        self.assertAlmostEqual(by[0]["contact_bearing_deg"], 0.0, places=4)
        self.assertEqual(by[1]["category"], "BLIND")
        self.assertTrue(by[1]["left_side"])
        self.assertGreater(by[1]["contact_bearing_deg"], 80)
        self.assertTrue(by[1]["vel_out_of_fov_at_contact"] and by[1]["vel_out_of_fov_1s"])
        self.assertFalse(by[1]["seen_2s"])
        self.assertEqual(by[2]["category"], "LEFT_FOV")
        self.assertTrue(by[2]["seen_1s"] and not by[2]["in_fov_at_contact"])
        self.assertTrue(by[2]["left_side"])                                 # the heading turned right
        self.assertEqual(by[3]["category"], "IN_VIEW")
        self.assertTrue(by[3]["EARLY"])
        self.assertAlmostEqual(by[3]["t_contact_s"], 0.5)
        self.assertTrue(by[3]["actions_match"] and by[0]["actions_match"])
        self.assertTrue(by[4]["ambiguous"])
        self.assertFalse(by[0]["ambiguous"])
        for e in eps:
            self.assertTrue(e["valid"] and e["frames_contiguous"] and e["extrapolated_bar_agrees"], e["env"])
            self.assertEqual(sum(e[c] for c in X.CATEGORIES), 1)
        self.assertAlmostEqual(by[0]["dist_m"], 0.25, places=4)
        everything, _ = X.cell_episodes(cell, dict(PRIMARY, scope="all"), OT)
        self.assertEqual(sorted(e["env"] for e in everything), [0, 1, 2, 3, 4, 6])

    def test_half_fov_moves_a_category(self):
        # a bar at 40 deg for the whole episode, contact distance kept small: IN_VIEW at 43.5, BLIND at 33
        n = 25
        cell = self.d / "c40"
        cell.mkdir()
        b = np.radians(40)
        xy = np.stack([np.linspace(0, 1.0, n) * math.cos(b), np.linspace(0, 1.0, n) * math.sin(b)], 1)
        c = np.array([[1.3 * math.cos(b), 1.3 * math.sin(b), 1.5]])
        write_trace(cell, episode(0, 0, xy, np.zeros(n)), {(0, 0): (c, np.array([[0.02, 0.02, 1.5]]))})
        (cell / "episodes.jsonl").write_text(json.dumps({"env_index": 0, "env_episode_index": 0, "outcome": "crash",
                                                         "in_quota": True}) + "\n")
        prof = dict(G3P, scope="quota")
        self.assertEqual(X.cell_episodes(cell, prof, OT)[0][0]["category"], "IN_VIEW")
        self.assertEqual(X.cell_episodes(cell, dict(prof, half_fov_deg=33.0), OT)[0][0]["category"], "BLIND")
        centre = dict(PRIMARY, origin="centre")
        self.assertEqual(X.cell_episodes(cell, dict(centre, half_fov_deg=43.5), OT)[0][0]["category"], "IN_VIEW")
        self.assertEqual(X.cell_episodes(cell, centre, OT)[0][0]["category"], "BLIND")
        # from the eye 0.1 m ahead the near bar's bearing grows past 43.5 deg in the last frames
        self.assertEqual(X.cell_episodes(cell, dict(PRIMARY, half_fov_deg=43.5), OT)[0][0]["category"], "LEFT_FOV")

    def test_rotated_obstacles_refused(self):
        cell = self.d / "rot"
        cell.mkdir()
        w = OT.TraceWriter(cell, {"dt_s": 0.1, "camera": {"translation_m": [0.1, 0, 0.03]}})
        w.add_reset(0, 0, "s", np.array([[1.0, 0, 1.5]]), np.array([[0.2, 0.2, 1.5]]),
                    np.array([[0, 0, math.sin(0.3), math.cos(0.3)]]))
        w.write_frames(episode(0, 0, np.zeros((3, 2)), np.zeros(3)))
        w.close("complete")
        (cell / "episodes.jsonl").write_text("")
        with self.assertRaises(ValueError):
            X.cell_episodes(cell, PRIMARY, OT)


class TestStatistics(unittest.TestCase):
    def test_cluster_ci(self):
        rng = np.random.default_rng(1)
        vals = [1.0, 0.0] * 10
        clusters = list(range(20))
        self.assertEqual(X.cluster_ci([1.0] * 6, [0, 0, 1, 1, 2, 2], [0] * 6, rng), [1.0, 1.0])
        lo, hi = X.cluster_ci(vals, clusters, [0] * 20, rng)
        self.assertLess(lo, 0.5)
        self.assertGreater(hi, 0.5)
        # identical clusters (each 50 %) give a zero-width interval; the episode bootstrap of the same data does not
        cl = [i // 2 for i in range(20)]
        self.assertEqual(X.cluster_ci(vals, cl, [0] * 20, rng), [0.5, 0.5])
        # strata are kept: two strata with fixed shares never mix
        self.assertEqual(X.cluster_ci([1, 1, 0, 0], ["a", "b", "c", "d"], [0, 0, 1, 1], rng), [0.5, 0.5])
        self.assertIsNone(X.cluster_ci([None, None], [0, 1], [0, 0], rng))

    def test_guidance(self):
        def pooled(b, l, i, ci=0.02, early=0.05):
            return {"BLIND": {"mean": b, "ci95_cell_within_density": [b - ci, b + ci]},
                    "LEFT_FOV": {"mean": l, "ci95_cell_within_density": [l - ci, l + ci]},
                    "IN_VIEW": {"mean": i, "ci95_cell_within_density": [i - ci, i + ci]},
                    "EARLY": {"mean": early}}
        self.assertEqual(X.guidance(pooled(0.6, 0.15, 0.25), pooled(0.7, 0.1, 0.2))["dominant"], "BLIND")
        self.assertEqual(X.guidance(pooled(0.2, 0.15, 0.65), pooled(0.2, 0.1, 0.7))["dominant"], "IN_VIEW")
        self.assertEqual(X.guidance(pooled(0.35, 0.3, 0.35), pooled(0.4, 0.3, 0.3))["dominant"], "MIXED")
        self.assertEqual(X.guidance(pooled(0.2, 0.15, 0.65), pooled(0.6, 0.1, 0.3))["dominant"],
                         "DEFINITION_SENSITIVE")
        self.assertIn("early_note", X.guidance(pooled(0.6, 0.15, 0.25, early=0.3), pooled(0.7, 0.1, 0.2)))


class TestRealData(unittest.TestCase):
    @unittest.skipUnless((G3_RUN / "no_lidar/E2_obstacle_aware__70bars__seed4501").is_dir(), "G3 traces absent")
    def test_g3_positive_control(self):
        with tempfile.TemporaryDirectory() as tmp:
            res = X.g3_control(G3_RUN, pathlib.Path(tmp) / "pc.json")
            self.assertEqual(res["reading"], "POSITIVE_CONTROL_PASS")
            got = {k: round(v["at_half_fov"]["out_last_pct"], 2) for k, v in res["conditions"].items()}
            self.assertEqual(got, {"A": 48.84, "B_out": 72.07, "C": 72.85})
            self.assertEqual(round(res["conditions"]["C"]["at_half_fov"]["never_in_1s_pct"], 2), 56.74)
            self.assertEqual(round(res["conditions"]["A"]["at_half_fov"]["never_in_1s_pct"], 2), 33.68)
            neg = X.g3_control(G3_RUN, pathlib.Path(tmp) / "neg.json", half_fov_deg=33.0)
            self.assertEqual(neg["reading"], "POSITIVE_CONTROL_FAIL")                # the parameter is live

    @unittest.skipUnless(K2E.is_dir(), "CAM-K2-E raw absent")
    def test_stage_b_gate_controls(self):
        out, manifest = X.gate_controls(K2E, X.stage_b_blocks())
        self.assertTrue(manifest)
        self.assertTrue(all(c["ok"] for c in out.values()))
        self.assertEqual({k: v["label"] for k, v in out.items()},
                         {"K1s1_minus_K2s1": "WORSE", "K2s1_minus_K1s1": "SMALL_REDUCTION",
                          "K1s2_minus_K2s2": "WORSE", "K2s2_minus_K1s2": "SMALL_REDUCTION",
                          "K2s1_minus_K2s2_null": "NEGLIGIBLE"})
        self.assertAlmostEqual(out["K1s1_minus_K2s1"]["crash"]["mean_pp"], 3.54, places=2)
        self.assertAlmostEqual(out["K1s2_minus_K2s2"]["crash"]["mean_pp"], 4.93, places=2)

    @unittest.skipUnless(E3_CELL.is_dir(), "CAM-K2-E3 raw absent")
    def test_cell_rates_real(self):
        r = X.cell_rates(E3_CELL)
        self.assertEqual(r["n"], 2048)
        self.assertAlmostEqual(r["crash"] + r["capture"] + r["timeout"], 1.0)
        self.assertEqual(r["median_capture_s"], 5.0)
        self.assertLessEqual(r["commanded_max_mps"], float(R.NATIVE_FREE) + R.CAP_TOLERANCE_MPS)
        self.assertGreater(r["share_commanded_over_2p5"], 0.5)                     # the caps bind


class TestRepairR1(unittest.TestCase):
    """Review r1 repairs: F1 consistency gate, F2 VALID-only shares, F7 bindings, F8 no labels on CONTROL_FAIL,
    F10 a written reading on unsupported geometry and a GPU idle check before every cell."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    @staticmethod
    def eps(n=100, ps=1, cat="BLIND", valid=True, actions_match=True, contiguous=True, density=70, cell_seed=4501):
        out = []
        for i in range(n):
            e = {"policy_seed": ps, "category": cat, "valid": valid, "actions_match": actions_match,
                 "frames_contiguous": contiguous, "density": density, "seed": cell_seed, "env": i, "EARLY": False,
                 "dist_m": 0.3 if valid else 2.0}
            e.update({c: c == cat for c in X.CATEGORIES})
            for k in X.CI_METRICS:
                e.setdefault(k, 0.0)
            e.update({k: 0.0 for k in ("contact_bearing_deg", "speed_mps", "roll_deg", "pitch_deg", "t_contact_s",
                                        "vel_bearing_1s_deg")})
            e.update(extrapolated_bar_agrees=True)
            out.append(e)
        return out

    @staticmethod
    def coverage(traced=(100, 100), missing=(0, 0)):
        return {"V3s%d/70/4501" % ps: {"policy_seed": ps, "crash_episodes": traced[ps - 1],
                                         "crash_rows_without_trace": missing[ps - 1]} for ps in (1, 2)}

    def test_f1_consistency_gate(self):
        good = self.eps(ps=1) + self.eps(ps=2)
        self.assertEqual(X.consistency_problems(good, self.coverage()), [])               # positive control
        self.assertEqual(X.consistency_problems(good, self.coverage(missing=(1, 0))), [])  # 100/101 = 0.990
        self.assertTrue(X.consistency_problems(good, self.coverage(missing=(2, 0))))       # 100/102 < 0.99
        bad_actions = self.eps(98, ps=1) + self.eps(2, ps=1, actions_match=False) + self.eps(ps=2)
        self.assertTrue(X.consistency_problems(bad_actions, self.coverage()))
        ok_actions = self.eps(99, ps=1) + self.eps(1, ps=1, actions_match=False) + self.eps(ps=2)
        self.assertEqual(X.consistency_problems(ok_actions, self.coverage()), [])
        unknown = self.eps(99, ps=1) + self.eps(1, ps=1, actions_match=None) + self.eps(ps=2)
        self.assertTrue(X.consistency_problems(unknown, self.coverage()))
        gaps = self.eps(98, ps=1) + self.eps(2, ps=1, contiguous=False) + self.eps(ps=2)
        self.assertTrue(X.consistency_problems(gaps, self.coverage()))
        invalid = self.eps(89, ps=2) + self.eps(11, ps=2, valid=False) + self.eps(ps=1)
        self.assertTrue(X.consistency_problems(invalid, self.coverage()))
        self.assertTrue(X.consistency_problems(self.eps(ps=1), self.coverage()))           # seed 2 has no crash
        self.assertTrue(any(r.startswith("V3s2") for r in X.consistency_problems(gaps[:0] + self.eps(ps=1) +
                        self.eps(98, ps=2) + self.eps(2, ps=2, contiguous=False), self.coverage())))

    def test_f2_shares_over_valid_only(self):
        rng = np.random.default_rng(1)
        eps = (self.eps(60, ps=1, cat="IN_VIEW") + self.eps(40, ps=1, cat="BLIND", valid=False)
               + self.eps(50, ps=2, cat="IN_VIEW", density=115))
        r = X.split_summaries(eps, rng)
        self.assertEqual(r["pooled"]["crash_episodes"], 110)
        self.assertEqual(r["pooled"]["IN_VIEW"]["mean"], 1.0)                 # the invalid BLIND are not in it
        self.assertEqual(r["pooled"]["BLIND"]["mean"], 0.0)
        self.assertEqual(r["invalid_not_in_shares"]["V3s1"]["count"], 40)
        self.assertEqual(r["invalid_not_in_shares"]["V3s1"]["categories"]["BLIND"], 40)
        self.assertEqual(r["invalid_not_in_shares"]["V3s2"]["count"], 0)
        self.assertAlmostEqual(r["sensitivity_all_crashes"]["pooled"]["BLIND"], 40 / 150)
        self.assertEqual(r["per_density_invalid"]["V3s1/70"], 40)
        self.assertEqual(r["per_density_counts_valid"]["V3s2/115"]["IN_VIEW"], 50)

    def test_f7_bindings(self):
        trace = self.d / "raw" / "trace_01"
        trace.mkdir(parents=True)
        pc = self.d / "pc.json"
        sha = X.E1.sha(X.__file__)
        good_pc = {"reading": "POSITIVE_CONTROL_PASS", "half_fov_deg": 43.5, "script_sha256": sha}
        pc.write_text(json.dumps(good_pc))
        rep = {"commit": "c" * 40}
        self.assertFalse(X.stage_a_binding_extra(trace, rep, pc)["identity_pass_same_commit"])   # no identity
        (self.d / "raw" / "identity_01").mkdir()
        idp = self.d / "raw" / "identity_01" / R.REPORT["identity"]
        idp.write_text(json.dumps({"reading": "IDENTITY_PASS", "commit": "c" * 40}))
        self.assertEqual(X.stage_a_binding_extra(trace, rep, pc), {"identity_pass_same_commit": True,
                                                                   "g3_positive_control_pass": True})
        idp.write_text(json.dumps({"reading": "NOT_REPRODUCED", "commit": "c" * 40}))
        self.assertFalse(X.stage_a_binding_extra(trace, rep, pc)["identity_pass_same_commit"])
        idp.write_text(json.dumps({"reading": "IDENTITY_PASS", "commit": "d" * 40}))
        self.assertFalse(X.stage_a_binding_extra(trace, rep, pc)["identity_pass_same_commit"])
        idp.write_text(json.dumps({"reading": "IDENTITY_PASS", "commit": "c" * 40}))
        for bad in ({"reading": "POSITIVE_CONTROL_FAIL"}, {"half_fov_deg": 33.0}, {"script_sha256": "0" * 64}):
            pc.write_text(json.dumps(dict(good_pc, **bad)))
            self.assertFalse(X.stage_a_binding_extra(trace, rep, pc)["g3_positive_control_pass"], bad)
        self.assertFalse(X.stage_a_binding_extra(trace, rep, None)["g3_positive_control_pass"])
        self.assertFalse(X.stage_a_binding_extra(trace, rep, self.d / "missing.json")["g3_positive_control_pass"])

    def test_f10_unsupported_geometry_writes_a_reading(self):
        trace = self.d / "trace_01"
        arm, d, s = R.trace_plan()[0]
        cell = trace / arm / (R.CELL % (d, s))
        cell.mkdir(parents=True)
        w = OT.TraceWriter(cell, {"dt_s": 0.1, "camera": {"translation_m": [0.1, 0, 0.03]}})
        w.add_reset(0, 0, "s", np.array([[1.0, 0, 1.5]]), np.array([[0.2, 0.2, 1.5]]),
                    np.array([[0, 0, math.sin(0.3), math.cos(0.3)]]))
        w.write_frames(episode(0, 0, np.zeros((3, 2)), np.zeros(3)))
        w.close("complete")
        (cell / "episodes.jsonl").write_text("")
        saved = (X.trace_binding, X.stage_a_binding_extra)
        try:
            X.trace_binding = lambda *a, **k: ({"stub": True}, {"commit": "c", "reading": "NONINTERFERENCE_PASS"})
            X.stage_a_binding_extra = lambda *a, **k: {"stub2": True}
            with contextlib.redirect_stdout(io.StringIO()):
                res = X.stage_a(trace, self.d / "a.json", E3)
        finally:
            X.trace_binding, X.stage_a_binding_extra = saved
        self.assertEqual(res["reading"], "MEASUREMENT_INVALID")
        self.assertIn("non-identity obstacle orientation", res["reasons"][0])
        self.assertEqual(json.loads((self.d / "a.json").read_text())["reading"], "MEASUREMENT_INVALID")

    def test_n6_missing_dt_writes_a_reading(self):
        """Review r2 N6: trace metadata without dt_s (a TypeError) also writes MEASUREMENT_INVALID."""
        trace = self.d / "trace_n6"
        arm, d, s = R.trace_plan()[0]
        cell = trace / arm / (R.CELL % (d, s))
        cell.mkdir(parents=True)
        w = OT.TraceWriter(cell, {"camera": {"translation_m": [0.1, 0, 0.03]}})     # no dt_s
        w.add_reset(0, 0, "s", np.array([[1.0, 0, 1.5]]), np.array([[0.2, 0.2, 1.5]]))
        w.write_frames(episode(0, 0, np.zeros((3, 2)), np.zeros(3)))
        w.close("complete")
        (cell / "episodes.jsonl").write_text("")
        with self.assertRaises(TypeError):                                   # the raw failure the catch covers
            X.cell_episodes(cell, PRIMARY, OT)
        saved = (X.trace_binding, X.stage_a_binding_extra)
        try:
            X.trace_binding = lambda *a, **k: ({"stub": True}, {"commit": "c", "reading": "NONINTERFERENCE_PASS"})
            X.stage_a_binding_extra = lambda *a, **k: {"stub2": True}
            with contextlib.redirect_stdout(io.StringIO()):
                res = X.stage_a(trace, self.d / "n6.json", E3)
        finally:
            X.trace_binding, X.stage_a_binding_extra = saved
        self.assertEqual(res["reading"], "MEASUREMENT_INVALID")
        self.assertTrue((self.d / "n6.json").is_file())

    @unittest.skipUnless(E3_CELL.is_dir() and K2E.is_dir(), "CAM-K2-E3 / CAM-K2-E raw absent")
    def test_f8_no_labels_on_control_fail(self):
        root = self.d / "raw"
        for name, plan, report in (("identity_01", R.identity_plan(), {"reading": "IDENTITY_PASS"}),
                                   ("control_01", R.control_plan(), {"reading": "CONTROL_PASS"}),
                                   ("envelope_01", R.envelope_plan(), {"reading": "COMPLETE", "cells_expected": 36})):
            cells = []
            for arm, den, seed in plan:
                c = root / name / arm / (R.CELL % (den, seed))
                c.mkdir(parents=True)
                ref = R.reference_dir(E3, arm, den, seed)
                for f in ("episodes.jsonl", "kroc2027_episodes.jsonl", "result.json", "receipt.json"):
                    (c / f).symlink_to(ref / f)
                cells.append({"arm": arm, "density": den, "seed": seed})
            stage = name.split("_")[0]
            (root / name / R.REPORT[stage]).write_text(json.dumps(dict(report, commit="c", cells=cells)))
        saved = (R.R2.receipt_problems, R.V3.validity_problems, R.camt1_problems, X.E1.file_matches_commit,
                 X.gate_controls)
        real_gate = X.gate_controls
        try:
            R.R2.receipt_problems = R.V3.validity_problems = R.camt1_problems = lambda *a, **k: []
            X.E1.file_matches_commit = lambda *a, **k: True
            with contextlib.redirect_stdout(io.StringIO()):
                ok = X.stage_b(root / "control_01", root / "envelope_01", self.d / "ok.json", E3, K2E)

                def failing(*a, **k):
                    out, man = real_gate(*a, **k)
                    out["K1s1_minus_K2s1"]["ok"] = False
                    return out, man
                X.gate_controls = failing
                bad = X.stage_b(root / "control_01", root / "envelope_01", self.d / "bad.json", E3, K2E)
        finally:
            (R.R2.receipt_problems, R.V3.validity_problems, R.camt1_problems, X.E1.file_matches_commit,
             X.gate_controls) = saved
        self.assertEqual(ok["reading"], "REPORTED")                                     # positive control
        self.assertEqual(ok["labels"]["d1"]["reading"], "NEGLIGIBLE")                   # every arm equals native
        self.assertEqual(bad["reading"], "CONTROL_FAIL")
        text = (self.d / "bad.json").read_text()
        self.assertIsNone(json.loads(text)["labels"])
        self.assertNotIn("labels_before_controls", text)
        self.assertNotIn("NEGLIGIBLE\"", text.split('"controls"')[0])
        (root / "identity_01" / R.REPORT["identity"]).write_text(json.dumps({"reading": "NOT_REPRODUCED",
                                                                              "commit": "c"}))
        saved = (R.R2.receipt_problems, R.V3.validity_problems, R.camt1_problems, X.E1.file_matches_commit)
        try:
            R.R2.receipt_problems = R.V3.validity_problems = R.camt1_problems = lambda *a, **k: []
            X.E1.file_matches_commit = lambda *a, **k: True
            nr = X.stage_b(root / "control_01", root / "envelope_01", self.d / "nr.json", E3, K2E)
        finally:
            R.R2.receipt_problems, R.V3.validity_problems, R.camt1_problems, X.E1.file_matches_commit = saved
        self.assertEqual(nr["reading"], "EVAL_BINDING_FAIL")                            # identity bound

    @unittest.skipUnless(E3_CELL.is_dir(), "CAM-K2-E3 raw absent")
    def test_gpu_idle_check_before_every_cell(self):
        calls = []

        def fake_run_cell(root, protocol, out_root, level, density, seed, quota, run_id, scale, head, start,
                          records, timeout_s, extra=None, launcher=None, ckpt_sha=None):
            cid = R.CELL % (density, seed)
            d = out_root / level / cid
            d.mkdir(parents=True)
            ref = R.reference_dir(E3, level, density, seed)
            for f in ("episodes.jsonl", "kroc2027_episodes.jsonl", "result.json", "receipt.json"):
                (d / f).symlink_to(ref / f)
            rec = {"cell_id": cid, "verdict": "OK", "problems": [], "wall_s": 1.0}
            records.append(rec)
            calls.append(level)
            return rec
        saved = (R.G.run_cell, R.G.gpu_apps, R.R2.receipt_problems, R.V3.validity_problems, R.camt1_problems)
        protocol = json.loads(R.K.PROTOCOL.read_text())
        host = {"fault_count": 0, "boot_id": "b", "online": "x"}
        try:
            R.G.run_cell = fake_run_cell
            R.R2.receipt_problems = R.V3.validity_problems = R.camt1_problems = lambda *a, **k: []
            saved_host = R.G.host
            R.G.host = lambda: dict(host)
            try:
                with contextlib.redirect_stdout(io.StringIO()):
                    R.G.gpu_apps = lambda: []
                    rep = {}
                    (self.d / "ok").mkdir()
                    self.assertEqual(R.run_stage("identity", protocol, self.d / "ok", "H", dict(host), CKPTS, {},
                                                 "v", E3, rep), "IDENTITY_PASS")
                    R.G.gpu_apps = lambda: ["123, python"]
                    rep = {}
                    (self.d / "busy").mkdir()
                    self.assertEqual(R.run_stage("identity", protocol, self.d / "busy", "H", dict(host), CKPTS, {},
                                                 "v", E3, rep), "INCOMPLETE")
                    self.assertEqual(rep["gpu_busy_stop"]["before_cell"], 0)

                    def hang():
                        raise subprocess.TimeoutExpired("nvidia-smi", 30)
                    for err in (hang, lambda: (_ for _ in ()).throw(OSError("no nvidia-smi"))):
                        R.G.gpu_apps = err                           # review r2 N2: a failing query stops the stage
                        rep = {}
                        d = self.d / ("err%d" % len(list(self.d.glob("err*"))))
                        d.mkdir()
                        self.assertEqual(R.run_stage("identity", protocol, d, "H", dict(host), CKPTS, {}, "v", E3,
                                                     rep), "INCOMPLETE")
                        self.assertEqual(rep["gpu_check_error"]["before_cell"], 0)
            finally:
                R.G.host = saved_host
        finally:
            R.G.run_cell, R.G.gpu_apps, R.R2.receipt_problems, R.V3.validity_problems, R.camt1_problems = saved
        self.assertEqual(calls, ["native_identity"])                                # the busy stage ran no cell


class TestCli(unittest.TestCase):
    def test_refuses_overwrite(self):
        with tempfile.NamedTemporaryFile() as f:
            for argv in (["controls", f.name], ["g3-control", "x", f.name],
                         ["stage-a", "x", f.name, "--g3-control", "y"],
                         ["stage-b", "x", "y", f.name]):
                with self.assertRaises(SystemExit):
                    X.main(argv)

    def test_runner_refuses_bad_stage(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            R.main(["eval", "--approved-commit", "abc1234", "--out", "/nonexistent/x"])


if __name__ == "__main__":
    unittest.main()
