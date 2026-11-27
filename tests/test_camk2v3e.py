"""CPU tests of CAM-K2-E3 (docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md): tools/camk2v3e_run.py and
tools/camk2v3e_analyse.py. No GPU, no simulator. Tests that need the real CAM-K2-E / CAM-K1-E raw folders or the
CAM-K2 checkpoints skip when those are absent (another machine)."""
import importlib.util
import json
import pathlib
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
_saved_path = list(sys.path)
try:
    sys.path.insert(0, str(TOOLS))
    import camk2v3e_run as V                                # noqa: E402
    import camk2v3e_analyse as X                            # noqa: E402
finally:
    sys.path[:] = _saved_path
_PATH_RESTORED = sys.path == _saved_path

HOME = pathlib.Path.home()
K2E = HOME / "motar_tmp/camk2e_20261108/raw"
K1E = HOME / "motar_tmp/camk1e_20261108/raw"
K2E_CELL = K2E / "eval_01/K2s1/E2_obstacle_aware__205bars__seed4501"
VSHA = "f" * 64


def telemetry(near=10, invalid=100, pixels=1000, mismatch=0, calls=None, settings=None, drop=()):
    pre = V.T3.DV_MOD.LOG_PREFIX
    lines = {"install": {"event": "install", "settings": settings or V.T3.DEPTH_VALIDITY},
             "first": {"event": "first", "calls": 1, "near_pixels": 0, "invalid_pixels": invalid // 10,
                       "pixels": pixels // 50, "sentinel_mismatch": 0},
             "summary": {"event": "summary", "calls": calls or V.T3.DV_MOD.STATS_CALLS, "near_pixels": near,
                         "invalid_pixels": invalid, "pixels": pixels, "sentinel_mismatch": mismatch}}
    return "".join(pre + json.dumps(v, sort_keys=True) + "\n" for k, v in lines.items() if k not in drop)


def make_cell(d, on=True, settings=None, sha=VSHA, env=None, log=None):
    d.mkdir(parents=True, exist_ok=True)
    rec = {"settings": {"depth_validity": settings or V.T3.DEPTH_VALIDITY} if on else {}}
    if on:
        rec["depth_validity"] = {"settings": settings or V.T3.DEPTH_VALIDITY}
        rec["depth_validity_sha256"] = sha
    snap = dict(V.VALIDITY_ENV if env is None else env) if on else {}
    (d / "realveh_receipt.json").write_text(json.dumps(rec))
    (d / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": snap}))
    (d / "cell.log").write_text("hfov=66deg\n" + (telemetry() if (on and log is None) else (log or "")))
    return d


class TestDesign(unittest.TestCase):
    def test_constants(self):
        self.assertEqual(V.PREREG, "docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md")
        self.assertIn(V.PREREG, V.COMMITTED_FILES)
        self.assertNotIn(V.R2.PREREG, V.COMMITTED_FILES)
        self.assertIn("tools/camk2v3e_analyse.py", V.COMMITTED_FILES)
        self.assertIn(V.T3.VALIDITY_REL, V.COMMITTED_FILES)
        self.assertEqual((V.MAX_HOURS, V.MIN_FREE_GB, V.RATE_CELLS, V.RATE_MARGIN), (3.5, 1.0, 48, 1.10))
        self.assertEqual(V.EVAL_ARMS, ("V3s1", "V3s2", "V2Rs1", "V2Rs2"))
        self.assertEqual(V.V3_SHA[1][:8] + V.V3_SHA[2][:8], "888e8056203deea4")
        self.assertEqual(V.VALIDITY_ENV, {"NAVRL_DEPTH_VALIDITY": "blob_near", "NAVRL_DEPTH_VALIDITY_WINDOW": "5",
                                          "NAVRL_DEPTH_VALIDITY_DENSITY": "0.5"})
        self.assertTrue(_PATH_RESTORED)

    def test_plan(self):
        plan = V.eval_plan()
        self.assertEqual(len(plan), 48)
        self.assertEqual(plan[:4], [("V3s1", 70, 4501), ("V3s2", 70, 4501), ("V2Rs1", 70, 4501), ("V2Rs2", 70, 4501)])
        self.assertEqual(sorted({(d, s) for _, d, s in plan}),
                         [(d, s) for d in V.G.DENSITIES for s in V.G.SEEDS])
        self.assertEqual([a for a, _, _ in V.IDENTITY_PLAN],
                         ["V2s1_identity", "V2s2_identity", "V2Rs1_negative", "V2Rs1_repeat", "V3s1_smoke"])
        self.assertFalse(V.rule("V2s1_identity"))
        self.assertFalse(V.rule("V2s2_identity"))
        self.assertEqual(V.IDENTITY_REFERENCES, {"V2s2_identity": "eval_01/K2s2/E2_obstacle_aware__205bars__seed4501"})
        for arm in ("V2Rs1_negative", "V2Rs1_repeat", "V3s1_smoke") + V.EVAL_ARMS:
            self.assertTrue(V.rule(arm), arm)

    def test_arm_checkpoints(self):
        ck = {"K2s1": ("a", "1"), "K2s2": ("b", "2"), "V3s1": ("c", "3"), "V3s2": ("d", "4")}
        want = {"V2s1_identity": "1", "V2s2_identity": "2", "V2Rs1_negative": "1", "V2Rs1_repeat": "1", "V3s1_smoke": "3",
                "V3s1": "3", "V3s2": "4", "V2Rs1": "1", "V2Rs2": "2"}
        for arm, sha in want.items():
            self.assertEqual(V.arm_ckpt(arm, ck)[1], sha, arm)

    def test_snapshot_allowance(self):
        self.assertTrue(set(V.VALIDITY_KEYS) <= X.SNAPSHOT_EXPECTED)
        self.assertNotIn("NAVRL_DEPTH_SEED", X.SNAPSHOT_EXPECTED)          # treatment keys must match exactly
        self.assertNotIn("NAVRL_VEH_RAND_SEED", X.SNAPSHOT_EXPECTED)

    def test_gate_d_labels(self):
        self.assertEqual(X.gate_d([-8, -6], [-9, -5.5], -7), "MATERIAL_REDUCTION")
        self.assertEqual(X.gate_d([-4, -1], [-5, -0.5], -3), "SMALL_REDUCTION")
        self.assertEqual(X.gate_d([1, 4], [0.5, 4.5], 3), "WORSE")
        self.assertEqual(X.gate_d([-1, 1], [-1.5, 1.5], 0), "NEGLIGIBLE")
        self.assertEqual(X.gate_d([-4, -1], [-5, 0.5], -3), "UNRESOLVED")
        self.assertEqual(X.gate_d([-8, -6], [-9, -5.5], -7, timeout_mean=2.5), "CRASH_TO_TIMEOUT")
        self.assertEqual(X.gate_d([-3, 1], [-4, 1.5], -1.0), "NO_DETECTED_CHANGE")


class TestValidityChecks(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = pathlib.Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def test_good_on_cell_passes(self):
        self.assertEqual(V.validity_problems(make_cell(self.d / "ok"), True, VSHA), [])

    def test_near_zero_summary_is_allowed(self):
        c = make_cell(self.d / "z", log=telemetry(near=0))
        self.assertEqual(V.validity_problems(c, True, VSHA), [])

    def test_good_off_cell_passes(self):
        self.assertEqual(V.validity_problems(make_cell(self.d / "off", on=False), False, VSHA), [])

    def test_deviations_fail(self):
        other = dict(V.T3.DEPTH_VALIDITY, NAVRL_DEPTH_VALIDITY_WINDOW=7)
        cases = {
            "settings": make_cell(self.d / "s", settings=other),
            "sha": make_cell(self.d / "h", sha="0" * 64),
            "env": make_cell(self.d / "e", env=dict(V.VALIDITY_ENV, NAVRL_DEPTH_VALIDITY_DENSITY="0.6")),
            "no_env": make_cell(self.d / "ne", env={}),
            "no_summary": make_cell(self.d / "ns", log=telemetry(drop=("summary",))),
            "no_first": make_cell(self.d / "nf", log=telemetry(drop=("first",))),
            "no_install": make_cell(self.d / "ni", log=telemetry(drop=("install",))),
            "two_installs": make_cell(self.d / "ti", log=telemetry() + telemetry(drop=("first", "summary"))),
            "mismatch": make_cell(self.d / "m", log=telemetry(mismatch=2)),
            "near_gt_invalid": make_cell(self.d / "n", log=telemetry(near=200, invalid=100)),
            "short_summary": make_cell(self.d / "c", log=telemetry(calls=10)),
            "install_settings": make_cell(self.d / "is", log=telemetry(settings=other)),
        }
        for name, c in cases.items():
            self.assertTrue(V.validity_problems(c, True, VSHA), name)
        self.assertTrue(V.validity_problems(make_cell(self.d / "u"), True, None))           # uncommitted module
        self.assertTrue(V.validity_problems(make_cell(self.d / "on_as_off"), False, VSHA))   # rule present, OFF expected

    @unittest.skipUnless(K2E_CELL.is_dir(), "CAM-K2-E raw cells absent")
    def test_real_camk2e_cell(self):
        self.assertEqual(V.validity_problems(K2E_CELL, False, VSHA), [])                     # control: no rule
        self.assertTrue(V.validity_problems(K2E_CELL, True, VSHA))


class TestRateGate(unittest.TestCase):
    @unittest.skipUnless((K2E / "identity_01/identity_report.json").is_file(), "CAM-K2-E raw absent")
    def test_normal_passes_slow_refuses(self):
        def rep(w):
            return {"records": [{"wall_s": w} for _ in V.IDENTITY_PLAN]}
        self.assertEqual(V.rate_problems(rep(180.0), K2E), [])
        self.assertEqual(V.rate_problems(rep(230.0), K2E), [])
        self.assertTrue(V.rate_problems(rep(400.0), K2E))
        self.assertTrue(V.rate_problems({"records": []}, K2E))
        p = V.rate_projection(rep(177.2), K2E)
        self.assertLess(p["projected_s"], V.MAX_HOURS * 3600)


class TestRealData(unittest.TestCase):
    @unittest.skipUnless((HOME / "motar_tmp/camk2v3_20261108/train_s2/train_report.json").is_file(),
                         "CAM-K2 checkpoints absent")
    def test_checkpoints_pass(self):
        ck, problems = V.checkpoints()
        self.assertEqual(problems, [])
        self.assertEqual(sorted(ck), ["K2s1", "K2s2", "V3s1", "V3s2"])
        bad, problems = V.checkpoints(camk2v3_dir=pathlib.Path("/nonexistent"))
        self.assertTrue(problems)                                                            # control

    @unittest.skipUnless(K2E.is_dir() and K1E.is_dir(), "CAM-K2-E / CAM-K1-E raw absent")
    def test_controls_and_reference_binding(self):
        blocks = [(d, s) for d in V.G.DENSITIES for s in V.G.SEEDS]
        nominal = [(d, s) for d in V.R2.NOMINAL_DENSITIES for s in V.G.SEEDS]
        self.assertTrue(X.camk2e_manifest_ok(K2E, blocks))
        cells = X.load_reference(K2E, blocks)
        nc = X.null_control(cells, blocks)
        self.assertTrue(nc["ok"])
        self.assertEqual(nc["label"], "NEGLIGIBLE")
        ctrl = X.A2.controls(X.A2.load_camk1e(K1E, blocks), blocks, nominal)
        self.assertTrue(all(c["ok"] for c in ctrl.values()))
        self.assertEqual(ctrl["A_on_K1E_K1s1_minus_F66"]["label"], "MATERIAL_ADAPTATION")   # MATERIAL reachable
        self.assertEqual(ctrl["A_on_K1E_F66_minus_K1s1"]["label"], "WORSE")
        self.assertFalse(X.camk2e_manifest_ok(K1E, blocks))                                  # control: other root
        self.assertTrue(X.camk2e_manifest_ok(K2E, blocks, arms=("K1s1", "K1s2")))           # review r2 m2-1
        self.assertFalse(X.camk2e_manifest_ok(K2E, blocks, arms=("K9s9",)))                 # control: no such cells

    @unittest.skipUnless(K2E.is_dir(), "CAM-K2-E raw absent")
    def test_claim_scale_controls(self):
        blocks = [(d, s) for d in V.G.DENSITIES for s in V.G.SEEDS]
        cells = {**X.load_reference(K2E, blocks), **X.load_camk2e_k1(K2E, blocks)}
        out = X.claim_scale_controls(cells, blocks)
        self.assertEqual(len(out), 4)
        self.assertTrue(all(c["ok"] for c in out.values()))
        self.assertEqual(out["D_on_CAMK2E_K1s1_minus_K2s1"]["label"], "WORSE")
        self.assertEqual(out["D_on_CAMK2E_K2s2_minus_K1s2"]["label"], "SMALL_REDUCTION")
        self.assertAlmostEqual(out["D_on_CAMK2E_K1s1_minus_K2s1"]["crash"]["mean_pp"], 3.8004557, places=5)
        null = {**cells, "K2E_K1s1": cells["V2s1"], "K2E_K1s2": cells["V2s2"]}                # review r2 LOW: null fails
        self.assertFalse(any(c["ok"] for c in X.claim_scale_controls(null, blocks).values()))

    @unittest.skipUnless(K2E_CELL.is_dir(), "CAM-K2-E raw absent")
    def test_snapshot_binding_accepts_and_rejects(self):
        ident = K2E / "identity_01/K2s1_smoke/E2_obstacle_aware__205bars__seed4501"
        other = K2E / "eval_01/K2s2/E2_obstacle_aware__205bars__seed4501"
        self.assertTrue(X.snapshot_ok(ident, K2E_CELL))
        self.assertTrue(X.snapshot_ok(other, K2E_CELL))                                     # another checkpoint
        with tempfile.TemporaryDirectory() as tmp:
            for key, value in (("NAVRL_DEPTH_SEED", "1"), ("NAVRL_VEH_RAND_SEED", "1"), ("NAVRL_SENSOR_ABLATION", "full")):
                d = pathlib.Path(tmp) / key
                d.mkdir()
                r = json.loads((K2E_CELL / "receipt.json").read_text())
                r["env_snapshot_navrl"][key] = value
                (d / "receipt.json").write_text(json.dumps(r))
                self.assertFalse(X.snapshot_ok(d, K2E_CELL), key)
            d = pathlib.Path(tmp) / "rule"
            d.mkdir()
            r = json.loads((K2E_CELL / "receipt.json").read_text())
            r["env_snapshot_navrl"].update(V.VALIDITY_ENV)
            (d / "receipt.json").write_text(json.dumps(r))
            self.assertTrue(X.snapshot_ok(d, K2E_CELL))                                     # the rule's keys are allowed


class TestAnalysisCli(unittest.TestCase):
    def test_refuses_overwrite_and_bad_usage(self):
        with tempfile.NamedTemporaryFile() as f:
            with self.assertRaises(SystemExit):
                X.main(["eval", "a", "b", "c", "d", f.name])
        with self.assertRaises(SystemExit):
            X.main(["controls", "a", "b"])


if __name__ == "__main__":
    unittest.main()
