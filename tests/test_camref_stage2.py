"""CPU tests of CAM-REF stage 2 (docs/plans/prereg_camonly_ref_stage2_2026-11-26.md): the factor table of
tools/camref_eval_run.py (stage-1 cells unchanged by it), tools/camref_stage2_run.py and tools/camref_stage2_analyse.py.
Stage-1 equality is checked against the runner as committed at 358c363 (the stage-1 GPU commit), loaded from git under a
private name; skipped (not passed) when git cannot show it. Stage 2's constants are installed only inside
camref_stage2_run.installed(), which restores stage 1's (PUBLIC_CPU runs every test in one process)."""
import importlib.util
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
import camref_stage2_run as S                               # noqa: E402
import camref_stage2_analyse as B                           # noqa: E402

STAGE1_COMMIT = "358c363b4069334379770a217263cb8f83b89e7c"
STAGE1_RUNGS = ("F0", "F3", "F4", "F8", "K1PK", "K1T", "K1S", "S0", "S3", "S6")
STAGE1_ARMS = ("F0ID", "S0ID", "F3", "F4", "F8", "S3", "S6", "K1ID", "K1T", "K1S", "F8_smoke", "K1S_smoke", "S6_smoke")
SEEDS = (4501, 4502, 4503)


def _stage1_runner():
    done = subprocess.run(["git", "-C", str(ROOT), "show", "%s:tools/camref_eval_run.py" % STAGE1_COMMIT],
                          capture_output=True)
    if done.returncode:
        return None
    d = pathlib.Path(tempfile.mkdtemp(prefix="camref_stage1_"))
    path = d / "camref_eval_run_stage1.py"
    path.write_bytes(done.stdout)
    spec = importlib.util.spec_from_file_location("camref_eval_run_stage1_under_test", str(path))
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(TOOLS))                          # its sibling imports (gr_run, kroc2027_eval, ...)
    try:
        spec.loader.exec_module(module)
    finally:
        shutil.rmtree(str(d), ignore_errors=True)           # review r1 P3-2 (stage 2); CAM-S3-E r1 P3-5: __pycache__
    return module


OLD = _stage1_runner()


@unittest.skipIf(OLD is None, "stage-1 runner not readable from git")
class Stage1Unchanged(unittest.TestCase):
    def test_every_stage1_rung_delta_is_the_committed_one(self):
        n = 0
        for r in STAGE1_RUNGS:
            for s in SEEDS:
                self.assertEqual(X.rung_delta(r, s), OLD.rung_delta(r, s), (r, s))
                n += len(X.rung_delta(r, s))
        self.assertGreater(n, 100)

    def test_installs_telemetry_and_protocol_of_stage1_arms_unchanged(self):
        for a in STAGE1_ARMS:
            self.assertEqual(X.expected_installed(a), OLD.expected_installed(a), a)
            self.assertEqual(X.visible_start(a), OLD.visible_start(a), a)
            self.assertEqual(X.mismatch_allow(a), OLD.mismatch_allow(a), a)
        for r in STAGE1_RUNGS:
            self.assertEqual(X.RUNG_PROTOCOL[r], OLD.RUNG_PROTOCOL[r], r)
        self.assertEqual(X.expected_installed("F8"), ["vehicle_rand", "camref_rules", "camref_record", "yawclip",
                                                      "inview_start", "inview_diag"])

    def test_stage1_constants_and_scope_unchanged_outside_install(self):
        for k in S.PATCHED:
            self.assertEqual(getattr(X, k), getattr(OLD, k), k)
        self.assertEqual(X.scope_digest(), OLD.scope_digest())


class FactorTable(unittest.TestCase):
    def test_each_new_rung_is_f4_plus_exactly_its_factor(self):
        for s in SEEDS:
            f4 = X.rung_delta("F4", s)
            union = {}
            for arm, (fac,) in zip(S.NEW_ARMS, [("delay",), ("rand",), ("yaw",), ("free",), ("geom",)]):
                d = X.rung_delta(arm, s)
                extra = {k: v for k, v in d.items() if f4.get(k) != v}
                self.assertEqual(set(f4) - set(d), set(), arm)
                self.assertEqual(extra, X.f_factor_delta((fac,), s), arm)
                self.assertGreater(len(extra), 0, arm)
                self.assertEqual(X.factors(arm), (fac,))
                union.update(extra)
            self.assertEqual(union, X.f8_extra(s))                 # the five factors together are F8's step
            self.assertEqual(dict(f4, **union), X.rung_delta("F8", s))

    def test_factor_keys_are_disjoint(self):
        seen = set()
        for fac in X.F_FACTOR_KEYS:
            keys = set(X.f_factor_delta((fac,), 4501))
            self.assertFalse(keys & seen, fac)
            seen |= keys
        self.assertGreater(len(seen), 10)

    def test_unknown_factor_refused_and_known_accepted(self):
        with self.assertRaises(ValueError):
            X.f_factor_delta(("speed",), 4501)
        self.assertTrue(X.f_factor_delta(("free",), 4501))

    def test_protocol_point_of_each_new_cell_matches_its_rung_and_not_the_others(self):
        for arm in S.NEW_ARMS:
            env = X.cell_navrl_env(arm, 4501, "/x/cell", 8, 4096, {})
            self.assertEqual(X.protocol_problems(env, arm), [], arm)
            others = [o for o in S.NEW_ARMS + ("F4", "F8") if o != arm]
            for o in others:                                    # non-vacuous: every other rung's protocol refuses it
                self.assertTrue(X.protocol_problems(env, o), (arm, o))

    def test_new_arms_install_only_their_modules(self):
        base = ["camref_rules", "camref_record", "inview_start", "inview_diag"]
        self.assertEqual(X.expected_installed("F4D"), base)
        self.assertEqual(X.expected_installed("F4R"), ["vehicle_rand"] + base)
        self.assertEqual(X.expected_installed("F4Y"), base[:2] + ["yawclip"] + base[2:])
        self.assertEqual(X.expected_installed("F4V_smoke"), base)
        self.assertEqual(X.expected_installed("F4A"), base)
        self.assertEqual(X.expected_installed("F4ID"), X.expected_installed("F4"))
        self.assertEqual(X.expected_installed("F8ID"), X.expected_installed("F8"))


@unittest.skipUnless(S.STAGE1_EVAL.is_dir(), "stage-1 cells absent")
class GeometryRecord(unittest.TestCase):
    """Review r1 P2-1: the applied geometry is read from the rule module's record, both ways, on stage-1 cells."""
    def _rec(self, arm):
        import json
        return json.loads((S.STAGE1_EVAL / arm / ("CAMREF_%s__205bars__seed4501" % arm) / "camref_receipt.json").read_text())

    def test_f4_cell_passes_as_f4_and_fails_as_f4a(self):
        rec = self._rec("F4")
        for arm in ("F4", "F4D", "F4R", "F4Y", "F4V"):
            self.assertEqual(X.geometry_problems(rec, arm), [], arm)
        self.assertTrue(X.geometry_problems(rec, "F4A"))
        self.assertTrue(X.geometry_problems(rec, "F8"))

    def test_f8_cell_passes_as_f8_and_f4a_and_fails_as_f4(self):
        rec = self._rec("F8")
        self.assertEqual(X.geometry_problems(rec, "F8"), [])
        self.assertEqual(X.geometry_problems(rec, "F4A"), [])
        self.assertTrue(X.geometry_problems(rec, "F4"))
        self.assertTrue(X.geometry_problems(rec, "F4D"))


class Stage2Install(unittest.TestCase):
    def test_installed_patches_and_restores(self):
        before = {k: getattr(X, k) for k in S.PATCHED}
        with S.installed() as R:
            self.assertEqual(R.SCOPE, S.SCOPE)
            self.assertNotEqual(R.scope_digest(), before["SCOPE"] and X.hashlib.sha256(before["SCOPE"].encode()).hexdigest())
            self.assertEqual([a for a, _, _ in R.eval_plan()][:5], list(S.NEW_ARMS))
            self.assertEqual(len(R.eval_plan()), 15)
            self.assertIn(S.PREREG, R.COMMITTED_FILES)
            self.assertIn("tools/camref_stage2_run.py", R.COMMITTED_FILES)
            self.assertEqual(R.reference_dir("CAMREF1_F8", 4502).name, "CAMREF_F8__205bars__seed4502")
        self.assertEqual({k: getattr(X, k) for k in S.PATCHED}, before)

    def test_plans_name_known_arms_and_identity_refs_are_pinned(self):
        for a, _, _ in S.SMOKE_PLAN:
            self.assertIn(X.ARM_BASE[a], S.NEW_ARMS)
        for a, (ref, seed) in S.IDENTITY_REF.items():
            self.assertIn((ref, seed), S.REF_SHA)
            self.assertEqual(X.rung(a), ref.split("_")[-1])
        self.assertEqual(len(S.REF_SHA), 6)

    def test_new_files_fit_the_code_allowlist(self):
        import fnmatch
        for f in S.NEW_FILES:
            self.assertTrue(any(fnmatch.fnmatch(f, p) for p in X.CODE_ALLOWLIST), f)

    @unittest.skipUnless(S.STAGE1_EVAL.is_dir(), "stage-1 cells absent")
    def test_stage1_references_at_their_pins(self):
        with S.installed() as R:
            self.assertEqual(R.reference_problems(), [])


class Analysis(unittest.TestCase):
    def test_controls_pass(self):
        ok, res = B.controls()
        self.assertTrue(ok, res)

    def test_interaction_is_zero_when_effects_add(self):
        cells = B.synthetic()
        for a, bump in (("F4D", 0.03), ("F4Y", 0.06)):
            cells[a] = B.synthetic(bump=(a, bump))[a]
        cells["F8"] = B.synthetic(bump=("F8", 0.09))["F8"]
        r = B.readings(cells)
        self.assertLess(abs(r["Q2"]["interaction"]["crash_bar"]["mean_pp"]), 0.25)     # count rounding (1/1024 = 0.1 pp)
        self.assertAlmostEqual(r["Q2"]["F8-F4"]["crash_bar"]["mean_pp"], 9.0, delta=0.25)
        self.assertEqual(B.use_of_result(r)["carriers"], ["F4Y"])

    def test_no_carrier_when_only_the_whole_step_is_material(self):
        cells = B.synthetic()
        cells["F8"] = B.synthetic(bump=("F8", 0.12))["F8"]
        r = B.readings(cells)
        self.assertEqual(r["Q2"]["F8-F4"]["crash_bar"]["label"], "MATERIAL")
        self.assertEqual(B.use_of_result(r)["carriers"], [])
        self.assertIn("no single factor", B.use_of_result(r)["text"])


if __name__ == "__main__":
    unittest.main()
