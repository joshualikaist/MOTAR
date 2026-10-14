"""KROC2027 protocol, statistics, B rule and gate, pilot provenance, and the evaluation CLI (CPU).

The CLI's import guard is tested against the real failure mode: from a directory that is not the
repository root and without PYTHONPATH, an editable install resolves `aerial_gym` to another checkout.
"""
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import protocol as kp  # noqa: E402
from motar_eval.contracts import GateTolerances, MotionLimits  # noqa: E402


def _load_cli():
    spec = importlib.util.spec_from_file_location("_kroc2027_cli", ROOT / "tools/kroc2027_eval.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


cli = _load_cli()
PROTOCOL = kp.load_protocol(kp.PROTOCOL_PATH)
SEEDS = PROTOCOL["experiment_A"]["seeds"]


class ProtocolFileTest(unittest.TestCase):
    def test_motion_gate_matches_the_code(self):
        gate = PROTOCOL["motion_gate"]
        self.assertEqual(GateTolerances(**gate["tolerances"]), GateTolerances())
        self.assertEqual(MotionLimits(**gate["limits"]), MotionLimits())
        tol = GateTolerances()
        derived = gate["derived_g2"]
        self.assertEqual(derived["ulp_m"], tol.ulp_m())
        self.assertAlmostEqual(derived["accel_mps2"], tol.g2_accel_mps2(0.1), delta=1e-7)
        self.assertAlmostEqual(derived["speed_mps"], tol.g2_speed_mps(0.1), delta=1e-8)
        self.assertAlmostEqual(derived["position_m"], tol.g2_position_m(), delta=1e-9)
        self.assertAlmostEqual(derived["turn_deg_s_at_0p20_mps"],
                               math.degrees(tol.g2_turn_rad_s(0.20, 0.1)), delta=1e-3)

    def test_pinned_files(self):
        pol = PROTOCOL["policy"]
        self.assertEqual(kp.sha256_file(ROOT / pol["launcher"]), pol["launcher_sha256"])
        self.assertEqual(kp.sha256_file(ROOT / pol["player"]), pol["player_sha256"])
        helpers = PROTOCOL["analysis_A"]
        self.assertEqual(kp.sha256_file(ROOT / helpers["helpers"]), helpers["helpers_sha256"])

    def test_seeds_are_fresh_and_disjoint(self):
        a = set(PROTOCOL["experiment_A"]["seeds"])
        b = set(PROTOCOL["experiment_B"]["seeds"])
        canary = {PROTOCOL["experiment_A"]["canary"]["seed"]}
        historic = set(range(4101, 4111))
        self.assertEqual(len(a), 7)
        self.assertEqual(len(b), 7)
        self.assertFalse(a & b or a & canary or b & canary)
        self.assertFalse((a | b | canary) & historic)

    def test_cells(self):
        main = kp.a_cells(PROTOCOL, "main", "E2_obstacle_aware")
        self.assertEqual(len(main), 112)
        self.assertEqual(main[0]["seed"], SEEDS[0])
        self.assertEqual(main[-1]["seed"], SEEDS[-1])
        self.assertEqual(main[0]["cell_dir"], "A1/E0_static__70bars__seed4201")
        self.assertEqual({c["target_model"] for c in main if c["target"] == "moving"}, {"E2_obstacle_aware"})
        self.assertEqual(len(kp.a_cells(PROTOCOL, "main", None)), 56)
        canary = kp.a_cells(PROTOCOL, "canary", "E1_cv")
        self.assertEqual({c["seed"] for c in canary}, {4190})
        with self.assertRaises(ValueError):
            kp.a_cells(PROTOCOL, "pilot", None)


def _rows(num_envs, quota, extra=0):
    rows = []
    for env in range(num_envs):
        for k in range(quota + extra):
            rows.append({"env_index": env, "env_episode_index": k, "in_quota": k < quota,
                         "outcome": "capture" if (env + k) % 3 else "timeout",
                         "episode_id": "c/env%03d/ep%03d" % (env, k)})
    return rows


class QuotaJoinMetricsTest(unittest.TestCase):
    def test_quota_rows(self):
        rows = _rows(4, 3, extra=2)
        self.assertEqual(len(kp.quota_rows(rows, 4, 3)), 12)
        with self.assertRaises(ValueError):
            kp.quota_rows(rows[:-3], 4, 3)
        with self.assertRaises(ValueError):
            kp.quota_rows(rows, 5, 3)

    def test_join(self):
        quota = kp.quota_rows(_rows(2, 2), 2, 2)
        records = [{"env_index": r["env_index"], "env_episode_index": r["env_episode_index"],
                    "outcome": r["outcome"], "ever_fused_valid": True} for r in quota]
        self.assertEqual(len(kp.join_episode_records(quota, records)), 4)
        with self.assertRaises(ValueError):
            kp.join_episode_records(quota, records[:-1])
        with self.assertRaises(ValueError):
            kp.join_episode_records(quota, records + records[:1])
        wrong = [dict(r) for r in records]
        wrong[0]["outcome"] = "crash" if wrong[0]["outcome"] != "crash" else "capture"
        with self.assertRaises(ValueError):
            kp.join_episode_records(quota, wrong)

    def test_cell_metrics_and_seed_level(self):
        episodes = [
            {"outcome": "capture", "ever_fused_valid": True},
            {"outcome": "timeout", "ever_fused_valid": False},
            {"outcome": "crash", "ever_fused_valid": False},
            {"outcome": "crash", "ever_fused_valid": True},
        ]
        m = kp.cell_metrics(episodes)
        self.assertEqual((m["capture"], m["crash"], m["timeout"]), (0.25, 0.5, 0.25))
        self.assertEqual(m["never_acquired"], 0.5)
        self.assertEqual(m["never_acquired_timeout"], 0.25)
        self.assertEqual(m["never_acquired_crash"], 0.25)
        self.assertEqual(m["never_acquired_failure"], 0.5)
        self.assertEqual(m["acquired_failure"], 0.25)
        per_cell = {("A1", d, 1): {"capture": d / 1000.0} for d in (70, 115, 160, 205)}
        level = kp.seed_level(per_cell, ["A1"], [70, 115, 160, 205], [1], "capture")
        self.assertAlmostEqual(level[("A1", 1)], (70 + 115 + 160 + 205) / 4000.0)


class StatisticsTest(unittest.TestCase):
    def test_paired_t_matches_scipy(self):
        from scipy import stats
        diffs = [0.031, 0.018, 0.044, 0.025, 0.012, 0.037, 0.029]
        out = kp.paired_t(diffs)
        ref = stats.ttest_1samp(diffs, 0.0)
        self.assertAlmostEqual(out["t"], float(ref.statistic), places=10)
        self.assertAlmostEqual(out["p_two_sided"], float(ref.pvalue), places=12)
        half = stats.t.ppf(0.975, 6) * stats.sem(diffs)
        self.assertAlmostEqual(out["ci"][0], sum(diffs) / 7 - half, places=12)

    def _capture(self, static_gap, moving_gap, noise=0.002):
        capture = {}
        for i, s in enumerate(SEEDS):
            wobble = noise * ((i % 3) - 1)
            capture[("A1", s)] = 0.95 + wobble
            capture[("A2", s)] = 0.95 - static_gap - wobble
            capture[("A3", s)] = 0.90 + wobble
            capture[("A4", s)] = 0.90 - moving_gap + 0.5 * wobble
        return capture

    def test_contrast_definitions(self):
        c = kp.a_contrasts(self._capture(0.10, 0.04, noise=0.0), SEEDS)
        for d, i, s, m in zip(c["Delta_A"], c["I_A"], c["static_visible_minus_hidden"],
                              c["moving_visible_minus_hidden"]):
            self.assertAlmostEqual(d, 0.5 * (s + m))
            self.assertAlmostEqual(i, m - s)
        self.assertAlmostEqual(c["Delta_A"][0], 0.07)
        self.assertAlmostEqual(c["I_A"][0], -0.06)

    def test_decisions(self):
        analysis = PROTOCOL["analysis_A"]
        big = kp.analyse_a(self._capture(0.10, 0.04), SEEDS, analysis, True)["contrasts"]
        self.assertEqual(big["Delta_A"]["decision"], "PRACTICALLY_SUPPORTED")
        self.assertEqual(big["I_A"]["decision"], "PRACTICALLY_SUPPORTED")
        self.assertEqual(big["Delta_A"]["sensitivity"]["seeds_in_predicted_direction"], 7)
        small = kp.analyse_a(self._capture(0.012, 0.012, noise=0.001), SEEDS, analysis, True)["contrasts"]
        self.assertEqual(small["Delta_A"]["decision"], "SUPPORTED")
        self.assertEqual(small["I_A"]["decision"], "INCONCLUSIVE")
        opposite = kp.analyse_a(self._capture(0.02, 0.10), SEEDS, analysis, True)["contrasts"]
        self.assertEqual(opposite["I_A"]["decision"], "OPPOSITE_DIRECTION")
        invalid = kp.analyse_a(self._capture(0.10, 0.04), SEEDS, analysis, False)["contrasts"]
        self.assertEqual(invalid["Delta_A"]["decision"], "INVALID")
        holm = [big[name]["holm_adjusted_p"] for name in ("Delta_A", "I_A")]
        raw = sorted(big[name]["paired_t"]["p_two_sided"] for name in ("Delta_A", "I_A"))
        self.assertAlmostEqual(min(holm), min(1.0, 2 * raw[0]))
        self.assertAlmostEqual(big["Delta_A"]["practical_interval_confidence"], 0.975)

    def test_helpers_are_the_replication_tools_own(self):
        helpers = kp.replication_helpers()
        self.assertEqual(Path(helpers.__file__).resolve(), (ROOT / "tools/target_motion_replication.py").resolve())
        self.assertEqual(helpers.minimum_exact_p(7), 2 / 128)


class ExperimentBRuleTest(unittest.TestCase):
    def test_arm_rule(self):
        def rates(static, moving):
            out = {("A2", s): static for s in SEEDS}
            if moving is not None:
                out.update({("A4", s): moving for s in SEEDS})
            return out
        self.assertEqual(kp.select_b_arm(rates(0.14, 0.06), SEEDS)["arm"], "A2")
        self.assertEqual(kp.select_b_arm(rates(0.06, 0.14), SEEDS)["arm"], "A4")
        self.assertEqual(kp.select_b_arm(rates(0.100, 0.104), SEEDS)["arm"], "A2")
        self.assertEqual(kp.select_b_arm(rates(0.100, None), SEEDS)["arm"], "A2")

    def test_density_rule_and_gate(self):
        self.assertEqual(kp.b_density_count(11.8, 9.0, 0.25, 2.0), 4)
        self.assertEqual(kp.b_density_count(9.5, 9.0, 0.25, 2.0), 2)
        good = [0.05, 0.06, 0.04, 0.07, 0.05, 0.06, 0.05]
        gate = kp.b_gate(True, {"arm": "A2"}, good, good, 0.8, 12.0, 9.0, 1.0, True, True)
        self.assertEqual(gate["verdict"], "GO")
        no = kp.b_gate(True, {"arm": "A2"}, good, good, 0.4, 12.0, 9.0, 1.0, True, True)
        self.assertEqual(no["verdict"], "NO_GO")
        self.assertFalse(no["criteria"]["3_never_acquired_failures_majority"])
        unapproved = kp.b_gate(True, {"arm": "A2"}, good, good, 0.8, 12.0, 9.0, 1.0, True, False)
        self.assertEqual(unapproved["verdict"], "NO_GO")


class PilotProvenanceTest(unittest.TestCase):
    def test_committed_pilot_is_traceable_to_the_manifest(self):
        pilot = json.loads((ROOT / "docs/plans/kroc2027_pilot_v1.json").read_text())
        self.assertEqual(pilot["role"], "EXPLORATORY_ALREADY_OBSERVED_NOT_CONFIRMATORY")
        manifest = json.loads((ROOT / "results/target_motion_replication_7seed/RAW_MANIFEST.json").read_text())
        listing = "\n".join("%s %s" % (k, manifest["files"][k])
                            for k in sorted(k for k in manifest["files"] if k.endswith("/result.json")))
        self.assertEqual(pilot["source_files"], 112)
        self.assertEqual(pilot["source_listing_sha256"], hashlib.sha256(listing.encode()).hexdigest())

    def test_regenerates_exactly_when_raw_cells_exist(self):
        raw = ROOT / "results/target_motion_replication_7seed"
        if not (raw / "E0_static__70bars__seed4104" / "result.json").is_file():
            self.skipTest("the replication cell folders are git-ignored and absent from this checkout")
        record = kp.pilot_from_replication(raw)
        text = json.dumps(record, indent=1, sort_keys=True) + "\n"
        self.assertEqual(text, (ROOT / "docs/plans/kroc2027_pilot_v1.json").read_text())


class CliPureFunctionTest(unittest.TestCase):
    def test_environment_is_scrubbed_and_pinned_to_the_tree(self):
        inherited = {"PATH": "/usr/bin", "NAVRL_KROC2027_SEARCH": "b1", "NAVRL_EPISODE_LEDGER": "1",
                     "PYTHONPATH": "/elsewhere", "NAVRL_SEED": "1"}
        env = cli.base_env(inherited=inherited)
        self.assertEqual(env["PYTHONPATH"], str(ROOT))
        self.assertNotIn("NAVRL_KROC2027_SEARCH", env)
        self.assertNotIn("NAVRL_EPISODE_LEDGER", env)
        self.assertEqual(env["NAVRL_SEED"], "1")

    def test_cell_env(self):
        env = cli.base_env(inherited={"PATH": "/usr/bin"})
        off = cli.cell_env(env, PROTOCOL, target_model="E0_static", density=70, seed=4104,
                           out_dir=Path("/tmp/x"), quota=16)
        self.assertFalse(any(k.startswith("NAVRL_KROC2027_") for k in off))
        self.assertEqual((off["ARMS"], off["GAMES"], off["NAVRL_EPISODE_QUOTA_PER_ENV"]), ("E0_static", "2048", "16"))
        on = cli.cell_env(env, PROTOCOL, target_model="E2_obstacle_aware", density=205, seed=4201,
                          out_dir=Path("/tmp/x"), quota=4, view="hidden", recorder=True,
                          run_id="KROC27-A-C01", arm="A4")
        self.assertEqual(on["NAVRL_KROC2027_INITIAL_VIEW"], "hidden")
        self.assertEqual(on["NAVRL_KROC2027_INITIAL_DISTANCE_M"], "10.0")
        self.assertEqual(on["NAVRL_KROC2027_RECORDER"], "1")
        self.assertEqual(on["GAMES"], "512")
        self.assertEqual(on["NAVRL_KROC2027_PROTOCOL_SHA256"], kp.sha256_file(kp.PROTOCOL_PATH))

    def test_ledger_and_recorder_checks(self):
        rows = _rows(2, 2)
        summary = {"status": "COMPLETE", "partition_violations": 0}
        self.assertEqual(cli.ledger_problems(summary, rows, 2, 2), [])
        self.assertTrue(cli.ledger_problems(dict(summary, status="RUNNING"), rows, 2, 2))
        records = [{"env_index": r["env_index"], "env_episode_index": r["env_episode_index"],
                    "outcome": r["outcome"]} for r in rows]
        rec_summary = {"import_origins": {"aerial_gym": str(ROOT / "aerial_gym/__init__.py"),
                                          "motar_eval": str(ROOT / "motar_eval/__init__.py")},
                       "motion_gate": {"verdict": "PASS"},
                       "swept_clearance": {"violations": 0, "wall_violations": 0},
                       "initial_view": {"failed_closed": 0}}
        self.assertEqual(cli.recorder_problems(rec_summary, records, rows), [])
        outside = json.loads(json.dumps(rec_summary))
        outside["import_origins"]["aerial_gym"] = "/home/someone/else/aerial_gym/__init__.py"
        self.assertTrue(cli.recorder_problems(outside, records, rows))
        self.assertTrue(cli.recorder_problems(rec_summary, records[:-1], rows))
        failed = dict(rec_summary, motion_gate={"verdict": "FAIL"})
        self.assertTrue(cli.recorder_problems(failed, records, rows))
        self.assertEqual(cli.recorder_problems(failed, records, rows, require_motion=False), [])

    def test_scale_rule_and_projection(self):
        seconds = {("A1", 70): 100.0, ("A2", 70): 120.0}
        hours = cli.project_a_hours(seconds, [70, 115], 4, 16, SEEDS)
        self.assertAlmostEqual(hours, (100 + 120) * 4 * 7 / 3600.0)
        budget = PROTOCOL["gpu_budget_hours"]
        self.assertEqual(cli.scale_decision(15.0, 2.0, budget)["densities"], [70, 115, 160, 205])
        self.assertEqual(cli.scale_decision(25.0, 2.0, budget)["densities"], [115, 205])

    def test_comparable_strips_only_the_nonce(self):
        a = {"outcome": {"capture": 1}, "condition": {"evaluation_nonce": "x", "bars": 70}}
        b = {"outcome": {"capture": 1}, "condition": {"evaluation_nonce": "y", "bars": 70}}
        self.assertEqual(cli.comparable(a), cli.comparable(b))
        b["condition"]["bars"] = 115
        self.assertNotEqual(cli.comparable(a), cli.comparable(b))

    def test_canary_plan(self):
        runs = cli.canary_subruns(PROTOCOL, Path("/tmp/canary"), "KROC27-A-C01")
        names = [n for n, _ in runs]
        self.assertEqual(names[:3], ["off_path", "recorder_off", "recorder_on"])
        self.assertEqual(len(names), 3 + 4 * 2 + 4 * 2 * 2)
        off = dict(runs)["off_path"]
        self.assertEqual((off["target_model"], off["density"], off["seed"], off["quota"]),
                         ("E0_static", 70, 4104, 16))
        self.assertIsNone(off.get("view"))

    def test_run_refuses_without_approval(self):
        out = Path(tempfile.mkdtemp()) / "never"
        head, problems = cli.preconditions("canary", PROTOCOL, "KROC27-A-C01", "", out,
                                           gpus=["NVIDIA GeForce RTX 3070"])
        self.assertTrue(any("approved-commit" in p for p in problems))
        _, problems = cli.preconditions("canary", PROTOCOL, "KROC27-A-M01", head, out,
                                        gpus=["NVIDIA GeForce RTX 3070"])
        self.assertTrue(any("run id" in p for p in problems))
        _, problems = cli.preconditions("canary", PROTOCOL, "KROC27-A-C01", head, out,
                                        gpus=["NVIDIA GeForce GTX 1650 Ti"])
        self.assertTrue(any("RTX 3070" in p for p in problems))
        _, problems = cli.preconditions("main", PROTOCOL, "KROC27-A-M01", head, out,
                                        gpus=["NVIDIA GeForce RTX 3070"])
        self.assertTrue(any("CANARY_PASS" in p for p in problems))


class ImportGuardTest(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp, True)
        for package in ("aerial_gym", "motar_eval"):
            (self.tmp / package).mkdir()
            (self.tmp / package / "__init__.py").write_text("")
        (self.tmp / "launcher_dir").mkdir()

    def _env(self, with_path):
        env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}
        env["PYTHONNOUSERSITE"] = "1"
        if with_path:
            env["PYTHONPATH"] = str(self.tmp)
        return env

    def test_tree_on_pythonpath_passes(self):
        problems, origins = cli.import_problems(self._env(True), self.tmp / "launcher_dir", root=self.tmp)
        self.assertEqual(problems, [])
        self.assertTrue(cli.is_inside(origins["aerial_gym"], self.tmp))

    def test_without_pythonpath_the_tree_is_not_what_imports(self):
        problems, origins = cli.import_problems(self._env(False), self.tmp / "launcher_dir", root=self.tmp)
        self.assertTrue(any(p.startswith("aerial_gym") for p in problems), (problems, origins))
        self.assertTrue(any(p.startswith("motar_eval") for p in problems), (problems, origins))

    def test_symlink_out_of_the_tree_is_outside(self):
        outside = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, outside, True)
        (outside / "pkg.py").write_text("")
        link = self.tmp / "link"
        link.symlink_to(outside, target_is_directory=True)
        self.assertFalse(cli.is_inside(link / "pkg.py", self.tmp))
        self.assertTrue(cli.is_inside(self.tmp / "aerial_gym/__init__.py", self.tmp))

    def test_this_tree_resolves_to_itself_from_the_launcher_directory(self):
        problems, origins = cli.import_problems(cli.base_env(), cli.launcher_path(PROTOCOL).parent)
        self.assertEqual(problems, [], origins)


if __name__ == "__main__":
    unittest.main()
