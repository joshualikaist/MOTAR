"""The seven-seed replication's design, statistics and analysis are fixed and correct before any data exist.

CPU only. Nothing here calls `run`, which starts the GPU.
"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import target_motion_replication as rep  # noqa: E402

PREREG = ROOT / rep.PREREG


def _ledger_module():
    spec = importlib.util.spec_from_file_location("navrl_episode_ledger_rep",
                                                  ROOT / "aerial_gym/task/navrl_task/navrl_episode_ledger.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DesignTest(unittest.TestCase):
    def test_cells_episodes_and_fresh_seeds(self):
        cells = rep.cells()
        self.assertEqual(len(cells), 112)
        self.assertEqual(len(set(cells)), 112)
        self.assertEqual(len(cells) * rep.EPISODES_PER_CELL, 229376)
        self.assertEqual(rep.EPISODES_PER_CELL, 2048)
        self.assertEqual(rep.SEEDS, (4104, 4105, 4106, 4107, 4108, 4109, 4110))
        original = json.loads((ROOT / "results/target_motion_e0_e2_2026-09-18/canonical_summary.json").read_text())
        used = {int(key.split("|")[1]) for key in original["arm_seed_means"]}
        self.assertFalse(used & set(rep.SEEDS), "a replication seed was already used")
        self.assertEqual([c[2] for c in cells[:16]], [4104] * 16, "seed-major order")
        self.assertNotIn(rep.CANARY_CELL[2], rep.SEEDS, "the canary must not consume a fresh seed")

    def test_the_condition_matches_the_original(self):
        self.assertEqual(rep.ARMS, ("H_historical", "E0_static", "E1_cv", "E2_obstacle_aware"))
        self.assertEqual(rep.DENSITIES, (70, 115, 160, 205))
        launcher = rep.LAUNCHER.read_text(encoding="utf-8")
        self.assertIn('EXPECT_SHA="f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"', launcher)

    def test_the_preregistration_states_the_same_design(self):
        text = PREREG.read_text(encoding="utf-8")
        for phrase in ("4104, 4105, 4106, 4107, 4108, 4109, 4110", "112 cells", "229,376", "exactly 2,048",
                       "16 episodes", "tools/target_motion_replication.py", "0.015625", "0.046875",
                       "NOT_RECORDED", "not part of the original preregistration", "Holm",
                       "instrumentation non-interference only", "No optional stopping",
                       "INDEPENDENT BIAS-CORRECTED REPLICATION", "pooled legacy stopping window",
                       "is reported descriptively and is **NOT** an additional pass/fail", "successful directional"
                       " statistical replication", "does not guarantee rejection"):
            self.assertIn(phrase, " ".join(text.split()))
        for claim in ("one seed of the opposite sign", "requires all seven", "all seven seeds must",
                      "every seed difference", "shares a sign"):
            self.assertNotIn(claim, text)


class StatisticsTest(unittest.TestCase):
    def test_minimum_exact_p_by_seed_count(self):
        self.assertEqual({n: rep.minimum_exact_p(n) for n in (3, 5, 6, 7)},
                         {3: 0.25, 5: 0.0625, 6: 0.03125, 7: 0.015625})
        self.assertEqual(rep.exact_sign_flip_p([0.01, 0.02, 0.015, 0.03, 0.012, 0.018]), 0.03125)
        self.assertGreater(rep.exact_sign_flip_p([0.01, 0.02, 0.015, 0.03, 0.012, -0.001]), 0.05)

    def test_seven_seeds_is_the_smallest_design_whose_holm_floor_is_below_005(self):
        floors = {n: min(rep.holm({"E0": rep.minimum_exact_p(n), "E1": rep.minimum_exact_p(n),
                                   "E2": rep.minimum_exact_p(n)}).values()) for n in (3, 5, 6, 7)}
        self.assertEqual(floors, {3: 0.75, 5: 0.1875, 6: 0.09375, 7: 0.046875})
        self.assertEqual(min(n for n, f in floors.items() if f < 0.05), 7)
        self.assertEqual(len(rep.SEEDS), 7)

    def test_bca_reproduces_the_original_intervals(self):
        original = json.loads((ROOT / "results/target_motion_e0_e2_2026-09-18/canonical_summary.json").read_text())
        for row in original["contrasts_vs_H"] + original["contrasts_within_E"]:
            diffs = [row["seed_diffs"][s] for s in sorted(row["seed_diffs"])]
            lo, hi = rep.bca_paired(diffs)
            self.assertAlmostEqual(lo, row["bca95"][0], places=12, msg=(row["arm"], row["metric"]))
            self.assertAlmostEqual(hi, row["bca95"][1], places=12, msg=(row["arm"], row["metric"]))


def synthetic(effects, flip_seed=None):
    """per_cell values with capture shifted per arm; one seed can flip every sign."""
    per_cell = {}
    for arm, bars, seed in rep.cells():
        shift = effects[arm] * (-1 if seed == flip_seed else 1)
        rate = {"capture": 0.85 + shift + 0.0001 * (seed % 7), "crash": 0.1, "timeout": 0.05 - shift,
                "min_relative_distance_m": 1.0}
        per_cell[(arm, bars, seed)] = {"quota": rate, "legacy": dict(rate, capture=rate["capture"] + 0.004),
                                       "legacy_episodes": 2049}
    return per_cell


class AnalysisTest(unittest.TestCase):
    EFFECTS = {"H_historical": 0.0, "E0_static": -0.035, "E1_cv": -0.028, "E2_obstacle_aware": 0.013}

    def test_consistent_effects_replicate(self):
        report = rep.analyze(synthetic(self.EFFECTS))
        self.assertEqual(report["verdict"], "REPLICATED")
        quota = report["estimators"]["quota"]
        self.assertTrue(all(r["exact_p"] == 0.015625 for r in quota["primary"]))
        self.assertTrue(all(r["holm_adjusted_p"] == 0.046875 for r in quota["primary"]))
        self.assertEqual(report["holm_floor_three_contrasts"], 0.046875)
        self.assertEqual(len(quota["contrasts"]), 6 * 4)
        self.assertAlmostEqual(report["length_bias_quota_minus_legacy_pp"]["E0_static"], -0.4)

    def test_classification_follows_only_the_inferential_calculation(self):
        # One seed with a fully reversed, equally large effect: the Holm-adjusted exact p decides.
        report = rep.analyze(synthetic(self.EFFECTS, flip_seed=4110))
        for r in report["estimators"]["quota"]["primary"]:
            expected = r["holm_adjusted_p"] < 0.05 and (r["mean_diff"] > 0) == (r["original_sign"] > 0)
            self.assertEqual(r["replicates"], expected)
            self.assertEqual(r["seeds_same_sign"], 6)          # reported, never a criterion
        source = (ROOT / "tools/target_motion_replication.py").read_text(encoding="utf-8")
        self.assertNotIn("seeds_same_sign ==", source)
        self.assertNotIn("seeds_same_sign >=", source)

    def test_the_legacy_window_never_decides_the_verdict(self):
        per_cell = synthetic(self.EFFECTS)
        for key in per_cell:
            per_cell[key]["legacy"] = dict(per_cell[key]["legacy"], capture=0.5)
        self.assertEqual(rep.analyze(per_cell)["verdict"], "REPLICATED")

    def test_load_cell_accepts_only_a_complete_ledger(self):
        ledger_module = _ledger_module()
        with tempfile.TemporaryDirectory() as tmp:
            cell = Path(tmp) / rep.cell_id("E2_obstacle_aware", 115, 4104)
            ledger = ledger_module.EpisodeLedger(
                128, 16, cell / ledger_module.ROWS_NAME, cell / ledger_module.SUMMARY_NAME,
                {"cell_id": cell.name, "arm": "e2_obstacle_aware", "bars": 115, "seed": 4104}, 2048)
            ones = torch.ones(128, dtype=torch.bool)
            for step in range(17):
                ledger.record(ones, ones, torch.zeros(128, dtype=torch.int32), ~ones, torch.full((128,), 0.3))
            ledger.finalize(legacy_result_written=True)
            loaded = rep.load_cell(tmp, "E2_obstacle_aware", 115, 4104)
            self.assertEqual(loaded["quota"]["capture"], 1.0)
            self.assertEqual(loaded["legacy_episodes"], 2048)
            import shutil
            shutil.copytree(cell, Path(tmp) / rep.cell_id("E2_obstacle_aware", 115, 4105))
            with self.assertRaises(ValueError):
                rep.load_cell(tmp, "E2_obstacle_aware", 115, 4105)  # a cell filed under the wrong seed
            (cell / ledger_module.ROWS_NAME).write_text("tampered\n")
            with self.assertRaises(ValueError):
                rep.load_cell(tmp, "E2_obstacle_aware", 115, 4104)


class GuardTest(unittest.TestCase):
    def test_canary_checks_cover_the_instrumentation_contract(self):
        self.assertEqual(len(rep.CANARY_CHECKS), 11)
        self.assertEqual([c.split("_")[0] for c in rep.CANARY_CHECKS], [str(n) for n in range(1, 12)])
        source = (ROOT / "tools/target_motion_replication.py").read_text(encoding="utf-8")
        self.assertIn("not evidence about target motion", source)
        self.assertIn('NAVRL_TRAJECTORY_DIGEST="1"', source)

    def test_comparable_ignores_only_the_nonce(self):
        a = {"outcome": {"captured": 1}, "condition": {"seed": 1, "evaluation_nonce": "x"}}
        b = {"outcome": {"captured": 1}, "condition": {"seed": 1, "evaluation_nonce": "y"}}
        self.assertEqual(rep.comparable(a), rep.comparable(b))
        b["outcome"]["captured"] = 2
        self.assertNotEqual(rep.comparable(a), rep.comparable(b))


class CanaryGateTest(unittest.TestCase):
    def ledger(self, tmp, crash_distance=0.4):
        ledger_module = _ledger_module()
        ledger = ledger_module.EpisodeLedger(128, 16, Path(tmp) / "e.jsonl", Path(tmp) / "s.json",
                                             {"cell_id": "c", "arm": "historical", "bars": 70, "seed": 4101}, 2048)
        ones = torch.ones(128, dtype=torch.bool)
        crash = torch.zeros(128, dtype=torch.int32)
        crash[0] = 1
        for _ in range(16):
            ledger.record(ones, ones & (crash == 0), crash, ~ones, torch.full((128,), crash_distance))
        summary = ledger.finalize(legacy_result_written=True)
        rows = [json.loads(line) for line in (Path(tmp) / "e.jsonl").read_text().splitlines()]
        return summary, rows

    def test_ledger_gates_pass_on_a_complete_ledger_and_fail_on_damage(self):
        with tempfile.TemporaryDirectory() as tmp:
            summary, rows = self.ledger(tmp)
            self.assertTrue(all(rep.ledger_gates(summary, rows).values()))
            self.assertTrue(any(r["crash"] for r in rows))
            self.assertFalse(rep.ledger_gates(summary, rows[:-1])["9_sixteen_per_environment"])
            dup = rows + [dict(rows[0])]
            self.assertFalse(rep.ledger_gates(summary, dup)["10_episode_ids_unique_complete"])
            bad = [dict(r, min_relative_distance_m=float("nan")) if r["crash"] else r for r in rows]
            self.assertFalse(rep.ledger_gates(summary, bad)["11_min_distance_every_episode"])

    def test_observation_gate_is_bitwise_on_common_samples(self):
        import numpy as np
        off = {"call_index": np.array([0, 0, 20, 20]), "env_id": np.array([0, 1, 0, 1]),
               "obs": np.arange(8, dtype=np.float32).reshape(4, 2)}
        on = {"call_index": np.array([0, 0, 20, 20, 40]), "env_id": np.array([0, 1, 0, 1, 0]),
              "obs": np.vstack([off["obs"], [[9, 9]]]).astype(np.float32)}
        self.assertEqual(rep.observation_gate(off, on), (True, 4))
        on["obs"][2, 1] = np.nextafter(on["obs"][2, 1], np.float32(100))
        self.assertEqual(rep.observation_gate(off, on)[0], False)
        self.assertEqual(rep.observation_gate(off, {k: v[4:] for k, v in on.items()}), (False, 0))

    def test_source_gate_allows_only_the_ledger_and_its_hook(self):
        ok, changed = rep.source_gate()
        self.assertTrue(ok, changed)


if __name__ == "__main__":
    unittest.main()
