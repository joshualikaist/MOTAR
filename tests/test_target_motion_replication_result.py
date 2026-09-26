"""The finalized seven-seed replication record is complete, internally consistent and worded within its limits.

Runs without the raw cells, which stay out of Git: every check uses the committed compact files.
"""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
R = ROOT / "results/target_motion_replication_7seed"
sys.path.insert(0, str(ROOT / "tools"))
import target_motion_replication as rep  # noqa: E402


def load(name):
    return json.loads((R / name).read_text(encoding="utf-8"))


def cells():
    with open(R / "cells.csv", encoding="utf-8") as handle:
        return {(row["arm"], int(row["bars"]), int(row["seed"])): row for row in csv.DictReader(handle)}


def seed_rates(table, estimator):
    """The preregistered aggregation: per seed, the unweighted mean over the four density cells."""
    out = {}
    for arm in rep.ARMS:
        for seed in rep.SEEDS:
            rows = [table[(arm, bars, seed)] for bars in rep.DENSITIES]
            n = "primary_episodes" if estimator == "quota" else "legacy_episodes"
            prefix = "primary_" if estimator == "quota" else "legacy_"
            out[(arm, seed)] = {m: sum(int(r[prefix + m]) / int(r[n]) for r in rows) / len(rows) for m in rep.METRICS}
    return out


class RecordCompletenessTest(unittest.TestCase):
    def test_grid_counts_and_integrity(self):
        table = cells()
        self.assertEqual(set(table), set(rep.cells()))
        self.assertTrue(all(int(r["primary_episodes"]) == 2048 for r in table.values()))
        self.assertEqual(sum(int(r["primary_episodes"]) for r in table.values()), 229376)
        for r in table.values():
            self.assertEqual(sum(int(r["primary_" + m]) for m in rep.METRICS), 2048)
            self.assertEqual(int(r["raw_ledger_rows"]) - 2048, int(r["nonprimary_tail_rows"]))
        integrity = load("integrity_report.json")
        self.assertTrue(integrity["VALID"])
        self.assertEqual(integrity["raw_ledger_rows"], sum(int(r["raw_ledger_rows"]) for r in table.values()))
        self.assertEqual(integrity["nonprimary_tail_rows"], sum(int(r["nonprimary_tail_rows"]) for r in table.values()))

    def test_frozen_files_match_their_recorded_hashes(self):
        record = load("replication_record.json")
        for name, key in (("RAW_MANIFEST.json", "raw_manifest_sha256"),
                          ("primary_selection_manifest.json", "primary_selection_manifest_sha256")):
            self.assertEqual(hashlib.sha256((R / name).read_bytes()).hexdigest(), record[key], name)
        raw = load("RAW_MANIFEST.json")
        commit = "b1c0bc612339d2f8dfb86a0ca16de11bdffe09b5"
        self.assertEqual(raw["source_commit"], commit)
        self.assertEqual(record["measurement_commit"], commit)
        for row in cells().values():
            self.assertEqual(raw["files"][row["cell_id"] + "/episodes.jsonl"], row["rows_sha256"])
        selection = load("primary_selection_manifest.json")
        self.assertEqual(len(selection), 112)
        self.assertTrue(all(len(v["primary_episode_ids"]) == 2048 for v in selection.values()))

    def test_canary_gates_and_digests(self):
        canary = load("canary_provenance.json")
        self.assertEqual(canary["verdict"], "CANARY_PASS")
        self.assertEqual(len(canary["gates"]), 11)
        self.assertTrue(all(canary["gates"].values()))
        self.assertTrue(canary["trajectory_digest"]["ledger_on_equal"])
        eq = canary["observation_equality"]
        self.assertEqual(eq["sha256_ledger_off"], eq["sha256_ledger_on"])
        self.assertEqual(canary["legacy_reproduction"]["original_outcome"], canary["legacy_reproduction"]["ledger_off_outcome"])


class AnalysisConsistencyTest(unittest.TestCase):
    def setUp(self):
        self.analysis = load("analysis.json")

    def test_analysis_matches_the_compact_table(self):
        table = cells()
        for estimator in ("quota", "legacy"):
            rates = seed_rates(table, estimator)
            block = self.analysis["estimators"][estimator]
            for arm in rep.ARMS:
                mean = sum(rates[(arm, s)]["capture"] for s in rep.SEEDS) / len(rep.SEEDS)
                self.assertAlmostEqual(block["arm_mean_capture"][arm], mean, places=12)
            for row in block["contrasts"]:
                if row["metric"] == "min_relative_distance_m":
                    continue
                diffs = [rates[(row["arm"], s)][row["metric"]] - rates[(row["baseline"], s)][row["metric"]]
                         for s in rep.SEEDS]
                self.assertEqual(len(diffs), 7)
                for got, want in zip(diffs, row["seed_diffs"].values()):
                    self.assertAlmostEqual(got, want, places=12)

    def test_inference_follows_the_preregistered_rule(self):
        primary = self.analysis["estimators"]["quota"]["primary"]
        holm = rep.holm({r["arm"]: rep.exact_sign_flip_p(list(r["seed_diffs"].values())) for r in primary})
        for r in primary:
            diffs = list(r["seed_diffs"].values())
            self.assertEqual(r["exact_p"], rep.exact_sign_flip_p(diffs))
            self.assertEqual(list(rep.bca_paired(diffs)), r["bca95"])
            self.assertEqual(r["holm_adjusted_p"], holm[r["arm"]])
            direction_kept = (r["mean_diff"] > 0) == (rep.ORIGINAL_SIGN[r["arm"]] > 0)
            self.assertEqual(r["replicates"], direction_kept and r["holm_adjusted_p"] < 0.05)
        self.assertEqual(self.analysis["verdict"], "REPLICATED")
        self.assertEqual(self.analysis["design"], "INDEPENDENT BIAS-CORRECTED REPLICATION")


class ReportWordingTest(unittest.TestCase):
    def setUp(self):
        self.text = " ".join((R / "README.md").read_text(encoding="utf-8").split())
        self.analysis = load("analysis.json")

    def test_headline_numbers_come_from_the_analysis(self):
        q = self.analysis["estimators"]["quota"]
        for arm, pct in q["arm_mean_capture"].items():
            self.assertIn(f"{100 * pct:.2f} %", self.text, arm)
        for r in q["primary"]:
            mean = 100 * r["mean_diff"]
            self.assertIn(f"**{'+' if mean > 0 else '−'}{abs(mean):.2f}**", self.text, r["arm"])
            lo, hi = (100 * x for x in r["bca95"])
            self.assertIn("[%s%.2f, %s%.2f]" % ("+" if lo > 0 else "−", abs(lo), "+" if hi > 0 else "−", abs(hi)),
                          self.text)
        legacy = self.analysis["estimators"]["legacy"]
        for rq, rl in zip(q["primary"], legacy["primary"]):
            delta = 100 * (rq["mean_diff"] - rl["mean_diff"])
            self.assertIn("%s%.2f pp" % ("+" if delta > 0 else "−", abs(delta)), self.text)

    def test_arm_tables_come_from_the_compact_table(self):
        table = cells()
        quota, legacy = seed_rates(table, "quota"), seed_rates(table, "legacy")

        def mean(rates, arm, metric):
            return 100 * sum(rates[(arm, s)][metric] for s in rep.SEEDS) / len(rep.SEEDS)
        for arm, short in zip(rep.ARMS, ("H", "E0", "E1", "E2")):
            row = "| %s | %.2f %% | %.2f %% | %.2f %% |" % (short, *(mean(quota, arm, m) for m in rep.METRICS))
            self.assertIn(row, self.text, arm)
            diffs = [mean(quota, arm, m) - mean(legacy, arm, m) for m in rep.METRICS]
            cells_text = " | ".join("%s%.2f" % ("+" if d > 0 else "−", abs(d)) for d in diffs)
            self.assertIn("| %s | %s |" % (short, cells_text), self.text, arm)

    def test_required_and_forbidden_wording(self):
        for phrase in ("All three preregistered capture contrasts met the directional replication criterion in the "
                       "independent seven-seed bias-corrected replication",
                       "at the resolution boundary of the seven-seed exact sign-flip design",
                       "attenuated all three capture contrasts toward zero",
                       "seed-set variation cannot be separated from the cross-campaign difference",
                       "was not uniformly expressed at high obstacle densities",
                       "5/7 at 160 bars", "4/7 at 205 bars", "not pooled", "PPO_TRAINING_STARTED = false"):
            self.assertIn(phrase, self.text)
        lowered = self.text.lower()
        for claim in ("highly significant", "strong statistical significance", "conclusive across all densities",
                      "universally improves", "significant at every density"):
            self.assertNotIn(claim, lowered)

    def test_density_counts_are_the_recorded_ones(self):
        dens = self.analysis["estimators"]["quota"]["per_density_exploratory"]
        self.assertEqual(dens["E2_obstacle_aware-H@160"]["seeds_same_sign"], 5)
        self.assertEqual(dens["E2_obstacle_aware-H@205"]["seeds_same_sign"], 4)

    def test_min_distance_is_withheld_while_its_semantic_check_fails(self):
        check = load("min_distance_semantic_check.json")
        self.assertEqual(check["verdict"], "SEMANTIC_MISMATCH_METRIC_NOT_PUBLISHED")
        self.assertGreater(check["violations"], 0)
        self.assertIn("not published", self.text)
        self.assertNotRegex(self.text, r"\d\.\d+ m \|")          # no per-arm distance column


class RawRederivationTest(unittest.TestCase):
    def test_compact_record_rederives_from_raw_when_present(self):
        if not (R / rep.cell_id(*rep.cells()[0])).is_dir():
            self.skipTest("raw cells are archived outside Git")
        out = subprocess.run([sys.executable, "-B", str(ROOT / "tools/build_tm_replication_record.py"), "--check"],
                             capture_output=True, text=True)
        self.assertEqual(out.returncode, 0, out.stdout + out.stderr)


if __name__ == "__main__":
    unittest.main()
