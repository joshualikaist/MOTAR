"""The per-episode ledger counts exactly quota x envs episodes, stays unbiased, and changes nothing.

CPU only. The GPU half of the non-interference proof is the replication canary: identical aggregate
`result.json` with the ledger on and off (docs/prereg_2026-09-24_target_motion_replication_7seed.md).
"""
import ast
import hashlib
import json
from pathlib import Path
import random
import tempfile
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT / "aerial_gym/task/navrl_task/navrl_task.py"
LEDGER = ROOT / "aerial_gym/task/navrl_task/navrl_episode_ledger.py"


def _load():
    """Load by file path: importing the package would pull in Isaac Gym, which refuses to be
    imported after torch. The ledger imports nothing from aerial_gym, so the file is the whole module."""
    import importlib.util
    import sys
    spec = importlib.util.spec_from_file_location("navrl_episode_ledger", LEDGER)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ledger_module = _load()
EpisodeLedger, build_episode_ledger = ledger_module.EpisodeLedger, ledger_module.build_episode_ledger


def simulate(num_envs, quota, export_at, rng, lengths, probabilities, directory, max_steps=100000):
    """Drive a ledger with a renewal process: each env runs episodes back to back."""
    ledger = EpisodeLedger(num_envs, quota, directory / "episodes.jsonl", directory / "summary.json",
                           {"cell_id": "cell", "arm": "arm", "bars": 70, "seed": 1}, export_at)
    remaining = np.zeros(num_envs, dtype=int)
    outcome = np.zeros(num_envs, dtype=int)
    distance = torch.full((num_envs,), 5.0)

    def start(env):
        outcome[env] = rng.choice(3, p=probabilities)
        remaining[env] = lengths[outcome[env]](rng)
    for env in range(num_envs):
        start(env)
    steps = 0
    while not ledger.complete and steps < max_steps:
        steps += 1
        remaining -= 1
        finished = torch.from_numpy(remaining <= 0)
        codes = torch.from_numpy(outcome.copy())
        ledger.record(finished, finished & (codes == 0), (finished & (codes == 1)).to(torch.int32),
                      finished & (codes == 2), distance)
        for env in np.nonzero(remaining <= 0)[0]:
            start(env)
    return ledger


LENGTHS = (lambda r: int(r.integers(3, 25)),   # capture: short
           lambda r: int(r.integers(2, 40)),   # crash
           lambda r: 60)                       # timeout: the full episode


class QuotaAndLegacyWindowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def rows(self, ledger):
        ledger.finalize(legacy_result_written=True)
        return [json.loads(line) for line in (self.dir / "episodes.jsonl").read_text().splitlines()]

    def test_exactly_quota_episodes_from_every_environment(self):
        ledger = simulate(16, 8, 128, np.random.default_rng(3), LENGTHS, [0.8, 0.1, 0.1], self.dir)
        rows = self.rows(ledger)
        quota_rows = [r for r in rows if r["in_quota"]]
        self.assertEqual(len(quota_rows), 16 * 8)
        for env in range(16):
            mine = [r["env_episode_index"] for r in rows if r["env_index"] == env]
            self.assertEqual(mine, list(range(len(mine))), "per-env ordinals are consecutive")
            self.assertEqual([r["env_episode_index"] for r in quota_rows if r["env_index"] == env],
                             list(range(8)))
        summary = json.loads((self.dir / "summary.json").read_text())
        self.assertEqual(summary["status"], "COMPLETE")
        self.assertEqual(summary["quota"]["episodes"], 128)
        self.assertEqual(sum(summary["quota"]["counts"].values()), 128)
        self.assertEqual(summary["rows_sha256"],
                         hashlib.sha256((self.dir / "episodes.jsonl").read_bytes()).hexdigest())

    def test_rows_carry_the_required_fields_in_completion_order(self):
        rows = self.rows(simulate(8, 4, 32, np.random.default_rng(5), LENGTHS, [0.7, 0.2, 0.1], self.dir))
        for field in ("cell_id", "seed", "bars", "arm", "episode_id", "outcome", "min_relative_distance_m",
                      "capture", "crash", "timeout", "completion_step", "env_index", "in_quota"):
            self.assertIn(field, rows[0])
        order = [(r["completion_step"], r["env_index"]) for r in rows]
        self.assertEqual(order, sorted(order))
        self.assertEqual([r["completion_rank"] for r in rows], list(range(1, len(rows) + 1)))
        self.assertEqual(len({r["episode_id"] for r in rows}), len(rows))
        for r in rows:
            self.assertEqual(sum((r["capture"], r["crash"], r["timeout"])), 1)

    def test_legacy_window_is_the_old_count_based_stop(self):
        rows = self.rows(simulate(16, 8, 128, np.random.default_rng(7), LENGTHS, [0.8, 0.1, 0.1], self.dir))
        # Reimplement the 2026-09-18 rule: export on the first step whose pooled total reaches N.
        total, cutoff = 0, None
        for step in sorted({r["completion_step"] for r in rows}):
            total += sum(1 for r in rows if r["completion_step"] == step)
            if total >= 128:
                cutoff = step
                break
        expected = [r["episode_id"] for r in rows if r["completion_step"] <= cutoff]
        self.assertEqual([r["episode_id"] for r in rows if r["in_legacy_window"]], expected)
        self.assertGreaterEqual(len(expected), 128)

    def test_quota_rule_is_unbiased_and_the_count_stop_is_not(self):
        rng = np.random.default_rng(11)
        truth = np.array([0.80, 0.10, 0.10])
        quota_bias, legacy_bias = [], []
        for rep in range(60):
            sub = self.dir / str(rep)
            sub.mkdir()
            ledger = simulate(32, 8, 256, rng, LENGTHS, truth, sub)
            s = ledger.finalize(legacy_result_written=True)
            quota_bias.append(s["quota"]["rates"]["capture"] - truth[0])
            legacy_bias.append(s["legacy_window"]["rates"]["capture"] - truth[0])
        self.assertLess(abs(np.mean(quota_bias)), 0.006)
        self.assertGreater(np.mean(legacy_bias), 0.02, "the count-based stop over-counts short episodes")

    def test_invalid_partition_and_missing_aggregate_are_not_complete(self):
        ledger = EpisodeLedger(2, 1, self.dir / "e.jsonl", self.dir / "s.json", {"cell_id": "c"}, 2)
        both = torch.tensor([True, True])
        ledger.record(both, both, torch.tensor([1, 0]), torch.tensor([False, True]), torch.ones(2))
        self.assertEqual(ledger.finalize(True)["status"], "INVALID_OUTCOME_PARTITION")
        ledger = EpisodeLedger(2, 1, self.dir / "f.jsonl", self.dir / "t.json", {"cell_id": "c"}, 2)
        ledger.record(both, both, torch.zeros(2, dtype=torch.int32), torch.tensor([False, False]), torch.ones(2))
        self.assertEqual(ledger.finalize(False)["status"], "INVALID_LEGACY_RESULT_MISSING")


class NonInterferenceTest(unittest.TestCase):
    def test_inputs_are_not_mutated_and_no_random_state_is_consumed(self):
        with tempfile.TemporaryDirectory() as tmp:
            ledger = EpisodeLedger(4, 2, Path(tmp) / "e.jsonl", Path(tmp) / "s.json", {"cell_id": "c"}, 8)
            inputs = (torch.tensor([True, False, True, False]), torch.tensor([True, False, False, False]),
                      torch.tensor([0, 0, 1, 0], dtype=torch.int32), torch.tensor([False] * 4),
                      torch.tensor([0.4, 3.0, 1.2, 7.5]))
            before = [t.clone() for t in inputs]
            versions = [t._version for t in inputs]
            states = (torch.get_rng_state(), np.random.get_state()[1].copy(), random.getstate())
            ledger.record(*inputs)
            ledger.finalize(True)
            for tensor, copy, version in zip(inputs, before, versions):
                self.assertTrue(torch.equal(tensor, copy))
                self.assertEqual(tensor._version, version)
            self.assertTrue(torch.equal(states[0], torch.get_rng_state()))
            self.assertTrue(np.array_equal(states[1], np.random.get_state()[1]))
            self.assertEqual(states[2], random.getstate())

    def test_ledger_source_draws_no_randomness_and_touches_no_task_state(self):
        tree = ast.parse(LEDGER.read_text(encoding="utf-8"))
        names = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
        names |= {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
        for forbidden in ("rand", "randn", "randint", "normal", "bernoulli", "multinomial", "manual_seed",
                          "random", "obs_dict", "rewards", "terminations", "truncations", "reset_idx"):
            self.assertNotIn(forbidden, names, forbidden)

    def test_task_wiring_reads_outcomes_after_they_resolve_and_before_reset(self):
        source = TASK.read_text(encoding="utf-8")
        step = source[source.index("finished = (self.terminations > 0) | (self.truncations > 0)"):]
        record = step.index("self._episode_ledger.record(")
        self.assertLess(step.index("self._log_progress(successes, crashes, timeouts, finished)"), record)
        self.assertLess(record, step.index("self.reset_idx(reset_envs)"))
        self.assertIn("finished, successes, crashes, timeouts, self.ep_min_goal_dist", step[record:record + 200])
        self.assertIn("raise SystemExit(0)", step[record:record + 700])
        self.assertEqual(source.count("self._episode_ledger.record("), 1)
        self.assertEqual(source.count("build_episode_ledger("), 1)

    def test_player_cap_changes_only_the_stop_and_defaults_to_the_old_behaviour(self):
        runner = (ROOT / "aerial_gym/rl_training/rl_games/runner.py").read_text(encoding="utf-8")
        self.assertIn('os.environ.get("NAVRL_PLAYER_GAMES_CAP") or os.environ.get("PLAY_GAMES_NUM", "64")',
                      runner)
        self.assertEqual(runner.count("NAVRL_PLAYER_GAMES_CAP"), 2)  # the comment and the read


class ConfigurationTest(unittest.TestCase):
    ENV = {"NAVRL_EPISODE_LEDGER": "1", "NAVRL_EVAL_CHECKPOINT": "ckpt", "NAVRL_EPISODE_QUOTA_PER_ENV": "16",
           "NAVRL_PLAYER_GAMES_CAP": "1000000000", "NAVRL_TARGET_BEHAVIOR_LEVEL": "e2_obstacle_aware",
           "NAVRL_NUM_BARS": "115", "NAVRL_SEED": "4104"}

    def test_off_by_default(self):
        self.assertIsNone(build_episode_ledger(128, True, "/x/result.json", 2048, environ={}))

    def test_refuses_incomplete_configurations(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = str(Path(tmp) / "E2_obstacle_aware__115bars__seed4104" / "result.json")
            for key, value in (("NAVRL_EVAL_CHECKPOINT", ""), ("NAVRL_EPISODE_QUOTA_PER_ENV", "15"),
                               ("NAVRL_EPISODE_QUOTA_PER_ENV", ""), ("NAVRL_PLAYER_GAMES_CAP", "4096")):
                with self.assertRaises(RuntimeError, msg=key):
                    build_episode_ledger(128, True, out, 2048, environ=dict(self.ENV, **{key: value}))
            with self.assertRaises(RuntimeError):
                build_episode_ledger(128, False, out, 2048, environ=self.ENV)

    def test_valid_configuration_writes_next_to_the_cell_result(self):
        with tempfile.TemporaryDirectory() as tmp:
            cell = Path(tmp) / "E2_obstacle_aware__115bars__seed4104"
            ledger = build_episode_ledger(128, True, str(cell / "result.json"), 2048, environ=self.ENV)
            self.assertEqual(ledger.quota, 16)
            self.assertEqual(ledger.rows_path, cell / ledger_module.ROWS_NAME)
            self.assertEqual(ledger.context, {"cell_id": cell.name, "arm": "e2_obstacle_aware", "bars": 115,
                                              "seed": 4104})
            ledger.finalize(False)


if __name__ == "__main__":
    unittest.main()
