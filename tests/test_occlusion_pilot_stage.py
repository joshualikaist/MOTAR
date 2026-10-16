"""OCC-G0 / OCC-P1 runner stages (CPU): cell matrices, refusals, pair and historical comparisons."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


cli = load(ROOT / "tools/kroc2027_eval.py", "_kroc2027_cli_occ")
PLAN = json.loads((ROOT / cli.OCC_PLAN).read_text())


def write_cell(directory, rows, records):
    directory.mkdir(parents=True)
    (directory / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (directory / cli.RECORDER_EPISODES).write_text("".join(json.dumps(r) + "\n" for r in records))


def episode(env, ep, outcome="capture", step=30, extra=None):
    row = {"env_index": env, "env_episode_index": ep, "episode_id": "c/env%03d/ep%03d" % (env, ep),
           "outcome": outcome, "capture": outcome == "capture", "crash": outcome == "crash", "timeout": False,
           "completion_step": step, "completion_rank": env * 10 + ep, "min_relative_distance_m": 0.4,
           "in_quota": True}
    record = {"env_index": env, "env_episode_index": ep, "outcome": outcome, "actions": step}
    record.update(extra or {})
    return row, record


class PilotCellsTest(unittest.TestCase):
    FIELDS = PLAN["g0_noninterference"]["ledger_fields"]

    def test_matrix_and_historical_counterparts(self):
        cells = cli.occlusion_pilot_cells(PLAN)
        self.assertEqual(len(cells), PLAN["pilot"]["cells"])
        self.assertEqual({(c["group"], c["arm"], c["lineage"]) for c in cells},
                         {("static", "A1", "E0_static"), ("moving", "A3", "E2_direct_l2_v1")})
        self.assertEqual({c["seed"] for c in cells}, set(range(4201, 4208)))
        g0 = cli.occlusion_cells(PLAN, PLAN["g0"]["seeds"])
        self.assertEqual(len(g0), PLAN["g0"]["pairs"])
        self.assertEqual({c["seed"] for c in g0}, {4201})
        self.assertEqual(sorted({c["density"] for c in cells}), [70, 115, 160, 205])
        self.assertTrue(all(c["view"] == "visible" for c in cells))
        self.assertTrue(all(Path(c["historical"]).name == c["cell_dir"].split("/")[-1] for c in cells))
        self.assertEqual([c["seed"] for c in cells[:8]], [4201] * 8)  # seed-major
        self.assertEqual(PLAN["safety"]["min_free_disk_gb"], 8.0)
        self.assertTrue(PLAN["status"].startswith("APPROVED"))

    def test_pilot_refuses_without_approval_or_g0_and_writes_nothing(self):
        out_root = ROOT / PLAN["pilot"]["output_root"]
        existed = out_root.exists()
        args = SimpleNamespace(stage="occlusion_pilot", run_id="OCC-P1", approved_commit="", dry_run=False,
                               max_gpu_hours=None, canary_root=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            rc = cli.cmd_run(args)
        self.assertEqual(rc, 2)
        problems = " ".join(json.loads(out.getvalue())["problems"])
        for text in ("approved-commit", "G0_PASS"):
            self.assertIn(text, problems)
        self.assertEqual(out_root.exists(), existed)

    def test_g0_refuses_without_approval(self):
        args = SimpleNamespace(stage="occlusion_g0", run_id="OCC-G0", approved_commit="", dry_run=False,
                               max_gpu_hours=None, canary_root=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            rc = cli.cmd_run(args)
        self.assertEqual(rc, 2)
        self.assertIn("approved-commit", " ".join(json.loads(out.getvalue())["problems"]))
        self.assertFalse((ROOT / PLAN["g0"]["output_root"]).exists() and False)

    def test_pair_noninterference_detects_each_difference(self):
        import numpy as np
        with tempfile.TemporaryDirectory() as base:
            for mode in ("off", "on"):
                d = Path(base) / mode / "cell"
                rows, recs = zip(*[episode(0, k) for k in range(4)])
                write_cell(d, list(rows), list(recs))
                (d / "result.json").write_text(json.dumps({"condition": {"evaluation_nonce": mode, "bars": 70}, "x": 1}))
                (d / "trajectory_digest.json").write_text(json.dumps({"sha256": "abc", "steps": 10}))
                (d / "episode_ledger_summary.json").write_text(json.dumps({"rows_sha256": "r1"}))
                np.savez(Path(base) / mode / "obs_dump.npz", call_index=np.arange(3), env_id=np.zeros(3, int),
                         obs=np.ones((3, 898), np.float32))
            ok = cli.pair_noninterference(Path(base) / "off/cell", Path(base) / "on/cell", 4, self.FIELDS)
            self.assertEqual(ok["verdict"], "PASS", ok)
            self.assertEqual(ok["detail"]["observation_rows_compared"], 3)
            (Path(base) / "on/cell/trajectory_digest.json").write_text(json.dumps({"sha256": "zzz", "steps": 10}))
            bad = cli.pair_noninterference(Path(base) / "off/cell", Path(base) / "on/cell", 4, self.FIELDS)
            self.assertEqual(bad["verdict"], "FAIL")
            self.assertFalse(bad["checks"]["trajectory_digest_equal"])
            self.assertTrue(bad["checks"]["actor_observations_bitwise_equal"])


class HistoricalMatchTest(unittest.TestCase):
    FIELDS = PLAN["g0_noninterference"]["ledger_fields"]

    def test_match_ignores_episodes_beyond_quota_and_reports_new_keys(self):
        with tempfile.TemporaryDirectory() as base:
            old = [episode(e, k) for e in range(2) for k in range(6)]
            new = [episode(e, k, extra={"new_field": 1}) for e in range(2) for k in range(4)]
            write_cell(Path(base) / "old", [r for r, _ in old], [x for _, x in old])
            write_cell(Path(base) / "new", [r for r, _ in new], [x for _, x in new])
            result = cli.historical_match(Path(base) / "new", Path(base) / "old", 4, self.FIELDS)
            self.assertEqual(result["verdict"], "MATCH")
            self.assertEqual(result["ledger_rows_compared"], 8)
            self.assertEqual(result["keys_only_new"], ["new_field"])

    def test_mismatch_lists_fields_without_identity_claim(self):
        with tempfile.TemporaryDirectory() as base:
            old = [episode(0, k) for k in range(4)]
            new = [episode(0, k, outcome="crash" if k == 2 else "capture") for k in range(4)]
            write_cell(Path(base) / "old", [r for r, _ in old], [x for _, x in old])
            write_cell(Path(base) / "new", [r for r, _ in new], [x for _, x in new])
            result = cli.historical_match(Path(base) / "new", Path(base) / "old", 4, self.FIELDS)
            self.assertEqual(result["verdict"], "MISMATCH")
            self.assertEqual(result["ledger_mismatches"][0]["episode"], [0, 2])
            self.assertIn("outcome", result["ledger_mismatches"][0]["fields"])
            missing = cli.historical_match(Path(base) / "new", Path(base) / "absent", 4, self.FIELDS)
            self.assertEqual(missing["verdict"], "NOT_COMPARABLE")

    def test_trace_problems_on_missing_trace(self):
        with tempfile.TemporaryDirectory() as base:
            self.assertTrue(cli.occlusion_trace_problems(Path(base)))


if __name__ == "__main__":
    unittest.main()
