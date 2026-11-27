"""MT-P2-C01 runner stage (CPU): the 16-cell matrix, refusal before approval and the identity check."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load_cli():
    spec = importlib.util.spec_from_file_location("_kroc2027_cli_mt", ROOT / "tools/kroc2027_eval.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


cli = _load_cli()
PLAN = json.loads((ROOT / cli.MT_PLAN).read_text())


class MtCanaryCellsTest(unittest.TestCase):
    def test_matrix(self):
        cells = cli.mt_canary_cells(PLAN)
        self.assertEqual(len(cells), PLAN["future_run"]["cells"])
        self.assertEqual(len({c["cell_dir"] for c in cells}), 16)
        self.assertEqual({c["seed"] for c in cells}, {4190})
        self.assertEqual({(c["arm"], c["view"]) for c in cells}, {("A3", "visible"), ("A4", "hidden")})
        self.assertEqual({(c["lineage"], c["target_model"]) for c in cells},
                         {("E1_direct_l2_v1", "E1_cv"), ("E2_direct_l2_v1", "E2_obstacle_aware")})
        self.assertEqual(sorted({c["density"] for c in cells}), [70, 115, 160, 205])
        self.assertIn("E1_direct_l2_v1/A3/E1_cv__70bars__seed4190", [c["cell_dir"] for c in cells])

    def test_backend_is_scrubbed_from_inherited_env(self):
        env = cli.base_env(inherited={"NAVRL_TARGET_DISTANCE_BACKEND": "direct_l2_v1", "HOME": "/tmp"})
        self.assertNotIn("NAVRL_TARGET_DISTANCE_BACKEND", env)

    def test_refuses_without_approval_and_writes_nothing(self):
        out_root = ROOT / PLAN["future_run"]["output_root"]
        existed = out_root.exists()
        args = SimpleNamespace(stage="mt_canary", run_id="MT-P2-C01", approved_commit="", dry_run=False,
                               max_gpu_hours=None, canary_root=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            rc = cli.cmd_run(args)
        self.assertEqual(rc, 2)
        report = json.loads(out.getvalue())
        self.assertEqual(report["status"], "REFUSED")
        self.assertTrue(any("approved-commit" in p for p in report["problems"]))
        self.assertEqual(out_root.exists(), existed)

    def test_refuses_a_cap_above_the_plan(self):
        args = SimpleNamespace(stage="mt_canary", run_id="MT-P2-C01", approved_commit="", dry_run=True,
                               max_gpu_hours=2.5, canary_root=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            cli.cmd_run(args)
        self.assertTrue(any("max-gpu-hours" in p for p in json.loads(out.getvalue())["problems"]))


class MtIdentityTest(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.dir)

    def _write(self, backend="direct_l2_v1", model=cli.MT_MODEL, tf32=True):
        env = {"NAVRL_TARGET_DISTANCE_BACKEND": backend, "NAVRL_KROC2027_PACKET_SCHEMA": "full_v2",
               "NAVRL_KROC2027_PACKETS": "1", "NAVRL_KROC2027_ON_VIOLATION": "stop"}
        (self.dir / "receipt.json").write_text(json.dumps({"env_snapshot_navrl": env}))
        (self.dir / "result.json").write_text(json.dumps({"condition": {"target_motion_model": model}}))
        (self.dir / cli.RECORDER_SUMMARY).write_text(json.dumps({
            "context": {"cfg_target_distance_backend": backend, "target_motion_model_id": model},
            "matmul_precision": {"matmul_allow_tf32": tf32, "float32_matmul_precision": "high"},
            "violation_packets": {"schema": "full_v2"}}))

    def test_correct_cell_passes(self):
        self._write()
        self.assertEqual(cli.mt_identity_problems(self.dir, PLAN), [])

    def test_wrong_backend_model_or_precision_is_caught(self):
        self._write(backend="legacy_cdist", model="bounded_planar_drone_v1_rollout", tf32=False)
        problems = " ".join(cli.mt_identity_problems(self.dir, PLAN))
        for word in ("NAVRL_TARGET_DISTANCE_BACKEND", "target_motion_model", "recorder backend",
                     "matmul_allow_tf32"):
            self.assertIn(word, problems)

    def test_missing_files_are_problems(self):
        self.assertTrue(cli.mt_identity_problems(self.dir, PLAN))
        self.assertEqual(cli.mt_cell_evidence(self.dir)["bundle_files"], [])


if __name__ == "__main__":
    unittest.main()
