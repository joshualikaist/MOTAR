"""KROC27-A-M02 (amendment 2, CPU): lineage choice, the 56-cell matrix, refusal, manifest check."""
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
    spec = importlib.util.spec_from_file_location("_kroc2027_cli_mt_main", ROOT / "tools/kroc2027_eval.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


cli = _load_cli()
PLAN = json.loads((ROOT / cli.MT_PLAN).read_text())
AMENDMENT = json.loads((ROOT / cli.A_AMENDMENT2).read_text())


def _report(failing=()):
    cells = {c["cell_dir"]: {"issues": ["x"] if c["cell_dir"] in failing else []}
             for c in cli.mt_canary_cells(PLAN)}
    return {"cells": cells}


class LineageChoiceTest(unittest.TestCase):
    def test_section_5_order_on_the_new_lineage(self):
        order = AMENDMENT["moving_candidates"]
        self.assertEqual(order, ["E2_direct_l2_v1", "E1_direct_l2_v1"])
        self.assertEqual(cli.select_mt_lineage(_report(), PLAN, order), "E2_direct_l2_v1")
        e2_cell = next(c["cell_dir"] for c in cli.mt_canary_cells(PLAN) if c["lineage"] == "E2_direct_l2_v1")
        self.assertEqual(cli.select_mt_lineage(_report({e2_cell}), PLAN, order), "E1_direct_l2_v1")
        both = {c["cell_dir"] for c in cli.mt_canary_cells(PLAN) if c["density"] == 160}
        self.assertIsNone(cli.select_mt_lineage(_report(both), PLAN, order))

    def test_a_lineage_with_missing_cells_is_not_chosen(self):
        report = _report()
        for key in [k for k in report["cells"] if k.startswith("E2_direct_l2_v1/A4")]:
            del report["cells"][key]
        self.assertEqual(cli.select_mt_lineage(report, PLAN, AMENDMENT["moving_candidates"]), "E1_direct_l2_v1")


class MainMatrixTest(unittest.TestCase):
    def test_cells(self):
        cells = cli.mt_main_cells(AMENDMENT, PLAN, "E2_direct_l2_v1")
        self.assertEqual(len(cells), 56)
        self.assertEqual({c["seed"] for c in cells}, set(range(4201, 4208)))
        self.assertEqual({(c["arm"], c["view"]) for c in cells}, {("A3", "visible"), ("A4", "hidden")})
        self.assertEqual({c["target_model"] for c in cells}, {"E2_obstacle_aware"})
        self.assertEqual([c["seed"] for c in cells[:8]], [4201] * 8)  # seed-major
        self.assertEqual(AMENDMENT["quota_per_env"], 16)

    def test_refuses_without_approval(self):
        out_root = ROOT / AMENDMENT["output_root"]
        existed = out_root.exists()
        args = SimpleNamespace(stage="mt_main", run_id=AMENDMENT["run_id"], approved_commit="", dry_run=False,
                               max_gpu_hours=None, canary_root=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            rc = cli.cmd_run(args)
        self.assertEqual(rc, 2)
        self.assertTrue(any("approved-commit" in p for p in json.loads(out.getvalue())["problems"]))
        self.assertEqual(out_root.exists(), existed)

    def test_cap_above_the_amendment_is_refused(self):
        args = SimpleNamespace(stage="mt_main", run_id=AMENDMENT["run_id"], approved_commit="", dry_run=True,
                               max_gpu_hours=AMENDMENT["max_gpu_hours"] + 1, canary_root=None)
        with contextlib.redirect_stdout(io.StringIO()) as out:
            cli.cmd_run(args)
        self.assertTrue(any("max-gpu-hours" in p for p in json.loads(out.getvalue())["problems"]))


class ManifestTest(unittest.TestCase):
    def test_changed_raw_file_is_caught(self):
        root = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, root)
        cell = root / "A1" / "E0_static__70bars__seed4201"
        cell.mkdir(parents=True)
        (cell / "result.json").write_text("{}")
        manifest = {"files": {"A1/E0_static__70bars__seed4201/result.json": cli.kp.sha256_file(cell / "result.json")}}
        self.assertEqual(cli.manifest_problems(cell, root, manifest), [])
        (cell / "result.json").write_text("{ }")
        self.assertTrue(cli.manifest_problems(cell, root, manifest))
        self.assertTrue(cli.manifest_problems(root / "A2" / "x", root, manifest))


if __name__ == "__main__":
    unittest.main()
