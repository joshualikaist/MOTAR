"""CPU-only checks of the SL1D-C02 runner contract and its independent analysis."""
import ast
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
for extra in ("tools", "tools/audits"):
    if str(ROOT / extra) not in sys.path:
        sys.path.insert(0, str(ROOT / extra))
import s_l1d_c02_run as runner  # noqa: E402

PLAN = runner.load_plan()


class ContractTest(unittest.TestCase):
    def test_sensor_only_matrix(self):
        specs = runner.process_specs(PLAN)
        self.assertEqual([s["id"] for s in specs], ["sensors_s_l1c", "sensors_s_l1d"])
        self.assertEqual([s["lineage"] for s in specs], ["s_l1c_v1", "s_l1d_v1"])
        self.assertTrue(all(s["num_envs"] == 13 for s in specs))
        self.assertEqual(PLAN["run"]["run_id"], "SL1D-C02")

    def test_preflight_refuses_changed_production_or_uncommitted_inputs(self):
        def git(*args):
            if args[0] == "diff":
                return SimpleNamespace(returncode=1, stdout="")
            return SimpleNamespace(returncode=0, stdout="")
        with tempfile.TemporaryDirectory() as temp:
            with mock.patch.object(runner.base, "preflight", return_value=([], {"policy_f_sha256": PLAN["policy_f_sha256"]})), \
                    mock.patch.object(runner.a, "committed", return_value=False), \
                    mock.patch.object(runner.base, "git", side_effect=git):
                problems, checks = runner.preflight(PLAN, "a" * 40, Path(temp) / "new", False)
        self.assertTrue(any("not committed" in x for x in problems))
        self.assertTrue(any("76f0f4c" in x for x in problems))
        self.assertFalse(checks["production_identical_to_76f0f4c"])


class AnalysisContractTest(unittest.TestCase):
    def test_analysis_imports_no_production_code(self):
        tree = ast.parse((ROOT / "tools/s_l1d_c02_analysis.py").read_text())
        names = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        names |= {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
        self.assertFalse(any("aerial_gym" in x or "torch" in x or "isaacgym" in x for x in names))

    def test_scale_rule_precedes_mapping_in_analysis(self):
        source = (ROOT / "tools/s_l1d_c02_analysis.py").read_text()
        self.assertIn("if alone < min_px:\n            allowed = ('PRED_SCALE_LOSS',)", source)


if __name__ == "__main__":
    unittest.main()
