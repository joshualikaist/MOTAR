import json
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools/verify_navrl_corrected_nonoverlap_route_gate.py"
PREREG = ROOT / "docs/preregistration_corrected_nonoverlap_route_gate_2026-08-31.md"


def probe(module: str, expression: str) -> object:
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            f"import json; import {module} as m; print(json.dumps({expression}, sort_keys=True))",
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if completed.returncode != 0:
        raise AssertionError(completed.stderr)
    return json.loads(completed.stdout)


class CorrectedNonOverlapRouteGateTest(unittest.TestCase):
    def test_historical_defaults_remain_unchanged(self):
        values = probe(
            "tools.verify_navrl_physical_target_routed_simulator_gate",
            "{'robot': m.frozen_environment('off', 0.6)['NAVRL_ROBOT'], "
            "'densities': m.DENSITIES, "
            "'placement': m.frozen_environment('off', 0.6)['NAVRL_PLACEMENT_MODE']}",
        )
        self.assertEqual(values["robot"], "navrl_ref5in_quad")
        self.assertEqual(values["densities"], [70, 150, 205, 300])
        self.assertEqual(values["placement"], "navrl_band")

    def test_corrected_contract_is_complete_and_separate(self):
        values = probe(
            "tools.verify_navrl_corrected_nonoverlap_route_gate",
            "{'densities': m.DENSITIES, 'seed': m.SEED, "
            "'env': m.frozen_environment('global_astar_v1', 1.5)}",
        )
        env = values["env"]
        self.assertEqual(values["densities"], [70, 115, 160, 205])
        self.assertEqual(values["seed"], 829)
        self.assertEqual(env["NAVRL_ROBOT"], "navrl_ref5in_v2_quad")
        self.assertEqual(env["NAVRL_PLACEMENT_MODE"], "footprint_clearance")
        self.assertEqual(env["NAVRL_PLACEMENT_SURFACE_CLEARANCE_M"], "0.45")
        self.assertEqual(env["NAVRL_MAX_BARS"], "300")
        self.assertEqual(env["NAVRL_PHYSICAL_GEOMETRY_VERSION"], "v2")
        self.assertEqual(env["NAVRL_TARGET_BOX_XY_M"], "0.283")
        self.assertNotIn("NAVRL_PLACEMENT_TOUCH_M", env)
        self.assertNotIn("NAVRL_PLACEMENT_GAP_M", env)

    def test_preregistration_and_hermetic_child_environment(self):
        self.assertTrue(TOOL.is_file())
        text = PREREG.read_text(encoding="utf-8")
        self.assertIn("70/115/160/205", text)
        self.assertIn("PASS_FULL_1P5_CONTRACT", text)
        values = probe(
            "tools.verify_navrl_corrected_nonoverlap_route_gate",
            "{'stale_survives': 'NAVRL_PLACEMENT_MODE' in "
            "m.build_child_environment({'PATH': __import__('os').environ['PATH'], "
            "'NAVRL_PLACEMENT_MODE': 'navrl_band'}, __import__('sys').executable)}",
        )
        self.assertFalse(values["stale_survives"])


if __name__ == "__main__":
    unittest.main()
