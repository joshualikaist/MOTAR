"""CPU contracts for the recovery-v2 NO_CONNECTOR geometry preregistration."""

import importlib.util
import math
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "tools/diagnose_navrl_physical_target_recovery_v2_no_connector.py"
SPEC = importlib.util.spec_from_file_location("recovery_v2_no_connector_forensics", PATH)
MOD = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MOD
assert SPEC.loader is not None
SPEC.loader.exec_module(MOD)


class NoConnectorForensicsContractTest(unittest.TestCase):
    def test_frozen_probe_is_70bar_recovery_v2_lower_speeds_only(self):
        self.assertEqual(MOD.DENSITIES, (70,))
        self.assertEqual(MOD.SPEEDS, (0.6, 0.9, 1.2, 1.25))
        self.assertEqual(MOD.ENVS, 32)
        self.assertEqual(MOD.SEED, 827)
        self.assertEqual(MOD.ROUTE_MODE, "global_astar_recovery_v2")
        self.assertEqual(MOD.CONTRACT_VARIANT, "baseline_1p25")
        self.assertEqual(MOD.VEL_KP, 2.5)
        self.assertEqual(MOD.TRACKING_MARGIN_M, 0.45)
        self.assertEqual(MOD.ANCHOR_RADIUS_CELLS, 3)
        self.assertEqual(MOD.SOFT_HYSTERESIS_M, 0.25)
        self.assertAlmostEqual(MOD.RECOVERY_HARD_EPSILON_M, 1e-4 + 0.0123)
        contract = MOD.probe_contract()
        self.assertEqual(contract["runtime_wall_margin_m"], 0.50)
        self.assertEqual(contract["route_boundary_margin_m"], 1.25)
        self.assertEqual(contract["boundary_soft_minus_hard_m"], 0.75)
        self.assertTrue(contract["gate_artifacts_read_only"])
        self.assertFalse(contract.get("passes_32_cell_mechanism", False))
        self.assertEqual(
            contract["output_root"],
            "results/navrl_physical_target_recovery_v2_no_connector_forensics_seed827",
        )
        source = PATH.read_text(encoding="utf-8")
        self.assertIn('"NAVRL_NUM_BARS": "70"', source)
        self.assertIn('"NAVRL_MAX_BARS": "300"', source)
        self.assertIn('"NAVRL_TARGET_ROUTE_MODE": ROUTE_MODE', source)
        self.assertNotIn("1.5", "".join(str(s) for s in MOD.SPEEDS))

    def test_recovery_v2_anchor_kwargs_are_not_the_v1_forensic_search(self):
        recovery = MOD.recovery_v2_anchor_kwargs()
        legacy = MOD.v1_forensic_anchor_kwargs()
        self.assertEqual(recovery["soft_hysteresis_m"], 0.25)
        self.assertEqual(legacy["soft_hysteresis_m"], 0.0)
        self.assertGreater(recovery["hard_epsilon_m"], legacy["hard_epsilon_m"])
        self.assertAlmostEqual(recovery["hard_epsilon_m"], 0.0124)

    def test_replica_matches_planner_recovery_anchor_and_can_disagree_with_v1(self):
        point = [20.0, 20.0]
        bars = np.zeros((0, 2), dtype=np.float64)
        half = np.zeros((0, 2), dtype=np.float64)
        lo, hi = [0.0, 0.0], [40.0, 40.0]
        support = [0.2068816086567407, 0.2068816086567407]
        replica = MOD.recovery_anchor_query(point, bars, half, lo, hi, support)
        planner = MOD._load_planner()
        direct = planner.nearest_soft_free_anchor(
            point, bars, half, lo, hi, support, 0.50, 1.25,
            **MOD.recovery_v2_anchor_kwargs(),
        )
        self.assertEqual(replica["exists"], True)
        self.assertEqual(replica["hard_connector_safe"], True)
        self.assertEqual(replica["cell_ij"], direct["cell_ij"])
        self.assertEqual(MOD.recovery_anchor_query(
            point, bars, half, lo, hi, support, variant="v1"
        )["exists"], True)

        boxed_point = [1.0, 1.0]
        recovery = MOD.recovery_anchor_query(
            boxed_point, bars, half, lo, hi, [0.20, 0.20]
        )
        legacy = MOD.recovery_anchor_query(
            boxed_point, bars, half, lo, hi, [0.20, 0.20], variant="v1"
        )
        self.assertEqual(legacy.get("exists"), True)
        self.assertEqual(legacy.get("hard_connector_safe"), True)
        self.assertEqual(recovery.get("exists"), False)
        self.assertEqual(recovery.get("hard_connector_safe"), False)

    def test_descriptive_verdict_cannot_pass_the_32cell_mechanism(self):
        present = MOD.descriptive_verdict(18, 20)
        self.assertEqual(present["label"], "ANCHOR_PRESENT_LATCH")
        self.assertFalse(present["passes_32_cell_mechanism"])
        self.assertFalse(present["authorizes_retune_or_ppo"])
        absent = MOD.descriptive_verdict(2, 20)
        self.assertEqual(absent["label"], "ANCHOR_ABSENT_AT_LATCH")
        small = MOD.descriptive_verdict(19, 19)
        self.assertEqual(small["label"], "INCONCLUSIVE")
        mixed = MOD.descriptive_verdict(10, 20)
        self.assertEqual(mixed["label"], "INCONCLUSIVE")

    def test_wilson_interval_is_the_v1_formula(self):
        lower, upper = MOD.wilson_interval(18, 20)
        z = 1.96
        p = 18.0 / 20.0
        denominator = 1.0 + z * z / 20.0
        center = p + z * z / 40.0
        spread = z * math.sqrt(p * (1.0 - p) / 20.0 + z * z / (4.0 * 20.0 * 20.0))
        self.assertAlmostEqual(lower, (center - spread) / denominator)
        self.assertAlmostEqual(upper, (center + spread) / denominator)
        self.assertGreater(lower, 0.5)
        self.assertIsNone(MOD.wilson_interval(0, 0))


if __name__ == "__main__":
    unittest.main()
