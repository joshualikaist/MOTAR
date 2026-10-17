"""EQ-1B plant and scene (CPU): motor integrator, URDF, passability certificates, braking law.

Production verdicts asserted here include FAILs on purpose (preserved counterexamples). The
recorded-data results are checked against the committed evidence records, so the suite does not
need the raw OCC-P2 tree; the fixtures below exercise the same certificate code on synthetic scenes.
"""
import json
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/audits"))
import eq1_flight_envelope as env  # noqa: E402
import eq1_motor_reference as motor  # noqa: E402
import eq1_scene_feasibility as scene  # noqa: E402

EVIDENCE = ROOT / "docs/evidence/eq1_validation_2026-10-18"


class MotorTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = motor.run()

    def test_named_method_is_not_classical_rk4(self):
        m1 = self.r["M1_named_method_algebra"]
        self.assertEqual(m1["production"]["status"], "FAIL")
        row = next(x for x in m1["rows"] if x["dt"] == 0.01 and x["tau"] == 0.04)
        self.assertAlmostEqual(row["production_gain"], 0.2214, places=12)
        self.assertAlmostEqual(row["classical_rk4_same_ode"], 0.18126666666666666, places=12)
        self.assertAlmostEqual(row["exact_intended_tau"], 1 - math.exp(-0.25), places=12)
        self.assertTrue(all(abs(x["production_gain"] - x["production_closed_form"]) < 1e-12 for x in m1["rows"]))

    def test_convergence_order_three_not_four(self):
        m2 = self.r["M2_convergence"]
        self.assertEqual(m2["production"]["status"], "FAIL")
        self.assertTrue(all(2.5 < o < 3.5 for o in m2["observed_orders"]))
        self.assertTrue(all(o > 3.9 for o in m2["classical_rk4_observed_orders"]))
        self.assertEqual(m2["proposed_correction"]["status"], "PASS")

    def test_fixed_timestep_preservation(self):
        c = self.r["M3_fixed_timestep_preservation"]["candidates"]
        self.assertEqual(c["zoh_exact_nominal_tau"]["status"], "PASS")
        self.assertEqual(c["classical_rk4_continuous_coefficient"]["status"], "PASS")
        self.assertEqual(c["naive_sign_flip_keep_coefficient"]["status"], "FAIL")
        self.assertAlmostEqual(c["naive_sign_flip_keep_coefficient"]["effective_tau_s"], 0.05, places=4)
        self.assertEqual(c["backward_euler_discrete_coefficient"]["status"], "FAIL")
        self.assertTrue(self.r["k_T_cancellation"]["equal"])
        self.assertEqual(self.r["M4_rate_limited"]["production"]["status"], "PASS")
        self.assertFalse(self.r["config"]["rate_limit_active_in_policy_f"])


class UrdfTest(unittest.TestCase):
    def test_collision_versus_visual(self):
        c = scene.case_urdf()
        self.assertEqual(c["robot"]["visual_inside_collision_3d"]["status"], "FAIL")
        self.assertEqual(c["robot"]["visual_inside_collision_xy"]["status"], "PASS")
        sphere = next(v for v in c["robot"]["visuals"] if v["type"] == "sphere")
        self.assertAlmostEqual(sphere["outside_collision_m"], 0.01, places=9)
        self.assertEqual(c["robot"]["rotor_envelope"]["status"], "NOT_TESTED")
        self.assertEqual(c["bars"]["assets"], 40)
        self.assertEqual(c["bars"]["all_visual_equal_collision"]["status"], "PASS")
        self.assertAlmostEqual(scene.R_CONS, math.sqrt(0.14 ** 2 * 2 + 0.04 ** 2), places=12)


class CertificateFixtureTest(unittest.TestCase):
    """Synthetic scenes with known answers for the passability certificates."""

    def wall(self, gap_lo=None, gap_hi=None):
        centres, halves = [], []
        y = -2.0
        while y < 42.0:
            lo, hi = y, y + 0.8
            if gap_lo is not None and hi > gap_lo and lo < gap_hi:
                if lo < gap_lo:
                    centres.append([20.0, (lo + gap_lo) / 2]); halves.append([0.2, (gap_lo - lo) / 2])
                if hi > gap_hi:
                    centres.append([20.0, (gap_hi + hi) / 2]); halves.append([0.2, (hi - gap_hi) / 2])
            else:
                centres.append([20.0, (lo + hi) / 2]); halves.append([0.2, 0.4])
            y = hi
        return np.array(centres), np.array(halves)

    def test_open_wall_and_gaps(self):
        start, target = np.array([10.0, 20.0]), np.array([30.0, 20.0])
        empty = (np.zeros((0, 2)), np.zeros((0, 2)))
        self.assertEqual(scene.classify(*empty, start, target, scene.R_CONS, scene.R_OPT)[0], "CONNECTED")
        self.assertEqual(scene.classify(*self.wall(), start, target, scene.R_CONS, scene.R_OPT)[0], "DISCONNECTED")
        wide = self.wall(19.0, 20.0)  # 1.0 m gap: passable for the conservative body
        self.assertEqual(scene.classify(*wide, start, target, scene.R_CONS, scene.R_OPT)[0], "CONNECTED")
        narrow = self.wall(19.0, 19.15)  # 0.15 m < 2 r_opt: impossible even for the optimistic body
        self.assertEqual(scene.classify(*narrow, start, target, scene.R_CONS, scene.R_OPT)[0], "DISCONNECTED")
        ambiguous = self.wall(19.0, 19.30)  # 2 r_opt < 0.30 m < 2 r_cons: neither certificate holds
        self.assertEqual(scene.classify(*ambiguous, start, target, scene.R_CONS, scene.R_OPT)[0], "UNRESOLVED")


class BrakingLawTest(unittest.TestCase):
    def test_ray_caster_against_marching(self):
        rng = np.random.default_rng(3)
        c, h = rng.uniform(0, 8, (12, 2)), rng.uniform(0.2, 0.4, (12, 2))
        p, v = rng.uniform(0, 8, (60, 2)), rng.normal(size=(60, 2))
        t = env.free_travel(p, v, c, h, 0.2)
        d = v / np.linalg.norm(v, axis=1, keepdims=True)
        s = np.arange(0, 20, 0.0005)
        for i in range(len(p)):
            pts = p[i] + s[:, None] * d[i]
            q = np.maximum(np.abs(pts[:, None, :] - c[None]) - h[None], 0)
            hit = np.nonzero(np.sqrt((q ** 2).sum(2)).min(1) <= 0.2)[0]
            expected = s[hit[0]] if len(hit) else np.inf
            if np.isinf(expected):
                self.assertTrue(np.isinf(t[i]))
            else:
                self.assertAlmostEqual(t[i], expected, delta=6e-4)

    def test_riskcap_counterexample_and_stopcap(self):
        g = env.case_governor()
        self.assertEqual(g["counterexample"]["executed_mps"], 2.0)
        self.assertAlmostEqual(g["counterexample"]["stopping_margin_executed_m"], -0.875474, places=5)
        self.assertAlmostEqual(g["counterexample"]["clearance_needed_at_2mps_m"], 1.325474, places=5)
        self.assertEqual(g["implementation_fidelity"]["status"], "PASS")
        self.assertEqual(g["riskcap_nonnegative_margin"]["status"], "FAIL")
        self.assertEqual(g["stopcap_nonnegative_margin"]["status"], "PASS")


@unittest.skipUnless((EVIDENCE / "eq1b_scene.json").exists(), "EQ-1B evidence not present")
class RecordedEvidenceTest(unittest.TestCase):
    def test_scene_record(self):
        r = json.loads((EVIDENCE / "eq1b_scene.json").read_text())
        p = r["cases"][1]
        self.assertEqual((p["cells"], p["quota_episodes"]), (112, 57344))
        self.assertEqual(p["all_scenes_certified_connected"]["status"], "PASS")
        self.assertEqual(p["captures_in_certified_disconnected"], [])
        self.assertEqual(p["recorded_half_extents_not_in_bar_assets"], [])
        self.assertEqual(p["dynamic_feasibility"]["status"], "NOT_TESTED")
        self.assertTrue(r["source"]["identical_to_measurement_source"])

    def test_envelope_record(self):
        r = json.loads((EVIDENCE / "eq1b_envelope.json").read_text())
        e = r["cases"][1]
        self.assertEqual(e["cells"], 112)
        self.assertEqual(e["tilt_limit_in_normal_flight"]["status"], "FAIL")
        self.assertEqual(e["contact_model_consistency"]["status"], "PASS")
        last = e["by_group_density"]["all"]["pre_crash_last"]
        self.assertEqual(last["counts"]["frames"], 1898)  # one per quota crash
        self.assertGreater(last["fractions"]["margin_gov_negative"], 0.9)


if __name__ == "__main__":
    unittest.main()
