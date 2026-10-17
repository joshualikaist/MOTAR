"""EQ-1D (CPU parts): plan integrity, telemetry, perception snapshot/restore, analysis references, orchestrator.

The GPU processes themselves run only under ``tools/eq1d_run.py``; these tests cover everything that
can be checked without Isaac Gym, including that the analysis gates evaluate correctly on synthetic
records with known answers.
"""
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "tools/audits"))

from motar_eval import eq1d_telemetry as tel  # noqa: E402
import eq1d_analysis as ana  # noqa: E402
import eq1d_run as runner  # noqa: E402

PLAN = tel.load_plan()


class PlanTest(unittest.TestCase):
    def test_matrix(self):
        self.assertEqual(len(PLAN["D1"]["fixtures"]), 24)
        self.assertEqual(len(PLAN["D2"]["scenes"]), 32)
        self.assertEqual(len(PLAN["D2"]["attitudes_deg"]), 3)
        ids = [p["id"] for p in PLAN["run"]["processes"]]
        self.assertEqual(ids, ["d1_dt10", "d1_dt5", "d1_dt2p5", "d2_scenes", "d3_n4", "d3_n1", "d3_n2", "d3_n128"])
        for p in PLAN["run"]["processes"]:
            if p["batch"] == "D1":
                self.assertAlmostEqual(p["dt_s"] * p["substeps_per_action"], 0.1, places=12)
        self.assertEqual(PLAN["run"]["disk_min_free_gb"], 8.0)
        self.assertTrue(PLAN["status"].startswith("APPROVED"))
        self.assertEqual(sorted(PLAN["D1"]["gates"]), sorted(["D1-T1", "D1-M1", "D1-M2", "D1-C1", "D1-H1", "D1-V1",
                                                               "D1-B1", "D1-A1", "D1-K1", "D1-K2"]))


class TelemetryTest(unittest.TestCase):
    def test_d1_commands(self):
        f = {fx["id"]: fx for fx in PLAN["D1"]["fixtures"]}
        self.assertEqual(tel.d1_command(f[5], 0.0), [0, 0, 0, 0])
        self.assertEqual(tel.d1_command(f[5], 0.5), [2.5, 0, 0, 0])
        self.assertEqual(tel.d1_command(f[5], 2.5), [-2.5, 0, 0, 0])
        self.assertEqual(tel.d1_command(f[5], 4.6), [0, 0, 0, 0])
        self.assertAlmostEqual(tel.d1_command(f[16], 1.0)[0], 2.0 * math.sin(2 * math.pi * 0.5 * 0.5), places=12)
        c = tel.d1_command(f[22], 1.0)
        self.assertAlmostEqual(c[0], 3.0 * math.cos(math.radians(30)), places=12)
        self.assertEqual(tel.d1_command(f[22], 0.4), [0.0, 0.0, 0.0, 0.0])

    def test_recorder_and_tap(self):
        state = {"x": torch.zeros(2, 3)}

        class Motor:
            def update_motor_thrusts(self, ref):
                return ref * 2

        m = Motor()
        tap = tel.MotorTap(m)
        out = m.update_motor_thrusts(torch.ones(2, 4))
        self.assertTrue(torch.equal(out, torch.full((2, 4), 2.0)))
        self.assertTrue(torch.equal(tap.last_ref, torch.ones(2, 4)))
        tap.detach()
        self.assertIsNot(m.update_motor_thrusts, tap)
        src = {k: (lambda k=k: torch.zeros((2,) + tel.SUBSTEP_FIELDS[k])) for k in tel.SUBSTEP_FIELDS if k != "action_index"}
        src["position"] = lambda: state["x"]
        rec = tel.SubstepRecorder(2, 3, "cpu", src)
        for k in range(3):
            rec.current_action = k // 2
            rec()
            state["x"] = state["x"] + 1
            rec.post_physics_step()
        a = rec.arrays()
        self.assertEqual(a["action_index"].tolist(), [0, 0, 1])
        self.assertEqual(a["position"][:, 0, 0].tolist(), [1.0, 2.0, 3.0])
        with self.assertRaises(RuntimeError):
            rec()

    def test_snapshot_restore_plain_object(self):
        class Inner:
            pass

        class Outer:
            pass

        Inner.__module__ = Outer.__module__ = "aerial_gym.fake"
        o, i = Outer(), Inner()
        o.a, o.n, o.inner, o.d = torch.zeros(3), 1, i, {"k": torch.ones(2)}
        i.b = torch.zeros(2)
        alias = o.a
        snap = tel.snapshot(o)
        o.a += 5
        o.n = 7
        i.b[:] = 9
        o.d["k"] = torch.zeros(4)
        o.d["new"] = 1
        tel.restore(snap)
        self.assertTrue(torch.equal(o.a, torch.zeros(3)))
        self.assertIs(o.a, alias)
        self.assertEqual(o.n, 1)
        self.assertTrue(torch.equal(i.b, torch.zeros(2)))
        self.assertTrue(torch.equal(o.d["k"], torch.ones(2)))
        self.assertNotIn("new", o.d)

    def test_write_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            info = tel.write_json(Path(d) / "x.json", {"a": 1})
            self.assertEqual(info["sha256"], tel.sha256_file(Path(d) / "x.json"))
            with self.assertRaises(FileExistsError):
                tel.write_json(Path(d) / "x.json", {"a": 2})


@unittest.skipUnless((ROOT / "../aerial_gym_simulator/results/occlusion_confirmatory_v1").exists(), "receipt absent")
class PerceptionSnapshotTest(unittest.TestCase):
    def test_recompute_from_restored_state_is_bitwise(self):
        import eq1_common as common
        import eq1_information_flow as flow

        with common.production(common.receipt_env()):
            m, tc, p = common.build_perception(4)

            def step(t):
                x = flow.synthetic_inputs(t, [0, 1, 2, 3], p.HBEAMS, p.VBEAMS)
                return m.observe(**x, max_velocity=tc.max_velocity, flight_altitude=tc.flight_altitude,
                                 training=False, env_bounds_min=torch.zeros(4, 3),
                                 env_bounds_max=torch.tensor([[40.0, 40.0, 3.0]] * 4))[0].clone()

            for t in range(8):
                step(t)
            snap = tel.snapshot(m)
            a = step(8)
            tel.restore(snap)
            b = step(8)
            self.assertTrue(torch.equal(a, b))


class AnalysisReferenceTest(unittest.TestCase):
    def test_motor_gain_and_sat(self):
        self.assertAlmostEqual(ana.production_gain(0.01), 0.2214, places=12)
        r = np.eye(3)
        self.assertAlmostEqual(ana.sat_separation(np.zeros(3), r, ana.BOX_HALF, np.array([1.0, 0, 0]),
                                                  np.array([0.76, 0.5, 1.5])), 0.1, places=9)
        self.assertLess(ana.sat_separation(np.zeros(3), r, ana.BOX_HALF, np.array([0.5, 0, 0]),
                                           np.array([0.4, 0.5, 1.5])), 0)
        c = math.cos(math.radians(22.5)), math.sin(math.radians(22.5))
        rz = np.array([[c[0], -c[1], 0], [c[1], c[0], 0], [0, 0, 1]])
        self.assertGreater(ana.sat_separation(np.zeros(3), rz, ana.BOX_HALF, np.array([1.0, 0, 0]),
                                              np.array([0.76, 0.5, 1.5])), 0)

    def test_ray_tables(self):
        az, el = ana.lidar_rays()
        self.assertAlmostEqual(math.degrees(az[0]), 180.0, places=9)
        self.assertAlmostEqual(math.degrees(az[36]), 0.0, places=9)
        self.assertAlmostEqual(math.degrees(az[71]), -175.0, places=9)
        self.assertEqual([round(math.degrees(e), 9) for e in el], [20.0, 10.0, 0.0, -10.0])
        d = ana.obstacle_ray(19.5, 11.5)
        self.assertTrue(np.allclose(d, [1, 0, 0]))

    def test_d1_motor_fidelity_on_synthetic_record(self):
        dt, n = 0.01, 3
        ref = np.full((20, n, 4), 1.2)
        th = np.zeros((20, n, 4))
        prev = np.full((n, 4), 0.6)
        for k in range(20):
            w = np.sqrt(prev)
            th[k] = (w + ana.production_gain(dt) * (np.sqrt(ref[k]) - w)) ** 2
            prev = th[k]
        w, wr = np.sqrt(np.concatenate([np.full((1, n, 4), 0.6), th[:-1]])), np.sqrt(ref)
        pred = (w + ana.production_gain(dt) * (wr - w)) ** 2
        self.assertLess(float(np.abs(pred - th).max()), 1e-12)


class D3AnalysisTest(unittest.TestCase):
    def test_gates_on_synthetic_meta(self):
        frozen = PLAN["frozen"]["rms_digest_sha256"]
        with tempfile.TemporaryDirectory() as d:
            for p in PLAN["run"]["processes"]:
                if p["batch"] != "D3":
                    continue
                n = p["num_envs"]
                eq = {"rows_equal": [True] * n, "actions_equal": [True] * n, "differing_columns": [], "max_abs_diff": 0.0}
                log = [{"step": s, "reason": r, "branches": {"B_all": eq, "B_perm": eq, "B_subset": eq,
                                                             "P_control": {"changed_envs": 1, "in_view_envs": 1}}}
                       for s, r in ((20, "reset_boundary"), (50, "fixed_step"))]
                events = [["camera_render", 3, 4], ["lidar_render", 3, 4], ["observe", 3, 4]]
                meta = {"num_envs": n, "branch_log": log, "events": events, "rms_digest_start": frozen,
                        "rms_digest_end": frozen, "model_training_flag": False, "rms_training_flag": False,
                        "actor_width": 898, "origins": {"aerial_gym": str(ROOT / "aerial_gym/__init__.py")},
                        "tf32_matmul": True, "loop_wall_s": 1.0}
                (Path(d) / p["id"]).mkdir()
                (Path(d) / p["id"] / "d3_meta.json").write_text(json.dumps(meta))
            res = ana.analyse_d3(d)
            self.assertTrue(all(v["status"] == "PASS" for v in res["gates"].values()), res["gates"])
            bad = Path(d) / "d3_n2" / "d3_meta.json"
            meta = json.loads(bad.read_text())
            meta["branch_log"][1]["branches"]["B_perm"] = dict(meta["branch_log"][1]["branches"]["B_perm"],
                                                               rows_equal=[True, False])
            bad.write_text(json.dumps(meta))
            res = ana.analyse_d3(d)
            self.assertEqual(res["gates"]["D3-F2"]["status"], "FAIL")
            self.assertEqual(res["gates"]["D3-B1"]["status"], "FAIL")


EVIDENCE = ROOT / "docs/evidence/eq1d_qualification_2026-10-18"


@unittest.skipUnless((EVIDENCE / "eq1d_analysis.json").exists(), "EQ-1D evidence not present")
class RecordedEvidenceTest(unittest.TestCase):
    def test_official_run_record(self):
        report = json.loads((EVIDENCE / "run_records/run_report.json").read_text())
        self.assertEqual(report["status"], "COMPLETED")
        self.assertEqual([p["id"] for p in report["processes"]], [p["id"] for p in PLAN["run"]["processes"]])
        self.assertTrue(all(p["returncode"] == 0 for p in report["processes"]))
        summary = json.loads((EVIDENCE / "eq1d_analysis.json").read_text())["summary"]
        self.assertTrue(all(summary[g] == "PASS" for g in PLAN["D3"]["gates"]), summary)
        for gate in ("D1-T1", "D1-M1", "D1-M2", "D1-H1", "D1-B1", "D2-R2", "D2-S1", "D2-N1", "D2-T1", "D2-Y1"):
            self.assertEqual(summary[gate], "PASS", gate)
        for gate in ("D1-C1", "D1-V1", "D1-A1", "D2-R3", "D2-F1", "D2-O1", "D2-N2"):
            self.assertEqual(summary[gate], "FAIL", gate)  # preserved findings, not regressions to fix here
        pres = json.loads((EVIDENCE / "preservation_after.json").read_text())
        self.assertEqual(pres["eq1d_r1_manifest"]["mismatches"], [])
        self.assertEqual(pres["occ_p2_manifest"]["mismatches"], 0)
        self.assertTrue(pres["main_checkout_unchanged"])
        self.assertEqual(pres["policy_f"]["worktree"], PLAN["frozen"]["policy_f_sha256"])


class OrchestratorTest(unittest.TestCase):
    def test_env_scrub_and_overrides(self):
        os.environ["NAVRL_EQ1D_LEAK"] = "1"
        try:
            spec = next(p for p in PLAN["run"]["processes"] if p["id"] == "d1_dt5")
            env = runner.process_env(PLAN, spec) if runner.resolve(PLAN["frozen"]["receipt_environment"]).exists() else None
        finally:
            del os.environ["NAVRL_EQ1D_LEAK"]
        if env is None:
            self.skipTest("receipt absent")
        self.assertNotIn("NAVRL_EQ1D_LEAK", env)
        self.assertEqual(env["AERIAL_GYM_SIM_NAME"], "eq1d_d1_dt5")
        self.assertEqual(env["NAVRL_LIDAR_HBEAMS"], "72")
        self.assertNotIn("NAVRL_KROC2027_OCCLUSION_TRACE", env)
        self.assertEqual(env["PYTHONPATH"], str(runner.ROOT))

    def test_official_run_refuses_subset_and_wrong_commit(self):
        import contextlib
        import io

        with contextlib.redirect_stdout(io.StringIO()) as out:
            rc = runner.main(["run", "--approved-commit", "x", "--processes", "d1_dt10"])
        self.assertEqual(rc, 2)
        self.assertIn("smoke runs only", out.getvalue())


if __name__ == "__main__":
    unittest.main()
