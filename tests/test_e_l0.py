"""E-L0 evaluation lineage (CPU): label functions against independent oracles, contact chain, adapter
gating, in-run non-interference comparison and plan integrity. The recorder itself and its
non-interference are validated on the GPU by ``tools/e_l0_run.py`` (run EL0-R1)."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tools"), str(ROOT / "tools/audits")):
    if p not in sys.path:
        sys.path.insert(0, p)

from motar_eval import e_l0  # noqa: E402
import e_l0_analysis as ana  # noqa: E402
import e_l0_run as orchestrator  # noqa: E402
import eq1_geometry_reference as geo  # noqa: E402

PLAN = json.loads((ROOT / "docs/plans/e_l0_evaluation_lineage_v1.json").read_text())


def camera_rays():
    """The detector's pinhole ray table for 160 x 90, 87 x 58 deg, built here from the declared intrinsics."""
    fx = 160 / (2 * math.tan(math.radians(43.5)))
    fy = 90 / (2 * math.tan(math.radians(29.0)))
    vv, uu = torch.meshgrid(torch.arange(90.0), torch.arange(160.0), indexing="ij")
    rays = torch.stack([torch.ones_like(uu), -(uu - 79.5) / fx, -(vv - 44.5) / fy], dim=-1)
    return rays / rays.norm(dim=-1, keepdim=True)


class SilhouetteTest(unittest.TestCase):
    def test_alone_mask_matches_analytic_oracle(self):
        rays = camera_rays().double()
        eye = torch.tensor([[0.10, 0.0, 0.03]], dtype=torch.float64)
        quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]], dtype=torch.float64)
        for rel in ([3.0, 0.0, 0.03], [8.0, 0.0, 0.03], [6 * math.cos(0.35), 6 * math.sin(0.35), 0.03], [20.0, 0.0, 0.03]):
            tgt = torch.tensor([rel], dtype=torch.float64)
            mask = e_l0.target_alone_mask(rays, eye, quat, tgt, 0.15, 20.0)
            alone, _ = ana.oracle_counts(np.array(geo.CAM_T), np.array(rel), [])
            self.assertEqual(int(mask.sum()), alone, rel)

    def test_yaw_rotation_and_frustum(self):
        q = torch.tensor([[0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]])  # yaw +90 deg
        v = e_l0.quat_rotate(q, torch.tensor([[1.0, 0.0, 0.0]]))
        self.assertTrue(torch.allclose(v, torch.tensor([[0.0, 1.0, 0.0]]), atol=1e-6))
        origin = torch.zeros(2, 3)
        ident = torch.tensor([[0.0, 0.0, 0.0, 1.0]] * 2)
        targets = torch.tensor([[10.0, 0.0, 0.0], [10 * math.cos(math.radians(50)), 10 * math.sin(math.radians(50)), 0.0]])
        f = e_l0.centre_in_frustum(origin, ident, targets, math.radians(87), math.radians(58), 20.0)
        self.assertEqual(f.tolist(), [True, False])

    def test_classify_rules(self):
        L = lambda name: e_l0.CLASSES.index(name)  # noqa: E731
        alone = torch.tensor([0, 0, 0, 10, 10, 10, 10])
        vis = torch.tensor([0, 0, 0, 0, 4, 4, 10])
        fr = torch.tensor([True, False, True, True, True, True, True])
        rng = torch.tensor([25.0, 10.0, 10.0, 5.0, 5.0, 5.0, 5.0])
        blk = torch.tensor([False, False, False, True, True, False, False])
        got = e_l0.classify(alone, vis, fr, rng, blk, 0.15, 20.0).tolist()
        self.assertEqual(got, [L("RANGE_EXIT"), L("FOV_EXIT"), L("SCALE_LOSS"), L("FULL"), L("CENTRE_ONLY"),
                               L("PARTIAL"), L("VISIBLE")])
        for a, v, f, r, b, expected in zip(alone.tolist(), vis.tolist(), fr.tolist(), rng.tolist(), blk.tolist(), got):
            self.assertEqual(ana.oracle_class(a, v, f, r, b), e_l0.CLASSES[expected])

    def test_camera_provenance(self):
        cam = torch.tensor([True, True, False, False])
        px = torch.tensor([5, 0, 5, 1])
        got = [e_l0.CAMERA_PROV[i] for i in e_l0.camera_provenance(cam, px, 2).tolist()]
        self.assertEqual(got, ["TRUE_TARGET", "FALSE_DETECTION", "MISSED", "NONE"])


class LidarProvenanceTest(unittest.TestCase):
    def test_beam_row_semantic_and_labels(self):
        angles = torch.tensor([math.radians(180.0 - 5.0 * j) for j in range(72)])
        scan = torch.full((3, 4, 72), 12.0)
        sem = torch.full((3, 4, 72), -2, dtype=torch.int32)
        scan[:, 2, 36], scan[:, 1, 36] = 4.8, 5.1
        sem[0, 2, 36], sem[1, 2, 36], sem[2, 2, 36] = 50, 3, 50
        valid = torch.tensor([True, True, False])
        bearing = torch.zeros(3)
        measured = torch.full((3,), 4.8)
        out = e_l0.lidar_provenance(valid, bearing, measured, scan, sem, angles)
        self.assertEqual(out["beam"].tolist(), [36, 36, 36])
        self.assertEqual(out["row"].tolist(), [2, 2, 2])
        self.assertEqual([e_l0.LIDAR_PROV[i] for i in out["label"].tolist()], ["CORRECT", "FALSE_ASSOCIATION", "NONE"])
        self.assertTrue(bool(out["consistent"].all()))

    def test_lidar_first_hit_oracle(self):
        box = [(np.array([2.6, -0.4, -1.5]), np.array([3.4, 0.4, 1.5]))]
        obj, rng = ana.lidar_first_hit(np.zeros(3), 0.0, 0.0, np.array([6.0, 0.0, 0.0]), box)
        self.assertEqual(obj, "BAR")
        self.assertAlmostEqual(rng, 2.6, places=9)
        obj, rng = ana.lidar_first_hit(np.zeros(3), 0.0, 0.0, np.array([6.0, 0.0, 0.0]), [])
        self.assertEqual(obj, "TARGET")
        self.assertAlmostEqual(rng, 5.8, places=9)


class ContactChainTest(unittest.TestCase):
    def test_chains_existing_callback_and_records(self):
        calls = []

        class Existing:
            def __call__(self):
                calls.append("pre")

            def post_physics_step(self):
                calls.append("post")

        forces = [torch.tensor([[0.0, 0, 0], [0.0, 0, 0]]), torch.tensor([[0.2, 0, 0], [0.0, 0, 0]]),
                  torch.tensor([[1.0, 0, 0], [0.03, 0, 0]])]
        state = {"k": 0}
        chain = e_l0.ContactChain(Existing(), lambda: forces[state["k"]], 0.05, 2, "cpu")
        chain.begin_action()
        for k in range(3):
            state["k"] = k
            chain()
            chain.post_physics_step()
        self.assertEqual(calls, ["pre", "post"] * 3)
        self.assertEqual(chain.count.tolist(), [2, 0])
        self.assertEqual(chain.first.tolist(), [1, -1])
        self.assertAlmostEqual(float(chain.peak[0]), 1.0)
        chain.begin_action()
        self.assertEqual(chain.count.tolist(), [0, 0])


class AttachTest(unittest.TestCase):
    """Runtime attachment: originals still run with the same arguments and return values; E-L0 order."""

    def make(self):
        calls = []

        class Adapter:
            context, import_origins = {"cell_id": "x"}, {}

            def on_reset(self, env_ids):
                calls.append(("adapter.on_reset", list(map(int, env_ids))))
                return "r"

            def after_perception(self, d):
                calls.append(("adapter.after_perception", d["k"]))
                return "p"

            def before_action(self):
                calls.append(("adapter.before_action",))
                return "b"

            def after_physics(self, actions, command):
                calls.append(("adapter.after_physics", float(actions.sum()), float(command.sum())))
                return "f"

            def on_episode_end(self, finished, successes, crashes, timeouts):
                calls.append(("adapter.on_episode_end", finished.tolist()))
                return "e"

            def finalize(self, reason="complete"):
                calls.append(("adapter.finalize", reason))
                return "z"

        class Rec:
            device = "cpu"

            def on_reset(self, ids):
                calls.append(("e_l0.on_reset", ids.tolist()))

            def after_perception(self, d):
                calls.append(("e_l0.after_perception", d["k"]))

            def before_action(self):
                calls.append(("e_l0.before_action",))

            def after_physics(self, actions, command):
                calls.append(("e_l0.after_physics",))

            def on_episode_end(self, ids, capture, crash, timeout):
                calls.append(("e_l0.on_episode_end", ids.tolist(), capture, crash, timeout))

            def finalize(self, reason, meta):
                calls.append(("e_l0.finalize", reason, meta["attachment"].split(" ")[0]))

        adapter = Adapter()
        rec = e_l0.attach(adapter, recorder=Rec())
        return adapter, rec, calls

    def test_order_arguments_and_returns(self):
        adapter, rec, calls = self.make()
        self.assertIs(adapter._e_l0, rec)
        self.assertEqual(adapter.on_reset([1, 2]), "r")
        self.assertEqual(adapter.after_perception({"k": 7}), "p")
        self.assertEqual(adapter.before_action(), "b")
        self.assertEqual(adapter.after_physics(torch.ones(2, 4), torch.ones(2, 4) * 2), "f")
        fin = torch.tensor([False, True])
        self.assertEqual(adapter.on_episode_end(fin, torch.tensor([False, True]), torch.tensor([False, False]),
                                                torch.tensor([False, False])), "e")
        self.assertEqual(adapter.finalize("done"), "z")
        self.assertEqual(calls, [
            ("adapter.on_reset", [1, 2]), ("e_l0.on_reset", [1, 2]),
            ("adapter.after_perception", 7), ("e_l0.after_perception", 7),
            ("adapter.before_action",), ("e_l0.before_action",),
            ("e_l0.after_physics",), ("adapter.after_physics", 8.0, 16.0),
            ("e_l0.on_episode_end", [1], [True], [False], [False]), ("adapter.on_episode_end", [False, True]),
            ("adapter.finalize", "done"), ("e_l0.finalize", "done", "runtime")])
        with self.assertRaises(RuntimeError):
            e_l0.attach(adapter, recorder=rec)

    def test_switch(self):
        self.assertFalse(e_l0.switch_on({}))
        self.assertFalse(e_l0.switch_on({"NAVRL_KROC2027_E_L0": "0"}))
        self.assertTrue(e_l0.switch_on({"NAVRL_KROC2027_E_L0": "1"}))
        with self.assertRaises(ValueError):
            e_l0.switch_on({"NAVRL_KROC2027_E_L0": "yes"})

    def test_frozen_adapter_untouched(self):
        import subprocess

        diff = subprocess.run(["git", "diff", "--quiet", "5ff285ad053f181c5a1c86dddb2462bb44e6327b", "HEAD", "--",
                               "aerial_gym/task/navrl_task/kroc_eval_adapter.py"], cwd=ROOT).returncode
        dirty = subprocess.run(["git", "diff", "--quiet", "--", "aerial_gym/task/navrl_task/kroc_eval_adapter.py"],
                               cwd=ROOT).returncode
        self.assertEqual((diff, dirty), (0, 0))


class OrchestratorTest(unittest.TestCase):
    def test_compare_arms_detects_a_difference(self):
        with tempfile.TemporaryDirectory() as d:
            rows = [{"step": k, **{key: "h%d" % k for key in orchestrator.DIGEST_KEYS}} for k in range(3)]
            for name in ("a", "b", "c"):
                (Path(d) / name).mkdir()
                (Path(d) / name / "v3_steps.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
            self.assertTrue(orchestrator.compare_arms(d, ["a", "b", "c"])["identical"])
            bad = [dict(r) for r in rows]
            bad[1]["tracker"] = "different"
            (Path(d) / "c" / "v3_steps.jsonl").write_text("".join(json.dumps(r) + "\n" for r in bad))
            cmp = orchestrator.compare_arms(d, ["a", "b", "c"])
            self.assertFalse(cmp["identical"])
            self.assertEqual(cmp["pairs"]["c_vs_a"]["differences"], [{"step": 1, "key": "tracker"}])

    def test_arm_environments(self):
        spec = {s["id"]: s for s in PLAN["run"]["processes"]}
        if not orchestrator.config_receipt(PLAN, spec["v3_static_p0"]).exists():
            self.skipTest("receipt absent")
        with tempfile.TemporaryDirectory() as d:
            e0 = orchestrator.process_env(PLAN, spec["v3_static_p0"], d)
            e1 = orchestrator.process_env(PLAN, spec["v3_static_p1"], d)
            e2 = orchestrator.process_env(PLAN, spec["v3_static_p2"], d)
            mv = orchestrator.process_env(PLAN, spec["v3_moving_p2"], d)
        self.assertNotIn("NAVRL_KROC2027_RECORDER", e0)
        self.assertEqual((e1["NAVRL_KROC2027_RECORDER"], e1.get("NAVRL_KROC2027_E_L0")), ("1", None))
        self.assertEqual(e2["NAVRL_KROC2027_E_L0"], "1")
        self.assertEqual(e0["NAVRL_NUM_BARS"], "160")
        self.assertEqual(mv["NAVRL_TARGET_BEHAVIOR_LEVEL"], "e2_obstacle_aware")
        drop = {"NAVRL_KROC2027_RECORDER", "NAVRL_KROC2027_OCCLUSION_TRACE", "NAVRL_KROC2027_RUN_ID",
                "NAVRL_KROC2027_ARM", "NAVRL_KROC2027_E_L0"}
        self.assertEqual({k: v for k, v in e2.items() if k not in drop}, {k: v for k, v in e0.items() if k not in drop})


EVIDENCE = ROOT / "docs/evidence/e_l0_evaluation_2026-10-18"


@unittest.skipUnless((EVIDENCE / "e_l0_analysis.json").exists(), "E-L0 evidence not present")
class RecordedEvidenceTest(unittest.TestCase):
    def test_official_run(self):
        report = json.loads((EVIDENCE / "run_records/run_report.json").read_text())
        self.assertEqual(report["status"], "COMPLETED")
        self.assertEqual([p["id"] for p in report["processes"]], [p["id"] for p in PLAN["run"]["processes"]])
        self.assertTrue(all(v["identical"] for v in report["noninterference"].values()))
        summary = json.loads((EVIDENCE / "e_l0_analysis.json").read_text())["summary"]
        self.assertEqual(set(summary), set(PLAN["gates"]))
        self.assertTrue(all(v == "PASS" for v in summary.values()), summary)
        pres = json.loads((EVIDENCE / "preservation_after.json").read_text())
        for key in ("el0_r1", "eq1d_r1", "occ_p2"):
            self.assertEqual(pres[key]["mismatches"], 0)
        self.assertTrue(pres["adapter_identical_to_measurement_source"])
        self.assertTrue(pres["main_checkout_unchanged"])


class PlanTest(unittest.TestCase):
    def test_contract(self):
        self.assertTrue(PLAN["status"].startswith("APPROVED"))
        self.assertEqual(PLAN["switch"]["variable"], "NAVRL_KROC2027_E_L0")
        ids = [p["id"] for p in PLAN["run"]["processes"]]
        self.assertEqual(len(ids), 11)
        self.assertEqual(sum(p["batch"] == "V3" for p in PLAN["run"]["processes"]), 9)
        self.assertEqual(PLAN["run"]["disk_min_free_gb"], 8.0)
        self.assertEqual(sorted(s["id"] for s in PLAN["V1"]["extra_scenes"]), ["A1", "A2", "A3", "A4"])
        self.assertIn("NOT_IMPLEMENTED", PLAN["records"]["episode"])


if __name__ == "__main__":
    unittest.main()
