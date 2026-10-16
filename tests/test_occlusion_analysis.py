"""End to end on CPU stand-ins: recorder trace -> offline line of sight -> intervals -> pilot summary."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import torch

ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


rp1a = load(ROOT / "tests/test_recovery_observability_contract.py", "_occlusion_e2e_fixture")
legacy, adapter = rp1a.legacy, rp1a.adapter
tool = load(ROOT / "tools/occlusion_analysis.py", "_occlusion_analysis_tool")

# Target y per consumed frame (bar centre at y = 15, half 0.3): 16 clear, 15 blocked.
TARGET_Y = [16.0, 15.0, 15.0, 16.0, 15.0]
CAMERA = [True, False, False, True, False]


def make_cell(test, directory, seed):
    task = legacy.make_task(n=1, bars=[[17.0, 15.0], [5.0, 35.0]])
    rp1a_task = rp1a.task_fixture(n=1)
    task.perception = rp1a_task.perception
    for name in ("_target_dynamics", "_physical_target", "_target_behavior_level", "_target_distance_backend",
                 "_target_motion_model", "task_obs"):
        setattr(task, name, getattr(rp1a_task, name))
    env = legacy.recorder_env(directory, NAVRL_EVAL_CHECKPOINT=rp1a.audit.POLICY_F_SHA256,
                              NAVRL_KROC2027_OCCLUSION_TRACE="1", NAVRL_SEED=str(seed))
    with mock.patch.dict(os.environ, env):
        rec = adapter.build_kroc_adapter(task)
    test.addCleanup(rec.finalize, "test cleanup")
    rec.on_reset(torch.arange(1))
    for y, camera in zip(TARGET_Y, CAMERA):
        task.target_position[:, 1] = y
        rec.after_perception(rp1a.flags(task, camera=camera))
        rec.before_action()
        rec._steps += 1
    rec.on_episode_end(torch.tensor([True]), torch.tensor([True]), torch.tensor([False]), torch.tensor([False]))
    rec.finalize()
    (Path(directory) / "episodes.jsonl").write_text(
        json.dumps({"env_index": 0, "env_episode_index": 0, "in_quota": True}) + "\n")


class EndToEndTest(unittest.TestCase):
    def test_cell_intervals_parity_and_censoring(self):
        with tempfile.TemporaryDirectory() as directory:
            make_cell(self, directory, 4201)
            out = tool.analyse_cell(directory)
            self.assertEqual(out["problems"], [])
            self.assertEqual(out["quota_episodes"], 1)
            self.assertEqual(out["parity"]["offline_vs_online_box_disagree"], 0)
            self.assertEqual(out["parity"]["probe_frames"], 0)
            occl = [(r["start_frame"], r["end_frame"], r["end"]) for r in out["_intervals"]["center_occlusion_in_view"]]
            self.assertEqual(occl, [(1, 3, "restored"), (4, 5, "capture")])
            fused = [(r["start_frame"], r["end_frame"], r["end"]) for r in out["_intervals"]["fused_loss"]]
            self.assertEqual(fused, [(1, 3, "restored"), (4, 5, "capture")])
            summary = out["indicators"]["center_occlusion_in_view"]
            self.assertEqual(summary["censored_intervals"], 1)
            self.assertEqual(summary["end_causes"]["capture"], 1)
            self.assertAlmostEqual(summary["kaplan_meier"]["S"]["0.5"], 0.0)
            self.assertEqual(out["indicators"]["rendered_full_in_view"], "NOT_RECORDED")
            reacq = out["reacquisition_after_center_occlusion"]
            self.assertEqual((reacq["restored_intervals"], reacq["reacquired"]), (1, 1))

    def test_pilot_groups_seeds_and_sweep_uses_recorded_layouts(self):
        with tempfile.TemporaryDirectory() as base:
            cells = []
            for seed in (4201, 4202):
                cell = Path(base) / "E2_direct_l2_v1" / "A3" / ("E2_obstacle_aware__70bars__seed%d" % seed)
                cell.mkdir(parents=True)
                make_cell(self, cell, seed)
                cells.append(cell)
            pilot = tool.analyse_pilot(base, resamples=50)
            group = pilot["groups"]["e2_obstacle_aware/70bars"]
            self.assertEqual(group["seeds"], [4201, 4202])
            rate = group["indicators"]["center_occlusion_in_view"]
            self.assertEqual(rate["intervals"], 4)
            self.assertEqual(set(rate["per_seed_rate_per_exposure_s"]), {"4201", "4202"})
            sweep = tool.run_sweep(cells, synthetic=False, paths=2, seed=0)
            self.assertEqual(sweep["label"], "GEOMETRY_ONLY_KINEMATIC_BASELINE")
            self.assertEqual(set(sweep["families"]), {"static_pursuit", "static_crossing", "moving_pursuit"})
            self.assertTrue(all(list(f["densities"]) == ["70"] for f in sweep["families"].values()))


if __name__ == "__main__":
    unittest.main()
