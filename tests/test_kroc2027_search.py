"""KROC2027 B1 yaw-scanning supervisor (CPU only).

Trigger timing, the 10.5 s / 4.5 s / 15 s limits, return on acquisition, no trigger on a single
missed frame, LiDAR suppression, balanced per-episode scan direction from (env, episode) parity, no
randomness, the task glue, and the recorder's per-episode invariants (horizontal command 0 and yaw
rate at most 0.6 rad/s while searching).
"""
import ast
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

import torch

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval.search_supervisor import (  # noqa: E402
    MODE_INITIAL,
    MODE_NONE,
    MODE_RESEARCH,
    SearchSpec,
    YawScanSupervisor,
)

ADAPTER = ROOT / "aerial_gym/task/navrl_task/kroc_eval_adapter.py"
TASK = ROOT / "aerial_gym/task/navrl_task/navrl_task.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


adapter = _load(ADAPTER, "_kroc2027_adapter_for_search")


def make(n=1, hard_margin=0.45):
    sup = YawScanSupervisor(n, "cpu", yaw_rate_max_rad_s=3.0, hard_margin_m=hard_margin)
    sup.on_reset(torch.arange(n))
    return sup


def tick(sup, fused=False, track=False, clearance=5.0, n=None):
    n = sup.num_envs if n is None else n
    actions = torch.full((n, 4), 0.7)
    out = sup.step(actions, torch.full((n,), fused), torch.full((n,), track), torch.full((n,), clearance))
    return out


class SpecTest(unittest.TestCase):
    def test_step_counts(self):
        spec = SearchSpec()
        self.assertEqual([spec.steps(s) for s in (1.0, 10.5, 4.5, 15.0)], [10, 105, 45, 150])

    def test_scan_rate_within_vehicle_limit(self):
        with self.assertRaises(ValueError):
            YawScanSupervisor(1, "cpu", yaw_rate_max_rad_s=0.5, hard_margin_m=0.45)
        self.assertAlmostEqual(make().yaw_action, 0.2)


class InitialSearchTest(unittest.TestCase):
    def test_starts_after_one_second_and_lasts_at_most_ten_and_a_half(self):
        sup = make()
        for _ in range(10):
            out = tick(sup)
            self.assertTrue(torch.equal(out, torch.full((1, 4), 0.7)))  # the policy acts
        steps = 0
        while True:
            out = tick(sup)
            if int(sup.mode[0]) == MODE_NONE:
                break
            steps += 1
            self.assertEqual(out[0, 0:2].tolist(), [0.0, 0.0])
            self.assertAlmostEqual(abs(float(out[0, 3])), 0.2, places=6)
            self.assertAlmostEqual(float(out[0, 2]), 0.7, places=6)  # vertical action untouched
        self.assertEqual(steps, 105)
        self.assertEqual(int(sup.stats["ended_by_limit"][0]), 1)
        for _ in range(50):  # never restarts
            tick(sup)
            self.assertEqual(int(sup.mode[0]), MODE_NONE)
        self.assertEqual(int(sup.stats["initial_searches"][0]), 1)

    def test_returns_on_the_step_that_sees_the_target(self):
        sup = make()
        for _ in range(15):
            tick(sup)
        self.assertEqual(int(sup.mode[0]), MODE_INITIAL)
        out = tick(sup, fused=True, track=True)
        self.assertEqual(int(sup.mode[0]), MODE_NONE)
        self.assertTrue(torch.equal(out, torch.full((1, 4), 0.7)))
        self.assertEqual(int(sup.stats["ended_by_acquisition"][0]), 1)

    def test_measured_early_means_no_initial_search(self):
        sup = make()
        tick(sup, fused=True, track=True)
        for _ in range(40):
            tick(sup, fused=False, track=True)  # coasting track
        self.assertEqual(int(sup.stats["initial_searches"][0]), 0)
        self.assertEqual(int(sup.stats["search_steps"][0]), 0)


class ResearchTest(unittest.TestCase):
    def test_single_miss_never_triggers_but_expiry_does(self):
        sup = make()
        tick(sup, fused=True, track=True)
        tick(sup, fused=False, track=True)  # a missed frame while the track coasts
        self.assertEqual(int(sup.mode[0]), MODE_NONE)
        tick(sup, fused=False, track=False)  # the track expired
        self.assertEqual(int(sup.mode[0]), MODE_RESEARCH)
        steps = 1
        while int(sup.mode[0]) == MODE_RESEARCH:
            tick(sup, fused=False, track=False)
            steps += 1
        self.assertEqual(steps, 46)  # 45 searching steps, then the 46th returns to the policy
        self.assertEqual(int(sup.stats["research_steps"][0]), 45)

    def test_episode_budget(self):
        sup = make()
        tick(sup, fused=True, track=True)
        for _ in range(10):
            # Establish and lose the track repeatedly; each loss allows at most 45 steps.
            tick(sup, fused=True, track=True)
            for _ in range(60):
                tick(sup, fused=False, track=False)
        self.assertEqual(int(sup.stats["search_steps"][0]), 150)


class SuppressionAndDirectionTest(unittest.TestCase):
    def test_suppression_holds_position(self):
        for clearance in (0.40, float("nan")):
            sup = make()
            for _ in range(11):
                out = tick(sup, clearance=clearance)
            self.assertTrue(bool(sup.active[0]))
            self.assertEqual(out[0, [0, 1, 3]].tolist(), [0.0, 0.0, 0.0])
            self.assertEqual(int(sup.stats["suppressed_steps"][0]), 1)

    def test_scan_direction_is_balanced_and_deterministic(self):
        sup = make(n=8)
        first = sup.sign.clone()
        self.assertEqual(int((first > 0).sum()), 4)
        sup.on_reset(torch.tensor([0, 1]))
        self.assertEqual(sup.sign[0:2].tolist(), [-first[0].item(), -first[1].item()])
        again = make(n=8)
        self.assertTrue(torch.equal(again.sign, first))

    def test_no_randomness(self):
        torch.manual_seed(3)
        before = torch.get_rng_state().clone()
        sup = make(n=4)
        for i in range(200):
            tick(sup, fused=(i % 37 == 0), track=(i % 50 < 20))
        self.assertTrue(torch.equal(before, torch.get_rng_state()))


def make_task(n=2):
    task = SimpleNamespace(
        num_envs=n, device=torch.device("cpu"), step_dt=0.1,
        task_config=SimpleNamespace(yaw_rate_max=3.0, max_velocity=2.5, flight_altitude=1.0),
        speed_governor_cfg=SimpleNamespace(hard_margin_m=0.45),
        _visible_now=torch.zeros(n, dtype=torch.bool),
        perception=SimpleNamespace(tracker=SimpleNamespace(active=torch.zeros(n, dtype=torch.bool)),
                                   last_scan_nearest=torch.full((n, 72), 6.0)),
    )
    return task


class TaskGlueTest(unittest.TestCase):
    def test_switch(self):
        task = make_task()
        self.assertIsNone(adapter.build_search_supervisor(task, {}))
        self.assertIsNone(adapter.build_search_supervisor(task, {"NAVRL_KROC2027_SEARCH": "off"}))
        with self.assertRaises(RuntimeError):
            adapter.build_search_supervisor(task, {"NAVRL_KROC2027_SEARCH": "b2"})
        search = adapter.build_search_supervisor(task, {"NAVRL_KROC2027_SEARCH": "b1"})
        self.assertAlmostEqual(search.supervisor.hard_margin_m, 0.45)

    def test_reads_only_actor_side_signals(self):
        task = make_task()
        search = adapter.build_search_supervisor(task, {"NAVRL_KROC2027_SEARCH": "b1"})
        search.on_reset(torch.arange(2))
        actions = torch.full((2, 4), 0.5)
        for _ in range(11):
            out = search.apply(actions)
        self.assertTrue(bool(search.active.all()))
        self.assertEqual(out[:, 0:2].abs().sum().item(), 0.0)
        task.perception.last_scan_nearest = None  # no scan -> clearance unknown -> hold
        out = search.apply(actions)
        self.assertEqual(out[:, 3].abs().sum().item(), 0.0)

    def test_task_hooks_are_guarded(self):
        tree = ast.parse(TASK.read_text(encoding="utf-8"))
        calls, guarded = [], []
        for node in ast.walk(tree):
            if isinstance(node, ast.If) and "_kroc_search" in ast.dump(node.test) \
                    and "IsNot" in ast.dump(node.test):
                for inner in ast.walk(node):
                    if isinstance(inner, ast.Call) and isinstance(inner.func, ast.Attribute) \
                            and "_kroc_search" in ast.dump(inner.func.value):
                        guarded.append(inner.func.attr)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                    and "_kroc_search" in ast.dump(node.func.value):
                calls.append(node.func.attr)
        self.assertEqual(sorted(calls), sorted(guarded))
        self.assertEqual(sorted(set(calls)), ["apply", "on_reset"])


class RecorderInvariantTest(unittest.TestCase):
    def test_records_search_fields_and_invariants(self):
        contract = _load(ROOT / "tests/test_kroc2027_adapter_contract.py", "_kroc2027_contract_helpers")
        task = contract.make_task(n=2)
        task.task_config.yaw_rate_max = 3.0
        task.speed_governor_cfg = SimpleNamespace(hard_margin_m=0.45)
        task._visible_now = torch.zeros(2, dtype=torch.bool)
        task.perception.last_scan_nearest = torch.full((2, 72), 6.0)
        task._kroc_search = adapter.build_search_supervisor(task, {"NAVRL_KROC2027_SEARCH": "b1"})
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        with mock.patch.dict(os.environ, contract.recorder_env(tmp.name)):
            rec = adapter.build_kroc_adapter(task)
        rec.on_reset(torch.arange(2))
        task._kroc_search.on_reset(torch.arange(2))
        rec.after_perception({"visible": torch.zeros(2, dtype=torch.bool)})
        for _ in range(20):
            rec.before_action()
            actions = task._kroc_search.apply(torch.full((2, 4), 0.6))
            command = torch.zeros(2, 4)
            command[:, 0:2] = actions[:, 0:2] * 2.5   # the task's scaling
            command[:, 3] = actions[:, 3] * 3.0
            rec.after_target_advance()
            rec.after_physics(actions, command)
            rec.after_perception({"visible": torch.zeros(2, dtype=torch.bool)})
        rec.on_episode_end(torch.ones(2, dtype=torch.bool), torch.zeros(2, dtype=torch.bool),
                           torch.zeros(2, dtype=torch.bool), torch.ones(2, dtype=torch.bool))
        rec.finalize()
        records = [json.loads(l) for l in rec.episodes_path.read_text().splitlines()]
        summary = json.loads(rec.summary_path.read_text())
        self.assertEqual([r["search_search_steps"] for r in records], [10, 10])
        self.assertEqual([r["search_commanded_xy_max_mps"] for r in records], [0.0, 0.0])
        for r in records:
            self.assertAlmostEqual(r["search_yaw_rate_max_rad_s"], 0.6, places=5)
            self.assertTrue(r["search_active_at_end"])
        self.assertEqual(sorted(r["search_scan_sign"] for r in records), [-1, 1])
        self.assertEqual(summary["search"]["invariant_violations"], 0)
        self.assertEqual(summary["search"]["episodes_with_search"], 2)


if __name__ == "__main__":
    unittest.main()
