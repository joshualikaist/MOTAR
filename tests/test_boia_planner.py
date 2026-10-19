"""U7 synthetic integration, independent covariance/hash oracles and header-only IO."""
import ast
from dataclasses import FrozenInstanceError, fields, replace
import hashlib
import math
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock
import zipfile

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import access, arbitration, belief, geometry, planner, trace, types, visibility  # noqa: E402
from tools import boia_raw_inventory as inventory  # noqa: E402
from tools.boia_replay import fixture_params, make_scene, run_replay  # noqa: E402

for module in (access, arbitration, belief, geometry, planner, trace, types, visibility, inventory):
    assert str(Path(module.__file__).resolve()).startswith(str(ROOT) + "/")


def generator(seed=20261019):
    g = torch.Generator(device="cpu")
    g.manual_seed(seed)
    return g


def tensors(value, name=""):
    if isinstance(value, torch.Tensor):
        return {name: value.clone()}
    if isinstance(value, SimpleNamespace):
        return tensors(vars(value), name)
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            result.update(tensors(item, name + "/" + str(key)))
        return result
    return {}


def assert_trace_equal(test, left, right):
    test.assertEqual(set(left), set(right))
    for key in left:
        np.testing.assert_array_equal(left[key], right[key], err_msg=key)


class TraceTest(unittest.TestCase):
    def test_sha256_matches_hashlib_and_padding_boundaries(self):
        self.assertEqual(trace.sha256(b"abc"), hashlib.sha256(b"abc").digest())
        for size in (0, 1, 55, 56, 63, 64, 65, 127, 128, 1000):
            payload = bytes(torch.randint(0, 256, (size,), generator=generator(size), dtype=torch.int64).tolist())
            self.assertEqual(trace.sha256(payload), hashlib.sha256(payload).digest(), size)

    def test_digest_matches_canonical_bytes_and_sensitive_allowed_fields(self):
        task, perception, action = make_scene("clear", 2, torch.float64)
        p = fixture_params()
        inp = access.read_online(task, perception, action, p)
        _, decision = planner.BoiaPlanner(p).step(task, perception, action, torch.zeros(2, dtype=torch.bool), generator())
        for e in range(2):
            expected = np.frombuffer(hashlib.sha256(trace.canonical_input(inp, e)).digest()[:8], dtype=np.uint8)
            np.testing.assert_array_equal(decision.trace["input_digest"][e], expected)
        changed = replace(inp, age=inp.age + 1)
        self.assertNotEqual(trace.canonical_input(inp, 0), trace.canonical_input(changed, 0))
        changed = replace(inp, cam=replace(inp.cam, fx=inp.cam.fx + 1))
        self.assertNotEqual(trace.canonical_input(inp, 0), trace.canonical_input(changed, 0))

    def test_trace_arrays_are_copies_and_have_no_evaluator_fields(self):
        task, perception, action = make_scene("clear")
        _, decision = planner.BoiaPlanner(fixture_params()).step(task, perception, action, torch.zeros(1, dtype=torch.bool), generator())
        before = decision.action.clone()
        for key, array in decision.trace.items():
            self.assertIsInstance(array, np.ndarray)
            self.assertFalse(set(access.FORBIDDEN_ATTRS) & set(decision.trace))
            if np.issubdtype(array.dtype, np.floating):
                self.assertFalse(np.isnan(array).any(), key)
        decision.trace["action"][:] = 99
        self.assertTrue(torch.equal(decision.action, before))


class HorizonTest(unittest.TestCase):
    def test_recursive_expected_covariance_matches_independent_axis_equations(self):
        sigma = torch.eye(6, dtype=torch.float64)[None]
        q = torch.diag(torch.tensor([.00025] * 3 + [.025] * 3, dtype=torch.float64))
        r = torch.diag(torch.tensor([.04, .09, .16], dtype=torch.float64))[None]
        probability = torch.tensor([[[.2, .5, 1.], [0., 0., 0.]]], dtype=torch.float64)
        actual = planner.expected_horizon(sigma, probability, q, r, .1)
        expected = torch.zeros_like(actual)
        for candidate, probabilities in enumerate(probability[0].tolist()):
            for axis, measurement in enumerate((.04, .09, .16)):
                a, c, b = 1., 0., 1.
                for j, p in enumerate(probabilities):
                    ap, cp, bp = a + .2 * c + .01 * b + .00025, c + .1 * b, b + .025
                    a, c, b = ap - p * ap * ap / (ap + measurement), cp - p * ap * cp / (ap + measurement), bp - p * cp * cp / (ap + measurement)
                    expected[0, candidate, j, axis, axis] = a
                    expected[0, candidate, j, axis, axis + 3] = c
                    expected[0, candidate, j, axis + 3, axis] = c
                    expected[0, candidate, j, axis + 3, axis + 3] = b
        torch.testing.assert_close(actual, expected, atol=1e-12, rtol=0)
        self.assertTrue(bool((torch.linalg.eigvalsh(actual) > 0).all()))


class PlannerTest(unittest.TestCase):
    def test_three_scenes_deterministic_shapes_and_no_nan(self):
        for name in ("clear", "bar", "narrow_gap"):
            left = run_replay(4, scene=name, n=2, terminal_at=2)
            right = run_replay(4, scene=name, n=2, terminal_at=2)
            self.assertEqual(left["actions"].shape, (4, 2, 4))
            np.testing.assert_array_equal(left["actions"], right["actions"])
            for a, b in zip(left["traces"], right["traces"]):
                assert_trace_equal(self, a, b)
                for key, value in a.items():
                    if np.issubdtype(value.dtype, np.floating):
                        self.assertFalse(np.isnan(value).any(), (name, key))
            self.assertEqual(left["traces"][2]["env_index"].size, 0)

    def test_128_step_synthetic_replay_clear_never_overrides(self):
        result = run_replay(128, scene="clear")
        self.assertEqual(result["actions"].shape, (128, 1, 4))
        self.assertTrue(np.isfinite(result["actions"]).all())
        self.assertTrue(all(not t["engaged"].any() and not t["action_changed"].any() for t in result["traces"]))

    def test_bar_safety_override_and_inactive_belief_does_not_delete_obstacles(self):
        for active in (True, False):
            task, perception, action = make_scene("bar")
            perception.tracker.active[:] = active
            if not active:
                perception.tracker.state[:] = 0
                perception.tracker.cov[:] = torch.eye(6)
            chosen, decision = planner.BoiaPlanner(fixture_params()).step(task, perception, action, torch.zeros(1, dtype=torch.bool), generator())
            self.assertTrue(bool(decision.engaged[0]))
            self.assertEqual(int(decision.reason[0]), 1)
            self.assertFalse(bool(decision.hard_ok[0, 0] & decision.stop_ok[0, 0]))
            self.assertFalse(torch.equal(chosen, action))

    def test_all_terminal_bypasses_access_model_priors_and_rng(self):
        action = torch.tensor([[.2, 0., .5, 0.], [-.1, .2, 0., .3]])
        old = arbitration.initial_state(2, action)
        old = replace(old, engaged=torch.ones(2, dtype=torch.bool), dwell=torch.ones(2, dtype=torch.int64) * 4,
                      ever_detected=torch.ones(2, dtype=torch.bool), a_prev=torch.ones_like(action))
        p = planner.BoiaPlanner(types.BoiaParams(tau_on=.1), old)
        g = generator()
        rng = g.get_state().clone()
        chosen, d = p.step(object(), object(), action, torch.ones(2, dtype=torch.bool), g)
        self.assertTrue(torch.equal(chosen, action))
        self.assertTrue(torch.equal(g.get_state(), rng))
        self.assertTrue(bool((d.mode == arbitration.Mode.TERMINAL).all()))
        self.assertFalse(bool(d.engaged.any() or d.new_state.ever_detected.any()))
        self.assertTrue(bool((d.new_state.a_prev == 0).all()))
        self.assertEqual(d.trace["input_digest"].shape, (0, 8))
        self.assertTrue(bool(old.engaged.all()))

    def test_partial_terminal_observation_is_not_evaluated_or_logged(self):
        task, perception, action = make_scene("clear", 2)
        mask = torch.tensor([True, False])
        p = planner.BoiaPlanner(fixture_params())
        first, a = p.step(task, perception, action, mask, generator())
        task.obs_dict["robot_position"][0] = 99
        task.obs_dict["robot_linvel"][0] = -100
        perception.tracker.state[0] = 123
        perception.tracker.cov[0] *= 1e6
        task._camera_visible_now[0] = False
        second, b = p.step(task, perception, action, mask, generator())
        self.assertTrue(torch.equal(first, second))
        assert_trace_equal(self, a.trace, b.trace)
        np.testing.assert_array_equal(a.trace["env_index"], [1])
        self.assertTrue(torch.equal(second[0], action[0]))
        self.assertEqual(int(b.new_state.mode[0]), arbitration.Mode.TERMINAL)

    def test_forbidden_mutation_firewall_at_planner_and_trace_level(self):
        task, perception, action = make_scene("clear", 2)
        for name in access.FORBIDDEN_ATTRS:
            if name != "task_obs":
                setattr(task, name, torch.ones(2, 3))
        p = planner.BoiaPlanner(fixture_params())
        a, left = p.step(task, perception, action, torch.zeros(2, dtype=torch.bool), generator())
        g = generator(42)
        for name in access.FORBIDDEN_ATTRS:
            if name != "task_obs":
                setattr(task, name, torch.randn(2, 3, generator=g) * 1000)
        for container in (task.obs_dict, vars(task.detector), task.task_obs):
            for key in list(container):
                if key in access.FORBIDDEN_ATTRS or container is task.task_obs:
                    container[key] = torch.randn(container[key].shape, generator=g)
        b, right = p.step(task, perception, action, torch.zeros(2, dtype=torch.bool), generator())
        self.assertTrue(torch.equal(a, b))
        assert_trace_equal(self, left.trace, right.trace)

    def test_no_source_write_global_rng_or_implicit_planner_state(self):
        task, perception, action = make_scene("clear", 2)
        before = tensors(task)
        before.update(tensors(perception, "perception"))
        p = planner.BoiaPlanner(fixture_params())
        global_rng, default = torch.get_rng_state().clone(), torch.get_default_dtype()
        tf32 = torch.backends.cuda.matmul.allow_tf32
        g = generator()
        local = g.get_state().clone()
        _, d = p.step(task, perception, action, torch.zeros(2, dtype=torch.bool), g)
        self.assertTrue(torch.equal(global_rng, torch.get_rng_state()))
        self.assertFalse(torch.equal(local, g.get_state()))
        self.assertEqual(default, torch.get_default_dtype())
        self.assertEqual(tf32, torch.backends.cuda.matmul.allow_tf32)
        after = tensors(task)
        after.update(tensors(perception, "perception"))
        self.assertEqual(set(before), set(after))
        for key in before:
            self.assertTrue(torch.equal(before[key], after[key]), key)
        self.assertIsNone(p.state)
        self.assertIsNot(p.with_state(d.new_state), p)
        with self.assertRaises(FrozenInstanceError):
            p.state = d.new_state

    def test_mode_coverage_including_history_and_known_model_pre_trigger(self):
        modes = set()
        params = fixture_params()
        for desired in (arbitration.Mode.NEVER_ACQUIRED, arbitration.Mode.TRACK_VISIBLE,
                        arbitration.Mode.LOST_PREDICTABLE, arbitration.Mode.LOST_UNLOCALIZED,
                        arbitration.Mode.INTERCEPT, arbitration.Mode.PRE_OCCLUSION, arbitration.Mode.SAFE_HOLD):
            task, perception, action = make_scene("clear")
            p = planner.BoiaPlanner(params)
            if desired in (arbitration.Mode.LOST_PREDICTABLE, arbitration.Mode.LOST_UNLOCALIZED):
                _, prefix = p.step(task, perception, action, torch.zeros(1, dtype=torch.bool), generator())
                p = p.with_state(prefix.new_state)
                task._camera_visible_now[:] = False
                perception.tracker.active[:] = desired == arbitration.Mode.LOST_PREDICTABLE
            elif desired == arbitration.Mode.NEVER_ACQUIRED:
                task._camera_visible_now[:] = False
            elif desired == arbitration.Mode.INTERCEPT:
                perception.tracker.state[:, 0] = 2.
            elif desired == arbitration.Mode.PRE_OCCLUSION:
                # Declared model limitation: current camera clear, first target-like
                # hit caps known-free before centre. This is NOT a valid forecast claim.
                task._fixture_scan[:, 144 + 36] = 5.8
            elif desired == arbitration.Mode.SAFE_HOLD:
                task._fixture_scan[:] = 0
                task.obs_dict["robot_linvel"][:, 0] = 2.
            _, decision = p.step(task, perception, action, torch.zeros(1, dtype=torch.bool), generator())
            self.assertEqual(int(decision.mode[0]), desired)
            modes.add(desired)
            if desired == arbitration.Mode.SAFE_HOLD:
                self.assertEqual(int(decision.reason[0]), 3)
                self.assertTrue(bool((decision.action[0, :3] == 0).all()))
        _, terminal = p.step(task, perception, action, torch.ones(1, dtype=torch.bool), generator())
        modes.add(int(terminal.mode[0]))
        self.assertEqual(modes, set(range(8)))

    def test_missing_priors_generator_and_unavailable_yaw_rate_fail_closed(self):
        task, perception, action = make_scene("clear")
        mask = torch.zeros(1, dtype=torch.bool)
        for params in (types.BoiaParams(tau_on=.1), replace(fixture_params(), tau_psi=.2)):
            with self.assertRaises(ValueError):
                planner.BoiaPlanner(params).step(task, perception, action, mask, generator())
        with self.assertRaises(TypeError):
            planner.BoiaPlanner(fixture_params()).step(task, perception, action, mask, None)

    def test_package_imports_and_forbidden_attribute_names(self):
        forbidden = set(access.FORBIDDEN_ATTRS)
        for path in (ROOT / "motar_eval/boia").glob("*.py"):
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    self.assertTrue(all(n.name in {"math", "torch", "dataclasses", "typing"} for n in node.names))
                if isinstance(node, ast.ImportFrom):
                    self.assertTrue(node.level == 1 or node.module in {"math", "torch", "dataclasses", "typing"})
                if isinstance(node, ast.Attribute):
                    self.assertNotIn(node.attr, forbidden)


class InventoryTest(unittest.TestCase):
    def test_header_only_forged_huge_shape_never_allocates_or_reads_array(self):
        # No array payload exists. A loader attempting data access cannot succeed.
        raw = repr({"descr": "<f4", "fortran_order": False, "shape": (1000000000, 288)}).encode("ascii") + b"\n"
        npy = b"\x93NUMPY\x01\x00" + struct.pack("<H", len(raw)) + raw
        with tempfile.TemporaryDirectory(dir="/tmp/codex_occ") as folder:
            file = Path(folder) / "header_only.npz"
            with zipfile.ZipFile(file, "w") as z:
                z.writestr("lidar.npy", npy)
            with mock.patch.object(np, "load", side_effect=AssertionError("array loader forbidden")):
                result = inventory.npz_headers(file)
            self.assertEqual(result["arrays"]["lidar"]["shape"], [1000000000, 288])
            self.assertFalse(result["array_payload_read"])
            self.assertEqual(result["arrays"]["lidar"]["header_sha256"], hashlib.sha256(raw).hexdigest())

    def test_bad_headers_fail_and_missing_raw_is_not_promoted(self):
        with tempfile.TemporaryDirectory(dir="/tmp/codex_occ") as folder:
            file = Path(folder) / "bad.npz"
            with zipfile.ZipFile(file, "w") as z:
                z.writestr("x.npy", b"not-npy")
            with self.assertRaises(ValueError):
                inventory.npz_headers(file)
            result = inventory.build_inventory(Path(folder) / "missing")
            self.assertEqual(result["p03_trace"]["status"], "MISSING")
            self.assertFalse(result["payload_integrity_verified"])


if __name__ == "__main__":
    unittest.main()
