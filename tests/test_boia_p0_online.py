"""Independent CPU audit characterizations at 13d5f195; no simulator/raw access.

Passing a characterization test records an exposed limitation, not qualification.
See docs/evidence/boia_g4_p0_2026-10-20/online_audit.json for classifications.
"""
from dataclasses import replace
import hashlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import access, arbitration, belief, candidates, cost, planner, trace, visibility
from tools.boia_replay import fixture_params, make_scene


def local_rng(seed=20261020):
    result = torch.Generator(device="cpu")
    result.manual_seed(seed)
    return result


def sphere_pixels(width, height, hfov, vfov, distance, radius):
    """Independent ray/sphere discriminant, pixel centres, on-axis sphere; no renderer."""
    fx, fy = width / (2 * math.tan(hfov / 2)), height / (2 * math.tan(vfov / 2))
    count = 0
    for v in range(height):
        z = (v - (height - 1) / 2) / fy
        for u in range(width):
            y = (u - (width - 1) / 2) / fx
            a = 1 + y * y + z * z
            count += distance * distance - a * (distance * distance - radius * radius) >= 0
    return count


class OnlineAdversarialAudit(unittest.TestCase):
    def test_o01_mixed_terminal_nan_blocks_unrelated_live_row(self):
        for container, field in (("task", "robot_position"), ("tracker", "state")):
            task, perception, action = make_scene("clear", 2, torch.float64)
            mask = torch.tensor([True, False])
            p = planner.BoiaPlanner(fixture_params())
            p.step(task, perception, action, mask, local_rng())
            target = task.obs_dict[field] if container == "task" else getattr(perception.tracker, field)
            target[0, 0] = float("nan")
            with self.assertRaisesRegex(ValueError, "must be finite"):
                p.step(task, perception, action, mask, local_rng())
            # All-terminal genuinely bypasses sensor reads, including invalid observations.
            result, decision = p.step(task, perception, action, torch.ones(2, dtype=torch.bool), local_rng())
            self.assertTrue(torch.equal(result, action))
            self.assertEqual(decision.trace["input_digest"].shape, (0, 8))

    def test_o02_other_terminal_mask_reassigns_live_env_samples(self):
        task, perception, action = make_scene("clear", 2, torch.float64)
        p = planner.BoiaPlanner(fixture_params())
        original = belief.sample
        paths = []

        def observe(*args, **kwargs):
            result = original(*args, **kwargs)
            paths.append(result[0].clone())
            return result

        with mock.patch.object(belief, "sample", side_effect=observe):
            _, both = p.step(task, perception, action, torch.tensor([False, False]), local_rng())
            _, one = p.step(task, perception, action, torch.tensor([True, False]), local_rng())
        # Same input digest for physical env1, but different MC samples after env0 is masked.
        self.assertEqual(both.trace["input_digest"][1].tolist(), one.trace["input_digest"][0].tolist())
        self.assertGreater(float((paths[0][1] - paths[1][0]).abs().max()), .01)
        self.assertEqual(one.trace["env_index"].tolist(), [1])

    def test_o03_detect_resolution_is_not_part_of_current_accessor(self):
        task, perception, action = make_scene("clear", 1, torch.float64)
        params = fixture_params()
        a = access.read_online(task, perception, action, params)
        perception.detect_fx = perception.fx * 2
        perception.detect_fy = perception.fy * 2
        perception.detect_width, perception.detect_height = 320, 180
        b = access.read_online(task, perception, action, params)
        self.assertEqual(trace.canonical_input(a, 0), trace.canonical_input(b, 0))
        # The independently ray-tested target crosses the actual 2-pixel detector gate.
        low = sphere_pixels(160, 90, a.cam.hfov, a.cam.vfov, 18., .15)
        high = sphere_pixels(320, 180, a.cam.hfov, a.cam.vfov, 18., .15)
        self.assertEqual(low, 0)
        self.assertEqual(high, 4)
        primary_area = math.pi * (b.cam.fx * .15 / 18.) ** 2
        self.assertLess(primary_area, 2)

    def test_o03_primary_gate_ignores_copied_camera_pixel_threshold(self):
        task, perception, action = make_scene("clear", 1, torch.float64)
        perception.min_pixels = 100
        params = replace(fixture_params(), unknown_blocks=False)
        inp = access.read_online(task, perception, action, params)
        camera = replace(inp.cam, camera_offset=(0., 0., 0.))
        position = torch.zeros(1, 1, 1, 3, dtype=torch.float64)
        yaw = torch.zeros(1, 1, 1, dtype=torch.float64)
        samples = torch.zeros(1, 1, 1, 6, dtype=torch.float64)
        samples[..., 0] = 6
        points = torch.zeros(1, 0, 2, dtype=torch.float64)
        valid = torch.zeros(1, 0, dtype=torch.bool)
        actual = visibility.forecast(position, yaw, samples, points, valid, None, camera, params)
        self.assertEqual(float(actual.item()), 1.)
        self.assertLess(math.pi * (camera.fx * .15 / 6.) ** 2, camera.min_pixels)

    def test_o04_active_and_fused_age_are_not_identity_or_timestamp(self):
        task, perception, action = make_scene("clear", 1, torch.float64)
        task._camera_visible_now.zero_()
        perception.tracker.age.zero_()  # A fused range correction may reset this.
        perception.tracker.active.fill_(True)
        inp = access.read_online(task, perception, action, fixture_params())
        state = arbitration.initial_state(1, action)
        state = replace(state, ever_detected=torch.ones(1, dtype=torch.bool),
                        miss_streak=torch.tensor([19]))
        result = arbitration.update_mode(state, inp, torch.ones(1, 1, 1), fixture_params())
        self.assertEqual(int(result.miss_streak.item()), 20)
        self.assertEqual(int(result.mode.item()), arbitration.Mode.LOST_PREDICTABLE)
        self.assertFalse(hasattr(inp, "measurement_timestamp"))
        self.assertFalse(hasattr(inp, "identity_confirmed"))

    def test_o05_candidate19_translates_and_intercept_never_decline_gain(self):
        task, perception, action = make_scene("clear", 1, torch.float64)
        params = fixture_params()
        inp = access.read_online(task, perception, action, params)
        cmd, _, _ = candidates.generate(action, inp, inp.mu, torch.empty(1, 0, 2, dtype=torch.float64),
                                         torch.empty(1, 0, dtype=torch.bool), params)
        self.assertAlmostEqual(float(torch.linalg.vector_norm(cmd[0, 19, :2]) * 2.5), .6)
        J = torch.arange(20, dtype=torch.float64)[None]
        J[0, 0], J[0, 2] = 10, -10
        safe = torch.ones_like(J, dtype=torch.bool)
        state = arbitration.initial_state(1, action)
        for mode in (arbitration.Mode.INTERCEPT, arbitration.Mode.NEVER_ACQUIRED):
            chosen, engaged, _, _ = arbitration.decide(J, safe, safe, cmd, action, torch.tensor([mode]), state, params)
            self.assertFalse(bool(engaged[0]))
            self.assertTrue(torch.equal(chosen, action))

    def test_expected_update_matches_independent_scalar_and_does_not_calibrate(self):
        prior = torch.eye(6, dtype=torch.float64)[None, None, None]
        r = torch.eye(3, dtype=torch.float64)[None] * .25
        probability = torch.tensor([[[.4]]], dtype=torch.float64)
        actual = belief.expected_cov(prior, probability, r)
        expected_pos = 1 - .4 * 1 / (1 + .25)
        self.assertAlmostEqual(float(actual[0, 0, 0, 0, 0]), expected_pos, places=14)
        self.assertEqual(float(actual[0, 0, 0, 3, 3]), 1.)
        self.assertTrue(torch.equal(belief.expected_cov(prior, probability * 0, r), prior))
        with self.assertRaisesRegex(ValueError, "positive definite"):
            cost._logdet_pd(torch.zeros(1, 3, 3, dtype=torch.float64))

    def test_o10_overwritten_vertical_action_changes_smoothness_cost(self):
        task, perception, action = make_scene("clear", 1, torch.float64)
        params = fixture_params()
        inp = access.read_online(task, perception, action, params)
        commands = torch.zeros(1, 2, 4, dtype=torch.float64)
        commands[0, 0, 2] = .9  # Overwritten by the altitude PI; other components identical.
        roll = SimpleNamespace(step_pos=torch.zeros(1, 2, 1, 3, dtype=torch.float64))
        samples = inp.mu[:, None, None].clone()
        probability = torch.ones(1, 2, 1, 1, dtype=torch.float64)
        covariance = inp.Sigma[:, None, None].expand(1, 2, 1, 6, 6).clone()
        feasible = torch.ones(1, 2, dtype=torch.bool)
        scores, terms = cost.evaluate(inp, roll, samples, probability, covariance, commands,
                                      torch.zeros(1, 4, dtype=torch.float64),
                                      torch.tensor([arbitration.Mode.PRE_OCCLUSION]),
                                      feasible, feasible, torch.full((1, 2, 1), float("inf")), params)
        self.assertAlmostEqual(float(terms["S"][0, 0]), .9 ** 2 / 4)
        self.assertEqual(float(terms["S"][0, 1]), 0.)
        # All physical/other modeled cost terms coincide; only the ignored channel differs.
        self.assertAlmostEqual(float(scores[0, 0] - scores[0, 1]), .05 * .9 ** 2 / 4)

    def test_sha_standard_vectors_and_trace_copy_remain_isolated(self):
        for payload in (b"", b"abc", b"a" * 56, b"a" * 64, b"a" * 1000):
            self.assertEqual(trace.sha256(payload), hashlib.sha256(payload).digest())
        task, perception, action = make_scene("clear", 1, torch.float64)
        _, decision = planner.BoiaPlanner(fixture_params()).step(task, perception, action,
                                                               torch.zeros(1, dtype=torch.bool), local_rng())
        before = decision.action.clone()
        decision.trace["action"][:] = 99
        self.assertTrue(torch.equal(before, decision.action))
        self.assertFalse(set(access.FORBIDDEN_ATTRS) & set(decision.trace))


if __name__ == "__main__":
    unittest.main()
