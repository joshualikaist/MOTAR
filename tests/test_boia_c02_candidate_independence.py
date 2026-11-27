"""G4-R3 candidate isolation: exact single/batch invariance, not flight evidence.

Fixtures use independent scalar U4 recurrences from the previous new test file.
Expected healthy status follows an open-room/clear-point analytic fixture; single
calls are used only for the invariance contract, not as a collision oracle.
"""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
import test_boia_c02_safety as original  # noqa: E402
from motar_eval.boia import safety  # noqa: E402
from motar_eval.boia.types import FreeContext, Rollout, SafetyStatus  # noqa: E402

assert Path(original.__file__).resolve() == ROOT / "tests/test_boia_c02_safety.py"
assert Path(safety.__file__).resolve() == ROOT / "motar_eval/boia/safety.py"


def fixture(n=1, k=20, dtype=torch.float64):
    """Distinct common scans/points/frames and distinct, analytically safe candidates."""
    envs, origins, points = [], [], []
    for e in range(n):
        origin = (3. * e, -float(e))
        origins.append(list(origin) + [1.])
        points.append([[origin[0] + 1.7 + .2 * e, origin[1] + .75]])
        candidates = []
        for c in range(k):
            speed = .2 + .03 * c + .05 * e
            candidates.append(original.scalar_roll((speed, 0.), (speed, 0.), p0=origin))
        envs.append(Rollout(*(torch.cat([getattr(x, name) for x in candidates], 1)
                            for name in ("pos", "vel", "psi", "step_pos", "step_vel", "step_psi", "governed_cmd")),
                            {name: torch.cat([x.diagnostics[name] for x in candidates], 1)
                             for name in ("initial_pos", "initial_vel")}))
    roll = Rollout(*(torch.cat([getattr(x, name) for x in envs], 0).to(dtype)
                    for name in ("pos", "vel", "psi", "step_pos", "step_vel", "step_psi", "governed_cmd")),
                    {name: torch.cat([x.diagnostics[name] for x in envs], 0).to(dtype)
                     for name in ("initial_pos", "initial_vel")})
    ctx = original.context(n=n)
    radii = torch.tensor([[12. - 2. * e] * 72 for e in range(n)], dtype=dtype)
    ctx = FreeContext(torch.tensor(origins, dtype=dtype), torch.arange(n, dtype=dtype) * .1,
                      radii, ctx.angles.to(dtype), 12.)
    pts = torch.tensor(points, dtype=dtype)
    return roll, ctx, pts, torch.ones(n, 1, dtype=torch.bool)


def evaluate(data):
    roll, ctx, points, valid = data
    return safety.screen_next_cycle(roll, points, valid, ctx, original.params())


def select(data, env, candidate):
    roll, ctx, points, valid = data
    cut = lambda x: x[env:env + 1, candidate:candidate + 1].clone()
    one = Rollout(*(cut(x) for x in (roll.pos, roll.vel, roll.psi, roll.step_pos, roll.step_vel,
                                   roll.step_psi, roll.governed_cmd)),
                  {name: cut(value) for name, value in roll.diagnostics.items()})
    return one, FreeContext(ctx.pos[env:env + 1].clone(), ctx.yaw[env:env + 1].clone(),
                            ctx.free_radius[env:env + 1].clone(), ctx.angles.clone(), ctx.far_m), \
        points[env:env + 1].clone(), valid[env:env + 1].clone()


def permute(data, order, env_order=None):
    roll, ctx, points, valid = data
    if env_order is None:
        env_order = torch.arange(roll.pos.shape[0])
    change = lambda x: x.index_select(0, env_order).index_select(1, order)
    rr = Rollout(*(change(x) for x in (roll.pos, roll.vel, roll.psi, roll.step_pos, roll.step_vel,
                                     roll.step_psi, roll.governed_cmd)),
                 {name: change(value) for name, value in roll.diagnostics.items()})
    cc = FreeContext(ctx.pos[env_order], ctx.yaw[env_order], ctx.free_radius[env_order], ctx.angles, ctx.far_m)
    return rr, cc, points[env_order], valid[env_order]


def malformed(data, env, candidate, kind):
    roll, ctx, points, valid = data
    if kind == "position":
        roll.pos[env, candidate, 0, 0, 0] = float("nan")
    elif kind == "velocity":
        roll.vel[env, candidate, 0, 0, 0] = float("nan")
    elif kind == "diagnostic":
        roll.diagnostics["initial_vel"][env, candidate, 0] = float("inf")
    elif kind == "step":
        roll.step_pos[env, candidate, 0, 0] += .01
    elif kind == "timing":
        roll.pos[env, candidate, ..., :2] = (roll.pos[env, candidate, ..., :2]
            - roll.diagnostics["initial_pos"][env, candidate, :2]) * 2 \
            + roll.diagnostics["initial_pos"][env, candidate, :2]
        roll.step_pos[env, candidate] = roll.pos[env, candidate, :, -1]
    elif kind == "vertical":
        roll.vel[env, candidate, ..., 2] = .2
        roll.diagnostics["initial_vel"][env, candidate, 2] = .2
        steps = torch.arange(1, 101, dtype=roll.pos.dtype).reshape(10, 10)
        roll.pos[env, candidate, ..., 2] = 1. + steps * .01 * .2
        roll.step_pos[env, candidate] = roll.pos[env, candidate, :, -1]
        roll.step_vel[env, candidate] = roll.vel[env, candidate, :, -1]
    else:
        raise ValueError(kind)
    return data


def case(letter):
    if letter == "B" or letter == "E":
        data = fixture(3)
        bad = ((1, 5), (2, 2), (2, 11))
        for e, c in bad:
            malformed(data, e, c, "position")
    else:
        data = fixture()
        bad = ((0, 4 if letter == "D" else 9 if letter == "C" else 19),)
        malformed(data, *bad[0], "vertical" if letter == "D" else "timing" if letter == "C" else "position")
    expected = torch.full(data[0].pos.shape[:2], SafetyStatus.MODEL_SAFE, dtype=torch.int64)
    for e, c in bad:
        expected[e, c] = SafetyStatus.UNKNOWN if letter == "D" else SafetyStatus.INVALID
    return data, expected


class CandidateIndependenceTest(unittest.TestCase):
    def assert_same_candidate(self, result, one, e, c):
        for name in ("status", "prefix_ok", "backup_ok", "legacy_hard_ok", "legacy_stop_ok",
                     "prefix_min_clearance_m", "backup_min_clearance_m", "backup_distance_m"):
            torch.testing.assert_close(getattr(result, name)[e, c], getattr(one, name)[0, 0], rtol=0, atol=0,
                                       equal_nan=True, msg=name)

    def test_case_a_one_nan_candidate_preserves_nineteen_healthy_candidates(self):
        for field in ("position", "velocity"):
            with self.subTest(field=field):
                data = malformed(fixture(), 0, 19, field)
                out = evaluate(data)
                self.assertEqual(out.status[0, :19].tolist(), [SafetyStatus.MODEL_SAFE] * 19)
                self.assertEqual(int(out.status[0, 19]), SafetyStatus.INVALID)
                self.assert_same_candidate(out, evaluate(select(data, 0, 7)), 0, 7)

    def test_case_b_three_environments_isolate_exactly_three_bad_candidates(self):
        data, expected = case("B")
        out = evaluate(data)
        self.assertTrue(torch.equal(out.status, expected))
        self.assertEqual((out.status == SafetyStatus.MODEL_SAFE).sum(-1).tolist(), [20, 19, 18])
        for e, c in ((0, 5), (1, 4), (2, 12)):
            self.assert_same_candidate(out, evaluate(select(data, e, c)), e, c)

    def test_case_c_timing_and_residual_are_candidate_shaped(self):
        data, expected = case("C")
        out = evaluate(data)
        self.assertTrue(torch.equal(out.status, expected))
        check = out.diagnostics["candidate_validation"]
        self.assertEqual(check["timing_ok"].shape, (1, 20))
        self.assertEqual(check["timing_ok"][0].tolist(), [True] * 9 + [False] + [True] * 10)
        self.assertGreater(float(check["timing_max_residual_m"][0, 9]), .004)
        self.assert_same_candidate(out, evaluate(select(data, 0, 8)), 0, 8)

    def test_case_d_vertical_candidate_is_unknown_without_rejecting_others(self):
        data, expected = case("D")
        out = evaluate(data)
        self.assertTrue(torch.equal(out.status, expected))
        check = out.diagnostics["candidate_validation"]
        self.assertTrue(bool(check["timing_ok"].all()))
        self.assertFalse(bool(check["planar"][0, 4]))
        self.assertTrue(bool(check["planar"][0, :4].all()))
        self.assertTrue(bool(out.prefix_ok[0, :4].all() & out.backup_ok[0, :4].all()))

    def test_case_e_candidate_and_environment_permutations_preserve_results(self):
        data, expected = case("E")
        base = evaluate(data)
        self.assertTrue(torch.equal(base.status, expected))
        order = torch.tensor(list(range(19, -1, -2)) + list(range(18, -1, -2)))
        env_order = torch.tensor([2, 0, 1])
        out = evaluate(permute(data, order, env_order))
        inverse, env_inverse = torch.argsort(order), torch.argsort(env_order)
        restore = lambda x: x.index_select(0, env_inverse).index_select(1, inverse)
        for name in ("status", "prefix_ok", "backup_ok", "legacy_hard_ok", "legacy_stop_ok",
                     "prefix_min_clearance_m", "backup_min_clearance_m", "backup_distance_m"):
            torch.testing.assert_close(restore(getattr(out, name)), getattr(base, name), rtol=0, atol=0,
                                       equal_nan=True, msg=name)
        for name, value in base.diagnostics["candidate_validation"].items():
            torch.testing.assert_close(restore(out.diagnostics["candidate_validation"][name]), value,
                                       rtol=0, atol=0, equal_nan=True, msg=name)
        for name in ("backup_positions_m", "backup_velocities_mps", "backup_remaining_cap_m"):
            torch.testing.assert_close(restore(out.diagnostics["candidate_backup"][name]),
                                       base.diagnostics["candidate_backup"][name], rtol=0, atol=0,
                                       equal_nan=True, msg=name)

    def test_case_f_single_vs_mixed_batch_is_numerically_exact(self):
        data, expected = case("F")
        out = evaluate(data)
        self.assertTrue(torch.equal(out.status, expected))
        for c in (0, 3, 5, 18):
            one = evaluate(select(data, 0, c))
            self.assertEqual(int(one.status), SafetyStatus.MODEL_SAFE)
            self.assert_same_candidate(out, one, 0, c)
            torch.testing.assert_close(out.diagnostics["candidate_backup"]["backup_positions_m"][0, c],
                                       one.diagnostics["candidate_backup"]["backup_positions_m"][0, 0],
                                       rtol=0, atol=0)

    def test_common_scan_invalid_rejects_only_its_environment(self):
        data = fixture(3)
        data[1].free_radius[1, 7] = float("nan")
        out = evaluate(data)
        self.assertEqual(out.status[0].tolist(), [SafetyStatus.MODEL_SAFE] * 20)
        self.assertEqual(out.status[1].tolist(), [SafetyStatus.INVALID] * 20)
        self.assertEqual(out.status[2].tolist(), [SafetyStatus.MODEL_SAFE] * 20)
        check = out.diagnostics["candidate_validation"]
        self.assertEqual(check["common_valid"][:, 0].tolist(), [True, False, True])
        self.assertTrue(bool(check["finite"].all()))

    def test_candidate_diagnostics_and_step_consistency_faults_remain_local(self):
        for kind in ("diagnostic", "step"):
            with self.subTest(kind=kind):
                data = malformed(fixture(), 0, 13, kind)
                out = evaluate(data)
                self.assertEqual(int(out.status[0, 13]), SafetyStatus.INVALID)
                self.assertEqual(int(out.model_safe.sum()), 19)
                check = out.diagnostics["candidate_validation"]
                self.assertEqual(check["finite"].shape, (1, 20))
                self.assertEqual(check["step_end_consistent"].shape, (1, 20))
                self.assertEqual(check["status"].shape, (1, 20))
                self.assertEqual(check["reason"].shape, (1, 20))

    def test_all_consumed_candidate_fields_isolate_nonfinite_or_step_faults(self):
        fields = ("initial_pos", "initial_vel", "psi", "step_pos", "step_vel", "step_psi", "governed_cmd")
        for field in fields:
            with self.subTest(field=field):
                data = fixture()
                roll = data[0]
                tensor = roll.diagnostics[field] if field.startswith("initial_") else getattr(roll, field)
                tensor[0, 7].reshape(-1)[0] = float("inf")
                out = evaluate(data)
                self.assertEqual(int(out.status[0, 7]), SafetyStatus.INVALID)
                self.assertEqual(int(out.model_safe.sum()), 19)
                self.assertFalse(bool(out.diagnostics["candidate_validation"]["finite"][0, 7]))
                self.assert_same_candidate(out, evaluate(select(data, 0, 8)), 0, 8)
        data = fixture()
        data[0].step_psi[0, 7, 0] += .1
        out = evaluate(data)
        self.assertEqual(int(out.model_safe.sum()), 19)
        self.assertFalse(bool(out.diagnostics["candidate_validation"]["step_end_consistent"][0, 7]))

    def test_asymmetric_scan_binding_has_independently_different_outcomes(self):
        data = fixture(3, 4)
        roll, ctx, points, valid = data
        # Move trajectories one metre along world x from each scan origin. The
        # +x notch in env1 cuts through that position; the same beam in env2 is
        # rotated by pi and is behind the vehicle. Correct scan/yaw binding is
        # observable as UNKNOWN only in env1, independently of point distances.
        roll.pos[..., 0] += 1.
        roll.step_pos[..., 0] += 1.
        roll.diagnostics["initial_pos"][..., 0] += 1.
        ctx.yaw[:] = torch.tensor([.3, 0., torch.pi], dtype=ctx.yaw.dtype)
        ctx.free_radius[1, 36] = .8
        ctx.free_radius[2, 36] = .8
        out = evaluate(data)
        self.assertEqual(out.status[0].tolist(), [SafetyStatus.MODEL_SAFE] * 4)
        self.assertEqual(out.status[1].tolist(), [SafetyStatus.UNKNOWN] * 4)
        self.assertEqual(out.status[2].tolist(), [SafetyStatus.MODEL_SAFE] * 4)
        for e in range(3):
            for c in range(4):
                self.assert_same_candidate(out, evaluate(select(data, e, c)), e, c)
        order, env_order = torch.tensor([2, 0, 3, 1]), torch.tensor([2, 0, 1])
        changed = evaluate(permute(data, order, env_order))
        for name in ("status", "prefix_min_clearance_m", "backup_min_clearance_m", "backup_distance_m"):
            restored = getattr(changed, name)[torch.argsort(env_order)][:, torch.argsort(order)]
            torch.testing.assert_close(restored, getattr(out, name), atol=0, rtol=0, equal_nan=True)

    def test_structural_schema_failures_return_global_invalid(self):
        for diagnostics in ([], "malformed"):
            data = fixture()
            malformed_roll = replace(data[0], diagnostics=diagnostics)
            out = evaluate((malformed_roll,) + data[1:])
            self.assertEqual(out.status.tolist(), [[SafetyStatus.INVALID] * 20])
        for valid in (None, [[True]]):
            roll, ctx, points, _ = fixture()
            out = safety.screen_next_cycle(roll, points, valid, ctx, original.params())
            self.assertEqual(out.status.tolist(), [[SafetyStatus.INVALID] * 20])

    def test_float32_candidate_isolation_matches_single_result(self):
        data = malformed(fixture(3, dtype=torch.float32), 1, 5, "velocity")
        out = evaluate(data)
        self.assertEqual(int(out.model_safe.sum()), 59)
        for e, c in ((0, 5), (1, 4), (2, 5)):
            self.assert_same_candidate(out, evaluate(select(data, e, c)), e, c)

    def test_repeatability_rng_and_input_preservation(self):
        data, _ = case("B")
        tensors = [data[0].pos, data[0].vel, data[0].diagnostics["initial_pos"],
                   data[0].diagnostics["initial_vel"], data[1].free_radius, data[2], data[3]]
        copies = [x.clone() for x in tensors]
        rng = torch.get_rng_state().clone()
        first, second = evaluate(data), evaluate(data)
        torch.testing.assert_close(first.status, second.status, rtol=0, atol=0)
        torch.testing.assert_close(first.backup_distance_m, second.backup_distance_m, rtol=0, atol=0, equal_nan=True)
        self.assertTrue(torch.equal(rng, torch.get_rng_state()))
        for before, after in zip(copies, tensors):
            torch.testing.assert_close(before, after, rtol=0, atol=0, equal_nan=True)


if __name__ == "__main__":
    unittest.main()
