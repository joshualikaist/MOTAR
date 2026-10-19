"""Focused U4 CPU checks; raw-scan reference executes only in an isolated subprocess."""
import ast
import cmath
from dataclasses import asdict, replace
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest

import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.boia import dynamics as dyn  # noqa: E402
from motar_eval.boia.types import BoiaParams  # noqa: E402

assert str(Path(dyn.__file__).resolve()).startswith(str(ROOT) + "/")
REFERENCE = ROOT / "aerial_gym/task/navrl_task/speed_governor.py"


def fixture(n=1, k=1, **kwargs):
    cmd = torch.zeros(n, k, 4, dtype=torch.float64)
    p = torch.zeros(n, 3, dtype=torch.float64)
    p[:, 2] = 1.
    v, yaw = torch.zeros_like(p), torch.zeros(n, dtype=p.dtype)
    points = torch.empty(n, 0, 2, dtype=p.dtype)
    valid = torch.empty(n, 0, dtype=torch.bool)
    params = BoiaParams(tau_on=.1, r_bar_est=.2)
    return cmd, p, v, yaw, points, valid, dyn.RiskcapConfig(), params


def call_reference(payload):
    """No package import: upstream file's imports are checked before loading the pure file."""
    child = r'''
import ast, importlib.util, json, sys
import torch
data = json.load(sys.stdin)
path = data["source"]
tree = ast.parse(open(path).read())
for node in ast.walk(tree):
    if isinstance(node, ast.Import):
        assert all(a.name in {"math", "torch"} for a in node.names)
    if isinstance(node, ast.ImportFrom):
        assert node.level == 0 and node.module == "dataclasses"
assert "isaacgym" not in sys.modules and "aerial_gym" not in sys.modules
spec = importlib.util.spec_from_file_location("boia_governor_cpu_reference", path)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)
dtype = getattr(torch, data["dtype"])
t = lambda x: torch.tensor(x, dtype=dtype)
mask = None if data["mask"] is None else torch.tensor(data["mask"], dtype=torch.bool)
width = t(data["width"]) if isinstance(data["width"], list) else data["width"]
clear = module.directional_lidar_clearance(t(data["scan"]), t(data["angles"]), t(data["cmd"]),
    max_range_m=12., path_half_width_m=width, target_return_mask=mask)
cfg = module.SpeedGovernorConfig(**{k:v for k,v in data["config"].items() if k in module.SpeedGovernorConfig.__dataclass_fields__})
governed, diag = module.apply_speed_governor(t(data["cmd"]), t(data["clearance"]), cfg)
assert "isaacgym" not in sys.modules and "aerial_gym" not in sys.modules
print(json.dumps({"clear": clear.tolist(), "governed": governed.tolist(),
    "diag": {k:v.tolist() for k,v in diag.items()}, "source": module.__file__,
    "simulator_imported": False}))
'''
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1")
    result = subprocess.run([sys.executable, "-B", "-c", child], input=json.dumps(payload),
                            text=True, capture_output=True, env=env, cwd=str(ROOT), timeout=30)
    if result.returncode:
        raise AssertionError(result.stderr)
    return json.loads(result.stdout)


class GovernorTest(unittest.TestCase):
    def test_original_normal_import_chain_is_not_executed(self):
        tree = ast.parse((ROOT / "aerial_gym/__init__.py").read_text())
        self.assertTrue(any(isinstance(x, ast.Import) and any(a.name == "isaacgym" for a in x.names) for x in tree.body))
        self.assertNotIn("aerial_gym", dyn.__dict__)

    def test_raw_corridor_and_riskcap_bitwise_against_original_subprocess(self):
        gen = torch.Generator(device="cpu")
        gen.manual_seed(20261019)
        for dtype in (torch.float32, torch.float64):
            scan = torch.rand(13, 4, 72, generator=gen, dtype=dtype) * 13
            scan[0, 0, :5] = torch.tensor([0., -1., float("nan"), float("inf"), .995 * 12], dtype=dtype)
            cmd = torch.randn(13, 2, generator=gen, dtype=dtype) * 5
            cmd[0], cmd[1] = 0, 1e-8
            ang = torch.linspace(math.pi, -math.pi + 2 * math.pi / 72, 72, dtype=dtype)
            clearance = torch.tensor([0., 2.9999999, 3., 4., 5., 5.0000001, 12., .1, 2., 6., 9., 4.5, 3.5], dtype=dtype)
            mask = torch.rand(13, 4, 72, generator=gen) < .2
            cfg = dyn.RiskcapConfig()
            for use_mask, width in ((None, .45), (mask, torch.linspace(.2, .9, 13, dtype=dtype))):
                payload = {"source": str(REFERENCE), "dtype": str(dtype).split(".")[-1],
                           "scan": scan.tolist(), "cmd": cmd.tolist(), "angles": ang.tolist(),
                           "clearance": clearance.tolist(), "config": asdict(cfg),
                           "mask": None if use_mask is None else use_mask.tolist(),
                           "width": width.tolist() if isinstance(width, torch.Tensor) else width}
                before = torch.get_rng_state().clone()
                original = call_reference(payload)
                mirrored = dyn.directional_lidar_clearance(scan, ang, cmd, max_range_m=12.,
                              path_half_width_m=width, target_return_mask=use_mask)
                governed, diag = dyn.apply_riskcap(cmd, clearance, cfg)
                self.assertTrue(torch.equal(mirrored, torch.tensor(original["clear"], dtype=dtype)))
                self.assertTrue(torch.equal(governed, torch.tensor(original["governed"], dtype=dtype)))
                self.assertEqual(set(diag), set(original["diag"]))
                for key in diag:
                    self.assertTrue(torch.equal(diag[key], torch.tensor(original["diag"][key], dtype=dtype)), key)
                self.assertEqual(original["source"], str(REFERENCE))
                self.assertFalse(original["simulator_imported"])
                self.assertTrue(torch.equal(before, torch.get_rng_state()))

    def test_policy_f_values_are_declared_and_riskcap_does_not_force_braking(self):
        cfg = dyn.RiskcapConfig()
        self.assertEqual(cfg.free_speed_cap_mps, 3.53553390593)
        self.assertEqual(cfg.brake_mps2, 2.96)
        cmd = torch.tensor([[3., 0.], [.5, 0.], [0., 0.]], dtype=torch.float64)
        governed, diag = dyn.apply_riskcap(cmd, torch.zeros(3, dtype=cmd.dtype), cfg)
        torch.testing.assert_close(governed[:, 0], torch.tensor([2., .5, 0.], dtype=cmd.dtype), rtol=0, atol=0)
        self.assertEqual(float(diag["speed_cap_mps"][0]), 2.)

    def test_moved_pose_point_corridor_is_explicitly_geometry_not_new_scan(self):
        points = torch.tensor([[[4., 0.], [0., 2.]]], dtype=torch.float64)
        valid = torch.ones(1, 2, dtype=torch.bool)
        p = torch.tensor([[[0., 0., 1.], [1., 0., 1.], [0., 0., 1.]]], dtype=torch.float64)
        yaw = torch.tensor([[0., 0., math.pi / 2]], dtype=torch.float64)
        cmd = torch.tensor([[[1., 0.], [1., 0.], [1., 0.]]], dtype=torch.float64)
        result = dyn.point_corridor_clearance(points, valid, p, yaw, cmd, dyn.RiskcapConfig())
        torch.testing.assert_close(result, torch.tensor([[4., 3., 2.]], dtype=torch.float64))


class RolloutTest(unittest.TestCase):
    def test_current_yaw_rerotation_matches_independent_complex_closed_form(self):
        args = list(fixture())
        args[0][0, 0, 0] = .4  # 1 m/s vehicle-frame command.
        args[0][0, 0, 3] = .5 / 3
        h, sub, dt = 20, 10, .1
        result = dyn.rollout(*args, H=h, S=sub, dt=dt)
        delta, steps = dt / sub, h * sub
        decay, forcing = 1 - delta / .4, delta / .4
        rotation = cmath.exp(1j * delta * .5)
        vel = forcing * (rotation ** steps - decay ** steps) / (rotation - decay)
        sum_rotation = rotation * (1 - rotation ** steps) / (1 - rotation)
        sum_decay = decay * (1 - decay ** steps) / (1 - decay)
        pos = delta * forcing * (sum_rotation - sum_decay) / (rotation - decay)
        torch.testing.assert_close(result.step_pos[0, 0, -1, :2], torch.tensor([pos.real, pos.imag], dtype=torch.float64), atol=2e-13, rtol=0)
        torch.testing.assert_close(result.step_vel[0, 0, -1, :2], torch.tensor([vel.real, vel.imag], dtype=torch.float64), atol=2e-13, rtol=0)
        self.assertAlmostEqual(float(result.step_psi[0, 0, -1]), 1., places=13)
        self.assertGreater(float(result.step_pos[0, 0, -1, 1]), .3)

    def test_clip_acceleration_bound_and_altitude_zero_authority(self):
        args = list(fixture(n=2, k=3))
        args[0][..., :2] = torch.tensor([9., -9.])
        args[0][..., 2] = 9.
        args[0][..., 3] = 9.
        args[2][:, :2], args[2][:, 2] = -3., 4.
        original = [x.clone() for x in args[:6]]
        out = dyn.rollout(*args, H=3, S=10, dt=.1)
        self.assertEqual(tuple(out.pos.shape), (2, 3, 3, 10, 3))
        self.assertEqual(tuple(out.psi.shape), (2, 3, 3, 10))
        self.assertTrue(bool((out.pos[..., 2] == 1.).all()))
        self.assertTrue(bool((out.vel[..., 2] == 0.).all()))
        self.assertTrue(bool((out.governed_cmd[..., 2] == 0.).all()))
        self.assertTrue(bool((out.governed_cmd[..., 3] == 3.).all()))
        self.assertTrue(bool((out.diagnostics["clipped_action"].abs() <= 1).all()))
        accel = (out.diagnostics["acceleration_xy"] ** 2).sum(-1).sqrt()
        self.assertLessEqual(float(accel.max()), 9.81 + 1e-12)
        args[0][..., 2] = -9.
        other = dyn.rollout(*args, H=3, S=10, dt=.1)
        self.assertTrue(torch.equal(out.pos, other.pos))
        self.assertTrue(torch.equal(out.vel, other.vel))
        for before, after in zip(original[1:], args[1:6]):
            self.assertTrue(torch.equal(before, after))

    def test_prior_braking_target_miss_is_limited_validity_not_retuned(self):
        args = list(fixture(n=3))
        speeds = torch.tensor([2., 2.5, 3.52], dtype=torch.float64)
        args[2][:, 0] = speeds
        # 80 decisions = 8 s / 800 substeps: compare to the converged discrete sum,
        # so the table disagreement is not an artifact of the normal 1 s horizon.
        result = dyn.rollout(*args, H=80, S=10, dt=.1)
        distance = result.step_pos[:, 0, -1, 0]
        exact_discrete = speeds * (.4 - .01)
        torch.testing.assert_close(distance, exact_discrete, rtol=0, atol=3e-9)
        reference = torch.tensor([.82, 1.05, 1.57], dtype=distance.dtype)
        errors = (distance - reference).abs() / reference
        self.assertTrue(bool((errors <= .25).all()))  # User-approved hard rejection threshold.
        self.assertEqual((errors > .10).tolist(), [False, False, True])
        self.assertEqual(args[-1].tau_v, .4)
        self.assertAlmostEqual(float(errors[-1]), .1256050955414, places=8)

    def test_yaw_lag_requires_explicit_rate_and_matches_scalar_solution(self):
        args = list(fixture())
        args[-1] = replace(args[-1], tau_psi=.1)
        args[0][0, 0, 3] = 1 / 3
        with self.assertRaises(ValueError):
            dyn.rollout(*args, H=1, S=10, dt=.1)
        out = dyn.rollout(*args, H=1, S=10, dt=.1, initial_yaw_rate=torch.tensor([.4], dtype=torch.float64))
        decay, steps, delta = .9, 10, .01
        expected = delta * (steps + (.4 - 1) * decay * (1 - decay ** steps) / (1 - decay))
        self.assertAlmostEqual(float(out.step_psi[0, 0, -1]), expected, places=14)

    def test_rollout_determinism_batch_and_rng_noninterference(self):
        args = fixture(n=2, k=2)
        before = torch.get_rng_state().clone()
        a, b = dyn.rollout(*args, H=2, S=10, dt=.1), dyn.rollout(*args, H=2, S=10, dt=.1)
        self.assertTrue(torch.equal(a.pos, b.pos))
        self.assertTrue(torch.equal(a.vel, b.vel))
        self.assertTrue(torch.equal(before, torch.get_rng_state()))
        self.assertTrue(bool(torch.isfinite(a.pos).all()))
        self.assertTrue(torch.equal(a.step_pos, a.pos[:, :, :, -1]))


class BrakingTableTest(unittest.TestCase):
    def test_knots_interpolation_and_reaction_exactly_once(self):
        v = torch.tensor([0., 1., 2., 2.25, 2.5, 3.52, 4.], dtype=torch.float64)
        no_reaction, warning = dyn.braking_distance(v, t_react=0.)
        with_reaction, _ = dyn.braking_distance(v, t_react=.1)
        expected = torch.tensor([0., .205, .82, .935, 1.05, 1.57, 1.57 + (16 - 3.52 ** 2) * .205], dtype=torch.float64)
        torch.testing.assert_close(no_reaction, expected, rtol=0, atol=5e-16)
        torch.testing.assert_close(with_reaction - no_reaction, .1 * v, rtol=0, atol=3e-16)
        self.assertEqual(warning.tolist(), [False, False, False, False, False, False, True])

    def test_piecewise_extension_continuity_and_invalid_table(self):
        for knot in (2., 2.5, 3.52):
            values, _ = dyn.braking_distance(torch.tensor([knot - 1e-8, knot, knot + 1e-8], dtype=torch.float64), t_react=.1)
            self.assertLess(float(values.max() - values.min()), 1e-7)
        with self.assertRaises(ValueError):
            dyn.braking_distance(torch.tensor([2.]), table=((2., .8), (1., .5)), t_react=.1)
        with self.assertRaises(AssertionError):
            dyn.braking_distance(torch.tensor([-1.]), t_react=.1)


if __name__ == "__main__":
    unittest.main()
