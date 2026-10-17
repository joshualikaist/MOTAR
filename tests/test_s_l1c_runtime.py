"""S-L1C runtime CPU gates against the committed float64 oracle and S-L1B source a1f97e9."""
import ast
import importlib.util
import inspect
import math
from pathlib import Path
import subprocess
import sys
import unittest
from unittest import mock

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
AUDITS = ROOT / "tools/audits"
if str(AUDITS) not in sys.path:
    sys.path.insert(0, str(AUDITS))
import eq1_common as common  # noqa: E402
import s_l1c_reference as oracle  # noqa: E402

PATH = ROOT / "aerial_gym/task/navrl_task"
S_L1B_SOURCE = "a1f97e93af57f97b9778e395ffa8d38f7365e989"
GATE = 0.35 + 2 * math.sqrt(0.03)
ELEV = torch.deg2rad(torch.tensor([20., 10., 0., -10.]))
ANGLES = torch.from_numpy(oracle.ANGLES).float()


def helper():
    spec = importlib.util.spec_from_file_location("_s_l1c_runtime_helper", PATH / "navrl_sensor_lineage.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor = helper()


def old_source(name):
    return subprocess.check_output(["git", "show", S_L1B_SOURCE + ":aerial_gym/task/navrl_task/" + name], cwd=ROOT, text=True)


def bar(x, y):
    return [[x, y, .5]], [[.3, .3, 1.5]]


def primed_in_front(centres, halves, beam_deg):
    a = math.radians(beam_deg)
    d = np.array([math.cos(a), math.sin(a), 0.])
    t = min(oracle.ref.first_hits(np.zeros(3), d[None], [c], [h], far_m=50.)["depth_m"][0] for c, h in zip(centres, halves))
    return (t + 0.15) * d


NEGATIVES = [(bar(5.3, 0), 0), (bar(3.12, 1.03), 20), (bar(8.3, 0), 0), (bar(0, 5.3), 90), (bar(-5.3, 0), 180),
             (bar(2.3, 0), 0), (bar(10.3, 0), 0)]
POSITIVES = [np.array([d * math.cos(math.radians(b)), d * math.sin(math.radians(b)), 0.])
             for d in (1.5, 3., 6., 9., 11.5) for b in (0., 20., -45.)]


def run_helper(scan, rel, camera=False, active=True):
    out = sensor.range_only_lidar_association(
        torch.from_numpy(np.asarray(scan, dtype=np.float32)).view(1, 4, 72), ANGLES, ELEV,
        torch.from_numpy(np.asarray([rel], dtype=np.float32)), torch.tensor([camera]), torch.tensor([active]),
        torch.tensor([GATE]), lidar_far_m=12.)
    return {k: v[0] for k, v in out.items()}


class HelperVersusOracleTest(unittest.TestCase):
    def test_switch_levels(self):
        self.assertEqual(sensor.sensor_lineage_level({sensor.SWITCH: "s_l1c_v1"}), 3)
        with self.assertRaises(ValueError):
            sensor.sensor_lineage_level({sensor.SWITCH: "s_l1z_v1"})
        # s_l1d_v1 was unknown when S-L1C was qualified; it is now the preregistered cumulative
        # successor (docs/plans/s_l1d_v1.json) and must rank above S-L1C.
        self.assertGreater(sensor.sensor_lineage_level({sensor.SWITCH: "s_l1d_v1"}), 3)

    def test_negatives_rejected_and_positives_accepted_like_oracle(self):
        cases = [(oracle.lidar_scan(*geom)[0], primed_in_front(*geom, beam)) for geom, beam in NEGATIVES]
        cases += [(oracle.lidar_scan(sphere_center=c)[0], c + off * c / np.linalg.norm(c))
                  for c in POSITIVES for off in (0., -.15, .15)]
        accepted = 0
        for scan, rel in cases:
            expected = oracle.associate(scan, rel, gate_m=GATE)
            actual = run_helper(scan, rel)
            self.assertEqual(bool(actual["valid"]), expected["valid"], (rel, expected))
            if expected["valid"]:
                accepted += 1
                self.assertAlmostEqual(float(actual["centre_range"]), expected["centre_range"], delta=1e-5)
                np.testing.assert_allclose(actual["measurement_vehicle"].double().numpy(), expected["measurement_point"], atol=2e-5)
        negatives = [run_helper(*c) for c in cases[:len(NEGATIVES)]]
        self.assertFalse(any(bool(n["valid"]) for n in negatives))
        self.assertTrue(all(bool(n["extended"]) for n in negatives))
        self.assertEqual(accepted, len(cases) - len(NEGATIVES))

    def test_range_only_covariance_matches_oracle(self):
        u = torch.tensor([[math.cos(.3) * math.cos(.7), math.cos(.3) * math.sin(.7), math.sin(.3)]])
        r = sensor.range_only_covariance(u)[0].double().numpy()
        un = u[0].double().numpy()
        expected = .08 ** 2 * np.outer(un, un) + 50. ** 2 * (np.eye(3) - np.outer(un, un))
        np.testing.assert_allclose(r, expected, rtol=1e-5, atol=1e-3)


class RealPerceptionTest(unittest.TestCase):
    def prime(self, current, rel, prior=.01):
        tr = current.tracker
        current.reset_idx(torch.arange(1))
        tr.state[0, :3] = torch.tensor(rel, dtype=torch.float32)
        tr.state[0, 3:] = 0.
        tr.cov[0] = torch.eye(6) * prior
        tr.active[0] = True
        tr.age[0] = 1.

    def associate(self, current, scan):
        return current._associate_lidar_target(torch.from_numpy(np.asarray(scan, dtype=np.float32)).view(1, 288),
                                               torch.zeros(1, 3), torch.tensor([[0., 0., 0., 1.]]),
                                               torch.zeros(1, dtype=torch.bool))

    def test_production_association_negatives_positives_and_covariance(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1c_v1")), torch.random.fork_rng(devices=[]):
            current, _, module = common.build_perception(1)
            self.assertTrue(current.sensor_lineage_s_l1c)
            for geom, beam in NEGATIVES:
                rel = primed_in_front(*geom, beam)
                self.prime(current, rel)
                valid, *_ = self.associate(current, oracle.lidar_scan(*geom)[0])
                self.assertFalse(bool(valid[0]), (geom, beam))
            for centre in POSITIVES:
                with self.subTest(centre=centre.tolist()):
                    self.prime(current, centre, prior=1.0)
                    before = current.tracker.cov[0, :3, :3].double().numpy().copy()
                    valid, conf, measured, bearing = self.associate(current, oracle.lidar_scan(sphere_center=centre)[0])
                    self.assertTrue(bool(valid[0]))
                    self.assertAlmostEqual(float(current.last_s_l1c_association["centre_range"][0]), np.linalg.norm(centre), delta=1e-4)
                    after = current.tracker.cov[0, :3, :3].double().numpy()
                    u, t1, t2 = oracle.orthonormal_basis(centre)
                    self.assertGreaterEqual((t1 @ after @ t1) / (t1 @ before @ t1), .99)
                    self.assertGreaterEqual((t2 @ after @ t2) / (t2 @ before @ t2), .99)
                    self.assertLess(u @ after @ u, u @ before @ u)
                    self.assertEqual(float(current.tracker.age[0]), 0.)  # fused-recency contract unchanged

    def test_identity_clock_and_update_source(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1c_v1")), torch.random.fork_rng(devices=[]):
            current, config, module = common.build_perception(1)
            centre = np.array([4., 0., 0.])
            scan = oracle.lidar_scan(sphere_center=centre)[0]
            lidar = torch.from_numpy(scan.astype(np.float32)).view(1, 288)
            depth = torch.full((1, 90, 160), 10.)
            sources, ages, track_ages = [], [], []
            for step, camera in enumerate((True, False, False, True, False)):
                det = (torch.tensor([[4., 0., 0.]]), torch.tensor([3.85]), torch.zeros(1), torch.tensor([camera]),
                       torch.tensor([.9 if camera else 0.]), torch.zeros(1, 90, 160, dtype=torch.bool))
                with mock.patch.object(current, "_detect_rgbd", return_value=det):
                    _, diag = current.observe(torch.zeros(1, 3, 90, 160), depth, lidar, torch.zeros(1, 3), torch.zeros(1, 3),
                                              torch.tensor([[0., 0., 0., 1.]]), torch.zeros(1), torch.zeros(1, 4),
                                              3.5, 1.5, training=False)
                sources.append(int(diag["s_l1c_update_source"][0]))
                ages.append(float(diag["s_l1c_identity_age"][0]))
                track_ages.append(float(diag["track_age"][0]))
            self.assertEqual(sources, [1, 2, 2, 1, 2])
            dt = current.step_dt
            np.testing.assert_allclose(ages, [0., dt, 2 * dt, 0., dt], atol=1e-6)
            self.assertTrue(all(a == 0. for a in track_ages))  # LiDAR accepts still refresh fused recency


class RegressionTest(unittest.TestCase):
    def construct_pair(self):
        current, config, module = common.build_perception(1)
        namespace = {"__name__": "_s_l1c_measured_perception", "__file__": str(PATH / "navrl_perception.py")}
        exec(compile(old_source("navrl_perception.py"), "s_l1b:navrl_perception.py", "exec"), namespace)
        old = namespace["NavRLPerceptionModule"](1, torch.device("cpu"), config.perception, .1, config.vision)
        return current, old

    def test_lower_lineages_association_and_observe_bitwise_vs_a1f97e9(self):
        geom, beam = NEGATIVES[0]
        scan = torch.from_numpy(oracle.lidar_scan(*geom)[0].astype(np.float32)).view(1, 288)
        rel = torch.tensor(primed_in_front(*geom, beam), dtype=torch.float32)
        for env in ("off", "s_l1a_v1", "s_l1b_v1"):
            with self.subTest(env=env), common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)), \
                    torch.random.fork_rng(devices=[]):
                current, old = self.construct_pair()
                self.assertFalse(current.sensor_lineage_s_l1c)
                outs = []
                for module in (current, old):
                    module.reset_idx(torch.arange(1))
                    module.tracker.state[0, :3] = rel
                    module.tracker.cov[0] = torch.eye(6) * .01
                    module.tracker.active[0] = True
                    module.tracker.age[0] = 1.
                    res = module._associate_lidar_target(scan, torch.zeros(1, 3), torch.tensor([[0., 0., 0., 1.]]),
                                                         torch.zeros(1, dtype=torch.bool))
                    outs.append((res, module.tracker.state.clone(), module.tracker.cov.clone(), module.tracker.age.clone()))
                for a, b in zip(outs[0][0], outs[1][0]):
                    self.assertTrue(torch.equal(a, b))
                for a, b in zip(outs[0][1:], outs[1][1:]):
                    self.assertTrue(torch.equal(a, b))
                self.assertTrue(bool(outs[0][0][0][0]))  # legacy association still accepts the bar (Known FAIL kept)


    def test_lower_lineages_observe_bitwise_vs_a1f97e9(self):
        centre = np.array([4., -.3, 0.])
        scan = torch.from_numpy(oracle.lidar_scan(*bar(6.3, 1.0), sphere_center=centre)[0].astype(np.float32)).view(1, 288)
        depth = torch.full((1, 90, 160), 10.)
        depth[0, 30:60, 20:40] = 3.1
        args = (torch.zeros(1, 3, 90, 160), depth, scan, torch.zeros(1, 3), torch.zeros(1, 3),
                torch.tensor([[0., 0., 0., 1.]]), torch.zeros(1), torch.zeros(1, 4))
        for env in ("off", "s_l1a_v1", "s_l1b_v1"):
            with self.subTest(env=env), common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)):
                with torch.random.fork_rng(devices=[]):
                    torch.manual_seed(3)
                    current, old = self.construct_pair()
                results = []
                for module in (current, old):
                    module.reset_idx(torch.arange(1))
                    module.tracker.state[0, :3] = torch.tensor(centre, dtype=torch.float32)
                    module.tracker.cov[0] = torch.eye(6) * .01
                    module.tracker.active[0] = True
                    module.tracker.age[0] = 1.
                    with torch.random.fork_rng(devices=[]):
                        torch.manual_seed(5)
                        obs, diag = module.observe(*args, 3.5, 1.5, training=False)
                    results.append((obs, module.tracker.state.clone(), module.tracker.cov.clone(), diag["lidar_visible"].clone()))
                for a, b in zip(*results):
                    self.assertTrue(torch.equal(a, b))
                self.assertNotIn("s_l1c_identity_age", diag)


class FirewallTest(unittest.TestCase):
    FORBIDDEN = ("target_position", "semantic", "segmentation", "ground_truth", "gt_", "simulator", "drone_pos")

    def test_s_l1c_helpers_accept_only_sensor_track_and_constants(self):
        for name in ("range_only_lidar_association", "range_only_covariance"):
            for param in inspect.signature(getattr(sensor, name)).parameters:
                self.assertFalse(any(w in param.lower() for w in self.FORBIDDEN), (name, param))
        tree = ast.parse((PATH / "navrl_sensor_lineage.py").read_text())
        self.assertEqual({a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}, {"math", "os", "torch"})


if __name__ == "__main__":
    unittest.main()
