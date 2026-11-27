"""S-L1D runtime CPU gates against the committed float64 oracle and S-L1C source 4d5911d."""
import ast
import importlib.util
import inspect
import math
from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
AUDITS = ROOT / "tools/audits"
if str(AUDITS) not in sys.path:
    sys.path.insert(0, str(AUDITS))
import eq1_common as common  # noqa: E402
import s_l1d_reference as oracle  # noqa: E402

PATH = ROOT / "aerial_gym/task/navrl_task"
S_L1C_SOURCE = "4d5911d3d2fcbf0a285f395e38428dc39ef67868"
TIGHT = np.eye(3) * 1e-6
FX, FY = 160 / (2 * math.tan(math.radians(43.5))), 90 / (2 * math.tan(math.radians(29.)))


def helper():
    spec = importlib.util.spec_from_file_location("_s_l1d_runtime_helper", PATH / "navrl_sensor_lineage.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor = helper()


def old_source(name):
    return subprocess.check_output(["git", "show", S_L1C_SOURCE + ":aerial_gym/task/navrl_task/" + name], cwd=ROOT, text=True)


CASES = [([], [], [5., 0., 0.]), ([[3.3, 0., .5]], [[.3, .6, 1.5]], [6., 0., 0.]),
         ([[2.3, .7, .5]], [[.3, .7, 1.5]], [4., .05, 0.]), ([[2.3, .74, .5]], [[.3, .7, 1.5]], [4., 0., 0.]),
         ([], [], [3.214, 3.830, 0.]), ([], [], [24., 0., 0.]), ([], [], [16., 0., 0.]),
         ([[4.0, 0., .5]], [[.05, .6, 1.5]], [4.4, 0., 0.]), ([], [], [3., 0., -.5]), ([[2.6, -.4, .5]], [[.3, .3, 1.5]], [3., 0., 0.])]


def run_helper(depth, centre, cov, detected=False, active=True):
    rays = sensor.camera_unit_rays(160, 90, FX, FY, 79.5, 44.5)
    out = sensor.measurement_validity(
        torch.from_numpy(np.asarray(depth, dtype=np.float32)).view(1, 90, 160), torch.tensor([detected]), torch.tensor([active]),
        torch.tensor([centre], dtype=torch.float32), torch.from_numpy(np.asarray(cov, dtype=np.float32)).view(1, 3, 3),
        rays, torch.tensor([.10, 0., .03]), target_radius=.15, camera_far_m=20., min_target_pixels=2,
        hfov=math.radians(87.), vfov=math.radians(58.), fx=FX, fy=FY, cx=79.5, cy=44.5)
    return {k: v[0] for k, v in out.items()}


class HelperVersusOracleTest(unittest.TestCase):
    def test_switch_levels(self):
        self.assertEqual(sensor.sensor_lineage_level({sensor.SWITCH: "s_l1d_v1"}), 4)
        with self.assertRaises(ValueError):
            sensor.sensor_lineage_level({sensor.SWITCH: "s_l1e_v1"})

    def test_classes_and_support_equal_oracle(self):
        for boxes, halves, target in CASES:
            for cov in (TIGHT, np.eye(3) * .01):
                with self.subTest(target=target, cov=float(cov[0, 0])):
                    ev = oracle.evaluator_class(boxes, halves, target)
                    depth = oracle.delivered_depth(ev).astype(np.float32).astype(np.float64)
                    expected = oracle.online_class(depth, False, True, target, cov)
                    actual = run_helper(depth, target, cov)
                    self.assertEqual(oracle.ONLINE[int(actual["code"])], expected["class"], (expected, ev["class"]))
                    self.assertEqual(int(actual["support_px"]), expected["support"])
                    self.assertEqual(int(actual["occluded_px"]), expected["occluded"])
                    self.assertAlmostEqual(float(actual["tau_m"]), expected["tau_m"], places=5)

    def test_precedence(self):
        depth = np.full((90, 160), 10.)
        self.assertEqual(oracle.ONLINE[int(run_helper(depth, [5., 0., 0.], TIGHT, detected=True)["code"])], "DETECTED")
        self.assertEqual(oracle.ONLINE[int(run_helper(depth, [5., 0., 0.], TIGHT, active=False)["code"])], "NO_TRACK")


class RealPerceptionTest(unittest.TestCase):
    def test_s_l1d_is_diagnostic_only_and_equals_s_l1c_observation(self):
        ev = oracle.evaluator_class([[3.3, 0., .5]], [[.3, .6, 1.5]], [6., 0., 0.])
        depth = torch.from_numpy(oracle.delivered_depth(ev).astype(np.float32)).view(1, 90, 160)
        lidar = torch.full((1, 288), 12.)
        args = (torch.zeros(1, 3, 90, 160), depth, lidar, torch.zeros(1, 3), torch.zeros(1, 3),
                torch.tensor([[0., 0., 0., 1.]]), torch.zeros(1), torch.zeros(1, 4))
        results = {}
        for env in ("s_l1c_v1", "s_l1d_v1"):
            with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)), torch.random.fork_rng(devices=[]):
                torch.manual_seed(1)
                current, _, _ = common.build_perception(1)
                tr = current.tracker
                tr.state[0, :3] = torch.tensor([6., 0., 0.])
                tr.cov[0] = torch.eye(6) * 1e-6
                tr.active[0] = True
                tr.age[0] = 1.
                torch.manual_seed(2)
                obs, diag = current.observe(*args, 3.5, 1.5, training=False)
                results[env] = (obs, tr.state.clone(), tr.cov.clone(), diag)
        a, b = results["s_l1c_v1"], results["s_l1d_v1"]
        for x, y in zip(a[:3], b[:3]):
            self.assertTrue(torch.equal(x, y))
        self.assertNotIn("s_l1d_online_class", a[3])
        self.assertIn("s_l1d_online_class", b[3])
        self.assertEqual(oracle.ONLINE[int(b[3]["s_l1d_online_class"][0])], "OCCLUSION_FULL_EVIDENCE")


class RegressionTest(unittest.TestCase):
    def test_lower_lineages_observe_bitwise_vs_4d5911d(self):
        ev = oracle.evaluator_class([[2.3, .74, .5]], [[.3, .7, 1.5]], [4., 0., 0.])
        depth = torch.from_numpy(oracle.delivered_depth(ev).astype(np.float32)).view(1, 90, 160)
        lidar = torch.full((1, 288), 12.)
        lidar.view(1, 4, 72)[0, 2, 36] = 3.8
        args = (torch.zeros(1, 3, 90, 160), depth, lidar, torch.zeros(1, 3), torch.zeros(1, 3),
                torch.tensor([[0., 0., 0., 1.]]), torch.zeros(1), torch.zeros(1, 4))
        for env in ("off", "s_l1a_v1", "s_l1b_v1", "s_l1c_v1"):
            with self.subTest(env=env), common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)):
                with torch.random.fork_rng(devices=[]):
                    torch.manual_seed(3)
                    current, config, _ = common.build_perception(1)
                    namespace = {"__name__": "_s_l1d_measured_perception", "__file__": str(PATH / "navrl_perception.py")}
                    exec(compile(old_source("navrl_perception.py"), "s_l1c:navrl_perception.py", "exec"), namespace)
                    old = namespace["NavRLPerceptionModule"](1, torch.device("cpu"), config.perception, .1, config.vision)
                self.assertFalse(current.sensor_lineage_s_l1d)
                outs = []
                for module in (current, old):
                    module.reset_idx(torch.arange(1))
                    module.tracker.state[0, :3] = torch.tensor([4., 0., 0.])
                    module.tracker.cov[0] = torch.eye(6) * .01
                    module.tracker.active[0] = True
                    module.tracker.age[0] = 1.
                    with torch.random.fork_rng(devices=[]):
                        torch.manual_seed(5)
                        obs, diag = module.observe(*args, 3.5, 1.5, training=False)
                    outs.append((obs, module.tracker.state.clone(), module.tracker.cov.clone(), module.tracker.age.clone()))
                for x, y in zip(*outs):
                    self.assertTrue(torch.equal(x, y))


class FirewallTest(unittest.TestCase):
    def test_validity_helper_reads_no_ground_truth(self):
        forbidden = ("target_position", "semantic", "segmentation", "ground_truth", "simulator", "el0", "mask", "drone_pos")
        for param in inspect.signature(sensor.measurement_validity).parameters:
            self.assertFalse(any(w in param.lower() for w in forbidden), param)
        tree = ast.parse((PATH / "navrl_perception.py").read_text())
        fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_s_l1d_validity")
        names = {n.attr for n in ast.walk(fn) if isinstance(n, ast.Attribute)}
        self.assertFalse(names & {"target_position", "target_mask", "segmentation", "silhouette_from_task"})


if __name__ == "__main__":
    unittest.main()
