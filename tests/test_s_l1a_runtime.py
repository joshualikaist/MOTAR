"""S-L1A runtime CPU gates against the separately committed NumPy oracle and old source.

No Warp/Isaac initialization. Detector tests execute its real Python methods with
a recording query stub; they qualify composition/wiring, not GPU ray accuracy.
"""
import ast
import importlib.util
import math
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

import numpy as np
import torch
import torch.nn.functional as F

ROOT = Path(__file__).resolve().parents[1]
AUDITS = ROOT / "tools/audits"
if str(AUDITS) not in sys.path:
    sys.path.insert(0, str(AUDITS))
import eq1_common as common
import s_l1a_reference as reference

PATH = ROOT / "aerial_gym/task/navrl_task"
LEGACY = "183a333bd01c0124a95ae379dba0a8509aebc999"


def helper():
    spec = importlib.util.spec_from_file_location("_s_l1a_runtime_helper", PATH / "navrl_sensor_lineage.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor = helper()


def old_source(name):
    return subprocess.check_output(["git", "show", LEGACY + ":aerial_gym/task/navrl_task/" + name], cwd=ROOT, text=True)


def construct_pair(n=2):
    current, config, module = common.build_perception(n)
    namespace = {"__name__": "_s_l1a_measured_perception", "__file__": str(PATH / "navrl_perception.py")}
    exec(compile(old_source("navrl_perception.py"), "legacy:navrl_perception.py", "exec"), namespace)
    old = namespace["NavRLPerceptionModule"](n, torch.device("cpu"), config.perception, .1, config.vision)
    return current, old, config, module


class SwitchTest(unittest.TestCase):
    def test_strict_opt_in(self):
        self.assertFalse(sensor.sensor_lineage_enabled({}))
        self.assertFalse(sensor.sensor_lineage_enabled({sensor.SWITCH: "off"}))
        self.assertTrue(sensor.sensor_lineage_enabled({sensor.SWITCH: "s_l1a_v1"}))
        for value in ("", "1", "on", "s_l1", "OFF", " s_l1a_v1", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                sensor.sensor_lineage_enabled({sensor.SWITCH: value})

    def test_fresh_configuration_guard(self):
        base = dict(common.POLICY_F_ENV, NAVRL_SENSOR_LINEAGE="s_l1a_v1")
        with common.production(base), torch.random.fork_rng(devices=[]):
            current, _, _ = common.build_perception(1)
            self.assertTrue(current.sensor_lineage_s_l1a)
            self.assertEqual(current.tracker.memory_s, 5.)
        for key, value in (("NAVRL_DETECTION_LATENCY_S", ".01"), ("NAVRL_TARGET_MASK_BACKFILL", "1"),
                           ("NAVRL_LATENCY_OBSTACLE_FIX", "predict"), ("NAVRL_LATENCY_OBSTACLE_FIX", "skip")):
            with self.subTest(key=key, value=value), common.production(dict(base, **{key: value})), \
                    torch.random.fork_rng(devices=[]), self.assertRaisesRegex(ValueError, "fresh camera"):
                common.build_perception(1)

    def test_empirical_latency_rejected_but_zero_latency_noise_not_disabled(self):
        base = dict(common.POLICY_F_ENV, NAVRL_SENSOR_LINEAGE="s_l1a_v1")
        with common.production(base), torch.random.fork_rng(devices=[]):
            _, config, module = common.build_perception(1)
            fake_cfg = SimpleNamespace(**{k: getattr(config.perception, k) for k in dir(config.perception)
                                          if not k.startswith("_")})
            fake_cfg.empirical_error_model = "test_fixture"
            fake = SimpleNamespace(max_latency_steps=1)
            imported = SimpleNamespace(EmpiricalPerceptionError=lambda *a, **kw: fake)
            with mock.patch.dict(sys.modules, {"aerial_gym.task.navrl_task.navrl_empirical_error": imported}):
                with self.assertRaisesRegex(ValueError, "fresh camera"):
                    module.NavRLPerceptionModule(1, torch.device("cpu"), fake_cfg, .1, config.vision)
                fake.max_latency_steps = 0
                instance = module.NavRLPerceptionModule(1, torch.device("cpu"), fake_cfg, .1, config.vision)
                self.assertIs(instance.empirical_error, fake)


class CarveOracleTest(unittest.TestCase):
    def inputs(self):
        # Hand-set cases: nearer bar, anchor, within upper window, upper edge,
        # zero, negative, no return, NaN and Inf; three independent camera conditions.
        scan = np.array([[[2.32, 2.85, 3., 3.4, 0., -1., 12., np.nan, np.inf]]] * 3)
        depth = np.array([[[2.85, 2.9], [3., np.nan]]] * 3)
        candidates = np.ones(depth.shape, bool)
        candidates[1] = False
        return scan, np.zeros(9), depth, candidates, np.array([True, True, False]), np.zeros(3)

    def compare(self, arrays):
        before = [x.copy() for x in arrays]
        expected = reference.carve_masks(*arrays)
        actual = sensor.conservative_carve_masks(*[torch.from_numpy(x) for x in arrays])
        for key, value in expected.items():
            if value.dtype.kind in "bfiu":
                np.testing.assert_allclose(actual[key].numpy(), value, atol=1e-14, rtol=0, err_msg=key)
        for a, b in zip(arrays, before):
            np.testing.assert_equal(a, b)
        return actual

    def test_independent_exact_windows_abstention_and_immutability(self):
        result = self.compare(self.inputs())
        self.assertFalse(bool(result["lidar_carve_mask"][0, 0, 0]))
        self.assertTrue(bool(result["lidar_carve_mask"][0, 0, 1]))
        self.assertFalse(bool(result["lidar_carve_mask"][1:].any()))
        self.assertEqual(result["anchor_m"].tolist(), [2.85, 0., 0.])

    def test_current_pixel_minimum_count_and_camera_far(self):
        arrays = list(self.inputs())
        arrays[2] = np.array([[[2., 20.], [np.inf, -1.]], [[2., 0.], [np.nan, np.inf]],
                              [[np.nan, np.inf], [20., 0.]]])
        arrays[3][:] = True
        self.assertFalse(bool(self.compare(arrays)["supported"].any()))

    def test_wrapped_bearing_and_narrow_support(self):
        arrays = list(self.inputs())
        arrays[0][:] = 2.9
        limit = math.asin(.15 / 3.)
        arrays[1] = np.array([math.pi-.01, -math.pi+.01, math.pi-.02, 0., 1., 2., -.1, .2, -.2])
        arrays[5][0] = -math.pi
        result = self.compare(arrays)
        self.assertAlmostEqual(float(result["angular_limit_rad"][0]), limit)
        self.assertEqual(result["lidar_carve_mask"][0, 0].tolist()[:4], [True, True, True, False])

    def test_nonfinite_bearing_and_invalid_arguments(self):
        arrays = list(self.inputs())
        arrays[5][0] = np.nan
        self.assertFalse(bool(self.compare(arrays)["supported"].any()))
        tensors = [torch.from_numpy(x) for x in self.inputs()]
        for kwargs in ({"target_radius": 0.}, {"upper_agreement_m": math.inf}, {"min_target_pixels": 0},
                       {"min_target_pixels": True}, {"max_angle_deg": 90.}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                sensor.conservative_carve_masks(*tensors, **kwargs)
        with self.assertRaises(ValueError):
            sensor.conservative_carve_masks(*tensors[:3], tensors[3].float(), *tensors[4:])


class RealPerceptionTest(unittest.TestCase):
    def inputs(self, n, module):
        depth = torch.full((n, 90, 160), 10.)
        depth[:, 40:44, 78:82] = 2.85
        pixels = torch.zeros_like(depth, dtype=torch.bool)
        pixels[:, 40:44, 78:82] = True
        lidar = torch.full((n, module.VBEAMS * module.HBEAMS), 12.)
        forward = int(module.lidar_bin_bearings().abs().argmin())
        lidar.view(n, module.VBEAMS, module.HBEAMS)[:, :, forward] = 2.32
        return lidar, depth, pixels, torch.full((n,), 2.85), torch.zeros(n), torch.ones(n, dtype=torch.bool)

    def test_legacy_real_fusion_bitwise_vs_183a333(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="off")), torch.random.fork_rng(devices=[]):
            current, old, _, module = construct_pair(3)
            inputs = self.inputs(3, module)
            for case in range(3):
                values = [x.clone() for x in inputs]
                if case == 1:
                    values[2].zero_(); values[5].zero_()
                if case == 2:
                    values[1][:, :30] = 4.; values[4][:] = math.radians(20)
                snapshot = [x.clone() for x in values]
                with mock.patch.object(module, "conservative_carve_masks", create=True,
                                       side_effect=AssertionError("OFF called new helper")):
                    a = current._fuse_static_and_extract_obstacles(*values)
                b = old._fuse_static_and_extract_obstacles(*values)
                for x, y in zip(a, b):
                    self.assertTrue(torch.equal(x, y))
                self.assertTrue(torch.equal(current.last_target_like, old.last_target_like))
                self.assertTrue(torch.equal(current.last_scan_nearest, old.last_scan_nearest))
                for x, y in zip(values, snapshot):
                    self.assertTrue(torch.equal(x, y))

    def test_real_fusion_preserves_s14_foreground_and_requires_camera(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1a_v1")), torch.random.fork_rng(devices=[]):
            current, _, module = common.build_perception(1)
            args = self.inputs(1, module)
            forward = int(current._lidar_angles.abs().argmin())
            static, _ = current._fuse_static_and_extract_obstacles(*args, camera_visible=torch.ones(1, dtype=torch.bool))
            self.assertFalse(bool(current.last_target_like[:, :, forward].any()))
            self.assertTrue(torch.all(static.view(1, module.VBEAMS, module.HBEAMS)[:, :, forward]*12 <= 2.32+1e-6))
            # Explicit false camera with sufficient stale candidate pixels and fused-valid True.
            args[0].fill_(2.9)
            for camera in (None, torch.zeros(1, dtype=torch.bool)):
                current._fuse_static_and_extract_obstacles(*args, camera_visible=camera)
                self.assertFalse(bool(current.last_target_like.any()))
            current._fuse_static_and_extract_obstacles(*args, camera_visible=torch.ones(1, dtype=torch.bool))
            self.assertTrue(bool(current.last_target_like[:, :, forward].all()))

    def test_observe_passes_camera_confirmation_and_keeps_schema(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1a_v1")), torch.random.fork_rng(devices=[]):
            current, config, module = common.build_perception(1)
            lidar, depth, pixels, surface, bearing, visible = self.inputs(1, module)
            original = current._fuse_static_and_extract_obstacles
            with mock.patch.object(current, "_fuse_static_and_extract_obstacles", wraps=original) as fusion:
                observation, diagnostic = current.observe(torch.zeros(1, 3, 90, 160), depth, lidar,
                    torch.zeros(1, 3), torch.zeros(1, 3), torch.tensor([[0.,0.,0.,1.]]), torch.zeros(1),
                    torch.zeros(1,4), config.max_velocity, config.flight_altitude, training=False)
            self.assertEqual(tuple(observation.shape), (1, 898))
            self.assertTrue(torch.equal(fusion.call_args.kwargs["camera_visible"], diagnostic["camera_visible"]))
            self.assertFalse(bool(current.last_target_like.any()))


class RecordingWarp:
    vec3 = staticmethod(lambda *values: values)
    quat = object()
    uint64 = object()
    int32 = object()
    float32 = object()

    def __init__(self):
        self.launches = []
        self.from_calls = []

    def array(self, values, **kwargs):
        return list(values)

    def from_torch(self, value, **kwargs):
        self.from_calls.append(value)
        return value

    def launch(self, **kwargs):
        self.launches.append(kwargs)


def detector_class(source, warp):
    """Compile actual detector methods without importing/initializing Warp or Isaac."""
    tree = ast.parse(source)
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "NavRLTargetDetector")
    names = {"__init__", "_render", "render_raw_rgbd", "_resample_appearance"}
    cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in names]
    namespace = dict(torch=torch, F=F, math=math, os=os, wp=warp, sensor_lineage_enabled=sensor.sensor_lineage_enabled,
                     _distractor_count=lambda:0, _render_target_camera_kernel="target", _render_obstacle_depth_kernel="obstacle",
                     quat_rotate=lambda q,v:v, quat_mul=lambda q,v:q)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), "actual_detector_methods", "exec"), namespace)
    return namespace["NavRLTargetDetector"]


class DetectorCompositionTest(unittest.TestCase):
    def build(self, enabled, source=None):
        warp = RecordingWarp()
        cls = detector_class(source or (PATH/"navrl_detector.py").read_text(), warp)
        config = SimpleNamespace(detector_max_range=20., detector_hfov_deg=87., detector_vfov_deg=58.,
                                 tracker_memory_s=5., camera_obstacle_max_range=10.)
        with common.production({sensor.SWITCH:"s_l1a_v1" if enabled else "off"}):
            # ON imports the helper by package path; stub packages keep Isaac Gym out of CPU tests.
            for package in ("aerial_gym", "aerial_gym.task", "aerial_gym.task.navrl_task"):
                stub = ModuleType(package)
                stub.__path__ = [str(ROOT / package.replace(".", "/"))]
                stub.__eq1_stub__ = True
                sys.modules[package] = stub
            sys.modules["aerial_gym.task.navrl_task.navrl_sensor_lineage"] = sensor
            detector = cls(SimpleNamespace(CONST_WARP_MESH_ID_LIST=[1]), 1, "cpu", config, .1)
        detector.obstacle_depth[:, :, :20] = 3.
        detector.target_mask[:, 40, 80] = 1
        detector.target_depth[:, 40, 80] = 18.
        return detector, warp

    def render(self, detector):
        return detector.render_raw_rgbd(torch.zeros(1,3), torch.tensor([[0.,0.,0.,1.]]), torch.tensor([[18.,0.,0.]]))

    def test_off_allocation_queries_and_return_bitwise_vs_legacy(self):
        current, current_wp = self.build(False)
        old, old_wp = self.build(False, old_source("navrl_detector.py"))
        self.assertIsNone(current.s_l1a_obstacle_depth)
        self.assertFalse(hasattr(current, "_s_l1a_obstacle_depth_wp"))
        self.assertEqual(len(current_wp.from_calls), len(old_wp.from_calls))
        for a,b in zip(self.render(current), self.render(old)):
            self.assertTrue(torch.equal(a,b))
        self.assertEqual(len(current_wp.launches), len(old_wp.launches))
        self.assertEqual([q["dim"] for q in current_wp.launches], [(1,90,160),(1,24,40)])

    def test_on_query_wiring_rgb_native_depth_and_target_overlay(self):
        corrected, warp = self.build(True)
        baseline, _ = self.build(False)
        # Independent plane ray oracle supplies a depth fixture. Actual GPU querying
        # remains a separate canary gate; no CPU stub pretends to render it.
        expected = reference.render_reference([[4.,0.,0.]], [[.2,20.,20.]], origin=(.1,0.,.03))["depth_m"]
        corrected.s_l1a_obstacle_depth.copy_(torch.from_numpy(expected).float().unsqueeze(0))
        rgb, depth = self.render(corrected)
        old_rgb, _ = self.render(baseline)
        self.assertTrue(torch.equal(rgb, old_rgb))
        self.assertTrue(torch.equal(corrected.obstacle_depth, baseline.obstacle_depth))
        self.assertEqual(depth[0,40,80].item(),18.)
        mask=corrected.target_mask.bool()
        self.assertTrue(torch.equal(depth[~mask], corrected.s_l1a_obstacle_depth[~mask]))
        query=warp.launches[-1]
        self.assertEqual(query["kernel"],"obstacle")
        self.assertEqual(query["dim"],(1,90,160))
        self.assertIs(query["inputs"][3],corrected._ray_vectors_wp)
        self.assertIs(query["inputs"][-1],corrected._s_l1a_obstacle_depth_wp)
        self.assertEqual(query["inputs"][-2],10.)

    def test_actual_ray_tables_depth_vs_independent_oracle_false_clear_and_false_obstacle(self):
        corrected, _ = self.build(True)
        legacy, _ = self.build(False)
        np.testing.assert_allclose(corrected._ray_vectors_wp.double().numpy(), reference.normalized_pixel_rays(),
                                   atol=1e-6, rtol=0)

        def query(rays, centers, half):
            # Float64 first hits along the detector's own ray tables stand in for the Warp mesh query.
            rays = rays.double().numpy()
            return reference.first_hits((0., 0., 0.), rays / np.linalg.norm(rays, axis=-1, keepdims=True),
                                        centers, half, far_m=10.)["depth_m"]

        scenes = (("two_depth", [[2.1, 5, 0], [6.1, -5, 0]], [[.1, 5, 10], [.1, 5, 10]], "false_clear"),
                  ("edge", [[3.1, 5, 0]], [[.1, 5, 10]], "false_obstacle"),
                  ("thin", [[8.01, .04745, 0]], [[.01, .01, 1]], "false_clear"))
        for name, centers, half, legacy_defect in scenes:
            with self.subTest(scene=name):
                exact = reference.render_reference(centers, half)
                coarse = torch.from_numpy(query(legacy._obstacle_ray_vectors_wp, centers, half)).float()
                for detector in (corrected, legacy):
                    detector.obstacle_depth.copy_(coarse.unsqueeze(0))
                    detector.target_mask.zero_()
                corrected.s_l1a_obstacle_depth.copy_(
                    torch.from_numpy(query(corrected._ray_vectors_wp, centers, half)).float().unsqueeze(0))
                robust = exact["robust_mask"]
                hit, miss = robust & (exact["hit_class"] == reference.BOX), robust & (exact["hit_class"] == reference.MISS)
                counts = {}
                for label, detector in (("fixed", corrected), ("legacy", legacy)):
                    depth = self.render(detector)[1][0].double().numpy()
                    counts[label] = {"false_clear": int((hit & (depth > exact["depth_m"] + .05)).sum()),
                                     "false_obstacle": int((miss & (depth < 9.95)).sum()),
                                     "max_error_m": float(np.abs(depth - exact["depth_m"])[robust].max())}
                self.assertTrue(hit.any())
                self.assertLessEqual(counts["fixed"]["max_error_m"], 1e-4, counts)
                self.assertEqual(counts["fixed"]["false_clear"] + counts["fixed"]["false_obstacle"], 0, counts)
                self.assertGreater(counts["legacy"][legacy_defect], 0, counts)


if __name__ == "__main__":
    unittest.main()
