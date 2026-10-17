"""S-L1B runtime CPU gates against the committed float64 oracle and S-L1A source 139184b.

Exact synthetic fixtures feed float32 oracle depth into the production helpers and the real
production perception fusion. No Warp/Isaac initialization; GPU rendering is the canary's job.
"""
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
import s_l1b_reference as oracle  # noqa: E402

PATH = ROOT / "aerial_gym/task/navrl_task"
S_L1A_SOURCE = "139184bc0d2a7aa855fe8ab5a2a7f547702defa5"
STEP = 2 * math.pi / 72
ANGLES = np.linspace(math.pi, -math.pi + STEP, 72)


def helper():
    spec = importlib.util.spec_from_file_location("_s_l1b_runtime_helper", PATH / "navrl_sensor_lineage.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


sensor = helper()


def old_source(name):
    return subprocess.check_output(["git", "show", S_L1A_SOURCE + ":aerial_gym/task/navrl_task/" + name], cwd=ROOT, text=True)


def production_rays(width=160, height=90):
    fx, fy = width / (2 * math.tan(math.radians(43.5))), height / (2 * math.tan(math.radians(29.)))
    return sensor.camera_unit_rays(width, height, fx, fy, (width - 1) / 2, (height - 1) / 2)


def production_points(depth, rotation=np.eye(3), translation=oracle.CAMERA_TRANSLATION, blank=None):
    return sensor.camera_points_lidar_frame(
        torch.as_tensor(depth, dtype=torch.float32).view(-1, 90, 160), production_rays(),
        torch.as_tensor(rotation, dtype=torch.float32), torch.as_tensor(translation, dtype=torch.float32),
        camera_far_m=10., blank=blank)


def bars_at(bearing_deg, body_range, half=.05):
    a = math.radians(bearing_deg)
    return [[body_range * math.cos(a), body_range * math.sin(a), 0.]], [[half, half, 3.]]


FIXTURES = {"F1_front_plane": ([[5.1, 0, 0]], [[.1, 20, 20]])}
for _deg in (5, -5, 20, -20, 40, -40):
    for _rng in (3., 6.):
        FIXTURES["F2_bar_%+d_%g" % (_deg, _rng)] = bars_at(_deg, _rng)
FIXTURES["F3_overhang_and_low_box"] = ([[3., 0, 2.1], [2.5, 1.0, -1.6]], [[.3, .6, .5], [.3, .3, .4]])


class SwitchAndCalibrationTest(unittest.TestCase):
    def test_cumulative_levels_and_fail_closed(self):
        self.assertEqual(sensor.sensor_lineage_level({}), 0)
        self.assertEqual(sensor.sensor_lineage_level({sensor.SWITCH: "off"}), 0)
        self.assertEqual(sensor.sensor_lineage_level({sensor.SWITCH: "s_l1a_v1"}), 1)
        self.assertEqual(sensor.sensor_lineage_level({sensor.SWITCH: "s_l1b_v1"}), 2)
        self.assertTrue(sensor.sensor_lineage_enabled({sensor.SWITCH: "s_l1b_v1"}))
        for value in ("", "on", "s_l1b", "S_L1B_V1", "s_l1z_v1", " s_l1b_v1"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                sensor.sensor_lineage_level({sensor.SWITCH: value})
        # s_l1c_v1 was unknown when S-L1B was qualified; it is now the preregistered cumulative
        # successor (docs/plans/s_l1c_v1.json) and must sit above S-L1B, never below it.
        self.assertGreater(sensor.sensor_lineage_level({sensor.SWITCH: "s_l1c_v1"}), 2)

    def test_declared_constants_match_sensor_configs(self):
        tree = ast.parse((ROOT / "aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "NavRLLidarConfig")
        values = {}
        for n in cls.body:
            if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name):
                try:
                    values[n.targets[0].id] = ast.literal_eval(n.value)
                except ValueError:
                    continue
        self.assertEqual(values["target_radius"], sensor.LIDAR_TARGET_PROXY_RADIUS_M)
        self.assertEqual(values["vertical_fov_deg_max"], sensor.LIDAR_ELEVATION_MAX_DEG)
        self.assertEqual(values["vertical_fov_deg_min"], sensor.LIDAR_ELEVATION_MIN_DEG)
        self.assertTrue(values["yaw_only_attach"])
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1b_v1")):
            current, config, module = common.build_perception(1)
            self.assertTrue(current.sensor_lineage_s_l1a and current.sensor_lineage_s_l1b)
            np.testing.assert_allclose(current.camera_offset.view(3).numpy(), oracle.CAMERA_TRANSLATION, atol=1e-7)
            self.assertEqual((module.VBEAMS, module.HBEAMS), (4, 72))
            torch.testing.assert_close(module.lidar_bin_bearings().double(), torch.from_numpy(ANGLES), atol=1e-6, rtol=0)
            np.testing.assert_allclose(current._s_l1b_rays.double().numpy(),
                                       oracle.ref.normalized_pixel_rays(), atol=1e-6, rtol=0)
            self.assertTrue(torch.equal(current._s_l1b_rotation, torch.eye(3)))

    def test_s_l1a_carve_helper_is_byte_identical_to_qualified_source(self):
        def function_source(text, name):
            tree = ast.parse(text)
            node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
            return ast.get_source_segment(text, node)
        current = (PATH / "navrl_sensor_lineage.py").read_text()
        self.assertEqual(function_source(current, "conservative_carve_masks"),
                         function_source(old_source("navrl_sensor_lineage.py"), "conservative_carve_masks"))


class CalibrationGateTest(unittest.TestCase):
    def check_fixture(self, centers, half, rotation=np.eye(3), translation=oracle.CAMERA_TRANSLATION):
        truth = oracle.true_surface(centers, half, rotation=rotation, translation=translation)
        depth32 = truth["depth_m"].astype(np.float32)
        prod = production_points(depth32, rotation, translation)
        mask = truth["robust_mask"] & truth["box"] & ~truth["grazing"] & prod["valid"][0].numpy()
        self.assertGreater(mask.sum(), 0)
        rho_err = np.abs(prod["rho"][0].double().numpy() - truth["rho"])[mask].max()
        az = prod["az"][0].double().numpy() - truth["az"]
        az_err = np.abs(np.arctan2(np.sin(az), np.cos(az)))[mask].max()
        el_err = np.abs(prod["el"][0].double().numpy() - truth["el"])[mask].max()
        self.assertLessEqual(rho_err, 1e-4)
        self.assertLessEqual(max(az_err, el_err), 1e-5)
        cam = oracle.camera_points(depth32.astype(np.float64)[None], rotation=rotation, translation=translation)
        for clamp in (False, True):
            expected, robust = oracle.robust_bins(cam, clamp=clamp)
            actual = sensor.lidar_envelope_bins(prod, hbeams=72, vbeams=4, lidar_far_m=12., clamp=clamp)
            err = np.abs(actual.double().numpy() - expected)[robust]
            self.assertEqual(int((err > 1e-4).sum()), 0, (clamp, float(err.max())))
            self.assertGreater(int(robust.sum()), 280)
        return truth, prod

    def test_b_point_and_b_envelope_exact_fixtures(self):
        for name, (centers, half) in FIXTURES.items():
            with self.subTest(fixture=name):
                self.check_fixture(centers, half)

    def test_f4_rotated_translated_mount(self):
        rotation = oracle.rotation_matrix(pitch=math.radians(10), yaw=math.radians(5))
        for name in ("F1_front_plane", "F2_bar_+20_3", "F2_bar_-40_6"):
            with self.subTest(fixture=name):
                self.check_fixture(*FIXTURES[name], rotation=rotation, translation=(.2, .05, -.04))
        # A mount error would be visible: identity reprojection of the rotated render is wrong.
        truth = oracle.true_surface(*FIXTURES["F1_front_plane"], rotation=rotation, translation=(.2, .05, -.04))
        wrong = production_points(truth["depth_m"].astype(np.float32))
        mask = truth["robust_mask"] & truth["box"] & wrong["valid"][0].numpy()
        self.assertGreater(np.abs(wrong["rho"][0].double().numpy() - truth["rho"])[mask].max(), .05)

    def test_b_a1a2_counterexamples_against_legacy_production(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="off")):
            legacy, _, module = common.build_perception(1)
            empty = torch.full((1, 288), 12.)
            no_target = torch.zeros(1, 90, 160, dtype=torch.bool)
            zero, false = torch.zeros(1), torch.zeros(1, dtype=torch.bool)

            def legacy_scan(depth):
                static, _ = legacy._fuse_static_and_extract_obstacles(
                    empty.clone(), torch.as_tensor(depth, dtype=torch.float32).view(1, 90, 160), no_target, zero, zero, false)
                return static.view(4, 72).double().numpy() * 12.
            # A2 front plane: legacy on-axis body range error versus the exact per-pixel S-L1B range.
            truth, prod = self.check_fixture(*FIXTURES["F1_front_plane"])
            on_axis = legacy_scan(truth["depth_m"])[2, 36]
            self.assertAlmostEqual(on_axis - 5.0, -0.0998, delta=2e-4)
            center = (slice(44, 46), slice(79, 81))
            self.assertLessEqual(np.abs(prod["rho"][0].double().numpy()[center] - truth["rho"][center]).max(), 1e-4)
            # A1 20-degree bar at 6 m: legacy fuses it at the wrong bearing; S-L1B bins the true one.
            truth, prod = self.check_fixture(*FIXTURES["F2_bar_+20_6"])
            hit = truth["box"] & prod["valid"][0].numpy()
            bins = sensor.lidar_envelope_bins(prod, hbeams=72, vbeams=4, lidar_far_m=12., clamp=True)[0].numpy()
            true_bins = set(np.unique(oracle.bin_index(truth["az"][hit], truth["el"][hit])[0]).tolist())
            self.assertEqual({int(j) for j in np.where((bins < 12).any(axis=0))[0]}, true_bins)
            legacy_bins = {int(j) for j in np.where((legacy_scan(truth["depth_m"]) < 12).any(axis=0))[0]}
            self.assertNotEqual(legacy_bins, true_bins)

    def test_b_vertical_rows_and_clamped_envelope(self):
        truth, prod = self.check_fixture(*FIXTURES["F3_overhang_and_low_box"])
        strict = sensor.lidar_envelope_bins(prod, hbeams=72, vbeams=4, lidar_far_m=12., clamp=False)[0]
        clamped = sensor.lidar_envelope_bins(prod, hbeams=72, vbeams=4, lidar_far_m=12., clamp=True)[0]
        valid = prod["valid"][0]
        el = prod["el"][0]
        above, below = valid & (el > math.radians(25.)), valid & (el < math.radians(-15.))
        self.assertTrue(bool(above.any()) and bool(below.any()))
        # Out-of-coverage points enter only the clamped edge rows; middle rows are identical.
        self.assertTrue(torch.equal(strict[1:3], clamped[1:3]))
        self.assertTrue(bool((clamped[0] <= strict[0]).all()) and bool((clamped[3] <= strict[3]).all()))
        self.assertTrue(bool((clamped[0] < strict[0]).any()) and bool((clamped[3] < strict[3]).any()))
        # The 2-D envelope (row minimum) contains every valid point of its azimuth bin.
        j = torch.remainder(torch.round((math.pi - prod["az"][0]) / STEP), 72).long()
        two_d = clamped.amin(dim=0)
        self.assertTrue(bool((two_d[j[valid]] <= prod["rho"][0][valid] + 1e-6).all()))

    def test_validity_and_no_return_conventions(self):
        depth = torch.full((1, 90, 160), 10.)
        depth[0, 45, 10:18] = torch.tensor([9.9989, 9.9991, 0., -1., float("nan"), float("inf"), 3., 3.])
        blank = torch.zeros(1, 90, 160, dtype=torch.bool)
        blank[0, 45, 17] = True
        points = production_points(depth, blank=blank)
        self.assertEqual(points["valid"][0, 45, 10:18].tolist(), [True, False, False, False, False, False, True, False])
        self.assertEqual(int(points["valid"].sum()), 2)
        bins = sensor.lidar_envelope_bins(points, hbeams=72, vbeams=4, lidar_far_m=12., clamp=True)
        self.assertEqual(int((bins < 12).sum()), 2 if bool(torch.isfinite(bins).all()) else -1)


class RealPerceptionTest(unittest.TestCase):
    def lidar_truth(self, centers, half, sphere=None):
        elev = np.radians([20., 10., 0., -10.])
        beams = np.stack(np.broadcast_arrays(np.cos(elev[:, None]) * np.cos(ANGLES), np.cos(elev[:, None]) * np.sin(ANGLES),
                                             np.sin(elev[:, None]) + 0 * ANGLES), axis=-1)
        hits = oracle.ref.first_hits(np.zeros(3), beams, centers, half, sphere_center=sphere,
                                     sphere_radius=oracle.LIDAR_PROXY_RADIUS_M, far_m=12.)
        return hits["depth_m"]

    def test_fused_scan_equals_lidar_min_clamped_envelope(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1b_v1")), torch.random.fork_rng(devices=[]):
            current, _, module = common.build_perception(1)
            for name in ("F1_front_plane", "F2_bar_+40_3", "F3_overhang_and_low_box"):
                with self.subTest(fixture=name):
                    centers, half = FIXTURES[name]
                    truth = oracle.true_surface(centers, half)
                    lidar = self.lidar_truth(centers, half)
                    depth = torch.from_numpy(truth["depth_m"].astype(np.float32)).view(1, 90, 160)
                    static, _ = current._fuse_static_and_extract_obstacles(
                        torch.from_numpy(lidar.astype(np.float32)).view(1, 288), depth,
                        torch.zeros(1, 90, 160, dtype=torch.bool), torch.zeros(1), torch.zeros(1),
                        torch.zeros(1, dtype=torch.bool), camera_visible=torch.zeros(1, dtype=torch.bool))
                    cam = oracle.camera_points(depth.double().numpy())
                    envelope, robust = oracle.robust_bins(cam, clamp=True)
                    expected = np.minimum(lidar[None], envelope)
                    actual = static.view(1, 4, 72).double().numpy() * 12.
                    err = np.abs(actual - expected)[robust]
                    self.assertLessEqual(float(err.max()), 1e-4)
                    self.assertFalse(bool(current.last_target_like.any()))
                    # The 2-D conservative envelope never exceeds the LiDAR's own beam truth.
                    self.assertTrue(bool((actual.min(axis=1)[0] <= lidar.min(axis=0) + 1e-5).all()))

    def carve_case(self, name):
        target = [3.0, 0., 0.]
        bars = {"S14": ([[2.6 + .3, -.08 - .3, 0.], [2.6 + .3, -.01 + .3, 0.]], [[.3, .3, 3.], [.3, .3, 3.]]),
                "A_FOREGROUND": ([[2.6, -.08 - .262, 0.], [2.6, -.01 + .245, 0.]], [[.34, .262, 3.], [.289, .245, 3.]]),
                "B_TIGHT": ([[2.77 + .005, 0., 0.]], [[.005, .005, 3.]])}[name]
        cam_truth = oracle.true_surface(*bars, sphere_center=target, sphere_radius=.15, far_m=20.)
        lidar = self.lidar_truth(*bars, sphere=target)
        candidates = cam_truth["hit_class"] == oracle.ref.SPHERE
        return bars, cam_truth, lidar, candidates

    def test_b_carve_common_frame_never_removes_nearer_bar_returns(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1b_v1")), torch.random.fork_rng(devices=[]):
            current, _, module = common.build_perception(1)
            for name in ("S14", "A_FOREGROUND", "B_TIGHT"):
                with self.subTest(case=name):
                    bars, cam_truth, lidar, candidates = self.carve_case(name)
                    self.assertGreaterEqual(int(candidates.sum()), 2)
                    depth = torch.from_numpy(cam_truth["depth_m"].astype(np.float32)).view(1, 90, 160)
                    cand = torch.from_numpy(candidates).view(1, 90, 160)
                    bearing = torch.zeros(1)
                    current._fuse_static_and_extract_obstacles(
                        torch.from_numpy(lidar.astype(np.float32)).view(1, 288), depth, cand, torch.full((1,), 2.85),
                        bearing, torch.ones(1, dtype=torch.bool), camera_visible=torch.ones(1, dtype=torch.bool))
                    carved = current.last_target_like[0].numpy()
                    expected = oracle.common_frame_carve_masks(lidar[None].astype(np.float32).astype(np.float64), ANGLES,
                                                               depth.double().numpy(), candidates[None], np.array([True]),
                                                               np.array([0.]))
                    self.assertTrue(np.array_equal(carved, expected["lidar_carve_mask"][0]))
                    elev = np.radians([20., 10., 0., -10.])
                    beams = np.stack(np.broadcast_arrays(np.cos(elev[:, None]) * np.cos(ANGLES), np.cos(elev[:, None]) * np.sin(ANGLES),
                                                         np.sin(elev[:, None]) + 0 * ANGLES), axis=-1)
                    labels = oracle.ref.first_hits(np.zeros(3), beams, *bars, sphere_center=[3., 0., 0.],
                                                   sphere_radius=.20, far_m=12.)["hit_class"]
                    bar_beams = labels == oracle.ref.BOX
                    self.assertFalse(bool((carved & bar_beams).any()))
                    if name == "B_TIGHT":
                        self.assertTrue(bool(bar_beams[2, 36]))
                        old = oracle.ref.carve_masks(lidar[None], ANGLES, depth.double().numpy(), candidates[None],
                                                     np.array([True]), np.array([0.]), target_radius=.15, camera_far_m=20.)
                        self.assertTrue(bool(old["lidar_carve_mask"][0, 2, 36]))  # S-L1A frame mismatch

    def test_unsupported_evidence_never_carves(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1b_v1")), torch.random.fork_rng(devices=[]):
            current, _, module = common.build_perception(1)
            bars, cam_truth, lidar, candidates = self.carve_case("S14")
            depth = torch.from_numpy(cam_truth["depth_m"].astype(np.float32)).view(1, 90, 160)
            cand = torch.from_numpy(candidates).view(1, 90, 160)
            for camera in (None, torch.zeros(1, dtype=torch.bool)):
                current._fuse_static_and_extract_obstacles(
                    torch.from_numpy(lidar.astype(np.float32)).view(1, 288), depth, cand, torch.full((1,), 2.85),
                    torch.zeros(1), torch.ones(1, dtype=torch.bool), camera_visible=camera)
                self.assertFalse(bool(current.last_target_like.any()))


class RegressionTest(unittest.TestCase):
    def construct_pair(self, env):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)):
            pass
        current, config, module = common.build_perception(2)
        namespace = {"__name__": "_s_l1b_measured_perception", "__file__": str(PATH / "navrl_perception.py")}
        exec(compile(old_source("navrl_perception.py"), "s_l1a:navrl_perception.py", "exec"), namespace)
        old = namespace["NavRLPerceptionModule"](2, torch.device("cpu"), config.perception, .1, config.vision)
        return current, old, module

    def inputs(self, module):
        truth = oracle.true_surface(*FIXTURES["F2_bar_+20_3"], sphere_center=[4., -.5, 0.], sphere_radius=.15, far_m=20.)
        depth = torch.from_numpy(truth["depth_m"].astype(np.float32)).view(1, 90, 160).repeat(2, 1, 1)
        pixels = torch.from_numpy(truth["hit_class"] == oracle.ref.SPHERE).view(1, 90, 160).repeat(2, 1, 1)
        lidar = torch.full((2, 288), 12.)
        lidar.view(2, 4, 72)[:, :, 36] = 3.7
        lidar.view(2, 4, 72)[:, 2, 32] = 2.9
        return lidar, depth, pixels, torch.full((2,), 3.7), torch.full((2,), -.12), torch.ones(2, dtype=torch.bool)

    def test_off_and_s_l1a_fusion_bitwise_vs_139184b(self):
        for env in ("off", "s_l1a_v1"):
            with self.subTest(env=env), common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)), \
                    torch.random.fork_rng(devices=[]):
                current, old, module = self.construct_pair(env)
                self.assertFalse(current.sensor_lineage_s_l1b)
                self.assertFalse(hasattr(current, "_s_l1b_rays"))
                values = self.inputs(module)
                for camera in ((None,) if env == "off" else (None, torch.ones(2, dtype=torch.bool), torch.tensor([True, False]))):
                    kw = {} if camera is None else {"camera_visible": camera}
                    with mock.patch.object(module, "camera_points_lidar_frame", create=True,
                                           side_effect=AssertionError("S-L1B helper called")):
                        a = current._fuse_static_and_extract_obstacles(*[x.clone() for x in values], **kw)
                    b = old._fuse_static_and_extract_obstacles(*[x.clone() for x in values], **kw)
                    for x, y in zip(a, b):
                        self.assertTrue(torch.equal(x, y))
                    self.assertTrue(torch.equal(current.last_target_like, old.last_target_like))

    def test_off_and_s_l1a_observe_bitwise_vs_139184b(self):
        for env in ("off", "s_l1a_v1"):
            with self.subTest(env=env), common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE=env)):
                with torch.random.fork_rng(devices=[]):
                    torch.manual_seed(7)
                    current, old, module = self.construct_pair(env)
                lidar, depth, pixels, *_ = self.inputs(module)
                rgb = torch.zeros(2, 3, 90, 160)
                args = (rgb, depth, lidar, torch.zeros(2, 3), torch.zeros(2, 3), torch.tensor([[0., 0., 0., 1.]] * 2),
                        torch.zeros(2), torch.zeros(2, 4))
                with torch.random.fork_rng(devices=[]):
                    torch.manual_seed(11)
                    a, _ = current.observe(*args, 3.5, 1.5, training=False)
                with torch.random.fork_rng(devices=[]):
                    torch.manual_seed(11)
                    b, _ = old.observe(*args, 3.5, 1.5, training=False)
                self.assertEqual(tuple(a.shape), (2, 898))
                self.assertTrue(torch.equal(a, b))

    def test_s_l1b_observe_schema_and_changes_only_the_scan_path(self):
        with common.production(dict(common.receipt_env(), NAVRL_SENSOR_LINEAGE="s_l1b_v1")), torch.random.fork_rng(devices=[]):
            current, _, module = common.build_perception(2)
            lidar, depth, pixels, *_ = self.inputs(module)
            obs, diag = current.observe(torch.zeros(2, 3, 90, 160), depth, lidar, torch.zeros(2, 3), torch.zeros(2, 3),
                                        torch.tensor([[0., 0., 0., 1.]] * 2), torch.zeros(2), torch.zeros(2, 4),
                                        3.5, 1.5, training=False)
            self.assertEqual(tuple(obs.shape), (2, 898))
            self.assertTrue(bool(torch.isfinite(obs).all()))
            self.assertEqual(tuple(current.last_s_l1b_envelope.shape), (2, 4, 72))


class FirewallTest(unittest.TestCase):
    FORBIDDEN = ("target_position", "target_pos", "semantic", "segmentation", "gt", "ground_truth", "drone_pos",
                 "tracker", "simulator", "pose_w", "mask_gt")

    def test_s_l1b_helpers_accept_only_sensor_arrays_and_calibration(self):
        for name in ("camera_unit_rays", "camera_points_lidar_frame", "lidar_envelope_bins", "common_frame_carve_masks"):
            params = inspect.signature(getattr(sensor, name)).parameters
            for param in params:
                self.assertFalse(any(word in param.lower() for word in self.FORBIDDEN), (name, param))
        tree = ast.parse((PATH / "navrl_sensor_lineage.py").read_text())
        imports = {a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        self.assertEqual(imports, {"math", "os", "torch"})
        self.assertFalse(any(isinstance(n, ast.ImportFrom) for n in ast.walk(tree)))


if __name__ == "__main__":
    unittest.main()
