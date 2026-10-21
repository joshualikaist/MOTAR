"""Q02-A CPU tests: contract constants, frames, validation and the evidence hypothesis rule.

Scenes are synthetic ray casts (sphere target, vertical-cylinder bars). They test the rule's
logic against known geometry; they are not evidence of detection accuracy in the simulator.
"""
import ast
from dataclasses import fields, replace
import math
from pathlib import Path
import re
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va

CAM, LID = c.FROZEN_F_CAMERA, c.FROZEN_F_LIDAR
CALIB = ("YAW_ONLY", CAM.fx, CAM.fy, CAM.cx, CAM.cy, .10, 0., .03, "EUCLIDEAN_RAY_RANGE_M", 12., .2,
         "LIDAR_ZERO_WIDTH_4x72_5deg_10deg", "NO_CALIBRATED_ERROR_BOUNDS")
POS = np.array([0., 0., 1.5])


def yaw_quat(yaw):
    return np.array([0., 0., math.sin(yaw / 2), math.cos(yaw / 2)])


def _cylinder_hits(origin, dirs, bars):
    t = np.full(dirs.shape[:-1], np.inf)
    for cx, cy, r in bars:
        ox, oy = origin[0] - cx, origin[1] - cy
        a = dirs[..., 0] ** 2 + dirs[..., 1] ** 2
        b = 2 * (ox * dirs[..., 0] + oy * dirs[..., 1])
        cc = ox * ox + oy * oy - r * r
        disc = b * b - 4 * a * cc
        with np.errstate(invalid="ignore", divide="ignore"):
            hit = (-b - np.sqrt(np.clip(disc, 0, None))) / (2 * a)
        t = np.minimum(t, np.where((disc >= 0) & (hit > 0), hit, np.inf))
    return t


def scene(target_v=None, bars=(), yaw=0.0):
    """Vehicle-frame synthetic RGB-D (camera at its offset) and LiDAR (vehicle origin)."""
    h, w = int(2 * CAM.cy + 1), int(2 * CAM.cx + 1)
    vv, uu = np.mgrid[0:h, 0:w]
    rays = geo.pixel_rays(uu, vv, CAM)
    cam_o = np.array(CAM.camera_offset)
    t_cam = _cylinder_hits(cam_o, rays, bars)
    lid_dirs = geo.lidar_directions(LID)
    t_lid = _cylinder_hits(np.zeros(3), lid_dirs, bars)
    if target_v is not None:
        t_cam = np.minimum(t_cam, geo.sphere_first_hit(cam_o, rays, target_v, CAM.target_radius))
        t_lid = np.minimum(t_lid, geo.sphere_first_hit(np.zeros(3), lid_dirs, target_v, LID.target_proxy_radius_m))
    depth = np.minimum(t_cam, CAM.max_camera_range)
    lidar = np.clip(np.minimum(t_lid, LID.max_range_m), 0, LID.max_range_m)
    return depth.astype(np.float32), lidar.astype(np.float32)


def make(target_v=(8., 0., 0.), *, render_target=True, bars=(), detected=False, yaw=0.0, cov_pos=0.01,
         age=0.3, active=True, cam_clock=None, lidar_clock=None, decision=0.0, quat=None, calibration=CALIB, **kw):
    depth, lidar = scene(np.array(target_v) if render_target else None, bars)
    world = geo.vehicle_to_world(np.array(target_v), POS, yaw)
    cov = np.diag([cov_pos] * 3 + [0.1] * 3)
    clock = c.Clock(0.0, 0.0, "DECLARED_SIM_SCHEDULE")
    args = dict(rgb=np.zeros((3, 90, 160)), depth=depth, lidar=lidar.reshape(-1), camera_detected=detected,
                fused_update=False, confidence=0.0, track_mean=np.r_[world, 0, 0, 0], track_cov=cov,
                track_age_s=age, track_active=active, position=POS, velocity=np.zeros(3),
                vehicle_quat=yaw_quat(yaw) if quat is None else quat, camera_clock=cam_clock or clock,
                lidar_clock=lidar_clock or clock, decision_s=decision, calibration=calibration)
    args.update(kw)
    return c.EvidenceInput.build(**args)


class ContractTests(unittest.TestCase):
    def test_constants_match_production_config_text(self):
        task = (ROOT / "aerial_gym/config/task_config/navrl_task_config.py").read_text()
        lidar = (ROOT / "aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py").read_text()
        for pattern in (r"detector_hfov_deg = 87\.0", r"detector_vfov_deg = 58\.0", r"camera_target_radius = 0\.15",
                        r"camera_translation = \[0\.10, 0\.0, 0\.03\]", r"NAVRL_DETECTOR_MAX_RANGE\", 20\.0",
                        r"NAVRL_CAMERA_WIDTH\", 160", r"NAVRL_CAMERA_HEIGHT\", 90", r"NAVRL_DETECTOR_MIN_PIXELS\", 2"):
            self.assertRegex(task, pattern)
        for pattern in (r"vertical_fov_deg_min = -10\.0", r"vertical_fov_deg_max = 20\.0",
                        r"horizontal_fov_deg_min = -180\.0 \+ 360\.0 / _NAVRL_HBEAMS", r"horizontal_fov_deg_max = 180\.0"):
            self.assertRegex(lidar, pattern)

    def test_lidar_layout_and_camera_projection_conventions(self):
        az, el = np.degrees(LID.azimuth_rad()), np.degrees(LID.elevation_rad())
        self.assertTrue(np.allclose(az, 180 - 5 * np.arange(72)))  # +y is left
        self.assertTrue(np.allclose(el, [20, 10, 0, -10]))
        d = geo.lidar_directions(LID)
        self.assertTrue(np.allclose(d[2, 36], [1, 0, 0]) and np.allclose(d[2, 18], [0, 1, 0], atol=1e-9))
        u, v, front, inside = geo.project(np.array([[5., 0, 0], [5., 1., 0], [5., 0, 1.], [-5., 0, 0]]), CAM)
        self.assertAlmostEqual(u[0], CAM.cx); self.assertAlmostEqual(v[0], CAM.cy)
        self.assertLess(u[1], CAM.cx); self.assertLess(v[2], CAM.cy)  # left -> smaller u, up -> smaller v
        self.assertFalse(front[3] or inside[3])
        uu, vv = np.array([3.2, 150.7]), np.array([10.1, 80.3])
        r = geo.pixel_rays(uu, vv, CAM)
        u2, v2, _, _ = geo.project(r * 7.0, CAM)
        self.assertTrue(np.allclose(u2, uu) and np.allclose(v2, vv))
        prod = np.stack([np.ones(2), -(uu - CAM.cx) / CAM.fx, -(vv - CAM.cy) / CAM.fy], 1)  # navrl_perception
        self.assertTrue(np.allclose(r, prod / np.linalg.norm(prod, axis=1, keepdims=True)))

    def test_yaw_frame_round_trip_and_known_rotation(self):
        p = np.array([[0., 5., 1.5]])  # 5 m along world +y; the vehicle at POS faces +y (yaw 90 deg)
        v = geo.world_to_vehicle(p, POS, math.pi / 2)
        self.assertTrue(np.allclose(v, [[5., 0., 0.]], atol=1e-9))  # straight ahead in YAW_VEHICLE
        self.assertTrue(np.allclose(geo.vehicle_to_world(v, POS, math.pi / 2), p))
        self.assertAlmostEqual(geo.quat_yaw_xyzw(yaw_quat(1.2)), 1.2)
        self.assertFalse(geo.is_yaw_only([0.1, 0, 0, math.sqrt(0.99)]))

    def test_sphere_hit_matches_G4R5_B04_value(self):
        t = geo.sphere_first_hit(np.zeros(3), np.array([[1., 0, 0]]), [6., 0, 0], .20)
        self.assertAlmostEqual(float(t[0]), 5.8)


class ValidationTests(unittest.TestCase):
    def report(self, **kw):
        return va.validate(make(**kw), CAM, LID)

    def test_nominal_inputs_validate(self):
        r = self.report()
        self.assertEqual(r.problems(), ()); self.assertTrue(r.camera_usable and r.lidar_usable and r.track_usable)

    def test_missing_stale_invalid_and_frame_errors_are_reported(self):
        cases = dict(
            missing_clock=(dict(cam_clock=c.Clock(None, None)), "camera_clock", va.MISSING),
            bad_provenance=(dict(cam_clock=c.Clock(0., 0., "HOST_TIME")), "camera_clock", va.MISSING),
            stale_clock=(dict(lidar_clock=c.Clock(0., 0., "DECLARED_SIM_SCHEDULE"), decision=0.3), "lidar_clock", va.STALE),
            future_clock=(dict(cam_clock=c.Clock(0.5, 0.5, "DECLARED_SIM_SCHEDULE")), "camera_clock", va.INVALID),
            wrong_frame=(dict(cam_clock=c.Clock(0., 0., "DECLARED_SIM_SCHEDULE", frame="BODY_TILTED")), "camera_clock", va.FRAME_MISMATCH),
            optical_z=(dict(cam_clock=c.Clock(0., 0., "DECLARED_SIM_SCHEDULE", depth_convention="OPTICAL_Z")), "camera_clock", va.FRAME_MISMATCH),
            tilted=(dict(quat=np.array([0.1, 0., 0., math.sqrt(0.99)])), "pose", va.FRAME_MISMATCH),
            calibration=(dict(calibration=CALIB[:1] + (CAM.fx + 1,) + CALIB[2:]), "calibration", va.FRAME_MISMATCH),
            no_calibration=(dict(calibration=()), "calibration", va.MISSING),
            inactive=(dict(active=False), "track", va.MISSING),
            not_psd=(dict(track_cov=np.diag([1., 1., -1., 1., 1., 1.])), "track", va.INVALID),
            nan_lidar=(dict(lidar=np.full(288, np.nan)), "lidar", va.INVALID),
            negative_depth=(dict(depth=-np.ones((90, 160))), "depth", va.INVALID),
            rgb_range=(dict(rgb=np.full((3, 90, 160), 2.)), "rgb", va.INVALID))
        for name, (kw, check, status) in cases.items():
            with self.subTest(name=name):
                self.assertEqual(self.report(**kw).status(check), status)

    def test_shape_errors_refused_at_construction(self):
        with self.assertRaises(ValueError):
            make(depth=np.zeros((160, 90)))


class EvidenceTests(unittest.TestCase):
    def hyp(self, **kw):
        return ev.estimate(make(**kw))

    def test_visible_occluded_out_of_fov_and_detection_miss(self):
        bar = ((4., 0., .3),)
        self.assertEqual(self.hyp(detected=True).hypothesis, ev.TARGET_VISIBLE)
        occ = self.hyp(bars=bar)
        self.assertEqual(occ.hypothesis, ev.OCCLUDED)
        self.assertGreater(occ.depth_foreground, 0); self.assertGreater(occ.lidar_foreground, 0)
        self.assertEqual(self.hyp(target_v=(-6., 2., 0.)).hypothesis, ev.OUT_OF_FOV)
        miss = self.hyp(render_target=False)  # free line of sight, nothing at the predicted place
        self.assertEqual(miss.hypothesis, ev.DETECTION_MISS)
        self.assertIn("BELIEF_ERROR_NOT_EXCLUDED", miss.reasons)

    def test_surface_at_expected_range_is_ambiguous_not_visible(self):
        undetected = self.hyp()  # target rendered but the detector flag is False
        self.assertEqual(undetected.hypothesis, ev.IDENTITY_AMBIGUOUS)
        lookalike = self.hyp(render_target=False, bars=((8., 0., .15),))  # B04: foreground surface at target range
        self.assertEqual(lookalike.hypothesis, ev.IDENTITY_AMBIGUOUS)

    def test_unknown_when_inputs_cannot_separate_cases(self):
        for name, kw, reason in (
                ("stale belief", dict(age=3.0), "BELIEF_STALE"),
                ("diffuse belief", dict(cov_pos=4.0), "BELIEF_TOO_DIFFUSE"),
                ("fov boundary", dict(target_v=(6 * math.cos(math.radians(43.0)), 6 * math.sin(math.radians(43.0)), 0.),
                                      cov_pos=0.05), "FOV_BOUNDARY_UNCERTAIN"),
                ("beyond range", dict(target_v=(25., 0., 0.)), "BEYOND_DETECTION_RANGE"),
                ("no track", dict(active=False), "NO_USABLE_TRACK"),
                ("camera clock missing", dict(cam_clock=c.Clock(None, None)), "CAMERA_INPUT_UNUSABLE"),
                ("tilted pose", dict(quat=np.array([0.1, 0., 0., math.sqrt(0.99)])), "NO_USABLE_TRACK")):
            with self.subTest(name=name):
                r = self.hyp(**kw)
                self.assertEqual(r.hypothesis, ev.UNKNOWN); self.assertIn(reason, r.reasons)
        stale_detection = self.hyp(detected=True, cam_clock=c.Clock(0., 0., "DECLARED_SIM_SCHEDULE"), decision=0.5)
        self.assertEqual(stale_detection.hypothesis, ev.UNKNOWN)

    def test_lidar_failure_degrades_to_depth_only_with_reason(self):
        r = self.hyp(bars=((4., 0., .3),), lidar=np.full(288, np.nan))
        self.assertEqual(r.hypothesis, ev.OCCLUDED); self.assertIn("LIDAR_INPUT_UNUSABLE", r.reasons)
        self.assertEqual(r.lidar_beams, 0)

    def test_frames_used_consistently_under_yaw(self):
        # The same vehicle-frame scene seen with the vehicle yawed 90 degrees in the world.
        for yaw in (0.0, math.pi / 2, -2.0):
            with self.subTest(yaw=yaw):
                r = self.hyp(bars=((4., 0., .3),), yaw=yaw)
                self.assertEqual(r.hypothesis, ev.OCCLUDED); self.assertAlmostEqual(r.bearing_deg, 0.0, places=6)

    def test_record_semantics_and_no_ground_truth(self):
        r = self.hyp(bars=((4., 0., .3),))
        self.assertEqual((r.utility, r.ground_truth_used, r.calibration), ("ABSTAIN", False, "NONE_DECLARED_PRIORS"))
        self.assertEqual(r.confidence_semantics, "SEGMENTER_SCORE_X_PIXEL_SUPPORT_NOT_A_PROBABILITY")
        self.assertEqual(r.belief_kind, "PREDICTED_ONLY")
        self.assertIn(r.hypothesis, ev.HYPOTHESES)
        names = {f.name for f in fields(c.EvidenceInput)}
        self.assertFalse(any(k in n for n in names for k in ("target_position", "segmentation", "semantic", "obstacle", "gt")))
        with self.assertRaisesRegex(ValueError, "NON_SENSOR_FIELDS_REFUSED"):
            c.EvidenceInput.from_arrays(dict(target_position=np.zeros(3)), None, None, 0.)

    def test_pure_deterministic_and_inputs_read_only(self):
        inp = make(bars=((4., 0., .3),))
        state = np.random.get_state()[1].copy()
        a, b = ev.estimate(inp), ev.estimate(inp)
        self.assertEqual(a, b); self.assertTrue(np.array_equal(state, np.random.get_state()[1]))
        with self.assertRaises(ValueError):
            inp.depth[0, 0] = 1.0

    def test_package_imports_no_simulator_torch_or_rng(self):
        for path in sorted((ROOT / "motar_eval/q02").glob("*.py")):
            tree = ast.parse(path.read_text())
            names = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
            names |= {(n.module or "").split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
            with self.subTest(path=path.name):
                self.assertFalse(names & {"torch", "isaacgym", "aerial_gym", "warp", "random"})
                self.assertNotIn("np.random", path.read_text())

    def test_adapter_from_Q01_sensor_packet(self):
        import torch
        from motar_eval import boia_shadow as s
        tensors = [torch.zeros(shape, dtype=torch.float32) for shape in s.SHAPES]
        v = s.SensorValues(*tensors)
        for name in s.BOOL_FIELDS:
            object.__setattr__(v, name, torch.zeros(1, dtype=torch.bool))
        v.depth.fill_(10); v.lidar.fill_(12); v.raw_lidar.fill_(1); v.vehicle_quat[:, 3] = 1; v.body_quat[:, 3] = 1
        v.covariance[:] = torch.eye(6)[None] * .01; v.position[:, 2] = 1.5; v.mean[:, 0] = 8; v.mean[:, 2] = 1.5
        object.__setattr__(v, "active", torch.ones(1, dtype=torch.bool))
        stamp = s.SensorTime(0., 0., 0., 1., "DECLARED_SIM_SCHEDULE")
        packet = s.make_packet(s.JoinKey(0, 0, 0, 0, 0), s.Binding("a" * 40, "b" * 64, "c" * 64), stamp, stamp, v, CALIB)
        r = ev.estimate(c.EvidenceInput.from_packet(packet))
        self.assertEqual(r.validation_problems, ())
        self.assertEqual(r.hypothesis, ev.DETECTION_MISS)  # empty 10 m depth / 12 m LiDAR beyond a target at 8 m


if __name__ == "__main__":
    unittest.main()
