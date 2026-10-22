"""Q02-A CPU tests (contract Q02A-v2.6): constants, frames, sensor semantics, validation and the rule.

Scenes are synthetic ray casts reproducing the production sensor pipeline: camera depth = a native
40x24 obstacle render (bars only, no hit -> 10 m) bilinearly upsampled to 160x90 with the target
pasted at full resolution where it is in front of the bars; LiDAR = first hit of bars and the 0.20 m
target proxy, < 0.2 m -> 0.0, no hit -> 12 m. The v1 helper rendered a 20 m full-resolution depth
image, which is not what the Frozen F profile delivers; that is why v1 tests missed the Fable F1
defect (REVIEW.md). Scenes test the rule's logic against known geometry, not detection accuracy.
"""
import ast
from dataclasses import fields, replace
import json
import math
from pathlib import Path
import re
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va

CAM, LID, PROF = c.FROZEN_F_CAMERA, c.FROZEN_F_LIDAR, c.FROZEN_F_PROFILE
CALIB = ("YAW_ONLY", CAM.fx, CAM.fy, CAM.cx, CAM.cy, .10, 0., .03, "EUCLIDEAN_RAY_RANGE_M", 12., .2,
         "LIDAR_ZERO_WIDTH_4x72_5deg_10deg", "NO_CALIBRATED_ERROR_BOUNDS")
POS = np.array([0., 0., 1.5])
AV, AH = geo.bilinear_matrix(90, 24), geo.bilinear_matrix(160, 40)


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


def scene(target_v=None, bars=(), target_in_camera=True, target_in_lidar=True):
    """Production-faithful synthetic depth (90, 160) and LiDAR (4, 72) in the vehicle frame."""
    cam_o = np.array(CAM.camera_offset)
    native = np.minimum(_cylinder_hits(cam_o, geo.native_camera_rays(CAM, PROF.obstacle_native_wh), bars), PROF.obstacle_far_m)
    depth = np.clip(AV @ native @ AH.T, 0.0, PROF.obstacle_far_m)
    vv, uu = np.mgrid[0:90, 0:160]
    rays = geo.pixel_rays(uu, vv, CAM)
    if target_v is not None and target_in_camera:
        t_tgt = geo.sphere_first_hit(cam_o, rays, target_v, CAM.target_radius)
        visible = np.isfinite(t_tgt) & (t_tgt < _cylinder_hits(cam_o, rays, bars)) & (t_tgt <= CAM.max_camera_range)
        depth = np.where(visible, t_tgt, depth)
    dirs = geo.lidar_directions(LID)
    t = _cylinder_hits(np.zeros(3), dirs, bars)
    if target_v is not None and target_in_lidar:
        t = np.minimum(t, geo.sphere_first_hit(np.zeros(3), dirs, target_v, LID.target_proxy_radius_m))
    lidar = np.where(t < LID.min_range_m, 0.0, np.where(t >= LID.max_range_m, LID.max_range_m, t))
    return depth.astype(np.float32), lidar.astype(np.float32)


def make(target_v=(8., 0., 0.), *, render_target=True, bars=(), detected=False, yaw=0.0, cov_pos=0.01,
         age=0.3, active=True, cam_clock=None, lidar_clock=None, decision=0.0, quat=None, calibration=CALIB,
         target_in_camera=True, target_in_lidar=True, cov=None, **kw):
    depth, lidar = scene(np.array(target_v) if render_target else None, bars, target_in_camera, target_in_lidar)
    world = geo.vehicle_to_world(np.array(target_v), POS, yaw)
    if cov is None:
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
        self.assertGreater(occ.camera.foreground, 0); self.assertGreater(occ.lidar.foreground, 0)  # v2 field names
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
        self.assertEqual(r.lidar.rays, 0)  # v2 field name

    def test_frames_used_consistently_under_yaw(self):
        # The same vehicle-frame scene seen with the vehicle yawed 90 degrees in the world.
        for yaw in (0.0, math.pi / 2, -2.0):
            with self.subTest(yaw=yaw):
                r = self.hyp(bars=((4., 0., .3),), yaw=yaw)
                self.assertEqual(r.hypothesis, ev.OCCLUDED); self.assertAlmostEqual(r.bearing_deg, 0.0, places=6)

    def test_record_semantics_and_no_ground_truth(self):
        r = self.hyp(bars=((4., 0., .3),))
        self.assertEqual((r.utility, r.ground_truth_used, r.calibration), ("ABSTAIN", False, "NONE_DECLARED_PRIORS"))
        # F7: production confidence = max(camera segmenter support, LiDAR association gate), navrl_perception.py:2396
        self.assertEqual(r.confidence_semantics, "MAX_OF_CAMERA_SEGMENTER_SUPPORT_AND_LIDAR_ASSOCIATION_GATE_NOT_A_PROBABILITY")
        self.assertEqual(r.contract_version, "Q02A-v2.6")   # v2.1: R1-R4; v2.2: D1/D2/D7; v2.3: C1; v2.4: E1; v2.5: F1; v2.6: G1/G2
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


def hyp(**kw):
    return ev.estimate(make(**kw))


class SensorSemanticsTests(unittest.TestCase):
    """Fable review F1-F7 (docs/evidence/q02a_fable_review_2026-10-21/REVIEW.md) as regression tests."""

    def test_production_sentinels_in_config_text(self):
        task = (ROOT / "aerial_gym/config/task_config/navrl_task_config.py").read_text()
        det = (ROOT / "aerial_gym/task/navrl_task/navrl_detector.py").read_text()
        lidar = (ROOT / "aerial_gym/config/sensor_config/lidar_config/navrl_lidar_config.py").read_text()
        for pattern in (r"camera_obstacle_width = 40", r"camera_obstacle_height = 24", r"camera_obstacle_max_range = 10\.0"):
            self.assertRegex(task, pattern)
        self.assertRegex(det, r'mode="bilinear",\s*align_corners=False')
        self.assertRegex(det, r"dist = far_plane")
        self.assertRegex(lidar, r"far_out_of_range_value = max_range")
        self.assertRegex(lidar, r"near_out_of_range_value = -max_range")

    def test_native_recovery_is_exact_and_matches_torch_operator(self):
        import torch
        rng = np.random.default_rng(11)
        x = rng.uniform(0.5, 10.0, (24, 40)); x[rng.uniform(size=x.shape) < 0.7] = 10.0
        y = torch.nn.functional.interpolate(torch.tensor(x)[None, None], size=(90, 160), mode="bilinear",
                                            align_corners=False)[0, 0].numpy()
        self.assertLess(np.abs(AV @ x @ AH.T - y).max(), 1e-12)
        rec = geo.recover_native_depth(y.astype(np.float32), (40, 24))
        self.assertLess(np.abs(rec.native - x).max(), 1e-5); self.assertFalse(rec.excluded.any())
        self.assertEqual(rec.status, geo.RECOVERY_OK)
        vv, uu = np.mgrid[0:90, 0:160]
        y2 = y.copy(); blob = (vv - 45) ** 2 + (uu - 90) ** 2 < 49; y2[blob] = 3.0 + 0.001 * (uu[blob] - 90) ** 2
        rec2 = geo.recover_native_depth(y2.astype(np.float32), (40, 24))
        seen = np.isfinite(rec2.native)
        self.assertTrue(rec2.excluded[blob].all())                    # every pasted pixel is excluded
        # v2.3 (C1): foreign = flagged along both row and column, so it marks the paste without being a
        # segmentation of it (v2.1 asserted every blob pixel foreign; this blob is constant along columns).
        rows, cols = np.nonzero(blob.any(axis=1))[0], np.nonzero(blob.any(axis=0))[0]
        box = np.zeros_like(blob); box[rows.min():rows.max() + 1, cols.min():cols.max() + 1] = True
        self.assertFalse((rec2.foreign & ~box).any())                 # inside the paste's tight box (E2)
        self.assertGreaterEqual((rec2.foreign & blob).sum(), 0.5 * blob.sum())
        self.assertLess(np.abs(rec2.native[seen] - x[seen]).max(), 1e-5)  # observed cells stay exact
        self.assertLess(seen.mean(), 1.0)                               # cells hidden by the blob are UNOBSERVED

    def test_F1_ten_metre_fill_is_no_return_not_a_surface(self):
        self.assertEqual(list(va.classify_depth_native([10.0, 9.9995, 9.5, 0.01, np.nan], PROF)),
                         [c.NO_RETURN, c.NO_RETURN, c.VALID_HIT, c.NEAR_CLIPPED, c.INVALID_SAMPLE])
        far = hyp(target_v=(12., 0., 0.), render_target=False)          # v1: OCCLUDED from the fill
        self.assertEqual(far.hypothesis, ev.UNKNOWN)                    # 0.35 m envelope falls between native rays
        self.assertIn("CAMERA_NATIVE_NO_COVERAGE", far.reasons)
        wide = np.diag([0.01, 0.04, 0.04, .1, .1, .1])                  # envelope 0.55 m: native rays reach it
        far = hyp(target_v=(12., 0., 0.), render_target=False, cov=wide)
        self.assertEqual(far.hypothesis, ev.UNKNOWN); self.assertGreater(far.camera.censored, 0)
        self.assertIn("CAMERA_NO_RETURN_BEYOND_OBSTACLE_FAR_PLANE", far.reasons)
        at10 = hyp(target_v=(10.0, 0., 0.), render_target=False)        # v1: IDENTITY_AMBIGUOUS from the fill
        self.assertNotIn(at10.hypothesis, (ev.OCCLUDED, ev.IDENTITY_AMBIGUOUS))
        near = hyp(target_v=(9., 0., 0.), render_target=False)          # space before the far plane is observed
        self.assertEqual(near.hypothesis, ev.DETECTION_MISS)

    def test_F1_real_foreground_before_the_far_plane_is_occlusion(self):
        r = hyp(target_v=(9.6, 0., 0.), render_target=False, bars=((8.6, 0., .3),))
        self.assertEqual(r.hypothesis, ev.OCCLUDED); self.assertIn("CAMERA_OBSTACLE_FOREGROUND", r.reasons)
        r = hyp(target_v=(12., 0., 0.), render_target=False, bars=((6., 0., .3),))   # target beyond 10 m, bar before
        self.assertEqual(r.hypothesis, ev.OCCLUDED)

    def test_F1_upsampling_blend_beside_the_line_of_sight_is_not_occlusion(self):
        bars = ((4., 0.5, .3),)   # bar edge 0.2 m beside the LOS at 4 m: outside the envelope (0.175 m there)
        inp = make(render_target=False, bars=bars)
        u, v, _, _ = geo.project(geo.vehicle_to_camera(np.array([8., 0., 0.]), CAM)[None], CAM)
        window = inp.depth[int(round(v[0])) - 3:int(round(v[0])) + 4, int(round(u[0])) - 6:int(round(u[0])) + 7]
        self.assertLess(window.min(), PROF.obstacle_far_m - 0.5)   # the v1 square window saw a blended "surface"
        self.assertEqual(ev.estimate(inp).hypothesis, ev.DETECTION_MISS)

    def test_F2_lidar_rays_must_pass_through_the_target_envelope(self):
        side = ((4 * math.cos(math.radians(8)), 4 * math.sin(math.radians(8)), .30),)   # Fable F2 case
        r = hyp(render_target=False, bars=side)
        self.assertNotEqual(r.hypothesis, ev.OCCLUDED); self.assertEqual(r.lidar.foreground, 0)
        crossing = hyp(render_target=False, bars=((4., 0., .3),))
        self.assertEqual(crossing.hypothesis, ev.OCCLUDED); self.assertGreater(crossing.lidar.foreground, 0)
        behind = hyp(render_target=False, bars=((9.5, 0., .3),))
        self.assertEqual(behind.hypothesis, ev.DETECTION_MISS)
        self.assertEqual((behind.camera.foreground, behind.lidar.foreground), (0, 0))

    def test_F2_adjacent_object_depends_on_belief_spread_and_spread_is_capped(self):
        beside = ((8., 0.6, .15),)
        self.assertEqual(hyp(render_target=False, bars=beside).hypothesis, ev.DETECTION_MISS)   # small sigma: outside
        wide = np.diag([0.01, 0.09, 0.09, .1, .1, .1])    # lateral sigma 0.3 m -> envelope 0.75 m
        self.assertEqual(hyp(render_target=False, bars=beside, cov=wide).hypothesis, ev.IDENTITY_AMBIGUOUS)
        huge = np.diag([0.01, 0.5, 0.5, .1, .1, .1])      # 2 * 0.71 m > 0.8 m cap
        r = hyp(render_target=False, bars=beside, cov=huge)
        self.assertEqual((r.hypothesis, r.reasons), (ev.UNKNOWN, ("BELIEF_TOO_DIFFUSE_FOR_RAY_TEST",)))

    def test_F2_each_sensor_uses_its_own_origin(self):
        d = np.array([[1., 0., 0.]])
        cam_in, cam_entry, _, _ = geo.ray_envelope(np.array(CAM.camera_offset), d, [8., 0., 0.03], 0.15)
        lid_in, lid_entry, _, _ = geo.ray_envelope(np.zeros(3), d, [8., 0., 0.03], 0.15)
        self.assertTrue(cam_in[0] and lid_in[0])
        self.assertAlmostEqual(cam_entry[0], 7.9 - 0.15)                         # camera at (0.10, 0, 0.03)
        self.assertAlmostEqual(lid_entry[0], 8.0 - math.sqrt(0.15 ** 2 - 0.03 ** 2))  # LiDAR at the vehicle centre
        off_axis_in, _, _, perp = geo.ray_envelope(np.array(CAM.camera_offset), d, [8., 0., 0.20], 0.15)
        self.assertFalse(off_axis_in[0]); self.assertAlmostEqual(perp[0], 0.17)

    def test_F3_zero_values_are_near_clipped_not_obstacles(self):
        self.assertEqual(list(va.classify_lidar([0.0, 0.1, 0.2, 5.0, 12.0, np.nan, np.inf], LID, PROF)),
                         [c.NEAR_CLIPPED, c.INVALID_SAMPLE, c.VALID_HIT, c.VALID_HIT, c.NO_RETURN,
                          c.INVALID_SAMPLE, c.INVALID_SAMPLE])
        zero_depth = hyp(render_target=False, depth=np.zeros((90, 160)))
        self.assertEqual((zero_depth.hypothesis, zero_depth.reasons), (ev.UNKNOWN, ("CAMERA_INPUT_UNUSABLE",)))
        self.assertIn("depth:INVALID", " ".join(zero_depth.validation_problems))
        zero_lidar = hyp(render_target=False, lidar=np.zeros(288))   # LiDAR invalid, camera still informs
        self.assertEqual(zero_lidar.hypothesis, ev.DETECTION_MISS); self.assertIn("LIDAR_INPUT_UNUSABLE", zero_lidar.reasons)
        base = make(render_target=False)
        on_los = np.array(base.lidar); on_los[1:3, 36] = 0.0          # near-clipped beams on the line of sight
        r = hyp(render_target=False, lidar=on_los.reshape(-1))
        self.assertEqual(r.hypothesis, ev.UNKNOWN); self.assertIn("NEAR_CLIPPED_IN_LINE_OF_SIGHT", r.reasons)
        self.assertTrue(r.near_obstacle_warning)
        elsewhere = np.array(base.lidar); elsewhere[2, 0] = 0.0       # behind the vehicle: warning only
        r = hyp(render_target=False, lidar=elsewhere.reshape(-1))
        self.assertEqual(r.hypothesis, ev.DETECTION_MISS); self.assertTrue(r.near_obstacle_warning)
        self.assertEqual(r.near_clipped_lidar_beams, 1)
        for bad in (np.full(288, np.nan), np.r_[np.full(287, 12.0), np.inf], np.r_[np.full(287, 12.0), 0.1]):
            self.assertEqual(va.validate(make(render_target=False, lidar=bad), CAM, LID).status("lidar"), va.INVALID)

    def test_F3_foreign_depth_content_fails_closed(self):
        noise = np.random.default_rng(5).uniform(1, 9, (90, 160))   # not a 40x24 upsample
        report = va.validate(make(render_target=False, depth=noise), CAM, LID)
        self.assertEqual(report.status("depth_structure"), va.FRAME_MISMATCH)
        self.assertEqual(hyp(render_target=False, depth=noise).hypothesis, ev.UNKNOWN)

    def test_F4_foreground_inside_the_envelope_vetoes_a_clear_line_of_sight(self):
        r = hyp(render_target=False, bars=((4., 0.12, .08),))   # thin bar partly across the envelope
        self.assertNotEqual(r.hypothesis, ev.DETECTION_MISS)
        # R9: the actual outcome. Two of four native rays hit the bar; the LiDAR beam passes beside it.
        self.assertEqual((r.hypothesis, r.reasons), (ev.UNKNOWN, ("SENSOR_DISAGREEMENT_CAMERA_FOREGROUND_LIDAR_CLEAR",)))
        self.assertEqual((r.camera.rays, r.camera.foreground, r.camera.clear), (4, 2, 2))

    def test_F5_target_nearer_than_predicted_is_not_called_occluded(self):
        inp = make(target_v=(7., 0., 0.), target_in_camera=False)
        mean = np.array(inp.track_mean); mean[0] += 1.0             # belief 1 m too far
        r = ev.estimate(make(target_v=(7., 0., 0.), target_in_camera=False, track_mean=mean))
        self.assertEqual(r.hypothesis, ev.UNKNOWN)          # LiDAR sees the target early, camera sees nothing
        self.assertIn("SENSOR_DISAGREEMENT_LIDAR_FOREGROUND_CAMERA_CLEAR", r.reasons)
        far = make(target_v=(10.5, 0., 0.), target_in_camera=False)
        mean = np.array(far.track_mean); mean[0] += 1.0     # beyond the camera far plane: only LiDAR informs
        r = ev.estimate(make(target_v=(10.5, 0., 0.), target_in_camera=False, track_mean=mean))
        self.assertEqual(r.hypothesis, ev.UNKNOWN); self.assertIn("FOREGROUND_COMPATIBLE_WITH_TARGET_SURFACE", r.reasons)
        self.assertFalse(r.lidar.foreground_extended)
        bar = ev.estimate(make(target_v=(10.5, 0., 0.), render_target=False, bars=((9.5, 0., .3),), track_mean=mean))
        self.assertEqual(bar.hypothesis, ev.OCCLUDED); self.assertTrue(bar.lidar.foreground_extended)  # a 3 m bar is not target-sized
        r = ev.estimate(make(target_v=(7., 0., 0.), track_mean=mean))   # camera shows the pasted target nearer
        self.assertEqual(r.hypothesis, ev.IDENTITY_AMBIGUOUS)
        self.assertIn("PASTED_TARGET_LIKE_PIXELS_IN_FOOTPRINT_WITHOUT_DETECTION", r.reasons)

    def test_F6_camera_flag_is_cross_checked_against_the_track(self):
        ok = hyp(detected=True)
        self.assertEqual((ok.hypothesis, ok.identity_status), (ev.TARGET_VISIBLE, "SUPPORTED_BY_FOOTPRINT"))
        self.assertIn("CAMERA_FLAG_VISIBLE", ok.reasons)
        wrong = hyp(detected=True, render_target=False)              # flag fires, nothing at the tracked place
        self.assertEqual((wrong.hypothesis, wrong.identity_status), (ev.IDENTITY_AMBIGUOUS, "CONTRADICTED"))
        distractors = ev.estimate(make(detected=True), profile=replace(PROF, distractors=3))
        self.assertEqual(distractors.hypothesis, ev.IDENTITY_AMBIGUOUS)

    def test_q01_frames_v2_read_only_if_present(self):
        import json, os
        run_root = Path("/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON")
        if not run_root.is_dir():
            self.skipTest("Q01-V2 raw not present on this machine")
        before = {p.name: os.stat(p).st_mtime_ns for p in run_root.iterdir()}
        rows = [json.loads(l) for l in (run_root / "sensors.jsonl").read_text().splitlines()]
        ck = lambda t: c.Clock(t["capture_sim_s"], t["delivery_sim_s"], t["provenance"], t["frame"], t["depth_convention"])
        for i in range(3):
            z = dict(np.load(run_root / ("sensor_%03d.npz" % i), allow_pickle=False))
            inp = c.EvidenceInput.from_arrays(z, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]),
                                              rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
            r = ev.estimate(inp)
            self.assertEqual(r.validation_problems, ()); self.assertEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")
        self.assertEqual(before, {p.name: os.stat(p).st_mtime_ns for p in run_root.iterdir()})


def _upsample_with_paste(native, mask, value):
    y = np.clip(AV @ native @ AH.T, 0.0, PROF.obstacle_far_m)
    y[mask] = value
    return y


class ReReviewR1toR4Tests(unittest.TestCase):
    """Q02A-v2.1: the P2 findings of the v2 re-review (docs/evidence/q02a_v2_fable_rereview_2026-10-21)."""

    VV, UU = np.mgrid[0:90, 0:160]

    def disc(self, r, cv=45, cu=80):
        return (self.VV - cv) ** 2 + (self.UU - cu) ** 2 < r * r

    def test_R1_constant_disc_inside_the_image_is_fully_excluded(self):
        bars = np.full((24, 40), 10.0); bars[:, 5:7] = 4.0; bars[:, 18:21] = 7.5
        for name, native in (("empty", np.full((24, 40), 10.0)), ("bars", bars)):
            for radius in (25, 40):
                with self.subTest(native=name, radius=radius):
                    m = self.disc(radius)
                    rec = geo.recover_native_depth(_upsample_with_paste(native, m, 2.0), (40, 24))
                    seen = np.isfinite(rec.native)
                    self.assertEqual(rec.status, geo.RECOVERY_OK)
                    self.assertTrue(rec.excluded[m].all())               # v2: 456 / 2616 pasted pixels read as obstacles
                    self.assertLess(np.abs(rec.native[seen] - native[seen]).max(), 1e-6)

    def test_R1_paste_spanning_opposite_borders_is_reported_not_read(self):
        left = np.zeros((90, 160), bool); left[:, :80] = True
        slabs = np.zeros((90, 160), bool); slabs[:, :17] = True; slabs[:, 18:] = True
        for name, m in (("disc r60", self.disc(60)), ("left half", left), ("all but one column", slabs)):
            with self.subTest(paste=name):
                depth = _upsample_with_paste(np.full((24, 40), 10.0), m, 3.0)
                self.assertEqual(geo.recover_native_depth(depth, (40, 24)).status, geo.RECOVERY_PASTE_SPANS_BORDERS)
                report = va.validate(make(render_target=False, depth=depth), CAM, LID)
                self.assertEqual(report.status("depth_structure"), va.OK)
                self.assertEqual(report.status("obstacle_depth"), va.UNAVAILABLE)
                self.assertIsNone(report.depth_native)                       # never handed to a consumer
                r = hyp(render_target=False, depth=depth)
                self.assertEqual((r.hypothesis, r.reasons), (ev.UNKNOWN, ("CAMERA_OBSTACLE_DEPTH_UNAVAILABLE",)))

    def test_R1_random_constant_pastes_are_excluded_or_reported(self):
        rng = np.random.default_rng(23)
        for k in range(40):
            native = rng.uniform(0.5, 10.0, (24, 40)); native[rng.uniform(size=native.shape) < 0.6] = 10.0
            if rng.uniform() < 0.5:
                m = self.disc(rng.uniform(3, 70), rng.uniform(-20, 110), rng.uniform(-20, 180))
            else:
                r0, c0 = rng.integers(0, 90), rng.integers(0, 160)
                m = np.zeros((90, 160), bool); m[r0:r0 + rng.integers(2, 60), c0:c0 + rng.integers(2, 100)] = True
            depth = _upsample_with_paste(native, m, rng.uniform(0.5, 9.5)).astype(np.float32)
            rec = geo.recover_native_depth(depth, (40, 24))
            with self.subTest(case=k, status=rec.status, pasted=int(m.sum())):
                if rec.status == geo.RECOVERY_OK and rec.excluded.mean() <= va.ValidationLimits().max_pasted_fraction:
                    seen = np.isfinite(rec.native)            # usable => every observed cell is the true obstacle depth
                    self.assertLess(np.abs(rec.native[seen] - native[seen]).max() if seen.any() else 0.0, 1e-3)

    def test_R1_segment_aligned_edges_and_unconverged_recovery_fail_closed(self):
        from unittest import mock
        m = np.zeros((90, 160), bool); m[32:47, 61:85] = True       # edges exactly on segment boundaries
        native = np.random.default_rng(3).uniform(0.5, 10.0, (24, 40))
        depth = np.clip(AV @ native @ AH.T, 0.0, PROF.obstacle_far_m); depth[m] += 0.5
        rec = geo.recover_native_depth(depth, (40, 24), max_passes=1)   # the window test sees them in one pass
        self.assertEqual(rec.status, geo.RECOVERY_OK); self.assertTrue(rec.excluded[m].all())
        for status, expected in ((geo.RECOVERY_NOT_CONVERGED, (va.FRAME_MISMATCH, va.INVALID)),
                                 (geo.RECOVERY_ILL_CONDITIONED, (va.OK, va.UNAVAILABLE))):
            with self.subTest(status=status), mock.patch.object(
                    va, "recover_native_depth", return_value=geo.NativeDepth(rec.native, rec.excluded, rec.foreign, 0.0, status)):
                structure, obstacle, _ = va._depth_structure(depth, PROF, 0.5)
                self.assertEqual((structure.status, obstacle.status), expected)

    def test_R2_unrelated_object_at_the_same_range_does_not_extend_the_foreground(self):
        base = make(target_v=(10.5, 0., 0.), target_in_camera=False)
        mean = np.array(base.track_mean); mean[0] += 1.0
        for bars in (((0., 10.3, .3),), ((-10.3, 0., .3),)):              # 90 deg left, directly behind the vehicle
            with self.subTest(bars=bars):
                r = ev.estimate(make(target_v=(10.5, 0., 0.), target_in_camera=False, track_mean=mean, bars=bars))
                self.assertEqual(r.hypothesis, ev.UNKNOWN); self.assertFalse(r.lidar.foreground_extended)
                self.assertIn("FOREGROUND_COMPATIBLE_WITH_TARGET_SURFACE", r.reasons)

    def test_R3_identity_support_needs_pasted_depth_inside_the_track_interval(self):
        ok = hyp(detected=True)
        self.assertEqual(ok.identity_status, "SUPPORTED_BY_FOOTPRINT"); self.assertIn("TARGET_SPHERE_AT_TRACK_RANGE", ok.reasons)
        self.assertGreater(ok.pasted_pixels_at_track_range, 0); self.assertGreater(ok.target_pixels_at_track_range, 0)
        near = make(target_v=(3., 0., 0.), detected=True)
        mean = np.array(near.track_mean); mean[0] += 5.0                  # target pasted at 3 m, belief at 8 m
        r = ev.estimate(make(target_v=(3., 0., 0.), detected=True, track_mean=mean))
        self.assertEqual((r.hypothesis, r.identity_status), (ev.IDENTITY_AMBIGUOUS, "RANGE_CONTRADICTED"))
        self.assertEqual(r.pasted_pixels_at_track_range, 0); self.assertGreater(r.pasted_pixels_in_footprint, 0)
        for bars in (((8., 0., .15),), ((8., 0., .4),)):                # an obstacle surface at the track range
            with self.subTest(bars=bars):
                r = hyp(render_target=False, bars=bars, detected=True)
                self.assertNotEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")
                self.assertIn("SURFACE_AT_TRACK_RANGE_NOT_IDENTITY_EVIDENCE", r.reasons)
                self.assertIn("NO_PASTED_PIXELS_IN_IMAGE", r.reasons)

    def test_R4_close_target_keeps_the_camera_flag_when_the_paste_dominates(self):
        for rng_ in (0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
            with self.subTest(range_m=rng_):
                flagged = hyp(target_v=(rng_, 0., 0.), detected=True)       # v2: UNKNOWN at 0.35-0.45 m
                self.assertEqual((flagged.hypothesis, flagged.identity_status), (ev.TARGET_VISIBLE, "SUPPORTED_BY_FOOTPRINT"))
                report = va.validate(make(target_v=(rng_, 0., 0.)), CAM, LID)
                self.assertEqual(report.status("depth_structure"), va.OK)
                if rng_ <= 0.45:                                            # too little obstacle render left to read
                    self.assertEqual(report.status("obstacle_depth"), va.UNAVAILABLE)
                    self.assertEqual(hyp(target_v=(rng_, 0., 0.)).reasons, ("CAMERA_OBSTACLE_DEPTH_UNAVAILABLE",))


class DeltaReviewD1D2Tests(unittest.TestCase):
    """Q02A-v2.2: the P1 findings of the v2.1 delta review (docs/evidence/q02a_v21_fable_delta_review_2026-10-22)."""

    def test_D1_contact_range_target_is_never_read_as_obstacle_depth(self):
        contact = 0
        for rng_ in np.arange(0.255, 0.355, 0.005):
            for lat, z in ((0.0, 0.0), (0.03, 0.0), (0.0, 0.05), (0.03, 0.05)):
                tv = (float(rng_), lat, z)
                report = va.validate(make(target_v=tv), CAM, LID)
                with self.subTest(target=tv, obstacle_depth=report.status("obstacle_depth")):
                    if report.obstacle_depth_usable:            # empty scene: every observed cell is the 10 m fill
                        seen = np.isfinite(report.depth_native)
                        self.assertGreaterEqual(report.depth_native[seen].min(), PROF.obstacle_far_m - 1e-3)
                    contact += report.camera_contact_range
        self.assertGreater(contact, 0)                           # v2.1: 16 such frames read as obstacle depth
        flagged = hyp(target_v=(0.28, 0., 0.), detected=True)
        self.assertEqual(flagged.hypothesis, ev.TARGET_VISIBLE); self.assertTrue(flagged.near_obstacle_warning)
        self.assertEqual(hyp(target_v=(0.28, 0., 0.)).reasons, ("CAMERA_OBSTACLE_DEPTH_UNAVAILABLE",))

    def test_D2_no_return_fill_pixels_never_count_as_pasted(self):
        inp = make(target_v=(8., 0., 0.))
        report = va.validate(inp, CAM, LID)
        self.assertTrue(report.depth_pasted.any())
        self.assertFalse((report.depth_pasted & (np.asarray(inp.depth) >= PROF.obstacle_far_m - 1e-3)).any())
        for rng_ in (10.0, 9.8, 9.5):                            # v2.1: SUPPORTED from fill pixels alone
            base = make(target_v=(rng_, 0., 0.), detected=True)
            mean = np.array(base.track_mean); mean[1] += 0.5       # track 0.5 m beside the pasted target
            with self.subTest(range_m=rng_):
                r = ev.estimate(make(target_v=(rng_, 0., 0.), detected=True, track_mean=mean))
                self.assertNotEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")
                self.assertEqual(r.pasted_pixels_at_track_range, 0)
        self.assertEqual(hyp(target_v=(10.0, 0., 0.), detected=True).identity_status, "SUPPORTED_BY_FOOTPRINT")

    def test_D7_stale_or_diffuse_belief_makes_the_flag_footprint_not_testable(self):
        for kw in (dict(age=5.0), dict(cov=np.diag([4.0, 4.0, 4.0, .1, .1, .1]))):
            with self.subTest(**{k: str(v)[:20] for k, v in kw.items()}):
                r = hyp(detected=True, **kw)
                self.assertEqual((r.hypothesis, r.identity_status), (ev.TARGET_VISIBLE, "NOT_TESTABLE"))
                self.assertIn("BELIEF_STALE_OR_DIFFUSE_FOOTPRINT_NOT_TESTABLE", r.reasons)


class ConfirmationC1Tests(unittest.TestCase):
    """Q02A-v2.3: the P1 finding of the v2.2 confirmation review (docs/evidence/q02a_v22_fable_confirmation_review_2026-10-22)."""

    def test_C1_blend_pixels_beside_a_paste_never_grant_identity_support(self):
        for rng_, gap in ((8.0, 0.1), (5.0, 0.2), (3.0, 0.3)):
            bars = ((rng_, 0.15 + gap + 0.3, 0.3),)               # bar at the target range, its edge `gap` beside it
            base = make(target_v=(rng_, 0., 0.), detected=True, bars=bars)
            self.assertEqual(ev.estimate(base).identity_status, "SUPPORTED_BY_FOOTPRINT")   # correct track
            for err in (0.5, 0.6, 0.7, 0.8):                       # track on the bar, no target pixel in the envelope
                mean = np.array(base.track_mean); mean[1] += err
                with self.subTest(range_m=rng_, gap=gap, track_error=err):
                    r = ev.estimate(make(target_v=(rng_, 0., 0.), detected=True, bars=bars, track_mean=mean))
                    self.assertNotEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")   # v2.2: SUPPORTED from blend pixels


class FinalReviewE1Tests(unittest.TestCase):
    """Q02A-v2.4: the P1 finding of the v2.3 final review (docs/evidence/q02a_v23_fable_final_review_2026-10-22)."""

    def test_E1_obstacle_behind_the_paste_never_supports_a_track_that_is_too_far(self):
        for rng_, gap in ((3.0, 1.0), (2.0, 0.6), (8.0, 1.0)):
            bars = ((rng_ + gap + 0.3, 0.0, 0.3),)                 # bar surface `gap` behind the target centre
            base = make(target_v=(rng_, 0., 0.), detected=True, bars=bars)
            ok = ev.estimate(base)
            self.assertEqual(ok.identity_status, "SUPPORTED_BY_FOOTPRINT"); self.assertEqual(ok.pasted_pixels_nearer_than_track, 0)
            for err in (0.8, 1.0, 1.3):                             # track beyond the target, at the bar's range
                mean = np.array(base.track_mean); mean[0] += err
                with self.subTest(range_m=rng_, behind=gap, range_error=err):
                    r = ev.estimate(make(target_v=(rng_, 0., 0.), detected=True, bars=bars, track_mean=mean))
                    self.assertEqual(r.identity_status, "RANGE_CONTRADICTED")   # v2.3: SUPPORTED from box-corner pixels
                    self.assertIn("TARGET_SPHERE_NEARER_THAN_TRACK", r.reasons)       # v2.4 reason: PASTED_DEPTH_MOSTLY_NEARER
                    self.assertEqual(r.target_pixels_at_track_range, 0)


def _box_hits(origin, dirs, boxes):
    """First entry distance into axis-aligned boxes (cx, cy, hx, hy, z0, z1) in the vehicle frame."""
    t_all, o = np.full(dirs.shape[:-1], np.inf), np.asarray(origin, float)
    for cx, cy, hx, hy, z0, z1 in boxes:
        lo, hi = np.array([cx - hx, cy - hy, z0]), np.array([cx + hx, cy + hy, z1])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (lo - o) / dirs, (hi - o) / dirs
        tmin, tmax = np.max(np.minimum(t1, t2), axis=-1), np.min(np.maximum(t1, t2), axis=-1)
        t_all = np.minimum(t_all, np.where((tmax >= np.maximum(tmin, 0.0)) & (tmin > 0.0), tmin, np.inf))
    return t_all


def box_scene(target_v, boxes):
    """Like scene(), with finite box bars (a bar's top corner can lie inside the target's silhouette)."""
    cam_o = np.array(CAM.camera_offset)
    native = np.minimum(_box_hits(cam_o, geo.native_camera_rays(CAM, PROF.obstacle_native_wh), boxes), PROF.obstacle_far_m)
    depth = np.clip(AV @ native @ AH.T, 0.0, PROF.obstacle_far_m)
    vv, uu = np.mgrid[0:90, 0:160]
    rays = geo.pixel_rays(uu, vv, CAM)
    t_tgt = geo.sphere_first_hit(cam_o, rays, target_v, CAM.target_radius)
    depth = np.where(np.isfinite(t_tgt) & (t_tgt < _box_hits(cam_o, rays, boxes)), t_tgt, depth)
    dirs = geo.lidar_directions(LID)
    t = np.minimum(_box_hits(np.zeros(3), dirs, boxes), geo.sphere_first_hit(np.zeros(3), dirs, target_v, LID.target_proxy_radius_m))
    lidar = np.where(t < LID.min_range_m, 0.0, np.where(t >= LID.max_range_m, LID.max_range_m, t))
    return depth.astype(np.float32), lidar.astype(np.float32).reshape(-1)


class ReviewF1Tests(unittest.TestCase):
    """Q02A-v2.5: the P1 finding of the v2.4 confirmation review (docs/evidence/q02a_v24_fable_confirmation_review_2026-10-22)."""

    def test_F1_front_bar_corner_inside_the_silhouette_never_supports_a_track_at_the_bar(self):
        # The reviewer's diagnostic case: target 4 m, a 2 m bar (top 1 m above the camera) 1.5 m in front,
        # its top corner inside the target's silhouette; v2.4 gave SUPPORTED from the bar's own pixels.
        cam_o = np.array(CAM.camera_offset); x_f, half = 2.5, 0.3
        corner = np.array([x_f, 0.0, 1.0]); c = (corner - cam_o) / np.linalg.norm(corner - cam_o)
        ey = np.cross([0, 0, 1.0], c); ey /= np.linalg.norm(ey); ez = np.cross(c, ey)
        th = CAM.target_radius / 4.0
        dr = c - 0.7 * th * ey - 0.7 * th * ez; dr /= np.linalg.norm(dr)
        tv = cam_o + 4.0 * dr
        boxes = [(x_f + half, -half, half, half, -1.0, 1.0)]
        depth, lidar = box_scene(tv, boxes)
        ok = ev.estimate(make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar))
        self.assertEqual(ok.identity_status, "SUPPORTED_BY_FOOTPRINT"); self.assertTrue(ok.target_sphere_found)
        for delta in (-0.3, 0.0, 0.3):                              # track on the target's ray at the bar face
            face = cam_o + ((x_f - cam_o[0]) / dr[0] + delta) * dr
            with self.subTest(delta=delta):
                r = ev.estimate(make(target_v=tuple(face), detected=True, depth=depth, lidar=lidar))
                self.assertNotEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")      # v2.4: SUPPORTED
                self.assertEqual(r.target_pixels_at_track_range, 0)


def _front_corner_scene(R, d_f, a, b, top=1.0, half=0.3):
    """Target behind a front bar whose top corner lies inside the target's silhouette (box_scene geometry)."""
    cam_o = np.array(CAM.camera_offset); x_f = R - d_f
    corner = np.array([x_f, 0.0, top]); c = (corner - cam_o) / np.linalg.norm(corner - cam_o)
    ey = np.cross([0, 0, 1.0], c); ey /= np.linalg.norm(ey); ez = np.cross(c, ey)
    th = CAM.target_radius / R
    dr = c + a * th * ey + b * th * ez; dr /= np.linalg.norm(dr)
    tv = cam_o + R * dr
    depth, lidar = box_scene(tv, [(x_f + half, -half, half, half, -1.0, top)])
    return tv, dr, depth, lidar


def box_target_scene(centre, half=(0.14, 0.14, 0.06)):
    """The physical-dynamics box target (axis-aligned) pasted with exact depth over an empty obstacle render."""
    cam_o = np.array(CAM.camera_offset); vv, uu = np.mgrid[0:90, 0:160]
    c, h = np.asarray(centre, float), np.asarray(half, float)
    t = _box_hits(cam_o, geo.pixel_rays(uu, vv, CAM), [(c[0], c[1], h[0], h[1], c[2] - h[2], c[2] + h[2])])
    base = np.clip(AV @ np.full((24, 40), 10.0) @ AH.T, 0.0, PROF.obstacle_far_m)
    return np.where(np.isfinite(t), t, base).astype(np.float32)


class IdentityContractV26Tests(unittest.TestCase):
    """Q02A-v2.6: Opus review G1/G2 and the split identity contract (docs/evidence/q02a_v25_opus_final_review_2026-10-22)."""

    def test_A_G1_blend_pixels_cannot_promote_a_lower_ranked_sphere(self):
        # The reviewer's fixture: target 3 m, a 2 m bar 0.4 m in front, corner offsets a = b = -0.7 (the v2.5 L5 case).
        # v2.5 scored inliers minus farther pixels; 22 obstacle blends vetoed the true sphere and a 2-pixel sphere on
        # blends at 7.3 m won, so the correct track was RANGE_CONTRADICTED and tracks at 6.3/6.6 m were SUPPORTED.
        tv, dr, depth, lidar = _front_corner_scene(3.0, 0.4, -0.7, -0.7)
        cam_o = np.array(CAM.camera_offset)
        report = va.validate(make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar), CAM, LID)
        fit = geo.fit_target_sphere(depth, report.depth_pasted, CAM, CAM.target_radius)
        self.assertLess(np.linalg.norm(fit.centre - tv), 0.01)          # ranking by consistency picks the true sphere
        self.assertEqual((fit.status, fit.contradictions), (geo.SPHERE_OK, 0))
        self.assertGreater(fit.farther_unexplained, 0)                   # blends are counted, not used as a veto
        ok = ev.estimate(make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar))
        self.assertEqual(ok.identity_status, "SUPPORTED_BY_FOOTPRINT")
        for rt, cov in ((6.3, 0.04), (6.6, 0.01), (7.3, 0.01)):
            with self.subTest(track_range=rt):
                r = ev.estimate(make(target_v=tuple(cam_o + rt * dr), detected=True, depth=depth, lidar=lidar, cov_pos=cov))
                self.assertNotEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")
                self.assertEqual(r.target_pixels_at_track_range, 0)

    def test_A_G1_a_high_ranked_candidate_with_its_own_contradiction_is_held_ambiguous(self):
        # Adversarial depth: a ring of a sphere at 3 m (more inliers) around the disc of a second sphere at 6 m that
        # shows through its core -- a target-like surface behind the best sphere, impossible for one target.
        cam_o = np.array(CAM.camera_offset); vv, uu = np.mgrid[0:90, 0:160]; rays = geo.pixel_rays(uu, vv, CAM)
        front, back = np.array([3.0, 0.0, 0.0]), np.array([6.0, 0.0, 0.0])
        t_front = geo.sphere_first_hit(cam_o, rays, front, CAM.target_radius)
        t_back = geo.sphere_first_hit(cam_o, rays, back, CAM.target_radius)
        depth = np.clip(AV @ np.full((24, 40), 10.0) @ AH.T, 0.0, 10.0)
        depth = np.where(np.isfinite(t_back), t_back, np.where(np.isfinite(t_front), t_front, depth)).astype(np.float32)
        report = va.validate(make(target_v=tuple(front), detected=True, depth=depth), CAM, LID)
        fit = geo.fit_target_sphere(depth, report.depth_pasted, CAM, CAM.target_radius)
        self.assertLess(np.linalg.norm(fit.centre - front), 0.01)        # the higher-ranked candidate
        self.assertGreater(fit.contradictions, 0); self.assertEqual(fit.status, geo.SPHERE_VETOED)
        for centre in (front, back):                                      # neither candidate is supported
            with self.subTest(track=tuple(centre)):
                r = ev.estimate(make(target_v=tuple(centre), detected=True, depth=depth))
                self.assertEqual((r.identity_status, r.identity_verdict), ("AMBIGUOUS", "AMBIGUOUS"))

    def test_B_G2_target_geometry_is_a_declared_contract(self):
        sphere = hyp(detected=True)
        self.assertEqual((sphere.identity_status, sphere.geometry_support), ("SUPPORTED_BY_FOOTPRINT", "FOOTPRINT_CONSISTENT"))
        self.assertEqual(sphere.identity_verdict, "CONSISTENT_NOT_CONFIRMED")          # geometry is not identity
        for centre in ((4.0, 0.2, 0.0), (2.0, 0.0, 0.1)):                              # box target, SPHERE declared
            with self.subTest(box=centre):
                r = ev.estimate(make(target_v=centre, detected=True, depth=box_target_scene(centre)))
                self.assertNotEqual(r.identity_status, "SUPPORTED_BY_FOOTPRINT")      # v2.5: SUPPORTED
                self.assertIn(r.identity_verdict, ("AMBIGUOUS", "NOT_TESTABLE"))
        for geometry in (c.TARGET_ORIENTED_BOX, c.TARGET_MESH, c.TARGET_UNKNOWN):      # declared non-sphere / unknown
            with self.subTest(geometry=geometry):
                r = ev.estimate(make(detected=True), profile=replace(PROF, target_geometry=geometry))
                self.assertEqual((r.identity_status, r.target_geometry), ("NOT_TESTABLE", geometry))
                self.assertIn("TARGET_GEOMETRY_MODEL_NOT_APPLICABLE", r.reasons)

    def test_B_identity_record_never_confirms_identity(self):
        cases = [hyp(detected=True), hyp(detected=True, render_target=False), hyp(render_target=False),
                 hyp(detected=True, age=5.0), ev.estimate(make(detected=True), profile=replace(PROF, distractors=2))]
        for r in cases:
            with self.subTest(identity=r.identity_status):
                self.assertNotIn("CONFIRMED", r.identity_verdict.replace("NOT_CONFIRMED", ""))
                self.assertEqual(r.identity_confirmation, "NOT_AVAILABLE_SINGLE_FRAME_ORACLE_FLAG")
                self.assertEqual((r.detector_provenance, r.schema_version), ("ORACLE_TRUE_POSE_RASTER", "Q02A-RECORD-3"))
        self.assertEqual(cases[0].detection_support, "CAMERA_FLAG"); self.assertEqual(cases[2].detection_support, "NONE")
        self.assertEqual(cases[2].geometry_support, "NOT_APPLICABLE")
        self.assertIn(cases[0].tracker_association, ("UPSTREAM_MEASUREMENT_UPDATE_NOT_VERIFIED", "PREDICTION_ONLY"))
        self.assertEqual(cases[4].identity_verdict, "AMBIGUOUS")                       # distractors: unverifiable

    def test_B_declared_geometry_matches_the_renderer_configuration_text(self):
        det = (ROOT / "aerial_gym/task/navrl_task/navrl_detector.py").read_text()
        self.assertRegex(det, r'NAVRL_TARGET_DYNAMICS", "legacy"\)\.strip\(\)\.lower\(\) == "physical"')
        self.assertRegex(det, r"target_half_extents = wp\.vec3\(0\.14, 0\.14, 0\.06\)")
        self.assertRegex(det, r"def attach_dynamic_mesh_treatment")
        q01 = json.loads((ROOT / "docs/plans/boia_shadow_q01_v2.json").read_text())
        flat = json.dumps(q01)
        self.assertIn('"NAVRL_TARGET_DYNAMICS": "bounded"', flat)                       # not "physical" -> sphere
        self.assertEqual(PROF.target_geometry, c.TARGET_SPHERE)

    def test_D_small_far_target_and_contradictory_ranking(self):
        far = make(target_v=(15.0, 0.3, 0.0), detected=True)
        ok = ev.estimate(far)
        self.assertIn(ok.identity_status, ("SUPPORTED_BY_FOOTPRINT", "NOT_TESTABLE"))
        mean = np.array(far.track_mean); mean[0] -= 2.0                               # track 2 m too near
        self.assertNotEqual(ev.estimate(make(target_v=(15.0, 0.3, 0.0), detected=True, track_mean=mean)).identity_status,
                            "SUPPORTED_BY_FOOTPRINT")


if __name__ == "__main__":
    unittest.main()
