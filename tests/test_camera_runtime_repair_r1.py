"""Camera-runtime repair r1 (2026-11-13): the independent review's P1/P2 findings F1-F10 and cheap P3 items.

F1  the invalid-pixel floor/ceiling plane rule is opt-in (default off): a real obstacle closer than MinZ whose
    top lies a few cm below the camera stays a near surface.
F2  the geofence scales the horizontal command by one direction-preserving factor; 2-D rollouts with a
    total-deceleration-limited double integrator (F9: a model independent of the clamp formula).
F3  coverage False over a valid depth return refuses the frame instead of erasing the depth.
F4  the tilted (z-buffer) path classifies a trained pixel by the share of invalid samples (0.5).
F5  a sensor-invalid pixel that the production colour rule scores as goal-coloured becomes no-return.
F6  an enabled floor/ceiling filter needs the operator's estimator-height assertion, else frames fail closed.
F7  ROS package records; F8  offboard publication needs an enabled fence or an explicit waiver.
F10 Gazebo +inf is no-return. P3: adapter/session in the contract sources; geofence bounds; disabled values.

Every check has a control. Synthetic scenes only (the review's probes, ~/motar_tmp/ros2_runtime_fix_20261112/
review_r1/probes): nothing here is real-sensor, PX4, ROS-graph, Gazebo or flight evidence. The geofence
latencies and the estimator-height assertions are CPU fixtures, not measurements.
"""
import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'tests'))
import camera_policy_contract as C
import camera_policy_runtime as R
import camera_ros2_adapter as A
import camera_ros2_px4 as P
import camera_ros2_px4_node as N
from camera_ros2_session import RosPolicySession
import test_camera_runtime_fixes as F
import test_camera_runtime_fixes_r2 as G
from test_camera_ros2_px4 import snapshot

CHECKPOINT_DIR = Path('/home/fair/motar_tmp/camk2v3_20261108/train_s1/checkpoint')
PX4_MSG_SOURCE = Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/msg')
DV = G.DV                                   # production blob_near, loaded privately by the r2 module
A_BRAKE = P.GOVERNOR_BRAKE_MPS2


def box_frame(source, height, pitch_deg, z_top, box_x=.4, half_y=.5, floor=False, wall_x=10., rgb=(110, 110, 110)):
    """The review's probe scene (probe_scan.py): a wall (valid), an optional textured floor (valid) and a box face
    at world x = box_x (0.3 m optical ahead of the camera at x = 0.1), |y| <= half_y, 0 <= z <= z_top, closer
    than MinZ and therefore reported invalid (depth 0) as the OAK-D does."""
    K = F.source_intrinsics(source)
    sw, sh = K['width'], K['height']
    uu, vv = np.meshgrid(np.arange(sw, dtype=np.float64), np.arange(sh, dtype=np.float64))
    d_cam = np.stack([np.ones_like(uu), -(uu - K['cx']) / K['fx'], -(vv - K['cy']) / K['fy']], axis=-1)
    d_cam /= np.linalg.norm(d_cam, axis=-1, keepdims=True)
    rot = F.body_rotation(0., pitch_deg)
    o = np.array([0., 0., height]) + rot @ np.array(F.MOUNT)
    d = d_cam @ rot.T
    with np.errstate(divide='ignore', invalid='ignore'):
        t_wall = np.where(d[..., 0] > 1e-9, (wall_x - o[0]) / d[..., 0], np.inf)
        t_floor = np.where(d[..., 2] < -1e-9, (0. - o[2]) / d[..., 2], np.inf) if floor else np.full(t_wall.shape, np.inf)
        tb = np.where(d[..., 0] > 1e-9, (box_x - o[0]) / d[..., 0], np.inf)
    hy, hz = o[1] + tb * d[..., 1], o[2] + tb * d[..., 2]
    tb = np.where((np.abs(hy) <= half_y) & (hz >= 0.) & (hz <= z_top), tb, np.inf)
    t = np.minimum(np.minimum(t_wall, t_floor), tb)
    box = tb <= t
    z = np.where(box, 0., t * d_cam[..., 0]).astype(np.float32)
    image = np.full((sh, sw, 3), 110, np.uint8)
    image[box] = rgb
    frame = {'rgb': image, 'depth_z_m': z, 'invalid': box.copy(), 'rgb_t': 1., 'depth_t': 1., 'intrinsics': K,
             'aligned': True, 'rectified': True, 'mount_flu_m': list(F.MOUNT), 'camera_to_body_rotation': np.eye(3)}
    return frame, int(box.sum())


def speckled_slab(pitch_deg, share, seed=3):
    """The review's probe C: a slab 0.3 m ahead (|y| <= 0.3, full height), invalid, with a share of spurious
    valid speckle at 6 m (stereo matchers produce such matches below MinZ)."""
    f, _ = box_frame('oakd_like', 1., pitch_deg, 3.0, half_y=.3)
    speck = f['invalid'] & (np.random.default_rng(seed).random(f['invalid'].shape) < share)
    f['invalid'] = f['invalid'] & ~speck
    f['depth_z_m'] = np.where(speck, np.float32(6.), f['depth_z_m']).astype(np.float32)
    return f


def trained_cols(optical_z, half_y):
    u = np.arange(160)
    return np.flatnonzero(np.abs((u - 79.5) / F.TRAINED['fx'] * optical_z) <= half_y)


def column_minima(frame, state, contract):
    _, depth, counts, projection = G.geometry_runtime(contract).preprocess(frame, state)
    return depth[0].amin(dim=0).numpy(), counts, projection


def legacy_column_minima(frame, state):
    """The pre-r1 (fc8aa4d) projection with production blob_near: the 'any valid point wins' z-buffer rule."""
    import torch
    c = F.contract()
    _, ranges, bad, _, cosine, _ = F.legacy_project_rgbd(frame, state, c)
    d = c['depth']
    depth, near = DV.apply_mask(torch.from_numpy(ranges), torch.from_numpy(bad), torch.from_numpy(cosine),
                                min_z_m=d['minz_m'], window=d['window'], density=d['density'])
    return depth[0].amin(dim=0).numpy(), int(near.sum())


# ----------------------------------------------------------------------------------------- F1 plane rule
class F1NearObstacleBelowCameraHeight(unittest.TestCase):
    TOPS = (1.00, .98, .90)                       # 3, 5 and 13 cm below the 1.03 m camera
    SOURCES = (('trained_equal', 0.), ('oakd_like', 0.), ('oakd_like', 1.), ('oakd_like', 3.))

    def test_default_contract_keeps_a_box_face_below_camera_height_near(self):
        default = F.contract(ground=True)        # floor filter ON, plane rule OFF: the default contract
        cols = trained_cols(.3, .5)
        cases = floor_removed = 0
        for source, pitch in self.SOURCES:
            for top in self.TOPS:
                for floor in (False, True):
                    f, n = box_frame(source, 1., pitch, top, floor=floor)
                    state = F.scene_state(1., pitch_deg=pitch)
                    colmin, counts, projection = column_minima(f, state, default)
                    label = (source, pitch, top, floor, projection['projection'])
                    with self.subTest(label=label):
                        self.assertGreater(n, 0)
                        self.assertGreater(counts['near_pixels'], 0, label)
                        self.assertTrue(.40 <= colmin[cols].min() <= .48, (label, colmin[cols].min()))
                        self.assertTrue(np.all(colmin[cols] < 1.), (label, colmin[cols].max()))   # no column reads the wall
                        self.assertEqual(counts['height_filter']['ground']['removed_invalid_pixels'], 0)
                    if floor:   # the valid floor is still removed where it is visible (the box can hide it)
                        floor_removed += counts['height_filter']['ground']['removed_valid_pixels']
                    cases += 1
        self.assertEqual(cases, 24)
        self.assertGreater(floor_removed, 0)
        self.assertFalse(C.GROUND_DEFAULT['invalid_plane_rule'])          # the production default is the rule OFF
        self.assertFalse(C.CEILING_DEFAULT['invalid_plane_rule'])

    def test_negative_control_the_plane_rule_erases_the_same_box(self):
        # The review's failing inputs: with the (now opt-in) rule ON the box vanishes behind the 9.9 m wall.
        rule_on = F.contract(ground=True, plane_rule=True)
        cols = trained_cols(.3, .5)
        for source, pitch, top in (('trained_equal', 0., .98), ('oakd_like', 1., .98), ('oakd_like', 3., .90),
                                   ('trained_equal', 0., 1.00)):
            f, _ = box_frame(source, 1., pitch, top, floor=True)
            colmin, counts, _ = column_minima(f, F.scene_state(1., pitch_deg=pitch), rule_on)
            with self.subTest(source=source, pitch=pitch, top=top):
                self.assertEqual(counts['near_pixels'], 0)
                self.assertGreater(colmin[cols].min(), 9.)
                self.assertGreater(counts['height_filter']['ground']['removed_invalid_pixels'], 0)
        # Positive control: a box reaching above the camera is kept even with the rule on.
        f, _ = box_frame('oakd_like', 1., 1., 1.20, floor=True)
        colmin, counts, _ = column_minima(f, F.scene_state(1., pitch_deg=1.), rule_on)
        self.assertGreater(counts['near_pixels'], 0)
        self.assertLessEqual(colmin[cols].min(), .45)

    def test_contract_field_is_a_validated_opt_in(self):
        value = C._metadata(1, ROOT)
        value['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        self.assertEqual(value['ground']['invalid_plane_rule'], False)
        self.assertEqual(C.validate_contract(value, verify_files=False), [])
        ok = copy.deepcopy(value)
        ok['ground']['invalid_plane_rule'] = True
        ok['ceiling'].update(enabled=True, height_m=3., invalid_plane_rule=True)
        self.assertEqual(C.validate_contract(ok, verify_files=False), [])          # positive control
        for key, change in (('ground', dict(invalid_plane_rule=1)), ('ground', dict(enabled=False, invalid_plane_rule=True)),
                            ('ceiling', dict(invalid_plane_rule=True)), ('ceiling', dict(invalid_plane_rule=None))):
            bad = copy.deepcopy(value)
            bad[key].update(change)
            with self.subTest(key=key, change=change):
                self.assertTrue(C.validate_contract(bad, verify_files=False))
        bad = copy.deepcopy(value)
        del bad['ground']['invalid_plane_rule']
        self.assertTrue(C.validate_contract(bad, verify_files=False))
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(C, 'build_contract', return_value=value) as build, \
                    contextlib.redirect_stdout(io.StringIO()):
                C.main(['--output', str(Path(directory) / 'a.json'), '--seed', '1', '--ceiling-height-m', '3',
                        '--ground-invalid-plane-rule', '--ceiling-invalid-plane-rule'])
            self.assertIs(build.call_args.kwargs['ground']['invalid_plane_rule'], True)
            self.assertIs(build.call_args.kwargs['ceiling']['invalid_plane_rule'], True)
            with mock.patch.object(C, 'build_contract', return_value=value) as build, \
                    contextlib.redirect_stdout(io.StringIO()):
                C.main(['--output', str(Path(directory) / 'b.json'), '--seed', '1'])
            self.assertIs(build.call_args.kwargs['ground']['invalid_plane_rule'], False)
            with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                C.main(['--output', str(Path(directory) / 'c.json'), '--seed', '1', '--ceiling-invalid-plane-rule'])


# -------------------------------------------------------------------------------------------- F2 geofence
def braking_rollout(p0, u, tau, a, box, dt=1e-3):
    """Independent vehicle model (F9): a point mass holds the published horizontal velocity u for the latency
    tau, then decelerates at TOTAL (norm) deceleration a opposite its velocity until it stops, as a tilt-limited
    multirotor does (PX4's horizontal acceleration limit is a norm). Exact constant-acceleration steps; the clamp
    formula is not used. Returns the largest excursion beyond any wall of box = (x_min, x_max, y_min, y_max)
    along the trajectory (> 0 means crossed)."""
    x, y = float(p0[0]), float(p0[1])
    ux, uy = float(u[0]), float(u[1])
    x_min, x_max, y_min, y_max = box

    def beyond(px, py):
        return max(px - x_max, x_min - px, py - y_max, y_min - py)

    worst, t = beyond(x, y), 0.
    while t < tau - 1e-15:
        step = min(dt, tau - t)
        x, y, t = x + ux * step, y + uy * step, t + step
        worst = max(worst, beyond(x, y))
    speed = math.hypot(ux, uy)
    if speed == 0.:
        return worst
    ex, ey = ux / speed, uy / speed
    while speed > 0.:
        step = min(dt, speed / a)
        travel = speed * step - .5 * a * step * step
        x, y = x + ex * travel, y + ey * travel
        speed = 0. if step == speed / a else speed - a * step
        worst = max(worst, beyond(x, y))
    return worst


def legacy_per_axis_limit(fence, v, p):
    """The pre-repair (r2) clamp, reproduced as the negative control: each component toward its wall limited
    independently to -a tau + sqrt(a^2 tau^2 + 2 a d), i.e. assuming deceleration a on both axes at once."""
    out = list(v)
    for axis, (low, high) in enumerate(((fence.x_min_m, fence.x_max_m), (fence.y_min_m, fence.y_max_m))):
        if v[axis] == 0.:
            continue
        d = max(0., high - p[axis]) if v[axis] > 0 else max(0., p[axis] - low)
        reach = fence.decel_mps2 * fence.latency_s
        v_max = max(0., -reach + math.sqrt(reach * reach + 2. * fence.decel_mps2 * d))
        if abs(v[axis]) > v_max:
            out[axis] = math.copysign(v_max, v[axis])
    return out


class F2DirectionPreservingGeofence(unittest.TestCase):
    BOX = (-5., 5., -5., 5.)

    def cases(self):
        for tau in (.2, .4):
            for d in (.4, .5, 1.5, 3.):
                for speed in (2.5, 3.54):
                    for name, p, angle in (('corner diagonal', (5. - d, 5. - d), 45.),
                                           ('corner 30 deg', (5. - d, 5. - d), 30.),
                                           ('corner 60 deg', (5. - d, 5. - d), 60.),
                                           ('corner asymmetric', (5. - d, 5. - 2. * d), 45.),
                                           ('opposite corner diagonal', (-5. + d, -5. + d), 225.),
                                           ('edge single axis', (5. - d, 0.), 0.),
                                           ('edge single axis -y', (0., -5. + d), 270.),
                                           ('edge diagonal', (5. - d, 0.), 45.)):
                        v = (speed * math.cos(math.radians(angle)), speed * math.sin(math.radians(angle)))
                        if abs(v[0]) < 1e-12:
                            v = (0., v[1])
                        if abs(v[1]) < 1e-12:
                            v = (v[0], 0.)
                        yield tau, d, speed, name, p, v

    def test_no_crossing_at_corners_and_edges_and_the_direction_is_kept(self):
        clamped = unclamped = 0
        for tau, d, speed, name, p, v in self.cases():
            fence = P.Geofence.from_config(G.fence(latency=tau))
            out, axes = fence.limit(list(v), p)
            label = (tau, d, speed, name)
            with self.subTest(label=label):
                overshoot = braking_rollout(p, out, tau, A_BRAKE, self.BOX)
                self.assertLessEqual(overshoot, 1e-9, label)
                norm_in, norm_out = math.hypot(*v), math.hypot(*out)
                self.assertLessEqual(norm_out, norm_in * (1. + 1e-15))
                self.assertLessEqual(abs(v[0] * out[1] - v[1] * out[0]), 1e-12 * norm_in * norm_in)   # same direction
                self.assertGreaterEqual(v[0] * out[0] + v[1] * out[1], 0.)
                if out != list(v):
                    clamped += 1
                    # Not conservative beyond need: the binding axis stops exactly at its wall.
                    self.assertGreater(overshoot, -1e-6, label)
                    self.assertTrue(any(a['clamped'] for a in axes))
                else:
                    unclamped += 1
                    self.assertIs(out[0], v[0])
                    self.assertIs(out[1], v[1])
                    self.assertLessEqual(braking_rollout(p, v, tau, A_BRAKE, self.BOX), 1e-9)
        self.assertEqual(clamped + unclamped, 2 * 4 * 2 * 8)
        self.assertGreater(clamped, 0)
        self.assertGreater(unclamped, 0)

    def test_negative_control_the_old_per_axis_clamp_crosses_the_corner(self):
        fence = P.Geofence.from_config(G.fence(latency=.2))
        p, v = (3.5, 3.5), (2.5, 2.5)
        old = legacy_per_axis_limit(fence, v, p)
        self.assertAlmostEqual(old[0], 2.4465, places=4)            # the review's published (2.4465, 2.4465)
        crossed = braking_rollout(p, old, .2, A_BRAKE, self.BOX)
        self.assertAlmostEqual(crossed, .419, delta=.002)           # the review's 0.419 m
        new, _ = fence.limit(list(v), p)
        self.assertLessEqual(braking_rollout(p, new, .2, A_BRAKE, self.BOX), 1e-9)
        # The old clamp is exact on a single axis (control): the defect is the diagonal only.
        old_single = legacy_per_axis_limit(fence, (2.5, 0.), (3.5, 0.))
        self.assertLessEqual(braking_rollout((3.5, 0.), old_single, .2, A_BRAKE, self.BOX), 1e-9)
        self.assertAlmostEqual(fence.limit([2.5, 0.], (3.5, 0.))[0][0], old_single[0], places=12)

    def test_gateway_publishes_the_scaled_diagonal_and_the_box_centre_bit_exactly(self):
        gate, _ = G.activate(G.fence(latency=.2))
        out = G.enu_velocity(G.send(gate, 1, (3.5, 3.5), (2.5, 2.5), yaw=.4))
        self.assertAlmostEqual(out[0], out[1], places=15)
        self.assertLess(out[0], 2.4465)
        self.assertLessEqual(braking_rollout((3.5, 3.5), out, .2, A_BRAKE, self.BOX), 1e-9)
        self.assertTrue(gate.geofence_record['geofence_clamped'])
        self.assertAlmostEqual(gate.geofence_record['scale'], out[0] / 2.5, places=12)
        # Centre of a 20 m box: bit-identical to the disabled twin (control).
        fenced, _ = G.activate(G.fence(box=(-10., 10., -10., 10.)))
        plain, _ = G.activate(P.geofence_template())
        for k, v in enumerate(((2.5, 2.5), (-2.5, 1.), (.3, -.7)), 1):
            self.assertEqual(G.bits(G.send(fenced, k, (0., 0.), v)), G.bits(G.send(plain, k, (0., 0.), v)))
            self.assertFalse(fenced.geofence_record['geofence_clamped'])
            self.assertEqual(fenced.geofence_record['scale'], 1.)
        pure = P.Geofence.from_config(G.fence(box=(-10., 10., -10., 10.)))
        v = [2.5, -1.7320508075688772]
        out, _ = pure.limit(v, (0., 0.))
        self.assertIs(out[0], v[0])
        self.assertIs(out[1], v[1])

    def test_axis_scale_matches_the_single_axis_stop_speed(self):
        for d in (0., .4, 1.5, 3.):
            for tau in (.2, .4):
                with self.subTest(d=d, tau=tau):
                    self.assertAlmostEqual(P.axis_scale(2.5, 2.5, d, A_BRAKE, tau) * 2.5,
                                           P.stop_speed(d, A_BRAKE, tau), places=12)
        self.assertEqual(P.axis_scale(1., 2., 0., A_BRAKE, .2), 0.)


# ---------------------------------------------------------------------------------------------- F3 coverage
class F3CoverageNeverErasesValidDepth(unittest.TestCase):
    def pole(self):
        f, kind = F.render('trained_equal', 1., wall_x=10., pole=(1.0, .45, .05))
        return f, np.flatnonzero((kind == 2).any(axis=0)), kind

    def test_coverage_false_over_a_valid_pole_refuses_the_frame(self):
        f, cols, kind = self.pole()
        cov = np.ones((90, 160), bool)
        cov[:, :26] = False
        self.assertGreater(int(((kind == 2) & ~cov).sum()), 0)       # the pole's valid returns lie under it
        state = F.scene_state(1.)
        for ground in (False, True):
            with self.subTest(ground=ground), self.assertRaisesRegex(R.InputError, 'calibration does not match'):
                column_minima(dict(f, coverage=cov), state, F.contract(ground=ground))
        with self.assertRaisesRegex(R.InputError, 'coverage is False'):
            R.project_rgbd(dict(f, coverage=cov), state, F.contract())
        # Control: the correct coverage mask (this undistorted source covers every pixel) reads the pole.
        for frame in (dict(f, coverage=np.ones((90, 160), bool)), f):
            colmin, counts, _ = column_minima(frame, state, F.contract(ground=True))
            self.assertAlmostEqual(float(colmin[cols].min()), .956, delta=.005)
        # Control: coverage False where there is no valid return (an undistortion border at depth 0) is no-return.
        g = dict(f, coverage=cov, depth_z_m=np.where(cov, f['depth_z_m'], 0.).astype(np.float32),
                 invalid=f['invalid'] | ~cov)
        colmin, counts, _ = column_minima(g, state, F.contract(ground=True))
        self.assertEqual(counts['no_return_pixels'] - counts['ground_removed_pixels'], int((~cov).sum()))
        self.assertEqual(counts['near_pixels'], 0)

    def test_ros_camera_info_with_distortion_over_an_already_undistorted_image_is_refused(self):
        # The review's mismatch: the driver delivers an undistorted image with valid depth everywhere, but its
        # CameraInfo still carries D, so the computed border covers measured returns.
        info = G.distorted_info(3.)
        with tempfile.TemporaryDirectory() as tmp:
            frame, state = G.ros_frame(tmp, info=info)
        self.assertGreater(int((~frame['coverage']).sum()), 500)
        with self.assertRaisesRegex(R.InputError, 'calibration does not match'):
            G.preprocess(frame, state)
        # Control: the same CameraInfo with the border delivered as depth 0 (what a rectifying driver writes).
        rgb, z = G.wall_scene()
        z = np.where(frame['coverage'], z, 0.)
        with tempfile.TemporaryDirectory() as tmp:
            frame, state = G.ros_frame(tmp, info=info, z_m=z, rgb=rgb)
        _, counts, _ = G.preprocess(frame, state)
        self.assertEqual(counts['near_pixels'], 0)
        self.assertEqual(counts['no_return_pixels'], int((~frame['coverage']).sum()))


# ------------------------------------------------------------------------------------------- F4 share rule
class F4ZbufferShareRule(unittest.TestCase):
    def test_speckled_near_slab_reads_near_on_the_tilted_path(self):
        cols = trained_cols(.3, .3)[3:-3]
        for pitch in (.5, 3.):
            for share in (.05, .10, .30):
                f = speckled_slab(pitch, share)
                state = F.scene_state(1., pitch_deg=pitch)
                with self.subTest(pitch=pitch, share=share):
                    for ground in (False, True):
                        colmin, counts, projection = column_minima(f, state, F.contract(ground=ground))
                        self.assertEqual(projection['projection'], 'measured-point-z-buffer')
                        self.assertGreater(counts['near_pixels'], 0)
                        self.assertTrue(np.all((colmin[cols] >= .40) & (colmin[cols] <= .48)),
                                        (colmin[cols].min(), colmin[cols].max()))
                    # Negative control: the r1/r2 'any valid point wins' rule (share 1.0 makes a pixel invalid only
                    # when no valid sample lands) reads the 6 m speckle.
                    with mock.patch.object(R, 'ZBUFFER_INVALID_SHARE', 1.0):
                        before, counts, _ = column_minima(f, state, F.contract())
                    self.assertEqual(counts['near_pixels'], 0)
                    self.assertGreater(before[cols].min(), 5.9)
                    if pitch == .5 and share == .05:
                        # The same with the real pre-r1 (fc8aa4d) projection, re-implemented by the r1 tests.
                        legacy, near = legacy_column_minima(f, state)
                        self.assertEqual(near, 0)
                        self.assertGreater(legacy[cols].min(), 5.9)
        # Level control: the inverse-nearest path already read the slab near (unchanged).
        f = speckled_slab(0., .05)
        colmin, counts, projection = column_minima(f, F.scene_state(1.), F.contract())
        self.assertEqual(projection['projection'], 'inverse-nearest')
        self.assertTrue(np.all(colmin[cols] <= .48))

    def test_iid_two_percent_holes_stay_valid_on_the_tilted_path(self):
        for source in ('oakd_like', 'trained_equal'):
            for pitch in (3., .5):
                f, _ = F.render(source, 1., pitch_deg=pitch, wall_x=10., holes=.02, seed=7)
                colmin, counts, projection = column_minima(f, F.scene_state(1., pitch_deg=pitch), F.contract(ground=True))
                with self.subTest(source=source, pitch=pitch):
                    self.assertEqual(projection['projection'], 'measured-point-z-buffer')
                    self.assertGreater(int(f['invalid'].sum()), 0)       # the holes are in the source
                    self.assertEqual(counts['near_pixels'], 0)
                    self.assertEqual(int((colmin < 1.).sum()), 0)

    def test_share_threshold_is_a_majority_and_a_tie_is_sensor_invalid(self):
        # A source at exactly twice the trained resolution: each trained pixel receives one 2x2 block of samples.
        K = {'width': 320, 'height': 180, 'fx': 2 * F.TRAINED['fx'], 'fy': 2 * F.TRAINED['fy'], 'cx': 159.5, 'cy': 89.5}
        z = np.full((180, 320), 2., np.float32)
        invalid = np.zeros((180, 320), bool)
        offsets = ((0, 0), (0, 1), (1, 0), (1, 1))
        groups = {k: (20 + 10 * k, 30 + 10 * k) for k in range(5)}     # k of 4 samples invalid
        for k, (c0, c1) in groups.items():
            for dy, dx in offsets[:k]:
                invalid[60 + dy:120:2, 2 * c0 + dx:2 * c1:2] = True
        f = {'rgb': np.zeros((180, 320, 3), np.uint8), 'depth_z_m': np.where(invalid, 0., z).astype(np.float32),
             'invalid': invalid, 'intrinsics': K, 'aligned': True, 'rectified': True, 'mount_flu_m': list(F.MOUNT),
             'camera_to_body_rotation': np.eye(3)}
        state = F.scene_state(1., pitch_deg=1e-4)                       # tilted by 1e-4 deg: z-buffer path
        _, _, si, nr, _, report = R.project_rgbd(f, state, F.contract())
        self.assertEqual(report['projection'], 'measured-point-z-buffer')
        _, _, legacy_bad, _, _, _ = F.legacy_project_rgbd(f, state, F.contract())
        for k, (c0, c1) in groups.items():
            block_si, block_nr = si[0, 30:60, c0:c1], nr[0, 30:60, c0:c1]
            with self.subTest(invalid_of_4=k):
                self.assertFalse(block_nr.any())
                self.assertEqual(bool(block_si.all()), k >= 2)
                self.assertEqual(bool(block_si.any()), k >= 2)
                # Negative control: the pre-r1 rule made every block with one valid sample valid.
                self.assertEqual(bool(legacy_bad[0, 30:60, c0:c1].all()), k == 4)
        self.assertEqual(R.ZBUFFER_INVALID_SHARE, .5)


# ------------------------------------------------------------------------------- F6 estimator height reference
class F6EstimatorHeightAssertion(unittest.TestCase):
    def test_enabled_filter_without_the_assertion_refuses_every_frame(self):
        f, _ = F.render('trained_equal', 1., wall_x=10., floor='textured')
        state = F.scene_state(1.)
        for ground, ceiling in ((True, None), (False, 3.), (True, 3.)):
            unasserted = F.contract(ground=ground, ceiling=ceiling, asserted=False)
            with self.subTest(ground=ground, ceiling=ceiling):
                with self.assertRaisesRegex(R.InputError, 'height above the floor'):
                    R.height_band(state, unasserted)
                with self.assertRaisesRegex(R.InputError, 'estimator_height'):
                    column_minima(f, state, unasserted)
                # Positive control: the same filters with the assertion run.
                colmin, counts, _ = column_minima(f, state, F.contract(ground=ground, ceiling=ceiling))
                self.assertTrue(R.height_band(state, F.contract(ground=ground, ceiling=ceiling))['in_band'])
        # Control: with both filters off no assertion is needed, and the frame runs.
        off = F.contract(asserted=False)
        self.assertTrue(R.height_band(state, off)['in_band'])
        colmin, counts, _ = column_minima(f, state, off)
        self.assertEqual(counts['ground_removed_pixels'], 0)
        # A malformed assertion is not an assertion.
        for bad in ({'z_is_height_above_floor': False, 'evidence': F.FIXTURE_ASSERTION['evidence']},
                    {'z_is_height_above_floor': True, 'evidence': None}, 'yes'):
            c = F.contract(ground=True)
            c['estimator_height'] = bad
            with self.subTest(bad=bad), self.assertRaises(R.InputError):
                R.height_band(state, c)

    def test_ros_session_faults_without_the_assertion_and_runs_with_it(self):
        for asserted in (False, True):
            runtime = G.BandRuntime()
            runtime.contract = F.contract(ground=True, asserted=asserted)
            with tempfile.TemporaryDirectory() as tmp:
                session = RosPolicySession(runtime, F.ros_binding(Path(tmp) / 'fixture.txt'))
                runtime.events.clear()
                msgs = F.image_messages(3., np.zeros((90, 160, 3), np.uint8), np.full((90, 160), 5000, np.uint16),
                                        F.TRAINED)
                msgs['odometry'] = F.odometry_message(3., [0., 0., 1.], 0., [0., 0., 0.], 0.)
                for kind, message in msgs.items():
                    session.offer(kind, message, received_monotonic_s=100.)
                item = session.prepare(now_ros_s=3.02, now_monotonic_s=100.02)
                with self.subTest(asserted=asserted):
                    if asserted:
                        proposal = session.finish(item, session.infer(item), now_ros_s=3.02, now_monotonic_s=100.02)
                        self.assertIsNotNone(proposal)
                        self.assertEqual(runtime.events, ['step'])
                    else:
                        with self.assertRaisesRegex(R.InputError, 'height above the floor'):
                            session.infer(item)
                        session.fail_inference('estimator height not asserted')
                        self.assertEqual(session.status, 'FAULT')
                        self.assertEqual(runtime.events, [])

    def test_contract_carries_a_hash_bound_assertion_and_never_invents_one(self):
        value = C._metadata(1, ROOT)
        value['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        self.assertIsNone(value['estimator_height'])                   # no invented assertion
        self.assertEqual(C.validate_contract(value, verify_files=False), [])
        self.assertIn('estimator_height', C.height_reference_problem(value))
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp) / 'landed_z.txt'
            evidence.write_text('CPU fixture, not a measurement')
            asserted = C.estimator_height_assertion(evidence)
            self.assertEqual(asserted['evidence']['sha256'], hashlib.sha256(evidence.read_bytes()).hexdigest())
            good = dict(copy.deepcopy(value), estimator_height=asserted)
            self.assertEqual(C.validate_contract(good, verify_files=False), [])
            self.assertIsNone(C.height_reference_problem(good))
            for bad in (dict(asserted, z_is_height_above_floor=1), dict(asserted, extra=1), {'evidence': asserted['evidence']},
                        dict(asserted, evidence=dict(asserted['evidence'], path='relative.txt')),
                        dict(asserted, evidence=dict(asserted['evidence'], sha256='A' * 64)), True):
                with self.subTest(bad=bad):
                    self.assertTrue(C.validate_contract(dict(copy.deepcopy(value), estimator_height=bad),
                                                        verify_files=False))
            missing = copy.deepcopy(value)
            del missing['estimator_height']
            self.assertTrue(C.validate_contract(missing, verify_files=False))
            # Filters off: no assertion needed (control).
            off = copy.deepcopy(value)
            off['ground']['enabled'] = False
            self.assertIsNone(C.height_reference_problem(off))
            # CLI: --height-reference-evidence binds the file; without it the contract stays unasserted.
            with mock.patch.object(C, 'build_contract', return_value=good) as build, \
                    contextlib.redirect_stdout(io.StringIO()) as stdout:
                C.main(['--output', str(Path(tmp) / 'c.json'), '--seed', '1', '--height-reference-evidence', str(evidence)])
            self.assertEqual(build.call_args.kwargs['estimator_height'], asserted)
            self.assertEqual(json.loads(stdout.getvalue())['height_filters'], 'ASSERTED')
            with mock.patch.object(C, 'build_contract', return_value=value) as build, \
                    contextlib.redirect_stdout(io.StringIO()) as stdout:
                C.main(['--output', str(Path(tmp) / 'd.json'), '--seed', '1'])
            self.assertIsNone(build.call_args.kwargs['estimator_height'])
            self.assertEqual(json.loads(stdout.getvalue())['height_filters'], 'REFUSES_EVERY_FRAME_UNTIL_ASSERTED')

    def test_evidence_bytes_are_verified_and_the_ros_node_refuses_an_unasserted_contract(self):
        if not CHECKPOINT_DIR.is_dir():
            self.skipTest('local trained camera checkpoint not distributed in the public repository')
        import camera_ros2_node as node
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            evidence = tmp / 'landed_z.txt'
            evidence.write_text('CPU fixture, not a measurement')
            asserted = C.build_contract(1, estimator_height=C.estimator_height_assertion(evidence))
            self.assertEqual(C.validate_contract(asserted), [])
            unasserted = C.build_contract(1)
            binding = F.ros_binding(tmp / 'binding_fixture.txt')
            binding_config = {'format': 'motar-ros2-node-v1', 'binding': binding,
                              'outputs': {'velocity_topic': '/motar/policy/velocity_enu',
                                          'diagnostic_topic': '/motar/policy/diagnostics'}}
            results = {}
            for name, contract in (('unasserted', unasserted), ('asserted', asserted)):
                path = tmp / ('%s.json' % name)
                path.write_text(json.dumps(contract))
                config = dict(binding_config, policy_contract_path=str(path),
                              policy_contract_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                config_path = tmp / ('%s_node.json' % name)
                config_path.write_text(json.dumps(config))
                try:
                    node.load_node_config(config_path, ROOT)
                    results[name] = 'LOADED'
                except ValueError as exc:
                    results[name] = str(exc)
            self.assertIn('UNCONFIGURED', results['unasserted'])
            self.assertIn('height above the floor', results['unasserted'])
            self.assertEqual(results['asserted'], 'LOADED')                # positive control
            evidence.write_text('changed after binding')
            self.assertIn('estimator_height evidence SHA256 changed', C.validate_contract(asserted))


# ------------------------------------------------------------------------------------------- F7 records
class F7PackageRecords(unittest.TestCase):
    def test_ros_readme_documents_the_deployment_layer(self):
        text = (ROOT / 'ros2/motar_camera_policy/README.md').read_text()
        for term in ('geofence', '~/reset_geofence', 'WAITING_ALTITUDE', 'geofence_waiver', 'invalid_plane_rule',
                     'estimator_height', '--height-reference-evidence', 'REP 117', 'coverage', 'python3-opencv',
                     'camera_policy_contract.py --output', 'policy_contract_sha256', 'latency_s',
                     '2 x (vehicle response delay + 0.1 s)'):
            with self.subTest(term=term):
                self.assertIn(term, text)

    def test_sitl_camera_record_has_a_dated_section_and_keeps_its_history(self):
        text = (ROOT / 'docs/evidence/sitl_2026-11-05/camera_runtime/README.md').read_text()
        self.assertIn('2026-11-13', text)
        self.assertIn('not comparable', text)
        out = subprocess.run(['git', '-C', str(ROOT), 'show', 'fc8aa4d:docs/evidence/sitl_2026-11-05/camera_runtime/README.md'],
                             capture_output=True, text=True)
        if out.returncode != 0:
            self.skipTest('fc8aa4d not available in this checkout')
        self.assertTrue(text.startswith(out.stdout.rstrip('\n')), 'earlier text must stay unchanged')


# ------------------------------------------------------------------------------------- F8 publication fence
class F8PublicationNeedsFenceOrWaiver(unittest.TestCase):
    def config(self, *, fence_block=None, waiver=None, publish=True):
        value = N.unconfigured_template()
        value.update(status='CONFIGURED', enable_offboard_publication=publish, frames=dict(world='map', body='base_link'),
                     geofence=fence_block or P.geofence_template(), geofence_waiver=waiver)
        return value

    def prime(self, config):
        session = N.BridgeSession(config, G.binding())
        sample = G.snap(100., 20., (0., 0.))
        for kind in ('odometry', 'local_position', 'status'):
            session.offer(kind, sample[kind], receipt=20.)
        session.poll(now=100., wall=20.)
        return session, session.prime(now=100., wall=20.)

    def test_bridge_refuses_to_start_publication_without_a_fence_or_waiver(self):
        self.assertIsNone(N.unconfigured_template()['geofence_waiver'])
        with self.assertRaisesRegex(ValueError, 'enabled, measured geofence or an explicit geofence_waiver'):
            self.prime(self.config())
        for waiver in ({'reason': 'Operator decision 2026-11-13: tethered bench test, no walls within 20 m'},):
            session, payload = self.prime(self.config(waiver=waiver))
            self.assertIsNotNone(payload)                                   # waived: publication starts
            self.assertEqual(session.diagnostics()['geofence_waiver'], waiver)
        session, payload = self.prime(self.config(fence_block=G.fence()))
        self.assertIsNotNone(payload)                                       # enabled fence: starts
        self.assertTrue(session.gate.geofence.enabled)
        # Malformed waivers are refused at construction.
        for bad in ({'reason': ''}, {'reason': '   '}, {'reason': 3}, {'why': 'x'}, 'waived'):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                N.BridgeSession(self.config(waiver=bad), G.binding())
        with self.assertRaisesRegex(ValueError, 'conflicts with an enabled geofence'):
            N.BridgeSession(self.config(fence_block=G.fence(), waiver={'reason': 'x'}), G.binding())
        with self.assertRaisesRegex(ValueError, 'observation-only'):
            N.BridgeSession(self.config(publish=False, waiver={'reason': 'x'}), G.binding())
        # Observation only needs neither (control).
        session = N.BridgeSession(self.config(publish=False), G.binding())
        self.assertIsNone(session.fault)

    def test_node_configuration_refuses_publication_without_a_fence_or_waiver(self):
        if not PX4_MSG_SOURCE.is_dir():
            self.skipTest('local PX4 message source unavailable')
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            for name in P.PX4_MESSAGE_SHA256:
                (tmp / name).write_bytes((PX4_MSG_SOURCE / name).read_bytes())
            proof = tmp / 'flight_checks.txt'
            proof.write_text('CPU fixture flight-check record, not a qualification')
            results = {}
            for name, fence_block, waiver in (('neither', None, None), ('fence', G.fence(), None),
                                              ('waiver', None, {'reason': 'Operator decision: tethered test'})):
                value = self.config(fence_block=fence_block, waiver=waiver)
                value['px4_msgs_msg_dir'] = str(tmp)
                value['firmware']['source'] = 'CPU fixture source schema only, clock/hardware NOT_TESTED'
                value['flight_checks_provenance'] = dict(path=str(proof), sha256=hashlib.sha256(proof.read_bytes()).hexdigest())
                path = tmp / ('%s.json' % name)
                path.write_text(json.dumps(value))
                stdout = io.StringIO()
                with contextlib.redirect_stdout(stdout):
                    code = N.main(config_path=path, source_root=ROOT, check_only=True)
                results[name] = (code, json.loads(stdout.getvalue()))
            self.assertEqual(results['neither'][0], 2)
            self.assertEqual(results['neither'][1]['status'], 'NOT_CONFIGURED')
            self.assertIn('geofence_waiver', results['neither'][1]['reason'])
            self.assertEqual(results['fence'][0], 0)
            self.assertEqual(results['fence'][1]['geofence'], 'ENABLED')
            self.assertEqual(results['waiver'][0], 0)
            self.assertEqual(results['waiver'][1]['geofence'], 'DISABLED')
            self.assertEqual(results['waiver'][1]['geofence_waiver']['reason'], 'Operator decision: tethered test')


# ------------------------------------------------------------------------------------------- F10 SITL +inf
class F10GazeboPlusInfIsNoReturn(unittest.TestCase):
    PATCH = (slice(10, 40), slice(60, 100))

    def packet(self, value):
        z = np.full((90, 160), 9.9, np.float32)
        z[self.PATCH] = value
        return {'rgb': np.zeros((90, 160, 3), np.uint8), 'depth': z, 'rgb_t': 1., 'depth_t': 1.}

    def test_plus_inf_is_no_return_and_nan_minus_inf_zero_stay_sensor_invalid(self):
        frame = R.gazebo_frame(self.packet(np.inf))
        self.assertFalse(frame['invalid'][self.PATCH].any())
        _, _, si, nr, _, _ = R.project_rgbd(frame, F.scene_state(1.), F.contract())
        self.assertEqual(int(si.sum()), 0)
        self.assertGreater(int(nr.sum()), 0)
        colmin, counts, _ = column_minima(frame, F.scene_state(1.), F.contract())
        self.assertEqual(counts['near_pixels'], 0)
        self.assertEqual(int((colmin < 1.).sum()), 0)
        for value in (np.nan, -np.inf, 0., -1.):
            frame = R.gazebo_frame(self.packet(value))
            with self.subTest(value=value):
                self.assertTrue(frame['invalid'][self.PATCH].all())
                colmin, counts, _ = column_minima(frame, F.scene_state(1.), F.contract())
                self.assertGreater(counts['near_pixels'], 0)
        # Negative control: the pre-r1 rule (~isfinite | z <= 0) flagged the +inf patch, which then read near.
        frame = R.gazebo_frame(self.packet(np.inf))
        frame['invalid'] = ~np.isfinite(frame['depth_z_m']) | (frame['depth_z_m'] <= 0)
        _, counts, _ = column_minima(frame, F.scene_state(1.), F.contract())
        self.assertGreater(counts['near_pixels'], 0)


# ------------------------------------------------------------------------------------------------- P3
class P3ContractSourcesAndGeofenceBounds(unittest.TestCase):
    def test_contract_binds_the_ros_adapter_and_session(self):
        for rel in ('tools/camera_ros2_adapter.py', 'tools/camera_ros2_session.py'):
            self.assertIn(rel, C.SOURCE_PATHS)
            self.assertTrue((ROOT / rel).is_file())
        value = C._metadata(1, ROOT)
        value['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS if rel != 'tools/camera_ros2_session.py'}
        self.assertIn('runtime source set differs from the supported contract', C.validate_contract(value, verify_files=False))

    def test_geofence_values_have_sane_upper_bounds(self):
        for change in (dict(decel_mps2=10.), dict(latency_s=1.), dict(decel_mps2=.5, latency_s=.05)):
            self.assertTrue(P.Geofence.from_config(G.fence(**change)).enabled, change)          # positive controls
        for change in (dict(decel_mps2=10.01), dict(decel_mps2=0.), dict(latency_s=1.01), dict(latency_s=5.),
                       dict(latency_s=-.1)):
            with self.subTest(change=change), self.assertRaises(P.GatewayError):
                P.Geofence.from_config(G.fence(**change))

    def test_disabled_fence_carrying_values_is_refused(self):
        self.assertFalse(P.Geofence.from_config(P.geofence_template()).enabled)            # positive control
        self.assertFalse(P.Geofence.from_config(dict(P.geofence_template(), decel_mps2=None)).enabled)
        for key, value in (('x_min_m', -5.), ('y_max_m', 5.), ('latency_s', .3)):
            block = dict(P.geofence_template(), **{key: value})
            with self.subTest(key=key), self.assertRaisesRegex(P.GatewayError, 'disabled geofence must leave'):
                P.Geofence.from_config(block)
        for decel in (0., 11., 'fast', True):
            with self.subTest(decel=decel), self.assertRaises(P.GatewayError):
                P.Geofence.from_config(dict(P.geofence_template(), decel_mps2=decel))


# ------------------------------------------------------------------- actual policy (seed 1), one fresh process
GOLDEN_SCENES = {
    # The review's frames.npz scenes (review_r1/probes/gen_frames.py), re-rendered here byte-identically.
    'level_wall10_trained': ('trained_equal', 0., dict(wall_x=10.)),
    'level_wall10_oak': ('oakd_like', 0., dict(wall_x=10.)),
    'level_pole3_target_trained': ('trained_equal', 0., dict(wall_x=10., pole=(3., 0., .1), sphere=(5., 1., 1.03, .15))),
    'level_holes2pct_trained': ('trained_equal', 0., dict(wall_x=10., holes=.02, seed=7)),
    'level_holes2pct_target_oak': ('oakd_like', 0., dict(wall_x=10., holes=.02, seed=7, sphere=(5., 0., 1.03, .15))),
    'pitch0.5_wall10_oak': ('oakd_like', .5, dict(wall_x=10.)),
    'pitch0.5_pole3_target_oak': ('oakd_like', .5, dict(wall_x=10., pole=(3., 0., .1), sphere=(5., 1., 1.03, .15))),
    'level_sphere_full_trained': ('trained_equal', 0., dict(wall_x=10., sphere=(5., 0., 1.03, .15))),
    'level_sphere1p5_full_trained': ('trained_equal', 0., dict(wall_x=10., sphere=(1.5, 0., 1.03, .15))),
}
# SHA-256 of the float32 898-D observation produced by the REAL fc8aa4d runtime with the seed-1 checkpoint
# (build_contract(1) of fc8aa4d): sha256(bytes.fromhex(obs_hex)) of ~/motar_tmp/ros2_runtime_fix_20261112/review_r1/
# probes/rt_base.json, extracted to ../repair_r1/golden_fc8aa4d_obs_sha256.json. Pinned so that the bit identity is
# checked against the actual pre-fix code, not a re-implementation (review F9). The wall-only scenes share one
# observation (the wall at 10 m lies beyond the 10 m obstacle range).
GOLDEN_FC8AA4D_OBS_SHA256 = {
    'level_wall10_trained': '53fc678b13c050681af84a16302679cb8edbfe77da6d80a032ef2c9317b63eb1',
    'level_wall10_oak': '53fc678b13c050681af84a16302679cb8edbfe77da6d80a032ef2c9317b63eb1',
    'level_pole3_target_trained': '1ee4f6e9db80a8a0386d9d6920ed43ddc58b6cc5aae645c765f97b496f02c463',
    'level_holes2pct_trained': '53fc678b13c050681af84a16302679cb8edbfe77da6d80a032ef2c9317b63eb1',
    'level_holes2pct_target_oak': '65e4e4ea70ed764adeb898c42a0b7b1de157b94409b94dbdb3b7af3201ecb0cd',
    'pitch0.5_wall10_oak': '53fc678b13c050681af84a16302679cb8edbfe77da6d80a032ef2c9317b63eb1',
    'pitch0.5_pole3_target_oak': 'afd3ee6bde8c33fbcc4fec9f76419451bef0767bdeeb7730c3c290f381ea238a',
    'level_sphere_full_trained': '973c50fc3b8c7a40d3100517fa70d9b21934bb4b4604b1060377e6760dbe4c7d',
    'level_sphere1p5_full_trained': '964dc83cf572170a9aea3900fbca292200b29a08858ec2e43a420394599c4b31',
}

ACTUAL = r'''
import hashlib, json, math, sys, tempfile
from pathlib import Path
import numpy as np
import test_camera_runtime_fixes as F
import test_camera_runtime_repair_r1 as RR
from camera_policy_contract import build_contract, estimator_height_assertion, GROUND_DEFAULT
from camera_policy_runtime import CameraPolicyRuntime
import camera_policy_runtime as R

_evidence_dir = tempfile.TemporaryDirectory()
_evidence = Path(_evidence_dir.name) / 'estimator_height_fixture.txt'
_evidence.write_text('CPU test fixture: synthetic scenes put estimator z at the height above the floor. Not evidence.')
ASSERTED = estimator_height_assertion(_evidence)
contracts = {'off': build_contract(1, ground=dict(GROUND_DEFAULT, enabled=False)),
             'on': build_contract(1, estimator_height=ASSERTED),                 # the default contract, asserted
             'on+plane': build_contract(1, ground=dict(GROUND_DEFAULT, invalid_plane_rule=True), estimator_height=ASSERTED)}
rt = CameraPolicyRuntime(contracts['on'])
torch = rt.torch
production_goal = rt.goal_coloured
fixed_preprocess = rt.preprocess
out = {'pixel_threshold': rt.perception.pixel_threshold, 'max_camera_range': rt.perception.max_camera_range,
       'segmenter': type(rt.perception.segmenter).__name__,
       'segmenter_depth_weight': float(rt.perception.segmenter.classifier.weight[0, 3].abs().max())}

def no_goal(rgb, ranges):
    return np.zeros(ranges.shape, bool)

def configure(variant, pre_f5):
    rt.contract = contracts[variant]
    rt.goal_coloured = no_goal if pre_f5 else production_goal

def step(frame, state, variant='on', pre_f5=False):
    configure(variant, pre_f5)
    seen = {}
    def capture(f, s):
        result = fixed_preprocess(f, s)
        seen['depth'] = result[1]
        return result
    rt.preprocess = capture
    try:
        rt.reset()
        res = rt.step(dict(frame, rgb_t=1., depth_t=1.), dict(state, t=1.), now_s=1.)
    finally:
        rt.preprocess, rt.goal_coloured = fixed_preprocess, production_goal
    obs = np.asarray(res['observation'], np.float32)
    tok = obs[288:288 + 480].reshape(5, 8, 12)[-1]
    tokens = [(round(float(12. * (1. - t[7])), 3), round(math.degrees(math.atan2(t[1], t[0])), 2))
              for t in tok[tok[:, 11] > .5]]
    cols = seen['depth'][0].amin(dim=0).numpy()
    return dict(sha=hashlib.sha256(obs.tobytes()).hexdigest(), tokens=tokens, near=res['near_pixels'],
                si=res['sensor_invalid_pixels'], goal_invalid=res['goal_invalid_pixels'],
                cols=[round(float(c), 4) for c in cols], projection=res['projection']['projection'],
                velocity=res['velocity_vehicle_flu_mps'])

def detect(frame, state, variant='on', pre_f5=False):
    configure(variant, pre_f5)
    try:
        rgb, depth, counts, _ = rt.preprocess(frame, state)
        rt.reset()
        with torch.inference_mode():
            _, surface, bearing, visible, _, mask = rt.perception._detect_rgbd(rgb, depth, False)
        rt.reset()
    finally:
        rt.goal_coloured = production_goal
    return dict(surface=float(surface[0]), visible=bool(visible[0]), mask=int(mask.sum()), near=counts['near_pixels'],
                goal_invalid=counts['goal_invalid_pixels'], si=counts['sensor_invalid_pixels'])

S, st = F.render, F.scene_state
# Golden scenes: the new runtime against the real fc8aa4d observation (filters off, and the default floor filter on).
for name, (source, pitch, kw) in RR.GOLDEN_SCENES.items():
    f, _ = S(source, 1., pitch_deg=pitch, **kw)
    for variant in ('off', 'on'):
        out['golden %s %s' % (name, variant)] = step(f, st(1., pitch_deg=pitch), variant)['sha']
# F5: half-invalid goal sphere (the invalid half is the red target), both projection paths.
for source, pitch, x in (('trained_equal', 0., 5.), ('oakd_like', 0., 5.), ('oakd_like', .5, 5.),
                         ('trained_equal', 0., 1.5), ('oakd_like', .5, 1.5)):
    f, kind = S(source, 1., pitch_deg=pitch, wall_x=10., sphere=(x, 0., 1.03, .15), sphere_invalid=True)
    for pre in (False, True):
        key = 'goal %s pitch %g x %g %s' % (source, pitch, x, 'pre_f5' if pre else 'fixed')
        out[key] = dict(step(f, st(1., pitch_deg=pitch), pre_f5=pre), detect=detect(f, st(1., pitch_deg=pitch), pre_f5=pre))
    full, _ = S(source, 1., pitch_deg=pitch, wall_x=10., sphere=(x, 0., 1.03, .15))
    out['goal %s pitch %g x %g full' % (source, pitch, x)] = dict(step(full, st(1., pitch_deg=pitch)),
                                                                  detect=detect(full, st(1., pitch_deg=pitch)))
# F5 control: a non-goal-coloured dense invalid slab is still near.
for pitch in (0., .5):
    f, _ = RR.box_frame('oakd_like', 1., pitch, 3.0, half_y=.3)
    out['grey slab pitch %g' % pitch] = step(f, st(1., pitch_deg=pitch))
# F1 at policy level: box face 0.3 m ahead, top below camera height, textured floor.
for source, pitch in (('trained_equal', 0.), ('oakd_like', 1.), ('oakd_like', 3.)):
    for top in (1.00, .98, .90):
        f, _ = RR.box_frame(source, 1., pitch, top, floor=True)
        for variant in ('on', 'on+plane'):
            out['box %s pitch %g top %.2f %s' % (source, pitch, top, variant)] = step(f, st(1., pitch_deg=pitch), variant)
# F4 at policy level: speckled near slab on the tilted path.
for pitch in (.5, 3.):
    out['speckle pitch %g' % pitch] = step(RR.speckled_slab(pitch, .05), st(1., pitch_deg=pitch))
# F10 at policy level: a Gazebo +inf patch is free space; a NaN patch is near (control).
for label, value in (('inf', float('inf')), ('nan', float('nan'))):
    z = np.full((90, 160), 9.9, np.float32)
    z[10:40, 60:100] = value
    frame = R.gazebo_frame({'rgb': np.zeros((90, 160, 3), np.uint8), 'depth': z, 'rgb_t': 1., 'depth_t': 1.})
    out['gazebo %s' % label] = step(frame, st(1.), 'off')
print('REPAIR_R1_RESULT ' + json.dumps(out, allow_nan=False))
'''


class ActualPolicySeed1RepairR1(unittest.TestCase):
    """Real CAM-K2-v3 seed-1 checkpoint and the production perception segmenter, one fresh CPU process."""

    @classmethod
    def setUpClass(cls):
        if not CHECKPOINT_DIR.is_dir():
            raise unittest.SkipTest('local trained camera checkpoint not distributed in the public repository')
        env = {k: v for k, v in os.environ.items() if not k.startswith('NAVRL_')}
        env.update(CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
                   OPENBLAS_NUM_THREADS='1', PYTHONPATH=os.pathsep.join([str(ROOT / 'tools'), str(ROOT / 'tests')]))
        done = subprocess.run([sys.executable, '-B', '-c', ACTUAL], env=env, capture_output=True, text=True,
                              timeout=900, cwd=str(ROOT))
        if done.returncode != 0:
            raise AssertionError(done.stdout[-2000:] + done.stderr[-4000:])
        line = [x for x in done.stdout.splitlines() if x.startswith('REPAIR_R1_RESULT ')]
        cls.out = json.loads(line[-1][len('REPAIR_R1_RESULT '):])

    def near_tokens(self, row, below=1., bearing=10.):
        return [t for t in row['tokens'] if t[0] < below and abs(t[1]) <= bearing]

    def test_f5_production_colour_rule_is_the_perception_segmenter(self):
        self.assertEqual(self.out['pixel_threshold'], .55)
        self.assertEqual(self.out['max_camera_range'], 20.)
        self.assertEqual(self.out['segmenter'], 'AppearanceTargetSegmenter')
        self.assertEqual(self.out['segmenter_depth_weight'], 0.)

    def test_f5_half_invalid_goal_gives_an_unbiased_range_and_no_phantom_obstacle(self):
        for source, pitch, x in (('trained_equal', 0, 5), ('oakd_like', 0, 5), ('oakd_like', .5, 5),
                                 ('trained_equal', 0, 1.5), ('oakd_like', .5, 1.5)):
            fixed = self.out['goal %s pitch %g x %g fixed' % (source, pitch, x)]
            pre = self.out['goal %s pitch %g x %g pre_f5' % (source, pitch, x)]
            full = self.out['goal %s pitch %g x %g full' % (source, pitch, x)]
            label = (source, pitch, x)
            with self.subTest(label=label):
                self.assertGreater(fixed['goal_invalid'], 0, label)
                self.assertEqual(fixed['near'], 0, label)
                self.assertFalse(self.near_tokens(fixed), (label, fixed['tokens']))
                self.assertTrue(fixed['detect']['visible'])
                self.assertAlmostEqual(fixed['detect']['surface'], x - .25, delta=.1)
                # Negative control (pre-F5 = r2 behaviour): the invalid goal half is a near blob, a 0.4 m token.
                self.assertEqual(pre['goal_invalid'], 0)
                self.assertGreater(pre['near'], 0, label)
                self.assertTrue([t for t in pre['tokens'] if t[0] <= .45 and abs(t[1]) <= 10.], (label, pre['tokens']))
                # Control: the fully valid sphere reads the same surface.
                self.assertAlmostEqual(full['detect']['surface'], x - .25, delta=.1)
                self.assertEqual(full['goal_invalid'], 0)
        self.assertEqual(self.out['goal oakd_like pitch 0.5 x 5 fixed']['projection'], 'measured-point-z-buffer')
        self.assertEqual(self.out['goal trained_equal pitch 0 x 5 fixed']['projection'], 'inverse-nearest')

    def test_f5_control_a_grey_dense_invalid_slab_is_still_near(self):
        for pitch in (0, .5):
            row = self.out['grey slab pitch %g' % pitch]
            with self.subTest(pitch=pitch):
                self.assertEqual(row['goal_invalid'], 0)
                self.assertGreater(row['near'], 0)
                self.assertTrue(self.near_tokens(row, .5), row['tokens'])

    def test_f5_f4_untilted_and_all_valid_scenes_are_bit_identical_to_the_real_fc8aa4d_runtime(self):
        self.assertEqual(set(GOLDEN_FC8AA4D_OBS_SHA256), set(GOLDEN_SCENES))
        self.assertGreaterEqual(len(set(GOLDEN_FC8AA4D_OBS_SHA256.values())), 6)      # non-vacuous: distinct scenes
        for name in GOLDEN_SCENES:
            for variant in ('off', 'on'):
                with self.subTest(scene=name, variant=variant):
                    self.assertEqual(self.out['golden %s %s' % (name, variant)], GOLDEN_FC8AA4D_OBS_SHA256[name])

    def test_f1_box_below_camera_height_is_a_near_token_with_the_default_contract(self):
        for source, pitch in (('trained_equal', 0), ('oakd_like', 1), ('oakd_like', 3)):
            for top in (1.00, .98, .90):
                on = self.out['box %s pitch %g top %.2f on' % (source, pitch, top)]
                rule = self.out['box %s pitch %g top %.2f on+plane' % (source, pitch, top)]
                with self.subTest(source=source, pitch=pitch, top=top):
                    self.assertGreater(on['near'], 0)
                    self.assertTrue(self.near_tokens(on, .5), on['tokens'])
                    # Negative control: the opt-in plane rule erases it (the review's F1 input).
                    self.assertEqual(rule['near'], 0)
                    self.assertFalse(self.near_tokens(rule), rule['tokens'])

    def test_f4_speckled_slab_is_a_near_token_on_the_tilted_path(self):
        for pitch in (.5, 3.):
            row = self.out['speckle pitch %g' % pitch]
            with self.subTest(pitch=pitch):
                self.assertEqual(row['projection'], 'measured-point-z-buffer')
                self.assertGreater(row['near'], 0)
                self.assertTrue(self.near_tokens(row, .5), row['tokens'])

    def test_f10_gazebo_plus_inf_patch_is_free_space(self):
        inf, nan = self.out['gazebo inf'], self.out['gazebo nan']
        self.assertEqual(inf['near'], 0)
        self.assertFalse([t for t in inf['tokens'] if t[0] < 1.])
        self.assertGreater(nan['near'], 0)                  # control
        self.assertTrue([t for t in nan['tokens'] if t[0] < 1.])


if __name__ == '__main__':
    unittest.main()
