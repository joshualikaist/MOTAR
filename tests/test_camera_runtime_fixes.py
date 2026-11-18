"""Camera-runtime deployment fixes r1 (2026-11-12): measured-input semantics only, no retraining.

H1/H5a: uncovered (no-return) trained pixels are split from sensor-invalid ones; blob_near sees only the
latter. H2/B2: floor and ceiling returns are removed by height in the level camera. H3: sensor-invalid,
near and no-return pixels are black before the colour detector. H4: the ROS pose is interpolated to the
RGB capture time when two odometry samples bracket it.

Synthetic scenes are exact ray intersections (verification probes, ~/motar_tmp/ros2_runtime_verify_20261112/
synth_scene.py, extended with ceilings, side walls, textureless planes, holes and borders). The actual-policy
class runs the real CAM-K2-v3 seed-1 checkpoint through CameraPolicyRuntime.step in one fresh CPU process
and compares every fixed reading with the pre-fix pipeline, reproduced by legacy_project_rgbd below.
Nothing here is real-sensor, Gazebo or flight evidence.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import camera_policy_runtime as R
import camera_ros2_adapter as A
from camera_ros2_session import RosPolicySession

CHECKPOINT_DIR = Path('/home/fair/motar_tmp/camk2v3_20261108/train_s1/checkpoint')
MOUNT = [0.10, 0.0, 0.03]
TRAINED = {'width': 160, 'height': 90, 'fx': 80 / math.tan(math.radians(33)),
           'fy': 45 / math.tan(math.radians(27)), 'cx': 79.5, 'cy': 44.5}


# Repair r1 (F6): a geometry contract with an enabled filter carries the operator's estimator-height assertion.
# This one is a structural CPU fixture (never file-verified here), not evidence about any vehicle.
FIXTURE_ASSERTION = {'z_is_height_above_floor': True,
                     'evidence': {'path': '/CPU_FIXTURE_NOT_EVIDENCE/estimator_height.txt', 'sha256': '0' * 64}}


def contract(ground=False, ceiling=None, *, plane_rule=False, asserted=True):
    """Geometry subset of the CAM-K2-v3 contract; no checkpoint or production import.

    plane_rule enables the opt-in invalid-pixel plane rule (repair r1, F1) on each enabled filter."""
    return {'camera': dict(TRAINED, translation_m=list(MOUNT)),
            'depth': {'minz_m': .4, 'maxz_m': 20., 'window': 5, 'density': .5},
            'observation': {'control_dt_s': .1},
            'ground': {'enabled': ground, 'margin_m': .1, 'height_reference': 'floor',
                       'invalid_plane_rule': bool(ground and plane_rule)},
            'ceiling': {'enabled': ceiling is not None, 'height_m': ceiling, 'margin_m': .1,
                        'height_reference': 'floor', 'invalid_plane_rule': bool(ceiling is not None and plane_rule)},
            'estimator_height': dict(FIXTURE_ASSERTION) if asserted else None}


def source_intrinsics(name):
    if name == 'trained_equal':
        return dict(TRAINED)
    if name == 'oakd_like':  # 1014x760 delivered by oakd_camera.py; 69 deg HFOV ASSUMED (published IMX378 value)
        f = 1014 / (2 * math.tan(math.radians(34.5)))
        return {'width': 1014, 'height': 760, 'fx': f, 'fy': f, 'cx': 506.5, 'cy': 379.5}
    raise ValueError(name)


def body_rotation(roll_deg=0., pitch_deg=0., yaw_rad=0.):
    """FLU body -> ENU world; positive pitch about +y turns the nose down."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0], [math.sin(yaw_rad), math.cos(yaw_rad), 0], [0, 0, 1]])
    return rz @ rx @ ry


def quaternion_xyzw(m):
    w = math.sqrt(max(0., 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0., 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0., 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0., 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([x, y, z, w])
    return (q / np.linalg.norm(q)).tolist()


def scene_state(height, roll_deg=0., pitch_deg=0., t=1., x=0., yaw_rad=0.):
    return {'pos': [float(x), 0., float(height)], 'vel': [0., 0., 0.],
            'quat_xyzw': quaternion_xyzw(body_rotation(roll_deg, pitch_deg, yaw_rad)), 'yaw_rate_rps': 0., 't': t}


def render(source, height, roll_deg=0., pitch_deg=0., *, x=0., wall_x=None, pole=None, floor=None, ceiling=None,
           side_wall_y=None, sphere=None, sphere_invalid=False, near_slab=None, holes=0., seed=0, border=0,
           border_kind=None, far_top=None, far_top_invalid=False):
    """Body-fixed camera (camera FLU = body FLU) frame, as oakd_camera.py delivers it.

    floor/ceiling: 'textured' (valid depth) or 'textureless' (depth 0, invalid); ceiling=(height, kind).
    near_slab=(world x, half width): a surface closer than MinZ, reported invalid like the OAK-D does.
    border/border_kind: a band of that many source pixels with depth 0 and invalid True (as an
    undistortion remap writes it); 'coverage' also supplies coverage False there, 'invalid' does not.
    far_top: optical z written (valid unless far_top_invalid) on every source row above the principal row.
    """
    K = source_intrinsics(source)
    sw, sh = K['width'], K['height']
    uu, vv = np.meshgrid(np.arange(sw, dtype=np.float64), np.arange(sh, dtype=np.float64))
    d_cam = np.stack([np.ones_like(uu), -(uu - K['cx']) / K['fx'], -(vv - K['cy']) / K['fy']], axis=-1)
    d_cam /= np.linalg.norm(d_cam, axis=-1, keepdims=True)
    rot = body_rotation(roll_deg, pitch_deg)
    o = np.array([x, 0., height]) + rot @ np.array(MOUNT)
    d = d_cam @ rot.T
    t_best = np.full((sh, sw), np.inf)
    kind = np.zeros((sh, sw), np.int8)   # 1 wall, 2 pole, 3 floor, 4 sphere, 5 slab, 6 ceiling, 7 side wall
    invalid_kinds = []

    def take(t, code):
        nonlocal t_best, kind
        ok = np.isfinite(t) & (t > 1e-6) & (t < t_best)
        t_best = np.where(ok, t, t_best)
        kind = np.where(ok, code, kind)

    def plane(axis, value, sign):
        with np.errstate(divide='ignore', invalid='ignore'):
            return np.where(sign * d[..., axis] > 1e-9, (value - o[axis]) / d[..., axis], np.inf)

    if wall_x is not None:
        take(plane(0, wall_x, 1), 1)
    if side_wall_y is not None:
        take(plane(1, side_wall_y, 1 if side_wall_y > o[1] else -1), 7)
    if pole is not None:
        px, py, pr = pole
        ox, oy = o[0] - px, o[1] - py
        a = d[..., 0] ** 2 + d[..., 1] ** 2
        b = 2 * (ox * d[..., 0] + oy * d[..., 1])
        disc = b * b - 4 * a * (ox * ox + oy * oy - pr * pr)
        with np.errstate(invalid='ignore', divide='ignore'):
            take(np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf), 2)
    if near_slab is not None:
        sx, hw = near_slab
        t = plane(0, sx, 1)
        take(np.where(np.abs(o[1] + t * d[..., 1]) <= hw, t, np.inf), 5)
        invalid_kinds.append(5)
    if floor is not None:
        take(plane(2, 0., -1), 3)
        if floor == 'textureless':
            invalid_kinds.append(3)
    if ceiling is not None:
        take(plane(2, ceiling[0], 1), 6)
        if ceiling[1] == 'textureless':
            invalid_kinds.append(6)
    if sphere is not None:
        oc = o - np.array(sphere[:3])
        b = 2 * (d @ oc)
        disc = b * b - 4 * (oc @ oc - sphere[3] ** 2)
        with np.errstate(invalid='ignore'):
            take(np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / 2, np.inf), 4)
    hit = np.isfinite(t_best)
    z = np.where(hit, t_best * d_cam[..., 0], 0.)
    invalid = ~hit | np.isin(kind, invalid_kinds)
    rgb = np.full((sh, sw, 3), 110, np.uint8)
    rgb[kind == 4] = (255, 0, 0)
    if sphere_invalid:
        cols = np.where((kind == 4).any(axis=0))[0]
        invalid |= (kind == 4) & (uu <= (cols.min() + cols.max()) / 2.)
    if holes:
        invalid |= np.random.default_rng(seed).random((sh, sw)) < holes
    if far_top is not None:
        top = vv < K['cy']
        z = np.where(top, far_top, z)
        invalid = np.where(top, bool(far_top_invalid), invalid)
    coverage = np.ones((sh, sw), bool)
    if border:
        coverage[:border], coverage[-border:], coverage[:, :border], coverage[:, -border:] = False, False, False, False
        invalid |= ~coverage
    z = np.where(invalid, 0., z).astype(np.float32)
    frame = {'rgb': rgb, 'depth_z_m': z, 'invalid': invalid, 'rgb_t': 1., 'depth_t': 1., 'intrinsics': K,
             'aligned': True, 'rectified': True, 'mount_flu_m': list(MOUNT), 'camera_to_body_rotation': np.eye(3)}
    if border_kind == 'coverage':
        frame['coverage'] = coverage
    return frame, kind


def legacy_project_rgbd(frame, state, contract):
    """Pre-fix (fc8aa4d) projection, reproduced as the negative control: one union mask of sensor-invalid,
    out-of-range and uncovered pixels, returned as sensor_invalid; nothing is no-return."""
    _, _, body_r, yaw, _, _ = R.validate_state(state)
    sw, sh, sfx, sfy, scx, scy = R._intrinsics(frame['intrinsics'])
    cam = contract['camera']
    tw, th, tfx, tfy, tcx, tcy = R._intrinsics(cam)
    rgb, z, invalid = np.asarray(frame['rgb']), np.asarray(frame['depth_z_m']), np.asarray(frame['invalid'])
    mount = np.asarray(frame['mount_flu_m'], np.float64)
    optical = np.asarray(frame['camera_to_body_rotation'], np.float64)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rel = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.]]).T @ body_r
    rotation, offset = rel @ optical, rel @ mount - np.asarray(cam['translation_m'])
    depth = contract['depth']
    with np.errstate(invalid='ignore'):
        bad = invalid | ~np.isfinite(z) | (z < depth['minz_m']) | (z > depth['maxz_m'])
    if np.allclose(rotation, np.eye(3), atol=1e-8) and np.allclose(offset, 0, atol=1e-8):
        uu = (np.arange(tw)-tcx)/tfx*sfx + scx
        vv = (np.arange(th)-tcy)/tfy*sfy + scy
        xi = np.clip(np.floor(uu + .5).astype(int), 0, sw-1)
        yi = np.clip(np.floor(vv + .5).astype(int), 0, sh-1)
        out_rgb = rgb[yi[:, None], xi[None, :]].copy()
        out_z = z[yi[:, None], xi[None, :]].astype(np.float64)
        out_bad = bad[yi[:, None], xi[None, :]].copy()
        method = 'inverse-nearest'
    else:
        u, v = np.meshgrid((np.arange(sw)-scx)/sfx, (np.arange(sh)-scy)/sfy)
        good = ~bad
        points = np.stack([z[good], -u[good]*z[good], -v[good]*z[good]], axis=-1) @ rotation.T + offset
        front = points[:, 0] > 1e-6
        points, colours = points[front], rgb[good][front]
        xx = np.floor(-points[:, 1]/points[:, 0]*tfx+tcx+.5).astype(int)
        yy = np.floor(-points[:, 2]/points[:, 0]*tfy+tcy+.5).astype(int)
        keep = (xx >= 0) & (xx < tw) & (yy >= 0) & (yy < th)
        indices, zs, colours = yy[keep]*tw + xx[keep], points[keep, 0], colours[keep]
        order = np.lexsort((zs, indices))
        indices, zs, colours = indices[order], zs[order], colours[order]
        first = np.r_[True, indices[1:] != indices[:-1]] if len(indices) else np.zeros(0, bool)
        out_z, out_rgb, out_bad = np.zeros((th, tw)), np.zeros((th, tw, 3), np.uint8), np.ones((th, tw), bool)
        out_z.flat[indices[first]] = zs[first]
        out_rgb.reshape(-1, 3)[indices[first]] = colours[first]
        out_bad.flat[indices[first]] = False
        out_bad |= (out_z < depth['minz_m']) | (out_z > depth['maxz_m'])
        method = 'measured-point-z-buffer'
    u, v = np.meshgrid((np.arange(tw)-tcx)/tfx, (np.arange(th)-tcy)/tfy)
    cosine = 1 / np.sqrt(1+u*u+v*v)
    ranges = np.full((th, tw), float(depth['maxz_m']))
    np.divide(out_z, cosine, out=ranges, where=~out_bad)
    ranges = np.minimum(ranges, float(depth['maxz_m'])).astype(np.float32)
    return (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None], out_bad[None],
            np.zeros_like(out_bad)[None], cosine.astype(np.float32), {'projection': method})


def slab_columns(half_width, optical_z=.3, shrink=3):
    """Trained columns whose level ray meets |y| <= half_width at optical z, shrunk at both edges."""
    u = np.arange(160)
    inside = np.abs((u - TRAINED['cx']) / TRAINED['fx'] * optical_z) <= half_width
    cols = np.flatnonzero(inside)
    return cols[shrink:-shrink], np.flatnonzero(~inside)


class SourceClassification(unittest.TestCase):
    """project_rgbd's split, on frames small enough to inspect pixel by pixel."""

    def frame(self, z=2.):
        f = {'rgb': np.zeros((90, 160, 3), np.uint8), 'depth_z_m': np.full((90, 160), z, np.float32),
             'invalid': np.zeros((90, 160), bool), 'intrinsics': dict(TRAINED), 'aligned': True,
             'rectified': True, 'mount_flu_m': list(MOUNT), 'camera_to_body_rotation': np.eye(3)}
        return f

    def test_precedence_coverage_then_sensor_then_range(self):
        f = self.frame()
        f['depth_z_m'][0, :6] = [25., np.inf, -np.inf, np.nan, .1, 25.]
        f['invalid'][0, 5] = True              # explicit sensor flag beats z > MaxZ
        f['invalid'][1, :3] = True
        f['coverage'] = np.ones((90, 160), bool)
        f['coverage'][1, :2] = False           # coverage beats the explicit flag
        # Repair r1 (F3): an uncovered pixel WITH a valid return refuses the frame (it no longer becomes
        # no-return); an uncovered pixel without one (here +inf, beyond MaxZ) is no-return.
        f['coverage'][2, 0] = False
        with self.assertRaisesRegex(R.InputError, 'coverage is False at 1 pixels'):
            R.project_rgbd(f, scene_state(1.), contract())
        f['depth_z_m'][2, 0] = np.inf
        _, ranges, si, nr, _, report = R.project_rgbd(f, scene_state(1.), contract())
        si, nr = si[0], nr[0]
        np.testing.assert_array_equal(nr[0, :6], [True, True, False, False, False, False])
        np.testing.assert_array_equal(si[0, :6], [False, False, True, True, True, True])
        np.testing.assert_array_equal(si[1, :3], [False, False, True])
        np.testing.assert_array_equal(nr[1, :3], [True, True, False])
        self.assertTrue(nr[2, 0] and not si[2, 0])
        self.assertFalse((si & nr).any())
        self.assertEqual((int(si.sum()), int(nr.sum())), (5, 5))
        self.assertTrue(np.all(ranges[0][si | nr] == 20.))
        self.assertTrue(report['coverage_mask'])
        self.assertEqual((report['sensor_invalid_pixels'], report['no_return_pixels']), (5, 5))

    def test_coverage_mask_is_validated(self):
        for bad in (np.ones((90, 160), np.uint8), np.ones((89, 160), bool)):
            f = self.frame()
            f['coverage'] = bad
            with self.subTest(dtype=bad.dtype, shape=bad.shape), self.assertRaisesRegex(R.InputError, 'coverage'):
                R.project_rgbd(f, scene_state(1.), contract())

    def test_zbuffer_invalid_rays_rotate_and_ignore_translation(self):
        # Pitch 10 deg nose down: an all-invalid frame projects only ray directions.
        f = self.frame(0.)
        f['invalid'][:] = True
        state = scene_state(1., pitch_deg=10.)
        _, ranges, si, nr, _, report = R.project_rgbd(f, state, contract())
        self.assertEqual(report['projection'], 'measured-point-z-buffer')
        # Rows near the top were not imaged (the camera looks down): no-return, not sensor-invalid.
        self.assertGreater(int(nr.sum()), 0)
        self.assertGreater(int(si.sum()), 0)
        self.assertTrue(nr[0, 0].all() and not si[0, 0].any())
        # Rotation only: with a 0.5 m mount offset the invalid-ray band is unchanged.
        shifted = dict(f, mount_flu_m=[.6, .4, .3])
        _, _, si2, nr2, _, _ = R.project_rgbd(shifted, state, contract())
        np.testing.assert_array_equal(si2, si)
        np.testing.assert_array_equal(nr2, nr)
        # Control: a valid frame under the same tilt leaves the same top rows no-return and no sensor-invalid.
        _, _, si3, nr3, _, _ = R.project_rgbd(self.frame(), state, contract())
        self.assertEqual(int(si3.sum()), 0)
        self.assertGreater(int(nr3.sum()), 0)

    def test_valid_majority_pixels_stay_valid_and_level_minz_applies(self):
        # Renamed in repair r1 (was ..._valid_point_wins_over_invalid_ray_...): the z-buffer now classifies a
        # trained pixel by its invalid-sample share (F4); pixels receiving only valid samples stay valid.
        f = self.frame(2.)
        f['invalid'][::2] = True                       # every other source row invalid
        state = scene_state(1., pitch_deg=2.)
        _, ranges, si, nr, cosine, _ = R.project_rgbd(f, state, contract())
        valid = ~(si | nr)[0]
        self.assertGreater(int(valid.sum()), 0)
        np.testing.assert_allclose((ranges[0] * cosine)[valid], 2., atol=.08)
        # A valid point that lands nearer than MinZ in the level camera is sensor-invalid there.
        g = self.frame(.405)
        _, _, si4, _, _, _ = R.project_rgbd(g, scene_state(1., pitch_deg=20.), contract())
        self.assertGreater(int(si4.sum()), 0)


class FloorAndCeilingGeometry(unittest.TestCase):
    """remove_floor_and_ceiling on exact level-camera planes (trained grid)."""

    def planes(self, height=1., ceiling=None, textureless=False, slab=False):
        f, kind = render('trained_equal', height, wall_x=10., floor='textureless' if textureless else 'textured',
                         ceiling=None if ceiling is None else (ceiling, 'textureless' if textureless else 'textured'),
                         near_slab=(.4, .07) if slab else None)
        return f, kind

    def filtered(self, frame, height, ground, ceiling, plane_rule=False):
        c = contract(ground=ground, ceiling=ceiling, plane_rule=plane_rule)
        state = scene_state(height)
        _, ranges, si, nr, cosine, _ = R.project_rgbd(frame, state, c)
        return R.remove_floor_and_ceiling(ranges, si, nr, cosine, state, c), (ranges, si, nr)

    def test_textured_floor_and_ceiling_become_no_return_and_walls_stay(self):
        f, kind = self.planes(1., ceiling=3.)
        (ranges, si, nr, info), (raw, _, raw_nr) = self.filtered(f, 1., True, 3.)
        floor_px = int((raw[0] < 9.).sum())
        self.assertGreater(info['ground']['removed_valid_pixels'], 0)
        self.assertGreater(info['ceiling']['removed_valid_pixels'], 0)
        # Every pixel below 9 m was floor or ceiling; after removal none is left (wall at 9.9 m stays).
        self.assertGreater(floor_px, 0)
        self.assertEqual(int((ranges[0] < 9.).sum()), 0)
        self.assertGreater(int(((ranges[0] > 9.8) & (ranges[0] < 10.5)).sum()), 0)
        self.assertTrue(np.all(ranges[0][nr[0] & ~raw_nr[0]] == 20.))
        # Disabled filters are the identity (negative control).
        (off, si_off, nr_off, info_off), (raw2, si2, nr2) = self.filtered(f, 1., False, None)
        np.testing.assert_array_equal(off, raw2)
        self.assertEqual(info_off['ground']['removed_valid_pixels'] + info_off['ceiling']['removed_valid_pixels'], 0)

    def test_hit_height_arithmetic_is_exact(self):
        f, kind = self.planes(1.)
        (ranges, si, nr, info), (raw, _, raw_nr) = self.filtered(f, 1., True, None)
        v = (np.arange(90) - 44.5) / TRAINED['fy']
        u = (np.arange(160) - 79.5) / TRAINED['fx']
        cos = 1 / np.sqrt(1 + u[None] ** 2 + v[:, None] ** 2)
        hit = 1.03 - raw[0].astype(np.float64) * v[:, None] * cos
        floor = kind == 3                      # trained-equal level source: source pixel == trained pixel
        np.testing.assert_allclose(hit[floor], 0., atol=1e-4)
        removed = nr[0] & ~raw_nr[0]
        self.assertTrue(floor.any() and np.all(removed[floor]))
        self.assertTrue(np.all(hit[removed] < .1))
        self.assertTrue(np.all(hit[~(si | nr)[0]] >= .1))
        # The wall's bottom 10 cm (kind 1) is inside the floor band too: removed, by the margin's definition.
        self.assertGreater(int((removed & (kind == 1)).sum()), 0)
        self.assertEqual(info['ground']['removed_valid_pixels'], int(removed.sum()))

    def test_textureless_planes_are_explained_and_a_near_slab_survives(self):
        # Repair r1 (F1): the invalid-pixel plane rule is opt-in; these cases enable it explicitly.
        for ceiling in (None, 2.5, 3.):
            with self.subTest(ceiling=ceiling):
                f, kind = self.planes(1., ceiling=ceiling, textureless=True)
                (ranges, si, nr, info), (raw, raw_si, _) = self.filtered(f, 1., True, ceiling, plane_rule=True)
                self.assertGreater(int(raw_si.sum()), 0)
                self.assertEqual(int(si.sum()), 0)
                self.assertGreater(info['ground']['removed_invalid_pixels'], 0)
                if ceiling:
                    self.assertGreater(info['ceiling']['removed_invalid_pixels'], 0)
                # Default contract (rule off): the textureless planes stay sensor-invalid (open item, F1).
                (_, si_default, _, info_default), _ = self.filtered(f, 1., True, ceiling)
                np.testing.assert_array_equal(si_default, raw_si)
                self.assertEqual(info_default['ground']['removed_invalid_pixels'], 0)
                self.assertFalse(info_default['ground']['invalid_plane_rule'])
                f, kind = self.planes(1., ceiling=ceiling, textureless=True, slab=True)
                (ranges, si, nr, info), _ = self.filtered(f, 1., True, ceiling, plane_rule=True)
                cols, _ = slab_columns(.07)
                # The slab keeps sensor-invalid rows where no plane is within [MinZ, MaxZ] (around the horizon).
                self.assertTrue(si[0][:, cols].any(axis=0).all())
                self.assertGreaterEqual(int(si[0][:, cols[len(cols) // 2]].sum()), 8)

    def test_camera_outside_the_bands_is_refused(self):
        f, _ = self.planes(1.)
        c = contract(ground=True, ceiling=3.)
        for height, message in ((.05, 'ground margin'), (2.9, 'ceiling band')):
            state = scene_state(height)
            _, ranges, si, nr, cosine, _ = R.project_rgbd(f, state, c)
            with self.subTest(height=height), self.assertRaisesRegex(R.InputError, message):
                R.remove_floor_and_ceiling(ranges, si, nr, cosine, state, c)
        state = scene_state(1.)
        _, ranges, si, nr, cosine, _ = R.project_rgbd(f, state, c)
        R.remove_floor_and_ceiling(ranges, si, nr, cosine, state, c)  # positive control


# ------------------------------------------------------------------------------------------------- H4 pose
def header(frame, t):
    ns = round(t * 1e9)
    return NS(frame_id=frame, stamp=NS(sec=ns // 1000000000, nanosec=ns % 1000000000))


def odometry_message(t, pos, yaw, vel_world, yaw_rate):
    c, s = math.cos(yaw), math.sin(yaw)
    body = [c * vel_world[0] + s * vel_world[1], -s * vel_world[0] + c * vel_world[1], vel_world[2]]
    return NS(header=header('odom', t), child_frame_id='base_link',
              pose=NS(pose=NS(position=NS(x=pos[0], y=pos[1], z=pos[2]),
                              orientation=NS(x=0., y=0., z=math.sin(yaw / 2), w=math.cos(yaw / 2)))),
              twist=NS(twist=NS(linear=NS(x=body[0], y=body[1], z=body[2]), angular=NS(x=0., y=0., z=yaw_rate))))


def image_messages(t, rgb, depth_mm, intrinsics):
    h, w = depth_mm.shape
    k = intrinsics
    return dict(rgb=NS(header=header('camera_optical', t), width=w, height=h, step=3 * w, encoding='rgb8',
                       is_bigendian=0, data=np.ascontiguousarray(rgb).tobytes()),
                depth=NS(header=header('camera_optical', t), width=w, height=h, step=2 * w, encoding='16UC1',
                         is_bigendian=0, data=np.ascontiguousarray(depth_mm.astype('<u2')).tobytes()),
                camera_info=NS(header=header('camera_optical', t), width=w, height=h, binning_x=0, binning_y=0,
                               roi=NS(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False),
                               k=[k['fx'], 0., k['cx'], 0., k['fy'], k['cy'], 0., 0., 1.],
                               p=[k['fx'], 0., k['cx'], 0., 0., k['fy'], k['cy'], 0., 0., 0., 1., 0.],
                               r=np.eye(3).ravel().tolist(), d=[]))


def ros_binding(path):
    path.write_text('CPU fixture calibration only, not real hardware evidence')
    proof = dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    b = A.unconfigured_template()
    b.update(status='CONFIGURED', frames=dict(world='odom', body='base_link', optical='camera_optical'))
    b['geometry'] = dict(mount_flu_m=list(MOUNT), optical_to_body_rotation=A.CAMERA_FLU_TO_OPTICAL.T.tolist(),
                         provenance=proof)
    b['producer'] = dict(rectified=True, depth_aligned_to_rgb=True, provenance=proof)
    b['clock']['source'] = 'fixture shared ROS clock'
    return b


def motion(t, speed=2.5, yaw_rate=0., t0=3., height=1.):
    """Constant-velocity, constant-yaw-rate truth: x = speed * (t - t0); yaw = yaw_rate * (t - t0)."""
    return [speed * (t - t0), 0., height], yaw_rate * (t - t0), [speed, 0., 0.], yaw_rate


def lifted_goal(state, vehicle_measurement=(3., .5, 0.)):
    rot = R.quaternion_rotation(state['quat_xyzw'])
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    m = vehicle_measurement
    return np.asarray(state['pos']) + np.array([c * m[0] - s * m[1], s * m[0] + c * m[1], m[2]])


class CapturingRuntime:
    def __init__(self): self.calls, self.failed = [], False
    def reset(self): self.failed = False
    def invalidate(self): self.failed = True
    # Always inside the floor/ceiling band (r2 altitude gating is tested in test_camera_runtime_fixes_r2).
    def height_band(self, state): return dict(in_band=True, camera_height_m=state['pos'][2] + .03, reason=None)
    def step(self, frame, state, *, now_s):
        self.calls.append((frame, state, now_s))
        return dict(t=now_s, velocity_world_enu_mps=[0., 0., 0.], yaw_rate_rps=0.)


class PoseInterpolation(unittest.TestCase):
    T_IMG = 3.2

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.runtime = CapturingRuntime()
        self.session = RosPolicySession(self.runtime, ros_binding(Path(self.tmp.name) / 'fixture.txt'))

    def tick(self, odometry_offsets, *, yaw_rate=0., now_offset=.03):
        t = self.T_IMG
        rgb, depth = np.zeros((90, 160, 3), np.uint8), np.full((90, 160), 2000, np.uint16)
        for kind, message in image_messages(t, rgb, depth, TRAINED).items():
            self.session.offer(kind, message, received_monotonic_s=100.)
        for dt in odometry_offsets:
            pos, yaw, vel, rate = motion(t + dt, yaw_rate=yaw_rate)
            self.session.offer('odometry', odometry_message(t + dt, pos, yaw, vel, rate), received_monotonic_s=100.)
        item = self.session.prepare(now_ros_s=t + now_offset, now_monotonic_s=100.01)
        if item is None:
            return None, None
        result = self.session.infer(item)
        proposal = self.session.finish(item, result, now_ros_s=t + now_offset, now_monotonic_s=100.01)
        return result, proposal

    def truth(self, yaw_rate=0.):
        pos, yaw, _, _ = motion(self.T_IMG, yaw_rate=yaw_rate)
        return {'pos': pos, 'quat_xyzw': [0., 0., math.sin(yaw / 2), math.cos(yaw / 2)]}

    def test_bracketed_stream_lifts_goal_at_capture_time_and_nearest_control_errs_5cm(self):
        for yaw_rate in (0., 1.):
            with self.subTest(yaw_rate=yaw_rate):
                self.setUp()
                result, proposal = self.tick((-.02, .02), yaw_rate=yaw_rate)
                _, state, _ = self.runtime.calls[-1]
                self.assertTrue(result['pose_interpolated'] and proposal['pose_interpolated'])
                self.assertEqual(state['t'], self.T_IMG)
                self.assertEqual(result['pose_skew_s'], 0.)
                error = np.linalg.norm(lifted_goal(state) - lifted_goal(self.truth(yaw_rate)))
                self.assertLess(error, .01)
                np.testing.assert_allclose(state['vel'][0], 2.5, atol=1e-9)
                # Control: only the sample 20 ms after the image (the current nearest selection).
                self.setUp()
                result, proposal = self.tick((.02,), yaw_rate=yaw_rate)
                _, state, _ = self.runtime.calls[-1]
                self.assertFalse(result['pose_interpolated'] or proposal['pose_interpolated'])
                self.assertAlmostEqual(result['pose_skew_s'], .02, places=6)
                control = np.linalg.norm(lifted_goal(state) - lifted_goal(self.truth(yaw_rate)))
                if yaw_rate == 0.:
                    self.assertAlmostEqual(control, .05, delta=.002)
                else:
                    self.assertGreater(control, .05)

    def test_gaps_are_rejected_or_not_interpolated_and_nothing_is_extrapolated(self):
        # Gap 0.2 s around the image: no sample within 25 ms, so the frame is still rejected.
        result, _ = self.tick((-.1, .1), now_offset=.11)
        self.assertIsNone(result)
        self.assertEqual(self.runtime.calls, [])
        # Gap 0.16 s > 0.15 s: the nearest (+10 ms) is used without interpolation. (In the session the
        # earlier end of such a bracket is always older than the 0.15 s age bound; interpolate_state
        # enforces the bracket bound by itself, see the next test.)
        self.setUp()
        result, _ = self.tick((-.15, .01), now_offset=.011)
        self.assertFalse(result['pose_interpolated'])
        self.assertAlmostEqual(self.runtime.calls[-1][1]['t'], self.T_IMG + .01, places=9)
        # Positive control: a 0.14 s bracket is interpolated.
        self.setUp()
        result, _ = self.tick((-.13, .01), now_offset=.011)
        self.assertTrue(result['pose_interpolated'])
        self.assertEqual(self.runtime.calls[-1][1]['t'], self.T_IMG)
        # Both samples before the image: nearest only, never extrapolated forward.
        self.setUp()
        result, _ = self.tick((-.02, -.01))
        self.assertFalse(result['pose_interpolated'])
        self.assertAlmostEqual(self.runtime.calls[-1][1]['t'], self.T_IMG - .01, places=9)
        # Exact sample at the image: used as is.
        self.setUp()
        result, _ = self.tick((0.,))
        self.assertFalse(result['pose_interpolated'])
        self.assertEqual(result['pose_skew_s'], 0.)

    def test_interpolate_state_refuses_extrapolation_and_long_brackets_and_slerps_short_arc(self):
        a = {'pos': [0., 0., 1.], 'vel': [1., 0., 0.], 'quat_xyzw': [0., 0., 0., 1.], 'yaw_rate_rps': 0., 't': 1.}
        yaw = math.radians(20)
        b = dict(a, pos=[.1, 0., 1.], quat_xyzw=[0., 0., -math.sin(yaw / 2), -math.cos(yaw / 2)], t=1.1)
        mid = A.interpolate_state(a, b, 1.05)
        self.assertEqual(set(mid), set(a))
        np.testing.assert_allclose(mid['pos'], [.05, 0., 1.], atol=1e-12)
        rot = R.quaternion_rotation(mid['quat_xyzw'])
        self.assertAlmostEqual(math.degrees(math.atan2(rot[1, 0], rot[0, 0])), 10., places=9)
        for t in (.99, 1.11):
            with self.assertRaisesRegex(A.RosInputError, 'extrapolation'):
                A.interpolate_state(a, b, t)
        with self.assertRaisesRegex(A.RosInputError, 'bracket'):
            A.interpolate_state(a, dict(b, t=1.16), 1.05)


# ----------------------------------------------------------------------------------- actual policy (seed 1)
ACTUAL = r'''
import json, math, sys, tempfile
from pathlib import Path
import numpy as np
import test_camera_runtime_fixes as F
from camera_policy_contract import build_contract, GROUND_DEFAULT, CEILING_DEFAULT
from camera_policy_runtime import CameraPolicyRuntime
import camera_policy_runtime as R
from camera_ros2_session import RosPolicySession

from camera_policy_contract import estimator_height_assertion
_evidence_dir = tempfile.TemporaryDirectory()
_evidence = Path(_evidence_dir.name) / 'estimator_height_fixture.txt'
_evidence.write_text('CPU test fixture: synthetic scenes put estimator z at the height above the floor. Not evidence.')
ASSERTED = estimator_height_assertion(_evidence)
def contract_for(ground, ceiling=None, plane_rule=False):
    """Repair r1: the plane rule (F1) is opt-in per variant; the estimator-height assertion (F6) is a fixture."""
    return build_contract(1, ground=dict(GROUND_DEFAULT, enabled=ground, invalid_plane_rule=bool(ground and plane_rule)),
                          ceiling=dict(CEILING_DEFAULT) if ceiling is None else
                          dict(CEILING_DEFAULT, enabled=True, height_m=ceiling, invalid_plane_rule=plane_rule),
                          estimator_height=ASSERTED)
contracts = {'off': contract_for(False), 'on': contract_for(True), 'on+plane': contract_for(True, plane_rule=True)}
rt = CameraPolicyRuntime(contracts['on'])
torch = rt.torch
bearings = rt.bearings.numpy()
inside = np.abs(bearings) <= math.radians(33.0)
fixed_preprocess = rt.preprocess

def legacy_preprocess(frame, state):
    rgb, ranges, bad, _, cosine, report = F.legacy_project_rgbd(frame, state, rt.contract)
    d = rt.contract['depth']
    depth, near = rt.validity.apply_mask(torch.from_numpy(ranges), torch.from_numpy(bad), torch.from_numpy(cosine),
                                         min_z_m=d['minz_m'], window=d['window'], density=d['density'])
    counts = dict(sensor_invalid_pixels=int(bad.sum()), no_return_pixels=0, near_pixels=int(near.sum()),
                  ground_removed_pixels=0, ceiling_removed_pixels=0, height_filter={}, rgb_blanked_pixels=0,
                  goal_invalid_pixels=0)
    return torch.from_numpy(rgb), depth, counts, report

def use(variant):
    """Test-only switch of the validated deployment filter fields; the contract is rebuilt and validated."""
    if variant == 'legacy':
        rt.contract = contracts['off']
        return legacy_preprocess
    if variant not in contracts:
        kind, height = variant.split('@')
        contracts[variant] = contract_for(kind.startswith('on'), float(height), plane_rule=kind.endswith('+plane'))
    rt.contract = contracts[variant]
    return fixed_preprocess

def step(frame, state, variant):
    preprocess = use(variant)
    seen = {}
    def capture(f, s):
        out = preprocess(f, s)
        seen['rgb'], seen['depth'] = out[0], out[1]
        return out
    rt.preprocess = capture
    try:
        rt.reset()
        res = rt.step(dict(frame, rgb_t=1., depth_t=1.), dict(state, t=1.), now_s=1.)
    finally:
        rt.preprocess = fixed_preprocess
    obs = np.asarray(res['observation'])
    scan = obs[:288].reshape(4, 72) * 12.
    nearest = scan.min(axis=0)
    tok = obs[288:288 + 480].reshape(5, 8, 12)[-1]
    valid = tok[:, 11] > .5
    tokens = [(round(float(12. * (1. - t[7])), 3), round(math.degrees(math.atan2(t[1], t[0])), 2)) for t in tok[valid]]
    cols = seen['depth'][0].amin(dim=0).numpy()
    return dict(near=res['near_pixels'], si=res['sensor_invalid_pixels'], nr=res['no_return_pixels'],
                ground=res['ground_removed_pixels'], ceiling=res['ceiling_removed_pixels'],
                infov=int(inside.sum()), infov_lt1=int((nearest[inside] < 1.).sum()),
                infov_min=float(nearest[inside].min()), cols=[round(float(c), 4) for c in cols],
                col_lt1=int((cols < 1.).sum()), tokens=tokens, obs=obs.tolist(), projection=res['projection']['projection'])

def detect(frame, state, variant):
    preprocess = use(variant)
    rgb, depth, counts, _ = preprocess(frame, state)
    rt.reset()
    with torch.inference_mode():
        _, surface, bearing, visible, _, mask = rt.perception._detect_rgbd(rgb, depth, False)
    rt.reset()
    return dict(surface=float(surface[0]), visible=bool(visible[0]), mask=int(mask.sum()), near=counts['near_pixels'],
                si=counts['sensor_invalid_pixels'], goal_invalid=counts.get('goal_invalid_pixels', 0),
                bearing=math.degrees(float(bearing[0])))

out = {}
S, st = F.render, F.scene_state
# 1. tilt over a wall at 10 m (OAK-like source)
for axis, angles in (('pitch', (0., 1., -1., 3., -3., 5., -5., 10., -10.)), ('roll', (5., -5.))):
    for a in angles:
        kw = {axis + '_deg': a}
        f, _ = S('oakd_like', 1., wall_x=10., **kw)
        for variant in ('off', 'on', 'legacy'):
            out['tilt %s %+g %s' % (axis, a, variant)] = step(f, st(1., **kw), variant)
# 2. pole at 3 m
for axis, a in (('pitch', 0.), ('pitch', 3.), ('pitch', 5.), ('roll', 5.)):
    kw = {axis + '_deg': a}
    f, _ = S('oakd_like', 1., wall_x=10., pole=(3., 0., .1), **kw)
    for variant in ('off', 'on', 'legacy'):
        out['pole %s %+g %s' % (axis, a, variant)] = step(f, st(1., **kw), variant)
# 3. dense slab closer than MinZ; iid 2 % holes
for a in (0., 3.):
    f, _ = S('oakd_like', 1., pitch_deg=a, wall_x=10., near_slab=(.4, .07))
    for variant in ('off', 'on', 'legacy'):
        out['slab pitch %+g %s' % (a, variant)] = step(f, st(1., pitch_deg=a), variant)
    for source in ('oakd_like', 'trained_equal'):
        f, _ = S(source, 1., pitch_deg=a, wall_x=10., holes=.02, seed=7)
        out['holes %s pitch %+g on' % (source, a)] = step(f, st(1., pitch_deg=a), 'on')
# 4. undistortion border, coverage-False vs sensor-invalid
for a in (0., 3.):
    for kind in ('coverage', 'invalid'):
        f, _ = S('oakd_like', 1., pitch_deg=a, wall_x=10., border=60, border_kind=kind)
        out['border %s pitch %+g off' % (kind, a)] = step(f, st(1., pitch_deg=a), 'off')
# 5. optical z beyond MaxZ above the horizon
for a in (0., 3.):
    for far in (25., float('inf')):
        for flagged in (False, True):
            f, _ = S('oakd_like', 1., pitch_deg=a, wall_x=10., far_top=far, far_top_invalid=flagged)
            out['far %g flagged %d pitch %+g off' % (far, flagged, a)] = step(f, st(1., pitch_deg=a), 'off')
# 6. floor
for h in (1., 1.5):
    for floor in ('textured', 'textureless'):
        f, _ = S('oakd_like', h, wall_x=10., floor=floor)
        for variant in ('on', 'off', 'on+plane'):
            out['floor %s h%g %s' % (floor, h, variant)] = step(f, st(h), variant)
f, _ = S('oakd_like', 1., pitch_deg=3., wall_x=10., floor='textured')
out['floor textured h1 pitch +3 on'] = step(f, st(1., pitch_deg=3.), 'on')
f, _ = S('oakd_like', 1.5, wall_x=10., floor='textured', pole=(3., 0., .1))
for variant in ('on', 'off'):
    out['pole over floor h1.5 %s' % variant] = step(f, st(1.5), variant)
f, _ = S('oakd_like', 1., wall_x=10., floor='textureless', near_slab=(.4, .07))
out['slab over textureless floor on'] = step(f, st(1.), 'on+plane')
# B2. ceiling (far wall at 15 m, as the coordinator's probe)
for hc in (2.5, 3.):
    for kind in ('textured', 'textureless'):
        f, _ = S('oakd_like', 1., wall_x=15., ceiling=(hc, kind))
        for variant in ('on@%g' % hc, 'on', 'on+plane@%g' % hc):
            out['ceiling %s %g %s' % (kind, hc, variant)] = step(f, st(1.), variant)
f, _ = S('oakd_like', 1.5, wall_x=15., floor='textured', ceiling=(3., 'textured'), pole=(3., 0., .1))
for variant in ('on@3', 'off'):
    out['room pole h1.5 %s' % variant] = step(f, st(1.5), variant)
f, _ = S('oakd_like', 1., wall_x=15., floor='textureless', ceiling=(3., 'textureless'), near_slab=(.4, .07))
for variant in ('on+plane@3', 'on@3', 'off'):
    out['room slab textureless %s' % variant] = step(f, st(1.), variant)
f, _ = S('oakd_like', 1., wall_x=15., floor='textureless', ceiling=(3., 'textureless'))
out['room textureless empty on+plane@3'] = step(f, st(1.), 'on+plane@3')
out['room textureless empty on@3'] = step(f, st(1.), 'on@3')
f, _ = S('oakd_like', 1., wall_x=2.1, floor='textured', ceiling=(3., 'textured'))
for variant in ('on@3', 'off'):
    out['room front wall 2m %s' % variant] = step(f, st(1.), variant)
f, _ = S('oakd_like', 1., wall_x=15., floor='textured', ceiling=(3., 'textured'), side_wall_y=1.1)
for variant in ('on@3', 'off'):
    out['room side wall y1.1 %s' % variant] = step(f, st(1.), variant)
# 7. goal sphere 5 m ahead at camera height; true surface range 4.75 m
for source, a in (('trained_equal', 0.), ('oakd_like', 0.), ('oakd_like', .5)):
    for half in (False, True):
        f, _ = S(source, 1., pitch_deg=a, wall_x=10., sphere=(5., 0., 1.03, .15), sphere_invalid=half)
        for variant in ('on', 'legacy'):
            out['goal %s pitch %g half %d %s' % (source, a, half, variant)] = detect(f, st(1., pitch_deg=a), variant)
# 8. ROS session -> adapter -> runtime: lifted goal (first tracker state) at 2.5 m/s, image 20 ms before the latest pose
rt.contract = contracts['on']
t_img = 3.2
pos_img, _, _, _ = F.motion(t_img)
f, _ = S('trained_equal', 1., x=pos_img[0], wall_x=pos_img[0] + 10., sphere=(pos_img[0] + 5., 0., 1.03, .15))
depth_mm = np.round(np.asarray(f['depth_z_m'], np.float64) * 1000.).astype(np.uint16)
def lifted(offsets=None, truth=False):
    if truth:
        rt.reset()
        frame = dict(f, rgb_t=t_img, depth_t=t_img)
        frame['depth_z_m'] = depth_mm.astype(np.float32) * np.float32(.001)
        frame['invalid'] = depth_mm == 0
        rt.step(frame, F.scene_state(1., x=pos_img[0], t=t_img), now_s=t_img + .03)
        return rt.perception.tracker.state[0, :3].tolist(), None
    with tempfile.TemporaryDirectory() as tmp:
        session = RosPolicySession(rt, F.ros_binding(Path(tmp) / 'fixture.txt'))
        for kind, message in F.image_messages(t_img, f['rgb'], depth_mm, F.TRAINED).items():
            session.offer(kind, message, received_monotonic_s=100.)
        for dt in offsets:
            pos, yaw, vel, rate = F.motion(t_img + dt)
            session.offer('odometry', F.odometry_message(t_img + dt, pos, yaw, vel, rate), received_monotonic_s=100.)
        item = session.prepare(now_ros_s=t_img + .03, now_monotonic_s=100.01)
        result = session.infer(item)
        session.finish(item, result, now_ros_s=t_img + .03, now_monotonic_s=100.01)
        return rt.perception.tracker.state[0, :3].tolist(), result['pose_interpolated']
out['lift truth'] = lifted(truth=True)
out['lift interpolated'] = lifted((-.02, .02))
out['lift nearest'] = lifted((.02,))
print('FIX_R1_RESULT ' + json.dumps(out, allow_nan=False))
'''


class ActualPolicySeed1(unittest.TestCase):
    """Full CameraPolicyRuntime.step with the real CAM-K2-v3 seed-1 checkpoint, one fresh CPU process."""

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
        line = [x for x in done.stdout.splitlines() if x.startswith('FIX_R1_RESULT ')]
        cls.out = json.loads(line[-1][len('FIX_R1_RESULT '):])

    def tokens_near(self, row, lo, hi, bearing=5.):
        return [t for t in row['tokens'] if lo <= t[0] <= hi and abs(t[1]) <= bearing]

    def assert_clear(self, row, label):
        self.assertEqual(row['near'], 0, label)
        self.assertEqual(row['infov_lt1'], 0, label)
        self.assertEqual(row['col_lt1'], 0, label)
        self.assertFalse([t for t in row['tokens'] if t[0] < 1.], label)

    def test_1_tilt_and_roll_leave_no_phantom_wall(self):
        out = self.out
        for key, row in out.items():
            if not key.startswith('tilt '):
                continue
            label = key + ' ' + json.dumps({k: row[k] for k in ('near', 'si', 'nr', 'infov_lt1', 'col_lt1')})
            angle = abs(float(key.split()[2]))
            if key.endswith(' legacy'):
                # Pre-fix pipeline (negative control): the uncovered band is a 0.4-0.5 m wall.
                if 'pitch' in key and angle >= 3:
                    self.assertEqual(row['infov_lt1'], row['infov'], label)
                    self.assertGreater(row['near'], 0, label)
                elif 'roll' in key:
                    self.assertGreater(row['infov_lt1'], 0, label)
                else:
                    self.assertEqual(row['infov_lt1'], 0, label)   # below the OAK-like onset (about 1.5 deg)
                continue
            self.assert_clear(row, label)
            if angle >= 3:
                self.assertGreater(row['nr'], 0, label)          # the band exists and is no-return
        # Positive control: where nothing was wrong (level), the fix is the identity on the observation.
        self.assertEqual(out['tilt pitch +0 off']['obs'], out['tilt pitch +0 legacy']['obs'])
        self.assertEqual(out['tilt pitch +0 off']['projection'], 'inverse-nearest')
        self.assertEqual(out['tilt pitch +3 off']['projection'], 'measured-point-z-buffer')

    def test_2_pole_at_3m_survives_tilt(self):
        for axis, a in (('pitch', 0.), ('pitch', 3.), ('pitch', 5.), ('roll', 5.)):
            for variant in ('off', 'on'):
                key = 'pole %s %+g %s' % (axis, a, variant)
                row = self.out[key]
                with self.subTest(key=key):
                    self.assertTrue(self.tokens_near(row, 2.7, 2.9), (key, row['tokens']))
                    self.assertFalse([t for t in row['tokens'] if t[0] < 1.], (key, row['tokens']))
        legacy = self.out['pole pitch +3 legacy']
        self.assertTrue([t for t in legacy['tokens'] if t[0] < 1.], legacy['tokens'])
        self.assertFalse(self.tokens_near(legacy, 2.7, 2.9), legacy['tokens'])
        self.assertTrue(self.tokens_near(self.out['pole pitch +0 legacy'], 2.7, 2.9))

    def test_3_dense_slab_stays_near_and_isolated_holes_do_not(self):
        cols, outside = slab_columns(.07)
        for a in (0., 3.):
            for variant in ('off', 'on', 'legacy'):
                key = 'slab pitch %+g %s' % (a, variant)
                row = self.out[key]
                with self.subTest(key=key):
                    c = np.asarray(row['cols'])
                    self.assertGreater(row['near'], 0)
                    self.assertTrue(np.all((c[cols] >= .40) & (c[cols] <= .48)), c[cols])
                    if variant != 'legacy' or a == 0.:
                        self.assertTrue(np.all(c[outside[(outside < cols[0] - 6) | (outside > cols[-1] + 6)]] >= 9.))
        for key, row in self.out.items():
            if key.startswith('holes '):
                with self.subTest(key=key):
                    self.assertEqual(row['near'], 0)
                    self.assertEqual(row['col_lt1'], 0)
        for key in ('holes oakd_like pitch +0 on', 'holes trained_equal pitch +0 on', 'holes trained_equal pitch +3 on'):
            self.assertGreater(self.out[key]['si'], 0, key)    # the holes are present and sensor-invalid

    def test_4_uncovered_border_is_no_return_while_invalid_border_is_near(self):
        for a in (0., 3.):
            cov, inv = self.out['border coverage pitch %+g off' % a], self.out['border invalid pitch %+g off' % a]
            with self.subTest(pitch=a):
                self.assert_clear(cov, 'coverage %g' % a)
                self.assertGreater(cov['nr'], 0)
                self.assertGreater(inv['near'], 0)
                self.assertGreater(inv['col_lt1'], 0)

    def test_5_beyond_maxz_is_no_return_not_near(self):
        for a in (0., 3.):
            for far in ('25', 'inf'):
                ok, flagged = (self.out['far %s flagged %d pitch %+g off' % (far, k, a)] for k in (0, 1))
                with self.subTest(pitch=a, far=far):
                    self.assert_clear(ok, 'far')
                    self.assertGreater(ok['nr'], 0)
                    self.assertGreater(flagged['near'], 0)
                    self.assertGreater(flagged['col_lt1'], 0)

    def test_6_floor_is_removed_and_obstacles_on_it_are_kept(self):
        out = self.out
        for h, (lo, hi) in ((1., (2.2, 2.45)), (1.5, (3.3, 3.55))):
            on, off = out['floor textured h%g on' % h], out['floor textured h%g off' % h]
            with self.subTest(height=h):
                self.assertFalse([t for t in on['tokens'] if t[0] < 9.], on['tokens'])
                self.assertGreater(on['ground'], 0)
                self.assertTrue(off['tokens'] and lo <= min(t[0] for t in off['tokens']) <= hi, off['tokens'])
                # Repair r1 (F1): the invalid-pixel plane rule is opt-in ('on+plane'). With it the textureless
                # floor is explained; with the default contract ('on', rule off) it reads near like 'off'.
                ton, toff = out['floor textureless h%g on+plane' % h], out['floor textureless h%g off' % h]
                tdefault = out['floor textureless h%g on' % h]
                self.assert_clear(ton, 'textureless on+plane %g' % h)
                self.assertGreater(ton['ground'], 0)
                self.assertGreater(toff['near'], 0)
                self.assertGreater(toff['col_lt1'], 0)
                self.assertGreater(tdefault['near'], 0)
                self.assertGreater(tdefault['col_lt1'], 0)
                self.assertEqual(out['floor textured h%g on+plane' % h]['tokens'], on['tokens'])
        tilted = out['floor textured h1 pitch +3 on']
        self.assertFalse([t for t in tilted['tokens'] if t[0] < 9.], tilted['tokens'])
        self.assertTrue(self.tokens_near(out['pole over floor h1.5 on'], 2.7, 2.9), out['pole over floor h1.5 on']['tokens'])
        slab = out['slab over textureless floor on']
        cols, _ = slab_columns(.07)
        c = np.asarray(slab['cols'])
        self.assertGreater(slab['near'], 0)
        self.assertTrue(np.all((c[cols] >= .40) & (c[cols] <= .48)), c[cols])

    def test_b2_ceiling_is_removed_and_vertical_walls_and_poles_are_kept(self):
        out = self.out
        for hc in ('2.5', '3'):
            on, off = out['ceiling textured %s on@%s' % (hc, hc)], out['ceiling textured %s on' % hc]
            # Repair r1 (F1): the textureless ceiling is explained only with the opt-in plane rule.
            ton, toff = out['ceiling textureless %s on+plane@%s' % (hc, hc)], out['ceiling textureless %s on' % hc]
            tdefault = out['ceiling textureless %s on@%s' % (hc, hc)]
            with self.subTest(ceiling=hc):
                for row in (on, ton):
                    self.assertEqual(row['near'], 0)
                    self.assertTrue(min(row['cols']) >= 9., min(row['cols']))
                    self.assertGreater(row['ceiling'], 0)
                lo, hi = (3.2, 3.6) if hc == '2.5' else (4.3, 4.8)     # textured ceiling without the filter
                self.assertTrue(lo <= min(off['cols']) <= hi, min(off['cols']))
                self.assertGreater(toff['near'], 0)
                self.assertTrue(.40 <= min(toff['cols']) <= .48, min(toff['cols']))
                self.assertGreater(tdefault['near'], 0)                 # rule off by default: reads near
                self.assertTrue(.40 <= min(tdefault['cols']) <= .48, min(tdefault['cols']))
                self.assertEqual(out['ceiling textured %s on+plane@%s' % (hc, hc)]['cols'], on['cols'])
        self.assertTrue(self.tokens_near(out['room pole h1.5 on@3'], 2.7, 2.9), out['room pole h1.5 on@3']['tokens'])
        cols, _ = slab_columns(.07)
        for variant in ('on+plane@3', 'on@3', 'off'):
            c = np.asarray(out['room slab textureless %s' % variant]['cols'])
            self.assertTrue(np.all((c[cols] >= .40) & (c[cols] <= .48)), (variant, c[cols]))
        self.assert_clear(out['room textureless empty on+plane@3'], 'empty textureless room, plane rule on')
        self.assertGreater(out['room textureless empty on@3']['near'], 0)     # default: an open item (F1)
        front = np.asarray(out['room front wall 2m on@3']['cols'])
        self.assertAlmostEqual(float(front[79:81].min()), 2.0, delta=.02)
        self.assertTrue(np.all(front <= 2.0 / math.cos(math.radians(33.5))), front.max())
        np.testing.assert_allclose(front, out['room front wall 2m off']['cols'], atol=1e-4)
        side_on, side_off = (np.asarray(out['room side wall y1.1 %s' % v]['cols']) for v in ('on@3', 'off'))
        u = np.arange(160)
        expected = 1.1 * np.sqrt(1 + ((u - 79.5) / TRAINED['fx']) ** 2) / np.maximum((79.5 - u) / TRAINED['fx'], 1e-9)
        left = (u <= 20)
        # Filters on: exactly the side wall's horizon range (2.03 m at the left edge).
        np.testing.assert_allclose(side_on[left], expected[left], rtol=.01)
        self.assertTrue(np.all(side_on[left] < 2.6))
        # Filters off: the floor (hit height 0) is nearer in some of these columns; never farther.
        self.assertTrue(np.all(side_off[left] <= side_on[left] + 1e-4))
        self.assertTrue(np.any(side_off[left] < side_on[left] - .01))

    def test_7_goal_range_never_uses_substituted_pixels(self):
        out = self.out
        for source, a in (('trained_equal', 0), ('oakd_like', 0), ('oakd_like', .5)):
            fixed = out['goal %s pitch %g half 1 on' % (source, a)]
            legacy = out['goal %s pitch %g half 1 legacy' % (source, a)]
            valid_fixed = out['goal %s pitch %g half 0 on' % (source, a)]
            valid_legacy = out['goal %s pitch %g half 0 legacy' % (source, a)]
            with self.subTest(source=source, pitch=a):
                self.assertTrue(fixed['visible'] and valid_fixed['visible'])
                self.assertAlmostEqual(fixed['surface'], 4.75, delta=.1)
                # Repair r1 (F5): the invalid half is red, so the production colour rule makes it no-return:
                # present (goal_invalid > 0), no longer sensor-invalid, and never a near blob on either path.
                self.assertGreater(fixed['goal_invalid'], 0)
                self.assertEqual(fixed['si'], 0)
                self.assertEqual(fixed['near'], 0)
                self.assertAlmostEqual(valid_fixed['surface'], valid_legacy['surface'], places=5)
                self.assertEqual(valid_fixed['mask'], valid_legacy['mask'])
                if a == 0:
                    self.assertLess(legacy['surface'], 4.75 - 1.0)   # pre-fix inverse-nearest path read short

    def test_8_ros_pose_interpolation_lifts_the_goal_at_capture_time(self):
        truth, _ = self.out['lift truth']
        interpolated, flag = self.out['lift interpolated']
        nearest, nearest_flag = self.out['lift nearest']
        self.assertTrue(flag)
        self.assertFalse(nearest_flag)
        self.assertLess(np.linalg.norm(np.subtract(interpolated, truth)), .01)
        self.assertAlmostEqual(float(np.linalg.norm(np.subtract(nearest, truth))), .05, delta=.005)


if __name__ == '__main__':
    unittest.main()
