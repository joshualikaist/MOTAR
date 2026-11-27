"""Camera-runtime outdoor profile r1 (2026-11-13): sky rule, camera-verified ground plane, virtual ceiling.

The policy (CAM-K2 v3, camera-only) was trained without floor, walls or ceiling: a ray that hit nothing was
no-return (MaxZ 20 m). The OAK-D reports depth 0 for open sky, so with the indoor runtime the sky became a 0.4 m
wall on every bearing (blob_near). The outdoor profile (contract profile 'outdoor', opt-in) verifies the ground
plane from the camera on every frame, cross-checks it with PX4 (z, z_valid, epv, dead_reckoning) and refuses the
frame with a named reason when it cannot (FrameRefused); the floor filter and the 3.0 m virtual ceiling are
measured from that plane; the sky rule turns sky and far-ground sensor-invalid pixels into no-return.

OAK-D emulation used by every scene here (outdoor_scene). SYNTHETIC, not real-sensor evidence:
  * depth 0 (sensor-invalid) for: rays that hit nothing (sky, clouds); surfaces closer than MinZ (0.4 m optical);
    surfaces declared textureless; thin structures (wires 1-3 source pixels thick: stereo fails on them);
  * far ground beyond MaxZ: variant 'valid' keeps z > MaxZ (the runtime's no-return) and variant 'invalid' reports
    depth 0 there; ground inside MaxZ is valid with depth noise;
  * stereo depth noise in the disparity domain: disparity = fB / z + N(0, 0.15 px), fB = 60 px m (7.5 cm baseline,
    800 px mono focal length: ASSUMED OAK-D-Pro-like values, not a calibration); about 1.3 % at 5 m and 2.5 % at
    10 m; a noisy disparity <= 0 is invalid; quantised to 1 mm;
  * RGB: textured dark green ground, bright smooth sky (vertical gradient, N(0, 1.5/255) noise), dark textured
    bark, dark branches and wires, optional textured bright clouds, white smooth objects, a saturated sun blob;
    texture is brightness noise shared by the three channels plus N(0, 1.5/255) sensor noise per channel;
  * source camera 'oakd_like' (1014x760, 69 deg HFOV ASSUMED, as test_camera_runtime_fixes) and 'trained_equal'.
Camera heights: body z 1.0 and 1.5 m (camera 1.03 / 1.53 m). Pitch -10..+10 deg (positive = nose down), roll +-5.
Every negative check has a positive control with a non-zero count. Thresholds are PROVISIONAL contract values.
Nothing here is real-sensor, PX4, ROS-graph, Gazebo or flight evidence.

Outdoor repair r1 (2026-11-13, independent review r1): the virtual ceiling no longer connects the sky (P0-2b); the
far-ground rule is opt-in, default off (P0-1); an edge-enclosure veto keeps sensor-invalid pixels that lie between
near valid returns (P0-2/P0-3: realistic twins with valid vertical silhouette edges are kept, ObjectsWithValidEdges);
wrong-surface ground fits are refused (P1-1: GroundWrongSurface); the ROS node refuses an outdoor contract and the
session holds N accepted frames after a refusal (P2-1); P3 items. Objects that are sensor-invalid with no valid edge
at all cannot be resolved in one frame: FullyInvalidLimitations holds them as unittest.expectedFailure tests that
assert the SAFE outcome (detected), so the limitation stays visible and flips if ever fixed.
"""
import contextlib
import copy
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
import test_camera_runtime_fixes as F
import test_camera_runtime_fixes_r2 as G

CHECKPOINT_DIR = Path('/home/fair/motar_tmp/camk2v3_20261108/train_s1/checkpoint')
MINZ, MAXZ = .4, 20.
STEREO_FB_PX_M = 60.                   # ASSUMED baseline x focal length [px m]
DEPTH_NOISE = .15                      # ASSUMED disparity noise sigma [px]
SKY_RGB_TOP, SKY_RGB_HORIZON = (150., 190., 235.), (205., 222., 240.)
GROUND_RGB, BARK_RGB, DARK_RGB, WHITE_RGB = (70., 92., 48.), (62., 48., 38.), (38., 38., 40.), (246., 246., 246.)


# ------------------------------------------------------------------------------------------------ scene builder
def material(rgb, texture=25., invalid='auto'):
    """invalid: 'auto' (depth 0 only closer than MinZ), 'always' (textureless or thin: depth 0), 'never'."""
    return {'rgb': rgb, 'texture': texture, 'invalid': invalid}


BARK, DARK, WHITE = material(BARK_RGB, 22.), material(DARK_RGB, 6.), material(WHITE_RGB, 0.)


def _vertical_cylinder(o, d, cx, cy, r, z0, z1):
    ox, oy = o[0] - cx, o[1] - cy
    a = d[..., 0] ** 2 + d[..., 1] ** 2
    b = 2 * (ox * d[..., 0] + oy * d[..., 1])
    disc = b * b - 4 * a * (ox * ox + oy * oy - r * r)
    with np.errstate(invalid='ignore', divide='ignore'):
        t = np.where((disc >= 0) & (a > 1e-12), (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
    hz = o[2] + t * d[..., 2]
    return np.where((t > 1e-6) & (hz >= z0) & (hz <= z1), t, np.inf)


def _horizontal_cylinder(o, d, cx, cz, r, y0, y1):
    """Axis along world y at (x = cx, z = cz)."""
    ox, oz = o[0] - cx, o[2] - cz
    a = d[..., 0] ** 2 + d[..., 2] ** 2
    b = 2 * (ox * d[..., 0] + oz * d[..., 2])
    disc = b * b - 4 * a * (ox * ox + oz * oz - r * r)
    with np.errstate(invalid='ignore', divide='ignore'):
        t = np.where((disc >= 0) & (a > 1e-12), (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
    hy = o[1] + t * d[..., 1]
    return np.where((t > 1e-6) & (hy >= y0) & (hy <= y1), t, np.inf)


def _box(o, d, lo, hi):
    with np.errstate(divide='ignore', invalid='ignore'):
        t0 = np.full(d.shape[:-1], -np.inf)
        t1 = np.full(d.shape[:-1], np.inf)
        for k in range(3):
            dk = d[..., k]
            a = np.where(np.abs(dk) > 1e-12, (lo[k] - o[k]) / dk, np.where((o[k] >= lo[k]) & (o[k] <= hi[k]), -np.inf, np.inf))
            b = np.where(np.abs(dk) > 1e-12, (hi[k] - o[k]) / dk, np.where((o[k] >= lo[k]) & (o[k] <= hi[k]), np.inf, -np.inf))
            t0 = np.maximum(t0, np.minimum(a, b))
            t1 = np.minimum(t1, np.maximum(a, b))
    return np.where((t1 >= t0) & (t0 > 1e-6), t0, np.inf)


def outdoor_scene(source, height, roll_deg=0., pitch_deg=0., *, objects=(), ground='textured', far_ground='valid',
                  slope_deg=0., sky='clear', sun=None, seed=0, depth_noise=DEPTH_NOISE):
    """Ray-cast outdoor scene seen by a body-fixed camera (camera FLU = body FLU, mount F.MOUNT).

    ground: 'textured' (valid), 'textureless' (depth 0 everywhere) or None (no ground: training-like).
    far_ground: 'valid' (z > MaxZ kept) or 'invalid' (depth 0 where the ground's optical z exceeds MaxZ).
    slope_deg: the ground rises ahead (+x) by this angle (plane z = x tan(slope)).
    sky: 'clear' (smooth bright gradient), 'cloudy' (plus textured bright cloud patches) or 'none' (training-like:
    a miss is +inf, i.e. no-return, unflagged).
    objects: ('vcyl', cx, cy, r, z0, z1, mat), ('hcyl', cx, cz, r, y0, y1, mat), ('box', lo, hi, mat),
    ('sphere', cx, cy, cz, r, mat).
    sun: (azimuth_deg, elevation_deg, radius_deg): a saturated white disc, sensor-invalid, drawn over everything.
    Returns (frame, kind): kind 0 sky, 1 ground, 2.. objects in order."""
    K = oakd_pro_af_intrinsics() if source == 'oakd_pro_af' else F.source_intrinsics(source)
    sw, sh = K['width'], K['height']
    uu, vv = np.meshgrid(np.arange(sw, dtype=np.float64), np.arange(sh, dtype=np.float64))
    d_cam = np.stack([np.ones_like(uu), -(uu - K['cx']) / K['fx'], -(vv - K['cy']) / K['fy']], axis=-1)
    d_cam /= np.linalg.norm(d_cam, axis=-1, keepdims=True)
    rot = F.body_rotation(roll_deg, pitch_deg)
    o = np.array([0., 0., height]) + rot @ np.array(F.MOUNT)
    d = d_cam @ rot.T
    rng = np.random.default_rng(seed)
    t_best = np.full((sh, sw), np.inf)
    kind = np.zeros((sh, sw), np.int16)
    mats = {}
    if ground is not None:
        s = math.radians(slope_deg)
        n = np.array([-math.sin(s), 0., math.cos(s)])
        nd = d @ n
        with np.errstate(divide='ignore', invalid='ignore'):
            t = np.where(nd < -1e-12, -(o @ n) / nd, np.inf)
        t_best, kind = np.where(t < t_best, t, t_best), np.where(t < t_best, 1, kind)
        mats[1] = material(GROUND_RGB, 28., 'always' if ground == 'textureless' else 'auto')
    for k, obj in enumerate(objects, start=2):
        if obj[0] == 'vcyl':
            t = _vertical_cylinder(o, d, *obj[1:6])
        elif obj[0] == 'hcyl':
            t = _horizontal_cylinder(o, d, *obj[1:6])
        elif obj[0] == 'box':
            t = _box(o, d, np.asarray(obj[1], float), np.asarray(obj[2], float))
        elif obj[0] == 'sphere':
            oc = o - np.array(obj[1:4], float)
            b = 2 * (d @ oc)
            disc = b * b - 4 * (oc @ oc - obj[4] ** 2)
            with np.errstate(invalid='ignore'):
                t = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / 2, np.inf)
            t = np.where(t > 1e-6, t, np.inf)
        else:
            raise ValueError(obj[0])
        hit = t < t_best
        t_best, kind = np.where(hit, t, t_best), np.where(hit, k, kind)
        mats[k] = obj[-1]
    z = np.where(np.isfinite(t_best), t_best * d_cam[..., 0], 0.)
    rgb = np.zeros((sh, sw, 3))
    invalid = np.zeros((sh, sw), bool)
    elevation = np.degrees(np.arcsin(np.clip(d[..., 2], -1, 1)))
    w = np.clip(elevation / 40., 0., 1.)[..., None]
    sky_rgb = (1 - w) * np.array(SKY_RGB_HORIZON) + w * np.array(SKY_RGB_TOP) + rng.normal(0., 1.5, (sh, sw, 3))
    miss = kind == 0
    rgb[miss] = sky_rgb[miss]
    if sky == 'cloudy':
        az = np.degrees(np.arctan2(d[..., 1], d[..., 0]))
        cloud = np.zeros((sh, sw), bool)
        for caz, cel, crad in ((-20., 14., 6.), (10., 22., 8.), (28., 9., 4.), (-5., 35., 7.)):
            cloud |= (az - caz) ** 2 + (elevation - cel) ** 2 <= crad ** 2
        cloud &= miss
        rgb[cloud] = (np.array((228., 228., 232.)) + rng.normal(0., 18., (int(cloud.sum()), 1))
                      + rng.normal(0., 1.5, (int(cloud.sum()), 3)))
    invalid |= miss
    for k, m in mats.items():
        # Texture is brightness variation shared by the three channels (plus sensor noise): independent per-channel
        # noise of this size would create scattered red pixels that the bootstrap red-target detector picks up.
        sel = kind == k
        n = int(sel.sum())
        rgb[sel] = np.array(m['rgb']) + rng.normal(0., m['texture'], (n, 1)) + rng.normal(0., 1.5, (n, 3))
        if m['invalid'] == 'always':
            invalid |= sel
    invalid |= ~miss & (z < MINZ)
    if ground is not None and far_ground == 'invalid':
        invalid |= (kind == 1) & (z > MAXZ)
    if sun is not None:
        az0, el0, rad = (math.radians(x) for x in sun)
        centre = np.array([math.cos(el0) * math.cos(az0), math.cos(el0) * math.sin(az0), math.sin(el0)])
        disc = d @ centre >= math.cos(rad)
        rgb[disc] = 255.
        invalid |= disc
        kind = np.where(disc, -1, kind)
    if depth_noise:
        with np.errstate(divide='ignore'):
            disparity = np.where(z > 0, STEREO_FB_PX_M / np.where(z > 0, z, 1.), 0.) + rng.normal(0., depth_noise, z.shape)
        invalid |= (z > 0) & (disparity <= 0)
        with np.errstate(divide='ignore'):
            z = np.where(disparity > 0, STEREO_FB_PX_M / np.where(disparity > 0, disparity, 1.), 0.)
    z = np.round(z * 1000.) / 1000.
    if sky == 'none':
        # Training semantics: a miss is +inf (no-return, unflagged), never sensor-invalid.
        z = np.where(miss, np.inf, z)
        invalid &= ~miss
    z = np.where(invalid, 0., z).astype(np.float32)
    frame = {'rgb': np.clip(np.round(rgb), 0, 255).astype(np.uint8), 'depth_z_m': z, 'invalid': invalid,
             'rgb_t': 1., 'depth_t': 1., 'intrinsics': K, 'aligned': True, 'rectified': True,
             'mount_flu_m': list(F.MOUNT), 'camera_to_body_rotation': np.eye(3)}
    return frame, kind


def outdoor_state(height, roll_deg=0., pitch_deg=0., *, z_bias=0., z_valid=True, epv=.1, dead_reckoning=False,
                  attitude_error=(0., 0.), vertical=True, t=1.):
    """F.scene_state plus PX4's vertical fields; attitude_error (roll, pitch) deg is an estimator error."""
    state = F.scene_state(height + z_bias, roll_deg + attitude_error[0], pitch_deg + attitude_error[1], t=t)
    if vertical:
        state.update(z_valid=z_valid, epv=epv, dead_reckoning=dead_reckoning)
    return state


def outdoor_contract(sky=True, far_ground=False, **ground_estimate):
    """Geometry subset of an outdoor-profile contract (no checkpoint): F.contract plus the outdoor blocks.
    far_ground=True opts in to the far-ground rule (sky.far_ground_rule, default off since outdoor repair r1)."""
    c = F.contract(ground=True, ceiling=C.OUTDOOR_CEILING_HEIGHT_M, asserted=False)
    c['ceiling']['physical'] = False
    c.update(profile='outdoor', sky=copy.deepcopy(C.SKY_OUTDOOR_DEFAULT),
             ground_estimate=copy.deepcopy(C.GROUND_ESTIMATE_OUTDOOR_DEFAULT))
    c['sky']['enabled'] = bool(sky)
    c['sky']['far_ground_rule'] = C.provisional(bool(far_ground))
    for key, value in ground_estimate.items():
        c['ground_estimate'][key] = C.provisional(value)
    return c


def indoor_contract():
    """The shipped default (floor filter on, plane rule off, asserted CPU fixture): the runtime 'before' the fix."""
    return F.contract(ground=True)


def run(frame, state, contract):
    """Production preprocess (geometry runtime, production blob_near, no-goal perception stand-in).
    Returns a summary: refused reason or column minima, near columns and counts."""
    try:
        _, depth, counts, projection = G.geometry_runtime(contract).preprocess(frame, state)
    except R.FrameRefused as exc:
        return {'refused': exc.reason, 'report': exc.report}
    cols = depth[0].amin(dim=0).numpy()
    return {'refused': None, 'cols': cols, 'near_cols': int((cols < 1.).sum()), 'min': float(cols.min()),
            'counts': counts, 'projection': projection['projection'], 'depth': depth[0].numpy()}



TREE_LINE = ('box', (15., -50., 0.), (16., 50., 5.), material((40., 70., 35.), 30.))
HEIGHTS, PITCHES = (1.0, 1.5), (-10., 0., 10.)


def cylinder_cols(cx, cy, r, shrink=2):
    """Trained columns inside the silhouette of a vertical cylinder seen from the trained camera (x 0.1, y 0)."""
    dx, dy = cx - F.MOUNT[0], cy
    centre, half = math.atan2(dy, dx), math.asin(r / math.hypot(dx, dy))
    u = np.arange(160)
    az = np.arctan(-(u - 79.5) / F.TRAINED['fx'])
    cols = np.flatnonzero(np.abs(az - centre) <= half)
    return cols[shrink:-shrink] if shrink else cols


def face_cols(y0, y1, x, shrink=2):
    """Trained columns whose level ray meets y in [y0, y1] at world x (a front face)."""
    u = np.arange(160)
    y = -(u - 79.5) / F.TRAINED['fx'] * (x - F.MOUNT[0])
    cols = np.flatnonzero((y >= y0) & (y <= y1))
    return cols[shrink:-shrink] if shrink else cols


def camera_height(height):
    return height + F.MOUNT[2]


def edge_ranges(rgb_ranges, invalid, no_return, after_no_return, roof):
    """The production edge ranges of _preprocess_outdoor: valid returns before the floor/ceiling filters, floor
    returns excluded, +inf elsewhere."""
    floor = after_no_return[0] & ~no_return[0] & ~roof
    return np.where(~(invalid | no_return)[0] & ~floor, rgb_ranges[0].astype(np.float64), np.inf)


def sky_inputs(frame, state, contract):
    """Production outdoor preprocess up to the sky rule (no goal pixels): returns the apply_sky_rule arguments."""
    base, _ = R.split_state(state, contract)
    rgb, ranges, si, nr, cos, _, unimaged = R.project_rgbd(frame, base, contract, unimaged=True)
    plane = R.estimate_ground(frame, state, contract)['plane']
    _, si2, nr2, _, roof = R.remove_floor_and_ceiling_plane(ranges, si, nr, cos, plane, contract)
    return rgb, si2, nr2, unimaged, plane, edge_ranges(ranges, si, nr, nr2, roof), (si, nr)


WHITE_INV, LGREY_INV = material(WHITE_RGB, 0., 'always'), material((200., 205., 210.), 0., 'always')
DARK_INV = material(DARK_RGB, 6., 'always')


def valid_like(mat):
    return material(mat['rgb'], mat['texture'], 'never')


def edge_strips(x, y_lo, y_hi, z0, z1, mat, w=.03):
    """Valid vertical silhouette edges (width w) at both sides of a face at x: what the OAK-D's horizontal stereo
    baseline usually matches even where the face itself is textureless and invalid (ASSUMED, synthetic)."""
    vm = valid_like(mat)
    return [('box', (x - .002, y_lo, z0), (x + .05, y_lo + w, z1), vm), ('box', (x - .002, y_hi - w, z0), (x + .05, y_hi, z1), vm)]


def tree(top):
    return ('box', (15., -50., 0.), (16., 50., top), material((40., 70., 35.), 30.))


def kept_columns(out, cols):
    """Object columns whose minimum lies inside the 10 m obstacle range (near or ranged)."""
    return int((out['cols'][cols] < 10.).sum())


# ------------------------------------------------------------------------------------------- contract (A)
class OutdoorContractProfile(unittest.TestCase):
    def value(self):
        value = C._metadata(1, ROOT)
        value['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        return value

    def outdoor(self):
        value = self.value()
        value.update(C.outdoor_blocks())
        return value

    def test_defaults_and_existing_contracts_are_unchanged(self):
        value = self.value()
        self.assertEqual(C.validate_contract(value, verify_files=False), [])            # positive control
        for key in C.OUTDOOR_KEYS:
            self.assertNotIn(key, value)
        self.assertNotIn('physical', C.CEILING_DEFAULT)
        self.assertNotIn('physical', value['ceiling'])
        self.assertEqual(C.GROUND_DEFAULT, {'enabled': True, 'margin_m': .10, 'height_reference': 'floor',
                                            'invalid_plane_rule': False})
        self.assertIn('estimator_height', C.height_reference_problem(value))           # indoor still needs it
        self.assertFalse(C.is_outdoor(value))
        self.assertFalse(R._is_outdoor(F.contract(ground=True)))

    def test_outdoor_profile_validates_and_every_threshold_is_provisional(self):
        value = self.outdoor()
        self.assertEqual(C.validate_contract(value, verify_files=False), [])
        self.assertTrue(C.is_outdoor(value))
        self.assertIsNone(C.height_reference_problem(value))      # per-frame camera verification replaces it
        self.assertEqual(value['ceiling'], dict(C.CEILING_DEFAULT, enabled=True, height_m=3.0, physical=False))
        self.assertEqual(value['ground'], C.GROUND_DEFAULT)
        self.assertIsNone(value['estimator_height'])
        provisional = [k for block in ('sky', 'ground_estimate') for k, v in value[block].items() if isinstance(v, dict)]
        self.assertEqual(len(provisional), len(C.SKY_BOUNDS) + len(C.SKY_FLAGS) + len(C.GROUND_ESTIMATE_BOUNDS))
        # Outdoor repair r1: the far-ground rule is an explicit PROVISIONAL opt-in, default off; dead reckoning refused.
        self.assertEqual(value['sky']['far_ground_rule'], {'value': False, 'status': 'PROVISIONAL'})
        self.assertIs(value['ground_estimate']['refuse_dead_reckoning'], True)
        for key in ('edge_veto_range_m', 'edge_veto_span_deg', 'edge_veto_range_tolerance_m'):
            self.assertEqual(value['sky'][key]['status'], 'PROVISIONAL')
        for key in ('rival_min_share', 'min_half_share', 'min_range_span_m', 'planarity_max_angle_deg'):
            self.assertEqual(value['ground_estimate'][key]['status'], 'PROVISIONAL')
        for block in ('sky', 'ground_estimate'):
            for key, item in value[block].items():
                if isinstance(item, dict):
                    self.assertEqual(item['status'], 'PROVISIONAL', (block, key))
        self.assertEqual(value['ground_estimate']['estimator_fields'], ['z_valid', 'epv', 'dead_reckoning'])

    def test_malformed_outdoor_profiles_are_refused(self):
        bad = []
        def change(fn, label):
            value = self.outdoor()
            fn(value)
            bad.append((label, value))
        change(lambda v: v.pop('sky'), 'partial keys')
        change(lambda v: v.update(profile='indoor'), 'profile name')
        change(lambda v: v['sky']['luma_min'].update(status='CALIBRATED'), 'status')
        change(lambda v: v['sky'].update(luma_min=.5), 'bare value')
        change(lambda v: v['sky']['gradient_max'].update(value=0.), 'bound')
        change(lambda v: v['ground_estimate']['min_inliers'].update(value=300.), 'integer')
        change(lambda v: v['ground_estimate']['tilt_max_deg'].update(value=float('nan')), 'nan')
        change(lambda v: v['ground_estimate'].update(enabled=False), 'estimate off')
        change(lambda v: v['ground_estimate'].update(estimator_fields=['z_valid']), 'fields')
        change(lambda v: v['ground'].update(enabled=False), 'floor off')
        change(lambda v: v['ceiling'].update(physical=True), 'physical ceiling')
        change(lambda v: v['ceiling'].update(invalid_plane_rule=True), 'virtual ceiling explains invalid')
        change(lambda v: v['ground'].update(invalid_plane_rule=True), 'floor plane rule')
        change(lambda v: v.update(estimator_height=dict(F.FIXTURE_ASSERTION)), 'operator assertion')
        change(lambda v: v['sky'].update(connectivity='any'), 'connectivity')
        change(lambda v: v['sky'].update(extra=1), 'extra key')
        # Outdoor repair r1 (review P3-3)
        change(lambda v: v['ground_estimate'].update(refuse_dead_reckoning=False), 'dead reckoning allowed')
        change(lambda v: v['ground_estimate']['tilt_max_deg'].update(value=5.5), 'tilt above 5 deg')
        change(lambda v: v['ground_estimate']['height_tolerance_m'].update(value=.6), 'height tolerance above 0.5 m')
        change(lambda v: v['ceiling'].update(height_m=1.2), 'virtual ceiling 1.2 m')
        change(lambda v: v['ceiling'].update(height_m=6.5), 'virtual ceiling 6.5 m')
        change(lambda v: v['sky'].update(far_ground_rule=True), 'far-ground rule bare value')
        change(lambda v: v['sky']['far_ground_rule'].update(value=1), 'far-ground rule not a boolean')
        change(lambda v: v['sky']['far_ground_rule'].update(status='CALIBRATED'), 'far-ground rule status')
        change(lambda v: v['sky']['edge_veto_range_m'].update(value=.3), 'edge veto range below MinZ')
        change(lambda v: v['ground_estimate']['planarity_max_angle_deg'].update(value=0.), 'planarity bound')
        for label, value in bad:
            with self.subTest(label=label):
                self.assertTrue(C.validate_contract(value, verify_files=False), label)
                self.assertIsNotNone(C.height_reference_problem(value), label)  # a malformed profile never exempts
        self.assertEqual(len(bad), 26)
        # Positive controls at the new bounds: tilt 5 deg, tolerance 0.5 m, ceiling 2.5 and 6.0 m, far-ground opt-in.
        for fn in (lambda v: v['ground_estimate']['tilt_max_deg'].update(value=5.),
                   lambda v: v['ground_estimate']['height_tolerance_m'].update(value=.5),
                   lambda v: v['ceiling'].update(height_m=2.5), lambda v: v['ceiling'].update(height_m=6.),
                   lambda v: v['sky']['far_ground_rule'].update(value=True)):
            value = self.outdoor()
            fn(value)
            self.assertEqual(C.validate_contract(value, verify_files=False), [])
            self.assertIsNone(C.height_reference_problem(value))
        # ceiling.physical exists for the outdoor profile only (review P3-3): an indoor contract carrying it, true or
        # false, is refused as it was before the outdoor profile existed.
        for physical in (False, True, 'no'):
            value = self.value()
            value['ceiling'].update(enabled=True, height_m=3., physical=physical)
            self.assertTrue(any('physical' in p for p in C.validate_contract(value, verify_files=False)), physical)
        value = self.value()
        value['ceiling'].update(enabled=True, height_m=3.)
        self.assertEqual(C.validate_contract(value, verify_files=False), [])            # positive control

    def test_cli_profile_outdoor(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.object(C, 'build_contract', return_value=self.outdoor()) as build, \
                    contextlib.redirect_stdout(io.StringIO()) as stdout:
                C.main(['--output', str(Path(directory) / 'a.json'), '--seed', '1', '--profile', 'outdoor'])
            blocks = build.call_args.kwargs['outdoor']
            self.assertEqual(blocks, C.outdoor_blocks())
            self.assertNotIn('estimator_height', build.call_args.kwargs)
            summary = json.loads(stdout.getvalue())
            self.assertEqual((summary['profile'], summary['height_filters'], summary['thresholds']),
                             ('outdoor', 'CAMERA_VERIFIED_PER_FRAME', 'PROVISIONAL'))
            with mock.patch.object(C, 'build_contract', return_value=self.outdoor()) as build, \
                    contextlib.redirect_stdout(io.StringIO()):
                C.main(['--output', str(Path(directory) / 'b.json'), '--seed', '1', '--profile', 'outdoor',
                        '--ceiling-height-m', '2.8', '--ground-margin-m', '.15'])
            self.assertEqual(build.call_args.kwargs['outdoor']['ceiling']['height_m'], 2.8)
            self.assertEqual(build.call_args.kwargs['outdoor']['ground']['margin_m'], .15)
            for extra in (['--no-ground-removal'], ['--ground-invalid-plane-rule'], ['--ceiling-invalid-plane-rule'],
                          ['--height-reference-evidence', str(Path(directory) / 'x.txt')]):
                with self.subTest(extra=extra), self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
                    C.main(['--output', str(Path(directory) / 'c.json'), '--seed', '1', '--profile', 'outdoor'] + extra)
            # Without --profile the CLI builds exactly the indoor contract as before (control).
            with mock.patch.object(C, 'build_contract', return_value=self.value()) as build, \
                    contextlib.redirect_stdout(io.StringIO()) as stdout:
                C.main(['--output', str(Path(directory) / 'd.json'), '--seed', '1'])
            self.assertNotIn('outdoor', build.call_args.kwargs)
            self.assertNotIn('profile', json.loads(stdout.getvalue()))

    def test_real_outdoor_contract_binds_files_and_the_ros_node_refuses_it_until_px4_fields_arrive(self):
        """Outdoor repair r1 (review P2-1): the ROS adapter/gateway do not forward z_valid/epv/dead_reckoning, so
        every outdoor frame would be refused; the node refuses the contract at load (UNCONFIGURED, named reason)
        instead of starting a session that holds forever. Positive control: the indoor contract still loads."""
        if not CHECKPOINT_DIR.is_dir():
            self.skipTest('local trained camera checkpoint not distributed in the public repository')
        import hashlib
        import camera_ros2_node as node
        contract = C.build_contract(1, outdoor=C.outdoor_blocks())
        self.assertEqual(C.validate_contract(contract), [])
        with self.assertRaisesRegex(ValueError, 'own ground and ceiling'):
            C.build_contract(1, ground=dict(C.GROUND_DEFAULT), outdoor=C.outdoor_blocks())
        with self.assertRaisesRegex(ValueError, 'virtual ceiling height_m'):
            C.build_contract(1, outdoor=C.outdoor_blocks(1.2))
        self.assertFalse(node.ROS_PATH_FORWARDS_VERTICAL_FIELDS)
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            evidence = tmp / 'estimator_height_fixture.txt'
            evidence.write_text('CPU test fixture, not evidence')
            results = {}
            for name, value in (('outdoor', contract),
                                ('indoor', C.build_contract(1, estimator_height=C.estimator_height_assertion(evidence)))):
                path = tmp / ('%s.json' % name)
                path.write_text(json.dumps(value))
                config = {'format': 'motar-ros2-node-v1', 'binding': F.ros_binding(tmp / 'binding_fixture.txt'),
                          'outputs': {'velocity_topic': '/motar/policy/velocity_enu',
                                      'diagnostic_topic': '/motar/policy/diagnostics'},
                          'policy_contract_path': str(path),
                          'policy_contract_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
                config_path = tmp / ('%s_node.json' % name)
                config_path.write_text(json.dumps(config))
                try:
                    results[name] = node.load_node_config(config_path, ROOT)[1]
                except ValueError as exc:
                    results[name] = str(exc)
                with contextlib.redirect_stdout(io.StringIO()) as stdout:
                    code = node.main(config_path=config_path, source_root=ROOT, check_only=True)
                results[name + '_check'] = (code, json.loads(stdout.getvalue())['status'])
        self.assertIsInstance(results['outdoor'], str)
        self.assertTrue(results['outdoor'].startswith('UNCONFIGURED: ' + node.OUTDOOR_NODE_REFUSAL), results['outdoor'])
        self.assertEqual(results['outdoor_check'], (2, 'NOT_CONFIGURED'))
        self.assertIsInstance(results['indoor'], dict)                                  # positive control
        self.assertEqual(results['indoor_check'], (0, 'CONFIG_VALID'))


# ------------------------------------------------------------------------------- sky rule (C), cases 1 and 2
class SkyRule(unittest.TestCase):
    def test_sky_above_a_tree_line_is_no_return_and_the_blocker_reproduces_without_the_rule(self):
        cases = sky = 0
        for height in HEIGHTS:
            for pitch in PITCHES:
                for roll in (0., 5.):
                    f, _ = outdoor_scene('oakd_like', height, roll, pitch, objects=[TREE_LINE])
                    after = run(f, outdoor_state(height, roll, pitch), outdoor_contract())
                    off = run(f, outdoor_state(height, roll, pitch), outdoor_contract(sky=False))
                    before = run(f, outdoor_state(height, roll, pitch, vertical=False), indoor_contract())
                    label = (height, pitch, roll)
                    with self.subTest(label=label):
                        self.assertIsNone(after['refused'], label)
                        self.assertEqual(after['near_cols'], 0, label)
                        self.assertEqual(after['counts']['near_pixels'], 0, label)
                        self.assertGreater(after['min'], 10., label)                 # beyond the obstacle range
                        self.assertGreater(after['counts']['outdoor']['sky']['sky_pixels'], 0, label)
                        if roll == 0.:                                               # the blocker: a wall everywhere
                            self.assertEqual(off['near_cols'], 160, label)
                            self.assertEqual(before['near_cols'], 160, label)
                        else:
                            self.assertGreater(off['near_cols'], 100, label)
                            self.assertGreater(before['near_cols'], 100, label)
                        self.assertLessEqual(off['min'], .45)
                    sky += after['counts']['outdoor']['sky']['sky_pixels']
                    cases += 1
        self.assertEqual(cases, 12)
        self.assertGreater(sky, 0)
        # trained_equal source, level (inverse-nearest path)
        f, _ = outdoor_scene('trained_equal', 1., objects=[TREE_LINE])
        self.assertEqual(run(f, outdoor_state(1.), outdoor_contract())['near_cols'], 0)
        self.assertEqual(run(f, outdoor_state(1., vertical=False), indoor_contract())['near_cols'], 160)

    def test_open_field_to_the_horizon_both_far_ground_variants(self):
        """Outdoor repair r1 (review P0-1): the far-ground rule is OFF by default. Far ground that the OAK-D still
        ranges (z > MaxZ, variant 'valid') is no-return as before; far ground reported as depth 0 (variant
        'invalid') now stays sensor-invalid and becomes a near band on every bearing (a measured phantom, the safe
        direction). Opting in to the rule frees it again (control)."""
        far = 0
        for far_ground in ('valid', 'invalid'):
            for height in HEIGHTS:
                for pitch in PITCHES:
                    f, _ = outdoor_scene('oakd_like', height, 0., pitch, far_ground=far_ground)
                    after = run(f, outdoor_state(height, 0., pitch), outdoor_contract())
                    opted = run(f, outdoor_state(height, 0., pitch), outdoor_contract(far_ground=True))
                    before = run(f, outdoor_state(height, 0., pitch, vertical=False), indoor_contract())
                    label = (far_ground, height, pitch)
                    with self.subTest(label=label):
                        self.assertIsNone(after['refused'])
                        self.assertEqual(after['counts']['outdoor']['sky']['far_ground_pixels'], 0, label)
                        self.assertFalse(after['counts']['outdoor']['sky']['far_ground_rule'])
                        self.assertEqual(before['near_cols'], 160, label)
                        self.assertEqual(opted['near_cols'], 0, label)                  # opt-in control
                        self.assertGreater(opted['min'], 10., label)
                        if far_ground == 'valid':
                            self.assertEqual(after['near_cols'], 0, label)
                            self.assertGreater(after['min'], 10., label)
                        else:
                            self.assertEqual(after['near_cols'], 160, label)            # phantom band
                            self.assertLessEqual(after['min'], .45, label)
                            self.assertGreater(opted['counts']['outdoor']['sky']['far_ground_pixels'], 0, label)
                    far += opted['counts']['outdoor']['sky']['far_ground_pixels']
        self.assertGreater(far, 0)

    def test_far_ground_band_width_follows_atan_h_over_maxz(self):
        # Level plane, camera heights 1.0/1.5 m: rows below the horizon whose ray meets the ground beyond MaxZ. Only
        # with the opt-in rule; by default (outdoor repair r1) no row below the horizon is freed.
        c, default = outdoor_contract(far_ground=True), outdoor_contract()
        v = (np.arange(90) - 44.5) / F.TRAINED['fy']
        no_edges = np.full((90, 160), np.inf)
        for h, rows in ((1.0, 4), (1.5, 7)):
            plane = {'normal': [0., 0., 1.], 'height_m': h}
            si = np.ones((1, 90, 160), bool)
            rgb = np.zeros((1, 3, 90, 160), np.float32)           # dark: rule 1 never applies
            out_si, out_nr, info = R.apply_sky_rule(rgb, si, np.zeros_like(si), np.zeros((90, 160), bool), plane, c,
                                                    no_edges)
            band = np.flatnonzero(out_nr[0].all(axis=1))
            with self.subTest(h=h):
                self.assertEqual(len(band), rows)
                self.assertEqual(band[0], 45)
                self.assertTrue(np.all(h / v[band] > MAXZ))
                self.assertLessEqual(h / v[band[-1] + 1], MAXZ)
                self.assertAlmostEqual(math.degrees(math.atan(h / MAXZ)), {1.0: 2.862, 1.5: 4.289}[h], places=3)
                self.assertEqual(info['sky_pixels'], 0)
                self.assertFalse(out_nr[0, :45].any())            # above the horizon nothing without sky-like RGB
                off_si, off_nr, off_info = R.apply_sky_rule(rgb, si, np.zeros_like(si), np.zeros((90, 160), bool),
                                                            plane, default, no_edges)
                self.assertFalse(off_nr.any())
                self.assertEqual(off_info['far_ground_pixels'], 0)

    def test_sky_rule_never_alters_valid_depth_and_needs_the_verified_plane(self):
        # Outdoor repair r2: the trunk stands beyond edge_veto_range_m (6 m), so the sky beside it stays a positive control.
        f, _ = outdoor_scene('oakd_like', 1., 0., 3., objects=[TREE_LINE, ('vcyl', 6., 0., .2, 0., 5., BARK)])
        state = outdoor_state(1., 0., 3.)
        c = outdoor_contract()
        rgb, si, nr, unimaged, plane, edge, _ = sky_inputs(f, state, c)
        out_si, out_nr, info = R.apply_sky_rule(rgb, si, nr, unimaged, plane, c, edge)
        valid = ~(si | nr)
        self.assertGreater(int(valid.sum()), 0)
        self.assertFalse((out_si | out_nr)[valid].any())          # every valid pixel stays valid
        self.assertFalse((out_si & ~si).any())                    # nothing becomes sensor-invalid
        self.assertGreater(info['sky_pixels'], 0)
        # A refused frame never reaches the rule: the runtime raises FrameRefused before it.
        with self.assertRaises(R.FrameRefused):
            G.geometry_runtime(c).preprocess(f, outdoor_state(1., 0., 3., vertical=False))
        # The edge ranges are required (no silent fail-open default) and must be the trained grid.
        with self.assertRaises(TypeError):
            R.apply_sky_rule(rgb, si, nr, unimaged, plane, c)
        with self.assertRaises(R.InputError):
            R.apply_sky_rule(rgb, si, nr, unimaged, plane, c, edge[:-1])

    def test_virtual_ceiling_never_connects_the_sky(self):
        """Outdoor repair r1 (review P0-2b, P3-5): the virtual ceiling drops valid returns above it and touches
        nothing else. Sky enclosed below a removed canopy is no longer freed through the removed pixels: it stays
        sensor-invalid and becomes a near band on every bearing (the measured consequence, a phantom). A white
        panel beside a 5 m tree line whose top the ceiling removes is kept (the reviewer's 0/66 case)."""
        c = outdoor_contract()
        canopy = ('box', (1., -5., 3.2), (6., 5., 3.3), material((35., 60., 30.), 25., 'never'))
        f, _ = outdoor_scene('oakd_like', 1., objects=[canopy, TREE_LINE])
        out = run(f, outdoor_state(1.), c)
        self.assertGreater(out['counts']['ceiling_removed_pixels'], 0)             # the canopy return is removed
        self.assertEqual(out['near_cols'], 160)                                    # the sky below it: phantom
        self.assertEqual(out['counts']['outdoor']['sky']['sky_pixels'], 0)
        self.assertGreater(out['counts']['outdoor']['sky']['sky_candidate_pixels'], 0)
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
        self.assertEqual(run(f, outdoor_state(1.), c)['near_cols'], 0)              # control: open sky is freed
        cols = face_cols(-.4, .4, 1.5)
        cam = camera_height(1.)
        for mat in (WHITE_INV, LGREY_INV):
            panel = ('box', (1.5, -.5, cam - .10), (1.55, .5, cam + .45), mat)
            f, _ = outdoor_scene('oakd_like', 1., objects=[panel, tree(5.)])
            out = run(f, outdoor_state(1.), c)
            with self.subTest(rgb=mat['rgb']):
                self.assertGreater(out['counts']['ceiling_removed_pixels'], 0)
                self.assertEqual(kept_columns(out, cols), len(cols))
        self.assertEqual(C.SKY_CONNECTIVITY, 'top_border_4_through_sky_or_unimaged')

    def test_unimaged_connector_frees_no_near_object_with_a_valid_edge(self):
        """The sky flood also passes through pixels OUTSIDE the source camera's field of view (the top band a
        nose-down camera does not image); without it the open sky becomes a near wall on every bearing at 5-10 deg
        nose-down. Objects with valid vertical edges touching that band are kept with and without the connector
        (identical columns); the dark twin controls the geometry. A fully sensor-invalid sky-like object touching
        the band is the FullyInvalidLimitations class (the same as one touching the image top at pitch 0)."""
        c = outdoor_contract()
        cam = camera_height(1.)
        cols = face_cols(-.4, .4, 1.5)
        # The panel's face in the level trained image (a frontal rectangle, one pixel inside its border): rows with
        # height (camera +0.05..+0.55 m) / (1.5 m - camera x), columns with |y| < 0.47 m (inside the valid edges).
        u = (np.arange(160) - 79.5) / F.TRAINED['fx']
        v = (np.arange(90) - 44.5) / F.TRAINED['fy']
        depth = 1.5 - F.MOUNT[0]
        rows = np.flatnonzero((-v * depth > .05) & (-v * depth < .55))[1:-1]
        face = np.ix_(rows, np.flatnonzero(np.abs(u) * depth < .47)[1:-1])
        checked = 0
        for pitch in (5., 10., 15.):
            if pitch < 15.:            # at 15 deg nose-down the sky above a 5 m tree line lies wholly outside the FOV
                f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=[TREE_LINE])
                rgb, si, nr, unimaged, plane, edge, _ = sky_inputs(f, outdoor_state(1., 0., pitch), c)
                self.assertGreater(int(unimaged.sum()), 0)
                on = R.apply_sky_rule(rgb, si, nr, unimaged, plane, c, edge)
                off = R.apply_sky_rule(rgb, si, nr, np.zeros_like(unimaged), plane, c, edge)
                self.assertGreater(on[2]['sky_pixels'], 0)                          # control: the connector matters
                self.assertEqual(off[2]['sky_pixels'], 0)
            for mat in (WHITE_INV, LGREY_INV, DARK_INV):
                objs = edge_strips(1.5, -.5, .5, cam + .05, cam + .55, mat) + [
                    ('box', (1.5, -.5, cam + .05), (1.55, .5, cam + .55), mat), TREE_LINE]
                f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=objs)
                state = outdoor_state(1., 0., pitch)
                rgb, si, nr, unimaged, plane, edge, _ = sky_inputs(f, state, c)
                touching = (unimaged[:-1][:, cols] & si[0, 1:][:, cols]).any()      # the twin meets the band
                on_si, _, _ = R.apply_sky_rule(rgb, si, nr, unimaged, plane, c, edge)
                off_si, _, _ = R.apply_sky_rule(rgb, si, nr, np.zeros_like(unimaged), plane, c, edge)
                with self.subTest(pitch=pitch, rgb=mat['rgb']):
                    invalid_face = si[0][face] & ~unimaged[face]
                    self.assertGreater(int(invalid_face.sum()), 1000)
                    self.assertTrue(on_si[0][face][invalid_face].all())           # every invalid face pixel kept
                    self.assertTrue(off_si[0][face][invalid_face].all())
                    self.assertEqual(kept_columns(run(f, state, c), cols), len(cols))
                checked += bool(touching)
        self.assertGreater(checked, 0)                                             # some twins touch the band


# ------------------------------------------------------------------ obstacles kept (C), cases 3, 4, 6, 7
class NearObstaclesStayDetected(unittest.TestCase):
    def test_tree_trunk_close_is_near_and_far_is_ranged(self):
        for height in HEIGHTS:
            for pitch in PITCHES:
                f, _ = outdoor_scene('oakd_like', height, 0., pitch, objects=[('vcyl', .55, 0., .15, 0., 5., BARK), TREE_LINE])
                close = run(f, outdoor_state(height, 0., pitch), outdoor_contract())
                cols = cylinder_cols(.55, 0., .15)
                f, _ = outdoor_scene('oakd_like', height, 0., pitch, objects=[('vcyl', 1.75, 0., .15, 0., 5., BARK), TREE_LINE])
                far = run(f, outdoor_state(height, 0., pitch), outdoor_contract())
                fcols = cylinder_cols(1.75, 0., .15)
                with self.subTest(height=height, pitch=pitch):
                    self.assertGreater(len(cols), 60)
                    self.assertTrue(np.all(close['cols'][cols] < .5), close['cols'][cols].max())
                    self.assertGreater(close['counts']['near_pixels'], 0)
                    # Control: the rest of the view is open. The sky pixels touching the trunk are not smooth
                    # against it and may stay sensor-invalid and join its blob (at most one column per side).
                    sil = cylinder_cols(.55, 0., .15, shrink=0)
                    others = np.setdiff1d(np.arange(160), np.arange(sil.min() - 1, sil.max() + 2))
                    self.assertTrue(np.all(close['cols'][others] > 10.))
                    self.assertTrue(np.all(np.abs(far['cols'][fcols] - 1.5) <= .1), far['cols'][fcols])
                    # Outdoor repair r2 (review r2 P0-N1): the sky in the rows of the 1.5 m trunk (a near valid return
                    # within edge_veto_range_m) is enclosed by the one-edge rule and stays near: a measured phantom
                    # beside it (OneEdgePhantomCost), never in the trunk's own columns.
                    self.assertGreater(far['counts']['near_pixels'], 0)
                    self.assertGreater(far['near_cols'], 100)
                    self.assertFalse(np.any(far['cols'][fcols] < 1.))

    def test_low_branch_above_camera_height(self):
        for height in HEIGHTS:
            for pitch in PITCHES:
                top = camera_height(height)
                f, kind = outdoor_scene('oakd_like', height, 0., pitch, objects=[('hcyl', .45, top + .08, .05, -1., 1., DARK), TREE_LINE])
                close = run(f, outdoor_state(height, 0., pitch), outdoor_contract())
                f, _ = outdoor_scene('oakd_like', height, 0., pitch, objects=[('hcyl', 1.15, top + .15, .05, -1., 1., DARK), TREE_LINE])
                ranged = run(f, outdoor_state(height, 0., pitch), outdoor_contract())
                with self.subTest(height=height, pitch=pitch):
                    self.assertGreater(int((kind == 2).sum()), 0)
                    self.assertEqual(close['near_cols'], 160)
                    self.assertLessEqual(close['min'], .45)
                    self.assertTrue(np.all(np.abs(ranged['cols'] - 1.05) <= .2), (ranged['cols'].min(), ranged['cols'].max()))
                    self.assertEqual(ranged['counts']['near_pixels'], 0)

    def test_overhead_canopy_kept_below_the_virtual_ceiling_and_removed_above(self):
        for z in (2.0, 2.5):
            f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1., -5., z), (6., 5., z + .1), material((35., 60., 30.), 25., 'never')), TREE_LINE])
            valid = run(f, outdoor_state(1.), outdoor_contract())
            f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1., -5., z), (6., 5., z + .1), material(DARK_RGB, 6., 'always')), TREE_LINE])
            dark = run(f, outdoor_state(1.), outdoor_contract())
            with self.subTest(z=z):
                self.assertLess(valid['min'], 4.)                    # the canopy underside is an obstacle return
                self.assertGreater(valid['min'], 1.)
                self.assertEqual(dark['near_cols'], 160)             # invalid dark canopy stays near
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1., -5., 3.2), (6., 5., 3.3), material((35., 60., 30.), 25., 'never')), TREE_LINE])
        high = run(f, outdoor_state(1.), outdoor_contract())
        self.assertGreater(high['counts']['ceiling_removed_pixels'], 0)
        # Outdoor repair r1: the canopy's own return is removed, but the sky enclosed below it is no longer freed
        # through the removed pixels (SkyRule.test_virtual_ceiling_never_connects_the_sky): a phantom near band.
        self.assertEqual(high['near_cols'], 160)
        # Measured limitation: an INVALID canopy above the virtual ceiling stays near (never explained).
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1., -5., 3.2), (6., 5., 3.3), material(DARK_RGB, 6., 'always')), TREE_LINE])
        self.assertEqual(run(f, outdoor_state(1.), outdoor_contract())['near_cols'], 160)

    def test_bright_smooth_post_on_the_ground_is_detected_by_its_lower_part(self):
        cols = face_cols(-.1, .1, .4)
        for pitch in PITCHES:
            f, kind = outdoor_scene('oakd_like', 1., 0., pitch, objects=[('box', (.4, -.1, 0.), (.5, .1, 3.), WHITE), TREE_LINE])
            out = run(f, outdoor_state(1., 0., pitch), outdoor_contract())
            with self.subTest(pitch=pitch):
                self.assertGreater(len(cols), 70)
                self.assertTrue(np.all(out['cols'][cols] < .5), out['cols'][cols].max())
                lower = out['depth'][45:, cols]
                self.assertTrue((lower < .5).any())                # the near pixels lie below the horizon
        # A wall filling the whole lower view hides the ground: the frame is refused (hold), never passed.
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (.4, -2., 0.), (.5, 2., 3.), WHITE), TREE_LINE])
        self.assertEqual(run(f, outdoor_state(1.), outdoor_contract())['refused'], 'ground_insufficient_points')


# ---------------------------------------------- outdoor repair r1: realistic twins with valid vertical edges (P0-2/P0-3)
def rail_twin(mat, D, posts=(-1., 0., 1.), ends=True, h=1.):
    """A rail 3 m wide, 0.10 m tall, centred on camera height, against an OPEN horizon (no tree line), with valid
    wooden posts in front of it and (ends=True) valid vertical end edges."""
    cam = camera_height(h)
    wood = material((120., 90., 60.), 30., 'never')
    objs = [('box', (D - .01, py - .04, 0.), (D + .1, py + .04, cam + .15), wood) for py in posts]
    if ends:
        objs += edge_strips(D, -1.5, 1.5, cam - .05, cam + .05, mat)
    return objs + [('box', (D, -1.5, cam - .05), (D + .1, 1.5, cam + .05), mat)]


def gazebo_twin(mat, z=2.0):
    """A 3 x 3 m canopy at x 1.5..4.5 whose underside (mat) is sensor-invalid, with valid 0.25 m side/front/back
    skirts and four valid posts: its visible silhouette edges are valid."""
    sk = valid_like(mat)
    post = material((90., 90., 95.), 20., 'never')
    x0, x1, y = 1.5, 4.5, 1.5
    return ([('box', (x0, -y, z - .25), (x1, -y + .03, z + .05), sk), ('box', (x0, y - .03, z - .25), (x1, y, z + .05), sk),
             ('box', (x1 - .03, -y, z - .25), (x1, y, z + .05), sk), ('box', (x0, -y, z - .25), (x0 + .03, y, z + .05), sk)]
            + [('vcyl', xx, yy, .04, 0., z - .25, post) for xx in (x0 + .05, x1 - .05) for yy in (-y + .05, y - .05)]
            + [('box', (x0, -y, z), (x1, y, z + .05), mat), TREE_LINE])


class ObjectsWithValidEdges(unittest.TestCase):
    """Outdoor repair r1 (review P0-2/P0-3): realistic twins of the reviewer's panel, raised box, canopy and rail.
    Their face is bright (white or light grey, sky-like) and sensor-invalid, their vertical silhouette edges are
    valid. The edge-enclosure veto keeps every object column (near or ranged), as for the dark twin; without the
    veto (control) the bright twins are lost."""

    def no_veto(self):
        c = outdoor_contract()
        c['sky']['edge_veto_range_m'] = C.provisional(.4000001)                    # no return is a near edge
        return c

    def check(self, objs, cols, pitch=0., expect=None):
        c = outdoor_contract()
        f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=objs)
        out = run(f, outdoor_state(1., 0., pitch), c)
        self.assertIsNone(out['refused'])
        self.assertEqual(kept_columns(out, cols), len(cols) if expect is None else expect)
        return out, run(f, outdoor_state(1., 0., pitch), self.no_veto())

    def test_panel_twins_at_and_above_the_flight_altitude(self):
        cam = camera_height(1.)
        lost_without_veto = 0
        for mat in (WHITE_INV, LGREY_INV, DARK_INV):
            for D in (1.5, 3., 4.5):
                cols = face_cols(-.4, .4, D)
                for pitch in (0., 10.):
                    objs = edge_strips(D, -.5, .5, cam + .05, cam + .55, mat) + [
                        ('box', (D, -.5, cam + .05), (D + .05, .5, cam + .55), mat), TREE_LINE]
                    with self.subTest(rgb=mat['rgb'], D=D, pitch=pitch):
                        out, control = self.check(objs, cols, pitch)
                        self.assertGreater(out['counts']['outdoor']['sky']['edge_vetoed_pixels'] if mat is not DARK_INV
                                           else 1, 0)
                        lost_without_veto += len(cols) - kept_columns(control, cols)
            objs = edge_strips(1.5, -.5, .5, cam - .10, cam + .45, mat) + [
                ('box', (1.5, -.5, cam - .10), (1.55, .5, cam + .45), mat), tree(5.)]
            with self.subTest(rgb=mat['rgb'], panel='altitude'):
                self.check(objs, face_cols(-.4, .4, 1.5))
        self.assertGreater(lost_without_veto, 150)                                 # the veto is what keeps them

    def test_raised_box_twins_wholly_above_the_horizon(self):
        """A trailer-like body 2.4 m wide whose lower edge is 0.05 m ABOVE the camera (no part below the horizon):
        at 2 m its face spans 64.6 deg, inside edge_veto_span_deg (70)."""
        cam = camera_height(1.)
        lost_without_veto = 0
        for mat in (WHITE_INV, LGREY_INV):
            for D in (2., 3.):
                cols = face_cols(-1.1, 1.1, D)
                objs = edge_strips(D, -1.2, 1.2, cam + .05, 2.6, mat, w=.05) + [
                    ('box', (D, -1.2, cam + .05), (D + 4., 1.2, 2.6), mat), tree(5.)]
                for pitch in (0., 10.):
                    with self.subTest(rgb=mat['rgb'], D=D, pitch=pitch):
                        _, control = self.check(objs, cols, pitch)
                        lost_without_veto += len(cols) - kept_columns(control, cols)
        self.assertGreater(lost_without_veto, 400)

    def test_gazebo_canopy_twin_below_the_virtual_ceiling(self):
        cols = face_cols(-1.4, 1.4, 2.3)
        for mat in (WHITE_INV, LGREY_INV, DARK_INV):
            for z in (2.0, 2.5):
                with self.subTest(rgb=mat['rgb'], z=z):
                    self.check(gazebo_twin(mat, z), cols)

    def test_rail_twin_against_an_open_horizon(self):
        """At 3 m the posts and both rail ends are in view: all columns kept (one column beside a post may lose
        blob_near density). At 2 m the rail runs out of the field of view beyond the outer posts: only the segments
        between valid edges are enclosed (the image border is not an edge)."""
        lost_without_veto = 0
        for mat in (WHITE_INV, LGREY_INV, DARK_INV):
            cols = face_cols(-1.4, 1.4, 3.)
            with self.subTest(rgb=mat['rgb'], D=3.):
                c = outdoor_contract()
                f, _ = outdoor_scene('oakd_like', 1., objects=rail_twin(mat, 3.))
                out = run(f, outdoor_state(1.), c)
                self.assertGreaterEqual(kept_columns(out, cols), len(cols) - 1)
                lost_without_veto += len(cols) - kept_columns(run(f, outdoor_state(1.), self.no_veto()), cols)
            inner = np.arange(15, 145)                                             # between the outer posts at 2 m
            with self.subTest(rgb=mat['rgb'], D=2.):
                f, _ = outdoor_scene('oakd_like', 1., objects=rail_twin(mat, 2.))
                out = run(f, outdoor_state(1.), outdoor_contract())
                self.assertEqual(kept_columns(out, inner), len(inner))
        self.assertGreater(lost_without_veto, 50)


class VetoPhantomCost(unittest.TestCase):
    """Outdoor repair r1: what the edge veto costs. Open sky, a tree line and open fields gain no near bearing; sky
    seen between two near trunks (both within edge_veto_range_m, ranges within the tolerance) becomes a near
    phantom between them; beyond the range nothing changes. Measured, not tuned (safe direction)."""

    def near(self, objs, veto=True, **scene):
        c = outdoor_contract()
        if not veto:
            c['sky']['edge_veto_range_m'] = C.provisional(.4000001)
        f, _ = outdoor_scene('oakd_like', 1., objects=objs, **scene)
        return run(f, outdoor_state(1.), c)['near_cols']

    def test_no_phantom_in_open_scenes(self):
        # Outdoor repair r2: the single trunk at 1.75 m and the golden-like bars (the nearest at 3 m) moved to
        # OneEdgePhantomCost (the one-edge rule keeps the sky in their rows near); objects beyond the edge range do not.
        for label, objs, scene in (('tree line', [TREE_LINE], {}), ('open field', [], {}),
                                   ('single trunk 6 m', [('vcyl', 6., 0., .15, 0., 5., BARK), TREE_LINE], {}),
                                   ('bars beyond 5 m', [('vcyl', 6., .5, .1, 0., 3., BARK), ('vcyl', 7., -1., .1, 0., 3., BARK),
                                                        TREE_LINE], {})):
            with self.subTest(label=label):
                self.assertEqual(self.near(objs), 0)

    def test_sky_between_near_trunks_becomes_a_phantom(self):
        pair = lambda d, s: [('vcyl', d, s / 2, .15, 0., 5., BARK), ('vcyl', d, -s / 2, .15, 0., 5., BARK), TREE_LINE]
        added = {}
        for d, s in ((3., 3.), (4., 5.), (6., 3.)):
            added[(d, s)] = (self.near(pair(d, s)), self.near(pair(d, s), veto=False))
        self.assertEqual(added[(3., 3.)][1], 0)
        self.assertGreater(added[(3., 3.)][0], 50)                                  # phantom between the trunks
        self.assertGreater(added[(4., 5.)][0], 100)
        self.assertEqual(added[(6., 3.)], (0, 0))                                   # beyond edge_veto_range_m


class ReviewerScenesFixed(unittest.TestCase):
    """The independent review r1 failing scenes that this repair resolves (geometry copied from its probes)."""

    def test_p0_1_invalid_rail_at_the_flight_altitude_is_near(self):
        """A dark sensor-invalid rail 0.10 m tall just below the camera (before: 0 of 156 columns at 1.5-3 m)."""
        for h in HEIGHTS:
            cam = camera_height(h)
            for pitch in (0., 5., -5.):
                for D in (1.5, 2., 3.):
                    cols = face_cols(-1.4, 1.4, D)
                    for dz in ((-.11, -.01), (-.05, .05)):
                        objs = [('box', (D, -1.5, cam + dz[0]), (D + .1, 1.5, cam + dz[1]), DARK_INV), TREE_LINE]
                        f, _ = outdoor_scene('oakd_like', h, 0., pitch, objects=objs)
                        out = run(f, outdoor_state(h, 0., pitch), outdoor_contract())
                        with self.subTest(h=h, pitch=pitch, D=D, dz=dz):
                            self.assertEqual(kept_columns(out, cols), len(cols))
                            self.assertEqual(out['counts']['outdoor']['sky']['far_ground_pixels'], 0)
        # Control: the same scene with the opt-in far-ground rule loses the rail (the reason it is off).
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (2., -1.5, cam - .11), (2.1, 1.5, cam - .01), DARK_INV), TREE_LINE])
        self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract(far_ground=True)), face_cols(-1.4, 1.4, 2.)), 0)

    def test_p0_2_altitude_panel_and_raised_box_reaching_below_the_horizon(self):
        """Fully sensor-invalid bright objects whose lower edge is 0.10 m below the camera: their rows below the
        horizon stay sensor-invalid (no far-ground rule) and make the columns near (before: 0 of 66 / 0 of 138)."""
        cam = camera_height(1.)
        for mat in (WHITE_INV, LGREY_INV):
            for background in ([tree(5.)], [tree(2.5)], []):
                f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1.5, -.5, cam - .10), (1.55, .5, cam + .45), mat)] + background)
                with self.subTest(rgb=mat['rgb'], background=len(background)):
                    self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-.4, .4, 1.5)), 66)
            for pitch in (0., 5., -5.):
                f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=[('box', (2., -1.2, cam - .10), (6., 1.2, 2.6), mat), tree(5.)])
                with self.subTest(rgb=mat['rgb'], pitch=pitch):
                    self.assertEqual(kept_columns(run(f, outdoor_state(1., 0., pitch), outdoor_contract()),
                                                  face_cols(-1.1, 1.1, 2.)), 138)


class FullyInvalidLimitations(unittest.TestCase):
    """Objects that are sensor-invalid with NO valid edge, bright and smooth like the sky, and wholly above the
    horizon cannot be told from sky in one frame. Each test asserts the SAFE outcome (detected) and is an expected
    failure: the limitation stays visible, and a fix flips it to an unexpected success."""

    @unittest.expectedFailure
    def test_bright_smooth_near_object_above_the_horizon_is_detected(self):
        # Replaces r1's MeasuredLimitations.test_bright_smooth_near_object_above_the_horizon_is_lost.
        cols = face_cols(-.3, .3, .4)
        top = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (.4, -.3, top + .05), (.45, .3, top + .6), WHITE), TREE_LINE])
        self.assertTrue(np.all(run(f, outdoor_state(1.), outdoor_contract())['cols'][cols] < .5))

    def test_controls_of_the_fully_invalid_class(self):
        # Before the outdoor profile the object was near (as was the sky everywhere); its dark twin stays near.
        cols = face_cols(-.3, .3, .4)
        top = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (.4, -.3, top + .05), (.45, .3, top + .6), WHITE), TREE_LINE])
        self.assertEqual(run(f, outdoor_state(1., vertical=False), indoor_contract())['near_cols'], 160)
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (.4, -.3, top + .05), (.45, .3, top + .6), DARK), TREE_LINE])
        self.assertTrue(np.all(run(f, outdoor_state(1.), outdoor_contract())['cols'][cols] < .5))   # MinZ / cos

    @unittest.expectedFailure
    def test_light_grey_panel_above_the_horizon_is_detected(self):
        """Review P0-2a / p2: a light-grey panel (luma close to the sky) at 1.5 m, camera +0.05..+0.55 m."""
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1.5, -.5, cam + .05), (1.55, .5, cam + .55), LGREY_INV), TREE_LINE])
        self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-.4, .4, 1.5)), 66)

    @unittest.expectedFailure
    def test_white_panel_touching_the_unimaged_band_is_detected(self):
        """The white panel kept at pitch 0 (its luma edge against the sky encloses it) touches the band a 10 deg
        nose-down camera does not image; like an object touching the image top at pitch 0, it is freed."""
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., 0., 10., objects=[('box', (1.5, -.5, cam + .05), (1.55, .5, cam + .55), WHITE_INV), TREE_LINE])
        self.assertEqual(kept_columns(run(f, outdoor_state(1., 0., 10.), outdoor_contract()), face_cols(-.4, .4, 1.5)), 66)

    def test_white_panel_control_at_pitch_zero(self):
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1.5, -.5, cam + .05), (1.55, .5, cam + .55), WHITE_INV), TREE_LINE])
        self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-.4, .4, 1.5)), 66)

    @unittest.expectedFailure
    def test_bright_invalid_canopy_below_the_virtual_ceiling_is_detected(self):
        """Review P0-3 / p7: a white canopy underside 10 m wide at 2.0 m, its edges outside the field of view."""
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1., -5., 2.), (6., 5., 2.1), WHITE_INV), TREE_LINE])
        self.assertEqual(run(f, outdoor_state(1.), outdoor_contract())['near_cols'], 160)


# ------------------------------------------------------------- measured residual risks (not tuned), cases 4, 5, 9, 10
class MeasuredLimitations(unittest.TestCase):
    """Characterisation tests of phantoms (the safe direction) and of the wires; misses are expected failures."""

    def wire(self, dist, px, invalid, pitch):
        fx = F.source_intrinsics('oakd_like')['fx']
        r = px * dist / fx / 2
        f, kind = outdoor_scene('oakd_like', 1., 0., pitch, objects=[
            ('hcyl', dist + .1, camera_height(1.) + dist * .25, r, -3., 3., material(DARK_RGB, 6., invalid)), TREE_LINE])
        self.assertGreater(int((kind == 2).sum()), 0)                    # the wire is in the source image
        return run(f, outdoor_state(1., 0., pitch), outdoor_contract())

    def test_valid_wires_on_the_tilted_path_are_ranged_never_closer(self):
        """Wires with valid depth, 1-3 source pixels, 2 and 5 m: the tilted (z-buffer) path ranges them at their
        distance on many bearings (review P3-1: the r1 assertion here was vacuous) and never closer."""
        for dist in (2., 5.):
            for px in (1, 3):
                out = self.wire(dist, px, 'never', 3.)
                with self.subTest(dist=dist, px=px):
                    self.assertEqual(out['near_cols'], 0)
                    self.assertGreater(out['min'], dist - .15)
                    self.assertGreater(int((np.abs(out['cols'] - dist) < .3).sum()), 50)

    @unittest.expectedFailure
    def test_valid_thin_wires_on_the_level_path_are_detected(self):
        """Pitch 0 (level path, one nearest source sample per trained pixel): a 1-3 px wire is sub-pixel at 160x90."""
        found = [int((np.abs(self.wire(dist, px, 'never', 0.)['cols'] - dist) < .3).sum()) for dist in (2., 5.) for px in (1, 3)]
        self.assertTrue(all(n > 0 for n in found), found)

    @unittest.expectedFailure
    def test_invalid_thin_wires_are_detected(self):
        """Review P2-2 (pre-existing): sensor-invalid wires are never near; blob_near needs a dense blob."""
        found = [self.wire(dist, px, 'always', pitch)['near_cols'] for dist in (2., 5.) for px in (1, 3) for pitch in (0., 3.)]
        self.assertTrue(all(n > 0 for n in found), found)

    def test_textured_clouds_stay_near_and_a_sun_blob_below_the_horizon_stays_near(self):
        for pitch in PITCHES:
            f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=[TREE_LINE], sky='cloudy')
            cloudy = run(f, outdoor_state(1., 0., pitch), outdoor_contract())
            f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=[TREE_LINE])
            clear = run(f, outdoor_state(1., 0., pitch), outdoor_contract())
            f, kind = outdoor_scene('oakd_like', 1., 0., pitch, objects=[TREE_LINE], sun=(0., -8., 3.))
            sun = run(f, outdoor_state(1., 0., pitch), outdoor_contract())
            with self.subTest(pitch=pitch):
                self.assertEqual(clear['near_cols'], 0)                                  # control
                self.assertGreater(cloudy['near_cols'], 0)
                self.assertLess(cloudy['near_cols'], 160)
                self.assertGreater(int((kind == -1).sum()), 0)
                self.assertGreater(sun['near_cols'], 0)
                self.assertTrue(np.all(sun['cols'][np.flatnonzero(sun['cols'] < 1.)] <= .45))


# --------------------------------------------------------------------------- ground estimate (B), case 8
class GroundEstimate(unittest.TestCase):
    def estimate(self, f, state, **changes):
        return R.estimate_ground(f, state, outdoor_contract(**changes))

    def test_flat_textured_ground_is_accepted_with_a_small_height_error(self):
        for source in ('oakd_like', 'trained_equal'):
            for height in HEIGHTS:
                f, _ = outdoor_scene(source, height, objects=[TREE_LINE])
                rep = self.estimate(f, outdoor_state(height))
                with self.subTest(source=source, height=height):
                    self.assertTrue(rep['accepted'], rep['reasons'])
                    self.assertLess(abs(rep['plane']['height_m'] - camera_height(height)), .02)
                    self.assertLess(rep['plane']['tilt_deg'], .5)
                    self.assertGreaterEqual(rep['plane']['inliers'], 300)
                    self.assertGreater(rep['plane']['share'], .9)
                    self.assertLess(rep['plane']['rms_m'], .03)
        # Deterministic: the same frame gives the same report.
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
        self.assertEqual(json.dumps(self.estimate(f, outdoor_state(1.)), sort_keys=True),
                         json.dumps(self.estimate(f, outdoor_state(1.)), sort_keys=True))

    def test_named_refusals_and_their_controls(self):
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
        self.assertTrue(self.estimate(f, outdoor_state(1.))['accepted'])                 # positive control
        for label, state, reason in (
                ('z_valid false', outdoor_state(1., z_valid=False), 'estimator_z_invalid'),
                ('epv large', outdoor_state(1., epv=2.), 'estimator_epv_exceeds_limit'),
                ('dead reckoning', outdoor_state(1., dead_reckoning=True), 'estimator_dead_reckoning'),
                ('fields absent', outdoor_state(1., vertical=False), 'estimator_vertical_fields_absent'),
                ('drift +0.5', outdoor_state(1., z_bias=.5), 'ground_height_disagrees_with_px4'),
                ('drift -0.5', outdoor_state(1., z_bias=-.5), 'ground_height_disagrees_with_px4')):
            rep = self.estimate(f, state)
            with self.subTest(label=label):
                self.assertFalse(rep['accepted'])
                self.assertEqual(rep['reason'], reason)
                self.assertIsNone(rep['plane'])
        for bias in (.2, -.2):
            rep = self.estimate(f, outdoor_state(1., z_bias=bias))
            self.assertTrue(rep['accepted'])
            self.assertAlmostEqual(rep['plane']['height_error_m'], -bias, delta=.02)
        state = outdoor_state(1.)
        state['epv'] = float('nan')
        with self.assertRaises(R.InputError):                     # malformed is an input error, not a refusal
            self.estimate(f, state)
        state = outdoor_state(1.)
        state['z_valid'] = 1
        with self.assertRaises(R.InputError):
            self.estimate(f, state)
        # Outdoor repair r1 (review P3-2): epv must be a finite real scalar; strings and arrays are malformed.
        for bad in ('0.1', np.array([.1]), np.array(.1), [.1], None, 1j, True):
            state = outdoor_state(1.)
            state['epv'] = bad
            with self.subTest(epv=repr(bad)), self.assertRaises(R.InputError):
                self.estimate(f, state)
        for good in (.1, 0, np.float32(.1), np.float64(.1), np.int64(0)):                 # positive controls
            state = outdoor_state(1.)
            state['epv'] = good
            with self.subTest(epv=repr(good)):
                self.assertTrue(self.estimate(f, state)['accepted'])
        # Dead reckoning is refused even by an (unvalidated) contract whose flag says otherwise (review P3-3).
        c = outdoor_contract()
        c['ground_estimate']['refuse_dead_reckoning'] = False
        self.assertEqual(R.estimate_ground(f, outdoor_state(1., dead_reckoning=True), c)['reason'], 'estimator_dead_reckoning')
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE], ground='textureless')
        self.assertEqual(self.estimate(f, outdoor_state(1.))['reason'], 'ground_insufficient_points')
        # The verified camera height must lie in the band (camera at 0.03 m on the ground).
        f, _ = outdoor_scene('oakd_like', .06, objects=[TREE_LINE])
        self.assertIn('verified_camera_height_outside_band', self.estimate(f, outdoor_state(.06))['reasons'])

    def test_slopes_per_tolerance(self):
        for height in HEIGHTS:
            for slope, accepted in ((2., True), (5., False), (10., False)):
                f, _ = outdoor_scene('oakd_like', height, objects=[TREE_LINE], slope_deg=slope)
                rep = self.estimate(f, outdoor_state(height))
                with self.subTest(height=height, slope=slope):
                    self.assertEqual(rep['accepted'], accepted, rep['reasons'])
                    tilt = rep['plane']['tilt_deg'] if accepted else rep['candidates'][0]['tilt_deg']
                    self.assertAlmostEqual(tilt, slope, delta=.3)
                    if not accepted:
                        self.assertEqual(rep['reason'], 'ground_tilt_exceeds_tolerance')
                        # At the PROVISIONAL 3 deg tolerance; a 12 deg tolerance would accept the same plane.
                        self.assertTrue(self.estimate(f, outdoor_state(height), tilt_max_deg=12.)['accepted'])

    def test_attitude_error_and_pose_sweep(self):
        for error in ((0., 2.), (0., -2.), (2., 0.), (-2., 0.)):
            f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
            rep = self.estimate(f, outdoor_state(1., attitude_error=error))
            with self.subTest(error=error):
                self.assertTrue(rep['accepted'])
                self.assertAlmostEqual(rep['plane']['tilt_deg'], 2., delta=.2)
                self.assertLess(abs(rep['plane']['height_error_m']), .02)
        refused = 0
        for height in HEIGHTS:
            for pitch in (-10., -5., 0., 5., 10.):
                for roll in (-5., 5.):
                    f, _ = outdoor_scene('oakd_like', height, roll, pitch, objects=[TREE_LINE])
                    refused += not self.estimate(f, outdoor_state(height, roll, pitch))['accepted']
        self.assertEqual(refused, 0)

    def test_low_wide_box(self):
        """Ground visible beside/behind a low wide box: outdoor repair r1 (review P1-1) refuses every such frame
        (r1 accepted 10 of 14 sweep variants, 4 with a wrong plane up to +0.11 m / 2.80 deg). A box hiding all
        ground is bounded by the PX4 tolerance (0.3 m top: refused); the 0.2 m top is a named residual
        (GroundWrongSurface.test_raised_platform_hiding_all_ground_is_refused, an expected failure)."""
        for height, pitch, lo, hi in ((1., 10., (3., -6., 0.), (8., 6., .3)), (1., 0., (4., -3., 0.), (9., 3., .3)),
                                      (1.5, 0., (2., -2., 0.), (6., 2., .3)), (1., 0., (3., -6., 0.), (8., 6., .3)),
                                      (1.5, 0., (3., -6., 0.), (8., 6., .2)), (1., 0., (3., -6., 0.), (8., 6., .2)),
                                      (1., 10., (3., -6., 0.), (8., 6., .2))):
            f, _ = outdoor_scene('oakd_like', height, 0., pitch, objects=[('box', lo, hi, material((120., 110., 100.), 25.)), TREE_LINE])
            rep = self.estimate(f, outdoor_state(height, 0., pitch))
            with self.subTest(height=height, pitch=pitch, box=(lo, hi)):
                self.assertFalse(rep['accepted'])
                self.assertIn(rep['reason'], ('ground_not_lowest_surface', 'ground_tilt_exceeds_tolerance',
                                              'ground_ambiguous', 'ground_height_disagrees_with_px4', 'ground_not_planar',
                                              'ground_inlier_spread_insufficient'))
        for top in (.3, .6):
            f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1.5, -6., 0.), (8., 6., top), material((120., 110., 100.), 25.)), TREE_LINE])
            self.assertFalse(self.estimate(f, outdoor_state(1.))['accepted'])
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
        self.assertTrue(self.estimate(f, outdoor_state(1.))['accepted'])                     # positive control


class GroundWrongSurface(unittest.TestCase):
    """Outdoor repair r1 (review P1-1): wrong-surface fits are refused (rival plane, inlier spread, planarity) while
    every must-accept case and the obstacle scenes are still accepted."""

    def test_bridging_plane_is_refused_and_the_must_accept_sweep_is_accepted(self):
        box = material((120., 110., 100.), 25.)
        for pitch in (0., 10.):                 # r1 accepted +0.110 m / 2.80 deg and +0.069 m / 2.15 deg here
            f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=[('box', (3., -6., 0.), (8., 6., .2), box), TREE_LINE])
            rep = R.estimate_ground(f, outdoor_state(1., 0., pitch), outdoor_contract())
            with self.subTest(pitch=pitch):
                self.assertFalse(rep['accepted'])
                self.assertIn('ground_not_planar', rep['reasons'])
                self.assertGreater(rep['candidates'][0]['planarity_deg']['near_third'] or 0., 1.)
        refused, worst = [], 0.
        cases = [(h, roll, pitch, {}, {}) for h in HEIGHTS for pitch in (-10., -5., 0., 5., 10.) for roll in (-5., 0., 5.)]
        cases += [(h, 0., 0., {}, {'z_bias': b}) for h in HEIGHTS for b in (.2, -.2)]
        cases += [(h, 0., 0., {}, {'attitude_error': e}) for h in HEIGHTS for e in ((0., 2.), (0., -2.), (2., 0.), (-2., 0.))]
        cases += [(h, 0., 0., {'slope_deg': 2.}, {}) for h in HEIGHTS]
        for h, roll, pitch, scene, state in cases:
            f, _ = outdoor_scene('oakd_like', h, roll, pitch, objects=[TREE_LINE], **scene)
            rep = R.estimate_ground(f, outdoor_state(h, roll, pitch, **state), outdoor_contract())
            if not rep['accepted']:
                refused.append((h, roll, pitch, scene, state, rep['reasons']))
            else:
                worst = max(worst, max(rep['plane']['planarity_deg'].values()))
        self.assertEqual(len(cases), 44)
        self.assertEqual(refused, [])
        self.assertLess(worst, .6)                                    # margin to planarity_max_angle_deg (1.0)

    def test_obstacle_scenes_are_not_refused(self):
        wall = material((150., 140., 130.), 25.)
        scenes = {'trunk 0.55': [('vcyl', .55, 0., .15, 0., 5., BARK)], 'trunk 1.75': [('vcyl', 1.75, 0., .15, 0., 5., BARK)],
                  'wall alongside 1 m': [('box', (0., 1., 0.), (20., 1.3, 4.), wall)],
                  'bench 2 m': [('box', (2., -.8, .4), (2.4, .8, .45), material((120., 110., 100.), 25.))],
                  'car-like box 3 m': [('box', (3., -1., .3), (7.5, 1., 1.5), material((60., 60., 140.), 25.))],
                  'rail twin': rail_twin(LGREY_INV, 2.)}
        for name, objs in scenes.items():
            for pitch in (0., 10.):
                f, _ = outdoor_scene('oakd_like', 1., 0., pitch, objects=objs + [TREE_LINE])
                rep = R.estimate_ground(f, outdoor_state(1., 0., pitch), outdoor_contract())
                with self.subTest(scene=name, pitch=pitch):
                    self.assertTrue(rep['accepted'], rep['reasons'])
                    self.assertLess(abs(rep['plane']['height_error_m']), .02)

    def test_rivals_are_horizontal_surfaces_only(self):
        # The refinement may turn a hypothesis into a near-vertical plane on a trunk: that is not a rival ground.
        f, _ = outdoor_scene('oakd_like', 1., objects=[('vcyl', 1.75, 0., .15, 0., 5., BARK), TREE_LINE])
        rep = R.estimate_ground(f, outdoor_state(1.), outdoor_contract())
        self.assertTrue(rep['accepted'])
        self.assertTrue(any(q['tilt_deg'] > R.GROUND_SEARCH_TILT_DEG for q in rep['candidates']))

    def test_fixed_runtime_constants_are_marked_provisional(self):
        """Review P3-4: the ground fit's fixed constants are thresholds in effect; each is marked PROVISIONAL."""
        source = (ROOT / 'tools/camera_policy_runtime.py').read_text().splitlines()
        self.assertIn('GROUND_SEARCH_TILT_DEG', R.PROVISIONAL_RUNTIME_CONSTANTS)
        # Outdoor repair r2 added EDGE_VETO_MAX_INCIDENCE_DEG and REFUSAL_REENTRY_FRAMES.
        self.assertEqual(len(R.PROVISIONAL_RUNTIME_CONSTANTS), 12)
        for name in R.PROVISIONAL_RUNTIME_CONSTANTS:
            line = [x for x in source if x.startswith(name + ' = ')]
            with self.subTest(name=name):
                self.assertEqual(len(line), 1)
                self.assertIn('# PROVISIONAL', line[0])
                self.assertTrue(hasattr(R, name))

    @unittest.expectedFailure
    def test_raised_platform_hiding_all_ground_is_refused(self):
        """Named residual: a horizontal surface 0.2 m high that hides all ground within the fit range is one clean
        plane; only PX4's 0.3 m height tolerance bounds it, and that tolerance must accept +-0.2 m z drift."""
        f, _ = outdoor_scene('oakd_like', 1., objects=[('box', (1.5, -6., 0.), (8., 6., .2), material((120., 110., 100.), 25.)), TREE_LINE])
        self.assertFalse(R.estimate_ground(f, outdoor_state(1.), outdoor_contract())['accepted'])


# ----------------------------------------------------------------------------- training equivalence (D)
def _golden_scenes():
    S = F.render
    yield 'wall trained', S('trained_equal', 1., wall_x=10.)[0], F.scene_state(1.)
    yield 'pole oak', S('oakd_like', 1., wall_x=10., pole=(3., 0., .1))[0], F.scene_state(1.)
    yield 'pole oak pitch', S('oakd_like', 1., pitch_deg=.5, wall_x=10., pole=(3., 0., .1))[0], F.scene_state(1., pitch_deg=.5)
    yield 'pole oak roll3 pitch-4', S('oakd_like', 1.2, 3., -4., wall_x=8., pole=(2., .3, .1))[0], F.scene_state(1.2, 3., -4.)
    yield 'holes floor', S('oakd_like', 1., pitch_deg=3., wall_x=10., floor='textured', holes=.02, seed=7)[0], F.scene_state(1., pitch_deg=3.)
    yield 'floor textureless pitch5', S('oakd_like', 1., 0., 5., wall_x=9., floor='textureless')[0], F.scene_state(1., 0., 5.)
    yield 'ceiling textured', S('trained_equal', 1., wall_x=10., floor='textured', ceiling=(2.5, 'textured'))[0], F.scene_state(1.)
    yield 'ceiling textureless', S('trained_equal', 1., wall_x=10., floor='textured', ceiling=(2.5, 'textureless'))[0], F.scene_state(1.)
    yield 'near slab', S('oakd_like', 1., wall_x=10., near_slab=(.3, .2))[0], F.scene_state(1.)
    yield 'border coverage', S('oakd_like', 1., wall_x=10., border=12, border_kind='coverage')[0], F.scene_state(1.)
    yield 'sphere invalid', S('oakd_like', 1., wall_x=10., sphere=(2., 0., 1., .3), sphere_invalid=True)[0], F.scene_state(1.)
    for pitch, roll in ((0., 0.), (5., 0.), (-5., 3.)):
        f, _ = outdoor_scene('oakd_like', 1., roll, pitch, objects=[TREE_LINE, ('vcyl', .55, 0., .15, 0., 5., BARK)])
        yield 'outdoor p%g r%g' % (pitch, roll), f, F.scene_state(1., roll, pitch)


def _golden_contracts():
    yield 'off', F.contract()
    yield 'default', F.contract(ground=True)
    yield 'ground+ceiling2.8', F.contract(ground=True, ceiling=2.8)
    yield 'plane rule', F.contract(ground=True, ceiling=2.8, plane_rule=True)
    yield 'unasserted', F.contract(ground=True, asserted=False)
    yield 'ceiling only 3.0', F.contract(ceiling=3.0)


def _golden_contract_variants():
    base = C._metadata(1, ROOT)
    base['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
    yield 'base', base
    v = copy.deepcopy(base); v['ceiling'].update(enabled=True, height_m=3.0); yield 'ceiling3', v
    v = copy.deepcopy(base); v['ceiling'].update(enabled=True, height_m=3.0, physical=False); yield 'ceiling3 physical false', v
    v = copy.deepcopy(base); v['ceiling'].update(physical=True); yield 'physical true', v
    v = copy.deepcopy(base); v['profile'] = 'outdoor'; yield 'profile only', v
    v = copy.deepcopy(base); v['ground']['enabled'] = False; yield 'ground off', v
    v = copy.deepcopy(base); v['extra'] = 1; yield 'extra key', v
    v = copy.deepcopy(base); v['estimator_height'] = dict(F.FIXTURE_ASSERTION); yield 'asserted', v


def indoor_golden_digests():
    """SHA-256 of every indoor/default-path output: per scene, preprocess (RGB, depth, counts, projection report),
    project_rgbd (five arrays and the report) and height_band over the six indoor contracts, exceptions included
    (type and message), plus the indoor state carrying PX4 vertical fields; and the contract layer (validation
    problems and height_reference_problem of indoor variants). Only APIs that existed before the outdoor profile are
    used, so a fresh process that loads the pre-change camera_policy_* / camera_ros2_* modules first computes the
    same function: that is how INDOOR_GOLDENS was produced (repair_r1/scratch/gen_goldens.py, 2026-11-13)."""
    import hashlib
    import torch
    H = lambda b: hashlib.sha256(b).hexdigest()
    def guard(fn):
        try:
            return fn()
        except Exception as exc:                                    # noqa: BLE001 (exceptions are outputs here)
            return ['EXC', type(exc).__name__, str(exc)]
    scenes = {}
    vertical = dict(F.scene_state(1.), z_valid=True, epv=.1, dead_reckoning=False)
    for name, f, state in _golden_scenes():
        parts = []
        for cname, contract in _golden_contracts():
            rt = object.__new__(R.CameraPolicyRuntime)
            rt.contract, rt.validity, rt.torch, rt.perception = contract, G.DV, torch, G.NoGoalPerception()
            def pre():
                a = rt.preprocess(f, state)
                return [H(a[0].numpy().tobytes()), H(a[1].numpy().tobytes()), json.dumps(a[2], sort_keys=True),
                        json.dumps(a[3], sort_keys=True)]
            def proj():
                q = R.project_rgbd(f, state, contract)
                return [H(x.tobytes()) for x in q[:5]] + [json.dumps(q[5], sort_keys=True)]
            parts.append([cname, guard(pre), guard(proj), guard(lambda: json.dumps(R.height_band(state, contract), sort_keys=True)),
                          guard(lambda: rt.preprocess(f, vertical) and 'ACCEPTED')])
        scenes[name] = H(json.dumps(parts, sort_keys=True).encode())
    contracts = {}
    for name, value in _golden_contract_variants():
        contracts[name] = {'valid': C.validate_contract(value, verify_files=False, root=ROOT) == [],
                           'height_reference_problem_none': C.height_reference_problem(value) is None}
    return {'scenes': scenes, 'contracts': contracts}


# Produced from the pre-change tree (pre_tree, before the outdoor profile) in a fresh process; see
# indoor_golden_digests. Equal digests mean byte-identical indoor/default outputs. No absolute path is read.
INDOOR_GOLDENS = {
    'scenes': {
        'border coverage': '297e4c51d5bb93c5eba44e1d7ff4a229c7bc9f54c9aa251a06bcc5d27471a119',
        'ceiling textured': '358727684933b1f00ceab89e96e072ce8c7aee13db88db463023ce0059f4ec13',
        'ceiling textureless': '872491f4d8aa0b1b7adc491387bcef22ee70818c8effc31a6cfcaba3c215cf6f',
        'floor textureless pitch5': '19c1bc21c1e0989aba627385b6103bb0eb9bb792cc0862b29118c1206861d35f',
        'holes floor': '0428986af7d06ed872f97f4498ce7c9e850f9a0fc8567fc9af620bea47910669',
        'near slab': '669ebb9864b7a630f0b5b5d5a37e6d73a716e9333ee83a49daa2cb87c68ef932',
        'outdoor p-5 r3': '089277f534a26ad7e613b1065dbaf5be95e8ad95a7ffcc0443aeba2bb496566a',
        'outdoor p0 r0': '2d147d2e7c2ef36df518299b81e905fa9aa712e467a69e9764c0d33ac1fcd767',
        'outdoor p5 r0': 'c3ecd592b940150aef203cfb165971aaddda63c4efd12d8afda9ae1415ecb12c',
        'pole oak': 'af7def37298c323c934b8ebedd28db5f70917962597e08cbcd012b921cf9aa8a',
        'pole oak pitch': '945babec5c8875287aa4d676b3c3f8e9e3e13abd26122e2ac3f37e0fdbefafa1',
        'pole oak roll3 pitch-4': '55909b9b208ecc8f99e064cba5b45e9e135f710fa2458f6bed118f597e4c0738',
        'sphere invalid': '0aa33d324e66df563d01841df9d3ad70b1d921252fd9ce5016ba80e4f2a5c2a6',
        'wall trained': 'ff751c8021f630686e6ab905d11ff5313138096aacb34083a151a2ee9cc3d39b',
    },
    'contracts': {
        'asserted': {'valid': True, 'height_reference_problem_none': True},
        'base': {'valid': True, 'height_reference_problem_none': False},
        'ceiling3': {'valid': True, 'height_reference_problem_none': False},
        'ceiling3 physical false': {'valid': False, 'height_reference_problem_none': False},
        'extra key': {'valid': False, 'height_reference_problem_none': False},
        'ground off': {'valid': True, 'height_reference_problem_none': True},
        'physical true': {'valid': False, 'height_reference_problem_none': False},
        'profile only': {'valid': False, 'height_reference_problem_none': False},
    },
}


class OutdoorOffIsBitIdentical(unittest.TestCase):
    def test_projection_default_outputs_are_unchanged_by_the_unimaged_option(self):
        for name, f, state in _golden_scenes():
            plain = R.project_rgbd(f, state, F.contract())
            extended = R.project_rgbd(f, state, F.contract(), unimaged=True)
            with self.subTest(scene=name):
                self.assertEqual(len(plain), 6)
                self.assertEqual(len(extended), 7)
                for a, b in zip(plain[:5], extended[:5]):
                    self.assertEqual(a.tobytes(), b.tobytes())
                self.assertEqual(plain[5], extended[5])

    def test_indoor_outputs_equal_the_pre_change_golden_digests(self):
        """Review P3-6: compares with digests embedded here (produced from the pre-change modules in a fresh
        process), not with a file under ~/motar_tmp, and never mixes old and new modules in one process."""
        current = indoor_golden_digests()
        self.assertEqual(len(current['scenes']), 14)
        self.assertEqual(set(current['scenes']), set(INDOOR_GOLDENS['scenes']))
        for name, digest in INDOOR_GOLDENS['scenes'].items():
            with self.subTest(scene=name):
                self.assertEqual(current['scenes'][name], digest)
        self.assertEqual(current['contracts'], INDOOR_GOLDENS['contracts'])
        self.assertTrue(current['contracts']['base']['valid'])                          # positive control
        self.assertFalse(current['contracts']['ceiling3 physical false']['valid'])
        # Control: the outdoor profile does change the observation of an outdoor scene.
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
        self.assertNotEqual(run(f, outdoor_state(1.), outdoor_contract())['near_cols'],
                            run(f, outdoor_state(1., vertical=False), indoor_contract())['near_cols'])

    def test_indoor_state_with_vertical_fields_is_still_refused(self):
        f, _ = outdoor_scene('trained_equal', 1., objects=[TREE_LINE])
        with self.assertRaisesRegex(R.InputError, 'state requires only estimator fields'):
            R.project_rgbd(f, outdoor_state(1.), F.contract())
        with self.assertRaisesRegex(R.InputError, 'state requires only estimator fields'):
            R.height_band(outdoor_state(1.), F.contract(ground=True))
        self.assertTrue(R.height_band(outdoor_state(1.), outdoor_contract())['in_band'])   # outdoor splits them off
        self.assertEqual(R.split_state(outdoor_state(1.), F.contract()), (outdoor_state(1.), None))


# ------------------------------------------------------------------------- ROS session: refusal -> zero hold
class RefusalRuntime(G.BandRuntime):
    """Session stand-in: the production height_band with the outdoor contract; step refuses while `refuse` is set."""
    def __init__(self):
        super().__init__()
        self.contract = outdoor_contract()
        self.refuse = None

    def step(self, frame, state, *, now_s):
        if self.refuse == 'input':
            raise R.InputError('malformed frame')
        if self.refuse:
            raise R.FrameRefused(self.refuse)
        self.events.append('step')
        return dict(t=now_s, velocity_world_enu_mps=[.8, -.4, 0.], yaw_rate_rps=.3)

    def applied_zero_hold(self):
        self.events.append('zero_hold_applied')


class SessionHoldOnRefusal(unittest.TestCase):
    tick = G.AltitudeGating.tick

    def setUp(self):
        from camera_ros2_session import RosPolicySession
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.runtime = RefusalRuntime()
        self.session = RosPolicySession(self.runtime, F.ros_binding(Path(self.tmp.name) / 'fixture.txt'))
        self.runtime.events.clear()
        self.k = 0

    def test_production_runtime_records_the_applied_zero_hold(self):
        rt = object.__new__(R.CameraPolicyRuntime)
        import torch
        rt.torch, rt.failed, rt.prev = torch, False, torch.tensor([[.5, -.2, 0., .3]])
        rt.applied_zero_hold()
        self.assertEqual(rt.prev.tolist(), [[0., 0., 0., 0.]])
        rt.failed = True
        with self.assertRaises(RuntimeError):
            rt.applied_zero_hold()

    def start_hold(self):
        """Outdoor repair r2 (review r2 P3-N7): an outdoor session starts with the re-entry count (a history reset)."""
        from camera_ros2_session import REFUSAL_REENTRY_FRAMES
        for k in range(REFUSAL_REENTRY_FRAMES):
            proposal, _ = self.tick(1.)
            self.assertEqual(proposal['velocity_enu_mps'], [0., 0., 0.])
            self.assertEqual(proposal['reentry_hold_remaining'], REFUSAL_REENTRY_FRAMES - 1 - k)
            self.assertEqual(self.session.status, 'HOLD_REENTRY')
        proposal, _ = self.tick(1.)
        self.assertEqual(proposal['velocity_enu_mps'], [.8, -.4, 0.])                    # positive control
        self.assertEqual(self.session.status, 'RUNNING_PROPOSALS')
        return proposal

    def test_take_off_gate_still_uses_px4_z_with_the_outdoor_contract(self):
        self.assertIsNone(self.tick(0.)[0])                    # on the ground: WAITING_ALTITUDE, runtime not called
        self.assertEqual(self.session.status, 'WAITING_ALTITUDE')
        self.assertEqual(self.runtime.events, [])
        self.assertIsNotNone(self.tick(1.)[0])                 # control: in band (held: a fresh history)
        self.assertEqual(self.runtime.events, ['reset', 'step', 'zero_hold_applied'])

    def test_refused_frames_propose_an_explicit_zero_hold_then_restart_history(self):
        self.start_hold()
        self.runtime.events.clear()
        self.runtime.refuse = 'ground_ambiguous'
        for k in range(3):
            proposal, item = self.tick(1.)
            self.assertEqual(proposal['velocity_enu_mps'], [0., 0., 0.])
            self.assertEqual(proposal['yaw_rate_enu_rps'], 0.)
            self.assertTrue(proposal['hold'])
            self.assertEqual(proposal['refusal'], 'ground_ambiguous')
            self.assertEqual(proposal['valid_until_s'], proposal['capture_stamp_s'] + .15)
            self.assertIsNone(self.session.fault)
            self.assertEqual(self.session.status, 'HOLD_FRAME_REFUSED')
            self.assertEqual(self.session.consecutive_refusals, k + 1)
        self.assertEqual(self.runtime.events, ['reset', 'reset'])           # no step advanced while refusing
        self.runtime.refuse = None
        # Outdoor repair r1 (review P2-1): re-entry hysteresis. The first REFUSAL_REENTRY_FRAMES accepted frames run
        # the policy (fresh history) but still propose the zero hold, and the runtime is told the hold was applied.
        from camera_ros2_session import REFUSAL_REENTRY_FRAMES
        self.assertEqual(REFUSAL_REENTRY_FRAMES, 5)
        for k in range(REFUSAL_REENTRY_FRAMES):
            proposal, item = self.tick(1.)
            with self.subTest(reentry=k):
                self.assertEqual(item.fresh_history, k == 0)
                self.assertEqual(proposal['velocity_enu_mps'], [0., 0., 0.])
                self.assertEqual(proposal['yaw_rate_enu_rps'], 0.)
                self.assertTrue(proposal['hold'])
                self.assertEqual(proposal['reentry_hold_remaining'], REFUSAL_REENTRY_FRAMES - 1 - k)
                self.assertEqual(self.session.status, 'HOLD_REENTRY')
                self.assertEqual(self.runtime.events[-2:], ['step', 'zero_hold_applied'])
        proposal, item = self.tick(1.)
        self.assertEqual(self.runtime.events[-1], 'step')
        self.assertEqual(proposal['velocity_enu_mps'], [.8, -.4, 0.])
        self.assertNotIn('hold', proposal)
        self.assertEqual((self.session.status, self.session.consecutive_refusals), ('RUNNING_PROPOSALS', 0))
        self.assertEqual(self.session.reentry_hold_remaining, 0)

    def test_a_refusal_during_reentry_restarts_the_count(self):
        from camera_ros2_session import REFUSAL_REENTRY_FRAMES
        self.start_hold()
        self.runtime.refuse = 'ground_not_planar'
        self.assertTrue(self.tick(1.)[0]['hold'])
        self.runtime.refuse = None
        for _ in range(REFUSAL_REENTRY_FRAMES - 1):
            self.assertEqual(self.tick(1.)[0]['velocity_enu_mps'], [0., 0., 0.])
        self.runtime.refuse = 'ground_ambiguous'                                         # intermittent refusal
        proposal, _ = self.tick(1.)
        self.assertEqual(proposal['refusal'], 'ground_ambiguous')
        self.runtime.refuse = None
        held = 0
        while True:
            proposal, _ = self.tick(1.)
            if proposal['velocity_enu_mps'] != [0., 0., 0.]:
                break
            held += 1
        self.assertEqual(held, REFUSAL_REENTRY_FRAMES)                                   # the count restarted
        self.assertEqual(self.session.status, 'RUNNING_PROPOSALS')

    def test_reentry_hold_needs_the_runtime_to_record_the_applied_hold(self):
        # Fail closed: a runtime that cannot record the applied zero hold faults the session at re-entry.
        self.assertIsNotNone(self.tick(1.)[0])
        self.runtime.refuse = 'ground_not_found'
        self.tick(1.)
        self.runtime.refuse = None
        self.runtime.applied_zero_hold = None
        with self.assertRaises(TypeError):
            self.tick(1.)
        self.assertEqual(self.session.status, 'FAULT')

    def test_any_other_input_error_still_faults(self):
        self.assertIsNotNone(self.tick(1.)[0])
        self.runtime.refuse = 'input'
        t, wall = 3. + self.k * .1, 100. + self.k * .1
        msgs = F.image_messages(t, np.zeros((90, 160, 3), np.uint8), np.full((90, 160), 5000, np.uint16), F.TRAINED)
        msgs['odometry'] = F.odometry_message(t, [0., 0., 1.], 0., [0., 0., 0.], 0.)
        for kind, message in msgs.items():
            self.session.offer(kind, message, received_monotonic_s=wall)
        item = self.session.prepare(now_ros_s=t + .02, now_monotonic_s=wall + .02)
        with self.assertRaises(R.InputError):
            self.session.infer(item)
        self.session.fail_inference('malformed frame')
        self.assertEqual(self.session.status, 'FAULT')

    def test_the_gateway_publishes_the_hold_as_zero_velocity_with_the_altitude_hold(self):
        gate, _ = G.activate(G.fence())
        moving = G.send(gate, 1, (0., 0.), (1.5, -.5), yaw=.4)
        self.assertNotEqual(G.enu_velocity(moving), (0., 0.))
        hold = G.send(gate, 2, (0., 0.), (0., 0.), yaw=0.)
        self.assertEqual(G.enu_velocity(hold), (0., 0.))
        self.assertEqual(hold['trajectory_setpoint']['yawspeed'], 0.)
        self.assertEqual(hold['trajectory_setpoint']['position'][2], moving['trajectory_setpoint']['position'][2])
        self.assertEqual(gate.state, 'ACTIVE')


class SessionReentryRestartsOnEveryHistoryReset(unittest.TestCase):
    """Outdoor repair r2 (review r2 P3-N7): the re-entry count restarts on every history reset of an outdoor session:
    the session start, an explicit reset(), an altitude-gated stretch in the middle of a re-entry and a refusal. An
    indoor session (no refusals) never holds (control: its proposals are unchanged)."""
    setUp, tick, start_hold = SessionHoldOnRefusal.setUp, G.AltitudeGating.tick, SessionHoldOnRefusal.start_hold

    def held_before_command(self, height=1.):
        held = 0
        while True:
            proposal, item = self.tick(height)
            if proposal['velocity_enu_mps'] != [0., 0., 0.]:
                return held
            self.assertTrue(proposal['hold'])
            held += 1

    def test_session_start_and_explicit_reset(self):
        from camera_ros2_session import REFUSAL_REENTRY_FRAMES
        self.assertEqual(self.session.reentry_hold_remaining, REFUSAL_REENTRY_FRAMES)
        self.assertEqual(self.held_before_command(), REFUSAL_REENTRY_FRAMES)
        self.assertEqual(self.session.reentry_hold_remaining, 0)
        self.session.reset()                                                  # review r2 S4: explicit reset
        self.assertEqual(self.session.reentry_hold_remaining, REFUSAL_REENTRY_FRAMES)
        self.k += 3                                                          # a new epoch: fresh stamps
        self.assertEqual(self.held_before_command(), REFUSAL_REENTRY_FRAMES)

    def test_altitude_gated_stretch_in_the_middle_of_a_reentry(self):
        from camera_ros2_session import REFUSAL_REENTRY_FRAMES
        self.start_hold()
        self.runtime.refuse = 'ground_not_found'
        self.assertTrue(self.tick(1.)[0]['hold'])
        self.runtime.refuse = None
        for _ in range(2):                                                   # 2 of the 5 re-entry frames
            self.assertEqual(self.tick(1.)[0]['velocity_enu_mps'], [0., 0., 0.])
        self.assertIsNone(self.tick(0.)[0])                                  # gated (review r2 S3)
        self.assertEqual(self.session.status, 'WAITING_ALTITUDE')
        self.assertEqual(self.session.reentry_hold_remaining, REFUSAL_REENTRY_FRAMES)
        self.runtime.events.clear()
        self.assertEqual(self.held_before_command(), REFUSAL_REENTRY_FRAMES)  # not 3: the count restarted
        self.assertEqual(self.runtime.events[0], 'reset')                    # with the fresh history

    def test_indoor_session_never_holds(self):
        from camera_ros2_session import RosPolicySession
        runtime = G.BandRuntime()
        session = RosPolicySession(runtime, F.ros_binding(Path(self.tmp.name) / 'fixture_indoor.txt'))
        self.assertEqual(session.reentry_hold_remaining, 0)
        self.session, self.runtime = session, runtime
        proposal, _ = self.tick(1.)
        self.assertEqual(proposal['velocity_enu_mps'], [.2, .1, 0.])
        self.assertNotIn('hold', proposal)
        self.assertIsNone(self.tick(0.)[0])
        self.assertEqual(session.reentry_hold_remaining, 0)
        self.assertEqual(self.tick(1.)[0]['velocity_enu_mps'], [.2, .1, 0.])


class NpzReplayRefusal(unittest.TestCase):
    def test_replay_writes_a_hold_line_and_resets_on_refusal(self):
        """Outdoor repair r2 (review r2 P3-N8): the NPZ replay applies the ROS session's re-entry hysteresis: the first
        REFUSAL_REENTRY_FRAMES accepted frames (a replay starts from a fresh history) and as many after every refused
        frame propose the zero hold, keep the policy's own command as policy_*, and record the applied zero hold."""
        N = R.REFUSAL_REENTRY_FRAMES
        refuse_at = N + 1                                                   # frames 0..N-1 held, frame N free

        class Fake:
            contract = outdoor_contract()
            def __init__(self):
                self.calls, self.resets, self.holds = [], 0, 0
            def reset(self):
                self.resets += 1
            def applied_zero_hold(self):
                self.holds += 1
            def step(self, frame, state, *, now_s):
                self.calls.append(dict(state))
                if len(self.calls) == refuse_at + 1:
                    raise R.FrameRefused('ground_not_found')
                return {'t': float(now_s), 'velocity_world_enu_mps': [1., 0., 0.], 'yaw_rate_rps': .2}
        n = 2 * N + 3
        data = {'camera_metadata': np.array(json.dumps({'intrinsics': F.TRAINED})), 'rgb': np.zeros((n, 2, 2, 3), np.uint8),
                'depth_z_m': np.zeros((n, 2, 2), np.float32), 'invalid': np.zeros((n, 2, 2), bool),
                'rgb_t': np.arange(n) * .1, 'depth_t': np.arange(n) * .1, 'pos': np.zeros((n, 3)), 'vel': np.zeros((n, 3)),
                'quat_xyzw': np.tile([0., 0., 0., 1.], (n, 1)), 'yaw_rate_rps': np.zeros(n), 'state_t': np.arange(n) * .1,
                'now_s': np.arange(n) * .1, 'z_valid': np.ones(n, bool), 'epv': np.full(n, .1), 'dead_reckoning': np.zeros(n, bool)}
        fake = Fake()
        with tempfile.TemporaryDirectory() as tmp:
            report = R.replay(fake, data, Path(tmp) / 'out.jsonl')
            lines = [json.loads(x) for x in (Path(tmp) / 'out.jsonl').read_text().splitlines()]
        self.assertEqual((report['frames'], report['refused_frames'], fake.resets), (n, 1, 1))
        self.assertEqual(report['reentry_held_frames'], 2 * N)
        self.assertEqual(fake.holds, 2 * N)
        held = [k for k, x in enumerate(lines) if x.get('hold') and 'frame_refused' not in x]
        self.assertEqual(held, list(range(N)) + list(range(refuse_at + 1, refuse_at + 1 + N)))
        self.assertEqual(lines[refuse_at]['frame_refused'], 'ground_not_found')
        self.assertEqual(lines[refuse_at]['velocity_world_enu_mps'], [0., 0., 0.])
        for k in held:
            self.assertEqual(lines[k]['velocity_world_enu_mps'], [0., 0., 0.])
            self.assertEqual(lines[k]['yaw_rate_rps'], 0.)
            self.assertEqual(lines[k]['policy_velocity_world_enu_mps'], [1., 0., 0.])
        self.assertEqual([lines[k]['reentry_hold_remaining'] for k in range(N)], list(range(N - 1, -1, -1)))
        for k in (N, n - 1):                                                  # controls: commands resume
            self.assertEqual(lines[k]['velocity_world_enu_mps'], [1., 0., 0.])
            self.assertNotIn('hold', lines[k])
        self.assertIs(fake.calls[0]['z_valid'], True)
        self.assertEqual(fake.calls[0]['epv'], .1)

    def test_indoor_replay_is_unchanged(self):
        """Control: an indoor contract never holds (no refusals, no re-entry fields in the lines or the report)."""
        class Fake:
            contract = indoor_contract()
            def step(self, frame, state, *, now_s):
                return {'t': float(now_s), 'velocity_world_enu_mps': [1., 0., 0.], 'yaw_rate_rps': .2}
            def applied_zero_hold(self):
                raise AssertionError('indoor replay must not hold')
        n = 3
        data = {'camera_metadata': np.array(json.dumps({'intrinsics': F.TRAINED})), 'rgb': np.zeros((n, 2, 2, 3), np.uint8),
                'depth_z_m': np.zeros((n, 2, 2), np.float32), 'invalid': np.zeros((n, 2, 2), bool),
                'rgb_t': np.arange(n) * .1, 'depth_t': np.arange(n) * .1, 'pos': np.zeros((n, 3)), 'vel': np.zeros((n, 3)),
                'quat_xyzw': np.tile([0., 0., 0., 1.], (n, 1)), 'yaw_rate_rps': np.zeros(n), 'state_t': np.arange(n) * .1,
                'now_s': np.arange(n) * .1}
        with tempfile.TemporaryDirectory() as tmp:
            report = R.replay(Fake(), data, Path(tmp) / 'out.jsonl')
            lines = [json.loads(x) for x in (Path(tmp) / 'out.jsonl').read_text().splitlines()]
        self.assertEqual(report, {'status': 'CPU_REPLAY_COMPLETE', 'frames': 3, 'commands_sent': 0,
                                  'gazebo': 'NOT_TESTED', 'hardware': 'NOT_TESTED'})
        self.assertTrue(all(x == {'t': x['t'], 'velocity_world_enu_mps': [1., 0., 0.], 'yaw_rate_rps': .2} for x in lines))



# ------------------------------------------------------------------------ real OAK-D replay tool (synthetic folder)
def write_recording(folder, frames, *, source='synthetic', up=True):
    """frames: [(scene kwargs, height, roll, pitch)] rendered by outdoor_scene; written in the replay folder format."""
    folder = Path(folder)
    (folder / 'frames').mkdir(parents=True)
    names, tape = [], None
    for k, (kw, height, roll, pitch) in enumerate(frames):
        f, _ = outdoor_scene(kw.pop('source', 'trained_equal'), height, roll, pitch, **kw)
        rot = F.body_rotation(roll, pitch)
        tape = height + float((rot @ np.array(F.MOUNT))[2])
        mm = np.where(f['invalid'], 0., np.round(np.asarray(f['depth_z_m'], np.float64) * 1000.))
        mm = np.where(mm > 65535, 0, mm).astype(np.uint16)
        up_flu = rot.T @ np.array([0., 0., 1.])
        arrays = {'rgb': f['rgb'], 'depth_mm': mm, 't': np.float64(10. + .1 * k)}
        if up:
            arrays['up_optical'] = np.array([-up_flu[1], -up_flu[2], up_flu[0]])
        else:
            arrays['roll_pitch_deg'] = np.array([roll, pitch])
        name = 'frames/%06d.npz' % k
        np.savez(folder / name, **arrays)
        names.append(name)
        intrinsics = f['intrinsics']
    meta = {'format': 'motar-oakd-outdoor-recording-v1', 'source': source, 'device': {'model': 'none: synthetic'},
            'intrinsics': intrinsics, 'camera_height_m': tape, 'camera_height_uncertainty_m': .02,
            'scene': 'synthetic outdoor_scene frames (test fixture, not evidence)', 'frames': names}
    (folder / 'recording.json').write_text(json.dumps(meta))
    return meta


class OakdOutdoorReplayTool(unittest.TestCase):
    def contract_file(self, directory):
        value = C._metadata(1, ROOT)
        value['sources'] = {rel: 'a' * 64 for rel in C.SOURCE_PATHS}
        value.update(C.outdoor_blocks())
        path = Path(directory) / 'outdoor_contract.json'
        path.write_text(json.dumps(value))
        return path, value

    def test_synthetic_folder_replays_with_refusals_and_a_summary(self):
        import oakd_outdoor_replay as T
        frames = [({'objects': [TREE_LINE]}, 1., 0., 0.), ({'objects': [TREE_LINE]}, 1., 2., 4.),
                  ({'objects': [TREE_LINE], 'ground': 'textureless'}, 1., 0., 0.),
                  ({'objects': [TREE_LINE, ('vcyl', .55, 0., .15, 0., 5., BARK)]}, 1., 0., -3.)]
        with tempfile.TemporaryDirectory() as tmp:
            write_recording(Path(tmp) / 'rec', [(dict(kw), h, r, p) for kw, h, r, p in frames])
            contract, value = self.contract_file(tmp)
            out = Path(tmp) / 'out'
            with contextlib.redirect_stdout(io.StringIO()) as stdout:
                code = T.main(['--contract', str(contract), '--recording', str(Path(tmp) / 'rec'), '--output', str(out),
                               '--no-verify-files'])
            self.assertEqual(code, 0)
            summary = json.loads((out / 'summary.json').read_text())
            rows = [json.loads(x) for x in (out / 'frames.jsonl').read_text().splitlines()]
            self.assertEqual(json.loads(stdout.getvalue())['evidence'], 'SYNTHETIC_NOT_EVIDENCE')
        self.assertEqual((summary['frames'], summary['accepted'], summary['refused']), (4, 3, 1))
        self.assertEqual(summary['refusal_reasons'], {'ground_insufficient_points': 1})
        self.assertEqual(summary['source'], 'synthetic')
        self.assertLess(summary['ground_height_error_vs_tape_m']['max_abs'], .03)
        self.assertEqual(rows[2]['refused'], 'ground_insufficient_points')
        self.assertEqual(rows[0]['near_columns'], 0)
        self.assertGreater(rows[0]['sky']['sky_pixels'], 0)
        self.assertGreater(rows[3]['near_columns'], 60)                         # the trunk 0.3 m ahead
        self.assertAlmostEqual(rows[1]['roll_deg'], 2., places=6)
        self.assertAlmostEqual(rows[1]['pitch_deg'], 4., places=6)
        self.assertEqual(summary['frames_with_near_columns'], 1)
        # Outdoor repair r2 (review r2 P3-N8): re-entry labels as in the ROS session (start of the recording and after
        # the refused frame 2); no policy runs in this tool.
        self.assertEqual(summary['reentry_hold_frames_after_reset'], R.REFUSAL_REENTRY_FRAMES)
        self.assertEqual([r.get('reentry_hold') for r in rows], [True, True, None, True])
        self.assertEqual((summary['reentry_held_frames'], summary['frames_free_to_command']), (3, 0))
        self.assertFalse(summary['contract_files_verified'])
        self.assertNotIn('depthai', sys.modules)

    def test_folder_format_is_validated(self):
        import oakd_outdoor_replay as T
        with tempfile.TemporaryDirectory() as tmp:
            meta = write_recording(Path(tmp) / 'rec', [({'objects': [TREE_LINE]}, 1., 0., 0.)], up=False)
            contract = dict(self.contract_file(tmp)[1])
            rows, summary = T.replay_recording(Path(tmp) / 'rec', contract)            # positive control
            self.assertEqual(summary['accepted'], 1)
            bad = []
            for change in ({'format': 'v0'}, {'source': 'real'}, {'camera_height_m': 0}, {'frames': ['/abs.npz']},
                           {'frames': []}, {'extra': 1}):
                bad.append(dict(meta, **change))
            for value in bad:
                (Path(tmp) / 'rec' / 'recording.json').write_text(json.dumps(value))
                with self.subTest(change=value), self.assertRaises(T.RecordingError):
                    T.replay_recording(Path(tmp) / 'rec', contract)
            (Path(tmp) / 'rec' / 'recording.json').write_text(json.dumps(meta))
            with np.load(Path(tmp) / 'rec' / meta['frames'][0]) as data:
                arrays = {k: data[k] for k in data.files}
            for label, change in (('depth float', dict(depth_mm=arrays['depth_mm'].astype(np.float32))),
                                  ('both attitudes', dict(up_optical=np.array([0., -1., 0.]))),
                                  ('rgb shape', dict(rgb=arrays['rgb'][:10]))):
                np.savez(Path(tmp) / 'rec' / meta['frames'][0], **dict(arrays, **change))
                with self.subTest(label=label), self.assertRaises(T.RecordingError):
                    T.replay_recording(Path(tmp) / 'rec', contract)
            with self.assertRaisesRegex(T.RecordingError, 'outdoor-profile'):
                T.replay_recording(Path(tmp) / 'rec', F.contract(ground=True))
        self.assertEqual(T.roll_pitch_from_up([0., -1., 0.]), (0., 0.))           # level: up is -y optical




# ------------------------------------------------------------------------------------------------ records
# SHA-256 and length (characters, trailing newlines stripped) of each README before the outdoor profile (pre_tree,
# 2026-11-13): the dated sections must append to that text without changing it. Embedded (review P3-6) instead of
# read from ~/motar_tmp.
PRE_README = {'ros2/motar_camera_policy/README.md':
              (16865, '4caab165e60a470b47f8bfb92f255707c52f1bfcb8a7ad3289c6336e862d5d91'),
              'docs/evidence/sitl_2026-11-05/camera_runtime/README.md':
              (16007, '5b5ce24b92269c3c69e8fc6a1e5f9a76ab71ad5f81f4097796cf4ff2ccea38f1')}


class OutdoorRecords(unittest.TestCase):
    def test_ros_readme_threshold_table_matches_the_contract(self):
        text = (ROOT / 'ros2/motar_camera_policy/README.md').read_text()
        section = text[text.index('## Outdoor profile (2026-11-13, opt-in)'):]
        rows = {}
        for line in section.splitlines():
            parts = [x.strip() for x in line.strip().strip('|').split('|')]
            if len(parts) == 3 and parts[0] in ('sky', 'ground_estimate'):
                rows[(parts[0], parts[1])] = json.loads(parts[2])
        expected = {('sky', k): C.threshold_value(C.SKY_OUTDOOR_DEFAULT, k) for k in tuple(C.SKY_BOUNDS) + C.SKY_FLAGS}
        expected.update({('ground_estimate', k): C.threshold_value(C.GROUND_ESTIMATE_OUTDOOR_DEFAULT, k)
                         for k in C.GROUND_ESTIMATE_BOUNDS})
        self.assertEqual(rows, expected)
        self.assertIs(rows[('sky', 'far_ground_rule')], False)
        for term in ('--profile outdoor', 'physical: false', 'FrameRefused', 'HOLD_FRAME_REFUSED', 'COM_OF_LOSS_T',
                     'estimator_vertical_fields_absent', 'tools/oakd_outdoor_replay.py', 'PROVISIONAL', 'up_optical',
                     'far_ground_rule', 'HOLD_REENTRY', 'outdoor_vertical_fields_not_forwarded', 'edge_veto_range_m',
                     'REFUSAL_REENTRY_FRAMES', 'expectedFailure'):
            self.assertIn(term, section)
        for reason in R.REFUSAL_REASONS:
            self.assertIn('`%s`' % reason, section)

    def test_other_numbers_in_the_new_sections_match_their_sources(self):
        import camera_ros2_px4 as P
        ros = (ROOT / 'ros2/motar_camera_policy/README.md').read_text()
        ros = ros[ros.index('## Outdoor profile (2026-11-13, opt-in)'):]
        evidence = (ROOT / 'docs/evidence/sitl_2026-11-05/camera_runtime/README.md').read_text()
        evidence = evidence[evidence.index('## Outdoor profile r1, 2026-11-13'):]
        self.assertEqual(C.OUTDOOR_CEILING_HEIGHT_M, 3.0)
        self.assertIn('virtual ceiling at 3.0 m', ros)
        self.assertIn('3.0 m virtual', evidence)
        self.assertEqual(C.outdoor_blocks()['ground']['margin_m'], .10)
        self.assertIn('the floor filter on', ros)
        self.assertEqual(C._metadata(1, ROOT)['camera']['translation_m'][2], .03)
        self.assertIn("PX4's camera height (z + 0.03 m)", ros)
        import camera_ros2_session as S
        self.assertEqual(S.REFUSAL_REENTRY_FRAMES, 5)
        self.assertIn('`REFUSAL_REENTRY_FRAMES` = 5', ros)
        obs = C._metadata(1, ROOT)['observation']
        self.assertAlmostEqual(S.REFUSAL_REENTRY_FRAMES * obs['control_dt_s'], obs['history_interval_s'])
        self.assertIn('0.5 s, one trained history interval', ros)
        veto = tuple(C.threshold_value(C.SKY_OUTDOOR_DEFAULT, k) for k in
                     ('edge_veto_range_m', 'edge_veto_span_deg', 'edge_veto_range_tolerance_m'))
        self.assertEqual(veto, (5.0, 70.0, 1.0))
        self.assertEqual(R.EDGE_VETO_DILATION_PX, 2)
        self.assertIn('`EDGE_VETO_DILATION_PX` = 2', ros)
        self.assertIn('range 5.0 m, span 70 deg, tolerance 1.0 m', evidence)
        self.assertIn('first %d accepted frames' % S.REFUSAL_REENTRY_FRAMES, evidence)
        self.assertEqual(C.OUTDOOR_CEILING_BOUNDS_M, (2.5, 6.0))
        self.assertIn('[2.5, 6.0] m', ros)
        # Repair r1: the edge-veto rationale (governor release distance, worst-case stop, a 2.4 m face at 2 m) and the
        # far-ground band width.
        env = C._metadata(1, ROOT)['runtime_env']                    # the trained riskcap governor (pinned record)
        release, free, brake = (float(env['NAVRL_SPEED_GOVERNOR_' + k]) for k in ('RELEASE_M', 'FREE_MPS', 'BRAKE_MPS2'))
        self.assertEqual(release, C.threshold_value(C.SKY_OUTDOOR_DEFAULT, 'edge_veto_range_m'))
        self.assertIn("5.0 m release distance", ros)
        self.assertAlmostEqual(free, 3.54, places=2)
        self.assertIn('3.54 m/s', ros)
        self.assertEqual(brake, P.GOVERNOR_BRAKE_MPS2)
        self.assertAlmostEqual(brake, 2.96, places=2)
        self.assertIn('2.96 m/s^2', ros)
        stop = free * .3 + free ** 2 / (2. * brake)
        self.assertAlmostEqual(stop, 3.2, delta=.05)
        self.assertIn('about 3.2 m', ros)
        self.assertLess(stop, C.threshold_value(C.SKY_OUTDOOR_DEFAULT, 'edge_veto_range_m'))
        span = 2. * math.degrees(math.atan(1.2 / (2. - F.MOUNT[0])))
        self.assertAlmostEqual(span, 64.6, places=1)
        self.assertIn('64.6 deg', ros)
        self.assertLess(span, C.threshold_value(C.SKY_OUTDOOR_DEFAULT, 'edge_veto_span_deg'))
        self.assertAlmostEqual(math.degrees(math.atan(camera_height(1.) / MAXZ)), 2.9, places=1)
        self.assertIn('2.9 deg at a 1.03 m camera', ros)
        self.assertEqual(P.MAX_AGE_S, .15)
        self.assertIn('revoked within about 0.15 s', ros)
        self.assertEqual(P.PX4_VERSION, 'v1.14.3')
        self.assertIn('`COM_OF_LOSS_T`, 1.0 s by default in v1.14.3', ros)
        params = Path('/home/fair/workspaces/px4_ws/PX4-Autopilot/src/modules/commander/commander_params.c')
        if params.is_file():                                   # the local PX4 v1.14.3 source the gateway pins
            self.assertIn('PARAM_DEFINE_FLOAT(COM_OF_LOSS_T, 1.0f);', params.read_text())

    def test_repair_r2_numbers_match_their_sources(self):
        """Outdoor repair r2: every new number in the two READMEs comes from a constant, the contract or OAKD_PRO_DOCS."""
        import camera_ros2_session as S
        ros = (ROOT / 'ros2/motar_camera_policy/README.md').read_text()
        ros = ros[ros.index('## Outdoor profile (2026-11-13, opt-in)'):]
        evidence = (ROOT / 'docs/evidence/sitl_2026-11-05/camera_runtime/README.md').read_text()
        evidence = evidence[evidence.index('**Repair r2 after the second independent review (2026-11-15).**'):]
        self.assertEqual(R.EDGE_VETO_MAX_INCIDENCE_DEG, 70.)
        self.assertIn('`EDGE_VETO_MAX_INCIDENCE_DEG` = 70 deg', ros)
        self.assertIn('up to a 70 deg incidence', evidence)
        edge = C.threshold_value(C.SKY_OUTDOOR_DEFAULT, 'edge_veto_range_m')
        self.assertEqual(edge, 5.)
        self.assertIn('any near valid return within 5.0 m', ros)
        self.assertIn('beyond 5.0 m ray range', ros)
        self.assertIn('within 5.0 m stays near', evidence)
        self.assertIn('valid edges lie beyond 5.0 m', evidence)
        depth = outdoor_contract()['depth']
        self.assertEqual((depth['window'], depth['density']), (5, .5))
        self.assertIn("blob_near's 5 x 5 window at density 0.5", ros)
        self.assertIs(S.REFUSAL_REENTRY_FRAMES, R.REFUSAL_REENTRY_FRAMES)
        self.assertIn('holds the first %d accepted frames after every history reset' % R.REFUSAL_REENTRY_FRAMES, evidence)
        for term in ('`RepairR2Residuals`', '`OakdProPresets`', 'REPAIR_r2.md', '`WAITING_ALTITUDE`', 'reentry_hold',
                     'NOT_MEASURED', 'accessed 2026-11-15'):
            self.assertIn(term, ros)
        # OAK-D Pro facts (OAKD_PRO_DOCS) and the presets derived from them
        for url in ('url', 'datasheet', 'depthai_header', 'depthai_bindings'):
            self.assertIn(OAKD_PRO_DOCS[url], ros)
        d = OAKD_PRO_DOCS
        self.assertIn('%s, global shutter, %d x %d, DFOV / HFOV / VFOV %g / %g / %g deg' % (
            (d['stereo']['sensor'],) + d['stereo']['resolution'] + d['stereo']['dfov_hfov_vfov_deg']), ros)
        for key, label in (('rgb_af', 'auto-focus'), ('rgb_ff', 'fixed-focus')):
            self.assertIn('| RGB, %s variant | %s, %d x %d, %g / %g / %g deg |' % (
                (label, d[key]['sensor']) + d[key]['resolution'] + d[key]['dfov_hfov_vfov_deg']), ros)
        self.assertIn('| Baseline | %g cm |' % (d['baseline_m'] * 100), ros)
        self.assertIn('%d nm each, up to %g W each' % (d['ir_wavelength_nm'], d['dot_projector_max_w']), ros)
        self.assertEqual(d['dot_projector_max_w'], d['flood_led_max_w'])
        self.assertIn('| IMU | %s |' % d['imu'], ros)
        self.assertIn('ideal range about %g-%g m' % d['ideal_range_m'], ros)
        self.assertIn('disparity search 0-%d px (extended 0-%d px)' % d['disparity_search_px'], ros)
        for name in d['api']:
            self.assertIn('`%s(intensity, mask=-1)`' % name, ros)
        af = oakd_pro_af_intrinsics()
        self.assertAlmostEqual(math.degrees(2 * math.atan(af['height'] / 2 / af['fy'])), 51.9, places=1)
        self.assertIn('about 51.9 deg VFOV', ros)
        self.assertIn('%g deg HFOV equals the trained HFOV' % d['rgb_af']['dfov_hfov_vfov_deg'][1], ros)
        self.assertEqual(d['rgb_af']['dfov_hfov_vfov_deg'][1], C._metadata(1, ROOT)['camera']['hfov_deg'])
        K = F.source_intrinsics('oakd_like')
        margin = K['cy'] - K['fy'] * 44.5 / F.TRAINED['fy']                           # source px above the top row
        self.assertAlmostEqual(margin, 7.8, places=1)
        self.assertIn('about 8 source pixels above and below', ros)
        self.assertIn('1014 x 760', ros)
        self.assertEqual((K['width'], K['height']), (1014, 760))
        self.assertIn('0.5 px disparity noise at a 2.0 m camera: refused', ros)

    def test_dated_sections_append_without_changing_earlier_text(self):
        for rel in ('ros2/motar_camera_policy/README.md', 'docs/evidence/sitl_2026-11-05/camera_runtime/README.md'):
            text = (ROOT / rel).read_text()
            heading = {'ros2/motar_camera_policy/README.md': '## Outdoor profile (2026-11-13, opt-in)',
                       'docs/evidence/sitl_2026-11-05/camera_runtime/README.md': '## Outdoor profile r1, 2026-11-13'}[rel]
            with self.subTest(rel=rel):
                self.assertEqual(text.count(heading), 1)
                length, digest = PRE_README[rel]
                import hashlib
                self.assertEqual(hashlib.sha256(text[:length].encode()).hexdigest(), digest, rel)
                self.assertIn(heading, text[length:])


# ------------------------------------------------------- outdoor repair r2 (independent review r2, 2026-11-15)
def panel(y0, y1, D, zb, zt, mat, edges='both', w=.03):
    """The review r2 panel (review_r2/probes/probe_common.panel): a frontal face at x = D (0.05 m deep) with optional
    valid vertical silhouette edge strips; +y = left in FLU; edges 'both', 'left', 'right' or 'none'."""
    objs, vm = [], valid_like(mat)
    if edges in ('both', 'left'):
        objs.append(('box', (D - .002, y1 - w, zb), (D + .05, y1, zt), vm))
    if edges in ('both', 'right'):
        objs.append(('box', (D - .002, y0, zb), (D + .05, y0 + w, zt), vm))
    return objs + [('box', (D, y0, zb), (D + .05, y1, zt), mat)]


def oblique_face(p0, p1, zb, zt, mat, step=.02):
    """The review r2 oblique face (q2_veto_geometry.oblique): invalid strips from p0 to p1 with valid 4 cm end posts."""
    (x0, y0), (x1, y1) = p0, p1
    n = max(2, int(math.ceil(math.hypot(x1 - x0, y1 - y0) / step)))
    xs, ys = np.linspace(x0, x1, n + 1), np.linspace(y0, y1, n + 1)
    objs = [('box', (min(xs[i], xs[i + 1]), min(ys[i], ys[i + 1]), zb), (max(xs[i], xs[i + 1]) + .04, max(ys[i], ys[i + 1]), zt),
             mat) for i in range(n)]
    vm = valid_like(mat)
    return [('box', (x - .02, y - .02, zb), (x + .02, y + .02, zt), vm) for (x, y) in (p0, p1)] + objs


def columns_between(p0, p1):
    az = [math.atan2(y, x - F.MOUNT[0]) for x, y in (p0, p1)]
    a = np.arctan(-(np.arange(160) - 79.5) / F.TRAINED['fx'])
    return np.flatnonzero((a >= min(az)) & (a <= max(az)))[2:-2]


def one_edge_columns(D):
    return np.intersect1d(face_cols(-.25, 2.9, D), np.arange(160))


def coverage_border(frame, px, sides=('top', 'bottom', 'left', 'right')):
    """The review r2 rectification border (q2 group f): coverage False on a px-wide source border."""
    cov = np.ones(frame['invalid'].shape, bool)
    for side, sl in (('left', np.s_[:, :px]), ('right', np.s_[:, -px:]), ('top', np.s_[:px]), ('bottom', np.s_[-px:])):
        if side in sides:
            cov[sl] = False
    f = dict(frame, coverage=cov, invalid=frame['invalid'] | ~cov)
    f['depth_z_m'] = np.where(cov, frame['depth_z_m'], 0.).astype(np.float32)
    return f


def no_veto_contract():
    c = outdoor_contract()
    c['sky']['edge_veto_range_m'] = C.provisional(.4000001)                       # no return is a near edge
    return c


class EdgeEnclosureRules(unittest.TestCase):
    """Outdoor repair r2 (review r2 P0-N1): edge_enclosure on synthetic masks. In a ROW one near edge closes the run when
    the other stop is the image border or any non-near stop; columns keep the two-edge rule; two near edges whose
    ranges differ by more than the tolerance enclose when their chord's incidence is <= EDGE_VETO_MAX_INCIDENCE_DEG."""

    def enclosed(self, si, edge):
        c = outdoor_contract()
        return R.edge_enclosure(si, edge, c)

    def test_one_edge_closes_a_row_against_the_border_or_any_non_near_stop(self):
        si = np.zeros((90, 160), bool)
        edge = np.full((90, 160), np.inf)
        si[20:30, 80:] = True                                    # an invalid run from column 80 to the right border
        edge[20:30, 79] = 2.                                     # a near valid edge on its left
        out = self.enclosed(si, edge)
        self.assertTrue(out[20:30, 80:].all())                   # closed by the image border
        si2, edge2 = si.copy(), edge.copy()
        si2[20:30, 150:] = False                                 # the run now stops at a far valid return (12 m)
        edge2[20:30, 150] = 12.
        self.assertTrue(self.enclosed(si2, edge2)[20:30, 80:150].all())
        si3 = si2.copy()                                         # ... or at a no-return / unimaged / floor pixel (+inf)
        edge3 = edge.copy()
        self.assertTrue(self.enclosed(si3, edge3)[20:30, 80:150].all())
        # Controls: no near edge on either side (two far stops, or border and border): nothing enclosed.
        edge4 = edge2.copy()
        edge4[20:30, 79] = 12.
        self.assertFalse(self.enclosed(si2, edge4)[20:30, 82:148].any())
        # Columns keep the two-edge rule: a run below a near edge reaching the image bottom is not enclosed.
        si5 = np.zeros((90, 160), bool)
        edge5 = np.full((90, 160), np.inf)
        si5[50:, 40:60] = True
        edge5[49, 40:60] = 2.
        self.assertFalse(self.enclosed(si5, edge5)[52:, 43:57].any())
        si5[50:, 39] = si5[50:, 60] = False                      # rows closed by near edges left and right: enclosed
        edge5[50:, 39] = edge5[50:, 60] = 2.
        self.assertTrue(self.enclosed(si5, edge5)[50:, 40:60].all())

    def test_two_near_edges_of_an_oblique_face_and_two_separate_objects(self):
        si = np.zeros((90, 160), bool)
        si[30:40, 60:100] = True
        edge = np.full((90, 160), np.inf)
        u = (np.array([59, 100]) - 79.5) / F.TRAINED['fx']
        # an oblique face: edge 1 at 2.0 m, edge 2 at 3.6 m (differ by 1.6 m > tolerance 1.0)
        edge[30:40, 59], edge[30:40, 100] = 2., 3.6
        pa = np.array([1., -u[0], 0.]) / math.hypot(1., u[0]) * 2.
        pb = np.array([1., -u[1], 0.]) / math.hypot(1., u[1]) * 3.6
        chord, middle = pb - pa, pa + pb
        incidence = math.degrees(math.asin(abs(chord @ middle) / np.linalg.norm(chord) / np.linalg.norm(middle)))
        self.assertLess(incidence, R.EDGE_VETO_MAX_INCIDENCE_DEG)
        self.assertTrue(self.enclosed(si, edge)[30:40, 60:100].all())
        # control: a chord seen nearly end-on (2.0 m and 4.9 m, 41 columns apart): two separate objects
        edge[30:40, 100] = 4.9
        pb = np.array([1., -u[1], 0.]) / math.hypot(1., u[1]) * 4.9
        chord, middle = pb - pa, pa + pb
        self.assertGreater(math.degrees(math.asin(abs(chord @ middle) / np.linalg.norm(chord) / np.linalg.norm(middle))),
                           R.EDGE_VETO_MAX_INCIDENCE_DEG)
        self.assertFalse(self.enclosed(si, edge)[30:40, 63:97].any())


class OneEdgeAndObliqueFaces(unittest.TestCase):
    """Outdoor repair r2 (review r2 P0-N1): the review's light-grey sensor-invalid faces in the altitude slice with ONE
    valid vertical edge in view (the other out of the field of view), and the oblique face whose edge ranges differ by
    1.60 m, are kept (all object columns < 10 m). Before (repair r1) they were freed as sky (0 of 84-98 columns)."""

    def kept(self, objs, cols, pitch=0., roll=0., contract=None, hook=None):
        f, _ = outdoor_scene('oakd_like', 1., roll, pitch, objects=objs)
        if hook:
            f = hook(f)
        out = run(f, outdoor_state(1., roll, pitch), contract or outdoor_contract())
        self.assertIsNone(out['refused'])
        return kept_columns(out, cols)

    def test_one_edge_faces_against_an_open_horizon(self):
        cam = camera_height(1.)
        lost_without_veto = 0
        for mat in (LGREY_INV, WHITE_INV, DARK_INV):
            for D in (1.5, 2., 3., 4.):
                cols = one_edge_columns(D)
                objs = panel(-.3, 3., D, cam + .03, cam + .30, mat)
                with self.subTest(rgb=mat['rgb'], D=D):
                    self.assertEqual(self.kept(objs, cols), len(cols))
                if mat is LGREY_INV:
                    lost_without_veto += len(cols) - self.kept(objs, cols, contract=no_veto_contract())
        self.assertGreater(lost_without_veto, 300)                                    # the veto is what keeps them

    def test_one_edge_faces_with_pose_and_a_tree_line(self):
        cam = camera_height(1.)
        for D in (1.5, 2., 3.):
            cols = one_edge_columns(D)
            for pitch, roll in ((5., 0.), (-5., 0.), (10., 0.), (0., 5.), (0., -5.)):
                with self.subTest(D=D, pitch=pitch, roll=roll):
                    self.assertEqual(self.kept(panel(-.3, 3., D, cam + .03, cam + .30, LGREY_INV), cols, pitch, roll), len(cols))
            for pitch, roll in ((0., 0.), (10., 0.), (0., 5.)):
                with self.subTest(D=D, pitch=pitch, roll=roll, background='tree'):
                    self.assertEqual(self.kept(panel(-.3, 3., D, cam + .03, cam + .55, WHITE_INV) + [tree(5.)], cols, pitch, roll),
                                     len(cols))

    def test_oblique_faces(self):
        cam = camera_height(1.)
        for p0, p1 in (((2.0, -.6), (3.6, .8)), ((1.6, -.6), (3.8, .9)), ((2.0, .6), (4.2, -.9))):
            cols = columns_between(p0, p1)
            objs = oblique_face(p0, p1, cam + .03, cam + .30, LGREY_INV)
            with self.subTest(p0=p0, p1=p1):
                self.assertEqual(self.kept(objs, cols), len(cols))
                self.assertLess(self.kept(objs, cols, contract=no_veto_contract()), len(cols) // 2)   # control

    def test_one_edge_face_beside_a_side_coverage_border(self):
        """Review r2 P1-N3 group f: a 30 px rectification border and roll 5 deg (before: 0 of 92 columns)."""
        cam = camera_height(1.)
        cols = one_edge_columns(2.)
        for mat in (WHITE_INV, LGREY_INV, DARK_INV):
            for pitch, roll in ((0., 5.), (5., 5.), (0., 0.)):
                with self.subTest(rgb=mat['rgb'], pitch=pitch, roll=roll):
                    self.assertEqual(self.kept(panel(-.3, 3., 2., cam + .03, cam + .30, mat) + [TREE_LINE], cols, pitch, roll,
                                               hook=lambda f: coverage_border(f, 30)), len(cols))


class OneEdgePhantomCost(unittest.TestCase):
    """Outdoor repair r2: what the one-edge rule costs (measured, not tuned; the safe direction). One frame cannot tell
    a face with one edge in view from sky beside a single near object, so the sky in the rows of any near valid return
    (within edge_veto_range_m, rising above the horizon) stays near. Open sky, a tree line, an open field with ranged far
    ground and objects beyond the edge range add nothing; invalid far ground stays the r1 phantom band."""

    def near(self, objs, h=1., **scene):
        f, _ = outdoor_scene('oakd_like', h, objects=objs, **scene)
        return run(f, outdoor_state(h), outdoor_contract())['near_cols']

    def test_sky_beside_a_single_near_object_is_near(self):
        for D in (1.5, 3., 4.5):
            with self.subTest(D=D):
                self.assertGreater(self.near([('vcyl', D, 0., .15, 0., 5., BARK), TREE_LINE]), 100)
        for D, expect_phantom in ((1.5, True), (3., True), (4.5, False), (6., False)):
            y = (D - F.MOUNT[0]) * math.tan(math.radians(29.))                         # a trunk at the image border
            with self.subTest(D=D, border=True):
                # at 4.5 m and 29 deg the trunk's silhouette edges lie at about 5.0 m ray range, beyond
                # edge_veto_range_m: no phantom (control), as at 6 m
                near = self.near([('vcyl', D, y, .15, 0., 5., BARK), TREE_LINE])
                if expect_phantom:
                    self.assertGreater(near, 100)
                else:
                    self.assertEqual(near, 0)
        self.assertGreater(self.near([('vcyl', 2., .3, .025, 0., 2.5, material((120., 90., 60.), 30., 'never')), TREE_LINE]), 100)

    def test_controls_without_a_near_object_above_the_horizon(self):
        cam = camera_height(1.)
        for label, objs, scene in (('open sky', [], {}), ('tree line', [TREE_LINE], {}),
                                   ('single trunk 6 m', [('vcyl', 6., 0., .15, 0., 5., BARK), TREE_LINE], {}),
                                   ('post 2 m, top 0.2 m below the camera',
                                    [('vcyl', 2., .3, .05, 0., cam - .2, material((120., 90., 60.), 30., 'never')), TREE_LINE], {})):
            with self.subTest(label=label):
                self.assertEqual(self.near(objs, **scene), 0)
        self.assertEqual(self.near([], far_ground='invalid'), 160)                     # unchanged r1 phantom band


class SkyConnectorTopBandOnly(unittest.TestCase):
    """Outdoor repair r2 (review r2 P1-N3): only unimaged pixels ABOVE the source camera's imaged region connect the sky
    flood (above its top FOV edge, or in the uncovered border above the first covered row of a source column; a side
    border column takes the top edge of the nearest covered column). Uncovered side and bottom borders never connect.
    The open sky is still freed at nose-down pitch with roll and a coverage border (the connector's purpose)."""

    def connectors(self, frame, pitch=0., roll=0.):
        c = outdoor_contract()
        base, _ = R.split_state(outdoor_state(1., roll, pitch), c)
        return R.project_rgbd(frame, base, c, unimaged=True)[6]

    def source_rows(self, pitch, roll):
        """[90, 160] source row (oakd_like) of each trained pixel's ray, from the level ray and the body attitude."""
        K = F.source_intrinsics('oakd_like')
        _, _, body, yaw, _, _ = R.validate_state(F.scene_state(1., roll, pitch))        # the trained camera is yaw-level
        level = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]]).T @ body
        tu, tv = np.meshgrid((np.arange(160) - 79.5) / F.TRAINED['fx'], (np.arange(90) - 44.5) / F.TRAINED['fy'])
        src = np.stack([np.ones_like(tu), -tu, -tv], axis=-1) @ level
        return -src[..., 2] / src[..., 0] * K['fy'] + K['cy']

    def test_side_and_bottom_borders_are_not_connectors(self):
        for pitch, roll in ((0., 0.), (0., 5.), (5., 5.), (10., -10.), (15., 10.)):
            f, _ = outdoor_scene('oakd_like', 1., roll, pitch)
            ys = self.source_rows(pitch, roll)
            plain = self.connectors(f, pitch, roll)
            sides = self.connectors(coverage_border(f, 30, ('left', 'right', 'bottom')), pitch, roll)
            full = self.connectors(coverage_border(f, 30), pitch, roll)
            with self.subTest(pitch=pitch, roll=roll):
                # every connector's ray lies above the source image's top edge (sides/bottom border: no exception) or,
                # with a 30 px top border, above its first covered row
                self.assertFalse((plain & (ys >= -.5)).any())
                self.assertFalse((sides & (ys >= -.5)).any())
                self.assertFalse((full & (ys >= 29.5)).any())
                self.assertTrue((full >= plain).all())
                if pitch == 0. and roll == 0.:
                    self.assertGreater(int(full.sum()), 0)                             # a top border adds connectors
        # the level path (identity rotation): only the top border rows of the sampled source column
        f, _ = outdoor_scene('oakd_like', 1.)
        self.assertFalse(self.connectors(coverage_border(f, 30, ('left', 'right', 'bottom'))).any())

    def test_open_sky_is_still_freed_at_nose_down_pitch_with_roll_and_a_border(self):
        c = outdoor_contract()
        for objs in ([], [TREE_LINE]):
            for pitch in (5., 10., 15., 20.):
                for roll in (0., 10., -10.):
                    f, _ = outdoor_scene('oakd_like', 1., roll, pitch, objects=objs)
                    f = coverage_border(f, 30)
                    out = run(f, outdoor_state(1., roll, pitch), c)
                    with self.subTest(tree=bool(objs), pitch=pitch, roll=roll):
                        self.assertEqual(out['near_cols'], 0)
        # control: without connectors the open sky is a near wall at 10 deg nose-down
        f, _ = outdoor_scene('oakd_like', 1., 0., 10.)
        rgb, si, nr, unimaged, plane, edge, _ = sky_inputs(f, outdoor_state(1., 0., 10.), c)
        self.assertGreater(int(unimaged.sum()), 0)
        self.assertEqual(R.apply_sky_rule(rgb, si, nr, np.zeros_like(unimaged), plane, c, edge)[2]['sky_pixels'], 0)


class RepairR2Residuals(unittest.TestCase):
    """Review r2 findings NOT fixed in repair r2 (no measured fix without new misses or an unbounded phantom; see
    REPAIR_r2.md). Each expected failure asserts the SAFE (or correct) outcome, so it flips if ever fixed."""

    @unittest.expectedFailure
    def test_fully_invalid_light_grey_panel_reaching_just_below_the_horizon_is_detected(self):
        """P0-N2: 1 m wide, camera -0.05..+0.30 m at 2 m, open horizon: 2 invalid trained rows below the horizon are
        fewer than blob_near's 5 x 5 window at density 0.5 needs (about 3); its rows above the horizon are freed as sky."""
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=panel(-.5, .5, 2., cam - .05, cam + .3, LGREY_INV, 'none'))
        self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-.4, .4, 2.)), 48)

    def test_controls_of_the_shallow_reach_class(self):
        cam = camera_height(1.)
        self.assertEqual((C.threshold_value(outdoor_contract()['depth'], 'window'), outdoor_contract()['depth']['density']),
                         (5, .5))
        for mat, below, D, expect in ((LGREY_INV, .05, 1.5, 66), (DARK_INV, .05, 2., 48), (LGREY_INV, .10, 2., 48)):
            f, _ = outdoor_scene('oakd_like', 1., objects=panel(-.5, .5, D, cam - below, cam + .3, mat, 'none'))
            with self.subTest(rgb=mat['rgb'], below=below, D=D):
                self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-.4, .4, D)), expect)

    @unittest.expectedFailure
    def test_face_whose_edges_lie_beyond_the_edge_range_is_detected(self):
        """P2-N4 (edge range): a light-grey face 4.4 m wide at 4.8 m; its valid edges lie at 5.19 m ray range, beyond
        edge_veto_range_m (5.0 m), so nothing encloses it (control: the same face at 4.6 m, edges at 4.92 m, is kept)."""
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=panel(-2.2, 2.2, 4.8, cam + .03, cam + .3, LGREY_INV))
        self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-2.1, 2.1, 4.8)), 106)

    def test_face_with_edges_inside_the_edge_range_is_kept(self):
        cam = camera_height(1.)
        f, _ = outdoor_scene('oakd_like', 1., objects=panel(-2., 2., 4.6, cam + .03, cam + .3, LGREY_INV))
        self.assertEqual(kept_columns(run(f, outdoor_state(1.), outdoor_contract()), face_cols(-1.9, 1.9, 4.6)), 100)

    @unittest.expectedFailure
    def test_flat_ground_with_half_pixel_disparity_noise_is_accepted(self):
        """P2-N6: flat textured ground seen from 2.0 m with 0.5 px disparity noise (ASSUMED; the OAK-D Pro's is
        NOT_MEASURED) is refused (ground_not_lowest_surface: the safe direction, a zero hold), not accepted."""
        f, _ = outdoor_scene('oakd_like', 2., objects=[TREE_LINE], depth_noise=.5)
        self.assertTrue(R.estimate_ground(f, outdoor_state(2.), outdoor_contract())['accepted'])

    def test_flat_ground_noise_refusal_is_named_and_the_low_noise_control_is_accepted(self):
        f, _ = outdoor_scene('oakd_like', 2., objects=[TREE_LINE], depth_noise=.5)
        report = R.estimate_ground(f, outdoor_state(2.), outdoor_contract())
        self.assertFalse(report['accepted'])
        self.assertIn(report['reason'], R.REFUSAL_REASONS)
        f, _ = outdoor_scene('oakd_like', 2., objects=[TREE_LINE])
        self.assertTrue(R.estimate_ground(f, outdoor_state(2.), outdoor_contract())['accepted'])


# OAK-D Pro facts used by the presets below and stated in ros2/motar_camera_policy/README.md (Luxonis documentation,
# accessed 2026-11-15; the user's unit and its variant are NOT_MEASURED). Machine-readable record for the README test.
OAKD_PRO_DOCS = {
    'url': 'https://docs.luxonis.com/hardware/products/OAK-D%20Pro',
    'datasheet': 'https://github.com/luxonis/depthai-hardware/blob/master/DM9098_OAK-D-Pro/Datasheet/OAK-D-Pro_Datasheet.pdf',
    'depthai_header': 'https://raw.githubusercontent.com/luxonis/depthai-core/v2.29.0/include/depthai/device/DeviceBase.hpp',
    'depthai_bindings': 'https://raw.githubusercontent.com/luxonis/depthai-python/v2.29.0.0/src/DeviceBindings.cpp',
    'stereo': {'sensor': 'OV9282', 'resolution': (1280, 800), 'dfov_hfov_vfov_deg': (89.5, 80., 55.), 'shutter': 'global'},
    'rgb_af': {'sensor': 'IMX378', 'resolution': (4056, 3040), 'dfov_hfov_vfov_deg': (78., 66., 54.), 'focus': 'auto'},
    'rgb_ff': {'sensor': 'IMX378', 'resolution': (4056, 3040), 'dfov_hfov_vfov_deg': (82., 69., 55.), 'focus': 'fixed'},
    'baseline_m': .075, 'ir_wavelength_nm': 940, 'dot_projector_max_w': 1., 'flood_led_max_w': 1., 'imu': 'BNO086',
    'ideal_range_m': (.8, 12.), 'disparity_search_px': (95, 190),
    'api': ('setIrLaserDotProjectorIntensity', 'setIrFloodLightIntensity'),
}


def oakd_pro_af_intrinsics():
    """PROVISIONAL preset of the auto-focus OAK-D Pro RGB at the delivered 1014 x 760 (ISP 1/4): HFOV 66 deg (docs); the
    4:3 sensor then gives VFOV 51.9 deg, while the docs list 54 deg for this variant (inconsistent; calibrate)."""
    f = 1014 / (2 * math.tan(math.radians(OAKD_PRO_DOCS['rgb_af']['dfov_hfov_vfov_deg'][1] / 2)))
    return {'width': 1014, 'height': 760, 'fx': f, 'fy': f, 'cx': 506.5, 'cy': 379.5}


class OakdProPresets(unittest.TestCase):
    """The existing 'oakd_like' preset (69 deg HFOV, 1014 x 760) matches the FIXED-FOCUS OAK-D Pro RGB (69/55 deg); the
    AUTO-FOCUS variant (66/54 deg in the docs) does not, so it gets its own preset here (oakd_like is unchanged)."""

    def test_oakd_like_matches_the_fixed_focus_variant(self):
        K = F.source_intrinsics('oakd_like')
        hfov = math.degrees(2 * math.atan(K['width'] / 2 / K['fx']))
        vfov = math.degrees(2 * math.atan(K['height'] / 2 / K['fy']))
        _, h, v = OAKD_PRO_DOCS['rgb_ff']['dfov_hfov_vfov_deg']
        self.assertAlmostEqual(hfov, h, places=6)
        self.assertLess(abs(vfov - v), .6)                                             # 54.5 vs the docs' rounded 55
        self.assertEqual((K['width'] * 4, K['height'] * 4), OAKD_PRO_DOCS['rgb_ff']['resolution'])

    def test_auto_focus_variant_does_not_cover_the_trained_fov(self):
        """66 deg equals the trained HFOV and 51.9 deg is below the trained 54 deg VFOV: the level path refuses the
        frame and oakd_camera.Rectifier.require_training_fov refuses the intrinsics (fail closed); on the tilted path
        the uncovered top band is a sky connector and the open sky stays free (positive control)."""
        import oakd_camera
        K = oakd_pro_af_intrinsics()
        self.assertLess(math.degrees(2 * math.atan(380 / K['fy'])), 54.)
        f, _ = outdoor_scene('oakd_like', 1., objects=[TREE_LINE])
        f = dict(f, intrinsics=K)
        with self.assertRaises(R.InputError):
            run(f, outdoor_state(1.), outdoor_contract())
        rect = object.__new__(oakd_camera.Rectifier)
        rect.k = np.array([[K['fx'], 0., K['cx']], [0., K['fy'], K['cy']], [0., 0., 1.]])
        rect.width, rect.height = K['width'], K['height']
        with self.assertRaises(oakd_camera.OakDInputError):
            rect.require_training_fov()
        ff = object.__new__(oakd_camera.Rectifier)                                      # control: the FF variant passes
        Kf = F.source_intrinsics('oakd_like')
        ff.k = np.array([[Kf['fx'], 0., Kf['cx']], [0., Kf['fy'], Kf['cy']], [0., 0., 1.]])
        ff.width, ff.height = Kf['width'], Kf['height']
        ff.require_training_fov()
        for pitch in (1., 5.):
            fa, _ = outdoor_scene('oakd_pro_af', 1., 0., pitch, objects=[TREE_LINE])
            with self.subTest(pitch=pitch):
                self.assertEqual(run(fa, outdoor_state(1., 0., pitch), outdoor_contract())['near_cols'], 0)


# ------------------------------------------------------------------- actual policy (seed 1), one fresh process
ACTUAL = r"""
import json, math, tempfile
from pathlib import Path
import numpy as np
import test_camera_runtime_outdoor_r1 as O
from camera_policy_contract import build_contract, estimator_height_assertion, outdoor_blocks
from camera_policy_runtime import CameraPolicyRuntime, FrameRefused

_dir = tempfile.TemporaryDirectory()
_fixture = Path(_dir.name) / 'estimator_height_fixture.txt'
_fixture.write_text('CPU test fixture: synthetic scenes put estimator z at the height above the floor. Not evidence.')
contracts = {'indoor': build_contract(1, estimator_height=estimator_height_assertion(_fixture)),
             'outdoor': build_contract(1, outdoor=outdoor_blocks())}
rt = CameraPolicyRuntime(contracts['outdoor'])

def step(frame, state, variant):
    rt.contract = contracts[variant]
    rt.reset()
    res = rt.step(dict(frame, rgb_t=1., depth_t=1.), dict(state, t=1.), now_s=1.)
    obs = np.asarray(res['observation'], np.float32)
    tok = obs[288:288 + 480].reshape(5, 8, 12)[-1]
    tokens = [(round(float(12. * (1. - t[7])), 3), round(math.degrees(math.atan2(t[1], t[0])), 2)) for t in tok[tok[:, 11] > .5]]
    return dict(tokens=tokens, near=res['near_pixels'], outdoor=res.get('outdoor'), v=res['velocity_vehicle_flu_mps'])

out = {}
f, _ = O.outdoor_scene('oakd_like', 1., objects=[O.TREE_LINE])
out['sky indoor'] = step(f, O.outdoor_state(1., vertical=False), 'indoor')
out['sky outdoor'] = step(f, O.outdoor_state(1.), 'outdoor')
f, _ = O.outdoor_scene('oakd_like', 1., 0., 5., objects=[O.TREE_LINE])
out['sky outdoor pitch 5'] = step(f, O.outdoor_state(1., 0., 5.), 'outdoor')
f, _ = O.outdoor_scene('oakd_like', 1., objects=[('vcyl', .55, 0., .15, 0., 5., O.BARK), O.TREE_LINE])
out['trunk outdoor'] = step(f, O.outdoor_state(1.), 'outdoor')
# Outdoor repair r1: a light-grey sensor-invalid panel at 1.5 m (camera +0.05..+0.55 m) with valid vertical edges.
cam = O.camera_height(1.)
f, _ = O.outdoor_scene('oakd_like', 1., objects=O.edge_strips(1.5, -.5, .5, cam + .05, cam + .55, O.LGREY_INV)
                       + [('box', (1.5, -.5, cam + .05), (1.55, .5, cam + .55), O.LGREY_INV), O.TREE_LINE])
out['panel twin outdoor'] = step(f, O.outdoor_state(1.), 'outdoor')
rt.contract = contracts['outdoor']
rt.reset()
f, _ = O.outdoor_scene('oakd_like', 1., objects=[O.TREE_LINE], ground='textureless')
try:
    rt.step(dict(f, rgb_t=1., depth_t=1.), dict(O.outdoor_state(1.), t=1.), now_s=1.)
    out['refusal'] = None
except FrameRefused as exc:
    out['refusal'] = dict(reason=exc.reason, failed=rt.failed, untouched=rt._last_stamps is None,
                          step_count=int(rt.perception.step_count))
print('OUTDOOR_R1_RESULT ' + json.dumps(out, allow_nan=False))
"""


class ActualPolicySeed1OutdoorR1(unittest.TestCase):
    """Real CAM-K2-v3 seed-1 checkpoint (read-only) through CameraPolicyRuntime.step, one fresh CPU process."""

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
        line = [x for x in done.stdout.splitlines() if x.startswith('OUTDOOR_R1_RESULT ')]
        cls.out = json.loads(line[-1][len('OUTDOOR_R1_RESULT '):])

    def test_open_sky_is_a_near_wall_indoors_and_free_space_outdoors(self):
        before = self.out['sky indoor']
        self.assertGreater(before['near'], 0)
        self.assertTrue([t for t in before['tokens'] if t[0] < .5], before['tokens'])       # the blocker
        for key in ('sky outdoor', 'sky outdoor pitch 5'):
            after = self.out[key]
            with self.subTest(key=key):
                self.assertEqual(after['near'], 0)
                self.assertFalse([t for t in after['tokens'] if t[0] < 10.], after['tokens'])
                self.assertGreater(after['outdoor']['sky']['sky_pixels'], 0)
                self.assertLess(abs(after['outdoor']['ground_plane']['height_error_m']), .02)

    def test_close_trunk_is_a_near_token_ahead(self):
        row = self.out['trunk outdoor']
        self.assertGreater(row['near'], 0)
        self.assertTrue([t for t in row['tokens'] if t[0] < .5 and abs(t[1]) <= 20.], row['tokens'])

    def test_light_grey_panel_twin_is_a_near_token_ahead(self):
        # Outdoor repair r1: the edge-enclosure veto keeps the bright panel near in the real policy observation.
        row = self.out['panel twin outdoor']
        self.assertGreater(row['near'], 0)
        self.assertGreater(row['outdoor']['sky']['edge_vetoed_pixels'], 0)
        self.assertTrue([t for t in row['tokens'] if t[0] < .5 and abs(t[1]) <= 20.], row['tokens'])

    def test_refused_frame_never_advances_the_history(self):
        self.assertEqual(self.out['refusal'], {'reason': 'ground_insufficient_points', 'failed': False,
                                               'untouched': True, 'step_count': 0})


if __name__ == '__main__':
    unittest.main()
