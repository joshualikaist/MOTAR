"""CPU CAM-K2-v3 inference from measured, aligned RGB-D and estimator state.

No simulator, synthetic depth model, target pose, LiDAR device, or MAVLink sender
is constructed. The 288 scan slots remain the trained no-return representation;
the production no_lidar adapter fills camera-visible governor bins from depth.
Run in a fresh process: production constants are fixed at import time.

The CLI replays an explicit NPZ input contract and writes proposed commands only.
Physical camera calibration, target appearance and closed-loop flight remain
separate qualifications. See the camera runtime record in the SITL evidence.
"""
import argparse
import importlib
import importlib.util
import inspect
import json
import math
import os
from pathlib import Path
import sys
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MAX_AGE_S = 0.15
MAX_SKEW_S = 0.025
PERIOD_TOLERANCE_S = 0.025
# Repair r1 (F4): on the tilted (z-buffer) path a trained pixel is sensor-invalid when at least this share
# of the source samples landing on it (valid points plus sensor-invalid rays) are sensor-invalid. The level
# path classifies a trained pixel by ONE nearest source sample, so its probability of being sensor-invalid
# is the local invalid share; a 0.5 majority reproduces that lookup in expectation. With about 53 OAK-D
# samples per trained pixel, iid 2 % holes reach a 0.5 share with negligible probability, so they stay
# valid (no new near blob), while sparse valid speckle on a dense invalid surface no longer hides it.
ZBUFFER_INVALID_SHARE = 0.5


class InputError(ValueError):
    """A rejected input must not advance the policy or perception history."""


def _finite(value, shape, name):
    a = np.asarray(value, dtype=np.float64)
    if a.shape != shape or not np.isfinite(a).all():
        raise InputError('%s must be finite with shape %s' % (name, shape))
    return a


def quaternion_rotation(q):
    """Estimator FLU-body -> ENU-world quaternion, xyzw; never HIL truth."""
    x, y, z, w = _finite(q, (4,), 'quat_xyzw')
    if abs(np.linalg.norm([x, y, z, w]) - 1.0) > 1e-3:
        raise InputError('quat_xyzw must be normalized')
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def validate_state(state):
    required = {'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 't'}
    if set(state) != required:
        raise InputError('state requires only estimator fields %s' % sorted(required))
    p = _finite(state['pos'], (3,), 'pos')
    v = _finite(state['vel'], (3,), 'vel')
    r = quaternion_rotation(state['quat_xyzw'])
    try:
        rate, t = float(state['yaw_rate_rps']), float(state['t'])
    except (ValueError, TypeError):
        raise InputError('invalid estimator timestamp/yaw rate') from None
    if not math.isfinite(rate) or not math.isfinite(t) or t < 0:
        raise InputError('invalid estimator timestamp/yaw rate')
    yaw = math.atan2(r[1, 0], r[0, 0])
    return p, v, r, yaw, rate, t


def _intrinsics(c):
    try:
        w, h = c['width'], c['height']
        if type(w) is not int or type(h) is not int or not (1 <= w <= 4096 and 1 <= h <= 4096):
            raise ValueError()
        fx, fy, cx, cy = (float(c[k]) for k in ('fx', 'fy', 'cx', 'cy'))
        if not all(math.isfinite(x) for x in (fx, fy, cx, cy)) or min(fx, fy) <= 0:
            raise ValueError()
        if not (-0.5 <= cx <= w-0.5 and -0.5 <= cy <= h-0.5):
            raise ValueError()
    except (KeyError, TypeError, ValueError):
        raise InputError('invalid calibrated intrinsics') from None
    return w, h, fx, fy, cx, cy


def _source_classes(frame, z, invalid, depth):
    """Split source pixels into sensor-invalid and no-return; every other pixel is valid.

    sensor_invalid: the sensor's own invalid return (explicit mask, NaN, or optical z < MinZ,
    including -inf). Only these may become a near surface under blob_near, as in training.
    no_return: geometrically not covered (optional `coverage` mask False, e.g. the zero border
    of an undistortion remap) or optical z > MaxZ (including +inf). These hold MaxZ and are
    never near. An uncovered pixel without a valid return (flagged, NaN, out of range) is
    no-return. An uncovered pixel WITH a valid return (finite MinZ <= z <= MaxZ, not flagged)
    contradicts the producer contract: the calibration does not describe this image, so the
    whole frame is refused (InputError) instead of erasing measured depth (repair r1, F3).
    """
    coverage = frame.get('coverage')
    if coverage is None:
        covered = np.ones(z.shape, bool)
    else:
        covered = np.asarray(coverage)
        if covered.shape != z.shape or covered.dtype != np.bool_:
            raise InputError('coverage must be a same-frame bool mask [H,W]')
    with np.errstate(invalid='ignore'):
        too_close = z < float(depth['minz_m'])
        too_far = z > float(depth['maxz_m'])
    flagged = invalid | np.isnan(z) | too_close
    contradicted = ~covered & ~flagged & ~too_far
    if contradicted.any():
        raise InputError('coverage is False at %d pixels that carry a valid depth return: the CameraInfo/'
                         'producer calibration does not match the delivered image; frame refused'
                         % int(contradicted.sum()))
    sensor_invalid = covered & flagged
    no_return = ~covered | (too_far & ~sensor_invalid)
    return sensor_invalid, no_return


def _splat_index(direction, tw, th, tfx, tfy, tcx, tcy, offset=False):
    """Nearest trained pixel of FLU directions/points [N,3] in front of the camera.

    With offset=True also returns each ray's squared distance (in pixels) from that pixel's centre."""
    front = direction[:, 0] > 1e-6
    d = direction[front]
    xf = -d[:, 1]/d[:, 0]*tfx+tcx
    yf = -d[:, 2]/d[:, 0]*tfy+tcy
    xx = np.floor(xf+.5).astype(int)
    yy = np.floor(yf+.5).astype(int)
    keep = (xx >= 0) & (xx < tw) & (yy >= 0) & (yy < th)
    selected = np.flatnonzero(front)[keep]
    if offset:
        return selected, yy[keep]*tw + xx[keep], (xf-xx)[keep]**2 + (yf-yy)[keep]**2
    return selected, yy[keep]*tw + xx[keep]


def project_rgbd(frame, state, contract):
    """Rectify a calibrated body-mounted camera into the trained yaw-level camera.

    RGB and optical-z depth must already be rectified and aligned to the SAME
    camera intrinsics. Level/coincident cameras use inverse nearest sampling and
    classify each trained pixel by its sampled source pixel.
    For a tilted/translated camera, measured 3-D points are reprojected with a
    nearest-depth z buffer. Sensor-invalid source pixels have no depth, so only
    their ray DIRECTION is reprojected (rotation only; the camera translation is
    ignored for these rays because it cannot be applied without a depth). Each
    trained pixel is classified by the share of the source samples (valid points
    plus sensor-invalid rays) that land on it (repair r1, F4): sensor-invalid if
    the invalid share is >= ZBUFFER_INVALID_SHARE (0.5), otherwise valid at the
    nearest valid depth; no-return if nothing lands (not imaged by the source
    camera: a tilt band, disocclusion or uncovered border). A sensor-invalid
    trained pixel carries the colour of the invalid ray nearest to its centre, as
    the level path carries its sampled source colour. No background or range is
    invented. This is an adapter, not sensor equivalence.

    Returns (rgb [1,3,H,W], ranges [1,H,W], sensor_invalid [1,H,W],
    no_return [1,H,W], cosine [H,W], report). Both masked kinds hold MaxZ range;
    only sensor_invalid is eligible for the blob_near rule.
    """
    _, _, body_r, yaw, _, _ = validate_state(state)
    if frame.get('aligned') is not True or frame.get('rectified') is not True:
        raise InputError('RGB/depth must be explicitly aligned and rectified')
    sw, sh, sfx, sfy, scx, scy = _intrinsics(frame['intrinsics'])
    cam = contract['camera']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    rgb = np.asarray(frame['rgb'])
    z = np.asarray(frame['depth_z_m'])
    invalid = np.asarray(frame['invalid'])
    if rgb.shape != (sh, sw, 3) or rgb.dtype != np.uint8:
        raise InputError('rgb must be aligned uint8 RGB [H,W,3]')
    if z.shape != (sh, sw) or not np.issubdtype(z.dtype, np.floating):
        raise InputError('depth_z_m must be floating optical metres [H,W]')
    if invalid.shape != z.shape or invalid.dtype != np.bool_:
        raise InputError('invalid must be a same-frame bool mask [H,W]')
    mount = _finite(frame['mount_flu_m'], (3,), 'mount_flu_m')
    optical = _finite(frame['camera_to_body_rotation'], (3, 3), 'camera_to_body_rotation')
    if not np.allclose(optical.T @ optical, np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(optical), 1):
        raise InputError('camera_to_body_rotation must be a proper FLU rotation')
    cy, sy = math.cos(yaw), math.sin(yaw)
    yaw_r = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.]])
    rel = yaw_r.T @ body_r
    rotation = rel @ optical
    offset = rel @ mount - np.asarray(cam['translation_m'])
    depth = contract['depth']
    minz, maxz = float(depth['minz_m']), float(depth['maxz_m'])
    src_invalid, src_no_return = _source_classes(frame, z, invalid, depth)
    if np.allclose(rotation, np.eye(3), atol=1e-8) and np.allclose(offset, 0, atol=1e-8):
        uu = (np.arange(tw)-tcx)/tfx*sfx + scx
        vv = (np.arange(th)-tcy)/tfy*sfy + scy
        # Reject unsupported FOV; silently padding it would invent observations.
        if min(uu.min(), vv.min()) < -0.5 or uu.max() > sw-0.5 or vv.max() > sh-0.5:
            raise InputError('source camera does not cover the trained FOV')
        xi = np.clip(np.floor(uu + .5).astype(int), 0, sw-1)
        yi = np.clip(np.floor(vv + .5).astype(int), 0, sh-1)
        out_rgb = rgb[yi[:, None], xi[None, :]].copy()
        out_z = z[yi[:, None], xi[None, :]].astype(np.float64)
        out_invalid = src_invalid[yi[:, None], xi[None, :]].copy()
        out_no_return = src_no_return[yi[:, None], xi[None, :]].copy()
        method = 'inverse-nearest'
    else:
        u, v = np.meshgrid((np.arange(sw)-scx)/sfx, (np.arange(sh)-scy)/sfy)
        good = ~(src_invalid | src_no_return)
        points = np.stack([z[good], -u[good]*z[good], -v[good]*z[good]], axis=-1)
        points = points @ rotation.T + offset
        selected, indices = _splat_index(points, tw, th, tfx, tfy, tcx, tcy)
        n_valid = np.bincount(indices, minlength=th*tw).reshape(th, tw)
        zs, colours = points[selected, 0], rgb[good][selected]
        order = np.lexsort((zs, indices))
        indices, zs, colours = indices[order], zs[order], colours[order]
        first = np.r_[True, indices[1:] != indices[:-1]] if len(indices) else np.zeros(0, bool)
        out_z = np.zeros((th, tw), np.float64)
        out_rgb = np.zeros((th, tw, 3), np.uint8)
        out_z.flat[indices[first]] = zs[first]
        out_rgb.reshape(-1, 3)[indices[first]] = colours[first]
        # Sensor-invalid rays: direction only (rotation), translation ignored.
        rays = np.stack([np.ones(int(src_invalid.sum())), -u[src_invalid], -v[src_invalid]], axis=-1)
        ray_selected, ray_indices, ray_offset = _splat_index(rays @ rotation.T, tw, th, tfx, tfy, tcx, tcy,
                                                             offset=True)
        n_invalid = np.bincount(ray_indices, minlength=th*tw).reshape(th, tw)
        samples = n_valid + n_invalid
        # F4: the share rule (see ZBUFFER_INVALID_SHARE); n_valid > 0 wherever a sampled pixel is not invalid.
        invalid_share = (samples > 0) & (n_invalid >= ZBUFFER_INVALID_SHARE * samples)
        hit = (samples > 0) & ~invalid_share
        # A sensor-invalid trained pixel takes the colour of the invalid ray nearest to its centre.
        ray_order = np.lexsort((ray_offset, ray_indices))
        ray_sorted = ray_indices[ray_order]
        ray_first = np.r_[True, ray_sorted[1:] != ray_sorted[:-1]] if len(ray_sorted) else np.zeros(0, bool)
        ray_colours = rgb[src_invalid][ray_selected][ray_order][ray_first]
        take = invalid_share.flat[ray_sorted[ray_first]]
        out_rgb.reshape(-1, 3)[ray_sorted[ray_first][take]] = ray_colours[take]
        # The level camera's own MinZ/MaxZ apply to reprojected valid points, as in training.
        out_invalid = invalid_share | (hit & (out_z < minz))
        out_no_return = (samples == 0) | (hit & (out_z > maxz))
        method = 'measured-point-z-buffer'
    masked = out_invalid | out_no_return
    u, v = np.meshgrid((np.arange(tw)-tcx)/tfx, (np.arange(th)-tcy)/tfy)
    cosine = 1 / np.sqrt(1+u*u+v*v)
    ranges = np.full((th, tw), maxz, np.float64)
    np.divide(out_z, cosine, out=ranges, where=~masked)
    # Match depth-model ray-range far clamping, retaining the independent masks.
    ranges = np.minimum(ranges, maxz).astype(np.float32)
    return (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None],
            out_invalid[None], out_no_return[None], cosine.astype(np.float32),
            {'projection': method, 'sensor_invalid_pixels': int(out_invalid.sum()),
             'no_return_pixels': int(out_no_return.sum()), 'pixels': th*tw,
             'coverage_mask': frame.get('coverage') is not None, 'synthetic_noise_applied': False})


def height_band(state, contract):
    """Whether the camera height lies inside the enabled floor/ceiling filter band; pure, no history.

    Camera height above the floor = estimator ENU z + the trained camera's z offset. With the floor
    filter on it must exceed ground.margin_m; with the ceiling filter on it must stay below
    ceiling.height_m - ceiling.margin_m. Outside the band every return at camera height would be
    removed as floor or ceiling, so remove_floor_and_ceiling refuses such a frame, and the ROS
    session waits (WAITING_ALTITUDE) without calling the runtime (e.g. on the ground before take-off).
    An enabled filter without the operator's estimator-height assertion (contract estimator_height,
    repair r1 F6) raises InputError instead: the frame fails closed, and the ROS session faults.
    Returns {'in_band', 'camera_height_m', 'reason'}.
    """
    from camera_policy_contract import height_reference_problem
    problem = height_reference_problem(contract)
    if problem:
        raise InputError(problem)
    p, _, _, _, _, _ = validate_state(state)
    ground, ceiling = contract['ground'], contract['ceiling']
    height = float(p[2]) + float(contract['camera']['translation_m'][2])
    reason = None
    if ground['enabled'] and not height > float(ground['margin_m']):
        reason = 'camera height above the floor does not exceed the ground margin'
    elif ceiling['enabled'] and not height < float(ceiling['height_m']) - float(ceiling['margin_m']):
        reason = 'camera height is not below the ceiling band'
    return {'in_band': reason is None, 'camera_height_m': height, 'reason': reason}


def remove_floor_and_ceiling(ranges, sensor_invalid, no_return, cosine, state, contract):
    """Floor and ceiling returns become no-return in the level trained camera.

    The training arena had neither a floor nor a ceiling (navrl_bars_env.py
    create_ground_plane = False), so both are removed by height; vertical walls and
    poles, whose hit heights lie between the two bands, are kept. Contract fields
    `ground` and `ceiling` enable each band; heights are above the floor, so the
    estimator ENU z must be the height above the floor, which the operator asserts in
    contract `estimator_height` (without it height_band refuses the frame, F6).
    Port of tools/sitl_s4_gazebo.py:remove_ground. S4 rotated body-frame depth rays
    by the full attitude; here the projected image is already yaw-level, so a
    trained pixel's ray-up component is -(v-cy)/fy * cosine and
    hit_z = pos_z + camera z + range * ray_up.
      * floor: a valid pixel with hit_z < ground.margin_m -> range MaxZ, no-return.
        Only with ground.invalid_plane_rule (opt-in, default OFF): a sensor-invalid
        pixel whose ray points below the horizon and meets the floor (z = 0) at
        level-camera optical z in [MinZ, MaxZ] -> no-return.
      * ceiling: a valid pixel with hit_z > ceiling.height_m - ceiling.margin_m ->
        range MaxZ, no-return. Only with ceiling.invalid_plane_rule (default OFF): a
        sensor-invalid pixel whose ray points above the horizon and meets the ceiling
        plane (z = height_m) at optical z in [MinZ, MaxZ] -> no-return.
    Repair r1 (F1): the invalid-pixel plane rule looks only at the row, never at the
    pixel's content, so it also erased a real obstacle closer than MinZ whose top lies
    a few cm below camera height (a box, a table edge). It is therefore opt-in. With it
    off, a textureless floor or ceiling inside the sensor range reads as a near surface
    again (an open item to decide from bench data). MinZ and MaxZ are compared with
    optical z, on which the sensor defines them. Exact float64 arithmetic; no distance
    matrix. Inputs are not mutated.
    """
    ground, ceiling = contract['ground'], contract['ceiling']
    info = {'ground': {'enabled': bool(ground['enabled']), 'margin_m': float(ground['margin_m']),
                       'invalid_plane_rule': ground['invalid_plane_rule'] is True,
                       'removed_valid_pixels': 0, 'removed_invalid_pixels': 0},
            'ceiling': {'enabled': bool(ceiling['enabled']), 'height_m': ceiling['height_m'],
                        'margin_m': float(ceiling['margin_m']),
                        'invalid_plane_rule': ceiling['invalid_plane_rule'] is True,
                        'removed_valid_pixels': 0, 'removed_invalid_pixels': 0}}
    if not ground['enabled'] and not ceiling['enabled']:
        return ranges, sensor_invalid, no_return, info
    p, _, _, _, _, _ = validate_state(state)
    cam, depth = contract['camera'], contract['depth']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    if ranges.shape != (1, th, tw) or cosine.shape != (th, tw):
        raise InputError('floor/ceiling removal needs the trained camera grid')
    height = float(p[2]) + float(cam['translation_m'][2])
    # Outside the band every return at camera height would read as floor or ceiling (same predicate
    # as height_band, which the ROS session checks first so that it never calls the runtime there).
    band = height_band(state, contract)
    if not band['in_band']:
        raise InputError(band['reason'])
    minz, maxz = float(depth['minz_m']), float(depth['maxz_m'])
    v = (np.arange(th, dtype=np.float64)-tcy)/tfy
    hit_z = height + ranges[0].astype(np.float64)*(-v[:, None]*cosine.astype(np.float64))
    si, nr = sensor_invalid[0], no_return[0]
    valid = ~(si | nr)
    removed_valid = np.zeros_like(valid)
    removed_invalid = np.zeros_like(si)
    if ground['enabled']:
        margin = float(ground['margin_m'])
        info['ground']['camera_height_m'] = height
        floor = valid & (hit_z < margin)
        floor_invalid = np.zeros_like(si)
        if ground['invalid_plane_rule'] is True:
            below = v > 0
            floor_z = np.full(th, -np.inf)
            floor_z[below] = height / v[below]
            floor_invalid = si & ((floor_z >= minz) & (floor_z <= maxz))[:, None]
        info['ground']['removed_valid_pixels'] = int(floor.sum())
        info['ground']['removed_invalid_pixels'] = int(floor_invalid.sum())
        removed_valid |= floor
        removed_invalid |= floor_invalid
    if ceiling['enabled']:
        top = float(ceiling['height_m'])
        limit = top - float(ceiling['margin_m'])
        info['ceiling']['camera_height_m'] = height
        roof = valid & (hit_z > limit)
        roof_invalid = np.zeros_like(si)
        if ceiling['invalid_plane_rule'] is True:
            above = v < 0
            ceiling_z = np.full(th, -np.inf)
            ceiling_z[above] = (top - height) / -v[above]
            roof_invalid = si & ((ceiling_z >= minz) & (ceiling_z <= maxz))[:, None]
        info['ceiling']['removed_valid_pixels'] = int(roof.sum())
        info['ceiling']['removed_invalid_pixels'] = int(roof_invalid.sum())
        removed_valid |= roof
        removed_invalid |= roof_invalid
    out = ranges.copy()
    out[0][removed_valid] = maxz
    return (out, (si & ~removed_invalid)[None], (nr | removed_valid | removed_invalid)[None], info)


def gazebo_frame(packet):
    """S4's RGB-D wire packet; ranges/lidar_t are deliberately never inspected.

    Repair r1 (F10), REP 117 semantics: Gazebo's +inf (no hit, e.g. the sky) stays z = +inf and
    unflagged, which the runtime classifies as z > MaxZ, i.e. no-return (never near). NaN, -inf and
    z <= 0 stay sensor-invalid. SITL camera results before and after this change are not comparable.
    """
    hfov = math.radians(87.)
    f = 160/(2*math.tan(hfov/2))
    z = np.asarray(packet['depth'])
    return {'rgb': packet['rgb'], 'depth_z_m': z, 'invalid': np.isnan(z) | np.isneginf(z) | (z <= 0),
            'rgb_t': packet['rgb_t'], 'depth_t': packet['depth_t'],
            'intrinsics': {'width': 160, 'height': 90, 'fx': f, 'fy': f, 'cx': 79.5, 'cy': 44.5},
            'aligned': True, 'rectified': True, 'mount_flu_m': [.1, 0, .03],
            'camera_to_body_rotation': np.eye(3)}


def _load(name, relative):
    spec = importlib.util.spec_from_file_location(name, str(ROOT / relative))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class CameraPolicyRuntime:
    """One sequential CPU policy/tracker session, without any device or sender."""

    def __init__(self, contract):
        from camera_policy_contract import validate_contract
        # Callers cannot change the checked geometry/environment after loading.
        contract = json.loads(json.dumps(contract, allow_nan=False))
        problems = validate_contract(contract)
        if problems:
            raise ValueError('; '.join(problems))
        # A prior F/perception import has already frozen different constants.
        if any(k == 'aerial_gym' or k.startswith('aerial_gym.') for k in sys.modules):
            raise RuntimeError('camera runtime requires a fresh process without aerial_gym imports')
        inherited = {k: v for k, v in os.environ.items() if k.startswith('NAVRL_')}
        collisions = [k for k, v in inherited.items() if contract['runtime_env'].get(k) != v]
        if collisions:
            raise ValueError('inherited NAVRL environment conflicts with contract: %s' % sorted(collisions))
        os.environ.update(contract['runtime_env'])
        self.contract = contract
        import torch
        self.torch = torch
        # Bare packages expose only simulator-free leaf modules, without Warp.
        for name, rel in (('aerial_gym', 'aerial_gym'), ('aerial_gym.task', 'aerial_gym/task'),
                          ('aerial_gym.task.navrl_task', 'aerial_gym/task/navrl_task'),
                          ('aerial_gym.config', 'aerial_gym/config'),
                          ('aerial_gym.config.task_config', 'aerial_gym/config/task_config')):
            mod = types.ModuleType(name)
            mod.__path__ = [str(ROOT / rel)]
            sys.modules[name] = mod
        sys.modules['aerial_gym'].AERIAL_GYM_DIRECTORY = str(ROOT)
        pm = importlib.import_module('aerial_gym.task.navrl_task.navrl_perception')
        self.ablation = importlib.import_module('aerial_gym.task.navrl_task.navrl_sensor_ablation')
        self.validity = importlib.import_module('aerial_gym.task.navrl_task.navrl_depth_validity')
        self.sg = importlib.import_module('aerial_gym.task.navrl_task.speed_governor')
        tc = importlib.import_module('aerial_gym.config.task_config.navrl_task_config').task_config
        tc.vision.detector_hfov_deg = contract['camera']['hfov_deg']
        tc.vision.detector_vfov_deg = contract['camera']['vfov_deg']
        self.ablation.install_perception(pm, 'no_lidar')
        if pm.STRUCTURED_OBS_DIM != 898 or (pm.VBEAMS, pm.HBEAMS) != (4, 72):
            raise RuntimeError('camera observation schema differs from trained 898-D')
        self.tc = tc
        self.perception = pm.NavRLPerceptionModule(num_envs=1, device=torch.device('cpu'),
                                                 cfg=tc.perception, step_dt=.1, camera_cfg=tc.vision)
        self.bearings = pm.lidar_bin_bearings(torch.device('cpu'))
        self.gov_cfg = self.sg.SpeedGovernorConfig.from_environ(os.environ)
        if (self.gov_cfg.mode != 'riskcap' or self.gov_cfg.yaw_cap_enabled
                or self.gov_cfg.lateral_channel_enabled or self.gov_cfg.open_width_enabled):
            raise RuntimeError('camera runtime supports the trained directional riskcap only')
        self.vfov = inspect.signature(self.sg.directional_lidar_clearance).parameters['vertical_fov_deg'].default
        audit = _load('camera_runtime_offline_audit', 'tools/navrl_reflection_offline_audit.py')
        ckpt = contract['checkpoint']['path']
        state = torch.load(ckpt, map_location='cpu', weights_only=False)
        schema = audit.read_checkpoint_schema(state)
        if schema['structured_obs_dim'] != 898:
            raise RuntimeError('checkpoint has wrong observation schema')
        exported = audit.apply_schema_environment(schema)
        # The shared audit also exports truncated-normal's default dmin for
        # squashed checkpoints, whose training snapshot correctly omits it.
        # It is not read by the trained squashed action model.
        relevant = {k: v for k, v in exported.items()
                    if not (k == 'NAVRL_TRUNCATED_DMIN' and schema['cfg_action_policy'] == 'squashed_gaussian')}
        if any(contract['runtime_env'].get(k) != v for k, v in relevant.items()):
            raise RuntimeError('checkpoint schema differs from recorded environment')
        del state
        original = torch.load
        def cpu_load(*a, **kw):
            kw.setdefault('map_location', 'cpu')
            return original(*a, **kw)
        torch.load = cpu_load
        try:
            self.player = audit.build_player(ckpt, schema, 'cpu', ROOT / 'aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml')
        finally:
            torch.load = original
        self.audit = audit
        self.reset()

    def reset(self):
        # Perception rolls histories during inference, producing inference-mode
        # tensors. Reset them under the same mode, including after failed ticks.
        with self.torch.inference_mode():
            self.perception.reset_idx(self.torch.tensor([0]))
        # Vectorized training resets individual environments while retaining its
        # global counter. This standalone reset starts a whole new history clock.
        self.perception.step_count = 0
        self.prev = self.torch.zeros((1, 4), dtype=self.torch.float32)
        self._last_stamps = None
        self._last_now = None
        self.failed = False

    def invalidate(self):
        """Stop reuse after the companion rejects a tick following inference."""
        self.failed = True

    def height_band(self, state):
        """height_band() with this runtime's contract; reads no history and changes nothing."""
        return height_band(state, self.contract)

    def _validate_timing(self, frame, state, now_s):
        try:
            stamps = np.array([frame['rgb_t'], frame['depth_t'], state['t']], np.float64)
            now = float(now_s)
        except (TypeError, ValueError, KeyError):
            raise InputError('missing/invalid shared-clock timestamps') from None
        if stamps.shape != (3,) or not np.isfinite(stamps).all() or not math.isfinite(now):
            raise InputError('timestamps must be finite shared-clock seconds')
        ages = now-stamps
        if np.any(stamps < 0) or np.any(ages < -1e-9) or np.any(ages > MAX_AGE_S+1e-9):
            raise InputError('stale/future RGB-D or estimator sample')
        if np.ptp(stamps) > MAX_SKEW_S+1e-9:
            raise InputError('RGB-D/estimator capture skew exceeds declared bound')
        if self._last_stamps is not None and np.any(stamps <= self._last_stamps):
            raise InputError('duplicate/regressing RGB-D or estimator sample')
        dt = float(self.contract['observation']['control_dt_s'])
        if self._last_now is not None and abs(now-self._last_now-dt) > PERIOD_TOLERANCE_S+1e-9:
            raise InputError('control period differs from trained 10 Hz; reset required')
        return stamps, now

    def goal_coloured(self, rgb, ranges):
        """The production colour rule on the projected frame: the perception's own segmenter score
        >= its pixel_threshold (0.55), exactly the first term of _detect_rgbd's mask. rgb [1,3,H,W] in
        [0, 1]; ranges [1,H,W] (MaxZ at masked pixels; the bootstrap head has zero depth weight)."""
        torch, perception = self.torch, self.perception
        with torch.no_grad():
            score = perception.segmenter(torch.from_numpy(np.ascontiguousarray(rgb)),
                                         torch.from_numpy(np.ascontiguousarray(ranges)), perception.max_camera_range)
        return (score >= perception.pixel_threshold).numpy()

    def preprocess(self, frame, state):
        """Measured frame -> the trained camera's RGB and depth tensors; no history is read or changed.

        Order: projection (sensor-invalid / no-return split) -> goal-coloured sensor-invalid pixels
        become no-return -> optional floor/ceiling removal -> blob_near on the sensor-invalid mask
        ONLY -> RGB blanking. No-return pixels hold MaxZ and are never turned into a near surface.
        Repair r1 (F5): a sensor-invalid pixel that the production colour rule (goal_coloured) scores
        as goal-coloured becomes no-return and keeps its RGB: it is not near, so it adds no phantom
        obstacle at the goal's bearing, and its MaxZ depth drops it from the goal mask through the
        detector's own depth < max range gate, so it cannot bias the goal range either. This is the
        training carve-out of target pixels without the range bias. Other sensor-invalid pixels are
        unchanged. Blanking (RGB 0) of the remaining sensor-invalid (hence every near) and no-return
        pixels keeps the colour detector from taking its range from a substituted MinZ surface.
        """
        rgb, ranges, invalid, no_return, cosine, projection = project_rgbd(frame, state, self.contract)
        goal = invalid & self.goal_coloured(rgb, ranges)
        invalid, no_return = invalid & ~goal, no_return | goal
        ranges, invalid, no_return, bands = remove_floor_and_ceiling(ranges, invalid, no_return, cosine,
                                                                     state, self.contract)
        torch = self.torch
        depth, near = self.validity.apply_mask(torch.from_numpy(ranges), torch.from_numpy(invalid),
                                              torch.from_numpy(cosine), min_z_m=self.contract['depth']['minz_m'],
                                              window=self.contract['depth']['window'], density=self.contract['depth']['density'])
        near_np = near.numpy()
        blank = (invalid | no_return | near_np) & ~goal
        rgb = np.where(blank[:, None], np.float32(0.), rgb).astype(np.float32)
        counts = {'sensor_invalid_pixels': int(invalid.sum()), 'no_return_pixels': int(no_return.sum()),
                  'near_pixels': int(near_np.sum()), 'goal_invalid_pixels': int(goal.sum()),
                  'ground_removed_pixels': bands['ground']['removed_valid_pixels']
                  + bands['ground']['removed_invalid_pixels'],
                  'ceiling_removed_pixels': bands['ceiling']['removed_valid_pixels']
                  + bands['ceiling']['removed_invalid_pixels'],
                  'height_filter': bands, 'rgb_blanked_pixels': int(blank.sum())}
        return torch.from_numpy(rgb), depth, counts, projection

    def step(self, frame, state, *, now_s):
        if self.failed:
            raise RuntimeError('runtime failed after state advance; reset required')
        p, v, _, yaw, rate, _ = validate_state(state)
        stamps, now = self._validate_timing(frame, state, now_s)
        rgb, depth, counts, projection = self.preprocess(frame, state)
        torch = self.torch
        T = lambda x: torch.as_tensor(x, dtype=torch.float32).reshape(1, -1)
        # Shape compatibility only, exactly as trained: no measured or rendered
        # LiDAR is accepted, looked up or used in perception/the governor.
        no_lidar = torch.full((1, 288), float(self.tc.lidar_max_range))
        qyaw = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
        self.failed = True
        with torch.inference_mode():
            obs, diag = self.perception.observe(rgb=rgb, depth=depth, lidar_m=no_lidar,
                drone_pos_w=T(p), drone_vel_w=T(v), vehicle_quat=T(qyaw),
                yaw_rate=torch.tensor([rate/float(self.tc.yaw_rate_max)]), previous_action=self.prev,
                max_velocity=self.tc.max_velocity, flight_altitude=self.tc.flight_altitude, training=False)
            if tuple(obs.shape) != (1, 898) or not bool(torch.isfinite(obs).all()):
                raise RuntimeError('nonfinite/wrong-shape actor observation')
            a = self.audit.deterministic_actions(self.player, obs).reshape(1, 4)
            if not bool(torch.isfinite(a).all()) or bool((a.abs() > 1.000001).any()):
                raise RuntimeError('invalid actor output')
            a = a.clamp(-1., 1.)
            cmd = a[:, :2] * float(self.tc.max_velocity)
            scan = self.ablation.governor_scan(self.perception, 'no_lidar', no_lidar.reshape(1, 4, 72),
                                              float(self.tc.lidar_max_range), self.vfov)
            half = self.sg.speed_dependent_half_width(self.gov_cfg, cmd.norm(dim=1), open_m=None)
            clearance = self.sg.directional_lidar_clearance(scan, self.bearings, cmd,
                max_range_m=float(self.tc.lidar_max_range), path_half_width_m=half,
                target_return_mask=self.perception.last_target_like)
            clearance = torch.where(self.ablation.out_of_fov_commands(self.perception, cmd),
                                    torch.zeros_like(clearance), clearance)
            governed, _ = self.sg.apply_speed_governor(cmd, clearance, self.gov_cfg, lateral_clearance_m=None)
            prev = torch.cat([(governed/float(self.tc.max_velocity)).clamp(-1., 1.), a[:, 2:]], dim=1)
            if not bool(torch.isfinite(governed).all()) or not bool(torch.isfinite(prev).all()):
                raise RuntimeError('invalid governed command')
        vx, vy = governed[0].tolist()
        self.prev = prev
        self._last_stamps, self._last_now, self.failed = stamps, now, False
        return {'t': now, 'velocity_world_enu_mps': [math.cos(yaw)*vx-math.sin(yaw)*vy,
                                                   math.sin(yaw)*vx+math.cos(yaw)*vy, 0.],
                'velocity_vehicle_flu_mps': [vx, vy, 0.],
                'yaw_rate_rps': float(a[0, 3])*float(self.tc.yaw_rate_max),
                'previous_action': prev[0].tolist(), 'actor_output': a[0].tolist(),
                'observation': obs[0].tolist(), 'camera_visible': bool(diag['camera_visible'][0]),
                'lidar_visible': bool(diag['lidar_visible'][0]),
                # depth_near_pixels is the historical name of near_pixels, kept for existing readers.
                'depth_near_pixels': counts['near_pixels'], 'near_pixels': counts['near_pixels'],
                'sensor_invalid_pixels': counts['sensor_invalid_pixels'],
                'no_return_pixels': counts['no_return_pixels'],
                'ground_removed_pixels': counts['ground_removed_pixels'],
                'ceiling_removed_pixels': counts['ceiling_removed_pixels'], 'height_filter': counts['height_filter'],
                'rgb_blanked_pixels': counts['rgb_blanked_pixels'],
                'goal_invalid_pixels': counts['goal_invalid_pixels'],
                'projection': projection, 'checkpoint_sha256': self.contract['checkpoint']['sha256'],
                'command_sent': False}


def replay(runtime, data, output):
    """Recorded inputs only; no target state or simulator is ever constructed."""
    meta = json.loads(str(data['camera_metadata'].item()))
    required = ('rgb', 'depth_z_m', 'invalid', 'rgb_t', 'depth_t', 'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 'state_t', 'now_s')
    # Optional per-frame source coverage (False = not imaged, e.g. undistortion border).
    optional = ('coverage',) if 'coverage' in data else ()
    counts = {len(data[k]) for k in required + optional}
    if len(counts) != 1 or next(iter(counts)) == 0:
        raise InputError('replay arrays must contain equal nonzero frame counts')
    count = 0
    with Path(output).open('x') as stream:
        for i in range(next(iter(counts))):
            frame = dict(meta, **{k: data[k][i] for k in ('rgb', 'depth_z_m', 'invalid', 'rgb_t', 'depth_t') + optional})
            state = {k: data[k][i] for k in ('pos', 'vel', 'quat_xyzw', 'yaw_rate_rps')}
            state['t'] = data['state_t'][i]
            result = runtime.step(frame, state, now_s=data['now_s'][i])
            stream.write(json.dumps(result, allow_nan=False) + '\n')
            count += 1
    return {'status': 'CPU_REPLAY_COMPLETE', 'frames': count, 'commands_sent': 0,
            'gazebo': 'NOT_TESTED', 'hardware': 'NOT_TESTED'}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--contract', required=True)
    ap.add_argument('--replay', required=True, help='aligned measured input NPZ, allow_pickle=False')
    ap.add_argument('--output', required=True, help='new JSONL path for proposed commands, never sent')
    args = ap.parse_args(argv)
    from camera_policy_contract import load_contract
    if Path(args.output).exists():
        raise ValueError('output exists; choose a fresh path')
    runtime = CameraPolicyRuntime(load_contract(args.contract))
    with np.load(args.replay, allow_pickle=False) as data:
        report = replay(runtime, data, args.output)
    print(json.dumps(report, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
