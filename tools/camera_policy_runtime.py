"""CPU CAM-K2-v3 and CAM-S2 inference from measured, aligned RGB-D and estimator state.

Runtime v1 (2026-11-24): a contract with lineage CAM-S2 (camera_policy_contract_cams2.py) takes its own branch:
min-valid projection with invalid depth = no return (project_rgbd_min_valid), the training's altitude and camera mount
(navrl_cams2.apply_config), the camsafe blind-direction cap on the published command and the trained delayed
previous-action channel (_cams2_command). Every other contract runs the code paths it ran before.

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
import functools
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
# Outdoor profile (2026-11-13, opt-in through the contract; see estimate_ground, apply_sky_rule). Fixed algorithm
# constants of the deterministic ground fit. They are thresholds in effect, chosen on synthetic scenes only, and
# are PROVISIONAL like the contract thresholds (outdoor repair r1, review P3-4); PROVISIONAL_RUNTIME_CONSTANTS lists
# them. The contract binds this file by SHA-256, so changing one requires a rebuilt contract.
VERTICAL_FIELDS = ('z_valid', 'epv', 'dead_reckoning')     # PX4 VehicleLocalPosition fields, outdoor state only
GROUND_SOURCE_PIXELS = 20000        # PROVISIONAL: source pixels are sampled on a regular stride down to about this many
GROUND_MAX_POINTS = 2000            # PROVISIONAL: candidate points kept (evenly spaced in raster order) for the RANSAC
GROUND_RANSAC_SEED = 0
GROUND_RANSAC_ITERATIONS = 512      # PROVISIONAL
GROUND_SEARCH_TILT_DEG = 30.0       # PROVISIONAL: hypotheses steeper than this are walls, not ground candidates
GROUND_MIN_SAMPLE_AREA_M2 = 0.01    # PROVISIONAL: degenerate 3-point samples are skipped
GROUND_MAX_PLANES = 3               # PROVISIONAL: sequential RANSAC: the dominant plane, then up to two rivals
GROUND_REFINE_ROUNDS = 2            # PROVISIONAL: total-least-squares refinements on the inliers
GROUND_REMOVE_FACTOR = 2.0          # PROVISIONAL: points within this x inlier_threshold_m of a found plane are
                                    # removed before the next sequential RANSAC pass
GROUND_SPLIT_MIN_POINTS = 30        # PROVISIONAL: fewest inliers in a third/half for the planarity check (fewer: refused)
EDGE_VETO_DILATION_PX = 2           # PROVISIONAL: the edge-enclosed set grows by this many 4-neighbour steps (a row whose
                                    # edge sample is missing, e.g. at the source FOV boundary, stays protected)
# Outdoor repair r2 (review r2 P0-N1). PROVISIONAL, chosen on synthetic scenes (REPAIR_r2.md sweeps): the review's
# oblique faces need 51-59 deg; 45-90 deg gave identical kept columns and phantom bearings in the r2 sweep.
EDGE_VETO_MAX_INCIDENCE_DEG = 70.0  # PROVISIONAL: two near edges whose ranges differ by more than the tolerance still
                                    # enclose when the chord between them is seen at most this far off its normal
# Outdoor repair r2 (review r2 P3-N7/N8): accepted outdoor frames held at zero after every history reset (session
# start or reset, an altitude-gated stretch, a refused frame) before a nonzero command; 5 frames = 0.5 s at the
# trained 10 Hz, one interval of the trained 0.5 s observation history. Shared by the ROS session and the replays.
REFUSAL_REENTRY_FRAMES = 5          # PROVISIONAL
# Runtime v1 (CAM-S2): a near-constant observation slot (trained running variance below the contract's
# near_constant_var) is reported as outside its trained value beyond this many normalised units.
NEAR_CONSTANT_TOLERANCE = 1.0
# Review r1 P3-1: STRUCTURAL slots are those the training fed one constant value on every sample: their running
# variance is the normaliser's floor (1 + mean^2) / count left by its (mean 0, var 1, count epsilon) initialisation
# (this relative tolerance), e.g. out-of-FOV scan bins = 1, zero obstacle velocity/height fields. A runtime value off
# the trained constant by more than STRUCTURAL_SLOT_TOLERANCE can only be a pipeline defect: the step is refused.
STRUCTURAL_VAR_REL_TOL = 1e-3
STRUCTURAL_SLOT_TOLERANCE = 1e-6
# Review r1 P1-1: relative slack on the source-versus-trained focal length comparison (float rounding only).
SOURCE_RESOLUTION_TOLERANCE = 1e-9
PROVISIONAL_RUNTIME_CONSTANTS = ('GROUND_SOURCE_PIXELS', 'GROUND_MAX_POINTS', 'GROUND_RANSAC_ITERATIONS',
                                 'GROUND_SEARCH_TILT_DEG', 'GROUND_MIN_SAMPLE_AREA_M2', 'GROUND_MAX_PLANES',
                                 'GROUND_REFINE_ROUNDS', 'GROUND_REMOVE_FACTOR', 'GROUND_SPLIT_MIN_POINTS',
                                 'EDGE_VETO_DILATION_PX', 'EDGE_VETO_MAX_INCIDENCE_DEG', 'REFUSAL_REENTRY_FRAMES')
# Named outdoor refusals, in reporting priority (estimator validity first, then the camera's ground evidence).
# Outdoor repair r1 (review P1-1) added the three wrong-surface checks: ground_inlier_spread_insufficient,
# ground_not_planar and a wider ground_ambiguous (any substantial rival horizontal plane, see estimate_ground).
REFUSAL_REASONS = ('estimator_vertical_fields_absent', 'estimator_z_invalid', 'estimator_epv_exceeds_limit',
                   'estimator_dead_reckoning', 'ground_insufficient_points', 'ground_not_found',
                   'ground_insufficient_support', 'ground_tilt_exceeds_tolerance', 'ground_not_lowest_surface',
                   'ground_height_disagrees_with_px4', 'ground_inlier_spread_insufficient', 'ground_not_planar',
                   'ground_ambiguous', 'verified_camera_height_outside_band')


class InputError(ValueError):
    """A rejected input must not advance the policy or perception history."""


class FrameRefused(InputError):
    """Outdoor profile only: this frame's ground could not be verified (reason is one of REFUSAL_REASONS).

    Raised before the policy or perception history advances. The ROS session answers it with an explicit
    zero-velocity hold proposal and restarts the history on the next accepted frame (camera_ros2_session.py);
    every other InputError still faults the session. report carries the ground-estimate details."""
    def __init__(self, reason, report=None):
        if reason not in REFUSAL_REASONS:
            raise ValueError('unknown refusal reason %r' % (reason,))
        self.reason, self.report = reason, report
        super().__init__('outdoor frame refused: %s' % reason)


def _is_outdoor(contract):
    return isinstance(contract, dict) and contract.get('profile') == 'outdoor'


# Unit B (2026-11-18): bounded refusal hold. The session fault's name (not a frame refusal: REFUSAL_REASONS name refused
# frames, this names the end of a session).
REFUSAL_HOLD_TIMEOUT = 'refusal_hold_timeout'
# Capture stamps are ROS seconds rebuilt from integer nanoseconds; a sum of 0.1 s steps may land 1e-15 s short.
HOLD_TIMEOUT_TOLERANCE_S = 1e-6


def refusal_hold_limit_s(contract):
    """Outdoor profile: the contract's PROVISIONAL ground_estimate.refusal_hold_max_s in seconds; None for every other
    contract (indoor contracts never refuse a frame, so they never hold). A missing or malformed outdoor value raises
    ValueError (fail closed: a session never starts with an unbounded hold)."""
    if not _is_outdoor(contract):
        return None
    from camera_policy_contract import GROUND_ESTIMATE_BOUNDS, PROVISIONAL
    block = contract.get('ground_estimate')
    item = block.get('refusal_hold_max_s') if isinstance(block, dict) else None
    low, high, _ = GROUND_ESTIMATE_BOUNDS['refusal_hold_max_s']          # inclusive at both ends
    value = item.get('value') if isinstance(item, dict) and set(item) == {'value', 'status'} else None
    if (not isinstance(item, dict) or item.get('status') != PROVISIONAL or isinstance(value, bool)
            or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high):
        raise ValueError('outdoor contract needs ground_estimate.refusal_hold_max_s = {"value": v in [%g, %g] s, '
                         '"status": "%s"}' % (low, high, PROVISIONAL))
    return float(value)


class RefusalHoldEpisode:
    """Outdoor profile: the current refusal-hold episode, shared by the ROS session and the replays.

    An episode is the run of consecutive ticks whose proposal is the explicit zero hold (a refused frame or a re-entry
    hold) without an intervening policy command; refusals inside it never restart it. Its duration is measured on the
    held proposals' capture stamps (seconds of the declared shared ROS clock): elapsed = this hold's capture stamp minus
    the episode's first one. hold() returns (elapsed_s, timed_out); timed_out is True once elapsed reaches limit_s
    (within HOLD_TIMEOUT_TOLERANCE_S), so held proposals span less than the limit. command() ends the episode; a tick
    that proposes nothing (altitude gating) neither extends nor ends it. Capture stamps must increase."""
    def __init__(self, limit_s):
        if limit_s is not None and (isinstance(limit_s, bool) or not isinstance(limit_s, (int, float))
                                    or not math.isfinite(limit_s) or limit_s <= 0):
            raise ValueError('refusal hold limit must be a positive number of seconds or None')
        self.limit_s = None if limit_s is None else float(limit_s)
        self.start_s = self.last_s = self.elapsed_s = None
        self.ticks = 0
        self.longest_s = 0.

    def hold(self, capture_s):
        capture = float(capture_s)
        if not math.isfinite(capture):
            raise ValueError('hold capture stamp must be finite')
        if self.limit_s is None:
            raise ValueError('a zero hold without a refusal-hold limit (indoor contracts never hold)')
        if self.last_s is not None and not capture > self.last_s:
            raise ValueError('hold capture stamps must increase')
        if self.start_s is None:
            self.start_s, self.ticks = capture, 0
        self.last_s, self.ticks = capture, self.ticks + 1
        self.elapsed_s = capture - self.start_s
        self.longest_s = max(self.longest_s, self.elapsed_s)
        return self.elapsed_s, self.elapsed_s >= self.limit_s - HOLD_TIMEOUT_TOLERANCE_S

    def command(self):
        self.start_s = self.elapsed_s = None
        self.ticks = 0

    @property
    def active(self):
        return self.start_s is not None


def split_state(state, contract):
    """(estimator state, PX4 vertical-validity fields or None).

    Outdoor profile: the state may carry z_valid, epv [m] and dead_reckoning (PX4 VehicleLocalPosition) beside
    the five estimator fields; they are split off here, and estimate_ground refuses the frame when they are
    absent. Every other contract returns the state unchanged, so extra keys are refused by validate_state as
    before."""
    if not _is_outdoor(contract) or not isinstance(state, dict):
        return state, None
    vertical = {k: state[k] for k in VERTICAL_FIELDS if k in state}
    return {k: v for k, v in state.items() if k not in VERTICAL_FIELDS}, (vertical or None)


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


def _first_covered_rows(coverage):
    """[W] index of the first covered row of each source column of a bool coverage [H, W]. A column with no covered
    pixel (an uncovered side border) takes the value of the nearest covered column, so its rays are 'above' only
    where they lie above the imaged region's top edge; with no covered pixel at all every ray is above (H)."""
    any_cov = coverage.any(axis=0)
    if not any_cov.any():
        return np.full(coverage.shape[1], coverage.shape[0])
    columns = np.flatnonzero(any_cov)
    top = coverage.argmax(axis=0)
    nearest = columns[np.clip(np.searchsorted(columns, np.arange(coverage.shape[1])), 0, len(columns) - 1)]
    left = columns[np.clip(np.searchsorted(columns, np.arange(coverage.shape[1])) - 1, 0, len(columns) - 1)]
    pick = np.where(np.abs(left - np.arange(coverage.shape[1])) < np.abs(nearest - np.arange(coverage.shape[1])),
                    left, nearest)
    return top[pick]


def project_rgbd(frame, state, contract, *, unimaged=False):
    """Rectify a calibrated body-mounted camera into the trained yaw-level camera.

    unimaged=True (outdoor sky rule only) appends a seventh element: the [H,W] mask of trained pixels the
    source camera did not image ABOVE its imaged region, the sky rule's connectors (level path: coverage False
    above the first covered row of the sampled source column; tilted path: no sample landed AND the ray lies above
    the source camera's top FOV edge or in that uncovered top border; outdoor repairs r1 and r2).
    The six default outputs are computed identically either way.

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
        if unimaged:
            coverage = frame.get('coverage')
            if coverage is None:
                out_unimaged = np.zeros((th, tw), bool)
            else:
                # Outdoor repair r2 (review r2 P1-N3): only the uncovered border ABOVE the covered region of each
                # source column (the top band); uncovered side and bottom borders are never sky connectors.
                cov = np.asarray(coverage)
                top = _first_covered_rows(cov)
                out_unimaged = ~cov[yi[:, None], xi[None, :]] & (yi[:, None] < top[xi][None, :])
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
        if unimaged:
            # Outdoor repair r1: only trained pixels whose ray lies OUTSIDE the source camera's field of view (or
            # on its uncovered border) count as unimaged; a pixel inside it that no sample reached (a disocclusion
            # or sampling gap) does not, so it never connects the sky rule. Outdoor repair r2 (review r2 P1-N3):
            # only rays ABOVE the source camera's imaged region (above its top FOV edge, or in the uncovered border
            # above the first covered row of that source column) count: the band a nose-down camera does not image.
            # Rays beyond a side or below the bottom edge, and uncovered side/bottom borders, are not connectors.
            tu, tv = np.meshgrid((np.arange(tw)-tcx)/tfx, (np.arange(th)-tcy)/tfy)
            src = np.stack([np.ones_like(tu), -tu, -tv], axis=-1) @ rotation      # level ray -> source camera FLU
            ahead = src[..., 0] > 1e-9
            with np.errstate(divide='ignore', invalid='ignore'):
                xs = np.where(ahead, -src[..., 1]/np.where(ahead, src[..., 0], 1.)*sfx + scx, -1.)
                ys = np.where(ahead, -src[..., 2]/np.where(ahead, src[..., 0], 1.)*sfy + scy, -1.)
            inside = ahead & (xs >= -.5) & (xs <= sw-.5) & (ys >= -.5) & (ys <= sh-.5)
            coverage = frame.get('coverage')
            cov = np.ones((sh, sw), bool) if coverage is None else np.asarray(coverage)
            xi = np.clip(np.floor(xs + .5).astype(int), 0, sw-1)
            if coverage is not None:
                yi = np.clip(np.floor(ys + .5).astype(int), 0, sh-1)
                inside &= cov[yi, xi]
            above = np.where(ahead, ys < _first_covered_rows(cov)[xi] - .5, src[..., 2] > 0.)
            out_unimaged = (samples == 0) & ~inside & above
        method = 'measured-point-z-buffer'
    masked = out_invalid | out_no_return
    u, v = np.meshgrid((np.arange(tw)-tcx)/tfx, (np.arange(th)-tcy)/tfy)
    cosine = 1 / np.sqrt(1+u*u+v*v)
    ranges = np.full((th, tw), maxz, np.float64)
    np.divide(out_z, cosine, out=ranges, where=~masked)
    # Match depth-model ray-range far clamping, retaining the independent masks.
    ranges = np.minimum(ranges, maxz).astype(np.float32)
    result = (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None],
              out_invalid[None], out_no_return[None], cosine.astype(np.float32),
              {'projection': method, 'sensor_invalid_pixels': int(out_invalid.sum()),
               'no_return_pixels': int(out_no_return.sum()), 'pixels': th*tw,
               'coverage_mask': frame.get('coverage') is not None, 'synthetic_noise_applied': False})
    if unimaged:
        return result + (out_unimaged,)
    return result


def _is_cams2(contract):
    return isinstance(contract, dict) and contract.get('lineage') == 'CAM-S2'


def downscale_min_valid_2x2(frame, minz_m=0., maxz_m=math.inf):
    """Runtime v1 (CAM-S2): a [2H, 2W] aligned RGB-D frame -> [H, W] by 2x2 blocks, nearest valid depth.

    Each output pixel takes the smallest optical z among the block's VALID samples (finite, not flagged, MinZ/MaxZ are
    applied later by the projection) and the RGB of that sample; a block without a valid sample is invalid (flagged)
    and takes the RGB of its top-left sample. Valid means finite, not flagged and minz_m <= z <= maxz_m (the real
    OAK-D reports 0 outside its range, so the defaults only drop zeros). Intrinsics follow the block centres: f/2 and (c - 0.5)/2. coverage, if
    given, is True where any sample of the block is covered. This is the 'nearest valid depth' reduction the vehicle
    side proposed for its 624x468 stream; project_rgbd_min_valid applies the same min-over-valid rule after the
    reprojection into the level trained camera, and the two agree exactly for a level camera whose intrinsics are
    exactly twice the trained ones (tests/test_camera_runtime_cams2.py). The runtime itself always reprojects from the
    full-resolution source: the real principal point (cy 246.7 for 624x468) is not the trained one (233.5 after
    scaling), and a tilted vehicle needs the reprojection; doing the block reduction first would leave reprojection
    holes, which this lineage reads as free space."""
    z = np.asarray(frame['depth_z_m'], dtype=np.float64)
    invalid = np.asarray(frame['invalid'])
    rgb = np.asarray(frame['rgb'])
    h2, w2 = z.shape
    if h2 % 2 or w2 % 2 or rgb.shape != (h2, w2, 3) or invalid.shape != z.shape or invalid.dtype != np.bool_:
        raise InputError('2x2 reduction needs an even-sized aligned frame')
    h, w = h2 // 2, w2 // 2
    with np.errstate(invalid='ignore'):
        valid = ~invalid & np.isfinite(z) & (z > 0) & (z >= float(minz_m)) & (z <= float(maxz_m))
    blocks = np.where(valid, z, np.inf).reshape(h, 2, w, 2).transpose(0, 2, 1, 3).reshape(h, w, 4)
    pick = np.argmin(blocks, axis=2)
    best = np.take_along_axis(blocks, pick[..., None], axis=2)[..., 0]
    any_valid = np.isfinite(best)
    pick = np.where(any_valid, pick, 0)
    colours = rgb.reshape(h, 2, w, 2, 3).transpose(0, 2, 1, 3, 4).reshape(h, w, 4, 3)
    out = dict(frame)
    out['depth_z_m'] = np.where(any_valid, best, 0.).astype(np.float32)
    out['invalid'] = ~any_valid
    out['rgb'] = np.take_along_axis(colours, pick[..., None, None], axis=2)[:, :, 0].astype(np.uint8)
    if frame.get('coverage') is not None:
        out['coverage'] = np.asarray(frame['coverage']).reshape(h, 2, w, 2).any(axis=(1, 3))
    k = frame['intrinsics']
    out['intrinsics'] = {'width': w, 'height': h, 'fx': float(k['fx']) / 2., 'fy': float(k['fy']) / 2.,
                         'cx': (float(k['cx']) - .5) / 2., 'cy': (float(k['cy']) - .5) / 2.}
    return out


def project_rgbd_min_valid_reference(frame, state, contract):
    """CAM-S2 projection: calibrated body-mounted RGB-D -> the level trained camera, nearest valid depth per pixel.

    Training semantics (navrl_depth_model.py step 3, no blob_near): a pixel whose depth is invalid (z < MinZ, z > MaxZ,
    a hole) holds perception.max_camera_range, i.e. NO RETURN for every consumer. Here every valid source sample
    (finite, not flagged, MinZ <= optical z <= MaxZ, covered) is moved into the level camera by the measured attitude
    and mount (rotation + translation, as the CAM-K2 z-buffer path) and splatted to its nearest trained pixel; a
    trained pixel takes the SMALLEST level-camera optical z among the samples that land on it and lie inside
    [MinZ, MaxZ] of the level camera, and the colour of that sample. A trained pixel without such a sample is
    no-return: either only sensor-invalid rays landed on it (reported as sensor_invalid, colour of the invalid ray
    nearest its centre) or nothing did (not imaged: tilt band, disocclusion, uncovered border; colour 0). Both hold
    MaxZ range. No range is invented and nothing is turned into a near surface. Returns the six outputs of
    project_rgbd; ranges are ray ranges z / cos clamped to MaxZ (the depth model's far clamp).

    Faster ports (vehicle side, 2026-11-24): the source camera is body-fixed and the trained camera is level, so the
    source-pixel -> trained-pixel map depends on the current roll and pitch (yaw drops out) and, through the mount
    offset, slightly on depth. A precomputed lookup table is therefore exact for ONE attitude only. Exact options: a
    vectorised or GPU version of this same arithmetic (rotation, translation, nearest-pixel splat, min-z per pixel,
    the same refusals). Approximate option: tables per roll/pitch bin, whose output must then be checked against this
    function with a stated bound on the pixel-class mismatch. Golden pairs and a checker:
    ~/motar_tmp/runtime_v1_20261124/golden/ (check_golden.py)."""
    _, _, body_r, yaw, _, _ = validate_state(state)
    if frame.get('aligned') is not True or frame.get('rectified') is not True:
        raise InputError('RGB/depth must be explicitly aligned and rectified')
    sw, sh, sfx, sfy, scx, scy = _intrinsics(frame['intrinsics'])
    cam = contract['camera']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    # Review r1 P1-1: a source coarser than the trained camera (e.g. the vehicle side's 3x3 decimation to 208x156,
    # 158 px/rad against the trained 237 px/rad) leaves trained pixels without a sample, which this lineage would read
    # as free space. Refused; the exact trained resolution (312x234) and finer sources (624x468) are accepted.
    if sfx < tfx * (1. - SOURCE_RESOLUTION_TOLERANCE) or sfy < tfy * (1. - SOURCE_RESOLUTION_TOLERANCE):
        raise InputError('source intrinsics fx=%.3f fy=%.3f are coarser than the trained camera fx=%.3f fy=%.3f; '
                         'feed the aligned stream at (at least) the trained angular resolution' % (sfx, sfy, tfx, tfy))
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
    u, v = np.meshgrid((np.arange(sw)-scx)/sfx, (np.arange(sh)-scy)/sfy)
    good = ~(src_invalid | src_no_return)
    zz = z[good].astype(np.float64)
    points = np.stack([zz, -u[good]*zz, -v[good]*zz], axis=-1) @ rotation.T + offset
    selected, indices = _splat_index(points, tw, th, tfx, tfy, tcx, tcy)
    zs, colours = points[selected, 0], rgb[good][selected]
    level_valid = (zs >= minz) & (zs <= maxz)
    indices, zs, colours = indices[level_valid], zs[level_valid], colours[level_valid]
    order = np.lexsort((zs, indices))
    indices, zs, colours = indices[order], zs[order], colours[order]
    first = np.r_[True, indices[1:] != indices[:-1]] if len(indices) else np.zeros(0, bool)
    hit = np.zeros(th*tw, bool)
    hit[indices[first]] = True
    hit = hit.reshape(th, tw)
    out_z = np.zeros((th, tw), np.float64)
    out_rgb = np.zeros((th, tw, 3), np.uint8)
    out_z.flat[indices[first]] = zs[first]
    out_rgb.reshape(-1, 3)[indices[first]] = colours[first]
    # Sensor-invalid rays (direction only) mark pixels the camera imaged without a valid return.
    rays = np.stack([np.ones(int(src_invalid.sum())), -u[src_invalid], -v[src_invalid]], axis=-1)
    ray_selected, ray_indices, ray_offset = _splat_index(rays @ rotation.T, tw, th, tfx, tfy, tcx, tcy, offset=True)
    rayed = np.zeros(th*tw, bool)
    rayed[ray_indices] = True
    # Review r1 P1-1: a trained pixel whose centre ray lies inside the covered source image (one source pixel of
    # margin), that received no sample at all and lies between sampled pixels in its row or column, is a sampling
    # hole (a source too coarse for this attitude), not an absence of return. The frame is refused rather than reading
    # the hole as free space. Measured on a flat 5 m scene: 624x468 has no hole up to 30 deg pitch or 20 deg roll; a
    # 312x234 source has holes from 2.5 deg pitch (refused) and none when level.
    tu, tv = np.meshgrid((np.arange(tw)-tcx)/tfx, (np.arange(th)-tcy)/tfy)
    src = np.stack([np.ones_like(tu), -tu, -tv], axis=-1) @ rotation          # level ray -> source camera FLU
    ahead = src[..., 0] > 1e-9
    with np.errstate(divide='ignore', invalid='ignore'):
        xs = np.where(ahead, -src[..., 1]/np.where(ahead, src[..., 0], 1.)*sfx + scx, -10.)
        ys = np.where(ahead, -src[..., 2]/np.where(ahead, src[..., 0], 1.)*sfy + scy, -10.)
    imaged = ahead & (xs >= .5) & (xs <= sw-1.5) & (ys >= .5) & (ys <= sh-1.5)
    if frame.get('coverage') is not None:
        cov = np.asarray(frame['coverage'])
        imaged &= cov[np.clip(np.floor(ys+.5).astype(int), 0, sh-1), np.clip(np.floor(xs+.5).astype(int), 0, sw-1)]
    # Sampling coverage is measured on ray DIRECTIONS of every covered source pixel (rotation only), so it depends on
    # the source resolution and the attitude, not on depth: a disocclusion behind a near edge (mount parallax) is
    # not-imaged no-return, as on the CAM-K2 path, never a refusal.
    covered = np.ones((sh, sw), bool) if frame.get('coverage') is None else np.asarray(frame['coverage'])
    dirs = np.stack([np.ones(int(covered.sum())), -u[covered], -v[covered]], axis=-1)
    _, dir_indices = _splat_index(dirs @ rotation.T, tw, th, tfx, tfy, tcx, tcy)
    direction_hit = np.zeros(th*tw, bool)
    direction_hit[dir_indices] = True
    sampled = direction_hit.reshape(th, tw)
    # A hole must lie between sampled pixels, above and below in its column OR left and right in its row (an empty
    # interior row or column counts): a gap at the edge of the imaged region (camera translation parallax, the FOV
    # boundary) is not a sampling hole.
    enclosed = ((np.maximum.accumulate(sampled, axis=0) & np.maximum.accumulate(sampled[::-1], axis=0)[::-1])
                | (np.maximum.accumulate(sampled, axis=1) & np.maximum.accumulate(sampled[:, ::-1], axis=1)[:, ::-1]))
    holes = imaged & ~sampled & enclosed
    if holes.any():
        raise InputError('%d trained pixels inside the source image received no source sample (source too coarse '
                         'for the trained camera at this attitude); frame refused' % int(holes.sum()))
    out_invalid = rayed.reshape(th, tw) & ~hit
    ray_order = np.lexsort((ray_offset, ray_indices))
    ray_sorted = ray_indices[ray_order]
    ray_first = np.r_[True, ray_sorted[1:] != ray_sorted[:-1]] if len(ray_sorted) else np.zeros(0, bool)
    ray_colours = rgb[src_invalid][ray_selected][ray_order][ray_first]
    take = out_invalid.flat[ray_sorted[ray_first]]
    out_rgb.reshape(-1, 3)[ray_sorted[ray_first][take]] = ray_colours[take]
    out_no_return = ~hit & ~out_invalid
    cosine = 1 / np.sqrt(1+tu*tu+tv*tv)
    ranges = np.full((th, tw), maxz, np.float64)
    np.divide(out_z, cosine, out=ranges, where=hit)
    ranges = np.minimum(ranges, maxz).astype(np.float32)
    level = bool(np.allclose(rotation, np.eye(3), atol=1e-8) and np.allclose(offset, 0, atol=1e-8))
    return (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None], out_invalid[None],
            out_no_return[None], cosine.astype(np.float32),
            {'projection': 'measured-point-min-valid', 'level_camera': level,
             'sensor_invalid_pixels': int(out_invalid.sum()), 'no_return_pixels': int(out_no_return.sum()),
             'valid_pixels': int(hit.sum()), 'pixels': th*tw, 'source_pixels': sh*sw,
             'imaged_pixels': int(imaged.sum()), 'unsampled_imaged_pixels': int(holes.sum()),
             'coverage_mask': frame.get('coverage') is not None, 'synthetic_noise_applied': False})



@functools.lru_cache(maxsize=8)
def _pixel_rays(w, h, fx, fy, cx, cy):
    """Normalised optical ray coordinates (u, v) of every pixel, flattened row-major (read only; cached per intrinsics)."""
    u, v = np.meshgrid((np.arange(w)-cx)/fx, (np.arange(h)-cy)/fy)
    u, v = u.reshape(-1), v.reshape(-1)
    u.flags.writeable = False
    v.flags.writeable = False
    return u, v


def _first_min_per_pixel(indices, keys, size):
    """Per pixel, the position in `indices` of the sample with the smallest key; ties -> the smallest position.

    Exactly the first entry of each index run after np.lexsort((keys, indices)) (lexsort is stable), without the
    O(N log N) two-key sort: two np.minimum.at passes (keys are finite). Returns (pixels ascending, positions)."""
    n = len(indices)
    if not n:
        return np.zeros(0, np.int64), np.zeros(0, np.int64)
    best = np.full(size, np.inf)
    np.minimum.at(best, indices, keys)
    candidates = np.flatnonzero(keys == best[indices])
    first = np.full(size, n, np.int64)
    np.minimum.at(first, indices[candidates], candidates)
    pixels = np.flatnonzero(first < n)
    return pixels, first[pixels]


def project_rgbd_min_valid_v1_fast(frame, state, contract):
    """CAM-S2 projection, latency port (2026-11-26) of project_rgbd_min_valid_reference: the same arithmetic and the
    same refusals with identical outputs (tests/test_camera_runtime_cams2.py compares every output array exactly on
    level, tilted, translated, covered and random frames). Changes: the per-pixel nearest-sample selections use
    _first_min_per_pixel instead of a two-key lexsort; pixel ray grids are cached per intrinsics; colours are gathered
    only for the selected samples. See project_rgbd_min_valid_reference for the semantics."""
    _, _, body_r, yaw, _, _ = validate_state(state)
    if frame.get('aligned') is not True or frame.get('rectified') is not True:
        raise InputError('RGB/depth must be explicitly aligned and rectified')
    sw, sh, sfx, sfy, scx, scy = _intrinsics(frame['intrinsics'])
    cam = contract['camera']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    if sfx < tfx * (1. - SOURCE_RESOLUTION_TOLERANCE) or sfy < tfy * (1. - SOURCE_RESOLUTION_TOLERANCE):
        raise InputError('source intrinsics fx=%.3f fy=%.3f are coarser than the trained camera fx=%.3f fy=%.3f; '
                         'feed the aligned stream at (at least) the trained angular resolution' % (sfx, sfy, tfx, tfy))
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
    u, v = _pixel_rays(sw, sh, sfx, sfy, scx, scy)
    rgb_flat = rgb.reshape(-1, 3)
    good = np.flatnonzero(~(src_invalid | src_no_return))
    zz = z.reshape(-1)[good].astype(np.float64)
    points = np.stack([zz, -u[good]*zz, -v[good]*zz], axis=-1) @ rotation.T + offset
    selected, indices = _splat_index(points, tw, th, tfx, tfy, tcx, tcy)
    zs = points[selected, 0]
    level_valid = np.flatnonzero((zs >= minz) & (zs <= maxz))
    indices, zs = indices[level_valid], zs[level_valid]
    pixels, pos = _first_min_per_pixel(indices, zs, th*tw)
    hit = np.zeros(th*tw, bool)
    hit[pixels] = True
    hit = hit.reshape(th, tw)
    out_z = np.zeros((th, tw), np.float64)
    out_rgb = np.zeros((th, tw, 3), np.uint8)
    out_z.flat[pixels] = zs[pos]
    out_rgb.reshape(-1, 3)[pixels] = rgb_flat[good[selected[level_valid[pos]]]]
    invalid_src = np.flatnonzero(src_invalid)
    rays = np.stack([np.ones(len(invalid_src)), -u[invalid_src], -v[invalid_src]], axis=-1)
    ray_selected, ray_indices, ray_offset = _splat_index(rays @ rotation.T, tw, th, tfx, tfy, tcx, tcy, offset=True)
    rayed = np.zeros(th*tw, bool)
    rayed[ray_indices] = True
    tu, tv = _pixel_rays(tw, th, tfx, tfy, tcx, tcy)
    tu, tv = tu.reshape(th, tw), tv.reshape(th, tw)
    src = np.stack([np.ones_like(tu), -tu, -tv], axis=-1) @ rotation
    ahead = src[..., 0] > 1e-9
    with np.errstate(divide='ignore', invalid='ignore'):
        xs = np.where(ahead, -src[..., 1]/np.where(ahead, src[..., 0], 1.)*sfx + scx, -10.)
        ys = np.where(ahead, -src[..., 2]/np.where(ahead, src[..., 0], 1.)*sfy + scy, -10.)
    imaged = ahead & (xs >= .5) & (xs <= sw-1.5) & (ys >= .5) & (ys <= sh-1.5)
    if frame.get('coverage') is not None:
        cov = np.asarray(frame['coverage'])
        imaged &= cov[np.clip(np.floor(ys+.5).astype(int), 0, sh-1), np.clip(np.floor(xs+.5).astype(int), 0, sw-1)]
        covered = np.flatnonzero(cov)
        dirs = np.stack([np.ones(len(covered)), -u[covered], -v[covered]], axis=-1)
    else:
        dirs = np.stack([np.ones(sh*sw), -u, -v], axis=-1)
    _, dir_indices = _splat_index(dirs @ rotation.T, tw, th, tfx, tfy, tcx, tcy)
    direction_hit = np.zeros(th*tw, bool)
    direction_hit[dir_indices] = True
    sampled = direction_hit.reshape(th, tw)
    enclosed = ((np.maximum.accumulate(sampled, axis=0) & np.maximum.accumulate(sampled[::-1], axis=0)[::-1])
                | (np.maximum.accumulate(sampled, axis=1) & np.maximum.accumulate(sampled[:, ::-1], axis=1)[:, ::-1]))
    holes = imaged & ~sampled & enclosed
    if holes.any():
        raise InputError('%d trained pixels inside the source image received no source sample (source too coarse '
                         'for the trained camera at this attitude); frame refused' % int(holes.sum()))
    out_invalid = rayed.reshape(th, tw) & ~hit
    # Only sensor-invalid trained pixels take an invalid ray's colour: select among their rays only (every ray of
    # such a pixel is kept in order, so the per-pixel choice is unchanged).
    need = np.flatnonzero(out_invalid.flat[ray_indices])
    ray_pixels, ray_pos = _first_min_per_pixel(ray_indices[need], ray_offset[need], th*tw)
    out_rgb.reshape(-1, 3)[ray_pixels] = rgb_flat[invalid_src[ray_selected[need[ray_pos]]]]
    out_no_return = ~hit & ~out_invalid
    cosine = 1 / np.sqrt(1+tu*tu+tv*tv)
    ranges = np.full((th, tw), maxz, np.float64)
    np.divide(out_z, cosine, out=ranges, where=hit)
    ranges = np.minimum(ranges, maxz).astype(np.float32)
    level = bool(np.allclose(rotation, np.eye(3), atol=1e-8) and np.allclose(offset, 0, atol=1e-8))
    return (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None], out_invalid[None],
            out_no_return[None], cosine.astype(np.float32),
            {'projection': 'measured-point-min-valid', 'level_camera': level,
             'sensor_invalid_pixels': int(out_invalid.sum()), 'no_return_pixels': int(out_no_return.sum()),
             'valid_pixels': int(hit.sum()), 'pixels': th*tw, 'source_pixels': sh*sw,
             'imaged_pixels': int(imaged.sum()), 'unsampled_imaged_pixels': int(holes.sum()),
             'coverage_mask': frame.get('coverage') is not None, 'synthetic_noise_applied': False})


# ---------------------------------------------------------------- canonical v2 (CPU numpy == GPU torch, bit for bit)
# Runtime v2 projection (2026-11-26, user-approved GPU port). v1 rotates with a BLAS matrix product, whose rounding
# depends on the library and device (x86 vs ARM BLAS, cuBLAS FMA), so a point that falls on a trained-pixel border can
# change pixel between machines. v2 fixes every float64 operation and its order as plain IEEE-754 elementwise products,
# sums, quotients and floors (no matrix product, no fused multiply-add, no pow), which numpy and torch (CPU or CUDA,
# float64) both round identically; the pixel grids, rotation and cosine are computed once on the CPU and shared.
# So CPU and GPU agree bit for bit ON THE SAME HOST. Across hosts the attitude rotation (validate_state, cos / sin and
# the 3x3 products, host libm / BLAS) may still differ in the last bit (review r1 P3-1).
# Semantics are v1's; it may differ from v1 only where a sample lies within rounding of a pixel border.

def _projection_inputs(frame, state, contract):
    """Validation, attitude and source classes shared by every v2 backend (CPU numpy)."""
    _, _, body_r, yaw, _, _ = validate_state(state)
    if frame.get('aligned') is not True or frame.get('rectified') is not True:
        raise InputError('RGB/depth must be explicitly aligned and rectified')
    sw, sh, sfx, sfy, scx, scy = _intrinsics(frame['intrinsics'])
    cam = contract['camera']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    if sfx < tfx * (1. - SOURCE_RESOLUTION_TOLERANCE) or sfy < tfy * (1. - SOURCE_RESOLUTION_TOLERANCE):
        raise InputError('source intrinsics fx=%.3f fy=%.3f are coarser than the trained camera fx=%.3f fy=%.3f; '
                         'feed the aligned stream at (at least) the trained angular resolution' % (sfx, sfy, tfx, tfy))
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
    rotation = rel @ optical                   # 3x3 on the CPU, shared by every backend
    offset = rel @ mount - np.asarray(cam['translation_m'])
    depth = contract['depth']
    src_invalid, src_no_return = _source_classes(frame, z, invalid, depth)
    coverage = frame.get('coverage')
    return {'src': (sw, sh, sfx, sfy, scx, scy), 'dst': (tw, th, tfx, tfy, tcx, tcy), 'rgb': rgb, 'z': z,
            'R': [[float(rotation[i, j]) for j in range(3)] for i in range(3)],
            'offset': [float(x) for x in offset], 'minz': float(depth['minz_m']), 'maxz': float(depth['maxz_m']),
            'src_invalid': src_invalid, 'src_no_return': src_no_return,
            'coverage': None if coverage is None else np.asarray(coverage),
            'level': bool(np.allclose(rotation, np.eye(3), atol=1e-8) and np.allclose(offset, 0, atol=1e-8))}


@functools.lru_cache(maxsize=8)
def _trained_grid(w, h, fx, fy, cx, cy):
    """(tu, tv, cosine float64 [H, W]) of the trained camera, computed once on the CPU (read only)."""
    tu, tv = np.meshgrid((np.arange(w)-cx)/fx, (np.arange(h)-cy)/fy)
    cosine = 1 / np.sqrt(1+tu*tu+tv*tv)
    for a in (tu, tv, cosine):
        a.flags.writeable = False
    return tu, tv, cosine


def _v2_rows(R, a, b, c, out=None):
    """[(R[i][0]*a + R[i][1]*b) + R[i][2]*c (+ out[i])] for i = 0..2: the fixed v2 order (numpy and torch alike)."""
    rows = []
    for i in range(3):
        r = (R[i][0]*a + R[i][1]*b) + R[i][2]*c
        rows.append(r if out is None else r + out[i])
    return rows


def _v2_cols(R, a, b, c):
    """[(a*R[0][j] + b*R[1][j]) + c*R[2][j]] for j = 0..2: a level ray -> the source camera, fixed order."""
    return [(a*R[0][j] + b*R[1][j]) + c*R[2][j] for j in range(3)]


def _v2_finish(P, out_rgb, out_z, hit, out_invalid, imaged, holes_count, device='cpu'):
    tw, th = P['dst'][:2]
    _, _, cosine = _trained_grid(*P['dst'])
    out_no_return = ~hit & ~out_invalid
    ranges = np.full((th, tw), P['maxz'], np.float64)
    np.divide(out_z, cosine, out=ranges, where=hit)
    ranges = np.minimum(ranges, P['maxz']).astype(np.float32)
    sw, sh = P['src'][:2]
    return (out_rgb.transpose(2, 0, 1)[None].astype(np.float32)/255., ranges[None], out_invalid[None],
            out_no_return[None], cosine.astype(np.float32),
            {'projection': 'measured-point-min-valid', 'arithmetic': 'v2-fixed-order-float64', 'device': device,
             'level_camera': P['level'],
             'sensor_invalid_pixels': int(out_invalid.sum()), 'no_return_pixels': int(out_no_return.sum()),
             'valid_pixels': int(hit.sum()), 'pixels': th*tw, 'source_pixels': sh*sw,
             'imaged_pixels': int(imaged.sum()), 'unsampled_imaged_pixels': int(holes_count),
             'coverage_mask': P['coverage'] is not None, 'synthetic_noise_applied': False})


def _holes_error(n):
    return InputError('%d trained pixels inside the source image received no source sample (source too coarse '
                      'for the trained camera at this attitude); frame refused' % int(n))


def _v2_splat_np(d0, d1, d2, dst, offset=False):
    tw, th, tfx, tfy, tcx, tcy = dst
    sel = np.flatnonzero(d0 > 1e-6)
    a0, a1, a2 = d0[sel], d1[sel], d2[sel]
    xf = -a1/a0*tfx + tcx
    yf = -a2/a0*tfy + tcy
    xx = np.floor(xf + .5)
    yy = np.floor(yf + .5)
    keep = (xx >= 0) & (xx < tw) & (yy >= 0) & (yy < th)
    xi, yi = xx[keep].astype(np.int64), yy[keep].astype(np.int64)
    if offset:
        dx, dy = xf[keep] - xx[keep], yf[keep] - yy[keep]
        return sel[keep], yi*tw + xi, dx*dx + dy*dy
    return sel[keep], yi*tw + xi


def project_rgbd_min_valid(frame, state, contract):
    """CAM-S2 projection, runtime v2 canonical arithmetic on the CPU (numpy float64). project_rgbd_min_valid_cuda is
    bit-identical (tests/test_camera_runtime_cams2.py). Semantics: project_rgbd_min_valid_reference (v1)."""
    P = _projection_inputs(frame, state, contract)
    sw, sh, sfx, sfy, scx, scy = P['src']
    tw, th = P['dst'][:2]
    R, rgb_flat = P['R'], P['rgb'].reshape(-1, 3)
    u, v = _pixel_rays(sw, sh, sfx, sfy, scx, scy)
    good = np.flatnonzero(~(P['src_invalid'] | P['src_no_return']))
    zz = P['z'].reshape(-1)[good].astype(np.float64)
    p0, p1, p2 = _v2_rows(R, zz, -u[good]*zz, -v[good]*zz, P['offset'])
    selected, indices = _v2_splat_np(p0, p1, p2, P['dst'])
    zs = p0[selected]
    level_valid = np.flatnonzero((zs >= P['minz']) & (zs <= P['maxz']))
    indices, zs = indices[level_valid], zs[level_valid]
    pixels, pos = _first_min_per_pixel(indices, zs, th*tw)
    hit = np.zeros(th*tw, bool)
    hit[pixels] = True
    hit = hit.reshape(th, tw)
    out_z = np.zeros((th, tw), np.float64)
    out_rgb = np.zeros((th, tw, 3), np.uint8)
    out_z.flat[pixels] = zs[pos]
    out_rgb.reshape(-1, 3)[pixels] = rgb_flat[good[selected[level_valid[pos]]]]
    invalid_src = np.flatnonzero(P['src_invalid'])
    ui, vi = -u[invalid_src], -v[invalid_src]
    r0, r1, r2 = _v2_rows(R, np.ones(len(invalid_src)), ui, vi)
    ray_selected, ray_indices, ray_offset = _v2_splat_np(r0, r1, r2, P['dst'], offset=True)
    rayed = np.zeros(th*tw, bool)
    rayed[ray_indices] = True
    tu, tv, _ = _trained_grid(*P['dst'])
    s0, s1, s2 = _v2_cols(R, np.ones((th, tw)), -tu, -tv)
    ahead = s0 > 1e-9
    with np.errstate(divide='ignore', invalid='ignore'):
        den = np.where(ahead, s0, 1.)
        xs = np.where(ahead, -s1/den*sfx + scx, -10.)
        ys = np.where(ahead, -s2/den*sfy + scy, -10.)
    imaged = ahead & (xs >= .5) & (xs <= sw-1.5) & (ys >= .5) & (ys <= sh-1.5)
    cov = P['coverage']
    if cov is not None:
        imaged &= cov[np.clip(np.floor(ys+.5).astype(int), 0, sh-1), np.clip(np.floor(xs+.5).astype(int), 0, sw-1)]
        covered = np.flatnonzero(cov)
        c0, c1, c2 = _v2_rows(R, np.ones(len(covered)), -u[covered], -v[covered])
    else:
        c0, c1, c2 = _v2_rows(R, np.ones(sh*sw), -u, -v)
    _, dir_indices = _v2_splat_np(c0, c1, c2, P['dst'])
    direction_hit = np.zeros(th*tw, bool)
    direction_hit[dir_indices] = True
    sampled = direction_hit.reshape(th, tw)
    enclosed = ((np.maximum.accumulate(sampled, axis=0) & np.maximum.accumulate(sampled[::-1], axis=0)[::-1])
                | (np.maximum.accumulate(sampled, axis=1) & np.maximum.accumulate(sampled[:, ::-1], axis=1)[:, ::-1]))
    holes = imaged & ~sampled & enclosed
    if holes.any():
        raise _holes_error(holes.sum())
    out_invalid = rayed.reshape(th, tw) & ~hit
    need = np.flatnonzero(out_invalid.flat[ray_indices])
    ray_pixels, ray_pos = _first_min_per_pixel(ray_indices[need], ray_offset[need], th*tw)
    out_rgb.reshape(-1, 3)[ray_pixels] = rgb_flat[invalid_src[ray_selected[need[ray_pos]]]]
    return _v2_finish(P, out_rgb, out_z, hit, out_invalid, imaged, 0)


class CudaProjector:
    """Runtime v2 projection on a CUDA device (torch float64), bit-identical to project_rgbd_min_valid.

    Grids and the cosine come from the CPU functions above (uploaded once per intrinsics); every per-frame float64
    operation is a plain elementwise torch op in v2 order (no addcmul / alpha / matmul / pow), so no fused multiply-add
    is formed. warm_up() runs the first-call initialisation (1.2-1.9 s measured on the Orin NX) before flight."""

    def __init__(self, torch, device='cuda'):
        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is not available for the v2 projector')
        self.torch, self.device = torch, torch.device(device)
        self._grids = {}

    def _grid(self, key, make):
        if key not in self._grids:
            self._grids[key] = tuple(self.torch.from_numpy(np.array(a)).to(self.device) for a in make())   # writable copy
        return self._grids[key]

    def _splat(self, d0, d1, d2, dst, offset=False):
        torch = self.torch
        tw, th, tfx, tfy, tcx, tcy = dst
        sel = torch.nonzero(d0 > 1e-6).squeeze(1)
        a0, a1, a2 = d0[sel], d1[sel], d2[sel]
        xf = -a1/a0*tfx + tcx
        yf = -a2/a0*tfy + tcy
        xx = torch.floor(xf + .5)
        yy = torch.floor(yf + .5)
        keep = (xx >= 0) & (xx < tw) & (yy >= 0) & (yy < th)
        xi, yi = xx[keep].to(torch.int64), yy[keep].to(torch.int64)
        if offset:
            dx, dy = xf[keep] - xx[keep], yf[keep] - yy[keep]
            return sel[keep], yi*tw + xi, dx*dx + dy*dy
        return sel[keep], yi*tw + xi

    def _first_min(self, indices, keys, size):
        torch = self.torch
        n = int(indices.numel())
        if not n:
            empty = torch.zeros(0, dtype=torch.int64, device=self.device)
            return empty, empty
        best = torch.full((size,), math.inf, dtype=keys.dtype, device=self.device)
        best = best.scatter_reduce(0, indices, keys, reduce='amin', include_self=True)
        candidates = torch.nonzero(keys == best[indices]).squeeze(1)
        first = torch.full((size,), n, dtype=torch.int64, device=self.device)
        first = first.scatter_reduce(0, indices[candidates], candidates, reduce='amin', include_self=True)
        pixels = torch.nonzero(first < n).squeeze(1)
        return pixels, first[pixels]

    def __call__(self, frame, state, contract):
        torch, dev = self.torch, self.device
        P = _projection_inputs(frame, state, contract)
        sw, sh, sfx, sfy, scx, scy = P['src']
        tw, th = P['dst'][:2]
        R = P['R']
        u, v = self._grid(('src',) + P['src'], lambda: _pixel_rays(sw, sh, sfx, sfy, scx, scy))
        tu, tv = self._grid(('dst',) + P['dst'], lambda: _trained_grid(*P['dst'])[:2])
        to = lambda a: torch.from_numpy(np.ascontiguousarray(a)).to(dev)    # noqa: E731
        rgb_flat = to(P['rgb'].reshape(-1, 3))
        src_invalid = to(P['src_invalid'].reshape(-1))
        good = torch.nonzero(~(src_invalid | to(P['src_no_return'].reshape(-1)))).squeeze(1)
        zz = to(P['z'].reshape(-1))[good].to(torch.float64)
        p0, p1, p2 = _v2_rows(R, zz, -u[good]*zz, -v[good]*zz, P['offset'])
        selected, indices = self._splat(p0, p1, p2, P['dst'])
        zs = p0[selected]
        level_valid = torch.nonzero((zs >= P['minz']) & (zs <= P['maxz'])).squeeze(1)
        indices, zs = indices[level_valid], zs[level_valid]
        pixels, pos = self._first_min(indices, zs, th*tw)
        hit = torch.zeros(th*tw, dtype=torch.bool, device=dev)
        hit[pixels] = True
        out_z = torch.zeros(th*tw, dtype=torch.float64, device=dev)
        out_rgb = torch.zeros((th*tw, 3), dtype=torch.uint8, device=dev)
        out_z[pixels] = zs[pos]
        out_rgb[pixels] = rgb_flat[good[selected[level_valid[pos]]]]
        invalid_src = torch.nonzero(src_invalid).squeeze(1)
        r0, r1, r2 = _v2_rows(R, torch.ones(int(invalid_src.numel()), dtype=torch.float64, device=dev),
                              -u[invalid_src], -v[invalid_src])
        ray_selected, ray_indices, ray_offset = self._splat(r0, r1, r2, P['dst'], offset=True)
        rayed = torch.zeros(th*tw, dtype=torch.bool, device=dev)
        rayed[ray_indices] = True
        s0, s1, s2 = _v2_cols(R, torch.ones((th, tw), dtype=torch.float64, device=dev), -tu, -tv)
        ahead = s0 > 1e-9
        one = torch.ones_like(s0)
        den = torch.where(ahead, s0, one)
        xs = torch.where(ahead, -s1/den*sfx + scx, one*-10.)
        ys = torch.where(ahead, -s2/den*sfy + scy, one*-10.)
        imaged = ahead & (xs >= .5) & (xs <= sw-1.5) & (ys >= .5) & (ys <= sh-1.5)
        if P['coverage'] is not None:
            cov = to(P['coverage'])
            imaged &= cov[torch.clamp(torch.floor(ys+.5).to(torch.int64), 0, sh-1),
                          torch.clamp(torch.floor(xs+.5).to(torch.int64), 0, sw-1)]
            covered = torch.nonzero(cov.reshape(-1)).squeeze(1)
            c0, c1, c2 = _v2_rows(R, torch.ones(int(covered.numel()), dtype=torch.float64, device=dev),
                                  -u[covered], -v[covered])
        else:
            c0, c1, c2 = _v2_rows(R, torch.ones(sh*sw, dtype=torch.float64, device=dev), -u, -v)
        _, dir_indices = self._splat(c0, c1, c2, P['dst'])
        sampled = torch.zeros(th*tw, dtype=torch.bool, device=dev)
        sampled[dir_indices] = True
        sampled = sampled.reshape(th, tw)
        s8 = sampled.to(torch.uint8)

        def acc(x, dim):
            return torch.cummax(x, dim=dim).values.bool()
        enclosed = ((acc(s8, 0) & torch.flip(acc(torch.flip(s8, (0,)), 0), (0,)))
                    | (acc(s8, 1) & torch.flip(acc(torch.flip(s8, (1,)), 1), (1,))))
        holes = int((imaged & ~sampled & enclosed).sum().item())
        if holes:
            raise _holes_error(holes)
        out_invalid = rayed & ~hit
        need = torch.nonzero(out_invalid[ray_indices]).squeeze(1)
        ray_pixels, ray_pos = self._first_min(ray_indices[need], ray_offset[need], th*tw)
        out_rgb[ray_pixels] = rgb_flat[invalid_src[ray_selected[need[ray_pos]]]]
        return _v2_finish(P, out_rgb.reshape(th, tw, 3).cpu().numpy(), out_z.reshape(th, tw).cpu().numpy(),
                          hit.reshape(th, tw).cpu().numpy(), out_invalid.reshape(th, tw).cpu().numpy(),
                          imaged.cpu().numpy(), 0, device='cuda')

    def warm_up(self, frame, state, contract, repeats=3):
        """First-call CUDA initialisation before flight; returns the wall time of each call (s)."""
        import time
        times = []
        for _ in range(repeats):
            t = time.perf_counter()
            self(frame, state, contract)
            self.torch.cuda.synchronize(self.device)
            times.append(time.perf_counter() - t)
        return times

def height_band(state, contract):
    """Whether the camera height lies inside the enabled floor/ceiling filter band; pure, no history.

    Camera height above the floor = estimator ENU z + the trained camera's z offset. With the floor
    filter on it must exceed ground.margin_m; with the ceiling filter on it must stay below
    ceiling.height_m - ceiling.margin_m. Outside the band every return at camera height would be
    removed as floor or ceiling, so remove_floor_and_ceiling refuses such a frame, and the ROS
    session waits (WAITING_ALTITUDE) without calling the runtime (e.g. on the ground before take-off).
    An enabled filter without the operator's estimator-height assertion (contract estimator_height,
    repair r1 F6) raises InputError instead: the frame fails closed, and the ROS session faults.
    Outdoor profile: no assertion is needed and this PX4-z band only gates take-off (WAITING_ALTITUDE);
    each frame's filters then use the camera-verified ground plane (estimate_ground).
    Returns {'in_band', 'camera_height_m', 'reason'}.
    """
    from camera_policy_contract import height_reference_problem
    problem = height_reference_problem(contract)
    if problem:
        raise InputError(problem)
    p, _, _, _, _, _ = validate_state(split_state(state, contract)[0])
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


def _vertical_validity(vertical):
    """Parse PX4's z_valid / epv [m] / dead_reckoning; None when absent or incomplete; malformed -> InputError."""
    if vertical is None or set(vertical) != set(VERTICAL_FIELDS):
        return None
    z_valid, epv, dead = vertical['z_valid'], vertical['epv'], vertical['dead_reckoning']
    if not isinstance(z_valid, (bool, np.bool_)) or not isinstance(dead, (bool, np.bool_)):
        raise InputError('estimator z_valid and dead_reckoning must be booleans')
    # Outdoor repair r1 (review P3-2): a real scalar only. Strings ("0.1"), arrays (also 0-d and 1-element ones),
    # booleans and complex numbers are malformed input, never parsed into a value.
    if isinstance(epv, (bool, np.bool_)) or not isinstance(epv, (int, float, np.integer, np.floating)):
        raise InputError('estimator epv must be a finite nonnegative real scalar in metres')
    epv = float(epv)
    if not math.isfinite(epv) or epv < 0:
        raise InputError('estimator epv must be a finite nonnegative number of metres')
    return {'z_valid': bool(z_valid), 'epv': epv, 'dead_reckoning': bool(dead)}


def _signed_distances(points, normals, offsets):
    """[n, k] signed distances of points [n, 3] from planes n.x + d = 0; elementwise (no BLAS reduction order)."""
    return (points[:, 0:1] * normals[None, :, 0] + points[:, 1:2] * normals[None, :, 1]
            + points[:, 2:3] * normals[None, :, 2] + offsets[None, :])


def _fit_plane(points):
    """Total-least-squares plane through points [n, 3]; normal oriented up (+z), returns (normal, offset)."""
    centre = points.mean(axis=0)
    q = points - centre
    _, vectors = np.linalg.eigh(np.einsum('ij,ik->jk', q, q))
    normal = vectors[:, 0]
    if normal[2] < 0:
        normal = -normal
    return normal, -float(normal @ centre)


def _ransac_plane(points, rng, threshold, cos_search):
    """One deterministic RANSAC pass (rng draws in a fixed order) plus TLS refinement; None if no hypothesis.

    Hypotheses are scored by MSAC (sum of squared distances truncated at the inlier threshold; the lowest wins,
    first on ties), which prefers the tight fit of one real surface over a plane that loosely spans two (a
    bridging plane across a step has as many inliers but larger residuals)."""
    idx = rng.integers(0, len(points), size=(GROUND_RANSAC_ITERATIONS, 3))
    a, b, c = points[idx[:, 0]], points[idx[:, 1]], points[idx[:, 2]]
    normals = np.cross(b - a, c - a)
    norms = np.linalg.norm(normals, axis=1)
    ok = norms >= 2. * GROUND_MIN_SAMPLE_AREA_M2
    normals, a = normals[ok] / norms[ok, None], a[ok]
    normals[normals[:, 2] < 0] *= -1.
    offsets = -(normals * a).sum(axis=1)
    # Near-horizontal hypotheses below the camera only (a wall or a plane above the camera is not ground).
    keep = (normals[:, 2] >= cos_search) & (offsets > 0)
    normals, offsets = normals[keep], offsets[keep]
    if not len(normals):
        return None
    cost = np.minimum(_signed_distances(points, normals, offsets) ** 2, threshold * threshold).sum(axis=0)
    best = int(np.argmin(cost))
    normal, offset = normals[best], float(offsets[best])
    for _ in range(GROUND_REFINE_ROUNDS):
        inliers = np.abs(_signed_distances(points, normal[None], np.array([offset]))[:, 0]) <= threshold
        if int(inliers.sum()) < 3:
            break
        normal, offset = _fit_plane(points[inliers])
    distance = _signed_distances(points, normal[None], np.array([offset]))[:, 0]
    return normal, offset, distance


def estimate_ground(frame, state, contract):
    """Outdoor profile: the ground plane from the camera, cross-checked with PX4. Pure and deterministic.

    Points: VALID source pixels (not sensor-invalid, not no-return) on a regular stride, reprojected into the
    gravity-aligned (PX4 attitude), yaw-level frame of the trained camera exactly as project_rgbd does, whose
    ray from the real camera points below the horizon and whose range is <= fit_max_range_m; at most
    GROUND_MAX_POINTS of them, evenly spaced in raster order. Fit: sequential RANSAC (fixed seed, MSAC score, at
    most GROUND_MAX_PLANES near-horizontal planes below the camera, each refined by total least squares) on these
    points. For each plane: trained-camera height above it, tilt from gravity, inlier count and share, residual RMS.
    Accept only if PX4's vertical fields are present, z_valid is true, epv <= epv_max_m, not dead reckoning
    (always refused), and some plane has >= min_inliers and >= min_inlier_share, tilt <= tilt_max_deg, at
    most max_below_share of the points below it by more than inlier_threshold_m + floor_noise_margin_per_m * range
    (the ground is the lowest surface: this rejects a bridging plane across a step and a raised top whose sides
    reach lower) and |height - PX4 camera height| <= height_tolerance_m (PX4 camera height = ENU z + camera z
    offset, as height_band). The consistent plane with the most support is the candidate ground. Outdoor repair r1
    (review P1-1) then refuses wrong-surface fits: ground_ambiguous when another found plane (consistent or not)
    holds >= rival_min_share of the points, or a consistent one >= ambiguity_support_ratio of the candidate's
    support, at a height more than ambiguity_separation_m away (rivals are horizontal surfaces below the camera:
    tilt <= GROUND_SEARCH_TILT_DEG after refinement); ground_inlier_spread_insufficient when the
    candidate's inliers cover less than min_half_share of either image half or less than min_range_span_m of
    range (10th-90th percentile); ground_not_planar when the planes fitted to the nearest and farthest third of
    its inliers or to either image half tilt more than planarity_max_angle_deg from it (a plane bridging the near
    ground and a raised top). Finally the verified camera height must lie inside the floor/ceiling band.
    Returns a JSON-serializable report: accepted, reason (first of REFUSAL_REASONS that applies, None if
    accepted), reasons, plane (accepted plane or None), candidates, estimator and point counts. Malformed inputs
    raise InputError; a refusal is a report, raised as FrameRefused by the caller.
    """
    from camera_policy_contract import threshold_value
    ge, cam, depth = contract['ground_estimate'], contract['camera'], contract['depth']
    def T(key):
        return float(threshold_value(ge, key))
    base, vertical = split_state(state, contract)
    p, _, body_r, yaw, _, _ = validate_state(base)
    px4_height = float(p[2]) + float(cam['translation_m'][2])
    reasons = []
    estimator = _vertical_validity(vertical)
    if estimator is None:
        reasons.append('estimator_vertical_fields_absent')
    else:
        if not estimator['z_valid']:
            reasons.append('estimator_z_invalid')
        if not estimator['epv'] <= T('epv_max_m'):
            reasons.append('estimator_epv_exceeds_limit')
        # Outdoor repair r1 (review P3-3): dead reckoning is always refused (the contract requires
        # refuse_dead_reckoning true; the runtime does not depend on that flag even for an unvalidated contract).
        if estimator['dead_reckoning']:
            reasons.append('estimator_dead_reckoning')
    sw, sh, sfx, sfy, scx, scy = _intrinsics(frame['intrinsics'])
    z, invalid = np.asarray(frame['depth_z_m']), np.asarray(frame['invalid'])
    if z.shape != (sh, sw) or not np.issubdtype(z.dtype, np.floating) or invalid.shape != z.shape \
            or invalid.dtype != np.bool_:
        raise InputError('ground estimate needs the aligned depth_z_m and invalid arrays')
    mount = _finite(frame['mount_flu_m'], (3,), 'mount_flu_m')
    optical = _finite(frame['camera_to_body_rotation'], (3, 3), 'camera_to_body_rotation')
    cy, sy = math.cos(yaw), math.sin(yaw)
    rel = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.]]).T @ body_r
    rotation, offset = rel @ optical, rel @ mount - np.asarray(cam['translation_m'], np.float64)
    stride = max(1, int(math.ceil(math.sqrt(sw * sh / GROUND_SOURCE_PIXELS))))
    rows, cols = np.arange(stride // 2, sh, stride), np.arange(stride // 2, sw, stride)
    sub = np.ix_(rows, cols)
    coverage = frame.get('coverage')
    sub_frame = {} if coverage is None else {'coverage': np.asarray(coverage)[sub]}
    zz = z[sub].astype(np.float64)
    src_invalid, src_no_return = _source_classes(sub_frame, zz, invalid[sub], depth)
    good = ~(src_invalid | src_no_return)
    uu, vv = np.meshgrid((cols - scx) / sfx, (rows - scy) / sfy)
    zg = zz[good]
    camera_points = np.stack([zg, -uu[good] * zg, -vv[good] * zg], axis=-1)
    rays = np.einsum('ij,nj->ni', rotation, camera_points)
    distance = np.sqrt(np.einsum('ni,ni->n', camera_points, camera_points))
    candidates = (rays[:, 2] < 0) & (distance <= T('fit_max_range_m'))
    points = (rays + offset)[candidates]
    n_candidates = len(points)
    if n_candidates > GROUND_MAX_POINTS:
        points = points[np.linspace(0, n_candidates - 1, GROUND_MAX_POINTS).round().astype(int)]
    below_margin = T('inlier_threshold_m') + T('floor_noise_margin_per_m') * np.sqrt(np.einsum('ni,ni->n', points, points))
    min_inliers, threshold = int(threshold_value(ge, 'min_inliers')), T('inlier_threshold_m')
    planes = []
    if len(points) < min_inliers:
        reasons.append('ground_insufficient_points')
    else:
        rng = np.random.default_rng(GROUND_RANSAC_SEED)
        cos_search = math.cos(math.radians(GROUND_SEARCH_TILT_DEG))
        remaining = points
        for _ in range(GROUND_MAX_PLANES):
            if len(remaining) < max(3, min_inliers):
                break
            fit = _ransac_plane(remaining, rng, threshold, cos_search)
            if fit is None:
                break
            normal, d, signed = fit
            inliers = np.abs(signed) <= threshold
            count = int(inliers.sum())
            if count < 3:
                break
            tilt = math.degrees(math.acos(min(1., max(-1., float(normal[2])))))
            failed = []
            if count < min_inliers or count < T('min_inlier_share') * len(points):
                failed.append('ground_insufficient_support')
            if not tilt <= T('tilt_max_deg'):
                failed.append('ground_tilt_exceeds_tolerance')
            below = int((_signed_distances(points, normal[None], np.array([d]))[:, 0] < -below_margin).sum())
            if below > T('max_below_share') * len(points):
                failed.append('ground_not_lowest_surface')
            if not abs(d - px4_height) <= T('height_tolerance_m'):
                failed.append('ground_height_disagrees_with_px4')
            planes.append({'normal': [float(x) for x in normal], 'height_m': float(d), 'tilt_deg': tilt,
                           'inliers': count, 'share': count / len(points), 'below_share': below / len(points),
                           'rms_m': float(np.sqrt(np.mean(signed[inliers] ** 2))),
                           'height_error_m': float(d - px4_height), 'failed': failed})
            remaining = remaining[np.abs(signed) > GROUND_REMOVE_FACTOR * threshold]
        consistent = sorted((q for q in planes if not q['failed']), key=lambda q: -q['inliers'])
        if not planes:
            reasons.append('ground_not_found')
        elif not consistent:
            reasons.append(planes[0]['failed'][0])
        else:
            best = consistent[0]
            # Wrong-surface checks (outdoor repair r1, review P1-1), all fail closed:
            # (1) a rival: any other near-horizontal plane below the camera found by the sequential RANSAC, whether
            #     or not it passes the other checks, with at least rival_min_share of the points at a height more than
            #     ambiguity_separation_m away (a box top, a step or a platform beside the ground); the earlier rule
            #     (a consistent rival with ambiguity_support_ratio of the support) is kept.
            # A rival is a horizontal surface below the camera: the TLS refinement may turn a hypothesis into a wall
            # (a trunk, a post), which is an obstacle, not a second ground level.
            rivals = [q for q in planes if q is not best and q['tilt_deg'] <= GROUND_SEARCH_TILT_DEG
                      and q['height_m'] > 0. and (
                q['inliers'] >= T('rival_min_share') * len(points)
                or (not q['failed'] and q['inliers'] >= T('ambiguity_support_ratio') * best['inliers']))]
            if any(abs(q['height_m'] - best['height_m']) > T('ambiguity_separation_m') for q in rivals):
                reasons.append('ground_ambiguous')
            # (2) spread: the accepted plane's inliers must cover both image halves (level-frame azimuth) and a range
            #     interval, so that a small patch (a bench top, one side of the view) is not taken for the ground;
            # (3) planarity: the planes fitted to the nearest and the farthest third of those inliers (by horizontal
            #     range) and to each image half must agree with the whole fit. A plane bridging the near ground and a
            #     farther raised top is tilted while each of its pieces is level (gravity from PX4's attitude).
            normal_b = np.asarray(best['normal'], np.float64)
            inl = points[np.abs(_signed_distances(points, normal_b[None], np.array([best['height_m']]))[:, 0])
                         <= threshold]
            horizontal = np.hypot(inl[:, 0], inl[:, 1])
            left = float(np.mean(inl[:, 1] >= 0.)) if len(inl) else 0.
            p10, p90 = (np.percentile(horizontal, [10., 90.]) if len(inl) else (0., 0.))
            spread = {'left_share': left, 'right_share': 1. - left if len(inl) else 0.,
                      'range_p10_m': float(p10), 'range_p90_m': float(p90)}
            if (min(spread['left_share'], spread['right_share']) < T('min_half_share')
                    or not float(p90 - p10) >= T('min_range_span_m')):
                reasons.append('ground_inlier_spread_insufficient')
            order = np.argsort(horizontal, kind='stable')
            third = len(inl) // 3
            parts = {'near_third': inl[order[:third]], 'far_third': inl[order[len(inl) - third:]],
                     'left_half': inl[inl[:, 1] >= 0.], 'right_half': inl[inl[:, 1] < 0.]}
            planarity = {}
            for name, part in parts.items():
                if len(part) < GROUND_SPLIT_MIN_POINTS:
                    planarity[name] = None
                    continue
                part_normal, _ = _fit_plane(part)
                planarity[name] = math.degrees(math.acos(min(1., abs(float(part_normal @ normal_b)))))
            if any(a is None or not a <= T('planarity_max_angle_deg') for a in planarity.values()):
                reasons.append('ground_not_planar')
            best['spread'], best['planarity_deg'] = spread, planarity
    plane = None if not planes or any(r.startswith('ground_') for r in reasons) else \
        sorted((q for q in planes if not q['failed']), key=lambda q: -q['inliers'])[0]
    if plane is not None:
        ground, ceiling = contract['ground'], contract['ceiling']
        if not plane['height_m'] > float(ground['margin_m']) or (
                ceiling['enabled'] and not plane['height_m'] < float(ceiling['height_m']) - float(ceiling['margin_m'])):
            reasons.append('verified_camera_height_outside_band')
    reasons = sorted(set(reasons), key=REFUSAL_REASONS.index)
    accepted = not reasons
    return {'accepted': accepted, 'reason': None if accepted else reasons[0], 'reasons': reasons,
            'plane': plane if accepted else None, 'candidates': planes, 'estimator': estimator,
            'px4_camera_height_m': px4_height,
            'points': {'source_stride': stride, 'candidates': int(n_candidates), 'used': int(len(points))}}


def remove_floor_and_ceiling_plane(ranges, sensor_invalid, no_return, cosine, plane, contract):
    """Outdoor counterpart of remove_floor_and_ceiling: heights are measured from the camera-verified plane.

    A VALID trained pixel at optical z and level-camera ray (1, -u, -v) lies at height n.(z (1, -u, -v)) + h above
    the verified plane (normal n, camera height h). Floor: height < ground.margin_m + z * (0.5 / fy +
    ground_estimate.floor_noise_margin_per_m) -> range MaxZ, no-return; the range-dependent part covers half a
    trained pixel of angular quantisation on the z-buffer path and the stereo noise (with the z-buffer's
    nearest-sample bias), both of which grow with range. Virtual ceiling: height > ceiling.height_m -
    ceiling.margin_m -> range MaxZ, no-return (no range term: the ceiling errs towards keeping returns).
    Sensor-invalid pixels are never touched here (the outdoor profile has no invalid plane rule; the sky rule is
    separate). Exact float64; inputs are not mutated. Returns (ranges, sensor_invalid, no_return, info, roof) where
    roof [H,W] marks the valid returns removed by the (virtual) ceiling; it is reported only and never connects the
    sky rule (outdoor repair r1: the virtual ceiling drops valid returns above it and touches nothing else)."""
    from camera_policy_contract import threshold_value
    ground, ceiling, cam, depth = contract['ground'], contract['ceiling'], contract['camera'], contract['depth']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    slope = .5 / tfy + float(threshold_value(contract['ground_estimate'], 'floor_noise_margin_per_m'))
    if ranges.shape != (1, th, tw) or cosine.shape != (th, tw):
        raise InputError('floor/ceiling removal needs the trained camera grid')
    n, h = np.asarray(plane['normal'], np.float64), float(plane['height_m'])
    u, v = np.meshgrid((np.arange(tw, dtype=np.float64) - tcx) / tfx, (np.arange(th, dtype=np.float64) - tcy) / tfy)
    optical_z = ranges[0].astype(np.float64) * cosine.astype(np.float64)
    height = h + optical_z * (n[0] - n[1] * u - n[2] * v)
    si, nr = sensor_invalid[0], no_return[0]
    valid = ~(si | nr)
    floor = valid & (height < float(ground['margin_m']) + slope * optical_z)
    roof = (valid & (height > float(ceiling['height_m']) - float(ceiling['margin_m'])) if ceiling['enabled']
            else np.zeros_like(valid))
    info = {'ground': {'enabled': True, 'margin_m': float(ground['margin_m']), 'invalid_plane_rule': False,
                       'margin_per_m': slope, 'reference': 'camera_verified_plane', 'camera_height_m': h,
                       'removed_valid_pixels': int(floor.sum()), 'removed_invalid_pixels': 0},
            'ceiling': {'enabled': bool(ceiling['enabled']), 'height_m': ceiling['height_m'],
                        'margin_m': float(ceiling['margin_m']), 'invalid_plane_rule': False,
                        'physical': ceiling.get('physical', True), 'reference': 'camera_verified_plane',
                        'camera_height_m': h, 'removed_valid_pixels': int(roof.sum()), 'removed_invalid_pixels': 0}}
    out = ranges.copy()
    out[0][floor | roof] = float(depth['maxz_m'])
    return out, si[None].copy(), (nr | floor | roof)[None], info, roof


def _flood_from_top(passable):
    """Pixels of `passable` [H, W] 4-connected to row 0 through `passable`; iterative dilation to a fixed point."""
    reach = np.zeros_like(passable)
    reach[0] = passable[0]
    for _ in range(passable.size):
        grown = reach.copy()
        grown[1:] |= reach[:-1]
        grown[:-1] |= reach[1:]
        grown[:, 1:] |= reach[:, :-1]
        grown[:, :-1] |= reach[:, 1:]
        grown &= passable
        if np.array_equal(grown, reach):
            break
        reach = grown
    return reach


def sky_luma_gradient(rgb, usable):
    """Rec.601 luma of the projected RGB [1,3,H,W] in [0,1] and its local gradient max(|dL/dx|, |dL/dy|) from
    central differences among `usable` pixels (the sensor-invalid ones): a neighbour that is not usable (a valid
    return, an unimaged pixel) or outside the image is replaced by the pixel itself. The smoothness of a
    candidate is thus judged against the other invalid pixels around it, never against the edge of a valid object
    (a tree line, a removed canopy) or the black of unimaged pixels."""
    r, g, b = (rgb[0, k].astype(np.float64) for k in range(3))
    luma = .299 * r + .587 * g + .114 * b
    imaged = usable

    def neighbour(dy, dx):
        out, ok = luma.copy(), np.zeros(luma.shape, bool)
        h, w = luma.shape
        ys, yd = (slice(dy, h), slice(0, h - dy)) if dy >= 0 else (slice(0, h + dy), slice(-dy, h))
        xs, xd = (slice(dx, w), slice(0, w - dx)) if dx >= 0 else (slice(0, w + dx), slice(-dx, w))
        out[yd, xd] = luma[ys, xs]
        ok[yd, xd] = imaged[ys, xs]
        return np.where(ok, out, luma)

    gx = np.abs(neighbour(0, 1) - neighbour(0, -1)) / 2.
    gy = np.abs(neighbour(1, 0) - neighbour(-1, 0)) / 2.
    return luma, np.maximum(gx, gy)


def _unit_rays(tw, th, tfx, tfy, tcx, tcy):
    """[H, W, 3] unit level-camera rays (1, -u, -v) of the trained pixels."""
    u, v = np.meshgrid((np.arange(tw, dtype=np.float64) - tcx) / tfx, (np.arange(th, dtype=np.float64) - tcy) / tfy)
    d = np.stack([np.ones_like(u), -u, -v], axis=-1)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def edge_enclosure(sensor_invalid, edge_range, contract):
    """Outdoor repair r1 (review P0-2/P0-3): [H, W] mask of sensor-invalid pixels enclosed by near valid returns.

    Scanning a pixel's row to the left and to the right (or its column up and down) through sensor-invalid pixels
    only, the first pixel that is not sensor-invalid on each side is its stop: edge_range [H, W] holds the ray
    range of a valid return of the projection (before the floor/ceiling filters, floor returns excluded) and +inf
    elsewhere, and a stop is a near edge when its range is <= sky.edge_veto_range_m. A pixel is enclosed when
      two edges (rows and columns): both stops are near edges subtending at most sky.edge_veto_span_deg whose ranges
        differ by at most sky.edge_veto_range_tolerance_m or, outdoor repair r2 (review r2 P0-N1), whose chord is
        seen at an incidence of at most EDGE_VETO_MAX_INCIDENCE_DEG (one oblique planar face); or
      one edge (rows only, outdoor repair r2, review r2 P0-N1): exactly one stop is a near edge and the other is the
        image border or any stop that is not a near edge (unimaged/uncovered, no-return, a valid return beyond
        edge_veto_range_m, a floor return). Two near edges that are not one face (different objects) do not enclose.
    The OAK-D's horizontal stereo baseline usually matches an object's VERTICAL silhouette edges even where its
    textureless, bright face is invalid; such an enclosed pixel is never freed as sky or far ground (apply_sky_rule),
    whatever its colour. Cost: sky in the same row as a near valid return (a trunk, a post, a branch within
    edge_veto_range_m) is enclosed too and stays near (measured in REPAIR_r2.md). The enclosed set then grows by
    EDGE_VETO_DILATION_PX sensor-invalid 4-neighbour steps, so that a row whose edge sample the projection did not
    resolve (the source FOV boundary of a tilted camera, a missing edge pixel) is still protected. Pure; inputs are
    not mutated."""
    from camera_policy_contract import threshold_value
    sky, cam = contract['sky'], contract['camera']
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    si = np.asarray(sensor_invalid, bool)
    edge_range = np.asarray(edge_range, np.float64)
    if si.shape != (th, tw) or edge_range.shape != (th, tw) or np.isnan(edge_range).any():
        raise InputError('edge enclosure needs [H,W] masks and ranges of the trained camera grid')
    near = edge_range <= float(threshold_value(sky, 'edge_veto_range_m'))
    cos_span = math.cos(math.radians(float(threshold_value(sky, 'edge_veto_span_deg'))))
    tolerance = float(threshold_value(sky, 'edge_veto_range_tolerance_m'))
    sin_incidence = math.sin(math.radians(EDGE_VETO_MAX_INCIDENCE_DEG))
    rays = _unit_rays(tw, th, tfx, tfy, tcx, tcy)
    enclosed = np.zeros((th, tw), bool)
    for axis in (1, 0):                                          # rows (left/right), then columns (up/down)
        n = si.shape[axis]
        index = np.broadcast_to(np.arange(n)[None, :] if axis == 1 else np.arange(n)[:, None], si.shape)
        stop = ~si
        lo = np.maximum.accumulate(np.where(stop, index, -1), axis=axis)
        hi = np.flip(np.minimum.accumulate(np.flip(np.where(stop, index, n), axis=axis), axis=axis), axis=axis)
        lo_c, hi_c = np.clip(lo, 0, n - 1), np.clip(hi, 0, n - 1)
        rows, cols = np.indices(si.shape)
        a = (rows, lo_c) if axis == 1 else (lo_c, cols)
        b = (rows, hi_c) if axis == 1 else (hi_c, cols)
        near_a = si & (lo >= 0) & near[a]                        # the first non-invalid pixel on this side is a
        near_b = si & (hi < n) & near[b]                         # near valid return (not the image border)
        ok = near_a & near_b & (np.einsum('ijk,ijk->ij', rays[a], rays[b]) >= cos_span)
        ra, rb = np.where(ok, edge_range[a], 1.), np.where(ok, edge_range[b], 1.)
        # Outdoor repair r2 (review r2 P0-N1, oblique faces): two near edges also belong to one planar face when the
        # chord between their 3-D points is seen at an incidence of at most EDGE_VETO_MAX_INCIDENCE_DEG (the angle
        # between the face normal and the line of sight to the chord's midpoint; a frontal face has 0).
        pa, pb = rays[a] * ra[..., None], rays[b] * rb[..., None]
        chord, middle = pb - pa, pa + pb
        scale = np.linalg.norm(chord, axis=-1) * np.linalg.norm(middle, axis=-1)
        sin_inc = np.abs(np.einsum('ijk,ijk->ij', chord, middle)) / np.where(scale > 0., scale, 1.)
        ok &= (np.abs(ra - rb) <= tolerance) | (sin_inc <= sin_incidence)
        enclosed |= ok
        if axis == 1:
            # Outdoor repair r2 (review r2 P0-N1, one edge): in a ROW, a run with a near valid return on exactly one
            # side is enclosed when the other side is the image border or any stop that is not a near valid return
            # (an unimaged or uncovered pixel, a no-return beyond MaxZ, a valid return beyond edge_veto_range_m,
            # a floor return): a face whose other silhouette edge is out of view, unresolved or seen against far
            # background. One frame cannot tell it from sky beside a single near object, so that sky stays
            # sensor-invalid (the safe direction; measured phantom cost in REPAIR_r2.md). Rows only: the OAK-D's
            # horizontal baseline ranges vertical silhouette edges; columns keep the two-edge rule.
            enclosed |= near_a ^ near_b
    for _ in range(EDGE_VETO_DILATION_PX):
        grown = enclosed.copy()
        grown[1:] |= enclosed[:-1]
        grown[:-1] |= enclosed[1:]
        grown[:, 1:] |= enclosed[:, :-1]
        grown[:, :-1] |= enclosed[:, 1:]
        enclosed = grown & si
    return enclosed


def apply_sky_rule(rgb, sensor_invalid, no_return, unimaged, plane, contract, edge_range):
    """Outdoor profile: sensor-invalid pixels that are sky (or, opt-in, far ground) become no-return (as in training,
    where rays that hit nothing were no-return). Valid depth is never altered. Needs the frame's verified plane
    (normal n, camera height h) and the attitude that levelled the trained camera.

    For a trained pixel with level ray d = (1, -u, -v): it meets the verified ground iff n.d < 0, at optical
    z_g = h / -(n.d). A SENSOR-INVALID pixel becomes no-return if EITHER
      sky (rule 1): it never meets the plane and points above the gravity horizon (v < 0), its RGB is sky-like
        (luma >= luma_min and gradient <= gradient_max) and it is 4-connected to the top row of the trained image
        through such sky candidates, or through pixels above the horizon that are unimaged (outside the source
        camera's field of view: the band a nose-down camera does not image); OR
      far ground (rule 2, no RGB test; OFF unless sky.far_ground_rule is true): it meets the ground only beyond
        MaxZ (z_g > MaxZ), or it points below the gravity horizon (v > 0) and never meets the plane.
    Outdoor repair r1: (a) the virtual ceiling never connects the sky (review P0-2b): pixels whose valid return it
    removed are ordinary no-return pixels here, so sky enclosed below a removed canopy stays sensor-invalid (a
    phantom near region, the safe direction); (b) rule 2 is opt-in, default off (review P0-1): it frees dark,
    invalid structures at the vehicle's own altitude, which depth alone cannot tell from far ground; (c) a pixel
    enclosed by near valid edges (edge_enclosure, edge_range) is neither a sky candidate nor freed far ground, and
    it blocks the sky flood (review P0-2/P0-3).
    Every other sensor-invalid pixel stays sensor-invalid (a near candidate under blob_near): dark or textured
    overhead objects, near obstacles, textureless ground within MaxZ, bright blobs below the horizon.
    Returns (sensor_invalid, no_return, info); inputs are not mutated."""
    from camera_policy_contract import threshold_value
    sky, cam, depth = contract['sky'], contract['camera'], contract['depth']
    si, nr = sensor_invalid[0], no_return[0]
    far_rule = threshold_value(sky, 'far_ground_rule') is True
    info = {'enabled': sky['enabled'] is True, 'far_ground_rule': far_rule, 'far_ground_pixels': 0, 'sky_pixels': 0,
            'sky_candidate_pixels': 0, 'edge_vetoed_pixels': 0, 'unimaged_connector_pixels': 0}
    if sky['enabled'] is not True:
        return si[None].copy(), nr[None].copy(), info
    tw, th, tfx, tfy, tcx, tcy = _intrinsics(cam)
    if si.shape != (th, tw) or unimaged.shape != (th, tw) or rgb.shape != (1, 3, th, tw):
        raise InputError('sky rule needs the trained camera grid')
    n, h = np.asarray(plane['normal'], np.float64), float(plane['height_m'])
    u, v = np.meshgrid((np.arange(tw, dtype=np.float64) - tcx) / tfx, (np.arange(th, dtype=np.float64) - tcy) / tfy)
    dot = n[0] - n[1] * u - n[2] * v
    meets = dot < 0
    with np.errstate(divide='ignore'):
        z_ground = np.where(meets, h / -np.where(meets, dot, -1.), np.inf)
    maxz = float(depth['maxz_m'])
    veto = edge_enclosure(si, edge_range, contract)
    far_raw = si & ((meets & (z_ground > maxz)) | (~meets & (v > 0))) if far_rule else np.zeros_like(si)
    luma, gradient = sky_luma_gradient(rgb, si)
    raw = (si & ~meets & (v < 0) & (luma >= float(threshold_value(sky, 'luma_min')))
           & (gradient <= float(threshold_value(sky, 'gradient_max'))))
    candidate = raw & ~veto
    # Connectors: unimaged top-band pixels (project_rgbd, outdoor repair r2: never uncovered side/bottom borders) above
    # the gravity horizon. An elevation margin was measured and not adopted (REPAIR_r2.md, review r2 P1-N3/P2-N5).
    connector = np.asarray(unimaged, bool) & (v < 0)
    reach = _flood_from_top(candidate | connector)
    sky_px = candidate & reach
    far_ground = far_raw & ~veto
    freed = far_ground | sky_px
    info.update(far_ground_pixels=int(far_ground.sum()), sky_pixels=int(sky_px.sum()),
                sky_candidate_pixels=int(candidate.sum()), edge_vetoed_pixels=int((veto & (raw | far_raw)).sum()),
                unimaged_connector_pixels=int(connector.sum()))
    return (si & ~freed)[None], (nr | freed)[None], info


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


THREAD_ENV = ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS')


def pin_cpu_threads(torch, n):
    """Pin torch CPU threads to n (None: leave them); return what the process runs with. The BLAS / OpenMP pools of
    numpy are fixed at import from THREAD_ENV, so a flight launcher exports those (=1) before Python starts; their
    values are reported, not changed."""
    report = {'requested': n, 'env': {k: os.environ.get(k) for k in THREAD_ENV}}
    if n is not None:
        if type(n) is not int or n < 1:
            raise ValueError('torch_threads must be a positive int or None')
        torch.set_num_threads(n)
        try:
            torch.set_num_interop_threads(n)
            report['interop_pinned'] = True
        except RuntimeError:                    # torch allows it only before the first parallel work
            report['interop_pinned'] = False
    report['torch_num_threads'] = torch.get_num_threads()
    report['torch_num_interop_threads'] = torch.get_num_interop_threads()
    return report


class CameraPolicyRuntime:
    """One sequential CPU policy/tracker session, without any device or sender."""
    # Runtime v1: the CAM-S2 branch state (_setup_cams2); None for every other contract, also on instances that tests
    # build without __init__ (geometry-only runtimes).
    cams2 = None
    threads = None            # set by __init__ (pin_cpu_threads)

    def __init__(self, contract, torch_threads=None, projection_device=None):
        """torch_threads: None keeps the process's torch thread settings (tests share one process); an int pins torch
        intra-op threads (and inter-op threads where torch still allows it) for a flight / replay process. The vehicle
        side measured 337 ms capture -> result with 1 thread against 898 ms with 8 on the Orin NX (2026-11-26).
        projection_device: None or 'cpu' -> the CAM-S2 v2 projection in numpy; 'cuda' -> CudaProjector (bit-identical,
        float64); call warm_up_projection() before flight (first CUDA call 1.2-1.9 s on the Orin NX)."""
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
        if projection_device not in (None, 'cpu', 'cuda'):            # review r1 P3-4: before any thread change
            raise ValueError("projection_device must be None, 'cpu' or 'cuda'")
        self.threads = pin_cpu_threads(torch, torch_threads)
        self.projection_device = projection_device or 'cpu'
        self._project = CudaProjector(torch) if projection_device == 'cuda' else project_rgbd_min_valid
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
        # Runtime v1: a CAM-S2 contract applies the training's own navrl_cams2.apply_config (flight altitude for the
        # height feature, camera mount) before perception is built; every other contract is unchanged.
        self.cams2 = self._setup_cams2(tc, contract) if _is_cams2(contract) else None
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
        if self.cams2 is not None:
            # Review r1 P2-3: the checkpoint's own training snapshot must agree with the contract's receipt.
            from camera_policy_contract_cams2 import checkpoint_env_problems
            receipt_env = json.loads(Path(contract['training_record']['path']).read_text())['env_navrl']
            problems = checkpoint_env_problems(state.get('env_state'), receipt_env)
            if problems:
                raise RuntimeError('checkpoint does not belong to the contract receipt: ' + '; '.join(problems))
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
        if self.cams2 is not None:
            self._check_cams2_build()
        self.reset()

    def _setup_cams2(self, tc, contract):
        """CAM-S2 only: training settings through the training modules' own parsers (os.environ already holds the
        contract's flight environment), applied with navrl_cams2.apply_config exactly as tools/realveh_entry.py does."""
        cams2 = importlib.import_module('aerial_gym.task.navrl_task.navrl_cams2')
        camsafe = importlib.import_module('aerial_gym.task.navrl_task.navrl_camsafe')
        settings = cams2.settings_from_env(os.environ)
        safe = camsafe.settings_from_env(os.environ)
        if settings is None or safe is None:
            raise RuntimeError('CAM-S2 contract without NAVRL_CAMS2/NAVRL_CAMSAFE in its environment')
        applied = cams2.apply_config(tc, settings)
        obs, action, cam = contract['observation'], contract['action'], contract['camera']
        if (float(tc.flight_altitude) != float(obs['flight_altitude_m'])
                or [float(x) for x in tc.vision.camera_translation] != [float(x) for x in cam['translation_m']]
                or float(tc.yaw_rate_max) != float(action['yaw_rate_max_rps'])
                or float(tc.max_velocity) != float(action['max_velocity_mps'])
                or float(safe['NAVRL_CAMSAFE_BLIND_MPS']) != float(action['blind_cap_mps'])
                or [settings['NAVRL_CAMS2_DELAY_MIN'], settings['NAVRL_CAMS2_DELAY_MAX']] != action['trained_delay_steps']):
            raise RuntimeError('CAM-S2 task configuration differs from the contract')
        k_hat = int(action['previous_action_delay_steps'])
        if not 1 <= k_hat <= int(settings['NAVRL_CAMS2_DELAY_MAX']):
            raise RuntimeError('previous_action_delay_steps outside the trained delay support')
        return {'cams2': cams2, 'camsafe': camsafe, 'settings': settings, 'applied': applied,
                'blind_mps': float(safe['NAVRL_CAMSAFE_BLIND_MPS']), 'k_hat': k_hat,
                'history': int(settings['NAVRL_CAMS2_DELAY_MAX']) + 1}

    def _check_cams2_build(self):
        """CAM-S2 only: the built perception is the trained camera, and the checkpoint's own normaliser equals the
        exported one the contract binds; keeps the near-constant slots for the per-step report."""
        pm, c = self.perception, self.contract
        if ((pm.width, pm.height) != (c['camera']['width'], c['camera']['height']) or pm.detect_decoupled
                or float(pm.max_camera_range) != float(c['depth']['maxz_m'])
                or abs(float(pm.fx) - float(c['camera']['fx'])) > 1e-9
                or abs(float(pm.fy) - float(c['camera']['fy'])) > 1e-9
                or [float(x) for x in pm.camera_offset.reshape(-1).tolist()] != [float(np.float32(x)) for x in c['camera']['translation_m']]):
            raise RuntimeError('CAM-S2 perception differs from the contract camera')
        from camera_policy_contract import sha256_file
        from camera_policy_contract_cams2 import near_constant_slots
        path = c['checkpoint']['obs_norm_path']
        if sha256_file(path) != c['checkpoint']['obs_norm_sha256']:
            raise RuntimeError('observation normaliser differs from the contract')
        idx, mean, var, eps = near_constant_slots(path, c['checkpoint']['near_constant_var'])
        with np.load(path, allow_pickle=False) as data:
            count = float(data['count'])
        rms = self.audit._running_mean_std(self.player.model)
        ck_mean = rms.running_mean.detach().double().cpu().numpy()
        ck_var = rms.running_var.detach().double().cpu().numpy()
        if not (np.array_equal(ck_mean, mean) and np.array_equal(ck_var, var)):
            raise RuntimeError("exported observation normaliser differs from the checkpoint's running_mean_std")
        self.near_constant = {'index': idx, 'mean': mean[idx], 'scale': np.sqrt(var[idx] + eps)}
        # Review r1 P3-1: structural slots (one constant value on every training sample) are enforced, see step().
        floor = (1. + mean * mean) / count
        structural = np.flatnonzero(np.abs(var / floor - 1.) <= STRUCTURAL_VAR_REL_TOL)
        if len(structural) == 0:
            raise RuntimeError('no structural observation slot found in the normaliser')
        self.structural = {'index': structural, 'value': mean[structural]}

    def structural_violations(self, obs):
        """CAM-S2 only: indices of structural slots whose value is off the trained constant (refused in step())."""
        st = self.structural
        x = np.asarray(obs, np.float64)[st['index']]
        return st['index'][np.abs(x - st['value']) > STRUCTURAL_SLOT_TOLERANCE]

    def latency(self, stamps, now):
        """CAM-S2 only (review r1 P1-2): the capture-to-proposal age of this step against the trained delay.

        age = now - oldest capture stamp; implied_delay_steps = floor(age / dt + 0.5). The trained support is
        [DELAY_MIN, DELAY_MAX] control steps (1..2 for CAM-S2a). The runtime sees only the part of the vehicle latency
        up to the proposal; the gateway, MAVROS and PX4 add to it, so this is a lower bound of the k the vehicle
        imposes. A step whose implied delay already exceeds DELAY_MAX is refused (before any history advances); that
        is the same as the capture-age bound one control period later: a proposal accepted here at age <= 0.25 s is
        used until the next one, i.e. up to the 0.35 s bound."""
        dt = float(self.contract['observation']['control_dt_s'])
        age = float(now - float(np.min(stamps)))
        steps = int(math.floor(age / dt + .5))
        dmin, dmax = (int(x) for x in self.contract['action']['trained_delay_steps'])
        return {'capture_age_s': age, 'implied_delay_steps': steps,
                'previous_action_delay_steps': self.cams2['k_hat'], 'trained_delay_steps': [dmin, dmax],
                'above_trained_max': steps > dmax, 'equals_k_hat': steps == self.cams2['k_hat']}

    def near_constant_report(self, obs):
        """CAM-S2 only (report, no refusal): how far the observation sits from the trained value in the slots whose
        trained running variance is below contract checkpoint.near_constant_var, in normalised units
        |x - mean| / sqrt(var + eps); a slot beyond 1.0 is out of the trained (constant) distribution."""
        nc = self.near_constant
        x = np.asarray(obs, np.float64)[nc['index']]
        dev = np.abs(x - nc['mean']) / nc['scale']
        outside = np.flatnonzero(dev > NEAR_CONSTANT_TOLERANCE)
        return {'slots': int(len(nc['index'])), 'outside': int(len(outside)),
                'max_normalised_deviation': float(dev.max()) if len(dev) else 0.,
                'outside_indices': nc['index'][outside][:32].tolist()}

    def reset(self):
        # Perception rolls histories during inference, producing inference-mode
        # tensors. Reset them under the same mode, including after failed ticks.
        with self.torch.inference_mode():
            self.perception.reset_idx(self.torch.tensor([0]))
        # Vectorized training resets individual environments while retaining its
        # global counter. This standalone reset starts a whole new history clock.
        self.perception.step_count = 0
        self.prev = self.torch.zeros((1, 4), dtype=self.torch.float32)
        if self.cams2 is not None:
            # Training zeroes the delayed-command history at every reset (navrl_cams2.reset_idx).
            self._action_history = self.torch.zeros((self.cams2['history'], 4), dtype=self.torch.float32)
        self._last_stamps = None
        self._last_now = None
        self.failed = False

    def invalidate(self):
        """Stop reuse after the companion rejects a tick following inference."""
        self.failed = True

    def applied_zero_hold(self):
        """The ROS session proposed an explicit zero hold instead of the command of the step that just finished
        (outdoor re-entry hysteresis, camera_ros2_session.py): the next observation's previous action is the
        applied zero command (zero velocity and yaw rate), as in training, where it is the applied governed command."""
        if self.failed:
            raise RuntimeError('runtime failed after state advance; reset required')
        if self.cams2 is not None:
            # CAM-S2: the held tick's own action is zero (newest history slot, z and yaw); the xy channel is the
            # command of k_hat >= 1 steps ago, which the hold does not change.
            history = self._action_history.clone()
            history[0] = 0.
            self._action_history = history
            self.prev = self.prev.clone()
            self.prev[:, 2:] = 0.
            return
        self.prev = self.torch.zeros((1, 4), dtype=self.torch.float32)

    def height_band(self, state):
        """height_band() with this runtime's contract; reads no history and changes nothing.
        Outdoor profile: PX4's vertical-validity fields in state are ignored here (split_state)."""
        return height_band(state, self.contract)

    @property
    def capture_age_bound_s(self):
        """Capture-age bound from the contract's trained action delay (camera_policy_contract.capture_age_bound_s)."""
        from camera_policy_contract import capture_age_bound_s
        return capture_age_bound_s(self.contract)

    def _validate_timing(self, frame, state, now_s):
        try:
            stamps = np.array([frame['rgb_t'], frame['depth_t'], state['t']], np.float64)
            now = float(now_s)
        except (TypeError, ValueError, KeyError):
            raise InputError('missing/invalid shared-clock timestamps') from None
        if stamps.shape != (3,) or not np.isfinite(stamps).all() or not math.isfinite(now):
            raise InputError('timestamps must be finite shared-clock seconds')
        ages = now-stamps
        if np.any(stamps < 0) or np.any(ages < -1e-9) or np.any(ages > self.capture_age_bound_s+1e-9):
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
        The outdoor profile (contract profile 'outdoor') takes _preprocess_outdoor instead; every other
        contract runs exactly the code below.
        """
        if _is_outdoor(self.contract):
            return self._preprocess_outdoor(frame, state)
        if self.cams2 is not None:
            return self._preprocess_cams2(frame, state)
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

    def _preprocess_cams2(self, frame, state):
        """CAM-S2: min-valid projection -> optional floor/ceiling removal of valid returns -> every pixel without a
        valid return (sensor-invalid, not imaged, out of range, removed floor/ceiling) holds perception.max_camera_range
        (= MaxZ), the training's no-return encoding. No blob_near, no goal carve-out (an invalid goal pixel already
        fails the detector's depth < max_camera_range gate, as in training) and no RGB blanking: training never blanks
        RGB at invalid depth."""
        project = getattr(self, '_project', project_rgbd_min_valid)       # instances built without __init__: CPU
        rgb, ranges, invalid, no_return, cosine, projection = project(frame, state, self.contract)
        ranges, invalid, no_return, bands = remove_floor_and_ceiling(ranges, invalid, no_return, cosine,
                                                                     state, self.contract)
        far = np.float32(self.perception.max_camera_range)
        absent = invalid | no_return
        ranges = np.where(absent, far, ranges).astype(np.float32)
        counts = {'sensor_invalid_pixels': int(invalid.sum()), 'no_return_pixels': int(no_return.sum()),
                  'near_pixels': 0, 'goal_invalid_pixels': 0,
                  'ground_removed_pixels': bands['ground']['removed_valid_pixels'],
                  'ceiling_removed_pixels': bands['ceiling']['removed_valid_pixels'],
                  'height_filter': bands, 'rgb_blanked_pixels': 0}
        torch = self.torch
        return torch.from_numpy(np.ascontiguousarray(rgb)), torch.from_numpy(ranges), counts, projection

    def warm_up_projection(self, frame, state, repeats=3):
        """Run the CAM-S2 projection `repeats` times on a real frame before flight; returns the wall times (s)."""
        if isinstance(self._project, CudaProjector):
            return self._project.warm_up(frame, state, self.contract, repeats)
        import time
        out = []
        for _ in range(repeats):
            t = time.perf_counter()
            self._project(frame, state, self.contract)
            out.append(time.perf_counter() - t)
        return out

    def _govern(self, cmd, perception=None):
        """The trained no_lidar riskcap governor on a vehicle-frame command [1, 2] m/s (navrl_sensor_ablation
        install_governor + navrl_task._apply_sensor_speed_governor), on perception's current frame."""
        torch = self.torch
        perception = self.perception if perception is None else perception
        no_lidar = torch.full((1, 288), float(self.tc.lidar_max_range))
        scan = self.ablation.governor_scan(perception, 'no_lidar', no_lidar.reshape(1, 4, 72),
                                          float(self.tc.lidar_max_range), self.vfov)
        half = self.sg.speed_dependent_half_width(self.gov_cfg, cmd.norm(dim=1), open_m=None)
        clearance = self.sg.directional_lidar_clearance(scan, self.bearings, cmd,
            max_range_m=float(self.tc.lidar_max_range), path_half_width_m=half,
            target_return_mask=perception.last_target_like)
        clearance = torch.where(self.ablation.out_of_fov_commands(perception, cmd),
                                torch.zeros_like(clearance), clearance)
        governed, _ = self.sg.apply_speed_governor(cmd, clearance, self.gov_cfg, lateral_clearance_m=None)
        return governed

    def _cams2_command(self, a, governed, perception=None):
        """CAM-S2 (review r1 F1): (published xy [1,2], previous action [1,4], new history, blind mask).

        published = camsafe.cap_blind(governed a_t): the current command, the vehicle's latency delays it.
        previous action as trained: xy = clamp(cap_blind(governor(a_{t-k_hat} xy * v_max)) / v_max) from the history
        of the runtime's own clamped actions (zero before the first action after a reset), z/yaw = a_t."""
        torch, c2 = self.torch, self.cams2
        perception = self.perception if perception is None else perception
        vmax = float(self.tc.max_velocity)
        half = float(perception.hfov) * 0.5
        published, blind = c2['camsafe'].cap_blind(governed, half, c2['blind_mps'])
        history = torch.roll(self._action_history, 1, dims=0)
        history[0] = a[0]
        delayed = c2['cams2'].delayed(history.unsqueeze(0), torch.tensor([c2['k_hat']]))
        delayed_capped, _ = c2['camsafe'].cap_blind(self._govern(delayed[:, :2].clamp(-1., 1.) * vmax, perception),
                                                    half, c2['blind_mps'])
        prev = torch.cat([(delayed_capped / vmax).clamp(-1., 1.), a[:, 2:]], dim=1)
        return published, prev, history, blind

    def _preprocess_outdoor(self, frame, state):
        """Outdoor profile: projection -> ground estimate (refuse the frame with FrameRefused if the plane cannot be
        verified) -> goal carve-out -> floor/virtual-ceiling removal of valid returns relative to the verified plane
        -> sky rule on sensor-invalid pixels (edge-enclosure veto from the valid returns before the floor/ceiling
        filters, floor returns excluded; the virtual ceiling never connects the sky) -> blob_near -> blanking.
        state may carry PX4's z_valid/epv/dead_reckoning (split_state). Nothing here reads or changes history."""
        base, _ = split_state(state, self.contract)
        rgb, ranges, invalid, no_return, cosine, projection, unimaged = project_rgbd(frame, base, self.contract,
                                                                                    unimaged=True)
        ground = estimate_ground(frame, state, self.contract)
        if not ground['accepted']:
            raise FrameRefused(ground['reason'], ground)
        goal = invalid & self.goal_coloured(rgb, ranges)
        invalid, no_return = invalid & ~goal, no_return | goal
        returned = ~(invalid | no_return)[0]
        measured = ranges[0].astype(np.float64)
        filtered_no_return = no_return
        ranges, invalid, no_return, bands, roof = remove_floor_and_ceiling_plane(ranges, invalid, no_return, cosine,
                                                                                 ground['plane'], self.contract)
        floor = no_return[0] & ~filtered_no_return[0] & ~roof
        edge_range = np.where(returned & ~floor, measured, np.inf)
        invalid, no_return, sky = apply_sky_rule(rgb, invalid, no_return, unimaged, ground['plane'], self.contract,
                                                 edge_range)
        torch = self.torch
        depth, near = self.validity.apply_mask(torch.from_numpy(ranges), torch.from_numpy(invalid),
                                              torch.from_numpy(cosine), min_z_m=self.contract['depth']['minz_m'],
                                              window=self.contract['depth']['window'], density=self.contract['depth']['density'])
        near_np = near.numpy()
        blank = (invalid | no_return | near_np) & ~goal
        rgb = np.where(blank[:, None], np.float32(0.), rgb).astype(np.float32)
        counts = {'sensor_invalid_pixels': int(invalid.sum()), 'no_return_pixels': int(no_return.sum()),
                  'near_pixels': int(near_np.sum()), 'goal_invalid_pixels': int(goal.sum()),
                  'ground_removed_pixels': bands['ground']['removed_valid_pixels'],
                  'ceiling_removed_pixels': bands['ceiling']['removed_valid_pixels'],
                  'height_filter': bands, 'rgb_blanked_pixels': int(blank.sum()),
                  'outdoor': {'ground': ground, 'sky': sky}}
        return torch.from_numpy(rgb), depth, counts, projection

    def step(self, frame, state, *, now_s):
        if self.failed:
            raise RuntimeError('runtime failed after state advance; reset required')
        # Outdoor profile only: PX4's vertical-validity fields travel beside the estimator state (split_state).
        estimator_state, _ = split_state(state, self.contract)
        p, v, _, yaw, rate, _ = validate_state(estimator_state)
        stamps, now = self._validate_timing(frame, estimator_state, now_s)
        latency = None
        if self.cams2 is not None:
            latency = self.latency(stamps, now)
            if latency['above_trained_max']:
                raise InputError('capture-to-proposal age %.3f s implies %d control steps of delay, above the trained '
                                 'maximum %d' % (latency['capture_age_s'], latency['implied_delay_steps'],
                                                 latency['trained_delay_steps'][1]))
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
            if self.cams2 is not None:
                bad = self.structural_violations(obs[0].numpy())
                if len(bad):
                    # After observe the perception history has advanced: failed stays True, reset required.
                    raise RuntimeError('structural observation slots differ from their trained constants: %s'
                                       % bad[:16].tolist())
            a = self.audit.deterministic_actions(self.player, obs).reshape(1, 4)
            if not bool(torch.isfinite(a).all()) or bool((a.abs() > 1.000001).any()):
                raise RuntimeError('invalid actor output')
            a = a.clamp(-1., 1.)
            cmd = a[:, :2] * float(self.tc.max_velocity)
            governed = self._govern(cmd)
            history = blind = None
            if self.cams2 is None:
                prev = torch.cat([(governed/float(self.tc.max_velocity)).clamp(-1., 1.), a[:, 2:]], dim=1)
                published = governed
            else:
                published, prev, history, blind = self._cams2_command(a, governed)
            if not bool(torch.isfinite(published).all()) or not bool(torch.isfinite(prev).all()):
                raise RuntimeError('invalid governed command')
        vx, vy = published[0].tolist()
        self.prev = prev
        if history is not None:
            self._action_history = history
        self._last_stamps, self._last_now, self.failed = stamps, now, False
        result = {'t': now, 'velocity_world_enu_mps': [math.cos(yaw)*vx-math.sin(yaw)*vy,
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
        if self.cams2 is not None:
            result.update(lineage='CAM-S2', variant=self.contract['variant'], blind_capped=bool(blind[0]),
                          governed_velocity_vehicle_flu_mps=governed[0].tolist(),
                          previous_action_delay_steps=self.cams2['k_hat'], latency=latency,
                          # Vehicle interface (ROS 1 design D4): publish velocity_vehicle_flu_mps (level body frame,
                          # forward/left) with this capture yaw; the gateway rotates once with the LATEST yaw.
                          # velocity_world_enu_mps (rotated by the capture yaw) is kept for existing readers only.
                          capture_yaw_rad=yaw, proposal_frame='level_body_flu',
                          near_constant_slots=self.near_constant_report(result['observation']))
        if 'outdoor' in counts:
            ground = counts['outdoor']['ground']
            result['outdoor'] = {'ground_plane': ground['plane'], 'ground_points': ground['points'],
                                 'sky': counts['outdoor']['sky']}
        return result


def replay(runtime, data, output):
    """Recorded inputs only; no target state or simulator is ever constructed."""
    meta = json.loads(str(data['camera_metadata'].item()))
    required = ('rgb', 'depth_z_m', 'invalid', 'rgb_t', 'depth_t', 'pos', 'vel', 'quat_xyzw', 'yaw_rate_rps', 'state_t', 'now_s')
    # Optional per-frame source coverage (False = not imaged, e.g. undistortion border).
    optional = ('coverage',) if 'coverage' in data else ()
    # Outdoor profile: optional PX4 vertical-validity arrays (all three or none); without them every outdoor
    # frame is refused (estimator_vertical_fields_absent).
    vertical = VERTICAL_FIELDS if _is_outdoor(runtime.contract) and all(k in data for k in VERTICAL_FIELDS) else ()
    counts = {len(data[k]) for k in required + optional + vertical}
    if len(counts) != 1 or next(iter(counts)) == 0:
        raise InputError('replay arrays must contain equal nonzero frame counts')
    outdoor = _is_outdoor(runtime.contract)
    count = refused = held = 0
    # Outdoor repair r2 (review r2 P3-N8): the ROS session's re-entry hysteresis. The replay starts from a fresh
    # history, so (outdoor profile only) the first REFUSAL_REENTRY_FRAMES accepted frames, and as many after every
    # refused frame, propose the explicit zero hold while the policy history refills; the policy's own command is kept
    # in the line as policy_*, and the runtime records the applied zero hold. Other contracts are unchanged.
    hold = REFUSAL_REENTRY_FRAMES if outdoor else 0
    # Unit B (2026-11-18), outdoor only and report-only: the ROS session's refusal-hold episode on the same capture
    # stamps (the oldest of rgb_t, depth_t and state_t); the line at which the session would latch
    # refusal_hold_timeout is marked. The replay keeps processing afterwards (the session would propose nothing more).
    episode = RefusalHoldEpisode(refusal_hold_limit_s(runtime.contract)) if outdoor else None
    timeout_at = None
    with Path(output).open('x') as stream:
        for i in range(next(iter(counts))):
            frame = dict(meta, **{k: data[k][i] for k in ('rgb', 'depth_z_m', 'invalid', 'rgb_t', 'depth_t') + optional})
            state = {k: data[k][i] for k in ('pos', 'vel', 'quat_xyzw', 'yaw_rate_rps')}
            state['t'] = data['state_t'][i]
            state.update({k: data[k][i].item() for k in vertical})
            try:
                result = runtime.step(frame, state, now_s=data['now_s'][i])
            except FrameRefused as exc:
                # Outdoor profile only: the frame is refused (no history advanced); as in the ROS session, the
                # proposal is an explicit zero hold and the next accepted frame starts a fresh history.
                runtime.reset()
                hold = REFUSAL_REENTRY_FRAMES
                result = {'t': float(data['now_s'][i]), 'frame_refused': exc.reason, 'velocity_world_enu_mps': [0., 0., 0.],
                          'yaw_rate_rps': 0., 'hold': True, 'command_sent': False}
                refused += 1
            else:
                if hold > 0:
                    runtime.applied_zero_hold()
                    hold -= 1
                    held += 1
                    result = dict(result, policy_velocity_world_enu_mps=result.get('velocity_world_enu_mps'),
                                  policy_yaw_rate_rps=result.get('yaw_rate_rps'), velocity_world_enu_mps=[0., 0., 0.],
                                  velocity_vehicle_flu_mps=[0., 0., 0.], yaw_rate_rps=0., hold=True,
                                  previous_action=[0., 0., 0., 0.], reentry_hold_remaining=hold)
            if episode is not None:
                if result.get('hold') is True:
                    capture = min(float(data[k][i]) for k in ('rgb_t', 'depth_t', 'state_t'))
                    elapsed, timed_out = episode.hold(capture)
                    result['hold_episode_s'] = elapsed
                    if timed_out and timeout_at is None:
                        timeout_at = capture
                        result[REFUSAL_HOLD_TIMEOUT] = True
                else:
                    episode.command()
            stream.write(json.dumps(result, allow_nan=False) + '\n')
            count += 1
    report = {'status': 'CPU_REPLAY_COMPLETE', 'frames': count, 'commands_sent': 0,
              'gazebo': 'NOT_TESTED', 'hardware': 'NOT_TESTED'}
    if outdoor:
        report['refused_frames'] = refused
        report['reentry_held_frames'] = held
        report['refusal_hold_max_s'] = episode.limit_s
        report['longest_hold_episode_s'] = episode.longest_s
        report['refusal_hold_timeout_capture_s'] = timeout_at
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--contract', required=True)
    ap.add_argument('--replay', required=True, help='aligned measured input NPZ, allow_pickle=False')
    ap.add_argument('--output', required=True, help='new JSONL path for proposed commands, never sent')
    ap.add_argument('--torch-threads', type=int, default=1, help='torch CPU threads (default 1, the measured faster '
                    'setting on the Orin NX); export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 before starting Python')
    args = ap.parse_args(argv)
    from camera_policy_contract import load_contract
    if Path(args.output).exists():
        raise ValueError('output exists; choose a fresh path')
    runtime = CameraPolicyRuntime(load_contract(args.contract), torch_threads=args.torch_threads)
    with np.load(args.replay, allow_pickle=False) as data:
        report = replay(runtime, data, args.output)
    report['threads'] = runtime.threads
    report['projection_device'] = runtime.projection_device
    print(json.dumps(report, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
