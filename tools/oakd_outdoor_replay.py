"""Replay recorded OAK-D outdoor frames through the camera runtime's OUTDOOR profile (perception only).

Purpose: calibrate and check the PROVISIONAL outdoor thresholds (sky rule, ground estimate) on the user's own OAK-D,
recorded on a tripod outdoors. No policy, checkpoint, simulator, ROS graph, flight controller or DepthAI SDK is
used: the frames run through the production projection, ground estimate, plane floor / virtual-ceiling filter, sky
rule and blob_near of tools/camera_policy_runtime.py (CameraPolicyRuntime.preprocess without the checkpoint). The
production colour-target carve-out needs the trained perception segmenter, so it is NOT applied here (no pixel is
treated as the goal); record without red objects in view.

Recording folder (format motar-oakd-outdoor-recording-v1):

  <folder>/recording.json
    {"format": "motar-oakd-outdoor-recording-v1",
     "source": "oakd_recording" | "synthetic",          # synthetic folders are never reported as real data
     "device": {...},                                     # free text: model, serial, DepthAI version, stereo mode
     "intrinsics": {"width", "height", "fx", "fy", "cx", "cy"},   # RECTIFIED, depth aligned to RGB (oakd_camera.Rectifier.metadata())
     "camera_height_m": <tape-measured height of the RGB optical centre above the ground>,
     "camera_height_uncertainty_m": <tape reading uncertainty, e.g. 0.02>,
     "scene": "<free text: site, surface, sky, objects and their distances>",
     "frames": ["frames/000000.npz", ...]}                # relative paths, in time order
  <folder>/frames/NNNNNN.npz   (numpy, allow_pickle=False)
    rgb       uint8  [H, W, 3]  RGB order, rectified, aligned to depth
    depth_mm  uint16 [H, W]     optical z in millimetres, 0 = invalid (the OAK-D's own invalid value)
    up_optical float64 [3]      unit vector pointing UP (against gravity) in the camera optical frame
                                (x right, y down, z forward), from the OAK-D IMU accelerometer averaged while still
                                and rotated by the device's IMU-to-RGB extrinsics; OR
    roll_pitch_deg float64 [2]  camera roll and pitch (positive pitch = nose down), if not up_optical
    t         float64 []        capture time [s], strictly increasing
    coverage  bool  [H, W]      optional: False outside the raw image after rectification

Per frame the replay state is: camera FLU = body FLU (identity rotation), mount = the contract camera translation,
the body z chosen so that the real optical centre sits at the tape height, yaw 0, zero velocity, and PX4 fields
z_valid = true, epv = camera_height_uncertainty_m, dead_reckoning = false. The ground estimate is therefore
cross-checked against the TAPE height: its height error is the measured accuracy of the camera ground fit.

Output: a JSONL line per frame (refusal reason, ground plane, sky / far-ground / near statistics) and a summary
JSON. Exit code 0 when every frame was processed (refusals are results, not errors).

Re-entry hysteresis (outdoor repair r2, review r2 P3-N8): as in the ROS session, the first REFUSAL_REENTRY_FRAMES
accepted frames of the recording and after every refused frame are marked reentry_hold (the session would propose
the explicit zero hold on them while the policy history refills); the summary counts them. No policy runs here.

Bounded hold (unit B, 2026-11-18): each row carries hold_episode_s, the session's refusal-hold episode at that frame
(camera_policy_runtime.RefusalHoldEpisode on the frame times t; None on a frame that would carry a policy command), and
the first frame at which the session would latch refusal_hold_timeout (the contract's PROVISIONAL
ground_estimate.refusal_hold_max_s) is marked refusal_hold_timeout: true and named in the summary. Frame times of a
tripod recording need not be 10 Hz; the episode uses them as recorded.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FORMAT = 'motar-oakd-outdoor-recording-v1'
SOURCES = ('oakd_recording', 'synthetic')
RECORDING_KEYS = {'format', 'source', 'device', 'intrinsics', 'camera_height_m', 'camera_height_uncertainty_m',
                  'scene', 'frames'}


class RecordingError(ValueError):
    pass


def _tools():
    if str(ROOT / 'tools') not in sys.path:
        sys.path.insert(0, str(ROOT / 'tools'))
    import camera_policy_contract as contract_module
    import camera_policy_runtime as runtime_module
    return contract_module, runtime_module


class _NoGoalPerception:
    """The colour-target carve-out needs the trained segmenter (checkpoint): here no pixel is goal-coloured."""
    pixel_threshold = .55
    max_camera_range = 20.

    @staticmethod
    def segmenter(rgb, depth, max_depth):
        import torch
        return torch.zeros(depth.shape)


def geometry_runtime(contract):
    """CameraPolicyRuntime's production preprocess without the checkpoint: the outdoor contract, the production
    blob_near (aerial_gym/task/navrl_task/navrl_depth_validity.py loaded under a private name, without importing the
    aerial_gym package) and the no-goal perception stand-in."""
    import torch
    _, R = _tools()
    spec = importlib.util.spec_from_file_location('oakd_outdoor_replay_depth_validity',
                                                  str(ROOT / 'aerial_gym/task/navrl_task/navrl_depth_validity.py'))
    validity = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validity)
    rt = object.__new__(R.CameraPolicyRuntime)
    rt.contract, rt.validity, rt.torch, rt.perception = contract, validity, torch, _NoGoalPerception()
    return rt


def _rotation(roll_deg, pitch_deg):
    """FLU body -> ENU world with yaw 0 (rz @ rx @ ry, the convention of the camera runtime tests)."""
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return rx @ ry


def _quaternion_xyzw(m):
    w = math.sqrt(max(0., 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0., 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = math.copysign(math.sqrt(max(0., 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = math.copysign(math.sqrt(max(0., 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([x, y, z, w])
    return (q / np.linalg.norm(q)).tolist()


def roll_pitch_from_up(up_optical):
    """Camera roll/pitch [deg] from the up vector in the optical frame (x right, y down, z forward)."""
    up = np.asarray(up_optical, np.float64)
    if up.shape != (3,) or not np.isfinite(up).all() or not .5 < np.linalg.norm(up) < 2.:
        raise RecordingError('up_optical must be a finite 3-vector of about unit length')
    up = up / np.linalg.norm(up)
    flu = np.array([up[2], -up[0], -up[1]])
    return math.degrees(math.asin(max(-1., min(1., flu[1])))), math.degrees(math.atan2(-flu[0], flu[2]))


def load_recording(folder):
    folder = Path(folder)
    try:
        meta = json.loads((folder / 'recording.json').read_text())
    except (OSError, ValueError) as exc:
        raise RecordingError('recording.json unreadable: %s' % exc) from None
    if not isinstance(meta, dict) or set(meta) != RECORDING_KEYS or meta['format'] != FORMAT:
        raise RecordingError('recording.json must have exactly %s and format %s' % (sorted(RECORDING_KEYS), FORMAT))
    if meta['source'] not in SOURCES:
        raise RecordingError('source must be one of %s' % (SOURCES,))
    for key in ('camera_height_m', 'camera_height_uncertainty_m'):
        value = meta[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise RecordingError('%s must be a positive number of metres' % key)
    frames = meta['frames']
    if not isinstance(frames, list) or not frames or any(not isinstance(p, str) or Path(p).is_absolute() or '..' in Path(p).parts
                                                         for p in frames):
        raise RecordingError('frames must be a nonempty list of relative paths inside the folder')
    return meta


def frame_inputs(folder, meta, relative, contract):
    """One recorded frame -> (runtime frame, outdoor state, roll, pitch, t)."""
    try:
        with np.load(Path(folder) / relative, allow_pickle=False) as data:
            arrays = {k: data[k] for k in data.files}
    except (OSError, ValueError) as exc:
        raise RecordingError('%s unreadable: %s' % (relative, exc)) from None
    rgb, depth = arrays.get('rgb'), arrays.get('depth_mm')
    k = meta['intrinsics']
    shape = (k.get('height'), k.get('width')) if isinstance(k, dict) else None
    if rgb is None or rgb.dtype != np.uint8 or rgb.shape != shape + (3,):
        raise RecordingError('%s: rgb must be uint8 [H, W, 3] at the recorded intrinsics' % relative)
    if depth is None or depth.dtype != np.uint16 or depth.shape != shape:
        raise RecordingError('%s: depth_mm must be uint16 [H, W] (0 = invalid)' % relative)
    if ('up_optical' in arrays) == ('roll_pitch_deg' in arrays):
        raise RecordingError('%s: exactly one of up_optical or roll_pitch_deg is required' % relative)
    if 'up_optical' in arrays:
        roll, pitch = roll_pitch_from_up(arrays['up_optical'])
    else:
        rp = np.asarray(arrays['roll_pitch_deg'], np.float64)
        if rp.shape != (2,) or not np.isfinite(rp).all():
            raise RecordingError('%s: roll_pitch_deg must be two finite degrees' % relative)
        roll, pitch = float(rp[0]), float(rp[1])
    t = arrays.get('t')
    if t is None or np.asarray(t).shape != () or not math.isfinite(float(t)):
        raise RecordingError('%s: t must be a finite scalar' % relative)
    frame = {'rgb': rgb, 'depth_z_m': depth.astype(np.float32) * np.float32(.001), 'invalid': depth == 0,
             'rgb_t': float(t), 'depth_t': float(t), 'intrinsics': dict(k), 'aligned': True, 'rectified': True,
             'mount_flu_m': list(contract['camera']['translation_m']), 'camera_to_body_rotation': np.eye(3)}
    if 'coverage' in arrays:
        coverage = arrays['coverage']
        if coverage.dtype != np.bool_ or coverage.shape != shape:
            raise RecordingError('%s: coverage must be bool [H, W]' % relative)
        frame['coverage'] = coverage
    body = _rotation(roll, pitch)
    z = float(meta['camera_height_m']) - float((body @ np.asarray(frame['mount_flu_m'], np.float64))[2])
    state = {'pos': [0., 0., z], 'vel': [0., 0., 0.], 'quat_xyzw': _quaternion_xyzw(body), 'yaw_rate_rps': 0.,
             't': float(t), 'z_valid': True, 'epv': float(meta['camera_height_uncertainty_m']), 'dead_reckoning': False}
    return frame, state, roll, pitch, float(t)


def replay_recording(folder, contract):
    """Process every frame; returns (per-frame rows, summary). Refusals are results."""
    C, R = _tools()
    if not C.is_outdoor(contract):
        raise RecordingError('the replay needs an outdoor-profile contract (camera_policy_contract.py --profile outdoor)')
    meta = load_recording(folder)
    rt = geometry_runtime(contract)
    rows, last_t = [], None
    tape = float(meta['camera_height_m'])
    hold = R.REFUSAL_REENTRY_FRAMES                     # the replay starts from a fresh history (review r2 P3-N8)
    # Unit B (2026-11-18): the ROS session's refusal-hold episode on the frames' capture times t (refused and
    # reentry_hold frames are holds, other accepted frames would carry a policy command). The first frame at which the
    # session would latch refusal_hold_timeout is marked; the replay keeps processing (calibration only).
    episode = R.RefusalHoldEpisode(R.refusal_hold_limit_s(contract))
    timeout = None

    def hold_tick(row, holding):
        nonlocal timeout
        if not holding:
            episode.command()
            row['hold_episode_s'] = None
            return
        elapsed, timed_out = episode.hold(row['t'])
        row['hold_episode_s'] = elapsed
        if timed_out and timeout is None:
            timeout = {'index': row['index'], 't': row['t'], 'episode_s': elapsed}
            row[R.REFUSAL_HOLD_TIMEOUT] = True
    for index, relative in enumerate(meta['frames']):
        frame, state, roll, pitch, t = frame_inputs(folder, meta, relative, contract)
        if last_t is not None and not t > last_t:
            raise RecordingError('%s: timestamps must increase' % relative)
        last_t = t
        row = {'index': index, 'file': relative, 't': t, 'roll_deg': roll, 'pitch_deg': pitch,
               'source_invalid_share': float(frame['invalid'].mean())}
        try:
            _, depth, counts, projection = rt.preprocess(frame, state)
        except R.FrameRefused as exc:
            report = exc.report or {}
            row.update(refused=exc.reason, reasons=report.get('reasons', [exc.reason]),
                       candidates=[{k: q[k] for k in ('height_m', 'tilt_deg', 'inliers', 'share', 'below_share', 'failed')}
                                   for q in report.get('candidates', [])],
                       points=report.get('points'))
            hold_tick(row, True)
            rows.append(row)
            hold = R.REFUSAL_REENTRY_FRAMES
            continue
        ground = counts['outdoor']['ground']
        plane = ground['plane']
        cols = depth[0].amin(dim=0).numpy()
        row.update(refused=None, reasons=[],
                   ground={k: plane[k] for k in ('height_m', 'tilt_deg', 'inliers', 'share', 'below_share', 'rms_m')},
                   # The replay's 'PX4' height is the tape height (moved to the trained camera), so the plane's
                   # height error is the camera fit's error against the tape.
                   ground_height_minus_tape_m=plane['height_error_m'],
                   points=ground['points'], sky=counts['outdoor']['sky'], projection=projection['projection'],
                   near_pixels=counts['near_pixels'], sensor_invalid_pixels=counts['sensor_invalid_pixels'],
                   ground_removed_pixels=counts['ground_removed_pixels'],
                   ceiling_removed_pixels=counts['ceiling_removed_pixels'],
                   near_columns=int((cols < 1.).sum()), obstacle_columns=int((cols < 10.).sum()),
                   min_column_m=float(cols.min()), reentry_hold=hold > 0)
        hold_tick(row, hold > 0)
        hold = max(0, hold - 1)
        rows.append(row)
    accepted = [r for r in rows if r['refused'] is None]
    reasons = {}
    for r in rows:
        if r['refused'] is not None:
            reasons[r['refused']] = reasons.get(r['refused'], 0) + 1
    errors = np.array([r['ground_height_minus_tape_m'] for r in accepted]) if accepted else np.zeros(0)

    def stats(values):
        if not len(values):
            return None
        a = np.abs(values)
        return {'median_abs': float(np.median(a)), 'p95_abs': float(np.percentile(a, 95)), 'max_abs': float(a.max()),
                'mean': float(np.mean(values))}
    summary = {'format': 'motar-oakd-outdoor-replay-summary-v1', 'recording_format': FORMAT, 'source': meta['source'],
               'evidence': 'REAL_OAKD_RECORDING' if meta['source'] == 'oakd_recording' else 'SYNTHETIC_NOT_EVIDENCE',
               'frames': len(rows), 'accepted': len(accepted), 'refused': len(rows) - len(accepted),
               'refusal_reasons': reasons, 'ground_height_error_vs_tape_m': stats(errors),
               'ground_tilt_deg': stats(np.array([r['ground']['tilt_deg'] for r in accepted])) if accepted else None,
               'frames_with_near_columns': sum(1 for r in accepted if r['near_columns'] > 0),
               'near_columns_max': max((r['near_columns'] for r in accepted), default=None),
               'frames_with_obstacle_columns': sum(1 for r in accepted if r['obstacle_columns'] > 0),
               'sky_pixels_total': sum(r['sky']['sky_pixels'] for r in accepted),
               'reentry_hold_frames_after_reset': R.REFUSAL_REENTRY_FRAMES,
               'reentry_held_frames': sum(1 for r in accepted if r['reentry_hold']),
               'frames_free_to_command': sum(1 for r in accepted if not r['reentry_hold']),
               'far_ground_pixels_total': sum(r['sky']['far_ground_pixels'] for r in accepted),
               # Unit B: the session's bounded hold on these capture times (PROVISIONAL limit from the contract).
               'refusal_hold_max_s': episode.limit_s, 'longest_hold_episode_s': episode.longest_s,
               'refusal_hold_timeout': timeout,
               'thresholds': {'sky': contract['sky'], 'ground_estimate': contract['ground_estimate']},
               'camera_height_m': tape, 'scene': meta['scene'], 'goal_carve_out': 'NOT_APPLIED (no checkpoint)',
               'policy': 'NOT_RUN', 'hardware_flight': 'NOT_TESTED'}
    return rows, summary


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--contract', required=True, help='outdoor-profile contract (camera_policy_contract.py --profile outdoor)')
    ap.add_argument('--recording', required=True, help='recording folder (format above)')
    ap.add_argument('--output', required=True, help='new directory for frames.jsonl and summary.json')
    ap.add_argument('--no-verify-files', action='store_true',
                    help='skip hashing the checkpoint and runtime sources bound by the contract (the replay uses no '
                         'checkpoint); the structure and the training record are still verified')
    args = ap.parse_args(argv)
    C, _ = _tools()
    out = Path(args.output)
    try:
        contract = C.load_contract(args.contract, verify_files=not args.no_verify_files)
        out.mkdir(parents=True, exist_ok=False)
        rows, summary = replay_recording(args.recording, contract)
    except (OSError, ValueError) as exc:
        raise SystemExit('oakd outdoor replay refused: %s' % exc) from None
    summary['contract_sha256'] = C.sha256_file(args.contract)
    summary['contract_files_verified'] = not args.no_verify_files
    with (out / 'frames.jsonl').open('x') as stream:
        for row in rows:
            stream.write(json.dumps(row, allow_nan=False, sort_keys=True) + '\n')
    with (out / 'summary.json').open('x') as stream:
        json.dump(summary, stream, indent=1, sort_keys=True, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: summary[k] for k in ('source', 'evidence', 'frames', 'accepted', 'refused', 'refusal_reasons')},
                     sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
