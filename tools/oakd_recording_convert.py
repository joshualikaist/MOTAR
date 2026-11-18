"""Convert an OAK-D field capture (tools/oakd_field_recorder.py) into the outdoor replay recording format (PC side).

Input:  a capture folder, format motar-oakd-raw-capture-v1 (capture.json, MANIFEST.sha256, frames/NNNNNN.npz with the
        ISP colour frame as delivered, depth aligned to CAM_A by the device, accelerometer samples and timestamps).
Output: a NEW folder, format motar-oakd-outdoor-recording-v1, as tools/oakd_outdoor_replay.load_recording and
        frame_inputs read it (recording.json, frames/NNNNNN.npz with rgb, depth_mm, up_optical or roll_pitch_deg, t,
        coverage, plus the colour 3A keys), plus MANIFEST.sha256. Nothing is written with --dry-run. The folder is
        written as <output>.partial and renamed when complete; a refusal removes that .partial folder.

Colour: the ISP frame (YUV420p / NV12) is converted with cv2.cvtColor(..., COLOR_YUV2BGR_IYUV / _NV12), the call
DepthAI 2.29's ImgFrame.getCvFrame makes (checked offline against depthai 2.29.0.0), then reordered to RGB.

Rectification: one output pinhole camera K_out (no rotation). For every output pixel the source position in the raw
colour image comes from the recorded CAM_A calibration at the colour output size (distortion model Perspective,
14 coefficients, via cv2.initUndistortRectifyMap; or Fisheye, 4 coefficients, via cv2.fisheye). RGB is resampled
bilinearly; depth is resampled with nearest neighbour only (cv2.INTER_NEAREST, zero border), so a depth value is
always one recorded value and 0 (invalid) never mixes with a neighbour. coverage is False where the colour source or
the depth source lies outside the raw image (ones image, same maps, nearest, zero border; tools/oakd_camera.Rectifier
definition).

Depth coordinate model (--depth-coordinate-model, REQUIRED, no default): where the device puts the CAM_A-aligned
depth is not established for this unit. Luxonis' RGB-D page (https://docs.luxonis.com/software/perception/rgb-d)
says "By default, the depth image is undistorted during the rectification-undistortion process, but the color image
is not. This can lead to misalignment between the two images", i.e. the aligned depth is in the UNDISTORTED CAM_A
pinhole image (K only) -> undistorted_pinhole: depth is resampled with the pure pinhole map K_raw -> K_out.
distorted_rgb (the assumption of tools/oakd_camera.validate_config) resamples depth with the colour map, so both are
"rectified with the same map". The choice is written into recording.json.device; it is a hypothesis until a capture
with a known target checks it.

Presets (--preset, default auto):
  adapter       IMX378 at 1014 x 760: K_out = raw K and the RGB maps of tools/oakd_camera.Rectifier (the existing
                adapter's output), Rectifier.require_training_fov must pass (else FOV_SHORTFALL).
  trained_crop  central crop that covers exactly the trained 66 x 54 deg camera of tools/camera_policy_contract.py:
                fx = W / (2 tan 33 deg), fy = H / (2 tan 27 deg), cx = (W - 1) / 2, cy = (H - 1) / 2 at an output size
                that is an integer multiple k of the trained 160 x 90 grid (default 640 x 360, k = 4), so the trained
                pixel (i, j) is exactly the k x k block of output pixels around (k i + (k - 1) / 2, k j + (k - 1) / 2)
                and the image edges subtend exactly 66 and 54 deg. Pixels are not square (fx != fy), as in the trained
                camera. Every output pixel must be covered by the raw image (else FOV_SHORTFALL).
  auto          OV9782 -> trained_crop, IMX378 -> adapter.

Gravity: up_optical = R_imu_to_cam_a @ (mean accelerometer / its norm), R from the recorded
CalibrationHandler.getImuToCameraExtrinsics(CAM_A) (DepthAI 2.29 docstring: "relative rotation and translation from
the IMU to Camera", 4 x 4, i.e. p_cam = R p_imu + T; for vectors such as acceleration only R applies, per
https://docs.luxonis.com/hardware/platform/sensors/imu/). A still accelerometer reads the reaction to gravity, i.e.
+g along UP (the Android / BNO08x sensor convention, https://developer.android.com/reference/android/hardware/
SensorEvent: "when the device lies flat on a table ... the acceleration value along z is +9.81"). Camera frame:
OpenCV optical x right, y down, z forward (Luxonis RDF). Raw IMU axes follow the vendor datasheet and forum reports
disagree on v2 handedness, so each frame's IMU roll / pitch is compared with the nominal (roll 0, --pitch-deg of
the capture) and a deviation above --max-imu-nominal-deviation-deg is refused (IMU_NOMINAL_PITCH_MISMATCH). Without
a stored extrinsic, roll_pitch_deg = (0, nominal pitch) is written and recording.json.device says so.

Gravity source (--gravity-source, repair r1 P2-2): auto (default) = imu when the capture holds an extrinsic, else
nominal; imu forces the accelerometer (refused NO_IMU_EXTRINSICS without one); nominal writes roll_pitch_deg =
(0, the recorded --pitch-deg) and ignores the IMU, e.g. for a dead IMU or a placeholder extrinsic. The choice is in
recording.json.device.conversion.gravity. The IMU-vs-nominal check is on unless --no-imu-nominal-check.

Colour 3A (repair r1 P1-1): the recorder's per-frame exposure_time_us, sensitivity_iso, colour_temperature_k,
lens_position(_raw) and auto_exposure / auto_white_balance_configured are copied into each output frame (extra npz
keys, which the replay ignores) and summarised in recording.json.device.colour_3a.

Disk (repair r1 P1-2): the output size is estimated from the first converted frame and printed (also with
--dry-run); a real run refuses INSUFFICIENT_DISK unless the output's file system has 1.2 x that + 10 MB free.

Refusal codes: CAPTURE_UNREADABLE, NO_IMU_EXTRINSICS, NO_NOMINAL_PITCH, INSUFFICIENT_DISK, CAPTURE_FORMAT,
INCOMPLETE_CAPTURE, FRAME_HASH_MISMATCH, FRAME_LAYOUT,
NON_INCREASING_TIMESTAMPS, CALIBRATION_MISSING, UNSUPPORTED_DISTORTION_MODEL, PRESET, FOV_SHORTFALL,
BAD_IMU_EXTRINSICS, IMU_NO_SAMPLES, IMU_NOT_STILL, IMU_NOMINAL_PITCH_MISMATCH, NO_GRAVITY_SOURCE, OUTPUT_EXISTS.
Hardware, the real calibration and the depth coordinate model are NOT_TESTED here.
"""
import argparse
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CAPTURE_FORMAT = 'motar-oakd-raw-capture-v1'
OUTPUT_FORMAT = 'motar-oakd-outdoor-recording-v1'
CAPTURE_SOURCES = {'oakd_field_recorder': 'oakd_recording', 'synthetic': 'synthetic'}
TRAINED = {'width': 160, 'height': 90, 'hfov_deg': 66.0, 'vfov_deg': 54.0}
DEPTH_MODELS = ('undistorted_pinhole', 'distorted_rgb')
PRESETS = ('auto', 'adapter', 'trained_crop')
ADAPTER_SIZE = (1014, 760)
DEFAULT_CROP = (640, 360)
GRAVITY = 9.80665
ACCEL_NORM_TOLERANCE = 0.8                # m/s^2, uncalibrated ACCELEROMETER_RAW bias allowance
ACCEL_STD_MAX = 0.5                       # m/s^2 per axis: above this the camera was not still
DEFAULT_MAX_DEVIATION_DEG = 5.0


class ConvertError(ValueError):
    def __init__(self, code, message):
        ValueError.__init__(self, '%s: %s' % (code, message))
        self.code = code


def sha256_file(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def _tools():
    if str(ROOT / 'tools') not in sys.path:
        sys.path.insert(0, str(ROOT / 'tools'))
    import oakd_camera
    import oakd_outdoor_replay
    return oakd_camera, oakd_outdoor_replay


def _cv2():
    import cv2
    return cv2


# ---------------------------------------------------------------- geometry

def trained_crop_intrinsics(width, height):
    """Pinhole K covering exactly the trained HFOV x VFOV at an integer multiple of the trained grid."""
    k = width // TRAINED['width']
    if (isinstance(width, bool) or isinstance(height, bool) or k < 1 or k > 8 or width != k * TRAINED['width']
            or height != k * TRAINED['height']):
        raise ConvertError('PRESET', 'crop size must be k x (%d x %d), k = 1..8; got %sx%s'
                           % (TRAINED['width'], TRAINED['height'], width, height))
    fx = width / (2.0 * math.tan(math.radians(TRAINED['hfov_deg'] / 2.0)))
    fy = height / (2.0 * math.tan(math.radians(TRAINED['vfov_deg'] / 2.0)))
    return np.array([[fx, 0.0, (width - 1) / 2.0], [0.0, fy, (height - 1) / 2.0], [0.0, 0.0, 1.0]])


def _k_dict(k, width, height):
    return {'width': int(width), 'height': int(height), 'fx': float(k[0, 0]), 'fy': float(k[1, 1]),
            'cx': float(k[0, 2]), 'cy': float(k[1, 2])}


def colour_maps(k_raw, distortion, model, k_out, size):
    """Output pixel -> raw distorted colour pixel (float32 maps)."""
    cv2 = _cv2()
    k_raw = np.asarray(k_raw, np.float64)
    d = np.asarray(distortion, np.float64).ravel()
    if model == 'Perspective':
        if d.size not in (4, 5, 8, 12, 14):
            raise ConvertError('UNSUPPORTED_DISTORTION_MODEL', 'Perspective needs 4/5/8/12/14 coefficients')
        return cv2.initUndistortRectifyMap(k_raw, d, np.eye(3), np.asarray(k_out, np.float64), tuple(size),
                                           cv2.CV_32FC1)
    if model == 'Fisheye':
        if d.size < 4:
            raise ConvertError('UNSUPPORTED_DISTORTION_MODEL', 'Fisheye needs 4 coefficients')
        return cv2.fisheye.initUndistortRectifyMap(k_raw, d[:4].reshape(4, 1), np.eye(3),
                                                   np.asarray(k_out, np.float64), tuple(size), cv2.CV_32FC1)
    raise ConvertError('UNSUPPORTED_DISTORTION_MODEL', 'distortion model %r is not Perspective or Fisheye' % model)


def pinhole_maps(k_raw, k_out, size):
    """Output pixel -> pixel of the UNDISTORTED raw pinhole image with K_raw (no distortion)."""
    width, height = size
    k_raw, k_out = np.asarray(k_raw, np.float64), np.asarray(k_out, np.float64)
    u, v = np.meshgrid(np.arange(width, dtype=np.float64), np.arange(height, dtype=np.float64))
    x = (u - k_out[0, 2]) / k_out[0, 0]
    y = (v - k_out[1, 2]) / k_out[1, 1]
    return ((k_raw[0, 0] * x + k_raw[0, 2]).astype(np.float32), (k_raw[1, 1] * y + k_raw[1, 2]).astype(np.float32))


def source_coverage(mapx, mapy, raw_size):
    cv2 = _cv2()
    ones = np.ones((raw_size[1], raw_size[0]), np.uint8)
    return cv2.remap(ones, mapx, mapy, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0) == 1


class Geometry:
    """Maps and output intrinsics for one capture."""

    def __init__(self, preset, k_raw, distortion, model, raw_size, depth_model, crop_size=DEFAULT_CROP,
                 allow_fov_shortfall=False):
        if depth_model not in DEPTH_MODELS:
            raise ConvertError('PRESET', 'depth coordinate model must be one of %s' % (DEPTH_MODELS,))
        self.preset, self.depth_model, self.raw_size = preset, depth_model, tuple(raw_size)
        self.k_raw = np.asarray(k_raw, np.float64)
        self.distortion = np.asarray(distortion, np.float64).ravel()
        self.model = model
        self.fov_check = None
        if preset == 'adapter':
            oakd_camera, _ = _tools()
            if tuple(raw_size) != ADAPTER_SIZE or model != 'Perspective':
                raise ConvertError('PRESET', 'adapter preset needs the IMX378 1014x760 Perspective calibration')
            try:
                rectifier = oakd_camera.Rectifier(self.k_raw, self.distortion, width=raw_size[0], height=raw_size[1])
            except oakd_camera.OakDInputError as exc:
                raise ConvertError('CALIBRATION_MISSING', 'oakd_camera.Rectifier refused: %s' % exc) from None
            try:
                rectifier.require_training_fov()
                self.fov_check = 'PASS (oakd_camera.Rectifier.require_training_fov)'
            except oakd_camera.OakDInputError as exc:
                if not allow_fov_shortfall:
                    raise ConvertError('FOV_SHORTFALL', '%s (--allow-fov-shortfall records it anyway)' % exc) from None
                self.fov_check = 'FAIL_ALLOWED: %s' % exc
            self.k_out, self.size = self.k_raw.copy(), tuple(raw_size)
            self.rgb_map = (rectifier.mapx, rectifier.mapy)
        elif preset == 'trained_crop':
            self.size = tuple(crop_size)
            self.k_out = trained_crop_intrinsics(*self.size)
            self.rgb_map = colour_maps(self.k_raw, self.distortion, model, self.k_out, self.size)
        else:
            raise ConvertError('PRESET', 'unknown preset %r' % (preset,))
        self.depth_map = (self.rgb_map if depth_model == 'distorted_rgb'
                          else pinhole_maps(self.k_raw, self.k_out, self.size))
        self.coverage = (source_coverage(*self.rgb_map, self.raw_size)
                         & source_coverage(*self.depth_map, self.raw_size))
        self.uncovered = int((~self.coverage).sum())
        if preset == 'trained_crop':
            if self.uncovered and not allow_fov_shortfall:
                raise ConvertError('FOV_SHORTFALL', '%d output pixels of the trained 66x54-deg crop lie outside the raw '
                                                    'image (--allow-fov-shortfall records it anyway)' % self.uncovered)
            self.fov_check = ('PASS (every output pixel covered)' if not self.uncovered
                              else 'FAIL_ALLOWED: %d uncovered pixels' % self.uncovered)

    def apply(self, rgb_raw, depth_raw):
        cv2 = _cv2()
        if rgb_raw.shape != (self.raw_size[1], self.raw_size[0], 3) or depth_raw.shape != rgb_raw.shape[:2]:
            raise ConvertError('FRAME_LAYOUT', 'raw frame size differs from the calibration output size')
        rgb = cv2.remap(rgb_raw, self.rgb_map[0], self.rgb_map[1], cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        depth = cv2.remap(depth_raw, self.depth_map[0], self.depth_map[1], cv2.INTER_NEAREST,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return rgb, depth, self.coverage.copy()

    def describe(self):
        return {'preset': self.preset, 'raw_size': list(self.raw_size), 'raw_intrinsics': self.k_raw.tolist(),
                'raw_distortion': self.distortion.tolist(), 'distortion_model': self.model,
                'output_intrinsics': _k_dict(self.k_out, *self.size), 'rectification_rotation': 'identity',
                'depth_coordinate_model': self.depth_model,
                'depth_coordinate_model_status': 'HYPOTHESIS (not checked on this unit)',
                'rgb_interpolation': 'linear', 'depth_interpolation': 'nearest (cv2.INTER_NEAREST), zero border',
                'coverage': 'colour and depth sources both inside the raw image (ones image, same maps, nearest)',
                'uncovered_output_pixels': self.uncovered, 'trained_fov_check': self.fov_check,
                'trained_camera': dict(TRAINED)}


# ---------------------------------------------------------------- gravity

def imu_rotation(calibration):
    """R (IMU -> CAM_A optical) from the recorded extrinsics, None if absent; refuses a non-rotation."""
    block = (calibration or {}).get('imu_to_cam_a') or {}
    matrix = block.get('matrix_4x4')
    if matrix is None:
        return None
    m = np.asarray(matrix, np.float64)
    if m.shape != (4, 4) or not np.isfinite(m).all():
        raise ConvertError('BAD_IMU_EXTRINSICS', 'imu_to_cam_a must be a finite 4x4 matrix')
    r = m[:3, :3]
    if np.abs(r).sum() == 0:
        return None
    if np.abs(r.T @ r - np.eye(3)).max() > 1e-3 or abs(np.linalg.det(r) - 1.0) > 1e-3:
        raise ConvertError('BAD_IMU_EXTRINSICS', 'imu_to_cam_a rotation is not a proper rotation')
    return r


def up_optical(accel_mean, accel_std, accel_count, rotation):
    count = int(accel_count)
    a = np.asarray(accel_mean, np.float64)
    if count < 1 or a.shape != (3,) or not np.isfinite(a).all():
        raise ConvertError('IMU_NO_SAMPLES', 'no accelerometer samples for this frame; --gravity-source nominal '
                                             'uses the recorded --pitch-deg instead')
    norm = float(np.linalg.norm(a))
    std = np.asarray(accel_std, np.float64)
    if abs(norm - GRAVITY) > ACCEL_NORM_TOLERANCE or (np.isfinite(std).all() and std.max() > ACCEL_STD_MAX):
        raise ConvertError('IMU_NOT_STILL', 'mean |a| %.2f m/s^2, max std %.2f m/s^2: the camera was not still'
                           % (norm, float(np.nanmax(std)) if std.size else float('nan')))
    up = rotation @ (a / norm)
    return up / np.linalg.norm(up)


# ---------------------------------------------------------------- capture

def _refuse_path(relative):
    p = Path(relative)
    return not isinstance(relative, str) or p.is_absolute() or '..' in p.parts


def load_capture(folder):
    folder = Path(folder)
    try:
        meta = json.loads((folder / 'capture.json').read_text())
    except (OSError, ValueError) as exc:
        raise ConvertError('CAPTURE_UNREADABLE', 'capture.json: %s' % exc) from None
    if not isinstance(meta, dict) or meta.get('format') != CAPTURE_FORMAT:
        raise ConvertError('CAPTURE_FORMAT', 'capture.json format must be %s' % CAPTURE_FORMAT)
    if meta.get('source') not in CAPTURE_SOURCES:
        raise ConvertError('CAPTURE_FORMAT', 'capture source must be one of %s' % sorted(CAPTURE_SOURCES))
    if meta.get('status') != 'COMPLETE':
        raise ConvertError('INCOMPLETE_CAPTURE', 'capture status %r (stop error %r)'
                           % (meta.get('status'), meta.get('stop_error')))
    frames = meta.get('frames')
    if not isinstance(frames, list) or not frames:
        raise ConvertError('CAPTURE_FORMAT', 'capture has no frames')
    last = None
    for entry in frames:
        if not isinstance(entry, dict) or _refuse_path(entry.get('file')):
            raise ConvertError('CAPTURE_FORMAT', 'frame entries need a relative file inside the capture')
        try:
            digest = sha256_file(folder / entry['file'])
        except OSError as exc:
            raise ConvertError('CAPTURE_UNREADABLE', '%s: %s' % (entry['file'], exc)) from None
        if digest != entry.get('sha256'):
            raise ConvertError('FRAME_HASH_MISMATCH', '%s differs from its recorded SHA-256' % entry['file'])
        t = entry.get('t_device_s')
        if not isinstance(t, (int, float)) or isinstance(t, bool) or not math.isfinite(t) or (last is not None
                                                                                             and not t > last):
            raise ConvertError('NON_INCREASING_TIMESTAMPS', '%s: device time %r after %r' % (entry['file'], t, last))
        last = t
    for key in ('camera_height_m', 'camera_height_uncertainty_m'):
        value = meta.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ConvertError('CAPTURE_FORMAT', '%s must be a positive number' % key)
    if not isinstance(meta.get('scene'), str) or not meta['scene'].strip():
        raise ConvertError('CAPTURE_FORMAT', 'scene text is required')
    return meta


def capture_calibration(meta):
    """(K_raw at the colour output size, distortion, model, (width, height), colour sensor)."""
    try:
        colour = meta['pipeline']['colour']
        size = tuple(int(v) for v in colour['size'])
        cam_a = meta['calibration']['sockets']['CAM_A']
        k = np.asarray(cam_a['intrinsics_at_output'], np.float64)
        d = np.asarray(cam_a['distortion_coefficients'], np.float64).ravel()
        model = cam_a['distortion_model']
        sensor = colour['sensor']
    except (KeyError, TypeError, ValueError) as exc:
        raise ConvertError('CALIBRATION_MISSING', 'CAM_A calibration at the colour output size: %r' % (exc,)) from None
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.isfinite(d).all()
            or list(cam_a.get('output_size', size)) != list(size)):
        raise ConvertError('CALIBRATION_MISSING', 'CAM_A intrinsics are not a valid 3x3 K at the colour output size')
    return k, d, model, size, sensor


def decode_frame(folder, entry, size):
    cv2 = _cv2()
    try:
        with np.load(Path(folder) / entry['file'], allow_pickle=False) as data:
            arrays = {k: data[k] for k in data.files}
    except (OSError, ValueError) as exc:
        raise ConvertError('CAPTURE_UNREADABLE', '%s: %s' % (entry['file'], exc)) from None
    width, height = size
    isp_type = str(arrays.get('isp_type', ''))
    isp = arrays.get('isp')
    codes = {'YUV420p': cv2.COLOR_YUV2BGR_IYUV, 'NV12': cv2.COLOR_YUV2BGR_NV12}
    if (isp_type not in codes or isp is None or isp.dtype != np.uint8 or isp.shape != (height * 3 // 2, width)
            or int(arrays.get('width', -1)) != width or int(arrays.get('height', -1)) != height):
        raise ConvertError('FRAME_LAYOUT', '%s: isp must be uint8 [%d, %d] YUV420p/NV12' % (entry['file'],
                                                                                           height * 3 // 2, width))
    depth = arrays.get('depth_mm')
    if depth is None or depth.dtype != np.uint16 or depth.shape != (height, width):
        raise ConvertError('FRAME_LAYOUT', '%s: depth_mm must be uint16 [%d, %d]' % (entry['file'], height, width))
    t = float(arrays['t_device_s'])
    if t != float(entry['t_device_s']):
        raise ConvertError('FRAME_LAYOUT', '%s: t_device_s differs from capture.json' % entry['file'])
    rgb = cv2.cvtColor(isp, codes[isp_type])[:, :, ::-1].copy()   # getCvFrame (BGR) -> RGB
    return rgb, depth, arrays, t


def choose_preset(preset, sensor):
    if preset != 'auto':
        return preset
    if sensor == 'OV9782':
        return 'trained_crop'
    if sensor == 'IMX378':
        return 'adapter'
    raise ConvertError('PRESET', 'no automatic preset for colour sensor %r' % (sensor,))


COLOUR_3A_KEYS = ('exposure_time_us', 'sensitivity_iso', 'colour_temperature_k', 'lens_position', 'lens_position_raw',
                  'auto_exposure_configured', 'auto_white_balance_configured')
GRAVITY_SOURCES = ('auto', 'imu', 'nominal')
DISK_MARGIN = 1.2
DISK_RESERVE = 10 << 20


def _colour_3a(arrays):
    """The recorder's per-frame colour 3A arrays (repair r1), copied through; absent keys -> None (NOT_RECORDED)."""
    out = {}
    for key in COLOUR_3A_KEYS:
        if key in arrays:
            value = arrays[key]
            out[key] = bool(value) if value.dtype == np.bool_ else (float(value) if value.dtype.kind == 'f'
                                                                     else int(value))
        else:
            out[key] = None
    return out


def _summary_3a(rows):
    summary = {}
    for key in COLOUR_3A_KEYS[:5]:
        values = [r['colour_3a'][key] for r in rows if r['colour_3a'][key] not in (None, -1, -1.0)]
        summary[key] = {'min': min(values), 'max': max(values), 'frames': len(values)} if values else 'NOT_RECORDED'
    return summary


def _encode(frame):
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **frame)
    return buffer.getvalue()


def check_output_space(output, needed):
    """Refuse (INSUFFICIENT_DISK) unless the output's file system has DISK_MARGIN x needed + DISK_RESERVE free."""
    parent = Path(output).resolve().parent
    free = shutil.disk_usage(str(parent)).free
    required = int(DISK_MARGIN * needed) + DISK_RESERVE
    if free < required:
        raise ConvertError('INSUFFICIENT_DISK', '%d MB free under %s, the conversion needs about %d MB (%d MB '
                                                'estimated output x %.1f + %d MB)'
                           % (free >> 20, parent, required >> 20, needed >> 20, DISK_MARGIN, DISK_RESERVE >> 20))
    return free, required


def convert(capture, output=None, *, depth_model, preset='auto', crop_size=DEFAULT_CROP, allow_fov_shortfall=False,
            max_deviation_deg=DEFAULT_MAX_DEVIATION_DEG, check_nominal=True, gravity_source='auto', dry_run=False):
    """Convert one capture. Returns (recording metadata, per-frame report). Writes nothing when dry_run.

    The output is written to <output>.partial and renamed to <output> only after every frame converted; a refusal
    removes the .partial folder this call created (review r1 P3-4), so a rerun is not blocked. The free space is
    checked from the first converted frame's compressed size before anything is written (review r1 P1-2).
    """
    _, replay = _tools()
    capture = Path(capture)
    if gravity_source not in GRAVITY_SOURCES:
        raise ConvertError('PRESET', 'gravity source must be one of %s' % (GRAVITY_SOURCES,))
    meta = load_capture(capture)
    k_raw, distortion, model, size, sensor = capture_calibration(meta)
    chosen = choose_preset(preset, sensor)
    geometry = Geometry(chosen, k_raw, distortion, model, size, depth_model, crop_size, allow_fov_shortfall)
    extrinsic = imu_rotation(meta.get('calibration'))
    nominal = meta.get('pitch_deg_nominal')
    requested_gravity = gravity_source
    if gravity_source == 'auto':
        gravity_source = 'imu' if extrinsic is not None else 'nominal'
    if gravity_source == 'imu':
        if extrinsic is None:
            raise ConvertError('NO_IMU_EXTRINSICS', 'the capture has no IMU-to-CAM_A extrinsic; use --gravity-source '
                                                    'nominal (roll 0, the recorded --pitch-deg)')
        if check_nominal and nominal is None:
            raise ConvertError('NO_NOMINAL_PITCH', 'the capture has no nominal pitch to check the IMU against '
                                                   '(--no-imu-nominal-check skips the check knowingly)')
    elif nominal is None:
        raise ConvertError('NO_GRAVITY_SOURCE', 'gravity source nominal needs the capture\'s pitch_deg_nominal')
    rotation = extrinsic if gravity_source == 'imu' else None
    out = None if dry_run else Path(output)
    partial = None
    if out is not None and (out.exists() or Path(str(out) + '.partial').exists()):
        raise ConvertError('OUTPUT_EXISTS', '%s (or %s.partial) exists; give a NEW folder' % (out, out))
    rows, files, hashes = [], [], []
    total_bytes, estimate = 0, None
    try:
        for index, entry in enumerate(meta['frames']):
            rgb_raw, depth_raw, arrays, t = decode_frame(capture, entry, size)
            rgb, depth, coverage = geometry.apply(rgb_raw, depth_raw)
            row = {'index': index, 'file': entry['file'], 't': t,
                   'valid_depth_share_raw': float((depth_raw > 0).mean()),
                   'valid_depth_share_out': float((depth > 0).mean()), 'coverage_share': float(coverage.mean()),
                   'colour_3a': _colour_3a(arrays)}
            frame = {'rgb': rgb, 'depth_mm': depth, 't': np.array(t, np.float64), 'coverage': coverage}
            for key, value in row['colour_3a'].items():          # extra keys; the replay reads only its own
                if value is not None:
                    frame[key] = np.array(value)
            if rotation is not None:
                up = up_optical(arrays['accel_mean'], arrays['accel_std'], arrays['accel_count'], rotation)
                roll, pitch = replay.roll_pitch_from_up(up)
                row.update(up_optical=up.tolist(), imu_roll_deg=roll, imu_pitch_deg=pitch,
                           accel_count=int(arrays['accel_count']))
                if check_nominal:
                    if abs(pitch - float(nominal)) > max_deviation_deg or abs(roll) > max_deviation_deg:
                        raise ConvertError(
                            'IMU_NOMINAL_PITCH_MISMATCH',
                            '%s: IMU roll %.1f / pitch %.1f deg vs nominal roll 0 / pitch %.1f deg (limit %.1f); '
                            'check the IMU axes / extrinsics or the pitch written in --pitch-deg '
                            '(--gravity-source nominal uses the written pitch instead)'
                            % (entry['file'], roll, pitch, float(nominal), max_deviation_deg))
                frame['up_optical'] = up.astype(np.float64)
            else:
                frame['roll_pitch_deg'] = np.array([0.0, float(nominal)], np.float64)
                row.update(roll_pitch_deg=[0.0, float(nominal)])
            data = _encode(frame)
            total_bytes += len(data)
            if estimate is None:
                estimate = len(data) * len(meta['frames']) + (1 << 20)        # + recording.json and manifest
                if out is not None:
                    check_output_space(out, estimate)
                    partial = Path(str(out) + '.partial')
                    (partial / 'frames').mkdir(parents=True)
            rows.append(row)
            relative = 'frames/%06d.npz' % index
            files.append(relative)
            if partial is not None:
                with (partial / relative).open('xb') as stream:
                    stream.write(data)
                hashes.append((hashlib.sha256(data).hexdigest(), relative))
        gravity = ({'source': 'imu', 'requested': requested_gravity,
                    'method': 'R_imu_to_cam_a @ mean ACCELEROMETER_RAW / |.|',
                    'imu_to_cam_a_rotation': rotation.tolist(),
                    'nominal_check': ('roll 0 / pitch %s within %.1f deg' % (nominal, max_deviation_deg)
                                      if check_nominal else 'NOT_CHECKED (--no-imu-nominal-check)')}
                   if rotation is not None else
                   {'source': 'nominal_pitch', 'requested': requested_gravity,
                    'method': 'roll_pitch_deg = (0, capture pitch_deg_nominal); the IMU is not used'
                              + ('' if extrinsic is not None else ' (the capture has no IMU-to-CAM_A extrinsic)'),
                    'pitch_deg_nominal': nominal})
        device = {'capture': {'path': str(capture.resolve()),
                              'capture_json_sha256': sha256_file(capture / 'capture.json'),
                              'source': meta['source'], 'started_utc': meta.get('started_utc'),
                              'ended_utc': meta.get('ended_utc'), 'recorder': meta.get('recorder')},
                  'identity': meta.get('device'), 'sensors': meta.get('sensors'), 'colour_sensor': sensor,
                  'pipeline': meta.get('pipeline'), 'calibration': meta.get('calibration'),
                  'calibration_eeprom': meta.get('calibration_eeprom'), 'pitch_deg_nominal': nominal,
                  'colour_3a': {'summary': _summary_3a(rows),
                                'per_frame': [dict(r['colour_3a'], file=f) for r, f in zip(rows, files)],
                                'note': 'the colour frames\' own exposure / ISO / white balance / lens state; '
                                        'None = NOT_RECORDED by the capture'},
                  'conversion': dict(geometry.describe(),
                                     colour_decoding='cv2.cvtColor YUV2BGR (as getCvFrame) -> RGB',
                                     time='t = colour device timestamp t_device_s [s]', gravity=gravity,
                                     output_bytes=total_bytes, output_bytes_estimate_from_first_frame=estimate,
                                     converter={'path': str(Path(__file__).resolve()),
                                                'sha256': sha256_file(Path(__file__).resolve())}),
                  'hardware_qualification': 'NOT_TESTED'}
        recording = {'format': OUTPUT_FORMAT, 'source': CAPTURE_SOURCES[meta['source']], 'device': device,
                     'intrinsics': _k_dict(geometry.k_out, *geometry.size),
                     'camera_height_m': float(meta['camera_height_m']),
                     'camera_height_uncertainty_m': float(meta['camera_height_uncertainty_m']),
                     'scene': meta['scene'], 'frames': files}
        if partial is not None:
            with (partial / 'recording.json').open('x') as stream:
                json.dump(recording, stream, indent=1, sort_keys=True, allow_nan=False)
                stream.write('\n')
            hashes.append((sha256_file(partial / 'recording.json'), 'recording.json'))
            with (partial / 'MANIFEST.sha256').open('x') as stream:
                stream.write(''.join('%s  %s\n' % pair for pair in hashes))
            replay.load_recording(partial)           # the replay's own structural check of what was written
            os.rename(str(partial), str(out))
            partial = None
    finally:
        if partial is not None and partial.is_dir():
            shutil.rmtree(str(partial))              # this call's own incomplete output only
    return recording, rows


def verify_manifest(folder):
    """Re-hash every file listed in MANIFEST.sha256; returns the number checked, raises on a mismatch."""
    folder = Path(folder)
    count = 0
    for line in (folder / 'MANIFEST.sha256').read_text().splitlines():
        digest, relative = line.split('  ', 1)
        if _refuse_path(relative) or sha256_file(folder / relative) != digest:
            raise ConvertError('FRAME_HASH_MISMATCH', '%s differs from MANIFEST.sha256' % relative)
        count += 1
    return count


def _size(text):
    try:
        width, height = (int(v) for v in text.lower().split('x'))
    except ValueError:
        raise argparse.ArgumentTypeError('size must be WIDTHxHEIGHT') from None
    return width, height


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--capture', required=True, help='capture folder written by tools/oakd_field_recorder.py')
    ap.add_argument('--output', help='NEW folder for the replay recording (not with --dry-run)')
    ap.add_argument('--depth-coordinate-model', required=True, choices=DEPTH_MODELS,
                    help='where the device put the CAM_A-aligned depth (see above; no default)')
    ap.add_argument('--preset', choices=PRESETS, default='auto')
    ap.add_argument('--crop-size', type=_size, default=DEFAULT_CROP, help='trained_crop output, k x 160x90')
    ap.add_argument('--allow-fov-shortfall', action='store_true',
                    help='write a recording whose source does not cover the trained FOV (recorded in device)')
    ap.add_argument('--max-imu-nominal-deviation-deg', type=float, default=DEFAULT_MAX_DEVIATION_DEG)
    ap.add_argument('--no-imu-nominal-check', action='store_true', help='skip the IMU vs nominal roll/pitch check')
    ap.add_argument('--gravity-source', choices=GRAVITY_SOURCES, default='auto',
                    help='imu (accelerometer + extrinsic), nominal (roll 0, the recorded --pitch-deg) or auto (imu '
                         'when the capture has an extrinsic, else nominal); recorded in device')
    ap.add_argument('--dry-run', action='store_true', help='validate and convert in memory; write nothing')
    args = ap.parse_args(argv)
    if args.dry_run == (args.output is not None):
        ap.error('give exactly one of --output NEW_DIR or --dry-run')
    try:
        recording, rows = convert(args.capture, args.output, depth_model=args.depth_coordinate_model,
                                  preset=args.preset, crop_size=args.crop_size,
                                  allow_fov_shortfall=args.allow_fov_shortfall,
                                  max_deviation_deg=args.max_imu_nominal_deviation_deg,
                                  check_nominal=not args.no_imu_nominal_check,
                                  gravity_source=args.gravity_source, dry_run=args.dry_run)
    except ConvertError as exc:
        print('oakd recording convert refused: %s' % exc, file=sys.stderr)
        return 2
    conversion = recording['device']['conversion']
    print(json.dumps({'dry_run': args.dry_run, 'frames': len(rows), 'source': recording['source'],
                      'preset': conversion['preset'], 'intrinsics': recording['intrinsics'],
                      'depth_coordinate_model': conversion['depth_coordinate_model'],
                      'gravity': conversion['gravity']['source'], 'trained_fov_check': conversion['trained_fov_check'],
                      'coverage_share_min': min(r['coverage_share'] for r in rows),
                      'output_bytes': conversion['output_bytes'],
                      'output_mb_estimate': round(conversion['output_bytes_estimate_from_first_frame'] / 1e6, 2),
                      'colour_3a': recording['device']['colour_3a']['summary']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
