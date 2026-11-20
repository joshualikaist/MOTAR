#!/usr/bin/env python3
"""OAK-D field recorder for the drone's Jetson (single file; stdlib + numpy + depthai 2.29.0.0 only).

Purpose: record still RGB-D frames and accelerometer samples of the user's OAK-D outdoors (on a tripod or carried by
hand, never flown) so that the PROVISIONAL sky / ground thresholds of the camera runtime can be checked on the real
camera. tools/oakd_recording_convert.py turns a capture into the replay format of tools/oakd_outdoor_replay.py on the
PC. This script never imports ROS, PX4, MAVLink, cv2 or repository code and never commands a vehicle. It runs under
Python 3.8 or newer (JetPack 5 ships 3.8).

Modes
  --probe   open the device and print one JSON object (into the file given with --output, else to stdout; the SDK
            logs to stdout, so the file form keeps the JSON clean). It holds the DepthAI version, product / EEPROM
            names, MX id, USB speed, every connected camera (socket, sensor, size), the IMU, a calibration summary
            (intrinsics, distortion model and coefficients per socket, IMU-to-CAM_A extrinsics if stored, stereo
            baseline and MinZ hints), the recording profile that --record would use (or the reason it would refuse)
            and, if an IMU is present, the mean accelerometer over --probe-imu-s seconds.
  --record  record --frames still frames into a NEW directory (layout below).

The colour sensor decides the pipeline (the camera model is not assumed; D-2026-11-18-1: the TA says "probably
OV9782", i.e. the OAK-D Pro W, UNCONFIRMED):
  OV9782 (OAK-D Pro W colour, global shutter)  -> ColorCamera THE_800_P, ISP output 1280 x 800, fixed focus.
  OV9282 on CAM_A                              -> the same 1280 x 800 pipeline, with a WARNING: the firmware reports
     some OV9782 modules as OV9282 and such a module delivers grey images (Luxonis forum thread 6070); the name is
     recorded as reported.
  IMX378 (OAK-D Pro colour)                    -> ColorCamera THE_12_MP with ISP scale 4/13 -> 1248 x 936 (the full
     4:3 sensor), focus set to the calibration lens position. Preflight r4: the device refuses an aligned depth
     whose width is not a multiple of 16 ("Disparity/depth width must be multiple of 16, but RGB camera width is
     1352", Luxonis forum thread 5933), so the 1/4 scale of tools/oakd_camera.build_pipeline (1014 x 760) cannot be
     used here. THE_1080_P is rejected because it is a 16:9 crop of the 4:3 sensor and drops the vertical field of
     view below the trained 54 deg.
  any other colour sensor, or no CAM_A           -> refused with a named reason.
Stereo: CAM_B / CAM_C mono THE_800_P (OV9282 or OV9782 sensors), StereoDepth HIGH_DENSITY preset, left-right check,
depth aligned to CAM_A at the colour output size, calibrated (not design-spec) translations, extended disparity or
subpixel. IMU: ACCELEROMETER_RAW at 100 Hz, plus ROTATION_VECTOR when the IMU is a BNO08x.
Colour and depth are paired on the device by the Sync node with a 60 ms threshold: above half the 10 FPS frame
period, so a partner always exists whatever the constant offset between the colour and the stereo timestamps
(DepthAI's own depth_video_synced example uses 50 ms). The SDK default (10 ms, and the node sends nothing until a
pair is that close) can deliver no frame at all; the measured depth-minus-colour time is saved per frame.
Warm-up: --warmup-s seconds of frames are discarded, counted from the FIRST received frame (not from the pipeline
start, which can take seconds), so the auto exposure of the colour and the mono cameras has settled; the exposure
at the start and the end of the warm-up is recorded.

Colour is stored as the ISP frame exactly as delivered (YUV420p / NV12, H*3/2 x W bytes), not as RGB: converting on
the Jetson would need cv2, and the PC converter applies the conversion DepthAI's own getCvFrame uses
(cv2.cvtColor ..._IYUV / _NV12; checked offline against depthai 2.29.0.0). It is also half the bytes of RGB.

Output directory (format motar-oakd-raw-capture-v1):
  capture.json        device, EEPROM / calibration dump (readCalibration().eepromToJson()), pipeline description,
                      scene, heights, nominal pitch, start / end times, status, per-frame sizes and SHA-256
  MANIFEST.sha256     "sha256  path" lines for every frame and capture.json (sha256sum -c works)
  imu.npz             accel float64 [N, 4]: EVERY accelerometer sample received from the warm-up to the last frame
                      (device time s, x, y, z), so gravity can be averaged over the whole still capture even when a
                      frame's own window is empty (review r4 F2)
  frames/NNNNNN.npz   np.savez_compressed, allow_pickle=False:
     isp uint8 [H*3/2, W] and isp_type ("YUV420p" | "NV12"), width, height     raw (distorted) colour, CAM_A
     depth_mm uint16 [H, W]        depth aligned to CAM_A by the device, 0 = invalid
     accel float64 [N, 4]          (device time s, x, y, z m/s^2) ACCELEROMETER_RAW samples in the IMU frame, from
                                   max(previous frame time, t - 0.5 s) to t
     accel_mean, accel_std float64 [3], accel_count int
     exposure_time_us float64, sensitivity_iso, colour_temperature_k, lens_position int64, lens_position_raw float64
                                   the colour frame's own 3A state (ImgFrame getters; -1 = not read), with
     auto_exposure_configured, auto_white_balance_configured bool   (AE on, AWB AUTO, set explicitly; review r1 P1-1)
     rotation_vector float64 [5]   (i, j, k, real, accuracy rad) last BNO08x sample, NaN if none
     t_device_s, t_depth_device_s, t_host_synced_s, t_host_monotonic_s float64, sequence_num int
     isp_raw uint8 [delivered bytes], isp_fb int64 [7]   ONLY when the delivered colour buffer is larger than
                                   H*3/2*W (stride or plane padding): the whole buffer and the SDK's frame-buffer
                                   layout (stride, p1Offset, p2Offset, p3Offset, width, height, bytesPP); isp is
                                   then NOT trustworthy and capture.json counts isp_padded_frames

The drone's VIO uses the same OAK-D (user, 2026-11-18): stop the VIO and any other program using the camera before
--probe / --record and restart it afterwards. This script has no ROS or VIO integration.

Refusals (exit code 2, "oakd_field_recorder refused: CODE: reason"): NO_DEVICE, DEVICE_BUSY (an OAK-D is connected
but another process, e.g. the VIO, holds it), DEVICE_PERMISSION (udev rule), DEVICE_OPEN_FAILED (listed, other
cause), PROJECTOR_NOT_APPLIED, OS_ERROR (e.g. disk full), UNSUPPORTED_DEPTHAI_VERSION,
NO_CAM_A, UNSUPPORTED_COLOUR_SENSOR, NO_STEREO_PAIR, UNSUPPORTED_STEREO_SENSOR, USB_SPEED (below SUPER without
--allow-usb2), NO_IR_PROJECTOR (--projector > 0 on a device without one), OUTPUT_EXISTS, FRAMES_OUT_OF_RANGE,
BAD_ARGUMENT, INSUFFICIENT_DISK, FRAME_TIMEOUT, UNEXPECTED_ISP_LAYOUT, UNEXPECTED_DEPTH_LAYOUT,
NON_INCREASING_DEVICE_TIMESTAMP, UNEXPECTED (any other SDK error, e.g. the USB link dropped: X_LINK_ERROR),
INTERRUPTED (Ctrl+C, exit code 130). A stop after the output folder was created still writes capture.json with
status INCOMPLETE and the error; the converter refuses incomplete captures.

Process exit (review r4 F1): with the pip --target install on Python 3.8 a process that exits within a few
milliseconds of "import depthai" was aborted by the SDK's own resource-loading thread (C++ trace, "core dumped",
exit 134) after the refusal line. Arguments are therefore checked BEFORE depthai is imported, and the script leaves
through os._exit after flushing, so the exit code is the one documented here.

Not a refusal (review r2 N1): an IMU that is reported but delivers no accelerometer sample. The frames are recorded,
counted in capture.json (frames_without_imu_samples) and the final status line, and announced with
"WARNING IMU_NO_SAMPLES"; the converter then needs --gravity-source nominal.

Install on the drone without touching its Python environment (review r1 P1-3): pip install --target into a folder
used only by this script, and run it with PYTHONNOUSERSITE=1 PYTHONPATH=<that folder> (FIELD_RECORDING_KO.md, step 2).
capture.json records which depthai file was imported (host.depthai_file).

Emitters: the IR dot projector is set only when --projector > 0 (setIrLaserDotProjectorIntensity); the flood LED is
set to 0; both setters' return values are recorded, and a dot projector that does not accept the value stops the
recording (PROJECTOR_NOT_APPLIED). --pitch-deg is required. Hardware, the Jetson, the aarch64 wheel and the real
calibration are NOT_TESTED by this source and its CPU fake-SDK tests. Nothing here is flight qualification.
"""
import argparse
import datetime
import hashlib
import io
import json
import math
import os
import platform
import shutil
import sys
import time
import warnings

import numpy as np

FORMAT = 'motar-oakd-raw-capture-v1'
SDK_VERSION = '2.29.0.0'
NUMPY_PIN = '1.24.4'                      # has aarch64 wheels for Python 3.8-3.11; numpy 2 is untested with this SDK
FPS = 10.0
IMU_RATE_HZ = 100
IMU_WINDOW_S = 0.5
SYNC_THRESHOLD_MS = 60                    # above half the frame period (50 ms at FPS 10), below the period
IMU_QUEUE_SIZE = 2000                     # host queue, messages: about 20 s at 100 Hz (review r4 F2)
MAX_FRAMES = 100
FIRST_FRAME_TIMEOUT_S = 10.0
FRAME_TIMEOUT_S = 3.0
DISK_MARGIN = 1.5
# Colour sensor -> pipeline profile. size = ISP output (width, height).
PROFILES = {
    'OV9782': {'resolution': 'THE_800_P', 'isp_scale': None, 'size': (1280, 800), 'manual_focus': False,
               'why': 'OAK-D Pro W colour sensor (global shutter, fixed focus); full 1280x800 sensor'},
    'OV9282': {'resolution': 'THE_800_P', 'isp_scale': None, 'size': (1280, 800), 'manual_focus': False,
               'why': 'CAM_A reported as OV9282: the OV9782 pipeline is used and the name is recorded as reported',
               'warning': 'the colour socket CAM_A reports the mono sensor name OV9282. The firmware reports some '
                          'OV9782 modules this way and then delivers GREY images (Luxonis forum 6070). Recording '
                          'continues; report this warning'},
    'IMX378': {'resolution': 'THE_12_MP', 'isp_scale': (4, 13), 'size': (1248, 936), 'manual_focus': True,
               'why': '4056x3040 ISP 4/13 keeps the full 4:3 sensor at a width that is a multiple of 16, which the '
                      'device requires of an aligned depth; THE_1080_P is a 16:9 crop below the trained 54 deg '
                      'vertical FOV'},
}
DEPTH_WIDTH_MULTIPLE = 16                 # device rule for a depth aligned to the colour camera (preflight r4)
STEREO_SENSORS = ('OV9282', 'OV9782')
FAST_USB = ('SUPER', 'SUPER_PLUS')


class RecorderError(RuntimeError):
    def __init__(self, code, message):
        RuntimeError.__init__(self, '%s: %s' % (code, message))
        self.code = code


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def _name(value):
    """Enum or plain value -> JSON-friendly text."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    name = getattr(value, 'name', None)
    return name if isinstance(name, str) else str(value)


def _jsonable(value):
    if isinstance(value, dict):
        return {str(_name(k)): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return _name(value)


def _seconds(stamp):
    return float(stamp.total_seconds()) if hasattr(stamp, 'total_seconds') else float(stamp)


def socket_name(socket):
    text = _name(socket)
    return text if isinstance(text, str) else str(text)


# ---------------------------------------------------------------- argument checks (before any device access)

def check_frames(frames):
    if isinstance(frames, bool) or not isinstance(frames, int) or not 1 <= frames <= MAX_FRAMES:
        raise RecorderError('FRAMES_OUT_OF_RANGE', '--frames must be an integer in 1..%d' % MAX_FRAMES)
    return frames


def check_output_dir(path):
    if os.path.lexists(path):
        raise RecorderError('OUTPUT_EXISTS', '%s already exists; give a NEW directory' % path)
    parent = os.path.dirname(os.path.abspath(path))
    if not os.path.isdir(parent):
        raise RecorderError('BAD_ARGUMENT', 'parent directory %s does not exist; create it first: mkdir -p %s'
                            % (parent, parent))
    return path


def check_record_args(args):
    check_frames(args.frames)
    check_output_dir(args.output)
    if not isinstance(args.scene, str) or not args.scene.strip():
        raise RecorderError('BAD_ARGUMENT', '--scene must describe the objects, tape distances and sun / shade')
    for key, low, high in (('camera_height_m', 0.0, 5.0), ('camera_height_uncertainty_m', 0.0, 0.5)):
        value = getattr(args, key)
        if value is None or not math.isfinite(value) or not low < value <= high:
            raise RecorderError('BAD_ARGUMENT', '--%s must be in (%g, %g]' % (key.replace('_', '-'), low, high))
    if args.pitch_deg is None or not math.isfinite(args.pitch_deg) or abs(args.pitch_deg) > 45.0:
        raise RecorderError('BAD_ARGUMENT', '--pitch-deg is required, finite and within +-45 (positive = nose down)')
    if not math.isfinite(args.projector) or not 0.0 <= args.projector <= 1.0:
        raise RecorderError('BAD_ARGUMENT', '--projector must be in 0..1')
    if args.stereo_mode not in ('extended', 'subpixel'):
        raise RecorderError('BAD_ARGUMENT', '--stereo-mode must be extended or subpixel')
    if not math.isfinite(args.warmup_s) or not 0.0 <= args.warmup_s <= 30.0:
        raise RecorderError('BAD_ARGUMENT', '--warmup-s must be in 0..30')


INSTALL_HINT = ("install it into the recorder's own folder and run through the oakrec alias "
                "(FIELD_RECORDING_KO.md step 2: PIP_USER=0 python3 -m pip install --no-cache-dir --target "
                "~/oakd_rec_pkgs depthai==%s numpy==%s); do NOT pip install into the drone's own Python"
                % (SDK_VERSION, NUMPY_PIN))
UNEXPECTED_HINT = ('this is usually the USB link or its power dropping (X_LINK_ERROR): unplug and replug the camera '
                   'and record again into a NEW folder; if it happens only with --projector above 0, record that '
                   'scene with --projector 0 and report this message')


def check_sdk(dai):
    version = getattr(dai, '__version__', None)
    if version != SDK_VERSION:
        raise RecorderError('UNSUPPORTED_DEPTHAI_VERSION',
                            'depthai %s is required (found %s); %s' % (SDK_VERSION, version, INSTALL_HINT))


def check_usb(speed, allow_usb2):
    name = socket_name(speed)
    if name in FAST_USB:
        return name
    if allow_usb2:
        return name
    raise RecorderError('USB_SPEED', 'USB speed %s is below SUPER (USB3); use a USB3 cable and port, or pass '
                                     '--allow-usb2 knowingly (frames may drop)' % name)


def sensor_map(dai, device):
    """{socket name: sensor name} from the connected device."""
    names = device.getCameraSensorNames()
    return {socket_name(k): str(v).strip().upper() for k, v in dict(names).items()}


def select_profile(sensors):
    """Pick the pipeline profile from {socket: sensor}. Refuses unknown hardware with a named reason."""
    colour = sensors.get('CAM_A')
    if colour is None:
        raise RecorderError('NO_CAM_A', 'no camera on CAM_A (the colour socket); connected: %s' % sensors)
    if colour not in PROFILES:
        raise RecorderError('UNSUPPORTED_COLOUR_SENSOR', 'CAM_A sensor %s has no recording profile (supported: %s)'
                            % (colour, sorted(PROFILES)))
    if 'CAM_B' not in sensors or 'CAM_C' not in sensors:
        raise RecorderError('NO_STEREO_PAIR', 'stereo needs CAM_B and CAM_C; connected: %s' % sensors)
    for socket in ('CAM_B', 'CAM_C'):
        if sensors[socket] not in STEREO_SENSORS:
            raise RecorderError('UNSUPPORTED_STEREO_SENSOR', '%s sensor %s is not one of %s'
                                % (socket, sensors[socket], STEREO_SENSORS))
    profile = dict(PROFILES[colour])
    profile.setdefault('warning', None)
    profile['colour_sensor'] = colour
    profile['stereo_sensors'] = [sensors['CAM_B'], sensors['CAM_C']]
    return profile


# ---------------------------------------------------------------- pipeline

def build_pipeline(dai, profile, stereo_mode, lens_position=None, rotation_vector=False, imu_present=True):
    """Pinned v2 graph only (no Device access). Returns (pipeline, description).

    Colour 3A is configured explicitly (repair r1, review P1-1): auto exposure ON, auto white balance AUTO, focus manual
    at the calibration lens position (IMX378) or fixed (OV9782); the per-frame values the camera chose are saved."""
    if stereo_mode not in ('extended', 'subpixel'):
        raise RecorderError('BAD_ARGUMENT', 'unsupported stereo mode %r' % (stereo_mode,))
    width, height = profile['size']
    if width % DEPTH_WIDTH_MULTIPLE or height % 2:
        raise RecorderError('BAD_ARGUMENT', 'profile size %dx%d: the device needs an aligned depth width that is a '
                                            'multiple of %d and an even height' % (width, height, DEPTH_WIDTH_MULTIPLE))
    pipeline = dai.Pipeline()
    rgb = pipeline.create(dai.node.ColorCamera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    sync = pipeline.create(dai.node.Sync)
    out = pipeline.create(dai.node.XLinkOut)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(getattr(dai.ColorCameraProperties.SensorResolution, profile['resolution']))
    if profile['isp_scale'] is not None:
        rgb.setIspScale(*profile['isp_scale'])
    rgb.setFps(FPS)
    rgb.initialControl.setAutoExposureEnable()
    rgb.initialControl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
    focus = None
    if profile['manual_focus'] and lens_position is not None:
        if isinstance(lens_position, bool) or not isinstance(lens_position, int) or not 0 <= lens_position <= 255:
            raise RecorderError('BAD_ARGUMENT', 'calibration lens position %r is not in 0..255' % (lens_position,))
        rgb.initialControl.setManualFocus(lens_position)
        focus = lens_position
    for mono, socket in ((left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)):
        mono.setBoardSocket(socket)
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
        mono.setFps(FPS)
    with warnings.catch_warnings():            # 2.29 prints "HIGH_DENSITY is deprecated" on every run (r4 F3)
        warnings.simplefilter('ignore', DeprecationWarning)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setDepthAlignmentUseSpecTranslation(False)
    stereo.setDisparityToDepthUseSpecTranslation(False)
    stereo.setOutputSize(width, height)
    stereo.setExtendedDisparity(stereo_mode == 'extended')
    stereo.setSubpixel(stereo_mode == 'subpixel')
    sync.setSyncThreshold(datetime.timedelta(milliseconds=SYNC_THRESHOLD_MS))
    out.setStreamName('rgbd')
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    rgb.isp.link(sync.inputs['rgb'])
    stereo.depth.link(sync.inputs['depth'])
    sync.out.link(out.input)
    imu_sensors = []
    if imu_present:
        imu = pipeline.create(dai.node.IMU)
        imu_out = pipeline.create(dai.node.XLinkOut)
        imu_sensors.append('ACCELEROMETER_RAW')
        imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_RATE_HZ)
        if rotation_vector:
            imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, IMU_RATE_HZ)
            imu_sensors.append('ROTATION_VECTOR')
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        imu_out.setStreamName('imu')
        imu.out.link(imu_out.input)
    stereo_config = {}
    for key, getter in (('confidence_threshold', 'getConfidenceThreshold'), ('median_filter', 'getMedianFilter'),
                        ('left_right_check_threshold', 'getLeftRightCheckThreshold'),
                        ('bilateral_filter_sigma', 'getBilateralFilterSigma'), ('depth_unit', 'getDepthUnit'),
                        ('max_disparity', 'getMaxDisparity')):
        try:
            stereo_config[key] = _jsonable(getattr(stereo.initialConfig, getter)())
        except Exception as exc:                   # fake SDKs and other versions: record, do not guess
            stereo_config[key] = 'NOT_RECORDED (%s)' % type(exc).__name__
    description = {
        'colour': {'socket': 'CAM_A', 'sensor': profile['colour_sensor'], 'resolution': profile['resolution'],
                   'isp_scale': list(profile['isp_scale']) if profile['isp_scale'] else None,
                   'output': 'isp', 'size': [width, height], 'fps': FPS, 'manual_focus_lens_position': focus,
                   'stored_as': 'ISP frame as delivered (YUV420p or NV12), converted on the PC like getCvFrame',
                   'auto_exposure': 'ON (initialControl.setAutoExposureEnable)',
                   'auto_white_balance': 'AUTO (initialControl.setAutoWhiteBalanceMode)',
                   'focus': 'manual, calibration lens position %s' % focus if focus is not None else
                            ('fixed focus (no lens control)' if not profile['manual_focus'] else
                             'NOT_SET (no calibration lens position)'),
                   'per_frame': 'exposure_time_us, sensitivity_iso, colour_temperature_k, lens_position in each npz',
                   'profile_reason': profile['why'], 'sensor_warning': profile.get('warning')},
        'stereo': {'sockets': ['CAM_B', 'CAM_C'], 'sensors': profile['stereo_sensors'], 'resolution': 'THE_800_P',
                   'fps': FPS, 'preset': 'HIGH_DENSITY', 'left_right_check': True, 'mode': stereo_mode,
                   'extended_disparity': stereo_mode == 'extended', 'subpixel': stereo_mode == 'subpixel',
                   'depth_align_socket': 'CAM_A', 'output_size': [width, height],
                   'depth_alignment_use_spec_translation': False, 'disparity_to_depth_use_spec_translation': False,
                   'distortion_correction': 'SDK default (not set)', 'depth_unit': 'mm, 0 = invalid',
                   'initial_config': stereo_config,
                   'aligned_depth_coordinate_model': 'NOT_ESTABLISHED (distorted CAM_A image or undistorted CAM_A '
                                                     'pinhole); chosen explicitly at conversion'},
        'sync': {'node': 'Sync', 'threshold_ms': SYNC_THRESHOLD_MS, 'attempts': 'SDK default (-1: only synced pairs)',
                 'per_frame': 'depth_minus_colour_s in capture.json frames'},
        'imu': {'present': bool(imu_present), 'sensors': imu_sensors, 'rate_hz': IMU_RATE_HZ,
                'rate_hz_is': 'the REQUESTED rate; the IMU rounds it to a rate it supports (a BNO08x up to 125 Hz)',
                'host_queue_messages': IMU_QUEUE_SIZE, 'all_samples_file': 'imu.npz',
                'batch_report_threshold': 1,
                'max_batch_reports': 10, 'frame_window_s': IMU_WINDOW_S},
    }
    return pipeline, description


# ---------------------------------------------------------------- device readout

def _try(fn, *args):
    try:
        return _jsonable(fn(*args)), None
    except Exception as exc:                       # the SDK raises RuntimeError for absent EEPROM entries
        return None, '%s: %s' % (type(exc).__name__, exc)


def calibration_summary(dai, calib, sockets, colour_size=None):
    """Intrinsics / distortion per socket, IMU-to-CAM_A extrinsics, baseline and MinZ hints."""
    summary = {'sockets': {}}
    for name in sockets:
        socket = getattr(dai.CameraBoardSocket, name, None)
        if socket is None:
            continue
        entry = {}
        entry['intrinsics_default'], err = _try(calib.getCameraIntrinsics, socket)
        if err:
            entry['intrinsics_error'] = err
        entry['distortion_model'], _ = _try(calib.getDistortionModel, socket)
        entry['distortion_coefficients'], _ = _try(calib.getDistortionCoefficients, socket)
        entry['fov_spec_deg'], _ = _try(calib.getFov, socket, True)
        entry['fov_from_intrinsics_deg'], _ = _try(calib.getFov, socket, False)
        entry['lens_position'], _ = _try(calib.getLensPosition, socket)
        summary['sockets'][name] = entry
    if colour_size is not None and 'CAM_A' in summary['sockets']:
        k, err = _try(calib.getCameraIntrinsics, dai.CameraBoardSocket.CAM_A, colour_size[0], colour_size[1])
        summary['sockets']['CAM_A']['intrinsics_at_output'] = k
        summary['sockets']['CAM_A']['output_size'] = list(colour_size)
        if err:
            summary['sockets']['CAM_A']['intrinsics_at_output_error'] = err
    ext, err = _try(calib.getImuToCameraExtrinsics, dai.CameraBoardSocket.CAM_A, False)
    summary['imu_to_cam_a'] = {'matrix_4x4': ext, 'error': err,
                               'convention': 'getImuToCameraExtrinsics(CAM_A): p_cam = R p_imu + T, T in cm'}
    baseline, _ = _try(calib.getBaselineDistance, dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_B, False)
    summary['stereo_baseline_cm'] = baseline
    hints = None
    k_mono = None
    if 'CAM_C' in sockets:
        k_mono, _ = _try(calib.getCameraIntrinsics, dai.CameraBoardSocket.CAM_C, 1280, 800)
    if isinstance(baseline, (int, float)) and k_mono:
        fx = float(k_mono[0][0])
        b_m = abs(float(baseline)) / 100.0
        hints = {'mono_fx_800p_px': fx, 'baseline_m': b_m,
                 'min_z_m_standard_95px': fx * b_m / 95.0, 'min_z_m_extended_190px': fx * b_m / 190.0,
                 'note': 'MinZ ~ fx * baseline / max disparity (DepthAI v2 StereoDepth docs); hint only'}
    summary['stereo_min_z_hints'] = hints
    return summary


def device_identity(dai, device, calib):
    eeprom = None
    try:
        data = calib.getEepromData()
        eeprom = {k: _jsonable(getattr(data, k, None)) for k in ('productName', 'boardName', 'boardRev', 'boardConf',
                                                                  'batchName', 'deviceName', 'version')}
    except Exception as exc:
        eeprom = {'error': '%s: %s' % (type(exc).__name__, exc)}
    cameras = []
    try:
        for feature in device.getConnectedCameraFeatures():
            cameras.append({'socket': socket_name(feature.socket), 'sensor': str(feature.sensorName),
                            'width': int(feature.width), 'height': int(feature.height),
                            'has_autofocus': _jsonable(getattr(feature, 'hasAutofocus', None))})
    except Exception as exc:
        cameras.append({'error': '%s: %s' % (type(exc).__name__, exc)})
    ident = {}
    for key, fn in (('product_name', 'getProductName'), ('device_name', 'getDeviceName'), ('mx_id', 'getMxId'),
                    ('usb_speed', 'getUsbSpeed'), ('imu', 'getConnectedIMU'), ('ir_drivers', 'getIrDrivers')):
        ident[key], err = _try(getattr(device, fn))
        if err:
            ident[key + '_error'] = err
    ident.update(eeprom=eeprom, cameras=cameras, depthai_version=getattr(dai, '__version__', None))
    return ident


def has_ir_projector(identity):
    drivers = identity.get('ir_drivers')
    return isinstance(drivers, list) and len(drivers) > 0


def imu_is_present(identity):
    imu = identity.get('imu')
    return isinstance(imu, str) and imu.strip().upper() not in ('', 'NONE')


def imu_has_rotation_vector(identity):
    return isinstance(identity.get('imu'), str) and identity['imu'].upper().startswith('BNO08')


BUSY_HINT = ('the drone VIO uses this same OAK-D: stop the VIO (and any other program using the camera) before '
             '--probe / --record and restart it afterwards')
UDEV_HINT = ('if the VIO is already stopped, check the udev rule for USB vendor 03e7 (FIELD_RECORDING_KO.md 2.5) '
             'and unplug / replug the camera')


def classify_open_failure(text, connected):
    """Name why dai.Device() failed, from the error text and the XLink state / status of the listed devices.

    DEVICE_PERMISSION: X_LINK_INSUFFICIENT_PERMISSIONS or 'permission' in the error (udev rule).
    DEVICE_BUSY: X_LINK_DEVICE_ALREADY_IN_USE / X_LINK_ALREADY_OPEN, a device already BOOTED by another process, or
    'in use' / 'already' / 'busy' in the error. DEVICE_OPEN_FAILED: listed but none of these. NO_DEVICE: nothing listed.
    """
    upper = text.upper()
    states = [(_name(getattr(d, 'state', None)), _name(getattr(d, 'status', None))) for d in connected]
    if 'PERMISSION' in upper or any(status == 'X_LINK_INSUFFICIENT_PERMISSIONS' for _, status in states):
        return 'DEVICE_PERMISSION'
    if (any(word in upper for word in ('IN_USE', 'IN USE', 'ALREADY', 'BUSY'))
            or any(status in ('X_LINK_DEVICE_ALREADY_IN_USE', 'X_LINK_ALREADY_OPEN') for _, status in states)
            or any(state in ('X_LINK_BOOTED', 'X_LINK_FLASH_BOOTED') for state, _ in states)):
        return 'DEVICE_BUSY'
    return 'DEVICE_OPEN_FAILED' if connected else 'NO_DEVICE'


def open_device(dai):
    """dai.Device() or a named refusal (classify_open_failure); every message names both likely causes."""
    try:
        return dai.Device()
    except RuntimeError as exc:
        text = str(exc)
        try:
            connected = list(dai.Device.getAllConnectedDevices())
        except Exception:
            connected = []
        code = classify_open_failure(text, connected)
        listed = [(_name(getattr(d, 'state', None)), _name(getattr(d, 'status', None))) for d in connected]
        messages = {
            'DEVICE_BUSY': 'an OAK-D is connected but another process holds it (%s; devices %s): %s; %s'
                           % (text, listed, BUSY_HINT, UDEV_HINT),
            'DEVICE_PERMISSION': 'an OAK-D is connected but this user may not open it (%s; devices %s): the udev rule '
                                 'for USB vendor 03e7 is missing or not loaded (FIELD_RECORDING_KO.md 2.5); also '
                                 'make sure the VIO is stopped' % (text, listed),
            'DEVICE_OPEN_FAILED': 'an OAK-D is listed but cannot be opened (%s; devices %s): %s; %s'
                                  % (text, listed, BUSY_HINT, UDEV_HINT),
            'NO_DEVICE': 'no OAK-D found (%s): check the USB3 cable and the udev rule; if the camera is plugged in, %s'
                         % (text, BUSY_HINT)}
        raise RecorderError(code, messages[code]) from None


# ---------------------------------------------------------------- frames

def isp_layout(isp, isp_type, width, height):
    """Validate the delivered ISP colour frame; returns it as uint8 [H*3/2, W]."""
    arr = np.asarray(isp)
    if isp_type not in ('YUV420p', 'NV12'):
        raise RecorderError('UNEXPECTED_ISP_LAYOUT', 'ISP frame type %s (expected YUV420p or NV12)' % isp_type)
    if arr.dtype != np.uint8 or arr.size != width * height * 3 // 2 or width % 2 or height % 2:
        raise RecorderError('UNEXPECTED_ISP_LAYOUT', 'ISP frame %s %s does not hold %dx%d YUV 4:2:0 without padding'
                            % (arr.dtype, arr.shape, width, height))
    return arr.reshape(height * 3 // 2, width)


def depth_layout(depth, width, height):
    arr = np.asarray(depth)
    if arr.dtype != np.uint16 or arr.shape != (height, width):
        raise RecorderError('UNEXPECTED_DEPTH_LAYOUT', 'depth %s %s is not uint16 [%d, %d] aligned to CAM_A'
                            % (arr.dtype, arr.shape, height, width))
    return arr


class TimestampGuard:
    """Device timestamps of saved frames must strictly increase."""
    def __init__(self):
        self.last = None

    def check(self, t):
        if not math.isfinite(t) or (self.last is not None and not t > self.last):
            raise RecorderError('NON_INCREASING_DEVICE_TIMESTAMP', 'frame device time %r after %r' % (t, self.last))
        self.last = t
        return t


def accel_window(samples, t_frame, t_previous):
    """Samples (t, x, y, z) with max(t_previous, t_frame - window) < t <= t_frame."""
    start = t_frame - IMU_WINDOW_S if t_previous is None else max(t_previous, t_frame - IMU_WINDOW_S)
    rows = [s for s in samples if start < s[0] <= t_frame]
    return np.asarray(rows, np.float64).reshape(-1, 4)


COLOUR_3A_KEYS = ('exposure_time_us', 'sensitivity_iso', 'colour_temperature_k', 'lens_position', 'lens_position_raw')
NOT_READ = {'exposure_time_us': -1.0, 'sensitivity_iso': -1, 'colour_temperature_k': -1, 'lens_position': -1,
            'lens_position_raw': -1.0}


def colour_3a(message):
    """The colour frame's own 3A state (DepthAI 2.29 ImgFrame getters); -1 where a getter is absent or fails."""
    values = dict(NOT_READ)
    for key, getter in (('exposure_time_us', 'getExposureTime'), ('sensitivity_iso', 'getSensitivity'),
                        ('colour_temperature_k', 'getColorTemperature'), ('lens_position', 'getLensPosition'),
                        ('lens_position_raw', 'getLensPositionRaw')):
        try:
            value = getattr(message, getter)()
        except Exception:
            continue
        if key == 'exposure_time_us':
            values[key] = _seconds(value) * 1e6
        elif key == 'lens_position_raw':
            values[key] = float(value)
        else:
            values[key] = int(value)
    return values


FB_FIELDS = ('stride', 'p1Offset', 'p2Offset', 'p3Offset', 'width', 'height', 'bytesPP')


def frame_buffer_layout(message):
    """The SDK's frame-buffer description of a colour frame (FB_FIELDS order); -1 where it cannot be read."""
    try:
        fb = message.getRaw().fb
    except Exception:
        return [-1] * len(FB_FIELDS)
    values = []
    for name in FB_FIELDS:
        try:
            values.append(int(getattr(fb, name)))
        except Exception:
            values.append(-1)
    return values


def frame_arrays(isp, isp_type, width, height, depth, accel, rotation, stamps, sequence_num, camera=None):
    """One frame's npz arrays. camera: colour_3a() values (NOT_READ when None) plus the configured 3A modes."""
    camera = dict(NOT_READ) if camera is None else dict(camera)
    accel = np.asarray(accel, np.float64).reshape(-1, 4)
    count = int(accel.shape[0])
    mean = accel[:, 1:].mean(axis=0) if count else np.full(3, np.nan)
    std = accel[:, 1:].std(axis=0) if count else np.full(3, np.nan)
    rot = np.full(5, np.nan) if rotation is None else np.asarray(rotation, np.float64).reshape(5)
    return {'isp': isp_layout(isp, isp_type, width, height), 'isp_type': np.array(isp_type),
            'width': np.array(width, np.int64), 'height': np.array(height, np.int64),
            'depth_mm': depth_layout(depth, width, height),
            'accel': accel, 'accel_mean': mean, 'accel_std': std, 'accel_count': np.array(count, np.int64),
            'rotation_vector': rot,
            't_device_s': np.array(float(stamps['t_device_s'])),
            't_depth_device_s': np.array(float(stamps['t_depth_device_s'])),
            't_host_synced_s': np.array(float(stamps['t_host_synced_s'])),
            't_host_monotonic_s': np.array(float(stamps['t_host_monotonic_s'])),
            'sequence_num': np.array(int(sequence_num), np.int64),
            'exposure_time_us': np.array(float(camera['exposure_time_us'])),
            'sensitivity_iso': np.array(int(camera['sensitivity_iso']), np.int64),
            'colour_temperature_k': np.array(int(camera['colour_temperature_k']), np.int64),
            'lens_position': np.array(int(camera['lens_position']), np.int64),
            'lens_position_raw': np.array(float(camera['lens_position_raw'])),
            'auto_exposure_configured': np.array(bool(camera.get('auto_exposure_configured', True))),
            'auto_white_balance_configured': np.array(bool(camera.get('auto_white_balance_configured', True)))}


def colour_3a_summary(frames):
    """min / max per 3A key over the saved frames (ignoring -1 = not read)."""
    summary = {}
    for key in COLOUR_3A_KEYS:
        values = [f[key] for f in frames if f.get(key) is not None and f[key] != -1]
        summary[key] = {'min': min(values), 'max': max(values), 'frames': len(values)} if values else 'NOT_RECORDED'
    return summary


def encode_frame(arrays):
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **arrays)
    return buffer.getvalue()


def synthetic_frame(width, height, noise=2.0, seed=0):
    """Outdoor-like test frame (sky over textured ground, sensor noise sigma in DN) for the size estimate."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:height, 0:width].astype(np.float32)
    horizon = 0.45 * height
    sky = 150.0 + 40.0 * (1.0 - y / height)
    ground = 90.0 + 25.0 * np.sin(x / 3.0 + y / 11.0) + 15.0 * np.sin(x / 7.0) * np.cos(y / 5.0)
    luma = np.where(y < horizon, sky, ground) + rng.normal(0.0, noise, (height, width))
    yuv = np.empty((height * 3 // 2, width), np.uint8)
    yuv[:height] = np.clip(luma, 0, 255).astype(np.uint8)
    chroma = 128.0 + rng.normal(0.0, noise, (height // 2, width)) + np.where(y[:height // 2] < horizon / 2, 20.0, -15.0)
    yuv[height:] = np.clip(chroma, 0, 255).astype(np.uint8)
    rows = np.maximum(y - horizon, 1.0)
    depth = np.where(y < horizon, 0.0, np.clip(1500.0 * (height - horizon) / rows, 400.0, 20000.0))
    depth = (depth * (1.0 + rng.normal(0.0, 0.01, depth.shape))).astype(np.uint16)
    accel = np.column_stack([np.linspace(0.0, 0.5, 50), rng.normal(0, 0.02, 50), rng.normal(-9.81, 0.02, 50),
                             rng.normal(0, 0.02, 50)])
    stamps = {'t_device_s': 1.0, 't_depth_device_s': 1.0, 't_host_synced_s': 1.0, 't_host_monotonic_s': 1.0}
    return frame_arrays(yuv, 'YUV420p', width, height, depth, accel, None, stamps, 1)


def estimate_bytes_per_frame(width, height, noise=2.0):
    """Compressed size of one synthetic frame at this size, encoded exactly as the recorder does."""
    return len(encode_frame(synthetic_frame(width, height, noise=noise)))


def check_disk(path, frames, bytes_per_frame):
    free = shutil.disk_usage(os.path.dirname(os.path.abspath(path))).free
    need = int(DISK_MARGIN * frames * bytes_per_frame) + (10 << 20)
    if free < need:
        raise RecorderError('INSUFFICIENT_DISK', '%d MB free, need about %d MB for %d frames'
                            % (free >> 20, need >> 20, frames))
    return free, need


# ---------------------------------------------------------------- probe / record

def _imu_mean(dai, device, seconds, clock=time.monotonic, sleep=time.sleep):
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    out = pipeline.create(dai.node.XLinkOut)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, IMU_RATE_HZ)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)
    out.setStreamName('imu')
    imu.out.link(out.input)
    device.startPipeline(pipeline)
    queue = device.getOutputQueue(name='imu', maxSize=50, blocking=False)
    samples, end = [], clock() + seconds
    while clock() < end:
        for data in queue.tryGetAll():
            for packet in data.packets:
                a = packet.acceleroMeter
                samples.append((float(a.x), float(a.y), float(a.z)))
        sleep(0.01)
    if not samples:
        return {'count': 0}
    arr = np.asarray(samples)
    return {'count': len(samples), 'mean_m_s2': arr.mean(axis=0).tolist(), 'std_m_s2': arr.std(axis=0).tolist(),
            'norm_m_s2': float(np.linalg.norm(arr.mean(axis=0))),
            'frame': 'raw IMU axes (vendor), hold the camera level and still while probing'}


def probe(dai, imu_seconds=1.0):
    check_sdk(dai)
    device = open_device(dai)
    try:
        calib = device.readCalibration()
        identity = device_identity(dai, device, calib)
        sensors = sensor_map(dai, device)
        try:
            profile = select_profile(sensors)
            plan = {'profile': profile['colour_sensor'], 'size': list(profile['size']),
                    'resolution': profile['resolution'], 'isp_scale': profile['isp_scale'],
                    'warning': profile.get('warning'), 'refusal': None}
        except RecorderError as exc:
            profile, plan = None, {'profile': None, 'refusal': str(exc)}
        report = {'format': 'motar-oakd-probe-v1', 'host': host_info(), 'device': identity, 'sensors': sensors,
                  'recording_plan': plan,
                  'calibration': calibration_summary(dai, calib, sorted(sensors),
                                                     profile['size'] if profile else None)}
        try:
            report['usb_check'] = check_usb(identity.get('usb_speed'), False)
        except RecorderError as exc:
            report['usb_check'] = str(exc)
        if identity.get('imu') not in (None, 'NONE', '') and imu_seconds > 0:
            try:
                report['imu_sample'] = _imu_mean(dai, device, imu_seconds)
            except Exception as exc:
                report['imu_sample'] = {'error': '%s: %s' % (type(exc).__name__, exc)}
        return report
    finally:
        device.close()


def host_info():
    return {'python': sys.version.split()[0], 'platform': platform.platform(), 'machine': platform.machine(),
            'numpy': np.__version__, 'hostname': platform.node()}


def _now_utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def _write_json(path, value):
    with open(path, 'x') as stream:
        json.dump(_jsonable(value), stream, indent=1, sort_keys=True, allow_nan=False)
        stream.write('\n')


def _next_group(queue, deadline, clock, sleep, idle=None):
    """Next synchronized group; idle() runs on every empty poll (the IMU queue is drained while waiting)."""
    while True:
        group = queue.tryGet()
        if group is not None:
            return group
        if idle is not None:
            idle()
        if clock() >= deadline:
            raise RecorderError('FRAME_TIMEOUT', 'no synchronized RGB-D frame before the deadline; the camera '
                                                 'delivered no colour + depth pair (replug the USB cable and record '
                                                 'into a NEW folder; if it repeats, send this message)')
        sleep(0.005)


def _drain_imu(queue, accel, rotation, keep=None):
    """Move every queued accelerometer sample into accel (per-frame windows) and keep (the whole capture)."""
    for data in queue.tryGetAll():
        for packet in data.packets:
            a = packet.acceleroMeter
            t = _seconds(a.getTimestampDevice())
            if t > 0:
                sample = (t, float(a.x), float(a.y), float(a.z))
                accel.append(sample)
                if keep is not None:
                    keep.append(sample)
            if rotation is not None:
                r = packet.rotationVector
                if _seconds(r.getTimestampDevice()) > 0:
                    rotation[:] = [float(r.i), float(r.j), float(r.k), float(r.real),
                                   float(r.rotationVectorAccuracy)]


def _say(message):
    print(message, flush=True)


def record(args, dai, clock=time.monotonic, sleep=time.sleep, log=_say):
    """Record args.frames frames into args.output. Returns the capture metadata."""
    check_record_args(args)
    check_sdk(dai)
    device = open_device(dai)
    output, meta, frames_dir, accel_all = args.output, None, None, None
    try:
        usb = check_usb(device.getUsbSpeed(), args.allow_usb2)
        sensors = sensor_map(dai, device)
        profile = select_profile(sensors)
        calib = device.readCalibration()
        identity = device_identity(dai, device, calib)
        if args.projector > 0 and not has_ir_projector(identity):
            raise RecorderError('NO_IR_PROJECTOR', 'this device lists no IR driver; record with --projector 0')
        width, height = profile['size']
        lens = None
        if profile['manual_focus']:
            lens, err = _try(calib.getLensPosition, dai.CameraBoardSocket.CAM_A)
            lens = int(lens) if isinstance(lens, (int, float)) and not err else None
        rotation_vector = imu_has_rotation_vector(identity)
        imu_present = imu_is_present(identity)
        pipeline, description = build_pipeline(dai, profile, args.stereo_mode, lens, rotation_vector, imu_present)
        if profile.get('warning'):
            log('WARNING COLOUR_SENSOR_NAME: %s' % profile['warning'])
        estimate = estimate_bytes_per_frame(width, height)
        free, need = check_disk(output, args.frames, estimate)
        log('estimated %.2f MB/frame (synthetic %dx%d, sensor noise 2 DN), %d frames -> about %.0f MB; %d MB free'
            % (estimate / 1e6, width, height, args.frames, estimate * args.frames / 1e6, free >> 20))
        calibration = calibration_summary(dai, calib, sorted(sensors), (width, height))
        try:
            eeprom = _jsonable(calib.eepromToJson())
        except Exception as exc:
            eeprom = {'error': '%s: %s' % (type(exc).__name__, exc)}
        os.makedirs(output)                      # check_output_dir above; makedirs refuses an existing path
        frames_dir = os.path.join(output, 'frames')
        os.mkdir(frames_dir)
        description['projector'] = {'ir_dot_projector_intensity': float(args.projector),
                                    'ir_dot_projector_applied': None, 'ir_flood_intensity': 0.0,
                                    'ir_flood_off_applied': None}
        description['usb_speed'] = usb
        meta = {'format': FORMAT, 'source': 'oakd_field_recorder', 'status': 'RECORDING',
                'recorder': {'path': os.path.abspath(__file__), 'sha256': sha256_file(os.path.abspath(__file__)),
                             'argv': list(getattr(args, 'argv', []) or [])},
                'host': dict(host_info(), depthai_file=getattr(dai, '__file__', None)), 'device': identity,
                'sensors': sensors, 'calibration': calibration,
                'calibration_eeprom': eeprom, 'pipeline': description, 'scene': args.scene.strip(),
                'camera_height_m': float(args.camera_height_m),
                'camera_height_uncertainty_m': float(args.camera_height_uncertainty_m),
                'pitch_deg_nominal': float(args.pitch_deg),
                'pitch_sign': 'positive = nose down', 'frames_requested': args.frames, 'warmup_s': args.warmup_s,
                'estimated_bytes_per_frame': estimate, 'started_utc': _now_utc(), 'frames': [],
                'vehicle_commands_sent': False, 'hardware_qualification': 'NOT_TESTED'}
        device.startPipeline(pipeline)
        emitters = description['projector']
        if has_ir_projector(identity):
            # Review r1 P2-1: the setters return bool; a failed set must not mislabel the frames.
            emitters['ir_flood_off_applied'] = bool(device.setIrFloodLightIntensity(0.0))
            if not emitters['ir_flood_off_applied']:
                log('WARNING: setIrFloodLightIntensity(0) returned false (boot default is off); recorded')
            if args.projector > 0:
                emitters['ir_dot_projector_applied'] = bool(device.setIrLaserDotProjectorIntensity(
                    float(args.projector)))
                if not emitters['ir_dot_projector_applied']:
                    raise RecorderError('PROJECTOR_NOT_APPLIED', 'setIrLaserDotProjectorIntensity(%g) returned false;'
                                        ' record this scene with --projector 0 only' % args.projector)
            else:
                emitters['ir_dot_projector_applied'] = 'NOT_SET (boot default off)'
        accel_all = []                             # every accelerometer sample of this capture (imu.npz)
        rgbd = device.getOutputQueue(name='rgbd', maxSize=4, blocking=False)
        imu_queue = (device.getOutputQueue(name='imu', maxSize=IMU_QUEUE_SIZE, blocking=False)
                     if imu_present else None)
        accel, rotation = [], ([math.nan] * 5 if rotation_vector else None)

        def drain():                               # also called on every empty poll, so the IMU queue never fills
            if imu_queue is not None:
                _drain_imu(imu_queue, accel, rotation, accel_all)
        guard, previous, previous_seq = TimestampGuard(), None, None
        meta['frames_without_imu_samples'] = 0
        meta['sequence_gaps'] = 0
        meta['isp_padded_frames'] = 0
        # Warm-up (preflight r4): the pipeline start itself can take seconds, so the warm-up time counts from the
        # first RECEIVED frame; every warm-up frame is discarded and the exposure it started and ended at is kept.
        warm = {'seconds_requested': float(args.warmup_s), 'counted_from': 'first received frame',
                'frames_discarded': 0, 'first': None, 'last': None}
        meta['warmup'] = warm
        if args.warmup_s > 0:
            group = _next_group(rgbd, clock() + FIRST_FRAME_TIMEOUT_S, clock, sleep, drain)
            warm_start = clock()
            while True:
                state = colour_3a(group['rgb'])
                state = {'exposure_time_us': state['exposure_time_us'], 'sensitivity_iso': state['sensitivity_iso']}
                warm['first'] = warm['first'] or state
                warm['last'] = state
                warm['frames_discarded'] += 1
                drain()
                if clock() - warm_start >= args.warmup_s:
                    break
                group = _next_group(rgbd, clock() + FRAME_TIMEOUT_S, clock, sleep, drain)
            log('warm-up: %d frames discarded in %.1f s after the first frame; exposure %.0f -> %.0f us, ISO %d -> %d'
                % (warm['frames_discarded'], clock() - warm_start, warm['first']['exposure_time_us'],
                   warm['last']['exposure_time_us'], warm['first']['sensitivity_iso'],
                   warm['last']['sensitivity_iso']))
        for index in range(args.frames):
            group = _next_group(rgbd, clock() + (FIRST_FRAME_TIMEOUT_S if index == 0 else FRAME_TIMEOUT_S),
                                clock, sleep, drain)
            host_t = float(clock())
            colour, depth = group['rgb'], group['depth']
            t = guard.check(_seconds(colour.getTimestampDevice()))
            drain()
            window = accel_window(accel, t, previous)
            accel[:] = [s for s in accel if s[0] > t]
            stamps = {'t_device_s': t, 't_depth_device_s': _seconds(depth.getTimestampDevice()),
                      't_host_synced_s': _seconds(colour.getTimestamp()), 't_host_monotonic_s': host_t}
            isp_type = socket_name(colour.getType())
            camera = colour_3a(colour)
            camera.update(auto_exposure_configured=True, auto_white_balance_configured=True)
            sequence = int(colour.getSequenceNum())
            # Review r4 F4: getFrame() returns exactly H*3/2 x W and silently drops excess bytes, so a frame with
            # stride or plane padding would be stored garbled. The delivered byte count is checked instead.
            # Review r4b N1: such a frame is not refused (it would repeat on every frame and cost the session). The
            # whole delivered buffer and the frame-buffer layout are saved as well, the capture is flagged
            # (isp_padded_frames) and the converter refuses it by name until the layout has been decoded on the PC.
            raw = np.asarray(colour.getData(), np.uint8).ravel()
            delivered = int(raw.size)
            expected = int(colour.getWidth()) * int(colour.getHeight()) * 3 // 2
            mono = colour_3a(depth)                # the depth frame's exposure state, if the device carries it
            arrays = frame_arrays(colour.getFrame(), isp_type, int(colour.getWidth()), int(colour.getHeight()),
                                  depth.getFrame(), window, rotation, stamps, sequence, camera)
            if delivered != expected:
                meta['isp_padded_frames'] += 1
                arrays['isp_raw'] = raw.copy()
                arrays['isp_fb'] = np.asarray(frame_buffer_layout(colour), np.int64)
                if meta['isp_padded_frames'] == 1:
                    log('WARNING ISP_PADDED: the colour frame carries %d bytes, %dx%d YUV 4:2:0 without padding has '
                        '%d. Recording CONTINUES and keeps the whole buffer (isp_raw) for decoding on the PC; '
                        'report this warning.' % (delivered, colour.getWidth(), colour.getHeight(), expected))
            if imu_present and int(arrays['accel_count']) == 0:
                # Review r2 N1: a silent IMU must not cost the field session. The frame is recorded without gravity
                # and counted; the PC conversion then needs --gravity-source nominal (the measured --pitch-deg).
                meta['frames_without_imu_samples'] += 1
                if meta['frames_without_imu_samples'] == 1:
                    log('WARNING IMU_NO_SAMPLES: the IMU (%s) delivered no accelerometer sample for frame %d. '
                        'Recording CONTINUES without gravity for such frames; the PC conversion must then use '
                        '--gravity-source nominal (the measured --pitch-deg), so measure the pitch carefully and '
                        'report this warning.' % (identity.get('imu'), index))
                else:
                    log('WARNING: frame %d has no accelerometer sample' % index)
            if previous_seq is not None and sequence > previous_seq + 1:
                meta['sequence_gaps'] += sequence - previous_seq - 1
            if (int(colour.getWidth()), int(colour.getHeight())) != (width, height):
                raise RecorderError('UNEXPECTED_ISP_LAYOUT', 'colour %dx%d, expected %dx%d'
                                    % (colour.getWidth(), colour.getHeight(), width, height))
            data = encode_frame(arrays)
            relative = 'frames/%06d.npz' % index
            with open(os.path.join(output, relative), 'xb') as stream:
                stream.write(data)
            meta['frames'].append({'file': relative, 'bytes': len(data), 'sha256': sha256_bytes(data),
                                   't_device_s': t, 't_host_monotonic_s': host_t,
                                   'depth_minus_colour_s': stamps['t_depth_device_s'] - t,
                                   'accel_count': int(arrays['accel_count']), 'isp_type': isp_type,
                                   'sequence_num': sequence,
                                   'valid_depth_share': float((arrays['depth_mm'] > 0).mean()),
                                   'isp_bytes_delivered': delivered,
                                   'depth_exposure_time_us': mono['exposure_time_us'],
                                   'depth_sensitivity_iso': mono['sensitivity_iso'],
                                   **{key: camera[key] for key in COLOUR_3A_KEYS}})
            if index == 0:
                log('first frame %.2f MB (%s); %d frames -> about %.0f MB; exposure %.0f us, ISO %d, %d K, '
                    'accelerometer samples %d; depth valid %.0f %%, depth-colour time %.1f ms'
                    % (len(data) / 1e6, isp_type, args.frames, len(data) * args.frames / 1e6,
                       camera['exposure_time_us'], camera['sensitivity_iso'], camera['colour_temperature_k'],
                       int(arrays['accel_count']), 100.0 * float((arrays['depth_mm'] > 0).mean()),
                       1e3 * (stamps['t_depth_device_s'] - t)))
            previous, previous_seq = t, sequence
        meta['colour_3a_summary'] = colour_3a_summary(meta['frames'])
        offsets = [1e3 * f['depth_minus_colour_s'] for f in meta['frames']]
        meta['depth_minus_colour_ms'] = {'min': min(offsets), 'max': max(offsets),
                                         'sync_threshold_ms': SYNC_THRESHOLD_MS}
        meta['status'] = 'COMPLETE'
        return meta
    except RecorderError as exc:
        if meta is not None:
            meta['status'] = 'INCOMPLETE'
            meta['stop_error'] = {'code': exc.code, 'message': str(exc)}
        raise
    except BaseException as exc:
        if meta is not None:
            meta['status'] = 'INCOMPLETE'
            meta['stop_error'] = {'code': 'OS_ERROR' if isinstance(exc, OSError) else 'UNEXPECTED',
                                  'message': '%s: %s' % (type(exc).__name__, exc)}
        raise
    finally:
        try:
            device.close()
        finally:
            if meta is not None:
                meta['ended_utc'] = _now_utc()
                meta['bytes_total'] = sum(f['bytes'] for f in meta['frames'])
                lines = ['%s  %s' % (f['sha256'], f['file']) for f in meta['frames']]
                imu_bytes = None
                if accel_all is not None:
                    samples = np.asarray(accel_all, np.float64).reshape(-1, 4)
                    buffer = io.BytesIO()
                    np.savez_compressed(buffer, accel=samples)
                    imu_bytes = buffer.getvalue()
                    meta['imu_all_samples'] = {
                        'file': 'imu.npz', 'sha256': sha256_bytes(imu_bytes), 'count': int(samples.shape[0]),
                        't_first_s': float(samples[0, 0]) if len(samples) else None,
                        't_last_s': float(samples[-1, 0]) if len(samples) else None,
                        'columns': 'device time s, x, y, z m/s^2 (raw IMU axes), warm-up included'}
                    lines.append('%s  imu.npz' % meta['imu_all_samples']['sha256'])
                # capture.json first (review r4b N2): a disk error on imu.npz must not lose the status record
                _write_json(os.path.join(output, 'capture.json'), meta)
                if imu_bytes is not None:
                    with open(os.path.join(output, 'imu.npz'), 'xb') as stream:
                        stream.write(imu_bytes)
                lines.append('%s  capture.json' % sha256_file(os.path.join(output, 'capture.json')))
                with open(os.path.join(output, 'MANIFEST.sha256'), 'x') as stream:
                    stream.write('\n'.join(lines) + '\n')
                try:
                    os.sync()                      # battery-powered host: the capture is on disk before the next
                except (AttributeError, OSError):
                    pass


def parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--probe', action='store_true', help='print the device / calibration readout as JSON')
    mode.add_argument('--record', action='store_true', help='record still frames into a new directory')
    ap.add_argument('--probe-imu-s', type=float, default=1.0, help='probe: seconds of accelerometer to average')
    ap.add_argument('--output', help='record: NEW directory; probe: file for the JSON (overwritten)')
    ap.add_argument('--scene', help='record: objects, tape distances, sky, sun or shade, projector value')
    ap.add_argument('--camera-height-m', type=float, help='record: tape height of the RGB lens centre [m]')
    ap.add_argument('--camera-height-uncertainty-m', type=float, help='record: tape reading uncertainty [m]')
    ap.add_argument('--pitch-deg', type=float, help='record (required): measured pitch [deg], positive = nose down')
    ap.add_argument('--frames', type=int, default=20, help='record: 1..%d frames (default 20)' % MAX_FRAMES)
    ap.add_argument('--projector', type=float, default=0.0, help='record: IR dot projector 0..1 (default 0 = off)')
    ap.add_argument('--stereo-mode', choices=('extended', 'subpixel'), default='extended')
    ap.add_argument('--warmup-s', type=float, default=3.0,
                    help='record: seconds of frames discarded first, counted from the first received frame')
    ap.add_argument('--allow-usb2', action='store_true', help='record even below USB3 SUPER speed')
    return ap


def main(argv=None, dai=None):
    ap = parser()
    argv = sys.argv[1:] if argv is None else list(argv)
    args = ap.parse_args(argv)
    args.argv = argv
    try:
        if args.record:                            # review r4 F1: every argument refusal comes BEFORE the import
            missing = [k for k in ('output', 'scene', 'camera_height_m', 'camera_height_uncertainty_m', 'pitch_deg')
                       if getattr(args, k) is None]
            if missing:
                raise RecorderError('BAD_ARGUMENT', '--record needs %s' % ', '.join('--' + k.replace('_', '-')
                                                                                   for k in missing))
            check_record_args(args)
        if dai is None:
            try:
                import depthai as dai
            except ImportError as exc:
                raise RecorderError('UNSUPPORTED_DEPTHAI_VERSION', 'depthai is not importable (%s); %s'
                                    % (exc, INSTALL_HINT)) from None
        if args.probe:
            text = json.dumps(_jsonable(probe(dai, args.probe_imu_s)), indent=1, sort_keys=True, allow_nan=False)
            if args.output:                        # review r4 F10: the SDK logs to stdout; keep the JSON apart
                with open(args.output, 'w') as stream:
                    stream.write(text + '\n')
                _say('probe written to %s' % args.output)
            else:
                print(text)
            return 0
        meta = record(args, dai)
        print(json.dumps({'status': meta['status'], 'frames': len(meta['frames']), 'output': args.output,
                          'bytes_total': sum(f['bytes'] for f in meta['frames']),
                          'frames_without_imu_samples': meta.get('frames_without_imu_samples', 0),
                          'sequence_gaps': meta.get('sequence_gaps', 0),
                          'depth_minus_colour_ms': meta.get('depth_minus_colour_ms'),
                          'imu_samples_total': (meta.get('imu_all_samples') or {}).get('count', 0),
                          'isp_padded_frames': meta.get('isp_padded_frames', 0),
                          'vehicle_commands_sent': False}, sort_keys=True))
        return 0
    except RecorderError as exc:
        print('oakd_field_recorder refused: %s' % exc, file=sys.stderr)
        return 2
    except OSError as exc:                         # e.g. ENOSPC: capture.json already says INCOMPLETE / OS_ERROR
        print('oakd_field_recorder refused: OS_ERROR: %s: %s' % (type(exc).__name__, exc), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('oakd_field_recorder refused: INTERRUPTED: stopped by Ctrl+C; a folder that was already created is '
              'INCOMPLETE and stays, record again into a NEW folder', file=sys.stderr)
        return 130
    except Exception as exc:                       # preflight r4: an SDK error is named, never a bare traceback
        print('oakd_field_recorder refused: UNEXPECTED: %s: %s; %s' % (type(exc).__name__, exc, UNEXPECTED_HINT),
              file=sys.stderr)
        return 2


def _leave(code):
    """Flush and leave without interpreter teardown (review r4 F1: the SDK's loader thread aborts a fast exit)."""
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == '__main__':
    _leave(main())
