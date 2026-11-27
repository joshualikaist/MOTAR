"""tools/oakd_field_recorder.py with a fake DepthAI v2 SDK only; no real Device, no hardware, no vehicle.

Every refusal is paired with a positive control that records or builds with a non-zero count. The module is loaded
under a private name and not left in sys.modules; nothing writes os.environ.
"""
from datetime import timedelta
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import re
import shlex
import sys
import tempfile
import types
import unittest
from unittest import mock
import warnings

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location('oakd_field_recorder_tested', ROOT / 'tools/oakd_field_recorder.py')
R = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(R)


class Enum:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return 'Enum(%s)' % self.name


SOCKETS = {name: Enum(name) for name in ('CAM_A', 'CAM_B', 'CAM_C')}


class Port:
    def __init__(self):
        self.links = []

    def link(self, target):
        self.links.append(target)


class Node:
    def __init__(self, kind):
        self.kind, self.calls = kind, []
        self.initialControl = self
        self.out, self.isp, self.video, self.depth = Port(), Port(), Port(), Port()
        self.left, self.right, self.input = Port(), Port(), Port()
        self.inputs = {'rgb': Port(), 'depth': Port()}

    def __getattr__(self, name):
        if name == 'setDefaultProfilePreset':      # DepthAI 2.29 warns here on every call (review r4 F3)
            def preset(*args):
                warnings.warn('HIGH_DENSITY is deprecated, will be removed in future releases', DeprecationWarning)
                self.calls.append((name, args))
            return preset
        if name.startswith(('set', 'enable')):
            return lambda *args: self.calls.append((name, args))
        raise AttributeError(name)

    def called(self, name):
        return [args for n, args in self.calls if n == name]


class Pipeline:
    def __init__(self):
        self.created = []

    def create(self, kind):
        node = Node(kind)
        self.created.append(node)
        return node

    def nodes(self, kind):
        return [n for n in self.created if n.kind == kind or getattr(n.kind, 'name', None) == kind]


class Calibration:
    def __init__(self, extrinsics=True):
        self.extrinsics = extrinsics

    def getCameraIntrinsics(self, socket, width=-1, height=-1):
        w, h = (1280, 800) if width == -1 else (width, height)
        f = 0.47 * w
        return [[f, 0.0, (w - 1) / 2.0], [0.0, f, (h - 1) / 2.0], [0.0, 0.0, 1.0]]

    def getDistortionModel(self, socket):
        return Enum('Perspective')

    def getDistortionCoefficients(self, socket):
        return [-0.1, 0.02] + [0.0] * 12

    def getFov(self, socket, use_spec=True):
        return 127.0

    def getLensPosition(self, socket):
        return 131

    def getImuToCameraExtrinsics(self, socket, use_spec=False):
        if not self.extrinsics:
            raise RuntimeError('There is no IMU extrinsics data available')
        return [[0.0, 1.0, 0.0, 1.0], [0.0, 0.0, -1.0, 0.5], [-1.0, 0.0, 0.0, 0.0], [0, 0, 0, 1]]

    def getBaselineDistance(self, a, b, use_spec=True):
        return 7.5

    def getEepromData(self):
        return types.SimpleNamespace(productName='OAK-D-PRO-W', boardName='DM9098', boardRev='R6M2E6',
                                     boardConf='', batchName='', deviceName='', version=7)

    def eepromToJson(self):
        return {'productName': 'OAK-D-PRO-W', 'cameraData': [], 'fixture': True}


class ColourMsg:
    def __init__(self, t, seq, width, height, depth_shape=None, padding=0):
        self.t, self.seq, self.width, self.height = t, seq, width, height
        self.depth_shape, self.padding = depth_shape, padding

    def getData(self):                             # the delivered bytes; getFrame() hides excess data (r4 F4)
        return np.full(self.width * self.height * 3 // 2 + self.padding, 7, np.uint8)

    def getRaw(self):
        return types.SimpleNamespace(fb=types.SimpleNamespace(stride=self.width + 16, p1Offset=0, p2Offset=1,
                                                              p3Offset=2, width=self.width, height=self.height,
                                                              bytesPP=1))

    def getExposureTime(self):
        return timedelta(microseconds=800 + 10 * self.seq)

    def getSensitivity(self):
        return 100 + self.seq

    def getColorTemperature(self):
        return 5200 + self.seq

    def getLensPosition(self):
        return 0

    def getLensPositionRaw(self):
        return 0.0

    def getTimestampDevice(self):
        return timedelta(seconds=self.t)

    def getTimestamp(self):
        return timedelta(seconds=self.t + 100.0)

    def getType(self):
        return Enum('YUV420p')

    def getWidth(self):
        return self.width

    def getHeight(self):
        return self.height

    def getSequenceNum(self):
        return self.seq

    def getFrame(self):
        rng = np.random.default_rng(self.seq)
        return rng.integers(0, 256, (self.height * 3 // 2, self.width), dtype=np.uint8)


class DepthMsg(ColourMsg):
    def getFrame(self):
        if self.depth_shape is not None:
            return np.full(self.depth_shape, 2000, np.uint16)
        depth = np.full((self.height, self.width), 2000, np.uint16)
        depth[: self.height // 3] = 0
        return depth


class Clock:
    def __init__(self):
        self.t = 50.0

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class FrameQueue:
    def __init__(self, stamps, size, clock, depth_shape=None, padding=0, depth_offsets=(0.001,)):
        self.groups = [{'rgb': ColourMsg(t, i, *size, padding=padding),
                        'depth': DepthMsg(t + depth_offsets[i % len(depth_offsets)], i, *size, depth_shape)}
                       for i, t in enumerate(stamps)]
        self.clock = clock

    def tryGet(self):
        self.clock.t += 0.1
        return self.groups.pop(0) if self.groups else None


class ImuQueue:
    """Accelerometer samples every 0.01 s up to the newest frame time; camera level, IMU axes rotated."""
    def __init__(self, frames, up_imu, silent=False):
        self.frames, self.up_imu, self.t, self.silent = frames, np.asarray(up_imu), 0.0, silent

    def tryGetAll(self):
        if self.silent or (not self.frames.groups and self.t > 10):
            return []
        end = self.frames.groups[0]['rgb'].t if self.frames.groups else self.t + 0.5
        packets = []
        while self.t < end + 0.05:
            self.t += 0.01
            a = 9.81 * self.up_imu
            packets.append(types.SimpleNamespace(
                acceleroMeter=types.SimpleNamespace(x=a[0], y=a[1], z=a[2], getTimestampDevice=lambda t=self.t: timedelta(seconds=t)),
                rotationVector=types.SimpleNamespace(i=0.0, j=0.0, k=0.0, real=1.0, rotationVectorAccuracy=0.1,
                                                     getTimestampDevice=lambda t=self.t: timedelta(seconds=t))))
        return [types.SimpleNamespace(packets=packets)]


class Device:
    def __init__(self, sensors, usb='SUPER', stamps=(1.0, 1.1, 1.2), ir=True, imu='BNO086', clock=None,
                 extrinsics=True, dot_ok=True, flood_ok=True, imu_silent=False, depth_shape=None, padding=0,
                 depth_offsets=(0.001,)):
        self.sensors, self.usb, self.stamps, self.ir, self.imu = sensors, usb, stamps, ir, imu
        self.dot_ok, self.flood_ok, self.imu_silent, self.depth_shape = dot_ok, flood_ok, imu_silent, depth_shape
        self.padding, self.depth_offsets = padding, depth_offsets
        self.queues, self.queue_sizes = [], {}
        self.clock = clock or Clock()
        self.calibration = Calibration(extrinsics)
        self.ir_calls, self.started, self.closed = [], [], False

    def getCameraSensorNames(self):
        return {SOCKETS[k]: v for k, v in self.sensors.items()}

    def getUsbSpeed(self):
        return Enum(self.usb)

    def readCalibration(self):
        return self.calibration

    def getConnectedCameraFeatures(self):
        return [types.SimpleNamespace(socket=SOCKETS[k], sensorName=v, width=1280, height=800, hasAutofocus=False)
                for k, v in self.sensors.items()]

    def getProductName(self):
        return 'OAK-D-PRO-W'

    def getDeviceName(self):
        return 'OAK-D-PRO-W'

    def getMxId(self):
        return 'FAKE-MXID-0001'

    def getConnectedIMU(self):
        return self.imu

    def getIrDrivers(self):
        return [('LM3644', 2, 0x63)] if self.ir else []

    def startPipeline(self, pipeline):
        self.started.append(pipeline)

    def setIrLaserDotProjectorIntensity(self, value):
        self.ir_calls.append(('dot', value))
        return self.dot_ok

    def setIrFloodLightIntensity(self, value):
        self.ir_calls.append(('flood', value))
        return self.flood_ok

    def getOutputQueue(self, name, maxSize, blocking):
        pipeline = self.started[-1]
        colour = pipeline.nodes('ColorCamera')[0]
        size = colour.called('setResolution')[0][0]
        width, height = {'THE_800_P': (1280, 800), 'THE_12_MP': (4056, 3040)}[size.name]
        for num, den in colour.called('setIspScale'):
            width, height = (width * num - 1) // den + 1, (height * num - 1) // den + 1
        self.queues.append(name)
        self.queue_sizes[name] = maxSize
        if name == 'rgbd':
            self.frames = FrameQueue(self.stamps, (width, height), self.clock, self.depth_shape, self.padding,
                                     self.depth_offsets)
            return self.frames
        return ImuQueue(self.frames, [0.0, 0.0, 1.0], silent=self.imu_silent)

    def close(self):
        self.closed = True


def fake_dai(device=None, connected=()):
    sdk = types.SimpleNamespace(__version__=R.SDK_VERSION, Pipeline=Pipeline,
                                node=types.SimpleNamespace(ColorCamera='ColorCamera', MonoCamera='MonoCamera',
                                                           Sync='Sync', XLinkOut='XLinkOut', IMU='IMU',
                                                           StereoDepth=types.SimpleNamespace(
                                                               name='StereoDepth', PresetMode=types.SimpleNamespace(HIGH_DENSITY='HD'))),
                                CameraBoardSocket=types.SimpleNamespace(**SOCKETS),
                                ColorCameraProperties=types.SimpleNamespace(SensorResolution=types.SimpleNamespace(
                                    THE_800_P=Enum('THE_800_P'), THE_12_MP=Enum('THE_12_MP'),
                                    THE_1080_P=Enum('THE_1080_P'))),
                                MonoCameraProperties=types.SimpleNamespace(SensorResolution=types.SimpleNamespace(
                                    THE_800_P=Enum('MONO_800_P'))),
                                IMUSensor=types.SimpleNamespace(ACCELEROMETER_RAW='ACCEL_RAW',
                                                                ROTATION_VECTOR='ROTATION_VECTOR'),
                                CameraControl=types.SimpleNamespace(AutoWhiteBalanceMode=types.SimpleNamespace(
                                    AUTO=Enum('AWB_AUTO'))))
    if device is None:
        sdk.Device = mock.Mock(side_effect=RuntimeError('No available devices'))
    else:
        sdk.Device = mock.Mock(return_value=device)
    sdk.Device.getAllConnectedDevices = mock.Mock(return_value=list(connected))
    return sdk


PRO_W = {'CAM_A': 'OV9782', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'}
PRO = {'CAM_A': 'IMX378', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'}


class RecorderTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.log = []

    def args(self, name='cap', **overrides):
        values = dict(output=os.path.join(self.tmp.name, name), scene='fixture scene: open grass, sun',
                      camera_height_m=1.0, camera_height_uncertainty_m=0.02, pitch_deg=0.0, frames=3,
                      projector=0.0, stereo_mode='extended', warmup_s=0.0, allow_usb2=False, argv=['--record'])
        values.update(overrides)
        return types.SimpleNamespace(**values)

    def record(self, device, **overrides):
        args = self.args(**overrides)
        return R.record(args, fake_dai(device), clock=device.clock, sleep=device.clock.sleep,
                        log=self.log.append), args

    # ---- pipeline selection
    def test_pipeline_per_colour_sensor_with_unknown_refused(self):
        dai = fake_dai()
        pro_w = R.select_profile(PRO_W)
        pipeline, description = R.build_pipeline(dai, pro_w, 'extended', lens_position=131)
        colour = pipeline.nodes('ColorCamera')[0]
        stereo = pipeline.nodes('StereoDepth')[0]
        self.assertEqual(colour.called('setResolution')[0][0].name, 'THE_800_P')
        self.assertEqual(colour.called('setIspScale'), [])
        self.assertEqual(colour.called('setManualFocus'), [])            # fixed-focus OV9782
        self.assertEqual(stereo.called('setOutputSize'), [(1280, 800)])
        self.assertEqual(stereo.called('setDepthAlign'), [(SOCKETS['CAM_A'],)])
        self.assertEqual(stereo.called('setLeftRightCheck'), [(True,)])
        self.assertEqual(stereo.called('setExtendedDisparity'), [(True,)])
        self.assertEqual(description['colour']['size'], [1280, 800])
        self.assertEqual(len(colour.isp.links), 1)
        self.assertEqual(len(pipeline.nodes('IMU')[0].called('enableIMUSensor')), 1)
        pro = R.select_profile(PRO)
        pipeline, description = R.build_pipeline(dai, pro, 'subpixel', lens_position=131, rotation_vector=True)
        colour = pipeline.nodes('ColorCamera')[0]
        stereo = pipeline.nodes('StereoDepth')[0]
        self.assertEqual(colour.called('setResolution')[0][0].name, 'THE_12_MP')
        self.assertEqual(colour.called('setIspScale'), [(4, 13)])
        self.assertEqual(colour.called('setManualFocus'), [(131,)])
        self.assertEqual(stereo.called('setOutputSize'), [(1248, 936)])
        self.assertEqual(description['colour']['size'], [1248, 936])
        self.assertEqual(stereo.called('setSubpixel'), [(True,)])
        self.assertEqual(len(pipeline.nodes('IMU')[0].called('enableIMUSensor')), 2)
        self.assertEqual(description['imu']['sensors'], ['ACCELEROMETER_RAW', 'ROTATION_VECTOR'])
        cases = (({'CAM_A': 'IMX214', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'}, 'UNSUPPORTED_COLOUR_SENSOR'),
                 ({'CAM_B': 'OV9282', 'CAM_C': 'OV9282'}, 'NO_CAM_A'),
                 ({'CAM_A': 'OV9782', 'CAM_B': 'OV9282'}, 'NO_STEREO_PAIR'),
                 ({'CAM_A': 'OV9782', 'CAM_B': 'OV7251', 'CAM_C': 'OV7251'}, 'UNSUPPORTED_STEREO_SENSOR'))
        for sensors, code in cases:
            with self.assertRaises(R.RecorderError) as caught:
                R.select_profile(sensors)
            self.assertEqual(caught.exception.code, code)

    def test_unknown_sensor_refused_before_output_is_created(self):
        device = Device({'CAM_A': 'IMX214', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'})
        with self.assertRaises(R.RecorderError) as caught:
            self.record(device)
        self.assertEqual(caught.exception.code, 'UNSUPPORTED_COLOUR_SENSOR')
        self.assertFalse(os.path.exists(os.path.join(self.tmp.name, 'cap')))
        self.assertTrue(device.closed)
        self.assertEqual(device.started, [])
        meta, args = self.record(Device(PRO_W), name='control')        # positive control
        self.assertEqual(meta['status'], 'COMPLETE')
        self.assertEqual(len(meta['frames']), 3)

    # ---- USB
    def test_usb2_refused_unless_allowed(self):
        for speed in ('HIGH', 'FULL', 'UNKNOWN'):
            device = Device(PRO_W, usb=speed)
            with self.assertRaises(R.RecorderError) as caught:
                self.record(device, name='usb_' + speed)
            self.assertEqual(caught.exception.code, 'USB_SPEED')
            self.assertFalse(os.path.exists(os.path.join(self.tmp.name, 'usb_' + speed)))
        meta, _ = self.record(Device(PRO_W, usb='HIGH'), name='usb2_allowed', allow_usb2=True)
        self.assertEqual(meta['pipeline']['usb_speed'], 'HIGH')
        self.assertEqual(len(meta['frames']), 3)
        self.assertEqual(R.check_usb(Enum('SUPER'), False), 'SUPER')
        self.assertEqual(R.check_usb(Enum('SUPER_PLUS'), False), 'SUPER_PLUS')

    # ---- timestamps
    def test_non_increasing_device_timestamp_stops_with_named_error(self):
        device = Device(PRO_W, stamps=(1.0, 1.1, 1.1, 1.2))
        with self.assertRaises(R.RecorderError) as caught:
            self.record(device, frames=4)
        self.assertEqual(caught.exception.code, 'NON_INCREASING_DEVICE_TIMESTAMP')
        meta = json.loads(Path(self.tmp.name, 'cap', 'capture.json').read_text())
        self.assertEqual(meta['status'], 'INCOMPLETE')
        self.assertEqual(meta['stop_error']['code'], 'NON_INCREASING_DEVICE_TIMESTAMP')
        self.assertEqual(len(meta['frames']), 2)                        # frames before the regression are kept
        self.assertTrue(device.closed)
        meta, _ = self.record(Device(PRO_W, stamps=(1.0, 1.1, 1.2, 1.3)), name='control', frames=4)
        self.assertEqual(meta['status'], 'COMPLETE')
        times = [f['t_device_s'] for f in meta['frames']]
        self.assertEqual(len(times), 4)
        self.assertTrue(all(b > a for a, b in zip(times, times[1:])))
        guard = R.TimestampGuard()
        guard.check(1.0)
        with self.assertRaises(R.RecorderError):
            guard.check(float('nan'))

    # ---- output dir / frames
    def test_existing_output_and_frame_range_refused_before_device(self):
        existing = os.path.join(self.tmp.name, 'exists')
        os.mkdir(existing)
        dai = fake_dai(Device(PRO_W))
        with self.assertRaises(R.RecorderError) as caught:
            R.record(self.args('exists'), dai)
        self.assertEqual(caught.exception.code, 'OUTPUT_EXISTS')
        dai.Device.assert_not_called()
        self.assertEqual(os.listdir(existing), [])
        for frames in (0, 101, -1, True):
            with self.assertRaises(R.RecorderError) as caught:
                R.record(self.args('n%s' % frames, frames=frames), dai)
            self.assertEqual(caught.exception.code, 'FRAMES_OUT_OF_RANGE')
        dai.Device.assert_not_called()
        self.assertEqual([R.check_frames(n) for n in (1, 100)], [1, 100])
        for bad in (dict(camera_height_m=0.0), dict(camera_height_uncertainty_m=-1.0), dict(projector=1.5),
                    dict(pitch_deg=float('nan')), dict(pitch_deg=None), dict(scene='  ')):
            with self.assertRaises(R.RecorderError) as caught:
                R.check_record_args(self.args('bad', **bad))
            self.assertEqual(caught.exception.code, 'BAD_ARGUMENT')
        meta, _ = self.record(Device(PRO_W), name='new', frames=1)
        self.assertEqual(len(meta['frames']), 1)

    # ---- capture contents, bytes estimate, projector
    def test_capture_files_hashes_estimate_and_projector(self):
        device = Device(PRO_W)
        meta, args = self.record(device, projector=0.5)
        out = Path(args.output)
        on_disk = json.loads((out / 'capture.json').read_text())
        self.assertEqual(on_disk['format'], R.FORMAT)
        self.assertEqual(on_disk['status'], 'COMPLETE')
        self.assertEqual(on_disk['calibration_eeprom'], {'productName': 'OAK-D-PRO-W', 'cameraData': [],
                                                         'fixture': True})
        self.assertEqual(on_disk['pipeline']['projector'], {'ir_dot_projector_intensity': 0.5,
                                                            'ir_dot_projector_applied': True,
                                                            'ir_flood_intensity': 0.0, 'ir_flood_off_applied': True})
        self.assertEqual(device.ir_calls, [('flood', 0.0), ('dot', 0.5)])
        self.assertEqual(on_disk['device']['mx_id'], 'FAKE-MXID-0001')
        self.assertEqual(on_disk['calibration']['sockets']['CAM_A']['output_size'], [1280, 800])
        self.assertIsNotNone(on_disk['calibration']['imu_to_cam_a']['matrix_4x4'])
        self.assertAlmostEqual(on_disk['calibration']['stereo_min_z_hints']['baseline_m'], 0.075)
        manifest = (out / 'MANIFEST.sha256').read_text().splitlines()
        self.assertEqual(len(manifest), 5)                              # 3 frames, imu.npz, capture.json
        for line in manifest:
            digest, relative = line.split('  ')
            self.assertEqual(R.sha256_file(out / relative), digest)
        for entry in on_disk['frames']:
            with np.load(out / entry['file'], allow_pickle=False) as data:
                self.assertEqual(data['isp'].shape, (1200, 1280))
                self.assertEqual(str(data['isp_type']), 'YUV420p')
                self.assertEqual(data['depth_mm'].dtype, np.uint16)
                self.assertGreater(int(data['accel_count']), 0)
                np.testing.assert_allclose(data['accel_mean'], [0.0, 0.0, 9.81], atol=1e-9)
                self.assertTrue(np.isfinite(data['rotation_vector']).all())
                self.assertEqual(float(data['t_device_s']), entry['t_device_s'])
            self.assertEqual(entry['bytes'], (out / entry['file']).stat().st_size)
        self.assertGreater(on_disk['estimated_bytes_per_frame'], 0)
        self.assertLessEqual(on_disk['estimated_bytes_per_frame'], 2_500_000)
        self.assertTrue(any('MB/frame' in line for line in self.log))
        self.assertTrue(device.closed)
        # projector 0: the dot projector is never set, the flood LED is set to 0
        device = Device(PRO_W)
        self.record(device, name='dark')
        self.assertEqual(device.ir_calls, [('flood', 0.0)])
        # no IR driver: --projector > 0 refused (named), 0 accepted (positive control)
        with self.assertRaises(R.RecorderError) as caught:
            self.record(Device(PRO_W, ir=False), name='no_ir', projector=1.0)
        self.assertEqual(caught.exception.code, 'NO_IR_PROJECTOR')
        meta, _ = self.record(Device(PRO_W, ir=False), name='no_ir_dark')
        self.assertEqual(len(meta['frames']), 3)

    def test_bytes_per_frame_estimate_is_reported_and_bounded(self):
        sizes = {size: R.estimate_bytes_per_frame(*size) for size in ((1280, 800), (1248, 936))}
        for size, value in sizes.items():
            raw = size[0] * size[1] * 3.5                             # YUV420 + uint16 depth, uncompressed
            self.assertGreater(value, 0.2 * raw)
            self.assertLess(value, 2_500_000)
        self.assertGreater(sizes[(1248, 936)], sizes[(1280, 800)])    # the IMX378 profile has more pixels
        with mock.patch.object(R.shutil, 'disk_usage', return_value=types.SimpleNamespace(free=1 << 20)):
            with self.assertRaises(R.RecorderError) as caught:
                R.check_disk(os.path.join(self.tmp.name, 'x'), 20, sizes[(1280, 800)])
            self.assertEqual(caught.exception.code, 'INSUFFICIENT_DISK')
        free, need = R.check_disk(os.path.join(self.tmp.name, 'x'), 20, sizes[(1280, 800)])
        self.assertGreater(free, need)

    def test_frame_layout_checks(self):
        yuv = np.zeros((1200, 1280), np.uint8)
        self.assertEqual(R.isp_layout(yuv.ravel(), 'NV12', 1280, 800).shape, (1200, 1280))
        for bad, kind in ((np.zeros((1201, 1280), np.uint8), 'YUV420p'), (yuv, 'RGB888i'),
                          (yuv.astype(np.uint16), 'YUV420p')):
            with self.assertRaises(R.RecorderError) as caught:
                R.isp_layout(bad, kind, 1280, 800)
            self.assertEqual(caught.exception.code, 'UNEXPECTED_ISP_LAYOUT')
        with self.assertRaises(R.RecorderError) as caught:
            R.depth_layout(np.zeros((800, 1280), np.float32), 1280, 800)
        self.assertEqual(caught.exception.code, 'UNEXPECTED_DEPTH_LAYOUT')
        window = R.accel_window([(0.1, 0, 0, 1), (0.6, 0, 0, 2), (0.9, 0, 0, 3), (1.2, 0, 0, 4)], 1.0, 0.6)
        self.assertEqual(window[:, 3].tolist(), [3.0])
        window = R.accel_window([(0.1, 0, 0, 1), (0.6, 0, 0, 2), (0.9, 0, 0, 3)], 1.0, None)
        self.assertEqual(window[:, 3].tolist(), [2.0, 3.0])

    # ---- device open / probe / main
    def test_no_device_and_busy_device_are_named(self):
        with self.assertRaises(R.RecorderError) as caught:
            R.open_device(fake_dai())
        self.assertEqual(caught.exception.code, 'NO_DEVICE')
        self.assertIn('VIO', str(caught.exception))
        info = lambda state, status='X_LINK_SUCCESS': types.SimpleNamespace(state=Enum(state), status=Enum(status))
        cases = ((['X_LINK_BOOTED'], 'X_LINK_SUCCESS', 'Failed to connect', 'DEVICE_BUSY'),
                 (['X_LINK_UNBOOTED'], 'X_LINK_DEVICE_ALREADY_IN_USE', 'Failed to connect', 'DEVICE_BUSY'),
                 (['X_LINK_UNBOOTED'], 'X_LINK_SUCCESS', 'X_LINK_DEVICE_ALREADY_IN_USE', 'DEVICE_BUSY'),
                 (['X_LINK_UNBOOTED'], 'X_LINK_INSUFFICIENT_PERMISSIONS', 'Failed to connect', 'DEVICE_PERMISSION'),
                 (['X_LINK_UNBOOTED'], 'X_LINK_SUCCESS', 'Insufficient permissions', 'DEVICE_PERMISSION'),
                 (['X_LINK_UNBOOTED'], 'X_LINK_SUCCESS', 'Failed to boot', 'DEVICE_OPEN_FAILED'))
        for states, status, text, code in cases:
            dai = fake_dai(connected=[info(state, status) for state in states])
            dai.Device.side_effect = RuntimeError(text)
            with self.assertRaises(R.RecorderError) as caught:
                R.open_device(dai)
            self.assertEqual(caught.exception.code, code, (states, status, text))
            message = str(caught.exception)
            self.assertIn('udev', message)                              # every listed-device message names both
            self.assertIn('VIO', message)
        device = Device(PRO_W)
        self.assertIs(R.open_device(fake_dai(device)), device)
        stderr = io.StringIO()
        with mock.patch.object(R.sys, 'stderr', stderr):
            code = R.main(['--record', '--output', os.path.join(self.tmp.name, 'nodev'), '--scene', 's',
                           '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02', '--pitch-deg', '0'],
                          dai=fake_dai())
        self.assertEqual(code, 2)
        self.assertIn('NO_DEVICE', stderr.getvalue())
        self.assertFalse(os.path.exists(os.path.join(self.tmp.name, 'nodev')))
        with mock.patch.object(R.sys, 'stderr', io.StringIO()):
            self.assertEqual(R.main(['--probe'], dai=fake_dai()), 2)
            wrong = fake_dai(Device(PRO_W))
            wrong.__version__ = '3.0.0'
            self.assertEqual(R.main(['--probe'], dai=wrong), 2)

    def test_probe_reports_device_calibration_and_plan(self):
        device = Device(PRO_W)
        report = R.probe(fake_dai(device), imu_seconds=0)
        text = json.dumps(R._jsonable(report), allow_nan=False)
        self.assertIn('OV9782', text)
        self.assertEqual(report['recording_plan']['profile'], 'OV9782')
        self.assertEqual(report['usb_check'], 'SUPER')
        self.assertEqual(sorted(report['sensors']), ['CAM_A', 'CAM_B', 'CAM_C'])
        self.assertEqual(report['device']['eeprom']['productName'], 'OAK-D-PRO-W')
        self.assertEqual(len(report['calibration']['sockets']['CAM_A']['distortion_coefficients']), 14)
        self.assertGreater(report['calibration']['stereo_min_z_hints']['min_z_m_standard_95px'], 0)
        self.assertTrue(device.closed)
        report = R.probe(fake_dai(Device({'CAM_A': 'IMX214', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'},
                                         extrinsics=False)), imu_seconds=0)
        self.assertIn('UNSUPPORTED_COLOUR_SENSOR', report['recording_plan']['refusal'])
        self.assertIsNone(report['calibration']['imu_to_cam_a']['matrix_4x4'])
        self.assertIn('no IMU extrinsics', report['calibration']['imu_to_cam_a']['error'])

    def test_source_is_single_file_without_vehicle_or_cv2_imports(self):
        source = (ROOT / 'tools/oakd_field_recorder.py').read_text()
        imports = [line.split()[1] for line in source.splitlines() if line.startswith(('import ', 'from '))]
        self.assertTrue(imports)
        allowed = {'argparse', 'datetime', 'hashlib', 'io', 'json', 'math', 'os', 'platform', 'shutil', 'sys',
                   'time', 'warnings', 'numpy'}
        self.assertEqual(set(imports) - allowed, set())
        for word in ('rclpy', 'mavsdk', 'pymavlink', 'cv2', 'px4'):
            self.assertNotIn('import %s' % word, source)
        self.assertNotIn('oakd_field_recorder_tested', __import__('sys').modules)

    # ---- repair r1
    def test_colour_3a_state_saved_per_frame_and_summarised(self):
        meta, args = self.record(Device(PRO_W))
        colour = meta['pipeline']['colour']
        self.assertTrue(colour['auto_exposure'].startswith('ON'))
        self.assertTrue(colour['auto_white_balance'].startswith('AUTO'))
        self.assertEqual(colour['focus'], 'fixed focus (no lens control)')
        exposures = []
        for entry in meta['frames']:
            with np.load(Path(args.output) / entry['file'], allow_pickle=False) as data:
                self.assertAlmostEqual(float(data['exposure_time_us']), entry['exposure_time_us'])
                self.assertEqual(int(data['sensitivity_iso']), entry['sensitivity_iso'])
                self.assertEqual(int(data['colour_temperature_k']), entry['colour_temperature_k'])
                self.assertTrue(bool(data['auto_exposure_configured']))
                self.assertTrue(bool(data['auto_white_balance_configured']))
                exposures.append(float(data['exposure_time_us']))
        self.assertEqual(len(exposures), 3)
        self.assertGreater(min(exposures), 0)
        summary = json.loads((Path(args.output) / 'capture.json').read_text())['colour_3a_summary']
        self.assertEqual(summary['exposure_time_us'], {'min': 800.0, 'max': 820.0, 'frames': 3})
        self.assertEqual(summary['sensitivity_iso'], {'min': 100, 'max': 102, 'frames': 3})
        self.assertEqual(summary['colour_temperature_k']['max'], 5202)
        self.assertEqual([f['sequence_num'] for f in meta['frames']], [0, 1, 2])
        self.assertEqual(meta['sequence_gaps'], 0)
        pipeline, _ = R.build_pipeline(fake_dai(), R.select_profile(PRO), 'extended', lens_position=131)
        colour_node = pipeline.nodes('ColorCamera')[0]
        self.assertEqual(len(colour_node.called('setAutoExposureEnable')), 1)
        self.assertEqual(colour_node.called('setAutoWhiteBalanceMode')[0][0].name, 'AWB_AUTO')
        # a message without the getters is recorded as -1 (not read), never as a value
        self.assertEqual(R.colour_3a(object()), R.NOT_READ)
        self.assertEqual(R.colour_3a_summary([{'exposure_time_us': -1.0}])['exposure_time_us'], 'NOT_RECORDED')

    def test_projector_and_flood_setter_results(self):
        device = Device(PRO_W, dot_ok=False)
        with self.assertRaises(R.RecorderError) as caught:
            self.record(device, name='dot_fail', projector=0.5)
        self.assertEqual(caught.exception.code, 'PROJECTOR_NOT_APPLIED')
        meta = json.loads(Path(self.tmp.name, 'dot_fail', 'capture.json').read_text())
        self.assertEqual(meta['status'], 'INCOMPLETE')
        self.assertIs(meta['pipeline']['projector']['ir_dot_projector_applied'], False)
        self.assertEqual(meta['frames'], [])
        meta, _ = self.record(Device(PRO_W, dot_ok=False), name='dot_fail_dark', projector=0.0)   # control
        self.assertEqual(len(meta['frames']), 3)
        self.assertEqual(meta['pipeline']['projector']['ir_dot_projector_applied'], 'NOT_SET (boot default off)')
        meta, _ = self.record(Device(PRO_W, flood_ok=False), name='flood_false')
        self.assertIs(meta['pipeline']['projector']['ir_flood_off_applied'], False)
        self.assertEqual(len(meta['frames']), 3)
        self.assertTrue(any('setIrFloodLightIntensity' in line for line in self.log))

    def test_imu_absent_silent_and_present(self):
        # Review r2 N1: a reported but silent IMU is a loud warning, never a refusal: the session keeps its frames.
        device = Device(PRO_W, imu_silent=True)
        meta, _ = self.record(device, name='imu_silent')
        self.assertEqual(meta['status'], 'COMPLETE')
        self.assertTrue(meta['pipeline']['imu']['present'])
        self.assertEqual([f['accel_count'] for f in meta['frames']], [0, 0, 0])
        self.assertEqual(meta['frames_without_imu_samples'], 3)
        warnings = [line for line in self.log if 'IMU_NO_SAMPLES' in line]
        self.assertEqual(len(warnings), 1)
        self.assertIn('--gravity-source nominal', warnings[0])
        self.assertIn('CONTINUES', warnings[0])
        self.assertEqual(sum('has no accelerometer sample' in line for line in self.log), 2)
        on_disk = json.loads(Path(self.tmp.name, 'imu_silent', 'capture.json').read_text())
        self.assertEqual((on_disk['status'], on_disk['frames_without_imu_samples']), ('COMPLETE', 3))
        self.log.clear()
        device = Device(PRO_W, imu='NONE')
        meta, args = self.record(device, name='imu_none')
        self.assertEqual(meta['status'], 'COMPLETE')
        self.assertFalse(meta['pipeline']['imu']['present'])
        self.assertEqual(device.queues, ['rgbd'])
        self.assertEqual(device.started[0].nodes('IMU'), [])
        self.assertEqual([f['accel_count'] for f in meta['frames']], [0, 0, 0])
        meta, _ = self.record(Device(PRO_W), name='imu_ok')                                    # control
        self.assertTrue(all(f['accel_count'] > 0 for f in meta['frames']))
        self.assertEqual(meta['frames_without_imu_samples'], 0)
        self.assertFalse(any('WARNING' in line and 'accelerometer' in line for line in self.log))

    def test_final_status_line_reports_frames_without_imu_samples(self):
        for name, silent, expected in (('status_silent', True, 3), ('status_ok', False, 0)):
            stdout = io.StringIO()
            device = Device(PRO_W, imu_silent=silent)
            with mock.patch.object(R.sys, 'stdout', stdout):
                code = R.main(['--record', '--output', os.path.join(self.tmp.name, name), '--scene', 's',
                               '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02', '--pitch-deg', '0',
                               '--frames', '3', '--warmup-s', '0'], dai=fake_dai(device))
            self.assertEqual(code, 0)
            status = json.loads(stdout.getvalue().strip().splitlines()[-1])
            self.assertEqual((status['status'], status['frames'], status['frames_without_imu_samples']),
                             ('COMPLETE', 3, expected))
            self.assertEqual(status['sequence_gaps'], 0)
            self.assertAlmostEqual(status['depth_minus_colour_ms']['max'], 1.0, places=6)   # fixture: depth 1 ms later
            self.assertEqual(status['depth_minus_colour_ms']['sync_threshold_ms'], 60)
            self.assertEqual(status['imu_samples_total'] > 0, not silent)

    def test_sequence_gaps_are_counted(self):
        class GapDevice(Device):
            def getOutputQueue(self, name, maxSize, blocking):
                queue = Device.getOutputQueue(self, name, maxSize, blocking)
                if name == 'rgbd':
                    for message in (queue.groups[2]['rgb'], queue.groups[2]['depth']):
                        message.seq += 3                       # the device dropped three frames before the third
                return queue
        meta, _ = self.record(GapDevice(PRO_W), name='gaps')
        self.assertEqual([f['sequence_num'] for f in meta['frames']], [0, 1, 5])
        self.assertEqual(meta['sequence_gaps'], 3)
        meta, _ = self.record(Device(PRO_W), name='no_gaps')                                   # control
        self.assertEqual(meta['sequence_gaps'], 0)

    def test_install_hint_never_points_at_the_drones_python(self):
        # Review r2 N4: a missing or wrong depthai must send the operator to the isolated install, not to pip install.
        wrong = types.SimpleNamespace(__version__='2.24.0.0')
        with self.assertRaises(R.RecorderError) as caught:
            R.check_sdk(wrong)
        self.assertEqual(caught.exception.code, 'UNSUPPORTED_DEPTHAI_VERSION')
        for text in ('oakrec', 'do NOT pip install into the drone',
                     '--no-cache-dir --target ~/oakd_rec_pkgs depthai==2.29.0.0 numpy==1.24.4'):
            self.assertIn(text, str(caught.exception))
        self.assertNotIn('; pip install depthai', str(caught.exception))
        R.check_sdk(types.SimpleNamespace(__version__=R.SDK_VERSION))                          # control: accepted
        stderr = io.StringIO()
        with mock.patch.dict(sys.modules, {'depthai': None}), mock.patch.object(R.sys, 'stderr', stderr):
            code = R.main(['--probe'])                                    # import depthai fails inside main
        self.assertEqual(code, 2)
        self.assertIn('UNSUPPORTED_DEPTHAI_VERSION', stderr.getvalue())
        self.assertIn('--target ~/oakd_rec_pkgs', stderr.getvalue())
        self.assertNotIn('; pip install depthai', stderr.getvalue())

    def test_record_refuses_depth_not_aligned_to_the_colour_size(self):
        with self.assertRaises(R.RecorderError) as caught:
            self.record(Device(PRO_W, depth_shape=(400, 640)), name='bad_depth')
        self.assertEqual(caught.exception.code, 'UNEXPECTED_DEPTH_LAYOUT')
        meta = json.loads(Path(self.tmp.name, 'bad_depth', 'capture.json').read_text())
        self.assertEqual(meta['frames'], [])
        meta, _ = self.record(Device(PRO_W), name='good_depth')                               # control
        self.assertEqual(len(meta['frames']), 3)

    def test_pitch_is_required_and_os_errors_are_named(self):
        stderr = io.StringIO()
        with mock.patch.object(R.sys, 'stderr', stderr):
            code = R.main(['--record', '--output', os.path.join(self.tmp.name, 'nopitch'), '--scene', 's',
                           '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02'],
                          dai=fake_dai(Device(PRO_W)))
        self.assertEqual(code, 2)
        self.assertIn('BAD_ARGUMENT', stderr.getvalue())
        self.assertIn('pitch-deg', stderr.getvalue())
        device = Device(PRO_W)
        stderr, stdout = io.StringIO(), io.StringIO()
        real_encode = R.encode_frame
        calls = []

        def full_disk(arrays):                     # the size estimate encodes first; the first frame write fails
            calls.append(1)
            if len(calls) > 1:
                raise OSError(28, 'No space left on device')
            return real_encode(arrays)
        with mock.patch.object(R, 'encode_frame', side_effect=full_disk), \
                mock.patch.object(R.sys, 'stderr', stderr), mock.patch.object(R.sys, 'stdout', stdout):
            code = R.main(['--record', '--output', os.path.join(self.tmp.name, 'enospc'), '--scene', 's',
                           '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02', '--pitch-deg', '0',
                           '--warmup-s', '0'], dai=fake_dai(device))
        self.assertEqual(code, 2)
        self.assertIn('OS_ERROR', stderr.getvalue())
        meta = json.loads(Path(self.tmp.name, 'enospc', 'capture.json').read_text())
        self.assertEqual(meta['stop_error']['code'], 'OS_ERROR')
        with mock.patch.object(R.sys, 'stdout', io.StringIO()):
            code = R.main(['--record', '--output', os.path.join(self.tmp.name, 'ok_cli'), '--scene', 's',
                           '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02', '--pitch-deg', '-10',
                           '--warmup-s', '0', '--frames', '3'], dai=fake_dai(Device(PRO_W)))
        self.assertEqual(code, 0)                                                              # control
        self.assertEqual(json.loads(Path(self.tmp.name, 'ok_cli', 'capture.json').read_text())['pitch_deg_nominal'],
                         -10.0)

    # ---- preflight r4 (real-SDK and Luxonis-forum findings before the first run on a device)
    def test_sync_threshold_is_above_half_the_frame_period(self):
        # The SDK default (10 ms, attempts -1) sends nothing until colour and depth are that close; the DepthAI
        # depth_video_synced example uses 50 ms. Above half the frame period a partner always exists.
        self.assertEqual(R.SYNC_THRESHOLD_MS, 60)
        self.assertGreater(R.SYNC_THRESHOLD_MS, 1000.0 / R.FPS / 2.0)
        self.assertLess(R.SYNC_THRESHOLD_MS, 1000.0 / R.FPS)
        pipeline, description = R.build_pipeline(fake_dai(), R.select_profile(PRO_W), 'extended')
        sync = pipeline.nodes('Sync')[0]
        self.assertEqual(sync.called('setSyncThreshold'), [(timedelta(milliseconds=60),)])
        self.assertEqual(sync.called('setSyncAttempts'), [])
        self.assertEqual(description['sync']['threshold_ms'], 60)
        meta, _ = self.record(Device(PRO_W, depth_offsets=(0.001, -0.004, 0.023)))
        self.assertEqual(meta['depth_minus_colour_ms']['sync_threshold_ms'], 60)
        self.assertAlmostEqual(meta['depth_minus_colour_ms']['min'], -4.0, places=6)
        self.assertAlmostEqual(meta['depth_minus_colour_ms']['max'], 23.0, places=6)
        self.assertEqual(len(meta['frames']), 3)
        self.assertTrue(any('depth-colour time 1.0 ms' in line and 'depth valid' in line for line in self.log))

    def test_profile_sizes_fit_the_device_depth_width_rule(self):
        # Device rule: "Disparity/depth width must be multiple of 16" for a depth aligned to the colour camera.
        sensor_sizes = {'THE_800_P': (1280, 800), 'THE_12_MP': (4056, 3040)}
        self.assertEqual(sorted(R.PROFILES), ['IMX378', 'OV9282', 'OV9782'])
        for name, profile in R.PROFILES.items():
            width, height = profile['size']
            self.assertEqual(width % 16, 0, name)
            self.assertEqual(height % 2, 0, name)
            full = sensor_sizes[profile['resolution']]
            num, den = profile['isp_scale'] or (1, 1)
            self.assertEqual((width, height), tuple((v * num - 1) // den + 1 for v in full), name)   # SDK getIspSize
        self.assertEqual(R.PROFILES['IMX378']['size'], (1248, 936))
        for size in ((1014, 760), (1280, 801)):                # the former 1/4 profile; an odd height
            bad = dict(R.select_profile(PRO), size=size, isp_scale=(1, 4))
            with self.assertRaises(R.RecorderError) as caught:
                R.build_pipeline(fake_dai(), bad, 'extended', lens_position=131)
            self.assertEqual(caught.exception.code, 'BAD_ARGUMENT')
            self.assertIn('multiple of 16', str(caught.exception))
        plan = R.probe(fake_dai(Device(PRO)), imu_seconds=0)['recording_plan']
        self.assertEqual((plan['profile'], plan['size'], list(plan['isp_scale'])), ('IMX378', [1248, 936], [4, 13]))
        meta, args = self.record(Device(PRO), name='imx')                             # positive control: records
        self.assertEqual(meta['status'], 'COMPLETE')
        with np.load(Path(args.output) / meta['frames'][0]['file'], allow_pickle=False) as data:
            self.assertEqual(data['isp'].shape, (1404, 1248))
            self.assertEqual(data['depth_mm'].shape, (936, 1248))
        self.assertEqual(meta['calibration']['sockets']['CAM_A']['output_size'], [1248, 936])

    def test_warmup_counts_from_the_first_received_frame(self):
        class SlowStart(Device):
            # the pipeline start takes 3 s before the first frame; warm-up time must not run during it
            def getOutputQueue(self, name, maxSize, blocking):
                queue = Device.getOutputQueue(self, name, maxSize, blocking)
                if name == 'rgbd':
                    real, calls = queue.tryGet, []

                    def late():
                        calls.append(1)
                        if len(calls) <= 30:
                            self.clock.t += 0.1
                            return None
                        return real()
                    queue.tryGet = late
                return queue
        stamps = tuple(1.0 + 0.1 * i for i in range(12))
        for name, cls in (('prompt', Device), ('slow', SlowStart)):
            self.log.clear()
            meta, _ = self.record(cls(PRO_W, stamps=stamps), name='warm_' + name, warmup_s=0.25)
            warm = meta['warmup']
            # fake clock: 0.1 s per received frame, so frames at 0.0, 0.1, 0.2 and 0.3 s are discarded (0.3 >= 0.25)
            self.assertEqual(warm['frames_discarded'], 4, name)
            self.assertEqual(warm['counted_from'], 'first received frame')
            self.assertEqual((warm['first']['exposure_time_us'], warm['last']['exposure_time_us']), (800.0, 830.0))
            self.assertEqual((warm['first']['sensitivity_iso'], warm['last']['sensitivity_iso']), (100, 103))
            self.assertEqual([f['sequence_num'] for f in meta['frames']], [4, 5, 6], name)
            self.assertEqual(sum('warm-up: 4 frames discarded' in line for line in self.log), 1)
            self.assertTrue(all(f['accel_count'] > 0 for f in meta['frames']))
            # every accelerometer sample is kept, the warm-up's included (review r4 F2)
            with np.load(Path(self.tmp.name, 'warm_' + name, 'imu.npz'), allow_pickle=False) as data:
                kept = data['accel']
            self.assertEqual(kept.shape[1], 4)
            self.assertEqual(kept.shape[0], meta['imu_all_samples']['count'])
            self.assertLess(kept[0, 0], 1.0)                              # before the first (discarded) frame
            self.assertGreaterEqual(kept[-1, 0], meta['frames'][-1]['t_device_s'])
            self.assertTrue(np.all(np.diff(kept[:, 0]) > 0))
            self.assertGreater(kept.shape[0], sum(f['accel_count'] for f in meta['frames']))
        meta, _ = self.record(Device(PRO_W), name='warm_zero')                         # --warmup-s 0 discards none
        self.assertEqual((meta['warmup']['frames_discarded'], meta['warmup']['first']), (0, None))
        self.assertEqual([f['sequence_num'] for f in meta['frames']], [0, 1, 2])
        self.assertEqual(R.parser().parse_args(['--record']).warmup_s, 3.0)

    def test_unexpected_sdk_error_is_named_and_keeps_the_capture(self):
        link_error = ("Communication exception - possible device error/misconfiguration. Original message "
                      "'Couldn't read data from stream: 'rgbd' (X_LINK_ERROR)'")

        class Dropping(Device):
            def __init__(self, sensors, error):
                Device.__init__(self, sensors)
                self.error = error

            def getOutputQueue(self, name, maxSize, blocking):
                queue = Device.getOutputQueue(self, name, maxSize, blocking)
                if name == 'rgbd':
                    real, calls = queue.tryGet, []

                    def dropping():
                        calls.append(1)
                        if len(calls) > 1:
                            raise self.error
                        return real()
                    queue.tryGet = dropping
                return queue

        def run(name, device):
            stderr = io.StringIO()
            with mock.patch.object(R.sys, 'stderr', stderr), mock.patch.object(R.sys, 'stdout', io.StringIO()):
                code = R.main(['--record', '--output', os.path.join(self.tmp.name, name), '--scene', 's',
                               '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02', '--pitch-deg', '0',
                               '--frames', '3', '--warmup-s', '0'], dai=fake_dai(device))
            return code, stderr.getvalue()
        code, text = run('dropped', Dropping(PRO_W, RuntimeError(link_error)))
        self.assertEqual(code, 2)
        self.assertTrue(text.startswith('oakd_field_recorder refused: UNEXPECTED: RuntimeError: '))
        for word in ('X_LINK_ERROR', 'NEW folder', '--projector 0'):
            self.assertIn(word, text)
        self.assertNotIn('Traceback', text)
        meta = json.loads(Path(self.tmp.name, 'dropped', 'capture.json').read_text())
        self.assertEqual((meta['status'], meta['stop_error']['code'], len(meta['frames'])),
                         ('INCOMPLETE', 'UNEXPECTED', 1))
        self.assertEqual(len(Path(self.tmp.name, 'dropped', 'MANIFEST.sha256').read_text().splitlines()), 3)
        code, text = run('ctrl_c', Dropping(PRO_W, KeyboardInterrupt()))
        self.assertEqual(code, 130)
        self.assertIn('INTERRUPTED', text)
        self.assertEqual(json.loads(Path(self.tmp.name, 'ctrl_c', 'capture.json').read_text())['status'], 'INCOMPLETE')
        broken = Device(PRO_W)                                           # an SDK error before any folder exists
        broken.readCalibration = mock.Mock(side_effect=RuntimeError('Failed to read calibration'))
        code, text = run('no_calibration', broken)
        self.assertEqual(code, 2)
        self.assertIn('UNEXPECTED: RuntimeError: Failed to read calibration', text)
        self.assertFalse(os.path.exists(os.path.join(self.tmp.name, 'no_calibration')))
        self.assertTrue(broken.closed)
        code, text = run('control', Device(PRO_W))                       # positive control
        self.assertEqual((code, text), (0, ''))

    def test_colour_socket_reported_as_ov9282_records_with_a_warning(self):
        grey = {'CAM_A': 'OV9282', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'}
        meta, _ = self.record(Device(grey), name='grey')
        self.assertEqual(meta['status'], 'COMPLETE')
        self.assertEqual(meta['pipeline']['colour']['sensor'], 'OV9282')
        self.assertIn('GREY', meta['pipeline']['colour']['sensor_warning'])
        self.assertEqual(sum('WARNING COLOUR_SENSOR_NAME' in line for line in self.log), 1)
        self.assertEqual(meta['pipeline']['colour']['size'], [1280, 800])
        report = R.probe(fake_dai(Device(grey)), imu_seconds=0)
        self.assertIn('GREY', report['recording_plan']['warning'])
        self.assertIsNone(report['recording_plan']['refusal'])
        self.log.clear()
        meta, _ = self.record(Device(PRO_W), name='colour')                            # control: no warning
        self.assertIsNone(meta['pipeline']['colour']['sensor_warning'])
        self.assertFalse(any('COLOUR_SENSOR_NAME' in line for line in self.log))
        self.assertIsNone(R.probe(fake_dai(Device(PRO_W)), imu_seconds=0)['recording_plan']['warning'])
        spaced = R.sensor_map(None, Device({'CAM_A': ' ov9782 ', 'CAM_B': 'OV9282', 'CAM_C': 'OV9282'}))
        self.assertEqual(R.select_profile(spaced)['colour_sensor'], 'OV9782')          # names are normalised

    # ---- review r4
    def test_argument_refusals_come_before_the_depthai_import(self):
        # F1: a process that exits right after "import depthai" was aborted by the SDK (exit 134). With depthai not
        # importable, an argument refusal must still be the one reported: the import was never reached.
        os.mkdir(os.path.join(self.tmp.name, 'taken'))
        base = ['--record', '--scene', 's', '--camera-height-m', '1', '--camera-height-uncertainty-m', '0.02']
        cases = ((base + ['--pitch-deg', '0', '--output', os.path.join(self.tmp.name, 'taken')], 'OUTPUT_EXISTS'),
                 (base + ['--pitch-deg', '0', '--output', os.path.join(self.tmp.name, 'a'), '--frames', '101'],
                  'FRAMES_OUT_OF_RANGE'),
                 (base + ['--output', os.path.join(self.tmp.name, 'b')], 'BAD_ARGUMENT'),
                 (base + ['--pitch-deg', '0', '--output', os.path.join(self.tmp.name, 'no_parent', 'c')],
                  'BAD_ARGUMENT'))
        for argv, code in cases:
            stderr = io.StringIO()
            with mock.patch.dict(sys.modules, {'depthai': None}), mock.patch.object(R.sys, 'stderr', stderr):
                self.assertEqual(R.main(argv), 2)
            self.assertIn('refused: %s:' % code, stderr.getvalue())
            self.assertNotIn('UNSUPPORTED_DEPTHAI_VERSION', stderr.getvalue())
        self.assertIn('mkdir -p', stderr.getvalue())                     # the missing parent names its fix
        stderr = io.StringIO()                                           # control: valid arguments reach the import
        with mock.patch.dict(sys.modules, {'depthai': None}), mock.patch.object(R.sys, 'stderr', stderr):
            self.assertEqual(R.main(base + ['--pitch-deg', '0', '--output', os.path.join(self.tmp.name, 'ok')]), 2)
        self.assertIn('UNSUPPORTED_DEPTHAI_VERSION', stderr.getvalue())
        events = []
        with mock.patch.object(R.sys, 'stdout', mock.Mock(flush=lambda: events.append('out'))), \
                mock.patch.object(R.sys, 'stderr', mock.Mock(flush=lambda: events.append('err'))), \
                mock.patch.object(R.os, '_exit', side_effect=lambda code: events.append(code)):
            R._leave(2)
        self.assertEqual(events, ['out', 'err', 2])                      # flushed, then left with the code
        source = (ROOT / 'tools/oakd_field_recorder.py').read_text()
        self.assertTrue(source.rstrip().endswith("if __name__ == '__main__':\n    _leave(main())"))

    def test_padded_isp_data_is_flagged_and_kept_whole(self):
        # Review r4 F4 / r4b N1: never stored silently as garbled colour, and never a refusal that repeats per frame.
        meta, args = self.record(Device(PRO_W, padding=4096), name='padded')
        self.assertEqual((meta['status'], meta['isp_padded_frames'], len(meta['frames'])), ('COMPLETE', 3, 3))
        self.assertEqual(sum('WARNING ISP_PADDED' in line for line in self.log), 1)
        with np.load(Path(args.output) / meta['frames'][0]['file'], allow_pickle=False) as data:
            self.assertEqual(data['isp_raw'].shape, (1280 * 1200 + 4096,))
            self.assertEqual(data['isp_raw'].dtype, np.uint8)
            self.assertTrue((data['isp_raw'] == 7).all())
            self.assertEqual(data['isp_fb'].tolist(), [1296, 0, 1, 2, 1280, 800, 1])
        self.assertEqual(meta['frames'][0]['isp_bytes_delivered'], 1280 * 1200 + 4096)
        self.assertEqual(R.frame_buffer_layout(object()), [-1] * 7)
        self.log.clear()
        meta, args = self.record(Device(PRO_W), name='unpadded')                               # control
        self.assertEqual(meta['isp_padded_frames'], 0)
        self.assertFalse(any('ISP_PADDED' in line for line in self.log))
        with np.load(Path(args.output) / meta['frames'][0]['file'], allow_pickle=False) as data:
            self.assertNotIn('isp_raw', data.files)
        self.assertEqual([f['isp_bytes_delivered'] for f in meta['frames']], [1280 * 1200] * 3)
        self.assertEqual([f['depth_exposure_time_us'] for f in meta['frames']], [800.0, 810.0, 820.0])

    def test_imu_queue_is_large_drained_while_waiting_and_synced_to_disk(self):
        device = Device(PRO_W)
        with mock.patch.object(R.os, 'sync') as synced:
            meta, args = self.record(device)
        synced.assert_called_once_with()
        self.assertEqual(device.queue_sizes, {'rgbd': 4, 'imu': R.IMU_QUEUE_SIZE})
        self.assertGreaterEqual(R.IMU_QUEUE_SIZE, 1000)
        self.assertEqual(meta['pipeline']['imu']['host_queue_messages'], R.IMU_QUEUE_SIZE)
        manifest = Path(args.output, 'MANIFEST.sha256').read_text().splitlines()
        self.assertEqual([line.split('  ')[1] for line in manifest][-2:], ['imu.npz', 'capture.json'])
        self.assertEqual(R.sha256_file(Path(args.output, 'imu.npz')), meta['imu_all_samples']['sha256'])
        self.assertGreater(meta['imu_all_samples']['count'], 0)
        polls, idles = [None, None, None, 'group'], []
        queue = types.SimpleNamespace(tryGet=lambda: polls.pop(0))
        clock = Clock()
        self.assertEqual(R._next_group(queue, clock() + 1.0, clock, clock.sleep, lambda: idles.append(1)), 'group')
        self.assertEqual(len(idles), 3)                                  # drained on every empty poll
        with self.assertRaises(R.RecorderError) as caught:
            R._next_group(types.SimpleNamespace(tryGet=lambda: None), clock() + 0.02, clock, clock.sleep)
        self.assertEqual(caught.exception.code, 'FRAME_TIMEOUT')
        on_disk = json.loads(Path(args.output, 'capture.json').read_text())      # r4b N2: the files, not memory
        self.assertEqual(on_disk['imu_all_samples'], meta['imu_all_samples'])

        def no_imu_file(path, *a, **k):
            if str(path).endswith('imu.npz'):
                raise OSError(28, 'No space left on device')
            return open(path, *a, **k)
        with mock.patch.object(R, 'open', side_effect=no_imu_file, create=True):
            with self.assertRaises(OSError):
                self.record(Device(PRO_W), name='imu_disk_full')
        kept = json.loads(Path(self.tmp.name, 'imu_disk_full', 'capture.json').read_text())
        self.assertEqual(len(kept['frames']), 3)                         # the status record survives the disk error
        self.assertFalse(os.path.exists(os.path.join(self.tmp.name, 'imu_disk_full', 'imu.npz')))
        meta, args = self.record(Device(PRO_W, imu='NONE'), name='no_imu')        # no IMU: an empty imu.npz
        self.assertEqual(meta['imu_all_samples']['count'], 0)
        with np.load(Path(args.output, 'imu.npz'), allow_pickle=False) as data:
            self.assertEqual(data['accel'].shape, (0, 4))

    def test_preset_deprecation_warning_is_not_shown_to_the_operator(self):
        with warnings.catch_warnings(record=True) as shown:
            warnings.simplefilter('always')
            pipeline, _ = R.build_pipeline(fake_dai(), R.select_profile(PRO_W), 'extended')
        self.assertEqual([str(w.message) for w in shown], [])
        self.assertEqual(len(pipeline.nodes('StereoDepth')[0].called('setDefaultProfilePreset')), 1)
        with warnings.catch_warnings(record=True) as shown:              # control: the fake does warn
            warnings.simplefilter('always')
            Node('x').setDefaultProfilePreset('HD')
        self.assertEqual(len(shown), 1)

    def test_probe_writes_its_json_to_a_file_when_asked(self):
        target = os.path.join(self.tmp.name, 'probe.json')
        stdout = io.StringIO()
        with mock.patch.object(R.sys, 'stdout', stdout):
            code = R.main(['--probe', '--probe-imu-s', '0', '--output', target], dai=fake_dai(Device(PRO_W)))
        self.assertEqual(code, 0)
        self.assertEqual(json.loads(Path(target).read_text())['recording_plan']['profile'], 'OV9782')
        self.assertEqual(stdout.getvalue().strip(), 'probe written to %s' % target)
        stdout = io.StringIO()
        with mock.patch.object(R.sys, 'stdout', stdout):                 # control: without --output, to stdout
            self.assertEqual(R.main(['--probe', '--probe-imu-s', '0'], dai=fake_dai(Device(PRO_W))), 0)
        self.assertEqual(json.loads(stdout.getvalue())['recording_plan']['profile'], 'OV9782')

    def test_protocol_commands_parse_and_match_the_pinned_versions(self):
        # Review r4 F16: the commands the TA types are checked against the argument parser and the pins.
        text = (ROOT / 'ros2/motar_camera_policy/FIELD_RECORDING_KO.md').read_text()
        blocks = '\n'.join(re.findall(r'```bash\n(.*?)```', text, re.S)).replace('\\\n', ' ')
        commands = [line.strip() for line in blocks.splitlines() if line.strip().startswith('oakrec ')]
        self.assertEqual(len(commands), 5)                               # probe, 2 desk-test, 2 M01 recordings
        home = self.tmp.name
        os.mkdir(os.path.join(home, 'oakd_rec'))
        recorded = 0
        for command in commands:
            argv = [word.replace('~', home) for word in shlex.split(command)[1:]]
            args = R.parser().parse_args(argv)
            if args.record:
                R.check_record_args(args)
                self.assertIn(args.projector, (0.0, 0.5))
                recorded += 1
            else:
                self.assertTrue(args.probe and args.output.endswith('oakd_probe.json'))
        self.assertEqual(recorded, 4)
        install = 'PIP_USER=0 python3 -m pip install --no-cache-dir --target ~/oakd_rec_pkgs depthai==%s numpy==%s' \
                  % (R.SDK_VERSION, R.NUMPY_PIN)
        self.assertIn(install + '\n', text)
        self.assertIn(install, R.INSTALL_HINT)
        alias = "alias oakrec='PYTHONNOUSERSITE=1 PYTHONPATH=$HOME/oakd_rec_pkgs python3 $HOME/oakd_field_recorder.py'"
        self.assertIn(alias, text)
        codes = set(re.findall(r"RecorderError\('([A-Z_0-9]+)'", (ROOT / 'tools/oakd_field_recorder.py').read_text()))
        codes |= {'OS_ERROR', 'UNEXPECTED', 'INTERRUPTED'}
        self.assertGreaterEqual(len(codes), 19)
        table = text.split('## 12.')[1].split('## 13.')[0]
        for code in sorted(codes) + ['WARNING IMU_NO_SAMPLES', 'WARNING ISP_PADDED', 'WARNING COLOUR_SENSOR_NAME']:
            self.assertIn('`%s`' % code, table, code)                    # every code the program prints is explained
        for line in ('mkdir -p ~/oakd_rec\n', 'USB="/media/$USER/USB이름"', '"$USB"/',
                     '/etc/udev/rules.d/99-oakd-recorder.rules', 'oakrec --probe --output ~/oakd_probe.json\n'):
            self.assertIn(line, text, line)
        self.assertNotIn('80-movidius', (ROOT / 'tools/oakd_field_recorder.py').read_text())


if __name__ == '__main__':
    unittest.main()
