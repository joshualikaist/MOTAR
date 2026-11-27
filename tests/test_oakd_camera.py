"""Fake DepthAI v2 graph/messages only; never construct a real Device or policy."""
from datetime import timedelta
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("oakd_camera_tested", ROOT / "tools/oakd_camera.py")
C = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = C
SPEC.loader.exec_module(C)


class Clock:
    def __init__(self, t=1.02):
        self.t = t

    def __call__(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class Message:
    def __init__(self, data, stamp=1.0, seq=1):
        self.data, self.stamp, self.seq = data, stamp, seq

    def getTimestamp(self):
        return timedelta(seconds=self.stamp)

    def getSequenceNum(self):
        return self.seq

    def getCvFrame(self):
        return self.data

    def getFrame(self):
        return self.data


class Queue:
    def __init__(self, groups):
        self.groups = list(groups)

    def tryGet(self):
        return self.groups.pop(0) if self.groups else None


class Port:
    def __init__(self):
        self.links = []

    def link(self, target):
        self.links.append(target)


class Node:
    def __init__(self):
        self.calls = []
        self.initialControl = self
        self.out, self.video, self.isp, self.depth = Port(), Port(), Port(), Port()
        self.left, self.right, self.input = Port(), Port(), Port()
        self.inputs = {"rgb": Port(), "depth": Port()}

    def __getattr__(self, name):
        if name.startswith("set"):
            return lambda *args: self.calls.append((name, args))
        raise AttributeError(name)


def fake_sdk():
    kinds = types.SimpleNamespace(ColorCamera="rgb", MonoCamera="mono", Sync="sync", XLinkOut="out",
                                  StereoDepth=types.SimpleNamespace(PresetMode=types.SimpleNamespace(HIGH_DENSITY="HD")))

    class Pipeline:
        def __init__(self):
            self.created = []

        def create(self, kind):
            node = Node()
            self.created.append((kind, node))
            return node

    return types.SimpleNamespace(__version__=C.SDK_VERSION, Pipeline=Pipeline, node=kinds,
                                 CameraBoardSocket=types.SimpleNamespace(CAM_A="A", CAM_B="B", CAM_C="C"),
                                 ColorCameraProperties=types.SimpleNamespace(SensorResolution=types.SimpleNamespace(THE_12_MP="12MP")),
                                 MonoCameraProperties=types.SimpleNamespace(SensorResolution=types.SimpleNamespace(THE_800_P="800")),
                                 DeviceInfo=lambda identifier: identifier,
                                 Device=mock.Mock(side_effect=AssertionError("real Device access forbidden")))


class OakDTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        evidence = Path(self.tmp.name) / "measured.txt"
        evidence.write_text("fixture-only measured geometry; not real hardware evidence")
        self.proof = {"path": str(evidence), "sha256": C.sha256_file(evidence)}
        self.config = {"format": "motar-oakd-calibration-v1", "sdk_version": C.SDK_VERSION,
                       "rgb_sensor": "IMX378",
                       "device_id": "fake-fixture-device", "stereo_mode": "extended",
                       "mount_flu_m": [0.1, 0.0, 0.03], "camera_to_body_rotation": np.eye(3).tolist(),
                       "mount_measurement": dict(self.proof),
                       "depth_alignment": {"socket": "CAM_A", "coordinate_model": "distorted_rgb",
                                           "verified": True, "evidence": dict(self.proof)}}
        self.rectifier = C.Rectifier([[10.0, 0, 1.5], [0, 10.0, 1.0], [0, 0, 1]], [0.0] * 5,
                                    width=4, height=3)
        self.bgr = np.zeros((3, 4, 3), np.uint8)
        self.bgr[:, :, 2] = 255
        self.depth = np.full((3, 4), 1234, np.uint16)
        self.depth[1, 1] = 0

    def group(self, stamp=1.0, seq=1, skew=0.0):
        return {"rgb": Message(self.bgr, stamp, seq), "depth": Message(self.depth, stamp + skew, seq)}

    def capture(self, groups, clock=None, offset=0.0):
        clock = clock or Clock()
        return C.OakDQueueCapture(Queue(groups), self.config, self.rectifier,
                                 C.HostClockBinding(offset, "fake measured clock"), sdk_version=C.SDK_VERSION,
                                 clock=clock, sleep=clock.sleep), clock

    def test_valid_two_frames_preserve_mm_invalid_rgb_and_measured_provenance(self):
        capture, clock = self.capture([self.group(), self.group(1.1, 2)])
        serviced = []
        first = capture.frames(deadline=clock() + 0.2, service=lambda: serviced.append(clock()))
        self.assertTrue((first["rgb"][:, :, 0] == 255).all())
        self.assertAlmostEqual(float(first["depth_z_m"][0, 0]), 1.234, places=5)
        self.assertTrue(first["invalid"][1, 1])
        self.assertEqual(int(first["invalid"].sum()), 1)
        self.assertTrue(first["aligned"] and first["rectified"])
        self.assertFalse(first["synthetic_noise_applied"])
        self.assertEqual(first["calibration_provenance"]["device_id"], "fake-fixture-device")
        self.assertNotIn("ranges", first)
        clock.t = 1.12
        second = capture.frames(deadline=clock() + 0.2, service=lambda: serviced.append(clock()))
        self.assertGreater(second["rgb_t"], first["rgb_t"])
        self.assertGreaterEqual(len(serviced), 4)

    def test_stale_future_skew_and_reuse_fail_closed(self):
        for group in (self.group(0.5), self.group(1.1), self.group(skew=0.04)):
            capture, clock = self.capture([group])
            with self.assertRaises(C.OakDInputError):
                capture.frames(deadline=clock() + 0.2, service=lambda: None)
            with self.assertRaisesRegex(C.OakDInputError, "failed/closed"):
                capture.frames(deadline=clock() + 0.2, service=lambda: None)
        capture, clock = self.capture([self.group(), self.group()])
        capture.frames(deadline=clock() + 0.2, service=lambda: None)
        with self.assertRaisesRegex(C.OakDInputError, "reused/regressing"):
            capture.frames(deadline=clock() + 0.2, service=lambda: None)

    def test_absolute_deadline_services_silent_queue_and_invalid_payload(self):
        capture, clock = self.capture([])
        service_t = []
        with self.assertRaises(TimeoutError):
            capture.frames(deadline=clock() + 0.1, service=lambda: service_t.append(clock()))
        self.assertGreaterEqual(len(service_t), 4)
        self.assertLessEqual(max(b - a for a, b in zip(service_t, service_t[1:])), C.SERVICE_QUANTUM_S + 1e-8)
        capture, clock = self.capture([{"rgb": Message(self.bgr)}])
        with self.assertRaises(KeyError):
            capture.frames(deadline=clock() + 0.1, service=lambda: None)
        self.assertTrue(capture.failed)
        with self.assertRaisesRegex(C.OakDInputError, "uint16"):
            self.rectifier.apply(self.bgr, self.depth.astype(np.float32))

    def test_missing_measured_geometry_alignment_and_v3_refused(self):
        self.assertEqual(C.validate_config(self.config), self.config)
        for key in ("mount_flu_m", "camera_to_body_rotation", "mount_measurement", "depth_alignment"):
            bad = dict(self.config)
            del bad[key]
            with self.assertRaises((C.OakDInputError, OSError)):
                C.validate_config(bad)
        bad = dict(self.config, sdk_version="3.0.0")
        with self.assertRaisesRegex(C.OakDInputError, "v3"):
            C.validate_config(bad)
        bad = dict(self.config, camera_to_body_rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
        with self.assertRaisesRegex(C.OakDInputError, "proper rotation"):
            C.validate_config(bad)

    def test_pipeline_factory_is_v2_graph_only_and_focus_matches_calibration(self):
        sdk = fake_sdk()
        pipeline = C.build_pipeline(sdk, lens_position=123, stereo_mode="extended")
        self.assertEqual(len(pipeline.created), 6)
        rgb, left, right, stereo, sync, out = [node for _, node in pipeline.created]
        self.assertIn(("setManualFocus", (123,)), rgb.calls)
        self.assertIn(("setResolution", ("12MP",)), rgb.calls)
        self.assertIn(("setIspScale", (1, 4)), rgb.calls)
        self.assertFalse(any(name == "setVideoSize" for name, _ in rgb.calls))
        self.assertIn(("setDepthAlign", ("A",)), stereo.calls)
        self.assertIn(("setDepthAlignmentUseSpecTranslation", (False,)), stereo.calls)
        self.assertIn(("setDisparityToDepthUseSpecTranslation", (False,)), stereo.calls)
        self.assertIn(("setExtendedDisparity", (True,)), stereo.calls)
        self.assertIn(("setSubpixel", (False,)), stereo.calls)
        self.assertIn(("setSyncThreshold", (timedelta(milliseconds=10),)), sync.calls)
        self.assertIn(("setOutputSize", (1014, 760)), stereo.calls)
        self.assertEqual(len(rgb.isp.links), 1)
        self.assertEqual(len(rgb.video.links), 0)
        self.assertEqual(len(stereo.depth.links), 1)
        self.assertEqual(len(sync.out.links), 1)
        sdk.Device.assert_not_called()

    def test_connect_is_explicit_and_wrong_device_closed_before_pipeline(self):
        sdk = fake_sdk()
        with self.assertRaisesRegex(C.OakDInputError, "allow_hardware"):
            C.connect(self.config, sdk=sdk)
        sdk.Device.assert_not_called()
        device = types.SimpleNamespace(getMxId=lambda: "another-device", close=mock.Mock())
        sdk.Device = mock.Mock(return_value=device)
        with self.assertRaisesRegex(C.OakDInputError, "does not match"):
            C.connect(self.config, allow_hardware=True, sdk=sdk)
        device.close.assert_called_once()

    def test_explicit_host_clock_offset_is_applied(self):
        capture, clock = self.capture([self.group(0.5)], offset=0.5)
        frame = capture.frames(deadline=clock() + 0.1, service=lambda: None)
        self.assertEqual(frame["rgb_t"], 1.0)
        sdk = types.SimpleNamespace(Clock=types.SimpleNamespace(now=lambda: timedelta(seconds=8)))
        measured = C.measure_host_clock(sdk, clock=clock)
        self.assertAlmostEqual(measured.translate(8.0), clock())

    def test_slow_service_and_slow_rectification_are_not_called_timely(self):
        capture, clock = self.capture([self.group()])
        with self.assertRaisesRegex(TimeoutError, "service callback exceeded"):
            capture.frames(deadline=clock() + 0.2, service=lambda: clock.sleep(0.03))
        capture, clock = self.capture([self.group()])
        original = capture.rectifier.apply

        def slow(*args):
            clock.sleep(0.03)
            return original(*args)

        with mock.patch.object(capture.rectifier, "apply", side_effect=slow):
            with self.assertRaisesRegex(TimeoutError, "service interval exceeded"):
                capture.frames(deadline=clock() + 0.2, service=lambda: None)

    def test_undistortion_border_is_reported_as_coverage_not_only_as_invalid_depth(self):
        # Positive control: no distortion -> every rectified pixel is covered.
        _, _, invalid, coverage = self.rectifier.apply(self.bgr, self.depth)
        self.assertEqual(coverage.dtype, np.bool_)
        self.assertTrue(coverage.all())
        self.assertEqual(int(invalid.sum()), 1)
        # Strong pincushion distortion (k1 > 0) with the same K: rectified corners sample outside the raw image.
        warped = C.Rectifier([[30.0, 0, 19.5], [0, 30.0, 14.5], [0, 0, 1]], [0.45, 0.0, 0.0, 0.0, 0.0],
                             width=40, height=30)
        bgr = np.full((30, 40, 3), 200, np.uint8)
        depth = np.full((30, 40), 1500, np.uint16)
        rgb, z, invalid, coverage = warped.apply(bgr, depth)
        border = ~coverage
        self.assertGreater(int(border.sum()), 0)
        self.assertGreater(int(coverage.sum()), 0)
        self.assertTrue(border[0, 0] and border[-1, -1] and coverage[15, 20])
        # The remap writes zero depth there; coverage is what tells the runtime it is not a sensor return.
        self.assertTrue(np.all(invalid[border]) and np.all(z[border] == 0))
        self.assertFalse(invalid[coverage].any())
        self.assertIn("coverage", warped.provenance())

    def test_capture_frame_carries_coverage(self):
        capture, clock = self.capture([self.group()])
        frame = capture.frames(deadline=clock() + 0.2, service=lambda: None)
        self.assertEqual(frame["coverage"].shape, frame["invalid"].shape)
        self.assertTrue(frame["coverage"].all())

    def test_measured_camera_coverage_accepts_wide_and_rejects_cropped_input(self):
        wide = C.Rectifier([[2.0, 0, 1.5], [0, 1.5, 1.0], [0, 0, 1]], [0.0] * 5, width=4, height=3)
        wide.require_training_fov()
        with self.assertRaisesRegex(C.OakDInputError, "do not cover"):
            self.rectifier.require_training_fov()


if __name__ == "__main__":
    unittest.main()
