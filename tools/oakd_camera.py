"""Source-only DepthAI 2.29.0.0 OAK-D RGB-D capture adapter; no import-time device access.

Capture never arms a vehicle or calls a policy. A measured mount calibration and
evidence establishing the SDK's aligned-depth coordinate convention are mandatory.
Unknown/different alignment is refused; it is not declared rectified by a flag alone.

Verified API references (v2, not v3):
https://raw.githubusercontent.com/luxonis/depthai-python/v2.29.0.0/examples/StereoDepth/rgb_depth_aligned.py
https://depthai-docs-website.readthedocs.io/projects/api/en/latest/components/nodes/color_camera/
https://depthai-docs-website.readthedocs.io/projects/api/en/latest/references/python/
https://docs.luxonis.com/software/depthai/examples/depth_video_sync
https://raw.githubusercontent.com/luxonis/depthai-core/v2.29.0/src/pipeline/node/ColorCamera.cpp
https://raw.githubusercontent.com/luxonis/depthai-core/v2.29.0/include/depthai/pipeline/node/StereoDepth.hpp

The SDK is optional and imported only by explicit connect(). Hardware and calibration
qualification remain NOT_TESTED by this source and fake-interface test suite.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import timedelta
import hashlib
import importlib
import importlib.metadata
import json
import math
from pathlib import Path
import time

SDK_VERSION = "2.29.0.0"
# v2.29 ColorCamera.cpp: THE_12_MP is 4056x3040; ISP 1/4 preserves the full
# sensor aspect. 1080P/video cropping would discard the policy's vertical FOV.
WIDTH, HEIGHT, FPS = 1014, 760, 10.0
MAX_AGE_S, MAX_SKEW_S, SERVICE_QUANTUM_S = 0.15, 0.025, 0.025


class OakDInputError(RuntimeError):
    pass


def sha256_file(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def sdk_inventory():
    """Package metadata only: never discover/connect a camera or import its SDK."""
    try:
        version = importlib.metadata.version("depthai")
    except importlib.metadata.PackageNotFoundError:
        version = None
    return {"required_depthai": SDK_VERSION, "installed_depthai": version,
            "sdk_status": "AVAILABLE_PINNED" if version == SDK_VERSION else "MISSING_OR_UNSUPPORTED",
            "hardware": "NOT_TESTED"}


def _numbers(value, count, label):
    try:
        raw = list(value)
        if any(isinstance(x, bool) for x in raw):
            raise ValueError("boolean is not a measured number")
        result = [float(x) for x in raw]
    except (TypeError, ValueError, OverflowError) as exc:
        raise OakDInputError("invalid %s" % label) from exc
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise OakDInputError("invalid %s" % label)
    return result


def _evidence(value, label):
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        raise OakDInputError("%s needs a measured evidence path and SHA256" % label)
    path = Path(value["path"])
    if not path.is_absolute() or sha256_file(path) != value["sha256"]:
        raise OakDInputError("%s evidence is missing or differs from its SHA256" % label)


def validate_config(config):
    """Validate caller measurements; never fabricate mount, alignment or device calibration."""
    if not isinstance(config, dict) or config.get("format") != "motar-oakd-calibration-v1":
        raise OakDInputError("unsupported measured OAK-D configuration")
    if config.get("sdk_version") != SDK_VERSION:
        raise OakDInputError("only DepthAI %s is supported; v3 is not interchangeable" % SDK_VERSION)
    if config.get("rgb_sensor") != "IMX378":
        raise OakDInputError("this full-sensor pipeline requires a measured IMX378 RGB sensor")
    if not isinstance(config.get("device_id"), str) or not config["device_id"].strip():
        raise OakDInputError("measured physical device_id is required")
    if config.get("stereo_mode") not in ("extended", "subpixel"):
        raise OakDInputError("select one supported stereo mode: extended or subpixel")
    _numbers(config.get("mount_flu_m"), 3, "measured mount_flu_m")
    rows = config.get("camera_to_body_rotation")
    if not isinstance(rows, list) or len(rows) != 3:
        raise OakDInputError("measured camera_to_body_rotation is required")
    r = [_numbers(row, 3, "camera_to_body_rotation") for row in rows]
    for i in range(3):
        for j in range(3):
            if abs(sum(r[k][i] * r[k][j] for k in range(3)) - float(i == j)) > 1e-5:
                raise OakDInputError("camera_to_body_rotation must be orthonormal")
    determinant = sum(r[0][i] * (r[1][(i + 1) % 3] * r[2][(i + 2) % 3]
                                  - r[1][(i + 2) % 3] * r[2][(i + 1) % 3]) for i in range(3))
    if abs(determinant - 1.0) > 1e-5:
        raise OakDInputError("camera_to_body_rotation must be a proper rotation")
    _evidence(config.get("mount_measurement"), "mount")
    alignment = config.get("depth_alignment")
    if (not isinstance(alignment, dict) or alignment.get("socket") != "CAM_A"
            or alignment.get("coordinate_model") != "distorted_rgb"
            or alignment.get("verified") is not True):
        raise OakDInputError("SDK aligned-depth distorted-RGB coordinates are unestablished; qualify alignment first")
    _evidence(alignment.get("evidence"), "RGB-depth alignment")
    return config


def load_config(path):
    config = json.loads(Path(path).read_text())
    validate_config(config)
    return config


@dataclass(frozen=True)
class HostClockBinding:
    """runtime host monotonic seconds = dai getTimestamp seconds + measured offset."""
    offset_s: float
    source: str

    def __post_init__(self):
        if (isinstance(self.offset_s, bool) or not math.isfinite(float(self.offset_s))
                or not isinstance(self.source, str) or not self.source.strip()):
            raise ValueError("an explicit finite host-clock binding and source are required")

    def translate(self, stamp):
        return float(stamp) + float(self.offset_s)


def measure_host_clock(sdk, *, clock=time.monotonic):
    before = float(clock())
    stamp = float(sdk.Clock.now().total_seconds())
    after = float(clock())
    if not all(math.isfinite(x) for x in (before, stamp, after)) or not 0 <= after - before <= 0.005:
        raise OakDInputError("host/DepthAI clock sampling interval is not bounded to 5 ms")
    sample = {"python_before": before, "depthai_now": stamp, "python_after": after}
    return HostClockBinding((before + after) * 0.5 - stamp, json.dumps(sample, sort_keys=True))


def build_pipeline(sdk, *, lens_position, stereo_mode):
    """Build the pinned v2 graph only; no Device(), discovery or USB access here."""
    if getattr(sdk, "__version__", None) != SDK_VERSION:
        raise OakDInputError("unsupported DepthAI SDK version")
    if type(lens_position) is not int or not 0 <= lens_position <= 255:
        raise OakDInputError("RGB focus must equal the device calibration lens position")
    if stereo_mode not in ("extended", "subpixel"):
        raise OakDInputError("unsupported stereo mode")
    pipeline = sdk.Pipeline()
    rgb = pipeline.create(sdk.node.ColorCamera)
    left = pipeline.create(sdk.node.MonoCamera)
    right = pipeline.create(sdk.node.MonoCamera)
    stereo = pipeline.create(sdk.node.StereoDepth)
    sync = pipeline.create(sdk.node.Sync)
    out = pipeline.create(sdk.node.XLinkOut)
    rgb.setBoardSocket(sdk.CameraBoardSocket.CAM_A)
    rgb.setResolution(sdk.ColorCameraProperties.SensorResolution.THE_12_MP)
    rgb.setIspScale(1, 4)
    rgb.setFps(FPS)
    rgb.initialControl.setManualFocus(lens_position)
    for mono, socket in ((left, sdk.CameraBoardSocket.CAM_B), (right, sdk.CameraBoardSocket.CAM_C)):
        mono.setBoardSocket(socket)
        mono.setResolution(sdk.MonoCameraProperties.SensorResolution.THE_800_P)
        mono.setFps(FPS)
    stereo.setDefaultProfilePreset(sdk.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(sdk.CameraBoardSocket.CAM_A)
    # StereoDepth.hpp v2.29.0: disparity depth scale and RGB alignment each
    # default to design translation; both must use measured device calibration.
    stereo.setDepthAlignmentUseSpecTranslation(False)
    stereo.setDisparityToDepthUseSpecTranslation(False)
    stereo.setOutputSize(WIDTH, HEIGHT)
    stereo.setExtendedDisparity(stereo_mode == "extended")
    stereo.setSubpixel(stereo_mode == "subpixel")
    # v2 modes are intentionally not combined or described as the trained noise model.
    sync.setSyncThreshold(timedelta(milliseconds=10))
    sync.setSyncAttempts(-1)
    out.setStreamName("rgbd")
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    rgb.isp.link(sync.inputs["rgb"])
    stereo.depth.link(sync.inputs["depth"])
    sync.out.link(out.input)
    return pipeline


class Rectifier:
    """Apply one calibrated inverse image map to both RGB and registered optical-z depth."""
    def __init__(self, raw_intrinsics, raw_distortion, *, width=WIDTH, height=HEIGHT):
        import numpy as np
        import cv2
        self.np, self.cv2 = np, cv2
        k, d = np.asarray(raw_intrinsics, np.float64), np.asarray(raw_distortion, np.float64)
        if (k.shape != (3, 3) or d.ndim != 1 or d.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(k).all() or not np.isfinite(d).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0 or not np.allclose(k[2], [0, 0, 1])
                or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9):
            raise OakDInputError("invalid device camera intrinsics/distortion")
        if type(width) is not int or type(height) is not int or not 1 < width <= WIDTH or not 1 < height <= HEIGHT:
            raise OakDInputError("unsupported camera buffer dimensions")
        self.width, self.height, self.k, self.d = width, height, k, d
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(k, d, np.eye(3), k, (width, height), cv2.CV_32FC1)

    def apply(self, bgr, depth_mm):
        """Return rectified RGB, optical z [m], sensor-invalid mask and source coverage.

        coverage is a ones image remapped with the depth's own map, interpolation and
        zero border: False marks rectified pixels outside the raw image. The zero depth
        there is not a sensor return, so the runtime treats it as no-return, never as
        a dense invalid (near) blob. invalid keeps the raw meaning, depth == 0.
        """
        np, cv2 = self.np, self.cv2
        bgr, depth = np.asarray(bgr), np.asarray(depth_mm)
        if bgr.shape != (self.height, self.width, 3) or bgr.dtype != np.uint8:
            raise OakDInputError("RGB video must be uint8 BGR at the calibrated output size")
        if depth.shape != (self.height, self.width) or depth.dtype != np.uint16:
            raise OakDInputError("depth must be uint16 optical-z millimetres at the aligned output size")
        color = cv2.remap(bgr, self.mapx, self.mapy, cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        rectified_mm = cv2.remap(depth, self.mapx, self.mapy, cv2.INTER_NEAREST,
                                 borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        coverage = cv2.remap(np.ones((self.height, self.width), np.uint8), self.mapx, self.mapy,
                             cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0) == 1
        return (color[:, :, ::-1].copy(), rectified_mm.astype(np.float32) * 0.001, rectified_mm == 0,
                coverage)

    def metadata(self):
        return {"width": self.width, "height": self.height, "fx": float(self.k[0, 0]),
                "fy": float(self.k[1, 1]), "cx": float(self.k[0, 2]), "cy": float(self.k[1, 2])}

    def require_training_fov(self, camera=None):
        """Check measured delivered K; sensor marketing FOV is never substituted.

        camera=None checks the CAM-K2 v3 camera (160x90, 66x54 deg) exactly as before. Runtime v1: a policy
        contract's camera block (width, height, fx, fy, cx, cy; e.g. CAM-S2 312x234 at 66.7x52.6 deg) checks that
        camera's corner pixel rays instead."""
        if camera is None:
            fx = 160.0 / (2.0 * math.tan(math.radians(66.0 / 2.0)))
            fy = 90.0 / (2.0 * math.tan(math.radians(54.0 / 2.0)))
            hx, hy, label = 79.5, 44.5, "66x54-degree"
        else:
            fx, fy = float(camera["fx"]), float(camera["fy"])
            hx, hy = float(camera["cx"]), float(camera["cy"])
            if (abs(hx - (camera["width"] - 1) / 2.0) > 1e-9 or abs(hy - (camera["height"] - 1) / 2.0) > 1e-9
                    or fx <= 0 or fy <= 0):
                raise OakDInputError("trained camera block must be centred with positive focal lengths")
            label = "%dx%d trained" % (camera["width"], camera["height"])
        uu = [(-hx / fx) * self.k[0, 0] + self.k[0, 2],
              (hx / fx) * self.k[0, 0] + self.k[0, 2]]
        vv = [(-hy / fy) * self.k[1, 1] + self.k[1, 2],
              (hy / fy) * self.k[1, 1] + self.k[1, 2]]
        if min(uu[0], vv[0]) < -0.5 or uu[1] > self.width - 0.5 or vv[1] > self.height - 0.5:
            raise OakDInputError("measured source intrinsics do not cover the %s pixel rays" % label)

    def provenance(self):
        return {"source": "device.readCalibration2 CAM_A at delivered output size",
                "raw_intrinsics": self.k.tolist(), "raw_distortion": self.d.tolist(),
                "rectification_rotation": "identity", "rectified_intrinsics": self.k.tolist(),
                "method": "cv2.initUndistortRectifyMap", "rgb_interpolation": "linear",
                "depth_interpolation": "nearest", "outside_depth": "zero_invalid",
                "coverage": "ones image, same map, nearest, zero border; False -> runtime no-return"}


class OakDQueueCapture:
    """Explicit opt-in consumer of one caller-owned v2 synchronized RGB-D output queue."""
    def __init__(self, queue, config, rectifier, clock_binding, *, sdk_version,
                 clock=time.monotonic, sleep=time.sleep, device=None):
        validate_config(config)
        if sdk_version != SDK_VERSION or not isinstance(clock_binding, HostClockBinding):
            raise OakDInputError("pinned SDK and measured host-clock binding are required")
        self.queue, self.config, self.rectifier = queue, config, rectifier
        self.binding, self.clock, self.sleep, self.device = clock_binding, clock, sleep, device
        self.failed, self.closed, self.previous = False, False, None

    def frames(self, *, deadline, service):
        if self.failed or self.closed:
            raise OakDInputError("capture has failed/closed; open a fresh session")
        if not callable(service) or not math.isfinite(deadline) or not 0 < deadline - self.clock() <= 2.0:
            raise ValueError("absolute frame deadline <=2 s and bounded service callback required")
        last_service = None

        def service_slice():
            nonlocal last_service
            current = float(self.clock())
            if last_service is not None and current - last_service > SERVICE_QUANTUM_S + 1e-8:
                raise TimeoutError("OAK-D service interval exceeded 25 ms; cooperative scheduling was late")
            service()
            last_service = current
            if self.clock() - current > SERVICE_QUANTUM_S + 1e-8:
                raise TimeoutError("OAK-D service callback exceeded 25 ms")

        try:
            while True:
                if self.clock() >= deadline:
                    raise TimeoutError("OAK-D RGB-D frame deadline")
                service_slice()
                if self.clock() >= deadline:
                    raise TimeoutError("OAK-D service exceeded frame deadline")
                group = self.queue.tryGet()
                if group is not None:
                    break
                self.sleep(min(SERVICE_QUANTUM_S, max(0.0, deadline - self.clock())))
            rgb_msg, depth_msg = group["rgb"], group["depth"]
            raw_t = [float(msg.getTimestamp().total_seconds()) for msg in (rgb_msg, depth_msg)]
            stamps = [self.binding.translate(t) for t in raw_t]
            now = float(self.clock())
            if (not all(math.isfinite(x) for x in raw_t + stamps + [now]) or min(raw_t + stamps) < 0
                    or any(not 0 <= now - stamp <= MAX_AGE_S for stamp in stamps)
                    or abs(stamps[0] - stamps[1]) > MAX_SKEW_S):
                raise OakDInputError("stale/future/unsynchronized OAK-D frames or wrong clock translation")
            seq = [msg.getSequenceNum() for msg in (rgb_msg, depth_msg)]
            if any(type(n) is not int or n < 0 for n in seq):
                raise OakDInputError("malformed OAK-D sequence numbers")
            if self.previous is not None and (any(t <= p for t, p in zip(stamps, self.previous[0]))
                                              or any(n <= p for n, p in zip(seq, self.previous[1]))):
                raise OakDInputError("reused/regressing OAK-D RGB-D packet")
            rgb, depth_z, invalid, coverage = self.rectifier.apply(rgb_msg.getCvFrame(), depth_msg.getFrame())
            service_slice()
            if self.clock() >= deadline or any(self.clock() - stamp > MAX_AGE_S for stamp in stamps):
                raise TimeoutError("OAK-D frame expired during conversion/service")
            self.previous = (stamps, seq)
            return {"rgb": rgb, "depth_z_m": depth_z, "invalid": invalid, "coverage": coverage,
                    "rgb_t": stamps[0], "depth_t": stamps[1], "now_s": float(self.clock()),
                    "intrinsics": self.rectifier.metadata(), "aligned": True, "rectified": True,
                    "mount_flu_m": list(self.config["mount_flu_m"]),
                    "camera_to_body_rotation": self.config["camera_to_body_rotation"],
                    "calibration_provenance": {"device_id": self.config["device_id"],
                                               "mount": self.config["mount_measurement"],
                                               "depth_alignment": self.config["depth_alignment"]},
                    "rectification_provenance": self.rectifier.provenance(),
                    "clock_provenance": {"source": self.binding.source, "offset_s": self.binding.offset_s,
                                         "raw_host_timestamps_s": raw_t},
                    "sdk_version": SDK_VERSION, "synthetic_noise_applied": False}
        except BaseException:
            self.failed = True
            raise

    def close(self):
        self.closed = True
        if self.device is not None:
            self.device.close()


def connect(config, *, allow_hardware=False, sdk=None, clock=time.monotonic, sleep=time.sleep, training_camera=None):
    """Explicit standalone startup; never enter this during active OFFBOARD control.

    Discovery, calibration and startPipeline are SDK blocking calls without a proven
    absolute startup bound. Only frames() has the serviced deadline. Never called
    by --check-config or package import.
    """
    validate_config(config)
    if allow_hardware is not True:
        raise OakDInputError("opening an OAK-D requires explicit allow_hardware=True")
    sdk = importlib.import_module("depthai") if sdk is None else sdk
    if getattr(sdk, "__version__", None) != SDK_VERSION:
        raise OakDInputError("DepthAI %s is required" % SDK_VERSION)
    device = sdk.Device(sdk.DeviceInfo(config["device_id"]))
    try:
        if device.getMxId() != config["device_id"]:
            raise OakDInputError("connected OAK-D does not match the measured device calibration")
        calibration = device.readCalibration2()
        socket = sdk.CameraBoardSocket.CAM_A
        sensors = device.getCameraSensorNames()
        if sensors.get(socket) != config["rgb_sensor"]:
            raise OakDInputError("connected RGB sensor does not match measured IMX378 configuration")
        k = calibration.getCameraIntrinsics(socket, WIDTH, HEIGHT)
        distortion = calibration.getDistortionCoefficients(socket)
        lens = calibration.getLensPosition(socket)
        rectifier = Rectifier(k, distortion)
        rectifier.require_training_fov(training_camera)     # None: CAM-K2 v3 camera, as before
        pipeline = build_pipeline(sdk, lens_position=lens, stereo_mode=config["stereo_mode"])
        binding = measure_host_clock(sdk, clock=clock)
        device.startPipeline(pipeline)
        queue = device.getOutputQueue(name="rgbd", maxSize=1, blocking=False)
        return OakDQueueCapture(queue, config, rectifier, binding, sdk_version=sdk.__version__,
                               clock=clock, sleep=sleep, device=device)
    except BaseException:
        device.close()
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check-config", type=Path, help="CPU metadata and calibration evidence checks only")
    mode.add_argument("--capture", type=Path, help="measured config for explicit camera-only recording")
    parser.add_argument("--allow-hardware", action="store_true")
    parser.add_argument("--output", type=Path, help="new directory of per-frame NPZ files")
    parser.add_argument("--frames", type=int, default=10)
    args = parser.parse_args(argv)
    config = load_config(args.check_config or args.capture)
    if args.check_config:
        if args.allow_hardware or args.output:
            parser.error("--check-config never opens hardware or writes capture files")
        print(json.dumps({"configuration": "VALID_MEASURED_INPUT", **sdk_inventory()}, sort_keys=True))
        return
    if not args.allow_hardware or args.output is None or not 1 <= args.frames <= 600:
        parser.error("--capture needs --allow-hardware, --output NEW_DIRECTORY and 1..600 --frames")
    args.output.mkdir(parents=True, exist_ok=False)
    capture = connect(config, allow_hardware=True)
    try:
        import numpy as np
        for k in range(args.frames):
            frame = capture.frames(deadline=time.monotonic() + 2.0, service=lambda: None)
            arrays = {key: frame[key] for key in ("rgb", "depth_z_m", "invalid", "coverage", "rgb_t", "depth_t",
                                                  "now_s")}
            metadata = {key: value for key, value in frame.items() if key not in arrays}
            arrays["camera_metadata"] = json.dumps(metadata, sort_keys=True)
            np.savez_compressed(args.output / ("frame_%05d.npz" % k), **arrays)
        print(json.dumps({"capture_frames": args.frames, "policy_executed": False,
                          "vehicle_commands_sent": False, "hardware_qualification": "NOT_TESTED"}))
    finally:
        capture.close()


if __name__ == "__main__":
    main()
