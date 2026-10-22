"""Q02-A input contract: Frozen F frames, sensor models and an immutable evidence input.

All constants are copied from the audited production source (Q02A_SPEC.md §2) and are
checked against a Q01 packet's calibration tuple when one is supplied.
"""
from dataclasses import dataclass
import math

import numpy as np


# Frames. WORLD: simulator world, z up. YAW_VEHICLE: level frame at the vehicle centre rotated by
# yaw only, +x forward, +y left, +z up (robot_vehicle_orientation). CAMERA: YAW_VEHICLE axes,
# origin at CAMERA_OFFSET_M; pixel u grows to the right (-y), v grows down (-z). LIDAR: YAW_VEHICLE
# axes and origin (yaw_only_attach, zero mount offset).
WORLD, YAW_VEHICLE, CAMERA, LIDAR = "WORLD", "YAW_VEHICLE", "CAMERA_PINHOLE_YAW_LEVEL", "LIDAR_YAW_LEVEL"
DEPTH_CONVENTION = "EUCLIDEAN_RAY_RANGE_M"     # range along the unit pixel ray, not optical z
LIDAR_LAYOUT = "LIDAR_ZERO_WIDTH_4x72_5deg_10deg"
CAMERA_OFFSET_M = (0.10, 0.0, 0.03)
ACCEPTED_CLOCK_PROVENANCE = ("DECLARED_SIM_SCHEDULE",)
EVIDENCE_CONTRACT_VERSION = "Q02A-v2.5"   # v1 (a5fee45) read depth as 20 m range and 0 m as a hit; v2 = 87cced6, v2.1 = aab9b31,
                                           # v2.2 = 94acd1a, v2.3 = 17cf5e9, v2.4 = 8eea494

# Per-sample semantics, shared by camera depth and LiDAR.
VALID_HIT, NO_RETURN, NEAR_CLIPPED, INVALID_SAMPLE = "VALID_HIT", "NO_RETURN", "NEAR_CLIPPED", "INVALID"


@dataclass(frozen=True)
class CameraParams:
    """Same field names as motar_eval.boia.types.CameraParams, without importing torch."""
    hfov: float
    vfov: float
    fx: float
    fy: float
    cx: float
    cy: float
    camera_offset: tuple
    target_radius: float = 0.15
    max_camera_range: float = 20.0
    min_pixels: float = 2.0


def frozen_f_camera(width=160, height=90, hfov_deg=87.0, vfov_deg=58.0):
    hfov, vfov = math.radians(hfov_deg), math.radians(vfov_deg)
    return CameraParams(hfov=hfov, vfov=vfov, fx=width / (2.0 * math.tan(hfov / 2)),
                        fy=height / (2.0 * math.tan(vfov / 2)), cx=(width - 1) / 2.0,
                        cy=(height - 1) / 2.0, camera_offset=CAMERA_OFFSET_M, target_radius=0.15,
                        max_camera_range=20.0, min_pixels=2.0)


@dataclass(frozen=True)
class LidarModel:
    """warp_lidar.initialize_ray_vectors with the navrl config: row i, column j."""
    vbeams: int = 4
    hbeams: int = 72
    az_max_deg: float = 180.0
    az_min_deg: float = -175.0
    el_max_deg: float = 20.0
    el_min_deg: float = -10.0
    min_range_m: float = 0.2
    max_range_m: float = 12.0
    target_proxy_radius_m: float = 0.20   # LiDAR target sphere, not the camera's 0.15 m

    def azimuth_rad(self):
        j = np.arange(self.hbeams)
        return np.radians(self.az_max_deg - (self.az_max_deg - self.az_min_deg) * j / (self.hbeams - 1))

    def elevation_rad(self):
        i = np.arange(self.vbeams)
        return np.radians(self.el_max_deg - (self.el_max_deg - self.el_min_deg) * i / (self.vbeams - 1))

    def spacing_rad(self):
        return (math.radians((self.az_max_deg - self.az_min_deg) / (self.hbeams - 1)),
                math.radians((self.el_max_deg - self.el_min_deg) / (self.vbeams - 1)))


FROZEN_F_CAMERA = frozen_f_camera()
FROZEN_F_LIDAR = LidarModel()


@dataclass(frozen=True)
class SensorProfile:
    """How the delivered depth/LiDAR samples were produced (audited source, Q02A_SPEC §2).

    Camera depth = obstacle depth rendered on a native 40x24 ray grid from the camera origin
    (environment mesh only, never the target), no hit -> far plane 10 m, bilinearly upsampled
    (align_corners=False) to 160x90, clamped [0, 10]; distractor and then target pixels are pasted
    over it at full resolution with their exact depth (navrl_detector.py:47-49, 875-886, 927-974).
    LiDAR = getter output: hits in [0.2, 12) m, no hit -> 12 m, nearer than 0.2 m -> 0.0,
    NaN/Inf -> 12 m (warp_sensor.py:233-238, navrl_lidar_config.py:71-72, navrl_task.py:6013-6017).
    The obstacle far plane and the distractor count are not in the Q01 calibration tuple; they are
    declared here for the pinned Frozen F profile and checked against config source text in tests.
    """
    obstacle_native_wh: tuple = (40, 24)
    obstacle_far_m: float = 10.0
    depth_near_clip_m: float = 0.05        # DECLARED: the obstacle kernel has no near clip (navrl_detector.py:28-49)
    obstacle_contact_m: float = 0.10       # DECLARED: a native cell nearer than this = contact; the grid is not read
    lidar_no_return_m: float = 12.0
    lidar_near_clip_value: float = 0.0
    distractors: int = 0
    min_obstacle_width_m: float = 0.40     # bar footprint random in [0.4, 0.8] m (env_object_config.py:796-797); 2 m tall,
                                           # 3 m with NAVRL_BAR_POOL=bars_h3 as pinned by Q01 v2 (:813-818)


FROZEN_F_PROFILE = SensorProfile()


@dataclass(frozen=True)
class Clock:
    """Declared simulation clock of one sensor delivery; None means not published."""
    capture_s: object
    delivery_s: object
    provenance: str = "MISSING"
    frame: str = YAW_VEHICLE
    depth_convention: str = DEPTH_CONVENTION


def _frozen(value, shape, dtype):
    a = np.array(value, dtype=dtype, copy=True)
    if a.shape != shape:
        raise ValueError("SHAPE_MISMATCH %s != %s" % (a.shape, shape))
    a.setflags(write=False)
    return a


@dataclass(frozen=True)
class EvidenceInput:
    """Sensor-side content only. There is no field for target/obstacle truth by construction."""
    rgb: np.ndarray            # (3, H, W) in [0, 1]
    depth: np.ndarray          # (H, W) metres, Euclidean ray range
    lidar: np.ndarray          # (4, 72) metres, getter-sanitized
    camera_detected: bool      # this delivery's pixel detection (not tracker state)
    fused_update: bool         # tracker received a camera OR LiDAR correction this step
    confidence: float          # segmenter score x pixel support; NOT a probability
    track_mean: np.ndarray     # (6,) world position m, velocity m/s
    track_cov: np.ndarray      # (6, 6)
    track_age_s: float         # seconds since the last accepted fused correction
    track_active: bool
    position: np.ndarray       # (3,) world, own state
    velocity: np.ndarray       # (3,) world
    vehicle_quat: np.ndarray   # (4,) xyzw, yaw-only
    camera_clock: Clock
    lidar_clock: Clock
    decision_s: float
    calibration: tuple = ()

    @classmethod
    def build(cls, *, rgb, depth, lidar, camera_detected, fused_update, confidence, track_mean, track_cov,
              track_age_s, track_active, position, velocity, vehicle_quat, camera_clock, lidar_clock,
              decision_s, calibration=(), camera=FROZEN_F_CAMERA, lidar_model=FROZEN_F_LIDAR):
        h, w = int(round(2 * camera.cy + 1)), int(round(2 * camera.cx + 1))
        return cls(_frozen(rgb, (3, h, w), np.float32), _frozen(depth, (h, w), np.float32),
                   _frozen(np.asarray(lidar, dtype=np.float32).reshape(lidar_model.vbeams, lidar_model.hbeams),
                           (lidar_model.vbeams, lidar_model.hbeams), np.float32),
                   bool(camera_detected), bool(fused_update), float(confidence),
                   _frozen(track_mean, (6,), np.float64), _frozen(track_cov, (6, 6), np.float64),
                   float(track_age_s), bool(track_active), _frozen(position, (3,), np.float64),
                   _frozen(velocity, (3,), np.float64), _frozen(vehicle_quat, (4,), np.float64),
                   camera_clock, lidar_clock, float(decision_s), tuple(calibration))

    @classmethod
    def from_arrays(cls, values, camera_clock, lidar_clock, decision_s, calibration=()):
        """`values`: the Q01 packet whitelist (e.g. a saved sensor_NNN.npz). Extra keys are refused."""
        allowed = {"rgb", "depth", "lidar", "raw_lidar", "camera", "fused", "mean", "covariance", "age",
                   "active", "position", "velocity", "vehicle_quat", "body_quat", "body_angvel", "confidence"}
        extra = set(values) - allowed
        if extra:
            raise ValueError("NON_SENSOR_FIELDS_REFUSED %s" % sorted(extra))
        v = {k: np.asarray(values[k]) for k in values}
        return cls.build(rgb=v["rgb"].reshape(v["rgb"].shape[-3:]), depth=v["depth"].reshape(v["depth"].shape[-2:]),
                         lidar=v["lidar"].reshape(-1), camera_detected=bool(v["camera"].reshape(-1)[0]),
                         fused_update=bool(v["fused"].reshape(-1)[0]), confidence=float(v["confidence"].reshape(-1)[0]),
                         track_mean=v["mean"].reshape(6), track_cov=v["covariance"].reshape(6, 6),
                         track_age_s=float(v["age"].reshape(-1)[0]), track_active=bool(v["active"].reshape(-1)[0]),
                         position=v["position"].reshape(3), velocity=v["velocity"].reshape(3),
                         vehicle_quat=v["vehicle_quat"].reshape(4), camera_clock=camera_clock,
                         lidar_clock=lidar_clock, decision_s=decision_s, calibration=calibration)

    @classmethod
    def from_packet(cls, packet):
        """A Q01 motar_eval.boia_shadow.SensorPacket (already a whitelisted immutable copy)."""
        values = {name: packet.tensor(name).array() for name, _ in packet.values}
        clock = lambda t: Clock(t.capture_sim_s, t.delivery_sim_s, t.provenance, t.frame, t.depth_convention)
        return cls.from_arrays(values, clock(packet.camera_time), clock(packet.lidar_time),
                               packet.camera_time.decision_sim_s, packet.calibration)
