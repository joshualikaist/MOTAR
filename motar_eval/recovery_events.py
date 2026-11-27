"""Evaluation-only recovery chronology from immutable, copied scalar observations.

This module does not inspect simulator state, control inputs, tensors or random generators.
Unknown geometry and unrecorded sensor provenance remain unknown. Intervals describe
post-acquisition observations, never a guarantee of physical target identity or safety.
"""
from dataclasses import asdict, dataclass
import copy
import hashlib
import json
import math
from typing import Optional

CONTRACT = "MOTAR_RECOVERY_EVENTS_P1A_V1"
FINAL_DIAGNOSTICS_CONTRACT = "MOTAR_FROZEN_F_FINAL_DIAGNOSTICS_V1"
NOT_RECORDED = "NOT_RECORDED"
_TIME_TOLERANCE_S = 1e-6


def _number(value, name, *, minimum=0.0):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + " must be a finite scalar number")
    if not math.isfinite(value) or value < minimum:
        raise ValueError(name + " must be finite and >= " + str(minimum))


def _text(value, name):
    if not isinstance(value, str) or not value:
        raise ValueError(name + " must be a nonempty string")


def _flag(value, name, *, optional=False):
    if not (isinstance(value, bool) or (optional and value is None)):
        raise ValueError(name + " must be a boolean" + (" or None" if optional else ""))


def _index(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(name + " must be a nonnegative integer")


@dataclass(frozen=True)
class FrameStamp:
    episode_id: str
    frame_index: int
    state_time_s: float
    capture_time_s: Optional[float] = None
    frame_token: Optional[str] = None

    def __post_init__(self):
        _text(self.episode_id, "episode_id")
        _index(self.frame_index, "frame_index")
        _number(self.state_time_s, "state_time_s")
        if self.capture_time_s is not None:
            _number(self.capture_time_s, "capture_time_s")
            if self.capture_time_s > self.state_time_s + _TIME_TOLERANCE_S:
                raise ValueError("capture time cannot follow state time")
        if self.frame_token is not None:
            _text(self.frame_token, "frame_token")


@dataclass(frozen=True)
class SensorFrame:
    camera_valid: bool
    lidar_valid: Optional[bool]
    fused_valid: bool
    track_active: bool
    track_age_s: Optional[float]
    track_memory_s: float = 5.0
    fused_sources_contract: Optional[str] = None
    camera_loss_cause: str = NOT_RECORDED
    track_age_dtype: str = NOT_RECORDED

    def __post_init__(self):
        for name in ("camera_valid", "fused_valid", "track_active"):
            _flag(getattr(self, name), name)
        _flag(self.lidar_valid, "lidar_valid", optional=True)
        if self.track_age_s is not None:
            _number(self.track_age_s, "track_age_s")
        _number(self.track_memory_s, "track_memory_s")
        if self.track_memory_s <= 0:
            raise ValueError("track_memory_s must be positive")
        if self.fused_sources_contract is not None:
            _text(self.fused_sources_contract, "fused_sources_contract")
        _text(self.camera_loss_cause, "camera_loss_cause")
        _text(self.track_age_dtype, "track_age_dtype")
        if self.fused_sources_contract == FINAL_DIAGNOSTICS_CONTRACT:
            if self.lidar_valid is None:
                raise ValueError("verified final diagnostics require a recorded LiDAR flag")
            if self.fused_valid != (self.camera_valid or self.lidar_valid):
                raise ValueError("fused validity disagrees with verified final sensor sources")


@dataclass(frozen=True)
class EvaluationGeometry:
    target_center_in_fov: Optional[bool]
    target_center_in_range: Optional[bool]
    center_los_blocked: Optional[bool]
    geometry_contract: str
    rendered_target_visible_pixels: Optional[int] = None
    full_visual_occlusion: str = NOT_RECORDED
    frame_index: Optional[int] = None
    state_time_s: Optional[float] = None
    measurement_alignment: str = NOT_RECORDED

    def __post_init__(self):
        for name in ("target_center_in_fov", "target_center_in_range", "center_los_blocked"):
            _flag(getattr(self, name), name, optional=True)
        _text(self.geometry_contract, "geometry_contract")
        if self.rendered_target_visible_pixels is not None:
            _index(self.rendered_target_visible_pixels, "rendered_target_visible_pixels")
        # This milestone has no target-alone silhouette support raster contract.
        if self.full_visual_occlusion != NOT_RECORDED:
            raise ValueError("full visual occlusion is unsupported by this centre-ray contract")
        if self.frame_index is not None:
            _index(self.frame_index, "geometry frame_index")
        if self.state_time_s is not None:
            _number(self.state_time_s, "geometry state_time_s")
        if self.measurement_alignment not in (NOT_RECORDED, "ZERO_LATENCY_RENDER_DELIVERY"):
            raise ValueError("unsupported measurement/render alignment contract")


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode("utf-8")).hexdigest()


def _identity(value):
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError("run_identity must be a scalar mapping")
    for key, item in value.items():
        _text(key, "run identity key")
        if item is not None and not isinstance(item, (str, bool, int, float)):
            raise ValueError("run identity values must be copied JSON scalars")
        if isinstance(item, float) and not math.isfinite(item):
            raise ValueError("run identity values must be finite")
    return copy.deepcopy(value)


class RecoveryEpisodeEvents:
    """One episode; observe each consumed frame exactly once, before its action.

    Eligibility is established by a camera-valid active fused track and applies only
    to later frames. The caller supplies actual tracker age, including its dtype;
    elapsed evaluator time never substitutes for the tracker expiry condition.
    """

    def __init__(self, episode_id, *, dt_s=0.1, run_identity=None,
                 scene_manifest_hash=NOT_RECORDED, pre_loss_actor_prefix_hash=NOT_RECORDED):
        _text(episode_id, "episode_id")
        _number(dt_s, "dt_s")
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        _text(scene_manifest_hash, "scene_manifest_hash")
        _text(pre_loss_actor_prefix_hash, "pre_loss_actor_prefix_hash")
        self.episode_id = episode_id
        self.dt_s = dt_s
        self.run_identity = _identity(run_identity)
        self.scene_manifest_hash = scene_manifest_hash
        self.pre_loss_actor_prefix_hash = pre_loss_actor_prefix_hash
        self.ever_acquired = False
        self.camera_started = False
        self.last_index = -1
        self.last_time = None
        self.last_capture_time = None
        self.previous_track = False
        self.open_runs = {}
        self.rows = []
        self.frames = 0
        self.closed = False
        self._tokens = set()
        self.prefix_hash = _digest({"contract": CONTRACT, "episode_id": episode_id,
                                    "dt_s": dt_s, "run_identity": self.run_identity,
                                    "scene_manifest_hash": scene_manifest_hash,
                                    "pre_loss_actor_prefix_hash": pre_loss_actor_prefix_hash})

    def _end_run(self, name, time_s, boundary, end_frame=None):
        start = self.open_runs.pop(name, None)
        if start is None:
            return None
        row = {"event": name, "episode_id": self.episode_id,
               "start_frame": start[0], "start_time_s": start[1],
               "end_frame": end_frame, "end_time_s": time_s,
               "duration_s": time_s - start[1], "boundary": boundary,
               "censored": boundary != "observed_end"}
        self.rows.append(row)
        return row

    def observe(self, stamp: FrameStamp, sensor: SensorFrame, geometry: EvaluationGeometry):
        if not isinstance(stamp, FrameStamp) or not isinstance(sensor, SensorFrame) or not isinstance(geometry, EvaluationGeometry):
            raise ValueError("observe requires immutable scalar input records")
        if self.closed or stamp.episode_id != self.episode_id:
            raise ValueError("episode is closed or frame belongs to a different episode")
        if stamp.frame_index != self.last_index + 1:
            raise ValueError("consumed frame indices must be consecutive and never duplicated")
        expected_time = 0.0 if self.last_time is None else self.last_time + self.dt_s
        if abs(stamp.state_time_s - expected_time) > _TIME_TOLERANCE_S:
            raise ValueError("state time must start at zero and follow the physical dt")
        if self.last_time is not None and stamp.state_time_s <= self.last_time:
            raise ValueError("state time must increase strictly")
        if stamp.frame_token is not None and stamp.frame_token in self._tokens:
            raise ValueError("consumed frame token must be unique within an episode")
        if (stamp.capture_time_s is not None and self.last_capture_time is not None
                and stamp.capture_time_s < self.last_capture_time - _TIME_TOLERANCE_S):
            raise ValueError("recorded capture time cannot move backwards")
        if geometry.frame_index is not None and geometry.frame_index != stamp.frame_index:
            raise ValueError("geometry frame index does not match consumed state")
        if geometry.state_time_s is not None and abs(geometry.state_time_s - stamp.state_time_s) > _TIME_TOLERANCE_S:
            raise ValueError("geometry timestamp does not match consumed state")

        eligible = self.ever_acquired
        fov, within_range, blocked = (geometry.target_center_in_fov,
                                     geometry.target_center_in_range, geometry.center_los_blocked)
        qualified = None if None in (fov, within_range, blocked) else fov and within_range and blocked
        aligned = geometry.measurement_alignment == "ZERO_LATENCY_RENDER_DELIVERY"
        signals = {"camera_measurement_loss": not sensor.camera_valid,
                   "fused_measurement_loss": not sensor.fused_valid,
                   "track_inactive": not sensor.track_active,
                   "fov_exit": None if fov is None else not fov,
                   "range_exit": None if within_range is None else not within_range,
                   "mesh_center_los_blocked": blocked,
                   "center_los_occlusion": qualified,
                   "post_acquisition_occlusion_candidate": (False if not eligible else
                       None if qualified is None or not aligned else qualified and not sensor.camera_valid)}
        emitted_start = len(self.rows)
        resumed = []
        for name, value in signals.items():
            if not eligible or value is None:
                unknown_boundary = ("alignment_unknown" if name == "post_acquisition_occlusion_candidate"
                                    and qualified is not None and not aligned else "geometry_unknown")
                self._end_run(name, stamp.state_time_s,
                              unknown_boundary if value is None else "ineligible", stamp.frame_index)
            elif value:
                self.open_runs.setdefault(name, (stamp.frame_index, stamp.state_time_s))
            else:
                ended = self._end_run(name, stamp.state_time_s, "observed_end", stamp.frame_index)
                if ended is not None and name in ("camera_measurement_loss", "fused_measurement_loss", "track_inactive"):
                    resumed.append((name, ended))
        for name, ended in resumed:
            event = {"camera_measurement_loss": "camera_measurement_reacquired",
                     "fused_measurement_loss": "fused_measurement_reacquired",
                     "track_inactive": "track_reacquired"}[name]
            self.rows.append({"event": event, "episode_id": self.episode_id,
                              "frame_index": stamp.frame_index, "time_s": stamp.state_time_s,
                              "loss_start_frame": ended["start_frame"],
                              "latency_s": ended["duration_s"],
                              "physical_target_identity": NOT_RECORDED})
        if eligible and self.previous_track and not sensor.track_active:
            # Under the frozen source contract a final valid camera or LiDAR update
            # requires an active tracker. Such a contradictory row cannot prove expiry.
            inconsistent_measurement = (sensor.fused_sources_contract == FINAL_DIAGNOSTICS_CONTRACT
                                        and sensor.fused_valid)
            expired = (sensor.track_age_s is not None and sensor.track_age_s > sensor.track_memory_s
                       and not inconsistent_measurement)
            self.rows.append({"event": "track_expiry_transition" if expired else "track_deactivation",
                              "episode_id": self.episode_id, "frame_index": stamp.frame_index,
                              "time_s": stamp.state_time_s, "actual_track_age_s": sensor.track_age_s,
                              "actual_track_memory_s": sensor.track_memory_s,
                              "track_age_dtype": sensor.track_age_dtype,
                              "reason": "AGE_GT_MEMORY" if expired else "UNKNOWN"})
        acquired_now = sensor.camera_valid and sensor.fused_valid and sensor.track_active
        self.camera_started |= acquired_now
        self.ever_acquired |= acquired_now
        self.previous_track = sensor.track_active
        self.last_index, self.last_time = stamp.frame_index, stamp.state_time_s
        if stamp.capture_time_s is not None:
            self.last_capture_time = stamp.capture_time_s
        if stamp.frame_token is not None:
            self._tokens.add(stamp.frame_token)
        self.frames += 1
        row = {"contract": CONTRACT, **asdict(stamp), **asdict(sensor),
               "geometry_contract": geometry.geometry_contract,
               "target_center_in_fov": fov, "target_center_in_range": within_range,
               "center_los_blocked": blocked,
               "rendered_target_visible_pixels": geometry.rendered_target_visible_pixels,
               "full_visual_occlusion": geometry.full_visual_occlusion,
               "geometry_frame_index": geometry.frame_index,
               "geometry_state_time_s": geometry.state_time_s,
               "measurement_alignment": geometry.measurement_alignment,
               "eligible_before_frame": eligible, "ever_acquired": self.ever_acquired,
               "initial_search": not eligible and not acquired_now,
               "fused_source_identity_verified": sensor.fused_sources_contract == FINAL_DIAGNOSTICS_CONTRACT,
               "lidar_valid_status": NOT_RECORDED if sensor.lidar_valid is None else "RECORDED",
               "track_age_status": NOT_RECORDED if sensor.track_age_s is None else "RECORDED",
               "capture_time_status": NOT_RECORDED if stamp.capture_time_s is None else "RECORDED",
               "safe_abort": NOT_RECORDED, **signals,
               "newly_emitted_events": copy.deepcopy(self.rows[emitted_start:])}
        self.prefix_hash = _digest({"previous_prefix_hash": self.prefix_hash, "observation": row})
        row["prefix_hash"] = self.prefix_hash
        return row

    def finish(self, terminal_time_s, outcome=None, *, terminal_kind="episode_end"):
        if self.closed or terminal_kind not in ("episode_end", "run_shutdown", "reset"):
            raise ValueError("episode already closed or unsupported terminal kind")
        if terminal_kind == "episode_end" and outcome not in ("capture", "crash", "timeout"):
            raise ValueError("episode_end requires the actual resolved task outcome")
        if terminal_kind != "episode_end" and outcome is not None:
            raise ValueError("shutdown/reset cannot invent an episode outcome")
        _number(terminal_time_s, "terminal_time_s")
        if self.last_time is not None and terminal_time_s < self.last_time:
            raise ValueError("terminal time precedes consumed observation")
        boundary = "terminal_" + outcome if terminal_kind == "episode_end" else terminal_kind
        emitted_start = len(self.rows)
        for name in list(self.open_runs):
            self._end_run(name, terminal_time_s, boundary)
        self.closed = True
        intervals = copy.deepcopy(self.rows)
        return {"contract": CONTRACT, "episode_id": self.episode_id, "frames": self.frames,
                "ever_acquired": self.ever_acquired, "outcome": outcome,
                "terminal_kind": terminal_kind, "terminal_time_s": terminal_time_s,
                "safe_abort": NOT_RECORDED, "run_identity": copy.deepcopy(self.run_identity),
                "dt_s": self.dt_s, "scene_manifest_hash": self.scene_manifest_hash,
                "pre_loss_actor_prefix_hash": self.pre_loss_actor_prefix_hash,
                "intervals": intervals, "events": copy.deepcopy(intervals),
                "newly_emitted_events": copy.deepcopy(self.rows[emitted_start:]),
                "prefix_hash": self.prefix_hash}
