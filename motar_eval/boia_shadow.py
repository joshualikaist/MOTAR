"""Passive evaluation only. Immutable sensor bytes, strict phase joins, no action sink.

Importing this module starts neither a simulator nor CUDA. Only EvaluatorTap holds a
task reference; neither packets, diagnostics nor the logger receive that reference.
The first qualification intentionally supports N=1 and the frozen OFF sensor lineage.

`drive` is the one player loop of every Q01 stage. The S1_NATIVE stage runs it with no
EvaluatorTap/ContactChain; its HostTrace is the read-only baseline the observer stages
are compared against. Host-side reads exist in every stage, NATIVE included.
"""
from dataclasses import dataclass, fields
import hashlib
import inspect
import json
import math
from pathlib import Path
import time

import numpy as np
import torch

from .boia.access import read_online
from .boia.types import BoiaParams
from .boia_research.evidence import build_hypotheses
from .boia_research.types import MeasuredEvidence, Provenance


class IntegrityError(RuntimeError):
    pass


def require(condition, reason):
    if not condition:
        raise IntegrityError(reason)


def integer(x):
    return type(x) is int and x >= 0


def finite(x):
    try:
        return type(x) in (int, float) and math.isfinite(x)
    except (OverflowError, TypeError):
        return False


def digest_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class TensorBytes:
    """Transitive immutability: no Tensor/ndarray/storage is retained."""
    shape: tuple
    dtype: str
    data: bytes

    def __post_init__(self):
        require(type(self.shape) is tuple and all(integer(x) for x in self.shape), "IMMUTABLE_TENSOR_SHAPE_REQUIRED")
        require(type(self.dtype) is str and type(self.data) is bytes, "IMMUTABLE_TENSOR_BYTES_REQUIRED")
        dtype = np.dtype(self.dtype)
        require(dtype.kind in "biuf" and len(self.data) == math.prod(self.shape) * dtype.itemsize,
                "INVALID_TENSOR_DTYPE_OR_BYTE_COUNT")

    @classmethod
    def copy(cls, tensor, shape=None, finite_required=True):
        require(isinstance(tensor, torch.Tensor), "EXPECTED_SENSOR_TENSOR")
        if shape is not None:
            require(tuple(tensor.shape) == tuple(shape), "SENSOR_SHAPE_MISMATCH")
        if finite_required:
            require(bool(torch.isfinite(tensor).all()), "NONFINITE_SENSOR_OR_STATE")
        a = tensor.detach().clone().to("cpu").contiguous().numpy()
        return cls(tuple(a.shape), a.dtype.str, a.tobytes())

    def array(self):
        return np.frombuffer(self.data, dtype=np.dtype(self.dtype)).reshape(self.shape)

    def sha(self):
        h = hashlib.sha256(json.dumps([self.shape, self.dtype]).encode())
        h.update(self.data)
        return h.hexdigest()

    def record(self, values=False):
        out = dict(shape=self.shape, dtype=self.dtype, sha256=self.sha())
        if values:
            # Raw invalid ranges have a separate categorical record; never emit NaN JSON.
            require(bool(np.isfinite(self.array()).all()), "NONFINITE_JSON_VALUES")
            out["values"] = self.array().tolist()
        return out


@dataclass(frozen=True)
class JoinKey:
    env: int
    episode: int
    action: int
    observation: int
    decision: int

    def __post_init__(self):
        require(all(integer(getattr(self, f.name)) for f in fields(self)), "INVALID_JOIN_ID")
        require(self.env == 0, "ONLY_N1_QUALIFIED_BY_THIS_RUNNER")


@dataclass(frozen=True)
class Binding:
    source: str
    config: str
    checkpoint: str
    lineage: str = "off"

    def __post_init__(self):
        for text, width in ((self.source, 40), (self.config, 64), (self.checkpoint, 64)):
            require(type(text) is str and len(text) == width and
                    all(c in "0123456789abcdef" for c in text), "INVALID_SOURCE_CONFIG_CHECKPOINT_ID")
        require(type(self.lineage) is str and self.lineage == "off", "UNQUALIFIED_SENSOR_LINEAGE")


@dataclass(frozen=True)
class SensorTime:
    capture_sim_s: object
    delivery_sim_s: object
    decision_sim_s: float
    host_arrival_monotonic_s: float
    provenance: str = "MISSING"
    frame: str = "YAW_VEHICLE"
    depth_convention: str = "EUCLIDEAN_RAY_RANGE_M"

    def __post_init__(self):
        require(all(x is None or (finite(x) and x >= 0) for x in
                    (self.capture_sim_s, self.delivery_sim_s)), "SENSOR_CLOCK_SCALARS_ONLY")
        require(all(finite(x) and x >= 0 for x in (self.decision_sim_s, self.host_arrival_monotonic_s)),
                "FINITE_DECISION_AND_HOST_CLOCK_REQUIRED")
        require(all(type(x) is str for x in (self.provenance, self.frame, self.depth_convention)),
                "SENSOR_CLOCK_TAGS_NO_HANDLES")

    def reasons(self):
        if not all(finite(x) and x >= 0 for x in (self.capture_sim_s, self.delivery_sim_s,
                                                  self.decision_sim_s, self.host_arrival_monotonic_s)):
            return ("MISSING_SENSOR_TIMESTAMP",)
        if not self.capture_sim_s <= self.delivery_sim_s <= self.decision_sim_s:
            return ("SENSOR_CLOCK_ORDER_MISMATCH",)
        if self.provenance != "DECLARED_SIM_SCHEDULE":
            return ("UNQUALIFIED_TIMESTAMP_PROVENANCE",)
        if self.frame != "YAW_VEHICLE" or self.depth_convention != "EUCLIDEAN_RAY_RANGE_M":
            return ("FRAME_OR_DEPTH_CONVENTION_MISMATCH",)
        if self.decision_sim_s - self.capture_sim_s > 1e-9:
            return ("PINNED_SYNCHRONOUS_FRAME_STALE",)
        return ()


@dataclass(frozen=True)
class SensorValues:
    rgb: torch.Tensor
    depth: torch.Tensor
    lidar: torch.Tensor
    raw_lidar: torch.Tensor
    camera: torch.Tensor
    fused: torch.Tensor
    mean: torch.Tensor
    covariance: torch.Tensor
    age: torch.Tensor
    active: torch.Tensor
    position: torch.Tensor
    velocity: torch.Tensor
    vehicle_quat: torch.Tensor
    body_quat: torch.Tensor
    body_angvel: torch.Tensor
    confidence: torch.Tensor


SHAPES = ((1, 3, 90, 160), (1, 90, 160), (1, 288), (1, 1, 4, 72),
          (1,), (1,), (1, 6), (1, 6, 6), (1,), (1,), (1, 3), (1, 3),
          (1, 4), (1, 4), (1, 3), (1,))
BOOL_FIELDS = ("camera", "fused", "active")


def lidar_categories(raw):
    require(type(raw) is TensorBytes, "RAW_SENSOR_BYTES_REQUIRED")
    x = raw.array()
    out = np.full(x.shape, "UNKNOWN", dtype="<U20")
    out[~np.isfinite(x)] = "INVALID"
    out[np.isfinite(x) & (x <= 0)] = "NEAR_OR_CLIPPED"
    out[np.isfinite(x) & (x >= 1)] = "FAR_OR_NO_RETURN"
    out[np.isfinite(x) & (x > 0) & (x < 1)] = "FINITE_RETURN"
    return tuple(str(v) for v in out.reshape(-1))


@dataclass(frozen=True)
class SensorPacket:
    key: JoinKey
    binding: Binding
    camera_time: SensorTime
    lidar_time: SensorTime
    values: tuple
    calibration: tuple
    phase: str = "OBSERVATION_READY_PRE_ACTION"

    def __post_init__(self):
        require(type(self.key) is JoinKey and type(self.binding) is Binding and
                type(self.camera_time) is SensorTime and type(self.lidar_time) is SensorTime,
                "STRICT_PACKET_METADATA_REQUIRED")
        require(type(self.values) is tuple and all(type(x) is tuple and len(x) == 2 and type(x[0]) is str
                for x in self.values), "IMMUTABLE_SENSOR_FIELD_PAIRS_REQUIRED")
        require(tuple(k for k, v in self.values) ==
                tuple(f.name for f in fields(SensorValues)), "PACKET_WHITELIST_VIOLATION")
        require(all(type(v) is TensorBytes and v.shape == shape for (_, v), shape in zip(self.values, SHAPES)),
                "PACKET_TENSOR_SCHEMA_VIOLATION")
        require(type(self.calibration) is tuple and all(type(x) is str or finite(x) for x in self.calibration),
                "PACKET_OBJECT_REFERENCE_FORBIDDEN")
        require(type(self.phase) is str and self.phase == "OBSERVATION_READY_PRE_ACTION", "SENSOR_PHASE_MISMATCH")

    def tensor(self, name):
        return dict(self.values)[name]

    def fingerprint(self):
        # Host time is deliberately excluded from deterministic OFF/ON comparison.
        return digest_json(dict(key=self.key.__dict__, binding=self.binding.__dict__,
                                values={k: v.sha() for k, v in self.values},
                                camera_sim={k: v for k, v in self.camera_time.__dict__.items()
                                            if k != "host_arrival_monotonic_s"},
                                lidar_sim={k: v for k, v in self.lidar_time.__dict__.items()
                                           if k != "host_arrival_monotonic_s"},
                                calibration=self.calibration, phase=self.phase))


def make_packet(key, binding, camera_time, lidar_time, values, calibration):
    require(type(values) is SensorValues and type(key) is JoinKey and type(binding) is Binding,
            "STRICT_SENSOR_SCHEMA_REQUIRED")
    require(type(camera_time) is SensorTime and type(lidar_time) is SensorTime, "EXPLICIT_CLOCK_SCHEMA_REQUIRED")
    require(type(calibration) is tuple and all(type(x) in (str, int, float) for x in calibration),
            "CALIBRATION_MUST_BE_SCALAR_TUPLE_NO_HANDLES")
    packed = []
    for f, shape in zip(fields(SensorValues), SHAPES):
        value = getattr(values, f.name)
        require(isinstance(value, torch.Tensor), "SENSOR_FIELDS_CANNOT_CONTAIN_SOURCE_HANDLES")
        if f.name in BOOL_FIELDS:
            require(value.dtype == torch.bool, "BOOLEAN_SENSOR_FLAG_REQUIRED")
        else:
            require(torch.is_floating_point(value), "FLOAT_SENSOR_UNITS_REQUIRED")
        packed.append((f.name, TensorBytes.copy(value, shape, f.name != "raw_lidar")))
    p = SensorPacket(key, binding, camera_time, lidar_time, tuple(packed), calibration)
    for name in ("rgb", "confidence"):
        require(bool(((p.tensor(name).array() >= 0) & (p.tensor(name).array() <= 1)).all()), "INVALID_CAMERA_UNIT_RANGE")
    require(bool((p.tensor("depth").array() >= 0).all()), "NEGATIVE_DEPTH_METRES")
    require(bool(((p.tensor("lidar").array() >= 0) & (p.tensor("lidar").array() <= 12)).all()), "INVALID_LIDAR_METRES")
    require(bool((p.tensor("age").array() >= 0).all()), "NEGATIVE_TRACK_AGE")
    return p


@dataclass(frozen=True)
class Diagnostic:
    utility: str
    reason: str
    hypotheses: tuple
    evidence_reasons: tuple
    sensor_reasons: tuple
    safety: str = "INPUT_CERTIFICATE_UNAVAILABLE"
    selector: str = "NOT_EVALUATED"
    physical_safety: str = "NOT_ASSESSED"
    opportunity: str = "UNKNOWN"
    occlusion: str = "UNKNOWN"
    owner: str = "F"
    action_applied: bool = False


def diagnose(packet):
    require(type(packet) is SensorPacket, "BOIA_REJECTS_EVALUATOR_OR_GENERIC_DICT")
    # Actual measured error intervals, identity support and calibrated mission model
    # do not exist. Reuse the research refusal path; never manufacture point intervals.
    m = MeasuredEvidence((str(packet.key.env), str(packet.key.episode), packet.key.decision),
                         None, None, packet.camera_time.capture_sim_s,
                         packet.camera_time.delivery_sim_s, packet.camera_time.decision_sim_s,
                         None, Provenance(), max_age_s=0.)
    e = build_hypotheses(m, "FROZEN_F_OFF_UNCALIBRATED")
    return Diagnostic("ABSTAIN", "ACTUAL_MISSION_MODEL_UNAVAILABLE", e.labels, e.reasons,
                      tuple(sorted(set(packet.camera_time.reasons() + packet.lidar_time.reasons()))))


@dataclass(frozen=True)
class ActionRecord:
    key: JoinKey
    policy: TensorBytes
    actor_input: TensorBytes
    command: TensorBytes
    rng_before: str
    rng_after: str
    owner: str = "F"
    action_applied: bool = False

    def __post_init__(self):
        require(self.owner == "F" and self.action_applied is False, "SHADOW_MUST_NOT_APPLY_ACTION")


@dataclass(frozen=True)
class TruthRecord:
    """Evaluator-only. Never accepted by make_packet or diagnose."""
    key: JoinKey
    capture: bool
    crash: bool
    timeout: bool
    contact_now: bool
    height_now: bool
    oob_now: bool
    contact_substeps: int
    contact_peak_n: float
    substeps: int
    threshold_n: float
    state_sha: str
    reset_expected: bool
    phase: str = "POST_PHYSICS_PRE_RESET"


class EventLogger:
    """No task/model/controller handle, action callback or RNG generator."""
    def __init__(self, directory, max_bytes=256000000):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.max_bytes, self.bytes, self.sequence = int(max_bytes), 0, 0
        self.streams = {}
        try:
            for ns in ("sensors", "actions", "evaluator", "shadow", "parity", "events"):
                self.streams[ns] = (self.directory / (ns + ".jsonl")).open("x")
        except Exception:
            self.close()
            raise

    def _write(self, namespace, value):
        line = json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":")) + "\n"
        size = len(line.encode())
        require(self.bytes + size <= self.max_bytes, "LOGGER_OUTPUT_CAP")
        try:
            self.streams[namespace].write(line)
            self.streams[namespace].flush()
        except (OSError, ValueError) as exc:
            raise IntegrityError("FATAL_LOGGER_IO: " + str(exc)) from exc
        self.bytes += size

    def sensor(self, packet):
        require(type(packet) is SensorPacket, "LOGGER_SENSOR_SCHEMA_ONLY")
        self._write("sensors", dict(key=packet.key.__dict__, binding=packet.binding.__dict__,
                   phase=packet.phase, camera_time=packet.camera_time.__dict__, lidar_time=packet.lidar_time.__dict__,
                   calibration=packet.calibration, fingerprint=packet.fingerprint(),
                   values={k: v.record(k not in ("rgb", "depth", "raw_lidar")) for k, v in packet.values},
                   lidar_postwarp_categories=lidar_categories(packet.tensor("raw_lidar")),
                   return_provenance="POST_WARP_CENSORED_NOT_KERNEL_HIT_TYPE"))

    def diagnostic(self, key, diagnostic, latency_ms):
        require(type(key) is JoinKey and type(diagnostic) is Diagnostic, "LOGGER_DIAGNOSTIC_SCHEMA_ONLY")
        # diagnose() reads key and clocks only, never sensor values: its time says nothing
        # about a planner that would consume the packet.
        self._write("shadow", dict(key=key.__dict__, diagnostic=diagnostic.__dict__,
                                    compute_ms=latency_ms, full_planner="SKIPPED_UNSUPPORTED",
                                    latency_scope="DIAGNOSTIC_ONLY_NOT_INFORMATIVE_FOR_FULL_PLANNER",
                                    diagnose_reads_sensor_values=False))

    def action(self, record):
        require(type(record) is ActionRecord, "LOGGER_ACTION_SCHEMA_ONLY")
        self._write("actions", dict(key=record.key.__dict__, policy=record.policy.record(True),
                    actor_input=record.actor_input.record(), command=record.command.record(True),
                    rng_before=record.rng_before, rng_after=record.rng_after,
                    action_phase="F_ACTION_BOUND_AT_TASK_STEP_ENTRY",
                    command_phase="TRANSFORMED_COMMAND_PRE_PHYSICS",
                    owner=record.owner, shadow_action_applied=record.action_applied))

    def truth(self, record, episode_summary):
        require(type(record) is TruthRecord, "LOGGER_EVALUATOR_SCHEMA_ONLY")
        self._write("evaluator", dict(record=dict(record.__dict__, key=record.key.__dict__),
                                       episode=episode_summary))

    def parity(self, value):
        self._write("parity", value)

    def event(self, kind, **value):
        self._write("events", dict(kind=kind, **value))

    def snapshot(self, packet, ordinal):
        require(type(packet) is SensorPacket, "LOGGER_SENSOR_SCHEMA_ONLY")
        path = self.directory / ("sensor_%03d.npz" % ordinal)
        with path.open("xb") as fh:
            np.savez_compressed(fh, **{k: v.array() for k, v in packet.values})
        self.bytes += path.stat().st_size
        require(self.bytes <= self.max_bytes, "LOGGER_OUTPUT_CAP")

    def close(self):
        for stream in self.streams.values():
            if not stream.closed:
                stream.close()


class Recorder:
    """Explicit N1 protocol; bookkeeping runs every action, diagnostics only at stride."""
    def __init__(self, logger, shadow, stride=5, snapshot_cap=0, reference=None):
        require(type(stride) is int and stride >= 1, "INVALID_SHADOW_STRIDE")
        self.logger, self.shadow, self.stride = logger, bool(shadow), stride
        self.episode, self.action_index, self.generation, self.completed = -1, 0, -1, 0
        self.pending, self.inflight, self.command = None, None, None
        self.last_terminal, self.actor, self.policy = False, None, None
        self.snapshot_cap, self.snapshots = snapshot_cap, 0
        self.contact_total, self.height_seen, self.oob_seen = 0, False, False
        self.latencies, self.slow_streak, self.episodes_ended = [], 0, 0
        self.reference = reference

    def _match(self, name, value):
        if self.reference is not None:
            require(self.completed < len(self.reference), "OFF_ON_TRACE_LENGTH_MISMATCH")
            require(digest_json(value) == digest_json(self.reference[self.completed][name]),
                    "OFF_ON_MISMATCH_" + name)

    def reset(self):
        require(self.inflight is None, "RESET_BEFORE_TERMINAL_RECORD")
        if self.episode >= 0:
            require(self.last_terminal, "RESET_ORPHAN_MISSING_TERMINAL")
        self.episode += 1
        self.action_index = 0
        self.pending = None
        self.contact_total, self.height_seen, self.oob_seen = 0, False, False
        self.last_terminal = False
        self.logger.event("RESET_BOUNDARY", phase="TASK_RESET_BEGIN", env=0,
                          episode=self.episode, completed_actions=self.completed)

    def observation(self, packet):
        require(type(packet) is SensorPacket, "STRICT_SENSOR_PACKET_REQUIRED")
        require(self.inflight is None and self.pending is None, "DUPLICATE_OR_OUT_OF_ORDER_OBSERVATION")
        expected = JoinKey(0, self.episode, self.action_index, self.generation + 1, self.completed)
        require(packet.key == expected, "OBSERVATION_EPISODE_GENERATION_MISMATCH")
        require(not self.last_terminal, "MISSING_RESET_AFTER_TERMINAL")
        self.pending, self.generation = packet, packet.key.observation

    def begin(self, policy, actor, rng_before):
        require(self.pending is not None and self.inflight is None, "ACTION_WITHOUT_CURRENT_OBSERVATION")
        self.inflight, self.pending = self.pending, None
        self.policy = TensorBytes.copy(policy, (1, 4))
        self.actor = TensorBytes.copy(actor, (1, 898))
        self.rng_before, self.command = rng_before, None
        for name, value in (("key", self.inflight.key.__dict__), ("sensor", self.inflight.fingerprint()),
                            ("actor", self.actor.sha()), ("policy", self.policy.sha()), ("rng_before", rng_before)):
            self._match(name, value)
        sampled = self.completed % self.stride == 0
        if sampled:
            self.logger.sensor(self.inflight)
            if self.snapshots < self.snapshot_cap:
                self.logger.snapshot(self.inflight, self.snapshots)
                self.snapshots += 1
            if self.shadow:
                t0 = time.perf_counter()
                d = diagnose(self.inflight)
                ms = (time.perf_counter() - t0) * 1000
                self.latencies.append(ms)
                self.slow_streak = self.slow_streak + 1 if ms > 500 else 0
                require(ms <= 2000 and self.slow_streak < 3, "SHADOW_LATENCY_WATCHDOG")
                self.logger.diagnostic(self.inflight.key, d, ms)

    def transformed(self, actual):
        require(self.inflight is not None and self.command is None, "DUPLICATE_OR_UNBOUND_COMMAND")
        self.command = TensorBytes.copy(actual, (1, 4))
        self._match("command", self.command.sha())

    def finish(self, truth, rng_after, state_groups):
        require(type(truth) is TruthRecord and self.inflight is not None and self.command is not None,
                "MISSING_ACTION_COMMAND_OR_TERMINAL_RECORD")
        require(truth.key == self.inflight.key and truth.phase == "POST_PHYSICS_PRE_RESET", "TRUTH_JOIN_PHASE_MISMATCH")
        require(sum((truth.capture, truth.crash, truth.timeout)) <= 1, "RESOLVED_OUTCOMES_NOT_EXCLUSIVE")
        require(truth.substeps == 10 and truth.threshold_n == .05, "CONTACT_OBSERVATION_CONTRACT_MISMATCH")
        terminal = truth.capture or truth.crash or truth.timeout
        require(truth.reset_expected == terminal, "TERMINAL_RESET_CONTRACT_MISMATCH")
        self.contact_total += truth.contact_substeps
        self.height_seen |= truth.height_now
        self.oob_seen |= truth.oob_now
        cf = bool(truth.capture and self.contact_total == 0 and not self.height_seen and not self.oob_seen)
        summary = dict(contact_substeps=self.contact_total, height_seen=self.height_seen, oob_seen=self.oob_seen,
                       terminal_reason="capture" if truth.capture else "crash" if truth.crash else "timeout" if truth.timeout else "CONTINUE",
                       historical_capture=truth.capture, simultaneous_capture_failure=bool(truth.capture and
                       (truth.contact_now or truth.height_now or truth.oob_now or truth.contact_substeps > 0)),
                       collision_free_capture=cf if terminal else None,
                       qualification="SUBSTEP_CONTACT_AND_ACTION_BOUNDARY_HEIGHT_OOB_ONLY",
                       contact_threshold_n=truth.threshold_n)
        action = ActionRecord(truth.key, self.policy, self.actor, self.command, self.rng_before, rng_after)
        self.logger.action(action)
        self.logger.truth(truth, summary)
        parity = dict(key=truth.key.__dict__, sensor=self.inflight.fingerprint(), actor=self.actor.sha(),
                      policy=self.policy.sha(), command=self.command.sha(), rng_before=self.rng_before,
                      rng_after=rng_after, state=state_groups, outcome=(truth.capture, truth.crash, truth.timeout),
                      contact=(truth.contact_substeps, truth.contact_peak_n, truth.height_now, truth.oob_now),
                      reset_expected=terminal)
        for name, value in parity.items():
            self._match(name, value)
        self.last_parity = parity  # completed after original step returns its next/reset observation
        self.inflight, self.command = None, None
        self.last_terminal = terminal
        self.completed += 1
        self.action_index += 1
        self.episodes_ended += int(terminal)
        return parity

    def censor(self, reason):
        require(self.inflight is None, "CENSOR_WITH_UNFINISHED_ACTION")
        if self.reference is not None:
            require(self.completed == len(self.reference), "OFF_ON_TRACE_LENGTH_MISMATCH")
        ended = self.last_terminal or (self.episodes_ended > 0 and self.action_index == 0)
        self.logger.event("CLOSED_AFTER_TERMINAL" if ended else "CENSORED",
                          episode=self.episode, actions=self.action_index, reason=reason, timeout=False)


def rng_fingerprint(device=None):
    """Read states only; never draw, seed, fork, or initialize a CUDA context."""
    import random
    h = hashlib.sha256(repr(random.getstate()).encode())
    n = np.random.get_state()
    h.update(n[0].encode()); h.update(n[1].tobytes()); h.update(repr(n[2:]).encode())
    h.update(torch.get_rng_state().numpy().tobytes())
    if device is not None and torch.device(device).type == "cuda":
        require(torch.cuda.is_initialized(), "CUDA_STATE_MUST_ALREADY_EXIST")
        h.update(torch.cuda.get_rng_state(device).cpu().numpy().tobytes())
    return h.hexdigest()


def runtime_attestation(device=None):
    """Read-only record of numerics settings. Frozen F's process sets them; this never does."""
    b = torch.backends
    out = dict(torch=torch.__version__, cuda=torch.version.cuda,
               cudnn=b.cudnn.version() if b.cudnn.is_available() else None,
               cudnn_benchmark=bool(b.cudnn.benchmark), cudnn_deterministic=bool(b.cudnn.deterministic),
               cudnn_allow_tf32=bool(b.cudnn.allow_tf32), matmul_allow_tf32=bool(b.cuda.matmul.allow_tf32),
               float32_matmul_precision=torch.get_float32_matmul_precision(),
               deterministic_algorithms=bool(torch.are_deterministic_algorithms_enabled()),
               device_name=None, device_capability=None)
    if device is not None and torch.device(device).type == "cuda":
        require(torch.cuda.is_initialized(), "CUDA_STATE_MUST_ALREADY_EXIST")
        out.update(device_name=torch.cuda.get_device_name(torch.device(device)),
                   device_capability=list(torch.cuda.get_device_capability(torch.device(device))))
    return out


HOOK_MARK = "_boia_shadow_hook"


def hook_targets(task):
    """The instance attributes EvaluatorTap wraps, in installation order."""
    return (("perception.observe", task.perception, "observe"),
            ("detector.render_raw_rgbd", task.detector, "render_raw_rgbd"),
            ("warp_sensor.capture", task.sim_env.robot_manager.warp_sensor.sensor, "capture"),
            ("task.reset_idx", task, "reset_idx"),
            ("task.transform_action_to_command", task, "transform_action_to_command"),
            ("task.compute_state_reward_and_terminations", task, "compute_state_reward_and_terminations"),
            ("sim_env.post_reward_calculation_step", task.sim_env, "post_reward_calculation_step"),
            ("task.step", task, "step"))


def hook_audit(task):
    """Which owned hooks are live now. Read-only; proves the NATIVE stage runs without them."""
    from .e_l0 import ContactChain
    out = {label: bool(getattr(getattr(obj, name), HOOK_MARK, False)) for label, obj, name in hook_targets(task)}
    out["contact_chain"] = isinstance(task.sim_env.physics_step_callback, ContactChain)
    return out


class HostTrace:
    """Player-loop record written identically by every stage, NATIVE included.

    Holds no task/model handle. Rows are evaluator-side; online code never reads them.
    """
    def __init__(self, directory, max_bytes=8000000):
        directory = Path(directory)
        require(directory.is_dir(), "HOST_TRACE_DIRECTORY_REQUIRED")
        self.max_bytes, self.bytes, self.rows = int(max_bytes), 0, 0
        self.stream = (directory / "host.jsonl").open("x")

    def write(self, row):
        line = json.dumps(row, sort_keys=True, allow_nan=False, separators=(",", ":")) + "\n"
        size = len(line.encode())
        require(self.bytes + size <= self.max_bytes, "HOST_TRACE_OUTPUT_CAP")
        try:
            self.stream.write(line)
            self.stream.flush()
        except (OSError, ValueError) as exc:
            raise IntegrityError("FATAL_HOST_TRACE_IO: " + str(exc)) from exc
        self.bytes += size
        self.rows += 1

    def close(self):
        if not self.stream.closed:
            self.stream.close()


def host_state(task):
    """Evaluator-only state hashes read from the host loop. Raw bytes, so NaN-safe; no writes."""
    od, p = task.obs_dict, task.perception
    groups = (("own_position", od["robot_position"]), ("own_velocity", od["robot_linvel"]),
              ("own_orientation", od["robot_orientation"]), ("own_body_angvel", od["robot_body_angvel"]),
              ("crash_buffer", od["crashes"]), ("truncations", task.truncations),
              ("target_evaluator_only", task.target_position),
              ("target_velocity_evaluator_only", task.target_vel_w),
              ("obstacle_evaluator_only", od["obstacle_position"]),
              ("obstacle_orientation_evaluator_only", od["obstacle_orientation"]),
              ("tracker_mean", p.tracker.state), ("tracker_covariance", p.tracker.cov),
              ("tracker_age", p.tracker.age), ("tracker_active", p.tracker.active),
              ("history_robot", p.robot_history), ("history_target", p.target_history),
              ("history_obstacle", p.obstacle_history), ("previous_action", task.prev_action),
              ("sim_steps", task.sim_env.sim_steps))  # drives the native time-limit truncation
    return {name: TensorBytes.copy(value, finite_required=False).sha() for name, value in groups}


def _actor(obs):
    return obs["obs"] if isinstance(obs, dict) else obs


def drive(task, reset, act, step, trace, actions, tap=None, recorder=None, after_policy=None):
    """The single player loop of every stage. Host reads only: no draw, seed or task write.

    The returned observation aliases the task actor buffer, which the next step overwrites
    in place, so each actor input is hashed before the policy call.
    """
    require((tap is None) == (recorder is None), "OBSERVER_AND_RECORDER_GO_TOGETHER")
    obs = reset()
    trace.write(dict(kind="INITIAL", returned=TensorBytes.copy(_actor(obs)).sha(),
                     rng=rng_fingerprint(task.device), state=host_state(task)))
    times, ended = [], 0
    for index in range(actions):
        start = time.perf_counter()
        actor = TensorBytes.copy(_actor(obs))
        require(actor == TensorBytes.copy(task.task_obs["observations"]), "ACTOR_INPUT_NOT_CURRENT_TASK_PACKET")
        rng_pre = rng_fingerprint(task.device)
        if tap is not None:
            tap.actor_rng_before = rng_pre
        action = act(obs)
        if after_policy is not None:
            after_policy()
        rng_policy = rng_fingerprint(task.device)
        obs, reward, done, _ = step(action)
        times.append((time.perf_counter() - start) * 1000)
        done_now = bool(torch.as_tensor(done).reshape(-1).bool().any())
        row = dict(kind="STEP", index=index, actor=actor.sha(), rng_pre_policy=rng_pre,
                   action=TensorBytes.copy(action, (1, 4)).record(True), rng_post_policy=rng_policy,
                   returned=TensorBytes.copy(_actor(obs)).sha(),
                   reward=TensorBytes.copy(torch.as_tensor(reward), finite_required=False).sha(),
                   done=done_now, rng_post_step=rng_fingerprint(task.device), state=host_state(task))
        if recorder is not None:
            last = recorder.last_parity
            require(recorder.completed == index + 1 and last["reset_expected"] == done_now,
                    "HOST_OBSERVER_TERMINAL_JOIN_MISMATCH")
            require((last["actor"], last["policy"], last["post"]["actor"]) ==
                    (row["actor"], row["action"]["sha256"], row["returned"]), "HOST_OBSERVER_FINGERPRINT_MISMATCH")
        trace.write(row)
        if done_now:
            ended += 1
            break
    return times, ended


class EvaluatorTap:
    """Instance call-through wrappers. All changes are reverted on context exit.

    Logger/diagnostics receive only frozen bytes/scalars. Evaluator state hashes and
    outcomes are isolated in a separate namespace and never passed to diagnose.
    """
    def __init__(self, task, recorder, binding):
        require(int(task.num_envs) == 1, "ONLY_N1_SUPPORTED")
        self.task, self.recorder, self.binding = task, recorder, binding
        self.originals, self.observe_calls, self.transform_calls = [], 0, 0
        self.compute_calls, self.lidar_calls, self.latest_lidar_schedule = 0, 0, None
        self.rgbd_calls = 0
        self.pre, self.resolved = None, None
        self.actor_rng_before = None
        self.observer_times, self.last_copy_ms, self.timing, self.pipeline_slow_streak = [], 0., {}, 0
        self.params = BoiaParams(tau_on=.5)  # read_online requires a params record; no gain selection.

    def _wrap(self, obj, name, factory):
        old = getattr(obj, name)
        had_own = name in vars(obj)
        own = vars(obj).get(name)
        wrapped = factory(old)
        setattr(wrapped, HOOK_MARK, True)
        setattr(obj, name, wrapped)
        self.originals.append((obj, name, old, wrapped, had_own, own))

    def __enter__(self):
        from .e_l0 import ContactChain  # CPU safe; uses the existing substep callback contract.
        t, r = self.task, self.recorder
        require(getattr(t, "_kroc_adapter", None) is None, "EXISTING_RECORDER_REQUIRES_SEPARATE_PROFILE")
        self.original_callback = t.sim_env.physics_step_callback
        require(not isinstance(self.original_callback, ContactChain), "CONTACT_CHAIN_ALREADY_INSTALLED")
        self.contact = ContactChain(self.original_callback, lambda: t.obs_dict["robot_contact_force_tensor"],
                                    float(t.sim_env.cfg.env.collision_force_threshold), 1, t.device)
        t.sim_env.set_physics_step_callback(self.contact)
        factories = dict(observe=self._observe, render_raw_rgbd=self._rgbd, capture=self._capture,
                         reset_idx=self._reset, transform_action_to_command=self._transform,
                         compute_state_reward_and_terminations=self._compute,
                         post_reward_calculation_step=self._post, step=self._step)
        try:
            for _, obj, name in hook_targets(t):
                self._wrap(obj, name, factories[name])
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *unused):
        conflict = False
        for obj, name, old, wrapper, had_own, own in reversed(self.originals):
            if getattr(obj, name) is not wrapper:
                conflict = True
                continue
            if had_own:
                setattr(obj, name, own)
            else:
                delattr(obj, name)
        self.originals.clear()
        if self.task.sim_env.physics_step_callback is self.contact:
            self.task.sim_env.set_physics_step_callback(self.original_callback)
        else:
            conflict = True
        require(not conflict, "ANOTHER_WRITER_CHANGED_OWNED_HOOK")

    def _capture(self, old):
        def wrapped(*a, **kw):
            out = old(*a, **kw)
            self.lidar_calls += 1
            self.latest_lidar_schedule = self.recorder.completed * .1
            return out
        return wrapped

    def _rgbd(self, old):
        def wrapped(*a, **kw):
            out = old(*a, **kw)
            self.rgbd_calls += 1  # never inspect/retain renderer GT arguments
            return out
        return wrapped

    def _reset(self, old):
        def wrapped(ids, *a, **kw):
            ids_list = torch.as_tensor(ids).reshape(-1).cpu().tolist()
            if ids_list:
                require(ids_list == [0], "UNEXPECTED_RESET_ENV_IDS")
                self.recorder.reset()
            return old(ids, *a, **kw)
        return wrapped

    def _observe(self, old):
        signature = inspect.signature(old)
        def wrapped(*a, **kw):
            args = signature.bind(*a, **kw).arguments
            out = old(*a, **kw)  # exactly once; arguments and return object are preserved.
            copy_start = time.perf_counter()
            self.observe_calls += 1
            t, r = self.task, self.recorder
            d = out[1]
            now, host = r.completed * .1, time.monotonic()
            camera = SensorTime(now, now, now, host, "DECLARED_SIM_SCHEDULE")
            lidar = SensorTime(self.latest_lidar_schedule, self.latest_lidar_schedule, now, host,
                              "DECLARED_SIM_SCHEDULE" if self.latest_lidar_schedule is not None else "MISSING")
            require(not bool(args["training"]), "PERTURBED_INPUT_DELIVERY_NOT_QUALIFIED")
            tr, p, od = t.perception.tracker, t.perception, t.obs_dict
            v = SensorValues(args["rgb"], args["depth"], args["lidar_m"], od["depth_range_pixels"],
                             d["camera_visible"], d["visible"], tr.state, tr.cov, tr.age, tr.active,
                             args["drone_pos_w"], args["drone_vel_w"], args["vehicle_quat"],
                             od["robot_orientation"], od["robot_body_angvel"], d["confidence"])
            calibration = ("YAW_ONLY", float(p.fx), float(p.fy), float(p.cx), float(p.cy),
                           .10, 0., .03, "EUCLIDEAN_RAY_RANGE_M", 12., .2,
                           "LIDAR_ZERO_WIDTH_4x72_5deg_10deg", "NO_CALIBRATED_ERROR_BOUNDS")
            key = JoinKey(0, r.episode, r.action_index, r.generation + 1, r.completed)
            r.observation(make_packet(key, self.binding, camera, lidar, v, calibration))
            self.last_copy_ms = (time.perf_counter() - copy_start) * 1000
            return out
        return wrapped

    def _step(self, old):
        def wrapped(actions, *a, **kw):
            t, r = self.task, self.recorder
            before_calls = (self.observe_calls, self.transform_calls, self.compute_calls, self.rgbd_calls)
            lidar_before = self.lidar_calls
            bind_start = time.perf_counter()
            consumed_copy_ms = self.last_copy_ms
            require(r.pending is not None, "MISSING_INITIAL_SENSOR_PACKET")
            current = read_online(t, t.perception, actions, self.params)
            for name, value in (("position", current.pos), ("velocity", current.vel),
                                ("mean", current.mu), ("covariance", current.Sigma),
                                ("camera", current.camera_visible), ("age", current.age)):
                require(TensorBytes.copy(value).sha() == r.pending.tensor(name).sha(), "STALE_OBSERVATION_ACTION_JOIN_" + name)
            r.begin(actions, t.task_obs["observations"], rng_fingerprint(t.device))
            self.timing = dict(key=r.inflight.key.__dict__, sampled=r.completed % r.stride == 0,
                               copy_ms=consumed_copy_ms, binding_diagnostic_log_ms=(time.perf_counter()-bind_start)*1000)
            self.contact.begin_action()
            self.pre, self.resolved = None, None
            out = old(actions, *a, **kw)
            require((self.observe_calls, self.transform_calls, self.compute_calls, self.rgbd_calls) ==
                    tuple(x + 1 for x in before_calls), "UNEXPECTED_OBSERVE_TRANSFORM_COMPUTE_COUNT")
            require(r.inflight is None, "MISSING_PRE_RESET_OUTCOME_RECORD")
            require(self.lidar_calls-lidar_before == 1 + int(r.last_parity["reset_expected"]),
                    "UNEXPECTED_LIDAR_CAPTURE_COUNT")
            boundary_start = time.perf_counter()
            require(r.pending is not None, "MISSING_RETURNED_OBSERVATION_BOUNDARY")
            post = dict(sensor=r.pending.fingerprint(), key=r.pending.key.__dict__,
                        actor=TensorBytes.copy(t.task_obs["observations"]).sha(),
                        rng=rng_fingerprint(t.device),
                        state={name: TensorBytes.copy(value).sha() for name, value in (
                            ("target_evaluator_only", t.target_position),
                            ("target_velocity_evaluator_only", t.target_vel_w),
                            ("obstacle_evaluator_only", t.obs_dict["obstacle_position"]),
                            ("obstacle_orientation_evaluator_only", t.obs_dict["obstacle_orientation"]),
                            ("history_robot", t.perception.robot_history),
                            ("history_target", t.perception.target_history),
                            ("history_obstacle", t.perception.obstacle_history),
                            ("previous_action", t.prev_action))})
            if r.reference is not None:
                require(digest_json(post) == digest_json(r.reference[r.completed-1]["post"]),
                        "OFF_ON_MISMATCH_POST_RESET_OR_SENSOR_BOUNDARY")
            r.last_parity["post"] = post
            r.logger.parity(r.last_parity)
            self.timing["boundary_hash_log_ms"] = (time.perf_counter()-boundary_start)*1000
            self.timing["total_observer_ms"] = sum(v for k, v in self.timing.items() if k.endswith("_ms"))
            self.observer_times.append(self.timing.copy())
            if self.timing["sampled"]:
                ms = self.timing["total_observer_ms"]
                self.pipeline_slow_streak = self.pipeline_slow_streak + 1 if ms > 500 else 0
                require(ms <= 2000 and self.pipeline_slow_streak < 3, "SHADOW_PIPELINE_LATENCY_WATCHDOG")
            return out
        return wrapped

    def _transform(self, old):
        def wrapped(action):
            require(TensorBytes.copy(action, (1, 4)) == self.recorder.policy, "TASK_ACTION_CHANGED_AFTER_F")
            out = old(action)
            start = time.perf_counter()
            self.transform_calls += 1
            self.recorder.transformed(out)
            self.timing["command_copy_ms"] = (time.perf_counter()-start)*1000
            return out
        return wrapped

    def _compute(self, old):
        def wrapped(*a, **kw):
            start = time.perf_counter()
            t, od = self.task, self.task.obs_dict
            pos = od["robot_position"].detach().clone()
            margin = float(t.vis_cfg.oob_margin)
            self.pre = dict(contact=bool((od["crashes"] > 0).item()),
                            height=bool(((pos[:, 2] < t.task_config.lower_height_bound) |
                                         (pos[:, 2] > t.task_config.upper_height_bound)).item()),
                            oob=bool(((pos[:, :2] < od["env_bounds_min"][:, :2] - margin) |
                                      (pos[:, :2] > od["env_bounds_max"][:, :2] + margin)).any().item()))
            pre_ms = (time.perf_counter()-start)*1000
            out = old(*a, **kw)
            start = time.perf_counter()
            self.compute_calls += 1
            self.resolved = bool(t.captured_now.item()), bool(t.crashed_now.item())
            self.timing["premask_copy_ms"] = pre_ms + (time.perf_counter()-start)*1000
            return out
        return wrapped

    def _post(self, old):
        def wrapped(*a, **kw):
            start = time.perf_counter()
            t, r, od = self.task, self.recorder, self.task.obs_dict
            require(self.pre is not None and self.resolved is not None, "PREMASK_TELEMETRY_MISSING")
            capture, crash = self.resolved
            timeout = bool(t.truncations.item()) and not capture and not crash
            groups = {name: TensorBytes.copy(value).sha() for name, value in (
                ("own_position", od["robot_position"]), ("own_velocity", od["robot_linvel"]),
                ("own_orientation", od["robot_orientation"]), ("target_evaluator_only", t.target_position),
                ("target_velocity_evaluator_only", t.target_vel_w),
                ("history_robot", t.perception.robot_history), ("history_target", t.perception.target_history),
                ("history_obstacle", t.perception.obstacle_history), ("previous_action", t.prev_action))}
            groups["observe_calls"] = self.observe_calls
            groups["actor_rng_before"] = self.actor_rng_before
            truth = TruthRecord(r.inflight.key, capture, crash, timeout, self.pre["contact"],
                                self.pre["height"], self.pre["oob"], int(self.contact.count.item()),
                                float(self.contact.peak.item()), self.contact.substep, self.contact.threshold,
                                digest_json(groups), capture or crash or timeout)
            r.finish(truth, rng_fingerprint(t.device), groups)
            self.timing["state_hash_terminal_log_ms"] = (time.perf_counter()-start)*1000
            out = old(*a, **kw)
            ids = torch.as_tensor(out).reshape(-1).cpu().tolist()
            require(ids == ([0] if truth.reset_expected else []), "ACTUAL_RESET_SET_MISMATCH")
            return out
        return wrapped
