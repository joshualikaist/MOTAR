"""Pure PX4 v1.14.3 ROS 2 message boundary; no rclpy, graph, publisher or flight lifecycle commands.

The local firmware source defines these messages. Binding its schema is a software compatibility check only;
actual flight-controller firmware, DDS delivery, clock alignment and offboard-loss response remain NOT_TESTED.
uXRCE-DDS already translates timestamp/timestamp_sample to the agent clock. Receive ages use a separate monotonic
clock. The caller owns calibrated camera stamps, message reception locks, actual publication and stop handling.
"""
from collections.abc import Mapping
from dataclasses import dataclass
import math
from types import MappingProxyType

import numpy as np

MAX_AGE_S = 0.15
MAX_SKEW_S = 0.025
STATUS_MAX_AGE_S = 2.0
PRIME_DURATION_S = 1.1
ARMED, POSITION, OFFBOARD = 2, 2, 14
PX4_VERSION = "v1.14.3"
PX4_MESSAGE_SHA256 = {
    "OffboardControlMode.msg": "7482f8af73a21de04427629051a87ce06cfb9bf3a580242d3cf9c212adf3f906",
    "TrajectorySetpoint.msg": "a9ed19116a311b89d41e8cd3e6fc49951ec23824126a102dd41d6f6b9e7b3cb6",
    "VehicleOdometry.msg": "f793b3b28c5ad65f9cf485b24816ba81de1fcaf9d5de903ffd75183930f95778",
    "VehicleLocalPosition.msg": "8fdca654db4b9245f71354dcbeffc8247b9ec4fe2a91735342bf89f8126c631a",
    "VehicleStatus.msg": "824ff9a94ee3600eff2f2dd45a3484d5bc805ff2d7fc56b701c754494a7f788d",
}
TOPICS = {
    "odometry": "/fmu/out/vehicle_odometry", "local_position": "/fmu/out/vehicle_local_position",
    "status": "/fmu/out/vehicle_status", "offboard_control_mode": "/fmu/in/offboard_control_mode",
    "trajectory_setpoint": "/fmu/in/trajectory_setpoint",
}
RESET_FIELDS = ("xy_reset_counter", "z_reset_counter", "vxy_reset_counter", "vz_reset_counter", "heading_reset_counter")


class GatewayError(ValueError):
    pass


@dataclass(frozen=True)
class Px4MessageBinding:
    """Explicit caller-supplied message-build provenance, never a hardware qualification claim."""
    firmware_version: str
    message_sha256: Mapping
    source: str

    def __post_init__(self):
        if (self.firmware_version != PX4_VERSION or not isinstance(self.message_sha256, Mapping)
                or dict(self.message_sha256) != PX4_MESSAGE_SHA256):
            raise GatewayError("firmware/message definitions must match the bound local PX4 v1.14.3 schema")
        if not isinstance(self.source, str) or not self.source.strip():
            raise GatewayError("explicit firmware/message binding provenance is required")
        object.__setattr__(self, "message_sha256", MappingProxyType(dict(self.message_sha256)))

    def metadata(self):
        return {"firmware_version": self.firmware_version, "message_sha256": dict(self.message_sha256),
                "source": self.source, "hardware_compatibility": "NOT_TESTED", "dds_delivery": "NOT_TESTED"}


def _number(value, label):
    if isinstance(value, (bool, np.bool_)):
        raise GatewayError("%s must be numeric" % label)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise GatewayError("%s must be numeric" % label) from exc
    if not math.isfinite(result):
        raise GatewayError("%s must be finite" % label)
    return result


def _vector(value, size, label):
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as exc:
        raise GatewayError("%s must be numeric" % label) from exc
    if result.shape != (size,) or not np.isfinite(result).all():
        raise GatewayError("%s must be a finite %d-vector" % (label, size))
    return result


def _uint(value, label, maximum=2**64 - 1):
    if type(value) is not int or not 0 <= value <= maximum:
        raise GatewayError("%s must be an unsigned integer" % label)
    return value


def _stamp(message, key, label):
    try:
        stamp = _uint(message[key], "%s.%s" % (label, key))
    except (KeyError, TypeError) as exc:
        raise GatewayError("%s.%s missing" % (label, key)) from exc
    if stamp == 0:
        raise GatewayError("%s.%s must be a measured nonzero timestamp" % (label, key))
    return stamp * 1e-6


def _fresh(stamp, now, maximum, label):
    if not 0 <= now - _number(stamp, label) <= maximum + 1e-9:
        raise GatewayError("stale/future %s" % label)


def _rotation(q):
    q = _vector(q, 4, "odometry quaternion wxyz")
    if abs(float(np.linalg.norm(q)) - 1.0) > 1e-3:
        raise GatewayError("odometry quaternion must be normalized")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _quaternion_xyzw(r):
    # Choose the largest component to avoid cancellation at rotations near pi.
    candidates = np.array([1+r[0, 0]-r[1, 1]-r[2, 2], 1-r[0, 0]+r[1, 1]-r[2, 2],
                           1-r[0, 0]-r[1, 1]+r[2, 2], 1+np.trace(r)])
    i = int(np.argmax(candidates))
    q = np.zeros(4)
    q[i] = math.sqrt(max(float(candidates[i]), 0.0)) / 2
    d = 4*q[i]
    if i == 0:
        q[1:] = [(r[0, 1]+r[1, 0])/d, (r[0, 2]+r[2, 0])/d, (r[2, 1]-r[1, 2])/d]
    elif i == 1:
        q[[0, 2, 3]] = [(r[0, 1]+r[1, 0])/d, (r[1, 2]+r[2, 1])/d, (r[0, 2]-r[2, 0])/d]
    elif i == 2:
        q[[0, 1, 3]] = [(r[0, 2]+r[2, 0])/d, (r[1, 2]+r[2, 1])/d, (r[1, 0]-r[0, 1])/d]
    else:
        q[:3] = [(r[2, 1]-r[1, 2])/d, (r[0, 2]-r[2, 0])/d, (r[1, 0]-r[0, 1])/d]
    return (q/np.linalg.norm(q)).tolist()


def _reset_identity(odometry, local_position):
    try:
        return tuple(_uint(local_position[name], name, 255) for name in RESET_FIELDS) \
            + (_uint(odometry["reset_counter"], "odometry.reset_counter", 255),)
    except KeyError as exc:
        raise GatewayError("estimator reset counters missing") from exc


def decode_estimator(odometry, local_position, status, *, now_s, now_monotonic_s, receipts):
    """Decode already-agent-clock PX4 mappings to the existing five-field ENU/FLU runtime state.

    Receipts are odometry/local_position/status monotonic timestamps recorded on corresponding messages only.
    Local-position validity and sample synchronization gate the odometry pose; arbitrary-heading FRD world frames
    are refused. The body angular velocity remains body-relative and is converted FRD to FLU.
    """
    now, mono = _number(now_s, "common ROS time"), _number(now_monotonic_s, "monotonic time")
    if now < 0 or mono < 0:
        raise GatewayError("clocks must be nonnegative")
    try:
        for name, message in (("odometry", odometry), ("local_position", local_position), ("status", status)):
            bound = STATUS_MAX_AGE_S if name == "status" else MAX_AGE_S
            _fresh(receipts[name], mono, bound, "%s receipt" % name)
            _fresh(_stamp(message, "timestamp", name), now, bound, "%s publication" % name)
        ot = _stamp(odometry, "timestamp_sample", "odometry")
        lt = _stamp(local_position, "timestamp_sample", "local_position")
        _fresh(ot, now, MAX_AGE_S, "odometry sample")
        _fresh(lt, now, MAX_AGE_S, "local_position sample")
        if ot > _stamp(odometry, "timestamp", "odometry") or lt > _stamp(local_position, "timestamp", "local_position"):
            raise GatewayError("estimator capture exceeds publication time")
        if abs(ot-lt) > MAX_SKEW_S + 1e-9:
            raise GatewayError("estimator capture skew exceeds 0.025 s")
        if odometry["pose_frame"] != 1 or odometry["velocity_frame"] != 1:
            raise GatewayError("odometry requires explicit NED pose and velocity frames")
        if any(local_position[name] is not True for name in
               ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control")):
            raise GatewayError("PX4 local-position/heading validity lost")
        _uint(status["arming_state"], "arming_state", 5)
        _uint(status["nav_state"], "nav_state", 23)
        if status["failsafe"] is not False:
            raise GatewayError("PX4 failsafe active or unavailable")
        _reset_identity(odometry, local_position)
        p, v = _vector(odometry["position"], 3, "NED position"), _vector(odometry["velocity"], 3, "NED velocity")
        rate = _vector(odometry["angular_velocity"], 3, "FRD angular velocity")
        ned_to_enu = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        r = ned_to_enu @ _rotation(odometry["q"]) @ np.diag([1., -1., -1.])
    except (KeyError, TypeError) as exc:
        raise GatewayError("required PX4 estimator/status fields missing") from exc
    return {"pos": (ned_to_enu @ p).tolist(), "vel": (ned_to_enu @ v).tolist(),
            "quat_xyzw": _quaternion_xyzw(r), "yaw_rate_rps": -float(rate[2]), "t": ot}


def _command_values(command):
    if not isinstance(command, Mapping):
        raise GatewayError("policy command must be a mapping")
    native = "velocity_world_enu_mps" in command
    if native and "velocity_enu_mps" in command:
        raise GatewayError("ambiguous policy command envelope")
    velocity_key, yaw_key = ("velocity_world_enu_mps", "yaw_rate_rps") if native else ("velocity_enu_mps", "yaw_rate_enu_rps")
    try:
        return _vector(command[velocity_key], 3, "policy ENU velocity"), _number(command[yaw_key], "policy yaw rate")
    except (KeyError, TypeError) as exc:
        raise GatewayError("policy command fields missing") from exc


def encode_setpoint(command, *, hold_z_ned_m, timestamp_us):
    """Velocity XY with position Z hold; unused fields are NaN, never implicit zero setpoints."""
    stamp = _uint(timestamp_us, "timestamp_us")
    if not stamp:
        raise GatewayError("timestamp_us must be nonzero common ROS/agent time")
    z = _number(hold_z_ned_m, "hold_z_ned_m")
    velocity, yaw_rate = _command_values(command)
    if abs(velocity[2]) > 1e-6 or np.linalg.norm(velocity[:2]) > math.sqrt(2)*2.5 + 1e-6 or abs(yaw_rate) > 3.0 + 1e-6:
        raise GatewayError("command outside the trained XY velocity/yaw envelope")
    nan = float("nan")
    return {"offboard_control_mode": {"timestamp": stamp, "position": True, "velocity": False,
                                      "acceleration": False, "attitude": False, "body_rate": False, "actuator": False},
            "trajectory_setpoint": {"timestamp": stamp, "position": [nan, nan, z],
                                    "velocity": [float(velocity[1]), float(velocity[0]), nan],
                                    "acceleration": [nan]*3, "jerk": [nan]*3, "yaw": nan, "yawspeed": -yaw_rate}}


class Px4OffboardGateway:
    """Latched software publication gate. None means stop BOTH offboard/trajectory streams.

    Explicit prime grants software ownership while the operator remains responsible for arming and switching modes.
    PRIMING emits neutral proposals with current measured Z; after 1.1 s of continuous proposal generation, fresh
    armed/OFFBOARD status plus a fresh command permit ACTIVE. This is not proof of actual DDS delivery. Any revoke
    needs a new gateway and a reset policy/tracker session, never an automatic re-entry. The standard Twist envelope
    is {capture_stamp_s, frame_id, velocity_enu_mps, yaw_rate_enu_rps}; capture_stamp_s is the oldest selected input,
    not publication time. The policy source owns synchronization of its hidden individual input stamps. Alternatively
    native runtime commands carry t/rgb_t/depth_t/state_t and their original velocity/yaw keys, with explicit skew checks.
    """
    def __init__(self, binding, *, command_frame_id):
        if not isinstance(binding, Px4MessageBinding):
            raise GatewayError("explicit Px4MessageBinding is required")
        if not isinstance(command_frame_id, str) or not command_frame_id.strip():
            raise GatewayError("an explicit ENU command frame_id contract is required")
        self.binding = binding
        self.command_frame_id = command_frame_id
        self.state, self.reason = "DISABLED", None
        self._last_clock = self._prime_mono = self._resets = self._command = self._command_receipt = None
        self._capture = self._signature = None
        self._command_contract = None
        self._message_receipts = {}

    def revoke(self, reason):
        self.state, self.reason = "REVOKED", str(reason)
        self._command = None
        return None

    def disable(self):
        if self.state != "DISABLED":
            self.revoke("operator disabled ownership")

    def _clock(self, now_s, mono):
        now, mono = _number(now_s, "common ROS time"), _number(mono, "monotonic time")
        if min(now, mono) < 0:
            raise GatewayError("clocks must be nonnegative")
        if self._last_clock is not None:
            ros_delta, mono_delta = now-self._last_clock[0], mono-self._last_clock[1]
            if min(ros_delta, mono_delta) < 0 or abs(ros_delta-mono_delta) > MAX_SKEW_S + 1e-9:
                raise GatewayError("common ROS/monotonic clock jump")
            if mono_delta > MAX_AGE_S + 1e-9:
                raise GatewayError("offboard proposal stream interrupted")
        self._last_clock = (now, mono)

    def _decode(self, snapshot, now_s, mono):
        receipts = dict(snapshot["receipts"])
        for name in ("odometry", "local_position", "status"):
            message = snapshot[name]
            identity = _stamp(message, "timestamp_sample" if name != "status" else "timestamp", name)
            fields = {"odometry": ("pose_frame", "velocity_frame", "position", "velocity", "q", "angular_velocity", "reset_counter"),
                      "local_position": ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid", "heading_good_for_control") + RESET_FIELDS,
                      "status": ("arming_state", "nav_state", "failsafe")}[name]
            signature = tuple(tuple(message[key]) if isinstance(message[key], (list, tuple, np.ndarray))
                              else message[key] for key in fields)
            if name in self._message_receipts:
                old_identity, old_receipt, old_signature = self._message_receipts[name]
                if identity < old_identity:
                    raise GatewayError("%s timestamp regressed" % name)
                if identity == old_identity:
                    if signature != old_signature:
                        raise GatewayError("%s changed payload at the same capture identity" % name)
                    receipts[name] = old_receipt
            self._message_receipts[name] = (identity, receipts[name], signature)
        state = decode_estimator(snapshot["odometry"], snapshot["local_position"], snapshot["status"],
                                 now_s=now_s, now_monotonic_s=mono, receipts=receipts)
        resets = _reset_identity(snapshot["odometry"], snapshot["local_position"])
        if self._resets is not None and resets != self._resets:
            raise GatewayError("estimator reset; new policy/ownership session required")
        self._resets = resets
        return state

    def prime(self, snapshot, *, now_s, now_monotonic_s, manual_hover_handover=False):
        if self.state != "DISABLED":
            raise GatewayError("prime requires a new disabled gateway")
        try:
            self._clock(now_s, now_monotonic_s)
            decoded = self._decode(snapshot, now_s, now_monotonic_s)
            if manual_hover_handover is not True:
                raise GatewayError("explicit operator manual-hover handover is required")
            if snapshot["status"]["arming_state"] != ARMED or snapshot["status"]["nav_state"] != POSITION:
                raise GatewayError("prime requires armed manual POSITION hover before OFFBOARD")
            self.hold_z_ned_m = -decoded["pos"][2]
            self._prime_nav = snapshot["status"]["nav_state"]
            self._prime_mono, self.state = float(now_monotonic_s), "PRIMING"
            return self._neutral(now_s)
        except (GatewayError, KeyError, TypeError, ValueError) as exc:
            self.revoke(exc)
            raise GatewayError(str(exc)) from exc

    def _neutral(self, now_s):
        return encode_setpoint({"velocity_world_enu_mps": [0., 0., 0.], "yaw_rate_rps": 0.},
                               hold_z_ned_m=self.hold_z_ned_m, timestamp_us=int(round(now_s*1e6)))

    def _accept_command(self, command, receipt, now_s, mono):
        if "capture_stamp_s" in command:
            # Standard TwistStamped carries only the oldest selected input stamp. Its source separately owns
            # multi-input synchronization. Do not fabricate missing RGB/depth/state stamps from this one value.
            if command["frame_id"] != self.command_frame_id:
                raise GatewayError("policy command ENU frame_id differs from explicit contract")
            captures = (_number(command["capture_stamp_s"], "oldest selected capture"),)
            generated, contract = (), "oldest_capture_twist_enu_v1"
        else:
            captures = tuple(_number(command[key], key) for key in ("rgb_t", "depth_t", "state_t"))
            generated = (_number(command["t"], "command generation time"),)
            contract = "native_rgbd_capture_v1"
            if max(captures) > generated[0] or max(captures)-min(captures) > MAX_SKEW_S + 1e-9:
                raise GatewayError("policy command capture skew/generation order")
        if self._command_contract is not None and self._command_contract != contract:
            raise GatewayError("policy command source contract changed")
        for stamp in captures + generated:
            _fresh(stamp, now_s, MAX_AGE_S, "policy command capture/generation")
        encoded = encode_setpoint(command, hold_z_ned_m=self.hold_z_ned_m, timestamp_us=int(round(now_s*1e6)))
        velocity, yaw_rate = _command_values(command)
        signature = tuple(velocity) + (yaw_rate,)
        if captures == self._capture:
            if signature != self._signature:
                raise GatewayError("same capture identity carries changed command")
            return  # duplicate delivery cannot renew either captured time or the first receipt
        if self._capture is not None and any(new <= old for new, old in zip(captures, self._capture)):
            raise GatewayError("policy command capture reused/regressed")
        _fresh(receipt, mono, MAX_AGE_S, "policy command receipt")
        self._command = {"velocity_world_enu_mps": velocity.tolist(), "yaw_rate_rps": yaw_rate}
        self._generated, self._command_contract = generated, contract
        self._capture, self._signature, self._command_receipt = captures, signature, float(receipt)
        return encoded

    def update(self, snapshot, command=None, *, command_received_monotonic_s=None, now_s, now_monotonic_s):
        if self.state in ("DISABLED", "REVOKED"):
            return None
        try:
            self._clock(now_s, now_monotonic_s)
            decoded = self._decode(snapshot, now_s, now_monotonic_s)
            status = snapshot["status"]
            if self.state == "ACTIVE" and (status["arming_state"] != ARMED or status["nav_state"] != OFFBOARD):
                raise GatewayError("armed/OFFBOARD ownership lost")
            if self.state == "PRIMING":
                if status["arming_state"] != ARMED:
                    raise GatewayError("disarmed during manual-hover handover")
                # Do not retain a ground/old altitude while the pilot owns the
                # vehicle. Freeze current measured z only upon ACTIVE entry.
                self.hold_z_ned_m = -decoded["pos"][2]
                if status["nav_state"] not in (self._prime_nav, OFFBOARD):
                    raise GatewayError("operator mode changed during priming; ownership revoked")
                if now_monotonic_s-self._prime_mono < PRIME_DURATION_S-1e-9:
                    if status["nav_state"] == OFFBOARD:
                        raise GatewayError("OFFBOARD entered before continuous priming completed")
                    return self._neutral(now_s)
                if status["nav_state"] != OFFBOARD:
                    return self._neutral(now_s)
                if status["arming_state"] != ARMED or command is None:
                    raise GatewayError("OFFBOARD activation requires armed state and a fresh policy command")
            if command is not None:
                self._accept_command(command, command_received_monotonic_s, now_s, now_monotonic_s)
            if self._command is None:
                raise GatewayError("fresh policy command unavailable")
            _fresh(self._command_receipt, now_monotonic_s, MAX_AGE_S, "policy command receipt")
            for stamp in self._capture + self._generated:
                _fresh(stamp, now_s, MAX_AGE_S, "policy command capture/generation")
            self.state = "ACTIVE"
            return encode_setpoint(self._command, hold_z_ned_m=self.hold_z_ned_m,
                                   timestamp_us=int(round(now_s*1e6)))
        except (GatewayError, KeyError, TypeError, ValueError) as exc:
            return self.revoke(exc)
