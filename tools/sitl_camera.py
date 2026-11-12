"""Estimator-only RGB-D companion control adapter; no automatic arm or simulator launch.

`run_control` operates caller-owned, already armed/OFFBOARD interfaces. The caller
owns startup, calibrated RGB/depth alignment, measured clock translation, transport
cleanup and the stop response. A service callback must keep the currently intended
hold/setpoint alive while bounded sensor I/O runs. Target motion can be supplied
only as an explicitly separate simulator callback; it never enters policy state.

The CLI verifies a prepared policy contract only. This source preparation and its
fake-interface CPU tests are not Gazebo, hardware or flight qualification.
"""
from __future__ import annotations

import argparse
import functools
from collections.abc import Mapping
from dataclasses import dataclass
import json
import math
import time

DT = 0.1
MAX_STEPS = 600
MAX_AGE_S = 0.15
HEARTBEAT_MAX_AGE_S = 2.0
TM_POS_Z_VEL_XY_YAWRATE = 1 | 2 | 32 | 64 | 128 | 256 | 1024


class CameraControlError(RuntimeError):
    pass


@dataclass(frozen=True)
class ClockBinding:
    """Explicit measured translation: sensor simulation time = PX4 boot time + offset.

    This is caller evidence, not an automatic clock calibration. `source` identifies
    its measurement/receipt; an unmeasured identity mapping must not be passed as one.
    """
    boot_to_sensor_offset_s: float
    source: str

    def __post_init__(self):
        if (isinstance(self.boot_to_sensor_offset_s, bool)
                or not math.isfinite(float(self.boot_to_sensor_offset_s))
                or not isinstance(self.source, str) or not self.source.strip()):
            raise ValueError("a finite clock offset and explicit measurement source are required")

    def translate(self, boot_s):
        return float(boot_s) + float(self.boot_to_sensor_offset_s)


def _finite(values, label):
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError) as exc:
        raise CameraControlError("malformed %s" % label) from exc
    if not all(math.isfinite(value) for value in result):
        raise CameraControlError("non-finite %s" % label)
    return result


def _enu_flu_quaternion(roll, pitch, yaw_ned):
    # NED/FRD ZYX -> ENU/FLU ZYX: roll, -pitch, pi/2-yaw.
    rr, pp, yy = roll / 2.0, -pitch / 2.0, (math.pi / 2.0 - yaw_ned) / 2.0
    cr, sr, cp, sp, cy, sy = math.cos(rr), math.sin(rr), math.cos(pp), math.sin(pp), math.cos(yy), math.sin(yy)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def estimator_state(link, clock_binding, *, clock=time.monotonic, max_age_s=MAX_AGE_S):
    """Read LOCAL_POSITION_NED, ATTITUDE and fresh HEARTBEAT only; never HIL/truth.

    The owner must expose snapshot() that copies pos, att, hb and their respective
    pos_wall_t, att_wall_t, hb_wall_t receive timestamps together under its receiver
    lock. The timestamp is updated only on receipt of the corresponding telemetry,
    using this same monotonic clock. A newly received heartbeat cannot freshen a
    frozen estimator tuple. Historical links without this boundary are refused.
    Receipt freshness is checked in monotonic time; boot timestamps are compared
    only with each other or translated to the explicitly bound sensor clock.
    """
    if not isinstance(clock_binding, ClockBinding):
        raise ValueError("ClockBinding is required; PX4 boot and sensor clocks are not implicitly equal")
    if not 0 < max_age_s <= MAX_AGE_S:
        raise ValueError("estimator age bound must be in (0, 0.15] s")
    snapshot = getattr(link, "snapshot", None)
    if not callable(snapshot):
        raise CameraControlError("atomic estimator snapshot unavailable")
    raw = snapshot()
    if not isinstance(raw, Mapping):
        raise CameraControlError("malformed atomic estimator snapshot")
    pos, att, hb = raw.get("pos"), raw.get("att"), raw.get("hb")
    pos_t, att_t, hb_t = (raw.get(key) for key in ("pos_wall_t", "att_wall_t", "hb_wall_t"))
    if any(value is None for value in (pos, att, hb, pos_t, att_t, hb_t)):
        raise CameraControlError("estimator/heartbeat timestamp unavailable")
    if len(pos) != 7 or len(att) != 7 or len(hb) != 2:
        raise CameraControlError("malformed estimator/heartbeat snapshot")
    pos, att = _finite(pos, "position"), _finite(att, "attitude")
    if type(hb[0]) is not int or type(hb[1]) is not int:
        raise CameraControlError("malformed heartbeat mode fields")
    received_now = _finite([clock()], "monotonic time")[0]
    for label, stamp, bound in (("position", pos_t, max_age_s), ("attitude", att_t, max_age_s),
                                ("heartbeat", hb_t, HEARTBEAT_MAX_AGE_S)):
        age = received_now - _finite([stamp], "%s receipt time" % label)[0]
        if not 0.0 <= age <= bound:
            raise CameraControlError("stale/future %s receipt" % label)
    if not hb[0] & 128 or ((hb[1] >> 16) & 0xFF) != 6:
        raise CameraControlError("armed/OFFBOARD state lost")
    if pos[0] < 0 or att[0] < 0 or abs(pos[0] - att[0]) > max_age_s:
        raise CameraControlError("incoherent estimator timestamps")
    if abs(att[1]) > math.pi / 4.0 or abs(att[2]) > math.pi / 4.0:
        raise CameraControlError("attitude outside the trained 45-degree envelope")
    return {"pos": [pos[2], pos[1], -pos[3]], "vel": [pos[5], pos[4], -pos[6]],
            "quat_xyzw": _enu_flu_quaternion(att[1], att[2], att[3]),
            "yaw_rate_rps": -att[6], "t": clock_binding.translate(pos[0]),
            "attitude_t": clock_binding.translate(att[0])}


def _packet_times(packet, state, previous, max_age_s):
    try:
        if 'clock_now_s' in packet and 'sim_now' in packet:
            raise CameraControlError('ambiguous host/simulation clock packet')
        now_key = 'clock_now_s' if 'clock_now_s' in packet else 'sim_now'
        now, rt, dt = _finite([packet[now_key], packet["rgb_t"], packet["depth_t"]], "sensor timestamps")
    except KeyError as exc:
        raise CameraControlError("RGB-D timestamp missing") from exc
    if rt > now or dt > now:
        raise CameraControlError("future RGB-D capture exceeds packet clock")
    current = max(now, state["t"], state["attitude_t"])
    if min(now, rt, dt) < 0 or any(not 0 <= current - ts <= max_age_s for ts in (now, rt, dt, state["t"], state["attitude_t"])):
        raise CameraControlError("stale/future RGB-D or estimator time; verify clock binding")
    if previous is not None and (rt <= previous[0] or dt <= previous[1] or now < previous[2]):
        raise CameraControlError("RGB-D frame reused or simulation time regressed")
    captures = (rt, dt, state["t"], state["attitude_t"])
    if max(captures) - min(captures) > 0.025 + 1e-9:
        raise CameraControlError("RGB/depth/estimator capture skew exceeds 0.025 s")
    return current, (rt, dt, now)


def _invalidate_on_failure(function):
    @functools.wraps(function)
    def guarded(runtime, *args, **kwargs):
        try:
            return function(runtime, *args, **kwargs)
        except BaseException:
            # A rejected post-inference tick cannot silently reuse its history.
            invalidate = getattr(runtime, 'invalidate', None)
            if invalidate is not None:
                invalidate()
            raise
    return guarded


@_invalidate_on_failure
def run_control(runtime, link, client, *, max_steps, service, clock_binding,
                frame_adapter=None, initial_target=None,
                altitude_m=1.0, io_timeout_s=2.0, max_age_s=MAX_AGE_S,
                clock=time.monotonic, sleep=time.sleep):
    """Run a bounded already-ready caller-owned control segment, returning an in-memory trace.

    service() must be bounded, maintain the intended setpoint, and receive fresh
    telemetry. frame_adapter(packet) supplies the runtime frame; its default is the
    source-bound Gazebo camera adapter, including actual intrinsics and invalid mask.
    No arming, mode change, truth, target state,
    automatic simulator launch or transport ownership is assumed.

    Errors stop policy command publication immediately and propagate to the caller's
    stop/cleanup response. Policy history after an error must not be resumed.
    """
    if type(max_steps) is not int or not 1 <= max_steps <= MAX_STEPS:
        raise ValueError("max_steps must be an explicit integer in [1, 600]")
    if frame_adapter is None:
        try:
            from .camera_policy_runtime import gazebo_frame
        except ImportError:
            from camera_policy_runtime import gazebo_frame
        frame_adapter = gazebo_frame
    if (not isinstance(clock_binding, ClockBinding) or not callable(service) or not callable(frame_adapter)
            or not math.isfinite(altitude_m) or altitude_m <= 0
            or not 0 < io_timeout_s <= 2.0 or not 0 < max_age_s <= MAX_AGE_S):
        raise ValueError("invalid camera control bounds/interfaces")

    def state():
        return estimator_state(link, clock_binding, clock=clock, max_age_s=max_age_s)

    def live_service():
        state()
        service()
        state()

    live_service()
    start = state()
    if abs(start["pos"][2] - altitude_m) >= 0.3:
        raise CameraControlError("initial estimator altitude outside hold tolerance")
    if initial_target is not None:
        try:
            from . import sitl_s4_gazebo as gazebo
        except ImportError:
            import sitl_s4_gazebo as gazebo
        gazebo.wait_initial_target(client, initial_target, timeout_s=io_timeout_s, maintain_hold=live_service,
                                   clock=clock, sleep=sleep, sensor_mode="rgbd")
        start = state()
        if abs(start["pos"][2] - altitude_m) >= 0.3:
            raise CameraControlError("post-readiness estimator altitude outside hold tolerance")
    first_t = start["t"]
    trace, previous = [], None
    for k in range(max_steps):
        due = first_t + k * DT
        waiting_deadline = clock() + io_timeout_s
        while True:
            live_service()
            sample = state()
            if sample["t"] >= due - 1e-8:
                break
            if clock() >= waiting_deadline:
                raise CameraControlError("estimator scheduling timeout")
            sleep(min(0.01, max(0.0, waiting_deadline - clock())))
        if sample["t"] - due > max_age_s:
            raise CameraControlError("policy tick late")
        io_deadline = clock() + io_timeout_s
        with client.bounded_io(io_deadline, live_service, clock=clock):
            packet = client.frames()
        if clock() >= io_deadline:
            raise CameraControlError("sensor I/O deadline exceeded")
        sample = state()
        now_s, selected = _packet_times(packet, sample, previous, max_age_s)
        frame = frame_adapter(packet)
        # Strip adapter bookkeeping before the production policy boundary. No truth is accepted.
        policy_state = {key: sample[key] for key in ("pos", "vel", "quat_xyzw", "yaw_rate_rps", "t")}
        call_started = clock()
        result = runtime.step(frame, policy_state, now_s=now_s)
        live_service()
        newest = state()
        if clock() - call_started > max_age_s or newest["t"] - due > max_age_s:
            raise CameraControlError("policy compute/control tick late; no command published")
        try:
            velocity = _finite(result["velocity_world_enu_mps"], "policy velocity")
            yaw_rate = _finite([result["yaw_rate_rps"]], "policy yaw rate")[0]
        except (KeyError, TypeError) as exc:
            raise CameraControlError("malformed policy command") from exc
        if (len(velocity) != 3 or abs(velocity[2]) > 1e-6
                or math.hypot(*velocity[:2]) > math.sqrt(2.0) * 2.5 + 1e-6 or abs(yaw_rate) > 3.0 + 1e-6):
            raise CameraControlError("policy command outside camera policy envelope")
        link.setpoint(TM_POS_Z_VEL_XY_YAWRATE, 0.0, 0.0, -altitude_m,
                      velocity[1], velocity[0], -yaw_rate)
        trace.append({"step": k, "scheduled_sensor_t": due, "state": policy_state,
                      "sensor_t": {"clock_now_s": selected[2], "rgb": selected[0], "depth": selected[1]},
                      "velocity_world_enu_mps": velocity, "yaw_rate_rps": yaw_rate,
                      "sensor_mode": "rgbd", "state_source": "PX4_LOCAL_POSITION_NED_ATTITUDE",
                      "clock_binding_source": clock_binding.source})
        previous = selected
    return trace


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", required=True, help="verify contract and bound bytes; no connection/arming")
    args = parser.parse_args(argv)
    try:
        from .camera_policy_contract import load_contract
    except ImportError:
        from camera_policy_contract import load_contract
    contract = load_contract(args.check)
    print(json.dumps({"status": "PREPARED_SOURCE_ONLY", "lineage": contract["lineage"], "seed": contract["seed"],
                      "sensor_mode": "rgbd", "live": "NOT_TESTED", "hardware": "NOT_TESTED"}, sort_keys=True))


if __name__ == "__main__":
    main()
