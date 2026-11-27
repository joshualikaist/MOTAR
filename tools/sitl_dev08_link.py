"""dev08 parameter and telemetry boundary (amendment1 runtime-stack row "Parameter/telemetry boundary"; motar-5md).

Library only: importing it starts nothing and opens no socket. Two guards between PX4 SITL and the dev08 runner.

1. Parameters (before takeoff). Read every recorded REAL32 parameter as PX4 booted. These are the fresh native
   values, including the yaw-rate gains the dev08 design says must not be assumed. Each booted value must equal
   its fresh-boot expectation: the airframe's `param set-default` value, else the firmware default from the bound
   build's parameters.json. A reused parameters.bson would not boot with them. Then set each override and require
   a PARAM_VALUE carrying exactly that float32, received after the set. Finally read everything back: overrides
   must hold and every other recorded parameter must be unchanged. Only messages from the autopilot (system 1,
   component 1) received after the request count, so an earlier or foreign PARAM_VALUE never answers it.
2. Telemetry (every 0.1 s checkpoint of PX4 simulator time).
   - LOCAL_POSITION_NED paces the loop as in the historical runner. Checkpoint k uses the first position sample
     with `time >= start + k*DT`, the historical loop's own floating-point comparison. It must arrive within a
     wall-clock stall bound (CLOCK_STALL) and lie within the skip bound of the schedule (CLOCK_SKIP), and the
     newest position may not be past that bound either (RUNNER_LATE).
   - ATTITUDE carries PX4's clock; the sample nearest the position, within the freshness bound, is used.
   - HIL_STATE_QUATERNION carries no clock: PX4 v1.14.3 never sets its time_usec
     (src/modules/mavlink/streams/HIL_STATE_QUATERNION.hpp), and the historical runner stamped it with wall-clock
     receipt. Its time is therefore taken from arrival. The truth sample is the latest one that meets all of:
     - received no earlier than TRUTH_AGE_S before the selected position sample;
     - received no later than BURST_S after it;
     - arriving before the next position message;
     - new at this checkpoint.
     This is the historical runner's latest received truth, with its receipt age bounded.
     - PX4 does not send the streams together. Each MAVLink stream keeps its own 20 ms schedule, and the truth
       stream's phase relative to the position stream is fixed per boot by when its stream-interval command is
       handled.
     - With BURST_S below one 4 ms PX4 pass, the truth used was sent in the position's own pass or 4, 8, 12 or
       16 ms before it, depending on that phase (review r3 probe).
     - A receive delay of about 1 ms or more on the position can instead admit the truth sent one pass (4 ms)
       after it. In that phase the choice can switch by one truth sample (20 ms) within an episode.
     - Which pass the truth came from is not checked. Its receipt age is bounded and recorded (truth_recv_skew_s);
       the PX4-time offset is not.
     The model assumes a real-time factor of about 1 and a prompt receiver thread (live-only).
   - Values must be finite and the truth quaternion unit length. Position and attitude simulator time must
     strictly increase.
   Validation starts at the gate's start cursor, so boot and takeoff messages before it are not judged. A failure
   is a TelemetryError with a code; it is never a censor (amendment1 aborts the unit).

Not verified live: no PX4 was connected. Integer parameters (EKF2_*) are not read back here; the airframe and
parameters.json are bound by hash in the owned session's runtime binding.
"""
import collections
import json
import math
import re
import struct
import threading
import time

STATUS = {"component": "parameter_telemetry_boundary", "contract": "amendment1 runtime-stack: Parameter/telemetry",
          "live_verified": False, "execution_supported": False, "launch_ready": False}

AUTOPILOT = (1, 1)
MAV_PARAM_TYPE_REAL32 = 9
DT = 0.1
SKIP_MAX_S = 0.05          # 50 Hz streams: the first sample after a checkpoint is <= 0.02 s late
FRESH_MAX_S = 0.05         # attitude: |t_att - t_pos|
TRUTH_AGE_S = 0.03         # truth received at most this long before the position sample (1.5 stream periods)
BURST_S = 0.003            # ... or at most this long after it: below one 4 ms PX4 cycle (review r2)
STALL_WALL_S = 2.0         # wall-clock wait for one 0.1 s checkpoint at real-time factor about 1
PARAM_TIMEOUT_S = 3.0
# REAL32 parameters recorded as booted and after the overrides: the historical S2 map, the candidate gains, the
# whole yaw-rate group, the gains that scale the overridden roll/pitch gains, and the airframe's flight envelope.
RECORD_PARAMS = ("MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_ROLLRATE_K", "MC_ROLLRATE_FF",
                 "MC_ROLLRATE_MAX", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_PITCHRATE_K",
                 "MC_PITCHRATE_FF", "MC_PITCHRATE_MAX", "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D",
                 "MC_YAWRATE_FF", "MC_YAWRATE_K", "MC_YAWRATE_MAX", "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC",
                 "MPC_XY_VEL_D_ACC", "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_TILTMAX_AIR",
                 "MPC_THR_HOVER", "THR_MDL_FAC")
STREAMS = {"LOCAL_POSITION_NED": "pos", "ATTITUDE": "att", "HIL_STATE_QUATERNION": "truth"}
FIELDS = {"pos": ("x", "y", "z", "vx", "vy", "vz"),
          "att": ("roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"),
          "truth": ("lat", "lon", "alt", "vx", "vy", "vz")}


class BoundaryAbort(RuntimeError):
    """A parameter or telemetry guard failed: the member is invalid and the unit stops (never an outcome)."""

    def __init__(self, code, detail):
        super().__init__("%s: %s" % (code, detail))
        self.code = code


class ParamError(BoundaryAbort):
    pass


class TelemetryError(BoundaryAbort):
    pass


def f32(value):
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


_SET_DEFAULT = re.compile(r"^\s*param\s+set-default\b")


def airframe_defaults(text):
    """{name: float32} of the airframe's `param set-default NAME VALUE` lines (last one wins, as in rcS). Any such
    line that does not parse is an error, so no parameter leaves the fresh check unnoticed."""
    out = {}
    for n, line in enumerate(text.splitlines(), 1):
        if not _SET_DEFAULT.match(line):
            continue
        m = re.match(r"^\s*param\s+set-default\s+(\w+)\s+([-+0-9.eE]+)(\s+#.*)?\s*$", line)
        if m is None:
            raise ValueError("airframe line %d does not parse: %r" % (n, line))
        out[m.group(1)] = f32(m.group(2))
    return out


def firmware_defaults(parameters_json_text, names):
    """{name: float32 default} from the build's parameters.json; every name must exist and be a Float (REAL32)."""
    found = {}
    for group in json.loads(parameters_json_text)["parameters"]:
        if group.get("name") in names:
            found[group["name"]] = group
    missing = sorted(set(names) - set(found))
    if missing:
        raise ValueError("parameters.json lacks %s" % missing)
    wrong = sorted(n for n, g in found.items() if g.get("type") != "Float")
    if wrong:
        raise ValueError("not REAL32 parameters: %s" % wrong)
    return {n: f32(found[n]["default"]) for n in names}


def fresh_expectations(airframe_text, parameters_json_text, names=RECORD_PARAMS):
    """The booted value of each recorded parameter in an empty rootfs: the airframe's set-default, else firmware."""
    exp = firmware_defaults(parameters_json_text, names)
    air = airframe_defaults(airframe_text)
    exp.update({n: air[n] for n in names if n in air})
    return exp


class RxError(RuntimeError):
    pass


class Inbox:
    """Messages in arrival order, each stamped with a sequence number and a monotonic receive time.
    A message is {"type": str, "src": (system, component), "fields": dict}. A receiver failure is latched."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._cv = threading.Condition()
        self._items = []
        self.error = None

    def now(self):
        return self._clock()

    def put(self, msg):
        with self._cv:
            item = dict(msg, seq=len(self._items) + 1, recv=self._clock())
            self._items.append(item)
            self._cv.notify_all()
            return item

    def fail(self, exc):
        with self._cv:
            if self.error is None:
                self.error = "%s: %s" % (exc.__class__.__name__, exc)
            self._cv.notify_all()

    def cursor(self):
        with self._cv:
            return len(self._items)

    def cursor_since(self, recv_time):
        """The cursor of the first message received at or after recv_time."""
        with self._cv:
            for i in range(len(self._items) - 1, -1, -1):
                if self._items[i]["recv"] < recv_time:
                    return i + 1
            return 0

    def since(self, cursor):
        with self._cv:
            return self._items[cursor:], len(self._items)

    def wait_new(self, cursor, timeout):
        with self._cv:
            return self._cv.wait_for(lambda: len(self._items) > cursor or self.error is not None, timeout)

    def wait_for(self, pred, cursor, timeout):
        """The first message after cursor satisfying pred, or None on timeout; RxError if the receiver failed."""
        end = time.monotonic() + timeout
        while True:
            items, cursor_new = self.since(cursor)
            for m in items:
                if pred(m):
                    return m
            if self.error is not None:
                raise RxError(self.error)
            cursor = cursor_new
            left = end - time.monotonic()
            if left <= 0 or not self.wait_new(cursor, left):
                items, _ = self.since(cursor)
                for m in items:
                    if pred(m):
                        return m
                return None


# ---------------------------------------------------------------- parameters

def _param_reply(name, src):
    return lambda m: (m["type"] == "PARAM_VALUE" and tuple(m["src"]) == src
                      and str(m["fields"].get("param_id", "")).rstrip("\x00") == name)


def _checked(m, name):
    f = m["fields"]
    if f.get("param_type") != MAV_PARAM_TYPE_REAL32:
        raise ParamError("PARAM_TYPE", "%s type %s, expected REAL32" % (name, f.get("param_type")))
    v = f.get("param_value")
    if not isinstance(v, float) or not math.isfinite(v):
        raise ParamError("PARAM_NONFINITE", "%s value %r" % (name, v))
    return {"name": name, "value": v, "seq": m["seq"], "recv": m["recv"], "param_index": f.get("param_index"),
            "param_count": f.get("param_count")}


def _wait_reply(inbox, name, src, cursor, timeout, code):
    try:
        m = inbox.wait_for(_param_reply(name, src), cursor, timeout)
    except RxError as e:
        raise ParamError("RX_ERROR", "receiver failed while waiting for %s: %s" % (name, e)) from None
    if m is None:
        raise ParamError(code, "no PARAM_VALUE for %s within %.1f s" % (name, timeout))
    return m


def read_param(inbox, transport, name, *, src=AUTOPILOT, timeout=PARAM_TIMEOUT_S):
    cursor = inbox.cursor()
    transport.param_read(name)
    return _checked(_wait_reply(inbox, name, src, cursor, timeout, "PARAM_MISSING"), name)


def set_param(inbox, transport, name, value, *, src=AUTOPILOT, timeout=PARAM_TIMEOUT_S):
    want = f32(value)
    cursor = inbox.cursor()
    transport.param_set(name, want)
    echo = _checked(_wait_reply(inbox, name, src, cursor, timeout, "PARAM_ECHO_MISSING"), name)
    if echo["value"] != want:
        raise ParamError("PARAM_ECHO_MISMATCH", "%s echoed %r, set %r" % (name, echo["value"], want))
    return echo


def apply_and_verify(inbox, transport, overrides, *, fresh_defaults, record=RECORD_PARAMS, src=AUTOPILOT,
                     timeout=PARAM_TIMEOUT_S):
    """Booted values -> fresh-boot check (every recorded parameter) -> overrides with echo -> full readback."""
    names = sorted(set(record) | set(overrides))
    unexpected = sorted(set(names) - set(fresh_defaults))
    if unexpected:
        raise ValueError("no fresh-boot expectation for %s" % unexpected)
    booted = {n: read_param(inbox, transport, n, src=src, timeout=timeout) for n in names}
    stale = {n: (booted[n]["value"], fresh_defaults[n]) for n in names if booted[n]["value"] != fresh_defaults[n]}
    if stale:
        raise ParamError("PARAM_NOT_FRESH", "booted values differ from the fresh-boot expectation: %s" % stale)
    echoes = {n: set_param(inbox, transport, n, v, src=src, timeout=timeout) for n, v in overrides.items()}
    after = {n: read_param(inbox, transport, n, src=src, timeout=timeout) for n in names}
    for n, v in overrides.items():
        if after[n]["value"] != f32(v):
            raise ParamError("PARAM_READBACK_MISMATCH", "%s reads %r, set %r" % (n, after[n]["value"], f32(v)))
    drift = {n: (booted[n]["value"], after[n]["value"]) for n in names
             if n not in overrides and after[n]["value"] != booted[n]["value"]}
    if drift:
        raise ParamError("PARAM_DRIFT", "parameters changed without an override: %s" % drift)
    return {"booted": {n: booted[n]["value"] for n in names}, "effective": {n: after[n]["value"] for n in names},
            "overrides": {n: f32(v) for n, v in overrides.items()}, "fresh_checked": names,
            "echo_seq": {n: e["seq"] for n, e in echoes.items()}}


# ---------------------------------------------------------------- telemetry

class TelemetryGate:
    """Pulls messages from an Inbox (from start_cursor on) and serves checked checkpoint snapshots."""

    def __init__(self, inbox, *, start_cursor=0, src=AUTOPILOT, dt=DT, skip_max_s=SKIP_MAX_S,
                 fresh_max_s=FRESH_MAX_S, truth_age_s=TRUTH_AGE_S, burst_s=BURST_S, stall_wall_s=STALL_WALL_S,
                 history=400):
        self.inbox = inbox
        self.src = tuple(src)
        self.dt, self.skip_max_s, self.fresh_max_s = dt, skip_max_s, fresh_max_s
        self.truth_age_s, self.burst_s, self.stall_wall_s = truth_age_s, burst_s, stall_wall_s
        self.samples = {k: collections.deque(maxlen=history) for k in FIELDS}
        self.start_cursor = self.cursor = start_cursor
        self.ignored = collections.Counter()
        self.start = None
        self.last_truth_seq = None
        self.last_k = None
        self.error = None

    def _fail(self, code, detail):
        if self.error is None:
            self.error = (code, detail)
        raise TelemetryError(code, detail)

    def _ingest(self, m):
        kind = STREAMS.get(m["type"])
        if kind is None:
            return
        if tuple(m["src"]) != self.src:
            self.ignored["foreign_source"] += 1
            return
        f = m["fields"]
        values = [float(f[k]) for k in FIELDS[kind]]
        if kind == "truth":
            t = None                                          # no clock on this message (PX4 v1.14.3)
            q = [float(x) for x in f["attitude_quaternion"]]
            values += q
            if all(math.isfinite(x) for x in q) and abs(math.sqrt(sum(x * x for x in q)) - 1.0) > 1e-3:
                self._fail("BAD_QUATERNION", "truth quaternion norm %.6f at seq %d" %
                           (math.sqrt(sum(x * x for x in q)), m["seq"]))
        else:
            t = f["time_boot_ms"] * 1e-3
            values.append(t)
        if not all(math.isfinite(x) for x in values):
            self._fail("NONFINITE", "%s sample at seq %d" % (kind, m["seq"]))
        prev = self.samples[kind][-1] if self.samples[kind] else None
        if t is not None and prev is not None and t <= prev["t"]:
            self._fail("CLOCK_NOT_INCREASING", "%s %.6f after %.6f" % (kind, t, prev["t"]))
        self.samples[kind].append({"t": t, "seq": m["seq"], "recv": m["recv"], "fields": f})

    def pump(self):
        if self.error:
            raise TelemetryError(*self.error)
        if self.inbox.error is not None:
            self._fail("RX_ERROR", self.inbox.error)
        items, self.cursor = self.inbox.since(self.cursor)
        for m in items:
            self._ingest(m)

    def _truth_for(self, pos):
        """The latest truth received within [pos.recv - truth_age_s, pos.recv + burst_s] that arrived before the
        next position message, or None."""
        lo, hi = pos["recv"] - self.truth_age_s, pos["recv"] + self.burst_s
        nxt = next((s["seq"] for s in self.samples["pos"] if s["seq"] > pos["seq"]), None)
        cand = [s for s in self.samples["truth"] if lo <= s["recv"] <= hi and (nxt is None or s["seq"] < nxt)]
        return max(cand, key=lambda s: s["seq"]) if cand else None

    def _att_for(self, pos):
        cand = [s for s in self.samples["att"] if abs(s["t"] - pos["t"]) <= self.fresh_max_s]
        return min(cand, key=lambda s: abs(s["t"] - pos["t"])) if cand else None

    def _settle(self, pos):
        """Wait until burst_s after the position sample's receipt (at most burst_s + 0.05 s of wall time), so a
        truth sent in the position's PX4 pass is in."""
        end = pos["recv"] + self.burst_s
        wall_end = time.monotonic() + self.burst_s + 0.05
        while self.inbox.now() < end and time.monotonic() < wall_end:
            self.inbox.wait_new(self.cursor, self.burst_s)
            self.pump()

    def _diagnostics(self, t_k):
        pos = list(self.samples["pos"])
        near = [p for p in pos if abs(p["t"] - t_k) <= 0.3]
        return {"t_k": round(t_k, 6), "newest_pos_t": round(pos[-1]["t"], 4) if pos else None,
                "positions_near": [(round(p["t"], 3), round(p["recv"], 4)) for p in near][-20:]}

    def align(self):
        """Fix the episode's start (latest position time); needs all three streams, recent and coherent."""
        if self.start is not None:
            self._fail("ALIGN_TWICE", "align() may be called once per gate")
        self.pump()
        if not all(self.samples.values()):
            self._fail("STREAM_MISSING", "streams present: %s" % sorted(k for k, v in self.samples.items() if v))
        now = self.inbox.now()
        quiet = {k: now - v[-1]["recv"] for k, v in self.samples.items() if now - v[-1]["recv"] > self.stall_wall_s}
        if quiet:
            self._fail("STREAM_STALE", "no new sample for %s s" % {k: round(v, 3) for k, v in quiet.items()})
        pos = self.samples["pos"][-1]
        self._settle(pos)
        if self._att_for(pos) is None:
            self._fail("STALE_ATT", "no attitude within %.3f s of the position at alignment" % self.fresh_max_s)
        if self._truth_for(pos) is None:
            self._fail("STALE_TRUTH", "no truth received within [-%.3f, +%.3f] s of the position at alignment" %
                       (self.truth_age_s, self.burst_s))
        self.start = pos["t"]
        return {"start_sim_s": self.start, "pos_seq": pos["seq"], "start_cursor": self.start_cursor,
                "truth_time_model": "receipt: latest within [-%g, +%g] s of the position sample" %
                                    (self.truth_age_s, self.burst_s)}

    def checkpoint(self, k):
        if self.start is None:
            raise TelemetryError("NOT_ALIGNED", "align() first")
        if (self.last_k is None and k != 0) or (self.last_k is not None and k != self.last_k + 1):
            self._fail("CHECKPOINT_ORDER", "checkpoint %d after %s" % (k, self.last_k))
        t_k = self.start + k * self.dt                      # the historical loop's expression
        wall_end = time.monotonic() + self.stall_wall_s
        pos = None
        while pos is None:
            self.pump()
            pos = next((s for s in self.samples["pos"] if not s["t"] < t_k), None)
            if pos is None:
                left = wall_end - time.monotonic()
                if left <= 0 or not self.inbox.wait_new(self.cursor, left):
                    self.pump()
                    pos = next((s for s in self.samples["pos"] if not s["t"] < t_k), None)
                    if pos is None:
                        self._fail("CLOCK_STALL", "no position at or after %.3f s within %.1f s wall %s" %
                                   (t_k, self.stall_wall_s, self._diagnostics(t_k)))
        if pos["t"] - t_k > self.skip_max_s:
            self._fail("CLOCK_SKIP", "first position %.3f s after checkpoint %d %s" %
                       (pos["t"] - t_k, k, self._diagnostics(t_k)))
        latest = self.samples["pos"][-1]
        if latest["t"] - t_k > self.skip_max_s:
            self._fail("RUNNER_LATE", "checkpoint %d served %.3f s of simulator time late %s" %
                       (k, latest["t"] - t_k, self._diagnostics(t_k)))
        self._settle(pos)
        att, truth = self._att_for(pos), self._truth_for(pos)
        if att is None:
            self._fail("STALE_ATT", "no attitude within %.3f s of %.3f s" % (self.fresh_max_s, pos["t"]))
        if truth is None:
            self._fail("STALE_TRUTH", "no truth received within [-%.3f, +%.3f] s of the position (checkpoint %d)" %
                       (self.truth_age_s, self.burst_s, k))
        if truth["seq"] == self.last_truth_seq:
            self._fail("STALE_TRUTH", "checkpoint %d would reuse the previous truth sample" % k)
        self.last_truth_seq = truth["seq"]
        self.last_k = k
        return {"k": k, "scheduled_sim_s": t_k, "pos": pos, "att": att, "truth": truth,
                "pos_lateness_s": pos["t"] - t_k, "att_skew_s": att["t"] - pos["t"],
                "truth_recv_skew_s": truth["recv"] - pos["recv"]}


# ---------------------------------------------------------------- pymavlink adapter (not verified live)

class PymavlinkTransport:
    """Sends parameter requests and feeds received messages into an Inbox. conn is a pymavlink connection.
    `lock` serialises every send on conn (pymavlink's sequence counter is not locked); share it with other senders."""

    def __init__(self, conn, inbox, *, target=AUTOPILOT, lock=None):
        self.conn, self.inbox, self.target = conn, inbox, tuple(target)
        self.lock = lock or threading.Lock()
        self.stop = threading.Event()

    def param_read(self, name):
        with self.lock:
            self.conn.mav.param_request_read_send(self.target[0], self.target[1], name.encode(), -1)

    def param_set(self, name, value):
        with self.lock:
            self.conn.mav.param_set_send(self.target[0], self.target[1], name.encode(), float(value),
                                         MAV_PARAM_TYPE_REAL32)

    def rx_loop(self, types=tuple(STREAMS) + ("PARAM_VALUE", "HEARTBEAT")):
        try:
            while not self.stop.is_set():
                m = self.conn.recv_match(blocking=True, timeout=0.5)
                if m is None or m.get_type() not in types:
                    continue
                self.inbox.put({"type": m.get_type(), "src": (m.get_srcSystem(), m.get_srcComponent()),
                                "fields": m.to_dict()})
        except Exception as e:                                # noqa: BLE001 - latched, reported by every waiter
            self.inbox.fail(e)


if __name__ == "__main__":
    print(json.dumps(STATUS))
