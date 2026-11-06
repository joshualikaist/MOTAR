"""AGQ collision-geometry diagnostic: evaluation-only recording and a guarded four-member launcher (motar-z5f).

Protocol: docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md. Four fixed development members, in this order:
205/env5 native gains with the native and then the training-box collision model, then 205/env2 candidate gains with
the training-box and then the native model. Only the four rotor collision elements differ inside a pair
(tools/sitl_training_geometry.py); Policy F, inputs, start, 10 Hz loop, gate (skip_max_s 0.15), parameters and
timings are the dev08 runtime stack's, which runs unchanged: tools/sitl_s3b_dev08_prefix.run_episode() is called
with injected dependencies that only add recording.

Recording (evaluation only; nothing here reaches Policy F, the controller or a gate decision):
  - every decoded MAVLink arrival the receiver keeps (the runner's own Inbox, with its monotonic receive times),
    COMMAND_ACK/STATUSTEXT arrivals and a per-type count of all other arrivals;
  - every MAVLink *_send call on the connection, with its arguments and entry/return times on the same monotonic
    clock (taken inside the runner's existing send lock, around the pymavlink call);
  - the complete position/attitude/truth snapshot each checkpoint used, and the truth origin of the episode;
  - Gazebo contacts: an owned `stdbuf -oL gz topic -u -e /gazebo/default/physics/contacts` subscriber writing one
    line per physics step into the session log. Gazebo 11 publishes one Contacts message per world step (4 ms),
    empty or not, stamped with simulator time, so silence or a time gap is recorder loss, never "no contact".
    Before the first setpoint, a positive control requires a recorded navrl_quad-floor contact in the last
    250 contiguous steps. PX4 v1.14.3 lockstep sets its clock from HIL_SENSOR.time_usec, which the Gazebo plugin
    takes from world SimTime: PX4 hrt (time_boot_ms, ULog timestamps) and contact times share one time axis.

The default CLI refuses. --execute needs an approval file that names this scope, the clean HEAD and skip_max_s 0.15,
and a fresh attempt directory under ~/motar_tmp. Live behaviour is NOT_VERIFIED until such a run.
"""
import collections
import hashlib
import json
import math
import os
import pathlib
import re
import threading
import time
import types

STATUS = {"component": "agq_geometry_diagnostic", "live_verified": False, "execution_supported": True}

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
PROTOCOL = "agq-geometry-v1"
PREREG = "docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md"
SCOPE = ("AGQ geometry-v1: 4 fixed development members (205/env5 native gains: native then training-box collision; "
         "205/env2 candidate gains: training-box then native collision)")
OWN_SOURCES = ("tools/sitl_training_geometry.py", "tools/sitl_agq_geometry_diag.py",
               "tools/sitl_agq_geometry_launch.py", PREREG)
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit", "skip_max_s")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "agq_geometry_diag_20261106"
ATTEMPT_DIR = UNIT_DIR / "attempt01"     # single use: the attempt recorder refuses an existing directory
AS_LIMIT_MAX = 16 * 1024 ** 3
SKIP_MAX_S = 0.15                       # D-2026-11-05-2, as the dev08 unit
MEMBERS = ({"bars": 205, "env": 5, "arm": "native", "geometry": "native"},
           {"bars": 205, "env": 5, "arm": "native", "geometry": "training-box"},
           {"bars": 205, "env": 2, "arm": "candidate", "geometry": "training-box"},
           {"bars": 205, "env": 2, "arm": "candidate", "geometry": "native"})
PAIRS = ({"scene": "205/env5", "native": 0, "training_box": 1}, {"scene": "205/env2", "native": 3, "training_box": 2})
# 850 s + one member's 240 s watchdog + about 95 s worst-case cleanup = 1185 s < the launcher's 1200 s timeout
BUDGET_S = {"overall": 850.0, "density_main": 850.0}     # density_main is unused: no member has phase "main"
STDBUF_FILES = {"contact_stdbuf_executable": "/usr/bin/stdbuf",
                "contact_libstdbuf": "/usr/lib/x86_64-linux-gnu/coreutils/libstdbuf.so"}

CONTACT_TOPIC = "/gazebo/default/physics/contacts"
CONTACT_ARGV = ("stdbuf", "-oL", "gz", "topic", "-u", "-e", CONTACT_TOPIC)
CONTACT_ROLE = "contacts"
STEP_NS = 4000000                       # empty.world max_step_size 0.004
STEP_TOL_NS = 1000
VEHICLE = "navrl_quad::"
FLOORS = ("asphalt_plane::", "ground_plane::")   # the vehicle rests on asphalt_plane (top at z = 0.05 m)
BAR = re.compile(r"^bar_(\d+)::")
CONTROL_WINDOW, CONTROL_MIN_FLOOR, CONTROL_TAIL_BYTES = 250, 125, 8 << 20
COVER_BEFORE_S, COVER_AFTER_S = 1.0, 0.5
CONTROL_FRESH_S = 0.5
INTERVAL_JOIN_NS = 2 * STEP_NS          # contact messages at most two steps apart form one contact interval
EXTRA_TYPES = ("COMMAND_ACK", "STATUSTEXT")
# fixed reading rules (protocol section "Analysis")
RATE_RAD_S, Z_LOW_M, ERR_M = 5.0, 0.8, 0.5
WINDOW_S, PATH_GUARD_S, PATH_MAX_M = 0.5, 0.2, 0.3
TAKEOFF_RISE_M, TAKEOFF_WINDOW_S = 0.01, (-0.05, 0.3)
RATE_JOIN_S = 0.03                      # ULog angular-velocity groundtruth is logged every 10 ms
BOX_WINDOW_AFTER_S = 1.5                # the box member is checked over [t_c - 0.5, t_c + 1.5] s
EXPOSURE_STEP_S = 0.005                 # exposure: positions interpolated between checkpoints every 5 ms
EXPOSURE_NUMERIC_M = 1e-9               # floating-point slack only, not a tolerance
RATE_GAP_S = 0.05                       # a ULog rate gap longer than this (or a dropout) is missing coverage
PRE_CALM_S = 0.2                        # a native excursion in progress in [t_c - 0.2, t_c) blocks reproduction


def schedule():
    out = []
    for i, m in enumerate(MEMBERS):
        out.append(dict(m, phase="diag", index=i, name="%02d_diag_%dbars_env%d_%s_%s" % (
            i, m["bars"], m["env"], m["arm"], m["geometry"].replace("-", "_"))))
    return out


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


# ---------------------------------------------------------------- Gazebo contact text (ShortDebugString)

_TOKEN = re.compile(r'\s*(?:(?P<p>[{}:])|"(?P<s>(?:[^"\\]|\\.)*)"|(?P<w>[A-Za-z_][A-Za-z0-9_]*)'
                    r'|(?P<n>[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?|[-+]?inf|[-+]?nan))')
_ESC = {"n": "\n", "t": "\t", "r": "\r", '"': '"', "'": "'", "\\": "\\"}


class ParseError(ValueError):
    pass


def _unescape(s):
    return re.sub(r"\\([0-7]{1,3}|.)", lambda m: chr(int(m.group(1), 8)) if m.group(1)[0] in "01234567"
                  else _ESC.get(m.group(1), m.group(1)), s)


def _tokens(text):
    out, i, n = [], 0, len(text.rstrip())
    while i < n:
        m = _TOKEN.match(text, i)
        if m is None or m.end() == i:
            raise ParseError("unexpected text at %d: %r" % (i, text[i:i + 20]))
        i = m.end()
        kind = m.lastgroup
        out.append((kind, m.group(kind)))
    return out


def _scalar(kind, v):
    if kind == "s":
        return _unescape(v)
    if kind == "n":
        if re.fullmatch(r"[-+]?\d+", v):
            return int(v)
        return float(v)
    if kind == "w" and v in ("true", "false"):
        return v == "true"
    if kind == "w" and v.lstrip("-+") in ("inf", "nan"):
        return float(v)
    return v                                                    # an enum identifier


def parse_text(text):
    """Protobuf text format (one ShortDebugString line) -> {field: [values]}; sub-messages are dicts."""
    toks = _tokens(text)

    def fields(i, depth):
        out = {}
        while i < len(toks):
            kind, v = toks[i]
            if (kind, v) == ("p", "}"):
                if depth == 0:
                    raise ParseError("unbalanced }")
                return out, i
            if kind != "w":
                raise ParseError("field name expected, got %r" % (v,))
            if i + 1 >= len(toks):
                raise ParseError("truncated after %s" % v)
            k2, v2 = toks[i + 1]
            if (k2, v2) == ("p", ":"):
                if i + 2 >= len(toks) or toks[i + 2][0] == "p":
                    raise ParseError("value expected after %s:" % v)
                out.setdefault(v, []).append(_scalar(*toks[i + 2]))
                i += 3
            elif (k2, v2) == ("p", "{"):
                sub, j = fields(i + 2, depth + 1)
                if j >= len(toks) or toks[j] != ("p", "}"):
                    raise ParseError("unterminated %s {" % v)
                out.setdefault(v, []).append(sub)
                i = j + 1
            else:
                raise ParseError("':' or '{' expected after %s" % v)
        if depth:
            raise ParseError("unterminated message")
        return out, i
    return fields(0, 0)[0]


def _one(d, key, default=None):
    v = d.get(key)
    return v[0] if v else default


def _vec(d):
    return [float(_one(d, k, 0.0)) for k in ("x", "y", "z")]


def contacts_message(text):
    """One Contacts line -> {"t_ns", "t", "pairs": [...]}; ParseError for anything else."""
    msg = parse_text(text)
    if set(msg) - {"contact", "time"} or "time" not in msg:
        raise ParseError("not a gazebo.msgs.Contacts message: fields %s" % sorted(msg))
    t = msg["time"][0]
    t_ns = int(_one(t, "sec", 0)) * 1000000000 + int(_one(t, "nsec", 0))
    pairs = []
    for c in msg.get("contact", []):
        c1, c2 = _one(c, "collision1"), _one(c, "collision2")
        if not isinstance(c1, str) or not isinstance(c2, str):
            raise ParseError("contact without collision names")
        depth = [float(x) for x in c.get("depth", [])]
        force = [0.0, 0.0, 0.0]
        for w in c.get("wrench", []):
            b1 = _one(w, "body_1_wrench", {})
            f = _vec(_one(b1, "force", {}))
            force = [a + b for a, b in zip(force, f)]
        pairs.append({"c1": c1, "c2": c2, "points": len(c.get("position", [])),
                      "depth_max": max(depth) if depth else None, "body_1_force_sum": force,
                      "positions": [_vec(p) for p in c.get("position", [])][:4]})
    return {"t_ns": t_ns, "t": t_ns * 1e-9, "pairs": pairs}


def _kind(pair):
    """(vehicle collision, other collision, other kind) or None when the vehicle is not involved."""
    a, b = pair["c1"], pair["c2"]
    if not a.startswith(VEHICLE):
        a, b = b, a
    if not a.startswith(VEHICLE) or b.startswith(VEHICLE):
        return None
    if b.startswith(FLOORS):
        return a, b, "floor"
    if BAR.match(b):
        return a, b, "bar"
    return a, b, "other"


def vehicle_part(collision):
    return "rotor" if re.search(r"::rotor_\d+_collision$", collision) else (
        "base" if collision.endswith("::base_link_inertia_collision") else "other")


def _lines(path, tail_bytes=None):
    """Complete lines of the log (a last line without newline is still being written or was cut by a kill)."""
    with open(path, "rb") as fh:
        size = os.fstat(fh.fileno()).st_size
        start = 0 if tail_bytes is None else max(0, size - tail_bytes)
        fh.seek(start)
        data = fh.read()
    parts = data.split(b"\n")
    partial_tail = parts.pop() != b""
    if start > 0 and parts:
        parts.pop(0)                                           # the first line of a tail read may be cut
    return [p.decode("utf-8", "replace") for p in parts], partial_tail


def floor_control(path, *, window=CONTROL_WINDOW, min_floor=CONTROL_MIN_FLOOR, tail_bytes=CONTROL_TAIL_BYTES):
    """The pre-takeoff positive control: the last `window` messages are parseable, contiguous physics steps and at
    least `min_floor` of them hold a navrl_quad-floor contact. Returns (ok, detail)."""
    detail = {"window": window, "min_floor": min_floor, "path": str(path)}
    if not pathlib.Path(path).is_file():
        return False, dict(detail, problem="contact log missing")
    lines, _ = _lines(path, tail_bytes)
    msgs, other = [], 0
    for line in reversed(lines):
        if len(msgs) == window:
            break
        if not line.strip():
            continue
        try:
            msgs.append(contacts_message(line))
        except ParseError:
            other += 1
            if msgs:                                                   # a foreign line inside the window
                return False, dict(detail, problem="non-message line inside the control window")
    msgs.reverse()
    detail.update(messages=len(msgs), other_lines_after_window=other)
    if len(msgs) < window:
        return False, dict(detail, problem="only %d contact messages" % len(msgs))
    gaps = sum(1 for a, b in zip(msgs, msgs[1:]) if abs(b["t_ns"] - a["t_ns"] - STEP_NS) > STEP_TOL_NS)
    floor = [m for m in msgs if any(k and k[2] == "floor" for k in map(_kind, m["pairs"]))]
    names = sorted({"%s|%s" % (k[0], k[1]) for m in floor for k in map(_kind, m["pairs"]) if k and k[2] == "floor"})
    detail.update(first_t=msgs[0]["t"], last_t=msgs[-1]["t"], gaps=gaps, floor_messages=len(floor),
                  floor_pairs=names)
    if gaps:
        return False, dict(detail, problem="%d simulator-time gaps in the control window" % gaps)
    if len(floor) < min_floor:
        return False, dict(detail, problem="%d of %d messages hold a vehicle-floor contact" % (len(floor), window))
    return True, detail


def contact_audit(path, *, start_sim_s=None, final_sim_s=None, out_jsonl=None):
    """Full-log audit after a member: continuity, coverage, floor and bar contact events. Writes the non-empty
    message summaries to out_jsonl when given. Returns (summary, problems)."""
    summary = {"path": str(path), "messages": 0, "other_lines": 0, "other_samples": [], "partial_tail": False,
               "gaps": 0, "missing_steps": 0, "non_increasing": 0, "gap_samples": [], "gaps_in_window": 0,
               "non_increasing_in_window": 0, "window": None, "first_t": None,
               "last_t": None, "nonempty_messages": 0, "floor": None, "bar_events": [], "other_events": [],
               "first_bar_contact": None, "first_bar_contact_after_start": None}
    problems = []
    if not pathlib.Path(path).is_file():
        return summary, ["RECORDER_LOSS: contact log missing"]
    lines, summary["partial_tail"] = _lines(path)
    prev = None
    # Continuity is required over the reading window; outside it (boot, after-phase) gaps are reported only.
    lo = None if start_sim_s is None else start_sim_s - COVER_BEFORE_S
    hi = None if final_sim_s is None else final_sim_s + COVER_AFTER_S
    summary["window"] = [lo, hi]

    def in_window(a, b):
        return (lo is None or b >= lo) and (hi is None or a <= hi)
    floor = {"messages": 0, "first_t": None, "last_t": None, "last_t_before_start": None, "pairs": set()}
    bars = collections.OrderedDict()
    others = collections.OrderedDict()
    fh = open(out_jsonl, "x") if out_jsonl else None
    try:
        for line in lines:
            if not line.strip():
                continue
            try:
                m = contacts_message(line)
            except ParseError:
                summary["other_lines"] += 1
                if len(summary["other_samples"]) < 5:
                    summary["other_samples"].append(line[:200])
                continue
            summary["messages"] += 1
            if prev is not None:
                d = m["t_ns"] - prev
                if d < STEP_NS - STEP_TOL_NS:
                    summary["non_increasing"] += 1
                    summary["non_increasing_in_window"] += in_window(m["t"], prev * 1e-9)
                elif d > STEP_NS + STEP_TOL_NS:
                    summary["gaps"] += 1
                    summary["missing_steps"] += int(round(d / STEP_NS)) - 1
                    summary["gaps_in_window"] += in_window(prev * 1e-9, m["t"])
                    if len(summary["gap_samples"]) < 10:
                        summary["gap_samples"].append([prev * 1e-9, m["t"]])
            prev = m["t_ns"]
            summary["first_t"] = m["t"] if summary["first_t"] is None else summary["first_t"]
            summary["last_t"] = m["t"]
            if not m["pairs"]:
                continue
            summary["nonempty_messages"] += 1
            if fh is not None:
                fh.write(json.dumps(m) + "\n")
            seen_floor = False
            for p in m["pairs"]:
                k = _kind(p)
                if k is None:
                    continue
                if k[2] == "floor":
                    seen_floor = True
                    floor["pairs"].add("%s|%s" % (k[0], k[1]))
                    continue
                table = bars if k[2] == "bar" else others
                key = (k[0], k[1])
                e = table.get(key)
                if e is None:
                    e = table[key] = {"vehicle_collision": k[0], "other_collision": k[1],
                                      "vehicle_part": vehicle_part(k[0]), "first_t": m["t"], "last_t": m["t"],
                                      "messages": 0, "depth_max": None, "force_max_n": 0.0, "intervals": []}
                    if k[2] == "bar":
                        e["bar"] = int(BAR.match(k[1]).group(1))
                if e["intervals"] and m["t_ns"] - e["_last_ns"] <= INTERVAL_JOIN_NS:
                    e["intervals"][-1][1] = m["t"]
                else:
                    e["intervals"].append([m["t"], m["t"]])
                e["_last_ns"] = m["t_ns"]
                e["last_t"] = m["t"]
                e["messages"] += 1
                if p["depth_max"] is not None:
                    e["depth_max"] = max(p["depth_max"], e["depth_max"] or 0.0)
                e["force_max_n"] = max(e["force_max_n"], math.sqrt(sum(x * x for x in p["body_1_force_sum"])))
            if seen_floor:
                floor["messages"] += 1
                floor["first_t"] = m["t"] if floor["first_t"] is None else floor["first_t"]
                floor["last_t"] = m["t"]
                if start_sim_s is not None and m["t"] < start_sim_s:
                    floor["last_t_before_start"] = m["t"]          # the takeoff, not a touchdown after a crash
    finally:
        if fh is not None:
            fh.close()
    floor["pairs"] = sorted(floor["pairs"])
    summary["floor"] = floor
    for e in list(bars.values()) + list(others.values()):
        e.pop("_last_ns", None)
    summary["bar_events"] = sorted(bars.values(), key=lambda e: e["first_t"])
    summary["other_events"] = sorted(others.values(), key=lambda e: e["first_t"])
    if summary["bar_events"]:
        summary["first_bar_contact"] = dict(summary["bar_events"][0])
    if start_sim_s is not None:
        onsets = [(max(a, start_sim_s), e) for e in summary["bar_events"] for a, b in e["intervals"]
                  if b >= start_sim_s]
        if onsets:
            t, first = min(onsets, key=lambda x: x[0])
            summary["first_bar_contact_after_start"] = dict(first, onset_t=t, elapsed_s=t - start_sim_s)
    if summary["messages"] == 0:
        problems.append("RECORDER_LOSS: no contact messages")
    if summary["gaps_in_window"]:
        problems.append("RECORDER_LOSS: %d simulator-time gaps in the reading window (%d gaps, %d missing steps in "
                        "the log)" % (summary["gaps_in_window"], summary["gaps"], summary["missing_steps"]))
    if summary["non_increasing_in_window"]:
        problems.append("RECORDER_ORDER: %d non-increasing simulator times in the reading window"
                        % summary["non_increasing_in_window"])
    if summary["messages"] and start_sim_s is not None and summary["first_t"] > start_sim_s - COVER_BEFORE_S:
        problems.append("RECORDER_COVERAGE: first message %.3f s after start-%.1f s" % (summary["first_t"],
                                                                                      COVER_BEFORE_S))
    if summary["messages"] and final_sim_s is not None and summary["last_t"] < final_sim_s + COVER_AFTER_S:
        problems.append("RECORDER_COVERAGE: last message %.3f s before final+%.1f s" % (summary["last_t"],
                                                                                      COVER_AFTER_S))
    return summary, problems


# ---------------------------------------------------------------- recording wrappers (evaluation only)

class Tap:
    """What one member records; filled by the wrappers below and written by write_recording()."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.sends, self.extra, self.snapshots = [], [], []
        self.counts = collections.Counter()
        self.inbox = None
        self.session = None
        self.truth_origin = None
        self.control = None
        self.parameters = None
        self.contacts_started = False
        self.contact_argv = None


def _jsonable(x):
    if isinstance(x, bytes):
        return x.decode("latin-1")
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    return x


class _SendProxy:
    """conn.mav for the runner: every *_send call is timed and recorded; anything else is the real object's."""

    def __init__(self, mav, tap):
        object.__setattr__(self, "_mav", mav)
        object.__setattr__(self, "_tap", tap)

    def __getattr__(self, name):
        attr = getattr(self._mav, name)
        if not (name.endswith("_send") and callable(attr)):
            return attr
        tap = self._tap

        def timed(*args, **kw):
            entry = {"i": None, "call": name, "args": _jsonable(list(args)), "t_entry": tap.clock()}
            try:
                return attr(*args, **kw)
            except BaseException as e:
                entry["error"] = "%s: %s" % (type(e).__name__, e)
                raise
            finally:
                entry["t_return"] = tap.clock()
                entry["i"] = len(tap.sends)
                tap.sends.append(entry)
        return timed

    def __setattr__(self, name, value):
        setattr(self._mav, name, value)


class RecordingConn:
    """The pymavlink connection with a timed send proxy; receive, close and the rest are the real connection's."""

    def __init__(self, conn, tap):
        self._conn = conn
        self.mav = _SendProxy(conn.mav, tap)

    def __getattr__(self, name):
        return getattr(self._conn, name)


def recording_link(LK, tap, control, *, harness_invalid):
    """The link module for run_episode, with recording only: the same Inbox class and contents, a receiver that also
    keeps COMMAND_ACK/STATUSTEXT and per-type counts aside, a gate that keeps each checkpoint's snapshot and checks
    that the contact subscriber still runs, and the floor-contact positive control right after the parameters
    (before the first setpoint)."""
    ns = types.SimpleNamespace(**{k: getattr(LK, k) for k in dir(LK) if not k.startswith("__")})
    base_types = tuple(LK.STREAMS) + ("PARAM_VALUE", "HEARTBEAT")

    class TapInbox(LK.Inbox):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            tap.inbox = self

    class RecordingTransport(LK.PymavlinkTransport):
        def rx_loop(self, types=base_types):
            try:
                while not self.stop.is_set():
                    m = self.conn.recv_match(blocking=True, timeout=0.5)
                    if m is None:
                        continue
                    typ = m.get_type()
                    tap.counts[typ] += 1
                    if typ in types:
                        self.inbox.put({"type": typ, "src": (m.get_srcSystem(), m.get_srcComponent()),
                                        "fields": m.to_dict()})
                    elif typ in EXTRA_TYPES:
                        tap.extra.append({"type": typ, "src": [m.get_srcSystem(), m.get_srcComponent()],
                                          "fields": m.to_dict(), "recv": tap.clock()})
            except Exception as e:                            # noqa: BLE001 - latched, as the base receiver
                self.inbox.fail(e)

    class RecordingGate(LK.TelemetryGate):
        def checkpoint(self, k):
            if tap.session is not None:
                tap.session.check(required=(CONTACT_ROLE,))      # ProcessExited: recorder loss ends the member
            snap = super().checkpoint(k)
            tap.snapshots.append(snap)
            return snap

    def apply_and_verify(inbox, transport, overrides, **kw):
        out = LK.apply_and_verify(inbox, transport, overrides, **kw)
        tap.parameters = out                                       # kept even when the control fails
        t_entry = tap.clock()
        ok, detail = control(tap)
        tap.control = dict(detail, ok=ok, t_entry=t_entry, t_return=tap.clock())
        if not ok:
            raise harness_invalid("HARNESS_INVALID CONTACT_CONTROL %s" % detail.get("problem"))
        return out

    ns.Inbox, ns.PymavlinkTransport, ns.TelemetryGate = TapInbox, RecordingTransport, RecordingGate
    ns.apply_and_verify = apply_and_verify
    return ns


def recording_step(ST, tap):
    ns = types.SimpleNamespace(**{k: getattr(ST, k) for k in dir(ST) if not k.startswith("__")})

    def prefix_episode(B, policy, spec, recorded_frames, truth_origin):
        tap.truth_origin = [float(x) for x in truth_origin]
        return ST.PrefixEpisode(B, policy, spec, recorded_frames, truth_origin)
    ns.PrefixEpisode = prefix_episode
    return ns


def _px4_now(inbox):
    """The newest LOCAL_POSITION_NED time_boot_ms in seconds (PX4 time = simulator time), or None."""
    if inbox is None:
        return None
    for m in reversed(inbox.since(0)[0]):
        if m["type"] == "LOCAL_POSITION_NED":
            return m["fields"]["time_boot_ms"] * 1e-3
    return None


def session_control(tap, *, fresh_s=CONTROL_FRESH_S):
    """floor_control on the running session's contact log, after checking that the subscriber still runs; the
    control window must also be current: its last message within fresh_s of PX4's newest position time."""
    if tap.session is None or not tap.contacts_started:
        return False, {"problem": "contact subscriber not started"}
    try:
        tap.session.check(required=(CONTACT_ROLE,))
    except Exception as e:                                        # noqa: BLE001 - the control fails
        return False, {"problem": "contact subscriber: %s" % e}
    ok, detail = floor_control(tap.session.episode_dir / "logs" / (CONTACT_ROLE + ".log"))
    now = _px4_now(tap.inbox)
    detail.update(px4_time_s=now, fresh_s=fresh_s)
    if ok and (now is None or abs(detail["last_t"] - now) > fresh_s):
        return False, dict(detail, problem="control window not current (last %.3f s, PX4 %s s)" % (
            detail["last_t"], None if now is None else "%.3f" % now))
    return ok, detail


def launch_with_contacts(launch, tap, argv=CONTACT_ARGV):
    """gzserver, spawn and px4 exactly as the dev08 launch, then the owned contact subscriber."""
    def run(session, plan):
        spawned = launch(session, plan)
        tap.session = session
        tap.contact_argv = list(argv)
        session.start(CONTACT_ROLE, list(argv), env=plan["env"] if "env" in plan else plan["gzserver"]["env"],
                      cwd=session.episode_dir)
        tap.contacts_started = True
        return spawned
    return run


def _dump(path, rows):
    with open(path, "x") as fh:
        for r in rows:
            fh.write(json.dumps(r, default=str) + "\n")


def write_recording(tap, episode_dir):
    """Writes the member's recording files beside the session's own; returns {file: {bytes, sha256}}."""
    d = pathlib.Path(episode_dir) / "recording"
    d.mkdir()
    items = tap.inbox.since(0)[0] if tap.inbox is not None else []
    _dump(d / "mavlink_arrivals.jsonl", items)
    _dump(d / "mavlink_sends.jsonl", tap.sends)
    _dump(d / "mavlink_extra.jsonl", tap.extra)
    _dump(d / "snapshots.jsonl", tap.snapshots)
    with open(d / "recording.json", "x") as fh:
        json.dump({"clock": "time.monotonic (one clock for arrivals and sends)", "counts": dict(tap.counts),
                   "inbox_error": getattr(tap.inbox, "error", None), "truth_origin_lat_lon_alt": tap.truth_origin,
                   "contact_control": tap.control, "parameters": tap.parameters, "contact_argv": tap.contact_argv,
                   "contacts_started": tap.contacts_started}, fh, indent=1, default=str)
    out = {}
    for f in sorted(d.iterdir()):
        data = f.read_bytes()
        out[f.name] = {"bytes": len(data), "sha256": sha256_bytes(data)}
    return {"directory": str(d), "files": out, "arrivals": len(items), "sends": len(tap.sends),
            "snapshots": len(tap.snapshots)}


def recorder_problems(record, tap, audit_problems, manifest, velocity_mask):
    """Recording completeness for one member. For a VALID member: the positive control passed, every checkpoint
    snapshot and every velocity setpoint send is recorded, and the contact log is complete and covers the prefix."""
    p = list(audit_problems)
    if manifest is None:
        p.append("RECORDER_LOSS: recording files not written")
    if record.get("operational_status") != "VALID":
        return p
    if not (tap.control or {}).get("ok"):
        p.append("CONTACT_CONTROL: positive control missing or failed")
    if len(tap.snapshots) != record.get("observation_checks"):
        p.append("RECORDER_LOSS: %d snapshots for %s checks" % (len(tap.snapshots), record.get("observation_checks")))
    sent = sum(1 for s in tap.sends if s["call"] == "set_position_target_local_ned_send"
               and len(s["args"]) > 4 and s["args"][4] == velocity_mask and "error" not in s)
    if sent != record.get("commands_issued"):
        p.append("RECORDER_LOSS: %d velocity sends for %s commands" % (sent, record.get("commands_issued")))
    if getattr(tap.inbox, "error", None):
        p.append("RECORDER_LOSS: receiver error %s" % tap.inbox.error)
    return p


# ---------------------------------------------------------------- fixed reading (protocol section "Analysis")

def _intervals(series, start, pred, join_s=RATE_JOIN_S):
    """[[first, last]] PX4 times at or after start where pred holds; samples at most join_s apart join."""
    out = []
    for t, v in series:
        if t < start or not pred(v):
            continue
        if out and t - out[-1][1] <= join_s:
            out[-1][1] = t
        else:
            out.append([t, t])
    return out


def member_events(record, rows, audit, ulog=None, scene=None):
    """Events in seconds after the gate start (start_sim_time_s; PX4 time = simulator time).
    rows: checkpoints.jsonl; audit: contact_audit summary; ulog: ulog_series() output or None (rates NOT_RECORDED);
    scene: {"centers", "halves"} of the member's bars (MOTAR frame), for the clearance to a named bar."""
    t0 = record["start_sim_time_s"]
    rel = lambda t: None if t is None else t - t0                 # noqa: E731
    final = rel(record.get("final_sim_time_s"))
    spans = [(max(a, t0) - t0, b - t0, e["vehicle_part"], e.get("bar")) for e in audit.get("bar_events") or []
             for a, b in e["intervals"] if b >= t0]
    first = audit.get("first_bar_contact_after_start")
    rotor = sorted((a, bar) for a, b, part, bar in spans if part == "rotor")
    base = [a for a, b, part, bar in spans if part == "base"]
    rate_windows, rate_span, rate_gaps = None, None, None
    if ulog is not None:
        series = ulog["rates"]
        rate_windows = [[a - t0, b - t0] for a, b in _intervals(series, t0, lambda w: max(abs(x) for x in w)
                                                                > RATE_RAD_S)]
        times = [t - t0 for t, _ in series]
        rate_span = [times[0], times[-1]] if times else None
        rate_gaps = [[a, b] for a, b in zip(times, times[1:]) if b - a > RATE_GAP_S]
        rate_gaps += [[a - t0, a - t0 + d] for a, d in ulog.get("dropouts", [])]
    z_low = next((r["sim_s"] for r in rows if r["p_true"][2] < Z_LOW_M), None)
    err = next((r["sim_s"] for r in rows if math.dist(r["p"], r["p_true"]) > ERR_M), None)
    return {"outcome": record.get("outcome"), "crash_reason": record.get("crash_reason"), "final_elapsed_s": final,
            "first_bar_contact_s": None if first is None else first["elapsed_s"],
            "first_bar_contact": first,
            "first_rotor_bar_contact_s": rotor[0][0] if rotor else None,
            "first_rotor_bar": rotor[0][1] if rotor else None,
            "first_base_bar_contact_s": min(base) if base else None,
            "rate5_s": rate_windows[0][0] if rate_windows else None, "rate_windows": rate_windows,
            "rate_span_s": rate_span, "rate_gaps_s": rate_gaps,
            "rate_source": "ULog" if ulog is not None else "NOT_RECORDED",
            "z_below_0p8_s": rel(z_low), "error_above_0p5_s": rel(err),
            "contact_windows": sorted([a, b] for a, b, _, _ in spans),
            "path": [[r["k"], r["sim_s"] - t0, r["p_true"], _finite_or_none(r.get("clearance_m"))] for r in rows],
            "scene": scene}


def _finite_or_none(x):
    return x if isinstance(x, (int, float)) and math.isfinite(x) else None


def _contact_before(ev, t, width=WINDOW_S):
    return any(a <= t and b >= t - width for a, b in ev["contact_windows"])


def _overlapping(windows, lo, hi):
    return [w for w in windows if w[0] <= hi and w[1] >= lo]


def _covered(ev, lo, hi):
    """The member's ULog rates cover [lo, hi] (elapsed s) with no gap longer than RATE_GAP_S or dropout in it."""
    span = ev.get("rate_span_s")
    return bool(span and span[0] <= lo and span[1] >= hi and not _overlapping(ev.get("rate_gaps_s") or [], lo, hi))


def _path_at(path, t):
    """Truth position at elapsed time t, linearly interpolated between the checkpoints either side of t."""
    pts = [(tt, p) for k, tt, p, _ in path]
    for (t1, p1), (t2, p2) in zip(pts, pts[1:]):
        if t1 <= t <= t2:
            w = 0.0 if t2 == t1 else (t - t1) / (t2 - t1)
            return [a + w * (b - a) for a, b in zip(p1, p2)]
    return None


def _path_extrapolated(path, t):
    """Truth position at t extrapolated linearly from the last two checkpoints at or before t, or None."""
    before = [(tt, p) for k, tt, p, _ in path if tt <= t]
    if len(before) < 2 or before[-1][0] == before[-2][0]:
        return None
    (t1, p1), (t2, p2) = before[-2], before[-1]
    w = (t - t2) / (t2 - t1)
    return [b + w * (b - a) for a, b in zip(p1, p2)]


def _path_samples(path, lo, hi, step=EXPOSURE_STEP_S):
    """Truth positions every `step` seconds over [lo, hi] (clipped to the path) and at hi, linearly interpolated.
    The grid is integer-indexed, so no time drifts."""
    if not path:
        return []
    lo, hi = max(lo, path[0][1]), min(hi, path[-1][1])
    if hi < lo:
        return []
    n = int(math.floor((hi - lo) / step + 1e-9))
    times = [lo + i * step for i in range(n + 1)] + [hi]
    return [q for q in (_path_at(path, t) for t in times) if q is not None]


def bar_clearance(p, center, half):
    """The runner's box_hit distance to one bar: horizontal distance to its rectangle, infinite outside its height."""
    if abs(p[2] - center[2]) > half[2]:
        return math.inf
    dx, dy = max(abs(p[0] - center[0]) - half[0], 0.0), max(abs(p[1] - center[1]) - half[1], 0.0)
    return math.hypot(dx, dy)


def pair_reading(native, box):
    """The protocol's fixed reading of one pair (native collision model, training-box model). Either argument is
    None when that member is not evaluable. Times are seconds after each member's gate start."""
    if native is None or box is None:
        return {"reading": "NOT_EVALUABLE"}
    if native["rate_source"] != "ULog" or box["rate_source"] != "ULog":
        return {"reading": "NOT_EVALUABLE", "why": "angular rate not recorded"}
    fin_n = native["final_elapsed_s"]
    if not _covered(native, 0.0, fin_n):
        return {"reading": "NOT_EVALUABLE", "why": "the native member's ULog rates do not cover its prefix"}
    tc, bar = native["first_rotor_bar_contact_s"], native["first_rotor_bar"]
    windows = [w for w in native["rate_windows"] if w[0] <= fin_n]           # onsets after the final do not count
    onsets = [a for a, b in windows]
    in_progress = tc is not None and bool(_overlapping([w for w in windows if w[0] < tc], tc - PRE_CALM_S, tc))
    tr = None if in_progress or tc is None else next((a for a in onsets if tc <= a <= tc + WINDOW_S), None)
    if tr is None:                                                 # not reproduced
        disturbances = sorted(onsets + ([native["z_below_0p8_s"]] if native["z_below_0p8_s"] is not None else []))
        free = [t for t in disturbances if not _contact_before(native, t)]
        if free:
            return {"reading": "DISTURBANCE_WITHOUT_CONTACT", "onset_s": free[0],
                    "why": "native-model disturbance with no vehicle-bar contact in the 0.5 s before it"}
        return {"reading": "INCONCLUSIVE_NOT_REPRODUCED", "first_rotor_bar_contact_s": tc, "rate_onsets_s": onsets[:10],
                "excursion_in_progress_at_contact": in_progress}
    out = {"first_rotor_bar_contact_s": tc, "bar": bar, "native_rate5_in_window_s": tr}
    guard = tc - PATH_GUARD_S
    pa = {k: p for k, t, p, c in native["path"] if t <= guard}
    pb = {k: p for k, t, p, c in box["path"]}
    if set(pa) - set(pb):
        return dict(out, reading="INCONCLUSIVE_PATHS_DIVERGED", why="the box member ended before the native contact")
    gap = max((math.dist(pa[k], pb[k]) for k in pa), default=0.0)
    out["max_path_difference_m"] = gap
    if gap > PATH_MAX_M:
        return dict(out, reading="INCONCLUSIVE_PATHS_DIVERGED")
    fin_b = box["final_elapsed_s"]
    if fin_b is None or fin_b < tc + WINDOW_S:
        return dict(out, reading="INCONCLUSIVE_PATHS_DIVERGED", why="the box member ended inside the window")
    lo, hi = tc - WINDOW_S, min(tc + BOX_WINDOW_AFTER_S, fin_b)             # the box flight ends at its final
    if not _covered(box, lo, hi):
        return dict(out, reading="NOT_EVALUABLE", why="the box member's ULog rates do not cover its window")
    scene = native.get("scene")
    if scene is None or box.get("scene") != scene or bar is None or not 0 <= bar < len(scene["centers"]):
        return dict(out, reading="NOT_EVALUABLE", why="scene geometry missing, different or without the bar")
    c, h = scene["centers"][bar], scene["halves"][bar]
    # N's clearance at t_c: the smaller of the value interpolated between the checkpoints either side and the value
    # extrapolated from N's last two checkpoints at or before t_c, so neither the sampling phase (r3 M1) nor a stop
    # or bounce at the contact (r4 m1) can enlarge the reference
    interp, extrap = _path_at(native["path"], tc), _path_extrapolated(native["path"], tc)
    cands = [q for q in (interp, extrap) if q is not None]
    ref = min(bar_clearance(q, c, h) for q in cands) if cands else None
    seen = [bar_clearance(q, c, h) for q in _path_samples(box["path"], tc - WINDOW_S, tc + WINDOW_S)]
    out.update(native_clearance_to_bar_m=ref, box_clearance_to_bar_m=min(seen) if seen else None,
               native_clearance_interpolated_m=None if interp is None else bar_clearance(interp, c, h),
               native_clearance_extrapolated_m=None if extrap is None else bar_clearance(extrap, c, h))
    if ref is None or not seen or min(seen) > ref + EXPOSURE_NUMERIC_M:
        return dict(out, reading="INCONCLUSIVE_NOT_EXPOSED",
                    why="the box member did not come as close to the contacted bar as the native member did")
    rates = _overlapping(box["rate_windows"], lo, hi)
    contacts = _overlapping(box["contact_windows"], lo, hi)
    out.update(box_window_s=[lo, hi], box_rate_windows=rates[:5], box_contact_windows=contacts[:5])
    if box.get("outcome") == "crash" and fin_b <= tc + BOX_WINDOW_AFTER_S and not rates:
        return dict(out, reading="INCONCLUSIVE", why="the box member crashed inside its window")
    if rates:
        reading = "CONTACT_PERSISTS" if _contact_before(box, rates[0][0]) else "DISTURBANCE_PERSISTS_WITHOUT_CONTACT"
    elif contacts:
        reading = "INCONCLUSIVE"
    else:
        reading = "CONSISTENT_WITH_ROTOR_CONTACT"
    return dict(out, reading=reading)


def velocities_at(rows, snapshots, t):
    """At PX4 time t: the last checkpoint k at or before t, its truth velocity, the command in effect before it
    (sent at checkpoint k-1, to which the truth at k responds) and the new command sent at k. PX4 local NED, m/s."""
    if t is None:
        return None
    upto = [r for r in rows if r["sim_s"] <= t]
    if not upto:
        return None
    r = upto[-1]
    prev = next((x for x in reversed(upto[:-1]) if x.get("command")), None)
    snap = next((x for x in snapshots if x["k"] == r["k"]), None)
    truth = None
    if snap is not None:
        f = snap["truth"]["fields"]
        truth = [f["vx"] * 1e-2, f["vy"] * 1e-2]
    effect = None if prev is None else [float(prev["command"][4]), float(prev["command"][5])]
    new = [float(r["command"][4]), float(r["command"][5])] if r.get("command") else None
    out = {"k": r["k"], "truth_ned_m_s": truth, "command_in_effect_ned_m_s": effect,
           "command_in_effect_from_k": None if prev is None else prev["k"], "new_command_ned_m_s": new}
    if truth is not None and effect is not None:
        out["truth_minus_command_in_effect_m_s"] = math.hypot(truth[0] - effect[0], truth[1] - effect[1])
    return out


def takeoff_check(takeoff_t, heights, *, rise_m=TAKEOFF_RISE_M, window=TAKEOFF_WINDOW_S):
    """Clock identity check at takeoff: takeoff_t is the last recorded floor contact before the gate start. The
    ULog truth height (NED z) must first rise rise_m above its median over [takeoff_t - 2, takeoff_t - 1] s within
    window (seconds after takeoff_t)."""
    if takeoff_t is None or not heights:
        return {"ok": None, "why": "floor contact or ULog height NOT_RECORDED"}
    base = sorted(z for t, z in heights if takeoff_t - 2.0 <= t <= takeoff_t - 1.0)
    if not base:
        return {"ok": None, "why": "no ULog height before the takeoff contact"}
    median = base[len(base) // 2]
    rise = next((t for t, z in heights if t >= takeoff_t - 1.0 and median - z > rise_m), None)
    if rise is None:
        return {"ok": False, "takeoff_contact_t": takeoff_t, "ulog_rise_t": None}
    d = rise - takeoff_t
    return {"ok": bool(window[0] <= d <= window[1]), "takeoff_contact_t": takeoff_t, "ulog_rise_t": rise,
            "rise_minus_takeoff_contact_s": d, "window_s": list(window), "rise_m": rise_m}


def ulog_series(episode_dir):
    """{"rates": [(px4_s, (wx, wy, wz))], "heights": [(px4_s, z_ned)], "dropouts": [(px4_s, duration_s)], "path"}
    from the member's single PX4 ULog (vehicle_angular_velocity_groundtruth and vehicle_local_position_groundtruth,
    multi_id 0), as plain Python floats. None without exactly one ULog. Needs pyulog (the px4 environment)."""
    logs = sorted(pathlib.Path(episode_dir).rglob("*.ulg"))
    if len(logs) != 1:
        return None
    from pyulog import ULog                                        # noqa: PLC0415 - analysis environment only
    topics = ("vehicle_angular_velocity_groundtruth", "vehicle_local_position_groundtruth")
    u = ULog(str(logs[0]), list(topics))
    d = {x.name: x.data for x in u.data_list if x.multi_id == 0}
    a, h = d[topics[0]], d[topics[1]]
    rates = [(float(t) * 1e-6, (float(x), float(y), float(z)))
             for t, x, y, z in zip(a["timestamp"], a["xyz[0]"], a["xyz[1]"], a["xyz[2]"])]
    heights = [(float(t) * 1e-6, float(z)) for t, z in zip(h["timestamp"], h["z"])]
    dropouts = [(float(x.timestamp) * 1e-6, float(x.duration) * 1e-3) for x in u.dropouts]
    return {"rates": rates, "heights": heights, "dropouts": dropouts, "path": str(logs[0])}


def _jsonl(path):
    return [json.loads(x) for x in pathlib.Path(path).read_text().splitlines()] if pathlib.Path(path).is_file() else []


def _file_sha(path):
    path = pathlib.Path(path)
    return sha256_bytes(path.read_bytes()) if path.is_file() else None


def analyze(attempt_dir, *, series=ulog_series, M=None):
    """The fixed reading of a finished attempt (section 6 of the protocol), from the verified ledger and raw files.
    A host-fault or unverifiable-host attempt reads nothing. A member is evaluable only if it is VALID, has no
    record, recorder or timing problem, its raw contact log, checkpoints, snapshots and ULog still have the recorded
    SHA-256 and the recomputed contact audit is clean."""
    attempt_dir = pathlib.Path(attempt_dir)
    M = M or modules()
    ledger, final = M.AT.read_ledger(attempt_dir)
    void = (final.get("status") == "VOID_HOST_FAULT" or final.get("reason") == "FAULT_CHECK_ERROR"
            or final.get("fault_check_error") is not None or final.get("kernel_faults_end") is None
            or final.get("kernel_faults_end") != final.get("kernel_faults_start"))
    members = {}
    for rec in ledger:
        ep = attempt_dir / rec["name"]
        log = ep / "logs" / (CONTACT_ROLE + ".log")
        why = []
        if void:
            why.append("attempt %s (%s), kernel faults %s -> %s" % (final.get("status"), final.get("reason"),
                                                                   final.get("kernel_faults_start"),
                                                                   final.get("kernel_faults_end")))
        if rec.get("operational_status") != "VALID":
            why.append("member %s" % rec.get("operational_status"))
        for key in ("record_problems", "recorder_problems", "timing_problems"):
            if rec.get(key):
                why.append("%s: %s" % (key, rec[key]))
        if rec.get("operational_status") == "VALID":
            bound = {"logs/contacts.log": ((rec.get("contact_files") or {}).get("contacts.log") or {}).get("sha256"),
                     "recording/snapshots.jsonl": (((rec.get("recording") or {}).get("files") or {})
                                                   .get("snapshots.jsonl") or {}).get("sha256")}
            bound.update((rec.get("member_files") or {}).get("sha256") or {})
            for rel in ("logs/contacts.log", "recording/snapshots.jsonl", "checkpoints.jsonl"):
                if rel not in bound:
                    bound[rel] = None
            for rel, want in sorted(bound.items()):
                if want is None or want != _file_sha(ep / rel):
                    why.append("%s missing or differs from its recorded SHA-256" % rel)
        entry = {"name": rec["name"], "geometry": rec.get("geometry"), "operational_status":
                 rec.get("operational_status"), "evaluable": not why, "not_evaluable_because": why, "events": None}
        if not why:
            rows = _jsonl(ep / "checkpoints.jsonl")
            snaps = _jsonl(ep / "recording" / "snapshots.jsonl")
            audit, problems = contact_audit(log, start_sim_s=rec["start_sim_time_s"],
                                            final_sim_s=rec["final_sim_time_s"])
            if problems:
                entry.update(evaluable=False, not_evaluable_because=["recomputed audit: %s" % problems])
            try:
                ulog = series(ep)
                ulog_error = None if ulog is not None else "no single ULog"
            except Exception as e:                                # noqa: BLE001 - recorded, rates NOT_RECORDED
                ulog, ulog_error = None, "%s: %s" % (type(e).__name__, e)
            ev = member_events(rec, rows, audit, ulog, scene=rec.get("scene_geometry"))
            t0 = rec["start_sim_time_s"]
            ev["velocities"] = {
                "at_first_bar_contact": velocities_at(rows, snaps, None if ev["first_bar_contact_s"] is None
                                                      else t0 + ev["first_bar_contact_s"]),
                "at_rate5": velocities_at(rows, snaps, None if ev["rate5_s"] is None else t0 + ev["rate5_s"])}
            ev["clock_check"] = takeoff_check((audit.get("floor") or {}).get("last_t_before_start"),
                                              None if ulog is None else ulog["heights"])
            ev.update(ulog=None if ulog is None else ulog.get("path"), ulog_error=ulog_error,
                      ulog_dropouts=None if ulog is None else ulog.get("dropouts"), audit_problems_recomputed=problems)
            entry["events"] = ev
        members[rec["schedule_index"]] = entry
    pairs = []
    for p in PAIRS:
        n, x = members.get(p["native"]), members.get(p["training_box"])
        ev = [m["events"] if m and m["evaluable"] else None for m in (n, x)]
        reading = pair_reading(*ev)
        clock = ("SUPPORTED" if all(e is not None and e["clock_check"]["ok"] is True for e in ev)
                 else "LIMITED_VALIDITY")
        if clock != "SUPPORTED" and reading["reading"] != "NOT_EVALUABLE":
            reading = dict(reading, provisional_reading=reading["reading"], reading="NOT_READ_CLOCK_UNVERIFIED")
        pairs.append(dict(p, **reading, cross_clock_order=clock))
    for m in members.values():                                     # the scene is in the ledger; keep the output small
        if m["events"] is not None:
            m["events"].pop("scene", None)
    return {"protocol": PROTOCOL, "prereg": PREREG, "attempt": str(attempt_dir), "final": final, "void": void,
            "members": members, "pairs": pairs,
            "limits": "one boot per member and one pair per scene: a mechanism diagnostic, not a treatment effect, "
                      "gain ranking, S3b acceptance or flight-readiness result"}


# ---------------------------------------------------------------- the unit (guarded)

def _load(name, rel, root=ROOT):
    import importlib.util
    import sys
    saved = list(sys.path)
    try:
        spec = importlib.util.spec_from_file_location(name, str(pathlib.Path(root) / rel))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path[:] = saved
    return mod


def modules(root=ROOT):
    """Private module instances. The attempt recorder's protocol constants and schedule are this unit's; nothing
    else is changed, and no other instance of those modules is affected."""
    PR = _load("agq_prefix_runner", "tools/sitl_s3b_dev08_prefix.py", root)
    AT = _load("agq_attempt", "tools/sitl_dev08_attempt.py", root)
    OS = _load("agq_owned_session", "tools/sitl_owned_session.py", root)
    TG = _load("agq_training_geometry", "tools/sitl_training_geometry.py", root)
    AT.PROTOCOL, AT.AMENDMENT = PROTOCOL, PREREG              # header/ledger labels; SEED/EPISODE: the dev08 scenes
    AT.schedule = schedule                                     # AttemptRecorder and read_ledger use this order
    AT.SOURCES = tuple(AT.SOURCES) + OWN_SOURCES
    return types.SimpleNamespace(PR=PR, AT=AT, OS=OS, TG=TG)


def member_deps(base, M, tap, package, base_env, *, control=session_control, contact_argv=CONTACT_ARGV,
                stdbuf_files=STDBUF_FILES):
    """run_episode deps for one member: the dev08 deps with the package's spawn SDF and recording added."""
    deps = dict(base)

    def plan(session, world, name):
        pl = M.TG.launch_plan(M.OS, session.episode_dir, name, base_env, package)
        if pathlib.Path(pl["world_path"]) != pathlib.Path(world):
            raise RuntimeError("plan world differs from the written world")
        return pl

    def binding(pl):
        b = base["binding"](pl)
        b["geometry_receipt"] = pl.get("geometry_receipt")
        b["contact_subscriber"] = {"argv": list(contact_argv)}
        files = b.setdefault("files", {})
        for key, path in stdbuf_files.items():                    # the line-buffering preload is part of the run
            if not pathlib.Path(path).is_file():
                raise RuntimeError("runtime file missing: %s %s" % (key, path))
            files[key] = {"path": path, "sha256": _file_sha(path)}
        return b

    deps.update(plan=plan, binding=binding, launch=launch_with_contacts(base["launch"], tap, contact_argv),
                connect=lambda: RecordingConn(base["connect"](), tap),
                link=recording_link(base["link"], tap, control, harness_invalid=M.PR.HarnessInvalid),
                step=recording_step(base["step"], tap))
    return deps


def _core(binding):
    """The runtime binding without what is meant to differ between members: the world and the model SDF."""
    files = dict((binding or {}).get("files", {}))
    files.pop("world", None)
    files.pop("model_sdf", None)
    return {k: v["sha256"] for k, v in files.items()}


def execute(attempt_dir, approval, *, root=ROOT, M=None, base_deps=None, specs=None, B=None, policy=None, run=None,
            git_state=None, environment=None, make_recorder=None, prepare_package=None, control=session_control,
            contact_argv=CONTACT_ARGV, timing=None):
    """The fixed four-member unit under a recorded approval. Injection points are for the CPU fixtures."""
    import signal
    import sys
    M = M or modules(root)
    PR, AT, TG = M.PR, M.AT, M.TG
    if not isinstance(approval, dict) or any(approval.get(k) in (None, "") for k in APPROVAL_KEYS):
        raise PermissionError("no recorded approval (%s)" % ", ".join(APPROVAL_KEYS))
    if approval["scope"] != SCOPE:
        raise PermissionError("approval scope differs: %r" % approval["scope"])
    if type(approval["skip_max_s"]) is not float or approval["skip_max_s"] != SKIP_MAX_S:
        raise PermissionError("skip_max_s must be %.2f (D-2026-11-05-2)" % SKIP_MAX_S)
    git_state = git_state or AT.git_state
    git0 = git_state(root)
    if approval["commit"] != git0["head"] or git0["dirty"]:
        raise PermissionError("approval names %s; tree is %s%s" % (approval["commit"], git0["head"],
                                                                   " (dirty)" if git0["dirty"] else ""))
    env_problems = AT.environment_problems(*(environment or (AT.process_environment(), AT.CORES)))
    if env_problems:
        raise PermissionError("process environment: %s" % env_problems)
    attempt_dir = pathlib.Path(attempt_dir)

    def on_signal(signum, frame):
        raise PR.RunnerTerminated("runner received signal %d" % signum)

    saved = {}
    if threading.current_thread() is threading.main_thread():
        for sig in (signal.SIGTERM, signal.SIGHUP):
            saved[sig] = signal.signal(sig, on_signal)
    rec = None
    current = None
    try:
        cfg = _load("agq_cfg", "tools/sitl_s3b_dev08.py", root)
        params = {arm: cfg.prepare(root, arm=arm)["selected_params"] for arm in ("native", "candidate")}
        prep_summary = None
        if specs is None:
            specs, windows, prep = PR.scene_specs(root)
            prep_summary = {k: prep[k] for k in ("status", "verified_inventory_files", "inventory_sha256")}
        else:
            specs, windows = specs
        if B is None:
            sys.path.insert(0, str(TOOLS))
            import sitl_bridge as B                               # noqa: N811
            B.setup(specs[(70, 0)]["receipt_env"])                 # as the dev08 unit: one setup, 70-bar receipt
            resolved = pathlib.Path(list(sys.modules["aerial_gym"].__path__)[0]).resolve()
            if not str(resolved).startswith(str(pathlib.Path(root).resolve()) + "/"):
                raise PermissionError("aerial_gym resolves outside this tree: %s" % resolved)
            policy = B.Policy()
            import torch
            torch.set_num_threads(1)
            if torch.get_num_threads() != 1:
                raise PermissionError("torch uses %d threads; amendment4 requires 1" % torch.get_num_threads())
            if PR.PYLINK not in sys.path:
                sys.path.append(PR.PYLINK)
            import pymavlink  # noqa: F401
        warm = PR.warm_up(B, policy, specs[(70, 0)])
        extra = {"inputs": prep_summary, "warm_up": warm, "skip_max_s": SKIP_MAX_S, "members": schedule(),
                 "pairs": PAIRS, "contact_argv": list(contact_argv),
                 "packages": {n: PR.package_manifest(n) for n in ("pymavlink", "rl_games")},
                 "bridge_setup": "one setup with the 70-bar receipt's NAVRL_* snapshot, as the dev08 unit"}
        P = _load("agq_prefix_contract", "tools/sitl_dev08_prefix_contract.py", root)
        rec = (make_recorder or (lambda d, scope: AT.AttemptRecorder(
            d, scope=scope, root=root, f_checkpoint=pathlib.Path(B.F_CKPT), f_sha=P.F_SHA, sources=AT.SOURCES,
            budget_s=BUDGET_S, extra_header=extra)))(attempt_dir, {"approval": dict(approval), "scope": SCOPE})
        rec.open()
        prepare_package = prepare_package or TG.prepare
        packages = {g: prepare_package(attempt_dir / "models" / g, g) for g in ("native", "training-box")}
        base = base_deps or PR.real_deps()
        base_env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1")
        velocity_mask = PR.historical_masks()[1]
        run = run or PR.run_episode
        core_ref = None
        for member in AT.schedule():
            rec.check_budget()
            host = rec.check_host()
            if git_state(root) != git0:
                rec.abort("SOURCE_CHANGED")
                break
            key = (member["bars"], member["env"])
            package = pathlib.Path(packages[member["geometry"]]["output_directory"])
            tap = Tap()
            deps = member_deps(base, M, tap, package, base_env, control=control, contact_argv=contact_argv)
            episode_dir = attempt_dir / member["name"]
            current = member
            manifest, write_error = None, None
            try:
                r = run(member, specs[key], windows[key]["recorded_frames"], params[member["arm"]], deps=deps,
                        B=B, policy=policy, episode_dir=episode_dir, gate_opts={"skip_max_s": SKIP_MAX_S},
                        **({"timing": timing} if timing else {}))
            finally:                                              # also on a termination signal
                if episode_dir.is_dir():
                    try:
                        manifest = write_recording(tap, episode_dir)
                    except Exception as e:                        # noqa: BLE001 - recorded as recorder loss
                        write_error = "%s: %s" % (type(e).__name__, e)
            after_ok, after = rec.host_check()
            log = episode_dir / "logs" / (CONTACT_ROLE + ".log")
            out_jsonl = episode_dir / "recording" / "contacts.jsonl" if manifest else None
            audit, audit_problems = contact_audit(log, start_sim_s=r.get("start_sim_time_s"),
                                                  final_sim_s=r.get("final_sim_time_s"), out_jsonl=out_jsonl)
            contact_files = {"contacts.log": {"sha256": _file_sha(log)},
                             "contacts.jsonl": {"sha256": _file_sha(out_jsonl) if out_jsonl else None}}
            ulogs = sorted(episode_dir.rglob("*.ulg")) if episode_dir.is_dir() else []
            member_files = {"sha256": dict({"checkpoints.jsonl": _file_sha(episode_dir / "checkpoints.jsonl")},
                                           **{str(u.relative_to(episode_dir)): _file_sha(u) for u in ulogs}),
                            "ulogs": [str(u.relative_to(episode_dir)) for u in ulogs]}
            spec = specs[key]
            scene_geometry = {"centers": [[float(x) for x in c] for c in spec["centers"]],
                              "halves": [[float(x) for x in h] for h in spec["halves"]],
                              "p0": [float(x) for x in spec["p0"]], "yaw0": float(spec["yaw0"])}
            problems = recorder_problems(r, tap, audit_problems, manifest, velocity_mask)
            if write_error:
                problems.append("RECORDER_LOSS: %s" % write_error)
            timing_problems = PR.smoke_problems(r) if r.get("operational_status") == "VALID" else []
            r = dict(r, geometry=member["geometry"], package=packages[member["geometry"]], recording=manifest,
                     contact_files=contact_files, member_files=member_files, scene_geometry=scene_geometry,
                     contacts={k: audit[k] for k in audit if k not in ("bar_events", "other_events")},
                     bar_events=audit["bar_events"][:50], other_events=audit["other_events"][:20],
                     recorder_problems=problems, timing_problems=timing_problems,
                     host_idle_before=host, host_idle_after=dict(after, ok=after_ok))
            core = _core(r.get("runtime_binding"))
            sdf = ((r.get("runtime_binding") or {}).get("files") or {}).get("model_sdf", {}).get("sha256")
            if r.get("operational_status") == "VALID" and core:
                core_ref = core_ref or core
                if core != core_ref or sdf != packages[member["geometry"]]["output_sdf_sha256"]:
                    r = dict(r, operational_status="BINDING_CHANGED")
            if git_state(root) != git0:
                r = dict(r, operational_status="SOURCE_CHANGED")
            rec.append(r)
            current = None
            if rec.status is None and problems:
                rec.abort("RECORDER_LOSS", problems=problems)
            if rec.status is None and timing_problems:
                rec.abort("TIMING_GUARD", problems=timing_problems)
            if rec.status is not None:
                break
        rec.finish()
    except BaseException as e:                                    # noqa: BLE001 - recorded, then re-raised
        if rec is not None and rec.t0 is not None:
            rec.abort("RUNNER_EXCEPTION", error="%s: %s" % (e.__class__.__name__, e),
                      interrupted_member=current["name"] if current is not None and rec.next == current["index"]
                      else None)
        raise
    finally:
        for sig, h in saved.items():
            signal.signal(sig, h)
    return rec.status


def envelope_problems(attempt_dir, *, attempt=ATTEMPT_DIR, as_max=AS_LIMIT_MAX, ppid=None):
    """--execute runs only inside the launcher's envelope: the fixed single-use attempt directory, an address-space
    limit of at most 16 GiB and an outer `timeout` as the parent process (nice and taskset exec in place)."""
    import resource
    p = []
    if pathlib.Path(attempt_dir).absolute() != pathlib.Path(attempt):
        p.append("attempt directory must be %s" % attempt)
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    if soft == resource.RLIM_INFINITY or soft > as_max:
        p.append("RLIMIT_AS must be set and at most %d bytes" % as_max)
    try:
        comm = pathlib.Path("/proc/%d/comm" % (ppid or os.getppid())).read_text().strip()
    except OSError:
        comm = None
    if comm != "timeout":
        p.append("parent process is %r, not timeout" % comm)
    return p


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--execute", action="store_true", help="run the four recorded members under an approval")
    parser.add_argument("--approval-file", type=pathlib.Path)
    parser.add_argument("--attempt-dir", type=pathlib.Path)
    parser.add_argument("--analyze", type=pathlib.Path, help="read a finished attempt (CPU only, px4 environment)")
    parser.add_argument("--out", type=pathlib.Path, help="new analysis JSON file for --analyze")
    args = parser.parse_args(argv)
    if args.analyze is not None:
        if args.out is None or args.execute:
            parser.error("--analyze needs --out and excludes --execute")
        result = analyze(args.analyze)
        with open(args.out, "x") as fh:
            json.dump(result, fh, indent=1, allow_nan=False)
            fh.write("\n")
        print(json.dumps(result["pairs"]))
        return 0
    if not args.execute:
        print(json.dumps(dict(STATUS, refused="--execute and a scoped approval are required", scope=SCOPE)))
        return 2
    if args.approval_file is None or args.attempt_dir is None:
        parser.error("--execute requires --approval-file and --attempt-dir")
    M = None
    try:
        if threading.current_thread() is not threading.main_thread():
            raise PermissionError("execute must run in the main thread")
        envelope = envelope_problems(args.attempt_dir)
        if envelope:                                              # only the launcher's envelope may run the unit
            raise PermissionError("envelope: %s" % envelope)
        raw = args.approval_file.read_bytes()
        approval = json.loads(raw)
        if not isinstance(approval, dict):
            raise PermissionError("approval file is not a JSON object")
        approval = dict(approval, approval_file={"path": str(args.approval_file.resolve()),
                                                 "sha256": sha256_bytes(raw)})
        M = modules()
        status = execute(args.attempt_dir, approval, M=M)
    except BaseException as e:                                    # noqa: BLE001 - reported, exit status 1
        terminated = M is not None and isinstance(e, M.PR.RunnerTerminated)
        if not (terminated or isinstance(e, (Exception, KeyboardInterrupt))):
            raise                                                 # e.g. SessionTerminated, as the dev08 CLI
        print(json.dumps({"status": "REFUSED_OR_ABORTED", "error": "%s: %s" % (type(e).__name__, e),
                          "attempt_dir": str(args.attempt_dir)}))
        return 1
    print(json.dumps({"status": status, "attempt_dir": str(args.attempt_dir), "scope": SCOPE}))
    return 0 if status == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
