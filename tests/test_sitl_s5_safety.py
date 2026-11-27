"""CPU tests for tools/sitl_s5_safety.py (S5 safety v2). No SITL, PX4, Gazebo or GPU is started: the live path runs
against a fake owned session and a fake MAVLink link carrying a small PX4 model on a fake clock.

The fake PX4 follows v1.14.3 where the review r1 found it mattered: HEARTBEAT at 1 Hz with +-20 ms jitter, rc_update
re-publishing unchanged RC only every 300 ms, kill switch edge-triggered after two identical frames, MANUAL_CONTROL
echo of the selected source with the kill switch bits, STATUSTEXT at the events. Every negative (a broken fake PX4, a
refused input, a violated criterion) is paired with the positive control that passes, and counts are asserted non-zero.
PUBLIC_CPU-safe: the module is loaded by path; sys.path and the tools modules the loading adds are restored at once, the
two environment variables pymavlink's mavutil reads (MAVLINK20, MAVLINK_DIALECT) are set only while it is imported and
then restored exactly, and no attribute of a shared module is patched.

Amendment 1: every fake link carries pymavlink 2.4.50's real addressing, target_component 0 after a heartbeat; the
fake-PX4 live path pins its autopilot through a real mavutil connection reading real MAVLink 2 bytes."""
import contextlib
import importlib.util
import io
import json
import math
import os
import pathlib
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
PX4_SRC = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/src")
NS = types.SimpleNamespace


@contextlib.contextmanager
def isolated_imports():
    saved_path, before = list(sys.path), set(sys.modules)
    try:
        yield
    finally:
        sys.path[:] = saved_path
        for name in set(sys.modules) - before:
            f = getattr(sys.modules.get(name), "__file__", None) or ""
            if f.startswith(str(TOOLS)):
                del sys.modules[name]


def _load():
    with isolated_imports():
        spec = importlib.util.spec_from_file_location("sitl_s5_safety_under_test", str(TOOLS / "sitl_s5_safety.py"))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    return mod


S5 = _load()
SF = S5.SF
CODE = {"MANUAL": 1 << 16, "POSCTL": 3 << 16, "OFFBOARD": 6 << 16, "AUTO_LOITER": (4 << 16) | (3 << 24),
        "AUTO_LAND": (4 << 16) | (6 << 24), "AUTO_RTL": (4 << 16) | (5 << 24)}
INT_NAMES = {n for n, t, _, _, _ in S5.all_params() if t == "INT32"}
SCEN = {s["id"]: s for s in S5.scenarios()}
OFF = "OFFBOARD"
KILL_OFF, KILL_ON = S5.SWITCH_OFF << S5.KILL_SHIFT, S5.SWITCH_ON << S5.KILL_SHIFT


PYLINK = pathlib.Path("/home/fair/motar_tmp/sitl_plan_20261104/pylink")   # the path sitl_s3b_closed_loop appends


def real_mavlink():
    """Amendment 1: pymavlink's MAVLink 2 'all' dialect, the module the live link decodes with, and pymavlink's mavutil
    ((None, None) when pymavlink is not installed). mavutil writes os.environ: MAVLINK20 is set during the import, so it
    starts on the MAVLink 2 'all' dialect (the live link switches to it at PX4's first MAVLink 2 byte, mavutil.py:291-298)
    and never writes the environment again; os.environ, sys.path and sys.modules are restored."""
    saved, before, env = list(sys.path), set(sys.modules), dict(os.environ)
    try:
        if PYLINK.is_dir():
            sys.path.append(str(PYLINK))
        os.environ["MAVLINK20"], os.environ["MAVLINK_DIALECT"] = "1", "all"
        from pymavlink.dialects.v20 import all as dialect
        from pymavlink import mavutil
        assert mavutil.mavlink is dialect and mavutil.mavlink.WIRE_PROTOCOL_VERSION == "2.0"
        return dialect, mavutil
    except ImportError:
        return None, None
    finally:
        sys.path[:] = saved
        for key in set(os.environ) - set(env):
            del os.environ[key]
        for key, value in env.items():
            if os.environ.get(key) != value:
                os.environ[key] = value
        for name in set(sys.modules) - before:
            if name == "pymavlink" or name.startswith("pymavlink."):
                del sys.modules[name]


MAV, MAVUTIL = real_mavlink()


def px4_wire(autopilot=True, telemetry=True, others=True):
    """Real MAVLink 2 bytes as the S5 link receives them: (others) a GCS heartbeat (system 255) and a camera-component
    heartbeat of the vehicle (1, 100; pymavlink's probably_vehicle_heartbeat accepts it and locks sysid 1 on it), then
    PX4's autopilot heartbeat (1, 1, quadrotor, MAV_AUTOPILOT_PX4) unless autopilot is False, then (telemetry) from
    (1, 1) LOCAL_POSITION_NED, ATTITUDE, HIL_STATE_QUATERNION, COMMAND_ACK and a second heartbeat, and (others) one more
    camera heartbeat. others=False is what smoke01's link received: PX4 alone."""
    def packed(src, *msgs):
        tx = MAV.MAVLink(None, srcSystem=src[0], srcComponent=src[1])
        return b"".join(bytes(m.pack(tx)) for m in msgs)
    hb = lambda t, a, mode=0: MAV.MAVLink_heartbeat_message(t, a, 0, mode, 3, 3)          # noqa: E731
    wire = b""
    if others:
        wire += packed((255, 190), hb(MAV.MAV_TYPE_GCS, MAV.MAV_AUTOPILOT_INVALID))
        wire += packed((1, 100), hb(MAV.MAV_TYPE_CAMERA, MAV.MAV_AUTOPILOT_PX4))
    if autopilot:
        wire += packed((1, 1), hb(MAV.MAV_TYPE_QUADROTOR, MAV.MAV_AUTOPILOT_PX4))
    if telemetry:
        wire += packed((1, 1), MAV.MAVLink_local_position_ned_message(12345, 1.0, 2.0, -1.0, 0.1, 0.2, 0.3),
                       MAV.MAVLink_attitude_message(12340, 0.01, 0.02, 1.5, 0.0, 0.0, 0.1),
                       MAV.MAVLink_hil_state_quaternion_message(0, [1.0, 0.0, 0.0, 0.0], 0, 0, 0, 470000000,
                                                                80000000, 101000, 10, 20, -30, 0, 0, 0, 0, 0),
                       MAV.MAVLink_command_ack_message(400, 1),
                       hb(MAV.MAV_TYPE_QUADROTOR, MAV.MAV_AUTOPILOT_PX4, 6 << 16))
        if others:
            wire += packed((1, 100), hb(MAV.MAV_TYPE_CAMERA, MAV.MAV_AUTOPILOT_PX4))
    return wire


def wire_connection(directory, wire):
    """A real pymavlink connection (mavutil.mavlink_connection -> mavlogfile, robust parsing, raw bytes without tlog
    timestamps) over the bytes: the same mavfile.recv_msg/post_message path as the live UDP link."""
    path = pathlib.Path(directory) / "px4_wire.raw"
    path.write_bytes(wire)
    return MAVUTIL.mavlink_connection(str(path), notimestamps=True)
REAL_TYPES = ["HEARTBEAT", "STATUSTEXT", "PARAM_VALUE", "FILE_TRANSFER_PROTOCOL", "BAD_DATA", "UNKNOWN_54321"]


def real_messages(src=(42, 7)):
    """Real pymavlink message objects decoded from wire bytes by a robust-parsing MAVLink receiver, as recv_match
    returns them: a heartbeat, a STATUSTEXT with a non-UTF-8 byte (char array), a PARAM_VALUE (char id), a uint8_t[251]
    payload, a CRC-broken heartbeat (BAD_DATA, data is a bytearray) and an unknown message id (UNKNOWN_54321, data is a
    bytearray). The default source is not the autopilot (1, 1), so the harness state is not touched."""
    tx = MAV.MAVLink(None, srcSystem=src[0], srcComponent=src[1])
    wire = b"".join(bytes(m.pack(tx)) for m in (
        MAV.MAVLink_heartbeat_message(2, 12, 129, 6 << 16, 4, 3),
        MAV.MAVLink_statustext_message(2, b"Failsafe \xff activated", 0, 0),
        MAV.MAVLink_param_value_message(b"NAV_DLL_ACT", 1.5, 6, 100, 3),
        MAV.MAVLink_file_transfer_protocol_message(0, 1, 1, list(range(251)))))
    bad = bytearray(MAV.MAVLink_heartbeat_message(2, 12, 129, 0, 4, 3).pack(tx))
    bad[-1] ^= 0xFF                                                      # broken CRC -> BAD_DATA
    unknown = bytearray(MAV.MAVLink_heartbeat_message(2, 12, 129, 0, 4, 3).pack(tx))
    unknown[7:10] = (54321).to_bytes(3, "little")                        # MAVLink 2 msgid -> UNKNOWN_54321
    rx = MAV.MAVLink(None)
    rx.robust_parsing = True
    return rx.parse_buffer(wire + bytes(bad) + bytes(unknown))


def unhex(value):
    """Inverse of the recorder's {"bytes_hex": ...} tag (the tests' decoder)."""
    if isinstance(value, dict):
        if set(value) == {"bytes_hex"}:
            return bytes.fromhex(value["bytes_hex"])
        return {k: unhex(v) for k, v in value.items()}
    if isinstance(value, list):
        return [unhex(v) for v in value]
    return value


def plain(d):
    return {k: bytes(v) if isinstance(v, (bytes, bytearray)) else v for k, v in d.items()}


def count_hex(value):
    if isinstance(value, dict):
        return (set(value) == {"bytes_hex"}) + sum(count_hex(v) for v in value.values())
    return sum(count_hex(v) for v in value) if isinstance(value, list) else 0


def jitter(k):
    return ((k * 7) % 5 - 2) * 0.01                                     # -0.02 .. +0.02 s


# ------------------------------------------------------------------------------------------- parameter set

class TestParameterSet(unittest.TestCase):
    REQUESTED = ("GF_ACTION", "GF_MAX_HOR_DIST", "GF_MAX_VER_DIST", "GF_SOURCE", "GF_PREDICT", "COM_OF_LOSS_T",
                 "COM_OBL_RC_ACT", "COM_RC_OVERRIDE", "COM_RC_STICK_OV", "COM_RC_IN_MODE", "NAV_RCL_ACT",
                 "COM_RCL_EXCEPT", "NAV_DLL_ACT", "COM_KILL_DISARM", "RC_MAP_KILL_SW")

    def test_requested_names_and_consistency(self):
        self.assertEqual(sorted(n for n, _, _, _, _ in S5.SAFETY_PARAMS), sorted(self.REQUESTED))
        self.assertEqual(S5.table_problems(), [])
        self.assertTrue(set(S5.USER_DECISIONS) <= set(S5.param_values()))
        for n in ("COM_RC_LOSS_T", "RC_KILLSWITCH_TH", "GF_PREDICT"):          # review r1 m5
            self.assertIn(n, S5.USER_DECISIONS)
        self.assertEqual(len(S5.UNTESTED_REAL_VEHICLE_PARAMS), 8)

    def test_inconsistent_tables_are_refused(self):
        def variant(**change):
            return tuple((n, t, change.get(n, v), r, m) for n, t, v, r, m in S5.all_params())
        cases = [(dict(COM_OBL_RC_ACT=6), "COM_OBL_RC_ACT"), (dict(GF_ACTION=4), "GF_ACTION"),
                 (dict(NAV_DLL_ACT=0), "NAV_DLL_ACT"), (dict(GF_MAX_HOR_DIST=25.0), "GF_MAX_HOR_DIST"),
                 (dict(GF_MAX_VER_DIST=0.8), "GF_MAX_VER_DIST"), (dict(COM_RC_IN_MODE=0), "COM_RC_IN_MODE"),
                 (dict(COM_RC_OVERRIDE=1), "COM_RC_OVERRIDE"), (dict(COM_RCL_EXCEPT=4), "COM_RCL_EXCEPT"),
                 (dict(RC_MAP_KILL_SW=8), "RC_MAP_KILL_SW"), (dict(RC_MAP_KILL_SW=2), "RC_MAP_KILL_SW")]
        for change, needle in cases:
            p = S5.table_problems(variant(**change))
            self.assertTrue(any(needle in x for x in p), (change, p))
        self.assertEqual(S5.table_problems(variant()), [])

    def test_int32_byte_cast_round_trip(self):
        n = 0
        for name, t, v, _, _ in S5.all_params():
            val, typ = S5.encode_param(t, v)
            self.assertEqual(typ, S5.MAV_PARAM_TYPE[t])
            self.assertEqual(SF.decode_param(NS(param_value=val, param_type=typ)), float(v), name)
            if t == "INT32" and v:
                self.assertNotEqual(val, float(v))
                n += 1
        self.assertGreater(n, 5)
        for bad in (-1, 1 << 23, 2.5):
            with self.assertRaises(ValueError):
                S5.encode_param("INT32", bad)
        with self.assertRaises(ValueError):
            S5.encode_param("UINT8", 1)
        self.assertEqual(S5.encode_param("INT32", (1 << 23) - 1)[1], 6)

    @unittest.skipUnless((PX4_SRC / "modules").is_dir(), "PX4 v1.14.3 tree not on this host")
    def test_types_and_lines_match_px4_definitions(self):
        def line(ref):
            f, n = ref.split(":")
            p = PX4_SRC / "modules" / f
            p = p if p.is_file() else PX4_SRC / f
            return p.read_text().splitlines()[int(n) - 1]
        for name, t, _, ref, _ in S5.all_params():
            want = "PARAM_DEFINE_%s(%s," % ("INT32" if t == "INT32" else "FLOAT", name)
            self.assertTrue(line(ref).startswith(want), (ref, line(ref)))
        for name, ref, _ in S5.UNTESTED_REAL_VEHICLE_PARAMS:
            self.assertIn("(%s," % name, line(ref), ref)
        gf = line("navigator/geofence_params.c:62")                          # negative: a wrong type
        self.assertTrue(gf.startswith("PARAM_DEFINE_INT32(GF_ACTION,"))
        self.assertFalse(gf.startswith("PARAM_DEFINE_FLOAT(GF_ACTION,"))


class TestScenarios(unittest.TestCase):
    def test_schedules(self):
        self.assertEqual(S5.SCHEDULE, ("N0", "OBL", "GFH", "GFV", "OVJ", "OVR", "RCL", "DLL", "KIL", "FDS"))
        self.assertEqual(S5.SMOKE_SCHEDULE, ("N0", "N0R", "N0J"))
        self.assertEqual([x["scenario"] for x in S5.smoke_schedule()], list(S5.SMOKE_SCHEDULE))
        self.assertTrue(all(SCEN[s].get("smoke_only") for s in ("N0R", "N0J")))
        self.assertEqual((SCEN["N0R"]["path"], SCEN["N0J"]["path"]), ("rc", "joy"))

    def test_criteria_follow_the_parameters(self):
        self.assertEqual(SCEN["OBL"]["expect"], "AUTO_LOITER")
        self.assertEqual((SCEN["RCL"]["expect"], SCEN["DLL"]["expect"]), ("AUTO_LAND", "AUTO_LAND"))
        self.assertAlmostEqual(SCEN["OBL"]["earliest_s"], 0.2)
        self.assertAlmostEqual(SCEN["DLL"]["deadline_s"], 5 + 2.5 + 1.25 + 5 + 3)
        self.assertAlmostEqual(SCEN["GFH"]["min_trigger_xy_m"], 16.5)
        self.assertEqual(SCEN["KIL"]["kill_channel"], 7)
        self.assertEqual([s for s in S5.SCHEDULE if SCEN[s].get("path") == "rc"], ["OVR", "RCL", "KIL"])
        self.assertGreaterEqual(S5.PERSIST_S, 2.5)                           # review r1 B1: 1 Hz heartbeats
        self.assertGreaterEqual(S5.TIMING["post_hold_s"], S5.PERSIST_S)
        self.assertNotIn(0, [m for m, _ in S5.MESSAGE_INTERVALS])            # no HEARTBEAT rate request
        self.assertIn((69, 50000), S5.MESSAGE_INTERVALS)                     # the MANUAL_CONTROL echo
        land = tuple((n, t, 4 if n == "COM_OBL_RC_ACT" else v, r, m) for n, t, v, r, m in S5.all_params())
        self.assertEqual({s["id"]: s for s in S5.scenarios(land)}["OBL"]["expect"], "AUTO_LAND")

    def test_mode_names(self):
        for name, code in CODE.items():
            self.assertEqual(S5.mode_name(code), name)
        self.assertEqual(S5.mode_name(9 << 16), "MAIN_9")


# ------------------------------------------------------------------------------------------- message handling

class Msg:
    def __init__(self, kind, src=(1, 1), **kw):
        self._kind, self._src = kind, src
        if kind == "HEARTBEAT":                                           # PX4 v1.14.3: quadrotor, MAV_AUTOPILOT_PX4
            kw = dict({"type": 2, "autopilot": 12}, **kw)
        self.__dict__.update(kw)

    def get_type(self):
        return self._kind

    def get_srcSystem(self):
        return self._src[0]

    def get_srcComponent(self):
        return self._src[1]


def sample_messages():
    as_float = struct.unpack("<f", struct.pack("<i", 3))[0]
    return [Msg("LOCAL_POSITION_NED", time_boot_ms=12345, x=1.0, y=2.0, z=-1.0, vx=0.1, vy=0.2, vz=0.3),
            Msg("HIL_STATE_QUATERNION", lat=470000000, lon=80000000, alt=101000, attitude_quaternion=(1, 0, 0, 0),
                vx=10, vy=20, vz=-30),
            Msg("HEARTBEAT", base_mode=128, custom_mode=CODE["OFFBOARD"]),
            Msg("ATTITUDE", time_boot_ms=12340, roll=0.01, pitch=0.02, yaw=1.5, rollspeed=0.0, pitchspeed=0.0,
                yawspeed=0.1),
            Msg("PARAM_VALUE", param_id=b"NAV_DLL_ACT\x00\x00", param_value=as_float, param_type=6),
            Msg("PARAM_VALUE", param_id="COM_OF_LOSS_T", param_value=0.5, param_type=9)]


def stub(pin=True):
    """A link as pymavlink 2.4.50 leaves it after a heartbeat (target_component 0); pinned to the autopilot by
    autopilot_of from a PX4 heartbeat unless pin is False."""
    L = NS(stop=False, params={}, pos=None, truth=None, hb=None, att=None, m=NS(target_system=1, target_component=0))
    L.autopilot = S5.autopilot_of(Msg("HEARTBEAT", base_mode=0, custom_mode=0)) if pin else None
    return L


class TestHandleMessage(unittest.TestCase):
    def test_parity_with_s3b_param_link(self):
        msgs = sample_messages()
        ref = stub()
        queue = list(msgs)

        def recv_match(blocking=True, timeout=0.5):
            if queue:
                return queue.pop(0)
            ref.stop = True
            return None
        ref.m.recv_match = recv_match
        with isolated_imports():
            sys.path.insert(0, str(TOOLS))
            SF._param_link_class().rx(ref)
        me = stub()
        S5.init_logs(me)
        for m in msgs:
            S5.handle_message(me, m)
        self.assertEqual((me.pos, me.hb, me.att, me.params), (ref.pos, ref.hb, ref.att, ref.params))
        self.assertEqual(me.truth[1:], ref.truth[1:])
        self.assertEqual(me.params, {"NAV_DLL_ACT": 3.0, "COM_OF_LOSS_T": 0.5})
        self.assertEqual(len(me.hb_log), 1)

    def test_logs(self):
        me = stub()
        S5.init_logs(me)
        S5.handle_message(me, Msg("HEARTBEAT", base_mode=128, custom_mode=CODE["OFFBOARD"]))
        S5.handle_message(me, sample_messages()[0])
        t = me.pos[0]
        S5.handle_message(me, Msg("HEARTBEAT", src=(1, 154), base_mode=0, custom_mode=0))
        S5.handle_message(me, Msg("HEARTBEAT", src=(2, 1), base_mode=0, custom_mode=0))
        S5.handle_message(me, Msg("STATUSTEXT", severity=2, text=b"Failsafe activated\x00\x00"))
        S5.handle_message(me, Msg("COMMAND_ACK", command=400, result=1))
        S5.handle_message(me, Msg("RC_CHANNELS", **{"chan%d_raw" % i: 1500 + i for i in range(1, 19)}))
        S5.handle_message(me, Msg("MANUAL_CONTROL", x=1, y=2, z=3, r=4, buttons=KILL_OFF))
        S5.handle_message(me, Msg("MANUAL_CONTROL", src=(245, 0), x=9, y=9, z=9, r=9, buttons=0))   # not PX4's
        S5.handle_message(me, Msg("EXTENDED_SYS_STATE", landed_state=2))
        self.assertEqual([h[0] for h in me.hb_log], [None])
        self.assertEqual((me.other_component_hb, me.hb), (1, (128, CODE["OFFBOARD"])))
        self.assertEqual(me.statustext, [(t, 2, "Failsafe activated")])
        self.assertEqual(me.acks, [(t, 400, 1)])
        self.assertEqual(me.rc_echo, [(t, 1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508)])
        self.assertEqual(me.man_echo, [(t, 1, 2, 3, 4, KILL_OFF)])
        self.assertEqual(me.landed, [(t, 2)])


@unittest.skipUnless(MAV is not None, "pymavlink not installed")
class TestRealMavlinkRecording(unittest.TestCase):
    """Amendment 1: smoke01 ABORTED on "TypeError: Object of type bytearray is not JSON serializable". Real pymavlink
    objects go through handle_message and instrument_link and the trace.json text (trace_text), and come back
    byte for byte."""

    def test_real_messages_round_trip_through_handle_message_and_trace_text(self):
        msgs = real_messages()
        self.assertEqual([m.get_type() for m in msgs], REAL_TYPES)
        self.assertIsInstance(msgs[4].data, bytearray)
        self.assertIsInstance(msgs[5].data, bytearray)
        # negative control: the raw to_dict of BAD_DATA is what v2's recorder could not write
        with self.assertRaisesRegex(TypeError, "bytearray"):
            json.dumps({"payload": msgs[4].to_dict()})
        me = stub()
        S5.init_logs(me)
        for m in msgs:
            S5.handle_message(me, m)
        self.assertEqual((me.pos, me.hb, me.params, me.statustext, me.other_component_hb), (None, None, {}, [], 0))
        loaded = json.loads(S5.trace_text({"rx_packets": me.rx_packets}))["rx_packets"]
        self.assertEqual([p["type"] for p in loaded], REAL_TYPES)
        self.assertEqual(count_hex(loaded), 2)                          # BAD_DATA.data and UNKNOWN_54321.data
        rx = MAV.MAVLink(None)
        rx.robust_parsing = True
        for p, m in zip(loaded, msgs):
            self.assertEqual(bytes.fromhex(p["packet_hex"]), bytes(m.get_msgbuf()))
            self.assertEqual(unhex(p["payload"]), plain(m.to_dict()))
            self.assertEqual(p["source"], [m.get_srcSystem(), m.get_srcComponent()])
            if not m.get_type().startswith(("BAD_DATA", "UNKNOWN")):    # the wire bytes decode to the same message
                again = rx.parse_buffer(bytes.fromhex(p["packet_hex"]))
                self.assertEqual([a.to_dict() for a in again], [m.to_dict()])
        self.assertEqual(loaded[3]["payload"]["payload"], list(range(251)))
        self.assertEqual(loaded[1]["payload"]["text"], msgs[1].to_dict()["text"])

    def test_sends_with_bytes_like_arguments_are_recorded_losslessly(self):
        sent = []
        L = stub()
        S5.init_logs(L)
        L.send = lambda fn, *a: fn(*a)
        L.cmd = lambda *a: sent.append(a)
        S5.instrument_link(L, lambda: 5.0)
        raw = bytearray(b"NAV_DLL_ACT\x00\xff")
        L.send(lambda *a: sent.append(a), 1, 1, raw, memoryview(b"\x01\x02"), b"\x00", 1.5)
        L.cmd(400, 1.0)
        loaded = json.loads(S5.trace_text({"tx_calls": L.tx_calls}))["tx_calls"]
        self.assertEqual(unhex(loaded[0]["args"]), [1, 1, bytes(raw), b"\x01\x02", b"\x00", 1.5])
        self.assertEqual(count_hex(loaded), 3)
        self.assertEqual(loaded[1]["args"], [400, 1.0])
        self.assertEqual(len(sent), 2)

    def test_json_safe_and_default_types(self):
        np = __import__("numpy")
        self.assertEqual(S5.json_safe({"b": bytearray(b"\x00\xff"), "m": memoryview(b"ab"), "x": b"",
                                       "f": np.float32(0.5), "a": np.arange(3), "t": (1, b"\x01")}),
                         {"b": {"bytes_hex": "00ff"}, "m": {"bytes_hex": "6162"}, "x": {"bytes_hex": ""}, "f": 0.5,
                          "a": [0, 1, 2], "t": [1, {"bytes_hex": "01"}]})
        text = S5.trace_text({"raw": bytearray(b"\x07"), "n": np.int64(3), "nan": float("nan"), "o": object()})
        out = json.loads(text)
        self.assertEqual((out["raw"], out["n"]), ({"bytes_hex": "07"}, 3))
        self.assertTrue(math.isnan(out["nan"]))                         # unchanged from v2: json writes NaN
        self.assertEqual(out["o"]["unserializable_type"], "builtins.object")
        self.assertIn("object at", out["o"]["repr"])


class TestAutopilotRule(unittest.TestCase):
    """Amendment 1: which HEARTBEAT pins the autopilot (positive and negative cases, no pymavlink needed)."""

    def test_autopilot_of(self):
        hb = lambda src=(1, 1), **kw: Msg("HEARTBEAT", src=src, base_mode=0, custom_mode=0, **kw)   # noqa: E731
        self.assertEqual(S5.autopilot_of(hb()), (1, 1))
        self.assertEqual(S5.autopilot_of(hb((2, 1))), (2, 1))                 # the system comes from the heartbeat
        for bad in (hb((1, 0)), hb((1, 100)), hb((1, 154)), hb(autopilot=8), hb(autopilot=3), hb(type=6),
                    hb(type=18), hb(type=26), hb(type=27), Msg("ATTITUDE"), None):
            self.assertIsNone(S5.autopilot_of(bad), getattr(bad, "__dict__", bad))

    def test_unpinned_link_reads_nothing(self):
        me = stub(pin=False)
        S5.init_logs(me)
        for m in sample_messages():
            S5.handle_message(me, m)
        self.assertEqual((me.pos, me.hb, me.att, me.params, me.hb_log, me.other_component_hb),
                         (None, None, None, {}, [], 0))
        self.assertEqual(len(me.rx_packets), 6)                          # still recorded raw


@unittest.skipUnless(MAVUTIL is not None, "pymavlink not installed")
class TestAutopilotPinRealMavutil(unittest.TestCase):
    """Amendment 1, smoke01's first failure ("HARNESS_INVALID no telemetry within 90 s"): with a real pymavlink 2.4.50
    connection, wait_heartbeat leaves target_component 0, so v2's filter [target_system, target_component] dropped
    every PX4 message; pin_autopilot pins (1, 1) from the heartbeat and the same bytes are read."""

    def setUp(self):
        self.tmp, self.conns = tempfile.mkdtemp(), []

    def tearDown(self):
        for c in self.conns:
            c.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def conn(self, wire):
        self.conns.append(wire_connection(tempfile.mkdtemp(dir=self.tmp), wire))
        return self.conns[-1]

    def link(self, wire):
        L = NS(stop=False, params={}, pos=None, truth=None, hb=None, att=None, m=self.conn(wire))
        S5.init_logs(L)
        return L

    def drain(self, L):
        types = []
        while True:
            msg = L.m.recv_match(blocking=False)
            if msg is None:
                return types
            types.append(msg.get_type())
            S5.handle_message(L, msg)

    def test_negative_control_v2_component_zero_drops_every_px4_message(self):
        L = self.link(px4_wire(others=False))                            # smoke01: PX4 alone on the link
        hb = L.m.wait_heartbeat(timeout=2)                               # v2's call
        self.assertEqual([hb.get_srcSystem(), hb.get_srcComponent()], [1, 1])
        self.assertEqual((L.m.target_system, L.m.target_component), (1, 0))       # pymavlink 2.4.50
        L.autopilot = (L.m.target_system, L.m.target_component)                    # v2's filter
        types = self.drain(L)
        self.assertEqual(types, ["LOCAL_POSITION_NED", "ATTITUDE", "HIL_STATE_QUATERNION", "COMMAND_ACK", "HEARTBEAT"])
        self.assertEqual((L.pos, L.att, L.truth, L.hb, L.hb_log, L.acks), (None, None, None, None, [], []))
        self.assertEqual(L.other_component_hb, 1)                        # PX4's own heartbeat counted as foreign
        self.assertEqual(len(L.rx_packets), 5)                           # received, recorded, never read
        # the same bytes with the amended pin: everything is read
        L = self.link(px4_wire(others=False))
        self.assertEqual(S5.pin_autopilot(L, 5.0), (1, 1))
        self.drain(L)
        self.assertEqual((L.pos[1:4], len(L.hb_log), len(L.acks), L.other_component_hb), ((1.0, 2.0, -1.0), 1, 1, 0))
        # v2's wait_heartbeat also returns a heartbeat that is not the autopilot's when one comes first
        L = self.link(px4_wire())
        self.assertEqual(L.m.wait_heartbeat(timeout=2).get_srcComponent(), 190)

    def test_pin_from_real_mavutil_accepts_px4_messages(self):
        L = self.link(px4_wire())
        S5.handle_message(L, Msg("LOCAL_POSITION_NED", time_boot_ms=1, x=0, y=0, z=0, vx=0, vy=0, vz=0))
        self.assertIsNone(L.pos)                                         # before the pin nothing is read
        self.assertEqual(S5.pin_autopilot(L, 5.0), (1, 1))
        self.assertEqual(L.autopilot_pin["source"], [1, 1])
        self.assertEqual(L.autopilot_pin["pymavlink_target"], [1, 0])    # what v2 filtered on
        self.assertEqual([r["source"] for r in L.autopilot_pin["rejected"]], [[255, 190], [1, 100]])
        self.assertEqual((L.autopilot_pin["type"], L.autopilot_pin["autopilot"]),
                         (MAV.MAV_TYPE_QUADROTOR, MAV.MAV_AUTOPILOT_PX4))
        types = self.drain(L)
        self.assertEqual(types, ["LOCAL_POSITION_NED", "ATTITUDE", "HIL_STATE_QUATERNION", "COMMAND_ACK", "HEARTBEAT",
                                 "HEARTBEAT"])
        self.assertEqual(L.pos[:4], (12.345, 1.0, 2.0, -1.0))
        self.assertEqual(L.att[0], 12.34)
        self.assertEqual([round(v, 9) for v in L.truth[1:4]], [47.0, 8.0, 101.0])
        self.assertEqual(L.hb, (0, 6 << 16))
        self.assertEqual([h[2:] for h in L.hb_log], [(0, 6 << 16)])
        self.assertEqual(L.acks, [(12.345, 400, 1)])
        self.assertEqual(L.other_component_hb, 1)                        # the camera heartbeat after the pin
        # the sends keep pymavlink's wire address (component 0 = MAV_COMP_ID_ALL), the replies match the autopilot
        t = iter(range(1000))
        L.send, L.cmd = (lambda fn, *a: fn(*a)), (lambda *a: None)
        S5.instrument_link(L, lambda: float(next(t)))
        L.cmd(400, 1.0)
        rec = L.tx_calls[0]
        self.assertEqual((rec["destination"], rec["autopilot"]), ([1, 0], [1, 1]))
        L.rx_packets = []                                                # replies after the command only
        L.m = self.conn(px4_wire())
        S5.pin_autopilot(L, 5.0)
        self.drain(L)
        trace = json.loads(S5.trace_text({"tx_calls": L.tx_calls, "rx_packets": L.rx_packets}))
        inv = S5.command_inventory(trace)
        self.assertEqual((inv[0]["ack_state"], inv[0]["ack_results"]), ("RECEIVED", ["TEMPORARILY_REJECTED"]))
        self.assertEqual(inv[0]["first_reported_actual"]["mode"], "OFFBOARD")
        del trace["tx_calls"][0]["autopilot"]                            # negative control: v2's destination match
        self.assertEqual(S5.command_inventory(trace)[0]["ack_state"], "NOT_RECORDED")

    def test_no_autopilot_heartbeat_pins_nothing(self):
        L = self.link(px4_wire(autopilot=False, telemetry=False))
        t0 = time.monotonic()
        self.assertIsNone(S5.pin_autopilot(L, 0.3))
        self.assertLess(time.monotonic() - t0, 5.0)
        self.assertIsNone(L.autopilot)
        self.assertEqual([r["source"] for r in L.autopilot_pin["rejected"]], [[255, 190], [1, 100]])
        self.assertEqual(self.drain(L), [])


class TestStreams(unittest.TestCase):
    def test_rates_gaps_dither_and_stop(self):
        sent = []
        rec = lambda k: (lambda *a: sent.append((k, a)))                # noqa: E731
        L = NS(pos=(1.0,), send=lambda fn, *a: fn(*a),
               m=NS(target_system=1, target_component=0,
                    mav=NS(set_position_target_local_ned_send=rec("sp"), heartbeat_send=rec("hb"),
                           rc_channels_override_send=rec("rc"), manual_control_send=rec("joy"))))
        S = S5.Streams(L, S5.TIMING)
        S.sp, S.hb, S.rc, S.track = (S5.TM_POS_XYZ, 0, 0, -1, 0, 0, 0, 0), True, list(S5.RC_CENTER), True
        for i in range(51):
            S.tick(i * 0.02)
        self.assertEqual((S.sent["sp"], S.sent["hb"], S.sent["joy"]), (11, 2, 0))
        self.assertGreater(S.sent["rc"], 15)
        rc = [a for k, a in sent if k == "rc"]
        self.assertEqual(len(rc[0]), 10)                                      # target system, component, 8 channels
        self.assertEqual([a[2 + S5.RC_DITHER_INDEX] for a in rc[:4]], [1000, 1001, 1000, 1001])   # review r1 M2
        self.assertTrue(all(list(a[2:9]) == list(S5.RC_CENTER[:7]) for a in rc))
        g = S.gaps()
        self.assertTrue(0.09 < g["sp"] <= S5.GAP_LIMIT_S["sp"] and 0 < g["rc"] <= S5.GAP_LIMIT_S["rc"], g)
        S.tick(1.5)
        self.assertGreater(S.gaps()["sp"], S5.GAP_LIMIT_S["sp"])
        n_rc = S.sent["rc"]
        S.rc = None
        for i in range(10):
            S.tick(1.52 + i * 0.02)
        self.assertEqual((S.sent["rc"], S.gaps()["rc"]), (n_rc, 0.0))
        self.assertEqual(sum(1 for k, a in sent if k == "sp" and a[3] == S5.MAV_FRAME_LOCAL_NED
                             and a[4] == S5.TM_POS_XYZ), S.sent["sp"])


# ------------------------------------------------------------------------------------------- assessment (pure, 1 Hz)

def hb_rows(*breaks, end=30.0, phase=0.3):
    """Heartbeats at 1 Hz (+-20 ms) from breakpoints [(t, mode, armed)]: the state is the last breakpoint <= t."""
    rows, k = [], 0
    while True:
        t = round(k + phase + jitter(k), 6)
        if t >= end:
            return rows
        _, mode, armed = [b for b in breaks if b[0] <= t][-1]
        rows.append([t, 0.0, 128 if armed else 0, CODE[mode]])
        k += 1


def echo(buttons=KILL_OFF, end=30.0, on_from=None):
    return [[round(0.05 * i, 3), 0, 0, 0, 0, KILL_ON if (on_from is not None and 0.05 * i >= on_from) else buttons]
            for i in range(int(end / 0.05))]


def record(t_inj=5.0, t_ref=None, **kw):
    gaps = {"sp": 0.1, "hb": 1.0, "rc": 0.06, "joy": 0.06}
    r = {"outcome": "COMPLETED", "param_problems": [], "stream_gaps": {"pre": dict(gaps), "post": dict(gaps)},
         "t_pre_start": 0.0, "t_inject": t_inj, "t_ref": t_inj if t_ref is None else t_ref}
    r.update(kw)
    return r


def trace(hb, **kw):
    t = {"hb": hb, "pos": [], "truth": [], "rc_echo": [[1.0, *S5.RC_CENTER]], "man_echo": echo(), "statustext": [],
         "acks": [], "home_ned": [0.0, 0.0, 0.0]}
    t.update(kw)
    return t


FLAGS = S5.CAUSE_FLAGS


def flag_log(rows):
    """A parsed-ULog stand-in carrying only failsafe_flags rows [(t_s, {flag: bool})]."""
    return [{"failsafe_flags": {"fields": list(FLAGS), "missing_fields": [],
                                "rows": [[int(round(t * 1e6))] + [int(bool(f[k])) for k in FLAGS] for t, f in rows]}}]


def flag_rows(sid, rises=None, t_inj=5.0, end=40.0):
    """failsafe_flags every 0.5 s plus a row at each rise: live flags false, manual_control_signal_lost true where it is
    not live (no RC/joystick), each flag in rises true from its time. Default: the scenario's own cause flag rises at
    t_inj + 0.1 (none for N0/OVJ/OVR/KIL/FDS)."""
    live = S5.live_flags(SCEN[sid])
    if rises is None:
        own = SCEN[sid].get("cause_flag")
        rises = {own: t_inj + 0.1} if own else {}
    times = sorted(set([i * 0.5 for i in range(int(end / 0.5))] + list(rises.values())))
    return [(t, {k: (k not in live) or (k in rises and t >= rises[k]) for k in FLAGS}) for t in times]


class TestAssess(unittest.TestCase):
    def go(self, sid, hb, res=None, flags=None, **tr):
        res = res or record()
        if "ulogs" not in res:
            res = dict(res, ulogs=flag_log(flags if flags is not None else flag_rows(sid, t_inj=res["t_inject"])))
        return S5.assess(SCEN[sid], res, trace(hb, **tr))

    def test_null(self):
        good = self.go("N0", hb_rows((0, OFF, True), end=21))
        self.assertEqual((good["valid"], good["passed"]), (True, True))
        self.assertGreaterEqual(good["criteria"]["window_observations"], 15)
        bad = self.go("N0", hb_rows((0, OFF, True), (12, "AUTO_LOITER", True), end=21))
        self.assertIn("MODE_CHANGED", bad["fail_reason"])
        few = self.go("N0", hb_rows((0, OFF, True), end=7))
        self.assertEqual((few["valid"], few["invalid_reason"]), (False, "TOO_FEW_OBSERVATIONS_AFTER"))

    def test_smoke_nulls_need_their_path(self):
        hb = hb_rows((0, OFF, True), end=21)
        self.assertTrue(self.go("N0R", hb)["passed"])
        self.assertTrue(self.go("N0J", hb)["passed"])
        self.assertEqual(self.go("N0R", hb, man_echo=echo(buttons=0))["invalid_reason"], "RC_PATH_NOT_SELECTED")
        self.assertEqual(self.go("N0J", hb, man_echo=[])["invalid_reason"], "JOYSTICK_PATH_NOT_SELECTED")
        self.assertTrue(self.go("N0J", hb, man_echo=echo(buttons=0))["passed"])      # no switches on the joystick

    def test_offboard_loss(self):
        ok = self.go("OBL", hb_rows((0, OFF, True), (5.6, "AUTO_LOITER", True)))
        self.assertTrue(ok["passed"], ok)
        self.assertEqual(ok["criteria"]["event_source"], "heartbeat")
        early = self.go("OBL", hb_rows((0, OFF, True), (5.25, "AUTO_LOITER", True)), res=record(t_inj=5.2))
        self.assertIn("TOO_EARLY", early["fail_reason"])
        late = self.go("OBL", hb_rows((0, OFF, True), (9.0, "AUTO_LOITER", True)))
        self.assertIn("LATE", late["fail_reason"])
        self.assertIn("NO_DEPARTURE", self.go("OBL", hb_rows((0, OFF, True)))["fail_reason"])
        wrong = self.go("OBL", hb_rows((0, OFF, True), (5.6, "POSCTL", True), (7.0, "AUTO_LOITER", True)))
        self.assertIn("UNEXPECTED_STATE", wrong["fail_reason"])
        flip = self.go("OBL", hb_rows((0, OFF, True), (5.6, "AUTO_LOITER", True), (7.0, "POSCTL", True)))
        self.assertIn("NOT_PERSISTENT", flip["fail_reason"])
        before = self.go("OBL", hb_rows((0, OFF, True), (2, "AUTO_LOITER", True), (2.5, OFF, True)))
        self.assertIn("BEFORE_INJECTION", before["fail_reason"])

    def test_rc_loss(self):
        hb = hb_rows((0, OFF, True), (5.6, "AUTO_LOITER", True), (10.6, "AUTO_LAND", True), (12.0, "AUTO_LAND", False))
        ok = self.go("RCL", hb)
        self.assertTrue(ok["passed"], ok)
        self.assertGreaterEqual(ok["criteria"]["manual_echo_pre"], S5.MIN_ECHO)
        text = [[5.52, 2, "Failsafe activated, entering Hold for 5 seconds"]]
        ok_text = self.go("RCL", hb, statustext=text)
        self.assertEqual((ok_text["passed"], ok_text["criteria"]["event_source"]), (True, "statustext"))
        early = self.go("RCL", hb, statustext=[[5.1, 2, "Failsafe activated, entering Hold for 5 seconds"]])
        self.assertIn("TOO_EARLY", early["fail_reason"])                        # review r1 m2: the event time
        hold_only = hb_rows((0, OFF, True), (5.6, "AUTO_LOITER", True))
        self.assertIn("NOT_REACHED", self.go("RCL", hold_only)["fail_reason"])
        self.assertIn("NO_DEPARTURE", self.go("RCL", hb_rows((0, OFF, True)))["fail_reason"])
        no_echo = self.go("RCL", hb, rc_echo=[[1.0, 1500, 1500, 1500, 1500, 1000, 1000, 2000, 1000]])
        self.assertEqual(no_echo["invalid_reason"], "RC_INPUT_NOT_SEEN")
        for man in ([], echo(buttons=0), [[t, 300, 0, 0, 0, KILL_OFF] for t, *_ in echo()]):   # review r1 M1
            self.assertEqual(self.go("RCL", hb, man_echo=man)["invalid_reason"], "RC_PATH_NOT_SELECTED")

    def test_datalink_loss(self):                                        # review r1 M3: DLL had no assess test
        res = record(t_ref=4.6)
        hb = hb_rows((0, OFF, True), (12.1, "AUTO_LOITER", True), (17.1, "AUTO_LAND", True))
        ok = self.go("DLL", hb, res=res)
        self.assertTrue(ok["passed"], ok)
        self.assertGreater(ok["criteria"]["event_after_ref_s"], SCEN["DLL"]["earliest_s"])
        early = self.go("DLL", hb_rows((0, OFF, True), (7.0, "AUTO_LOITER", True), (12.0, "AUTO_LAND", True)), res=res)
        self.assertIn("TOO_EARLY", early["fail_reason"])
        self.assertIn("NO_DEPARTURE", self.go("DLL", hb_rows((0, OFF, True)), res=res)["fail_reason"])
        hold = self.go("DLL", hb_rows((0, OFF, True), (12.1, "AUTO_LOITER", True)), res=res)
        self.assertIn("NOT_REACHED", hold["fail_reason"])
        late = self.go("DLL", hb_rows((0, OFF, True), (12.1, "AUTO_LOITER", True), (23.0, "AUTO_LAND", True)), res=res)
        self.assertIn("LATE", late["fail_reason"])

    def test_harness_validity(self):
        hb = hb_rows((0, OFF, True), (5.6, "AUTO_LOITER", True))
        self.assertTrue(self.go("OBL", hb)["valid"])
        gap = record()
        gap["stream_gaps"]["pre"]["sp"] = 0.4
        self.assertIn("STREAM_GAP", self.go("OBL", hb, res=gap)["invalid_reason"])
        rc_gap = record()
        rc_gap["stream_gaps"]["pre"]["rc"] = 0.21                       # review r1 M2: tighter rc limit
        self.assertIn("STREAM_GAP", self.go("OBL", hb, res=rc_gap)["invalid_reason"])
        self.assertEqual(self.go("OBL", hb, res=record(param_problems=["x"]))["invalid_reason"], "PARAM_MISMATCH")
        self.assertEqual(self.go("OBL", hb, res=record(outcome="ERROR"))["invalid_reason"], "ERROR")
        self.assertEqual(self.go("OBL", hb, res=record(sim_too_slow=True))["invalid_reason"], "SIM_TOO_SLOW")
        few = self.go("OBL", hb, res=record(t_pre_start=3.0))
        self.assertEqual(few["invalid_reason"], "TOO_FEW_OBSERVATIONS_BEFORE")

    def test_horizontal_fence(self):
        hb = hb_rows((0, OFF, True), (11.8, "AUTO_LOITER", True))
        pos = [[t / 10, min(17.0, 2.5 * max(0.0, t / 10 - 5.0)), 0.0, -1.0] for t in range(0, 300)]
        truth = [[r[0], r[2], r[1], 1.0] for r in pos]
        text = [[11.8, 2, "Maximum distance from home reached (17.00000)"]]
        ok = self.go("GFH", hb, pos=pos, truth=truth, statustext=text)
        self.assertTrue(ok["passed"], ok)
        self.assertEqual(ok["criteria"]["event_source"], "statustext")
        self.assertTrue(self.go("GFH", hb, pos=pos, truth=truth)["passed"])   # no text: heartbeat bracket
        early_text = [[9.0, 2, "Maximum distance from home reached (17.00000)"]]
        self.assertIn("TOO_EARLY_DISTANCE", self.go("GFH", hb, pos=pos, truth=truth,
                                                    statustext=early_text)["fail_reason"])
        early_hb = hb_rows((0, OFF, True), (9.0, "AUTO_LOITER", True))
        self.assertIn("TOO_EARLY_DISTANCE", self.go("GFH", early_hb, pos=pos, truth=truth)["fail_reason"])
        out = truth + [[29.95, 0.0, 21.0, 1.0]]
        self.assertIn("LEFT_ARENA", self.go("GFH", hb, pos=pos, truth=out)["fail_reason"])

    def test_vertical_fence(self):
        hb = hb_rows((0, OFF, True), (8.0, "AUTO_LOITER", True))
        pos = [[t / 10, 0.0, 0.0, -min(2.5, 1.0 + 0.5 * max(0.0, t / 10 - 5.0))] for t in range(0, 300)]
        truth = [[r[0], 0.0, 0.0, -r[3]] for r in pos]
        self.assertTrue(self.go("GFV", hb, pos=pos, truth=truth)["passed"])
        self.assertIn("ABOVE_ARENA", self.go("GFV", hb, pos=pos, truth=truth + [[29.95, 0, 0, 3.2]])["fail_reason"])
        early = hb_rows((0, OFF, True), (6.0, "AUTO_LOITER", True))
        self.assertIn("TOO_EARLY_ALTITUDE", self.go("GFV", early, pos=pos, truth=truth)["fail_reason"])

    def test_kill(self):
        on = echo(on_from=5.05)
        hb = hb_rows((0, OFF, True), (10.0, OFF, False))
        falling = [[4.9, 0, 0, 1.0], [5.2, 0, 0, 0.7], [5.5, 0, 0, 0.0], [9.0, 0, 0, 0.0]]
        hovering = [[4.9, 0, 0, 1.0], [5.5, 0, 0, 1.0], [6.4, 0, 0, 1.0], [9.0, 0, 0, 1.0]]
        text = [[5.1, 2, "Kill-switch engaged"]]                          # the cause text (v2 r1 M2)
        ok = self.go("KIL", hb, truth=falling, man_echo=on, statustext=text)
        self.assertTrue(ok["passed"], ok)
        self.assertAlmostEqual(ok["criteria"]["lockdown_drop_m"], 1.0)
        self.assertAlmostEqual(ok["criteria"]["cause"]["text_after_inject_s"], 0.1)
        self.assertIn("NO_LOCKDOWN", self.go("KIL", hb, truth=hovering, man_echo=on, statustext=text)["fail_reason"])
        quick = hb_rows((0, OFF, True), (5.5, OFF, False))
        self.assertTrue(self.go("KIL", quick, truth=hovering, man_echo=on, statustext=text)["passed"])
        self.assertIn("NO_DEPARTURE", self.go("KIL", hb_rows((0, OFF, True)), truth=falling,
                                              man_echo=on)["fail_reason"])
        late = hb_rows((0, OFF, True), (13.0, OFF, False))
        self.assertIn("LATE", self.go("KIL", late, truth=falling, man_echo=on, statustext=text)["fail_reason"])
        self.assertEqual(self.go("KIL", hb, truth=falling)["invalid_reason"], "KILL_SWITCH_NOT_SEEN")   # M1
        silent = self.go("KIL", hb, truth=falling, man_echo=on)                  # right disarm, no cause text
        self.assertEqual((silent["valid"], silent["invalid_reason"]), (False, "EVENT_TEXT_NOT_SEEN"))

    def test_forced_disarm(self):
        res = record(t_inject_b=8.0)
        acks = [[5.01, 400, 1], [8.01, 400, 0]]
        hb = hb_rows((0, OFF, True), (8.1, OFF, False))
        ok = self.go("FDS", hb, res=res, acks=acks)
        self.assertTrue(ok["passed"], ok)
        self.assertGreaterEqual(ok["criteria"]["reject_window_observations"], 2)
        acted = self.go("FDS", hb_rows((0, OFF, True), (6.0, OFF, False)), res=res, acks=acks)
        self.assertIn("NONFORCED_DISARM_ACTED", acted["fail_reason"])
        slow = self.go("FDS", hb_rows((0, OFF, True), (10.5, OFF, False)), res=res, acks=acks)
        self.assertIn("LATE", slow["fail_reason"])
        self.assertIn("NONFORCED_ACK", self.go("FDS", hb, res=res, acks=[[5.01, 400, 0], [8.01, 400, 0]])["fail_reason"])
        self.assertIn("FORCED_ACK", self.go("FDS", hb, res=res, acks=[[5.01, 400, 1], [8.01, 400, 4]])["fail_reason"])
        self.assertEqual(self.go("FDS", hb, res=res, acks=[[8.01, 400, 0]])["invalid_reason"], "NO_COMMAND_ACK")
        self.assertEqual(self.go("FDS", hb, res=res, acks=[[1.0, 400, 0], [5.01, 400, 1]])["invalid_reason"],
                         "NO_COMMAND_ACK")
        glitch = self.go("FDS", hb, res=res, acks=acks, flags=flag_rows("FDS", {"gcs_connection_lost": 6.0}))
        self.assertIn("WRONG_CAUSE gcs_connection_lost", glitch["fail_reason"])          # v2 r1 M2
        self.assertEqual(ok["criteria"]["cause"], {"evidence": "command_ack", "other_raised": None})

    def test_cause_attribution_rcl_vs_dll(self):                            # v2 review r1 M2
        rcl = hb_rows((0, OFF, True), (5.6, "AUTO_LOITER", True), (10.6, "AUTO_LAND", True))
        ok = self.go("RCL", rcl, flags=flag_rows("RCL", {"manual_control_signal_lost": 5.3}))
        self.assertTrue(ok["passed"], ok)
        self.assertEqual((ok["criteria"]["cause"]["first_raised"], ok["criteria"]["cause"]["flag"]),
                         (["manual_control_signal_lost"], "manual_control_signal_lost"))
        cases = [({"gcs_connection_lost": 5.3}, "WRONG_CAUSE gcs_connection_lost"),          # the DLL cause
                 ({"manual_control_signal_lost": 5.3, "gcs_connection_lost": 5.3}, "WRONG_CAUSE"),  # not attributable
                 ({}, "CAUSE_NOT_SEEN manual_control_signal_lost"),
                 ({"manual_control_signal_lost": 8.0}, "CAUSE_NOT_SEEN"),                    # after departure + lag
                 ({"gcs_connection_lost": 2.0, "manual_control_signal_lost": 5.3}, "FLAG_BEFORE_INJECTION")]
        for rises, needle in cases:
            got = self.go("RCL", rcl, flags=flag_rows("RCL", rises))
            self.assertEqual((got["valid"], got["passed"]), (True, False), rises)
            self.assertIn(needle, got["fail_reason"], rises)
        res = record(t_ref=4.6)
        dll = hb_rows((0, OFF, True), (12.1, "AUTO_LOITER", True), (17.1, "AUTO_LAND", True))
        good = self.go("DLL", dll, res=res, flags=flag_rows("DLL", {"gcs_connection_lost": 11.9}))
        self.assertTrue(good["passed"], good)
        self.assertEqual(good["criteria"]["cause"]["first_raised"], ["gcs_connection_lost"])
        for rises in ({"manual_control_signal_lost": 11.9}, {"offboard_control_signal_lost": 11.9}):
            got = self.go("DLL", dll, res=res, flags=flag_rows("DLL", rises))
            # manual is not live without RC (true from boot), so a DLL "explained" by it shows no cause at all
            self.assertFalse(got["passed"], rises)
            self.assertTrue(got["fail_reason"].startswith(("WRONG_CAUSE", "CAUSE_NOT_SEEN")), got["fail_reason"])
        self.assertIn("WRONG_CAUSE offboard_control_signal_lost",
                      self.go("DLL", dll, res=res, flags=flag_rows("DLL", {"offboard_control_signal_lost": 11.9}))
                      ["fail_reason"])
        none = self.go("RCL", rcl, flags=[])
        self.assertEqual((none["valid"], none["invalid_reason"]), (False, "ULOG_FLAGS_NOT_RECORDED"))

    def test_null_and_fence_flags(self):
        hb = hb_rows((0, OFF, True), end=21)
        self.assertTrue(self.go("N0", hb)["passed"])
        bad = self.go("N0", hb, flags=flag_rows("N0", {"gcs_connection_lost": 12.0}))
        self.assertIn("NULL_FLAG_RAISED gcs_connection_lost", bad["fail_reason"])
        self.assertTrue(self.go("N0", hb, flags=flag_rows("N0"))["passed"])     # manual lost without RC: not live
        n0r = self.go("N0R", hb, flags=flag_rows("N0R", {"manual_control_signal_lost": 9.0}))
        self.assertIn("NULL_FLAG_RAISED manual_control_signal_lost", n0r["fail_reason"])
        hb = hb_rows((0, OFF, True), (11.8, "AUTO_LOITER", True))
        pos = [[t / 10, min(17.0, 2.5 * max(0.0, t / 10 - 5.0)), 0.0, -1.0] for t in range(0, 300)]
        truth = [[r[0], r[2], r[1], 1.0] for r in pos]
        ok = self.go("GFH", hb, pos=pos, truth=truth, flags=flag_rows("GFH", {"primary_geofence_breached": 11.6}))
        self.assertTrue(ok["passed"], ok)
        wrong = self.go("GFH", hb, pos=pos, truth=truth, flags=flag_rows("GFH", {"offboard_control_signal_lost": 11.6}))
        self.assertIn("WRONG_CAUSE offboard_control_signal_lost (expected primary_geofence_breached)",
                      wrong["fail_reason"])

    def test_stick_pulse_and_cause_text(self):                             # v2 review r1 m4 and M2
        hb = hb_rows((0, OFF, True), (5.6, "POSCTL", True))
        pulse = echo(buttons=0) + [[5.05, 500, 0, 0, 0, 0]]
        text = [[5.1, 6, "Pilot took over using sticks"]]
        ok = self.go("OVJ", hb, man_echo=pulse, statustext=text)
        self.assertTrue(ok["passed"], ok)
        self.assertEqual(ok["criteria"]["cause"]["other_raised"], None)
        no_pulse = self.go("OVJ", hb, man_echo=echo(buttons=0), statustext=text)
        self.assertEqual((no_pulse["valid"], no_pulse["invalid_reason"]), (False, "STICK_PULSE_NOT_SEEN"))
        no_text = self.go("OVJ", hb, man_echo=pulse)
        self.assertEqual((no_text["valid"], no_text["invalid_reason"]), (False, "EVENT_TEXT_NOT_SEEN"))
        other = self.go("OVJ", hb, man_echo=pulse, statustext=text, flags=flag_rows("OVJ", {"gcs_connection_lost": 5.08}))
        self.assertIn("WRONG_CAUSE gcs_connection_lost", other["fail_reason"])
        later = self.go("OVJ", hb, man_echo=pulse, statustext=text, flags=flag_rows("OVJ", {"gcs_connection_lost": 7.0}))
        self.assertTrue(later["passed"], later)                 # a flag after the cause + lag is not attributed


# ------------------------------------------------------------------------------------------- live path (fake PX4)

class FakeClock:
    def __init__(self):
        self.t, self.link = 100.0, None

    def __call__(self):
        return self.t

    def sleep(self, dt):
        self.t += dt
        if self.link is not None:
            self.link.step(self.t)


OBL_MAP = {0: "POSCTL", 4: "AUTO_LAND", 5: "AUTO_LOITER"}
NAV_MAP = {1: "AUTO_LOITER", 3: "AUTO_LAND", 6: "DISARMED"}
GF_MAP = {2: "AUTO_LOITER", 5: "AUTO_LAND"}
DEFAULTS = {"COM_OF_LOSS_T": 1.0, "COM_OBL_RC_ACT": 0, "COM_RC_LOSS_T": 0.5, "COM_RCL_EXCEPT": 0, "NAV_RCL_ACT": 2,
            "NAV_DLL_ACT": 0, "COM_DL_LOSS_T": 10, "GF_MAX_HOR_DIST": 0.0, "GF_MAX_VER_DIST": 0.0, "GF_ACTION": 2,
            "COM_RC_OVERRIDE": 1, "COM_RC_STICK_OV": 30.0, "RC_MAP_KILL_SW": 0, "COM_KILL_DISARM": 5.0,
            "COM_FAIL_ACT_T": 5.0}


class RealWire:
    """Amendment 1: FakePX4Link's connection. recv_match, target_system and target_component are a real pymavlink
    mavutil connection reading px4_wire()'s real MAVLink 2 bytes, so run_scenario's pin_autopilot derives the autopilot
    exactly as on the live UDP link (pymavlink's target stays (1, 0)); the sends go to the fake PX4 model. Without
    pymavlink: a fake with the same addressing (target_component 0)."""

    def __init__(self, mav, close):
        self.mav, self._close = mav, close
        self.dir = tempfile.mkdtemp()
        if MAVUTIL is not None:
            self.conn = wire_connection(self.dir, px4_wire(telemetry=False))
        else:
            hb = [Msg("HEARTBEAT", base_mode=0, custom_mode=0)]
            self.conn = NS(target_system=1, target_component=0, close=lambda: None,
                           recv_match=lambda **kw: hb.pop(0) if hb else None)

    @property
    def target_system(self):
        return self.conn.target_system

    @property
    def target_component(self):
        return self.conn.target_component

    def recv_match(self, **kw):
        return self.conn.recv_match(**kw)

    def close(self):
        self.conn.close()
        shutil.rmtree(self.dir, ignore_errors=True)
        self._close()


class FakePX4Link:
    """The harness's link with a small PX4 v1.14.3 model behind it, stepped by the fake clock. `faults` breaks one
    mechanism (a negative control) or one harness path (rc_not_selected: rc_update never accepts the MAVLink RC)."""

    def __init__(self, clock, faults=()):
        self.clock, self.faults = clock, set(faults)
        if MAVUTIL is not None:
            self.mu = MAVUTIL
        clock.link = self
        S5.init_logs(self)
        mav = NS(param_set_send=self._pset, param_request_read_send=lambda *a: None,
                 set_position_target_local_ned_send=self._sp, heartbeat_send=self._hb,
                 rc_channels_override_send=self._rc, manual_control_send=self._joy)
        self.m = RealWire(mav, self._close)
        self.stop = self.closed = False
        self.pos = self.att = self.truth = self.hb = None
        self.params, self.store = {}, dict(DEFAULTS)
        self.armed, self.mode, self.lockdown_t, self.land_t, self.arm_t = False, "MANUAL", None, None, None
        self.x = self.y = self.z = self.vfall = 0.0
        self.last = dict.fromkeys(("sp", "hb", "rc", "joy"))
        self.sp = self.rc = self.joy = self.rc_prev = None
        self.rc_sample_t = None                          # rc_update's last published sample (change or 300 ms)
        self.sw_cand, self.sw_n, self.sw_state = None, 0, None
        self.source = None
        self.rc_at_arm, self.stick_prev, self.pending, self.done = False, None, None, set()
        self.k_hb = 0
        self.hist = []                                   # (sim_s, armed, failsafe flags): the fake ULog's source
        self.flags_at_arm, self.glitch_until = None, None
        self.t0 = self.t_prev = self.next_hb = self.next_echo = clock()
        self.step(self.t0)

    # -- harness side
    def rx(self):
        while not self.stop:
            time.sleep(0.001)

    def send(self, fn, *a):
        fn(*a)

    def read_params(self, names):
        return {n: self.params.get(n) for n in names}

    def _close(self):
        self.closed = True

    def _fresh(self, t_last, age):
        return t_last is not None and self.clock() - t_last <= age

    def _text(self, text):
        if "silent_text" in self.faults and text in ("Pilot took over using sticks", "Kill-switch engaged"):
            return                                                       # the cause is never announced
        S5.handle_message(self, Msg("STATUSTEXT", severity=2, text=text))

    def _glitch(self, now):
        """flag_glitch: another cause flag (GCS link lost) raised for 1 s from the injection, while the scenario's
        own mechanism acts as configured (the mode sequence stays right)."""
        if "flag_glitch" in self.faults and self.armed and self.glitch_until is None:
            self.glitch_until = now + 1.0

    def _flags_raw(self, t):
        g = self.store
        return {"offboard_control_signal_lost": self.last["sp"] is None or t - self.last["sp"] > g["COM_OF_LOSS_T"],
                "manual_control_signal_lost": self._source() is None,
                "gcs_connection_lost": self.last["hb"] is None or t - self.last["hb"] > g["COM_DL_LOSS_T"] + 2.5,
                "primary_geofence_breached": (g["GF_MAX_HOR_DIST"] > 0 and math.hypot(self.x, self.y)
                                              > g["GF_MAX_HOR_DIST"])
                or (g["GF_MAX_VER_DIST"] > 0 and -self.z > g["GF_MAX_VER_DIST"])}

    WRONG = {"offboard_control_signal_lost": "gcs_connection_lost", "manual_control_signal_lost": "gcs_connection_lost",
             "gcs_connection_lost": "offboard_control_signal_lost",
             "primary_geofence_breached": "offboard_control_signal_lost"}

    def _flags(self, t):
        """PX4's failsafe_flags. wrong_cause: a flag that rises after arming is reported as another one (WRONG), while
        the failsafe acts as configured: the mode sequence is right, the cause is not."""
        raw = self._flags_raw(t)
        f = dict(raw)
        if "wrong_cause" in self.faults and self.flags_at_arm is not None:
            for k, to in self.WRONG.items():
                if raw[k] and not self.flags_at_arm[k]:
                    f[k], f[to] = False, True
        if self.glitch_until is not None and t <= self.glitch_until:
            f["gcs_connection_lost"] = True
        return f

    def _pset(self, ts, tc, name, value, typ):
        name = name.decode()
        if "drop_param" in self.faults and name == "GF_ACTION":
            return
        is_int = name in INT_NAMES
        if is_int != (typ == 6):
            return                                                       # PX4: "param types mismatch", no echo
        self.store[name] = float(struct.unpack("<i", struct.pack("<f", value))[0]) if is_int else float(value)
        S5.handle_message(self, Msg("PARAM_VALUE", param_id=name.encode(), param_value=value, param_type=typ))

    def _sp(self, tb, ts, tc, frame, mask, x, y, z, vx, vy, vz, *rest):
        self.sp, self.last["sp"] = (mask, x, y, z, vx, vy, vz), self.clock()

    def _hb(self, *a):
        self.last["hb"] = self.clock()

    def _rc(self, ts, tc, *ch):
        now = self.clock()
        self.rc, self.last["rc"] = list(ch), now
        if "rc_not_selected" in self.faults:
            return
        if ch != self.rc_prev or self.rc_sample_t is None or now - self.rc_sample_t > 0.3:
            self.rc_sample_t = now                                      # rc_update.cpp:529-530
        self.rc_prev = ch
        k = int(self.store["RC_MAP_KILL_SW"])
        if ch[1] != 1500 or (k and ch[k - 1] > 1875):
            self._glitch(now)
        if not k:
            return
        pos = S5.SWITCH_ON if ch[k - 1] > 1875 else S5.SWITCH_OFF
        self.sw_n = self.sw_n + 1 if pos == self.sw_cand else 1
        self.sw_cand = pos
        if self.sw_n >= 2 and pos != self.sw_state:                     # two identical frames, then an edge
            prev, self.sw_state = self.sw_state, pos
            if prev == S5.SWITCH_OFF and pos == S5.SWITCH_ON and self.armed and "ignore_kill" not in self.faults:
                self.lockdown_t = now
                self._text("Kill-switch engaged")

    def _joy(self, ts, x, y, z, r, buttons):
        self.joy, self.last["joy"] = (x, y, z, r), self.clock()
        if (x, y, z, r) != tuple(S5.JOY_CENTER):
            self._glitch(self.clock())

    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        result = 0
        if command == S5.MAV_CMD_DO_SET_MODE and p[0] == 1:
            if p[1] == 6 and self._fresh(self.last["sp"], 0.5):
                self.mode = "OFFBOARD"
            elif p[1] == 4 and p[2] == 6:
                self.mode = "AUTO_LAND"
            else:
                result = 1
        elif command == S5.MAV_CMD_COMPONENT_ARM_DISARM:
            if p[0] == 1:
                if self.mode == "OFFBOARD" and (self.store["NAV_DLL_ACT"] == 0 or self._fresh(self.last["hb"], 3.5)):
                    self.armed, self.arm_t = True, self.clock()
                    self.rc_at_arm = self._source() == "rc"
                    self.flags_at_arm = self._flags_raw(self.clock())
                else:
                    result = 1
            else:
                self._glitch(self.clock())
                if int(round(p[1])) == S5.FORCE_MAGIC or self.z > -0.1 or "nonforced_accepted" in self.faults:
                    self.armed = False
                else:
                    result = 1
                    self._text("Disarming denied! Not landed")
        S5.handle_message(self, Msg("COMMAND_ACK", command=command, result=result))

    # -- PX4 side
    def _source(self):
        """ManualControlSelector in mode 2: keep the selected source while valid, else the first valid one."""
        valid = {"joy": self._fresh(self.last["joy"], 0.5), "rc": self._fresh(self.rc_sample_t, 0.5)}
        if self.source is None or not valid[self.source]:
            self.source = "joy" if valid["joy"] else ("rc" if valid["rc"] else None)
        return self.source

    def _sticks(self):
        src = self._source()
        if src == "joy":
            x, y, z, r = self.joy
            return x / 1000.0, y / 1000.0, z / 500.0 - 1.0, r / 1000.0
        if src == "rc":
            c = self.rc_prev
            return (c[1] - 1500) / 500.0, (c[0] - 1500) / 500.0, (c[2] - 1500) / 500.0, (c[3] - 1500) / 500.0
        return None

    def _set(self, mode):
        if mode == "DISARMED":
            self.armed = False
        else:
            self.mode = mode

    def _once(self, cause, target, t, delay):
        if cause in self.done:
            return
        self.done.add(cause)
        if delay:
            self.mode, self.pending = "AUTO_LOITER", (target, t + self.store["COM_FAIL_ACT_T"])
            self._text("Failsafe activated, entering Hold for 5 seconds")
        else:
            self._set(target)

    def _failsafes(self, t, dt):
        g = self.store
        if self.pending and t >= self.pending[1]:
            self._set(self.pending[0])
            self.pending = None
        off = self.mode == "OFFBOARD"
        if "spurious" in self.faults and t - self.arm_t > 10.0 and off:
            self.mode = "AUTO_LOITER"
        extra = 5.0 if "slow_obl" in self.faults else 0.0
        if off and self.last["sp"] is not None and t - self.last["sp"] > g["COM_OF_LOSS_T"] + extra:
            self._once("obl", OBL_MAP[int(g["COM_OBL_RC_ACT"])], t, False)
        rc_lost = self.rc_sample_t is not None and t - self.rc_sample_t > g["COM_RC_LOSS_T"]
        if self.rc_at_arm and rc_lost and "rcl_ignored" not in self.faults \
                and not (int(g["COM_RCL_EXCEPT"]) & 4 and off):
            self._once("rcl", NAV_MAP[int(g["NAV_RCL_ACT"])], t, True)
        if g["NAV_DLL_ACT"] and self.last["hb"] is not None and t - self.last["hb"] > g["COM_DL_LOSS_T"] + 2.5 \
                and "dll_ignored" not in self.faults and "dll" not in self.done:
            self._text("Connection to ground station lost")
            self._once("dll", NAV_MAP[int(g["NAV_DLL_ACT"])], t, True)
        if self.mode == "OFFBOARD" and "fence_ignored" not in self.faults:
            if g["GF_MAX_HOR_DIST"] > 0 and math.hypot(self.x, self.y) > g["GF_MAX_HOR_DIST"]:
                self._text("Maximum distance from home reached (%.5f)" % g["GF_MAX_HOR_DIST"])
                self._set(GF_MAP[int(g["GF_ACTION"])])
            elif g["GF_MAX_VER_DIST"] > 0 and -self.z > g["GF_MAX_VER_DIST"]:
                self._text("Maximum altitude above home reached (%.5f)" % g["GF_MAX_VER_DIST"])
                self._set(GF_MAP[int(g["GF_ACTION"])])
        s = self._sticks()
        v = s[0] if s is not None else None
        if v is not None and self.stick_prev is not None and dt > 0 and self.mode == "OFFBOARD" \
                and int(g["COM_RC_OVERRIDE"]) & 2 and "no_override" not in self.faults \
                and abs(v - self.stick_prev) / dt > g["COM_RC_STICK_OV"] / 100.0:
            self.mode = "POSCTL"
            self._text("Pilot took over using sticks")
        self.stick_prev = v

    def _move(self, t, dt):
        if self.lockdown_t is not None or not self.armed:
            if self.z < 0.0:
                self.vfall += 9.81 * dt
                self.z = min(0.0, self.z + self.vfall * dt)
            if self.armed and t - self.lockdown_t >= self.store["COM_KILL_DISARM"]:
                self.armed = False
            return
        clip = lambda d: max(-dt, min(dt, d))                           # noqa: E731
        if self.mode == "OFFBOARD" and self.sp is not None:
            mask, x, y, z, vx, vy, vz = self.sp
            if not mask & 8:
                self.x, self.y = self.x + vx * dt, self.y + vy * dt
            else:
                self.x, self.y = self.x + clip(x - self.x), self.y + clip(y - self.y)
            if not mask & 4:
                self.z += clip(z - self.z)
            elif not mask & 32:
                self.z += vz * dt
        elif self.mode == "AUTO_LAND":
            self.z = min(0.0, self.z + 0.7 * dt)
            if self.z >= 0.0:
                self.land_t = self.land_t or t
                if t - self.land_t > 2.0:
                    self.armed = False

    def step(self, t):
        dt, self.t_prev = t - self.t_prev, t
        if "stall" in self.faults and self.armed and t - self.arm_t > 2.0:
            return                                                       # PX4 simulation time stops
        if self.armed and self.lockdown_t is None:
            self._failsafes(t, dt)
        self._move(t, dt)
        sim = t - self.t0 + 1.0
        R = 6371000.0
        if "no_telemetry" not in self.faults:                           # amendment 1: smoke01's body failure
            S5.handle_message(self, Msg("LOCAL_POSITION_NED", time_boot_ms=int(round(sim * 1000)), x=self.x,
                                        y=self.y, z=self.z, vx=0.0, vy=0.0, vz=0.0))
        self.hist.append((round(sim, 6), self.armed, self._flags(t)))
        S5.handle_message(self, Msg("ATTITUDE", time_boot_ms=int(round(sim * 1000)), roll=0.0, pitch=0.0,
                                    yaw=math.pi / 2, rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0))
        S5.handle_message(self, Msg("HIL_STATE_QUATERNION", lat=int(round((47.0 + math.degrees(self.x / R)) * 1e7)),
                                    lon=int(round((8.0 + math.degrees(self.y / (R * math.cos(math.radians(47.0)))))
                                                  * 1e7)),
                                    alt=int(round((100.0 - self.z) * 1000)), attitude_quaternion=(1.0, 0.0, 0.0, 0.0),
                                    vx=0, vy=0, vz=0))
        if t >= self.next_hb - 1e-9:                                     # constant 1 Hz (mavlink_main.cpp:2143)
            self.k_hb += 1
            self.next_hb = t + 1.0 + jitter(self.k_hb)
            S5.handle_message(self, Msg("HEARTBEAT", base_mode=128 if self.armed else 0, custom_mode=CODE[self.mode]))
        if t >= self.next_echo - 1e-9:
            self.next_echo = t + 0.05
            if self._fresh(self.last["rc"], 0.5) and "no_rc_echo" not in self.faults:
                S5.handle_message(self, Msg("RC_CHANNELS", **{"chan%d_raw" % (i + 1): c for i, c in enumerate(self.rc)}))
            s = self._sticks()
            if s is not None:
                buttons = (self.sw_state or 0) << S5.KILL_SHIFT if self.source == "rc" else 0
                S5.handle_message(self, Msg("MANUAL_CONTROL", x=int(round(s[0] * 1000)), y=int(round(s[1] * 1000)),
                                            z=int(round(s[2] * 1000)), r=int(round(s[3] * 1000)), buttons=buttons))



PLAIN_TOPICS = ("vehicle_status", "actuator_armed", "vehicle_local_position", "vehicle_command", "vehicle_command_ack",
                "manual_control_setpoint", "input_rc", "manual_control_switches")


def synthetic_ulog(times_us=None, flag_rows=None, plain=PLAIN_TOPICS, flag_fields=FLAGS, dropout_after_us=None):
    """A valid ULog for the installed pyulog: plain topics (timestamp only) at times_us (default 0.1-150 s at 10 Hz),
    failsafe_flags (timestamp + bool flag_fields) at flag_rows [(t_us, *bools)] (default: all false at times_us), and
    optionally a logged dropout ('O' record) after the first sample at or past dropout_after_us."""
    def msg(kind, payload):
        return struct.pack("<HB", len(payload), ord(kind)) + payload
    times_us = list(range(100000, 150000001, 100000)) if times_us is None else list(times_us)
    if flag_rows is None:
        flag_rows = [(t,) + (0,) * len(flag_fields) for t in times_us]
    raw = bytearray(b"ULog\x01\x12\x35" + bytes([1]) + struct.pack("<Q", 1))
    for name in plain:
        raw += msg("F", (name + ":uint64_t timestamp;").encode())
    raw += msg("F", ("failsafe_flags:uint64_t timestamp;" + "".join("bool %s;" % f for f in flag_fields)).encode())
    fid = len(plain)
    for i, name in enumerate(plain):
        raw += msg("A", struct.pack("<BH", 0, i) + name.encode())
    raw += msg("A", struct.pack("<BH", 0, fid) + b"failsafe_flags")
    rows = [(t, 0, None) for t in times_us] + [(r[0], 1, r[1:]) for r in flag_rows]
    dropped = dropout_after_us is None
    for t, kind, vals in sorted(rows, key=lambda r: (r[0], r[1])):
        if kind == 0:
            for i in range(len(plain)):
                raw += msg("D", struct.pack("<HQ", i, t))
        else:
            raw += msg("D", struct.pack("<HQ", fid, t) + bytes(int(bool(v)) for v in vals))
        if not dropped and t >= dropout_after_us:
            raw += msg("O", struct.pack("<H", 150))
            dropped = True
    return bytes(raw)


def history_ulog(hist, end_rule="sdlog1"):
    """The fake PX4's ULog under SITL SDLOG_MODE 1: from boot until the first disarm, closed ~1 s after it
    (logger.cpp:1145-1150, 1588-1597; v2 review r1 B1). end_rule "before_disarm" stops 1 s BEFORE the first disarm (a
    negative). Plain topics at 10 Hz; failsafe_flags on a change or every 0.5 s, as PX4 publishes them."""
    armed_seen, t_dis = False, None
    for t, armed, _ in hist:
        if armed:
            armed_seen = True
        elif armed_seen:
            t_dis = t
            break
    if t_dis is None:
        end = hist[-1][0]
    else:
        end = t_dis + (S5.SDLOG_STOP_AFTER_DISARM_S if end_rule == "sdlog1" else -1.0)
    rows = [h for h in hist if h[0] <= end + 1e-9]
    flag_rows, last = [], None
    for t, _, f in rows:
        v = tuple(int(f[k]) for k in FLAGS)
        if last is None or v != last[1] or t - last[0] >= 0.5 - 1e-9:
            flag_rows.append((int(round(t * 1e6)),) + v)
            last = (t, v)
    return synthetic_ulog(times_us=[int(round(h[0] * 1e6)) for h in rows[::5]], flag_rows=flag_rows)


class TestRecording(unittest.TestCase):
    def test_real_installed_ulog_parser_positive_and_truncated_negative(self):
        with tempfile.TemporaryDirectory() as t:
            f = pathlib.Path(t) / "test.ulg"
            f.write_bytes(synthetic_ulog())
            out = S5.ulog_inventory(f)
            self.assertTrue(out["parse_ok"], out)
            self.assertEqual(S5.ulog_problems([out], SCEN["KIL"]), [])
            self.assertGreater(sum(x["count"] for x in out["topics"]), 100)
            f.write_bytes(synthetic_ulog()[:-2])
            self.assertFalse(S5.ulog_inventory(f)["parse_ok"])
            f.write_bytes(b"broken")
            self.assertFalse(S5.ulog_inventory(f)["parse_ok"])
            self.assertEqual(S5.ulog_problems([], SCEN["N0"]), ["NO_ULOG"])

    @unittest.skipUnless(pathlib.Path("/home/fair/motar_tmp/agq_geometry_diag_20261106/attempt01/00_diag_205bars_env5_native_native/rootfs/log/2026-11-06/02_54_50.ulg").is_file(), "historical ULog unavailable")
    def test_actual_historical_ulog_parser_positive(self):
        f = pathlib.Path("/home/fair/motar_tmp/agq_geometry_diag_20261106/attempt01/00_diag_205bars_env5_native_native/rootfs/log/2026-11-06/02_54_50.ulg")
        out = S5.ulog_inventory(f)
        self.assertTrue(out["parse_ok"], out)
        self.assertGreater(len(out["topics"]), 10)
        self.assertTrue(any(t["name"] == "vehicle_local_position" and t["count"] > 1 for t in out["topics"]))
        # v2 r1 M2: the four cause-flag names exist in a real v1.14.3 SITL ULog and are extracted row by row
        self.assertEqual(out["failsafe_flags"]["missing_fields"], [])
        self.assertGreater(len(S5.ulog_flag_rows([out])), 10)

    def test_ulog_dropout_missing_topic_and_flag_fields(self):            # v2 review r1 m4 (installed parser)
        with tempfile.TemporaryDirectory() as t:
            f = pathlib.Path(t) / "test.ulg"
            f.write_bytes(synthetic_ulog())
            good = S5.ulog_inventory(f)
            self.assertEqual((good["dropouts"], S5.ulog_problems([good], SCEN["OVR"])), ([], []))
            self.assertEqual(len(S5.ulog_flag_rows([good])), 1500)
            f.write_bytes(synthetic_ulog(dropout_after_us=50000000))
            drop = S5.ulog_inventory(f)
            self.assertTrue(drop["parse_ok"] and drop["dropouts"], drop)
            self.assertIn("ULOG_DROPOUT", S5.ulog_problems([drop], SCEN["N0"]))
            f.write_bytes(synthetic_ulog(plain=tuple(n for n in PLAIN_TOPICS if n != "input_rc")))
            no_rc = S5.ulog_inventory(f)
            self.assertTrue(no_rc["parse_ok"], no_rc)
            self.assertEqual(S5.ulog_problems([no_rc], SCEN["OVR"]), ["ULOG_TOPIC_MISSING input_rc"])
            self.assertEqual(S5.ulog_problems([no_rc], SCEN["OBL"]), [])      # scenario-specific: OBL has no RC
            f.write_bytes(synthetic_ulog(flag_fields=FLAGS[:2] + FLAGS[3:]))   # no gcs_connection_lost field
            no_gcs = S5.ulog_inventory(f)
            self.assertEqual(no_gcs["failsafe_flags"]["missing_fields"], ["gcs_connection_lost"])
            self.assertEqual(S5.ulog_problems([no_gcs], SCEN["N0"]), ["ULOG_FLAGS_FIELDS_MISSING"])
            self.assertEqual(S5.ulog_flag_rows([no_gcs]), [])

    def test_ulog_coverage_window(self):                                    # v2 review r1 B1, pure
        def inv(last, first=1.0, flags_every=0.5):
            topics = [{"name": n, "count": 100, "monotonic": True, "first_us": int(first * 1e6),
                       "last_us": int(last * 1e6), "max_gap_s": 0.1} for n in PLAIN_TOPICS + ("failsafe_flags",)]
            rows = [(first + i * flags_every, {k: False for k in FLAGS})
                    for i in range(int((last - first) / flags_every) + 1)]
            return dict(flag_log(rows)[0], parse_ok=True, dropouts=[], topics=topics)
        res = {"t_pre_start": 10.0, "t_observation_end": 30.0}
        disarm = {"hb": [[10.3 + k, 0.0, 128 if 10.3 + k < 22.4 else 0, CODE[OFF]] for k in range(20)]}
        armed = {"hb": [[10.3 + k, 0.0, 128, CODE[OFF]] for k in range(20)]}
        kil = SCEN["KIL"]
        end, why = S5.ulog_coverage_end(res, disarm)
        self.assertAlmostEqual(end, 23.3)
        self.assertEqual(why, "first_disarmed_heartbeat")
        self.assertEqual(S5.ulog_problems([inv(23.3)], kil, res, disarm), [])          # closes ~1 s after disarm
        self.assertEqual(S5.ulog_problems([inv(21.4)], kil, res, disarm),              # stops before the disarm
                         ["ULOG_POSITION_COVERAGE", "ULOG_FLAGS_COVERAGE"])
        self.assertIn("ULOG_POSITION_COVERAGE", S5.ulog_problems([inv(23.3)], kil, res))   # the old rule (B1)
        self.assertEqual(S5.ulog_coverage_end(res, armed), (30.0, "observation_end"))
        self.assertEqual(S5.ulog_problems([inv(30.0)], SCEN["N0"], res, armed), [])     # armed: full window
        self.assertIn("ULOG_POSITION_COVERAGE", S5.ulog_problems([inv(23.3)], SCEN["N0"], res, armed))
        late = inv(30.0)
        late["topics"] = [dict(t, first_us=int(10.5e6)) for t in late["topics"]]
        self.assertIn("ULOG_POSITION_COVERAGE", S5.ulog_problems([late], SCEN["N0"], res, armed))
        sparse = inv(30.0, flags_every=1.5)
        self.assertEqual(S5.ulog_problems([sparse], SCEN["N0"], res, armed), ["ULOG_FLAGS_COVERAGE"])

    def test_window_excludes_cleanup_status_ack_mode_and_truth(self):
        r = record(t_observation_end=10., ulogs=flag_log(flag_rows("GFH")))
        tr = trace(hb_rows((0., OFF, True), (11., "AUTO_LOITER", True)))
        tr["statustext"] = [[11., 2, "Maximum distance from home reached"]]
        got = S5.assess(SCEN["GFH"], r, tr)
        self.assertTrue(got["valid"])
        self.assertFalse(got["passed"])
        self.assertIn("NO_DEPARTURE", got["fail_reason"])
        r = record(t_observation_end=20., ulogs=flag_log(flag_rows("OBL")))
        got = S5.assess(SCEN["OBL"], r, tr)
        self.assertTrue(got["valid"])
        self.assertFalse(got["passed"])  # delayed response still rejected
        self.assertIn("LATE", got["fail_reason"])


class FakeSession:
    def __init__(self, ep_dir, links=None, ulog_end="sdlog1", **kw):
        self.episode_dir = pathlib.Path(ep_dir)
        self.links, self.ulog_end = links, ulog_end
        binary = "/home/fair/workspaces/px4_ws/PX4-Autopilot/build/px4_sitl_default/bin/px4"
        self._procs = {"px4": {"record": {"argv": [binary, "-d", str(pathlib.Path(binary).parent.parent / "etc")],
                                          "cwd": str(self.episode_dir / "rootfs")}}}

    def __enter__(self):
        self.episode_dir.mkdir(parents=True)
        return self

    def __exit__(self, *a):
        (self.episode_dir / "session.json").write_text(json.dumps({"cleanup": "complete"}))
        log = self.episode_dir / "rootfs/test.ulg"
        log.parent.mkdir(exist_ok=True)
        hist = self.links[0].hist if self.links else None
        log.write_bytes(history_ulog(hist, self.ulog_end) if hist else synthetic_ulog())
        return False

    def write_world(self, name, text):
        (self.episode_dir / "world").mkdir(exist_ok=True)
        (self.episode_dir / "world" / (name + ".world")).write_text(text)

    def sleep(self, s):
        pass

    def start(self, role, argv, cwd, env):
        self.logger_stopped = True

    def wait(self, role, timeout):
        return 0

    def check(self, required=()):
        pass


FakeOS = NS(OwnedSession=FakeSession, direct_launch_plan=lambda d, n, env, model: {"model": model,
                "px4": {"argv": ["/home/fair/workspaces/px4_ws/PX4-Autopilot/build/px4_sitl_default/bin/px4", "-d",
                         "/home/fair/workspaces/px4_ws/PX4-Autopilot/build/px4_sitl_default/etc"],
                        "cwd": pathlib.Path(d) / "rootfs", "env": env}},
            launch_px4_gazebo=lambda s, p: None,
            runtime_binding=lambda plan, airframe: {"files": {"px4_binary": {"path": __file__,
                                    "sha256": SF.sha256_file(__file__)}}, "airframe": airframe})
FAST = {"settle_s": 0.5, "boot_wait_s": 5.0, "pre_offboard_s": 0.5, "pre_arm_s": 0.3, "takeoff_s": 3.0}


def rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


class TestLivePath(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp())
        (self.tmp / "empty.world").write_text("<sdf><world><physics><ode><solver><iters>10</iters></solver></ode>"
                                              "</physics></world></sdf>")
        self.V0 = NS(WORLDS=self.tmp, ODE_ITERS=100, SPAWN=(1.01, 0.98), rot=rot, PARAMS={"MPC_XY_VEL_P_ACC": 2.5})
        self.n = 0

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_one(self, sid, faults=(), ulog_end="sdlog1", inject=(), raises=None):
        clock = FakeClock()
        links = []

        def factory():
            links.append(FakePX4Link(clock, faults))
            for m in inject:                                             # amendment 1: real pymavlink objects
                S5.handle_message(links[0], m)
            return links[0]
        ep = self.tmp / ("ep%03d" % self.n)
        self.n += 1
        fake_os = NS(**vars(FakeOS))
        fake_os.OwnedSession = lambda d, **kw: FakeSession(d, links=links, ulog_end=ulog_end, **kw)
        if raises is not None:
            with self.assertRaises(raises) as ctx:
                S5.run_scenario(fake_os, self.V0, SCEN[sid], ep, link_factory=factory, clock=clock,
                                sleep=clock.sleep, timing=FAST)
            self.assertTrue(links and links[0].closed and links[0].stop)
            return ctx.exception, ep
        res = S5.run_scenario(fake_os, self.V0, SCEN[sid], ep, link_factory=factory, clock=clock, sleep=clock.sleep,
                              timing=FAST)
        self.assertTrue(links and links[0].closed and links[0].stop)
        self.assertFalse(res.get("rx_thread_alive_after_join"))
        return res, ep

    def test_every_scenario_passes_against_a_faithful_px4(self):
        for sid in S5.SCHEDULE + ("N0R", "N0J"):
            res, ep = self.run_one(sid)
            self.assertEqual((res["outcome"], res["valid"], res["passed"]), ("COMPLETED", True, True),
                             (sid, res.get("error"), res.get("invalid_reason"), res.get("fail_reason")))
            self.assertEqual(res["param_problems"], [])
            self.assertEqual(res["trace_sha256"], SF.sha256_file(ep / "trace.json"))
            self.assertGreaterEqual(res["criteria"]["pre_observations"], S5.MIN_PRE_OBS)
            self.assertTrue(res["ended_disarmed"], sid)
            self.assertAlmostEqual(res["real_time_factor"], 1.0, places=2)
            if SCEN[sid].get("path"):
                self.assertGreaterEqual(res["criteria"]["manual_echo_pre"], S5.MIN_ECHO)
            if SCEN[sid].get("event_text"):
                self.assertEqual(res["criteria"]["event_source"], "statustext", sid)
            if SCEN[sid].get("cause_flag"):                                  # v2 r1 M2: the right cause
                self.assertEqual(res["criteria"]["cause"]["first_raised"], [SCEN[sid]["cause_flag"]], sid)
            elif SCEN[sid].get("cause_text") or sid == "FDS":
                self.assertIsNone(res["criteria"]["cause"]["other_raised"], sid)
            self.assertGreater(len(S5.ulog_flag_rows(res["ulogs"])), 10, sid)

    @unittest.skipUnless(MAV is not None, "pymavlink not installed")
    def test_live_path_records_real_bad_data_and_unknown_messages(self):  # amendment 1
        msgs = real_messages()
        res, ep = self.run_one("N0", inject=msgs)
        self.assertEqual((res["outcome"], res["valid"], res["passed"]), ("COMPLETED", True, True),
                         (res.get("error"), res.get("invalid_reason")))
        self.assertEqual(res["trace_sha256"], SF.sha256_file(ep / "trace.json"))
        trace = json.loads((ep / "trace.json").read_text())
        foreign = [p for p in trace["rx_packets"] if p["type"] in REAL_TYPES and p["source"] != [1, 1]]
        self.assertEqual([p["type"] for p in foreign], REAL_TYPES)
        for p, m in zip(foreign, msgs):
            self.assertEqual(bytes.fromhex(p["packet_hex"]), bytes(m.get_msgbuf()))
            self.assertEqual(unhex(p["payload"]), plain(m.to_dict()))
        # the ledger recomputation from the written trace agrees (no "differs" problem); only the short schedule is
        # reported, which the positive control of test_ledger_row_recomputes_and_tampering_voids also shows
        row = json.loads(json.dumps(dict({"scenario": "N0"}, **res, index=0), default=str))
        attempt = {"schedule": [{"scenario": "N0"}], "scenarios": json.loads(json.dumps(list(S5.scenarios()))),
                   "protocol": S5.PROTOCOL, "scope": S5.SCOPE, "prereg": S5.PREREG, "amendment": S5.AMENDMENT,
                   "kernel_faults_before": 0, "git": {"head": "h"}}
        final = {"status": "COMPLETE", "kernel_faults_after": 0, "git_after": {"head": "h", "dirty": ""}}
        problems = S5.integrity_problems(attempt, final, [row], self.tmp)
        self.assertEqual(problems, ["attempt schedule differs from registered schedule"])

    def test_live_path_pins_the_autopilot_through_real_mavutil(self):    # amendment 1, smoke01's first failure
        res, ep = self.run_one("N0")
        self.assertEqual((res["outcome"], res["valid"], res["passed"]), ("COMPLETED", True, True))
        pin = res["autopilot"]
        self.assertEqual(pin["source"], [1, 1])
        if MAVUTIL is not None:                                          # the target pymavlink itself derived
            self.assertEqual(pin["pymavlink_target"], [1, 0])
            self.assertEqual([r["source"] for r in pin["rejected"]], [[255, 190], [1, 100]])
        trace = json.loads((ep / "trace.json").read_text())
        self.assertTrue(trace["tx_calls"])
        self.assertTrue(all(r["destination"] == [1, 0] and r["autopilot"] == [1, 1] for r in trace["tx_calls"]))
        self.assertGreater(len(trace["hb"]), 5)
        self.assertEqual((res["unserializable_values"], res["recording_warnings"]), ({"count": 0, "types": []}, []))

    def test_body_failure_stays_the_reason_when_recording_fails(self):   # amendment 1, smoke01's masking
        class Unprintable:
            def __repr__(self):
                raise ValueError("repr failed")
        # positive control: smoke01's body failure alone is recorded as HARNESS_INVALID with its trace
        res, ep = self.run_one("N0", ("no_telemetry",))
        self.assertEqual((res["outcome"], res["error"]), ("HARNESS_INVALID",
                                                          "RuntimeError: HARNESS_INVALID no telemetry within 5 s"))
        self.assertFalse(res["valid"])
        self.assertTrue((ep / "trace.json").is_file())
        self.assertNotIn("recording_error", res)
        # the body fails, then the recording fails: the first failure leads, the second follows
        bad = Msg("BAD_REPR", src=(42, 7), field=Unprintable())
        exc, ep = self.run_one("N0", ("no_telemetry",), inject=[bad], raises=S5.RecordingAfterError)
        self.assertEqual(exc.first, "RuntimeError: HARNESS_INVALID no telemetry within 5 s")
        self.assertEqual(exc.second, "ValueError: repr failed")
        self.assertTrue(str(exc).startswith("RuntimeError: HARNESS_INVALID no telemetry within 5 s; then, while "
                                            "recording: ValueError"), str(exc))
        self.assertIsInstance(exc.__cause__, ValueError)
        # through run_unit the unit's abort reason carries the first failure ahead of the recording failure
        with tempfile.TemporaryDirectory() as t:
            S5.run_unit.terminated = None
            status, reason, done = S5.run_unit(S5.smoke_schedule()[:1], lambda i, item: (_ for _ in ()).throw(exc),
                                               FakeAT(), pathlib.Path(t) / "l.jsonl", busy_wait_s=0, host=HOST)
        self.assertEqual((status, done), ("ABORTED", 0))
        self.assertLess(reason.index("HARNESS_INVALID no telemetry"), reason.index("ValueError"))
        # negative control: a recording failure with no earlier failure is raised as itself
        exc, _ = self.run_one("N0", inject=[bad], raises=ValueError)
        self.assertEqual(str(exc), "repr failed")

    def test_unserializable_values_are_counted_and_warned(self):          # amendment 1, review MINOR-2
        class Opaque:
            def __repr__(self):
                return "<Opaque>"
        res, ep = self.run_one("N0", inject=[Msg("ODD", src=(42, 7), a=Opaque(), b={1, 2})])
        self.assertEqual((res["valid"], res["passed"]), (True, True))     # a warning, not a validity problem
        self.assertEqual(res["unserializable_values"]["count"], 2)
        self.assertEqual(res["unserializable_values"]["types"][0], "builtins.set")
        self.assertTrue(res["unserializable_values"]["types"][1].endswith(".Opaque"))
        self.assertEqual(len(res["recording_warnings"]), 1)
        self.assertIn("UNSERIALIZABLE_VALUES 2", res["recording_warnings"][0])
        trace = json.loads((ep / "trace.json").read_text())
        self.assertEqual(len(S5.unserializable_values(trace)), 2)
        clean, _ = self.run_one("N0")                                    # positive control: zero, no warning
        self.assertEqual((clean["unserializable_values"]["count"], clean["recording_warnings"]), (0, []))
        # the ledger recomputation checks the count against the written trace
        row = json.loads(json.dumps(dict({"scenario": "N0"}, **res, index=self.n - 2), default=str))
        attempt = {"schedule": [{"scenario": "N0"}], "scenarios": json.loads(json.dumps(list(S5.scenarios()))),
                   "protocol": S5.PROTOCOL, "scope": S5.SCOPE, "prereg": S5.PREREG, "amendment": S5.AMENDMENT,
                   "kernel_faults_before": 0, "git": {"head": "h"}}
        final = {"status": "COMPLETE", "kernel_faults_after": 0, "git_after": {"head": "h", "dirty": ""}}
        base = S5.integrity_problems(attempt, final, [row], self.tmp)
        self.assertNotIn("unserializable-value count of scenario N0 differs", base)
        row["unserializable_values"]["count"] = 0
        self.assertIn("unserializable-value count of scenario N0 differs",
                      S5.integrity_problems(attempt, final, [row], self.tmp))
        reading = S5.reading(final, [row], attempt)
        self.assertEqual(list(reading["recording_warnings"]), ["N0"])

    def test_ulog_closing_after_the_first_disarm_is_covered(self):        # v2 review r1 B1
        for sid in ("KIL", "FDS", "RCL"):
            res, ep = self.run_one(sid)
            self.assertEqual((res["valid"], res["passed"]), (True, True), (sid, res.get("invalid_reason")))
            tr = json.loads((ep / "trace.json").read_text())
            end, why = S5.ulog_coverage_end(res, tr)
            pos = [t for t in res["ulogs"][0]["topics"] if t["name"] == "vehicle_local_position"][0]
            if sid in ("KIL", "FDS"):
                self.assertEqual(why, "first_disarmed_heartbeat", sid)
                self.assertLess(end, res["t_observation_end"] - 1.0, sid)
                # the log closed ~1 s after the disarm, long before the observation end: the old rule (no trace,
                # coverage to the observation end) would have made this scenario INVALID on a faithful PX4
                self.assertLess(pos["last_us"] * 1e-6, res["t_observation_end"] - 1.0, sid)
                self.assertIn("ULOG_POSITION_COVERAGE", S5.ulog_problems(res["ulogs"], SCEN[sid], res), sid)
            self.assertEqual(S5.ulog_problems(res["ulogs"], SCEN[sid], res, tr), [], sid)
        for sid in ("KIL", "FDS"):                                       # coverage stops BEFORE the disarm
            res, _ = self.run_one(sid, ulog_end="before_disarm")
            self.assertEqual(res["valid"], False, sid)
            self.assertIn("ULOG_POSITION_COVERAGE", res["invalid_reason"], sid)

    def test_wrong_cause_fails_and_missing_cause_text_is_invalid(self):    # v2 review r1 M2
        for sid in ("OBL", "GFH", "GFV", "RCL", "DLL"):
            res, _ = self.run_one(sid, ("wrong_cause",))
            self.assertEqual((res["valid"], res["passed"]), (True, False), (sid, res.get("invalid_reason")))
            self.assertIn("WRONG_CAUSE", res["fail_reason"], sid)
            self.assertIn("expected " + SCEN[sid]["cause_flag"], res["fail_reason"], sid)
            self.assertEqual(res["criteria"]["departure_mode"], "AUTO_LOITER", sid)  # the mode sequence was right
        res, _ = self.run_one("RCL", ("wrong_cause",))                         # RC loss reported as GCS loss
        self.assertIn("WRONG_CAUSE gcs_connection_lost (expected manual_control_signal_lost)", res["fail_reason"])
        for sid in ("OVJ", "OVR", "KIL", "FDS"):
            res, _ = self.run_one(sid, ("flag_glitch",))
            self.assertEqual((res["valid"], res["passed"]), (True, False), (sid, res.get("invalid_reason")))
            self.assertIn("WRONG_CAUSE gcs_connection_lost", res["fail_reason"], sid)
        for sid in ("OVJ", "OVR", "KIL"):
            res, _ = self.run_one(sid, ("silent_text",))
            self.assertEqual((res["valid"], res["invalid_reason"]), (False, "EVENT_TEXT_NOT_SEEN"), sid)

    def test_each_broken_px4_fails_its_scenario(self):
        cases = [("N0", "spurious", "MODE_CHANGED"), ("OBL", "slow_obl", "LATE"),
                 ("GFH", "fence_ignored", "NO_DEPARTURE"), ("GFV", "fence_ignored", "NO_DEPARTURE"),
                 ("OVJ", "no_override", "NO_DEPARTURE"), ("OVR", "no_override", "NO_DEPARTURE"),
                 ("RCL", "rcl_ignored", "NO_DEPARTURE"), ("DLL", "dll_ignored", "NO_DEPARTURE"),
                 ("KIL", "ignore_kill", "NO_DEPARTURE"), ("FDS", "nonforced_accepted", "NONFORCED_DISARM_ACTED")]
        self.assertEqual(sorted({c[0] for c in cases}), sorted(S5.SCHEDULE))  # every scenario has a negative (M3)
        for sid, fault, needle in cases:
            res, _ = self.run_one(sid, (fault,))
            self.assertEqual((res["valid"], res["passed"]), (True, False), (sid, fault, res.get("invalid_reason")))
            self.assertIn(needle, res["fail_reason"], (sid, fault))

    def test_harness_faults_are_invalid_not_failed(self):
        for sid in ("OVR", "RCL", "KIL", "N0R"):                         # review r1 M1: never a FAIL_SITL
            res, _ = self.run_one(sid, ("rc_not_selected",))
            self.assertEqual((res["valid"], res["invalid_reason"]), (False, "RC_PATH_NOT_SELECTED"), sid)
        res, _ = self.run_one("KIL", ("no_rc_echo",))
        self.assertEqual((res["valid"], res["invalid_reason"]), (False, "RC_INPUT_NOT_SEEN"))
        res, _ = self.run_one("N0", ("drop_param",))
        self.assertEqual((res["outcome"], res["invalid_reason"]), ("PARAM_MISMATCH", "PARAM_MISMATCH"))
        self.assertTrue(any("GF_ACTION" in p for p in res["param_problems"]))
        res, _ = self.run_one("N0", ("stall",))
        self.assertEqual((res["outcome"], res["valid"]), ("ERROR", False))
        self.assertIn("PX4_TIME_STALLED", res["error"])

    def test_recording_contract_has_positive_counts_and_rejects_missing_stale_and_gaps(self):
        res, ep = self.run_one("N0")
        tr = json.loads((ep / "trace.json").read_text())
        good = S5.recording_contract(SCEN["N0"], res, tr)
        self.assertEqual(good["problems"], [])
        self.assertGreater(good["inventory"]["LOCAL_POSITION_NED"]["count"], 100)
        self.assertGreater(len(tr["tx_calls"]), 100)
        self.assertTrue(all("mono_return" in r for r in tr["tx_calls"]))
        self.assertTrue(any(r["kind"] == "command" for r in tr["tx_calls"]))
        for kind in ("HIL_STATE_QUATERNION", "HEARTBEAT", "LOCAL_POSITION_NED"):
            bad = dict(tr, rx_packets=[p for p in tr["rx_packets"] if p["type"] != kind])
            self.assertIn("RX_COVERAGE " + kind, S5.recording_contract(SCEN["N0"], res, bad)["problems"])
        bad = json.loads(json.dumps(tr))
        for r in bad["samples"]:
            r["truth_age_s"] = 1.
        self.assertIn("TRUTH_SAMPLE_STALE", S5.recording_contract(SCEN["N0"], res, bad)["problems"])
        bad = dict(tr, tx_streams=[])
        self.assertTrue(any("TX_COVERAGE" in x for x in S5.recording_contract(SCEN["N0"], res, bad)["problems"]))
        # v2 review r1 m4/m6: one stale (wall-clock) position-stamp age inside the window is INVALID
        lo, hi = res["t_pre_start"], res["t_observation_end"]
        stale = json.loads(json.dumps(tr))
        i = next(k for k, p in enumerate(stale["rx_packets"]) if p["type"] == "ATTITUDE" and p["sim_t"] is not None
                 and lo < p["sim_t"] < hi)              # amendment 1: packets before the first position have no sim_t
        stale["rx_packets"][i]["pos_age_s"] = 0.15
        self.assertEqual(S5.recording_contract(SCEN["N0"], res, stale)["problems"], ["RX_POSITION_STAMP_STALE ATTITUDE"])
        ok_age = json.loads(json.dumps(tr))
        ok_age["rx_packets"][i]["pos_age_s"] = 0.10                           # at the limit: still valid
        self.assertEqual(S5.recording_contract(SCEN["N0"], res, ok_age)["problems"], [])
        self.assertIn("RECORDING_CONTRACT", S5.assess(SCEN["N0"], dict(res, recording=S5.recording_contract(
            SCEN["N0"], res, stale)), tr)["invalid_reason"])

    def test_logger_stop_refuses_wrong_rootfs_and_failed_command(self):
        with FakeSession(self.tmp / "logger_ep") as session:
            plan = FakeOS.direct_launch_plan(session.episode_dir, "w", {}, S5.MODEL)
            self.assertEqual(S5.stop_owned_logger(session, plan)["returncode"], 0)
            session.wait = lambda *a, **kw: 1
            with self.assertRaisesRegex(RuntimeError, "LOGGER_STOP_FAILED"):
                S5.stop_owned_logger(session, plan)
            plan["px4"]["cwd"] = self.tmp
            with self.assertRaisesRegex(RuntimeError, "ROOTFS_MISMATCH"):
                S5.stop_owned_logger(session, plan)

    def test_logger_stop_refuses_plan_ownership_and_binary_mismatch(self):   # v2 review r1 m4
        bindir = self.tmp / "build/bin"
        bindir.mkdir(parents=True)
        binary = bindir / "px4"
        binary.write_bytes(b"#!/bin/sh\n")
        (bindir / "px4-logger").symlink_to("px4")
        argv = [str(binary), "-d", str(bindir.parent / "etc")]
        with FakeSession(self.tmp / "logger_ep2") as session:
            rootfs = session.episode_dir / "rootfs"
            session._procs["px4"]["record"] = {"argv": list(argv), "cwd": str(rootfs)}

            def plan(**px4):
                return {"px4": dict({"argv": list(argv), "cwd": rootfs, "env": {}}, **px4)}
            self.assertEqual(S5.stop_owned_logger(session, plan(), expected_binary=binary)["returncode"], 0)
            with self.assertRaisesRegex(RuntimeError, "LOGGER_STOP_INSTANCE_PLAN_MISMATCH"):
                S5.stop_owned_logger(session, plan(argv=argv + ["-i", "1"]), expected_binary=binary)
            session._procs["px4"]["record"] = {"argv": argv[:1], "cwd": str(rootfs)}
            with self.assertRaisesRegex(RuntimeError, "LOGGER_STOP_OWNERSHIP_MISMATCH"):
                S5.stop_owned_logger(session, plan(), expected_binary=binary)
            session._procs["px4"]["record"] = {"argv": list(argv), "cwd": str(rootfs)}
            (bindir / "px4-logger").unlink()
            (bindir / "px4-logger").write_bytes(b"#!/bin/sh\n")                 # a copy, not the launched binary
            with self.assertRaisesRegex(RuntimeError, "LOGGER_STOP_BINARY_MISMATCH"):
                S5.stop_owned_logger(session, plan(), expected_binary=binary)

    def test_ledger_row_recomputes_and_tampering_voids(self):
        res, ep = self.run_one("OBL")
        row = json.loads(json.dumps(dict({"scenario": "OBL"}, **res, index=0), default=str))
        attempt = {"schedule": S5.schedule(), "scenarios": json.loads(json.dumps(list(S5.scenarios()))),
                   "protocol": S5.PROTOCOL, "scope": S5.SCOPE, "prereg": S5.PREREG,
                   "kernel_faults_before": 0, "git": {"head": "h"}}
        row["scenario"] = "OBL"
        attempt["schedule"] = [{"scenario": "OBL"}]  # integrity correctly rejects a shortened registered schedule
        final = {"status": "COMPLETE", "kernel_faults_after": 0, "git_after": {"head": "h", "dirty": ""}}
        r = S5.reading(final, [row], attempt, self.tmp)
        self.assertEqual(r["reading"], "VOID")
        self.assertTrue(any("registered schedule" in x for x in r["integrity_problems"]))
        with open(ep / "trace.json", "a") as f:
            f.write(" ")
        r = S5.reading(final, [row], attempt, self.tmp)
        self.assertEqual(r["reading"], "VOID")
        self.assertTrue(any("missing or changed" in x for x in r["integrity_problems"]))


# ------------------------------------------------------------------------------------------- unit rules

class FakeAT:
    def __init__(self, faults=(0,), idle=True, source_p=(), git=("h",)):
        self.faults, self.i, self.idle, self.source_p = list(faults), 0, idle, list(source_p)
        self.git, self.gi = list(git), 0

    def git_state(self, root):
        v = self.git[min(self.gi, len(self.git) - 1)]
        self.gi += 1
        return {"head": v, "dirty": ""}

    def kernel_faults(self):
        v = self.faults[min(self.i, len(self.faults) - 1)]
        self.i += 1
        return v

    def idle_check(self):
        v = self.idle.pop(0) if isinstance(self.idle, list) else self.idle
        return v, {"core_idle": {}}

    def source_problems(self, root, sources):
        return list(self.source_p)


OK_REC = {"outcome": "COMPLETED", "valid": True, "invalid_reason": None, "passed": True,
          "runtime_binding": {"files": {"px4_binary": {"sha256": "p"}}}}
BAD_REC = {"outcome": "ERROR", "valid": False, "invalid_reason": "ERROR", "passed": None}
HOST = lambda: {"loadavg": [0.1, 0.1, 0.1], "gpu_compute_apps": []}           # noqa: E731


class TestPymavlinkBinding(unittest.TestCase):
    @unittest.skipUnless(MAVUTIL is not None, "pymavlink not installed")
    def test_two_real_connections_across_automatic_v1_v2_switch(self):
        """A fresh interpreter starts in v1, receives v2, then opens a second real mavutil connection.
        The old active-decoder binding changes; the fixed source set does not. Child globals cannot leak to PUBLIC_CPU.
        """
        code = '''
import json, pathlib, sys, types
sys.path.insert(0, sys.argv[1])
sys.path.append(sys.argv[2])
import sitl_s5_safety as s5
from pymavlink import mavutil as mu
assert mu.mavlink.WIRE_PROTOCOL_VERSION == "1.0"
records = []
for i in range(2):
    c = mu.mavlink_connection(sys.argv[3], notimestamps=True)
    try:
        link = types.SimpleNamespace(mu=mu, m=c)
        before = type(c.mav).__module__
        fixed = s5.pymavlink_files(link)
        old_path = pathlib.Path(sys.modules[before].__file__)
        old = {"pymavlink_dialect": {"sha256": s5.SF.sha256_file(old_path)}}
        msg = c.recv_match(type="HEARTBEAT", blocking=False)
        assert msg is not None and msg.get_srcComponent() == 1
        assert c.WIRE_PROTOCOL_VERSION == "2.0"
        assert fixed == s5.pymavlink_files(link)
        records.append({"fixed": fixed, "old": old, "before": before, "after": type(c.mav).__module__})
    finally:
        c.close()
assert records[0]["before"].startswith("pymavlink.dialects.v10.")
assert records[1]["before"].startswith("pymavlink.dialects.v20.")
print(json.dumps(records))
'''
        env = dict(os.environ, MAVLINK_DIALECT="all", PYTHONDONTWRITEBYTECODE="1")
        env.pop("MAVLINK20", None)
        with tempfile.TemporaryDirectory() as d:
            wire = pathlib.Path(d) / "wire.raw"
            wire.write_bytes(px4_wire(others=False))
            p = subprocess.run([sys.executable, "-B", "-c", code, str(TOOLS), str(PYLINK), str(wire)],
                               env=env, capture_output=True, text=True, timeout=60)
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        rows = json.loads(p.stdout.strip().splitlines()[-1])
        self.assertEqual(len(rows), 2)
        fixed = [SF.binding_core({"files": r["fixed"]}) for r in rows]
        old = [SF.binding_core({"files": r["old"]}) for r in rows]
        self.assertEqual(len(fixed[0]), 3)
        self.assertEqual(fixed[0], fixed[1])
        self.assertNotEqual(old[0], old[1])
        rules = TestUnitRules()
        good = [dict(OK_REC, runtime_binding={"files": r["fixed"]}) for r in rows]
        self.assertEqual(rules.go(good, n=2), ("COMPLETE", None, 2))
        bad = [dict(OK_REC, runtime_binding={"files": r["old"]}) for r in rows]
        self.assertEqual(rules.go(bad, n=2)[0], "ABORTED")
        self.assertEqual(rules.rows[-1]["invalid_reason"], "BINDING_CHANGED")

    def test_both_dialect_file_changes_still_abort_and_missing_source_refused(self):
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            mu = NS(__file__=str(root / "mavutil.py"), current_dialect="all")
            (root / "mavutil.py").write_text("mavutil source")
            paths = [root / "dialects" / v / "all.py" for v in ("v10", "v20")]
            for path in paths:
                path.parent.mkdir(parents=True)
                path.write_text("original " + str(path))
            link = NS(mu=mu)
            baseline = S5.pymavlink_files(link)
            rules = TestUnitRules()
            good = dict(OK_REC, runtime_binding={"files": baseline})
            self.assertEqual(rules.go([good, good], n=2)[0], "COMPLETE")
            for path in paths + [root / "mavutil.py"]:
                original = path.read_text()
                path.write_text(original + " changed")
                changed = dict(OK_REC, runtime_binding={"files": S5.pymavlink_files(link)})
                self.assertEqual(rules.go([good, changed], n=2)[0], "ABORTED")
                self.assertEqual(rules.rows[-1]["invalid_reason"], "BINDING_CHANGED")
                path.write_text(original)
            paths[-1].unlink()
            self.assertRaises(FileNotFoundError, S5.pymavlink_files, link)

    def test_binding_core_ignores_episode_world_paths_but_compares_source_bytes(self):
        a = {"files": {"world": {"sha256": "scene-a"}, "pymavlink_dialect_v10": {"path": "/ep0/a", "sha256": "x"}},
             "log_dir": "/ep0/logs"}
        b = {"files": {"world": {"sha256": "scene-b"}, "pymavlink_dialect_v10": {"path": "/ep1/a", "sha256": "x"}},
             "log_dir": "/ep1/logs"}
        self.assertEqual(SF.binding_core(a), SF.binding_core(b))
        b["files"]["pymavlink_dialect_v10"]["sha256"] = "y"
        self.assertNotEqual(SF.binding_core(a), SF.binding_core(b))


class TestUnitRules(unittest.TestCase):
    def go(self, recs, at=None, n=5, host=HOST):
        it = iter(recs)
        with tempfile.TemporaryDirectory() as t:
            S5.run_unit.terminated = None
            out = S5.run_unit(S5.schedule()[:n], lambda i, item: dict(next(it)), at or FakeAT(),
                              pathlib.Path(t) / "l.jsonl", busy_wait_s=0, host=host)
            p = pathlib.Path(t) / "l.jsonl"
            self.rows = [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []
        return out

    def test_complete(self):
        self.assertEqual(self.go([OK_REC] * 5), ("COMPLETE", None, 5))
        self.assertEqual([r["scenario"] for r in self.rows], list(S5.SCHEDULE[:5]))
        self.assertTrue(all(r["passed"] and r["host_before"]["loadavg"] for r in self.rows))

    def test_stop_rules(self):
        status, reason, done = self.go([BAD_REC] + [OK_REC] * 4)
        self.assertEqual((status, done), ("ABORTED", 1))
        self.assertIn("first INVALID", reason)
        status, reason, done = self.go([OK_REC, BAD_REC, BAD_REC, BAD_REC, OK_REC])
        self.assertEqual((status, done), ("ABORTED", 2))
        self.assertIn("first INVALID", reason)
        status, reason, done = self.go([OK_REC] * 5, at=FakeAT(faults=(0, 0, 1)))
        self.assertEqual((status, done), ("ABORTED", 2))
        self.assertIn("kernel fault", reason)
        status, reason, done = self.go([OK_REC] * 5, at=FakeAT(git=("h", "h", "x")))
        self.assertEqual((status, done), ("ABORTED", 2))
        self.assertIn("tree changed", reason)

    def test_host_busy_and_binding(self):
        status, _, done = self.go([OK_REC] * 5, at=FakeAT(idle=[True, False, True, True, True]))
        self.assertEqual((status, done), ("ABORTED", 2))
        self.assertEqual((self.rows[1]["outcome"], self.rows[1]["invalid_reason"]), ("NOT_RUN", "HOST_BUSY"))
        other = dict(OK_REC, runtime_binding={"files": {"px4_binary": {"sha256": "q"}}})
        self.go([OK_REC, other, OK_REC, OK_REC, OK_REC])
        self.assertEqual([r["invalid_reason"] for r in self.rows], [None, "BINDING_CHANGED"])
        self.assertIsNone(self.rows[1]["passed"])
        # v2 review r1 m3: a GPU compute process (other than the remote-desktop client) makes the host busy
        rustdesk = lambda: {"loadavg": [0.1] * 3, "gpu_compute_apps": ["1, /usr/share/rustdesk/rustdesk, 9 MiB"]}  # noqa
        self.assertEqual(self.go([OK_REC] * 5, host=rustdesk), ("COMPLETE", None, 5))
        gpu = lambda: {"loadavg": [0.1] * 3, "gpu_compute_apps": ["2, python, 900 MiB"]}           # noqa: E731
        status, _, done = self.go([OK_REC] * 5, host=gpu)
        self.assertEqual((status, done), ("ABORTED", 1))
        self.assertEqual((self.rows[0]["outcome"], self.rows[0]["invalid_reason"]), ("NOT_RUN", "HOST_BUSY"))
        self.assertIn("python", str(self.rows[0]["host_idle_before"]["gpu"]))

    def test_termination_recorded(self):
        def kb(i, item):
            raise KeyboardInterrupt("SIGTERM")
        with tempfile.TemporaryDirectory() as t:
            S5.run_unit.terminated = None
            out = S5.run_unit(S5.schedule(), kb, FakeAT(), pathlib.Path(t) / "l.jsonl", host=HOST)
        self.assertEqual(out[0], "ABORTED")
        self.assertIsInstance(S5.run_unit.terminated, KeyboardInterrupt)
        S5.run_unit.terminated = None


class TestLaunchRefusals(unittest.TestCase):
    HEAD = "a" * 40

    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp())
        self.appr = self.tmp / "execution_approval.json"
        self.write(scope=S5.SCOPE, commit=self.HEAD)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, **kw):
        a = {"approved_by": "user", "approved_at": "2026-11-09T10:00:00+09:00"}
        a.update(kw)
        self.appr.write_text(json.dumps(a))

    def problems(self, at=None, gs=None, environ=None, host=HOST, free=10 * 1024 ** 3):
        return S5.launch_problems(at or FakeAT(), gs or {"head": self.HEAD, "dirty": ""}, self.appr,
                                  self.tmp / "attempt01", [], env_report=lambda: ({}, []),
                                  environ={"HOME": "/x"} if environ is None else environ, host=host,
                                  disk_free=lambda path: free)

    def test_idle_host_and_free_disk(self):                                # v2 review r1 m2, m3
        rustdesk = lambda: {"gpu_compute_apps": ["4321, /usr/share/rustdesk/rustdesk, 52 MiB"]}   # noqa: E731
        self.assertEqual(self.problems(host=rustdesk), [])
        self.assertEqual(self.problems(free=S5.MIN_FREE_BYTES), [])
        gpu = lambda: {"gpu_compute_apps": ["4321, /usr/share/rustdesk/rustdesk, 52 MiB",            # noqa: E731
                                            "999, /home/fair/miniconda3/envs/aerialgym/bin/python, 3000 MiB"]}
        self.assertTrue(any("host not idle" in x and "aerialgym" in x for x in self.problems(host=gpu)))
        unknown = lambda: {"gpu_compute_apps": "unavailable: no nvidia-smi"}                       # noqa: E731
        self.assertTrue(any("GPU compute processes unknown" in x for x in self.problems(host=unknown)))
        low = self.problems(free=S5.MIN_FREE_BYTES - 1)
        self.assertEqual(len(low), 1)
        self.assertIn("free disk", low[0])
        self.assertGreater(S5.free_bytes(self.tmp / "no" / "such" / "dir"), 0)   # nearest existing ancestor

    def test_clean_passes_and_each_refusal(self):
        self.assertEqual(self.problems(), [])
        cases = [(dict(gs={"head": self.HEAD, "dirty": "M x"}), "not clean"),
                 (dict(gs={"head": "b" * 40, "dirty": ""}), "commit"),
                 (dict(at=FakeAT(source_p=["untracked source docs/plans/x.md"])), "untracked"),
                 (dict(at=FakeAT(idle=False)), "not idle"),
                 (dict(environ={"NAVRL_X": "1"}), "NAVRL_X")]
        for kw, needle in cases:
            p = self.problems(**kw)
            self.assertTrue(any(needle in x for x in p), (needle, p))
        self.write(scope="another scope", commit=self.HEAD)
        self.assertTrue(any("scope" in x for x in self.problems()))
        self.appr.unlink()
        self.assertTrue(any("no approval" in x for x in self.problems()))
        self.write(scope=S5.SCOPE, commit=self.HEAD)
        self.assertEqual(self.problems(), [])
        (self.tmp / "attempt01").mkdir()
        self.assertTrue(any("single use" in x for x in self.problems()))

    def test_scope_names_the_unit(self):
        for word in ("S5 safety v2", "navrl_quad", "N0 OBL GFH GFV OVJ OVR RCL DLL KIL FDS", "N0 N0R N0J",
                     "no retries", "SITL only"):
            self.assertIn(word, S5.SCOPE)
        self.assertIn(S5.PREREG, S5.OWN_SOURCES)

    def test_amendment2_binds_scope_smoke03_and_refuses_previous_approvals(self):
        text = (ROOT / S5.AMENDMENT).read_text()
        amend1 = S5.SCOPE.replace("amendment2:", "amendment1:").replace("smoke03", "smoke02")
        v2 = S5.SCOPE.replace("S5 safety v2 amendment2:", "S5 safety v2:").replace("smoke03", "smoke01")
        self.assertIn(S5.SCOPE, text)
        self.assertIn(amend1, (ROOT / S5.AMENDMENT1).read_text())
        self.assertIn(v2, (ROOT / S5.AMENDMENT1).read_text())
        self.assertIn(S5.AMENDMENT1, S5.OWN_SOURCES)
        self.assertIn(S5.AMENDMENT, S5.OWN_SOURCES)
        self.assertEqual((S5.SMOKE_DIR.name, S5.ATTEMPT_DIR.name), ("smoke03", "attempt01"))
        for old in (v2, amend1):
            self.write(scope=old, commit=self.HEAD)
            self.assertTrue(any("scope" in x for x in self.problems()))
        self.write(scope=S5.SCOPE, commit=self.HEAD)
        self.assertEqual(self.problems(), [])

    def test_smoke_gate(self):
        d = self.tmp / "smoke01"
        d.mkdir()
        src = {"tools/sitl_s5_safety.py": "s"}
        runner = TestLivePath()
        runner.setUp()
        self.addCleanup(runner.tearDown)
        rows = []
        for i, sid in enumerate(S5.SMOKE_SCHEDULE):
            res, ep = runner.run_one(sid)
            shutil.copytree(ep, d / ("ep%03d" % i))
            rows.append(dict(res, index=i))

        def write(rows, status="COMPLETE", head=self.HEAD, amendment=S5.AMENDMENT):
            (d / "attempt.json").write_text(json.dumps({"git": {"head": head}, "sources": src,
                "protocol": S5.PROTOCOL, "scope": S5.SCOPE, "prereg": S5.PREREG, "amendment": amendment,
                "schedule": S5.smoke_schedule(), "scenarios": list(S5.scenarios()), "kernel_faults_before": 0}))
            (d / "final.json").write_text(json.dumps({"status": status, "kernel_faults_after": 0,
                "git_after": {"head": head, "dirty": ""}}))
            (d / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
        write(rows)
        self.assertEqual(S5.smoke_problems(d, self.HEAD, src), [])
        self.assertTrue(S5.smoke_problems(d, "b" * 40, src))
        self.assertTrue(S5.smoke_problems(d, self.HEAD, {"tools/sitl_s5_safety.py": "t"}))
        for bad in ([dict(rows[0]), dict(rows[1], valid=False), dict(rows[2])], rows[:2],
                    [dict(rows[0]), dict(rows[1]), dict(rows[2], passed=False)]):
            write(bad)
            self.assertTrue(S5.smoke_problems(d, self.HEAD, src), bad)
        write(rows, status="ABORTED")
        self.assertTrue(S5.smoke_problems(d, self.HEAD, src))
        self.assertTrue(S5.smoke_problems(self.tmp / "missing", self.HEAD, src))
        write(rows, amendment=None)                                      # amendment 1: a v2-bound smoke is refused
        self.assertTrue(any("amendment" in x for x in S5.smoke_problems(d, self.HEAD, src)))
        write(rows)
        self.assertEqual(S5.smoke_problems(d, self.HEAD, src), [])


class TestExitStatus(unittest.TestCase):
    """Amendment 1: smoke01 ABORTED and exited 0, so the outer chain went on. --smoke and --launch now exit 0 only for a
    COMPLETE run (and a smoke the attempt's gate accepts); a refusal stays SystemExit("refused: ...") -> status 1."""

    def test_exit_code_rule(self):
        self.assertEqual(S5.exit_code({"status": "COMPLETE"}), 0)
        for final in ({"status": "ABORTED"}, {"status": "STOPPED_AFTER_A"}, {}, None):
            self.assertEqual(S5.exit_code(final), 3, final)
        self.assertEqual(S5.exit_code({"status": "COMPLETE"}, ["smoke gate problem"]), 4)
        self.assertEqual(S5.exit_code({"status": "ABORTED"}, ["smoke gate problem"]), 3)

    def run_main(self, argv, status, gate=(), refuse=()):
        """S5.main through the real launch(): its refusals, run_unit and the smoke gate are replaced; final.json is
        written by the real write_final."""
        at = NS(git_state=lambda root: {"head": "h" * 40, "branch": "b", "dirty": ""}, kernel_faults=lambda: 0,
                idle_check=lambda: (True, {"cores": "idle"}), process_environment=lambda: {"fake": True},
                environment_problems=lambda env: [])
        saved = signal.getsignal(signal.SIGTERM)
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            (t / "execution_approval.json").write_text("{}")
            patches = [mock.patch.object(S5, "UNIT_DIR", t), mock.patch.object(S5, "SMOKE_DIR", t / "smoke02"),
                       mock.patch.object(S5, "ATTEMPT_DIR", t / "attempt01"),
                       mock.patch.object(S5, "launch_problems", lambda *a, **k: list(refuse)),
                       mock.patch.object(S5, "smoke_problems", lambda *a, **k: list(gate)),
                       mock.patch.object(S5, "run_unit", lambda *a, **k: (status, "reason", 0)),
                       mock.patch.object(S5, "host_snapshot", lambda: {}),
                       mock.patch.dict(sys.modules, {"sitl_dev08_attempt": at, "sitl_owned_session": NS(),
                                                     "sitl_s3b_closed_loop": NS(PARAMS={})})]
            try:
                with contextlib.ExitStack() as stack, contextlib.redirect_stdout(io.StringIO()):
                    for p in patches:
                        stack.enter_context(p)
                    rc = S5.main(argv)
                out = t / ("smoke02" if argv == ["--smoke"] else "attempt01")
                final = json.loads((out / "final.json").read_text())
                return rc, final["status"] if final["status"] != "ABORTED" or status == "ABORTED" else final["reason"]
            finally:
                signal.signal(signal.SIGTERM, saved)

    def test_smoke_and_launch_exit_status(self):
        self.assertEqual(self.run_main(["--smoke"], "ABORTED"), (3, "ABORTED"))        # smoke01's case
        self.assertEqual(self.run_main(["--smoke"], "COMPLETE", gate=["N0R INVALID"]), (4, "COMPLETE"))
        self.assertEqual(self.run_main(["--smoke"], "COMPLETE"), (0, "COMPLETE"))     # positive control
        self.assertEqual(self.run_main(["--launch"], "ABORTED"), (3, "ABORTED"))
        self.assertEqual(self.run_main(["--launch"], "COMPLETE"), (0, "COMPLETE"))
        for kw in (dict(refuse=["no approval file"]), dict(gate=["no development smoke record"])):
            with self.assertRaises(SystemExit) as cm:                    # --launch: the smoke gate refuses first
                self.run_main(["--launch"], "COMPLETE", **kw)
            self.assertTrue(str(cm.exception.code).startswith("refused:"))  # a str code exits with status 1
        self.assertIn("sys.exit(main())", (TOOLS / "sitl_s5_safety.py").read_text())


class TestReading(unittest.TestCase):
    def rows(self, passed):
        return [{"scenario": s, "index": i, "outcome": "NOT_RUN" if p is None else "COMPLETED", "valid": p is not None,
                 "passed": p, "fail_reason": None if p in (True, None) else "x", "invalid_reason": None}
                for i, (s, p) in enumerate(zip(S5.SCHEDULE, passed))]

    def read(self, passed, status="COMPLETE", faults_after=0, smoke=False):
        attempt = {"schedule": S5.schedule(), "kernel_faults_before": 0, "git": {"head": "h"},
                   "development_smoke": smoke}
        final = {"status": status, "kernel_faults_after": faults_after, "git_after": {"head": "h", "dirty": ""}}
        return S5.reading(final, self.rows(passed), attempt)["reading"]

    def test_labels(self):
        self.assertEqual(self.read([True] * 10), "PASS_SITL")
        self.assertEqual(self.read([True] * 9 + [False]), "FAIL_SITL")
        self.assertEqual(self.read([True] * 9 + [None]), "INCOMPLETE")
        self.assertEqual(self.read([True] * 5, status="ABORTED"), "INCOMPLETE")
        self.assertEqual(self.read([True] * 10, faults_after=1), "VOID")
        self.assertEqual(self.read([True] * 10, smoke=True), "VOID")

    def test_analyze_refuses_to_overwrite(self):
        with tempfile.TemporaryDirectory() as t:
            d = pathlib.Path(t)
            (d / "attempt.json").write_text(json.dumps({"protocol": S5.PROTOCOL, "scope": S5.SCOPE, "prereg": S5.PREREG,
                     "amendment": S5.AMENDMENT, "schedule": S5.schedule(), "kernel_faults_before": 0,
                     "git": {"head": "h"}}))
            (d / "final.json").write_text(json.dumps({"status": "ABORTED", "kernel_faults_after": 0,
                                                      "git_after": {"head": "h", "dirty": ""}}))
            (d / "episodes.jsonl").write_text("")
            self.assertEqual(S5.analyze(d)["reading"], "INCOMPLETE")
            with self.assertRaises(FileExistsError):
                S5.analyze(d)


if __name__ == "__main__":
    unittest.main()
