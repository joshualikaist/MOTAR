"""CPU fixtures for the dev08 parameter and telemetry boundary (tools/sitl_dev08_link.py; motar-5md).

A fake PX4 parameter server and synthetic telemetry stand in for SITL; no socket, PX4 or GPU is used. The truth
stream is shaped as PX4 v1.14.3 sends it: HIL_STATE_QUATERNION.time_usec is 0. Receive times come from a fake
clock driven by the feed. Each rejection code has a mutant that triggers it and a valid fixture at the bound.
"""
import ast
import importlib.util
import json
import math
from pathlib import Path
import re
import struct
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
PX4 = Path("/home/fair/workspaces/px4_ws/PX4-Autopilot")


def load(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


L = load("sitl_dev08_link", "tools/sitl_dev08_link.py")
CFG = load("sitl_s3b_dev08_cfg", "tools/sitl_s3b_dev08.py")

AIRFRAME = """#!/bin/sh
. ${R}etc/init.d/rc.mc_defaults
param set-default MPC_XY_VEL_MAX 3.6
param set-default MPC_Z_VEL_MAX_UP 2.5
param set-default MPC_Z_VEL_MAX_DN 2.5
param set-default MPC_TILTMAX_AIR 45
param set-default MC_YAWRATE_MAX 172
param set-default MC_ROLLRATE_P 0.025
param set-default MC_ROLLRATE_I 0.033
param set-default MC_ROLLRATE_D 0.0005
param set-default MC_PITCHRATE_P 0.025
param set-default MC_PITCHRATE_I 0.033
param set-default MC_PITCHRATE_D 0.0005
param set-default MC_YAWRATE_P 0.11
param set-default MC_YAWRATE_I 0.055
param set-default MPC_THR_HOVER 0.31   # comment
param set-default EKF2_HGT_REF 2
"""
FIRMWARE = {"MC_ROLLRATE_P": 0.15, "MC_ROLLRATE_I": 0.2, "MC_ROLLRATE_D": 0.003, "MC_ROLLRATE_K": 1.0,
            "MC_ROLLRATE_FF": 0.0, "MC_ROLLRATE_MAX": 220.0, "MC_PITCHRATE_P": 0.15, "MC_PITCHRATE_I": 0.2,
            "MC_PITCHRATE_D": 0.003, "MC_PITCHRATE_K": 1.0, "MC_PITCHRATE_FF": 0.0, "MC_PITCHRATE_MAX": 220.0,
            "MC_YAWRATE_P": 0.2, "MC_YAWRATE_I": 0.1, "MC_YAWRATE_D": 0.0, "MC_YAWRATE_FF": 0.0, "MC_YAWRATE_K": 1.0,
            "MC_YAWRATE_MAX": 200.0, "MPC_XY_VEL_P_ACC": 1.8, "MPC_XY_VEL_I_ACC": 0.4, "MPC_XY_VEL_D_ACC": 0.2,
            "MPC_XY_VEL_MAX": 12.0, "MPC_Z_VEL_MAX_UP": 3.0, "MPC_Z_VEL_MAX_DN": 1.5, "MPC_TILTMAX_AIR": 45.0,
            "MPC_THR_HOVER": 0.5, "THR_MDL_FAC": 0.0}
PARAMS_JSON = json.dumps({"parameters": [{"name": n, "type": "Float", "default": v} for n, v in FIRMWARE.items()]
                         + [{"name": "EKF2_HGT_REF", "type": "Int32", "default": 1}]})
EXPECT = L.fresh_expectations(AIRFRAME, PARAMS_JSON)


def fresh_boot():
    return dict(EXPECT)


class FakePX4:
    """Answers PARAM_REQUEST_READ and PARAM_SET like PX4's parameter protocol, with optional faults."""

    def __init__(self, inbox, params, **faults):
        self.inbox, self.params, self.faults = inbox, dict(params), faults
        self.sent = []

    def reply(self, name, value, ptype=L.MAV_PARAM_TYPE_REAL32, src=(1, 1)):
        self.inbox.put({"type": "PARAM_VALUE", "src": self.faults.get("src", src),
                        "fields": {"param_id": name, "param_value": value, "param_type": ptype,
                                   "param_index": 0, "param_count": len(self.params)}})

    def param_read(self, name):
        self.sent.append(("read", name))
        if name in self.faults.get("drop_read", ()) or name not in self.params:
            return
        value = float("nan") if name in self.faults.get("nonfinite", ()) else self.params[name]
        ptype = 6 if name in self.faults.get("int_type", ()) else L.MAV_PARAM_TYPE_REAL32
        self.reply(name, value, ptype)

    def param_set(self, name, value):
        self.sent.append(("set", name, value))
        if name in self.faults.get("drop_echo", ()):
            return
        if name in self.faults.get("stale_echo", ()):
            self.reply(name, self.params[name])
            return
        if name in self.faults.get("ulp_echo", ()):                 # one float32 ulp off
            bits = struct.unpack("<I", struct.pack("<f", value))[0] + 1
            self.reply(name, struct.unpack("<f", struct.pack("<I", bits))[0])
            return
        if name in self.faults.get("ignore_set", ()):
            self.reply(name, value)
            return
        self.params[name] = L.f32(value)
        for other, v in self.faults.get("side_effect", {}).get(name, {}).items():
            self.params[other] = L.f32(v)
        self.reply(name, self.params[name])


def arm_params(arm):
    return CFG.prepare(arm=arm)["selected_params"]


class Parameters(unittest.TestCase):
    def run_arm(self, arm, boot=None, **faults):
        inbox = L.Inbox()
        px4 = FakePX4(inbox, boot or fresh_boot(), **faults)
        rec = L.apply_and_verify(inbox, px4, arm_params(arm), fresh_defaults=EXPECT, timeout=0.2)
        return rec, px4

    def test_native_and_candidate_records(self):
        native, px4 = self.run_arm("native")
        self.assertEqual(native["booted"]["MC_YAWRATE_P"], L.f32(0.11))       # fresh native yaw, read not assumed
        self.assertEqual(native["booted"]["MC_YAWRATE_I"], L.f32(0.055))
        for n, v in arm_params("native").items():
            self.assertEqual(native["effective"][n], L.f32(v))
        self.assertEqual(sorted(native["fresh_checked"]), sorted(L.RECORD_PARAMS))
        self.assertEqual([s[1] for s in px4.sent if s[0] == "set"], list(arm_params("native")))
        candidate, _ = self.run_arm("candidate")
        delta = {n for n in native["effective"] if native["effective"][n] != candidate["effective"][n]}
        self.assertEqual(delta, set(CFG.CANDIDATE))
        self.assertEqual(native["booted"], candidate["booted"])

    def test_recorded_set_covers_both_arms(self):
        self.assertTrue(set(arm_params("candidate")) <= set(L.RECORD_PARAMS))
        self.assertTrue(set(arm_params("native")) <= set(L.RECORD_PARAMS))
        self.assertTrue({"MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D", "MC_YAWRATE_FF", "MC_YAWRATE_K",
                         "MC_ROLLRATE_K", "MC_PITCHRATE_K"} <= set(L.RECORD_PARAMS))
        with self.assertRaises(ValueError):                         # every recorded name needs an expectation
            L.apply_and_verify(L.Inbox(), FakePX4(L.Inbox(), fresh_boot()), {}, fresh_defaults={}, timeout=0.1)

    def test_stale_parameter_file_is_detected(self):
        cases = {("native", "MC_ROLLRATE_P"): 0.013,     # the previous episode's override (both arms set it)
                 ("native", "MC_YAWRATE_P"): 0.2,        # a stale yaw gain the native arm does not set
                 ("native", "MC_YAWRATE_FF"): 0.5}       # a firmware-default parameter
        for (arm, name), value in cases.items():
            boot = fresh_boot()
            boot[name] = L.f32(value)
            with self.assertRaises(L.ParamError, msg=name) as cm:
                self.run_arm(arm, boot=boot)
            self.assertEqual(cm.exception.code, "PARAM_NOT_FRESH")

    def test_rejection_codes(self):
        cases = {
            "PARAM_MISSING": dict(drop_read=("MC_YAWRATE_K",)),
            "PARAM_ECHO_MISSING": dict(drop_echo=("MC_YAWRATE_P",)),
            "PARAM_ECHO_MISMATCH": dict(stale_echo=("MC_YAWRATE_P",)),
            "PARAM_READBACK_MISMATCH": dict(ignore_set=("MPC_XY_VEL_P_ACC",)),
            "PARAM_DRIFT": dict(side_effect={"MC_YAWRATE_P": {"MC_YAWRATE_FF": 0.5}}),
            "PARAM_NONFINITE": dict(nonfinite=("MC_YAWRATE_D",)),
            "PARAM_TYPE": dict(int_type=("THR_MDL_FAC",)),
        }
        for code, faults in cases.items():
            with self.assertRaises(L.ParamError, msg=code) as cm:
                self.run_arm("candidate", **faults)
            self.assertEqual(cm.exception.code, code)
        with self.assertRaises(L.ParamError) as cm:                  # exactly that float32: one ulp is a mismatch
            self.run_arm("candidate", ulp_echo=("MC_YAWRATE_P",))
        self.assertEqual(cm.exception.code, "PARAM_ECHO_MISMATCH")

    def test_only_later_autopilot_replies_count(self):
        inbox = L.Inbox()
        px4 = FakePX4(inbox, fresh_boot(), drop_read=("MC_YAWRATE_P",))
        px4.reply("MC_YAWRATE_P", L.f32(0.11))               # an earlier PARAM_VALUE, before the request
        with self.assertRaises(L.ParamError) as cm:
            L.read_param(inbox, px4, "MC_YAWRATE_P", timeout=0.1)
        self.assertEqual(cm.exception.code, "PARAM_MISSING")
        foreign = FakePX4(inbox, fresh_boot(), src=(2, 1))
        with self.assertRaises(L.ParamError):
            L.read_param(inbox, foreign, "MC_YAWRATE_P", timeout=0.1)
        ok = FakePX4(inbox, fresh_boot())                      # positive control
        self.assertEqual(L.read_param(inbox, ok, "MC_YAWRATE_P", timeout=0.1)["value"], L.f32(0.11))

    def test_fresh_check_is_exact_and_parser_takes_the_last_line(self):
        boot = fresh_boot()
        bits = struct.unpack("<I", struct.pack("<f", boot["MC_YAWRATE_K"]))[0] + 1
        boot["MC_YAWRATE_K"] = struct.unpack("<f", struct.pack("<I", bits))[0]    # one ulp off
        with self.assertRaises(L.ParamError) as cm:
            self.run_arm("native", boot=boot)
        self.assertEqual(cm.exception.code, "PARAM_NOT_FRESH")
        d = L.airframe_defaults("param set-default MC_YAWRATE_P 0.1\nparam set-default MC_YAWRATE_P 0.11\n")
        self.assertEqual(d["MC_YAWRATE_P"], L.f32(0.11))

    def test_receiver_failure_during_a_wait(self):
        inbox = L.Inbox()
        threading.Timer(0.1, lambda: inbox.fail(OSError(9, "Bad file descriptor"))).start()
        t0 = time.monotonic()
        with self.assertRaises(L.ParamError) as cm:
            L.read_param(inbox, FakePX4(inbox, fresh_boot(), drop_read=("MC_YAWRATE_P",)), "MC_YAWRATE_P",
                         timeout=2.0)
        self.assertEqual(cm.exception.code, "RX_ERROR")
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_receiver_failure_is_reported(self):
        inbox = L.Inbox()
        inbox.fail(OSError(9, "Bad file descriptor"))
        with self.assertRaises(L.ParamError) as cm:
            L.read_param(inbox, FakePX4(inbox, fresh_boot(), drop_read=("MC_YAWRATE_P",)), "MC_YAWRATE_P",
                         timeout=1.0)
        self.assertEqual(cm.exception.code, "RX_ERROR")

    def test_airframe_parser_is_strict(self):
        d = L.airframe_defaults(AIRFRAME)
        self.assertEqual(d["MPC_THR_HOVER"], L.f32(0.31))
        self.assertEqual(d["EKF2_HGT_REF"], 2.0)
        for bad in ("param set-default MC_YAWRATE_P 0.11#x", "param set-default MC_YAWRATE_P ${V}",
                    "param set-default MC_YAWRATE_P"):
            with self.assertRaises(ValueError, msg=bad):
                L.airframe_defaults(AIRFRAME + bad + "\n")

    def test_firmware_defaults_need_every_name_as_float(self):
        self.assertEqual(EXPECT["MC_YAWRATE_P"], L.f32(0.11))       # airframe wins
        self.assertEqual(EXPECT["MC_YAWRATE_K"], L.f32(1.0))        # firmware otherwise
        with self.assertRaises(ValueError):
            L.firmware_defaults(PARAMS_JSON, ("MC_YAWRATE_P", "NOT_A_PARAM"))
        with self.assertRaises(ValueError):
            L.firmware_defaults(PARAMS_JSON, ("EKF2_HGT_REF",))

    def test_installed_airframe_and_parameters_json(self):
        airframe = PX4 / "build/px4_sitl_default/etc/init.d-posix/airframes/10090_gazebo-classic_navrl_quad"
        pj = PX4 / "build/px4_sitl_default/parameters.json"
        if not (airframe.is_file() and pj.is_file()):
            self.skipTest("PX4 tree not on this host")
        real = L.fresh_expectations(airframe.read_text(), pj.read_text())    # parses fully; every name REAL32
        self.assertEqual((real["MC_YAWRATE_P"], real["MC_YAWRATE_I"]), (L.f32(0.11), L.f32(0.055)))
        self.assertEqual(real["MC_ROLLRATE_P"], L.f32(0.025))
        self.assertNotEqual(real["MC_ROLLRATE_P"], L.f32(arm_params("native")["MC_ROLLRATE_P"]))  # check can bite
        self.assertEqual({n: real[n] for n in ("MC_YAWRATE_K", "MC_ROLLRATE_K", "MPC_XY_VEL_P_ACC")},
                         {"MC_YAWRATE_K": 1.0, "MC_ROLLRATE_K": 1.0, "MPC_XY_VEL_P_ACC": L.f32(1.8)})


class Feed:
    """Synthetic PX4 streams at 50 Hz. Position and attitude carry PX4's clock; truth carries time_usec 0, as
    PX4 v1.14.3 sends it. Receive times (the inbox's fake clock) follow simulator time at real-time factor 1."""

    def __init__(self, rate=50.0):
        self.now = 0.0
        self.inbox = L.Inbox(clock=lambda: self.now)
        self.rate = rate
        self.t = 0.0
        self.skip = []           # [(t0, t1)] position gap
        self.silent = set()      # streams that stop
        self.patch = {}          # (kind, index) -> fields update
        self.stop = {}           # kind -> only samples with t < stop are sent
        self.truth_recv = 0.0002  # truth received this long after its position sample (same burst)
        self.i = 0

    def put(self, kind, t, recv=None, src=(1, 1), **extra):
        if kind == "pos":
            f = {"time_boot_ms": int(round(t * 1000)), "x": 1.0, "y": 2.0, "z": -1.0, "vx": 0.1, "vy": 0.0, "vz": 0.0}
            typ, r = "LOCAL_POSITION_NED", t
        elif kind == "att":
            f = {"time_boot_ms": int(round(t * 1000)), "roll": 0.0, "pitch": 0.0, "yaw": 0.3,
                 "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}
            typ, r = "ATTITUDE", t + 0.0001
        else:
            f = {"time_usec": 0, "attitude_quaternion": [1.0, 0.0, 0.0, 0.0],
                 "lat": 473977420, "lon": 85455940, "alt": 488000, "vx": 0, "vy": 0, "vz": 0}
            typ, r = "HIL_STATE_QUATERNION", t + self.truth_recv
        f.update(extra)
        f.update(self.patch.get((kind, self.i), {}))
        self.now = r if recv is None else recv
        self.inbox.put({"type": typ, "src": src, "fields": f})

    def until(self, t_end):
        while self.t <= t_end + 1e-9:
            if not any(a <= self.t < b for a, b in self.skip) and "pos" not in self.silent:
                self.put("pos", self.t)
            for kind in ("att", "truth"):
                if kind not in self.silent and self.t < self.stop.get(kind, float("inf")) - 1e-9:
                    self.put(kind, self.t)
            self.i += 1
            self.t = round(self.t + 1.0 / self.rate, 9)


class Telemetry(unittest.TestCase):
    def setUp(self):
        self.make()

    def make(self, **gate):
        self.feed = Feed()
        self.inbox = self.feed.inbox
        self.gate = L.TelemetryGate(self.inbox, **dict({"stall_wall_s": 0.2}, **gate))

    def aligned(self):
        self.feed.until(1.0)
        return self.gate.align()

    def step(self, k, ahead=0.02):
        self.feed.until(self.gate.start + k * L.DT + ahead)          # one later sample is already in
        return self.gate.checkpoint(k)

    def expect(self, code, fn):
        with self.assertRaises(L.TelemetryError) as cm:
            fn()
        self.assertEqual(cm.exception.code, code)
        return cm.exception

    def test_valid_streams_pass_every_check(self):
        a = self.aligned()
        self.assertIn("receipt", a["truth_time_model"])
        snaps = [self.step(k) for k in range(41)]
        self.assertEqual([s["k"] for s in snaps], list(range(41)))
        for s in snaps:                     # 0, or one sample where start + k*DT rounds above a sample time
            self.assertLessEqual(s["pos_lateness_s"], 0.0201)
            self.assertLessEqual(abs(s["att_skew_s"]), 1e-6)
            self.assertAlmostEqual(s["truth_recv_skew_s"], 0.0002, places=6)
            self.assertEqual(s["truth"]["fields"]["time_usec"], 0)
        self.assertEqual(len({s["truth"]["seq"] for s in snaps}), 41)

    def test_truth_is_the_same_burst_sample(self):
        """The truth sample received after the selected position and before the next one (receipt order)."""
        self.aligned()
        for k in range(5):
            s = self.step(k, ahead=0.04)
            self.assertEqual(s["truth"]["seq"], s["pos"]["seq"] + 2)       # pos, att, truth of one burst

    def test_historical_float_comparison(self):
        """Checkpoint k takes the first position with not (t < start + k*DT), the historical loop's comparison;
        where rounding puts start + k*DT just above a sample time, that is the next sample."""
        self.feed.t = 35.5
        self.feed.until(35.5)
        self.gate.align()
        start = self.gate.start
        self.assertEqual(start, 35.5)
        hit = next((k for k in range(1, 600) if (1775 + 5 * k) * 20 * 1e-3 < start + k * L.DT), None)
        self.assertEqual(hit, 164)                                       # as computed for the identity harness
        for j in range(hit + 1):
            s = self.step(j, ahead=0.04)
        self.assertAlmostEqual(s["pos_lateness_s"], 0.02, places=6)    # the next 50 Hz sample, as historically
        self.assertEqual(s["pos"]["fields"]["time_boot_ms"], (1775 + 5 * hit + 1) * 20)

    def test_nonfinite_and_bad_quaternion(self):
        self.aligned()
        self.feed.patch[("pos", self.feed.i + 3)] = {"vx": float("nan")}
        self.expect("NONFINITE", lambda: [self.step(k) for k in range(5)])
        self.make()
        self.feed.patch[("truth", 48)] = {"attitude_quaternion": [0.9, 0.0, 0.0, 0.0]}
        self.feed.until(1.0)
        self.expect("BAD_QUATERNION", self.gate.align)

    def test_clock_not_increasing(self):
        self.aligned()
        self.feed.put("pos", self.feed.t - 0.5)
        self.expect("CLOCK_NOT_INCREASING", lambda: [self.step(k) for k in range(3)])
        self.make()
        self.aligned()
        last = self.gate.samples["pos"][-1]["t"]
        self.feed.put("pos", last)                                        # a duplicated time stamp
        self.expect("CLOCK_NOT_INCREASING", lambda: self.step(0))

    def test_skip(self):
        self.aligned()
        a = self.gate.start
        for k in range(3):
            self.step(k)
        self.feed.skip.append((a + 0.29, a + 0.37))                         # positions missing around checkpoint 3
        e = self.expect("CLOCK_SKIP", lambda: self.step(3, ahead=0.08))
        self.assertIn("positions_near", str(e))                           # diagnostics in the error

    def test_stall(self):
        self.aligned()
        self.step(0)
        t0 = time.monotonic()
        self.expect("CLOCK_STALL", lambda: self.gate.checkpoint(1))      # nothing more arrives
        self.assertLess(time.monotonic() - t0, 2.0)

    def test_runner_late(self):
        self.aligned()
        for k in range(4):
            self.step(k)
        self.expect("RUNNER_LATE", lambda: self.step(4, ahead=0.2))       # the runner arrives 0.2 s late
        self.make()
        self.aligned()
        for k in range(4):
            self.step(k)
        self.step(4, ahead=0.04)                                          # positive: 0.04 s is inside the bound

    def age_case(self, kind, last_age, code=None):
        """Steps 0..2, then the kind's stream stops so that at checkpoint 3 its newest sample is last_age old."""
        self.make()
        self.aligned()
        for k in range(3):
            self.step(k)
        t3 = self.gate.start + 3 * L.DT
        self.feed.stop[kind] = t3 - last_age + 0.01                       # newest sent sample: t3 - last_age
        if code:
            self.expect(code, lambda: self.step(3, ahead=0.0))
            return None
        return self.step(3, ahead=0.0)

    def test_attitude_freshness_bound(self):
        self.assertAlmostEqual(self.age_case("att", 0.04)["att_skew_s"], -0.04, places=6)   # inside 0.05
        self.age_case("att", 0.08, "STALE_ATT")                            # outside 0.05, inside twice it

    def test_truth_age_bound(self):
        s = self.age_case("truth", 0.02)                                   # inside 0.03
        self.assertAlmostEqual(s["truth_recv_skew_s"], -0.02 + 0.0002, places=6)
        self.age_case("truth", 0.04, "STALE_TRUTH")                        # outside 0.03, inside twice it

    def test_attitude_freshness_at_alignment(self):
        self.feed.stop["att"] = 1.0 - 0.04 + 0.01                          # newest attitude 0.04 s old: accepted
        self.feed.until(1.0)
        self.gate.align()
        self.make()
        self.feed.stop["att"] = 1.0 - 0.08 + 0.01                          # 0.08 s old: rejected
        self.feed.until(1.0)
        self.expect("STALE_ATT", self.gate.align)

    def test_burst_window_bound(self):
        """A truth received up to BURST_S after the position counts; one received later (the next PX4 pass) does
        not."""
        for lag, same in ((0.002, True), (0.004, False)):
            self.make()
            self.feed.truth_recv = lag
            self.aligned()
            s = self.step(0, ahead=0.0)
            self.assertEqual(s["truth"]["seq"] == s["pos"]["seq"] + 2, same, lag)

    def test_truth_must_arrive_before_the_next_position(self):
        """Review r3 N10: a batched next pass (position +0.5 ms, its truth +1.0 ms) is inside the 3 ms window; the
        order bound keeps the selected position's own truth (seq + 2), not the next pass's (seq + 5)."""
        self.make(stall_wall_s=2.0)
        f = self.feed
        f.until(1.0)                                    # ends with pos, att, truth of t = 1.0
        f.put("pos", 1.02, recv=1.0005)                 # the next pass arrives batched
        f.put("att", 1.02, recv=1.0006)
        f.put("truth", 1.02, recv=1.0010)
        self.gate.align()
        self.assertEqual(self.gate.start, 1.02)          # alignment takes the newest position
        g = L.TelemetryGate(self.inbox, stall_wall_s=2.0)
        g.pump()
        g.start = 1.0                                   # a checkpoint at the earlier pass
        s = g.checkpoint(0)
        self.assertEqual(s["pos"]["fields"]["time_boot_ms"], 1000)
        self.assertEqual(s["truth"]["seq"], s["pos"]["seq"] + 2)

    def test_settle_waits_for_a_late_truth_fake_clock(self):
        """Review r3 N9/N11 on the fake clock: the same-pass truth arrives 10 ms of wall time after the gate starts
        waiting, at checkpoint and at alignment."""
        self.make(stall_wall_s=2.0)
        self.aligned()
        for k in range(2):
            self.step(k)
        f = self.feed
        t2 = round(self.gate.start + 2 * L.DT, 9)
        f.stop["truth"] = t2                      # the feed sends position/attitude up to t2, not that truth
        f.until(t2)
        f.stop.pop("truth")
        late = threading.Timer(0.01, lambda: f.put("truth", t2, recv=t2 + 0.0002))
        late.start()
        s = self.gate.checkpoint(2)
        late.join()
        self.assertAlmostEqual(s["truth_recv_skew_s"], 0.0002, places=6)  # the late same-pass truth, not t2-0.02
        self.make(stall_wall_s=2.0)
        f = self.feed
        f.stop["truth"] = 0.5                     # truth paused for 0.5 s before alignment
        f.until(1.0)
        late = threading.Timer(0.01, lambda: f.put("truth", 1.0, recv=1.0002))
        late.start()
        a = self.gate.align()                     # without the settle wait this is STALE_TRUTH
        late.join()
        self.assertEqual(a["start_sim_s"], 1.0)

    def test_same_burst_truth_is_awaited(self):
        """Real clock: the truth is sent 2 ms after its position; the gate waits for it (BURST_S). The burst bound is
        widened to 20 ms here so that host scheduling under the full suite cannot push the truth past it; the
        production bound and its edges are pinned by test_burst_window_bound."""
        inbox = L.Inbox()
        gate = L.TelemetryGate(inbox, skip_max_s=0.3, stall_wall_s=2.0, burst_s=0.02)
        stop = threading.Event()
        t0 = time.monotonic()

        def produce():
            n = 0
            while not stop.is_set():
                if time.monotonic() - t0 < n * 0.02:
                    time.sleep(0.0005)
                    continue
                inbox.put({"type": "LOCAL_POSITION_NED", "src": (1, 1), "fields": {
                    "time_boot_ms": 20 * n, "x": 0.0, "y": 0.0, "z": -1.0, "vx": 0.0, "vy": 0.0, "vz": 0.0}})
                inbox.put({"type": "ATTITUDE", "src": (1, 1), "fields": {
                    "time_boot_ms": 20 * n, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rollspeed": 0.0,
                    "pitchspeed": 0.0, "yawspeed": 0.0}})
                time.sleep(0.002)
                inbox.put({"type": "HIL_STATE_QUATERNION", "src": (1, 1), "fields": {
                    "time_usec": 0, "attitude_quaternion": [1.0, 0.0, 0.0, 0.0], "lat": 0, "lon": 0, "alt": 0,
                    "vx": 0, "vy": 0, "vz": 0}})
                n += 1

        th = threading.Thread(target=produce, daemon=True)
        th.start()
        try:
            time.sleep(0.3)
            gate.align()
            skews = [gate.checkpoint(k)["truth_recv_skew_s"] for k in range(10)]
        finally:
            stop.set()
            th.join()
        self.assertGreaterEqual(sum(s > 0 for s in skews), 8, skews)    # the same-burst sample, not the previous

    def test_nonfinite_truth_quaternion(self):
        self.feed.patch[("truth", 48)] = {"attitude_quaternion": [float("nan"), 0.0, 0.0, 0.0]}
        self.feed.until(1.0)
        self.expect("NONFINITE", self.gate.align)

    def test_truth_pairing_at_alignment(self):
        self.make(stall_wall_s=2.0)                                       # so the pairing check is the one hit
        self.feed.silent.add("truth")
        self.feed.until(1.0)
        self.feed.silent.discard("truth")
        self.feed.put("truth", 0.6, recv=0.6)                              # received 0.4 s before the position
        self.feed.put("pos", 1.02)
        self.feed.put("att", 1.02)
        self.expect("STALE_TRUTH", self.gate.align)

    def test_stale_companions(self):
        self.aligned()
        for k in range(3):
            self.step(k)
        self.feed.silent.add("att")
        self.expect("STALE_ATT", lambda: [self.step(k) for k in range(3, 6)])
        self.make()
        self.aligned()
        for k in range(3):
            self.step(k)
        self.feed.silent.add("truth")
        self.expect("STALE_TRUTH", lambda: [self.step(k) for k in range(3, 6)])

    def test_truth_sample_may_not_be_reused(self):
        self.make(truth_age_s=0.2)              # wide enough that the last truth sample is near two checkpoints
        self.aligned()
        self.feed.silent.add("truth")
        self.step(0)
        self.expect("STALE_TRUTH", lambda: self.step(1))

    def test_foreign_source_is_ignored(self):
        self.aligned()
        self.feed.put("pos", self.gate.start + 0.01, src=(2, 1), vx=float("nan"))   # ignored, not fatal
        for k in range(4):
            self.step(k)
        self.assertGreaterEqual(self.gate.ignored["foreign_source"], 1)
        self.make()
        for i in range(60):
            self.feed.put("pos", i * 0.02, src=(2, 1))
            self.feed.put("att", i * 0.02)
            self.feed.put("truth", i * 0.02)
        self.expect("STREAM_MISSING", self.gate.align)

    def test_alignment_and_order_rules(self):
        self.expect("NOT_ALIGNED", lambda: self.gate.checkpoint(0))
        feed = Feed()
        feed.until(1.0)
        feed.now += 10.0                                                  # nothing received for 10 s
        self.expect("STREAM_STALE", L.TelemetryGate(feed.inbox, stall_wall_s=2.0).align)
        self.aligned()
        self.expect("ALIGN_TWICE", self.gate.align)
        self.make()
        self.aligned()
        self.feed.until(self.gate.start + 0.5)
        self.expect("CHECKPOINT_ORDER", lambda: self.gate.checkpoint(1))  # the first checkpoint is 0
        self.make()
        self.aligned()
        self.step(0)
        self.step(1)
        self.expect("CHECKPOINT_ORDER", lambda: self.step(3))

    def test_start_cursor_bounds_what_is_judged(self):
        self.feed.until(0.5)
        self.feed.put("pos", 0.51, vx=float("nan"))                       # a boot-time glitch
        self.feed.t = 0.52
        self.feed.until(2.0)
        self.expect("NONFINITE", L.TelemetryGate(self.inbox, stall_wall_s=0.2).align)    # judged from 0
        gate = L.TelemetryGate(self.inbox, start_cursor=self.inbox.cursor_since(self.feed.now - 1.0),
                               stall_wall_s=0.2)
        gate.align()                                                      # positive: the last second is clean

    def test_receiver_failure(self):
        self.aligned()
        self.inbox.fail(RuntimeError("socket closed"))
        self.expect("RX_ERROR", lambda: self.gate.checkpoint(0))

    def test_errors_latch(self):
        self.aligned()
        self.feed.put("pos", 0.0)
        self.expect("CLOCK_NOT_INCREASING", self.gate.pump)
        self.feed.until(self.feed.t + 0.5)
        self.expect("CLOCK_NOT_INCREASING", lambda: self.gate.checkpoint(0))

    def test_live_feed_thread(self):
        """Positive control with a producer thread at real time and the real clock (the pymavlink rx shape).
        Timing bounds are widened: this checks the threaded hand-off, not host scheduling."""
        inbox = L.Inbox()
        gate = L.TelemetryGate(inbox, skip_max_s=0.3, stall_wall_s=2.0, truth_age_s=0.2)
        stop = threading.Event()
        t0 = time.monotonic()

        def produce():
            n = 0
            while not stop.is_set():
                if time.monotonic() - t0 < n * 0.02:
                    time.sleep(0.002)
                    continue
                inbox.put({"type": "LOCAL_POSITION_NED", "src": (1, 1), "fields": {
                    "time_boot_ms": 20 * n, "x": 0.0, "y": 0.0, "z": -1.0, "vx": 0.0, "vy": 0.0, "vz": 0.0}})
                inbox.put({"type": "ATTITUDE", "src": (1, 1), "fields": {
                    "time_boot_ms": 20 * n, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "rollspeed": 0.0,
                    "pitchspeed": 0.0, "yawspeed": 0.0}})
                inbox.put({"type": "HIL_STATE_QUATERNION", "src": (1, 1), "fields": {
                    "time_usec": 0, "attitude_quaternion": [1.0, 0.0, 0.0, 0.0], "lat": 0, "lon": 0, "alt": 0,
                    "vx": 0, "vy": 0, "vz": 0}})
                n += 1

        th = threading.Thread(target=produce, daemon=True)
        th.start()
        try:
            time.sleep(0.3)
            gate.align()
            snaps = [gate.checkpoint(k) for k in range(8)]
        finally:
            stop.set()
            th.join()
        self.assertEqual([s["k"] for s in snaps], list(range(8)))


class PX4Source(unittest.TestCase):
    def test_truth_message_has_no_clock_in_px4(self):
        """The gate's truth time model rests on this: PX4 v1.14.3 never assigns HIL_STATE_QUATERNION.time_usec,
        while LOCAL_POSITION_NED and ATTITUDE take time_boot_ms from the uORB timestamp."""
        streams = PX4 / "src/modules/mavlink/streams"
        if not streams.is_dir():
            self.skipTest("PX4 tree not on this host")
        truth = (streams / "HIL_STATE_QUATERNION.hpp").read_text()
        self.assertIn("mavlink_hil_state_quaternion_t msg{}", truth)
        self.assertIsNone(re.search(r"msg\.time_usec\s*=", truth))
        self.assertIn("msg.time_boot_ms = lpos.timestamp / 1000", (streams / "LOCAL_POSITION_NED.hpp").read_text())
        self.assertIn("msg.time_boot_ms = att.timestamp / 1000", (streams / "ATTITUDE.hpp").read_text())


class Adapter(unittest.TestCase):
    def test_pymavlink_transport_shapes(self):
        calls = []

        class Mav:
            def param_request_read_send(self, *a):
                calls.append(("read", a))

            def param_set_send(self, *a):
                calls.append(("set", a))

        class Msg:
            def __init__(self, typ, src, fields):
                self.typ, self.src, self.fields = typ, src, fields

            def get_type(self):
                return self.typ

            def get_srcSystem(self):
                return self.src[0]

            def get_srcComponent(self):
                return self.src[1]

            def to_dict(self):
                return dict(self.fields, mavpackettype=self.typ)

        msgs = [Msg("PARAM_VALUE", (1, 1), {"param_id": "MC_YAWRATE_P", "param_value": 0.11}),
                Msg("STATUSTEXT", (1, 1), {}), None]

        class Conn:
            mav = Mav()

            def recv_match(self, blocking, timeout):
                if msgs:
                    return msgs.pop(0)
                transport.stop.set()
                return None

        inbox = L.Inbox()
        transport = L.PymavlinkTransport(Conn(), inbox)
        transport.param_read("MC_YAWRATE_P")
        transport.param_set("MC_YAWRATE_P", 0.2)
        self.assertEqual(calls[0], ("read", (1, 1, b"MC_YAWRATE_P", -1)))
        self.assertEqual(calls[1], ("set", (1, 1, b"MC_YAWRATE_P", 0.2, L.MAV_PARAM_TYPE_REAL32)))
        transport.rx_loop()
        items, _ = inbox.since(0)
        self.assertEqual([m["type"] for m in items], ["PARAM_VALUE"])
        self.assertEqual(items[0]["src"], (1, 1))
        self.assertIsNone(inbox.error)

        class Broken:
            mav = Mav()

            def recv_match(self, blocking, timeout):
                raise OSError(9, "Bad file descriptor")

        inbox2 = L.Inbox()
        L.PymavlinkTransport(Broken(), inbox2).rx_loop()                 # returns; the failure is latched
        self.assertIn("Bad file descriptor", inbox2.error)


class SourceContract(unittest.TestCase):
    def test_import_starts_nothing_and_claims_nothing(self):
        tree = ast.parse((ROOT / "tools/sitl_dev08_link.py").read_text())
        for node in tree.body:
            self.assertIsInstance(node, (ast.Expr, ast.Import, ast.ImportFrom, ast.Assign, ast.FunctionDef,
                                         ast.ClassDef, ast.If))
        self.assertEqual({k: L.STATUS[k] for k in ("live_verified", "execution_supported", "launch_ready")},
                         {"live_verified": False, "execution_supported": False, "launch_ready": False})
        self.assertTrue(math.isclose(L.DT, 0.1))
        self.assertTrue(issubclass(L.ParamError, L.BoundaryAbort) and issubclass(L.TelemetryError, L.BoundaryAbort))


if __name__ == "__main__":
    unittest.main()
