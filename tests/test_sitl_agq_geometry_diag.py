"""CPU fixtures for the AGQ geometry diagnostic recorder and launcher (tools/sitl_agq_geometry_diag.py; motar-z5f).

No simulator runs: the owned session starts fake processes, PX4 is the dev08 tests' FakePX4Conn, and the Gazebo
contact subscriber is a small script printing Contacts lines in ShortDebugString form."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import threading
import time
import types
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load(name, rel):
    saved = list(sys.path)
    try:
        spec = importlib.util.spec_from_file_location(name, ROOT / rel)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path[:] = saved
    return mod


D = load("sitl_agq_geometry_diag_t", "tools/sitl_agq_geometry_diag.py")
PT = load("test_sitl_s3b_dev08_prefix_agq", "tests/test_sitl_s3b_dev08_prefix.py")   # FakePX4Conn, FAST, ports
R, OS_, LK, ST, LT, SI = PT.R, PT.OS_, PT.LK, PT.ST, PT.LT, PT.SI

BASE = "navrl_quad::base_link::base_link_inertia_collision"
ROTOR = "navrl_quad::rotor_2::rotor_2_collision"
FLOOR = "asphalt_plane::link::collision"


def pair(c1, c2, fz=3.0):
    return ('contact { collision1: "%s" collision2: "%s" position { x: 1 y: 1 z: 0.05 } normal { x: 0 y: 0 z: 1 } '
            'depth: 0.001 wrench { body_1_name: "a" body_1_id: 1 body_2_name: "b" body_2_id: 2 body_1_wrench '
            '{ force { x: 0 y: 0 z: %g } torque { x: 0 y: 0 z: 0 } } body_2_wrench { force { x: 0 y: 0 z: 0 } '
            'torque { x: 0 y: 0 z: 0 } } } time { sec: 0 nsec: 0 } world: "default" }' % (c1, c2, fz))


def line(n, *pairs):
    ns = n * D.STEP_NS
    return " ".join(list(pairs) + ["time { sec: %d nsec: %d }" % (ns // 10 ** 9, ns % 10 ** 9)])


def write_log(path, steps, contacts=lambda n: (), extra=None, tail=""):
    with open(path, "w") as fh:
        for n in steps:
            if extra and n in extra:
                fh.write(extra[n] + "\n")
            fh.write(line(n, *contacts(n)) + "\n")
        fh.write(tail)


class ContactText(unittest.TestCase):
    def test_parses_contacts_and_rejects_other_lines(self):
        m = D.contacts_message(line(3086, pair(BASE, FLOOR), pair("bar_12::l::c", ROTOR, -2.5)))
        self.assertEqual(m["t_ns"], 3086 * D.STEP_NS)
        self.assertAlmostEqual(m["t"], 12.344)
        self.assertEqual([(p["c1"], p["c2"]) for p in m["pairs"]], [(BASE, FLOOR), ("bar_12::l::c", ROTOR)])
        self.assertEqual(m["pairs"][1]["body_1_force_sum"], [0.0, 0.0, -2.5])
        self.assertEqual(D._kind(m["pairs"][1]), (ROTOR, "bar_12::l::c", "bar"))
        self.assertEqual(D._kind(m["pairs"][0])[2], "floor")
        self.assertEqual((D.vehicle_part(ROTOR), D.vehicle_part(BASE)), ("rotor", "base"))
        self.assertEqual(D.contacts_message("time { sec: 1 nsec: 4000000 }")["pairs"], [])
        self.assertEqual(D.parse_text(r'a: "x\"y\\z" b: -inf c: 1e-05 d: true e { }'),
                         {"a": ['x"y\\z'], "b": [float("-inf")], "c": [1e-05], "d": [True], "e": [{}]})
        for bad in ("Msg Waiting for master", "[Wrn] [Publisher.cc:141] Queue limit reached", "",
                    'contact { collision1: "a"', "time { sec: 1 } }", "name { sec: 1 }",
                    'contact { collision2: "b" } time { sec: 1 nsec: 0 }'):
            with self.assertRaises(D.ParseError, msg=bad):
                D.contacts_message(bad)


class Audit(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="agq_audit_"))
        self.log = self.tmp / "contacts.log"

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    @staticmethod
    def flight(n):               # floor until 8 s; rotor-bar 12.0-12.1 s and 13.0-13.02 s; base-bar 14 s
        out = []
        if n * 0.004 < 8.0:
            out.append(pair(BASE, FLOOR))
        if 3000 <= n <= 3025 or 3250 <= n <= 3255:
            out.append(pair(ROTOR, "bar_168::l::c"))
        if n == 3500:
            out.append(pair(BASE, "bar_7::l::c"))
        return out

    def test_complete_log_events_and_problems(self):
        write_log(self.log, range(0, 4000), self.flight, extra={10: "[Wrn] something"}, tail="contact { coll")
        summary, problems = D.contact_audit(self.log, start_sim_s=10.0, final_sim_s=15.0,
                                            out_jsonl=self.tmp / "c.jsonl")
        self.assertEqual(problems, [])
        self.assertEqual((summary["messages"], summary["other_lines"], summary["partial_tail"]), (4000, 1, True))
        self.assertEqual(summary["floor"]["pairs"], [BASE + "|" + FLOOR])
        self.assertAlmostEqual(summary["floor"]["last_t"], 7.996)
        first = summary["first_bar_contact_after_start"]
        self.assertEqual((first["bar"], first["vehicle_part"]), (168, "rotor"))
        self.assertAlmostEqual(first["elapsed_s"], 2.0)
        rotor = summary["bar_events"][0]
        self.assertEqual(len(rotor["intervals"]), 2)
        self.assertAlmostEqual(rotor["intervals"][1][0], 13.0)
        self.assertEqual([e["vehicle_part"] for e in summary["bar_events"]], ["rotor", "base"])
        self.assertEqual(len((self.tmp / "c.jsonl").read_text().splitlines()), summary["nonempty_messages"])
        self.assertGreater(summary["nonempty_messages"], 2000)

    def test_takeoff_anchor_ignores_a_touchdown_after_the_crash(self):
        write_log(self.log, range(0, 4000), lambda n: [pair(BASE, FLOOR)] if n * 0.004 < 8.0 or n >= 3880 else [])
        summary, problems = D.contact_audit(self.log, start_sim_s=10.0, final_sim_s=15.0)
        self.assertEqual(problems, [])
        self.assertAlmostEqual(summary["floor"]["last_t"], 15.996)              # the touchdown after the crash
        self.assertAlmostEqual(summary["floor"]["last_t_before_start"], 7.996)   # the takeoff

    def test_gaps_outside_the_reading_window_are_reported_only(self):
        write_log(self.log, [n for n in range(0, 4000) if not 100 <= n < 105], self.flight)   # a gap during boot
        summary, problems = D.contact_audit(self.log, start_sim_s=10.0, final_sim_s=15.0)
        self.assertEqual((problems, summary["gaps"], summary["gaps_in_window"]), ([], 1, 0))
        write_log(self.log, [n for n in range(0, 4000) if not 2400 <= n < 2405], self.flight)  # 9.6 s: in window
        self.assertTrue(D.contact_audit(self.log, start_sim_s=10.0, final_sim_s=15.0)[1])

    def test_loss_order_and_coverage_are_problems(self):
        cases = [(lambda: [n for n in range(0, 4000) if not 2400 <= n < 2405], "RECORDER_LOSS: 1 simulator-time "
                  "gaps in the reading window (1 gaps, 5 missing steps"),
                 (lambda: list(range(0, 2400)) + [2399] + list(range(2400, 4000)), "RECORDER_ORDER"),
                 (lambda: range(2300, 4000), "RECORDER_COVERAGE: first"),
                 (lambda: range(0, 3800), "RECORDER_COVERAGE: last")]
        for steps, want in cases:
            write_log(self.log, steps(), self.flight)
            _, problems = D.contact_audit(self.log, start_sim_s=10.0, final_sim_s=15.0)
            self.assertTrue(any(p.startswith(want) for p in problems), (want, problems))
        self.log.write_text("")
        self.assertEqual(D.contact_audit(self.log)[1], ["RECORDER_LOSS: no contact messages"])
        self.assertEqual(D.contact_audit(self.tmp / "none.log")[1], ["RECORDER_LOSS: contact log missing"])

    def test_floor_control_threshold_and_refusals(self):
        def floor_every_other(k):
            return lambda n: [pair(BASE, FLOOR)] if (n % 2 == 0 or n < k) else []
        write_log(self.log, range(0, 1000), floor_every_other(0))        # 125 of the last 250
        ok, detail = D.floor_control(self.log)
        self.assertTrue(ok, detail)
        self.assertEqual((detail["floor_messages"], detail["gaps"]), (125, 0))
        write_log(self.log, range(0, 1000), lambda n: [pair(BASE, FLOOR)] if n % 2 == 0 and n != 998 else [])
        self.assertFalse(D.floor_control(self.log)[0])                   # 124
        write_log(self.log, range(0, 1000), lambda n: [pair(ROTOR, "bar_1::l::c")])
        self.assertFalse(D.floor_control(self.log)[0])                   # a contact, but not the floor
        write_log(self.log, [n for n in range(0, 1000) if n != 900], lambda n: [pair(BASE, FLOOR)])
        self.assertIn("gaps", D.floor_control(self.log)[1]["problem"])
        write_log(self.log, range(0, 249), lambda n: [pair(BASE, FLOOR)])
        self.assertIn("only 249", D.floor_control(self.log)[1]["problem"])
        write_log(self.log, range(0, 1000), lambda n: [pair(BASE, FLOOR)], extra={900: "Msg noise"})
        self.assertIn("non-message", D.floor_control(self.log)[1]["problem"])
        write_log(self.log, range(0, 1000), lambda n: [pair(BASE, FLOOR)], extra={500: "Msg Waiting for master"})
        self.assertTrue(D.floor_control(self.log)[0])                    # noise before the window is allowed
        self.assertFalse(D.floor_control(self.tmp / "missing.log")[0])
        write_log(self.log, range(0, 5000), lambda n: [pair(BASE, FLOOR)])
        self.assertTrue(D.floor_control(self.log, tail_bytes=300000)[0])  # a tail read drops its first cut line


class Wrappers(unittest.TestCase):
    def test_send_proxy_times_every_send_only(self):
        tap = D.Tap(clock=iter(range(100)).__next__)
        calls = []

        class Mav:
            srcSystem = 245

            def set_position_target_local_ned_send(self, *a):
                calls.append(a)

            def param_set_send(self, *a):
                raise OSError("down")

            def other(self):
                return "x"
        conn = types.SimpleNamespace(mav=Mav(), recv_match=lambda **kw: "msg", close=lambda: None)
        rc = D.RecordingConn(conn, tap)
        rc.mav.set_position_target_local_ned_send(0, 1, 1, 1, 3527, 0.0)
        with self.assertRaises(OSError):
            rc.mav.param_set_send(1, 1, b"MPC_XY_P", 0.5, 9)
        self.assertEqual(rc.mav.other(), "x")
        self.assertEqual(rc.recv_match(), "msg")
        rc.mav.srcSystem = 7
        self.assertEqual(conn.mav.srcSystem, 7)
        self.assertEqual(len(calls), 1)
        self.assertEqual([(s["i"], s["call"], s["t_entry"], s["t_return"]) for s in tap.sends],
                         [(0, "set_position_target_local_ned_send", 0, 1), (1, "param_set_send", 2, 3)])
        self.assertEqual(tap.sends[1]["args"][2], "MPC_XY_P")
        self.assertIn("OSError", tap.sends[1]["error"])

    def stream(self):
        msgs = []
        for i in range(30):
            msgs.append(PT.FakeMsg("LOCAL_POSITION_NED", (1, 1), {"time_boot_ms": 20 * i, "x": 0.0, "y": 0.0,
                                                                  "z": -1.0, "vx": 0.0, "vy": 0.0, "vz": 0.0}))
            msgs.append(PT.FakeMsg("ATTITUDE", (1, 1), {"time_boot_ms": 20 * i, "roll": 0.0, "pitch": 0.0, "yaw": 0.1,
                                                        "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0}))
            msgs.append(PT.FakeMsg("HIL_STATE_QUATERNION", (1, 1), {"time_usec": 0, "attitude_quaternion":
                                                                    [1.0, 0.0, 0.0, 0.0], "lat": 1, "lon": 1,
                                                                    "alt": 1000, "vx": 0, "vy": 0, "vz": 0}))
            if i % 10 == 0:
                msgs.append(PT.FakeMsg("COMMAND_ACK", (1, 1), {"command": 176, "result": 0}))
                msgs.append(PT.FakeMsg("SYS_STATUS", (1, 1), {}))
        return msgs

    def feed(self, inbox_cls, transport_cls):
        msgs = self.stream()
        now = [100.0]
        inbox = inbox_cls(clock=lambda: now[0])

        class Conn:
            def __init__(self):
                self.it = iter(msgs)
                self.mav = None

            def recv_match(self, blocking=True, timeout=0.5):
                m = next(self.it, None)
                if m is None:
                    tr.stop.set()
                else:
                    now[0] += 0.02 / 3                             # one 50 Hz triple per 20 ms
                return m
        tr = transport_cls(Conn(), inbox)
        tr.rx_loop()
        return inbox

    def test_receiver_keeps_the_runner_inbox_identical(self):
        tap = D.Tap()
        ns = D.recording_link(LK, tap, lambda t: (True, {}), harness_invalid=R.HarnessInvalid)
        base = self.feed(LK.Inbox, LK.PymavlinkTransport)
        rec = self.feed(ns.Inbox, ns.PymavlinkTransport)
        self.assertIs(tap.inbox, rec)
        self.assertEqual(base.since(0)[0], rec.since(0)[0])                 # same items, seq and receive times
        self.assertEqual(len(base.since(0)[0]), 90)
        self.assertEqual([e["type"] for e in tap.extra], ["COMMAND_ACK"] * 3)
        self.assertEqual(tap.counts["SYS_STATUS"], 3)
        self.assertEqual(tap.counts["LOCAL_POSITION_NED"], 30)
        gates = []
        for inbox, cls in ((base, LK.TelemetryGate), (rec, ns.TelemetryGate)):
            g = cls(inbox, start_cursor=0, stall_wall_s=0.1)
            g.align()
            gates.append([g.checkpoint(0)])
        self.assertEqual(gates[0], gates[1])                                # same decisions and samples
        self.assertEqual(tap.snapshots, gates[1])

        class Exited:
            def check(self, required=()):
                raise OS_.ProcessExited("contacts exited with 1")
        tap.session = Exited()
        g = ns.TelemetryGate(rec, start_cursor=0, stall_wall_s=0.1)
        g.align()
        with self.assertRaises(OS_.ProcessExited):
            g.checkpoint(0)

    def test_control_runs_after_parameters_and_fails_closed(self):
        order = []
        fake_lk = types.SimpleNamespace(STREAMS=LK.STREAMS, Inbox=LK.Inbox, PymavlinkTransport=LK.PymavlinkTransport,
                                        TelemetryGate=LK.TelemetryGate,
                                        apply_and_verify=lambda *a, **k: order.append("params") or {"p": 1})
        tap = D.Tap()
        ns = D.recording_link(fake_lk, tap, lambda t: (order.append("control"), (True, {"x": 1}))[1],
                              harness_invalid=R.HarnessInvalid)
        self.assertEqual(ns.apply_and_verify(None, None, {}), {"p": 1})
        self.assertEqual((order, tap.parameters), (["params", "control"], {"p": 1}))
        self.assertEqual({k: tap.control[k] for k in ("x", "ok")}, {"x": 1, "ok": True})
        self.assertLessEqual(tap.control["t_entry"], tap.control["t_return"])
        ns = D.recording_link(fake_lk, tap, lambda t: (False, {"problem": "silent"}), harness_invalid=R.HarnessInvalid)
        with self.assertRaisesRegex(R.HarnessInvalid, "CONTACT_CONTROL silent"):
            ns.apply_and_verify(None, None, {})
        self.assertFalse(D.session_control(D.Tap())[0])                     # never started: no control


FAKE_CONTACTS = r'''
import signal, sys, time
signal.signal(signal.SIGINT, lambda *a: sys.exit(0))            # the owned cleanup interrupts first
mode = sys.argv[1]
BASE = "navrl_quad::base_link::base_link_inertia_collision"
def pair(c2, c1=BASE):
    return ('contact { collision1: "%s" collision2: "%s" position { x: 1 y: 1 z: 0.05 } depth: 0.001 '
            'time { sec: 0 nsec: 0 } world: "default" }' % (c1, c2))
def line(n):
    t = n * 0.004
    parts = []
    if mode != "nofloor" and t < 1.7:
        parts.append(pair("asphalt_plane::link::collision"))
    if 1.8 <= t <= 1.9:                                         # read as PX4 time: inside the fake flight
        parts.append(pair("bar_0::l::c", "navrl_quad::rotor_0::rotor_0_collision"))
    ns = n * 4000000
    parts.append("time { sec: %d nsec: %d }" % (ns // 10 ** 9, ns % 10 ** 9))
    return " ".join(parts)
print("Msg Waiting for master", flush=True)
t0 = time.monotonic()
n = 0
while True:
    target = int((time.monotonic() - t0 + 1.0) / 0.004)        # one simulated second ahead of the fake PX4
    while n <= target:
        if not (mode == "gaplate" and 700 <= n < 710):
            print(line(n), flush=True)
        n += 1
    if mode == "exit" and n > 300:
        sys.exit(1)
    time.sleep(0.01)
'''


def RELAXED(tap):                       # the fake subscriber's clock leads the fake PX4 by about 1 s
    return D.session_control(tap, fresh_s=5.0)


class Driver(unittest.TestCase):
    """run_episode with the recording dependencies, the real owned session and link, a fake PX4 and subscriber."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="agq_driver_"))
        self.allowed = self.tmp / "attempts"
        self.allowed.mkdir()
        self.ports = PT.free_ports()
        self.conns = []

    def tearDown(self):
        for c in self.conns:
            c.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def base(self):
        def session(episode_dir):
            return OS_.OwnedSession(episode_dir, allowed_root=self.allowed, lock_path=self.tmp / "lock",
                                    ports=self.ports, foreign_names=("motar_fake_gz",), forbidden_roots=(),
                                    deadline_s=60.0, kill_grace_s=0.5, sample_s=0.1, include_self_rss=False)

        def connect():
            c = PT.FakePX4Conn(LT.fresh_boot())
            self.conns.append(c)
            return c
        return {"session": session, "empty_world": lambda: PT.FIXTURE_EMPTY,
                "binding": lambda pl: {"files": {"px4_binary": {"sha256": "p"},
                                                 "model_sdf": {"sha256": pl["sdf_sha256"]}}},
                "launch": lambda sess, plan: OS_.launch_px4_gazebo(sess, plan, spawn_retry_s=0.05),
                "connect": connect, "fresh_defaults": lambda: LT.EXPECT, "link": LK, "step": ST}

    def M(self):
        argv = [sys.executable, "-c", PT.FAKE_PROC]
        env = dict(os.environ, PYTHONNOUSERSITE="1")

        def launch_plan(owned, episode_dir, name, base_env, package):
            receipt = json.loads((Path(package) / "geometry.json").read_text())
            return {"world_path": Path(episode_dir) / "world" / (name + ".world"), "env": env,
                    "sdf_sha256": receipt["output_sdf_sha256"], "geometry_receipt": {"path": str(package)},
                    "gzserver": {"argv": argv, "cwd": episode_dir, "env": env},
                    "spawn": {"argv": [sys.executable, "-c", "print('spawned')"], "cwd": episode_dir, "env": env},
                    "px4": {"argv": argv, "cwd": Path(episode_dir) / "rootfs", "env": env}}

        def prepare(directory, mode):
            directory.mkdir(parents=True)
            receipt = {"mode": mode, "output_directory": str(directory), "output_sdf_sha256": "sdf-" + mode}
            (directory / "geometry.json").write_text(json.dumps(receipt))
            return receipt
        return types.SimpleNamespace(PR=R, AT=D.modules().AT, OS=OS_,
                                     TG=types.SimpleNamespace(launch_plan=launch_plan, prepare=prepare))

    def member(self, mode, frames=12):
        M = self.M()
        package = self.tmp / "pkg"
        if not package.exists():
            M.TG.prepare(package, "training-box")
        tap = D.Tap()
        argv = (sys.executable, "-c", FAKE_CONTACTS, mode)
        deps = D.member_deps(self.base(), M, tap, package, {}, contact_argv=argv, control=RELAXED)
        member = dict(D.schedule()[1], name="m_%s" % mode)
        ep = self.allowed / member["name"]
        r = R.run_episode(member, PT.EpisodeDriver.spec(None, frames), frames,
                          load("cfg_agq", "tools/sitl_s3b_dev08.py").prepare(arm="native")["selected_params"],
                          deps=deps, B=SI.STUB_B, policy=SI.StubPolicy(), episode_dir=ep, timing=PT.FAST)
        manifest = D.write_recording(tap, ep)
        audit, problems = D.contact_audit(ep / "logs" / "contacts.log", start_sim_s=r.get("start_sim_time_s"),
                                          final_sim_s=r.get("final_sim_time_s"))
        return r, tap, manifest, audit, D.recorder_problems(r, tap, problems, manifest, R.historical_masks()[1])

    def test_valid_member_records_everything(self):
        r, tap, manifest, audit, problems = self.member("ok")
        self.assertEqual(r["operational_status"], "VALID", r.get("error"))
        self.assertEqual(problems, [])
        self.assertTrue(tap.control["ok"])
        self.assertGreaterEqual(tap.control["floor_messages"], 125)
        k = r["final_frame_index"]
        self.assertEqual(len(tap.snapshots), k + 1)
        velocity = [s for s in tap.sends if s["call"] == "set_position_target_local_ned_send"
                    and s["args"][4] == R.historical_masks()[1]]
        self.assertEqual(len(velocity), k)
        self.assertTrue(all(s["t_entry"] <= s["t_return"] for s in tap.sends))
        self.assertGreater(manifest["arrivals"], 100)
        self.assertEqual(audit["other_lines"], 1, audit["other_samples"])     # the "Waiting for master" line
        self.assertEqual(audit["gaps"], 0)
        self.assertEqual(audit["bar_events"][0]["vehicle_part"], "rotor")
        self.assertEqual(r["runtime_binding"]["files"]["model_sdf"]["sha256"], "sdf-training-box")
        self.assertEqual(r["runtime_binding"]["contact_subscriber"]["argv"][2:], [FAKE_CONTACTS, "ok"])
        self.assertEqual(r["runtime_binding"]["files"]["contact_stdbuf_executable"]["path"], "/usr/bin/stdbuf")
        self.assertIsNotNone(tap.parameters)
        self.assertLessEqual(tap.control["t_entry"], tap.control["t_return"])
        self.assertIsNotNone(tap.truth_origin)
        files = set(manifest["files"])
        self.assertEqual(files, {"mavlink_arrivals.jsonl", "mavlink_sends.jsonl", "mavlink_extra.jsonl",
                                 "snapshots.jsonl", "recording.json"})
        receipt = json.loads(Path(r["session_receipt"]["path"]).read_text())
        self.assertEqual(receipt["cleanup"]["leaked"], [])
        self.assertEqual([x["role"] for x in receipt["launches"]][-2:], ["px4", "contacts"])

    def test_control_and_recorder_failures(self):
        r, tap, _, _, problems = self.member("nofloor")
        self.assertEqual(r["operational_status"], "HARNESS_INVALID")
        self.assertIn("CONTACT_CONTROL", r["error"])
        self.assertEqual(tap.snapshots, [])                                    # stopped before any setpoint
        self.assertFalse([s for s in tap.sends if s["call"] == "set_position_target_local_ned_send"])
        r, tap, _, _, _ = self.member("exit")
        self.assertNotEqual(r["operational_status"], "VALID")
        self.assertRegex(r["error"], "CONTACT_CONTROL|exited")
        r, tap, _, audit, problems = self.member("gaplate")
        self.assertEqual(r["operational_status"], "VALID", r.get("error"))
        self.assertTrue(any(p.startswith("RECORDER_LOSS: 1 simulator-time gaps in the reading window")
                            for p in problems), problems)

    def test_missing_stdbuf_refuses_the_binding(self):
        M = self.M()
        package = self.tmp / "pkg_s"
        M.TG.prepare(package, "native")
        deps = D.member_deps(self.base(), M, D.Tap(), package, {}, stdbuf_files={"x": str(self.tmp / "none.so")})
        with self.assertRaisesRegex(RuntimeError, "runtime file missing"):
            deps["binding"]({"sdf_sha256": "s"})
        ok = D.member_deps(self.base(), M, D.Tap(), package, {})["binding"]({"sdf_sha256": "s"})
        self.assertEqual(set(ok["files"]) - {"px4_binary", "model_sdf"}, set(D.STDBUF_FILES))

    def test_control_freshness(self):
        tmp = self.tmp / "fresh"
        (tmp / "logs").mkdir(parents=True)
        write_log(tmp / "logs" / "contacts.log", range(0, 1000), lambda n: [pair(BASE, FLOOR)])   # ends 3.996 s

        class Running:
            episode_dir = tmp

            def check(self, required=()):
                pass

        def tap_at(px4_ms):
            t = D.Tap()
            t.session, t.contacts_started = Running(), True
            t.inbox = LK.Inbox()
            if px4_ms is not None:
                t.inbox.put({"type": "LOCAL_POSITION_NED", "src": (1, 1), "fields": {"time_boot_ms": px4_ms}})
            return t
        self.assertTrue(D.session_control(tap_at(4200))[0])
        ok, detail = D.session_control(tap_at(4600))                         # the log stopped 0.6 s ago
        self.assertFalse(ok)
        self.assertIn("not current", detail["problem"])
        self.assertFalse(D.session_control(tap_at(None))[0])


SCENE = {"centers": [[5.8, 0.34, 1.5], [40.0, 40.0, 1.5]], "halves": [[0.1, 0.1, 1.5], [0.1, 0.1, 1.5]]}


class Reading(unittest.TestCase):
    @staticmethod
    def ev(rates=(), rotor=None, windows=(), z=None, path_end=130, shift=0.0, y=0.0, final=13.0, source="ULog",
           span=(-30.0, 20.0), gaps=(), scene=SCENE, outcome="capture", late_y=None, xfun=None):
        def pos(k):
            yy = late_y if late_y is not None and k * 0.1 > 5.73 else y
            return [(xfun(k * 0.1) if xfun else k * 0.1) + shift, yy, 1.0]
        return {"rate_windows": [list(w) for w in rates], "rate5_s": rates[0][0] if rates else None,
                "first_rotor_bar_contact_s": rotor, "first_rotor_bar": 0 if rotor is not None else None,
                "contact_windows": [list(w) for w in windows], "z_below_0p8_s": z, "rate_source": source,
                "final_elapsed_s": final, "outcome": outcome, "rate_span_s": list(span),
                "rate_gaps_s": [list(g) for g in gaps], "scene": scene,
                "path": [[k, k * 0.1, pos(k), None] for k in range(path_end)]}

    def test_fixed_readings(self):
        native = self.ev(rates=[(5.77, 5.9)], rotor=5.73, windows=[(5.73, 5.80)], final=7.0, path_end=71)
        cases = [(self.ev(), "CONSISTENT_WITH_ROTOR_CONTACT"),
                 (self.ev(rates=[(5.9, 6.0)]), "DISTURBANCE_PERSISTS_WITHOUT_CONTACT"),
                 (self.ev(rates=[(5.9, 6.0)], windows=[(5.85, 5.9)]), "CONTACT_PERSISTS"),
                 (self.ev(windows=[(5.6, 5.7)]), "INCONCLUSIVE"),
                 (self.ev(rates=[(6.5, 6.6)]), "DISTURBANCE_PERSISTS_WITHOUT_CONTACT"),   # late: review r1 M1b
                 (self.ev(rates=[(6.3, 6.4)], windows=[(6.25, 6.3)]), "CONTACT_PERSISTS"),  # its own later contact
                 (self.ev(rates=[(7.3, 7.4)]), "CONSISTENT_WITH_ROTOR_CONTACT"),           # after t_c + 1.5 s
                 (self.ev(rates=[(4.9, 5.0), (5.9, 6.0)]), "DISTURBANCE_PERSISTS_WITHOUT_CONTACT"),  # M1a
                 (self.ev(rates=[(4.9, 5.3)]), "DISTURBANCE_PERSISTS_WITHOUT_CONTACT"),    # runs into the window
                 (self.ev(rates=[(4.9, 5.0)]), "CONSISTENT_WITH_ROTOR_CONTACT"),           # before the window
                 (self.ev(y=-0.03), "INCONCLUSIVE_NOT_EXPOSED"),                           # 0.27 m > 0.24 m
                 (self.ev(y=0.03), "CONSISTENT_WITH_ROTOR_CONTACT"),                       # came closer
                 (self.ev(y=-0.001), "INCONCLUSIVE_NOT_EXPOSED"),                          # no tolerance
                 (self.ev(shift=0.31), "INCONCLUSIVE_PATHS_DIVERGED"),
                 (self.ev(shift=0.29), "CONSISTENT_WITH_ROTOR_CONTACT"),
                 (self.ev(path_end=30, final=3.0), "INCONCLUSIVE_PATHS_DIVERGED"),          # ended before t_c
                 (self.ev(final=6.0), "INCONCLUSIVE_PATHS_DIVERGED"),                       # ended in the window
                 (self.ev(final=6.5, rates=[(6.7, 6.8)]), "CONSISTENT_WITH_ROTOR_CONTACT"), # after its flight ended
                 (self.ev(final=6.5, outcome="crash"), "INCONCLUSIVE"),                     # crashed in its window
                 (self.ev(span=(-30.0, 6.0)), "NOT_EVALUABLE"),                             # ULog ends: review r2 N2
                 (self.ev(gaps=[(6.0, 6.2)]), "NOT_EVALUABLE"),                             # a gap in the window
                 (self.ev(gaps=[(8.0, 8.2)]), "CONSISTENT_WITH_ROTOR_CONTACT"),             # a gap after it
                 (self.ev(scene={"centers": [[0, 0, 0]], "halves": [[1, 1, 1]]}), "NOT_EVALUABLE"),
                 (self.ev(source="NOT_RECORDED"), "NOT_EVALUABLE"),
                 (None, "NOT_EVALUABLE")]
        for box, want in cases:
            got = D.pair_reading(native, box)
            self.assertEqual(got["reading"], want, (box and {k: box[k] for k in box if k != "path"}, got))
        early = self.ev(rates=[(3.0, 3.05), (5.77, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0)
        self.assertEqual(D.pair_reading(early, self.ev())["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        closer_after = self.ev(rates=[(5.77, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0, late_y=0.2)
        got = D.pair_reading(closer_after, self.ev())                       # the reference is interpolated at t_c
        self.assertAlmostEqual(got["native_clearance_to_bar_m"], 0.18)
        self.assertEqual(got["reading"], "INCONCLUSIVE_NOT_EXPOSED")
        self.assertEqual(D.pair_reading(closer_after, self.ev(y=0.07))["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        bound = self.ev(rates=[(5.40, 5.52), (5.77, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0)
        self.assertEqual(D.pair_reading(bound, self.ev())["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")  # calm
        bound = self.ev(rates=[(5.40, 5.54), (5.77, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0)
        self.assertEqual(D.pair_reading(bound, self.ev())["reading"], "DISTURBANCE_WITHOUT_CONTACT")  # in progress
        late_crash = self.ev(final=7.4, outcome="crash")                     # crashed after its window
        self.assertEqual(D.pair_reading(native, late_crash)["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        in_progress = self.ev(rates=[(5.6, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0)
        self.assertEqual(D.pair_reading(in_progress, self.ev())["reading"], "DISTURBANCE_WITHOUT_CONTACT")  # r2 N5
        rejoin = self.ev(rates=[(5.6, 5.65), (5.8, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0)
        self.assertNotEqual(D.pair_reading(rejoin, self.ev())["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        after_final = self.ev(rates=[(7.2, 7.3)], final=7.0)
        self.assertEqual(D.pair_reading(after_final, self.ev())["reading"], "INCONCLUSIVE_NOT_REPRODUCED")  # r2 N4
        short = self.ev(rates=[(5.77, 5.9)], rotor=5.73, windows=[(5.73, 5.8)], final=7.0, span=(-30.0, 6.5))
        self.assertEqual(D.pair_reading(short, self.ev())["reading"], "NOT_EVALUABLE")
        self.assertEqual(D.pair_reading(self.ev(rates=[(4.0, 4.1)]), self.ev())["reading"],
                         "DISTURBANCE_WITHOUT_CONTACT")
        self.assertEqual(D.pair_reading(self.ev(z=4.0, windows=[(3.8, 3.9)]), self.ev())["reading"],
                         "INCONCLUSIVE_NOT_REPRODUCED")
        self.assertEqual(D.pair_reading(self.ev(rates=[(6.3, 6.4)], rotor=5.73, windows=[(5.73, 5.8)]),
                                        self.ev())["reading"], "INCONCLUSIVE_NOT_REPRODUCED")  # 0.57 s after
        self.assertEqual(D.pair_reading(self.ev(), self.ev())["reading"], "INCONCLUSIVE_NOT_REPRODUCED")
        self.assertEqual(D.bar_clearance([5.8, 0.0, 4.0], [5.8, 0.34, 1.5], [0.1, 0.1, 1.5]), float("inf"))

    def test_exposure_reference_bounds_sampling_phase(self):             # review r3 M1, r4 m1 (r5 l3: name)
        scene = {"centers": [[6.2, 0.0, 1.5]], "halves": [[0.1, 0.1, 1.5]]}   # clearance = 6.1 - x on approach
        native = self.ev(rates=[(5.83, 5.9)], rotor=5.79, windows=[(5.79, 5.85)], final=7.0, path_end=71,
                         scene=scene)
        stays_away = self.ev(scene=scene, xfun=lambda t: min(t, 5.77))      # 0.33 m: beyond any rotor reach
        reaches = self.ev(scene=scene, xfun=lambda t: min(t, 5.80))         # 0.30 m: as close as the native
        got = D.pair_reading(native, stays_away)
        self.assertAlmostEqual(got["native_clearance_to_bar_m"], 0.31)      # at t_c, not 0.40 at the last sample
        self.assertEqual(got["reading"], "INCONCLUSIVE_NOT_EXPOSED")
        self.assertEqual(D.pair_reading(native, reaches)["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        early = self.ev(rates=[(5.75, 5.9)], rotor=5.71, windows=[(5.71, 5.85)], final=7.0, path_end=71, scene=scene)
        self.assertAlmostEqual(D.pair_reading(early, stays_away)["native_clearance_to_bar_m"], 0.39)
        # the box window is [t_c - 0.5, t_c + 0.5]: a closest approach after it does not count
        later = self.ev(scene=scene, xfun=lambda t: min(t, 5.6) if t < 6.4 else 5.6 + (t - 6.4))
        self.assertEqual(D.pair_reading(native, later)["reading"], "INCONCLUSIVE_NOT_EXPOSED")
        # the box passes closest between two checkpoints: only the 5 ms interpolation sees it (r4 v5 survivor)
        native2 = self.ev(rates=[(5.87, 5.9)], rotor=5.83, windows=[(5.83, 5.85)], final=7.0, path_end=71,
                          scene=scene)                                       # reference 0.27 m at t_c = 5.83 s
        straddle = self.ev(scene=scene, late_y=0.35, xfun=lambda t: t if t <= 5.7 else 5.7 + 7.5 * (t - 5.7))
        self.assertEqual(D.pair_reading(native2, straddle)["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        # N stops at the contact between checkpoints: the extrapolated reference bounds the chord (review r4 m1)
        stops = self.ev(rates=[(5.78, 5.9)], rotor=5.75, windows=[(5.75, 5.8)], final=7.0, path_end=71, scene=scene,
                        xfun=lambda t: min(t, 5.75))
        near = self.ev(scene=scene, xfun=lambda t: min(t, 5.74))            # 0.36 m: farther than N's 0.35 m
        got = D.pair_reading(stops, near)
        self.assertAlmostEqual(got["native_clearance_to_bar_m"], 0.35)
        self.assertAlmostEqual(got["native_clearance_interpolated_m"], 0.375)   # both candidates are reported
        self.assertAlmostEqual(got["native_clearance_extrapolated_m"], 0.35)
        self.assertEqual(got["reading"], "INCONCLUSIVE_NOT_EXPOSED")
        same = self.ev(scene=scene, xfun=lambda t: min(t, 5.75))            # an identical path: exposed
        self.assertEqual(D.pair_reading(stops, same)["reading"], "CONSISTENT_WITH_ROTOR_CONTACT")
        self.assertEqual(D._path_at([[0, 0.0, [0.0, 0.0, 1.0], None], [1, 0.1, [1.0, 2.0, 1.0], None]], 0.05),
                         [0.5, 1.0, 1.0])
        self.assertEqual(D._path_extrapolated([[0, 0.0, [0.0, 0.0, 1.0], None], [1, 0.1, [1.0, 2.0, 1.0], None]],
                                              0.15), [1.5, 3.0, 1.0])
        self.assertIsNone(D._path_extrapolated([[0, 0.0, [0.0, 0.0, 1.0], None]], 0.05))
        samples = D._path_samples([[0, 0.0, [0.0, 0.0, 1.0], None], [1, 0.1, [1.0, 0.0, 1.0], None]], 0.0, 0.1)
        self.assertEqual((len(samples), samples[-1]), (22, [1.0, 0.0, 1.0]))   # 21 grid points and the end
        self.assertIsNone(D._path_at([[0, 0.0, [0.0, 0.0, 1.0], None]], 0.05))

    def test_member_events(self):
        rec = {"start_sim_time_s": 40.0, "final_sim_time_s": 47.0, "outcome": "crash", "crash_reason": "altitude"}
        rows = [{"k": k, "sim_s": 40.0 + 0.1 * k, "p_true": [k * 0.1, 0.0, 1.0 if k < 60 else 0.7],
                 "p": [k * 0.1 + (0.0 if k < 65 else 0.6), 0.0, 1.0], "clearance_m": 1.0 - 0.01 * k}
                for k in range(71)]
        audit = {"bar_events": [{"vehicle_part": "rotor", "bar": 7, "intervals": [[39.0, 39.1], [45.73, 45.8]]},
                                {"vehicle_part": "base", "bar": 2, "intervals": [[46.0, 46.01]]}],
                 "first_bar_contact_after_start": {"elapsed_s": 5.73}}
        times = [40.0 + 0.01 * i for i in range(900) if not 300 <= i < 310]
        rates = [(t, (0.0, 0.0, 6.0 if 42.0 <= t <= 42.1 or t > 45.77 else 1.0)) for t in times]
        ev = D.member_events(rec, rows, audit, {"rates": rates, "dropouts": [(48.0, 0.05)]}, scene=SCENE)
        self.assertAlmostEqual(ev["first_rotor_bar_contact_s"], 5.73)
        self.assertEqual(ev["first_rotor_bar"], 7)
        self.assertAlmostEqual(ev["first_base_bar_contact_s"], 6.0)
        self.assertAlmostEqual(ev["rate5_s"], 2.0, places=6)
        self.assertEqual(len(ev["rate_windows"]), 2)
        self.assertAlmostEqual(ev["rate_windows"][1][0], 5.78, places=6)
        self.assertEqual(len(ev["rate_gaps_s"]), 2)                           # the 0.1 s hole and the dropout
        self.assertAlmostEqual(ev["rate_gaps_s"][0][0], 2.99, places=6)
        self.assertAlmostEqual(ev["rate_gaps_s"][1][1], 8.05, places=6)
        self.assertAlmostEqual(ev["z_below_0p8_s"], 6.0)
        self.assertAlmostEqual(ev["error_above_0p5_s"], 6.5)
        self.assertEqual(len(ev["contact_windows"]), 2)                       # the pre-start contact is not counted
        self.assertAlmostEqual(ev["path"][10][3], 0.9)
        rows[5]["clearance_m"] = float("inf")                                # outside every bar's height
        self.assertIsNone(D.member_events(rec, rows, audit, None)["path"][5][3])
        self.assertEqual(D.member_events(rec, rows, audit, None)["rate_source"], "NOT_RECORDED")

    def test_numpy_typed_series(self):                                       # review r2 N1: pyulog gives numpy
        heights = [(np.float64(10.0 + 0.02 * i), np.float32(-0.09 if 10.0 + 0.02 * i < 20.06 else -0.2))
                   for i in range(800)]
        ok = D.takeoff_check(np.float64(20.0), heights)
        self.assertIs(ok["ok"], True)
        json.dumps(ok, allow_nan=False)

    def test_takeoff_clock_check_and_velocities(self):
        heights = [(10.0 + 0.02 * i, -0.09 if 10.0 + 0.02 * i < 20.06 else -0.2) for i in range(800)]
        ok = D.takeoff_check(20.0, heights)
        self.assertTrue(ok["ok"], ok)
        self.assertAlmostEqual(ok["rise_minus_takeoff_contact_s"], 0.06, places=6)
        self.assertFalse(D.takeoff_check(19.5, heights)["ok"])               # rise 0.56 s after the contact
        self.assertFalse(D.takeoff_check(20.2, heights)["ok"])               # rise 0.14 s before the contact
        self.assertIsNone(D.takeoff_check(None, heights)["ok"])
        self.assertIsNone(D.takeoff_check(20.0, None)["ok"])
        rows = [{"k": k, "sim_s": 40.0 + 0.1 * k, "command": None if k == 3 else (3527, 0, 0, -1, 1.0 * k, 2.0, 0)}
                for k in range(4)]
        snaps = [{"k": k, "truth": {"fields": {"vx": 100 * k - 50, "vy": 200}}} for k in range(4)]
        v = D.velocities_at(rows, snaps, 40.35)
        self.assertEqual((v["k"], v["command_in_effect_from_k"], v["command_in_effect_ned_m_s"], v["truth_ned_m_s"],
                          v["new_command_ned_m_s"]), (3, 2, [2.0, 2.0], [2.5, 2.0], None))
        self.assertAlmostEqual(v["truth_minus_command_in_effect_m_s"], 0.5)
        v = D.velocities_at(rows, snaps, 40.15)
        self.assertEqual((v["k"], v["command_in_effect_from_k"], v["new_command_ned_m_s"]), (1, 0, [1.0, 2.0]))
        self.assertIsNone(D.velocities_at(rows, snaps, 40.05)["command_in_effect_ned_m_s"])
        self.assertIsNone(D.velocities_at(rows, snaps, 39.0))
        self.assertIsNone(D.velocities_at(rows, snaps, None))


class Unit(unittest.TestCase):
    """execute(): the approval gate, the fixed order and the stop rules, with the recording wrappers in place."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="agq_unit_"))
        self.drv = Driver("test_valid_member_records_everything")
        self.drv.setUp()
        self.git = {"head": "abc", "branch": "camonly/dev", "dirty": ""}
        self.env = ({"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "1",
                     "MKL_NUM_THREADS": "1", "affinity": [6, 7], "nice": 10}, frozenset({6, 7, 12, 13}))

    def tearDown(self):
        self.drv.tearDown()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def approval(self, **kw):
        return dict({"approved_by": "fixture", "approved_at": "now", "scope": D.SCOPE, "commit": "abc",
                     "skip_max_s": 0.15}, **kw)

    def go(self, approval, *, run=None, mode="ok", git_state=None, M=None, faults=lambda: 0):
        M = M or self.drv.M()
        AT = M.AT
        f = self.tmp / "f.pth"
        f.write_bytes(b"f")

        def recorder(d, scope):
            return AT.AttemptRecorder(d, scope=scope, root=self.tmp, sources=(), external=(), f_checkpoint=f,
                                      f_sha=AT.sha256_file(f), require_clean=False, faults=faults,
                                      host_check=lambda: (True, {}), budget_s=D.BUDGET_S,
                                      cores=frozenset(os.sched_getaffinity(0)), min_nice=-20)
        spec = PT.EpisodeDriver.spec(None, 12)
        specs = {(70, 0): spec, (205, 5): spec, (205, 2): spec}
        windows = {k: {"recorded_frames": 12} for k in specs}
        self.attempt = self.drv.allowed / "attempt"
        return D.execute(self.attempt, approval, root=ROOT, M=M, base_deps=self.drv.base(), specs=(specs, windows),
                         B=SI.STUB_B, policy=SI.StubPolicy(), run=run, git_state=git_state or (lambda r: self.git),
                         environment=self.env, make_recorder=recorder, prepare_package=M.TG.prepare,
                         contact_argv=(sys.executable, "-c", FAKE_CONTACTS, mode), timing=PT.FAST,
                         control=RELAXED)

    def ledger(self):
        return [json.loads(x) for x in (self.attempt / "ledger.jsonl").read_text().splitlines()]

    def test_approval_gate(self):
        for bad in ({"scope": "dev08 observed-prefix-v1"}, {"skip_max_s": 0.1}, {"skip_max_s": 0.15 * 1 + 0j},
                    {"commit": "other"}, {"approved_by": ""}):
            with self.assertRaises(PermissionError, msg=bad):
                self.go(self.approval(**bad))
        self.git = dict(self.git, dirty=" M tools/x.py")
        with self.assertRaises(PermissionError):
            self.go(self.approval())
        self.git = dict(self.git, dirty="")
        self.env = (dict(self.env[0], OMP_NUM_THREADS="4"), self.env[1])
        with self.assertRaises(PermissionError):
            self.go(self.approval())
        self.assertFalse(self.attempt.exists())
        self.assertEqual(D.main([]), 2)

    def test_complete_unit_in_fixed_order(self):
        seen = []

        def run(member, *a, **kw):
            seen.append((member["name"], kw["deps"]["link"].Inbox is not LK.Inbox, kw["gate_opts"]))
            r = R.run_episode(member, *a, **kw)
            log = kw["episode_dir"] / "rootfs" / "log" / "2026-11-06"
            log.mkdir(parents=True)
            (log / "fake.ulg").write_bytes(b"ULog fixture %s" % member["name"].encode())   # bound like a PX4 ULog
            return r
        status = self.go(self.approval(), run=run)
        self.assertEqual(status, "COMPLETE", (self.attempt / "final.json").read_text())
        names = [m["name"] for m in D.schedule()]
        self.assertEqual([s[0] for s in seen], names)
        self.assertTrue(all(s[1] and s[2] == {"skip_max_s": 0.15} for s in seen))
        led = self.ledger()
        self.assertEqual([(x["env"], x["arm"], x["geometry"]) for x in led],
                         [(5, "native", "native"), (5, "native", "training-box"), (2, "candidate", "training-box"),
                          (2, "candidate", "native")])
        self.assertTrue(all(x["recorder_problems"] == [] and x["operational_status"] == "VALID" for x in led))
        self.assertEqual([x["runtime_binding"]["files"]["model_sdf"]["sha256"] for x in led],
                         ["sdf-native", "sdf-training-box", "sdf-training-box", "sdf-native"])
        header = json.loads((self.attempt / "attempt.json").read_text())
        self.assertEqual((header["protocol"], header["amendment"]), (D.PROTOCOL, D.PREREG))
        self.assertEqual(len(header["schedule"]), 4)
        self.assertEqual(len(D.modules().AT.read_ledger(self.attempt)[0]), 4)
        fresh = load("sitl_dev08_attempt_fresh_agq", "tools/sitl_dev08_attempt.py")
        self.assertEqual((fresh.PROTOCOL, len(fresh.schedule())), ("observed-prefix-v1", 42))   # no leak

        # the fixed reading of this attempt: without a ULog the pairs are not evaluable
        self.assertTrue(all("checkpoints.jsonl" in x["member_files"]["sha256"] and x["scene_geometry"]["centers"]
                            for x in led))
        result = D.analyze(self.attempt, series=lambda ep: None)
        self.assertEqual([p["reading"] for p in result["pairs"]], ["NOT_EVALUABLE", "NOT_EVALUABLE"])
        self.assertTrue(all(m["evaluable"] for m in result["members"].values()))
        ev0 = result["members"][0]["events"]
        self.assertEqual(ev0["audit_problems_recomputed"], [])
        self.assertIsNotNone(ev0["first_rotor_bar_contact_s"])                # the fake subscriber's rotor-bar contact
        self.assertIsNotNone(ev0["velocities"]["at_first_bar_contact"])

        def series(ep, heights=False):                                        # synthetic ULog rates and heights
            rec = next(x for x in led if x["name"] == ep.name)
            t0 = rec["start_sim_time_s"]
            first = next(e for e in rec["bar_events"])
            rate = (first["intervals"][0][0] + 0.04) if rec["geometry"] == "native" else None
            rates = [(np.float64(t0 - 3.0 + 0.004 * i), (np.float32(0.0), np.float32(0.0), np.float32(
                6.0 if rate and t0 - 3.0 + 0.004 * i >= rate else 0.0))) for i in range(3000)]
            anchor = rec["contacts"]["floor"]["last_t_before_start"]
            h = [(np.float64(anchor - 2.0 + 0.02 * i), np.float32(-0.09 if anchor - 2.0 + 0.02 * i < anchor + 0.05
                                                                 else -0.2)) for i in range(200)] if heights else []
            return {"rates": rates, "heights": h, "dropouts": [], "path": "synthetic"}
        allowed = ("CONTACT_PERSISTS", "INCONCLUSIVE", "INCONCLUSIVE_PATHS_DIVERGED", "INCONCLUSIVE_NOT_EXPOSED",
                   "CONSISTENT_WITH_ROTOR_CONTACT", "DISTURBANCE_PERSISTS_WITHOUT_CONTACT")
        result = D.analyze(self.attempt, series=series)
        readings = [(p["scene"], p["reading"], p["cross_clock_order"], p.get("provisional_reading"))
                    for p in result["pairs"]]
        self.assertEqual([r[0] for r in readings], ["205/env5", "205/env2"])
        self.assertTrue(all(r[1:3] == ("NOT_READ_CLOCK_UNVERIFIED", "LIMITED_VALIDITY") for r in readings), readings)
        self.assertTrue(all(r[3] in allowed for r in readings), readings)   # the fake box members also "touch"
        result = D.analyze(self.attempt, series=lambda ep: series(ep, heights=True))
        readings = [(p["reading"], p["cross_clock_order"]) for p in result["pairs"]]
        self.assertTrue(all(r[1] == "SUPPORTED" and r[0] in allowed for r in readings), readings)
        self.assertEqual(result["members"][0]["events"]["first_rotor_bar"], 0)
        json.dumps(result, allow_nan=False)                                   # numpy-typed series serialise
        ckpt = self.attempt / led[1]["name"] / "checkpoints.jsonl"
        saved = ckpt.read_bytes()
        ckpt.write_bytes(saved + b"\n")                                      # a changed checkpoint file: not read
        changed = D.analyze(self.attempt, series=series)["members"][1]
        self.assertFalse(changed["evaluable"])
        self.assertIn("checkpoints.jsonl", changed["not_evaluable_because"][0])
        ckpt.write_bytes(saved)
        self.assertEqual(led[2]["member_files"]["ulogs"], ["rootfs/log/2026-11-06/fake.ulg"])
        ulg = self.attempt / led[2]["name"] / "rootfs/log/2026-11-06/fake.ulg"
        ulg.write_bytes(b"changed")                                           # a changed ULog: not read
        self.assertFalse(D.analyze(self.attempt, series=series)["members"][2]["evaluable"])
        broken = D.analyze(self.attempt, series=lambda ep: 1 / 0)             # an unreadable ULog: not evaluable
        self.assertEqual([p["reading"] for p in broken["pairs"]], ["NOT_EVALUABLE", "NOT_EVALUABLE"])
        self.assertIn("ZeroDivisionError", broken["members"][0]["events"]["ulog_error"])
        log = self.attempt / led[0]["name"] / "logs" / "contacts.log"
        log.write_text(log.read_text() + "time { sec: 999 nsec: 0 }\n")       # the raw log no longer matches
        self.assertFalse(D.analyze(self.attempt, series=series)["members"][0]["evaluable"])

    def test_stop_rules(self):
        self.assertEqual(self.go(self.approval(), mode="gaplate"), "ABORTED")
        final = json.loads((self.attempt / "final.json").read_text())
        self.assertEqual((final["reason"], final["recorded"], len(final["unrun"])), ("RECORDER_LOSS", 1, 3))
        shutil.rmtree(self.attempt)
        self.assertEqual(self.go(self.approval(), mode="nofloor"), "ABORTED")
        final = json.loads((self.attempt / "final.json").read_text())
        self.assertEqual((final["reason"], final["recorded"]), ("MEMBER_NOT_VALID", 1))
        self.assertIn("CONTACT_CONTROL", self.ledger()[0]["error"])
        shutil.rmtree(self.attempt)

        def slow(member, *a, **kw):
            return dict(R.run_episode(member, *a, **kw), real_time_factor=0.9)
        self.assertEqual(self.go(self.approval(), run=slow), "ABORTED")
        self.assertEqual(json.loads((self.attempt / "final.json").read_text())["reason"], "TIMING_GUARD")
        result = D.analyze(self.attempt, series=lambda ep: None)
        self.assertFalse(result["members"][0]["evaluable"])                   # its timing problem: not read
        self.assertIn("timing_problems", result["members"][0]["not_evaluable_because"][0])
        self.assertEqual(result["pairs"][0]["reading"], "NOT_EVALUABLE")
        shutil.rmtree(self.attempt)
        counts = iter([0, 0, 0, 1, 1, 1, 1, 1])                               # the fault count rises in member 1
        self.assertEqual(self.go(self.approval(), faults=lambda: next(counts)), "VOID_HOST_FAULT")
        result = D.analyze(self.attempt, series=lambda ep: None)
        self.assertTrue(result["void"])
        self.assertFalse(any(m["evaluable"] for m in result["members"].values()))
        self.assertEqual([p["reading"] for p in result["pairs"]], ["NOT_EVALUABLE", "NOT_EVALUABLE"])
        shutil.rmtree(self.attempt)
        counts = iter([0, 0, 0, 0, 0, 1])                                     # a rise seen only at the final read
        self.assertEqual(self.go(self.approval(), faults=lambda: next(counts)), "COMPLETE")
        result = D.analyze(self.attempt, series=lambda ep: None)
        self.assertTrue(result["void"])                                       # review r2 N3
        self.assertFalse(any(m["evaluable"] for m in result["members"].values()))
        shutil.rmtree(self.attempt)
        calls = []

        def changing(root):
            calls.append(1)
            return self.git if len(calls) < 3 else dict(self.git, head="def")   # changes during member 0
        self.assertEqual(self.go(self.approval(), git_state=changing), "ABORTED")
        self.assertEqual([x["operational_status"] for x in self.ledger()], ["SOURCE_CHANGED"])
        shutil.rmtree(self.attempt)

        def term(member, *a, **kw):
            r = R.run_episode(member, *a, **kw)
            if member["index"] == 1:
                signal.raise_signal(signal.SIGTERM)
            return r
        with self.assertRaises(R.RunnerTerminated):
            self.go(self.approval(), run=term)
        final = json.loads((self.attempt / "final.json").read_text())
        self.assertEqual((final["reason"], final["interrupted_member"]), ("RUNNER_EXCEPTION", D.schedule()[1]["name"]))
        self.assertTrue((self.attempt / D.schedule()[1]["name"] / "recording" / "recording.json").is_file())
        shutil.rmtree(self.attempt)

        def other_runtime(member, *a, **kw):                     # member 1 runs a different PX4 binary
            r = R.run_episode(member, *a, **kw)
            if member["index"] == 1:
                files = dict(r["runtime_binding"]["files"], px4_binary={"sha256": "other"})
                r = dict(r, runtime_binding=dict(r["runtime_binding"], files=files))
            return r
        self.assertEqual(self.go(self.approval(), run=other_runtime), "ABORTED")
        self.assertEqual([x["operational_status"] for x in self.ledger()], ["VALID", "BINDING_CHANGED"])
        shutil.rmtree(self.attempt)

        def wrong_model(member, *a, **kw):                       # the spawned SDF is not the package's
            r = R.run_episode(member, *a, **kw)
            files = dict(r["runtime_binding"]["files"], model_sdf={"sha256": "sdf-native"})
            return dict(r, runtime_binding=dict(r["runtime_binding"], files=files))
        self.assertEqual(self.go(self.approval(), run=wrong_model), "ABORTED")
        self.assertEqual([x["operational_status"] for x in self.ledger()], ["VALID", "BINDING_CHANGED"])

    def test_recorder_problems_negative_controls(self):
        rec = {"operational_status": "VALID", "observation_checks": 3, "commands_issued": 2}
        mask = R.historical_masks()[1]
        tap = D.Tap()
        tap.control = {"ok": True}
        tap.snapshots = [{}, {}, {}]
        send = {"call": "set_position_target_local_ned_send", "args": [0, 1, 1, 1, mask]}
        tap.sends = [dict(send), dict(send)]
        self.assertEqual(D.recorder_problems(rec, tap, [], {"files": {}}, mask), [])
        for change, want in ((lambda t: setattr(t, "sends", t.sends[:1]), "1 velocity sends"),
                             (lambda t: setattr(t, "sends", [dict(send), dict(send, error="OSError")]), "1 velocity"),
                             (lambda t: setattr(t, "snapshots", t.snapshots[:2]), "2 snapshots"),
                             (lambda t: setattr(t, "control", {"ok": False}), "CONTACT_CONTROL"),
                             (lambda t: setattr(t, "control", None), "CONTACT_CONTROL")):
            t = D.Tap()
            t.control, t.snapshots, t.sends = dict(tap.control), list(tap.snapshots), list(tap.sends)
            change(t)
            problems = D.recorder_problems(rec, t, [], {"files": {}}, mask)
            self.assertTrue(any(want in p for p in problems), (want, problems))
        self.assertEqual(D.recorder_problems(rec, tap, [], None, mask), ["RECORDER_LOSS: recording files not written"])
        self.assertEqual(D.recorder_problems(dict(rec, operational_status="ERROR"), D.Tap(), ["x"], {}, mask), ["x"])


class Launcher(unittest.TestCase):
    def setUp(self):
        self.L = load("sitl_agq_geometry_launch_t", "tools/sitl_agq_geometry_launch.py")
        self.tmp = Path(tempfile.mkdtemp(prefix="agq_launch_"))

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_envelope_matches_the_protocol(self):
        L = self.L
        self.assertEqual(L.SCOPE, D.SCOPE)
        self.assertEqual(L.COMMAND[:12], ["timeout", "-s", "TERM", "-k", "120", "1200", "nice", "-n", "10",
                                          "taskset", "-c", "6,7,12,13"])
        self.assertEqual((L.ATTEMPT, L.AS_LIMIT), (D.ATTEMPT_DIR, D.AS_LIMIT_MAX))
        self.assertEqual(L.COMMAND[13:16], ["-B", "tools/sitl_agq_geometry_diag.py", "--execute"])
        self.assertEqual(L.AS_LIMIT, 16 * 1024 ** 3)
        self.assertIn("tools/sitl_agq_geometry_launch.py", D.OWN_SOURCES)
        text = (ROOT / D.PREREG).read_text()
        self.assertIn("timeout -s TERM -k 120 1200 nice -n 10 taskset -c 6,7,12,13", text)
        for name in [m["name"] for m in D.schedule()]:
            self.assertIn(name, text)

    def test_execute_runs_only_inside_the_envelope(self):
        problems = D.envelope_problems(self.tmp / "attempt01", ppid=1, as_max=1)
        self.assertEqual(len(problems), 3, problems)
        import contextlib
        import io
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(D.main(["--execute", "--approval-file", str(self.tmp / "a.json"),
                                     "--attempt-dir", str(self.tmp / "attempt01")]), 1)   # refused before anything
        self.assertIn("envelope", out.getvalue())                             # by the envelope, not the approval
        self.assertFalse((self.tmp / "attempt01").exists())
        code = ("import importlib.util, json, sys; s = importlib.util.spec_from_file_location('d', sys.argv[1]); "
                "d = importlib.util.module_from_spec(s); s.loader.exec_module(d); "
                "print(json.dumps(d.envelope_problems(d.ATTEMPT_DIR)))")
        import resource
        import subprocess

        def limit():
            resource.setrlimit(resource.RLIMIT_AS, (D.AS_LIMIT_MAX, D.AS_LIMIT_MAX))
        soft, hard = resource.getrlimit(resource.RLIMIT_AS)
        if hard != resource.RLIM_INFINITY and hard < D.AS_LIMIT_MAX:
            limit = None                                                      # already tighter: inherited
        out = subprocess.run(["timeout", "60", sys.executable, "-B", "-c", code,
                              str(ROOT / "tools/sitl_agq_geometry_diag.py")],
                             capture_output=True, text=True, preexec_fn=limit)
        self.assertEqual(json.loads(out.stdout), [], out.stderr)              # the launcher's envelope passes

    def test_refuses_without_a_matching_approval(self):
        L = self.L
        L.APPROVAL, L.ATTEMPT, L.RUN = self.tmp / "approval.json", self.tmp / "attempt01", self.tmp / "run01"
        with self.assertRaisesRegex(SystemExit, "no approval"):
            L.main(["--launch"])
        L.APPROVAL.write_text(json.dumps({"commit": "0" * 40, "scope": "other", "skip_max_s": 0.1}))
        with self.assertRaisesRegex(SystemExit, "approval commit is not HEAD.*scope differs.*skip_max_s"):
            L.main(["--launch"])
        with self.assertRaisesRegex(SystemExit, "usage"):
            L.main(["--launch", "--again"])
        self.assertFalse(L.RUN.exists())
        L.ATTEMPT.mkdir()
        with self.assertRaisesRegex(SystemExit, "attempt directory already exists"):
            L.main(["--launch"])


if __name__ == "__main__":
    unittest.main()
