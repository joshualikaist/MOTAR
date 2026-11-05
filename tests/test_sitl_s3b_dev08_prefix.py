"""CPU fixtures for the dev08 observed-prefix runner integration (tools/sitl_s3b_dev08_prefix.py; motar-5md)."""
import importlib.util
import json
import math
import os
from pathlib import Path
import queue
import shutil
import socket
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REAL_EMPTY = Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/"
                  "worlds/empty.world")
FIXTURE_EMPTY = ("<?xml version='1.0'?>\n<sdf version='1.6'><world name='default'><physics type='ode'><ode><solver>"
                 "<iters>10</iters></solver></ode></physics></world></sdf>\n")


def load(name, rel):
    saved = list(sys.path)
    try:
        spec = importlib.util.spec_from_file_location(name, ROOT / rel)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path[:] = saved
    return mod


R = load("sitl_s3b_dev08_prefix_t", "tools/sitl_s3b_dev08_prefix.py")
OS_ = load("sitl_owned_session_i", "tools/sitl_owned_session.py")
LK = load("sitl_dev08_link_i", "tools/sitl_dev08_link.py")
ST = load("sitl_dev08_step_i", "tools/sitl_dev08_step.py")
AT = load("sitl_dev08_attempt_i", "tools/sitl_dev08_attempt.py")
LT = load("test_sitl_dev08_link_i", "tests/test_sitl_dev08_link.py")      # fresh_boot / AIRFRAME fixtures
SI = load("test_sitl_dev08_step_i", "tests/test_sitl_dev08_step.py")      # stub bridge / policy


class WorldWriter(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="dev08_world_"))
        rng = np.random.default_rng(4501)
        self.centers = np.column_stack([rng.uniform(0, 40, 70), rng.uniform(0, 40, 70), np.full(70, 1.5)])
        self.halves = np.column_stack([rng.uniform(0.2, 0.4, 70), rng.uniform(0.2, 0.4, 70), np.full(70, 1.5)])
        self.quats = np.tile([0.0, 0.0, 0.0, 1.0], (70, 1))
        self.p0, self.yaw0 = np.array([12.3, 7.8, 1.0]), 2.1

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def historical_text(self, base):
        H = load("hist_runner_world", "tools/sitl_s3b_closed_loop.py")
        (self.tmp / "empty.world").write_text(base)
        H.WORLDS = self.tmp                       # the historical writer reads and writes here
        H.write_world("w", self.centers, self.halves, self.p0, self.yaw0)
        return (self.tmp / "w.world").read_text()

    def test_identical_to_the_historical_writer(self):
        bases = [FIXTURE_EMPTY] + ([REAL_EMPTY.read_text()] if REAL_EMPTY.is_file() else [])
        for base in bases:
            self.assertEqual(R.world_text(base, self.centers, self.halves, self.quats, self.p0, self.yaw0),
                             self.historical_text(base))

    def test_orientation_and_template_guards(self):
        q = self.quats.copy()
        q[3] = [0.0, 0.0, 0.2588, 0.9659]                     # a yawed bar: the historical writer would drop it
        with self.assertRaises(ValueError):
            R.world_text(FIXTURE_EMPTY, self.centers, self.halves, q, self.p0, self.yaw0)
        q[3] = [0.0, 0.0, 0.0, -1.0]                          # the same identity rotation: accepted
        R.world_text(FIXTURE_EMPTY, self.centers, self.halves, q, self.p0, self.yaw0)
        with self.assertRaises(ValueError):
            R.world_text(FIXTURE_EMPTY.replace("<iters>10</iters>", ""), self.centers, self.halves, self.quats,
                         self.p0, self.yaw0)

    def test_main_refuses_execution(self):
        self.assertEqual(R.main(), 2)
        self.assertFalse(R.STATUS["execution_supported"])


class FakeMsg:
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


class FakePX4Conn:
    """A pymavlink-shaped PX4: 50 Hz position/attitude/truth on PX4's clock (real time), 1 Hz heartbeat,
    the parameter protocol, offboard/arm commands, and position or velocity setpoint following."""
    LAT, LON, ALT0 = 47.397742, 8.545594, 488.0
    OFFSET = (0.1, -0.05)                     # PX4's estimate minus truth, NED metres

    def __init__(self, params, *, drop_echo=(), stall_after_s=None, never_arm=False):
        self.params = dict(params)
        self.drop_echo, self.stall_after_s, self.never_arm = set(drop_echo), stall_after_s, never_arm
        self.q = queue.Queue()
        self.mav = self
        self.lock = threading.Lock()
        self.truth = [0.0, 0.0, 0.0]          # NED
        self.yaw = 0.2
        self.armed, self.mode = False, 0
        self.sp = None
        self.setpoints = []
        self.velocity_setpoints = 0
        self.stop = threading.Event()
        self.t0 = time.monotonic()
        self.th = threading.Thread(target=self.run, daemon=True)
        self.th.start()

    def put(self, typ, fields):
        self.q.put(FakeMsg(typ, (1, 1), fields))

    def run(self):
        n = 0
        while not self.stop.is_set():
            target = self.t0 + n * 0.02
            time.sleep(max(0.0, target - time.monotonic()))
            if self.stall_after_s is not None and n * 0.02 > self.stall_after_s:
                continue
            with self.lock:
                self.advance(0.02)
                est = (self.truth[0] + self.OFFSET[0], self.truth[1] + self.OFFSET[1], self.truth[2])
                ms = 20 * n
                self.put("LOCAL_POSITION_NED", {"time_boot_ms": ms, "x": est[0], "y": est[1], "z": est[2],
                                                "vx": 0.0, "vy": 0.0, "vz": 0.0})
                self.put("ATTITUDE", {"time_boot_ms": ms, "roll": 0.0, "pitch": 0.0, "yaw": self.yaw,
                                      "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0})
                lat = self.LAT + math.degrees(self.truth[0] / 6371000.0)
                lon = self.LON + math.degrees(self.truth[1] / (6371000.0 * math.cos(math.radians(self.LAT))))
                self.put("HIL_STATE_QUATERNION", {"time_usec": 0,             # as PX4 v1.14.3 sends it
                                                  "attitude_quaternion": [math.cos(self.yaw / 2), 0.0, 0.0,
                                                                          math.sin(self.yaw / 2)],
                                                  "lat": int(round(lat * 1e7)), "lon": int(round(lon * 1e7)),
                                                  "alt": int(round((self.ALT0 - self.truth[2]) * 1000)),
                                                  "vx": 0, "vy": 0, "vz": 0})
                if n % 50 == 0:
                    self.put("HEARTBEAT", {"base_mode": 128 if self.armed else 0, "custom_mode": self.mode << 16})
            n += 1

    def advance(self, dt):
        if not self.armed or self.sp is None:
            return
        mask, x, y, z, vx, vy, wz = self.sp
        self.truth[2] = z                                         # altitude held by the z setpoint
        if mask == R.historical_masks()[0]:                       # position: first-order towards the setpoint
            tn, te = x - self.OFFSET[0], y - self.OFFSET[1]
            self.truth[0] += (tn - self.truth[0]) * min(1.0, dt / 0.05)
            self.truth[1] += (te - self.truth[1]) * min(1.0, dt / 0.05)
        else:
            self.truth[0] += vx * dt
            self.truth[1] += vy * dt
            self.yaw += wz * dt

    # pymavlink sends
    def param_request_read_send(self, ts, tc, name, index):
        name = name.decode()
        with self.lock:
            if name in self.params:
                self.put("PARAM_VALUE", {"param_id": name, "param_value": self.params[name], "param_type": 9,
                                         "param_index": 0, "param_count": len(self.params)})

    def param_set_send(self, ts, tc, name, value, ptype):
        name = name.decode()
        with self.lock:
            self.params[name] = LK.f32(value)
            if name not in self.drop_echo:
                self.put("PARAM_VALUE", {"param_id": name, "param_value": self.params[name], "param_type": 9,
                                         "param_index": 0, "param_count": len(self.params)})

    def command_long_send(self, ts, tc, command, confirmation, *p):
        with self.lock:
            if command == R.MAV_CMD_DO_SET_MODE and p[1] == 6:
                self.mode = 6
            elif command == R.MAV_CMD_COMPONENT_ARM_DISARM and p[0] == 1 and not self.never_arm:
                self.armed = True

    def set_position_target_local_ned_send(self, t, ts, tc, frame, mask, x, y, z, vx, vy, vz, ax, ay, az, yaw,
                                           yaw_rate):
        with self.lock:
            self.sp = (mask, x, y, z, vx, vy, yaw_rate)
            self.setpoints.append(self.sp)
            if mask != R.historical_masks()[0]:
                self.velocity_setpoints += 1

    def recv_match(self, blocking=True, timeout=0.5):
        try:
            return self.q.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        self.stop.set()
        self.th.join(timeout=2)


FAKE_PROC = "import time\nwhile True:\n    time.sleep(0.2)\n"
FAST = {"boot_wait_s": 0.05, "heartbeat_s": 5.0, "streams_s": 5.0, "settle_s": 0.1, "hold_pre_s": 0.1,
        "hold_mode_s": 0.1, "hold_arm_s": 0.3, "hold_at_s": 0.2, "hold_at_rounds": 3, "after_s": 0.05}


def free_ports():
    out = []
    for proto, kind in (("udp", socket.SOCK_DGRAM), ("tcp", socket.SOCK_STREAM)):
        s = socket.socket(socket.AF_INET, kind)
        s.bind(("0.0.0.0", 0))
        out.append((proto, s.getsockname()[1]))
        s.close()
    return tuple(out)


class EpisodeDriver(unittest.TestCase):
    """The real owned session (fake simulator processes), link, step and recorder against a fake PX4."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="dev08_driver_"))
        self.allowed = self.tmp / "attempts"
        self.allowed.mkdir()
        self.ports = free_ports()
        self.conns = []
        self.fake = {}

    def tearDown(self):
        for c in self.conns:
            c.close()
        shutil.rmtree(self.tmp, ignore_errors=True)

    def deps(self):
        def session(episode_dir):
            return OS_.OwnedSession(episode_dir, allowed_root=self.allowed, lock_path=self.tmp / "lock",
                                    ports=self.ports, foreign_names=("motar_fake_gz",), forbidden_roots=(),
                                    deadline_s=60.0, kill_grace_s=0.5, sample_s=0.1, include_self_rss=False)

        def plan(sess, world, name):
            argv = [sys.executable, "-c", FAKE_PROC]
            env = dict(os.environ, PYTHONNOUSERSITE="1")
            return {"world_path": world, "gzserver": {"argv": argv, "cwd": sess.episode_dir, "env": env},
                    "spawn": {"argv": [sys.executable, "-c", "print('spawned')"], "cwd": sess.episode_dir,
                              "env": env},
                    "px4": {"argv": argv, "cwd": sess.rootfs, "env": env}}

        def connect():
            c = FakePX4Conn(LT.fresh_boot(), **self.fake)
            self.conns.append(c)
            return c

        return {"session": session, "empty_world": lambda: FIXTURE_EMPTY, "plan": plan,
                "binding": lambda plan: {"fixture": True},
                "launch": lambda sess, plan: OS_.launch_px4_gazebo(sess, plan, spawn_retry_s=0.05),
                "connect": connect, "fresh_defaults": lambda: LT.EXPECT,
                "link": LK, "step": ST}

    def spec(self, frames, target=(35.0, 35.0, 1.0)):
        return {"centers": np.array([[30.0, 5.0, 1.5]]), "halves": np.array([[0.1, 0.1, 1.5]]),
                "quats": np.array([[0.0, 0.0, 0.0, 1.0]]), "p0": np.array([5.0, 6.0, 1.0]), "yaw0": 0.7,
                "tpos": np.tile(np.asarray(target, float), (frames, 1)), "bounds_min": [0.0, 0.0, 0.0],
                "bounds_max": [40.0, 40.0, 3.0]}

    def run_member(self, member, frames=12, arm="native"):
        params = AT.load_arm_params(arm) if hasattr(AT, "load_arm_params") else \
            load("cfg_i", "tools/sitl_s3b_dev08.py").prepare(arm=arm)["selected_params"]
        return R.run_episode(member, self.spec(frames), frames, params, deps=self.deps(), B=SI.STUB_B,
                             policy=SI.StubPolicy(), episode_dir=self.allowed / member["name"], timing=FAST)

    def test_valid_members_feed_the_recorder(self):
        plan = AT.schedule()
        rec = AT.AttemptRecorder(self.tmp / "attempt", scope={"approval": "fixture"}, root=self.tmp,
                                 sources=(), external=(), f_checkpoint=None, require_clean=False,
                                 host_check=lambda: (True, {}))
        with mock.patch.object(AT, "git_state", lambda root: {"head": "x", "branch": "x", "dirty": ""}), \
                mock.patch.object(AT, "environment_problems", lambda env, cores, min_nice: []):
            with self.assertRaises(AT.AttemptError):                  # Policy F must be bound
                rec.open()
            f = self.tmp / "f.pth"
            f.write_bytes(b"f")
            rec = AT.AttemptRecorder(self.tmp / "attempt", scope={"approval": "fixture"}, root=self.tmp,
                                     sources=(), external=(), f_checkpoint=f, f_sha=AT.sha256_file(f),
                                     require_clean=False, faults=lambda: 0, host_check=lambda: (True, {}))
            rec.open()
        for member in plan[:3]:
            r = self.run_member(member, arm=member["arm"])
            self.assertEqual(r["operational_status"], "VALID", r.get("error"))
            self.assertIn(r["outcome"], ("capture", "crash", "CENSORED_TRACE_END"))
            k = r["final_frame_index"]
            self.assertEqual((r["observation_checks"], r["commands_issued"]), (k + 1, k))
            self.assertEqual(self.conns[-1].velocity_setpoints, k)    # no command at the final checkpoint
            self.assertEqual(r["start_check"]["armed"], True)
            want = load("cfg_i2", "tools/sitl_s3b_dev08.py").prepare(arm=member["arm"])["selected_params"]
            for n, v in want.items():
                self.assertEqual(r["parameters"]["effective"][n], LK.f32(v))
            receipt = json.loads(Path(r["session_receipt"]["path"]).read_text())
            self.assertEqual(receipt["cleanup"]["leaked"], [])
            rows = (self.allowed / member["name"] / "checkpoints.jsonl").read_text().splitlines()
            self.assertEqual(len(rows), k + 1)
            rec.append(r)
        self.assertIsNone(rec.status)
        self.assertEqual(rec.next, 3)

    def test_member_failures_are_recorded_not_raised(self):
        cases = [("echo", {"drop_echo": ("MPC_XY_VEL_P_ACC",)}, "ERROR", "PARAM_ECHO_MISSING"),   # a native override
                 ("arm", {"never_arm": True}, "HARNESS_INVALID", None),
                 ("stall", {"stall_after_s": 2.5}, "ERROR", None)]
        for i, (name, fake, status, code) in enumerate(cases):
            self.fake = fake
            member = dict(AT.schedule()[0], name="fail_%s" % name)
            r = self.run_member(member, frames=40)
            self.assertEqual(r["operational_status"], status, (name, r.get("error")))
            if code:
                self.assertEqual(r["error_code"], code)
            receipt = json.loads(Path(r["session_receipt"]["path"]).read_text())
            self.assertEqual(receipt["cleanup"]["leaked"], [], name)
        self.assertIn(r["error_code"], ("CLOCK_STALL", "STALE_ATT", "STALE_TRUTH"))


class Execute(unittest.TestCase):
    """execute()'s approval gate and member loop, with run_episode replaced by an instant fake."""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="dev08_execute_"))
        self.head = {"head": "abc123", "branch": "camonly/dev", "dirty": ""}
        self.git = dict(self.head)
        f = self.tmp / "f.pth"
        f.write_bytes(b"f")
        self.f = f
        scene = EpisodeDriver.spec(None, 5)
        self.specs = ({(b, e): dict(scene, env=e) for b in (70, 205) for e in range(10)},
                      {(b, e): {"recorded_frames": 208 if (b, e) == (70, 5) else 600}
                       for b in (70, 205) for e in range(10)})
        self.B = type("B", (), {"F_CKPT": str(f), "Episode": SI.StubEpisode})
        self.calls = []
        self.env_ok = ({"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "affinity": [6, 7, 12, 13], "nice": 10,
                        "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"}, AT.CORES)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def approval(self, **kw):
        return dict({"approved_by": "fixture", "approved_at": "2026-11-05", "scope": R.SCOPE, "commit": "abc123",
                     "skip_max_s": 0.05}, **kw)

    def recorder(self, d, scope):
        return AT.AttemptRecorder(d, scope=scope, root=self.tmp, sources=(), external=(), host_check=lambda: (True, {}),
                                  f_checkpoint=self.f,
                                  f_sha=AT.sha256_file(self.f), require_clean=False, faults=lambda: 0,
                                  cores=frozenset(os.sched_getaffinity(0)), min_nice=os.nice(0))

    def run_fake(self, member, spec, frames, params, **kw):
        self.calls.append((member["name"], frames, dict(params)))
        k = frames - 1
        return {"phase": member["phase"], "bars": member["bars"], "env": member["env"], "arm": member["arm"],
                "recorded_frames": frames, "runtime_binding": {"files": {"px4_binary": {"sha256": "a"},
                                                                         "world": {"sha256": member["name"]}}},
                "operational_status": "VALID", "outcome": "CENSORED_TRACE_END", "final_frame_index": k,
                "observation_checks": k + 1, "commands_issued": k, "start_sim_time_s": 40.0,
                "final_sim_time_s": 40.0 + k * 0.1, "scheduled_elapsed_s": round(k * 0.1, 10)}

    def go(self, approval, name="attempt", run=None, environment=None):
        with mock.patch.dict(os.environ, {"CUDA_VISIBLE_DEVICES": "", "PYTHONNOUSERSITE": "1", "OMP_NUM_THREADS": "1",
                                          "MKL_NUM_THREADS": "1"}):
            return R.execute(self.tmp / name, approval, deps={}, specs=self.specs, B=self.B, policy=SI.StubPolicy(),
                             run=run or self.run_fake, root=ROOT, make_recorder=self.recorder,
                             git_state=lambda root: dict(self.git), environment=environment or self.env_ok)

    def test_approval_gate(self):
        for bad in (None, {}, self.approval(scope="something else"), self.approval(commit="other"),
                    self.approval(approved_by=""), self.approval(skip_max_s=None), self.approval(skip_max_s=0.05 * 20),
                    self.approval(skip_max_s="0.05")):
            with self.assertRaises(PermissionError):
                self.go(bad)
        self.git["dirty"] = " M tools/x.py"
        with self.assertRaises(PermissionError):
            self.go(self.approval())
        self.git["dirty"] = ""
        bad_env = (dict(self.env_ok[0], CUDA_VISIBLE_DEVICES="0"), AT.CORES)
        with self.assertRaises(PermissionError):                       # checked before the bridge is touched
            self.go(self.approval(), environment=bad_env)
        self.assertFalse((self.tmp / "attempt").exists())
        self.assertEqual(self.calls, [])

    def test_complete_unit_order_and_parameters(self):
        self.assertEqual(self.go(self.approval()), "COMPLETE")
        self.assertEqual([c[0] for c in self.calls], [m["name"] for m in AT.schedule()])
        header = json.loads((self.tmp / "attempt" / "attempt.json").read_text())
        self.assertEqual(header["scope"]["approval"]["skip_max_s"], 0.05)
        cfg = load("cfg_exec_t", "tools/sitl_s3b_dev08.py")
        for name, frames, params in self.calls:
            arm = name.rsplit("_", 1)[1]
            self.assertEqual(params, cfg.prepare(arm=arm)["selected_params"])
            self.assertEqual(frames, 208 if "70bars_env5_" in name else 600)
        mains = AT.main_records(self.tmp / "attempt")
        windows = {(b, e): load("pc_exec_t", "tools/sitl_dev08_prefix_contract.py").frame_window(
            208 if (b, e) == (70, 5) else 600) for b in (70, 205) for e in range(10)}
        load("pc_exec_t2", "tools/sitl_dev08_prefix_contract.py").summarize_complete_pairs(mains, windows)

    def test_source_change_and_invalid_member_stop_the_unit(self):
        def run_then_change(member, *a, **k):
            if member["index"] == 3:
                self.git["head"] = "def456"
            return self.run_fake(member, *a, **k)
        self.assertEqual(self.go(self.approval(), "changed", run_then_change), "ABORTED")
        lines, final = AT.read_ledger(self.tmp / "changed")
        self.assertEqual((final["reason"], final["recorded"]), ("MEMBER_NOT_VALID", 4))   # caught after member 3
        self.assertEqual(lines[-1]["operational_status"], "SOURCE_CHANGED")
        self.git = dict(self.head)
        self.calls = []

        def change_in_last(member, *a, **k):
            if member["index"] == 41:
                self.git["head"] = "def456"
            return self.run_fake(member, *a, **k)
        self.assertEqual(self.go(self.approval(), "last", change_in_last), "ABORTED")   # not COMPLETE
        self.git = dict(self.head)
        self.calls = []

        def run_invalid(member, *a, **k):
            r = self.run_fake(member, *a, **k)
            if member["index"] == 1:
                r["operational_status"] = "ERROR"
            return r
        self.assertEqual(self.go(self.approval(), "invalid", run_invalid), "ABORTED")
        self.assertEqual(len(self.calls), 2)
        final = json.loads((self.tmp / "invalid" / "final.json").read_text())
        self.assertEqual(final["reason"], "SMOKE_FAILED")


class ExecuteStops(Execute):
    def test_signal_and_binding_change_stop_the_unit(self):
        import signal as sig

        def run_then_term(member, *a, **k):
            r = self.run_fake(member, *a, **k)
            if member["index"] == 2:
                os.kill(os.getpid(), sig.SIGTERM)
            return r
        before = sig.getsignal(sig.SIGTERM)
        with self.assertRaises(R.RunnerTerminated):
            self.go(self.approval(), "term", run_then_term)
        self.assertIs(sig.getsignal(sig.SIGTERM), before)              # handlers restored
        final = json.loads((self.tmp / "term" / "final.json").read_text())
        self.assertEqual(final["reason"], "RUNNER_EXCEPTION")

        def run_binding(member, *a, **k):
            r = self.run_fake(member, *a, **k)
            if member["index"] == 4:
                r["runtime_binding"]["files"]["px4_binary"]["sha256"] = "b"
            return r
        self.calls = []
        self.assertEqual(self.go(self.approval(), "binding", run_binding), "ABORTED")
        lines, final = AT.read_ledger(self.tmp / "binding")
        self.assertEqual(lines[-1]["operational_status"], "BINDING_CHANGED")
        self.assertEqual(final["recorded"], 5)

    def test_an_interrupted_member_is_named(self):
        def run_interrupted(member, *a, **k):
            if member["index"] == 2:
                raise R.RunnerTerminated("signal during member 2")
            return self.run_fake(member, *a, **k)
        with self.assertRaises(R.RunnerTerminated):
            self.go(self.approval(), "interrupted", run_interrupted)
        final = json.loads((self.tmp / "interrupted" / "final.json").read_text())
        self.assertEqual((final["interrupted_member"], final["recorded"]), (AT.schedule()[2]["name"], 2))
        self.assertIn(AT.schedule()[2]["name"], final["unrun"])

    test_approval_gate = test_complete_unit_order_and_parameters = None
    test_source_change_and_invalid_member_stop_the_unit = None


class SlowEpisode(SI.StubEpisode):
    def __init__(self, *a, **k):
        time.sleep(0.1)                                   # the real bridge: 40-99 ms (review r1 M1)
        super().__init__(*a, **k)


class SlowFirstPolicy(SI.StubPolicy):
    def __init__(self):
        self.calls = 0

    def act(self, obs):
        self.calls += 1
        if self.calls == 1:
            time.sleep(0.5)                               # first call in a process (review r1 M1)
        return super().act(obs)


class Timing(EpisodeDriver):
    """Review r1 M1: Episode construction and the first policy call must not delay the checkpoints."""

    def test_slow_construction_and_first_call(self):
        B = type("B", (), {"Episode": SlowEpisode})
        member = dict(AT.schedule()[0], name="slow_nowarm")
        r = R.run_episode(member, self.spec(12), 12, load("cfg_t1", "tools/sitl_s3b_dev08.py").prepare(
            arm="native")["selected_params"], deps=self.deps(), B=B, policy=SlowFirstPolicy(),
            episode_dir=self.allowed / member["name"], timing=FAST)
        self.assertEqual((r["operational_status"], r["error_code"]), ("ERROR", "RUNNER_LATE"))   # negative control
        policy = SlowFirstPolicy()
        w = R.warm_up(B, policy, self.spec(12))
        self.assertGreater(w["warm_up_s"], 0.4)
        member = dict(AT.schedule()[0], name="slow_warm")
        r = R.run_episode(member, self.spec(12), 12, load("cfg_t2", "tools/sitl_s3b_dev08.py").prepare(
            arm="native")["selected_params"], deps=self.deps(), B=B, policy=policy,
            episode_dir=self.allowed / member["name"], timing=FAST)
        self.assertEqual(r["operational_status"], "VALID", r.get("error"))
        rows = [json.loads(x) for x in (self.allowed / member["name"] / "checkpoints.jsonl").read_text().splitlines()]
        self.assertTrue(all("pos_lateness_s" in x and "truth_seq" in x for x in rows))
        self.assertIn("ekf_minus_truth_xy_m", r["start_check"])
        self.assertFalse(r["rx_thread_alive_after_join"])

    test_valid_members_feed_the_recorder = test_member_failures_are_recorded_not_raised = None


if __name__ == "__main__":
    unittest.main()
