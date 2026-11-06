"""CPU tests for tools/sitl_s3b_formal.py (S3b formal v1). No SITL, PX4 or GPU is started."""
import importlib.util
import json
import pathlib
import shutil
import sys
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


SF = _load("sitl_s3b_formal_under_test", TOOLS / "sitl_s3b_formal.py")


def zero_row(env, actions, ep=0):
    return {"env_index": env, "env_episode_index": ep, "actions": actions}


def make_g8(tmp, zero70, zero205, full70, full205):
    for kind, bars, rows, name in (("zero", 70, zero70, "kroc2027_episodes.jsonl"),
                                   ("zero", 205, zero205, "kroc2027_episodes.jsonl"),
                                   ("full", 70, full70, "episodes.jsonl"), ("full", 205, full205, "episodes.jsonl")):
        d = tmp / kind / (SF.CELL % bars)
        d.mkdir(parents=True, exist_ok=True)
        (d / name).write_text("".join(json.dumps(r) + "\n" for r in rows))
    return tmp


class TestSchedule(unittest.TestCase):
    def test_eligibility_boundary(self):
        rows = [zero_row(0, 600), zero_row(1, 599), zero_row(2, 601), zero_row(3, 600, ep=1), zero_row(4, 66)]
        self.assertEqual(SF.eligible_envs(rows), [0, 2])           # positive and negative cases, non-empty

    def test_blocks_order_and_truncation(self):
        with tempfile.TemporaryDirectory() as t:
            g8 = make_g8(pathlib.Path(t), [zero_row(e, 600 if e % 3 else 100) for e in range(20)],
                         [zero_row(e, 600) for e in range(5)], [], [])
            sched = SF.schedule(g8, (("A", 70, None, False), ("B", 205, 3, False), ("C", 70, 2, True)))
        a = [s["env"] for s in sched if s["block"] == "A"]
        self.assertEqual(a, [e for e in range(20) if e % 3])
        self.assertEqual([s["env"] for s in sched if s["block"] == "B"], [0, 1, 2])
        self.assertEqual([s["env"] for s in sched if s["block"] == "C"], a[:2])
        self.assertTrue(all(s["repeat"] for s in sched if s["block"] == "C"))
        self.assertEqual([s["block"] for s in sched], ["A"] * len(a) + ["B"] * 3 + ["C"] * 2)

    @unittest.skipUnless((SF.G8 / "zero").is_dir(), "G8 r2 raw inputs not on this host")
    def test_real_inputs(self):
        self.assertEqual(SF.input_problems(), [])
        sched = SF.schedule()
        self.assertEqual({b: sum(1 for s in sched if s["block"] == b) for b in "ABC"}, {"A": 122, "B": 30, "C": 10})
        isaac = SF.isaac_outcomes(SF.G8, 70)
        a = [isaac[s["env"]] for s in sched if s["block"] == "A"]
        self.assertEqual((a.count("capture"), a.count("crash")), (121, 1))

    def test_input_pin_detects_change(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            d = t / "zero" / (SF.CELL % 70)
            d.mkdir(parents=True)
            (d / "receipt.json").write_text("{}")
            good = {"zero/70/receipt.json": SF.sha256_file(d / "receipt.json")}
            self.assertEqual(SF.input_problems(t, good), [])
            (d / "receipt.json").write_text("{ }")
            self.assertEqual(len(SF.input_problems(t, good)), 1)
            self.assertEqual(len(SF.input_problems(t, {"zero/205/receipt.json": "0" * 64})), 1)


class TestApproval(unittest.TestCase):
    HEAD = "a" * 40

    def ok(self):
        return {"approved_by": "user", "approved_at": "2026-11-06T16:00:00+09:00", "scope": SF.SCOPE,
                "commit": self.HEAD}

    def test_valid(self):
        self.assertEqual(SF.approval_problems(self.ok(), self.HEAD), [])
        short = dict(self.ok(), commit=self.HEAD[:7])
        self.assertEqual(SF.approval_problems(short, self.HEAD), [])

    def test_refusals(self):
        for change, reason in ((dict(scope="other"), "scope"), (dict(commit="b" * 40), "commit"),
                               (dict(approved_by=""), "approved_by")):
            p = SF.approval_problems(dict(self.ok(), **change), self.HEAD)
            self.assertTrue(any(reason in x for x in p), (change, p))
        self.assertTrue(SF.approval_problems([], self.HEAD))
        self.assertTrue(SF.approval_problems(self.ok(), ""))


class TestEpisodeRules(unittest.TestCase):
    def test_params(self):
        want = {"A": 2.5, "B": 0.013}
        self.assertEqual(SF.params_problems({"A": 2.5, "B": 0.013000000001}, want), [])
        self.assertEqual(len(SF.params_problems({"A": 2.0, "B": 0.013}, want)), 1)
        self.assertEqual(len(SF.params_problems({"A": 2.5}, want)), 1)

    def test_validity(self):
        base = {"outcome": "capture", "param_problems": [], "late_max_s": 0.02}
        self.assertEqual(SF.episode_valid(base), (True, None))
        self.assertEqual(SF.episode_valid(dict(base, late_max_s=0.15)), (True, None))
        self.assertEqual(SF.episode_valid(dict(base, late_max_s=0.1501)), (False, "LATE"))
        self.assertEqual(SF.episode_valid(dict(base, late_max_s=None)), (False, "LATE"))
        self.assertEqual(SF.episode_valid(dict(base, param_problems=["x"])), (False, "PARAM_MISMATCH"))
        self.assertEqual(SF.episode_valid(dict(base, outcome="HARNESS_INVALID")), (False, "HARNESS_INVALID"))

    def test_delay(self):
        d0 = SF.CommandDelay(0)
        c = (1.0, 2.0, 3.0)
        self.assertIs(d0(c), c)
        d1 = SF.CommandDelay(1)
        self.assertEqual([d1((float(i), 0.0, 0.0))[0] for i in range(1, 5)], [0.0, 1.0, 2.0, 3.0])


def rows_for(block, bars, outcomes, valid=None, start=0):
    valid = valid or [True] * len(outcomes)
    return [{"block": block, "bars": bars, "env": start + i, "outcome": o, "valid": v,
             "invalid_reason": None if v else "LATE"} for i, (o, v) in enumerate(zip(outcomes, valid))]


def attempt_for(rows):
    return {"schedule": [{"block": r["block"], "env": r["env"]} for r in rows], "kernel_faults_before": 0,
            "git": {"head": "h", "dirty": ""}}


def final_ok(status="COMPLETE"):
    return {"status": status, "reason": None, "kernel_faults_after": 0, "git_after": {"head": "h", "dirty": ""}}


class TestReading(unittest.TestCase):
    def isaac(self, n=122, crashes=1):
        return {e: ("crash" if e < crashes else "capture") for e in range(n)}

    def read(self, rows, final=None, attempt=None):
        isaac = {70: self.isaac(), 205: {}}
        return SF.reading(final or final_ok(), rows, isaac, attempt if attempt is not None else attempt_for(rows))

    def test_margin_boundary(self):
        # 7 SITL crashes vs 1 Isaac crash on 122 envs: +4.918 pp -> feasible; 8 crashes: +5.738 pp -> not feasible
        r7 = self.read(rows_for("A", 70, ["crash"] * 7 + ["capture"] * 115))
        r8 = self.read(rows_for("A", 70, ["crash"] * 8 + ["capture"] * 114))
        self.assertEqual(r7["reading"], "FEASIBLE_IN_SITL")
        self.assertAlmostEqual(r7["blocks"]["A"]["crash_diff_pp"], 600 / 122, places=9)
        self.assertEqual(r8["reading"], "NOT_FEASIBLE_IN_SITL")
        ci = r7["block_A_reported"]["sitl_crash_pct_ci95"]
        self.assertTrue(ci is None or ci[0] < 100 * 7 / 122 < ci[1])

    def test_void(self):
        rows = rows_for("A", 70, ["capture"] * 122)
        self.assertEqual(self.read(rows, final=dict(final_ok("ABORTED"), reason="x"))["reading"], "VOID")
        few = rows_for("A", 70, ["capture"] * 122, valid=[True] * 109 + [False] * 13)
        self.assertEqual(self.read(few)["reading"], "VOID")
        enough = rows_for("A", 70, ["capture"] * 122, valid=[True] * 110 + [False] * 12)
        r = self.read(enough)
        self.assertEqual(r["reading"], "FEASIBLE_IN_SITL")
        w = r["block_A_reported"]["worst_case_invalid_as_crash"]
        self.assertAlmostEqual(w["sitl_crash_pct"], 100 * 12 / 122)

    def test_integrity_voids(self):
        rows = rows_for("A", 70, ["capture"] * 122)
        self.assertEqual(SF.integrity_problems(attempt_for(rows), final_ok(), rows), [])
        swapped = list(rows)
        swapped[0], swapped[1] = swapped[1], swapped[0]
        self.assertEqual(self.read(swapped, attempt=attempt_for(rows))["reading"], "VOID")
        self.assertEqual(self.read(rows, final=dict(final_ok(), kernel_faults_after=1))["reading"], "VOID")
        self.assertEqual(self.read(rows, final=dict(final_ok(), git_after={"head": "h", "dirty": "M x"}))["reading"],
                         "VOID")
        short = attempt_for(rows + rows_for("B", 205, ["capture"]))
        self.assertEqual(self.read(rows, attempt=short)["reading"], "VOID")

    def test_stopped_after_a_reads_block_a(self):
        rows = rows_for("A", 70, ["capture"] * 122) + rows_for("B", 205, ["ERROR"] * 3, valid=[False] * 3)
        att = attempt_for(rows + rows_for("B", 205, ["capture"] * 5))
        self.assertEqual(self.read(rows, final=final_ok("STOPPED_AFTER_A"), attempt=att)["reading"], "FEASIBLE_IN_SITL")
        short = rows_for("A", 70, ["capture"] * 100)
        self.assertEqual(self.read(short, final=final_ok("STOPPED_AFTER_A"), attempt=att)["reading"], "VOID")

    def test_analyze_without_final_is_void(self):
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            rows = rows_for("A", 70, ["capture"] * 3)
            (t / "attempt.json").write_text(json.dumps(attempt_for(rows)))
            (t / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
            old = SF.isaac_outcomes, SF.input_problems
            SF.isaac_outcomes = lambda g8, bars: self.isaac()
            SF.input_problems = lambda *a, **k: []
            try:
                out = SF.analyze(t)
            finally:
                SF.isaac_outcomes, SF.input_problems = old
        self.assertEqual(out["reading"], "VOID")
        self.assertIn("NO_FINAL_RECORD", out["void_reason"])

    def test_repeat_pairs(self):
        rows = rows_for("A", 70, ["capture"] * 122) + rows_for("C", 70, ["capture", "crash", "capture"])
        r = self.read(rows)
        self.assertEqual((r["repeat_C"]["pairs"], r["repeat_C"]["same_outcome"]), (3, 2))


# ------------------------------------------------------------------------------ live path with a fake PX4 (r1 M3)
import math        # noqa: E402
import threading   # noqa: E402
import time        # noqa: E402
import types       # noqa: E402

NS = types.SimpleNamespace


class FakeSession:
    dead = False
    exits = 0

    def __init__(self, ep_dir):
        self.episode_dir = pathlib.Path(ep_dir)

    def __enter__(self):
        self.episode_dir.mkdir(parents=True)
        return self

    def __exit__(self, *a):
        FakeSession.exits += 1
        return False

    def write_world(self, name, text):
        (self.episode_dir / "world").mkdir(exist_ok=True)
        (self.episode_dir / "world" / (name + ".world")).write_text(text)

    def sleep(self, s):
        pass

    def check(self, required=()):
        if FakeSession.dead:
            raise RuntimeError("ProcessExited: px4")


FAKE_BINDING = {"files": {"px4_binary": {"sha256": "p"}, "world": {"sha256": "w"}}}
FakeOS = NS(OwnedSession=FakeSession, direct_launch_plan=lambda d, n, env, model: {"model": model},
            launch_px4_gazebo=lambda s, p: None,
            runtime_binding=lambda plan, airframe: dict(__import__("copy").deepcopy(FAKE_BINDING), airframe=airframe))


class FakeLink:
    made = []

    def __init__(self, stall_after_s=None, jump_after_ctrl=None, jump_s=0.3):
        self.stall_after_s, self.jump_after_ctrl, self.jump_s = stall_after_s, jump_after_ctrl, jump_s
        self.ctrl = 0
        self.m = NS(wait_heartbeat=lambda timeout: True, target_system=1, target_component=1, close=self._close,
                    mav=NS(param_set_send=self._pset, param_request_read_send=lambda *a: None))
        self.mu = NS(mavlink=NS(MAVLINK_MSG_ID_LOCAL_POSITION_NED=32, MAVLINK_MSG_ID_ATTITUDE=30,
                                MAVLINK_MSG_ID_HIL_STATE_QUATERNION=115, MAV_CMD_SET_MESSAGE_INTERVAL=511,
                                MAV_PARAM_TYPE_REAL32=9, MAV_CMD_DO_SET_MODE=176, MAV_CMD_COMPONENT_ARM_DISARM=400))
        self.stop = self.closed = self.armed = self.offboard = False
        self.params, self.pos, self.att, self.truth, self.hb = {}, None, None, None, None
        self.t0 = time.time()
        FakeLink.made.append(self)

    def rx(self):
        while not self.stop:
            w = time.time() - self.t0
            t = w if self.stall_after_s is None else min(w, self.stall_after_s)
            if self.jump_after_ctrl is not None and self.ctrl >= self.jump_after_ctrl:
                t += self.jump_s
            self.pos = (t, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0)
            self.att = (t, 0.0, 0.0, math.pi / 2, 0.0, 0.0, 0.0)
            self.truth = (time.time(), 47.0, 8.0, 101.0 if self.armed else 100.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.hb = (128 if self.armed else 0, (6 << 16) if self.offboard else 0)
            time.sleep(0.001)

    def cmd(self, command, *p):
        if command == 400:
            self.armed = True
        if command == 176 and p[:2] == (1, 6):
            self.offboard = True

    def send(self, fn, *a):
        fn(*a)

    def _pset(self, ts, tc, name, value, typ):
        self.params[name.decode()] = value

    def read_params(self, names):
        return {n: self.params.get(n) for n in names}

    def setpoint(self, mask, *a):
        if mask == 1:                                   # a control-loop velocity setpoint (fake V0's mask)
            self.ctrl += 1

    def _close(self):
        self.closed = True


def fake_bridge():
    import torch

    class Ep:
        def __init__(self, c, h, q):
            pass

        def observe(self, s):
            return torch.zeros(1, 898), {"visible": [False]}, torch.zeros(1, 288)

        def command(self, act, lidar):
            return torch.zeros(2), 0.0, torch.zeros(4)
    return NS(Episode=Ep), NS(act=lambda obs: [torch.zeros(4)])


def fake_v0(tmp, max_steps=15):
    import numpy as np
    sys.path.insert(0, str(TOOLS))
    V0 = _load("sitl_s3b_closed_loop_for_fake", TOOLS / "sitl_s3b_closed_loop.py")
    (tmp / "empty.world").write_text("<sdf><world><physics><ode><solver><iters>10</iters></solver></ode></physics>"
                                     "</world></sdf>")
    return NS(WORLDS=tmp, ODE_ITERS=100, SPAWN=V0.SPAWN, rot=V0.rot, PARAMS={"MPC_XY_VEL_P_ACC": 2.5}, DT=0.001,
              TM_POS_XYZ=0, TM_POS_Z_VEL_XY_YAWRATE=1, ALT=1.0, COMMAND_YAW="truth", MAX_STEPS=max_steps,
              CAPTURE_M=0.5, OOB_MARGIN=1.0, seg_min_dist=V0.seg_min_dist, box_hit=V0.box_hit), np


def fake_spec(np):
    return {"env": 3, "isaac_outcome": "capture", "centers": np.array([[35.0, 35.0, 1.5]]),
            "halves": np.array([[0.3, 0.3, 1.5]]), "quats": np.array([[0, 0, 0, 1.0]]),
            "p0": np.array([20.0, 20.0, 1.0]), "yaw0": 0.0, "tpos": np.tile([30.0, 30.0, 1.0], (600, 1)),
            "bounds_min": [0, 0, 0], "bounds_max": [40, 40, 3]}


class TestLivePath(unittest.TestCase):
    def setUp(self):
        FakeSession.dead = False
        FakeLink.made = []
        self.tmp = pathlib.Path(tempfile.mkdtemp())
        self.B, self.policy = fake_bridge()
        self.V0, self.np = fake_v0(self.tmp)

    def tearDown(self):
        FakeSession.dead = False
        shutil.rmtree(self.tmp, ignore_errors=True)

    def run_one(self, i, **link_kw):
        return SF.run_episode(self.B, FakeOS, self.V0, fake_spec(self.np), self.tmp / ("ep%03d" % i), self.policy,
                              link_factory=lambda: FakeLink(**link_kw))

    def test_two_consecutive_episodes_close_their_links(self):
        for i in range(2):
            r = self.run_one(i)
            self.assertEqual(r["outcome"], "timeout", r.get("error"))
            self.assertTrue(r["link_closed"])
            self.assertFalse(r["rx_thread_alive_after_join"])
            self.assertEqual(SF.episode_valid(r), (True, None))
            self.assertLess(r["late_max_s"], 0.05)
            self.assertEqual(r["steps"], 15)
            steps = self.np.load(self.tmp / ("ep%03d" % i) / "steps.npy")
            self.assertEqual(steps.shape, (15, 27))
            self.assertTrue((steps[:, 24] == 20.0).all())              # p_true y, not compute time (r1 f)
            self.assertTrue(((steps[:, 21] >= 0) & (steps[:, 21] < 1000)).all())
            self.assertEqual(r["compute_ms_max"], float(steps[:, 21].max()))
        self.assertTrue(all(l.closed and l.stop for l in FakeLink.made))
        self.assertEqual(len(FakeLink.made), 2)

    def test_stalled_time_ends_the_episode(self):
        old = SF.STEP_WALL_MAX_S
        SF.STEP_WALL_MAX_S = 0.3
        try:
            t0 = time.time()
            r = self.run_one(0, stall_after_s=0.05)
            self.assertLess(time.time() - t0, 60)
        finally:
            SF.STEP_WALL_MAX_S = old
        self.assertIn("in 0.3 s", r["error"])                     # the patched bound took effect (r2 LOW)
        self.assertEqual(r["outcome"], "ERROR")
        self.assertIn("PX4_TIME_STALLED", r["error"])
        self.assertTrue(FakeLink.made[0].closed)

    def test_dead_process_ends_the_episode(self):
        FakeSession.dead = True
        r = self.run_one(0)
        self.assertEqual(r["outcome"], "ERROR")
        self.assertIn("ProcessExited", r["error"])
        self.assertTrue(FakeLink.made[0].closed)

    def test_time_jump_is_late(self):
        V0, _ = fake_v0(self.tmp, max_steps=200)
        self.V0 = V0
        r = self.run_one(0, jump_after_ctrl=5, jump_s=0.3)
        self.assertGreater(r["late_max_s"], 0.15)
        self.assertEqual(SF.episode_valid(r), (False, "LATE"))

    def test_int32_parameter_decoding(self):
        import struct
        as_float = struct.unpack("<f", struct.pack("<i", 2))[0]
        self.assertEqual(SF.decode_param(NS(param_value=as_float, param_type=SF.MAV_PARAM_TYPE_INT32)), 2.0)
        self.assertEqual(SF.decode_param(NS(param_value=2.5, param_type=9)), 2.5)

    def test_param_value_parsing(self):
        msgs = [NS(get_type=lambda: "PARAM_VALUE", param_id=b"MPC_XY_VEL_P_ACC\x00\x00", param_value=2.5),
                NS(get_type=lambda: "PARAM_VALUE", param_id="THR_MDL_FAC", param_value=1.0)]
        me = NS(stop=False, params={}, m=None)

        def recv_match(blocking=True, timeout=0.5):
            if msgs:
                return msgs.pop(0)
            me.stop = True
            return None
        me.m = NS(recv_match=recv_match, target_system=1)
        SF._param_link_class().rx(me)
        self.assertEqual(me.params, {"MPC_XY_VEL_P_ACC": 2.5, "THR_MDL_FAC": 1.0})


class FakeAT:
    def __init__(self, faults=(0,), idle=True, source_p=(), ext_p=(), git=("h",)):
        self.faults, self.i = list(faults), 0
        self.idle, self.source_p, self.ext_p = idle, list(source_p), list(ext_p)
        self.git, self.gi = list(git), 0
        self.SOURCES = ()

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

    def external_binding(self, ext):
        return {}, list(self.ext_p)


class TestUnitRules(unittest.TestCase):
    def sched(self, n_a=5):
        return [{"block": "A", "bars": 70, "env": e, "repeat": False} for e in range(n_a)]

    def specs(self, n=5):
        return {(70, e): {"target_source": "zero_action_cell", "tpos": [0] * 600} for e in range(n)}

    def go(self, outcomes, faults=(0,), sched=None, at=None, specs=None):
        sched = sched or self.sched()
        it = iter(outcomes)
        with tempfile.TemporaryDirectory() as t:
            SF.run_unit.terminated = None
            res = SF.run_unit(sched, specs or self.specs(), lambda i, item, spec: dict(next(it)),
                              at or FakeAT(faults=faults), pathlib.Path(t) / "l.jsonl", busy_wait_s=0)
            path = pathlib.Path(t) / "l.jsonl"
            self.rows = [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []
        return res, len(self.rows)

    def test_binding_change_is_invalid(self):
        b1 = {"files": {"px4_binary": {"sha256": "p"}, "world": {"sha256": "w1"}}}
        b2 = {"files": {"px4_binary": {"sha256": "q"}, "world": {"sha256": "w2"}}}
        b3 = {"files": {"px4_binary": {"sha256": "p"}, "world": {"sha256": "w3"}}}
        (status, _, done), _ = self.go([dict(self.ok, runtime_binding=b) for b in (b1, b3, b2, b1, b1)])
        self.assertEqual((status, done), ("COMPLETE", 5))
        self.assertEqual([r["invalid_reason"] for r in self.rows], [None, None, "BINDING_CHANGED", None, None])

    def test_host_busy_is_invalid_without_running(self):
        (status, _, done), _ = self.go([self.ok] * 5, at=FakeAT(idle=[True, False, True, True, True]))
        self.assertEqual(status, "COMPLETE")
        self.assertEqual(self.rows[1]["invalid_reason"], "HOST_BUSY")
        self.assertEqual(self.rows[1]["outcome"], "NOT_RUN")
        self.assertEqual(sum(1 for r in self.rows if r["outcome"] == "capture"), 4)

    def test_tree_change_aborts(self):
        (status, reason, done), _ = self.go([self.ok] * 5, at=FakeAT(git=("h", "h", "x")))
        self.assertEqual((status, done), ("ABORTED", 2))
        self.assertIn("tree changed", reason)

    def test_stop_after_a(self):
        sched = self.sched(2) + [{"block": "B", "bars": 70, "env": e, "repeat": False} for e in (2, 3, 4)]
        (status, reason, done), _ = self.go([self.ok, self.ok, self.bad, self.bad, self.bad], sched=sched)
        self.assertEqual((status, done), ("STOPPED_AFTER_A", 5))
        (status, _, _), _ = self.go([self.ok, self.bad, self.bad, self.bad, self.ok])
        self.assertEqual(status, "ABORTED")

    ok = {"outcome": "capture", "param_problems": [], "late_max_s": 0.01}
    bad = {"outcome": "ERROR", "param_problems": [], "late_max_s": 0.01}

    def test_complete(self):
        (status, reason, done), n = self.go([self.ok] * 5)
        self.assertEqual((status, done, n), ("COMPLETE", 5, 5))

    def test_first_invalid_aborts(self):
        (status, reason, done), n = self.go([self.bad] + [self.ok] * 4)
        self.assertEqual((status, done), ("ABORTED", 1))
        self.assertIn("first episode invalid", reason)

    def test_fault_rise_aborts(self):
        (status, reason, done), _ = self.go([self.ok] * 5, faults=(0, 0, 1))
        self.assertEqual((status, done), ("ABORTED", 2))
        self.assertIn("kernel fault", reason)

    def test_invalid_count_aborts(self):
        old = SF.INVALID_ABORT_A
        try:
            with tempfile.TemporaryDirectory() as t:
                it = iter([self.ok] + [self.bad] * 4)
                status, reason, done = SF.run_unit(self.sched(), self.specs(), lambda i, item, spec: next(it),
                                                   FakeAT(), pathlib.Path(t) / "l.jsonl", invalid_abort_a=2)
        finally:
            SF.INVALID_ABORT_A = old
        self.assertEqual((status, done), ("ABORTED", 4))

    def test_termination_is_recorded_then_reraised(self):
        def raise_kb(i, item, spec):
            raise KeyboardInterrupt("SIGTERM")
        with tempfile.TemporaryDirectory() as t:
            SF.run_unit.terminated = None
            status, reason, done = SF.run_unit(self.sched(), self.specs(), raise_kb, FakeAT(),
                                               pathlib.Path(t) / "l.jsonl")
        self.assertEqual((status, done), ("ABORTED", 0))
        self.assertIsInstance(SF.run_unit.terminated, KeyboardInterrupt)


class TestLaunchRefusals(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp())
        self.f = self.tmp / "f.pth"
        self.f.write_bytes(b"policy")
        self._old = (SF.F_SHA256, SF.input_problems, SF.environment_report)
        SF.F_SHA256 = SF.sha256_file(self.f)
        SF.input_problems = lambda *a, **k: []
        SF.environment_report = lambda: ({}, [])
        self.head = "a" * 40
        self.appr = self.tmp / "execution_approval.json"
        self.appr.write_text(json.dumps({"approved_by": "u", "approved_at": "t", "scope": SF.SCOPE,
                                         "commit": self.head}))
        self.B = NS(ROOT=SF.ROOT, F_CKPT=self.f)

    def tearDown(self):
        SF.F_SHA256, SF.input_problems, SF.environment_report = self._old
        shutil.rmtree(self.tmp, ignore_errors=True)

    def problems(self, at=None, gs=None, B=None, attempt=None):
        return SF.launch_problems(at or FakeAT(), B or self.B, gs or {"head": self.head, "dirty": ""}, self.appr,
                                  sources=[], attempt_dir=attempt or (self.tmp / "attempt01"))

    def test_clean_passes(self):
        import os
        saved = {k: os.environ.pop(k) for k in list(os.environ) if k.startswith(SF.ENV_REFUSE_PREFIXES)}
        try:
            self.assertEqual(self.problems(), [])
            os.environ["NAVRL_SOMETHING"] = "1"
            self.assertTrue(any("NAVRL_SOMETHING" in x for x in self.problems()))
        finally:
            os.environ.pop("NAVRL_SOMETHING", None)
            os.environ.update(saved)
        self.assertEqual(SF.env_problems({"HOME": "/x", "PATH": "/y"}), [])
        self.assertEqual(len(SF.env_problems({"PX4_SIM_MODEL": "x", "S3B_COMMAND_YAW": "ekf"})), 2)

    def test_each_refusal(self):
        cases = [(dict(gs={"head": self.head, "dirty": "M x"}), "not clean"),
                 (dict(gs={"head": "b" * 40, "dirty": ""}), "commit"),
                 (dict(at=FakeAT(source_p=["source x differs from HEAD"])), "differs from HEAD"),
                 (dict(at=FakeAT(ext_p=["external source y has local changes"])), "local changes"),
                 (dict(at=FakeAT(idle=False)), "not idle"),
                 (dict(B=NS(ROOT=SF.ROOT, F_CKPT=self.tmp / "missing.pth")), "Policy F"),
                 (dict(B=NS(ROOT=self.tmp, F_CKPT=self.f)), "bridge loads")]
        for kw, needle in cases:
            p = self.problems(**kw)
            self.assertTrue(any(needle in x for x in p), (needle, p))
        (self.tmp / "attempt01").mkdir()
        self.assertTrue(any("single use" in x for x in self.problems()))


PX4_WORLDS = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/"
                          "sitl_gazebo-classic/worlds")


@unittest.skipUnless((PX4_WORLDS / "empty.world").is_file(), "PX4 tree not on this host")
class TestWorldIdentity(unittest.TestCase):
    def test_same_bytes_as_development_runner(self):
        import numpy as np
        sys.path.insert(0, str(TOOLS))
        V0 = _load("sitl_s3b_closed_loop_under_test", TOOLS / "sitl_s3b_closed_loop.py")
        with tempfile.TemporaryDirectory() as t:
            t = pathlib.Path(t)
            shutil.copy(PX4_WORLDS / "empty.world", t / "empty.world")
            V0.WORLDS = t
            centers = np.array([[3.0, 4.0, 1.5], [10.25, -2.5, 1.5]])
            halves = np.array([[0.2, 0.3, 1.5], [0.35, 0.25, 1.5]])
            p0, yaw0 = np.array([5.0, 6.0, 1.0]), 0.7
            V0.write_world("w", centers, halves, p0, yaw0)
            old = (t / "w.world").read_text()
            new = SF.world_text((t / "empty.world").read_text(), centers, halves, p0, yaw0, V0.ODE_ITERS, V0.SPAWN,
                                V0.rot)
        self.assertEqual(new, old)
        self.assertEqual(new.count("<model name='bar_"), 2)


if __name__ == "__main__":
    unittest.main()
