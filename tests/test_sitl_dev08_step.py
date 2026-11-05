"""Native command-prefix identity of the dev08 step against the historical runner (motar-5md).

The historical tools/sitl_s3b_closed_loop.run_episode runs unchanged with PX4, Gazebo, MAVLink and the clock
replaced by fakes: a scripted vehicle publishes the three telemetry messages on a 50 Hz simulator-time grid, and
the fake link converts them with the historical Link.rx expressions themselves (read from its source). The new
TelemetryGate + PrefixEpisode consume the same messages. A deterministic stub bridge/policy whose action depends
on every observed field (and on the fed-back previous action) stands in for Policy F, so any difference in the
state, the sample chosen or the frame transform changes every later command. CPU only; nothing is launched.
"""
import ast
import importlib.util
import math
from pathlib import Path
import shutil
import sys
import tempfile
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


ST = load("sitl_dev08_step_t", "tools/sitl_dev08_step.py")
LK = load("sitl_dev08_link_t", "tools/sitl_dev08_link.py")
GRID_MS = 20
LAT_REF, LON_REF, ALT_REF = 47.397742, 8.545594, 488.0
R_E = 6371000.0
T_UP, T_MOVE = 16.0, 36.0
WALL0 = 1.7e9


class World:
    """Scripted vehicle. ENU-local truth from the spawn point; PX4's estimate is offset from it."""

    def __init__(self, ve=0.3, vn=0.2, yaw_rate=0.05, vz=0.0):
        self.ve, self.vn, self.wr, self.vz = ve, vn, yaw_rate, vz

    def truth(self, t):
        tau = max(0.0, t - T_MOVE)
        up = (1.0 + self.vz * tau) if t >= T_UP else 0.0
        return self.ve * tau, self.vn * tau, up, 0.2 + self.wr * tau

    def messages(self, n):
        t = n * GRID_MS * 1e-3
        e, nn, up, yaw = self.truth(t)
        moving = t >= T_MOVE
        ve, vn = (self.ve, self.vn) if moving else (0.0, 0.0)
        pos = {"time_boot_ms": n * GRID_MS, "x": nn + 0.1, "y": e - 0.05, "z": -up + 0.02, "vx": vn, "vy": ve,
               "vz": -self.vz if moving else 0.0}
        att = {"time_boot_ms": n * GRID_MS, "roll": 0.01, "pitch": -0.02, "yaw": yaw + 0.01, "rollspeed": 0.0,
               "pitchspeed": 0.0, "yawspeed": self.wr if moving else 0.0}
        lat = LAT_REF + math.degrees(nn / R_E)
        lon = LON_REF + math.degrees(e / (R_E * math.cos(math.radians(LAT_REF))))
        truth = {"time_usec": n * GRID_MS * 1000, "attitude_quaternion": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)],
                 "lat": int(round(lat * 1e7)), "lon": int(round(lon * 1e7)), "alt": int(round((ALT_REF + up) * 1000)),
                 "vx": int(round(vn * 100)), "vy": int(round(ve * 100)), "vz": 0}
        return {"LOCAL_POSITION_NED": pos, "ATTITUDE": att, "HIL_STATE_QUATERNION": truth}

    def p_true(self, spec, t):
        e, nn, up, _ = self.truth(t)
        c, s = math.cos(spec["yaw0"]), math.sin(spec["yaw0"])
        return np.array([spec["p0"][0] + c * e - s * nn, spec["p0"][1] + s * e + c * nn, up])


def legacy_rx_expressions(source):
    """The historical Link.rx tuple expressions for self.pos / self.att / self.truth, compiled from its source."""
    tree = ast.parse(source)
    link = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Link")
    rx = next(n for n in link.body if isinstance(n, ast.FunctionDef) and n.name == "rx")
    out = {}
    for node in ast.walk(rx):
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Attribute) \
                and isinstance(node.targets[0].value, ast.Name) and node.targets[0].value.id == "self" \
                and node.targets[0].attr in ("pos", "att", "truth"):
            out[node.targets[0].attr] = compile(ast.Expression(node.value), "<Link.rx>", "eval")
    assert set(out) == {"pos", "att", "truth"}
    return out


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.links = []

    def sleep(self, dt):
        self.t += dt
        for link in self.links:
            link.advance(self.t)

    def time(self):
        return WALL0 + self.t


MU = types.SimpleNamespace(MAVLINK_MSG_ID_LOCAL_POSITION_NED=32, MAVLINK_MSG_ID_ATTITUDE=30,
                           MAVLINK_MSG_ID_HIL_STATE_QUATERNION=115, MAV_CMD_SET_MESSAGE_INTERVAL=511,
                           MAV_PARAM_TYPE_REAL32=9, MAV_CMD_DO_SET_MODE=176, MAV_CMD_COMPONENT_ARM_DISARM=400,
                           MAV_FRAME_LOCAL_NED=1)


class FakeLink:
    def __init__(self, world, clock, rx_exprs):
        self.world, self.clock, self.rx_exprs = world, clock, rx_exprs
        self.mu = types.SimpleNamespace(mavlink=MU)
        self.m = types.SimpleNamespace(wait_heartbeat=lambda timeout=None: object(), target_system=1,
                                       target_component=1,
                                       mav=types.SimpleNamespace(param_set_send=lambda *a: None,
                                                                 command_long_send=lambda *a: None))
        self.stop = False
        self.hb = (128, 6 << 16)
        self.n = None
        self.setpoints = []
        clock.links.append(self)
        self.advance(clock.t)

    def advance(self, t):
        n = int(math.floor(t * 1000 / GRID_MS + 1e-9))
        if n == self.n:
            return
        self.n = n
        m = self.world.messages(n)
        for attr, typ in (("pos", "LOCAL_POSITION_NED"), ("att", "ATTITUDE"), ("truth", "HIL_STATE_QUATERNION")):
            msg = types.SimpleNamespace(**m[typ])
            setattr(self, attr, eval(self.rx_exprs[attr], {"time": self.clock},
                                     {"msg": msg, "q": getattr(msg, "attitude_quaternion", None)}))

    def rx(self):
        return None

    def send(self, fn, *a):
        fn(*a)

    def cmd(self, command, *p):
        pass

    def setpoint(self, mask, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.setpoints.append((mask, x, y, z, vx, vy, yaw_rate))


class Arr:
    def __init__(self, a):
        self.a = np.asarray(a)

    def numpy(self):
        return self.a


class StubEpisode:
    """Deterministic stand-in for sitl_bridge.Episode: records every observed state dict."""
    created = []

    def __init__(self, centers, halves, quats):
        self.calls = []
        StubEpisode.created.append(self)

    def observe(self, s):
        snap = {k: None if v is None else np.array(v, dtype=np.float64).copy() for k, v in s.items()}
        self.calls.append(snap)
        flat = np.concatenate([snap[k].ravel() for k in sorted(snap) if snap[k] is not None])
        w = np.sin(np.arange(flat.size) * 0.7 + 0.3)
        obs = np.tanh(np.array([np.dot(flat, np.roll(w, i)) * 0.05 for i in range(16)]))
        return obs, {"visible": [bool(obs[0] > 0)]}, np.full(288, 3.0)

    def command(self, act, lidar):
        a = np.clip(np.asarray(act, np.float32), -1.0, 1.0)
        gov = a[:2] * np.float32(2.5) * np.float32(0.8)
        return gov, float(a[3] * 3.0), Arr(np.array([gov[0] / 2.5, gov[1] / 2.5, a[2], a[3]], np.float32))


class StubPolicy:
    def act(self, obs):
        o = np.asarray(obs)
        return [Arr(np.tanh(np.array([o[:4].sum(), o[4:8].sum() - 0.3, o[8:12].mean(), o[12:16].sum() * 0.5],
                                     np.float32)))]


STUB_B = types.SimpleNamespace(Episode=StubEpisode)


def moving(start, velocity, frames, dt=0.1):
    return np.array([np.asarray(start, float) + np.asarray(velocity, float) * dt * k for k in range(frames)])


def spec_for(scenario, world):
    spec = {"env": 0, "isaac_outcome": "unknown", "p0": np.array([5.0, 6.0, 1.0]), "yaw0": 0.7,
            "bounds_min": [0.0, 0.0, 0.0], "bounds_max": [40.0, 40.0, 3.0]}
    far_bar = (np.array([[30.0, 30.0, 1.5]]), np.array([[0.1, 0.1, 1.5]]))
    if scenario in ("moving", "vz", "fast_cross", "oob", "ceiling"):
        frames = {"moving": 80, "vz": 40, "fast_cross": 60, "oob": 200, "ceiling": 200}[scenario]
        centers, halves = far_bar
        if scenario == "moving":                    # a target walking towards the vehicle's path: index-sensitive
            tpos = moving(world.p_true(spec, T_MOVE + 6.0) + np.array([3.0, 0.0, 0.0]), (-0.5, 0.0, 0.0), frames)
        elif scenario == "fast_cross":              # hovering vehicle; target crosses 0.2 m away at 10 m/s
            c = world.p_true(spec, T_MOVE)              # checkpoint offsets ..., -0.5, +0.5: points 0.539 m away
            tpos = np.array([c + np.array([-8.5 + 1.0 * k, 0.2, 0.0]) for k in range(frames)])
        else:
            tpos = moving((35.0, 35.0, 1.0), (0.0, -0.05, 0.0), frames)
        if scenario == "oob":
            spec.update(p0=np.array([1.0, 20.0, 1.0]), yaw0=math.pi)
        spec.update(centers=centers, halves=halves, quats=np.tile([0.0, 0.0, 0.0, 1.0], (1, 1)), tpos=tpos)
        return spec, frames
    far_bar = (np.array([[30.0, 30.0, 1.5]]), np.array([[0.1, 0.1, 1.5]]))
    if scenario == "capture":
        target = world.p_true(spec, T_MOVE + 3.0) + np.array([0.05, -0.05, 0.0])
        centers, halves, frames = far_bar[0], far_bar[1], 60
    elif scenario == "crash":
        target = np.array([35.0, 35.0, 1.0])
        bar = world.p_true(spec, T_MOVE + 2.5)
        centers, halves, frames = np.array([[bar[0], bar[1], 1.5]]), np.array([[0.1, 0.1, 1.5]]), 60
    else:                         # censor / censor600: nothing happens inside 20 or 600 frames
        target = np.array([35.0, 35.0, 1.0])
        centers, halves, frames = far_bar[0], far_bar[1], (600 if scenario == "censor600" else 20)
    spec.update(centers=centers, halves=halves, quats=np.tile([0.0, 0.0, 0.0, 1.0], (len(centers), 1)),
                tpos=np.tile(target, (frames, 1)))
    return spec, frames


def run_legacy(world, spec, out_dir, B=STUB_B, policy=None, observed=None):
    H = load("hist_runner_for_identity", "tools/sitl_s3b_closed_loop.py")
    clock = FakeClock()
    rx = legacy_rx_expressions((ROOT / "tools/sitl_s3b_closed_loop.py").read_text())
    link = {}

    def make_link():
        link["L"] = FakeLink(world, clock, rx)
        return link["L"]

    H.time = clock
    H.Link = make_link
    H.write_world = lambda *a, **k: None
    H.other_sitl = lambda: ""
    H.start_sitl = lambda world_name, log: object()
    H.stop_sitl = lambda proc: None
    StubEpisode.created = []
    res = H.run_episode(B, spec, out_dir, 0, policy or StubPolicy())
    rows = np.load(out_dir / "ep00_steps.npy")
    velocity = [sp for sp in link["L"].setpoints if sp[0] == H.TM_POS_Z_VEL_XY_YAWRATE]
    return {"res": res, "rows": rows, "commands": velocity,
            "observed": (observed or (lambda: StubEpisode.created[0].calls))()}


def run_new(world, spec, frames, legacy_rows, shift=0, B=STUB_B, policy=None, observed=None):
    now = [0.0]                                    # receive clock: simulator time at real-time factor 1
    inbox = LK.Inbox(clock=lambda: now[0])
    gate = None
    fed = [None]

    def feed_to(n):
        start = 0 if fed[0] is None else fed[0] + 1
        for i in range(start, n + 1):
            for j, (typ, fields) in enumerate(world.messages(i).items()):   # one send burst per sample time
                now[0] = i * GRID_MS * 1e-3 + 1e-4 * j
                inbox.put({"type": typ, "src": (1, 1), "fields": fields})
        fed[0] = max(n, fed[0] if fed[0] is not None else -1)

    n0 = int(round(legacy_rows[0][1] * 1000 / GRID_MS)) + shift
    feed_to(n0)
    gate = LK.TelemetryGate(inbox, start_cursor=inbox.cursor_since(now[0] - 1.0), stall_wall_s=0.5)
    gate.align()
    ground = ST.truth_tuple(world.messages(int(13.0 * 1000 / GRID_MS))["HIL_STATE_QUATERNION"])
    StubEpisode.created = []
    ep = ST.PrefixEpisode(B, policy or StubPolicy(), spec, frames, (ground[1], ground[2], ground[3]))
    recs = []
    n = n0
    for k in range(frames):
        t_k = gate.start + k * LK.DT
        while n * GRID_MS * 1e-3 < t_k:
            n += 1
        feed_to(n + 1)                                       # one later sample is already there
        rec = ep.step(k, gate.checkpoint(k))
        recs.append(rec)
        if rec["decision"] != "CONTINUE":
            break
    return {"ep": ep, "recs": recs, "observed": (observed or (lambda: StubEpisode.created[0].calls))(),
            "start": gate.start}


def same_state(a, b):
    return set(a) == set(b) and all((a[k] is None and b[k] is None) or np.array_equal(a[k], b[k]) for k in a)


class Identity(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="dev08_identity_"))
        self.world = World()

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def compare(self, scenario):
        spec, frames = spec_for(scenario, self.world)
        old = run_legacy(self.world, spec, self.tmp)
        new = run_new(self.world, spec, frames, old["rows"])
        return spec, frames, old, new

    def check_prefix(self, old, new):
        n_cmd = len(new["ep"].commands)
        self.assertGreater(n_cmd, 5)
        self.assertEqual(new["start"], old["rows"][0][1])
        self.assertEqual(new["ep"].commands, old["commands"][:n_cmd])          # exact floats
        for k in range(n_cmd):
            self.assertEqual(new["recs"][k]["sim_s"], old["rows"][k][1], "sample chosen at checkpoint %d" % k)
            self.assertTrue(same_state(new["observed"][k], old["observed"][k]), "state at checkpoint %d" % k)
        return n_cmd

    def test_capture_prefix_identity(self):
        _, _, old, new = self.compare("capture")
        n_cmd = self.check_prefix(old, new)
        c = new["ep"].counts()
        self.assertEqual((old["res"]["outcome"], c["outcome"]), ("capture", "capture"))
        self.assertEqual(c["final_frame_index"], n_cmd)
        self.assertEqual(c["observation_checks"], old["res"]["steps"])           # historical steps = checks
        self.assertEqual(len(old["commands"]), n_cmd + 1)                         # its command at the terminal step
        self.assertEqual(len(old["observed"]), n_cmd + 1)
        self.assertIsNone(new["recs"][-1]["command"])
        self.assertEqual(c["min_dist_m"], old["res"]["min_dist_m"])

    def test_crash_prefix_identity(self):
        _, _, old, new = self.compare("crash")
        n_cmd = self.check_prefix(old, new)
        c = new["ep"].counts()
        self.assertEqual((old["res"]["outcome"], c["outcome"]), ("crash", "crash"))
        self.assertEqual((old["res"]["crash_reason"], new["recs"][-1]["crash_reason"]), ("bar", "bar"))
        self.assertEqual(c["observation_checks"], old["res"]["steps"])
        self.assertEqual(len(old["commands"]), n_cmd + 1)

    def test_censor_prefix_identity(self):
        _, frames, old, new = self.compare("censor")
        n_cmd = self.check_prefix(old, new)
        c = new["ep"].counts()
        self.assertEqual(c, dict(c, outcome="CENSORED_TRACE_END", final_frame_index=frames - 1,
                                 observation_checks=frames, commands_issued=frames - 1))
        self.assertGreater(len(old["commands"]), n_cmd)                          # legacy continued (clamped target)
        self.assertEqual(n_cmd, frames - 1)

    def test_full_600_frame_censor_prefix_identity(self):
        # long enough for the float rounding of start + k*DT to fall on sample times (k >= 164 here)
        _, frames, old, new = self.compare("censor600")
        n_cmd = self.check_prefix(old, new)
        self.assertEqual((n_cmd, new["ep"].counts()["outcome"]), (599, "CENSORED_TRACE_END"))
        self.assertEqual((old["res"]["outcome"], old["res"]["steps"], len(old["commands"])), ("timeout", 600, 600))

    def run_scenario(self, scenario, world, spec=None, frames=None):
        if spec is None:
            spec, frames = spec_for(scenario, world)
        old = run_legacy(world, spec, self.tmp)
        new = run_new(world, spec, frames, old["rows"])
        return old, new

    def assert_terminal(self, old, new, outcome, reason=None):
        n_cmd = self.check_prefix(old, new)
        c = new["ep"].counts()
        self.assertEqual((old["res"]["outcome"], c["outcome"]), (outcome, outcome))
        self.assertEqual(c["observation_checks"], old["res"]["steps"])
        self.assertEqual(len(old["commands"]), n_cmd + 1)
        if reason:
            self.assertEqual((old["res"]["crash_reason"], new["recs"][-1]["crash_reason"]), (reason, reason))
        return c

    def test_moving_target_capture(self):              # target index k (not k+1, not 0)
        old, new = self.run_scenario("moving", World())
        c = self.assert_terminal(old, new, "capture")
        k = c["final_frame_index"]
        self.assertEqual(new["recs"][k]["tpos"], list(spec_for("moving", World())[0]["tpos"][k]))

    def test_vertical_velocity_censor(self):           # NED vz sign into the observed velocity
        old, new = self.run_scenario("vz", World(vz=0.01))
        self.check_prefix(old, new)
        self.assertGreater(new["observed"][5]["vel"][2], 0.0)       # climbing: world-frame vz > 0
        self.assertEqual(new["ep"].counts()["outcome"], "CENSORED_TRACE_END")

    def test_fast_crossing_is_a_swept_capture(self):
        world = World(ve=0.0, vn=0.0, yaw_rate=0.0)
        old, new = self.run_scenario("fast_cross", world)
        self.assert_terminal(old, new, "capture")
        last = new["recs"][-1]
        self.assertGreater(last["dist_m"], 0.5)               # the point distance alone would not capture
        self.assertLess(last["sweep_m"], 0.5)

    def test_capture_wins_over_a_crash_at_the_same_checkpoint(self):
        world = World()
        spec, frames = spec_for("crash", world)
        crash = run_legacy(world, spec, self.tmp)
        k_c = crash["res"]["steps"] - 1
        p_c = crash["rows"][k_c][23:26]
        tpos = np.tile(np.array([35.0, 35.0, 1.0]), (frames, 1))
        tpos[k_c:] = p_c                                      # the target appears at the contact point
        both = dict(spec, tpos=tpos)
        shutil.rmtree(self.tmp)
        self.tmp.mkdir()
        old, new = self.run_scenario(None, world, both, frames)
        self.assert_terminal(old, new, "capture")
        self.assertEqual(new["ep"].counts()["final_frame_index"], k_c)

    def test_out_of_bounds_uses_the_margin(self):
        old, new = self.run_scenario("oob", World(ve=0.3, vn=0.0, yaw_rate=0.0))
        self.assert_terminal(old, new, "crash", "bounds")
        self.assertLess(new["recs"][-1]["p_true"][0], -1.0)    # beyond the 1 m margin, not at the boundary
        self.assertGreater(new["recs"][-2]["p_true"][0], -1.0)

    def test_altitude_ceiling(self):
        old, new = self.run_scenario("ceiling", World(ve=0.0, vn=0.0, yaw_rate=0.0, vz=0.5))
        self.assert_terminal(old, new, "crash", "altitude")
        self.assertGreater(new["recs"][-1]["p_true"][2], 4.0)

    def test_comparison_detects_a_one_sample_shift(self):
        spec, frames = spec_for("capture", self.world)
        old = run_legacy(self.world, spec, self.tmp)
        new = run_new(self.world, spec, frames, old["rows"], shift=1)
        n = min(len(new["ep"].commands), len(old["commands"]))
        self.assertNotEqual(new["ep"].commands[:n], old["commands"][:n])

    def test_step_refuses_out_of_order_and_outside_prefix(self):
        spec, frames = spec_for("censor", self.world)
        ep = ST.PrefixEpisode(STUB_B, StubPolicy(), spec, frames, (LAT_REF, LON_REF, ALT_REF))
        with self.assertRaises(ValueError):
            ep.step(1, {"k": 1})
        with self.assertRaises(ValueError):
            ST.PrefixEpisode(STUB_B, StubPolicy(), dict(spec, tpos=spec["tpos"][:5]), frames,
                             (LAT_REF, LON_REF, ALT_REF))

    def test_geometry_comes_from_the_pinned_historical_runner(self):
        H = ST.historical()
        self.assertEqual((H.CAPTURE_M, H.FOOT_HALF, H.OOB_MARGIN, H.DT, H.ALT), (0.5, 0.14, 1.0, 0.1, 1.0))
        self.assertEqual(ST.RUNNER_SHA, load("cfg_pins", "tools/sitl_s3b_dev08.py").PINS["tools/sitl_s3b_closed_loop.py"])


if __name__ == "__main__":
    unittest.main()
