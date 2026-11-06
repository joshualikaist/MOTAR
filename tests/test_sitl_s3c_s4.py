"""CPU tests for the S3c and S4 SITL units (tools/sitl_s3c.py, tools/sitl_s4.py, tools/sitl_s4_gazebo.py,
tools/sitl_s4_model.py). No SITL, PX4, Gazebo or GPU is started; the live paths run against fakes."""
import importlib.util
import json
import math
import pathlib
import shutil
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(ROOT / "tests"))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


T3B = _load("test_sitl_s3b_formal_helpers", ROOT / "tests" / "test_sitl_s3b_formal.py")
SF = T3B.SF
import sitl_s3c as S3C        # noqa: E402
import sitl_s4 as S4          # noqa: E402
import sitl_s4_gazebo as G    # noqa: E402
import sitl_s4_model as SM    # noqa: E402

NS = types.SimpleNamespace


class TestS3cPlan(unittest.TestCase):
    @unittest.skipUnless((SF.G8 / "zero").is_dir(), "G8 r2 raw inputs not on this host")
    def test_schedule(self):
        s = S3C.schedule()
        self.assertEqual([x["arm"] for x in s], ["iris"] * 40 + ["delay1"] * 40)
        a = [x["env"] for x in s if x["arm"] == "iris"]
        self.assertEqual(a, [x["env"] for x in s if x["arm"] == "delay1"])
        self.assertEqual(a, [x["env"] for x in SF.schedule() if x["block"] == "A"][:40])

    def test_params_namespace(self):
        V0 = NS(PARAMS={"A": 1}, DT=0.1, rot=abs)
        self.assertIs(S3C.v0_with_params(V0, None), V0)
        ns = S3C.v0_with_params(V0, {"B": 2})
        self.assertEqual((ns.PARAMS, ns.DT, ns.rot), ({"B": 2}, 0.1, abs))
        self.assertEqual(V0.PARAMS, {"A": 1})

    def test_arms(self):
        self.assertEqual([a for a, _, _ in S3C.ARMS], ["iris", "delay1"])
        self.assertEqual(S3C.ARMS[0][1], {"model": "navrl_iris", "delay_steps": 0})
        self.assertEqual(S3C.ARMS[1][1], {"model": "navrl_quad", "delay_steps": 1})
        self.assertIsNone(S3C.ARMS[1][2])


class TestArmUnitRules(unittest.TestCase):
    """S3c and S4 run SF.run_unit with key "arm" and every arm primary."""
    ok = {"outcome": "capture", "param_problems": [], "late_max_s": 0.01}
    bad = {"outcome": "ERROR", "param_problems": [], "late_max_s": 0.01}

    def go(self, outcomes, arms=("x",) * 3 + ("y",) * 3, invalid_abort=12):
        sched = [{"arm": a, "bars": 70, "env": i} for i, a in enumerate(arms)]
        specs = {i: {"target_source": "zero_action_cell", "tpos": [0] * 600} for i in range(len(arms))}
        it = iter(outcomes)
        with tempfile.TemporaryDirectory() as t:
            SF.run_unit.terminated = None
            out = SF.run_unit(sched, specs, lambda i, item, spec: dict(next(it)), T3B.FakeAT(), pathlib.Path(t) / "l",
                              key="arm", primary=None, invalid_abort_a=invalid_abort, busy_wait_s=0)
            self.rows = [json.loads(x) for x in (pathlib.Path(t) / "l").read_text().splitlines()]
        return out

    def test_per_arm_binding_reference(self):
        bx = {"files": {"px4_binary": {"sha256": "p"}, "installed_airframe": {"sha256": "quad"}}}
        by = {"files": {"px4_binary": {"sha256": "p"}, "installed_airframe": {"sha256": "iris"}}}
        out = self.go([dict(self.ok, runtime_binding=b) for b in (bx, bx, bx, by, by, bx)])
        self.assertEqual(out[0::2], ("COMPLETE", 6))
        self.assertEqual([r["invalid_reason"] for r in self.rows], [None] * 5 + ["BINDING_CHANGED"])

    def test_stops(self):
        st, why, done = self.go([self.ok, self.bad, self.bad, self.bad, self.ok, self.ok])
        self.assertEqual((st, done), ("ABORTED", 4))
        self.assertIn("consecutive", why)
        st, why, done = self.go([self.ok, self.bad, self.ok, self.bad, self.ok, self.bad], invalid_abort=2)
        self.assertEqual((st, done), ("ABORTED", 6))
        st, _, done = self.go([self.ok] * 6)
        self.assertEqual((st, done), ("COMPLETE", 6))

    def test_termination(self):
        def kb(i, item, spec):
            raise KeyboardInterrupt()
        with tempfile.TemporaryDirectory() as t:
            SF.run_unit.terminated = None
            st, _, done = SF.run_unit([{"arm": "x", "bars": 70, "env": 0}],
                                      {0: {"target_source": "zero_action_cell", "tpos": [0] * 600}}, kb, T3B.FakeAT(),
                                      pathlib.Path(t) / "l", key="arm", primary=None)
        self.assertEqual((st, done), ("ABORTED", 0))
        self.assertIsInstance(SF.run_unit.terminated, KeyboardInterrupt)


class RecordingLink(T3B.FakeLink):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.sent = []

    def setpoint(self, mask, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        super().setpoint(mask)
        if mask == 1:
            self.sent.append((vx, vy, yaw_rate))


def stepping_bridge():
    import torch

    class Ep:
        n = 0

        def __init__(self, c, h, q):
            pass

        def observe(self, s):
            return torch.zeros(1, 898), {"visible": [False]}, torch.zeros(1, 288)

        def command(self, act, lidar):
            Ep.n += 1
            return torch.tensor([0.1 * Ep.n, 0.0]), 0.0, torch.zeros(4)
    return NS(Episode=Ep), NS(act=lambda obs: [torch.zeros(4)])


class TestDelayThroughRunEpisode(unittest.TestCase):
    def test_delay1_shifts_commands(self):
        tmp = pathlib.Path(tempfile.mkdtemp())
        try:
            sent = {}
            for d in (0, 1):
                B, pol = stepping_bridge()
                V0, np_ = T3B.fake_v0(tmp, max_steps=6)
                links = []

                def mk():
                    links.append(RecordingLink())
                    return links[-1]
                r = SF.run_episode(B, T3B.FakeOS, V0, T3B.fake_spec(np_), tmp / ("d%d" % d), pol,
                                   condition={"model": "navrl_quad", "delay_steps": d}, link_factory=mk)
                self.assertEqual(r["outcome"], "timeout", r.get("error"))
                # the fake truth heading is NED yaw 0 (north), so the vehicle-x command is the north setpoint
                sent[d] = [round(s[0], 6) for s in links[0].sent]
                self.assertTrue(all(abs(s[1]) < 1e-9 for s in links[0].sent))
                self.assertTrue(links[0].closed)
            self.assertEqual(sent[0], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
            self.assertEqual(sent[1], [0.0, 0.1, 0.2, 0.3, 0.4, 0.5])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestS4Conversions(unittest.TestCase):
    def test_lidar_order_and_limits(self):
        gz = np.array([[100.0 * v + h for h in range(72)] for v in range(4)]) / 1000.0 + 1.0
        m = G.motar_lidar(gz).reshape(4, 72)
        for i in range(4):
            for j in (0, 17, 36, 71):
                self.assertAlmostEqual(float(m[i, j]), float(gz[3 - i, 71 - j]), places=5)
        gz2 = np.full((4, 72), 5.0)
        gz2[0, 0], gz2[1, 1], gz2[2, 2], gz2[3, 3], gz2[0, 4] = np.inf, 0.1, 13.0, -np.inf, np.nan
        m2 = G.motar_lidar(gz2).reshape(4, 72)
        self.assertEqual((m2[3, 71], m2[2, 70], m2[1, 69], m2[0, 68], m2[3, 67], m2[0, 0]),
                         (12.0, 0.0, 12.0, 0.0, 12.0, 5.0))           # +inf/NaN/far -> 12; -inf/too close -> 0
        with self.assertRaises(ValueError):
            G.motar_lidar(np.zeros((72, 4)))

    def test_rgbd(self):
        rgb = np.zeros((90, 160, 3), np.uint8)
        rgb[..., 0] = 255
        z = np.full((90, 160), 2.0, np.float32)
        z[0, 0], z[1, 1] = np.inf, 0.0
        r, d = G.motar_rgbd(rgb, z)
        self.assertEqual((r.shape, d.shape), ((1, 3, 90, 160), (1, 90, 160)))
        self.assertAlmostEqual(float(r[0, 0, 5, 5]), 1.0)
        self.assertLess(abs(float(d[0, 45, 80]) - 2.0), 0.002)           # centre ray: range ~ depth
        f = 80 / math.tan(math.radians(43.5))
        corner = 2.0 * math.sqrt(1 + ((89.5 - 80) / f) ** 2 + ((44.5 - 45) / f) ** 2)
        self.assertAlmostEqual(float(d[0, 45, 159]), 2.0 * math.sqrt(1 + (79.5 / f) ** 2 + (0.5 / f) ** 2), places=4)
        self.assertGreater(corner, 2.0)
        self.assertEqual((float(d[0, 0, 0]), float(d[0, 1, 1])), (20.0, 20.0))

    def test_ground_removal(self):
        q = (1.0, 0.0, 0.0, 0.0)
        L = np.full((4, 72), 12.0)
        L[3, :] = 1.0 / math.sin(math.radians(10))
        L[3, 36] = L[2, 36] = 2.8
        D = np.full((90, 160), 20.0)
        up = G.up_components(q, G._CAM_FLU)
        D[up < 0] = np.minimum(20.0, 1.03 / (-up[up < 0]))
        D[45, 80] = 2.7                                                  # a bar straight ahead
        lm, dm, c = G.remove_ground(L.reshape(1, 288), D[None], q, 1.0)
        lm = lm.reshape(4, 72)
        self.assertEqual(c["lidar_ground_removed"], 71)
        self.assertTrue((lm[3, :36] == 12.0).all())
        self.assertAlmostEqual(float(lm[3, 36]), 2.8, places=5)
        self.assertAlmostEqual(float(lm[2, 36]), 2.8, places=5)
        self.assertGreater(c["depth_ground_removed"], 1000)
        self.assertAlmostEqual(float(dm[0, 45, 80]), 2.7, places=5)
        self.assertEqual(float(dm.max()), 20.0)
        p = math.radians(-10)                                            # nose down 10 deg: forward beam points down
        self.assertAlmostEqual(float(G.up_components((math.cos(p / 2), 0, math.sin(p / 2), 0), G._LIDAR_FLU)[2, 36]),
                               -math.sin(math.radians(10)), places=6)

    def test_world_and_model(self):
        w = G.world_text_s4("<sdf><world name='default'></world></sdf>")
        self.assertEqual(w.count("<model name='motar_target'>"), 1)
        self.assertEqual(w.count("libmotar_s4_bridge.so"), 1)
        self.assertIn(G.SENSOR_SCOPE + "motar_lidar", w)
        with self.assertRaises(ValueError):
            G.world_text_s4("<world></world><world></world>")
        src = ("<sdf><model name='navrl_quad'><link name='base_link'><inertial/><visual name='b'><geometry><mesh>"
               "</mesh></geometry></visual><collision name='c'/></link><link name='rotor_0'><visual name='p'>"
               "</visual></link></model></sdf>")
        out = SM.model_sdf(src)
        self.assertEqual(out.count("<model name='navrl_quad_s4'>"), 1)
        self.assertEqual((out.count("<visual "), out.count("<collision"), out.count("<inertial/>")), (0, 1, 1))
        with self.assertRaises(ValueError):                          # nothing to strip: the check would be vacuous
            SM.model_sdf("<sdf><model name='navrl_quad'><link name='base_link'></link></model></sdf>")
        b = out[out.index("<link name='base_link'>"):out.index("</link>")]
        self.assertEqual((b.count("type='gpu_ray'"), b.count("type='camera'"), b.count("type='depth'")), (1, 1, 1))
        self.assertEqual(out[out.index("<link name='rotor_0'>"):].count("<sensor"), 0)
        with self.assertRaises(ValueError):
            SM.model_sdf("<sdf><model name='navrl_quad'></model></sdf>")


class FakeClient:
    made = []

    def __init__(self):
        self.targets, self.closed, self.n = [], False, 0
        FakeClient.made.append(self)

    def set_target(self, x, y, z):
        self.targets.append((x, y, z))
        return 0.0

    def frames(self):
        self.n += 1
        t = 0.0
        return {"sim_now": 1.0, "lidar_t": t, "rgb_t": t, "depth_t": t, "target_t": t, "target_count": self.n,
                "ranges": np.full((4, 72), 12.0), "rgb": np.zeros((90, 160, 3), np.uint8),
                "depth": np.full((90, 160), 25.0, np.float32)}

    def close(self):
        self.closed = True


def s4_fake_bridge():
    import torch

    class Det:
        target_mask = torch.zeros(1, 90, 160)

        def render_raw_rgbd(self, *a):
            return torch.zeros(1, 3, 90, 160), torch.full((1, 90, 160), 20.0)

    class Per:
        def observe(self, **kw):
            assert kw["rgb"].shape == (1, 3, 90, 160) and kw["depth"].shape == (1, 90, 160)
            assert kw["lidar_m"].shape == (1, 288)
            return torch.zeros(1, 898), {"visible": [False], "camera_visible": [False], "lidar_visible": [False]}

    class Ep:
        def __init__(self, c, h, q):
            self.torch, self.detector, self.perception = torch, Det(), Per()
            self.tc = NS(yaw_rate_max=3.0, max_velocity=2.5, flight_altitude=1.0)

        def lidar_m(self, *a):
            return torch.full((1, 288), 12.0)

        def command(self, act, lidar):
            return torch.zeros(2), 0.0, torch.zeros(4)
    return NS(Episode=Ep), NS(act=lambda obs: [torch.zeros(4)])


class TestS4LivePath(unittest.TestCase):
    def test_episode_with_fake_plugin(self):
        tmp = pathlib.Path(tempfile.mkdtemp())
        try:
            for arm in ("ground", "raw"):
                FakeClient.made = []
                B, pol = s4_fake_bridge()
                V0, np_ = T3B.fake_v0(tmp, max_steps=5)
                links = []

                def mk():
                    links.append(T3B.FakeLink())
                    return links[-1]
                r = S4.run_episode(B, T3B.FakeOS, V0, T3B.fake_spec(np_), tmp / arm, pol, arm, link_factory=mk,
                                   client_factory=FakeClient)
                self.assertEqual(r["outcome"], "timeout", r.get("error"))
                self.assertTrue(links[0].closed and FakeClient.made[0].closed and r["link_closed"])
                self.assertEqual(len(FakeClient.made[0].targets), 6)            # step 0, then k + 1 for 5 steps
                sens = np.load(tmp / arm / "sensor_steps.npy")
                self.assertEqual(sens.shape, (5, 16))
                self.assertEqual(r["hover_guard"], {"lidar_min_m": 12.0, "depth_min_m": 20.0})
                self.assertAlmostEqual(FakeClient.made[0].targets[0][2], 1.0 + S4.SF_H0)
                if arm == "ground":
                    self.assertTrue((sens[:, 11] >= 0).all())
                else:
                    self.assertTrue((sens[:, 11] == -1).all())
                self.assertIn("frame_age_max_s", r["sensor"])
                self.assertIn("s4_plugin", r["runtime_binding"]["files"])
                self.assertEqual(r["runtime_binding"]["airframe"], "10092_gazebo-classic_navrl_quad_s4")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)



class TestHoverGuardAndRecords(unittest.TestCase):
    def test_hover_guard_rejects_self_returns(self):
        tmp = pathlib.Path(tempfile.mkdtemp())
        try:
            class NearClient(FakeClient):
                def frames(self):
                    f = super().frames()
                    f["ranges"] = f["ranges"].copy()
                    f["ranges"][2, 10] = 0.22                       # a body return at 0.22 m
                    return f
            B, pol = s4_fake_bridge()
            V0, np_ = T3B.fake_v0(tmp, max_steps=3)
            r = S4.run_episode(B, T3B.FakeOS, V0, T3B.fake_spec(np_), tmp / "e", pol, "raw",
                               link_factory=T3B.FakeLink, client_factory=NearClient)
            self.assertEqual(r["outcome"], "HARNESS_INVALID")
            self.assertIn("hover guard", r["error"])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_iris_footprint_report(self):
        with tempfile.TemporaryDirectory() as t:
            d = pathlib.Path(t)
            for i, clr in enumerate((0.30, 0.20, np.inf)):
                (d / ("ep%03d" % i)).mkdir()
                s = np.zeros((4, 27))
                s[:, S3C.CLEARANCE_COL] = 5.0
                s[2, S3C.CLEARANCE_COL] = clr
                np.save(d / ("ep%03d" % i) / "steps.npy", s)
            rows = [{"index": i, "valid": True} for i in range(3)] + [{"index": 3, "valid": False}]
            out = S3C.iris_footprint(d, rows)
        self.assertEqual((out["episodes"], out["within_half_span"], out["steps_missing"]), (3, 1, 0))

    def test_smoke_gate(self):
        with tempfile.TemporaryDirectory() as t:
            d = pathlib.Path(t)
            self.assertTrue(S3C.smoke_problems(d))
            (d / "attempt.json").write_text(json.dumps({"git": {"head": "h"}, "sources": {"a": "1"},
                                                        "development_smoke": True}))
            (d / "final.json").write_text(json.dumps({"status": "COMPLETE"}))
            (d / "episodes.jsonl").write_text(json.dumps({"valid": False, "invalid_reason": "ERROR"}) + "\n")
            self.assertTrue(S3C.smoke_problems(d, "h", {"a": "1"}))
            (d / "episodes.jsonl").write_text(json.dumps({"valid": True}) + "\n")
            self.assertEqual(S3C.smoke_problems(d, "h", {"a": "1"}), [])
            self.assertIn("not HEAD", S3C.smoke_problems(d, "x", {"a": "1"})[0])
            self.assertIn("other source", S3C.smoke_problems(d, "h", {"a": "2"})[0])
            with self.assertRaises(SystemExit):                        # a smoke is never read as a result
                S3C.load_attempt(d)

    def test_arm_integrity_and_reference(self):
        rows = [{"arm": "x", "env": 1}, {"arm": "y", "env": 1}]
        att = {"schedule": rows, "kernel_faults_before": 0, "git": {"head": "h", "dirty": ""}}
        fin = {"status": "COMPLETE", "kernel_faults_after": 0, "git_after": {"head": "h", "dirty": ""}}
        self.assertEqual(S3C.arm_integrity_problems(att, fin, rows), [])
        self.assertTrue(S3C.arm_integrity_problems(att, fin, rows[::-1]))
        self.assertTrue(S3C.arm_integrity_problems(att, dict(fin, kernel_faults_after=2), rows))
        self.assertTrue(S3C.arm_integrity_problems(None, fin, rows))
        with tempfile.TemporaryDirectory() as t:
            ref_rows, ref = S3C.s3b_reference(pathlib.Path(t))
            self.assertEqual((ref_rows, ref["status"]), ({}, "S3B_NOT_AVAILABLE"))
            (pathlib.Path(t) / "episodes.jsonl").write_text(
                json.dumps({"block": "A", "env": 3, "valid": True, "outcome": "capture"}) + "\n")
            ref_rows, ref = S3C.s3b_reference(pathlib.Path(t))
            self.assertEqual((list(ref_rows), ref["status"], ref["reading"]), ([3], "BOUND", "NO_READING"))


if __name__ == "__main__":
    unittest.main()
