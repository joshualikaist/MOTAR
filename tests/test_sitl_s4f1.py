"""CPU tests for the S4F-1 SITL unit (tools/sitl_s4f1.py): world text per cell, schedule and Latin-square order,
the per-cell smoke rule, the ULog gzip record, the analysis on synthetic episode records, and the preflight refusals.
No SITL, PX4, Gazebo or GPU is started; the live path runs against the S3b/S4 test fakes. Every negative check is
paired with a positive control. Modules under test are loaded by path; patched attributes are restored."""
import contextlib
import gzip
import hashlib
import importlib.util
import json
import pathlib
import re
import shutil
import sys
import tempfile
import types
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


S4F1 = _load("sitl_s4f1_under_test", TOOLS / "sitl_s4f1.py")
T3B = _load("test_sitl_s3b_formal_helpers_for_s4f1", ROOT / "tests" / "test_sitl_s3b_formal.py")
SF, S3C, S4, G = S4F1.SF, S4F1.S3C, S4F1.S4, S4F1.G
NS = types.SimpleNamespace
PX4_SG = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic")
CELLS = S4F1.CELL_NAMES

FAKE_BASE = ("<?xml version=\"1.0\" ?>\n<sdf version=\"1.5\">\n  <world name=\"default\">\n    <include>\n"
             "      <uri>model://sun</uri>\n    </include>\n    <!-- A ground plane -->\n    <include>\n"
             "      <uri>model://ground_plane</uri>\n    </include>\n    <include>\n"
             "      <uri>model://asphalt_plane</uri>\n    </include>\n    <physics name='default_physics' default='0' "
             "type='ode'><ode><solver><iters>10</iters></solver></ode></physics>\n  </world>\n</sdf>\n")


def s4_world(base=FAKE_BASE):
    """S4's world text exactly as tools/sitl_s4.run_episode composes it (one bar)."""
    V0 = _load("sitl_s3b_closed_loop_for_s4f1", TOOLS / "sitl_s3b_closed_loop.py")
    c, h = np.array([[3.0, 4.0, 1.5]]), np.array([[0.2, 0.3, 1.5]])
    return G.world_text_s4(SF.world_text(base, c, h, np.array([5.0, 6.0, 1.0]), 0.7, V0.ODE_ITERS, V0.SPAWN, V0.rot))


def include_spans(text):
    return [m.group(0) for _, pat, _ in S4F1.GROUND_INCLUDES for m in [pat.search(text)]]


# ------------------------------------------------------------------------------------------------ world per cell
class TestWorldText(unittest.TestCase):
    def setUp(self):
        self.s4 = s4_world()

    def test_lit_rendered_is_s4_byte_for_byte(self):
        out = S4F1.world_text_cell("lit_rendered", self.s4)
        self.assertEqual(out.encode(), self.s4.encode())
        self.assertEqual(out.count(S4F1.LIT_MATERIAL), 1)

    def test_each_cell_changes_exactly_its_factors(self):
        inc = include_spans(self.s4)
        self.assertEqual(len(inc), 2)
        for cell in CELLS:
            a, g = S4F1.FACTORS[cell]
            out = S4F1.world_text_cell(cell, self.s4)
            want = self.s4
            if a == "emissive":
                want = want.replace(S4F1.LIT_MATERIAL, S4F1.EMISSIVE_MATERIAL)
            if g == "unrendered":
                want = want.replace(inc[0], S4F1.GROUND_PLANE_COLLISION_ONLY).replace(
                    inc[1], S4F1.ASPHALT_PLANE_COLLISION_ONLY)
            self.assertEqual(out, want, cell)
            self.assertEqual(out.count(S4F1.EMISSIVE_MATERIAL), int(a == "emissive"), cell)
            self.assertEqual(out.count(S4F1.LIT_MATERIAL), int(a == "lit"), cell)
            self.assertEqual(out.count("model://ground_plane") + out.count("model://asphalt_plane"),
                             2 * int(g == "rendered"), cell)
            self.assertEqual(out.count("<model name='ground_plane'>") + out.count("<model name='asphalt_plane'>"),
                             2 * int(g == "unrendered"), cell)
            self.assertEqual(out.count("<visual"), self.s4.count("<visual"), cell)     # bars + target only
            self.assertEqual(out.count("model://sun"), 1)                               # lighting unchanged
            back = out.replace(S4F1.EMISSIVE_MATERIAL, S4F1.LIT_MATERIAL).replace(
                S4F1.GROUND_PLANE_COLLISION_ONLY, inc[0]).replace(S4F1.ASPHALT_PLANE_COLLISION_ONLY, inc[1])
            self.assertEqual(back, self.s4, cell)                                       # nothing else differs
        self.assertEqual(len({S4F1.world_text_cell(c, self.s4) for c in CELLS}), 4)

    def test_emissive_material_values(self):
        m = S4F1.EMISSIVE_MATERIAL
        for tag in ("ambient", "diffuse", "specular"):
            self.assertEqual(re.search("<%s>(.*?)</%s>" % (tag, tag), m).group(1), "0 0 0 1")
        em = [float(x) for x in re.search("<emissive>(.*?)</emissive>", m).group(1).split()]
        self.assertEqual(em, [round(v, 3) for v in G.TARGET_RGB] + [1.0])
        lit = S4F1.LIT_MATERIAL
        self.assertIn("<ambient>0.880 0.080 0.045 1</ambient>", lit)              # S4's own string
        self.assertEqual(self.s4.count("<emissive>"), 0)

    def test_unrendered_floor_keeps_collisions_without_visuals(self):
        for txt in (S4F1.GROUND_PLANE_COLLISION_ONLY, S4F1.ASPHALT_PLANE_COLLISION_ONLY):
            self.assertNotIn("<visual", txt)
            self.assertNotIn("<pose>", txt)                         # at the origin, as the includes place them
            self.assertEqual(txt.count("<collision name='collision'>"), 1)
            self.assertIn("<static>true</static>", txt)
        size = re.search(r"<box><size>(.*?)</size>", S4F1.ASPHALT_PLANE_COLLISION_ONLY).group(1).split()
        self.assertAlmostEqual(float(size[2]) / 2.0, 0.05)          # box top at z = 0.05 m: resting height unchanged

    def test_refusals_with_positive_controls(self):
        self.assertEqual(S4F1.world_problems(self.s4), [])
        no_inc = s4_world(FAKE_BASE.replace("model://asphalt_plane", "model://other"))
        self.assertTrue(any("asphalt_plane" in p for p in S4F1.world_problems(no_inc)))
        for cell in CELLS:                                          # even the unchanged cell refuses a foreign text
            with self.assertRaises(ValueError):
                S4F1.world_text_cell(cell, no_inc)
        no_mat = SF.world_text(FAKE_BASE, [], [], np.zeros(3), 0.0, 100, (0, 0), lambda y, a, b: (a, b))
        with self.assertRaises(ValueError):
            S4F1.world_text_cell("emissive_rendered", no_mat)
        with self.assertRaises(ValueError):
            S4F1.world_text_cell("emissive_rendered", self.s4.replace("</world>", S4F1.EMISSIVE_MATERIAL + "</world>"))
        with self.assertRaises(ValueError):
            S4F1.world_text_cell("lit", self.s4)

    def test_cell_session_writes_the_cell_world(self):
        with tempfile.TemporaryDirectory() as t:
            for cell in CELLS:
                rec = {}
                ns = S4F1.cell_os(T3B.FakeOS, cell, rec)
                self.assertIs(ns.direct_launch_plan, T3B.FakeOS.direct_launch_plan)
                with ns.OwnedSession(pathlib.Path(t) / cell) as s:
                    s.write_world("motar_s4", self.s4)
                got = (pathlib.Path(t) / cell / "world" / "motar_s4.world").read_text()
                self.assertEqual(got, S4F1.world_text_cell(cell, self.s4))
                self.assertEqual(rec["world_sha256"], hashlib.sha256(got.encode()).hexdigest())
                self.assertEqual(rec["s4_world_sha256"], hashlib.sha256(self.s4.encode()).hexdigest())
                self.assertEqual(rec["equals_s4_world"], cell == "lit_rendered")
                self.assertEqual(rec["cell"], cell)


@unittest.skipUnless((PX4_SG / "worlds/empty.world").is_file(), "PX4 tree not on this host")
class TestWorldTextRealTree(unittest.TestCase):
    @staticmethod
    def norm(t):
        return re.sub(r">\s+<", "><", t).replace('"', "'").strip()

    def test_real_empty_world(self):
        s4 = s4_world((PX4_SG / "worlds/empty.world").read_text())
        self.assertEqual(S4F1.world_problems(s4), [])
        self.assertEqual(S4F1.world_text_cell("lit_rendered", s4), s4)
        for cell in CELLS[1:]:
            self.assertNotEqual(S4F1.world_text_cell(cell, s4), s4, cell)

    def test_collisions_equal_the_px4_models(self):
        for name, txt in (("ground_plane", S4F1.GROUND_PLANE_COLLISION_ONLY),
                          ("asphalt_plane", S4F1.ASPHALT_PLANE_COLLISION_ONLY)):
            src = (PX4_SG / "models" / name / "model.sdf").read_text()
            col = re.search(r"<collision.*?</collision>", src, re.S).group(0)
            self.assertIn(self.norm(col), self.norm(txt), name)
            self.assertEqual(re.findall(r"<pose>", src), [], name)                # no pose: both at the origin
            self.assertIn("<visual", src)                                         # the original does render
            self.assertIn("<model name='%s'>" % name, txt)


# ------------------------------------------------------------------------------------------------ schedule
class TestSchedule(unittest.TestCase):
    envs = list(range(3, 3 + 2 * 40, 2))

    def test_latin_square_balanced(self):
        self.assertEqual(S4F1.latin_square_problems(), [])
        cyclic = ((0, 1, 2, 3), (1, 2, 3, 0), (2, 3, 0, 1), (3, 0, 1, 2))   # each position once, carry-over not
        self.assertTrue(any("carry-over" in p for p in S4F1.latin_square_problems(cyclic)))
        self.assertTrue(S4F1.latin_square_problems(((0, 1, 2, 2),) * 4))
        sched = S4F1.build_schedule(self.envs)
        self.assertEqual(len(sched), 160)
        for cell in CELLS:
            pos = [s["position"] for s in sched if s["arm"] == cell]
            self.assertEqual(len(pos), 40)
            self.assertEqual({p: pos.count(p) for p in range(4)}, {0: 10, 1: 10, 2: 10, 3: 10})
        for i, e in enumerate(self.envs):
            block = sched[4 * i:4 * i + 4]
            self.assertEqual({s["env"] for s in block}, {e})
            self.assertEqual(sorted(s["arm"] for s in block), sorted(CELLS))
            self.assertEqual([s["arm"] for s in block], [CELLS[j] for j in S4F1.LATIN_SQUARE[i % 4]])
        self.assertTrue(all(S4F1.FACTORS[s["arm"]] == (s["appearance"], s["ground"]) for s in sched))

    def test_design_check(self):
        sched = S4F1.build_schedule(self.envs)
        self.assertEqual(S4F1.design_problems(sched, self.envs), [])
        swapped = list(sched)
        swapped[4], swapped[5] = swapped[5], swapped[4]
        self.assertTrue(any("Latin-square" in p for p in S4F1.design_problems(swapped, self.envs)))
        self.assertTrue(S4F1.design_problems(sched[:-1], self.envs))
        self.assertTrue(any("first 40" in p for p in S4F1.design_problems(sched, [e + 1 for e in self.envs])))

    @unittest.skipUnless((SF.G8 / "zero").is_dir(), "G8 r2 raw inputs not on this host")
    def test_scenes_are_s4_grounds(self):
        s4g = [s["env"] for s in S4.schedule() if s["arm"] == "ground"]
        self.assertEqual(len(s4g), 40)
        self.assertEqual(S4F1.scene_envs(), s4g)
        sched = S4F1.schedule()
        self.assertEqual(S4F1.design_problems(sched, s4g), [])
        self.assertEqual([s["env"] for s in sched[::4]], s4g)


# ------------------------------------------------------------------------------------------------ smoke per cell
def write_smoke(d, cell, head="h", sources=None, valid=True, status="COMPLETE", arm=None, ekf_alt=1.01):
    d.mkdir(parents=True)
    (d / "attempt.json").write_text(json.dumps({"git": {"head": head}, "sources": sources or {"a": "1"},
                                                "development_smoke": True, "smoke_cell": cell,
                                                "protocol": S4F1.PROTOCOL}))
    (d / "final.json").write_text(json.dumps({"status": status}))
    sc = None if ekf_alt is None else {"ekf_alt_m": ekf_alt, "truth_alt_m": 1.0}
    (d / "episodes.jsonl").write_text(json.dumps({"valid": valid, "arm": arm or cell, "start_check": sc,
                                                  "invalid_reason": None if valid else "LATE"}) + "\n")


class TestSmokePerCell(unittest.TestCase):
    def test_smoke_items(self):
        envs = [7, 9, 11, 13]
        sched = S4F1.build_schedule(envs)
        items = [S4F1.smoke_items(sched, c) for c in CELLS]
        self.assertTrue(all(len(x) == 1 and x[0]["env"] == 7 for x in items))
        self.assertEqual([x[0]["arm"] for x in items], list(CELLS))
        self.assertEqual(sorted((x[0]["arm"], x[0]["env"]) for x in items),
                         sorted((s["arm"], s["env"]) for s in sched[:4]))           # the first scene, every cell
        with self.assertRaises(ValueError):
            S4F1.smoke_items(sched, "raw")

    def test_all_four_smokes_required(self):
        with tempfile.TemporaryDirectory() as t:
            dirs = {c: pathlib.Path(t) / ("smoke01_" + c) for c in CELLS}
            self.assertEqual(len(S4F1.smokes_problems("h", {"a": "1"}, dirs)), 4)    # none exists
            for c in CELLS:
                write_smoke(dirs[c], c)
            self.assertEqual(S4F1.smokes_problems("h", {"a": "1"}, dirs), [])        # positive control
            self.assertEqual(len(S4F1.smokes_problems("x", {"a": "1"}, dirs)), 4)    # other HEAD
            self.assertEqual(len(S4F1.smokes_problems("h", {"a": "2"}, dirs)), 4)    # other source bytes
            shutil.rmtree(dirs["lit_unrendered"])
            p = S4F1.smokes_problems("h", {"a": "1"}, dirs)
            self.assertEqual(len(p), 1)
            self.assertIn("lit_unrendered", p[0])
            write_smoke(dirs["lit_unrendered"], "lit_unrendered", valid=False)
            self.assertIn("lit_unrendered", S4F1.smokes_problems("h", {"a": "1"}, dirs)[0])
            shutil.rmtree(dirs["lit_unrendered"])
            write_smoke(dirs["lit_unrendered"], "lit_unrendered", arm="lit_rendered")   # another cell's episode
            p = S4F1.smokes_problems("h", {"a": "1"}, dirs)
            self.assertEqual(len(p), 1)
            self.assertIn("not cell lit_unrendered's", p[0])
            shutil.rmtree(dirs["lit_unrendered"])
            write_smoke(dirs["lit_unrendered"], "lit_unrendered", status="ABORTED")
            self.assertEqual(len(S4F1.smokes_problems("h", {"a": "1"}, dirs)), 1)
            with self.assertRaises(SystemExit):                                       # a smoke has no reading
                S4F1.analyze(dirs["lit_rendered"], isaac={})


# ------------------------------------------------------------------------------------------------ ULog gzip
def make_ulog(ep, name="01_02_03.ulg", seed=0):
    d = ep / "rootfs" / "log" / "2026-11-08"
    d.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed)
    data = b"ULog\x01" + rng.integers(0, 4, 300000, dtype=np.uint8).tobytes() + bytes(200000)
    (d / name).write_bytes(data)
    return d / name, data


class TestUlogGzip(unittest.TestCase):
    def test_gzip_verified_positive(self):
        with tempfile.TemporaryDirectory() as t:
            ep = pathlib.Path(t)
            src, data = make_ulog(ep)
            out = S4F1.gzip_ulogs(ep)
            self.assertEqual((out["count"], out["all_verified"]), (1, True))
            r = out["files"][0]
            self.assertEqual((r["sha256"], r["bytes"]), (hashlib.sha256(data).hexdigest(), len(data)))
            self.assertFalse(src.exists())
            gz = ep / r["gz_path"]
            self.assertEqual(gzip.decompress(gz.read_bytes()), data)
            self.assertEqual(r["gz_sha256"], hashlib.sha256(gz.read_bytes()).hexdigest())
            self.assertEqual(r["gz_bytes"], gz.stat().st_size)
            self.assertLess(r["gz_bytes"], r["bytes"])
            self.assertTrue(r["original_removed"])
            self.assertEqual(S4F1.verify_ulog_record(ep, r), [])
            gz.write_bytes(gzip.compress(b"other bytes"))                      # tampered afterwards
            self.assertEqual(len(S4F1.verify_ulog_record(ep, r)), 2)
            gz.write_bytes(b"not gzip")
            self.assertTrue(S4F1.verify_ulog_record(ep, r))
            gz.unlink()
            self.assertIn("missing", S4F1.verify_ulog_record(ep, r)[0])

    def test_deterministic_bytes(self):
        hs = []
        for _ in range(2):
            with tempfile.TemporaryDirectory() as t:
                make_ulog(pathlib.Path(t))
                hs.append(S4F1.gzip_ulogs(pathlib.Path(t))["files"][0]["gz_sha256"])
        self.assertEqual(hs[0], hs[1])                                           # mtime 0, no file name

    def test_failed_verification_keeps_the_original(self):
        def wrong(src, raw, level):
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as g:
                g.write(b"truncated")
        with tempfile.TemporaryDirectory() as t:
            ep = pathlib.Path(t)
            src, data = make_ulog(ep)
            r = S4F1.gzip_verified(src, ep, compress=wrong)
            self.assertFalse(r["verified"] or r["original_removed"])
            self.assertIn("round trip", r["error"])
            self.assertEqual(src.read_bytes(), data)                             # original intact
            self.assertEqual(sorted(p.name for p in src.parent.iterdir()), [src.name])   # no partial left
            self.assertTrue(S4F1.verify_ulog_record(ep, r))
            r2 = S4F1.gzip_verified(src, ep)                                     # positive control on the same file
            self.assertTrue(r2["verified"] and r2["original_removed"])

    def test_existing_gzip_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as t:
            ep = pathlib.Path(t)
            src, data = make_ulog(ep)
            old = src.with_name(src.name + ".gz")
            old.write_bytes(b"earlier")
            r = S4F1.gzip_verified(src, ep)
            self.assertFalse(r["verified"])
            self.assertIn("overwrite", r["error"])
            self.assertEqual((old.read_bytes(), src.read_bytes()), (b"earlier", data))

    def test_no_ulog(self):
        with tempfile.TemporaryDirectory() as t:
            out = S4F1.gzip_ulogs(pathlib.Path(t))
            self.assertEqual((out["count"], out["all_verified"]), (0, False))
            make_ulog(pathlib.Path(t), "a.ulg")
            make_ulog(pathlib.Path(t), "b.ulg", seed=1)
            out = S4F1.gzip_ulogs(pathlib.Path(t))
            self.assertEqual((out["count"], out["all_verified"]), (2, True))
        self.assertEqual(S4F1.gzip_ulogs(pathlib.Path("/nonexistent/s4f1"))["count"], 0)


# ------------------------------------------------------------------------------------------------ live path (fakes)
class FakeClient:
    made = []

    def __init__(self):
        self.targets, self.closed = [], False
        FakeClient.made.append(self)

    def set_target(self, x, y, z):
        self.targets.append((x, y, z))
        return 1.05

    @contextlib.contextmanager
    def bounded_io(self, deadline, service, **kwargs):
        service()
        yield
        service()

    def frames(self):
        t = 1.1 if self.targets else 0.0
        return {"sim_now": 1.2 if self.targets else 1.0, "lidar_t": t, "rgb_t": t, "depth_t": t,
                "target_t": 1.0 if self.targets else -1.0, "target_count": len(self.targets),
                "ranges": np.full((4, 72), 12.0), "rgb": np.zeros((90, 160, 3), np.uint8),
                "depth": np.full((90, 160), 25.0, np.float32)}

    def close(self):
        self.closed = True


def fake_bridge():
    import torch

    class Det:
        target_mask = torch.zeros(1, 90, 160)

        def render_raw_rgbd(self, *a):
            return torch.zeros(1, 3, 90, 160), torch.full((1, 90, 160), 20.0)

    class Per:
        def observe(self, **kw):
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


class UlogSession(T3B.FakeSession):
    """The fake owned session, with a PX4-like ULog in the episode's rootfs."""

    def __enter__(self):
        s = super().__enter__()
        make_ulog(self.episode_dir)
        return s


class TestLivePath(unittest.TestCase):
    def test_episode_through_s4_with_cell_world_and_ulog(self):
        tmp = pathlib.Path(tempfile.mkdtemp())
        try:
            V0, np_ = T3B.fake_v0(tmp, max_steps=4)
            (tmp / "empty.world").write_text(FAKE_BASE)
            OS = NS(**vars(T3B.FakeOS))
            OS.OwnedSession = UlogSession
            FakeClient.made = []
            B, pol = fake_bridge()
            cell = "emissive_unrendered"
            spec = T3B.fake_spec(np_)
            r = S4F1.run_episode(B, OS, V0, spec, tmp / cell, pol, cell, link_factory=T3B.FakeLink,
                                 client_factory=FakeClient)
            self.assertEqual(r["outcome"], "timeout", r.get("error"))
            self.assertEqual((r["arm"], r["cell"], r["s4_arm"], r["appearance"], r["ground"]),
                             (cell, cell, "ground", "emissive", "unrendered"))
            self.assertTrue(FakeClient.made[0].closed and r["link_closed"])
            self.assertEqual(r['initial_target_readiness']['preflight_packets'], 2)
            self.assertTrue(r['post_readiness_start_check']['offboard'])
            written = (tmp / cell / "world" / "motar_s4.world").read_text()
            want = S4F1.world_text_cell(cell, G.world_text_s4(SF.world_text(
                FAKE_BASE, spec["centers"], spec["halves"], spec["p0"], spec["yaw0"], V0.ODE_ITERS, V0.SPAWN, V0.rot)))
            self.assertEqual(written, want)
            self.assertEqual(r["cell_world"]["world_sha256"], hashlib.sha256(written.encode()).hexdigest())
            self.assertFalse(r["cell_world"]["equals_s4_world"])
            sens = np.load(tmp / cell / "sensor_steps.npy")
            self.assertEqual(sens.shape, (4, 16))
            self.assertTrue((sens[:, 11] >= 0).all() and (sens[:, 12] >= 0).all())   # ground filter ON
            u = r["ulog"]
            self.assertEqual((u["count"], u["all_verified"]), (1, True))
            self.assertEqual(list((tmp / cell).glob(S4F1.ULOG_GLOB)), [])
            self.assertEqual(S4F1.verify_ulog_record(tmp / cell, u["files"][0]), [])
            self.assertIn("dataman_removed_bytes", r)
            self.assertIsNotNone(r["late_max_s"])          # synthetic timestamps: not a lateness test
            # negative: a base world without the ground includes cannot become a cell world; the episode errs
            (tmp / "empty.world").write_text(FAKE_BASE.replace("model://ground_plane", "model://x"))
            r2 = S4F1.run_episode(B, OS, V0, spec, tmp / "bad", pol, cell, link_factory=T3B.FakeLink,
                                  client_factory=FakeClient)
            self.assertEqual(r2["outcome"], "ERROR")
            self.assertIn("not as expected", r2["error"])
            self.assertIsNone(r2["cell_world"])
            with self.assertRaises(ValueError):
                S4F1.run_episode(B, OS, V0, spec, tmp / "x", pol, "raw")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------------------------------------------ analysis
ENVS = list(range(100, 140))
WM = {k: {"path": "/x/" + k, "sha256": v} for k, v in S4F1.WORLD_MODEL_SHA256.items()}
BIND = {"files": {"px4_binary": {"sha256": "p"}, "s4_plugin": {"sha256": "s"}}}


def synth(outcome_fn, invalid=(), binding=None, envs=ENVS):
    """Synthetic attempt records: outcome_fn(scene_index, cell) -> outcome."""
    sched = S4F1.build_schedule(envs)
    rows = []
    for i, it in enumerate(sched):
        ok = (it["env"], it["arm"]) not in invalid
        base = hashlib.sha256(("s4world%s" % it["env"]).encode()).hexdigest()     # the scene's S4 world
        wsha = base if it["arm"] == "lit_rendered" else hashlib.sha256(("%s%s" % (it["arm"], it["env"])).encode()
                                                                        ).hexdigest()
        b = json.loads(json.dumps((binding or {}).get(it["arm"], BIND)))
        b["files"]["world"] = {"sha256": wsha}
        rows.append(dict(it, index=i, outcome=outcome_fn(it["scene_index"], it["arm"]) if ok else "ERROR", valid=ok,
                         invalid_reason=None if ok else "ERROR", late_max_s=0.02, truth_age_max_s=0.01,
                         compute_ms_max=30.0, real_time_factor=1.0, param_problems=[], runtime_binding=b,
                         cell_world={"cell": it["arm"], "world_sha256": wsha, "s4_world_sha256": base,
                                     "equals_s4_world": it["arm"] == "lit_rendered"},
                         start_check={"ekf_alt_m": 1.01, "truth_alt_m": 1.0},
                         sensor={"steps_motar_visible": 10, "steps_camera_visible_when_motar_visible":
                                 9 if it["appearance"] == "emissive" else 6,
                                 "steps_lidar_only_visible_when_motar_visible": 1, "frame_age_max_s": 0.05,
                                 "colour_px_median_when_motar_visible": 20.0 if it["appearance"] == "emissive" else 5.0,
                                 "motar_px_median_when_motar_visible": 26.0, "frame_age_median_s": 0.03,
                                 "lidar_abs_diff_median_m": 0.0, "depth_abs_diff_median_m": 0.0},
                         ulog={"count": 1, "all_verified": True, "files": [{"bytes": 100, "gz_bytes": 33,
                                                                             "verified": True}]}))
    attempt = {"protocol": S4F1.PROTOCOL, "schedule": sched, "kernel_faults_before": 0,
               "git": {"head": "h", "dirty": ""}, "world_models": WM}
    final = {"status": "COMPLETE", "reason": None, "kernel_faults_after": 0, "git_after": {"head": "h", "dirty": ""}}
    return attempt, final, rows


def read(attempt, final, rows, s4rows=None):
    isaac = {e: "capture" for e in ENVS}
    return S4F1.reading_from(attempt, final, rows, isaac, {}, {"status": "S3B_NOT_AVAILABLE"}, s4rows or {},
                             {"status": "TEST"}, envs=ENVS)


def lit_half(s, cell):
    return "capture" if s % 2 == 0 else "crash"


class TestSignTest(unittest.TestCase):
    def test_exact_values(self):
        self.assertEqual(S4F1.sign_test_p(0, 0), 1.0)
        self.assertEqual(S4F1.sign_test_p(5, 5), 1.0)
        self.assertAlmostEqual(S4F1.sign_test_p(9, 1), 22 / 1024)
        self.assertAlmostEqual(S4F1.sign_test_p(1, 9), 22 / 1024)
        self.assertAlmostEqual(S4F1.sign_test_p(10, 0), 2 / 1024)
        self.assertAlmostEqual(S4F1.sign_test_p(3, 0), 0.25)
        try:
            from scipy.stats import binomtest
        except ImportError:
            return
        for a, b in ((13, 3), (7, 2), (20, 0), (4, 6)):
            self.assertAlmostEqual(S4F1.sign_test_p(a, b), binomtest(a, a + b, 0.5).pvalue, places=12)


class TestAnalysis(unittest.TestCase):
    def test_known_main_effect_detected_and_null_factor_not(self):
        # emissive captures every scene; lit captures the even scenes; the ground changes nothing
        out = read(*synth(lambda s, c: "capture" if c.startswith("emissive") else lit_half(s, c)))
        self.assertEqual(out["integrity_problems"], [])
        cap = out["paired"]["capture"]
        self.assertEqual(cap["scenes_valid_in_all_cells"], 40)
        a, g, i = (cap["main_A_emissive_minus_lit"], cap["main_G_unrendered_minus_rendered"], cap["interaction_AxG"])
        self.assertEqual((a["mean_pp"], a["scenes_positive"], a["scenes_negative"]), (50.0, 20, 0))
        self.assertLess(a["sign_test_p_two_sided"], 1e-5)
        self.assertEqual((g["mean_pp"], g["scenes_positive"] + g["scenes_negative"], g["sign_test_p_two_sided"]),
                         (0.0, 0, 1.0))
        self.assertEqual(i["sign_test_p_two_sided"], 1.0)
        s = cap["simple"]
        self.assertEqual((s["A_given_rendered"]["x_only"], s["A_given_rendered"]["y_only"]), (20, 0))
        self.assertLess(s["A_given_unrendered"]["mcnemar_exact_p_two_sided"], 1e-5)
        self.assertEqual(s["G_given_lit"]["mcnemar_exact_p_two_sided"], 1.0)
        crash = out["paired"]["crash"]["main_A_emissive_minus_lit"]
        self.assertEqual((crash["mean_pp"], crash["scenes_negative"]), (-50.0, 20))

    def test_symmetric_null_not_detected(self):
        flips = {1, 3, 5, 0, 2, 4}             # 3 scenes emissive better, 3 worse, the rest equal

        def fn(s, c):
            o = lit_half(s, c)
            if c.startswith("emissive") and s in flips:
                o = "crash" if o == "capture" else "capture"
            return o
        a = read(*synth(fn))["paired"]["capture"]["main_A_emissive_minus_lit"]
        self.assertEqual((a["scenes_positive"], a["scenes_negative"], a["mean_pp"]), (3, 3, 0.0))
        self.assertGreater(a["sign_test_p_two_sided"], 0.05)

    def test_interaction_detected(self):
        # only the emissive target over the unrendered ground captures the odd scenes
        out = read(*synth(lambda s, c: "capture" if c == "emissive_unrendered" else lit_half(s, c)))
        cap = out["paired"]["capture"]
        i = cap["interaction_AxG"]
        self.assertEqual((i["scenes_positive"], i["scenes_negative"], i["mean_pp"]), (20, 0, 50.0))
        self.assertLess(i["sign_test_p_two_sided"], 1e-5)
        self.assertEqual(cap["simple"]["A_given_rendered"]["mcnemar_exact_p_two_sided"], 1.0)

    def test_only_scenes_valid_in_all_cells_are_paired(self):
        att, fin, rows = synth(lambda s, c: "capture" if c.startswith("emissive") else lit_half(s, c),
                               invalid={(101, "lit_unrendered"), (104, "emissive_rendered")})
        out = read(att, fin, rows)
        cap = out["paired"]["capture"]
        self.assertEqual((cap["scenes_valid_in_all_cells"], cap["scenes_with_any_valid"]), (38, 40))
        self.assertNotIn(101, cap["envs"])
        self.assertEqual(out["cells"]["lit_unrendered"]["valid"], 39)
        self.assertEqual(out["integrity_problems"], [])

    def test_cell_summaries(self):
        out = read(*synth(lambda s, c: "capture" if c.startswith("emissive") else
                          ("timeout" if s % 4 == 1 else lit_half(s, c))))
        for cell in CELLS:
            cs = out["cells"][cell]
            self.assertEqual(cs["valid"], 40)
            self.assertEqual(cs["capture_n"] + cs["crash_n"] + cs["timeout_n"], 40)
            for k in S4F1.OUTCOMES:
                ci = cs["%s_ci95_pct" % k]
                if ci is not None:
                    self.assertLessEqual(ci[0], cs["%s_pct" % k])
                    self.assertGreaterEqual(ci[1], cs["%s_pct" % k])
            rec = cs["detector_recall_steps"]
            self.assertEqual(rec["motar_visible_steps"], 400)
            self.assertAlmostEqual(rec["camera_recall"], 0.9 if cell.startswith("emissive") else 0.6)
        lr = out["cells"]["lit_rendered"]
        self.assertEqual((lr["capture_n"], lr["timeout_n"], lr["crash_n"]), (20, 10, 10))
        self.assertEqual(out["ulog"]["files"], 160)
        self.assertEqual(out["timing_and_state"]["lit_rendered"]["late_max_s"], 0.02)

    def test_vs_s4_ground(self):
        att, fin, rows = synth(lit_half)
        s4rows = {e: {"outcome": "capture"} for e in ENVS[:30]}
        v = read(att, fin, rows, s4rows)["lit_rendered_vs_s4_ground"]
        self.assertEqual((v["pairs"], v["s4_capture"], v["s4f1_capture"], v["capture_s4_only"]), (30, 30, 15, 15))
        self.assertEqual(v["s4_to_s4f1_outcomes"], {"capture->capture": 15, "capture->crash": 15})
        with tempfile.TemporaryDirectory() as t:
            led = pathlib.Path(t) / "episodes.jsonl"
            led.write_text(json.dumps({"arm": "ground", "env": 3, "valid": True, "outcome": "capture"}) + "\n"
                           + json.dumps({"arm": "raw", "env": 3, "valid": True, "outcome": "crash"}) + "\n")
            rows4, ref = S4F1.s4_reference(led)
            self.assertEqual((rows4, ref["status"]), ({}, "S4_LEDGER_SHA_MISMATCH"))   # the pin refuses it
            rows4, ref = S4F1.s4_reference(led, SF.sha256_file(led))
            self.assertEqual((list(rows4), ref["status"]), ([3], "BOUND"))
            self.assertEqual(S4F1.s4_reference(pathlib.Path(t) / "none")[1]["status"], "S4_NOT_AVAILABLE")

    @unittest.skipUnless(S4F1.S4_LEDGER.is_file(), "S4 attempt01 ledger not on this host")
    def test_real_s4_ledger_is_bound(self):
        rows4, ref = S4F1.s4_reference()
        self.assertEqual(ref["status"], "BOUND")
        self.assertEqual(len(rows4), 39)                                          # 39 of 40 VALID (README)
        self.assertEqual(sum(1 for r in rows4.values() if r["outcome"] == "capture"), 20)

    def test_bad_inputs_refused(self):
        att, fin, rows = synth(lit_half)
        self.assertTrue(read(att, fin, rows))                                    # positive control
        dup = rows + [dict(rows[0])]
        with self.assertRaises(ValueError):
            S4F1.paired_effects(dup)
        with self.assertRaises(ValueError):
            S4F1.paired_effects([dict(rows[0], arm="ground")] + rows[1:])
        with self.assertRaises(ValueError):
            S4F1.paired_effects([dict(rows[0], outcome="ERROR")] + rows[1:])
        with self.assertRaises(ValueError):
            S4F1.paired_effects(rows, "landing")
        self.assertEqual(S4F1.paired_effects([dict(rows[0], outcome="ERROR", valid=False)] + rows[1:])
                         ["scenes_valid_in_all_cells"], 39)                      # an INVALID row is only excluded
        with self.assertRaises(ValueError):
            S4F1.reading_from(dict(att, development_smoke=True), fin, rows, {}, {}, {}, {}, {})

    def test_integrity_problems_detected(self):
        att, fin, rows = synth(lit_half)
        self.assertEqual(read(att, fin, rows)["integrity_problems"], [])
        other = {"files": {"px4_binary": {"sha256": "q"}, "s4_plugin": {"sha256": "s"}}}
        _, _, rows_b = synth(lit_half, binding={"emissive_unrendered": other})
        self.assertTrue(any("differs across cells" in p for p in read(att, fin, rows_b)["integrity_problems"]))
        rows_w = [dict(r) for r in rows]
        rows_w[5] = dict(rows_w[5], cell_world=dict(rows_w[5]["cell_world"], world_sha256="0" * 64))
        self.assertTrue(any("bound world" in p for p in read(att, fin, rows_w)["integrity_problems"]))
        rows_c = [dict(r) for r in rows]
        self.assertEqual(rows_c[1]["arm"], "emissive_rendered")
        rows_c[1] = dict(rows_c[1], cell_world=dict(rows_c[1]["cell_world"], cell="lit_rendered"))
        self.assertTrue(any("is not its cell" in p for p in read(att, fin, rows_c)["integrity_problems"]))
        rows_e = [dict(r) for r in rows]
        rows_e[1] = dict(rows_e[1], cell_world=dict(rows_e[1]["cell_world"], equals_s4_world=True))
        self.assertTrue(any("equals_s4_world" in p for p in read(att, fin, rows_e)["integrity_problems"]))
        swapped = dict(att, schedule=list(att["schedule"]))
        swapped["schedule"][0], swapped["schedule"][1] = swapped["schedule"][1], swapped["schedule"][0]
        self.assertTrue(any("Latin-square" in p for p in read(swapped, fin, rows)["integrity_problems"]))
        self.assertTrue(read(dict(att, protocol="s4-v1"), fin, rows)["integrity_problems"])
        self.assertTrue(read(att, dict(fin, kernel_faults_after=1), rows)["integrity_problems"])
        self.assertTrue(read(att, fin, rows[:-1])["integrity_problems"])        # COMPLETE but one short

    def test_analyze_writes_once(self):
        att, fin, rows = synth(lit_half)
        with tempfile.TemporaryDirectory() as t:
            d = pathlib.Path(t) / "attempt01"
            d.mkdir()
            (d / "attempt.json").write_text(json.dumps(att))
            (d / "final.json").write_text(json.dumps(fin))
            (d / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
            isaac = {e: "capture" for e in ENVS}
            kw = dict(s3b_attempt=pathlib.Path(t) / "no_s3b", s4_ledger=pathlib.Path(t) / "no_s4", isaac=isaac,
                      world_models_now=WM, envs=ENVS)
            out = S4F1.analyze(d, **kw)
            self.assertEqual((out["label"], out["integrity_problems"]), ("REPORTED", []))
            self.assertEqual(out["s4_reference"]["status"], "S4_NOT_AVAILABLE")
            self.assertEqual(out["replication"]["status"], "S4_NOT_AVAILABLE")
            self.assertEqual(len(out["ulog"]["file_problems"]), 160)       # the synthetic rows have no gzip files
            self.assertEqual(out["manipulation_checks"]["lit_rendered"], {"episodes": 0, "missing": 40})
            first = (d / "reading.json").read_text()
            S4F1.analyze(d, **kw)                                                # identical rerun: allowed
            self.assertEqual((d / "reading.json").read_text(), first)
            (d / "reading.json").write_text(first.replace("COMPLETE", "ABORTED"))
            with self.assertRaises(SystemExit):                                  # other bytes: refused
                S4F1.analyze(d, **kw)
            (d / "reading.json").unlink()
            (d / "episodes.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows + [rows[0]]))
            with self.assertRaises(SystemExit):                                  # malformed ledger: no reading
                S4F1.analyze(d, **kw)
            self.assertFalse((d / "reading.json").exists())


# ------------------------------------------------------------------------------------------------ preflight
class TestPreflight(unittest.TestCase):
    def test_extra_problems(self):
        with tempfile.TemporaryDirectory() as t:
            dirs = {c: pathlib.Path(t) / c for c in CELLS}
            for c in CELLS:
                write_smoke(dirs[c], c)
            so, src = pathlib.Path(t) / "p.so", pathlib.Path(t) / "p.cc"
            so.write_bytes(b"so")
            src.write_bytes(b"cc")
            res = pathlib.Path(t) / "result.json"
            runner = pathlib.Path(t) / "runner.py"
            runner.write_text("r")
            res.write_text(json.dumps({"verdict": "PASS", "criteria": {c: True for c in S4F1.COMPONENT_CRITERIA},
                                       "script_sha256": "s", "runner_sha256": SF.sha256_file(runner)}))
            ok = dict(s4_problems=[], free=10e9, smoke_dirs=dirs, world_models=WM,
                      plugin=(so, src, (SF.sha256_file(so), SF.sha256_file(src))),
                      component=dict(result=res, runner=runner, script_sha="s"))
            self.assertEqual(S4F1.launch_extra_problems(None, "h", {"a": "1"}, **ok), [])        # positive
            self.assertEqual(S4F1.launch_extra_problems("lit_rendered", "x", {}, **ok), [])      # smokes: no gate
            p = S4F1.launch_extra_problems(None, "h", {"a": "1"}, **dict(ok, free=1.0e9))
            self.assertEqual(len(p), 1)
            self.assertIn("disk free", p[0])
            p = S4F1.launch_extra_problems(None, "h", {"a": "1"}, **dict(ok, s4_problems=["no DISPLAY"]))
            self.assertEqual(p, ["no DISPLAY"])
            self.assertTrue(S4F1.launch_extra_problems(None, "other", {"a": "1"}, **ok))
            self.assertTrue(S4F1.launch_extra_problems("raw", "h", {}, **ok))
            p = S4F1.launch_extra_problems("lit_rendered", "h", {}, **dict(ok, plugin=(so, src, ("x", "y"))))
            self.assertEqual(len(p), 1)
            self.assertIn("pinned bytes", p[0])
            p = S4F1.launch_extra_problems("lit_rendered", "h", {}, **dict(ok, world_models={}))
            self.assertTrue(any("world model files" in x for x in p))
            p = S4F1.launch_extra_problems("lit_rendered", "h", {}, **dict(ok, component=dict(
                result=pathlib.Path(t) / "none.json", runner=runner, script_sha="s")))
            self.assertEqual(len(p), 1)
            self.assertIn("component smoke", p[0])
        self.assertEqual(S4F1.disk_problems(free=S4F1.DISK_FREE_LAUNCH_B), [])
        self.assertTrue(S4F1.disk_problems(free=S4F1.DISK_FREE_LAUNCH_B - 1))
        self.assertGreater(S4F1.disk_free(pathlib.Path(tempfile.gettempdir()) / "no" / "such"), 0)

    def test_approval_names_this_scope_and_head(self):
        head = "c" * 40
        appr = {"approved_by": "user", "approved_at": "2026-11-08T15:00:00+09:00", "scope": S4F1.SCOPE,
                "commit": head}
        self.assertNotEqual(S4F1.SCOPE, S4.SCOPE)
        self.assertEqual(SF.approval_problems(appr, head, scope=S4F1.SCOPE), [])
        self.assertTrue(any("scope" in p for p in SF.approval_problems(dict(appr, scope=S4.SCOPE), head,
                                                                         scope=S4F1.SCOPE)))
        self.assertTrue(any("commit" in p for p in SF.approval_problems(appr, "d" * 40, scope=S4F1.SCOPE)))

    def test_launch_problems_with_this_scope(self):
        tmp = pathlib.Path(tempfile.mkdtemp())
        old = (SF.F_SHA256, SF.input_problems, SF.environment_report)
        try:
            f = tmp / "f.pth"
            f.write_bytes(b"policy")
            SF.F_SHA256 = SF.sha256_file(f)
            SF.input_problems = lambda *a, **k: []
            SF.environment_report = lambda: ({}, [])
            head = "a" * 40
            ap = tmp / "execution_approval.json"
            B = NS(ROOT=SF.ROOT, F_CKPT=f)

            def probs(scope):
                ap.write_text(json.dumps({"approved_by": "u", "approved_at": "t", "scope": scope, "commit": head}))
                return SF.launch_problems(T3B.FakeAT(), B, {"head": head, "dirty": ""}, ap, scope=S4F1.SCOPE,
                                          sources=[], attempt_dir=tmp / "attempt01")
            ours = [p for p in probs(S4F1.SCOPE) if "environment variable" not in p and "command yaw" not in p]
            self.assertEqual(ours, [])
            self.assertTrue(any("scope" in p for p in probs(S4.SCOPE)))
            (tmp / "attempt01").mkdir()
            self.assertTrue(any("single use" in p for p in probs(S4F1.SCOPE)))
        finally:
            SF.F_SHA256, SF.input_problems, SF.environment_report = old
            shutil.rmtree(tmp, ignore_errors=True)

    def test_cli_and_constants(self):
        import contextlib
        import io
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            S4F1.main(["--smoke", "raw"])
        with self.assertRaises(SystemExit):
            S4F1.launch(smoke_cell="raw")
        self.assertEqual(set(S4F1.SMOKE_DIRS), set(CELLS))
        self.assertEqual(len({str(p) for p in S4F1.SMOKE_DIRS.values()} | {str(S4F1.ATTEMPT_DIR)}), 5)
        self.assertEqual(S4F1.S4_ARM, "ground")
        self.assertTrue(set(S4.OWN_SOURCES) <= set(S4F1.OWN_SOURCES))
        self.assertIn("tests/test_sitl_s4f1.py", S4F1.OWN_SOURCES)
        self.assertEqual(S4F1.INVALID_ABORT_TOTAL, 16)


# ------------------------------------------------------------------------------------------------ review r1 repairs
AGE = "plugin /x/libmotar_s4_bridge.so is older than its source /y.cc: rebuild it from the committed source"


class TestPluginPins(unittest.TestCase):
    """MAJOR-1: S4's mtime rule is dropped only when the .so and source are S4 attempt01's pinned bytes."""

    def test_pins(self):
        with tempfile.TemporaryDirectory() as t:
            so, src = pathlib.Path(t) / "a.so", pathlib.Path(t) / "a.cc"
            so.write_bytes(b"binary")
            src.write_bytes(b"source")
            pins = (SF.sha256_file(so), SF.sha256_file(src))
            self.assertEqual(S4F1.plugin_pin_problems([AGE, "no DISPLAY"], so, src, pins), ["no DISPLAY"])
            for bad in ((pins[0], "0" * 64), ("0" * 64, pins[1])):
                p = S4F1.plugin_pin_problems([AGE], so, src, bad)
                self.assertEqual(len(p), 2)                                        # the age rule stays
                self.assertIn("pinned bytes", p[1])
            p = S4F1.plugin_pin_problems([], pathlib.Path(t) / "none.so", src, pins)
            self.assertIn("missing", p[0])
        self.assertEqual(S4F1.S4_PLUGIN_SHA256[:8], "9b3109af")
        self.assertEqual(S4F1.S4_PLUGIN_SOURCE_SHA256[:8], "661a4d71")
        self.assertIn(S4F1.S4_PLUGIN_AGE, AGE)

    @unittest.skipUnless((S4.PLUGIN_DIR / "libmotar_s4_bridge.so").is_file(), "S4 plugin not built on this host")
    def test_real_s4_bytes(self):
        self.assertEqual(SF.sha256_file(S4.PLUGIN_SRC), S4F1.S4_PLUGIN_SOURCE_SHA256)
        self.assertEqual(S4F1.plugin_pin_problems([AGE]), [])                    # S4's own binary is accepted


class TestWorldModelsAndComponentGate(unittest.TestCase):
    def test_world_model_pins(self):
        self.assertEqual(S4F1.world_model_problems(WM), [])
        one = dict(WM, **{"sun/model.sdf": {"sha256": "0" * 64}})
        self.assertIn("sun/model.sdf", S4F1.world_model_problems(one)[0])
        less = {k: v for k, v in WM.items() if k != "ground_plane/model.sdf"}
        self.assertIn("ground_plane/model.sdf", S4F1.world_model_problems(less)[0])
        self.assertTrue(S4F1.world_model_problems(None))
        self.assertTrue(S4F1.world_model_problems({"error": "x"}))

    @unittest.skipUnless((PX4_SG / "models/sun").is_dir(), "PX4 tree not on this host")
    def test_real_world_models(self):
        files = S4F1.world_model_files()
        self.assertEqual(S4F1.world_model_problems(files), [])
        self.assertIn("asphalt_plane/materials/textures/tarmac.png", files)
        self.assertEqual(len(files), 9)

    def test_component_smoke_gate(self):
        with tempfile.TemporaryDirectory() as t:
            res, runner = pathlib.Path(t) / "result.json", pathlib.Path(t) / "runner.py"
            runner.write_text("runner bytes")
            good = {"verdict": "PASS", "criteria": {c: True for c in S4F1.COMPONENT_CRITERIA},
                    "script_sha256": "s", "runner_sha256": SF.sha256_file(runner)}
            kw = dict(result=res, runner=runner, script_sha="s")
            self.assertIn("no component smoke", S4F1.component_smoke_problems(**kw)[0])
            res.write_text(json.dumps(good))
            self.assertEqual(S4F1.component_smoke_problems(**kw), [])
            for change, needle in ((dict(verdict="FAIL"), "not PASS"),
                                   (dict(criteria=dict(good["criteria"], C4=False)), "not PASS"),
                                   (dict(criteria={c: True for c in S4F1.COMPONENT_CRITERIA[:-1]}), "not PASS"),
                                   (dict(script_sha256="other"), "another script"),
                                   (dict(runner_sha256="other"), "another runner")):
                res.write_text(json.dumps(dict(good, **change)))
                p = S4F1.component_smoke_problems(**kw)
                self.assertEqual(len(p), 1, change)
                self.assertIn(needle, p[0])
            res.write_text("{")
            self.assertIn("unreadable", S4F1.component_smoke_problems(**kw)[0])

    @unittest.skipUnless(S4F1.COMPONENT_SMOKE.is_file(), "component smoke script not on this host")
    def test_pinned_script_is_the_prepared_one(self):
        self.assertEqual(SF.sha256_file(S4F1.COMPONENT_SMOKE), S4F1.COMPONENT_SMOKE_SHA256)

    @unittest.skipUnless((ROOT / S4F1.PREREG).is_file(), "S4F-1 prereg not in this tree")
    def test_prereg_binds_the_runner_and_the_script(self):
        """Review r2 N3: the SHA-256s the prereg states are the runner's bytes and the pinned smoke script's."""
        text = (ROOT / S4F1.PREREG).read_text()
        self.assertIn(SF.sha256_file(TOOLS / "sitl_s4f1.py"), text)
        self.assertIn(S4F1.COMPONENT_SMOKE_SHA256, text)
        self.assertNotIn("0" * 64, text)                                         # negative control: no placeholder


class TestStartAltitudeGate(unittest.TestCase):
    def test_smoke_gate(self):
        with tempfile.TemporaryDirectory() as t:
            dirs = {c: pathlib.Path(t) / c for c in CELLS}
            for c in CELLS:
                write_smoke(dirs[c], c, ekf_alt=1.0 + (0.049 if c == "lit_unrendered" else -0.02))
            self.assertEqual(S4F1.smokes_problems("h", {"a": "1"}, dirs), [])        # within 0.05 m
            for alt in (1.06, 0.94, None):
                shutil.rmtree(dirs["emissive_unrendered"])
                write_smoke(dirs["emissive_unrendered"], "emissive_unrendered", ekf_alt=alt)
                p = S4F1.smokes_problems("h", {"a": "1"}, dirs)
                self.assertEqual(len(p), 1, alt)
                self.assertIn("rangefinder", p[0])
        self.assertAlmostEqual(S4F1.start_alt_diff({"start_check": {"ekf_alt_m": 1.02, "truth_alt_m": 1.0}}), 0.02)
        self.assertIsNone(S4F1.start_alt_diff({}))


class TestRunOne(unittest.TestCase):
    """LOW-8 and MINOR-5: the per-episode disk stop and GPU-job check, through run_unit."""

    def go(self, free, gpu, calls):
        def episode(B, OS, V0, spec, ep_dir, policy, cell):
            calls.append(cell)
            return {"outcome": "capture", "param_problems": [], "late_max_s": 0.01}
        sched = [{"arm": "lit_rendered", "bars": 70, "env": 0}, {"arm": "emissive_rendered", "bars": 70, "env": 0}]
        specs = {0: {"target_source": "zero_action_cell", "tpos": [0] * 600}}
        with tempfile.TemporaryDirectory() as t:
            run_one = S4F1.make_run_one(None, None, None, None, pathlib.Path(t), free_fn=lambda d: free,
                                        gpu_fn=gpu, wait_s=0, episode_fn=episode)
            SF.run_unit.terminated = None
            st = SF.run_unit(sched, specs, run_one, T3B.FakeAT(), pathlib.Path(t) / "l", key="arm", primary=None,
                             busy_wait_s=0)
            led = pathlib.Path(t) / "l"
            rows = [json.loads(x) for x in led.read_text().splitlines()] if led.exists() else []
        return st, rows

    def test_positive(self):
        calls = []
        st, rows = self.go(10e9, lambda: (True, [], ""), calls)
        self.assertEqual((st[0], len(rows), calls), ("COMPLETE", 2, ["lit_rendered", "emissive_rendered"]))
        self.assertTrue(rows[0]["host_gpu_before"]["gpu_ok"])
        self.assertIsNotNone(rows[0]["host_gpu_before"]["loadavg"])

    def test_disk_low_aborts_without_a_row(self):
        calls = []
        st, rows = self.go(0.4e9, lambda: (True, [], ""), calls)
        self.assertEqual((st[0], st[2], rows, calls), ("ABORTED", 0, [], []))
        self.assertIn("DISK_LOW", st[1])

    def test_gpu_job_is_invalid_without_running(self):
        for gpu, why in ((lambda: (False, ["123, python"], "123, python"), "GPU_BUSY"),
                         (lambda: (None, [], "rc 9"), "GPU_UNCHECKED")):
            calls = []
            st, rows = self.go(10e9, gpu, calls)
            self.assertEqual((st[0], calls), ("ABORTED", []))                    # the first episode is INVALID
            self.assertEqual((rows[0]["valid"], rows[0]["invalid_reason"]), (False, why))

    def test_gpu_jobs_parsing(self):
        import subprocess
        real = subprocess.run
        try:
            subprocess.run = lambda *a, **k: NS(returncode=0, stdout="9, /usr/share/rustdesk/rustdesk\n")
            self.assertEqual(S4F1.gpu_jobs()[:2], (True, []))
            subprocess.run = lambda *a, **k: NS(returncode=0, stdout="9, /usr/share/rustdesk/rustdesk\n7, python\n")
            self.assertEqual(S4F1.gpu_jobs()[:2], (False, ["7, python"]))
            subprocess.run = lambda *a, **k: NS(returncode=6, stdout="")
            self.assertIsNone(S4F1.gpu_jobs()[0])
        finally:
            subprocess.run = real


def s4rows_for(rows, change_env=None, core=None):
    out = {}
    for r in rows:
        if r["arm"] == "lit_rendered" and r["valid"]:
            b = json.loads(json.dumps(core or BIND))
            w = r["runtime_binding"]["files"]["world"]["sha256"]
            b["files"]["world"] = {"sha256": "f" * 64 if r["env"] == change_env else w}
            out[r["env"]] = {"outcome": "capture", "runtime_binding": b, "valid": True}
    return out


class TestReplicationAndLabels(unittest.TestCase):
    def test_replication(self):
        att, fin, rows = synth(lit_half)
        ref = {"status": "BOUND"}
        rep = S4F1.replication(rows, s4rows_for(rows), ref, att, WM)
        self.assertEqual((rep["status"], rep["worlds_compared"], rep["worlds_equal"]),
                         ("SAME_STACK_AND_WORLDS", 40, 40))
        rep = S4F1.replication(rows, s4rows_for(rows, change_env=107), ref, att, WM)
        self.assertEqual(rep["status"], "DIFFERS")
        self.assertIn("[107]", rep["problems"][0])
        other = {"files": {"px4_binary": {"sha256": "q"}, "s4_plugin": {"sha256": "s"}}}
        rep = S4F1.replication(rows, s4rows_for(rows, core=other), ref, att, WM)
        self.assertIn("px4_binary", rep["problems"][0])
        rep = S4F1.replication(rows, s4rows_for(rows), ref, att, dict(WM, **{"sun/model.sdf": {"sha256": "1"}}))
        self.assertIn("changed since the launch", rep["problems"][0])
        # review r2 N4: a comparison that could not be made gives INCOMPLETE, never SAME_STACK_AND_WORLDS
        full = S4F1.replication(rows, s4rows_for(rows), ref, att, WM)
        self.assertEqual((full["models_compared"], full["stack_compared"]), (True, True))
        for kw, flag in ((dict(world_models_now=None), "models_compared"),
                         (dict(attempt=dict(att, world_models=None)), "models_compared")):
            args = {"attempt": att, "world_models_now": WM}
            args.update(kw)
            rep = S4F1.replication(rows, s4rows_for(rows), ref, args["attempt"], args["world_models_now"])
            self.assertEqual((rep["status"], rep[flag]), ("INCOMPLETE", False))
        other_envs = {e + 1000: r for e, r in s4rows_for(rows).items()}               # no env in common
        rep = S4F1.replication(rows, other_envs, ref, att, WM)
        self.assertEqual((rep["status"], rep["worlds_compared"]), ("INCOMPLETE", 0))
        no_valid = [dict(r, valid=False) for r in rows]
        self.assertEqual(S4F1.replication(no_valid, s4rows_for(rows), ref, att, WM)["status"], "INCOMPLETE")
        self.assertEqual(S4F1.replication(rows, {}, {"status": "S4_NOT_AVAILABLE"})["status"], "S4_NOT_AVAILABLE")
        out = S4F1.reading_from(att, fin, rows, {e: "capture" for e in ENVS}, {}, {}, s4rows_for(rows), ref,
                                envs=ENVS, world_models_now=WM)
        self.assertEqual(out["replication"]["status"], "SAME_STACK_AND_WORLDS")

    def test_one_s4_world_per_scene(self):
        att, fin, rows = synth(lit_half)
        self.assertEqual(S4F1.record_problems(rows), [])
        r1 = [dict(r) for r in rows]
        r1[1] = dict(r1[1], cell_world=dict(r1[1]["cell_world"], s4_world_sha256="0" * 64))
        self.assertTrue(any("different S4 worlds" in p for p in S4F1.record_problems(r1)))
        r2 = [dict(r) for r in rows]
        i = next(j for j, r in enumerate(r2) if r["arm"] == "lit_rendered")
        w = "e" * 64
        r2[i] = dict(r2[i], cell_world=dict(r2[i]["cell_world"], world_sha256=w),
                     runtime_binding={"files": dict(r2[i]["runtime_binding"]["files"], world={"sha256": w})})
        self.assertTrue(any("not the scene's S4 world" in p for p in S4F1.record_problems(r2)))

    def test_labels(self):
        fn = lambda s, c: "capture" if c.startswith("emissive") else lit_half(s, c)          # noqa: E731
        att, fin, rows = synth(fn)
        out = read(att, fin, rows)
        self.assertEqual((out["label"], out["label_reason"]), ("REPORTED", None))
        a, g = out["primary"]["contrasts"]
        self.assertEqual((a["contrast"], g["contrast"]),
                         ("main_A_emissive_minus_lit", "main_G_unrendered_minus_rendered"))
        self.assertEqual((a["reached"], g["reached"]), (True, False))
        self.assertAlmostEqual(a["p_holm"], min(1.0, 2 * a["p_raw"]))
        void = read(att, dict(fin, status="ABORTED", reason="x"), rows)
        self.assertEqual(void["label"], "VOID")
        self.assertNotIn("cells", void)
        self.assertNotIn("paired", void)
        self.assertEqual(read(att, dict(fin, kernel_faults_after=1), rows)["label"], "VOID")
        self.assertEqual(read(dict(att, protocol="x"), fin, rows)["label"], "VOID")
        drop = {(e, "lit_unrendered") for e in ENVS[:11]}                           # 29 complete scenes
        out = read(*synth(fn, invalid=drop))
        self.assertEqual(out["label"], "INSUFFICIENT")
        self.assertIsNone(out["primary"]["contrasts"][0]["reached"])
        self.assertEqual(out["invalid_per_cell"]["lit_unrendered"], {"ERROR": 11})
        self.assertEqual(out["cells"]["lit_unrendered"]["invalid_n"], 11)
        out = read(*synth(fn, invalid={(e, "lit_unrendered") for e in ENVS[:10]}))  # 30: read
        self.assertEqual(out["label"], "REPORTED")

    def test_holm(self):
        self.assertEqual(S4F1.holm([0.01, 0.04]), [0.02, 0.04])
        self.assertEqual(S4F1.holm([0.04, 0.01]), [0.04, 0.02])
        self.assertEqual(S4F1.holm([0.03, 0.03]), [0.06, 0.06])
        self.assertEqual(S4F1.holm([0.6, 0.9]), [1.0, 1.0])

    def test_s3b_pin(self):
        with tempfile.TemporaryDirectory() as t:
            (pathlib.Path(t) / "episodes.jsonl").write_text(
                json.dumps({"block": "A", "env": 3, "valid": True, "outcome": "capture"}) + "\n")
            rows, ref = S4F1.s3b_reference(pathlib.Path(t))
            self.assertEqual((rows, ref["status"]), ({}, "S3B_LEDGER_SHA_MISMATCH"))
            rows, ref = S4F1.s3b_reference(pathlib.Path(t), SF.sha256_file(pathlib.Path(t) / "episodes.jsonl"))
            self.assertEqual((list(rows), ref["status"]), ([3], "BOUND"))

    @unittest.skipUnless((SF.ATTEMPT_DIR / "episodes.jsonl").is_file(), "S3b attempt01 not on this host")
    def test_real_s3b_pin(self):
        self.assertEqual(S4F1.s3b_reference()[1]["status"], "BOUND")

    @unittest.skipUnless(S4F1.S4_LEDGER.is_file(), "S4 attempt01 ledger not on this host")
    def test_real_s4_ground_has_one_core_and_39_worlds(self):
        rows4, ref = S4F1.s4_reference()
        self.assertEqual(len({S4F1._core(r) for r in rows4.values()}), 1)
        self.assertEqual(len({S4F1._bound_world(r) for r in rows4.values()}), 39)
        self.assertIn("s4_plugin", json.loads(S4F1._core(next(iter(rows4.values())))))

    def test_manipulation_checks(self):
        att, fin, rows = synth(lit_half)
        with tempfile.TemporaryDirectory() as t:
            for r in rows[:8]:
                d = pathlib.Path(t) / ("ep%03d" % r["index"])
                d.mkdir()
                S = np.zeros((5, 16))
                S[:, 5], S[:, 6] = 5.0, 20.0
                S[2, 6] = 1.0                                                   # shadow < 2 px: not a ratio step
                S[:, 11] = 6.0 if r["ground"] == "rendered" else 0.0
                S[:, 12] = 900.0 if r["ground"] == "rendered" else 30.0
                np.save(d / "sensor_steps.npy", S)
            m = S4F1.manipulation_checks(rows, pathlib.Path(t))
        lr, lu = m["lit_rendered"], m["lit_unrendered"]
        self.assertEqual((lr["episodes"], lr["missing"]), (2, 38))
        self.assertEqual(lr["lidar_ground_removed_per_step"], {"median": 6.0, "max": 6.0})
        self.assertEqual(lu["depth_ground_removed_per_step"]["max"], 30.0)
        self.assertEqual(lu["hover_step0"]["episodes_lidar_removed_gt0"], 0)
        self.assertEqual(lr["hover_step0"]["episodes_lidar_removed_gt0"], 2)
        self.assertAlmostEqual(lr["colour_to_shadow_px_ratio_median"], 0.25)
        self.assertEqual(lr["shadow_visible_steps"], 8)


if __name__ == "__main__":
    unittest.main()
