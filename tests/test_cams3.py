"""CPU tests for the CAM-S3 opt-in rules (aerial_gym/task/navrl_task/navrl_cams3.py), their install through
tools/realveh_entry.py, the S3-W driver's contract (tools/cams3_train.py), the mask-library builder and the thin bar pool.

No GPU, Isaac Gym or simulator. Safe inside the single-process PUBLIC_CPU suite: torch-dependent checks run in a child
process with path-only package stubs (the pattern of tests/test_depth_validity.py); in this process modules are loaded
by file path under private names. Every negative check is paired with a positive control.
Run: PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B tests/test_cams3.py
"""
import hashlib
import importlib.util
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import textwrap
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
TASK_DIR = ROOT / "aerial_gym/task/navrl_task"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


E = _load("cams3_test_realveh_entry", ROOT / "tools/realveh_entry.py")
C3 = _load("cams3_test_module", TASK_DIR / "navrl_cams3.py")
DRV = _load("cams3_test_driver", ROOT / "tools/cams3_train.py")

CHILD = textwrap.dedent(r'''
    import importlib, importlib.util, json, math, os, pathlib, sys, tempfile, types
    ROOT = pathlib.Path(sys.argv[1])
    for init in sorted((ROOT / "aerial_gym").rglob("__init__.py")):
        rel = init.parent.relative_to(ROOT)
        name = ".".join(rel.parts)
        stub = types.ModuleType(name)
        stub.__path__ = [str(init.parent)]
        if name == "aerial_gym":
            stub.AERIAL_GYM_DIRECTORY = str(ROOT)
        sys.modules[name] = stub
    import numpy as np
    import torch
    torch.set_num_threads(1)

    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, str(path))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    os.environ.update({"NAVRL_LIDAR_HBEAMS": "72", "NAVRL_LIDAR_VBEAMS": "4"})
    TASK = ROOT / "aerial_gym/task/navrl_task"
    DM = importlib.import_module("aerial_gym.task.navrl_task.navrl_depth_model")
    C3 = importlib.import_module("aerial_gym.task.navrl_task.navrl_cams3")
    ABL = importlib.import_module("aerial_gym.task.navrl_task.navrl_sensor_ablation")
    N, H, W, HB, VB, LMAX = 4, 40, 312, 72, 4, 12.0
    counter = [0]
    out = {}

    def perception_module():
        counter[0] += 1
        return load("aerial_gym.task.navrl_task.navrl_perception_c3_%d" % counter[0], TASK / "navrl_perception.py")

    def make(mod, n=N):
        camera = types.SimpleNamespace(detector_max_range=20.0, detector_hfov_deg=66.7, detector_vfov_deg=52.6,
                                       camera_width=W, camera_height=H, camera_translation=[0.08, 0.0, 0.03],
                                       camera_target_radius=0.15, tracker_memory_s=5.0,
                                       camera_obstacle_max_range=10.0)
        cfg = types.SimpleNamespace(lidar_max_range=LMAX, min_target_pixels=2, pixel_threshold=0.55,
                                    detection_dropout_prob=0.0, detection_latency_s=0.0, range_error_m=0.0,
                                    latency_compensate=False, latency_lidar_backup=False,
                                    latency_obstacle_fix=False, latency_ego_motion_fix=False,
                                    pose_noise_seed=9163, rgb_noise_std=0.0, depth_noise_std=0.0,
                                    history_interval_s=0.5, detector_checkpoint="")
        return mod.NavRLPerceptionModule(n, "cpu", cfg, 0.1, camera)

    def dms(**kw):
        s = {"NAVRL_DEPTH_MINZ_M": 0.0, "NAVRL_DEPTH_MAXZ_M": 20.0, "NAVRL_DEPTH_SIGMA_D_PX": 0.0,
             "NAVRL_DEPTH_FB_PXM": 29.88, "NAVRL_DEPTH_HOLE_P": 0.0, "NAVRL_DEPTH_SEED": 3}
        s.update({"NAVRL_DEPTH_" + k: v for k, v in kw.items()})
        return s

    def red(rgb, rows, cols):
        rgb[:, 0, rows[0]:rows[1], cols[0]:cols[1]] = 0.88
        rgb[:, 1, rows[0]:rows[1], cols[0]:cols[1]] = 0.08
        rgb[:, 2, rows[0]:rows[1], cols[0]:cols[1]] = 0.045

    def observe(p, rgb, depth):
        kw = dict(rgb=rgb, depth=depth.clone(), lidar_m=torch.full((N, VB * HB), LMAX),
                  drone_pos_w=torch.tensor([[0.0, 0.0, 1.5]]).repeat(N, 1),
                  drone_vel_w=torch.tensor([[0.4, 0.0, 0.0]]).repeat(N, 1),
                  vehicle_quat=torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(N, 1), yaw_rate=torch.zeros(N),
                  previous_action=torch.zeros(N, 4), max_velocity=2.5, flight_altitude=1.5, training=False)
        with torch.random.fork_rng():
            torch.manual_seed(1000)
            p.observe(**kw)
        return p.last_scan_nearest[0].clone()

    def bin_index(mod, deg):
        b = mod.lidar_bin_bearings("cpu")
        d = torch.atan2(torch.sin(b - math.radians(deg)), torch.cos(b - math.radians(deg))).abs()
        return int(d.argmin())

    def sampled_u(mod, p, deg):
        a = mod.lidar_bin_bearings("cpu")[bin_index(mod, deg)]
        return int(((p.hfov * 0.5 - a) / p.hfov * (W - 1)).round())

    mode = sys.argv[2]
    if mode == "sector":
        def chain(k):
            mod = perception_module()
            DM.install_perception(mod, dms())
            ABL.install_perception(mod, "no_lidar")
            if k is not None:
                C3.install_sector(mod, {"NAVRL_CAMS3_SECTOR_K": k})
            return mod, make(mod)
        results = {}
        for k in (None, 1, 24):
            mod, p = chain(k)
            u0, u10 = sampled_u(mod, p, 0.0), sampled_u(mod, p, 10.0)
            b0, b10, b20 = bin_index(mod, 0.0), bin_index(mod, 10.0), bin_index(mod, 20.0)
            r = {"u0": u0, "u10": u10}
            # thin bar at 4 m in 3 columns of the 0-degree bin, NOT the sampled column
            rgb = torch.full((N, 3, H, W), 0.15)
            depth = torch.full((N, H, W), 10.0)
            depth[:, :, u0 + 4:u0 + 7] = 4.0
            # one speckle pixel at 2 m in the 10-degree bin, not on the sampled column
            depth[:, 20, u10 + 3] = 2.0
            r["thin"] = observe(p, rgb, depth).tolist()
            # a wide bar covering the sampled column of the 20-degree bin (seen by every rule)
            depth2 = depth.clone()
            u20 = sampled_u(mod, p, 20.0)
            depth2[:, :, u20 - 3:u20 + 4] = 5.0
            r["wide"] = observe(p, rgb, depth2).tolist()
            # target patch (red) on the sampled column of the 0-degree bin at 3 m, and a grey control patch
            rgb_t = torch.full((N, 3, H, W), 0.15)
            depth_t = torch.full((N, H, W), 10.0)
            depth_t[:, 15:25, u0 - 3:u0 + 4] = 3.0
            red(rgb_t, (15, 25), (u0 - 3, u0 + 4))
            r["target"] = observe(p, rgb_t, depth_t).tolist()
            r["grey"] = observe(p, torch.full((N, 3, H, W), 0.15), depth_t).tolist()
            r["bins"] = [b0, b10, b20]
            if k is not None:
                r["cols"] = [int(x) for x in p._cams3_sector_cols[2].sum(dim=1).tolist()]
                r["sampled"] = [int(x) for x in p._cams3_sector_cols[0].tolist()]
                # direct call from the test is refused (only the perception's fusion may call it)
                try:
                    mod.camera_no_return_to_lidar_range(torch.zeros(N, W), 10.0, 12.0)
                    r["direct_refused"] = False
                except RuntimeError:
                    r["direct_refused"] = True
                # a caller of the right name whose column minima are not depth_no_target.amin(dim=1) is refused
                def _fuse_static_and_extract_obstacles(self, depth_no_target, col):
                    return mod.camera_no_return_to_lidar_range(col, 10.0, 12.0)
                d = torch.full((N, H, W), 10.0)
                try:
                    _fuse_static_and_extract_obstacles(p, d, d.amin(dim=1) - 1.0)
                    r["wrong_minima_refused"] = False
                except RuntimeError:
                    r["wrong_minima_refused"] = True
                ok = _fuse_static_and_extract_obstacles(p, d, d.amin(dim=1))
                r["right_minima_accepted"] = bool(torch.equal(ok, torch.full((N, W), 12.0)))
                try:
                    C3.install_sector(mod, {"NAVRL_CAMS3_SECTOR_K": k})
                    r["double_install_refused"] = False
                except RuntimeError:
                    r["double_install_refused"] = True
            results[str(k)] = r
        out = results
    elif mode == "measured":
        tmp = pathlib.Path(tempfile.mkdtemp())
        masks = np.zeros((5, H, W), dtype=bool)
        masks[1, :, 100:110] = True                  # frame 1: a vertical invalid stripe
        masks[2, 30:, :] = True
        lib = tmp / "lib.npz"
        np.savez(lib, masks=masks, scene_start=np.array([0, 3, 5]), scenes=np.array(["a", "b"]),
                 source_sha256=np.array(["x", "y"]))
        def settings(**kw):
            s = {"NAVRL_DEPTH_MEAS_LIBRARY": str(lib), "NAVRL_DEPTH_MEAS_SHA256": "0" * 64, "NAVRL_DEPTH_MEAS_SEED": 5,
                 "NAVRL_DEPTH_MEAS_TOP_ROWS": 10, "NAVRL_DEPTH_MEAS_ADVANCE_P": 0.5, "NAVRL_DEPTH_MEAS_JUMP_P": 0.0,
                 "NAVRL_DEPTH_MEAS_FALSE_NEAR": 0}
            s.update({"NAVRL_DEPTH_MEAS_" + a: b for a, b in kw.items()})
            return s
        def direct(depth, rgb, s, dm=None, ptr=None, n=N):
            mod = perception_module()
            p = make(mod, n)
            dm = dm or dms(MINZ_M=0.315)
            type(p)._depth_model_settings = dm
            modelled = DM.apply(p, depth, dm)
            inv_model = p._depth_model_invalid.clone()
            if ptr is not None:
                p._cams3_meas = C3.MeasuredDepth(p, s, depth.device)
                p._cams3_meas.ptr = torch.tensor(ptr)
                p._cams3_meas.s = dict(s, NAVRL_DEPTH_MEAS_ADVANCE_P=0.0, NAVRL_DEPTH_MEAS_JUMP_P=0.0)
            res = C3.apply_measured(p, rgb, modelled, s)
            return p, modelled, inv_model, res
        rgb = torch.full((N, 3, H, W), 0.15)
        depth = torch.full((N, H, W), 6.0)
        depth[:, :, :20] = 10.0                                  # renderer no-hit
        p, modelled, inv_model, res = direct(depth, rgb, settings(), ptr=[0, 1, 2, 4])
        hit = (depth != 10.0)
        top = torch.zeros((H, W), dtype=torch.bool)
        top[:10] = True
        out["top_far"] = bool((res[:, :10][hit[:, :10]] == 20.0).all())
        out["top_nohit_untouched"] = bool((res[:, :10, :20] == 10.0).all())
        out["frame0_rest_equal"] = bool(torch.equal(res[0, 10:], modelled[0, 10:]))
        out["frame1_stripe_far"] = bool((res[1, 10:, 100:110] == 20.0).all())
        out["frame1_rest_equal"] = bool(torch.equal(res[1, 10:, 110:], modelled[1, 10:, 110:]))
        out["frame2_bottom_far"] = bool((res[2, 30:, 20:] == 20.0).all())
        out["invalid_extended"] = bool(p._depth_model_invalid[1, 10:, 100:110].all() and
                                       not inv_model[1, 10:, 100:110].any())
        out["input_unmodified"] = bool((modelled[1, 10:, 100:110] == 6.0).all())
        # false near: off -> no change outside masks; on -> measured shares at 1 m and 3 m; target pixels never
        fn = {}
        for z_m in (1.0, 3.0):
            n = 64
            d = torch.full((n, H, W), z_m)
            rgb_n = torch.full((n, 3, H, W), 0.15)
            red(rgb_n, (20, 30), (200, 220))
            for flag in (0, 1):
                p, modelled, _, res = direct(d, rgb_n, settings(FALSE_NEAR=flag, TOP_ROWS=0), ptr=[0] * n, n=n)
                cos = p._depth_model_cos
                el = torch.ones_like(d, dtype=torch.bool)
                el[:, 20:30, 200:220] = False
                ratio = (res * cos) / (modelled * cos)
                gone = res == 20.0                               # shortened below MinZ -> invalid
                e40, e20 = C3.false_near_probabilities(modelled * cos)
                fn["%g_%d" % (z_m, flag)] = {
                    "short20": float(((ratio < 0.8 - 1e-6) | gone)[el].float().mean()),
                    "short40": float(((ratio < 0.6 - 1e-6) | gone)[el].float().mean()),
                    "target_changed": int((res[~el] != modelled[~el]).sum()),
                    "below_minz_invalid": int(((res == 20.0) & el).sum()),
                    "expected20": float(e20[el].mean()), "expected40": float(e40[el].mean())}
        out["false_near"] = fn
        # pointer: advance rate, scene boundary (frame 2 is the last of scene a; frame 4 the last of scene b)
        mod = perception_module()
        p = make(mod, 2000)
        st = C3.MeasuredDepth(p, settings(), "cpu")
        ends = st.scene_end
        rate, crossed, boundary_jumps, steps = 0.0, 0, 0, 50
        for _ in range(steps):
            old = st.ptr.clone()
            step, jump = st.advance()
            rate += float(step.float().mean()) / steps
            at_end = (old + step >= ends[old])
            crossed += int(((~jump) & at_end).sum())                 # an advance past a scene end without a jump
            crossed += int(((~jump) & (st.ptr != old + step)).sum())  # a move other than the drawn advance
            boundary_jumps += int((jump & at_end).sum())
        out["boundary_jumps"] = boundary_jumps
        out["advance_rate"] = rate
        out["crossed_scene"] = crossed
        out["in_range"] = bool(((st.ptr >= 0) & (st.ptr < 5)).all())
        # install order: after the depth model refused, before accepted; the chain runs and extends the invalid mask
        mod = perception_module()
        DM.install_perception(mod, dms())
        try:
            C3.install_measured(mod, settings())
            out["after_dm_refused"] = False
        except RuntimeError:
            out["after_dm_refused"] = True
        mod = perception_module()
        C3.install_measured(mod, settings())
        DM.install_perception(mod, dms(MINZ_M=0.315))
        ABL.install_perception(mod, "no_lidar")
        p = make(mod)
        observe(p, rgb, depth)
        out["chain_top_invalid"] = bool(p._depth_model_invalid[:, :10, 20:].all())
        out["chain_meas_calls"] = int(p._cams3_meas.stats["calls"])
        # the shipped library matches the camera grid when present
        out["false_near_probabilities"] = [[float(x) for x in t] for t in
                                           C3.false_near_probabilities(torch.tensor([0.5, 1.0, 1.5, 2.0, 3.0, 8.0]))]
    print("CHILD" + json.dumps(out))
''')


def child(mode, timeout=900):
    env = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")}
    env.update(PYTHONNOUSERSITE="1", CUDA_VISIBLE_DEVICES="")
    done = subprocess.run([sys.executable, "-B", "-c", CHILD, str(ROOT), mode], env=env, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, universal_newlines=True, timeout=timeout)
    if done.returncode != 0:
        raise AssertionError(done.stderr[-3000:])
    lines = [ln for ln in done.stdout.splitlines() if ln.startswith("CHILD")]
    if not lines:
        raise AssertionError(done.stdout[-2000:] + done.stderr[-2000:])
    return json.loads(lines[-1][len("CHILD"):])


_CACHE = {}


def cached(mode):
    if mode not in _CACHE:
        _CACHE[mode] = child(mode)
    return _CACHE[mode]


class SectorRule(unittest.TestCase):
    def setUp(self):
        self.r = cached("sector")

    def test_thin_bar_between_sampled_columns(self):
        b0 = self.r["None"]["bins"][0]
        self.assertEqual(self.r["None"]["thin"][b0], 12.0)            # native: the sampled column misses it
        self.assertAlmostEqual(self.r["1"]["thin"][b0], 4.0, places=4)
        self.assertAlmostEqual(self.r["24"]["thin"][b0], 4.0, places=4)

    def test_speckle_needs_k_pixels(self):
        b10 = self.r["None"]["bins"][1]
        self.assertAlmostEqual(self.r["1"]["thin"][b10], 2.0, places=4)   # positive control: k=1 reads the speckle
        self.assertEqual(self.r["24"]["thin"][b10], 12.0)
        self.assertEqual(self.r["None"]["thin"][b10], 12.0)

    def test_wide_bar_seen_by_every_rule_and_other_bins_unchanged(self):
        b20 = self.r["None"]["bins"][2]
        for k in ("None", "1", "24"):
            self.assertAlmostEqual(self.r[k]["wide"][b20], 5.0, places=4)
        b0, b10 = self.r["None"]["bins"][:2]
        far = [i for i, v in enumerate(self.r["None"]["wide"]) if i not in (b0, b10, b20)]
        self.assertGreater(len(far), 60)
        for k in ("1", "24"):
            self.assertEqual([self.r[k]["wide"][i] for i in far], [self.r["None"]["wide"][i] for i in far])

    def test_target_is_carved_as_natively(self):
        b0 = self.r["None"]["bins"][0]
        for k in ("None", "1", "24"):
            self.assertEqual(self.r[k]["target"][b0], 12.0)
            self.assertAlmostEqual(self.r[k]["grey"][b0], 3.0, places=4)     # control: the same patch, not red

    def test_columns_and_guards(self):
        for k in ("1", "24"):
            r = self.r[k]
            self.assertEqual(len(r["cols"]), 13)
            self.assertTrue(all(18 <= c <= 30 for c in r["cols"]), r["cols"])
            self.assertEqual(r["cols"], r["cols"][::-1])                 # symmetric about the optical axis
            self.assertIn(r["u0"], r["sampled"])
            self.assertTrue(r["direct_refused"])
            self.assertTrue(r["wrong_minima_refused"])
            self.assertTrue(r["right_minima_accepted"])
            self.assertTrue(r["double_install_refused"])


class MeasuredDepth(unittest.TestCase):
    def setUp(self):
        self.r = cached("measured")

    def test_masks_and_top_band(self):
        r = self.r
        for key in ("top_far", "top_nohit_untouched", "frame0_rest_equal", "frame1_stripe_far", "frame1_rest_equal",
                    "frame2_bottom_far", "invalid_extended", "input_unmodified"):
            self.assertTrue(r[key], key)

    def test_false_near_shares(self):
        fn = self.r["false_near"]
        for z, (p20, p40) in (("1", (0.51, 0.29)), ("3", (0.14, 0.011))):
            off, on = fn[z + "_0"], fn[z + "_1"]
            # range z_m off axis has z < z_m: the expected share is the model's probability at each pixel's z
            self.assertAlmostEqual(on["expected20"], p20, delta=0.02)
            self.assertAlmostEqual(on["expected40"], p40, delta=0.01)
            p20, p40 = on["expected20"], on["expected40"]
            self.assertEqual(off["short20"], 0.0)
            self.assertEqual(off["short40"], 0.0)
            self.assertAlmostEqual(on["short20"], p20, delta=0.01)
            self.assertAlmostEqual(on["short40"], p40, delta=0.006)
            self.assertEqual(on["target_changed"], 0)
        self.assertEqual(fn["1_1"]["below_minz_invalid"], fn["1_1"]["below_minz_invalid"])
        self.assertGreater(fn["1_1"]["short40"], 0.2)

    def test_probabilities(self):
        p40, p20 = self.r["false_near_probabilities"]
        self.assertEqual([round(x, 4) for x in p40], [0.29, 0.29, 0.157, 0.024, 0.011, 0.011])
        self.assertEqual([round(x, 4) for x in p20], [0.51, 0.51, 0.345, 0.18, 0.14, 0.14])

    def test_pointer(self):
        self.assertAlmostEqual(self.r["advance_rate"], 0.5, delta=0.02)
        self.assertEqual(self.r["crossed_scene"], 0)
        self.assertGreater(self.r["boundary_jumps"], 1000)          # positive control: scene ends do force jumps
        self.assertTrue(self.r["in_range"])

    def test_install_order_and_chain(self):
        self.assertTrue(self.r["after_dm_refused"])
        self.assertTrue(self.r["chain_top_invalid"])
        self.assertEqual(self.r["chain_meas_calls"], 1)


DM_ENV = {"NAVRL_DEPTH_MODEL": "oakd", "NAVRL_DEPTH_MINZ_M": "0.315", "NAVRL_DEPTH_MAXZ_M": "20",
          "NAVRL_DEPTH_SIGMA_D_PX": "0.125", "NAVRL_DEPTH_FB_PXM": "29.88", "NAVRL_DEPTH_HOLE_P": "0",
          "NAVRL_DEPTH_SEED": "4942"}
ENTRY_OK = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/realveh_receipt.json",
            "NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}


class Settings(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.lib = pathlib.Path(cls.tmp.name) / "lib.npz"
        cls.lib.write_bytes(b"library bytes")
        cls.sha = hashlib.sha256(b"library bytes").hexdigest()
        cls.meas = {"NAVRL_DEPTH_MEAS": "1", "NAVRL_DEPTH_MEAS_LIBRARY": str(cls.lib), "NAVRL_DEPTH_MEAS_SHA256": cls.sha,
                    "NAVRL_DEPTH_MEAS_SEED": "4943", "NAVRL_DEPTH_MEAS_TOP_ROWS": "10",
                    "NAVRL_DEPTH_MEAS_ADVANCE_P": "0.5", "NAVRL_DEPTH_MEAS_JUMP_P": "0.01",
                    "NAVRL_DEPTH_MEAS_FALSE_NEAR": "1"}

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_meas(self):
        s = C3.meas_settings_from_env(self.meas)
        self.assertEqual(s["NAVRL_DEPTH_MEAS_TOP_ROWS"], 10)
        self.assertIsNone(C3.meas_settings_from_env({}))
        for bad in ({"NAVRL_DEPTH_MEAS_SHA256": "1" * 64}, {"NAVRL_DEPTH_MEAS_JUMP_P": "1.5"},
                    {"NAVRL_DEPTH_MEAS_FALSE_NEAR": "2"}, {"NAVRL_DEPTH_MEAS": "yes"},
                    {"NAVRL_DEPTH_MEAS_LIBRARY": str(self.lib) + ".missing"}):
            with self.assertRaises(ValueError, msg=bad):
                C3.meas_settings_from_env(dict(self.meas, **bad))
        with self.assertRaises(ValueError):                      # stray key while off
            C3.meas_settings_from_env({"NAVRL_DEPTH_MEAS_SEED": "1"})

    def test_sector(self):
        self.assertEqual(C3.sector_settings_from_env({"NAVRL_CAMS3_SECTOR_K": "24"}), {"NAVRL_CAMS3_SECTOR_K": 24})
        self.assertIsNone(C3.sector_settings_from_env({}))
        for bad in ("0", "x", "1001"):
            with self.assertRaises(ValueError):
                C3.sector_settings_from_env({"NAVRL_CAMS3_SECTOR_K": bad})

    def test_realveh_entry(self):
        base = dict(ENTRY_OK, **DM_ENV)
        s = E.settings(dict(base, NAVRL_CAMS3_SECTOR_K="24", **self.meas))
        self.assertEqual(s["cams3_sector"], {"NAVRL_CAMS3_SECTOR_K": 24})
        self.assertEqual(s["cams3_meas"]["NAVRL_DEPTH_MEAS_SHA256"], self.sha)
        off = E.settings(base)                                   # positive control: off leaves no cams3 keys
        self.assertNotIn("cams3_meas", off)
        self.assertNotIn("cams3_sector", off)
        with self.assertRaises(SystemExit):                      # measured depth needs the depth model
            E.settings(dict(ENTRY_OK, **self.meas))
        with self.assertRaises(SystemExit):
            E.settings(dict(base, NAVRL_DEPTH_VALIDITY="blob_near", NAVRL_DEPTH_VALIDITY_WINDOW="5",
                            NAVRL_DEPTH_VALIDITY_DENSITY="0.5", **self.meas))
        self.assertIsNotNone(E.settings(dict(base, NAVRL_CAMS3_SECTOR_K="1")).get("cams3_sector"))


class Driver(unittest.TestCase):
    def test_expected_env_differences(self):
        ref = {"NAVRL_YAW_RATE_MAX": "1.0472", "NAVRL_BAR_POOL": "bars_h3", "NAVRL_DEPTH_HOLE_P": "0.02",
               "NAVRL_OTHER": "x"}
        env = DRV.expected_env(ref, "w", ROOT, "/r/receipt.json")
        self.assertEqual(env["NAVRL_YAW_RATE_MAX"], "1.5708")
        self.assertEqual(env["NAVRL_BAR_POOL"], "bars_h3_thin")
        self.assertEqual(env["NAVRL_DEPTH_HOLE_P"], "0")
        self.assertEqual(env["NAVRL_CAMS3_SECTOR_K"], "24")
        self.assertEqual(env["NAVRL_OTHER"], "x")
        receipt = {"env_navrl": env, "cams2": {}, "camsafe": {}, "cams3_meas": {}, "cams3_sector": {}}
        self.assertEqual(DRV.receipt_problems(receipt, ref, "w", ROOT, "/r/receipt.json"), [])
        bad = dict(receipt, env_navrl=dict(env, NAVRL_OTHER="y"))
        self.assertTrue(DRV.receipt_problems(bad, ref, "w", ROOT, "/r/receipt.json"))
        self.assertTrue(DRV.receipt_problems(dict(receipt, yawclip={}), ref, "w", ROOT, "/r/receipt.json"))
        missing = {k: v for k, v in receipt.items() if k != "cams3_sector"}
        self.assertTrue(DRV.receipt_problems(missing, ref, "w", ROOT, "/r/receipt.json"))

    def test_start_checks(self):
        lines = [
            "[cams3-python] NAVRL_YAW_RATE_MAX 3.0 -> 1.5708 rad/s (action scale and Lee yaw clamp); NAVRL_BAR_POOL "
            "bars_h3 -> bars_h3_thin",
            "NavRL CONFIG MISMATCH | NAVRL_YAW_RATE_MAX: checkpoint trained with 1.047, running with 1.571. x",
            "NavRL ARENA MISMATCH | NAVRL_BAR_POOL: checkpoint=bars_h3 running=bars_h3_thin. x",
            "NavRL density evidence RESET | discarded=3/16 eps resume_warmup=1000 epochs gate_not_before_step=512000",
            "NavRL checkpoint state restored | task_steps=480000 bars=205->205 k_min=20.0 k_max=28.0 density_window=0/16",
            '[cams3] {"event": "install_measured", "settings": {}}',
            '[cams3] {"event": "install_sector", "settings": {}}',
            '[cams3] {"calls": 1, "event": "meas_first"}',
            '[cams3] {"bins": 13, "event": "sector_first", "k": 24}']
        block = (15001, 18000, 1, 2, 1, 0, 205)
        self.assertEqual(DRV.start_problems("\n".join(lines), [block], "w", 18000), [])
        for i in range(len(lines)):
            text = "\n".join(lines[:i] + lines[i + 1:])
            self.assertTrue(DRV.start_problems(text, [block], "w", 18000), i)
        self.assertTrue(DRV.start_problems("\n".join(lines).replace('"k": 24', '"k": 1'), [block], "w", 18000))
        self.assertTrue(DRV.start_problems("\n".join(lines), [(15002,) + block[1:]], "w", 18000))

    def test_constants(self):
        self.assertEqual(DRV.BRANCH, "camonly/s2b")
        self.assertEqual(DRV.RUNS["w"]["max_epochs"], 18000)
        self.assertEqual(DRV.RUNS["w"]["stop_check_epoch"], 16000)
        launcher = (ROOT / "aerial_gym/rl_training/rl_games/train_navrl_cams3.sh").read_text()
        self.assertIn('SOURCE_SHA="%s"' % DRV.SOURCE_SHA, launcher)
        self.assertIn('MASK_SHA="%s"' % DRV.MASK_SHA, launcher)
        for key, value in (("NAVRL_DEPTH_MEAS_TOP_ROWS", "10"), ("NAVRL_DEPTH_MEAS_ADVANCE_P", "0.5"),
                           ("NAVRL_DEPTH_MEAS_JUMP_P", "0.01"),
                           ("NAVRL_DEPTH_HOLE_P", "0")):
            self.assertIn("export %s=%s\n" % (key, value), launcher)
            self.assertEqual(DRV.expected_env({}, "w", ROOT, "r")[key], value)
        # review r1 P1-1: false near OFF in S3-W (the launcher line carries a comment)
        self.assertIn("export NAVRL_DEPTH_MEAS_FALSE_NEAR=0 ", launcher)
        self.assertEqual(DRV.expected_env({}, "w", ROOT, "r")["NAVRL_DEPTH_MEAS_FALSE_NEAR"], "0")
        self.assertIn('NAVRL_DEPTH_MEAS_SEED="${CAMS3_MEAS_SEED:-4943}"', launcher)


class Acceptance(unittest.TestCase):
    def test_acceptance_record(self):
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / "a.json"
            self.assertFalse(DRV.acceptance_ok(path))                       # missing
            path.write_text(json.dumps({"accepted": "S3-W without false near", "user_words": "ok"}))
            self.assertTrue(DRV.acceptance_ok(path))                        # positive control
            for bad in ({"accepted": "S3-W", "user_words": "ok"}, {"accepted": "S3-W without false near"},
                        {"accepted": "S3-W without false near", "user_words": " "}, ["x"]):
                path.write_text(json.dumps(bad))
                self.assertFalse(DRV.acceptance_ok(path), bad)
            path.write_text("{not json")
            self.assertFalse(DRV.acceptance_ok(path))
        self.assertEqual(DRV.ACCEPTANCE.name, "user_acceptance_false_near_off.json")


class Speed(unittest.TestCase):
    def test_projection_on_the_s2b_ft_smoke_log_format(self):
        text = "".join("  epoch          : %d / 15025\n  step time (s)  : %.4f\n" % (15001 + i, t)
                       for i, t in enumerate([9.0, 6.0, 5.0, 5.0, 5.0] + [4.0] * 20))
        p = DRV.speed_projection(text)
        self.assertEqual(p["median_step_s"], 4.0)
        self.assertEqual(p["epochs_used"], 20)
        self.assertAlmostEqual(p["projected_h"], 4.0 * 3000 / 3600 * 1.108, places=3)
        self.assertTrue(p["within_cap"])
        slow = text.replace("step time (s)  : 4.0000", "step time (s)  : 4.8000")
        self.assertFalse(DRV.speed_projection(slow)["within_cap"])     # 4.8 s -> 4.43 h > 4.3 h
        self.assertIsNone(DRV.speed_projection(text[:200]))


class Assets(unittest.TestCase):
    def test_thin_pool(self):
        base = ROOT / "resources/models/environment_assets/bars_h3"
        thin = ROOT / "resources/models/environment_assets/bars_h3_thin"
        names = sorted(p.name for p in thin.iterdir())
        self.assertEqual(len(names), 54)
        for p in sorted(base.iterdir()):
            self.assertEqual((thin / p.name).read_bytes(), p.read_bytes())
        sizes = []
        for name in names:
            if name.startswith("thin_bar_"):
                boxes = re.findall(r'<box size="([\d.]+) ([\d.]+) ([\d.]+)"/>', (thin / name).read_text())
                self.assertEqual(len(boxes), 2)
                self.assertEqual(boxes[0], boxes[1])
                sizes.append(tuple(float(x) for x in boxes[0]))
        self.assertEqual(len(sizes), 14)
        self.assertTrue(all(0.2 <= x <= 0.3 and 0.2 <= y <= 0.3 and z == 3.0 for x, y, z in sizes))
        src = (ROOT / "aerial_gym/config/asset_config/env_object_config.py").read_text()
        self.assertIn('_bar_z_ratio = 0.5 if _bar_pool in ("bars_h3", "bars_h3_thin") else 0.3333', src)

    def test_library_builder(self):
        B = _load("cams3_test_builder", ROOT / "tools/cams3_build_mask_library.py")
        import numpy as np
        m = np.zeros((2, 468, 624), dtype=bool)
        m[0, 0:2, 0:2] = True                                    # full 2x2 block -> invalid
        m[0, 2:4, 2:4] = [[True, True], [True, False]]           # 3 of 4 -> valid (min-valid)
        small = B.downsample_all_invalid(m)
        self.assertEqual(small.shape, (2, 234, 312))
        self.assertTrue(small[0, 0, 0])
        self.assertFalse(small[0, 1, 1])
        self.assertEqual(int(small.sum()), 1)


if __name__ == "__main__":
    unittest.main()
