"""CPU tests for the opt-in invalid-depth validity rule (aerial_gym/task/navrl_task/navrl_depth_validity.py, CAM-K2 v3)
and its install through tools/realveh_entry.py.

No GPU, Isaac Gym or simulator. Safe inside the single-process PUBLIC_CPU suite: every module that needs torch or the
`aerial_gym` package runs in a child process with path-only package stubs (the pattern of tests/test_realveh.py);
in this process the modules are loaded by file path under private names and import only the standard library. The
environment is changed only through unittest.mock.patch. Every negative check is paired with a positive control.
Run: PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B tests/test_depth_validity.py
"""
import importlib.util
import json
import os
import pathlib
import re
import subprocess
import sys
import textwrap
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
TASK_DIR = ROOT / "aerial_gym/task/navrl_task"


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


E = _load("depth_validity_test_realveh_entry", ROOT / "tools/realveh_entry.py")
DV = _load("depth_validity_under_test", TASK_DIR / "navrl_depth_validity.py")

CHILD = textwrap.dedent(r'''
    import hashlib, importlib, importlib.util, inspect, io, json, math, os, pathlib, sys, types
    ROOT = pathlib.Path(sys.argv[1])
    for init in sorted((ROOT / "aerial_gym").rglob("__init__.py")):
        rel = init.parent.relative_to(ROOT)
        name = ".".join(rel.parts)
        stub = types.ModuleType(name)
        stub.__path__ = [str(init.parent)]
        if name == "aerial_gym":
            stub.AERIAL_GYM_DIRECTORY = str(ROOT)
        sys.modules[name] = stub
    import torch
    torch.set_num_threads(1)

    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, str(path))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    def rng_hash():
        return hashlib.sha256(torch.get_rng_state().numpy().tobytes()).hexdigest()

    os.environ.update({"NAVRL_LIDAR_HBEAMS": "72", "NAVRL_LIDAR_VBEAMS": "4"})
    TASK = ROOT / "aerial_gym/task/navrl_task"
    DM = importlib.import_module("aerial_gym.task.navrl_task.navrl_depth_model")
    DV = importlib.import_module("aerial_gym.task.navrl_task.navrl_depth_validity")
    ABL = importlib.import_module("aerial_gym.task.navrl_task.navrl_sensor_ablation")
    N, H, W, HB, VB, LMAX = 4, 45, 80, 72, 4, 12.0
    counter = [0]
    out = {}

    def perception_module():
        counter[0] += 1
        return load("aerial_gym.task.navrl_task.navrl_perception_dv%d" % counter[0], TASK / "navrl_perception.py")

    def make(mod):
        camera = types.SimpleNamespace(detector_max_range=20.0, detector_hfov_deg=87.0, detector_vfov_deg=58.0,
                                       camera_width=W, camera_height=H, camera_translation=[0.1, 0.0, 0.03],
                                       camera_target_radius=0.15, tracker_memory_s=5.0,
                                       camera_obstacle_max_range=10.0)
        cfg = types.SimpleNamespace(lidar_max_range=LMAX, min_target_pixels=2, pixel_threshold=0.55,
                                    detection_dropout_prob=0.0, detection_latency_s=0.0, range_error_m=0.0,
                                    latency_compensate=False, latency_lidar_backup=False,
                                    latency_obstacle_fix=False, latency_ego_motion_fix=False,
                                    pose_noise_seed=9163, rgb_noise_std=0.0, depth_noise_std=0.0,
                                    history_interval_s=0.5, detector_checkpoint="")
        return mod.NavRLPerceptionModule(N, "cpu", cfg, 0.1, camera)

    def dms(**kw):
        s = {"NAVRL_DEPTH_MINZ_M": 0.4, "NAVRL_DEPTH_MAXZ_M": 20.0, "NAVRL_DEPTH_SIGMA_D_PX": 0.0,
             "NAVRL_DEPTH_FB_PXM": 66.1875, "NAVRL_DEPTH_HOLE_P": 0.0, "NAVRL_DEPTH_SEED": 3}
        s.update({"NAVRL_DEPTH_" + k: v for k, v in kw.items()})
        return s

    VS = {"NAVRL_DEPTH_VALIDITY": "blob_near", "NAVRL_DEPTH_VALIDITY_WINDOW": 5, "NAVRL_DEPTH_VALIDITY_DENSITY": 0.5}

    def chain(dm_s, vs=VS, ablation=True):
        """The production order of tools/realveh_entry.py: validity, then the depth model, then the ablation."""
        mod = perception_module()
        if vs is not None:
            DV.install_perception(mod, vs)
        if dm_s is not None:
            DM.install_perception(mod, dm_s)
        if ablation:
            ABL.install_perception(mod, "no_lidar")
        return mod

    def frame(bar_m=None, cols=(38, 43), target=False, bg=6.0):
        depth = torch.full((N, H, W), bg)
        depth[:, :, :10] = 10.0                                  # the renderer's own no-hit pixels
        rgb = torch.full((N, 3, H, W), 0.15)
        if bar_m is not None:
            depth[:, :, cols[0]:cols[1]] = bar_m
        if target:
            rgb[:, 0, 20:24, 60:65] = 0.88
            rgb[:, 1, 20:24, 60:65] = 0.08
            rgb[:, 2, 20:24, 60:65] = 0.045
            depth[:, 20:24, 60:65] = 3.0
        return rgb, depth

    def observe(p, rgb, depth, k=0, fork=True):
        kw = dict(rgb=rgb, depth=depth.clone(), lidar_m=torch.full((N, VB * HB), LMAX),
                  drone_pos_w=torch.tensor([[0.0, 0.0, 1.0]]).repeat(N, 1),
                  drone_vel_w=torch.tensor([[0.4, 0.0, 0.0]]).repeat(N, 1),
                  vehicle_quat=torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(N, 1), yaw_rate=torch.zeros(N),
                  previous_action=torch.zeros(N, 4), max_velocity=2.5, flight_altitude=1.0, training=False)
        if fork:
            with torch.random.fork_rng():
                torch.manual_seed(1000 + k)
                obs, diag = p.observe(**kw)
        else:
            obs, diag = p.observe(**kw)
        return obs.clone(), (p._ablation_fused_m.clone() if getattr(p, "_ablation_fused_m", None) is not None else None)

    def direct(depth, dm_s, vs=VS):
        """depth model then validity on one perception object, as the nested wrappers call them."""
        mod = perception_module()
        p = make(mod)
        type(p)._depth_model_settings = dm_s
        modelled = DM.apply(p, depth, dm_s)
        ruled = DV.apply(p, modelled, vs)
        return p, modelled, ruled

    def near_zero_bins(mod):
        b = mod.lidar_bin_bearings("cpu")
        return (torch.atan2(torch.sin(b), torch.cos(b)).abs() < math.radians(2.0))

    mode = sys.argv[2]
    if mode == "units":
        # 1. a 0.3 m bar (< MinZ) is a dense invalid blob: becomes MinZ / cos; without the rule it is 20 m
        rgb, depth = frame(bar_m=0.3, target=True)
        p, modelled, ruled = direct(depth, dms())
        cos = p._depth_model_cos
        blob = torch.zeros_like(depth, dtype=torch.bool)
        blob[:, :, 38:43] = True
        out["blob_invalid_in_model"] = bool(p._depth_model_invalid[blob].all())
        out["blob_is_20_in_model"] = bool((modelled[blob] == 20.0).all())
        out["blob_near_pixels"] = int(p._depth_validity_near.sum())
        out["blob_pixels"] = int(blob.sum())
        out["blob_value_is_minz_over_cos"] = bool(torch.equal(ruled[blob], (0.4 / cos).expand_as(depth)[blob]))
        out["blob_optical_z"] = [float((ruled * cos)[blob].min()), float((ruled * cos)[blob].max())]
        out["rest_untouched"] = bool(torch.equal(ruled[~blob], modelled[~blob]))
        nohit = depth == 10.0
        out["nohit_pixels"] = int(nohit.sum())
        out["nohit_untouched"] = bool((ruled[nohit] == 10.0).all())
        tgt = torch.zeros_like(blob)
        tgt[:, 20:24, 60:65] = True
        out["target_untouched"] = bool((ruled[tgt] == 3.0).all())
        out["input_unmodified"] = bool((depth[blob] == 0.3).all())
        # 2. isolated random holes on a 6 m background: invalid but never near; output equals the model's
        rgb, depth = frame()
        p, modelled, ruled = direct(depth, dms(MINZ_M=0.4, HOLE_P=0.02, SEED=11))
        out["holes_invalid"] = int(p._depth_model_invalid.sum())
        out["holes_near"] = int(p._depth_validity_near.sum())
        out["holes_output_equal"] = bool(torch.equal(ruled, modelled))
        # control: the same hole share at 0.6 makes dense blobs, which the rule does turn near
        p, modelled, ruled = direct(depth, dms(MINZ_M=0.4, HOLE_P=0.5, SEED=11), dict(VS))
        out["dense_holes_near"] = int(p._depth_validity_near.sum())
        # 3. a frame of only no-hit 10 m and a frame with a valid 0.5 m bar: identical
        for name, (bar, bg) in (("all_nohit", (None, 10.0)), ("valid_bar", (0.5, 6.0))):
            rgb, depth = frame(bar_m=bar, bg=bg)
            p, modelled, ruled = direct(depth, dms())
            out[name + "_equal"] = bool(torch.equal(ruled, modelled))
            out[name + "_near"] = int(p._depth_validity_near.sum())
        # 4. density threshold edge (dense_mask on a constructed mask): k of 25 invalid around the centre pixel
        def centre_dense(k, density, window=5):
            m = torch.zeros((1, 11, 11), dtype=torch.bool)
            idx = [(5 + dy, 5 + dx) for dy in range(-2, 3) for dx in range(-2, 3) if (dy, dx) != (0, 0)]
            m[0, 5, 5] = True
            for (y, x) in idx[:k - 1]:
                m[0, y, x] = True
            return bool(DV.dense_mask(m, window, density)[0, 5, 5])
        out["edge"] = {"13_of_25_at_0.52": centre_dense(13, 0.52), "12_of_25_at_0.52": centre_dense(12, 0.52),
                       "13_of_25_at_0.5": centre_dense(13, 0.5), "12_of_25_at_0.5": centre_dense(12, 0.5),
                       "25_of_25_at_1.0": centre_dense(25, 1.0), "24_of_25_at_1.0": centre_dense(24, 1.0)}
        corner = torch.zeros((1, 6, 6), dtype=torch.bool)
        corner[0, :2, :2] = True                                 # 4 of the 9 in-image pixels of a corner window
        corner5 = corner.clone()
        corner5[0, 2, 0] = True                                  # 5 of 9
        out["corner_4_of_9_at_0.5"] = bool(DV.dense_mask(corner, 5, 0.5)[0, 0, 0])
        out["corner_5_of_9_at_0.5"] = bool(DV.dense_mask(corner5, 5, 0.5)[0, 0, 0])
        valid_centre = torch.ones((1, 5, 5), dtype=torch.bool)
        valid_centre[0, 2, 2] = False
        out["valid_pixel_never_near"] = not bool(DV.dense_mask(valid_centre, 5, 0.5)[0, 2, 2])
        # 5. refusals: rule without the depth model; a stale mask; MinZ 0
        mod = perception_module()
        p = make(mod)
        try:
            DV.apply(p, depth, VS)
            out["no_model"] = "accepted"
        except RuntimeError:
            out["no_model"] = "refused"
        p, modelled, ruled = direct(depth, dms())
        try:
            DV.apply(p, modelled, VS)
            out["stale_mask"] = "accepted"
        except RuntimeError:
            out["stale_mask"] = "refused"
        try:
            direct(depth, dms(MINZ_M=0.0))
            out["minz_zero"] = "accepted"
        except RuntimeError:
            out["minz_zero"] = "refused"

    elif mode == "external":
        # A measured invalid mask needs no perception/depth-model object and never infers invalid from a far value.
        depth = torch.full((2, 9, 11), 20.0)
        depth[:, 0, 10] = 2.75
        invalid = torch.zeros_like(depth, dtype=torch.bool)
        invalid[:, :, 3:8] = True
        invalid[:, 0, 0] = True                            # isolated invalid stays at the caller's far encoding
        cosine = torch.linspace(0.6, 1.0, 99).reshape(9, 11)
        original = [x.clone() for x in (depth, invalid, cosine)]
        before = rng_hash()
        ruled, near = DV.apply_mask(depth, invalid, cosine, min_z_m=0.4, window=5, density=0.5)
        out["near_count"] = int(near.sum())
        out["invalid_count"] = int(invalid.sum())
        out["valid_far_unchanged"] = bool((ruled[~invalid] == depth[~invalid]).all())
        out["near_value"] = bool(torch.equal(ruled[near], (0.4 / cosine).expand_as(depth)[near]))
        out["rest_unchanged"] = bool(torch.equal(ruled[~near], depth[~near]))
        out["isolated_not_near"] = not bool(near[:, 0, 0].any())
        out["inputs_unchanged"] = all(torch.equal(a, b) for a, b in zip((depth, invalid, cosine), original))
        out["new_depth"] = ruled.data_ptr() != depth.data_ptr()
        out["rng_unchanged"] = rng_hash() == before
        out["dtype_device"] = ruled.dtype == depth.dtype and ruled.device == depth.device and near.dtype == torch.bool
        clear, clear_near = DV.apply_mask(depth, torch.zeros_like(invalid), cosine,
                                         min_z_m=0.4, window=3, density=1.0)
        out["clear_equal"] = bool(torch.equal(clear, depth)) and not bool(clear_near.any())
        # Public measured-mask API agrees bit for bit with the established simulation wrapper.
        _, source = frame(bar_m=0.3, target=True)
        p, modelled, sim_ruled = direct(source, dms())
        measured, measured_near = DV.apply_mask(modelled, p._depth_model_invalid, p._depth_model_cos,
                                               min_z_m=0.4, window=5, density=0.5)
        out["simulation_equal"] = bool(torch.equal(measured, sim_ruled))
        out["simulation_mask_equal"] = bool(torch.equal(measured_near, p._depth_validity_near))

        valid = dict(depth=depth, invalid=invalid, cosine=cosine, min_z_m=0.4, window=5, density=0.5)
        bad = {
            "depth_not_tensor": {"depth": depth.tolist()},
            "depth_integer": {"depth": depth.to(torch.int64)},
            "depth_shape": {"depth": depth[0]},
            "depth_empty": {"depth": depth[:0], "invalid": invalid[:0]},
            "depth_nan": {"depth": torch.full_like(depth, float("nan"))},
            "depth_negative": {"depth": -depth},
            "invalid_not_bool": {"invalid": invalid.float()},
            "invalid_shape": {"invalid": invalid[:1]},
            "cosine_shape": {"cosine": cosine[None]},
            "cosine_integer": {"cosine": torch.ones_like(cosine, dtype=torch.int64)},
            "cosine_nan": {"cosine": torch.full_like(cosine, float("nan"))},
            "cosine_zero": {"cosine": torch.zeros_like(cosine)},
            "cosine_above_one": {"cosine": cosine + 1.0},
            "device_mismatch": {"invalid": torch.empty(invalid.shape, dtype=torch.bool, device="meta")},
            "minz_zero": {"min_z_m": 0.0},
            "minz_nan": {"min_z_m": float("nan")},
            "minz_bool": {"min_z_m": True},
            "window_even": {"window": 4},
            "window_small": {"window": 1},
            "window_large": {"window": 33},
            "window_float": {"window": 5.0},
            "window_bool": {"window": True},
            "density_zero": {"density": 0.0},
            "density_high": {"density": 1.1},
            "density_nan": {"density": float("nan")},
            "density_bool": {"density": True},
            "surface_overflow": {"depth": depth.half(), "cosine": torch.full_like(cosine, 1e-9)},
        }
        out["refused"] = {}
        for name, change in bad.items():
            try:
                DV.apply_mask(**dict(valid, **change))
                out["refused"][name] = False
            except ValueError:
                out["refused"][name] = True

    elif mode == "wrapped":
        rgb, depth = frame(bar_m=0.3)
        rgb0, depth0 = frame()
        rgb5, depth5 = frame(bar_m=0.5)
        # order: installed after the depth model or the ablation -> refused; production order accepted
        mod = perception_module()
        DM.install_perception(mod, dms())
        try:
            DV.install_perception(mod, VS)
            out["after_model"] = "accepted"
        except RuntimeError:
            out["after_model"] = "refused"
        mod = perception_module()
        ABL.install_perception(mod, "no_lidar")
        try:
            DV.install_perception(mod, VS)
            out["after_ablation"] = "accepted"
        except RuntimeError:
            out["after_ablation"] = "refused"
        try:
            DV.install_perception(chain(dms(), ablation=False), VS)
            out["twice"] = "accepted"
        except RuntimeError:
            out["twice"] = "refused"
        # telemetry bound at install although sys.stderr is swapped before the first frame (quiet startup)
        bound, quiet = io.StringIO(), io.StringIO()
        real_err = sys.stderr
        sys.stderr = bound
        on_mod = chain(dms(HOLE_P=0.02, SIGMA_D_PX=0.15))
        sys.stderr = quiet
        p_on = make(on_mod)
        for k in range(DV.STATS_CALLS):
            observe(p_on, rgb, depth, k)
        sys.stderr = real_err
        lines = [json.loads(x[len(DV.LOG_PREFIX):]) for x in bound.getvalue().splitlines()
                 if x.startswith(DV.LOG_PREFIX)]
        out["tele_events"] = [x["event"] for x in lines]
        out["tele_quiet_lines"] = sum(1 for x in quiet.getvalue().splitlines() if x.startswith(DV.LOG_PREFIX))
        out["tele_summary"] = [x for x in lines if x["event"] == "summary"]
        out["tele_install_settings"] = [x.get("settings") for x in lines if x["event"] == "install"]
        # signature
        orig = perception_module().NavRLPerceptionModule.observe
        wrapped = chain(None, ablation=False).NavRLPerceptionModule.observe
        out["signature_kept"] = str(inspect.signature(wrapped)) == str(inspect.signature(orig))
        out["has_wrapped"] = getattr(wrapped, "__wrapped__", None) is not None
        out["name_kept"] = wrapped.__name__ == "observe"
        out["unwrapped_signature_differs"] = (str(inspect.signature(wrapped, follow_wrapped=False))
                                             != str(inspect.signature(orig)))
        # fused scan straight ahead: 0.3 m bar -> 12 m no return with the model only; ~MinZ surface with the rule
        res = {}
        for name, vs, d in (("model_only_0p3", None, depth), ("rule_0p3", VS, depth), ("model_only_0p5", None, depth5),
                            ("rule_0p5", VS, depth5)):
            m = chain(dms(), vs=vs)
            obs, fused = observe(make(m), rgb, d)
            z = near_zero_bins(m)
            res[name] = [float(fused[:, :, z].min()), float(fused[:, :, z].max())]
        out["fused"] = res
        # rule inert when nothing is dense (holes on 6 m, valid 0.5 m bar): bitwise equal obs with / without the rule;
        # global RNG and the depth model's generator untouched; positive control: the 0.3 m bar frame differs
        def run(vs, frames):
            m = chain(dms(HOLE_P=0.02, SIGMA_D_PX=0.15, SEED=5), vs=vs)
            p = make(m)
            obs, rng = [], []
            for k, (r, d) in enumerate(frames):
                before = rng_hash()
                obs.append(observe(p, r, d, k, fork=False)[0])
                rng.append(rng_hash() == before)
            return obs, p._depth_model_generator.get_state(), rng
        torch.manual_seed(77)
        a_obs, a_gen, a_rng = run(None, [(rgb0, depth0), (rgb5, depth5)] * 3)
        torch.manual_seed(77)
        b_obs, b_gen, b_rng = run(VS, [(rgb0, depth0), (rgb5, depth5)] * 3)
        out["inert_equal"] = all(torch.equal(x, y) for x, y in zip(a_obs, b_obs))
        out["generator_equal"] = bool(torch.equal(a_gen, b_gen))
        out["rule_rng_untouched"] = all(b_rng)
        a_obs, _, _ = run(None, [(rgb, depth)])
        b_obs, _, _ = run(VS, [(rgb, depth)])
        out["near_bar_obs_differs"] = not torch.equal(a_obs[0], b_obs[0])
        before = rng_hash()
        torch.rand(1)
        out["rng_probe_detects_a_draw"] = rng_hash() != before

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


VALID = {"NAVRL_DEPTH_VALIDITY": "blob_near", "NAVRL_DEPTH_VALIDITY_WINDOW": "5", "NAVRL_DEPTH_VALIDITY_DENSITY": "0.5"}
DM_ENV = {"NAVRL_DEPTH_MODEL": "oakd", "NAVRL_DEPTH_MINZ_M": "0.4", "NAVRL_DEPTH_MAXZ_M": "20",
          "NAVRL_DEPTH_SIGMA_D_PX": "0.15", "NAVRL_DEPTH_FB_PXM": "66.1875", "NAVRL_DEPTH_HOLE_P": "0.02",
          "NAVRL_DEPTH_SEED": "4921"}
ENTRY_OK = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/realveh_receipt.json",
            "NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_CAMTRAIN_HFOV_DEG": "66", "NAVRL_CAMTRAIN_VFOV_DEG": "54",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}


class TestSettings(unittest.TestCase):
    def test_valid_and_bad(self):
        self.assertEqual(DV.settings_from_env(VALID), {"NAVRL_DEPTH_VALIDITY": "blob_near",
                                                       "NAVRL_DEPTH_VALIDITY_WINDOW": 5,
                                                       "NAVRL_DEPTH_VALIDITY_DENSITY": 0.5})
        bad = [{"NAVRL_DEPTH_VALIDITY": ""}, {"NAVRL_DEPTH_VALIDITY": "clamp"}, {"NAVRL_DEPTH_VALIDITY_WINDOW": ""},
               {"NAVRL_DEPTH_VALIDITY_WINDOW": "4"}, {"NAVRL_DEPTH_VALIDITY_WINDOW": "1"},
               {"NAVRL_DEPTH_VALIDITY_WINDOW": "33"}, {"NAVRL_DEPTH_VALIDITY_WINDOW": "5.0"},
               {"NAVRL_DEPTH_VALIDITY_DENSITY": "0"}, {"NAVRL_DEPTH_VALIDITY_DENSITY": "1.1"},
               {"NAVRL_DEPTH_VALIDITY_DENSITY": "nan"}, {"NAVRL_DEPTH_VALIDITY_DENSITY": ""},
               {"NAVRL_DEPTH_VALIDITY_DENSITY": "x"}]
        for change in bad:
            with self.assertRaises(ValueError, msg=change):
                DV.settings_from_env(dict(VALID, **change))
        edge = DV.settings_from_env(dict(VALID, NAVRL_DEPTH_VALIDITY_WINDOW="31", NAVRL_DEPTH_VALIDITY_DENSITY="1"))
        self.assertEqual((edge["NAVRL_DEPTH_VALIDITY_WINDOW"], edge["NAVRL_DEPTH_VALIDITY_DENSITY"]), (31, 1.0))

    def test_entry_lists_the_same_parameters(self):
        self.assertEqual(sorted(E.VALIDITY_PARAMETERS), sorted(DV.REQUIRED))
        self.assertEqual(E.VALIDITY_RULE, DV.RULE)
        self.assertEqual(pathlib.Path(E.VALIDITY).resolve(), (TASK_DIR / "navrl_depth_validity.py").resolve())


class TestRule(unittest.TestCase):
    def test_blob_turned_near_at_minz(self):
        r = cached("units")
        self.assertTrue(r["blob_invalid_in_model"])
        self.assertTrue(r["blob_is_20_in_model"])                                    # control: v2 encoding
        self.assertGreater(r["blob_pixels"], 0)
        self.assertEqual(r["blob_near_pixels"], r["blob_pixels"])
        self.assertTrue(r["blob_value_is_minz_over_cos"])
        self.assertAlmostEqual(r["blob_optical_z"][0], 0.4, places=5)
        self.assertAlmostEqual(r["blob_optical_z"][1], 0.4, places=5)
        self.assertTrue(r["rest_untouched"])
        self.assertTrue(r["input_unmodified"])

    def test_isolated_holes_untouched(self):
        r = cached("units")
        self.assertGreater(r["holes_invalid"], 0)                                    # holes exist
        self.assertEqual(r["holes_near"], 0)
        self.assertTrue(r["holes_output_equal"])
        self.assertGreater(r["dense_holes_near"], 0)                                 # control: dense holes are turned

    def test_no_return_and_target_untouched(self):
        r = cached("units")
        self.assertGreater(r["nohit_pixels"], 0)
        self.assertTrue(r["nohit_untouched"])                                        # beside a near blob
        self.assertTrue(r["target_untouched"])
        self.assertTrue(r["all_nohit_equal"])
        self.assertEqual(r["all_nohit_near"], 0)
        self.assertTrue(r["valid_bar_equal"])                                         # a valid 0.5 m bar is not near
        self.assertEqual(r["valid_bar_near"], 0)

    def test_density_threshold_edge(self):
        e = cached("units")["edge"]
        self.assertTrue(e["13_of_25_at_0.52"])                                       # equal counts as dense
        self.assertFalse(e["12_of_25_at_0.52"])
        self.assertTrue(e["13_of_25_at_0.5"])
        self.assertFalse(e["12_of_25_at_0.5"])
        self.assertTrue(e["25_of_25_at_1.0"])
        self.assertFalse(e["24_of_25_at_1.0"])
        r = cached("units")
        self.assertFalse(r["corner_4_of_9_at_0.5"])                                  # padding is not counted
        self.assertTrue(r["corner_5_of_9_at_0.5"])
        self.assertTrue(r["valid_pixel_never_near"])

    def test_refusals(self):
        r = cached("units")
        self.assertEqual(r["no_model"], "refused")
        self.assertEqual(r["stale_mask"], "refused")
        self.assertEqual(r["minz_zero"], "refused")
        self.assertGreater(r["blob_near_pixels"], 0)                                 # control: the fresh call works


class TestMeasuredMask(unittest.TestCase):
    def test_explicit_mask_preserves_valid_and_isolated_pixels(self):
        r = cached("external")
        self.assertGreater(r["near_count"], 0)
        self.assertGreater(r["invalid_count"], r["near_count"])
        for key in ("valid_far_unchanged", "near_value", "rest_unchanged", "isolated_not_near", "clear_equal"):
            self.assertTrue(r[key], key)

    def test_no_state_noise_or_input_mutation(self):
        r = cached("external")
        for key in ("inputs_unchanged", "new_depth", "rng_unchanged", "dtype_device"):
            self.assertTrue(r[key], key)

    def test_simulation_output_and_mask_remain_exact(self):
        r = cached("external")
        self.assertTrue(r["simulation_equal"])
        self.assertTrue(r["simulation_mask_equal"])
        self.assertGreater(r["near_count"], 0)

    def test_malformed_masks_geometry_and_settings_are_rejected(self):
        r = cached("external")
        self.assertEqual(len(r["refused"]), 27)
        for case, refused in r["refused"].items():
            self.assertTrue(refused, case)
        self.assertTrue(r["clear_equal"])  # positive control: strict validation accepts a valid unchanged frame


class TestWrapped(unittest.TestCase):
    def test_install_order(self):
        r = cached("wrapped")
        self.assertEqual(r["after_model"], "refused")
        self.assertEqual(r["after_ablation"], "refused")
        self.assertEqual(r["twice"], "refused")
        self.assertTrue(r["fused"])                                                  # control: production order ran

    def test_signature_preserved(self):
        r = cached("wrapped")
        self.assertTrue(r["signature_kept"])
        self.assertTrue(r["has_wrapped"])
        self.assertTrue(r["name_kept"])
        self.assertTrue(r["unwrapped_signature_differs"])

    def test_near_bar_reaches_the_fused_scan(self):
        f = cached("wrapped")["fused"]
        self.assertEqual(f["model_only_0p3"], [12.0, 12.0])                          # v2: no return
        self.assertLess(f["rule_0p3"][1], 0.45)                                       # rule: about MinZ
        self.assertGreater(f["rule_0p3"][0], 0.39)
        self.assertAlmostEqual(f["model_only_0p5"][0], 0.5, places=4)                # control: a valid bar
        self.assertEqual(f["rule_0p5"], f["model_only_0p5"])

    def test_inert_without_dense_invalid_and_rng_untouched(self):
        r = cached("wrapped")
        self.assertTrue(r["inert_equal"])
        self.assertTrue(r["generator_equal"])
        self.assertTrue(r["rule_rng_untouched"])
        self.assertTrue(r["near_bar_obs_differs"])                                   # positive control
        self.assertTrue(r["rng_probe_detects_a_draw"])

    def test_telemetry_bound_at_install(self):
        r = cached("wrapped")
        self.assertEqual(r["tele_events"], ["install", "first", "summary"])
        self.assertEqual(r["tele_quiet_lines"], 0)
        self.assertEqual(r["tele_install_settings"], [DV.settings_from_env(VALID)])
        s = r["tele_summary"][0]
        self.assertEqual(s["calls"], DV.STATS_CALLS)
        self.assertEqual(s["sentinel_mismatch"], 0)
        self.assertGreater(s["near_pixels"], 0)
        self.assertLessEqual(s["near_pixels"], s["invalid_pixels"])
        self.assertGreater(s["invalid_pixels"] - s["near_pixels"], 0)                # holes stayed no return


class TestEntry(unittest.TestCase):
    def test_off_does_not_load_the_module_and_keeps_v2_settings(self):
        calls = []
        real = E._load_private

        def spy(name, path):
            calls.append(pathlib.Path(path).name)
            return real(name, path)

        with mock.patch.object(E, "_load_private", side_effect=spy):
            s = E.settings(dict(ENTRY_OK, **DM_ENV))
        self.assertEqual(calls, ["navrl_depth_model.py"])
        self.assertNotIn("depth_validity", s)                                         # v2's settings keys unchanged
        self.assertEqual(sorted(s), ["camtrain", "depth_model", "receipt", "vehicle_rand"])
        calls.clear()
        with mock.patch.object(E, "_load_private", side_effect=spy):
            s = E.settings(dict(ENTRY_OK, **DM_ENV, **VALID))
        self.assertEqual(calls, ["navrl_depth_model.py", "navrl_depth_validity.py"])   # control: ON loads it
        self.assertEqual(s["depth_validity"], DV.settings_from_env(VALID))

    def test_refusals(self):
        bad = [dict(VALID), {"NAVRL_DEPTH_VALIDITY_WINDOW": "5"}, {"NAVRL_DEPTH_VALIDITY_DENSITY": "0.5"},
               dict(DM_ENV, NAVRL_DEPTH_VALIDITY="near"), dict(DM_ENV, NAVRL_DEPTH_VALIDITY_WINDOW="5"),
               dict(DM_ENV, **dict(VALID, NAVRL_DEPTH_VALIDITY_WINDOW="6")),
               dict(DM_ENV, **dict(VALID, NAVRL_DEPTH_VALIDITY_DENSITY=""))]
        for change in bad:
            with self.assertRaises(SystemExit, msg=change):
                E.settings(dict(ENTRY_OK, **change))
        self.assertIsNotNone(E.settings(dict(ENTRY_OK, **DM_ENV, **VALID))["depth_validity"])   # positive control

    def test_settings_import_no_torch(self):
        code = ("import importlib.util, json, sys\n"
                "spec = importlib.util.spec_from_file_location('e', sys.argv[1])\n"
                "e = importlib.util.module_from_spec(spec); spec.loader.exec_module(e)\n"
                "s = e.settings(json.loads(sys.argv[2]))\n"
                "print(json.dumps({'torch': 'torch' in sys.modules, 'validity': s.get('depth_validity') is not None}))\n")
        r = subprocess.run([sys.executable, "-B", "-c", code, str(ROOT / "tools/realveh_entry.py"),
                            json.dumps(dict(ENTRY_OK, **DM_ENV, **VALID))], capture_output=True, text=True,
                           env={"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1"}, timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        self.assertEqual(json.loads(r.stdout.strip().splitlines()[-1]), {"torch": False, "validity": True})

    def test_main_installs_validity_before_the_depth_model_and_hashes_it(self):
        text = (ROOT / "tools/realveh_entry.py").read_text()
        i_v = text.index("validity.install(s[\"depth_validity\"])")
        i_d = text.index("depth.install(s[\"depth_model\"])")
        i_a = text.index("ablation.install(ablation.condition_from_env())")
        self.assertLess(i_v, i_d)
        self.assertLess(i_d, i_a)
        self.assertIn("info[\"depth_validity_sha256\"] = sha256(VALIDITY)", text)
        self.assertNotIn("depth_validity", E.HASHED)                                  # v2 receipt hashes unchanged
        self.assertIn("depth_model", E.HASHED)                                        # control

    @staticmethod
    def _importers(name):
        pattern = re.compile(r"^\s*(?:import\s+[\w.]*%s\b|from\s+[\w.]*%s\s+import|from\s+[\w.]+\s+import\s+[^\n]*\b%s\b)"
                             r"|import_module\(\s*[\"'][\w.]*\b%s[\"']|spec_from_file_location\([^\n]*%s"
                             % ((name,) * 5), re.M)
        hits = []
        for base in ("aerial_gym", "motar_eval", "tools"):
            for path in (ROOT / base).rglob("*.py"):
                if path.stem == name:
                    continue
                if pattern.search(path.read_text(errors="ignore")):
                    hits.append(str(path.relative_to(ROOT)))
        return sorted(hits)

    def test_rule_import_is_limited_to_explicit_entries(self):
        importers = set(self._importers("navrl_depth_validity"))
        self.assertIn("tools/realveh_entry.py", importers)
        self.assertTrue(importers <= {"tools/realveh_entry.py", "tools/camera_policy_runtime.py"}, importers)
        self.assertEqual(self._importers("navrl_depth_model"), ["tools/realveh_entry.py"])   # rule does not import it


if __name__ == "__main__":
    unittest.main()
