"""CPU tests for the real-vehicle study (2026-11-08): the 0.50 kg navrl_rv05_quad (quad_navrl_rv05.urdf,
navrl_rv05_quad_config.py), the attitude-gain scaling and receipt of tools/realveh_entry.py (+ tools/realveh_python),
the opt-in vehicle randomization (navrl_vehicle_rand.py) and the OAK-D Pro depth model (navrl_depth_model.py).

No GPU, Isaac Gym or simulator. Safe inside the single-process PUBLIC_CPU suite (AGENTS.md known pitfalls): every
production module that needs the `aerial_gym` package runs in a child process, which replaces the package
`__init__` modules with path-only stubs; nothing here registers a module in this process's sys.modules, and the
environment and sys.argv are changed only through unittest.mock.patch, which restores them. In-process modules are
loaded by file path under private names and import only the standard library at module level. Every negative check
is paired with a positive control.
Run: PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES= python -B tests/test_realveh.py
"""
import importlib.util
import io
import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile
import textwrap
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1]
QUAD = ROOT / "resources/robots/quad"
OLD_URDF, NEW_URDF = QUAD / "quad_navrl_collide.urdf", QUAD / "quad_navrl_rv05.urdf"
TASK_DIR = ROOT / "aerial_gym/task/navrl_task"
G = 9.81


def _load(name, path):
    """A module by file path under a private name; not registered in sys.modules."""
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


E = _load("realveh_entry_under_test", ROOT / "tools/realveh_entry.py")
VR = _load("navrl_vehicle_rand_under_test", TASK_DIR / "navrl_vehicle_rand.py")
DM = _load("navrl_depth_model_under_test", TASK_DIR / "navrl_depth_model.py")

# ---------------------------------------------------------------------------------------------------------------------
# Child process: path-only stubs for every aerial_gym package, then the real modules.
CHILD = textwrap.dedent(r'''
    import hashlib, importlib, importlib.util, inspect, json, math, os, pathlib, sys, types
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

    def load(name, path):
        spec = importlib.util.spec_from_file_location(name, str(path))
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        return module

    def rng_hash():
        return hashlib.sha256(torch.get_rng_state().numpy().tobytes()).hexdigest()

    mode, arg = sys.argv[2], json.loads(sys.argv[3])
    out = {}
    TASK = ROOT / "aerial_gym/task/navrl_task"

    if mode == "config":
        cm = importlib.import_module("aerial_gym.config.robot_config.navrl_rv05_quad_config")
        bm = importlib.import_module("aerial_gym.config.robot_config.navrl_quad_config")
        C, B = cm.NavRLRv05QuadCfg, bm.NavRLQuadCfg
        m, mb = C.control_allocator_config.motor_model_config, B.control_allocator_config.motor_model_config
        out = {"file": C.robot_asset.file, "asset_folder": str(C.robot_asset.asset_folder),
               "base_file": B.robot_asset.file, "subclass": issubclass(C, B),
               "allocation": C.control_allocator_config.allocation_matrix,
               "base_allocation": B.control_allocator_config.allocation_matrix,
               "max_thrust": m.max_thrust, "base_max_thrust": mb.max_thrust, "min_thrust": m.min_thrust,
               "tau": [m.motor_time_constant_increasing_min, m.motor_time_constant_increasing_max,
                       m.motor_time_constant_decreasing_min, m.motor_time_constant_decreasing_max],
               "ttr": m.thrust_to_torque_ratio, "k": [m.motor_thrust_constant_min, m.motor_thrust_constant_max],
               "nominal_mass": cm.NOMINAL_MASS_KG, "tw": cm.THRUST_TO_WEIGHT,
               "declared_ratio": list(cm.INERTIA_RATIO_VS_NAVRL_QUAD), "docstring": C.__doc__,
               "module_file": cm.__file__}

    elif mode in ("vehrand", "equivalence", "vehrand_default"):
        BM = importlib.import_module("aerial_gym.robots.base_multirotor")
        MMm = importlib.import_module("aerial_gym.control.motor_model")
        CA = importlib.import_module("aerial_gym.control.control_allocation")
        VR = importlib.import_module("aerial_gym.task.navrl_task.navrl_vehicle_rand")
        quad = importlib.import_module("aerial_gym.config.robot_config.navrl_quad_config").NavRLQuadCfg
        rv05 = importlib.import_module("aerial_gym.config.robot_config.navrl_rv05_quad_config").NavRLRv05QuadCfg
        N = 6

        def fake_class():
            """A multirotor whose reset_idx IS BaseMultirotor.reset_idx and whose init_tensors builds what the real one
            builds for these checks: the controller's mass / gravity and the real ControlAllocator + MotorModel."""
            class FakeMultirotor:
                reset_idx = BM.BaseMultirotor.reset_idx

                def init_tensors(self, cfg, mass):
                    n = self.num_envs = N
                    self.device = "cpu"
                    self.robot_state = torch.zeros(n, 13)
                    self.min_init_state = torch.tensor(cfg.init_config.min_init_state, dtype=torch.float32).expand(n, -1)
                    self.max_init_state = torch.tensor(cfg.init_config.max_init_state, dtype=torch.float32).expand(n, -1)
                    self.env_bounds_min, self.env_bounds_max = torch.zeros(n, 3), torch.full((n, 3), 10.0)
                    self.controller = types.SimpleNamespace(
                        randomize_params=lambda env_ids: None, mass=torch.full((n,), float(mass)).unsqueeze(1),
                        gravity=torch.tensor([0.0, 0.0, -9.81]).expand(n, -1))
                    self.control_allocator = CA.ControlAllocator(num_envs=n, dt=0.01,
                                                                 config=cfg.control_allocator_config, device="cpu")
                    self.update_states = lambda: None
            return FakeMultirotor

        def wrench_sequence(steps=30, scale=1.0):
            g = torch.Generator().manual_seed(5)
            seq = []
            for _ in range(steps):
                w = torch.zeros(N, 6)
                w[:, 2] = 1.0 + 7.0 * torch.rand(N, generator=g)       # up to 8 N: the 1.38 N cap binds
                w[:, 3:6] = 0.04 * (2.0 * torch.rand(N, 3, generator=g) - 1.0)
                seq.append(w * scale)
            return seq

        if mode == "vehrand_default":
            orig = (BM.BaseMultirotor.init_tensors, BM.BaseMultirotor.reset_idx, MMm.MotorModel.update_motor_thrusts)
            VR.install(arg)                                   # production default: BaseMultirotor, MotorModel
            now = (BM.BaseMultirotor.init_tensors, BM.BaseMultirotor.reset_idx, MMm.MotorModel.update_motor_thrusts)
            out["wrapped"] = [a is not b and getattr(b, "__wrapped__", None) is a for a, b in zip(orig, now)]

        elif mode == "vehrand":
            settings, record = arg["settings"], arg.get("record")
            Fake = fake_class()
            if settings is not None:
                out["install"] = VR.install(settings, multirotor_cls=Fake, record_path=record)
                try:
                    VR.install(settings, multirotor_cls=Fake)
                    out["second_install"] = "accepted"
                except RuntimeError:
                    out["second_install"] = "refused"
            torch.manual_seed(11)
            robot = Fake()
            try:
                robot.init_tensors(quad, arg.get("mass", 0.25))
            except (RuntimeError, FileExistsError) as exc:
                out["init_error"] = str(exc)
                print("CHILD" + json.dumps(out))
                raise SystemExit(0)
            mm = robot.control_allocator.motor_model
            robot.reset_idx(torch.arange(N))
            out["state_after_reset"] = mm.current_motor_thrust.tolist()
            thrust, yaw, ratio_err, factors, ref_over = [], [], 0.0, {}, 0
            cap = getattr(mm, "_veh_rand_cap", None)
            for k, w in enumerate(wrench_sequence()):
                if k == 15:
                    factors["before"] = getattr(mm, "_veh_rand_factor", torch.ones(N, 4)).clone().tolist()
                    out["state_before_partial_reset"] = mm.current_motor_thrust.tolist()
                    robot.reset_idx(torch.tensor([1, 4]))
                    factors["after"] = getattr(mm, "_veh_rand_factor", torch.ones(N, 4)).clone().tolist()
                    out["state_after_partial_reset"] = mm.current_motor_thrust.tolist()
                ref = torch.bmm(robot.control_allocator.inv_force_torque_allocation_matrix, w.unsqueeze(-1)).squeeze(-1)
                ref_over += int((ref > (cap if cap is not None else mm.max_thrust)).sum())
                forces, torques = robot.control_allocator.allocate_output(w, "wrench")
                if settings is not None:
                    ratio_err = max(ratio_err, float((forces[:, :, 2] - mm.current_motor_thrust
                                                      * mm._veh_rand_factor).abs().max()))
                thrust.append(forces[:, :, 2].tolist())
                yaw.append(torques[:, :, 2].tolist())
            out.update({"thrust": thrust, "yaw": yaw, "ratio_err": ratio_err, "factors": factors,
                        "tau_inc": mm.motor_time_constants_increasing.tolist(),
                        "tau_dec": mm.motor_time_constants_decreasing.tolist(), "rng": rng_hash(),
                        "max_thrust": float(mm.max_thrust[0, 0]), "ref_over_cap": ref_over,
                        "cap": None if cap is None else cap.tolist(),
                        "record": getattr(robot, "_veh_rand_record", None)})
            unready = object.__new__(MMm.MotorModel)
            try:
                MMm.MotorModel.update_motor_thrusts(unready, torch.zeros(N, 4))
                out["update_before_init"] = "accepted"
            except RuntimeError:
                out["update_before_init"] = "refused"
            except Exception as exc:                          # OFF: the original fails on the bare object
                out["update_before_init"] = "original:" + type(exc).__name__

        elif mode == "equivalence":
            # (a) navrl_quad (0.25 kg) capped at T/W 2.25 by the module, no randomization; (b) Option A (0.50 kg,
            # quad_navrl_rv05.urdf, 2.7590625 N) unmodified, commanded with 2x the wrench (its velocity-loop forces are
            # proportional to mass and its attitude gains are x2). The wrapper goes on subclasses, so (b) stays plain.
            # The global seed is reset before each construction and each reset, so both draw the same numbers.
            FakeA, FakeB = fake_class(), fake_class()
            MMa = type("MotorModelCapped", (MMm.MotorModel,), {})
            VR.install(arg, multirotor_cls=FakeA, motor_model_cls=MMa)
            a, b = FakeA(), FakeB()
            torch.manual_seed(21)
            a.init_tensors(quad, 0.25)                        # the wrapper computes the cap and draws (own generator)
            a.control_allocator.motor_model.__class__ = MMa
            torch.manual_seed(21)
            b.init_tensors(rv05, 0.5)
            for r in (a, b):
                torch.manual_seed(22)
                r.reset_idx(torch.arange(N))
            ma, mb = a.control_allocator.motor_model, b.control_allocator.motor_model
            out["state0"] = {"a": ma.current_motor_thrust.tolist(), "b": mb.current_motor_thrust.tolist()}
            fa, fb, ta, tb = [], [], [], []
            for w in wrench_sequence(60):
                f1, t1 = a.control_allocator.allocate_output(w, "wrench")
                f2, t2 = b.control_allocator.allocate_output(2.0 * w, "wrench")
                fa.append(f1[:, :, 2].tolist()); fb.append(f2[:, :, 2].tolist())
                ta.append(t1[:, :, 2].tolist()); tb.append(t2[:, :, 2].tolist())
            sat = torch.zeros(N, 6)
            sat[:, 2] = 100.0                                  # every motor saturated until steady state
            for _ in range(100):
                f1, _ = a.control_allocator.allocate_output(sat, "wrench")
                f2, _ = b.control_allocator.allocate_output(2.0 * sat, "wrench")
            out.update({"fa": fa, "fb": fb, "ta": ta, "tb": tb, "sat_a": f1[:, :, 2].tolist(),
                        "sat_b": f2[:, :, 2].tolist(), "cap": float(ma._veh_rand_cap[0, 0]),
                        "max_b": float(mb.max_thrust[0, 0]), "record": a._veh_rand_record,
                        "alloc_equal": quad.control_allocator_config.allocation_matrix
                        == rv05.control_allocator_config.allocation_matrix,
                        "cq": [quad.control_allocator_config.motor_model_config.thrust_to_torque_ratio,
                               rv05.control_allocator_config.motor_model_config.thrust_to_torque_ratio],
                        "tau": [quad.control_allocator_config.motor_model_config.motor_time_constant_increasing_min,
                                rv05.control_allocator_config.motor_model_config.motor_time_constant_increasing_min]})

    elif mode.startswith("depth"):
        os.environ.update({"NAVRL_LIDAR_HBEAMS": "72", "NAVRL_LIDAR_VBEAMS": "4"})
        DM = importlib.import_module("aerial_gym.task.navrl_task.navrl_depth_model")
        ABL = importlib.import_module("aerial_gym.task.navrl_task.navrl_sensor_ablation")
        N, H, W, HB, VB, LMAX = 4, 45, 80, 72, 4, 12.0
        counter = [0]

        def perception_module():
            counter[0] += 1
            mod = load("aerial_gym.task.navrl_task.navrl_perception_rv%d" % counter[0], TASK / "navrl_perception.py")
            assert mod.HBEAMS == HB and mod.VBEAMS == VB
            return mod

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

        def scene(seed, steps=6, bar_m=None, target=True):
            g = torch.Generator().manual_seed(seed)
            frames, lidar = [], []
            for k in range(steps):
                depth = torch.full((N, H, W), 10.0)
                rgb = torch.full((N, 3, H, W), 0.15)
                for e in range(N):
                    if bar_m is not None:
                        depth[e, :, 38:43] = bar_m
                        continue
                    for _ in range(5):
                        c = int(torch.randint(0, W - 2, (1,), generator=g))
                        depth[e, :, c:c + 2] = float(torch.empty(1).uniform_(1.5, 9.0, generator=g))
                    if target and e < 2:
                        col = 34 + 2 * k + 4 * e
                        rgb[e, 0, 20:24, col:col + 5] = 0.88
                        rgb[e, 1, 20:24, col:col + 5] = 0.08
                        rgb[e, 2, 20:24, col:col + 5] = 0.045
                        depth[e, 20:24, col:col + 5] = 3.0 + 0.1 * k
                frames.append((rgb, depth))
                x = torch.full((N, VB, HB), LMAX)
                for e in range(N):
                    bins = torch.randint(0, HB, (8,), generator=g)
                    x[e, :, bins] = torch.empty(8).uniform_(3.2, 9.0, generator=g)
                lidar.append(x.reshape(N, VB * HB))
            return frames, lidar

        def run(perc, frames, lidar, fork=True):
            recs = []
            pos = torch.tensor([[0.0, 0.0, 1.0]]).repeat(N, 1)
            vel = torch.tensor([[0.4, 0.0, 0.0]]).repeat(N, 1)
            quat = torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(N, 1)
            for k, ((rgb, depth), L) in enumerate(zip(frames, lidar)):
                kw = dict(rgb=rgb, depth=depth.clone(), lidar_m=L.clone(), drone_pos_w=pos, drone_vel_w=vel,
                          vehicle_quat=quat, yaw_rate=torch.zeros(N), previous_action=torch.zeros(N, 4),
                          max_velocity=2.5, flight_altitude=1.0, training=False)
                if fork:
                    with torch.random.fork_rng():
                        torch.manual_seed(1000 + k)
                        obs, diag = perc.observe(**kw)
                else:
                    obs, diag = perc.observe(**kw)
                rec = {"obs": obs.clone(), "visible": diag["visible"].clone(), "state": perc.tracker.state.clone()}
                fused = getattr(perc, "_ablation_fused_m", None)
                if fused is not None:
                    rec["fused"] = fused.clone()
                recs.append(rec)
            return recs

        def equal(a, b, keys=("obs", "visible", "state")):
            return all(torch.equal(x[k], y[k]) for x, y in zip(a, b) for k in keys)

        def settings(**kw):
            s = {"NAVRL_DEPTH_MINZ_M": 0.0, "NAVRL_DEPTH_MAXZ_M": 20.0, "NAVRL_DEPTH_SIGMA_D_PX": 0.0,
                 "NAVRL_DEPTH_FB_PXM": 57.2, "NAVRL_DEPTH_HOLE_P": 0.0, "NAVRL_DEPTH_SEED": 3}
            s.update({"NAVRL_DEPTH_" + k: v for k, v in kw.items()})
            return s

        ON = settings(MINZ_M=0.4, SIGMA_D_PX=0.5, HOLE_P=0.05)

        if mode == "depth_identity":
            frames, lidar = scene(7)
            off_mod, zero_mod, on_mod = perception_module(), perception_module(), perception_module()
            orig = off_mod.NavRLPerceptionModule.observe
            DM.install_perception(zero_mod, settings())
            DM.install_perception(on_mod, ON)
            off, zero, on = run(make(off_mod), frames, lidar), run(make(zero_mod), frames, lidar), \
                run(make(on_mod), frames, lidar)
            out["zero_equals_off"] = equal(off, zero)
            out["on_differs"] = not equal(off, on, ("obs",))
            wrapped = zero_mod.NavRLPerceptionModule.observe
            out["signature_kept"] = str(inspect.signature(wrapped)) == str(inspect.signature(orig))
            out["has_wrapped"] = getattr(wrapped, "__wrapped__", None) is not None
            out["name_kept"] = wrapped.__name__ == "observe"
            out["unwrapped_signature_differs"] = (str(inspect.signature(wrapped, follow_wrapped=False))
                                                 != str(inspect.signature(orig)))
            try:
                DM.install_perception(zero_mod, settings())
                out["second_install"] = "accepted"
            except RuntimeError:
                out["second_install"] = "refused"
            # global RNG: the wrapped observe consumes exactly what the unwrapped one does
            torch.manual_seed(5)
            run(make(off_mod), frames, lidar, fork=False)
            a = rng_hash()
            torch.manual_seed(5)
            run(make(on_mod), frames, lidar, fork=False)
            out["observe_rng_equal"] = a == rng_hash()
            torch.rand(1)                                     # positive control: one global draw changes the hash
            out["rng_probe_detects_a_draw"] = a != rng_hash()

        elif mode == "depth_stats":
            perc = make(perception_module())
            cos = DM.ray_cosine(perc, "cpu")
            out["cos_min"], out["cos_max"] = float(cos.min()), float(cos.max())

            def plane(z0):
                return (z0 / cos).expand(N, H, W).clone()

            k = 0.15 / 57.2
            sig = {}
            for z0 in (1.0, 2.0, 4.0, 6.0):
                d = plane(z0)
                state = rng_hash()
                o = DM.apply(perc, d, settings(SIGMA_D_PX=0.15))
                out.setdefault("rng_untouched", True)
                out["rng_untouched"] = out["rng_untouched"] and state == rng_hash()
                resid_z = (o - d) * cos
                sig["%g" % z0] = [float(resid_z.std()), k * z0 * z0]
                out.setdefault("invalid_counts_noise", []).append(int(perc._depth_model_invalid.sum()))
            out["sigma"] = sig
            # determinism: same seed -> same frame, other seed -> other frame
            p1, p2, p3 = make(perception_module()), make(perception_module()), make(perception_module())
            d = plane(4.0)
            a = DM.apply(p1, d, settings(SIGMA_D_PX=0.15))
            b = DM.apply(p2, d, settings(SIGMA_D_PX=0.15))
            c = DM.apply(p3, d, settings(SIGMA_D_PX=0.15, SEED=4))
            out["same_seed_equal"], out["other_seed_differs"] = bool(torch.equal(a, b)), not bool(torch.equal(a, c))
            total = N * H * W
            out["total"] = total
            for name, z0, s in (("minz_below", 0.3, settings(MINZ_M=0.4)), ("minz_above", 0.5, settings(MINZ_M=0.4)),
                                ("maxz_beyond", 9.0, settings(MAXZ_M=8.0)), ("maxz_within", 7.0, settings(MAXZ_M=8.0)),
                                ("holes", 2.0, settings(HOLE_P=0.1)), ("no_holes", 2.0, settings(HOLE_P=0.0))):
                d = plane(z0)
                o = DM.apply(perc, d, s)
                inv = perc._depth_model_invalid
                no_hit = (d - 10.0).abs() <= 1e-3
                out[name] = {"invalid": int(inv.sum()), "no_hit": int(no_hit.sum()),
                             "invalid_values": sorted(set(o[inv].tolist())),
                             "valid_unchanged": bool(torch.equal(o[~inv], d[~inv]))}
            # the renderer's own no-hit pixels pass through unchanged, even with every effect on
            d = torch.full((N, H, W), 10.0)
            d[:, :, 10:20] = 3.0
            o = DM.apply(perc, d, settings(MINZ_M=0.4, MAXZ_M=8.0, SIGMA_D_PX=0.5, HOLE_P=0.2))
            out["no_hit_passthrough"] = bool(torch.equal(o[:, :, 20:], d[:, :, 20:]))
            out["no_hit_control_changed"] = not bool(torch.equal(o[:, :, 10:20], d[:, :, 10:20]))
            # MaxZ beyond perception's far plane refuses on the first frame; equal to it is accepted
            for name, maxz in (("maxz_too_far", 25.0), ("maxz_far_plane", 20.0)):
                try:
                    DM.apply(make(perception_module()), plane(2.0), settings(MAXZ_M=maxz))
                    out[name] = "accepted"
                except RuntimeError:
                    out[name] = "refused"

        elif mode == "depth_compose":
            def composed(s, ablation_first=False):
                mod = perception_module()
                if ablation_first:
                    ABL.install_perception(mod, "no_lidar")
                    DM.install_perception(mod, s)        # must refuse
                if s is not None:
                    DM.install_perception(mod, s)
                ABL.install_perception(mod, "no_lidar")
                return mod

            def near_zero_bins(mod):
                b = mod.lidar_bin_bearings("cpu")
                return (torch.atan2(torch.sin(b), torch.cos(b)).abs() < math.radians(2.0))

            try:
                composed(settings(), ablation_first=True)
                out["ablation_first"] = "accepted"
            except RuntimeError:
                out["ablation_first"] = "refused"
            mod = composed(settings(MINZ_M=0.4))
            frames, lidar = scene(9)
            p = make(mod)
            run(p, frames[:1], lidar[:1])
            out["ablation_bound_lidar_m"] = bool(torch.equal(p._ablation_raw_lidar_m, lidar[0]))
            _, lidar_b = scene(10)
            r1, r2 = run(make(mod), frames, lidar), run(make(mod), frames, lidar_b)
            out["no_lidar_still_removes_lidar"] = equal(r1, r2) and not all(
                torch.equal(a, b) for a, b in zip(lidar, lidar_b))
            for name, s, bar in (("bar_0p3_model", settings(MINZ_M=0.4), 0.3),
                                 ("bar_0p5_model", settings(MINZ_M=0.4), 0.5),
                                 ("bar_0p3_no_model", None, 0.3)):
                m = composed(s)
                fr, li = scene(11, steps=2, bar_m=bar, target=False)
                rec = run(make(m), fr, li)[-1]
                z = near_zero_bins(m)
                out[name] = [float(rec["fused"][:, :, z].min()), float(rec["fused"][:, :, z].max())]
            fr, li = scene(12, steps=3, bar_m=6.0, target=False)
            q = run(make(composed(settings())), fr, li)
            n = run(make(composed(settings(SIGMA_D_PX=0.5))), fr, li)
            out["noise_reaches_fused"] = not all(torch.equal(a["fused"], b["fused"]) for a, b in zip(q, n))
            out["zero_fused_equals_unmodelled"] = all(
                torch.equal(a["fused"], b["fused"]) for a, b in zip(q, run(make(composed(None)), fr, li)))

    print("CHILD" + json.dumps(out))
''')


def child(mode, arg=None, timeout=600):
    env = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")}
    env.update(PYTHONNOUSERSITE="1", CUDA_VISIBLE_DEVICES="")
    done = subprocess.run([sys.executable, "-B", "-c", CHILD, str(ROOT), mode, json.dumps(arg)], env=env,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, timeout=timeout)
    if done.returncode != 0:
        raise AssertionError(done.stderr[-3000:])
    lines = [ln for ln in done.stdout.splitlines() if ln.startswith("CHILD")]
    if not lines:
        raise AssertionError(done.stdout[-2000:] + done.stderr[-2000:])
    out = json.loads(lines[-1][len("CHILD"):])
    out["_stderr"] = done.stderr
    return out


def telemetry(result, prefix):
    """The JSON lines a module logged on the child's stderr."""
    return [json.loads(ln[len(prefix):]) for ln in result["_stderr"].splitlines() if ln.startswith(prefix)]


_CACHE = {}


def cached(mode, arg=None):
    key = (mode, json.dumps(arg, sort_keys=True))
    if key not in _CACHE:
        _CACHE[key] = child(mode, arg)
    return _CACHE[key]


# ---------------------------------------------------------------------------------------------------------------------
# 1. The 0.50 kg vehicle
def _links(path):
    return {link.get("name"): link for link in ET.parse(str(path)).getroot().findall("link")}


def _joints(path):
    return {j.get("name"): (j.find("child").get("link"), j.find("origin").get("xyz"), j.find("origin").get("rpy"))
            for j in ET.parse(str(path)).getroot().findall("joint")}


def _motor_origins(path):
    return {child_link: tuple(float(v) for v in xyz.split()) for child_link, xyz, _ in _joints(path).values()
            if child_link.startswith("motor_")}


def _independent_inertia(path):
    """The test's own parallel-axis sum (not the entry's): base tensor + point masses at the joint origins."""
    links, origins = _links(path), {c: tuple(float(v) for v in xyz.split()) for c, xyz, _ in _joints(path).values()}
    base = links["base_link"].find("inertial/inertia")
    ixx, iyy, izz = (float(base.get(k)) for k in ("ixx", "iyy", "izz"))
    for name, link in links.items():
        if name == "base_link":
            continue
        m = float(link.find("inertial/mass").get("value"))
        x, y, z = origins[name]
        ixx, iyy, izz = ixx + m * (y * y + z * z), iyy + m * (x * x + z * z), izz + m * (x * x + y * y)
    return ixx, iyy, izz


def _allocation_matches(allocation, origins):
    """Roll row = y_i, pitch row = -x_i of motor_i, z = 0."""
    roll, pitch = allocation[3], allocation[4]
    return all(abs(roll[i] - origins["motor_%d" % i][1]) < 1e-12 and abs(pitch[i] + origins["motor_%d" % i][0]) < 1e-12
               and origins["motor_%d" % i][2] == 0.0 for i in range(4))


class TestRv05Vehicle(unittest.TestCase):
    def test_urdf_total_mass_is_0p50_kg(self):
        total = sum(float(link.find("inertial/mass").get("value")) for link in _links(NEW_URDF).values())
        self.assertAlmostEqual(total, 0.50, places=12)
        reference = sum(float(link.find("inertial/mass").get("value")) for link in _links(OLD_URDF).values())
        self.assertAlmostEqual(reference, 0.25, places=12)                         # positive control: F's body

    def test_every_mass_and_inertia_doubled_and_the_rest_unchanged(self):
        old, new = _links(OLD_URDF), _links(NEW_URDF)
        self.assertEqual(sorted(old), sorted(new))
        doubled = 0
        for name in old:
            mo, mn = (float(x[name].find("inertial/mass").get("value")) for x in (old, new))
            self.assertEqual(mn, 2.0 * mo, name)
            doubled += int(mo > 0.0)
            io, inn = (x[name].find("inertial/inertia") for x in (old, new))
            for k in ("ixx", "ixy", "ixz", "iyx", "iyy", "iyz", "izx", "izy", "izz"):
                self.assertEqual(float(inn.get(k)), 2.0 * float(io.get(k)), (name, k))
            for tag in ("visual", "collision"):
                a, b = old[name].find(tag), new[name].find(tag)
                self.assertEqual(a is None, b is None)
                if a is not None:
                    self.assertEqual(ET.tostring(a), ET.tostring(b), (name, tag))
        self.assertEqual(doubled, 5)                                                # base + 4 motors carry mass
        self.assertEqual(_joints(OLD_URDF), _joints(NEW_URDF))
        self.assertIn("PLACEHOLDER GEOMETRY", NEW_URDF.read_text())

    def test_assembled_inertia_is_twice_navrl_quad(self):
        j_old, j_new = _independent_inertia(OLD_URDF), _independent_inertia(NEW_URDF)
        for a, b in zip(j_new, (1.69e-3, 1.69e-3, 3.38e-3)):
            self.assertAlmostEqual(a, b, places=12)
        self.assertEqual(tuple(n / o for n, o in zip(j_new, j_old)), (2.0, 2.0, 2.0))
        # positive control: the ratio of a body with itself is 1, so the 2.0 above is not a constant of the code
        self.assertEqual(tuple(n / o for n, o in zip(j_old, j_old)), (1.0, 1.0, 1.0))

    def test_config_allocation_arm_equals_urdf_joint_origins(self):
        r = cached("config")
        self.assertTrue(r["subclass"])
        self.assertEqual(r["file"], "quad_navrl_rv05.urdf")
        self.assertEqual(pathlib.Path(r["asset_folder"]).resolve(), QUAD.resolve())
        self.assertEqual(pathlib.Path(r["module_file"]).resolve(),
                         (ROOT / "aerial_gym/config/robot_config/navrl_rv05_quad_config.py").resolve())
        origins = _motor_origins(NEW_URDF)
        self.assertEqual(len(origins), 4)
        self.assertTrue(_allocation_matches(r["allocation"], origins))
        self.assertEqual(r["allocation"], r["base_allocation"])                     # same geometry as navrl_quad
        # positive control: the comparator rejects a wrong arm
        wrong = [row[:] for row in r["allocation"]]
        wrong[3] = [v * 1.1 for v in wrong[3]]
        self.assertFalse(_allocation_matches(wrong, origins))

    def test_max_thrust_gives_thrust_to_weight_2p25(self):
        r = cached("config")
        self.assertEqual(r["nominal_mass"], 0.50)
        self.assertAlmostEqual(r["max_thrust"], 2.25 * 0.5 * G / 4.0, places=12)
        self.assertAlmostEqual(r["max_thrust"], 2.7590625, places=12)
        mass = sum(float(link.find("inertial/mass").get("value")) for link in _links(NEW_URDF).values())
        self.assertAlmostEqual(4.0 * r["max_thrust"] / (mass * G), 2.25, places=9)
        self.assertEqual(r["min_thrust"], 0)
        self.assertEqual(r["tau"], [0.04, 0.04, 0.04, 0.04])
        self.assertEqual(r["ttr"], 0.01)
        self.assertEqual(r["k"][0], r["k"][1])
        self.assertEqual(r["declared_ratio"], [2.0, 2.0, 2.0])
        self.assertIn("PLACEHOLDER GEOMETRY", r["docstring"])
        # positive control: F's body has T/W 3.26, so the 2.25 check discriminates
        self.assertAlmostEqual(4.0 * r["base_max_thrust"] / (0.25 * G), 3.262, places=3)


# ---------------------------------------------------------------------------------------------------------------------
# 2. Vehicle randomization
OWN_TW = 4 * 2.0 / (0.25 * 9.81)                            # navrl_quad: 4 x 2 N / (0.25 kg x 9.81 m/s^2) = 3.262
VR_ON_ENV = {"NAVRL_VEH_RAND": "1", "NAVRL_VEH_RAND_SEED": "4601", "NAVRL_VEH_RAND_TW_NOMINAL": "2.25",
             "NAVRL_VEH_RAND_TW_SCALE": "0.15", "NAVRL_VEH_RAND_MOTOR_IMBAL": "0.05", "NAVRL_VEH_RAND_TAU_MIN": "0.03",
             "NAVRL_VEH_RAND_TAU_MAX": "0.06"}                  # CAM-K2's values
VR_ON = VR.settings_from_env(VR_ON_ENV)
VR_ENTRY_ENV = dict(VR_ON_ENV)
VR_IDENTITY = VR.settings_from_env(dict(VR_ON_ENV, NAVRL_VEH_RAND_TW_NOMINAL=repr(OWN_TW), NAVRL_VEH_RAND_TW_SCALE="0",
                                        NAVRL_VEH_RAND_MOTOR_IMBAL="0", NAVRL_VEH_RAND_TAU_MIN="0.04",
                                        NAVRL_VEH_RAND_TAU_MAX="0.04"))
VR_CAP_ONLY = dict(VR_IDENTITY, NAVRL_VEH_RAND_TW_NOMINAL=2.25)          # the equivalence arm: cap, no randomization
CAP_N = 2.25 * 0.25 * 9.81 / 4                                 # 1.37953125 N per motor, computed as the module does


class TestVehicleRandSettings(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(VR_ON, {"NAVRL_VEH_RAND_SEED": 4601, "NAVRL_VEH_RAND_TW_NOMINAL": 2.25,
                                 "NAVRL_VEH_RAND_TW_SCALE": 0.15, "NAVRL_VEH_RAND_MOTOR_IMBAL": 0.05,
                                 "NAVRL_VEH_RAND_TAU_MIN": 0.03, "NAVRL_VEH_RAND_TAU_MAX": 0.06})
        self.assertAlmostEqual(VR.worst_case_thrust_to_weight(VR_ON), 2.25 * 0.85 * 0.95, places=12)

    def test_bad_settings_refuse(self):
        bad = [{"NAVRL_VEH_RAND": ""}, {"NAVRL_VEH_RAND": "0"}, {"NAVRL_VEH_RAND": "2"},
               {"NAVRL_VEH_RAND_SEED": ""}, {"NAVRL_VEH_RAND_SEED": "-1"}, {"NAVRL_VEH_RAND_SEED": "1.5"},
               {"NAVRL_VEH_RAND_TW_NOMINAL": ""}, {"NAVRL_VEH_RAND_TW_NOMINAL": "0.5"},
               {"NAVRL_VEH_RAND_TW_NOMINAL": "11"}, {"NAVRL_VEH_RAND_TW_NOMINAL": "nan"},
               {"NAVRL_VEH_RAND_TW_SCALE": ""}, {"NAVRL_VEH_RAND_TW_SCALE": "0.6"}, {"NAVRL_VEH_RAND_TW_SCALE": "-0.1"},
               {"NAVRL_VEH_RAND_TW_SCALE": "nan"}, {"NAVRL_VEH_RAND_TW_SCALE": "x"},
               {"NAVRL_VEH_RAND_MOTOR_IMBAL": "0.3"}, {"NAVRL_VEH_RAND_MOTOR_IMBAL": ""},
               {"NAVRL_VEH_RAND_TAU_MIN": "0.001"}, {"NAVRL_VEH_RAND_TAU_MAX": "0.5"},
               {"NAVRL_VEH_RAND_TAU_MIN": "0.05", "NAVRL_VEH_RAND_TAU_MAX": "0.03"}, {"NAVRL_VEH_RAND_TAU_MAX": ""}]
        for change in bad:
            with self.assertRaises(ValueError, msg=change):
                VR.settings_from_env(dict(VR_ON_ENV, **change))
        self.assertEqual(len(bad), 21)
        # positive controls at the bounds
        edge = VR.settings_from_env(dict(VR_ON_ENV, NAVRL_VEH_RAND_TW_SCALE="0.5", NAVRL_VEH_RAND_MOTOR_IMBAL="0.2",
                                         NAVRL_VEH_RAND_TAU_MIN="0.005", NAVRL_VEH_RAND_TAU_MAX="0.2",
                                         NAVRL_VEH_RAND_SEED="0", NAVRL_VEH_RAND_TW_NOMINAL="10"))
        self.assertEqual(edge["NAVRL_VEH_RAND_TW_SCALE"], 0.5)


class TestVehicleRandDynamics(unittest.TestCase):
    """Real BaseMultirotor.reset_idx, ControlAllocator and MotorModel with navrl_quad's config (CAM-K2's vehicle), in a
    child; the multirotor's init_tensors is a stand-in that builds the controller's mass / gravity and the allocator."""

    def test_production_install_wraps_the_three_methods(self):
        self.assertEqual(child("vehrand_default", VR_ON)["wrapped"], [True, True, True])

    def test_identity_settings_are_bitwise_off(self):
        off, ident = cached("vehrand", {"settings": None}), cached("vehrand", {"settings": VR_IDENTITY})
        self.assertFalse(ident["record"]["cap_applied"])
        self.assertIsNone(ident["cap"])
        self.assertEqual(ident["thrust"], off["thrust"])
        self.assertEqual(ident["yaw"], off["yaw"])
        self.assertEqual(ident["tau_inc"], off["tau_inc"])
        self.assertEqual(ident["state_after_reset"], off["state_after_reset"])
        self.assertEqual(ident["rng"], off["rng"])
        self.assertAlmostEqual(off["max_thrust"], 2.0, places=6)
        # non-vacuous: the sequence exceeds the 1.38 N cap often and varies the thrust
        self.assertGreater(off["ref_over_cap"], 0)
        flat = [v for step in off["thrust"] for env in step for v in env]
        self.assertGreater(max(flat), 1.0 * CAP_N)
        self.assertGreater(len(set(round(v, 6) for v in flat)), 100)

    def test_cap_is_computed_from_the_controller_mass_and_gravity(self):
        on = cached("vehrand", {"settings": VR_ON})
        rec = on["record"]
        self.assertTrue(rec["cap_applied"])
        self.assertEqual(rec["mass_kg"], [0.25, 0.25])
        self.assertAlmostEqual(rec["gravity_mps2"][0], 9.81, places=5)
        self.assertAlmostEqual(rec["cap_per_motor_n"][0], CAP_N, places=6)
        self.assertAlmostEqual(rec["cap_per_motor_n"][0], 1.37953125, places=6)
        self.assertAlmostEqual(rec["own_thrust_to_weight"][0], OWN_TW, places=5)
        for row in on["cap"]:
            for c in row:
                self.assertAlmostEqual(c, CAP_N, places=6)
        # the cap follows the controller's mass: 0.30 kg gives a proportionally larger cap (positive control)
        heavier = child("vehrand", {"settings": VR_ON, "mass": 0.30})["record"]
        self.assertAlmostEqual(heavier["cap_per_motor_n"][0], 2.25 * 0.30 * 9.81 / 4, places=6)

    def test_partial_reset_leaves_the_other_envs_motor_state(self):
        on = cached("vehrand", {"settings": VR_ON})
        before, after = on["state_before_partial_reset"], on["state_after_partial_reset"]
        for e in range(6):
            if e in (1, 4):
                self.assertNotEqual(after[e], before[e], e)                          # redrawn and rescaled
                self.assertLessEqual(max(after[e]), CAP_N + 1e-6)
            else:
                self.assertEqual(after[e], before[e], e)                             # untouched

    def test_telemetry_lines(self):
        on, off = cached("vehrand", {"settings": VR_ON}), cached("vehrand", {"settings": None})
        self.assertEqual(telemetry(off, VR.LOG_PREFIX), [])                           # OFF logs nothing
        lines = telemetry(on, VR.LOG_PREFIX)
        events = [x["event"] for x in lines]
        self.assertEqual(events.count("install"), 1)
        self.assertEqual(events.count("init"), 1)
        resets = [x for x in lines if x["event"] == "reset"]
        self.assertEqual(len(resets), 2)                     # the run resets twice, both within RESET_LOG_FIRST
        init = [x for x in lines if x["event"] == "init"][0]
        self.assertTrue(init["cap_applied"])
        self.assertAlmostEqual(init["cap_per_motor_n"][0], CAP_N, places=6)
        self.assertLess(init["factor_min"], init["factor_max"])
        self.assertEqual([x["envs"] for x in resets][:2], [6, 2])                    # the full and the partial reset
        self.assertEqual([x["motor_updates"] for x in resets], [0, 15])               # N-7: 15 steps before reset 2
        updates = [x for x in lines if x["event"] == "update"]
        self.assertEqual([x["motor_updates"] for x in updates], [1])                   # the first motor update
        self.assertTrue(updates[0]["cap_applied"])
        for x in resets:
            self.assertGreaterEqual(x["factor_min"], 0.85 * 0.95 - 1e-6)
            self.assertLessEqual(x["factor_max"], 1.15 * 1.05 + 1e-6)
            self.assertGreaterEqual(x["tau_min"], 0.03 - 1e-7)
            self.assertLessEqual(x["tau_max"], 0.06 + 1e-7)

    def test_on_caps_scales_and_redraws_only_reset_envs(self):
        off, on = cached("vehrand", {"settings": None}), cached("vehrand", {"settings": VR_ON})
        self.assertEqual(on["second_install"], "refused")
        self.assertNotEqual(on["thrust"], off["thrust"])
        self.assertEqual(on["rng"], off["rng"])                                       # global RNG untouched
        self.assertLess(on["ratio_err"], 1e-6)                                       # delivered = state * factor
        lo, hi = on["install"]["factor_bounds"]
        self.assertAlmostEqual(lo, 0.85 * 0.95, places=12)
        self.assertAlmostEqual(hi, 1.15 * 1.05, places=12)
        self.assertAlmostEqual(on["install"]["thrust_to_weight_range"][0], 2.25 * 0.85 * 0.95, places=12)
        flat = [v for step in on["thrust"] for env in step for v in env]
        self.assertLessEqual(max(flat), CAP_N * hi + 1e-6)                           # cap x factor bounds delivery
        self.assertGreater(max(flat), CAP_N * lo)                                    # and is reached
        for state in (on["state_after_reset"], on["state_after_partial_reset"]):     # reset draws in [0, cap]
            self.assertLessEqual(max(v for row in state for v in row), CAP_N + 1e-6)
        before, after = on["factors"]["before"], on["factors"]["after"]
        changed = 0
        for e in range(6):
            for m in range(4):
                for f in (before[e][m], after[e][m]):
                    self.assertGreaterEqual(f, lo - 1e-6)
                    self.assertLessEqual(f, hi + 1e-6)
                if e in (1, 4):
                    changed += int(after[e][m] != before[e][m])
                else:
                    self.assertEqual(after[e][m], before[e][m], (e, m))              # not reset: kept
        self.assertEqual(changed, 8)
        self.assertGreater(sum(1 for row in before for f in row if f != 1.0), 20)
        tau_changed = 0
        for e in range(6):
            self.assertEqual(len(set(on["tau_inc"][e])), 1)                          # one tau per env
            self.assertEqual(on["tau_inc"][e], on["tau_dec"][e])
            for tv in on["tau_inc"][e]:
                self.assertGreaterEqual(tv, 0.03 - 1e-7)
                self.assertLessEqual(tv, 0.06 + 1e-7)
                tau_changed += int(abs(tv - 0.04) > 1e-6)
        self.assertGreater(tau_changed, 0)
        self.assertEqual(set(v for env in off["tau_inc"] for v in env), {off["tau_inc"][0][0]})   # control: nominal

    def test_refusals_at_run_time(self):
        too_high = child("vehrand", {"settings": dict(VR_ON, NAVRL_VEH_RAND_TW_NOMINAL=3.5)})
        self.assertIn("exceeds the robot's own thrust-to-weight", too_high["init_error"])
        just_below = child("vehrand", {"settings": dict(VR_ON, NAVRL_VEH_RAND_TW_NOMINAL=3.26)})   # control
        self.assertNotIn("init_error", just_below)
        self.assertTrue(just_below["record"]["cap_applied"])
        self.assertEqual(cached("vehrand", {"settings": VR_ON})["update_before_init"], "refused")
        self.assertTrue(cached("vehrand", {"settings": None})["update_before_init"].startswith("original:"))

    def test_record_file_written_once(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "rec.json")
            r = child("vehrand", {"settings": VR_ON, "record": path})
            self.assertEqual(json.loads(pathlib.Path(path).read_text()), r["record"])
            again = child("vehrand", {"settings": VR_ON, "record": path})             # mode "x": refuses
            self.assertIn("init_error", again)
            self.assertEqual(json.loads(pathlib.Path(path).read_text()), r["record"])

    def test_same_seed_repeats_other_seed_differs(self):
        a, b = cached("vehrand", {"settings": VR_ON}), child("vehrand", {"settings": VR_ON})
        self.assertEqual(a["thrust"], b["thrust"])
        c = child("vehrand", {"settings": dict(VR_ON, NAVRL_VEH_RAND_SEED=4602)})
        self.assertNotEqual(a["thrust"], c["thrust"])


class TestOptionAEquivalence(unittest.TestCase):
    """navrl_quad capped at T/W 2.25 (CAM-K2) against the 0.50 kg Option A files: same accelerations."""

    @classmethod
    def setUpClass(cls):
        cls.r = cached("equivalence", VR_CAP_ONLY)
        cls.j_quad, cls.j_rv05 = _independent_inertia(OLD_URDF), _independent_inertia(NEW_URDF)

    def test_parameters(self):
        r = self.r
        self.assertTrue(r["alloc_equal"])
        self.assertEqual(r["cq"][0], r["cq"][1])
        self.assertEqual(r["tau"][0], r["tau"][1])
        self.assertAlmostEqual(r["cap"], CAP_N, places=6)
        self.assertAlmostEqual(r["max_b"], 2.7590625, places=6)
        self.assertAlmostEqual(r["max_b"] / r["cap"], 2.0, places=6)
        # hover fraction and maximum thrust acceleration, from the computed cap and the Option A config
        self.assertAlmostEqual(0.25 * G / (4 * r["cap"]), 0.5 * G / (4 * r["max_b"]), places=6)
        self.assertAlmostEqual(0.25 * G / (4 * r["cap"]), 1 / 2.25, places=6)
        self.assertAlmostEqual(4 * r["cap"] / 0.25, 4 * r["max_b"] / 0.5, places=4)
        self.assertAlmostEqual(4 * r["cap"] / 0.25, 22.0725, places=3)
        # maximum roll and yaw angular accelerations (two motors at the cap, two off)
        roll_a = 2 * 0.13 * r["cap"] / self.j_quad[0]
        roll_b = 2 * 0.13 * r["max_b"] / self.j_rv05[0]
        self.assertAlmostEqual(roll_a / roll_b, 1.0, places=6)
        yaw_a, yaw_b = 2 * r["cq"][0] * r["cap"] / self.j_quad[2], 2 * r["cq"][1] * r["max_b"] / self.j_rv05[2]
        self.assertAlmostEqual(yaw_a / yaw_b, 1.0, places=6)
        # control: uncapped navrl_quad is not equivalent (T/W 3.26)
        self.assertGreater(abs(4 * 2.0 / 0.25 - 4 * r["max_b"] / 0.5), 5.0)            # 32.0 vs 22.07 m/s^2

    def test_trajectories_scale_by_the_mass_ratio(self):
        r = self.r
        sa = [v for row in r["state0"]["a"] for v in row]
        sb = [v for row in r["state0"]["b"] for v in row]
        self.assertTrue(all(abs(b - 2 * a) <= 1e-6 * max(1.0, abs(b)) for a, b in zip(sa, sb)))   # reset draws
        worst_f = worst_t = peak = 0.0
        for fa, fb, ta, tb in zip(r["fa"], r["fb"], r["ta"], r["tb"]):
            for ea, eb, qa, qb in zip(fa, fb, ta, tb):
                for x, y, p, q in zip(ea, eb, qa, qb):
                    worst_f = max(worst_f, abs(y - 2 * x))
                    worst_t = max(worst_t, abs(q - 2 * p))
                    peak = max(peak, x)
        self.assertLess(worst_f, 1e-5)                                                # N, delivered force
        self.assertLess(worst_t, 1e-7)                                                # N m, yaw torque
        self.assertGreater(peak, 0.9 * r["cap"])                                     # driven close to the cap
        # accelerations: forces / mass and the derived roll torque / inertia agree
        for fa, fb in zip(r["fa"], r["fb"]):
            for ea, eb in zip(fa, fb):
                self.assertAlmostEqual(sum(ea) / 0.25, sum(eb) / 0.5, places=4)
                roll_a = 0.13 * (-ea[0] - ea[1] + ea[2] + ea[3]) / self.j_quad[0]
                roll_b = 0.13 * (-eb[0] - eb[1] + eb[2] + eb[3]) / self.j_rv05[0]
                self.assertAlmostEqual(roll_a, roll_b, delta=1e-2)
        sat_a = [v for row in r["sat_a"] for v in row]
        sat_b = [v for row in r["sat_b"] for v in row]
        self.assertAlmostEqual(max(sat_a), r["cap"], places=5)                         # steady state at the cap
        self.assertAlmostEqual(sum(sat_a[:4]) / 0.25, sum(sat_b[:4]) / 0.5, places=4)


# ---------------------------------------------------------------------------------------------------------------------
# 3. OAK-D Pro depth model
DM_ON_ENV = {"NAVRL_DEPTH_MODEL": "oakd", "NAVRL_DEPTH_MINZ_M": "0.4", "NAVRL_DEPTH_MAXZ_M": "20",
             "NAVRL_DEPTH_SIGMA_D_PX": "0.15", "NAVRL_DEPTH_FB_PXM": "57.2", "NAVRL_DEPTH_HOLE_P": "0.01",
             "NAVRL_DEPTH_SEED": "4701"}


class TestDepthModelSettings(unittest.TestCase):
    def test_valid(self):
        s = DM.settings_from_env(DM_ON_ENV)
        self.assertEqual((s["NAVRL_DEPTH_MINZ_M"], s["NAVRL_DEPTH_SIGMA_D_PX"], s["NAVRL_DEPTH_FB_PXM"],
                          s["NAVRL_DEPTH_SEED"]), (0.4, 0.15, 57.2, 4701))

    def test_bad_settings_refuse(self):
        bad = [{"NAVRL_DEPTH_MODEL": ""}, {"NAVRL_DEPTH_MODEL": "stereo"}, {"NAVRL_DEPTH_MINZ_M": ""},
               {"NAVRL_DEPTH_MINZ_M": "-0.1"}, {"NAVRL_DEPTH_MINZ_M": "3"}, {"NAVRL_DEPTH_MAXZ_M": "0.3"},
               {"NAVRL_DEPTH_MAXZ_M": "50"}, {"NAVRL_DEPTH_SIGMA_D_PX": "-1"}, {"NAVRL_DEPTH_SIGMA_D_PX": "nan"},
               {"NAVRL_DEPTH_FB_PXM": "0"}, {"NAVRL_DEPTH_FB_PXM": ""}, {"NAVRL_DEPTH_HOLE_P": "0.6"},
               {"NAVRL_DEPTH_HOLE_P": "x"}, {"NAVRL_DEPTH_SEED": "-2"}, {"NAVRL_DEPTH_SEED": ""}]
        for change in bad:
            with self.assertRaises(ValueError, msg=change):
                DM.settings_from_env(dict(DM_ON_ENV, **change))
        zero = DM.settings_from_env(dict(DM_ON_ENV, NAVRL_DEPTH_MINZ_M="0", NAVRL_DEPTH_SIGMA_D_PX="0",
                                         NAVRL_DEPTH_HOLE_P="0", NAVRL_DEPTH_SEED="0"))      # positive control
        self.assertEqual(zero["NAVRL_DEPTH_HOLE_P"], 0.0)

    def test_entry_lists_the_same_parameters(self):
        self.assertEqual(sorted(E.DEPTH_PARAMETERS), sorted(DM.REQUIRED))


class TestDepthModel(unittest.TestCase):
    """The real NavRLPerceptionModule (path-loaded copies) and LiDAR ablation, in a child."""

    def test_off_and_zero_parameters_are_identical_on_is_not(self):
        r = cached("depth_identity")
        lines = [json.loads(ln[len(DM.LOG_PREFIX):]) for ln in r["_stderr"].splitlines() if ln.startswith(DM.LOG_PREFIX)]
        self.assertEqual([x["event"] for x in lines].count("install"), 2)              # zero and ON copies
        firsts = [x for x in lines if x["event"] == "first"]
        self.assertEqual(len(firsts), 3)                     # one per wrapped perception instance (zero, ON, ON)
        self.assertTrue(any(x["invalid_hole"] > 0 and x["mean_abs_noise_m"] > 0 for x in firsts))   # ON
        self.assertTrue(any(x["invalid_hole"] == 0 and x["mean_abs_noise_m"] == 0 for x in firsts))  # zero: inert
        self.assertTrue(r["zero_equals_off"])
        self.assertTrue(r["on_differs"])                                             # positive control
        self.assertEqual(r["second_install"], "refused")

    def test_signature_preserved_and_global_rng_untouched(self):
        r = cached("depth_identity")
        self.assertTrue(r["signature_kept"])
        self.assertTrue(r["has_wrapped"])
        self.assertTrue(r["name_kept"])
        self.assertTrue(r["unwrapped_signature_differs"])                            # control: wraps is what keeps it
        self.assertTrue(r["observe_rng_equal"])
        self.assertTrue(r["rng_probe_detects_a_draw"])                              # the probe is not blind
        self.assertTrue(cached("depth_stats")["rng_untouched"])

    def test_noise_grows_with_z_squared(self):
        r = cached("depth_stats")
        self.assertLess(r["cos_min"], 0.8)                                           # off-axis pixels exist
        self.assertAlmostEqual(r["cos_max"], 1.0, places=3)
        for z0, (got, want) in r["sigma"].items():
            self.assertGreater(got, 0.0, z0)
            self.assertLess(abs(got / want - 1.0), 0.05, (z0, got, want))
        self.assertLess(abs(r["sigma"]["4"][0] / r["sigma"]["2"][0] - 4.0), 0.2)
        self.assertLess(abs(r["sigma"]["6"][0] / r["sigma"]["1"][0] - 36.0), 1.8)
        self.assertEqual(r["invalid_counts_noise"], [0, 0, 0, 0])                    # noise alone marks nothing
        self.assertTrue(r["same_seed_equal"])
        self.assertTrue(r["other_seed_differs"])

    def test_minz_maxz_and_holes_make_no_return_pixels(self):
        r = cached("depth_stats")
        total = r["total"]
        self.assertEqual(r["minz_below"]["invalid"], total)
        self.assertEqual(r["minz_below"]["invalid_values"], [20.0])                  # perception.max_camera_range
        self.assertEqual(r["minz_above"]["invalid"], 0)                              # positive control
        self.assertTrue(r["minz_above"]["valid_unchanged"])
        self.assertGreater(r["maxz_beyond"]["invalid"], 0)
        self.assertEqual(r["maxz_beyond"]["invalid"], total - r["maxz_beyond"]["no_hit"])
        self.assertEqual(r["maxz_beyond"]["invalid_values"], [20.0])
        self.assertEqual(r["maxz_within"]["invalid"], 0)
        frac = r["holes"]["invalid"] / total
        self.assertGreater(r["holes"]["invalid"], 0)
        self.assertLess(abs(frac - 0.1), 0.015, frac)
        self.assertEqual(r["holes"]["invalid_values"], [20.0])
        self.assertTrue(r["holes"]["valid_unchanged"])
        self.assertEqual(r["no_holes"]["invalid"], 0)
        self.assertTrue(r["no_hit_passthrough"])
        self.assertTrue(r["no_hit_control_changed"])
        self.assertEqual(r["maxz_too_far"], "refused")
        self.assertEqual(r["maxz_far_plane"], "accepted")

    def test_composes_with_the_ablation_installed_after_it(self):
        r = cached("depth_compose")
        self.assertEqual(r["ablation_first"], "refused")
        self.assertTrue(r["ablation_bound_lidar_m"])                                 # bound through the wrapper
        self.assertTrue(r["no_lidar_still_removes_lidar"])
        # a bar at 0.3 m (< MinZ 0.4) becomes no return in the fused scan; at 0.5 m it is seen; without the model
        # the 0.3 m bar is seen (so the model, not the scene, removes it)
        self.assertEqual(r["bar_0p3_model"], [12.0, 12.0])
        self.assertAlmostEqual(r["bar_0p5_model"][0], 0.5, places=5)
        self.assertAlmostEqual(r["bar_0p3_no_model"][0], 0.3, places=5)
        self.assertTrue(r["noise_reaches_fused"])
        self.assertTrue(r["zero_fused_equals_unmodelled"])


# ---------------------------------------------------------------------------------------------------------------------
# 4. Entry point
ENTRY_OK = {"NAVRL_REALVEH_RECEIPT": "/nonexistent/realveh_receipt.json",
            "NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_CAMTRAIN_HFOV_DEG": "66", "NAVRL_CAMTRAIN_VFOV_DEG": "54",
            "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": "0.6", "NAVRL_CAMTRAIN_VISIBILITY_BONUS": "0.03"}


class TestEntrySettings(unittest.TestCase):
    def test_off_parses_without_loading_either_module(self):
        with mock.patch.object(E, "_load_private", side_effect=AssertionError("module loaded while OFF")):
            for extra in ({}, {"NAVRL_VEH_RAND": "0"}, {"NAVRL_ROBOT": "navrl_quad"}, {"NAVRL_REALVEH_ROBOT": ""},
                          {"NAVRL_CAMTRAIN_RECEIPT": ENTRY_OK["NAVRL_REALVEH_RECEIPT"]},
                          {"NAVRL_DEPTH_NOISE_MODEL": "stereo"}):
                s = E.settings(dict(ENTRY_OK, **extra))
                self.assertIsNone(s["vehicle_rand"], extra)
                self.assertIsNone(s["depth_model"], extra)
                self.assertEqual(s["camtrain"]["NAVRL_CAMTRAIN_HFOV_DEG"], 66.0)
                self.assertNotIn("robot", s)

    def test_on_parses_both_modules(self):
        calls = []
        real = E._load_private

        def spy(name, path):
            calls.append(pathlib.Path(path).name)
            return real(name, path)

        with mock.patch.object(E, "_load_private", side_effect=spy):
            s = E.settings(dict(ENTRY_OK, **VR_ENTRY_ENV, **DM_ON_ENV))
        self.assertEqual(sorted(calls), ["navrl_depth_model.py", "navrl_vehicle_rand.py"])
        self.assertEqual(s["vehicle_rand"], VR.settings_from_env(VR_ENTRY_ENV))
        self.assertEqual(s["depth_model"]["NAVRL_DEPTH_FB_PXM"], 57.2)

    def test_missing_or_bad_settings_refuse(self):
        bad = [{"NAVRL_REALVEH_ROBOT": "navrl_rv05_quad"}, {"NAVRL_REALVEH_ROBOT": "navrl_quad"},
               {"NAVRL_REALVEH_WARMSTART_FROM": "navrl_quad"}, {"NAVRL_REALVEH_OTHER": "1"}, {"NAVRL_REALVEH_RECEIPT": ""},
               {"NAVRL_ROBOT": "navrl_rv05_quad"}, {"NAVRL_ROBOT": "navrl_ref5in_quad"},
               {"NAVRL_CAMTRAIN_RECEIPT": "/tmp/other.json"}, {"NAVRL_SENSOR_ABLATION": "full"},
               {"NAVRL_CAMTRAIN_HFOV_DEG": ""}, {"NAVRL_VEH_RAND": "2"}, {"NAVRL_DEPTH_MODEL": "oakdpro_v1"},
               {"NAVRL_VEH_RAND_SEED": "1"}, {"NAVRL_DEPTH_MINZ_M": "0.4"},
               dict(VR_ENTRY_ENV, NAVRL_VEH_RAND_TW_SCALE="0.9"), dict(VR_ENTRY_ENV, NAVRL_VEH_RAND_SEED=""),
               dict(DM_ON_ENV, NAVRL_DEPTH_HOLE_P="0.9"), dict(DM_ON_ENV, NAVRL_DEPTH_SEED="")]
        for change in bad:
            with self.assertRaises(SystemExit, msg=change):
                E.settings(dict(ENTRY_OK, **change))
        self.assertIsNotNone(E.settings(dict(ENTRY_OK, **VR_ENTRY_ENV))["vehicle_rand"])  # positive controls
        self.assertIsNotNone(E.settings(dict(ENTRY_OK, **DM_ON_ENV))["depth_model"])

    def test_settings_import_no_torch_before_isaac_gym(self):
        """main() validates before importing Isaac Gym, which must come before torch: settings() must not import
        torch, even with both modules switched on (they import torch lazily)."""
        code = ("import importlib.util, json, sys\n"
                "spec = importlib.util.spec_from_file_location('e', sys.argv[1])\n"
                "e = importlib.util.module_from_spec(spec); spec.loader.exec_module(e)\n"
                "s = e.settings(json.loads(sys.argv[2]))\n"
                "print(json.dumps({'torch': 'torch' in sys.modules, 'veh': s['vehicle_rand'] is not None,"
                " 'depth': s['depth_model'] is not None}))\n")
        env = dict(ENTRY_OK, **VR_ENTRY_ENV, **DM_ON_ENV)
        r = subprocess.run([sys.executable, "-B", "-c", code, str(ROOT / "tools/realveh_entry.py"), json.dumps(env)],
                           capture_output=True, text=True, env={"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1"},
                           timeout=120)
        self.assertEqual(r.returncode, 0, r.stderr[-2000:])
        self.assertEqual(json.loads(r.stdout.strip().splitlines()[-1]), {"torch": False, "veh": True, "depth": True})
        r = subprocess.run([sys.executable, "-B", "-c", "import sys, torch; print('torch' in sys.modules)"],
                           capture_output=True, text=True, timeout=120)
        self.assertEqual(r.stdout.strip(), "True")                                   # control: the probe sees torch

    def test_receipt_written_once_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "receipt.json")
            E.write_receipt(path, {"a": 1})
            self.assertEqual(json.loads(pathlib.Path(path).read_text()), {"a": 1})
            with self.assertRaises(SystemExit) as cm:
                E.write_receipt(path, {"a": 2})
            self.assertIn("refusing to overwrite", str(cm.exception))
            self.assertEqual(json.loads(pathlib.Path(path).read_text()), {"a": 1})

    def test_hashed_files_exist(self):
        for name, path in E.HASHED.items():
            self.assertTrue(pathlib.Path(path).is_file(), name)
            self.assertEqual(len(E.sha256(path)), 64)


class TestEntryGuards(unittest.TestCase):
    """The worst-case T/W floor, the PERTURB refusal, the vehicle-randomization record and the retired robot path."""

    def test_thrust_to_weight_floor(self):
        self.assertEqual(E.MIN_WORST_CASE_THRUST_TO_WEIGHT, 1.8)
        ok = dict(ENTRY_OK, **VR_ENTRY_ENV)
        s = E.settings(ok)                                                            # 2.25 * 0.85 * 0.95 = 1.8169
        self.assertAlmostEqual(s["worst_case_thrust_to_weight"], 1.816875, places=12)
        for nominal, tw, im in (("2.25", "0.2", "0.05"), ("2.25", "0.15", "0.1"), ("2.0", "0.15", "0.05"),
                                ("1.79", "0", "0")):                               # 1.71, 1.72, 1.62, 1.79
            with self.assertRaises(SystemExit, msg=(nominal, tw, im)) as cm:
                E.settings(dict(ok, NAVRL_VEH_RAND_TW_NOMINAL=nominal, NAVRL_VEH_RAND_TW_SCALE=tw,
                                NAVRL_VEH_RAND_MOTOR_IMBAL=im))
            self.assertIn("worst-case thrust-to-weight", str(cm.exception))
        for nominal, tw, im in (("1.8", "0", "0"), ("2.0", "0.05", "0.05"), ("3.26", "0.3", "0.1")):   # controls
            self.assertGreaterEqual(E.settings(dict(ok, NAVRL_VEH_RAND_TW_NOMINAL=nominal, NAVRL_VEH_RAND_TW_SCALE=tw,
                                                    NAVRL_VEH_RAND_MOTOR_IMBAL=im))["worst_case_thrust_to_weight"], 1.8)
        self.assertNotIn("worst_case_thrust_to_weight", E.settings(dict(ENTRY_OK)))   # OFF: no floor applies

    def test_perturb_refused_with_the_depth_model(self):
        for value in ("1", "true", "on", "1.0", "x"):
            with self.assertRaises(SystemExit, msg=value) as cm:
                E.settings(dict(ENTRY_OK, **DM_ON_ENV, NAVRL_PERCEPTION_PERTURB=value))
            self.assertIn("NAVRL_PERCEPTION_PERTURB", str(cm.exception))
        for value in ("", "0", "false", "off"):                                       # positive controls
            self.assertIsNotNone(E.settings(dict(ENTRY_OK, **DM_ON_ENV, NAVRL_PERCEPTION_PERTURB=value))["depth_model"])
        self.assertIsNone(E.settings(dict(ENTRY_OK, NAVRL_PERCEPTION_PERTURB="1"))["depth_model"])   # no depth model

    def test_existing_vehicle_rand_record_refuses(self):
        with tempfile.TemporaryDirectory() as d:
            receipt = os.path.join(d, "r.json")
            env = dict(ENTRY_OK, **VR_ENTRY_ENV, NAVRL_REALVEH_RECEIPT=receipt)
            self.assertIsNotNone(E.settings(env)["vehicle_rand"])                     # positive control
            pathlib.Path(receipt + E.VEH_RAND_RECORD_SUFFIX).write_text("{}")
            with self.assertRaises(SystemExit):
                E.settings(env)
            self.assertIsNone(E.settings(dict(ENTRY_OK, NAVRL_REALVEH_RECEIPT=receipt))["vehicle_rand"])

    def test_robot_path_and_warm_start_bypass_are_gone(self):
        src = (ROOT / "tools/realveh_entry.py").read_text()
        for gone in ("robot_registry", "scale_attitude_gains", "set_env_state", "install_warmstart_transfer",
                     "navrl_rv05_quad_config", "os.environ[\"NAVRL_ROBOT\"]"):
            self.assertNotIn(gone, src, gone)
        self.assertEqual(E.ROBOT, "navrl_quad")
        self.assertIn('if task_config.robot_name != ROBOT:', src)                     # main() checks the lineage
        self.assertEqual(E.RETIRED, ("NAVRL_REALVEH_ROBOT", "NAVRL_REALVEH_WARMSTART_FROM"))
        self.assertTrue("NavRL ROBOT LINEAGE MISMATCH" in (TASK_DIR / "navrl_task.py").read_text())   # guard stands
        self.assertIn("robot_config", E.HASHED)
        self.assertEqual(pathlib.Path(E.HASHED["robot_config"]).name, "navrl_quad_config.py")
        self.assertEqual(pathlib.Path(E.HASHED["robot_urdf"]).name, "quad_navrl_collide.urdf")


class TestEntryMain(unittest.TestCase):
    """main() refuses before importing aerial_gym; os.environ and sys.argv are patched and restored."""

    def _main(self, argv, env):
        with mock.patch.dict(os.environ, env, clear=True), mock.patch.object(sys, "argv", argv):
            with self.assertRaises(SystemExit) as cm:
                E.main()
            self.assertNotIn("NAVRL_ROBOT", os.environ)              # refused before selecting the robot
        return str(cm.exception)

    def test_only_runner_py_is_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            receipt = os.path.join(d, "r.json")
            pathlib.Path(receipt).write_text("{}")
            env = dict(ENTRY_OK, NAVRL_REALVEH_RECEIPT=receipt)
            self.assertIn("only runner.py", self._main(["realveh_entry.py", "/tmp/evil.py"], env))
            self.assertIn("only runner.py", self._main(["realveh_entry.py", "/tmp/runner.py.bak"], env))
            # positive control: runner.py passes the script check and stops at the next guard (existing receipt)
            msg = self._main(["realveh_entry.py", str(ROOT / "aerial_gym/rl_training/rl_games/runner.py")], env)
            self.assertIn("refusing to overwrite", msg)
            self.assertNotIn("only runner.py", msg)

    def test_main_refuses_missing_settings(self):
        self.assertIn("missing", self._main(["realveh_entry.py", "runner.py"], {}))
        self.assertIn("usage", self._main(["realveh_entry.py"], dict(ENTRY_OK)))


class TestShim(unittest.TestCase):
    SHIM = ROOT / "tools/realveh_python"

    def test_syntax_and_executable(self):
        self.assertEqual(subprocess.run(["bash", "-n", str(self.SHIM)]).returncode, 0)
        self.assertTrue(os.access(str(self.SHIM), os.X_OK))

    def test_requires_real_python_and_runs_the_entry(self):
        r = subprocess.run(["bash", str(self.SHIM), "runner.py"], capture_output=True, text=True,
                           env={"PATH": "/usr/bin:/bin"})
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("REALVEH_REAL_PYTHON", r.stderr)
        r = subprocess.run(["bash", str(self.SHIM), "runner.py", "--x"], capture_output=True, text=True,
                           env={"PATH": "/usr/bin:/bin", "REALVEH_REAL_PYTHON": "/bin/echo"})
        self.assertEqual(r.returncode, 0, r.stderr)                                   # positive control
        self.assertEqual(r.stdout.strip(), "%s runner.py --x" % (ROOT / "tools/realveh_entry.py"))


class TestIsolation(unittest.TestCase):
    """No production module imports the opt-in modules (statement or importlib form); only the entry does."""

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

    def test_only_the_entry_imports_the_new_modules(self):
        self.assertEqual(self._importers("navrl_vehicle_rand"), ["tools/realveh_entry.py"])
        self.assertEqual(self._importers("navrl_depth_model"), ["tools/realveh_entry.py"])

    def test_ablation_importers_are_the_three_entries(self):
        hits = self._importers("navrl_sensor_ablation")
        self.assertIn("tools/realveh_entry.py", hits)                               # positive control: pattern sees it
        self.assertLessEqual(set(hits), {"tools/realveh_entry.py", "tools/camonly_entry.py",
                                         "tools/camtrain_entry.py"}, hits)

    def test_option_a_files_are_documentation_only(self):
        """No production file registers or imports the Option A vehicle; only its own config names its URDF."""
        self.assertNotIn("navrl_rv05", (ROOT / "aerial_gym/robots/__init__.py").read_text())
        hits = []
        for base in ("aerial_gym", "motar_eval", "tools"):
            for path in (ROOT / base).rglob("*.py"):
                if path.name != "navrl_rv05_quad_config.py" and "navrl_rv05_quad_config" in path.read_text(errors="ignore"):
                    hits.append(str(path.relative_to(ROOT)))
        self.assertEqual(hits, [])


if __name__ == "__main__":
    unittest.main()
