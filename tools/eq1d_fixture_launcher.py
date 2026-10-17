#!/usr/bin/env python3
"""EQ-1D fixture launcher: one Isaac Gym process per D1 / D2 / D3 process of the preregistration.

    python -B tools/eq1d_fixture_launcher.py d1 --process d1_dt10 --out DIR
    python -B tools/eq1d_fixture_launcher.py d2 --process d2_scenes --out DIR
    python -B tools/eq1d_fixture_launcher.py d3 --process d3_n4 --out DIR

Started by ``tools/eq1d_run.py`` with the receipt environment, ``PYTHONPATH`` pinned to this tree and
a fresh ``DIR``. Diagnostic only: the simulator, sensors, controller, task and Policy F are used as
built. The launcher drives them, reads tensors and registers the env manager's documented
physics-step callback. Its runtime wrappers (motor-reference tap, render/LiDAR caches, branch
replay) call the original functions unchanged and are listed in the preregistration. Exit codes: 0
done, 3 integrity (NaN, identity mismatch), 2 usage.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import isaacgym  # noqa: E402,F401  (must precede torch)
import numpy as np  # noqa: E402
import torch  # noqa: E402

from motar_eval import eq1d_telemetry as tel  # noqa: E402

PLAN = tel.load_plan()
INTEGRITY = 3
HOVER_MASS_KG = 0.25
G = 9.81
CAMERA_T = (0.10, 0.0, 0.03)


def log(msg):
    print("[eq1d] %s" % msg, flush=True)


def process_spec(pid):
    for p in PLAN["run"]["processes"]:
        if p["id"] == pid:
            return p
    raise SystemExit("unknown process id %s" % pid)


def check_origins():
    import aerial_gym
    import motar_eval

    for mod in (aerial_gym, motar_eval):
        path = Path(mod.__file__).resolve()
        if ROOT not in path.parents:
            raise SystemExit("integrity: %s imported from %s, outside %s" % (mod.__name__, path, ROOT))
    return {"aerial_gym": str(Path(aerial_gym.__file__).resolve()), "motar_eval": str(Path(motar_eval.__file__).resolve())}


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def build_task(num_envs, seed):
    from aerial_gym.registry.task_registry import task_registry

    task = task_registry.make_task("navrl_task", seed=seed, num_envs=num_envs, headless=True, use_warp=True)
    return task


def bar_slots(task):
    start = int(task._bar_offset)
    n = int(task.obs_dict["obstacle_position"].shape[1])
    return list(range(start, n))


def park_all(task, slots, env_ids=None):
    pos = task.obs_dict["obstacle_position"]
    quat = task.obs_dict["obstacle_orientation"]
    for k, s in enumerate(slots):
        rows = slice(None) if env_ids is None else env_ids
        pos[rows, s, 0] = 1000.0 + 3.0 * k
        pos[rows, s, 1] = 1000.0
        pos[rows, s, 2] = 1.5
        quat[rows, s, :] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=pos.device)


def set_robot(task, positions, quats):
    d = task.obs_dict
    d["robot_position"][:] = positions
    d["robot_orientation"][:] = quats
    d["robot_linvel"][:] = 0.0
    d["robot_angvel"][:] = 0.0
    task.sim_env.robot_manager.robot.update_states()


def nan_report(arrays):
    bad = [k for k, v in arrays.items() if np.issubdtype(np.asarray(v).dtype, np.floating) and not np.isfinite(v).all()]
    return bad


# ================================================================== D1
def run_d1(spec, out):
    from aerial_gym.config.sim_config.base_sim_config import BaseSimConfig
    from aerial_gym.registry.env_registry import env_config_registry
    from aerial_gym.registry.sim_registry import sim_config_registry

    dt, n_sub = float(spec["dt_s"]), int(spec["substeps_per_action"])
    sim_name = os.environ.get("AERIAL_GYM_SIM_NAME", "")
    if sim_name != "eq1d_" + spec["id"]:
        raise SystemExit("integrity: AERIAL_GYM_SIM_NAME %r does not name this process" % sim_name)

    class EqSim(BaseSimConfig):
        class sim(BaseSimConfig.sim):
            pass

    EqSim.sim.dt = dt
    sim_config_registry.register(sim_name, EqSim)
    env_cfg = env_config_registry.get_env_config("navrl_bars_env")
    env_cfg.env.num_physics_steps_per_env_step_mean = n_sub
    env_cfg.env.num_physics_steps_per_env_step_std = 0

    d1 = PLAN["D1"]
    fixtures = d1["fixtures"]
    n = len(fixtures)
    if int(spec["num_envs"]) != n:
        raise SystemExit("integrity: D1 needs one env per fixture")
    task = build_task(n, seed=4401)
    origins = check_origins()
    dev = task.device
    sim_dt = float(task.obs_dict["dt"])
    if abs(sim_dt - dt) > 1e-12:
        raise SystemExit("integrity: simulator dt %.6g != %.6g" % (sim_dt, dt))
    if task.sim_env.physics_step_callback is not None:
        raise SystemExit("integrity: a physics-step callback is already attached")

    robot = task.sim_env.robot_manager.robot
    motor = robot.control_allocator.motor_model
    tap = tel.MotorTap(motor)
    slots = bar_slots(task)
    half = task.obs_dict["asset_collision_half_extents"]

    # Initial state: hover at rest; contact fixtures get one bar in their path.
    starts = torch.tensor([d1["initial_state"]["position_env_m"]] * n, dtype=torch.float32, device=dev)
    park_all(task, slots)
    bars = {}
    for i, f in enumerate(fixtures):
        if "contact" not in f:
            continue
        c = f["contact"]
        starts[i] = torch.tensor(d1["contact_start_m"], device=dev)
        s = slots[0]
        hx, hy = float(half[i, s, 0]), float(half[i, s, 1])
        h = math.radians(c["heading_deg"])
        direction = (math.cos(h), math.sin(h))
        x0, y0 = float(starts[i, 0]), float(starts[i, 1])
        # Bar centre on the travel line, its near face gap_m ahead of the drone's front face (heading 0);
        # for oblique headings the same along-track distance. "corner": the travel line passes exactly
        # through the bar's near corner (corner = start + (0.14 + gap) * dir, bar extending away from it).
        if c["lateral_m"] == "corner":
            along = 0.14 + c["gap_m"]
            corner = (x0 + along * direction[0], y0 + along * direction[1])
            cx = corner[0] + math.copysign(hx, direction[0])
            cy = corner[1] + math.copysign(hy, direction[1])
        else:
            along = 0.14 + c["gap_m"] + hx
            cx, cy = x0 + along * direction[0], y0 + along * direction[1]
        task.obs_dict["obstacle_position"][i, s, 0] = cx
        task.obs_dict["obstacle_position"][i, s, 1] = cy
        task.obs_dict["obstacle_position"][i, s, 2] = 1.5
        bars[i] = {"slot": s, "centre": [cx, cy, 1.5], "half": [hx, hy, float(half[i, s, 2])]}
    quats = torch.tensor([[0.0, 0.0, 0.0, 1.0]] * n, device=dev)
    set_robot(task, starts, quats)
    task.sim_env.IGE_env.write_to_sim()
    motor.current_motor_thrust[:] = HOVER_MASS_KG * G / 4.0

    total_actions = int(round(d1["duration_s"] / d1["action_period_s"]))
    rec = tel.SubstepRecorder(n, total_actions * n_sub, dev, {
        "command": lambda: robot.action_tensor,
        "motor_ref_thrust": lambda: tap.last_ref,
        "motor_thrust": lambda: motor.current_motor_thrust,
        "robot_force": lambda: robot.robot_force_tensors.sum(dim=1),
        "robot_torque": lambda: robot.robot_torque_tensors.sum(dim=1),
        "position": lambda: task.obs_dict["robot_position"],
        "orientation_xyzw": lambda: task.obs_dict["robot_orientation"],
        "linvel_w": lambda: task.obs_dict["robot_linvel"],
        "angvel_w": lambda: task.obs_dict["robot_angvel"],
        "contact_force": lambda: task.obs_dict["robot_contact_force_tensor"],
    })
    task.sim_env.set_physics_step_callback(rec)
    initial = {k: task.obs_dict[k].detach().cpu().numpy().copy() for k in
               ("robot_position", "robot_orientation", "robot_linvel", "robot_angvel")}
    initial["motor_thrust"] = motor.current_motor_thrust.detach().cpu().numpy().copy()
    t0 = time.time()
    for a in range(total_actions):
        rec.current_action = a
        cmd = tel.d1_commands(fixtures, a * d1["action_period_s"], device=dev)
        task.sim_env.step(actions=cmd)
    torch.cuda.synchronize()
    wall = time.time() - t0
    task.sim_env.set_physics_step_callback(None)
    tap.detach()
    arrays = rec.arrays()
    arrays.update({"initial_" + k: v for k, v in initial.items()})
    bad = nan_report(arrays)
    files = [tel.write_npz(out / "d1_substeps.npz", arrays)]
    meta = {"process": spec, "dt_s": sim_dt, "substeps_per_action": n_sub, "actions": total_actions,
            "substeps_logged": int(rec.k), "fixtures": [f["name"] for f in fixtures], "contact_bars": bars,
            "motor_time_constants": [float(motor.motor_time_constants_increasing.min()),
                                     float(motor.motor_time_constants_increasing.max()),
                                     float(motor.motor_time_constants_decreasing.min()),
                                     float(motor.motor_time_constants_decreasing.max())],
            "motor_thrust_constant": motor.motor_thrust_constant[:, 0].detach().cpu().tolist()
            if hasattr(motor, "motor_thrust_constant") else None,
            "max_thrust": float(motor.max_thrust.max()), "mixing": motor.mixing_factor_function.__name__
            if hasattr(motor.mixing_factor_function, "__name__") else str(motor.mixing_factor_function),
            "integration_scheme": motor.integration_scheme, "use_rps": bool(motor.cfg.use_rps),
            "controller": type(robot.controller).__name__, "loop_wall_s": wall, "nan_fields": bad,
            "origins": origins}
    files.append(tel.write_json(out / "d1_meta.json", meta))
    return files, bad


# ================================================================== D2
def scene_target(spec_target):
    if spec_target == "absent":
        return list(PLAN["D2"]["absent_target_rel_m"]), False
    if isinstance(spec_target, str) and spec_target.startswith("polar:"):
        _, r, deg, z = spec_target.split(":")
        r, a = float(r), math.radians(float(deg))
        return [r * math.cos(a), r * math.sin(a), float(z)], True
    return [float(v) for v in spec_target], True


def bar_centre_y(b, hy):
    if "y" in b:
        return float(b["y"])
    if "edge_left_y" in b:   # bar's right face (min y) at this y
        return float(b["edge_left_y"]) + hy
    return float(b["edge_right_y"]) - hy  # bar's left face (max y) at this y


def ray_box_t(o, d, lo, hi):
    t0, t1 = -math.inf, math.inf
    for a in range(3):
        if abs(d[a]) < 1e-15:
            if o[a] < lo[a] or o[a] > hi[a]:
                return math.inf
            continue
        ta, tb = (lo[a] - o[a]) / d[a], (hi[a] - o[a]) / d[a]
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
    return t0 if t1 >= max(t0, 0.0) and t0 > 0 else math.inf


def run_d2(spec, out):
    d2 = PLAN["D2"]
    scenes = d2["scenes"]
    n = len(scenes)
    if int(spec["num_envs"]) != n:
        raise SystemExit("integrity: D2 needs one env per scene")
    task = build_task(n, seed=4401)
    origins = check_origins()
    dev = task.device
    slots = bar_slots(task)
    half = task.obs_dict["asset_collision_half_extents"]
    drone = torch.tensor([d2["drone_env_m"]] * n, dtype=torch.float32, device=dev)
    perception, detector = task.perception, task.detector

    targets, present, layout = [], [], []
    for i, s in enumerate(scenes):
        rel, here = scene_target(s["target"])
        targets.append([float(drone[i, 0]) + rel[0], float(drone[i, 1]) + rel[1], float(drone[i, 2]) + rel[2]])
        present.append(here)
        placed = []
        for j, b in enumerate(s["bars"]):
            slot = slots[j]
            hx, hy, hz = (float(half[i, slot, k]) for k in range(3))
            cy = bar_centre_y(b, hy)
            placed.append({"slot": slot, "centre": [float(drone[i, 0]) + float(b["x"]), float(drone[i, 1]) + cy, 1.5],
                           "half": [hx, hy, hz]})
        layout.append(placed)
    target_t = torch.tensor(targets, dtype=torch.float32, device=dev)

    def place(with_bars):
        park_all(task, slots)
        if with_bars:
            for i, placed in enumerate(layout):
                for b in placed:
                    task.obs_dict["obstacle_position"][i, b["slot"]] = torch.tensor(b["centre"], device=dev)
        task.sim_env.IGE_env.write_to_sim()
        task.sim_env.warp_env.reset_idx(torch.arange(n, device=dev))

    passes = {}
    counters = []
    seg_keys = [k for k in task.obs_dict if "segmentation" in k and isinstance(task.obs_dict[k], torch.Tensor)]
    for att_name, (roll, pitch) in d2["attitudes_deg"].items():
        q = torch.tensor([quat_from_rpy(math.radians(roll), math.radians(pitch), 0.0)] * n, device=dev)
        for mode in ("scene", "alone"):
            place(mode == "scene")
            set_robot(task, drone, q)
            task.target_position[:] = target_t
            task._sync_target_to_sensor()
            q_veh = task.obs_dict["robot_vehicle_orientation"].clone()
            c0 = int(task.sim_env.step_counter)
            rgb, depth = detector.render_raw_rgbd(drone, q_veh, task.target_position, task.target_orientation)
            mask = detector.target_mask.clone()
            tdepth = detector.target_depth.clone()
            odepth = detector.obstacle_depth.clone()
            task.sim_env.render(render_components="sensors")
            lidar = task._lidar_distance_m().clone()
            seg = {k: task.obs_dict[k].detach().clone() for k in seg_keys}
            c1 = int(task.sim_env.step_counter)
            # Production perception on exactly these frames (fresh state each pass).
            perception.reset_idx(torch.arange(n, device=dev))
            det = perception._detect_rgbd(rgb, depth, False, drone_pos_w=drone, vehicle_quat=q_veh)
            perception.reset_idx(torch.arange(n, device=dev))
            obs, diag = perception.observe(rgb, depth, lidar, drone, torch.zeros_like(drone), q_veh,
                                           torch.zeros(n, device=dev), torch.zeros(n, 4, device=dev),
                                           task.task_config.max_velocity, task.task_config.flight_altitude,
                                           training=False, env_bounds_min=task.obs_dict["env_bounds_min"],
                                           env_bounds_max=task.obs_dict["env_bounds_max"])
            # Target-absent association: prime a track where a target in front of the bar would be.
            assoc = np.full(n, -1, dtype=np.int8)
            primed = np.full((n, 3), np.nan, dtype=np.float32)
            if mode == "scene":
                perception.reset_idx(torch.arange(n, device=dev))
                tr = perception.tracker
                for i, s in enumerate(scenes):
                    beam = s.get("association_beam_deg")
                    if beam is None or not layout[i]:
                        continue
                    yaw = math.radians(beam)
                    d = (math.cos(yaw), math.sin(yaw), 0.0)
                    o = [float(v) for v in drone[i]]
                    tmin = min(ray_box_t(o, d, [b["centre"][k] - b["half"][k] for k in range(3)],
                                         [b["centre"][k] + b["half"][k] for k in range(3)]) for b in layout[i])
                    if not math.isfinite(tmin):
                        continue
                    p = [o[k] + (tmin + float(perception.target_radius)) * d[k] for k in range(3)]
                    primed[i] = p
                    tr.state[i, :3] = torch.tensor(p, device=dev)
                    tr.state[i, 3:] = 0.0
                    tr.cov[i] = torch.eye(6, device=dev) * 0.01
                    tr.active[i] = True
                    tr.age[i] = 1.0
                valid, conf, measured, bearing = perception._associate_lidar_target(
                    lidar, drone, q_veh, torch.zeros(n, dtype=torch.bool, device=dev))
                assoc = np.where(np.isnan(primed[:, 0]), -1, valid.detach().cpu().numpy().astype(np.int8)).astype(np.int8)
            key = "%s_%s" % (att_name, mode)
            passes[key] = {
                "rgb": rgb.detach().cpu().numpy().astype(np.float16), "depth": depth.detach().cpu().numpy(),
                "target_mask": mask.detach().cpu().numpy().astype(np.int8), "target_depth": tdepth.detach().cpu().numpy(),
                "obstacle_depth_native": odepth.detach().cpu().numpy(), "lidar_m": lidar.detach().cpu().numpy(),
                "det_measurement_vehicle": det[0].detach().cpu().numpy(), "det_surface_range": det[1].detach().cpu().numpy(),
                "det_bearing": det[2].detach().cpu().numpy(), "det_visible": det[3].detach().cpu().numpy(),
                "det_mask": det[5].detach().cpu().numpy().astype(np.int8),
                "observation": obs.detach().cpu().numpy(), "assoc_accept": assoc, "assoc_primed_world": primed,
                "robot_orientation": task.obs_dict["robot_orientation"].detach().cpu().numpy(),
                "vehicle_orientation": q_veh.detach().cpu().numpy(),
            }
            for k, v in seg.items():
                passes[key]["seg_" + k] = v.cpu().numpy()
            counters.append({"pass": key, "capture_step_counter": c0, "delivery_step_counter": c1})
    arrays = {"%s__%s" % (p, k): v for p, d in passes.items() for k, v in d.items()}
    arrays["drone_env_m"] = drone.cpu().numpy()
    arrays["target_world"] = target_t.cpu().numpy()
    arrays["target_present"] = np.array(present)
    bad = nan_report({k: v for k, v in arrays.items() if not k.endswith("assoc_primed_world")})
    files = [tel.write_npz(out / "d2_renders.npz", arrays)]
    lidar_cfg = task.sim_env.robot_manager.robot.cfg.sensor_config.lidar_config
    meta = {"process": spec, "scenes": [s["id"] for s in scenes], "layout": layout, "counters": counters,
            "segmentation_keys": seg_keys,
            "camera": {"width": int(detector.width), "height": int(detector.height),
                       "obstacle_width": int(detector.obstacle_width), "obstacle_height": int(detector.obstacle_height),
                       "max_range": float(detector.max_range), "obstacle_max_range": float(detector.obstacle_max_range),
                       "target_radius": float(detector.target_radius),
                       "camera_offset": detector.camera_offset_vehicle.detach().cpu().tolist()},
            "lidar": {"width": int(lidar_cfg.width), "height": int(lidar_cfg.height),
                      "hfov_deg": [float(lidar_cfg.horizontal_fov_deg_min), float(lidar_cfg.horizontal_fov_deg_max)],
                      "vfov_deg": [float(lidar_cfg.vertical_fov_deg_min), float(lidar_cfg.vertical_fov_deg_max)],
                      "max_range": float(lidar_cfg.max_range), "min_range": float(lidar_cfg.min_range),
                      "target_radius": float(lidar_cfg.target_radius),
                      "yaw_only_attach": bool(getattr(lidar_cfg, "yaw_only_attach", False))},
            "perception": {"target_radius": float(perception.target_radius), "min_pixels": int(perception.min_pixels),
                           "lidar_max_range": float(perception.lidar_max_range)},
            "nan_fields": bad, "origins": origins}
    files.append(tel.write_json(out / "d2_meta.json", meta))
    return files, bad


# ================================================================== D3
def build_player(num_envs, checkpoint):
    """The rl_games player exactly as runner.py's __main__ builds it for --play."""
    import runpy
    import yaml

    rl_dir = ROOT / "aerial_gym/rl_training/rl_games"
    os.chdir(rl_dir)
    sys.argv = ["runner.py", "--file", PLAN["frozen"]["player_yaml"], "--task", PLAN["frozen"]["task"],
                "--num_envs", str(num_envs), "--headless", "True", "--play", "--checkpoint", str(checkpoint)]
    if str(rl_dir) not in sys.path:
        sys.path.insert(0, str(rl_dir))
    g = runpy.run_path(str(rl_dir / "runner.py"), run_name="eq1d_runner")
    args = g["_prepare_rlgames_argv_dict"](vars(g["get_args"]()))
    with open(args["file"], "r") as stream:
        config = g["update_config"](yaml.safe_load(stream), args)
    from rl_games.torch_runner import Runner
    from rl_games.algos_torch import model_builder

    from aerial_gym.rl_training.rl_games.early_stop_a2c_agent import EarlyStopA2CAgent
    from aerial_gym.rl_training.rl_games.navrl_network import NavRLCNNBuilder
    from aerial_gym.rl_training.rl_games.navrl_vision_network import NavRLVisionBuilder
    from aerial_gym.rl_training.rl_games.navrl_vision_legacy_network import NavRLVisionLegacyBuilder
    from aerial_gym.rl_training.rl_games.navrl_transformer_network import NavRLTransformerBuilder
    from aerial_gym.rl_training.rl_games.navrl_action_models import (
        NavRLFixedGaussianModel, NavRLSquashedGaussianModel, NavRLTruncatedGaussianModel)
    from aerial_gym.rl_training.rl_games.navrl_players import NavRLPpoPlayerContinuous

    runner = Runner()
    runner.algo_factory.register_builder("a2c_continuous", lambda **kw: EarlyStopA2CAgent(**kw))
    model_builder.register_network("navrl_cnn", NavRLCNNBuilder)
    model_builder.register_network("navrl_vision", NavRLVisionBuilder)
    model_builder.register_network("navrl_vision_legacy", NavRLVisionLegacyBuilder)
    model_builder.register_network("navrl_transformer", NavRLTransformerBuilder)
    model_builder.register_model("navrl_fixed_gaussian", NavRLFixedGaussianModel)
    model_builder.register_model("navrl_squashed_gaussian", NavRLSquashedGaussianModel)
    model_builder.register_model("navrl_truncated_gaussian", NavRLTruncatedGaussianModel)
    runner.player_factory.register_builder("a2c_continuous", lambda **kw: NavRLPpoPlayerContinuous(**kw))
    runner.load(config)
    player = runner.create_player()
    player.restore(args["checkpoint"])
    return player, args


def find_task(player):
    obj = player.env
    for _ in range(6):
        if hasattr(obj, "_process_obs_perception"):
            return obj
        obj = getattr(obj, "env")
    raise SystemExit("integrity: task not reachable from the player")


def find_rms(model):
    """The actor-input RunningMeanStd: the module whose running_mean/var/count buffers are 898-D."""
    found = []
    for name, m in model.named_modules():
        rm = getattr(m, "running_mean", None)
        if isinstance(rm, torch.Tensor) and rm.numel() == 898 and hasattr(m, "running_var") and hasattr(m, "count"):
            found.append((name, m))
    if len(found) != 1:
        raise SystemExit("integrity: expected one 898-D RunningMeanStd, found %s" % [n for n, _ in found])
    return found[0][1]


def run_d3(spec, out):
    n = int(spec["num_envs"])
    d3 = PLAN["D3"]
    ckpt = (ROOT / "aerial_gym/rl_training/rl_games" / PLAN["frozen"]["policy_f_rel_to_rl_games"]).resolve()
    if tel.sha256_file(ckpt) != PLAN["frozen"]["policy_f_sha256"]:
        raise SystemExit("integrity: Policy F SHA mismatch")
    player, args = build_player(n, ckpt)
    origins = check_origins()
    task = find_task(player)
    dev = task.device
    if int(task.task_obs["observations"].shape[1]) != 898:
        raise SystemExit("integrity: actor width %d" % task.task_obs["observations"].shape[1])
    rms = find_rms(player.model)
    digest_start = tel.rms_digest(rms)
    if digest_start != PLAN["frozen"]["rms_digest_sha256"]:
        raise SystemExit("integrity: RMS digest %s" % digest_start)

    def det_action(obs):
        with torch.no_grad():
            res = player.model({"is_train": False, "prev_actions": None, "obs": player._preproc_obs(obs),
                                "rnn_states": player.states})
            a = player._model_action(res, True)
            if player.clip_actions:
                from rl_games.algos_torch.players import rescale_actions
                a = rescale_actions(player.actions_low, player.actions_high, torch.clamp(a, -1.0, 1.0))
        return a

    state = {"step": -1, "replay": False, "cache": {}, "reset_this_step": False, "reset_branches": 0, "branch": False}
    events = []
    branch_log = []
    store = {}

    orig_render = task.detector.render_raw_rgbd
    orig_lidar = task._lidar_distance_m
    orig_process = task._process_obs_perception
    orig_sim_render = task.sim_env.render
    orig_reset_idx = task.reset_idx

    def render_wrap(*a, **k):
        if state["replay"]:
            return state["cache"]["rgb"].clone(), state["cache"]["depth"].clone()
        rgb, depth = orig_render(*a, **k)
        state["cache"]["rgb"], state["cache"]["depth"] = rgb.clone(), depth.clone()
        events.append(("camera_render_branch" if state["branch"] else "camera_render", state["step"],
                       int(task.sim_env.step_counter)))
        return rgb, depth

    def lidar_wrap():
        if state["replay"]:
            return state["cache"]["lidar"].clone()
        out_ = orig_lidar()
        state["cache"]["lidar"] = out_.clone()
        return out_

    def sim_render_wrap(*a, **k):
        if k.get("render_components") == "sensors" or (a and a[0] == "sensors"):
            events.append(("lidar_render", state["step"], int(task.sim_env.step_counter)))
        return orig_sim_render(*a, **k)

    def reset_wrap(env_ids, *a, **k):
        if state["step"] > 5:
            state["reset_this_step"] = True
        return orig_reset_idx(env_ids, *a, **k)

    written_keys = ["navrl_raw_rgb", "navrl_raw_depth", "navrl_track_confidence", "navrl_track_age",
                    "navrl_track_covariance"]
    seg_keys = [k for k in task.obs_dict if "segmentation" in k and isinstance(task.obs_dict[k], torch.Tensor)]
    gen = torch.Generator(device=dev)
    gen.manual_seed(77)

    def privileged_fields():
        f = {"target_position": task.target_position, "target_vel_w": task.target_vel_w,
             "target_orientation": task.target_orientation, "states": task.task_obs["states"],
             "obstacle_position": task.obs_dict["obstacle_position"]}
        if hasattr(task, "target_dir_2d") and isinstance(task.target_dir_2d, torch.Tensor):
            f["target_dir_2d"] = task.target_dir_2d
        if isinstance(task.obs_dict.get("navrl_target_position"), torch.Tensor):
            f["navrl_target_position"] = task.obs_dict["navrl_target_position"]
        for k in seg_keys:
            f["seg:" + k] = task.obs_dict[k]
        f["_visible_now"] = task._visible_now
        f["_camera_visible_now"] = task._camera_visible_now
        return f

    def mutate(kind):
        fields = privileged_fields()
        saved = {k: v.clone() for k, v in fields.items()}
        envs = torch.arange(n, device=dev)
        if kind == "B_subset":
            mask = (envs % 2) == 0
        else:
            mask = torch.ones(n, dtype=torch.bool, device=dev)
        if kind == "B_perm":
            perm = (envs + 1) % n
            for k, v in fields.items():
                v[:] = saved[k][perm]
            if n == 1:  # a one-env permutation is the identity; mutate instead so the branch still tests
                kind = "B_all"
        if kind in ("B_all", "B_subset"):
            m = mask
            fields["target_position"][m] += torch.tensor([3.0, -2.0, 0.5], device=dev)
            fields["target_vel_w"][m] = (torch.rand(fields["target_vel_w"][m].shape, generator=gen, device=dev) - 0.5) * 4.0
            q = torch.randn(fields["target_orientation"][m].shape, generator=gen, device=dev)
            fields["target_orientation"][m] = q / q.norm(dim=-1, keepdim=True)
            fields["states"][m] += torch.randn(fields["states"][m].shape, generator=gen, device=dev)
            fields["obstacle_position"][m] += torch.tensor([0.7, 0.0, 0.0], device=dev)
            if "target_dir_2d" in fields:
                r = torch.randn(fields["target_dir_2d"][m].shape, generator=gen, device=dev)
                fields["target_dir_2d"][m] = r / r.norm(dim=-1, keepdim=True).clamp(min=1e-6)
            if "navrl_target_position" in fields:
                fields["navrl_target_position"][m] += torch.tensor([3.0, -2.0, 0.5], device=dev)
            for k in seg_keys:
                v = fields["seg:" + k]
                v[m] = (v[m] * 2 + 7).to(v.dtype)
            fields["_visible_now"][m] = ~fields["_visible_now"][m]
            fields["_camera_visible_now"][m] = ~fields["_camera_visible_now"][m]
        return fields, saved, mask

    def put_back(fields, saved):
        for k, v in fields.items():
            v.copy_(saved[k])

    def in_view_targets():
        from motar_eval import geometry as g

        pos = task.obs_dict["robot_position"]
        q = task.obs_dict["robot_vehicle_orientation"]
        eye = g.camera_eye(pos, q, torch.tensor(CAMERA_T, device=dev))
        return g.in_camera_frustum(eye, q, task.target_position, math.radians(87.0), math.radians(58.0), 20.0)

    def branches(pre, reason):
        k = state["step"]
        rec = {"step": k, "reason": reason, "branches": {}}
        obs_a = task.task_obs["observations"].clone()
        states_a = task.task_obs["states"].clone()
        post = tel.snapshot(task.perception)
        task_scalars = {"_visible_now": task._visible_now.clone(), "_camera_visible_now": task._camera_visible_now.clone()}
        written = {key: task.obs_dict.get(key) for key in written_keys}
        act_a = det_action(obs_a)
        adapters = {name: getattr(task, name, None) for name in ("_kroc_adapter", "_episode_forensics")}
        dist_flag = getattr(task, "_distractor_metric_enabled", False)
        store["s%03d_A_obs" % k] = obs_a.cpu().numpy()
        store["s%03d_A_act" % k] = act_a.cpu().numpy()
        for kind in ("B_all", "B_perm", "B_subset", "P_control"):
            tel.restore(pre)
            for name in adapters:
                setattr(task, name, None)
            task._distractor_metric_enabled = False
            if kind == "P_control":
                saved_tp = task.target_position.clone()
                task.target_position[:] += torch.tensor([0.0, 0.6, 0.0], device=dev)
                orig_process()
                task.target_position.copy_(saved_tp)
                in_view = in_view_targets()
                obs_b = task.task_obs["observations"].clone()
                rows = (obs_b != obs_a).any(dim=1)
                rec["branches"][kind] = {"changed_envs": int(rows.sum()), "in_view_envs": int(in_view.sum())}
                store["s%03d_%s_obs" % (k, kind)] = obs_b.cpu().numpy()
            else:
                fields, saved, mask = mutate(kind)
                state["replay"] = True
                try:
                    orig_process()
                finally:
                    state["replay"] = False
                    put_back(fields, saved)
                obs_b = task.task_obs["observations"].clone()
                act_b = det_action(obs_b)
                row_eq = [(bool(torch.equal(obs_a[i], obs_b[i]))) for i in range(n)]
                act_eq = [(bool(torch.equal(act_a[i], act_b[i]))) for i in range(n)]
                diff_cols = torch.nonzero((obs_a != obs_b).any(dim=0)).flatten().cpu().tolist()
                rec["branches"][kind] = {"rows_equal": row_eq, "actions_equal": act_eq, "mutated_envs": int(mask.sum()),
                                         "differing_columns": diff_cols[:50],
                                         "max_abs_diff": float((obs_a - obs_b).abs().max())}
                store["s%03d_%s_obs" % (k, kind)] = obs_b.cpu().numpy()
            for name, v in adapters.items():
                setattr(task, name, v)
            task._distractor_metric_enabled = dist_flag
        tel.restore(post)
        task.task_obs["observations"].copy_(obs_a)
        task.task_obs["states"].copy_(states_a)
        task._visible_now.copy_(task_scalars["_visible_now"])
        task._camera_visible_now.copy_(task_scalars["_camera_visible_now"])
        for key, v in written.items():
            if v is not None:
                task.obs_dict[key] = v
        branch_log.append(rec)

    def process_wrap():
        k = state["step"]
        reason = None
        if k in d3["branch_steps"]:
            reason = "fixed_step"
        elif state["reset_this_step"] and state["reset_branches"] < 2:
            reason = "reset_boundary"
            state["reset_branches"] += 1
        pre = tel.snapshot(task.perception) if reason else None
        orig_process()
        events.append(("observe", k, int(task.sim_env.step_counter)))
        if reason:
            fork = [torch.cuda.current_device()] if torch.cuda.is_available() else []
            state["branch"] = True
            try:
                with torch.random.fork_rng(devices=fork):
                    branches(pre, reason)
            finally:
                state["branch"] = False

    task.detector.render_raw_rgbd = render_wrap
    task._lidar_distance_m = lidar_wrap
    task.sim_env.render = sim_render_wrap
    task.reset_idx = reset_wrap
    task._process_obs_perception = process_wrap

    obs = player.env_reset(player.env)
    player.get_batch_size(obs, 1)  # as BasePlayer.run(): sets has_batch_dimension for batched obs
    traj_pos, traj_act, traj_done = [], [], []
    t0 = time.time()
    for k in range(int(d3["steps"])):
        state["step"] = k
        state["reset_this_step"] = False
        action = player.get_action(obs, is_deterministic=True)
        events.append(("action", k, int(task.sim_env.step_counter)))
        obs, rew, done, info = player.env_step(player.env, action)
        traj_pos.append(task.obs_dict["robot_position"].detach().cpu().numpy().copy())
        traj_act.append(action.detach().cpu().numpy().copy())
        traj_done.append(done.detach().cpu().numpy().copy() if isinstance(done, torch.Tensor) else np.asarray(done))
    torch.cuda.synchronize()
    wall = time.time() - t0
    digest_end = tel.rms_digest(rms)
    store["trajectory_position"] = np.stack(traj_pos)
    store["trajectory_action"] = np.stack(traj_act)
    store["trajectory_done"] = np.stack(traj_done)
    bad = nan_report({k: v for k, v in store.items()})
    files = [tel.write_npz(out / "d3_branches.npz", store)]
    meta = {"process": spec, "num_envs": n, "checkpoint": str(ckpt), "rms_digest_start": digest_start,
            "rms_digest_end": digest_end, "model_training_flag": bool(player.model.training),
            "rms_training_flag": bool(rms.training), "actor_width": int(task.task_obs["observations"].shape[1]),
            "branch_log": branch_log, "events": events, "segmentation_keys": seg_keys, "loop_wall_s": wall,
            "tf32_matmul": bool(torch.backends.cuda.matmul.allow_tf32), "tf32_cudnn": bool(torch.backends.cudnn.allow_tf32),
            "nan_fields": bad, "origins": origins}
    files.append(tel.write_json(out / "d3_meta.json", meta))
    return files, bad


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("batch", choices=("d1", "d2", "d3"))
    parser.add_argument("--process", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    spec = process_spec(args.process)
    if spec["batch"].lower() != args.batch:
        raise SystemExit("process %s is not batch %s" % (args.process, args.batch))
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    files, bad = {"d1": run_d1, "d2": run_d2, "d3": run_d3}[args.batch](spec, out)
    done = {"process": args.process, "files": files, "nan_fields": bad, "wall_s": time.time() - t0,
            "max_memory_allocated_bytes": int(torch.cuda.max_memory_allocated()) if torch.cuda.is_available() else 0}
    tel.write_json(out / "process_done.json", done)
    log("done %s in %.1f s" % (args.process, done["wall_s"]))
    return INTEGRITY if bad else 0


if __name__ == "__main__":
    sys.exit(main())
