#!/usr/bin/env python3
"""E-L0 validation launcher: one Isaac Gym process per V1 / V2 / V3 process of the E-L0 preregistration.

    python -B tools/e_l0_launcher.py v3 --process v3_static_p2 --out DIR
    python -B tools/e_l0_launcher.py v1 --process v1_scenes --out DIR
    python -B tools/e_l0_launcher.py v2 --process v2_contact --out DIR

Started by ``tools/e_l0_run.py``. V3 runs Policy F through the rl_games player exactly as runner.py
builds it and records, after every step, SHA-256 digests of the actor observation, deterministic
action, pursuer and target state, done/capture/crash flags, reset envs, rewards, camera/LiDAR
sensor tensors and tracker state; the arm (P0/P1/P2) is selected only by environment variables. V1
renders placed scenes and applies E-L0's own labelling functions to exactly those frames. V2 drives
the EQ-1D D1 fixtures with E-L0's contact chain wrapped around the independent substep recorder.
Exit codes: 0 done, 3 integrity (NaN, identity mismatch), 2 usage.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tools")):
    if p not in sys.path:
        sys.path.insert(0, p)

import isaacgym  # noqa: E402,F401  (must precede torch)
import numpy as np  # noqa: E402
import torch  # noqa: E402

import eq1d_fixture_launcher as eq  # noqa: E402  (EQ-1D helpers, used read-only)
from motar_eval import e_l0  # noqa: E402
from motar_eval import eq1d_telemetry as tel  # noqa: E402

PLAN = json.loads((ROOT / "docs/plans/e_l0_evaluation_lineage_v1.json").read_text())
EQ1D = eq.PLAN
INTEGRITY = 3


def spec_for(pid):
    for p in PLAN["run"]["processes"]:
        if p["id"] == pid:
            return p
    raise SystemExit("unknown process %s" % pid)


def sha(t):
    if isinstance(t, torch.Tensor):
        t = t.detach().contiguous().cpu().numpy()
    return hashlib.sha256(np.ascontiguousarray(t).tobytes()).hexdigest()


# ================================================================== V3 non-interference
def run_v3(spec, out):
    n, steps = int(spec["num_envs"]), int(spec["steps"])
    ckpt = (ROOT / "aerial_gym/rl_training/rl_games" / EQ1D["frozen"]["policy_f_rel_to_rl_games"]).resolve()
    if tel.sha256_file(ckpt) != EQ1D["frozen"]["policy_f_sha256"]:
        raise SystemExit("integrity: Policy F SHA mismatch")
    player, _ = eq.build_player(n, ckpt)
    origins = eq.check_origins()
    task = eq.find_task(player)
    if int(task.task_obs["observations"].shape[1]) != 898:
        raise SystemExit("integrity: actor width")
    rms = eq.find_rms(player.model)
    rms_start = tel.rms_digest(rms)
    if rms_start != EQ1D["frozen"]["rms_digest_sha256"]:
        raise SystemExit("integrity: RMS digest %s" % rms_start)
    adapter = getattr(task, "_kroc_adapter", None)
    if e_l0.switch_on(os.environ):  # attached before the first reset/observation of the loop below
        if adapter is None:
            raise SystemExit("integrity: %s=1 needs NAVRL_KROC2027_RECORDER=1" % e_l0.SWITCH)
        e_l0.attach(adapter)
    arm_state = {"recorder": adapter is not None, "e_l0": getattr(adapter, "_e_l0", None) is not None,
                 "occlusion_trace": getattr(adapter, "_occlusion", None) is not None}
    expected = {"P0": (False, False), "P1": (True, False), "P2": (True, True)}[spec["arm"]]
    if (arm_state["recorder"], arm_state["e_l0"]) != expected:
        raise SystemExit("integrity: arm %s built %s" % (spec["arm"], arm_state))
    resets = []
    orig_reset = task.reset_idx

    def reset_wrap(env_ids, *a, **k):  # identical call-through in every arm
        resets.append(torch.as_tensor(env_ids).detach().cpu().tolist())
        return orig_reset(env_ids, *a, **k)

    task.reset_idx = reset_wrap
    tracker = task.perception.tracker
    od = task.obs_dict
    log = open(out / "v3_steps.jsonl", "w")
    traj = {"position": [], "target": [], "action": [], "done": [], "captured": [], "crashed": [],
            "track_age": [], "track_active": []}

    def snapshot_tracker():
        traj["track_age"].append(tracker.age.detach().cpu().numpy().copy())
        traj["track_active"].append(tracker.active.detach().cpu().numpy().copy())

    obs = player.env_reset(player.env)
    player.get_batch_size(obs, 1)
    snapshot_tracker()
    t0 = time.time()
    for k in range(steps):
        resets.clear()
        action = player.get_action(obs, is_deterministic=True)
        obs, rew, done, info = player.env_step(player.env, action)
        rec = {"step": k, "obs": sha(obs), "action": sha(action), "reward": sha(rew), "done": sha(done),
               "captured": sha(task.captured_now), "crashed": sha(task.crashed_now),
               "pursuer": sha(torch.cat([od["robot_position"], od["robot_orientation"], od["robot_linvel"],
                                         od["robot_angvel"]], dim=1)),
               "target": sha(torch.cat([task.target_position, task.target_vel_w], dim=1)),
               "resets": [list(map(int, r)) for r in resets],
               "rgb": sha(od["navrl_raw_rgb"]), "depth": sha(od["navrl_raw_depth"]),
               "lidar": sha(od["depth_range_pixels"]), "semantic": sha(od["segmentation_pixels"]),
               "tracker": sha(torch.cat([tracker.state.flatten(1), tracker.cov.flatten(1), tracker.age.view(-1, 1),
                                         tracker.active.float().view(-1, 1)], dim=1))}
        log.write(json.dumps(rec, sort_keys=True) + "\n")
        traj["position"].append(od["robot_position"].detach().cpu().numpy().copy())
        traj["target"].append(task.target_position.detach().cpu().numpy().copy())
        traj["action"].append(action.detach().cpu().numpy().copy())
        traj["done"].append(done.detach().cpu().numpy().copy())
        traj["captured"].append(task.captured_now.detach().cpu().numpy().copy())
        traj["crashed"].append(task.crashed_now.detach().cpu().numpy().copy())
        snapshot_tracker()
    torch.cuda.synchronize()
    wall = time.time() - t0
    log.close()
    rms_end = tel.rms_digest(rms)
    if adapter is not None:
        adapter.finalize("complete")
    arrays = {k: np.stack(v) for k, v in traj.items()}
    bad = eq.nan_report(arrays)
    files = [tel.write_npz(out / "v3_trajectory.npz", arrays)]
    meta = {"process": spec, "rms_digest_start": rms_start, "rms_digest_end": rms_end,
            "model_training_flag": bool(player.model.training), "arm_state": arm_state, "loop_wall_s": wall,
            "origins": origins, "nan_fields": bad,
            "env": {k: os.environ.get(k) for k in sorted(os.environ) if k.startswith(("NAVRL_KROC2027", "NAVRL_SEED",
                                                                                        "NAVRL_NUM_BARS", "NAVRL_TARGET_BEHAVIOR",
                                                                                        "NAVRL_BULK"))}}
    files.append(tel.write_json(out / "v3_meta.json", meta))
    return files, bad


# ================================================================== V1 label fixtures
def v1_scenes():
    scenes = [dict(s, prime=("bar_front" if s.get("association_beam_deg") is not None else None))
              for s in EQ1D["D2"]["scenes"]]
    scenes += PLAN["V1"]["extra_scenes"]
    return scenes


def run_v1(spec, out):
    scenes = v1_scenes()
    n = len(scenes)
    if int(spec["num_envs"]) != n:
        raise SystemExit("integrity: V1 needs one env per scene (%d)" % n)
    task = eq.build_task(n, seed=4401)
    origins = eq.check_origins()
    dev = task.device
    slots = eq.bar_slots(task)
    half = task.obs_dict["asset_collision_half_extents"]
    drone = torch.tensor([EQ1D["D2"]["drone_env_m"]] * n, dtype=torch.float32, device=dev)
    perception, detector = task.perception, task.detector
    targets, present, layout = [], [], []
    for i, s in enumerate(scenes):
        rel, here = eq.scene_target(s["target"])
        targets.append([float(drone[i, k]) + rel[k] for k in range(3)])
        present.append(here)
        placed = []
        for j, b in enumerate(s["bars"]):
            slot = slots[j]
            hx, hy, hz = (float(half[i, slot, k]) for k in range(3))
            placed.append({"slot": slot, "centre": [float(drone[i, 0]) + float(b["x"]),
                                                    float(drone[i, 1]) + eq.bar_centre_y(b, hy), 1.5],
                           "half": [hx, hy, hz]})
        layout.append(placed)
    target_t = torch.tensor(targets, dtype=torch.float32, device=dev)
    eq.park_all(task, slots)
    for i, placed in enumerate(layout):
        for b in placed:
            task.obs_dict["obstacle_position"][i, b["slot"]] = torch.tensor(b["centre"], device=dev)
    task.sim_env.IGE_env.write_to_sim()
    task.sim_env.warp_env.reset_idx(torch.arange(n, device=dev))
    q = torch.tensor([[0.0, 0.0, 0.0, 1.0]] * n, device=dev)
    eq.set_robot(task, drone, q)
    task.target_position[:] = target_t
    task._sync_target_to_sensor()
    q_veh = task.obs_dict["robot_vehicle_orientation"].clone()
    rgb, depth = detector.render_raw_rgbd(drone, q_veh, task.target_position, task.target_orientation)
    task.sim_env.render(render_components="sensors")
    lidar = task._lidar_distance_m().clone()
    seg = task.obs_dict["segmentation_pixels"].clone()
    perception.reset_idx(torch.arange(n, device=dev))
    obs, diag = perception.observe(rgb, depth, lidar, drone, torch.zeros_like(drone), q_veh, torch.zeros(n, device=dev),
                                   torch.zeros(n, 4, device=dev), task.task_config.max_velocity,
                                   task.task_config.flight_altitude, training=False,
                                   env_bounds_min=task.obs_dict["env_bounds_min"],
                                   env_bounds_max=task.obs_dict["env_bounds_max"])
    camera_visible = diag["camera_visible"].bool().clone()
    sil = e_l0.silhouette_from_task(task)  # E-L0's own function on this exact frame
    cam_prov = e_l0.camera_provenance(camera_visible, sil["visible_px"], int(perception.min_pixels))
    # Association: prime the track per scene, then call the production association once.
    perception.reset_idx(torch.arange(n, device=dev))
    tr = perception.tracker
    primed = np.full((n, 3), np.nan, dtype=np.float32)
    for i, s in enumerate(scenes):
        beam = s.get("association_beam_deg")
        if beam is None:
            continue
        if s.get("prime") == "target":
            p = targets[i]
        else:
            yaw = math.radians(beam)
            d = (math.cos(yaw), math.sin(yaw), 0.0)
            o = [float(v) for v in drone[i]]
            tmin = min((eq.ray_box_t(o, d, [b["centre"][k] - b["half"][k] for k in range(3)],
                                     [b["centre"][k] + b["half"][k] for k in range(3)]) for b in layout[i]),
                       default=math.inf)
            if not math.isfinite(tmin):
                continue
            p = [o[k] + (tmin + float(perception.target_radius)) * d[k] for k in range(3)]
        primed[i] = p
        tr.state[i, :3] = torch.tensor(p, device=dev)
        tr.state[i, 3:] = 0.0
        tr.cov[i] = torch.eye(6, device=dev) * 0.01
        tr.active[i] = True
        tr.age[i] = 1.0
    valid, conf, measured, bearing = perception._associate_lidar_target(lidar, drone, q_veh, camera_visible)
    angles = perception._lidar_angles
    prov = e_l0.lidar_provenance(valid.bool(), bearing, measured, lidar.view(n, -1, angles.numel()),
                                 seg.view(n, -1, angles.numel()), angles)
    host = lambda x: x.detach().cpu().numpy()  # noqa: E731
    arrays = {"alone_px": host(sil["alone_px"]), "visible_px": host(sil["visible_px"]),
              "visible_outside_alone_px": host(sil["visible_outside_alone_px"]),
              "centre_in_frustum": host(sil["centre_in_frustum"]), "centre_range": host(sil["centre_range"]),
              "centre_blocked": host(sil["centre_blocked"]), "cls": host(sil["cls"]), "camera_visible": host(camera_visible),
              "perception_px": host(diag["target_pixels"]), "camera_prov": host(cam_prov),
              "assoc_valid": host(valid), "assoc_measured": host(measured), "assoc_bearing": host(bearing),
              "assoc_beam": host(prov["beam"]), "assoc_row": host(prov["row"]), "assoc_semantic": host(prov["semantic"]),
              "assoc_surface": host(prov["surface"]), "assoc_label": host(prov["label"]),
              "assoc_consistent": host(prov["consistent"]), "primed_world": primed,
              "lidar_m": host(lidar), "semantic": host(seg), "target_mask": host(detector.target_mask).astype(np.int8),
              "drone": host(drone), "target_world": host(target_t), "target_present": np.array(present)}
    bad = eq.nan_report({k: v for k, v in arrays.items() if k not in ("primed_world", "assoc_surface")})
    files = [tel.write_npz(out / "v1_labels.npz", arrays)]
    meta = {"process": spec, "scenes": [s["id"] for s in scenes], "primes": [s.get("prime") for s in scenes],
            "association_beam_deg": [s.get("association_beam_deg") for s in scenes], "layout": layout,
            "camera": {"target_radius": float(detector.target_radius), "max_range": float(detector.max_range),
                       "camera_offset": detector.camera_offset_vehicle.detach().cpu().tolist()},
            "lidar_target_radius": float(task.sim_env.robot_manager.robot.cfg.sensor_config.lidar_config.target_radius),
            "min_pixels": int(perception.min_pixels), "nan_fields": bad, "origins": origins}
    files.append(tel.write_json(out / "v1_meta.json", meta))
    return files, bad


# ================================================================== V2 contact chronology
def run_v2(spec, out):
    d1 = EQ1D["D1"]
    fixtures = d1["fixtures"]
    n = len(fixtures)
    task = eq.build_task(n, seed=4401)
    origins = eq.check_origins()
    dev = task.device
    if abs(float(task.obs_dict["dt"]) - 0.01) > 1e-12:
        raise SystemExit("integrity: V2 runs at the base 10 ms timestep")
    n_sub = int(task.sim_env.cfg.env.num_physics_steps_per_env_step_mean)
    robot = task.sim_env.robot_manager.robot
    motor = robot.control_allocator.motor_model
    tap = tel.MotorTap(motor)
    slots = eq.bar_slots(task)
    half = task.obs_dict["asset_collision_half_extents"]
    starts = torch.tensor([d1["initial_state"]["position_env_m"]] * n, dtype=torch.float32, device=dev)
    eq.park_all(task, slots)
    bars = {}
    for i, f in enumerate(fixtures):  # the same placement rule as the EQ-1D D1 launcher
        if "contact" not in f:
            continue
        c = f["contact"]
        starts[i] = torch.tensor(d1["contact_start_m"], device=dev)
        s = slots[0]
        hx, hy = float(half[i, s, 0]), float(half[i, s, 1])
        h = math.radians(c["heading_deg"])
        direction = (math.cos(h), math.sin(h))
        x0, y0 = float(starts[i, 0]), float(starts[i, 1])
        if c["lateral_m"] == "corner":
            along = 0.14 + c["gap_m"]
            cx = x0 + along * direction[0] + math.copysign(hx, direction[0])
            cy = y0 + along * direction[1] + math.copysign(hy, direction[1])
        else:
            along = 0.14 + c["gap_m"] + hx
            cx, cy = x0 + along * direction[0], y0 + along * direction[1]
        task.obs_dict["obstacle_position"][i, s] = torch.tensor([cx, cy, 1.5], device=dev)
        bars[i] = {"slot": s, "centre": [cx, cy, 1.5], "half": [hx, hy, float(half[i, s, 2])]}
    eq.set_robot(task, starts, torch.tensor([[0.0, 0.0, 0.0, 1.0]] * n, device=dev))
    task.sim_env.IGE_env.write_to_sim()
    motor.current_motor_thrust[:] = eq.HOVER_MASS_KG * eq.G / 4.0
    total = int(round(d1["duration_s"] / d1["action_period_s"]))
    rec = tel.SubstepRecorder(n, total * n_sub, dev, {
        "command": lambda: robot.action_tensor, "motor_ref_thrust": lambda: tap.last_ref,
        "motor_thrust": lambda: motor.current_motor_thrust,
        "robot_force": lambda: robot.robot_force_tensors.sum(dim=1),
        "robot_torque": lambda: robot.robot_torque_tensors.sum(dim=1),
        "position": lambda: task.obs_dict["robot_position"], "orientation_xyzw": lambda: task.obs_dict["robot_orientation"],
        "linvel_w": lambda: task.obs_dict["robot_linvel"], "angvel_w": lambda: task.obs_dict["robot_angvel"],
        "contact_force": lambda: task.obs_dict["robot_contact_force_tensor"]})
    task.sim_env.set_physics_step_callback(rec)
    chain = e_l0.ContactChain(task.sim_env.physics_step_callback, lambda: task.obs_dict["robot_contact_force_tensor"],
                              float(task.sim_env.cfg.env.collision_force_threshold), n, dev)
    task.sim_env.set_physics_step_callback(chain)
    counts, firsts, peaks = [], [], []
    for a in range(total):
        rec.current_action = a
        chain.begin_action()
        task.sim_env.step(actions=tel.d1_commands(fixtures, a * d1["action_period_s"], device=dev))
        counts.append(chain.count.detach().cpu().numpy().copy())
        firsts.append(chain.first.detach().cpu().numpy().copy())
        peaks.append(chain.peak.detach().cpu().numpy().copy())
    torch.cuda.synchronize()
    task.sim_env.set_physics_step_callback(None)
    tap.detach()
    arrays = rec.arrays()
    arrays.update(e_l0_count=np.stack(counts), e_l0_first=np.stack(firsts), e_l0_peak=np.stack(peaks))
    bad = eq.nan_report(arrays)
    files = [tel.write_npz(out / "v2_contact.npz", arrays)]
    meta = {"process": spec, "substeps_per_action": n_sub, "threshold_n": chain.threshold, "contact_bars": bars,
            "fixtures": [f["name"] for f in fixtures], "nan_fields": bad, "origins": origins}
    files.append(tel.write_json(out / "v2_meta.json", meta))
    return files, bad


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("batch", choices=("v1", "v2", "v3"))
    parser.add_argument("--process", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    spec = spec_for(args.process)
    if spec["batch"].lower() != args.batch:
        raise SystemExit("process %s is not batch %s" % (args.process, args.batch))
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    files, bad = {"v1": run_v1, "v2": run_v2, "v3": run_v3}[args.batch](spec, out)
    done = {"process": args.process, "files": files, "nan_fields": bad, "wall_s": time.time() - t0,
            "max_memory_allocated_bytes": int(torch.cuda.max_memory_allocated()) if torch.cuda.is_available() else 0}
    tel.write_json(out / "process_done.json", done)
    print("[e_l0] done %s in %.1f s" % (args.process, done["wall_s"]), flush=True)
    return INTEGRITY if bad else 0


if __name__ == "__main__":
    sys.exit(main())
