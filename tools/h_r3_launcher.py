#!/usr/bin/env python3
"""H-R3 pilot arm subprocess (docs/plans/h_r3_pilot_v1.json). Run only through tools/h_r3_run.py.

Runs Frozen Policy F through the rl_games player exactly as the E-L0 V3 loop does. Per env, at the
first camera-detection drop after acquisition in the first episode (an online signal), arm B1/B2/B3
replaces that env's action with the scripted controller for at most W steps or until the camera
detects again; B0 keeps Policy F. Every arm also computes the B3 candidate evaluation in shadow so
compute is comparable. After each step the evaluator records, for every env, a prefix digest and
evaluation-only labels (E-L0 silhouette); nothing recorded is fed back.
"""
from __future__ import annotations

import argparse
import hashlib
import json
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

import eq1d_fixture_launcher as eq  # noqa: E402
from motar_eval import e_l0, h_r3_arms  # noqa: E402
from motar_eval import eq1d_telemetry as tel  # noqa: E402

PLAN = json.loads((ROOT / "docs/plans/h_r3_pilot_v1.json").read_text())
EQ1D = eq.PLAN
ELEVATIONS_DEG = (20.0, 10.0, 0.0, -10.0)


def row_digests(rows):
    data = np.ascontiguousarray(rows.detach().float().cpu().numpy())
    return np.array([np.frombuffer(hashlib.sha256(r.tobytes()).digest()[:8], dtype=np.uint64)[0] for r in data],
                    dtype=np.uint64)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--arm", required=True, choices=("B0", "B1", "B2", "B3"))
    ap.add_argument("--num-envs", type=int, required=True)
    ap.add_argument("--steps", type=int, required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--process-id", required=True)
    args = ap.parse_args(argv)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    n, steps, window = args.num_envs, args.steps, int(PLAN["event"]["window_steps"])
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
        raise SystemExit("integrity: RMS digest")
    perception, tracker, od, dev = task.perception, task.perception.tracker, task.obs_dict, task.device
    elev = torch.deg2rad(torch.tensor(ELEVATIONS_DEG, device=dev))
    angles = perception._lidar_angles
    maxv, yaw_max, far = float(task.task_config.max_velocity), float(task.task_config.yaw_rate_max), float(perception.lidar_max_range)
    resets_this_step = []
    orig_reset = task.reset_idx

    def reset_wrap(env_ids, *a, **k):  # identical call-through in every arm
        resets_this_step.extend(torch.as_tensor(env_ids).detach().cpu().tolist())
        return orig_reset(env_ids, *a, **k)

    task.reset_idx = reset_wrap
    acquired = torch.zeros(n, dtype=torch.bool, device=dev)
    prev_cam = torch.zeros(n, dtype=torch.bool, device=dev)
    first_episode = torch.ones(n, dtype=torch.bool, device=dev)
    t0 = torch.full((n,), -1, dtype=torch.long, device=dev)
    active = torch.zeros(n, dtype=torch.bool, device=dev)       # inside the arm window
    finished = torch.zeros(n, dtype=torch.bool, device=dev)     # window ended (reobs / W / reset)
    end_reason = ["" for _ in range(n)]
    trace = {k: [] for k in ("camera", "lidar_only", "el0_cls", "el0_visible_px", "el0_alone_px", "el0_in_frustum",
                             "crashed", "captured", "reset", "robot_pos", "target_pos", "belief_pos", "action",
                             "intervening", "shadow_offset", "shadow_clear", "digest")}
    ctrl_ms = []
    obs = player.env_reset(player.env)
    player.get_batch_size(obs, 1)
    wall0 = time.time()
    for k in range(steps):
        resets_this_step.clear()
        action = player.get_action(obs, is_deterministic=True)
        torch.cuda.synchronize()
        c0 = time.perf_counter()
        lidar = task._lidar_distance_m()
        shadow, shadow_info = h_r3_arms.scripted_action(
            "B3", od["robot_position"], od["robot_vehicle_orientation"], tracker.state[:, :3], lidar, angles, elev,
            max_velocity=maxv, yaw_rate_max=yaw_max, lidar_far_m=far)
        if args.arm != "B0":
            scripted, _ = h_r3_arms.scripted_action(
                args.arm, od["robot_position"], od["robot_vehicle_orientation"], tracker.state[:, :3], lidar, angles, elev,
                max_velocity=maxv, yaw_rate_max=yaw_max, lidar_far_m=far)
            action = torch.where(active.view(-1, 1), scripted.to(action.dtype), action)
        torch.cuda.synchronize()
        ctrl_ms.append((time.perf_counter() - c0) * 1e3)
        trace["intervening"].append(active.cpu().numpy().copy())
        trace["action"].append(action.detach().cpu().numpy().copy())
        trace["shadow_offset"].append(shadow_info["chosen_offset_m"].cpu().numpy().copy())
        trace["shadow_clear"].append(shadow_info["clear_found"].cpu().numpy().copy())
        obs, rew, done, info = player.env_step(player.env, action)
        cam = task._camera_visible_now.clone().bool()
        fused = task._visible_now.clone().bool()
        sil = e_l0.silhouette_from_task(task)  # evaluator-only, recorded, never fed back
        reset_mask = torch.zeros(n, dtype=torch.bool, device=dev)
        if resets_this_step:
            reset_mask[torch.as_tensor(sorted(set(resets_this_step)), device=dev)] = True
        # Prefix digest of this step's state (obs after step k), per env.
        obs_t = obs["obs"] if isinstance(obs, dict) else obs
        rows = torch.cat([obs_t.float(), od["robot_position"], od["robot_orientation"], od["robot_linvel"], od["robot_angvel"],
                          task.target_position, task.target_vel_w, tracker.state, tracker.cov.flatten(1),
                          tracker.age.view(-1, 1), tracker.active.float().view(-1, 1), cam.float().view(-1, 1)], dim=1)
        trace["digest"].append(row_digests(rows))
        for key, val in (("camera", cam), ("lidar_only", fused & ~cam), ("el0_cls", sil["cls"]),
                         ("el0_visible_px", sil["visible_px"]), ("el0_alone_px", sil["alone_px"]),
                         ("el0_in_frustum", sil["centre_in_frustum"]), ("crashed", task.crashed_now.bool()),
                         ("captured", task.captured_now.bool()), ("reset", reset_mask),
                         ("robot_pos", od["robot_position"]), ("target_pos", task.target_position), ("belief_pos", tracker.state[:, :3])):
            trace[key].append(val.detach().cpu().numpy().copy())
        # Online state machine (camera flag and resets only).
        if bool(active.any()):
            ended = active & (cam | reset_mask | ((k + 1 - t0) >= window))
            for i in torch.nonzero(ended).flatten().tolist():
                end_reason[i] = "reobserved" if bool(cam[i]) else ("reset" if bool(reset_mask[i]) else "window")
            active &= ~ended
            finished |= ended
        trigger = first_episode & acquired & prev_cam & ~cam & (t0 < 0) & ~reset_mask & torch.tensor(k + 1 + window <= steps, device=dev)
        if bool(trigger.any()):
            t0 = torch.where(trigger, torch.full_like(t0, k + 1), t0)
            active |= trigger
        acquired |= cam
        first_episode &= ~reset_mask
        prev_cam = cam
    torch.cuda.synchronize()
    wall = time.time() - wall0
    rms_end = tel.rms_digest(rms)
    arrays = {k: np.stack(v) for k, v in trace.items()}
    arrays["t0"] = t0.cpu().numpy()
    arrays["controller_ms"] = np.asarray(ctrl_ms)
    bad = eq.nan_report({k: v for k, v in arrays.items() if v.dtype.kind == "f"})
    files = [tel.write_npz(out / "h_r3_trace.npz", arrays)]
    meta = {"arm": args.arm, "num_envs": n, "steps": steps, "window_steps": window, "rms_digest_start": rms_start,
            "rms_digest_end": rms_end, "model_training_flag": bool(player.model.training), "origins": origins,
            "vision_mode": bool(task.vision_mode), "step_dt": float(task.step_dt), "max_velocity": maxv,
            "yaw_rate_max": yaw_max, "speed_governor": task.speed_governor_cfg.mode, "end_reason": end_reason,
            "controller_params": h_r3_arms.PARAMS, "loop_wall_s": wall, "nan_fields": bad,
            "el0_classes": list(e_l0.CLASSES),
            "env": {k: os.environ.get(k) for k in sorted(os.environ) if k.startswith(("NAVRL_SEED", "NAVRL_NUM_BARS", "NAVRL_TARGET",
                                                                                        "NAVRL_SENSOR_LINEAGE", "NAVRL_SPEED_GOVERNOR", "NAVRL_MAX_VELOCITY"))}}
    files.append(tel.write_json(out / "h_r3_meta.json", meta))
    tel.write_json(out / "process_done.json", {"process": args.process_id, "files": files, "nan_fields": bad,
                                               "wall_s": wall, "actor_obs_dim": 898,
                                               "peak_allocated_bytes": int(torch.cuda.max_memory_allocated())})
    return 3 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
