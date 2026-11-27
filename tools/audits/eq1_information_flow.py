#!/usr/bin/env python3
"""EQ-1C information flow — actor firewall, checkpoint/RMS state, loss separation and populations (CPU).

    python -B tools/audits/eq1_information_flow.py [--out docs/evidence/.../eq1c_information_flow.json]

Validation only; Policy F and the OCC-P2 raw results are read, never written.

C1 firewall. (a) Static: in `_process_obs_perception` the actor tensor is written only from
`perception.observe(...)`, and no keyword argument of that call reads target / critic state (AST).
(b) Dynamic, held-fixed-input invariance on the production `NavRLPerceptionModule` built on CPU from
the OCC-P2 receipt environment: with the same sensor/ego/history sequence, the 898-D actor rows are
bit-identical across fresh instances, across changed torch / numpy / random RNG state, after an
environment permutation is undone and across a mid-sequence reset; batch sizes 1/2/4/128 are
compared for env 0 (bitwise status and max |diff| recorded separately, as the plan requires). A
positive control with perturbations on shows the comparison detects an RNG dependence. Mutating
the simulator's privileged target state while keeping the rendered images and LiDAR scan fixed
needs the task and renderer, so that part is EQ-1D.

C2 checkpoint / RMS. Policy F's SHA-256, the 898-D input RunningMeanStd digest (mean, var, count),
rl_games' own RunningMeanStd in eval mode left unchanged by 1000 forward batches, with a train-mode
positive control; the player source calls model.eval() and never model.train().

C4 loss separation (OCC-P2 traces, post-acquisition quota frames without a camera detection):
FOV exit / geometric centre occlusion / in-view clear camera loss, each split by the scale-loss
band from EQ-1A, plus LiDAR-only fused frames (range association, identity not verified).

C5 populations. Independent recomputation of the quota (k < 4 per env) and legacy-window rules
from every ledger row, compared with the recorded flags and with each other.
"""

from __future__ import annotations

import argparse
import ast
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import random
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402

RECEIPT = common.RECEIPT
TASK = "aerial_gym/task/navrl_task/navrl_task.py"
STEPS = 40
PERM = [2, 0, 3, 1]
RESET_STEP, RESET_ENV = 20, 2
SCALE_POSSIBLE_M = 12.75  # EQ-1A A5: first range where a clear target can render < 2 pixel centres
CAPTURE_R = 0.5


# ------------------------------------------------------------------ C1 static
def static_firewall():
    tree = ast.parse((common.ROOT / TASK).read_text())
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_process_obs_perception")
    calls = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == "observe"]
    forbidden = ("target_", "states", "critic", "privileged", "gt_", "oracle")
    args = {kw.arg: ast.unparse(kw.value) if hasattr(ast, "unparse") else ast.dump(kw.value) for kw in calls[0].keywords}
    leaks = {k: v for k, v in args.items() if any(f in v for f in forbidden)}
    writes = []
    for n in ast.walk(fn):
        if isinstance(n, ast.Assign):
            for t in n.targets:
                text = ast.dump(t)
                if "observations" in text:
                    writes.append(ast.dump(n.value))
    render = [n for n in ast.walk(fn) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
              and n.func.attr == "render_raw_rgbd"]
    return {"observe_calls": len(calls), "observe_keywords": sorted(args), "forbidden_argument_reads": leaks,
            "actor_writes": len(writes), "actor_written_from_observe_result_only": all("structured" in w for w in writes),
            "target_pose_reaches": "render_raw_rgbd only" if render else "no renderer call found",
            "status": "PASS" if len(calls) == 1 and not leaks and writes and all("structured" in w for w in writes)
            else "FAIL"}


# ------------------------------------------------------------------ C1 dynamic
receipt_env = common.receipt_env
build_module = common.build_perception


def synthetic_inputs(t, envs, hbeams, vbeams):
    """Deterministic sensor / ego / history inputs for step t and the given env ids."""
    n = len(envs)
    e = torch.tensor(envs, dtype=torch.float32)
    rgb = torch.full((n, 3, 90, 160), 0.5)
    depth = torch.full((n, 90, 160), 15.0)
    visible = (t % 10) < 6 or t < 3
    for i, env in enumerate(envs):
        if visible and not (env == 1 and t > 25):
            c = 60 + (t + 7 * env) % 40
            rgb[i, :, 40:48, c:c + 8] = torch.tensor([1.0, 0.0, 0.0]).view(3, 1, 1)
            depth[i, 40:48, c:c + 8] = 6.0 - 0.02 * t
    lidar = torch.full((n, vbeams * hbeams), 12.0)
    lidar.view(n, vbeams, hbeams)[:, :, 10:14] = 4.0 + 0.1 * e.view(-1, 1, 1)
    lidar.view(n, vbeams, hbeams)[:, :, 36] = 5.6 - 0.02 * t  # forward return near the target range
    pos = torch.stack([2.0 + 0.1 * t + e, 5.0 + 0.05 * e, torch.ones(n)], dim=1)
    vel = torch.stack([torch.ones(n), 0.1 * e, torch.zeros(n)], dim=1)
    yaw = 0.1 * e
    quat = torch.stack([torch.zeros(n), torch.zeros(n), torch.sin(yaw / 2), torch.cos(yaw / 2)], dim=1)
    action = torch.stack([torch.sin(0.3 * t + e), torch.cos(0.2 * t + e), 0.1 * e, 0.05 * torch.ones(n)], dim=1)
    return {"rgb": rgb, "depth": depth, "lidar_m": lidar, "drone_pos_w": pos, "drone_vel_w": vel,
            "vehicle_quat": quat, "yaw_rate": 0.01 * e, "previous_action": action}


def rollout(envs, training=False, perm=None, rng_seed=None, reset=True):
    m, tc, perception = build_module(len(envs))
    rows = []
    for t in range(STEPS):
        if rng_seed is not None:
            torch.manual_seed(rng_seed + 7919 * t)
            np.random.seed(rng_seed + t)
            random.seed(rng_seed - t)
        x = synthetic_inputs(t, envs, perception.HBEAMS, perception.VBEAMS)
        if reset and t == RESET_STEP and RESET_ENV in envs:
            m.reset_idx(torch.tensor([envs.index(RESET_ENV)]))
        obs, diag = m.observe(**x, max_velocity=tc.max_velocity, flight_altitude=tc.flight_altitude, training=training,
                              env_bounds_min=torch.zeros(len(envs), 3), env_bounds_max=torch.tensor([[40.0, 40.0, 3.0]] * len(envs)))
        rows.append(obs.clone())
    return torch.stack(rows)  # [T, N, 898]


def digest(t):
    return hashlib.sha256(t.contiguous().numpy().tobytes()).hexdigest()


def dynamic_firewall():
    with common.production(receipt_env()):
        return _dynamic_firewall()


def _dynamic_firewall():
    base_envs = [0, 1, 2, 3]
    a = rollout(base_envs)
    b = rollout(base_envs)
    c = rollout(base_envs, rng_seed=12345)
    permuted = [base_envs[i] for i in PERM]
    p = rollout(permuted)
    inverse = [permuted.index(e) for e in base_envs]
    p_back = p[:, inverse]
    sizes = {}
    for n in (1, 2, 4, 128):
        envs = [0] + [k % 4 for k in range(1, n)]
        r = rollout(envs, reset=False)[:, 0]
        sizes[n] = r
    ref = sizes[1]
    batch = {str(n): {"bitwise_equal_env0": bool(torch.equal(r, ref)), "max_abs_diff": float((r - ref).abs().max())}
             for n, r in sizes.items()}
    ctl1 = rollout(base_envs, training=True, rng_seed=1)
    ctl2 = rollout(base_envs, training=True, rng_seed=2)
    width = a.shape[-1]
    checks = {"actor_width": width, "steps": STEPS, "envs": len(base_envs),
              "fresh_instances_bitwise_equal": bool(torch.equal(a, b)),
              "rng_state_changed_bitwise_equal": bool(torch.equal(a, c)),
              "permutation_undone_bitwise_equal": bool(torch.equal(a, p_back)),
              "permutation_max_abs_diff": float((a - p_back).abs().max()),
              "reset_step": RESET_STEP, "reset_env": RESET_ENV,
              "batch_size_env0": batch,
              "positive_control_training_rng_differs": not bool(torch.equal(ctl1, ctl2)),
              "actor_digest_sha256": digest(a)}
    ok = (width == 898 and checks["fresh_instances_bitwise_equal"] and checks["rng_state_changed_bitwise_equal"]
          and checks["permutation_undone_bitwise_equal"] and checks["positive_control_training_rng_differs"])
    checks["status"] = "PASS" if ok else "FAIL"
    checks["scope"] = ("module-level held-fixed-input invariance; task-level privileged-state mutation with fixed "
                       "rendered sensors is EQ-1D (needs Isaac Gym)")
    return checks


# ------------------------------------------------------------------ C2 checkpoint / RMS
def checkpoint_rms():
    path = common.MAIN_CHECKOUT / common.POLICY_F
    sha = common.sha256_file(path)
    ck = torch.load(path, map_location="cpu")
    m = ck["model"]
    prefix = "_orig_mod.running_mean_std."
    mean, var, count = m[prefix + "running_mean"], m[prefix + "running_var"], m[prefix + "count"]
    h = hashlib.sha256()
    for t in (mean, var, count):
        h.update(t.detach().cpu().contiguous().numpy().tobytes())
    from rl_games.algos_torch.running_mean_std import RunningMeanStd

    rms = RunningMeanStd(mean.shape[0])
    rms.load_state_dict({"running_mean": mean.clone(), "running_var": var.clone(), "count": count.clone()})
    rms.eval()
    gen = torch.Generator().manual_seed(0)
    for _ in range(1000):
        rms(torch.randn(64, mean.shape[0], generator=gen, dtype=torch.float64).float())
    eval_unchanged = (torch.equal(rms.running_mean, mean) and torch.equal(rms.running_var, var)
                      and torch.equal(rms.count, count))
    rms.train()
    rms(torch.randn(64, mean.shape[0], generator=gen))
    train_changes = not torch.equal(rms.running_mean, mean)
    players = Path(sys.modules["rl_games.algos_torch.running_mean_std"].__file__).parent / "players.py"
    src = players.read_text()
    return {"checkpoint": common.POLICY_F, "sha256": sha, "sha256_expected": common.POLICY_F_SHA256,
            "checkpoint_identity": common.verdict(sha == common.POLICY_F_SHA256, "Policy F, read-only"),
            "rms_input_width": int(mean.shape[0]), "rms_count": float(count),
            "rms_digest_sha256": h.hexdigest(),
            "central_value_stats_present": "central_val_stats" in ck,
            "eval_forward_leaves_state_unchanged": eval_unchanged, "train_forward_changes_state": train_changes,
            "player_calls_eval": "self.model.eval()" in src, "player_calls_train": ".train()" in src,
            "rl_games_version_file": str(players),
            "status": "PASS" if (sha == common.POLICY_F_SHA256 and mean.shape[0] == 898 and eval_unchanged
                                 and train_changes and "self.model.eval()" in src and ".train()" not in src) else "FAIL"}


# ------------------------------------------------------------------ C4 loss separation
def cell_losses(cell_dir):
    from motar_eval import occlusion_trace as ot

    common.assert_in_tree(ot)
    cell_dir = Path(cell_dir)
    frames = ot.read_trace(cell_dir)["frames"]
    rows = [json.loads(l) for l in open(cell_dir / "episodes.jsonl")]
    quota = {(r["env_index"], r["env_episode_index"]) for r in rows if r["in_quota"]}
    key = frames["env"].astype(np.int64) * 1_000_000 + frames["episode"].astype(np.int64)
    qkeys = np.array([e * 1_000_000 + k for e, k in quota], dtype=np.int64)
    f = frames[np.isin(key, qkeys)]
    flag = lambda bit: (f["flags"] & bit) != 0  # noqa: E731
    acquired = flag(ot.ACQUIRED_BEFORE)
    camera, fused, lidar = flag(ot.CAMERA_VALID), flag(ot.FUSED_VALID), flag(ot.LIDAR_VALID)
    in_fov, geo_vis = flag(ot.GEO_IN_FOV), flag(ot.GEO_VISIBLE)
    rng = np.linalg.norm((f["t_pos"] - f["p_pos"]).astype(np.float64), axis=1)
    far = rng > SCALE_POSSIBLE_M
    lost = acquired & ~camera
    out = {"post_acquisition_frames": int(acquired.sum()), "camera_lost_frames": int(lost.sum()),
           "fov_exit": int((lost & ~in_fov).sum()),
           "geometric_centre_occlusion_in_view": int((lost & in_fov & ~geo_vis).sum()),
           "in_view_clear_camera_loss": int((lost & in_fov & geo_vis).sum()),
           "in_view_clear_camera_loss_beyond_scale_band": int((lost & in_fov & geo_vis & far).sum()),
           "in_view_clear_camera_loss_within_scale_band": int((lost & in_fov & geo_vis & ~far).sum()),
           "lidar_only_fused_frames": int((acquired & ~camera & fused).sum()),
           "lidar_flag_frames": int((acquired & lidar).sum()),
           "fused_lost_frames": int((acquired & ~fused).sum()),
           "camera_valid_but_centre_occluded": int((acquired & camera & in_fov & ~geo_vis).sum()),
           "camera_valid_out_of_fov": int((acquired & camera & ~in_fov).sum()),
           "range_beyond_scale_band_frames": int((acquired & far).sum())}
    return {"cell": cell_dir.name, "group": cell_dir.parent.parent.name, "bars": int(rows[0]["bars"]), **out}


def loss_separation(workers=12):
    from eq1_scene_feasibility import cell_dirs

    with ProcessPoolExecutor(max_workers=workers) as pool:
        cells = list(pool.map(cell_losses, cell_dirs()))
    table = {}
    for c in cells:
        for k in ("%s/%d" % (c["group"], c["bars"]), "%s/all" % c["group"], "all"):
            t = table.setdefault(k, {})
            for f, v in c.items():
                if isinstance(v, int) and f != "bars":
                    t[f] = t.get(f, 0) + v
    return {"scale_band_m": SCALE_POSSIBLE_M, "by_group_density": table,
            "identity_loss": {"status": "NOT_RECORDED", "note": "association provenance (which surface a LiDAR "
                              "return came from) is not in the trace; LiDAR-only fused frames are range associations"},
            "sensor_dropout": {"status": "NOT_ACTIVE", "note": "perturbations off in evaluation (training=False), so "
                               "configured dropout/noise did not run (EQ0-03)"},
            "note": "in-view clear camera loss = fewer than 2 rendered target pixels while the centre ray is clear and "
                    "in the frustum: partial occlusion of the silhouette or scale loss (EQ-1A A5)",
            "camera_valid_out_of_fov_note": "the in-FOV flag tests the target centre only; a camera-valid frame with "
                                            "the centre outside is an edge view with >= 2 silhouette pixels still in the "
                                            "image (spot check E0_static__115bars__seed4301: 2,851 frames, |bearing| "
                                            "43.6-53.3 deg at 5-95 %), so FOV exit also needs a silhouette label"}


# ------------------------------------------------------------------ C5 populations
def cell_population(cell_dir):
    cell_dir = Path(cell_dir)
    rows = [json.loads(l) for l in open(cell_dir / "episodes.jsonl")]
    summary = json.loads((cell_dir / "episode_ledger_summary.json").read_text())
    quota_k = int(summary.get("quota_per_env", 4))
    quota = {(r["env_index"], r["env_episode_index"]) for r in rows if r["env_episode_index"] < quota_k}
    rec_quota = {(r["env_index"], r["env_episode_index"]) for r in rows if r["in_quota"]}
    window = summary.get("legacy_window", {})
    export_at = window.get("export_at")
    by_step = sorted(rows, key=lambda r: (r["completion_step"], r["completion_rank"]))
    legacy, n, cutoff = set(), 0, None
    for r in by_step:
        if cutoff is not None and r["completion_step"] > cutoff:
            break
        legacy.add((r["env_index"], r["env_episode_index"]))
        n += r["outcome"] in ("capture", "crash", "timeout")
        if cutoff is None and export_at is not None and n >= export_at:
            cutoff = r["completion_step"]
    rec_legacy = {(r["env_index"], r["env_episode_index"]) for r in rows if r["in_legacy_window"]}
    envs = {r["env_index"] for r in rows}
    outcomes = {r["env_index"] * 1000 + r["env_episode_index"]: r["outcome"] for r in rows}
    count = lambda keys: {o: sum(outcomes[e * 1000 + k] == o for e, k in keys) for o in ("capture", "crash", "timeout")}  # noqa: E731
    q_counts, l_counts = count(rec_quota), count(rec_legacy)
    return {"cell": cell_dir.name, "rows": len(rows), "quota_expected": len(envs) * quota_k,
            "quota_independent": len(quota), "quota_recorded_equal": quota == rec_quota,
            "legacy_export_at": export_at, "legacy_independent": len(legacy),
            "legacy_recorded_equal": legacy == rec_legacy if export_at is not None else None,
            "legacy_recorded": len(rec_legacy), "legacy_cutoff_step_recorded": window.get("cutoff_step"),
            "legacy_cutoff_step_independent": cutoff, "quota_equals_legacy": rec_quota == rec_legacy,
            "quota_counts": q_counts, "legacy_counts": l_counts, "outcome_counts_equal": q_counts == l_counts,
            "quota_minus_legacy": len(rec_quota - rec_legacy), "legacy_minus_quota": len(rec_legacy - rec_quota)}


def populations():
    from eq1_scene_feasibility import cell_dirs

    cells = [cell_population(d) for d in cell_dirs()]
    differ = [c for c in cells if not c["quota_equals_legacy"]]
    return {"cells": len(cells),
            "quota_rule_reproduced": all(c["quota_recorded_equal"] and c["quota_independent"] == c["quota_expected"]
                                         for c in cells),
            "legacy_rule_reproduced": all(c["legacy_recorded_equal"] in (True, None) for c in cells),
            "legacy_rule_checked_cells": sum(c["legacy_recorded_equal"] is not None for c in cells),
            "cells_quota_differs_from_legacy": len(differ),
            "cells_outcome_counts_differ": sum(not c["outcome_counts_equal"] for c in cells),
            "max_quota_minus_legacy": max((c["quota_minus_legacy"] for c in cells), default=0),
            "max_legacy_minus_quota": max((c["legacy_minus_quota"] for c in cells), default=0),
            "same_population": common.verdict(not differ, "membership differs in %d of %d cells; outcome counts in %d" % (
                len(differ), len(cells), sum(not c["outcome_counts_equal"] for c in cells))),
            "per_cell": cells}


def run(workers=12):
    return {"schema": "motar.eq1c-information-flow.v1", "package": "EQ-1C information flow (CPU, validation only)",
            "source": common.source_binding([TASK, "aerial_gym/task/navrl_task/navrl_perception.py",
                                             "aerial_gym/config/task_config/navrl_task_config.py",
                                             "aerial_gym/task/navrl_task/navrl_episode_ledger.py",
                                             "motar_eval/occlusion_trace.py"]),
            "receipt_env_source": str(RECEIPT),
            "C1_static": static_firewall(), "C1_dynamic": dynamic_firewall(), "C2_checkpoint_rms": checkpoint_rms(),
            "C4_loss_separation": loss_separation(workers), "C5_populations": populations()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out")
    parser.add_argument("--workers", type=int, default=12)
    args = parser.parse_args(argv)
    record = run(args.workers)
    if args.out:
        common.write_record(args.out, record)
    for k in ("C1_static", "C1_dynamic", "C2_checkpoint_rms"):
        print(k, record[k]["status"])
    print("C5", record["C5_populations"]["same_population"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
