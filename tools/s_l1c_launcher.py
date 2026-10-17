#!/usr/bin/env python3
"""S-L1C fixture/closed-loop subprocess. Run only through the watchdogged orchestrator."""
from __future__ import annotations

import argparse
import copy
import json
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
import isaacgym  # noqa: E402,F401  must precede torch
import numpy as np  # noqa: E402
import torch  # noqa: E402
import eq1d_fixture_launcher as eq  # noqa: E402
import e_l0_launcher as el  # noqa: E402
from motar_eval import eq1d_telemetry as tel  # noqa: E402

PLAN = json.loads((ROOT / 'docs/plans/s_l1c_v1.json').read_text())
S_L1A_PLAN = json.loads((ROOT / 'docs/plans/s_l1a_v1.json').read_text())
S_L1B_PLAN = json.loads((ROOT / 'docs/plans/s_l1b_v1.json').read_text())
SENSOR_MODES = ('legacy', 's_l1b', 's_l1c')
VARIANTS = ('off', 's_l1a', 's_l1b', 's_l1c_p1', 's_l1c_p2')
OFFSETS_M = (0.0, -0.15, 0.15)


def process_spec(pid):
    run = PLAN['run']
    if pid.startswith('sensors_'):
        mode = pid.split('_', 1)[1]
        if mode not in SENSOR_MODES:
            raise ValueError('Unknown sensor mode')
        return {'id': pid, 'batch': 'SENSORS', 'mode': mode, 'num_envs': run['sensor_num_envs'], 'seed': run['sensor_seed']}
    for config in ('static', 'moving'):
        for variant in VARIANTS:
            if pid == config + '_' + variant:
                return {'id': pid, 'batch': 'V3', 'config': config, 'variant': variant,
                        'arm': 'P1' if variant == 's_l1c_p1' else 'P2',
                        'num_envs': run['closed_loop_num_envs'], 'steps': run['closed_loop_steps'],
                        'seed': run['closed_loop_seed_by_config'][config]}
    raise ValueError('Unknown process ' + pid)


def host(tensor):
    return tensor.detach().cpu().numpy().copy()


def association_probes(task, lidar_by_pass, drone):
    """Prime tracks at the true target centre (+-0.15 m along the line of sight) with the camera
    lost and run the production association once per scan. Priming is fixture input only."""
    perception = task.perception
    tr = perception.tracker
    n = drone.shape[0]
    q = torch.tensor([[0., 0., 0., 1.]] * n, device=task.device)
    target = task.target_position.clone()
    los = target - drone
    unit = los / los.norm(dim=1, keepdim=True).clamp(min=1e-6)
    rows = {}
    for k, lidar in lidar_by_pass.items():
        for oi, off in enumerate(OFFSETS_M):
            for prior_name, prior in (('gate', 0.01), ('cov', 1.0)):
                perception.reset_idx(torch.arange(n, device=task.device))
                tr.state[:, :3] = target + off * unit
                tr.state[:, 3:] = 0.0
                tr.cov[:] = torch.eye(6, device=task.device) * prior
                tr.active[:] = True
                tr.age[:] = 1.0
                before = tr.cov[:, :3, :3].clone()
                valid, conf, measured, bearing = perception._associate_lidar_target(
                    lidar, drone, q, torch.zeros(n, dtype=torch.bool, device=task.device))
                key = '%s__off%d__%s' % (k, oi, prior_name)
                rows[key + '__valid'] = host(valid)
                rows[key + '__measured'] = host(measured)
                rows[key + '__cov_before'] = host(before)
                rows[key + '__cov_after'] = host(tr.cov[:, :3, :3])
                rows[key + '__state_after'] = host(tr.state[:, :3])
                rows[key + '__primed'] = host(target + off * unit)
                assoc = getattr(perception, 'last_s_l1c_association', None)
                if assoc is not None:
                    for name in ('centre_range', 'row', 'column', 'extended', 'eligible_any', 'conclusive', 'gate'):
                        rows[key + '__' + name] = host(assoc[name])
    perception.reset_idx(torch.arange(n, device=task.device))
    rows['target_world'] = host(target)
    rows['drone_world'] = host(drone)
    return rows


def run_sensors(spec, out):
    eq.PLAN = copy.deepcopy(eq.PLAN)
    eq.PLAN['D2']['scenes'].extend(copy.deepcopy(S_L1A_PLAN['extra_scenes']) + copy.deepcopy(S_L1B_PLAN['extra_scenes']))
    ctx, lidar_by_pass = {}, {}
    original_build = eq.build_task

    def build(n, seed):
        task = original_build(n, seed)
        ctx['task'] = task
        perception = task.perception
        original_fuse = perception._fuse_static_and_extract_obstacles

        def fuse(*args, **kwargs):
            ctx.setdefault('calls', 0)
            lidar_by_pass['pass%d' % ctx['calls']] = args[0].clone()
            ctx['calls'] += 1
            return original_fuse(*args, **kwargs)

        perception._fuse_static_and_extract_obstacles = fuse
        return task

    eq.build_task = build
    try:
        files, bad = eq.run_d2(spec, out)
    finally:
        eq.build_task = original_build
    if len(lidar_by_pass) != 6:
        raise RuntimeError('Expected 3 attitudes x 2 scene/alone passes')
    task = ctx['task']
    drone = torch.tensor([eq.PLAN['D2']['drone_env_m']] * spec['num_envs'], dtype=torch.float32, device=task.device)
    scene_passes = {k: v for k, v in lidar_by_pass.items() if int(k[4:]) % 2 == 0}
    probes = association_probes(task, scene_passes, drone)
    probes.update({'lidar_' + k: host(v) for k, v in lidar_by_pass.items()})
    bad += eq.nan_report({k: v for k, v in probes.items() if v.dtype.kind == 'f'})
    files.append(tel.write_npz(out / 'association_probes.npz', probes))
    perception = task.perception
    meta = {'mode': spec['mode'], 'lineage_flag': os.environ.get('NAVRL_SENSOR_LINEAGE'),
            'perception_s_l1a': bool(perception.sensor_lineage_s_l1a),
            'perception_s_l1b': bool(perception.sensor_lineage_s_l1b),
            'perception_s_l1c': bool(getattr(perception, 'sensor_lineage_s_l1c', False)),
            'lidar_angles': host(perception._lidar_angles).tolist(),
            'lidar_max_range': float(perception.lidar_max_range), 'target_radius': float(perception.target_radius),
            'lidar_assoc_gate_m': float(perception.lidar_assoc_gate_m), 'step_dt': float(perception.step_dt),
            'offsets_m': list(OFFSETS_M), 'scene_pass_keys': sorted(scene_passes),
            'peak_allocated_bytes': int(torch.cuda.max_memory_allocated())}
    files.append(tel.write_json(out / 's_l1c_sensor_meta.json', meta))
    return files, bad


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--process', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args(argv)
    spec = process_spec(args.process)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    t0 = time.time()
    measured = {}
    if spec['batch'] == 'SENSORS':
        files, bad = run_sensors(spec, out)
    else:
        original_build = eq.build_player
        trace = {'identity_age': [], 'update_source': [], 'camera_visible': [], 'lidar_visible': [], 'track_age': []}

        def build(*args, **kwargs):
            result = original_build(*args, **kwargs)
            task = eq.find_task(result[0])
            perception = task.perception
            measured['actor_obs_dim'] = int(task.task_obs['observations'].shape[1])
            measured['perception_s_l1a'] = bool(perception.sensor_lineage_s_l1a)
            measured['perception_s_l1b'] = bool(perception.sensor_lineage_s_l1b)
            measured['perception_s_l1c'] = bool(getattr(perception, 'sensor_lineage_s_l1c', False))
            measured['detector_lineage_enabled'] = bool(task.detector.sensor_lineage_s_l1a)
            original_observe = perception.observe

            def observe(*a, **k):
                structured, diag = original_observe(*a, **k)
                # Evaluator-only trace: copies of diagnostics, never written back.
                trace['camera_visible'].append(host(diag['camera_visible']))
                trace['lidar_visible'].append(host(diag['lidar_visible']))
                trace['track_age'].append(host(diag['track_age']))
                if 's_l1c_identity_age' in diag:
                    trace['identity_age'].append(host(diag['s_l1c_identity_age']))
                    trace['update_source'].append(host(diag['s_l1c_update_source']))
                return structured, diag

            perception.observe = observe
            return result

        eq.build_player = build
        try:
            files, bad = el.run_v3(spec, out)
        finally:
            eq.build_player = original_build
        arrays = {k: np.stack(v) for k, v in trace.items() if v}
        files.append(tel.write_npz(out / 's_l1c_association_trace.npz', arrays))
    tel.write_json(out / 'process_done.json', {'process': spec['id'], 'process_spec': spec,
        'files': files, 'nan_fields': bad, 'wall_s': time.time() - t0,
        'peak_allocated_bytes': int(torch.cuda.max_memory_allocated()), **measured})
    return 3 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
