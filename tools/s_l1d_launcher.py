#!/usr/bin/env python3
"""S-L1D fixture/closed-loop subprocess. Run only through the watchdogged orchestrator."""
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
from motar_eval import e_l0, eq1d_telemetry as tel  # noqa: E402

PLAN = json.loads((ROOT / 'docs/plans/s_l1d_v1.json').read_text())
S_L1A_PLAN = json.loads((ROOT / 'docs/plans/s_l1a_v1.json').read_text())
S_L1B_PLAN = json.loads((ROOT / 'docs/plans/s_l1b_v1.json').read_text())
SENSOR_MODES = ('s_l1c', 's_l1d')
VARIANTS = ('off', 's_l1c', 's_l1d_p1', 's_l1d_p2')


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
                        'arm': 'P1' if variant == 's_l1d_p1' else 'P2',
                        'num_envs': run['closed_loop_num_envs'], 'steps': run['closed_loop_steps'],
                        'seed': run['closed_loop_seed_by_config'][config]}
    raise ValueError('Unknown process ' + pid)


def host(tensor):
    return tensor.detach().cpu().numpy().copy()


def run_sensors(spec, out):
    """EQ-1D scenes + S-L1A/S-L1B extras. Per pass: delivered depth, camera result and evaluator
    (E-L0) classes; afterwards primed true-centre validity probes (fixture input only)."""
    eq.PLAN = copy.deepcopy(eq.PLAN)
    eq.PLAN['D2']['scenes'].extend(copy.deepcopy(S_L1A_PLAN['extra_scenes']) + copy.deepcopy(S_L1B_PLAN['extra_scenes']))
    ctx, passes = {'camera_visible': None}, []
    original_build = eq.build_task

    def build(n, seed):
        task = original_build(n, seed)
        ctx['task'] = task
        perception = task.perception
        original_detect = perception._detect_rgbd

        def detect(*args, **kwargs):
            result = original_detect(*args, **kwargs)
            ctx['camera_visible'] = result[3].clone()
            return result

        perception._detect_rgbd = detect
        original_fuse = perception._fuse_static_and_extract_obstacles

        def fuse(*args, **kwargs):
            sil = e_l0.silhouette_from_task(task)
            passes.append({'depth': args[1].clone(), 'camera_visible': ctx['camera_visible'].clone(),
                           'el0_class': host(sil['cls']), 'el0_alone_px': host(sil['alone_px']),
                           'el0_visible_px': host(sil['visible_px'])})
            return original_fuse(*args, **kwargs)

        perception._fuse_static_and_extract_obstacles = fuse
        return task

    eq.build_task = build
    try:
        files, bad = eq.run_d2(spec, out)
    finally:
        eq.build_task = original_build
    if len(passes) != 6:
        raise RuntimeError('Expected 3 attitudes x 2 scene/alone passes')
    task = ctx['task']
    perception = task.perception
    n = spec['num_envs']
    drone = torch.tensor([eq.PLAN['D2']['drone_env_m']] * n, dtype=torch.float32, device=task.device)
    q = torch.tensor([[0., 0., 0., 1.]] * n, device=task.device)
    present = torch.as_tensor(np.asarray(task.target_position[:, 0].abs().cpu().numpy() < 400.0), device=task.device)
    arrays = {}
    for k, rec in enumerate(passes):
        for key in ('el0_class', 'el0_alone_px', 'el0_visible_px'):
            arrays['pass%d__%s' % (k, key)] = rec[key]
        arrays['pass%d__camera_visible' % k] = host(rec['camera_visible'])
        if getattr(perception, 'sensor_lineage_s_l1d', False):
            tr = perception.tracker
            perception.reset_idx(torch.arange(n, device=task.device))
            tr.state[:, :3] = task.target_position
            tr.state[:, 3:] = 0.0
            tr.cov[:] = torch.eye(6, device=task.device) * 1e-6
            tr.active[:] = present
            v = perception._s_l1d_validity(rec['depth'], rec['camera_visible'], drone, q)
            for key in ('code', 'support_px', 'occluded_px', 'centre_occluded', 'tau_m', 'in_frustum'):
                arrays['pass%d__online_%s' % (k, key)] = host(v[key])
            perception.reset_idx(torch.arange(n, device=task.device))
    arrays['target_world'] = host(task.target_position)
    arrays['target_present'] = host(present)
    arrays['drone_world'] = host(drone)
    bad += eq.nan_report({k: v for k, v in arrays.items() if v.dtype.kind == 'f'})
    files.append(tel.write_npz(out / 'validity_probes.npz', arrays))
    meta = {'mode': spec['mode'], 'lineage_flag': os.environ.get('NAVRL_SENSOR_LINEAGE'),
            'perception_s_l1c': bool(getattr(perception, 'sensor_lineage_s_l1c', False)),
            'perception_s_l1d': bool(getattr(perception, 'sensor_lineage_s_l1d', False)),
            'min_target_pixels': int(perception.min_pixels), 'camera_far_m': float(perception.max_camera_range),
            'target_radius': float(perception.target_radius), 'camera_offset': host(perception.camera_offset.view(3)).tolist(),
            'el0_classes': list(e_l0.CLASSES), 'peak_allocated_bytes': int(torch.cuda.max_memory_allocated())}
    files.append(tel.write_json(out / 's_l1d_sensor_meta.json', meta))
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
        trace = {}

        def build(*args, **kwargs):
            result = original_build(*args, **kwargs)
            task = eq.find_task(result[0])
            perception = task.perception
            measured['actor_obs_dim'] = int(task.task_obs['observations'].shape[1])
            for name in ('s_l1a', 's_l1b', 's_l1c', 's_l1d'):
                measured['perception_' + name] = bool(getattr(perception, 'sensor_lineage_' + name, False))
            measured['detector_lineage_enabled'] = bool(task.detector.sensor_lineage_s_l1a)
            original_observe = perception.observe

            def observe(*a, **k):
                structured, diag = original_observe(*a, **k)
                # Evaluator-only trace: copies of diagnostics plus GT labels, never written back.
                if 's_l1d_online_class' in diag:
                    sil = e_l0.silhouette_from_task(task)
                    row = {'online_class': diag['s_l1d_online_class'], 'update_source': diag['s_l1c_update_source'],
                           'identity_age': diag['s_l1c_identity_age'], 'camera_visible': diag['camera_visible'],
                           'lidar_visible': diag['lidar_visible'], 'track_active': task.perception.tracker.active,
                           'el0_class': sil['cls'],
                           'camera_gt_error_m': (diag['camera_measurement_world'] - task.target_position).norm(dim=1)}
                    for key, value in row.items():
                        trace.setdefault(key, []).append(host(value))
                return structured, diag

            perception.observe = observe
            return result

        eq.build_player = build
        try:
            files, bad = el.run_v3(spec, out)
        finally:
            eq.build_player = original_build
        if trace:
            files.append(tel.write_npz(out / 's_l1d_validity_trace.npz', {k: np.stack(v) for k, v in trace.items()}))
    tel.write_json(out / 'process_done.json', {'process': spec['id'], 'process_spec': spec,
        'files': files, 'nan_fields': bad, 'wall_s': time.time() - t0,
        'peak_allocated_bytes': int(torch.cuda.max_memory_allocated()), **measured})
    return 3 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
