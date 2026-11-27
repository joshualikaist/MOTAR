#!/usr/bin/env python3
"""S-L1B fixture/closed-loop subprocess. Run only through the watchdogged orchestrator."""
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

PLAN = json.loads((ROOT / 'docs/plans/s_l1b_v1.json').read_text())
S_L1A_PLAN = json.loads((ROOT / 'docs/plans/s_l1a_v1.json').read_text())
SENSOR_MODES = ('legacy', 's_l1a', 's_l1b')
VARIANTS = ('off', 's_l1a', 's_l1b_p1', 's_l1b_p2')


def process_spec(pid):
    run = PLAN['run']
    if pid.startswith('sensors_'):
        mode = pid.split('_', 1)[1]
        if mode not in SENSOR_MODES:
            raise ValueError('Unknown sensor mode')
        return {'id': pid, 'batch': 'SENSORS', 'mode': mode, 'num_envs': run['sensor_num_envs'],
                'seed': run['sensor_seed']}
    for config in ('static', 'moving'):
        for variant in VARIANTS:
            if pid == config + '_' + variant:
                return {'id': pid, 'batch': 'V3', 'config': config, 'variant': variant,
                        'arm': 'P1' if variant == 's_l1b_p1' else 'P2',
                        'num_envs': run['closed_loop_num_envs'], 'steps': run['closed_loop_steps'],
                        'seed': run['closed_loop_seed_by_config'][config]}
    raise ValueError('Unknown process ' + pid)


def host(tensor):
    return tensor.detach().cpu().numpy().copy()


def run_sensors(spec, out):
    """Reuse immutable EQ-1D scene construction with the S-L1A and S-L1B extra scenes.

    In the s_l1b process the fusion is replayed on identical delivered inputs under the legacy,
    S-L1A and S-L1B flags (the calibration tensors exist only there); the intended path runs
    last. Replays never supply GT to perception; E-L0 labels are evaluator-only.
    """
    eq.PLAN = copy.deepcopy(eq.PLAN)
    eq.PLAN['D2']['scenes'].extend(copy.deepcopy(S_L1A_PLAN['extra_scenes']) + copy.deepcopy(PLAN['extra_scenes']))
    ctx, captures = {}, []
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
            flags = (perception.sensor_lineage_s_l1a, perception.sensor_lineage_s_l1b)
            lidar, depth, pixels, surface, bearing, visible = args[:6]
            camera = kwargs.get('camera_visible', ctx['camera_visible'])
            shape = (lidar.shape[0], -1, perception._lidar_angles.numel())
            rec = {'lidar': host(lidar), 'depth': host(depth), 'pixels': host(pixels), 'surface': host(surface),
                   'bearing': host(bearing), 'visible': host(visible), 'camera_visible': host(camera)}
            try:
                if flags[1]:
                    for name, fa, fb in (('legacy', False, False), ('s_l1a', True, False), ('s_l1b', True, True)):
                        perception.sensor_lineage_s_l1a, perception.sensor_lineage_s_l1b = fa, fb
                        replay_kw = dict(kwargs)
                        replay_kw['camera_visible'] = camera
                        static, _ = original_fuse(*args, **replay_kw)
                        rec['scan_' + name] = host(static.view(shape) * perception.lidar_max_range)
                        rec['carve_' + name] = host(perception.last_target_like)
                        if fb:
                            points = perception.last_s_l1b_points
                            for key in ('rho', 'az', 'el', 'valid'):
                                rec['pt_' + key] = host(points[key])
                            rec['bins_clamped'] = host(perception.last_s_l1b_envelope)
                            rec['bins_strict'] = host(perception.last_s_l1b_strict)
                sil = e_l0.silhouette_from_task(task)
                rec['el0_class'] = host(sil['cls'])
                rec['el0_visible_px'] = host(sil['visible_px'])
                rec['el0_camera_provenance'] = host(e_l0.camera_provenance(
                    camera.bool(), sil['visible_px'], int(perception.min_pixels)))
            finally:
                perception.sensor_lineage_s_l1a, perception.sensor_lineage_s_l1b = flags
            result = original_fuse(*args, **kwargs)
            rec['scan_actual'] = host(result[0].view(shape) * perception.lidar_max_range)
            rec['carve_actual'] = host(perception.last_target_like)
            captures.append(rec)
            return result

        perception._fuse_static_and_extract_obstacles = fuse
        return task

    eq.build_task = build
    try:
        files, bad = eq.run_d2(spec, out)
    finally:
        eq.build_task = original_build
    if len(captures) != 6:
        raise RuntimeError('Expected 3 attitudes x 2 scene/alone passes')
    arrays = {key: np.stack([row[key] for row in captures]) for key in captures[0]}
    bad += eq.nan_report(arrays)
    files.append(tel.write_npz(out / 'fusion_replays.npz', arrays))
    task = ctx['task']
    perception = task.perception
    meta = {'mode': spec['mode'], 'lineage_flag': os.environ.get('NAVRL_SENSOR_LINEAGE'),
            'perception_s_l1a': bool(perception.sensor_lineage_s_l1a),
            'perception_s_l1b': bool(perception.sensor_lineage_s_l1b),
            'detector_lineage_enabled': bool(task.detector.sensor_lineage_s_l1a),
            'lidar_angles': host(perception._lidar_angles).tolist(),
            'camera_offset': host(perception.camera_offset.view(3)).tolist(),
            'camera_far_m': float(perception.max_camera_range),
            'camera_obstacle_far_m': float(perception.camera_obstacle_max_range),
            'lidar_max_range': float(perception.lidar_max_range),
            'min_target_pixels': int(perception.min_pixels), 'target_radius': float(perception.target_radius),
            'intrinsics': {'width': perception.width, 'height': perception.height, 'fx': perception.fx,
                           'fy': perception.fy, 'cx': perception.cx, 'cy': perception.cy}}
    if perception.sensor_lineage_s_l1b:
        meta['lidar_elevations_rad'] = host(perception._s_l1b_lidar_elevations).tolist()
        last = captures[-1]
        args = [torch.as_tensor(last[k], device=task.device) for k in
                ('lidar', 'depth', 'pixels', 'surface', 'bearing', 'visible')]
        camera = torch.as_tensor(last['camera_visible'], device=task.device)
        timings = {}
        for name, fb in (('s_l1a', False), ('s_l1b', True)):
            perception.sensor_lineage_s_l1b = fb
            for _ in range(10):
                perception._fuse_static_and_extract_obstacles(*args, camera_visible=camera)
            torch.cuda.synchronize()
            times = []
            for _ in range(50):
                start, stop = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
                start.record()
                perception._fuse_static_and_extract_obstacles(*args, camera_visible=camera)
                stop.record()
                stop.synchronize()
                times.append(float(start.elapsed_time(stop)))
            timings[name] = times
        perception.sensor_lineage_s_l1b = True
        meta['fusion_cuda_ms'] = timings
        meta['benchmark'] = 'last captured inputs (pitch15 alone pass), 10 warmups, 50 repeats'
    meta['peak_allocated_bytes'] = int(torch.cuda.max_memory_allocated())
    files.append(tel.write_json(out / 's_l1b_sensor_meta.json', meta))
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

        def build(*args, **kwargs):
            result = original_build(*args, **kwargs)
            task = eq.find_task(result[0])
            measured['actor_obs_dim'] = int(task.task_obs['observations'].shape[1])
            measured['perception_s_l1a'] = bool(task.perception.sensor_lineage_s_l1a)
            measured['perception_s_l1b'] = bool(task.perception.sensor_lineage_s_l1b)
            measured['detector_lineage_enabled'] = bool(task.detector.sensor_lineage_s_l1a)
            return result

        eq.build_player = build
        try:
            files, bad = el.run_v3(spec, out)
        finally:
            eq.build_player = original_build
    tel.write_json(out / 'process_done.json', {'process': spec['id'], 'process_spec': spec,
        'files': files, 'nan_fields': bad, 'wall_s': time.time() - t0,
        'peak_allocated_bytes': int(torch.cuda.max_memory_allocated()), **measured})
    return 3 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
