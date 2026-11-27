#!/usr/bin/env python3
"""S-L1A fixture/closed-loop subprocess. Run only through the watchdogged orchestrator."""
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

PLAN = json.loads((ROOT / 'docs/plans/s_l1a_v1.json').read_text())
AMENDMENT = json.loads((ROOT / 'docs/plans/s_l1a_v1_amendment1.json').read_text())


def process_spec(pid):
    if pid.startswith('sensors_'):
        mode = pid.split('_', 1)[1]
        if mode not in ('legacy', 'depth', 'full'):
            raise ValueError('Unknown sensor mode')
        return {'id': pid, 'batch': 'SENSORS', 'mode': mode, 'num_envs': 36, 'seed': 4401}
    for config in ('static', 'moving'):
        for variant in ('unset', 'off', 'on_p1', 'on_p2'):
            if pid == config + '_' + variant:
                return {'id': pid, 'batch': 'V3', 'config': config, 'variant': variant,
                        'arm': 'P1' if variant == 'on_p1' else 'P2',
                        'num_envs': PLAN['run']['closed_loop_num_envs'],
                        'steps': PLAN['run']['closed_loop_steps'],
                        'seed': AMENDMENT['overrides']['run']['closed_loop_seed_by_config'][config]}
    raise ValueError('Unknown process ' + pid)


def host(tensor):
    return tensor.detach().cpu().numpy().copy()


def run_sensors(spec, out):
    """Reuse immutable EQ-1D scene construction. Extra hooks only observe or replay fixed frames.

    Counterfactual carve replay never supplies GT to perception. Semantic/provenance labels are
    computed afterwards for evaluation. The intended path is called last to restore diagnostics.
    """
    eq.PLAN = copy.deepcopy(eq.PLAN)
    eq.PLAN['D2']['scenes'].extend(copy.deepcopy(PLAN['extra_scenes']))
    ctx, captures = {}, []
    original_build = eq.build_task

    def build(n, seed):
        task = original_build(n, seed)
        ctx['task'] = task
        perception = task.perception
        if spec['mode'] == 'depth':
            perception.sensor_lineage_s_l1a = False  # declared depth-only diagnostic
        original_detect = perception._detect_rgbd

        def detect(*args, **kwargs):
            result = original_detect(*args, **kwargs)
            ctx['camera_visible'] = result[3].clone()
            return result

        perception._detect_rgbd = detect
        original_fuse = perception._fuse_static_and_extract_obstacles

        def fuse(*args, **kwargs):
            enabled = perception.sensor_lineage_s_l1a
            lidar, depth, pixels, surface, bearing, visible = args[:6]
            current_camera = kwargs.get('camera_visible', ctx['camera_visible'])
            rec = {'lidar': host(lidar), 'depth': host(depth), 'pixels': host(pixels),
                   'surface': host(surface), 'bearing': host(bearing),
                   'visible': host(visible), 'camera_visible': host(current_camera)}
            try:
                for name, mode in (('off', False), ('legacy', False), ('fixed', True)):
                    perception.sensor_lineage_s_l1a = mode
                    replay = list(args)
                    replay_kw = dict(kwargs)
                    if name == 'off':
                        replay[2] = torch.zeros_like(pixels)
                        replay[5] = torch.zeros_like(visible)
                        replay_kw['camera_visible'] = torch.zeros_like(current_camera)
                    else:
                        replay_kw['camera_visible'] = current_camera
                    static, _ = original_fuse(*replay, **replay_kw)
                    rec['scan_' + name] = host(static.view(lidar.shape[0], -1, perception._lidar_angles.numel()) * perception.lidar_max_range)
                    rec['carve_' + name] = host(perception.last_target_like)
                # Actual fixed-frame sensor/semantic labels are evaluator-only.
                sil = e_l0.silhouette_from_task(task)
                rec['el0_class'] = host(sil['cls'])
                rec['el0_alone_px'] = host(sil['alone_px'])
                rec['el0_visible_px'] = host(sil['visible_px'])
                rec['el0_camera_provenance'] = host(e_l0.camera_provenance(
                    current_camera.bool(), sil['visible_px'], int(perception.min_pixels)))
            finally:
                perception.sensor_lineage_s_l1a = enabled
            result = original_fuse(*args, **kwargs)
            rec['scan_actual'] = host(result[0].view(lidar.shape[0], -1, perception._lidar_angles.numel()) * perception.lidar_max_range)
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
    arrays = {key: np.stack([row[key] for row in captures]) for key in captures[0]}
    if len(captures) != 6:
        raise RuntimeError('Expected 3 attitudes x 2 scene/alone passes')
    bad += eq.nan_report(arrays)
    files.append(tel.write_npz(out / 'carve_replays.npz', arrays))
    task = ctx['task']
    detector = task.detector
    drone = torch.tensor([[20., 20., 1.]] * 36, device=task.device)
    q = task.obs_dict['robot_vehicle_orientation'].clone()
    # Benchmark render_raw_rgbd including the unchanged native query and target overlay.
    # Replays above do not change geometry. Last EQ-1D pass is target-alone; state is recorded.
    for _ in range(10):
        detector.render_raw_rgbd(drone, q, task.target_position, task.target_orientation)
    torch.cuda.synchronize()
    allocated = int(torch.cuda.memory_allocated())
    reserved = int(torch.cuda.memory_reserved())
    times = []
    for _ in range(50):
        start, stop = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
        start.record()
        detector.render_raw_rgbd(drone, q, task.target_position, task.target_orientation)
        stop.record()
        stop.synchronize()
        times.append(float(start.elapsed_time(stop)))
    meta = {'mode': spec['mode'], 'benchmark_scene': 'last EQ-1D alone pass; pitch15 body / yaw-only sensor',
            'warmups': 10, 'repeats': 50, 'render_cuda_ms': times,
            'allocated_bytes_after_warmup': allocated, 'reserved_bytes_after_warmup': reserved,
            'peak_allocated_bytes': int(torch.cuda.max_memory_allocated()),
            'peak_reserved_bytes': int(torch.cuda.max_memory_reserved()),
            'lidar_angles': host(task.perception._lidar_angles).tolist(),
            'camera_far_m': float(task.perception.max_camera_range),
            'min_target_pixels': int(task.perception.min_pixels),
            'target_radius': float(task.perception.target_radius),
            'lineage_flag': os.environ.get('NAVRL_SENSOR_LINEAGE'),
            'perception_lineage_enabled': bool(task.perception.sensor_lineage_s_l1a),
            'detector_lineage_enabled': bool(task.detector.sensor_lineage_s_l1a)}
    files.append(tel.write_json(out / 's_l1a_sensor_meta.json', meta))
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
            measured['perception_lineage_enabled'] = bool(task.perception.sensor_lineage_s_l1a)
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
