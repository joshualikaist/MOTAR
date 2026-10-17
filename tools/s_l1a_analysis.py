#!/usr/bin/env python3
"""Independent CPU analysis of S-L1A raw sensor frames and closed-loop digests.

No production sensor or simulator import. Original raw is never modified.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools/audits'))
import s_l1a_reference as ref  # noqa: E402

DIGEST_KEYS = ('obs', 'action', 'reward', 'done', 'captured', 'crashed', 'pursuer', 'target',
               'resets', 'rgb', 'depth', 'lidar', 'semantic', 'tracker')
PASSES = ('level_scene', 'level_alone', 'roll15_scene', 'roll15_alone', 'pitch15_scene', 'pitch15_alone')


def pixel_summary(observed, expected, mask):
    observed, expected, mask = np.asarray(observed), np.asarray(expected), np.asarray(mask, bool)
    if observed.shape != expected.shape or mask.shape != expected.shape:
        raise ValueError('Pixel shapes differ')
    if not np.isfinite(observed).all() or not np.isfinite(expected).all():
        raise ValueError('Nonfinite depth')
    error = np.abs(observed[mask].astype(np.float64) - expected[mask])
    return {'n': int(error.size), 'max_m': float(error.max()) if error.size else None,
            'mean_m': float(error.mean()) if error.size else None,
            **{('p%d_m' % q): float(np.percentile(error, q)) if error.size else None for q in (50, 95, 99)},
            'over_1e_4': int((error > 1e-4).sum()), 'over_0p05': int((error > .05).sum())}


def mixed_reference(centers=(), half_extents=(), *, origin=(.1, 0., .03), sphere_center=None,
                    sphere_radius=.15, width=160, height=90, obstacle_far_m=10., target_far_m=20.):
    """Preserve separate sensor far planes; obstacle no-return=10 must not hide a target at 15."""
    boxes = ref.render_reference(centers, half_extents, origin=origin, width=width, height=height,
                                 far_m=obstacle_far_m)
    full = ref.render_reference(centers, half_extents, origin=origin, width=width, height=height,
                                sphere_center=sphere_center, sphere_radius=sphere_radius, far_m=target_far_m)
    target = full['hit_class'] == ref.SPHERE
    expected = np.where(target, full['depth_m'], boxes['depth_m'])
    return {'depth_m': expected, 'target': target,
            'obstacle': (boxes['hit_class'] == ref.BOX) & ~target,
            'miss': (boxes['hit_class'] == ref.MISS) & ~target,
            'robust': boxes['robust_mask'] & full['robust_mask']}


def compare_steps(a, b):
    def read(path):
        with open(path) as fh:
            return [json.loads(line) for line in fh]
    left, right = read(a), read(b)
    difference = {key: 0 for key in DIGEST_KEYS}
    for x, y in zip(left, right):
        for key in DIGEST_KEYS:
            if key not in x or key not in y or x[key] != y[key]:
                difference[key] += 1
    length_ok = len(left) == len(right) and bool(left)
    step_ok = all(x.get('step') == i for rows in (left, right) for i, x in enumerate(rows))
    return {'steps_a': len(left), 'steps_b': len(right), 'differences': difference,
            'identical': length_ok and step_ok and not any(difference.values())}


def _arrays(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def analyse_sensors(run):
    raw = {}
    rows, summaries, resource = [], {}, {}
    references = {}
    for mode in ('legacy', 'depth', 'full'):
        directory = Path(run) / ('sensors_' + mode)
        meta = json.loads((directory / 'd2_meta.json').read_text())
        perf = json.loads((directory / 's_l1a_sensor_meta.json').read_text())
        z = _arrays(directory / 'd2_renders.npz')
        replay = _arrays(directory / 'carve_replays.npz')
        raw[mode] = (z, replay, meta)
        expected_path = 'fixed' if mode == 'full' else 'legacy'
        actual_path_checks = {
            'detector_flag': bool(perf['detector_lineage_enabled']) == (mode != 'legacy'),
            'perception_flag': bool(perf['perception_lineage_enabled']) == (mode == 'full'),
            'actual_carve_equals_intended': np.array_equal(replay['carve_actual'], replay['carve_' + expected_path]),
            'actual_scan_equals_intended': np.array_equal(replay['scan_actual'], replay['scan_' + expected_path]),
        }
        times = np.asarray(perf['render_cuda_ms'])
        resource[mode] = {'render_median_ms': float(np.median(times)),
                          'render_p95_ms': float(np.percentile(times, 95)),
                          **{k: perf[k] for k in ('allocated_bytes_after_warmup', 'reserved_bytes_after_warmup',
                                                  'peak_allocated_bytes', 'peak_reserved_bytes')},
                          'benchmark_scene': perf['benchmark_scene'], 'repeats': len(times)}
        masks, observations, expectations = [], [], []
        bar_erasures = {kind: 0 for kind in ('off', 'legacy', 'fixed')}
        bar_false_clear = {kind: 0 for kind in bar_erasures}
        target_retained = {kind: 0 for kind in bar_erasures}
        bars_n = targets_n = uncertain_carves = mask_mismatch = 0
        s14 = []
        for pass_index, pass_name in enumerate(PASSES):
            scene_mode = pass_name.endswith('_scene')
            for i, scene in enumerate(meta['scenes']):
                centers = [b['centre'] for b in meta['layout'][i]] if scene_mode else []
                halves = [b['half'] for b in meta['layout'][i]] if scene_mode else []
                drone = z['drone_env_m'][i].astype(float)
                eye = drone + np.asarray(meta['camera']['camera_offset'])[0]
                target = z['target_world'][i].astype(float) if z['target_present'][i] else None
                key = (scene, scene_mode)
                if key not in references:
                    references[key] = mixed_reference(centers, halves, origin=eye, sphere_center=target)
                expected = references[key]
                depth = z[pass_name + '__depth'][i]
                eligible = expected['robust'] & ~expected['target']
                row = {'mode': mode, 'pass': pass_name, 'scene': scene,
                       'all_pixels': pixel_summary(depth, expected['depth_m'], np.ones_like(eligible)),
                       'robust_non_target': pixel_summary(depth, expected['depth_m'], eligible),
                       'robust_target': pixel_summary(depth, expected['depth_m'], expected['robust'] & expected['target']),
                       'excluded_nonrobust': int((~expected['robust']).sum()),
                       'obstacle_hit_pixels': int((eligible & expected['obstacle']).sum()),
                       'no_hit_pixels': int((eligible & expected['miss']).sum()),
                       'false_clear_pixels': int((eligible & expected['obstacle'] & (depth > expected['depth_m'] + .05)).sum()),
                       'false_positive_pixels': int((eligible & expected['miss'] & (depth < 9.95)).sum())}
                rows.append(row)
                masks.append(eligible.ravel()); observations.append(depth.ravel()); expectations.append(expected['depth_m'].ravel())
                # Independent LiDAR first hit from BODY origin; target radius .20, not camera .15.
                angles = np.asarray(perf['lidar_angles'])
                elevations = np.radians([20., 10., 0., -10.])
                beam = np.stack(np.broadcast_arrays(np.cos(elevations[:, None])*np.cos(angles),
                    np.cos(elevations[:, None])*np.sin(angles), np.sin(elevations[:, None])+np.zeros_like(angles)), axis=-1)
                hits = ref.first_hits(drone, beam, centers, halves, sphere_center=target,
                                      sphere_radius=float(meta['lidar']['target_radius']), far_m=12.)
                labels = hits['hit_class']
                lidar = replay['lidar'][pass_index, i].reshape(4, 72)
                actual_sem = z[pass_name + '__seg_segmentation_pixels'][i].reshape(4, 72)
                semantic_classes = np.where(actual_sem == 50, ref.SPHERE, np.where(actual_sem >= 0, ref.BOX, ref.MISS))
                labels_agree = labels == semantic_classes
                valid_ray = labels_agree & (np.abs(hits['depth_m'] - lidar) <= 1e-4)
                bar = (labels == ref.BOX) & valid_ray
                tgt = (labels == ref.SPHERE) & valid_ray
                bars_n += int(bar.sum()); targets_n += int(tgt.sum())
                expected_masks = ref.carve_masks(lidar[None], angles, replay['depth'][pass_index, i:i+1],
                    replay['pixels'][pass_index, i:i+1].astype(bool),
                    (replay['visible'][pass_index, i:i+1] & replay['camera_visible'][pass_index, i:i+1]).astype(bool), replay['bearing'][pass_index, i:i+1],
                    min_target_pixels=perf['min_target_pixels'], target_radius=perf['target_radius'], camera_far_m=perf['camera_far_m'])
                fixed_mask = replay['carve_fixed'][pass_index, i]
                mask_mismatch += int((fixed_mask != expected_masks['lidar_carve_mask'][0]).sum())
                uncertain_carves += int(fixed_mask.sum()) if not expected_masks['supported'][0] else 0
                counts = {}
                for kind in bar_erasures:
                    carved = replay['carve_' + kind][pass_index, i]
                    scan = replay['scan_' + kind][pass_index, i]
                    erased = int((carved & bar).sum())
                    false_clear = int((bar & (scan > lidar + 1e-4)).sum())
                    retained = int((tgt & (scan <= lidar + 1e-4)).sum())
                    bar_erasures[kind] += erased; bar_false_clear[kind] += false_clear; target_retained[kind] += retained
                    counts[kind] = {'bar_carved': erased, 'bar_false_clear': false_clear, 'target_retained': retained}
                row['carve'] = {'bar_beams': int(bar.sum()), 'target_beams': int(tgt.sum()),
                                'oracle_or_semantic_mismatch_beams': int((~valid_ray).sum()), 'modes': counts,
                                'el0_class': int(replay['el0_class'][pass_index, i]),
                                'el0_camera_provenance': int(replay['el0_camera_provenance'][pass_index, i])}
                if scene_mode and scene in ('S14', 'A_FOREGROUND'):
                    s14.append({'pass': pass_name, 'scene': scene, 'bar_beams': int(bar.sum()), **counts})
        aggregate = pixel_summary(np.concatenate(observations), np.concatenate(expectations), np.concatenate(masks))
        selected = [row for row in rows if row['mode'] == mode]
        summaries[mode] = {'robust_non_target': aggregate,
            'false_clear_pixels': sum(row['false_clear_pixels'] for row in selected),
            'false_positive_pixels': sum(row['false_positive_pixels'] for row in selected),
            'bar_beams': bars_n, 'target_beams': targets_n, 'bar_erasures': bar_erasures,
            'bar_false_clear': bar_false_clear, 'target_retained': target_retained,
            'uncertain_carves': uncertain_carves, 'independent_carve_mask_mismatches': mask_mismatch,
            'actual_path_checks': actual_path_checks, 'foreground': s14}
    # Same scenes/initialisation, before closed-loop interventions. No cross-trajectory sensor equality claim.
    equality = {}
    for mode in ('depth', 'full'):
        equality[mode] = {key: all(np.array_equal(raw['legacy'][0][p+'__'+key], raw[mode][0][p+'__'+key])
                                   for p in PASSES) for key in ('rgb', 'lidar_m', 'obstacle_depth_native', 'target_mask')}
        delta = np.concatenate([(raw[mode][0][p+'__depth'] - raw['legacy'][0][p+'__depth']).ravel() for p in PASSES])
        equality[mode]['depth_distribution'] = {'pixels': int(delta.size), 'changed': int((delta != 0).sum()),
            'mean_abs_delta_m': float(np.abs(delta).mean()), 'max_abs_delta_m': float(np.abs(delta).max())}
        resource[mode]['delta_allocated_bytes'] = resource[mode]['allocated_bytes_after_warmup'] - resource['legacy']['allocated_bytes_after_warmup']
        resource[mode]['delta_render_median_ms'] = resource[mode]['render_median_ms'] - resource['legacy']['render_median_ms']
    return {'summaries': summaries, 'per_scene': rows, 'resources': resource, 'paired_input_checks': equality}


def analyse_run(run):
    run = Path(run).resolve()
    report = json.loads((run / 'run_report.json').read_text())
    sensors = analyse_sensors(run)
    noninterference, on_changes = {}, {}
    historical = (ROOT / '../aerial_gym_simulator/results/e_l0_evaluation_v1/EL0-R1').resolve()
    for config in ('static', 'moving'):
        step = lambda name: run / (config + '_' + name) / 'v3_steps.jsonl'
        noninterference[config] = {
            'unset_vs_off': compare_steps(step('unset'), step('off')),
            'off_vs_historical': compare_steps(step('off'), historical / ('v3_' + config + '_p2') / 'v3_steps.jsonl'),
            'on_el0_off_vs_on': compare_steps(step('on_p1'), step('on_p2'))}
        on_changes[config] = compare_steps(step('off'), step('on_p2'))
        old = _arrays(run / (config + '_off') / 'v3_trajectory.npz')
        new = _arrays(run / (config + '_on_p2') / 'v3_trajectory.npz')
        on_changes[config]['position_delta_max_m'] = float(np.linalg.norm(new['position'] - old['position'], axis=-1).max())
        on_changes[config]['action_delta_max'] = float(np.abs(new['action'] - old['action']).max())
        on_changes[config]['episodes'] = {mode: {key: int(z[key].sum()) for key in ('captured', 'crashed')}
                                          for mode, z in (('off', old), ('on', new))}
    summary = sensors['summaries']['full']
    depth_pass = summary['robust_non_target']['n'] > 0 and summary['robust_non_target']['over_1e_4'] == 0 and summary['false_clear_pixels'] == summary['false_positive_pixels'] == 0
    edge_rows = [r for r in sensors['per_scene'] if r['mode'] == 'full' and r['scene'].startswith('A_') and r['pass'].endswith('_scene')]
    edge_pass = bool(edge_rows) and all(r['robust_non_target']['n'] > 0 and r['obstacle_hit_pixels'] > 0 and r['robust_non_target']['over_1e_4'] == 0 and not r['false_clear_pixels'] and not r['false_positive_pixels'] for r in edge_rows)
    foreground = summary['foreground']
    carve_pass = bool(foreground) and all(r['bar_beams'] > 0 and r['fixed']['bar_carved'] == r['fixed']['bar_false_clear'] == 0 for r in foreground) and summary['uncertain_carves'] == summary['independent_carve_mask_mismatches'] == 0
    off_pass = all(v[k]['identical'] for v in noninterference.values() for k in ('unset_vs_off', 'off_vs_historical'))
    el0_pass = all(v['on_el0_off_vs_on']['identical'] for v in noninterference.values())
    paths_pass = all(all(row['actual_path_checks'].values()) for row in sensors['summaries'].values())
    gates = {'A-DEPTH': depth_pass, 'A-BOUNDARY': edge_pass, 'A-CARVE': carve_pass,
             'A-OFF': off_pass, 'A-EL0': el0_pass, 'A-INTEGRITY': report.get('status') == 'COMPLETED' and paths_pass}
    return {'schema': 'motar.s-l1a-analysis.v1', 'run': str(run), 'source_sha': report.get('source_sha', report.get('preflight', {}).get('head')),
        'gates': {k: 'PASS' if v else 'FAIL' for k, v in gates.items()},
        'decision': 'S-L1A_QUALIFIED' if all(gates.values()) else 'S-L1A_FAIL',
        'sensors': sensors, 'noninterference': noninterference, 'on_behavior_changes': on_changes,
        'known_limits': ['Constructed fixture counts are not population rates.',
            'Conservative measurement inequality is not a universal foreground/identity guarantee; Camera-LiDAR origin/radius mismatch remains.',
            'S-L1B pinhole/SE(3), S-L1C identity/range-only covariance and S-L1D silhouette/validity are not corrected.',
            'Legacy target renderer float32 residual reported separately; hardware realism unqualified.',
            'ON changes sensor input distribution; legacy Policy F results measure robustness, not algorithm improvement.']}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True); p.add_argument('--out', required=True)
    a = p.parse_args(argv)
    result = analyse_run(a.run)
    out = Path(a.out); out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as fh:
        json.dump(result, fh, indent=2); fh.write('\n')
    print(json.dumps({'decision': result['decision'], 'gates': result['gates'], 'out': str(out)}, indent=2))


if __name__ == '__main__':
    main()
