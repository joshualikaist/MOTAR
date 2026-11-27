#!/usr/bin/env python3
"""Independent CPU analysis of S-L1B raw sensor frames and closed-loop digests.

No production sensor or simulator import. Expected carve masks, reprojected points and bins are
recomputed in float64 from the recorded delivered inputs; physical truth comes from the analytic
oracle. Original raw is never modified.
"""
from __future__ import annotations

import argparse
import ast
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/audits'), str(ROOT / 'tools')]
import s_l1b_reference as rb  # noqa: E402
import s_l1a_analysis as aa  # noqa: E402

ra = rb.ref
PASSES = aa.PASSES
EL0_R1 = (ROOT / '../aerial_gym_simulator/results/e_l0_evaluation_v1/EL0-R1').resolve()
SL1A_C01 = (ROOT / '../aerial_gym_simulator/results/s_l1a_v1/SL1A-C01').resolve()
FOREGROUND = ('S14', 'A_FOREGROUND')


def _arrays(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def load_mode(run, mode):
    d = Path(run) / ('sensors_' + mode)
    return {'meta': json.loads((d / 'd2_meta.json').read_text()),
            'sensor': json.loads((d / 's_l1b_sensor_meta.json').read_text()),
            'z': _arrays(d / 'd2_renders.npz'), 'r': _arrays(d / 'fusion_replays.npz')}


def geometry(meta, z, i, scene_mode):
    drone = z['drone_env_m'][i].astype(float)
    centers = [list(np.asarray(b['centre']) - drone) for b in meta['layout'][i]] if scene_mode else []
    halves = [b['half'] for b in meta['layout'][i]] if scene_mode else []
    target = (z['target_world'][i].astype(float) - drone) if z['target_present'][i] else None
    return centers, halves, target


def lidar_truth(centers, halves, target, angles, radius):
    elev = np.radians([20., 10., 0., -10.])
    beams = np.stack(np.broadcast_arrays(np.cos(elev[:, None]) * np.cos(angles), np.cos(elev[:, None]) * np.sin(angles),
                                         np.sin(elev[:, None]) + 0 * angles), axis=-1)
    return ra.first_hits(np.zeros(3), beams, centers, halves, sphere_center=target, sphere_radius=radius, far_m=12.)


def angle_error(a, b):
    d = np.asarray(a, dtype=np.float64) - b
    return np.abs(np.arctan2(np.sin(d), np.cos(d)))


def analyse_sensors(run):
    data = {mode: load_mode(run, mode) for mode in ('legacy', 's_l1a', 's_l1b')}
    sb = data['s_l1b']
    meta, sensor, z, r = sb['meta'], sb['sensor'], sb['z'], sb['r']
    angles = np.asarray(sensor['lidar_angles'])
    offset = tuple(sensor['camera_offset'])
    far_l, far_c, far_o = sensor['lidar_max_range'], sensor['camera_far_m'], sensor['camera_obstacle_far_m']
    elevations = np.degrees(sensor['lidar_elevations_rad'])
    point = {'n': 0, 'rho_max': 0., 'ang_max': 0., 'over': 0, 'grazing_excluded': 0, 'invalid_box_pixels': 0}
    envelope = {'robust_bins': 0, 'mismatch': 0, 'max_err': 0., 'nonrobust_bins': 0, 'validity_mismatch': 0}
    fused = {'robust_bins': 0, 'mismatch': 0, 'max_err': 0.}
    vertical = {'middle_rows_identical': True, 'edge_rows_conservative': True, 'two_d_contains_points': True}
    carve = {'mask_mismatch': 0, 'uncertain': 0, 'bar_beams': 0, 'target_beams': 0,
             'bar_carved': {m: 0 for m in ('legacy', 's_l1a', 's_l1b')},
             'bar_false_clear': {m: 0 for m in ('legacy', 's_l1a', 's_l1b')},
             'target_retained': {m: 0 for m in ('legacy', 's_l1a', 's_l1b')}, 'foreground': []}
    beam_offsets, bins_report = [], []
    truths = {}
    for k, pass_name in enumerate(PASSES):
        scene_mode = pass_name.endswith('_scene')
        for i, scene in enumerate(meta['scenes']):
            centers, halves, target = geometry(meta, z, i, scene_mode)
            key = (scene, scene_mode)
            if key not in truths:
                truths[key] = (rb.true_surface(centers, halves, sphere_center=target, sphere_radius=rb.CAMERA_TARGET_RADIUS_M,
                                               translation=offset, far_m=far_c),
                               lidar_truth(centers, halves, target, angles, float(meta['lidar']['target_radius'])))
            truth, lt = truths[key]
            depth = r['depth'][k, i].astype(np.float64)
            lidar = r['lidar'][k, i].reshape(4, 72).astype(np.float64)
            confirmed = bool(r['visible'][k, i]) and bool(r['camera_visible'][k, i])
            masks = rb.common_frame_carve_masks(
                lidar[None], angles, depth[None], r['pixels'][k, i][None].astype(bool), np.array([confirmed]),
                r['bearing'][k, i:i + 1].astype(np.float64), translation=offset, min_target_pixels=sensor['min_target_pixels'],
                lidar_far_m=far_l, camera_far_m=far_c, lidar_elevations_deg=tuple(elevations))
            produced = r['carve_s_l1b'][k, i]
            carve['mask_mismatch'] += int((produced != masks['lidar_carve_mask'][0]).sum())
            carve['uncertain'] += int(produced.sum()) if not masks['supported'][0] else 0
            # Calibration points: production versus the analytic first-hit surface (B-POINT).
            pv = r['pt_valid'][k, i]
            box = truth['box'] & (truth['depth_m'] < far_o - 1e-3) & truth['robust_mask']
            sel = box & ~truth['grazing'] & pv
            point['grazing_excluded'] += int((box & truth['grazing']).sum())
            point['invalid_box_pixels'] += int((box & ~pv).sum())
            if sel.any():
                rho_e = np.abs(r['pt_rho'][k, i].astype(np.float64) - truth['rho'])[sel]
                ang_e = np.maximum(angle_error(r['pt_az'][k, i], truth['az']), angle_error(r['pt_el'][k, i], truth['el']))[sel]
                point['n'] += int(sel.sum())
                point['rho_max'] = max(point['rho_max'], float(rho_e.max()))
                point['ang_max'] = max(point['ang_max'], float(ang_e.max()))
                point['over'] += int(((rho_e > 1e-4) | (ang_e > 1e-5)).sum())
            # Envelope definition on the delivered depth (B-ENVELOPE) and fused scan.
            cam = rb.camera_points(depth[None], translation=offset, camera_far_m=far_o,
                                   blank=masks['camera_blank_mask'][0][None])
            envelope['validity_mismatch'] += int((cam['valid'][0] != pv).sum())
            for clamp, name in ((False, 'bins_strict'), (True, 'bins_clamped')):
                expected, robust = rb.robust_bins(cam, clamp=clamp, lidar_far_m=far_l)
                err = np.abs(r[name][k, i].astype(np.float64) - expected[0])[robust[0]]
                envelope['robust_bins'] += int(robust.sum())
                envelope['nonrobust_bins'] += int((~robust).sum())
                envelope['mismatch'] += int((err > 1e-4).sum())
                envelope['max_err'] = max(envelope['max_err'], float(err.max()) if err.size else 0.)
                if clamp:
                    after = np.where(masks['lidar_carve_mask'][0], far_l, lidar)
                    exp_scan = np.minimum(after, expected[0])
                    ferr = np.abs(r['scan_s_l1b'][k, i].astype(np.float64) - exp_scan)[robust[0]]
                    fused['robust_bins'] += int(robust.sum())
                    fused['mismatch'] += int((ferr > 1e-4).sum())
                    fused['max_err'] = max(fused['max_err'], float(ferr.max()))
            strict, clamped = r['bins_strict'][k, i], r['bins_clamped'][k, i]
            vertical['middle_rows_identical'] &= bool(np.array_equal(strict[1:3], clamped[1:3]))
            vertical['edge_rows_conservative'] &= bool((clamped[[0, 3]] <= strict[[0, 3]]).all())
            j = np.mod(np.rint((math.pi - r['pt_az'][k, i].astype(np.float64)) / (2 * math.pi / 72)), 72).astype(int)
            keep = pv & (r['pt_rho'][k, i] < far_l)
            two_d = clamped.min(axis=0)
            vertical['two_d_contains_points'] &= bool((two_d[j[keep]] <= r['pt_rho'][k, i][keep] + 1e-6).all())
            # Carve physics: LiDAR labels from the oracle, agreeing with the delivered semantics.
            actual_sem = z[pass_name + '__seg_segmentation_pixels'][i].reshape(4, 72)
            sem = np.where(actual_sem == 50, ra.SPHERE, np.where(actual_sem >= 0, ra.BOX, ra.MISS))
            valid_ray = (lt['hit_class'] == sem) & (np.abs(lt['depth_m'] - lidar) <= 1e-4)
            bar, tgt = (lt['hit_class'] == ra.BOX) & valid_ray, (lt['hit_class'] == ra.SPHERE) & valid_ray
            carve['bar_beams'] += int(bar.sum())
            carve['target_beams'] += int(tgt.sum())
            counts = {}
            for m in ('legacy', 's_l1a', 's_l1b'):
                carved, scan = r['carve_' + m][k, i], r['scan_' + m][k, i]
                counts[m] = {'bar_carved': int((carved & bar).sum()), 'bar_false_clear': int((bar & (scan > lidar + 1e-4)).sum()),
                             'target_retained': int((tgt & (scan <= lidar + 1e-4)).sum())}
                for name in ('bar_carved', 'bar_false_clear', 'target_retained'):
                    carve[name][m] += counts[m][name]
            if scene_mode and scene in FOREGROUND:
                carve['foreground'].append({'pass': pass_name, 'scene': scene, 'bar_beams': int(bar.sum()), **counts})
            # Not gated: in-FOV beam envelope minus the LiDAR beam truth (D2-F1 definition).
            inside = np.abs(angles) <= math.radians(43.5)
            hit = (lt['hit_class'] == ra.BOX) & inside[None, :] & (clamped < far_l)
            if hit.any():
                beam_offsets.append((clamped - lt['depth_m'])[hit])
            if scene.startswith('B_') and scene_mode:
                tp = truth['box'] & (truth['depth_m'] < far_o - 1e-3)
                tj, _ = rb.bin_index(truth['az'][tp], truth['el'][tp])
                truth_set = set(np.unique(tj).tolist())
                s_l1b_set = {int(x) for x in np.where((clamped < far_l).any(axis=0))[0]}
                leg = data['legacy']['r']
                legacy_set = {int(x) for x in np.where((leg['scan_actual'][k, i] < leg['lidar'][k, i].reshape(4, 72) - 1e-6).any(axis=0))[0]}
                bins_report.append({'scene': scene, 'pass': pass_name, 'truth_bins': sorted(truth_set),
                                    's_l1b_bins': sorted(s_l1b_set), 'legacy_camera_bins': sorted(legacy_set),
                                    's_l1b_missing': len(truth_set - s_l1b_set), 's_l1b_extra': len(s_l1b_set - truth_set),
                                    'legacy_missing': len(truth_set - legacy_set), 'legacy_extra': len(legacy_set - truth_set)})
    offsets = np.concatenate(beam_offsets) if beam_offsets else np.zeros(0)
    beam = {'beams': int(offsets.size), 'within_1e_4': int((np.abs(offsets) <= 1e-4).sum()),
            'max_envelope_minus_truth_m': float(offsets.max()) if offsets.size else None,
            'min_envelope_minus_truth_m': float(offsets.min()) if offsets.size else None}
    # B-DEPTH: S-L1B delivered depth equals S-L1A; A-DEPTH thresholds on S-L1B depth.
    depth_equal = all(np.array_equal(data['s_l1a']['z'][p + '__depth'], z[p + '__depth']) for p in PASSES)
    obs, exp, msk = [], [], []
    fc = fp = 0
    for p in PASSES:
        scene_mode = p.endswith('_scene')
        for i, scene in enumerate(meta['scenes']):
            centers, halves, target = geometry(meta, z, i, scene_mode)
            eye = z['drone_env_m'][i].astype(float) + np.asarray(meta['camera']['camera_offset'])[0]
            world_c = [list(np.asarray(c) + z['drone_env_m'][i]) for c in centers]
            ref = aa.mixed_reference(world_c, halves, origin=eye, sphere_center=None if target is None else target + z['drone_env_m'][i])
            d = z[p + '__depth'][i]
            eligible = ref['robust'] & ~ref['target']
            obs.append(d.ravel()); exp.append(ref['depth_m'].ravel()); msk.append(eligible.ravel())
            fc += int((eligible & ref['obstacle'] & (d > ref['depth_m'] + .05)).sum())
            fp += int((eligible & ref['miss'] & (d < 9.95)).sum())
    depth_summary = aa.pixel_summary(np.concatenate(obs), np.concatenate(exp), np.concatenate(msk))
    paths = {
        'legacy_flags': (data['legacy']['sensor']['perception_s_l1a'], data['legacy']['sensor']['perception_s_l1b'],
                         data['legacy']['sensor']['detector_lineage_enabled']) == (False, False, False),
        's_l1a_flags': (data['s_l1a']['sensor']['perception_s_l1a'], data['s_l1a']['sensor']['perception_s_l1b'],
                        data['s_l1a']['sensor']['detector_lineage_enabled']) == (True, False, True),
        's_l1b_flags': (sensor['perception_s_l1a'], sensor['perception_s_l1b'], sensor['detector_lineage_enabled']) == (True, True, True),
        'actual_scan_equals_s_l1b_replay': bool(np.array_equal(r['scan_actual'], r['scan_s_l1b'])),
        'actual_carve_equals_s_l1b_replay': bool(np.array_equal(r['carve_actual'], r['carve_s_l1b'])),
        's_l1a_process_scan_equals_s_l1a_replay': bool(np.array_equal(data['s_l1a']['r']['scan_actual'], r['scan_s_l1a'])),
    }
    timing = {name: {'median_ms': float(np.median(v)), 'p95_ms': float(np.percentile(v, 95)), 'repeats': len(v)}
              for name, v in sensor['fusion_cuda_ms'].items()}
    return {'point': point, 'envelope': envelope, 'fused': fused, 'vertical': vertical, 'carve': carve,
            'beam_vs_lidar_truth': beam, 'bearing_bins': bins_report, 'depth_equal_s_l1a': depth_equal,
            'depth_summary': depth_summary, 'depth_false_clear': fc, 'depth_false_positive': fp,
            'paths': paths, 'fusion_timing': timing}


def firewall_ast():
    tree = ast.parse((ROOT / 'aerial_gym/task/navrl_task/navrl_sensor_lineage.py').read_text())
    forbidden = ('target_position', 'semantic', 'segmentation', 'ground_truth', 'drone_pos', 'tracker', 'simulator')
    bad = [(n.name, a.arg) for n in tree.body if isinstance(n, ast.FunctionDef)
           for a in n.args.args + n.args.kwonlyargs if any(w in a.arg.lower() for w in forbidden)]
    imports = sorted({a.name for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names})
    return {'forbidden_parameters': bad, 'imports': imports, 'import_from': any(isinstance(n, ast.ImportFrom) for n in ast.walk(tree))}


def analyse_run(run):
    run = Path(run).resolve()
    report = json.loads((run / 'run_report.json').read_text())
    sensors = analyse_sensors(run)
    regression, el0, on_changes, dims = {}, {}, {}, {}
    for config in ('static', 'moving'):
        step = lambda name: run / (config + '_' + name) / 'v3_steps.jsonl'
        regression[config] = {
            'off_vs_el0_r1': aa.compare_steps(step('off'), EL0_R1 / ('v3_' + config + '_p2') / 'v3_steps.jsonl'),
            's_l1a_vs_sl1a_c01': aa.compare_steps(step('s_l1a'), SL1A_C01 / (config + '_on_p2') / 'v3_steps.jsonl')}
        el0[config] = aa.compare_steps(step('s_l1b_p1'), step('s_l1b_p2'))
        for ref_name in ('s_l1a', 'off'):
            change = aa.compare_steps(step(ref_name), step('s_l1b_p2'))
            old = _arrays(run / (config + '_' + ref_name) / 'v3_trajectory.npz')
            new = _arrays(run / (config + '_s_l1b_p2') / 'v3_trajectory.npz')
            change['position_delta_max_m'] = float(np.linalg.norm(new['position'] - old['position'], axis=-1).max())
            change['action_delta_max'] = float(np.abs(new['action'] - old['action']).max())
            change['episodes'] = {m: {key: int(t[key].sum()) for key in ('captured', 'crashed')} for m, t in ((ref_name, old), ('s_l1b', new))}
            on_changes[config + '_s_l1b_vs_' + ref_name] = change
        for variant in ('off', 's_l1a', 's_l1b_p1', 's_l1b_p2'):
            dims[config + '_' + variant] = json.loads((run / (config + '_' + variant) / 'process_done.json').read_text()).get('actor_obs_dim')
    s = sensors
    fw = firewall_ast()
    gates = {
        'B-POINT': s['point']['n'] > 0 and s['point']['over'] == 0,
        'B-A1A2': bool(s['bearing_bins']) and s['point']['over'] == 0 and all(
            row['s_l1b_missing'] == 0 for row in s['bearing_bins']),
        'B-ENVELOPE': s['envelope']['robust_bins'] > 0 and s['envelope']['mismatch'] == 0 and s['fused']['mismatch'] == 0,
        'B-VERTICAL': all(s['vertical'].values()),
        'B-CARVE': bool(s['carve']['foreground']) and all(
            f['bar_beams'] > 0 and f['s_l1b']['bar_carved'] == 0 and f['s_l1b']['bar_false_clear'] == 0 for f in s['carve']['foreground'])
            and s['carve']['mask_mismatch'] == 0 and s['carve']['uncertain'] == 0
            and s['carve']['bar_carved']['s_l1b'] == 0 and s['carve']['bar_false_clear']['s_l1b'] == 0,
        'B-DEPTH': s['depth_equal_s_l1a'] and s['depth_summary']['n'] > 0 and s['depth_summary']['over_1e_4'] == 0
            and s['depth_false_clear'] == 0 and s['depth_false_positive'] == 0,
        'B-REGRESSION': all(v['identical'] for c in regression.values() for v in c.values()),
        'B-EL0': all(v['identical'] for v in el0.values()),
        'B-FIREWALL': not fw['forbidden_parameters'] and fw['imports'] == ['math', 'os', 'torch'] and not fw['import_from']
            and all(v == 898 for v in dims.values()),
        'B-INTEGRITY': report.get('status') == 'COMPLETED' and all(s['paths'].values()),
    }
    return {'schema': 'motar.s-l1b-analysis.v1', 'run': str(run), 'source_sha': report.get('source_commit'),
            'gates': {k: 'PASS' if v else 'FAIL' for k, v in gates.items()},
            'decision': 'S-L1B_QUALIFIED' if all(gates.values()) else 'S-L1B_FAIL',
            'sensors': sensors, 'regression': regression, 'e_l0': el0, 'on_behavior_changes': on_changes,
            'actor_obs_dim': dims, 'firewall_ast': fw,
            'known_limits': ['Constructed fixture counts are not population rates.',
                             'Envelope is a conservative per-bin minimum, not a beam-exact metric (D2-F1 reported, not gated).',
                             'A4/A7 centre-range radius biases unchanged (assigned to S-L1C).',
                             'Identity, range-only covariance (S-L1C) and silhouette validity (S-L1D) not corrected.',
                             'ON changes the input distribution; Frozen F results measure robustness, not improvement.']}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', required=True)
    p.add_argument('--out', required=True)
    a = p.parse_args(argv)
    result = analyse_run(a.run)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as fh:
        json.dump(result, fh, indent=2)
        fh.write('\n')
    print(json.dumps({'decision': result['decision'], 'gates': result['gates'], 'out': str(out)}, indent=2))


if __name__ == '__main__':
    main()
