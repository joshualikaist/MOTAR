#!/usr/bin/env python3
"""Independent CPU analysis of S-L1C association probes, closed-loop traces and digests.

No production sensor or simulator import. Eligibility of positives comes from the analytic LiDAR
truth (does a row near the target's elevation hit the 0.20 m proxy in the bearing-nearest column),
never from the S-L1C rule itself. Original raw is never modified.
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
import s_l1c_reference as rc  # noqa: E402
import s_l1a_analysis as aa  # noqa: E402

ra = rc.ref
EL0_R1 = (ROOT / '../aerial_gym_simulator/results/e_l0_evaluation_v1/EL0-R1').resolve()
SL1A_C01 = (ROOT / '../aerial_gym_simulator/results/s_l1a_v1/SL1A-C01').resolve()
SL1B_C01 = (ROOT / '../aerial_gym_simulator/results/s_l1b_v1/SL1B-C01').resolve()
NEGATIVES = ('N1', 'N2', 'N3', 'N5', 'N6', 'N7', 'N8')
SCENE_PASSES = ('level_scene', 'roll15_scene', 'pitch15_scene')
MODES = ('legacy', 's_l1b', 's_l1c')


def _arrays(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def load_mode(run, mode):
    d = Path(run) / ('sensors_' + mode)
    return {'meta': json.loads((d / 'd2_meta.json').read_text()), 'sensor': json.loads((d / 's_l1c_sensor_meta.json').read_text()),
            'z': _arrays(d / 'd2_renders.npz'), 'p': _arrays(d / 'association_probes.npz')}


def body_geometry(meta, z, i):
    drone = z['drone_env_m'][i].astype(float)
    centres = [list(np.asarray(b['centre']) - drone) for b in meta['layout'][i]]
    halves = [b['half'] for b in meta['layout'][i]]
    target = (z['target_world'][i].astype(float) - drone) if z['target_present'][i] else None
    return centres, halves, target


def eligible_positive(centres, halves, target):
    """Analytic LiDAR truth only: some row within the proxy's angular radius of the target centre
    hits the proxy in the bearing-nearest column, and the range is conclusive."""
    if target is None:
        return False, None
    d = float(np.linalg.norm(target))
    scan, cls = rc.lidar_scan(centres, halves, sphere_center=target)
    bearing = math.atan2(target[1], target[0])
    h = int(np.argmin(np.abs(np.arctan2(np.sin(rc.ANGLES - bearing), np.cos(rc.ANGLES - bearing)))))
    el = math.atan2(target[2], math.hypot(target[0], target[1]))
    rows = np.where(np.abs(rc.ELEVATIONS - el) < math.asin(min(1., rc.LIDAR_PROXY_RADIUS_M / d)))[0]
    hit = any(cls[r, h] == ra.SPHERE for r in rows)
    return bool(hit and d >= rc.MIN_CONCLUSIVE_M and d - rc.LIDAR_PROXY_RADIUS_M < 12 * .995), d


def analyse_sensors(run):
    data = {m: load_mode(run, m) for m in MODES}
    meta, z = data['s_l1c']['meta'], data['s_l1c']['z']
    gate_meta = data['s_l1c']['sensor']
    negatives = {m: {'accepted': 0, 'trials': 0} for m in MODES}
    oracle_mismatch = 0
    neg_rows = []
    for k, pass_name in enumerate(SCENE_PASSES):
        lidar_key = 'lidar_pass%d' % (2 * k)
        for i, scene in enumerate(meta['scenes']):
            if scene not in NEGATIVES:
                continue
            for m in MODES:
                acc = int(data[m]['z'][pass_name + '__assoc_accept'][i])
                negatives[m]['trials'] += 1
                negatives[m]['accepted'] += int(acc == 1)
            primed = z[pass_name + '__assoc_primed_world'][i].astype(float) - z['drone_env_m'][i].astype(float)
            scan = data['s_l1c']['p'][lidar_key][i].reshape(4, 72).astype(np.float64)
            gate = 0.35 + 2 * math.sqrt(0.03) if gate_meta['lidar_assoc_gate_m'] <= 0 else gate_meta['lidar_assoc_gate_m']
            expected = rc.associate(scan, primed, gate_m=min(gate, 1.0))
            actual = int(z[pass_name + '__assoc_accept'][i]) == 1
            oracle_mismatch += int(expected['valid'] != actual)
            neg_rows.append({'scene': scene, 'pass': pass_name, 'legacy': int(data['legacy']['z'][pass_name + '__assoc_accept'][i]),
                             's_l1c': int(actual), 'oracle_s_l1c': expected['valid'], 'extended': expected.get('extended')})
    positives = {m: {'eligible': 0, 'accepted': 0} for m in MODES}
    pos_rows, radius_errors, cov_rows = [], [], []
    p = data['s_l1c']['p']
    for k, pass_name in enumerate(SCENE_PASSES):
        key = 'pass%d' % (2 * k)
        for i, scene in enumerate(meta['scenes']):
            centres, halves, target = body_geometry(meta, z, i)
            ok, d = eligible_positive(centres, halves, target)
            if target is None:
                continue
            for off in range(3):
                for m in MODES:
                    v = bool(data[m]['p']['%s__off%d__gate__valid' % (key, off)][i])
                    if ok:
                        positives[m]['eligible'] += 1
                        positives[m]['accepted'] += int(v)
                if ok and off == 0:
                    centre = float(p['%s__off0__gate__centre_range' % key][i])
                    bearing_deg = math.degrees(math.atan2(target[1], target[0]))
                    beam_centred = abs(target[2]) < 1e-6 and abs((bearing_deg / 5.0) - round(bearing_deg / 5.0)) < 1e-6
                    if beam_centred:
                        radius_errors.append(abs(centre - d))
                pos_rows.append({'scene': scene, 'pass': pass_name, 'offset_index': off, 'eligible': ok,
                                 **{m: bool(data[m]['p']['%s__off%d__gate__valid' % (key, off)][i]) for m in MODES}})
            # Covariance probe (prior I): orthogonal variance ratios for accepted updates.
            for m in MODES:
                if not bool(data[m]['p']['%s__off0__cov__valid' % key][i]):
                    continue
                before = data[m]['p']['%s__off0__cov__cov_before' % key][i].astype(np.float64)
                after = data[m]['p']['%s__off0__cov__cov_after' % key][i].astype(np.float64)
                primed = data[m]['p']['%s__off0__cov__primed' % key][i].astype(np.float64) - data[m]['p']['drone_world'][i]
                u, t1, t2 = rc.orthonormal_basis(primed)
                cov_rows.append({'mode': m, 'scene': scene, 'pass': pass_name,
                                 't1_ratio': float((t1 @ after @ t1) / (t1 @ before @ t1)),
                                 't2_ratio': float((t2 @ after @ t2) / (t2 @ before @ t2)),
                                 'u_ratio': float((u @ after @ u) / (u @ before @ u))})
    cov = {m: {'n': sum(1 for r in cov_rows if r['mode'] == m),
               'min_orthogonal_ratio': min([min(r['t1_ratio'], r['t2_ratio']) for r in cov_rows if r['mode'] == m], default=None),
               'max_u_ratio': max([r['u_ratio'] for r in cov_rows if r['mode'] == m], default=None)} for m in MODES}
    flags = {m: (data[m]['sensor']['perception_s_l1a'], data[m]['sensor']['perception_s_l1b'], data[m]['sensor']['perception_s_l1c'])
             for m in MODES}
    return {'negatives': negatives, 'negative_rows': neg_rows, 'oracle_mismatch': oracle_mismatch,
            'positives': positives, 'positive_rows': pos_rows,
            'radius': {'n': len(radius_errors), 'max_abs_m': max(radius_errors) if radius_errors else None},
            'covariance': cov, 'covariance_rows': cov_rows, 'flags': flags}


def provenance(run, step_dt):
    out = {}
    for config in ('static', 'moving'):
        for variant in ('off', 's_l1a', 's_l1b', 's_l1c_p1', 's_l1c_p2'):
            t = _arrays(Path(run) / (config + '_' + variant) / 's_l1c_association_trace.npz')
            row = {'lidar_accepts': int(t['lidar_visible'].sum()), 'camera_frames': int(t['camera_visible'].sum()),
                   'steps': int(t['lidar_visible'].shape[0])}
            if variant.startswith('s_l1c'):
                src, age, lid, track = t['update_source'], t['identity_age'], t['lidar_visible'], t['track_age']
                cam_ok = bool((age[src == 1] == 0).all())
                lidar_never_resets = bool((age[src == 2] > 0).all())
                inc = np.abs(age[1:] - (age[:-1] + step_dt))[(src[1:] == 2) & (src[:-1] != 0)]
                fused_contract = bool((track[lid] == 0).all())
                row.update({'camera_resets_identity': cam_ok, 'lidar_never_resets_identity': lidar_never_resets,
                            'lidar_identity_increment_max_err': float(inc.max()) if inc.size else 0.0,
                            'fused_age_reset_on_lidar_accept': fused_contract,
                            'update_source_counts': {str(v): int((src == v).sum()) for v in (0, 1, 2)}})
            out[config + '_' + variant] = row
    return out


def firewall_ast():
    tree = ast.parse((ROOT / 'aerial_gym/task/navrl_task/navrl_sensor_lineage.py').read_text())
    forbidden = ('target_position', 'semantic', 'segmentation', 'ground_truth', 'drone_pos', 'simulator')
    bad = [(n.name, x.arg) for n in tree.body if isinstance(n, ast.FunctionDef)
           for x in n.args.args + n.args.kwonlyargs if any(w in x.arg.lower() for w in forbidden)]
    imports = sorted({x.name for n in ast.walk(tree) if isinstance(n, ast.Import) for x in n.names})
    return {'forbidden_parameters': bad, 'imports': imports, 'import_from': any(isinstance(n, ast.ImportFrom) for n in ast.walk(tree))}


def analyse_run(run):
    run = Path(run).resolve()
    report = json.loads((run / 'run_report.json').read_text())
    s = analyse_sensors(run)
    prov = provenance(run, float(json.loads((run / 'sensors_s_l1c' / 's_l1c_sensor_meta.json').read_text())['step_dt']))
    regression, el0, changes, dims = {}, {}, {}, {}
    for config in ('static', 'moving'):
        step = lambda name: run / (config + '_' + name) / 'v3_steps.jsonl'
        regression[config] = {
            'off_vs_el0_r1': aa.compare_steps(step('off'), EL0_R1 / ('v3_' + config + '_p2') / 'v3_steps.jsonl'),
            's_l1a_vs_sl1a_c01': aa.compare_steps(step('s_l1a'), SL1A_C01 / (config + '_on_p2') / 'v3_steps.jsonl'),
            's_l1b_vs_sl1b_c01': aa.compare_steps(step('s_l1b'), SL1B_C01 / (config + '_s_l1b_p2') / 'v3_steps.jsonl')}
        el0[config] = aa.compare_steps(step('s_l1c_p1'), step('s_l1c_p2'))
        for ref_name in ('s_l1b', 'off'):
            change = aa.compare_steps(step(ref_name), step('s_l1c_p2'))
            old = _arrays(run / (config + '_' + ref_name) / 'v3_trajectory.npz')
            new = _arrays(run / (config + '_s_l1c_p2') / 'v3_trajectory.npz')
            change['episodes'] = {m: {key: int(t[key].sum()) for key in ('captured', 'crashed')} for m, t in ((ref_name, old), ('s_l1c', new))}
            changes[config + '_s_l1c_vs_' + ref_name] = change
        for variant in ('off', 's_l1a', 's_l1b', 's_l1c_p1', 's_l1c_p2'):
            dims[config + '_' + variant] = json.loads((run / (config + '_' + variant) / 'process_done.json').read_text()).get('actor_obs_dim')
    fw = firewall_ast()
    c_prov = all(prov[c + '_' + v]['camera_resets_identity'] and prov[c + '_' + v]['lidar_never_resets_identity']
                 and prov[c + '_' + v]['lidar_identity_increment_max_err'] <= 1e-5 and prov[c + '_' + v]['fused_age_reset_on_lidar_accept']
                 for c in ('static', 'moving') for v in ('s_l1c_p1', 's_l1c_p2'))
    gates = {
        'C-NEG': s['negatives']['s_l1c']['trials'] == 21 and s['negatives']['s_l1c']['accepted'] == 0 and s['oracle_mismatch'] == 0,
        'C-POS': s['positives']['s_l1c']['eligible'] > 0 and s['positives']['s_l1c']['accepted'] == s['positives']['s_l1c']['eligible'],
        'C-COV': s['covariance']['s_l1c']['n'] > 0 and s['covariance']['s_l1c']['min_orthogonal_ratio'] >= .99
                 and s['covariance']['s_l1c']['max_u_ratio'] < 1.0,
        'C-PROV': c_prov,
        'C-RADIUS': s['radius']['n'] > 0 and s['radius']['max_abs_m'] <= 1e-3,
        'C-REGRESSION': all(v['identical'] for c in regression.values() for v in c.values()),
        'C-EL0': all(v['identical'] for v in el0.values()),
        'C-FIREWALL': not fw['forbidden_parameters'] and fw['imports'] == ['math', 'os', 'torch'] and not fw['import_from']
                      and all(v == 898 for v in dims.values()),
        'C-INTEGRITY': report.get('status') == 'COMPLETED' and s['flags'] == {
            'legacy': (False, False, False), 's_l1b': (True, True, False), 's_l1c': (True, True, True)},
    }
    return {'schema': 'motar.s-l1c-analysis.v1', 'run': str(run), 'source_sha': report.get('source_commit'),
            'gates': {k: 'PASS' if v else 'FAIL' for k, v in gates.items()},
            'decision': 'S-L1C_QUALIFIED' if all(gates.values()) else 'S-L1C_FAIL',
            'sensors': s, 'provenance': prov, 'regression': regression, 'e_l0': el0, 'on_behavior_changes': changes,
            'actor_obs_dim': dims, 'firewall_ast': fw,
            'known_limits': ['Constructed fixture counts are not population rates.',
                             'Shape test assumes vertical-faced obstacles; bars whose front face is beyond about 11.35 m return one row and remain indistinguishable.',
                             'LiDAR association is refused below 1.152 m (recall loss).',
                             'Camera centre-range A7 bias unchanged; silhouette validity (S-L1D) not corrected.',
                             'ON changes the input distribution; Frozen F results measure robustness, not improvement.']}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args(argv)
    result = analyse_run(args.run)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as fh:
        json.dump(result, fh, indent=2)
        fh.write('\n')
    print(json.dumps({'decision': result['decision'], 'gates': result['gates'], 'out': str(out)}, indent=2))


if __name__ == '__main__':
    main()
