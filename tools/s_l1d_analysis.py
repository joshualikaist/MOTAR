#!/usr/bin/env python3
"""Independent CPU analysis of S-L1D validity probes, closed-loop traces and digests.

No production sensor or simulator import. Evaluator truth is the recorded E-L0 class plus an
independent analytic recomputation (float64 oracle) of support, occluder gaps and the exact centre
ray. Original raw is never modified.
"""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/audits'), str(ROOT / 'tools')]
import s_l1d_reference as rd  # noqa: E402
import s_l1a_analysis as aa  # noqa: E402

ra = rd.ref
EL0_R1 = (ROOT / '../aerial_gym_simulator/results/e_l0_evaluation_v1/EL0-R1').resolve()
SL1C_C01 = (ROOT / '../aerial_gym_simulator/results/s_l1c_v1/SL1C-C01').resolve()
PASSES = aa.PASSES
PARTIAL_TYPES = ('OCCLUSION_CENTRE_EVIDENCE', 'OCCLUSION_PARTIAL_EVIDENCE')
OCCLUSION = PARTIAL_TYPES + ('OCCLUSION_FULL_EVIDENCE',)


def _arrays(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def required(el0_name, alone, min_pixels, centre_robust, centre_blocked):
    """Allowed online classes for an undetected primed fixture (preregistered mapping)."""
    if el0_name in ('RANGE_EXIT', 'FOV_EXIT', 'SCALE_LOSS'):
        return ('PRED_' + el0_name,)
    if el0_name in ('FULL', 'CENTRE_ONLY', 'PARTIAL') and alone < min_pixels:
        return ('PRED_SCALE_LOSS',)
    if el0_name == 'FULL':
        return ('OCCLUSION_FULL_EVIDENCE',)
    if el0_name in ('CENTRE_ONLY', 'PARTIAL'):
        if centre_robust:
            return ('OCCLUSION_CENTRE_EVIDENCE',) if centre_blocked else ('OCCLUSION_PARTIAL_EVIDENCE',)
        return PARTIAL_TYPES
    return None  # VISIBLE / NONE undetected: not constrained by the mapping (reported)


def analyse_sensors(run):
    d = Path(run) / 'sensors_s_l1d'
    meta = json.loads((d / 'd2_meta.json').read_text())
    sensor = json.loads((d / 's_l1d_sensor_meta.json').read_text())
    z = _arrays(d / 'd2_renders.npz')
    p = _arrays(d / 'validity_probes.npz')
    online_names, el0_names = rd.ONLINE, sensor['el0_classes']
    min_px = sensor['min_target_pixels']
    oracle_support_mismatch = el0_support_mismatch = 0
    gated = agree = within_tau = alone_occlusion = zero_full = 0
    rows, confusion = [], {}
    cache = {}
    for k, pass_name in enumerate(PASSES):
        scene_mode = pass_name.endswith('_scene')
        for i, scene in enumerate(meta['scenes']):
            if not p['target_present'][i]:
                continue
            drone = p['drone_world'][i].astype(np.float64)
            target = p['target_world'][i].astype(np.float64) - drone
            centres = [list(np.asarray(b['centre']) - drone) for b in meta['layout'][i]] if scene_mode else []
            halves = [b['half'] for b in meta['layout'][i]] if scene_mode else []
            key = (scene, scene_mode)
            if key not in cache:
                cache[key] = rd.evaluator_class(centres, halves, target)
            ev = cache[key]
            code = int(p['pass%d__online_code' % k][i])
            online = online_names[code]
            support = int(p['pass%d__online_support_px' % k][i])
            el0 = el0_names[int(p['pass%d__el0_class' % k][i])]
            alone_el0 = int(p['pass%d__el0_alone_px' % k][i])
            oracle_support_mismatch += int(support != ev['alone_px'])
            el0_support_mismatch += int(support != alone_el0)
            detected = bool(p['pass%d__camera_visible' % k][i])
            confusion.setdefault(el0, {}).setdefault(online, 0)
            confusion[el0][online] += 1
            if not scene_mode and online in OCCLUSION:
                alone_occlusion += 1
            if support == 0 and online == 'OCCLUSION_FULL_EVIDENCE':
                zero_full += 1
            row = {'scene': scene, 'pass': pass_name, 'el0': el0, 'oracle_class': ev['class'], 'online': online,
                   'detected': detected, 'support': support, 'alone_el0': alone_el0, 'alone_oracle': ev['alone_px'],
                   'occluded': int(p['pass%d__online_occluded_px' % k][i]), 'centre_robust': ev['centre_robust']}
            if not detected:
                allowed = required(el0, alone_el0, min_px, ev['centre_robust'], ev['centre_blocked'])
                tau = float(p['pass%d__online_tau_m' % k][i])
                mask = rd.predicted_support(target)['mask']
                box_px = mask & (ev['render_class'] == ra.BOX)
                gaps = (rd.predicted_support(target)['hit_m'] - ev['render_depth'])[box_px]
                close = bool(gaps.size and (gaps <= tau).any())
                row.update({'allowed': allowed, 'occluder_within_tau': close})
                if allowed is not None:
                    if close:
                        within_tau += 1
                    else:
                        gated += 1
                        agree += int(online in allowed)
            rows.append(row)
    return {'gated': gated, 'agree': agree, 'within_tau_reported': within_tau, 'alone_occlusion': alone_occlusion,
            'zero_support_full': zero_full, 'oracle_support_mismatch': oracle_support_mismatch,
            'el0_support_mismatch': el0_support_mismatch, 'confusion': confusion,
            'disagreements': [r for r in rows if r.get('allowed') and not r['occluder_within_tau'] and r['online'] not in r['allowed']],
            'rows': rows, 'flags': (sensor['perception_s_l1c'], sensor['perception_s_l1d'])}


def closed_loop(run):
    out = {}
    for config in ('static', 'moving'):
        for variant in ('s_l1d_p1', 's_l1d_p2'):
            t = _arrays(Path(run) / (config + '_' + variant) / 's_l1d_validity_trace.npz')
            online = t['online_class']
            el0 = t['el0_class']
            src = t['update_source']
            age = t['identity_age']
            cam = src == 1
            reacq = np.zeros_like(cam)
            reacq[1:] = cam[1:] & (age[:-1] > 0.5) & (src[:-1] != 1)
            fused_return = np.zeros_like(cam)
            fused_return[1:] = (src[1:] == 2) & (src[:-1] == 0)
            confusion = {}
            for e, o in zip(el0.ravel(), online.ravel()):
                confusion.setdefault(rd.EVALUATOR[int(e)], {}).setdefault(rd.ONLINE[int(o)], 0)
                confusion[rd.EVALUATOR[int(e)]][rd.ONLINE[int(o)]] += 1
            err = t['camera_gt_error_m'][reacq]
            out[config + '_' + variant] = {'steps': int(online.shape[0]), 'identity_confirmed_reacquisitions': int(reacq.sum()),
                                           'fused_valid_returns': int(fused_return.sum()),
                                           'reacquisition_gt_error_m': {'n': int(err.size), 'median': float(np.median(err)) if err.size else None,
                                                                        'max': float(err.max()) if err.size else None},
                                           'confusion': confusion}
    return out


def firewall_ast():
    tree = ast.parse((ROOT / 'aerial_gym/task/navrl_task/navrl_sensor_lineage.py').read_text())
    forbidden = ('target_position', 'semantic', 'segmentation', 'ground_truth', 'drone_pos', 'simulator', 'el0', 'renderer_mask')
    bad = [(n.name, x.arg) for n in tree.body if isinstance(n, ast.FunctionDef)
           for x in n.args.args + n.args.kwonlyargs if any(w in x.arg.lower() for w in forbidden)]
    imports = sorted({x.name for n in ast.walk(tree) if isinstance(n, ast.Import) for x in n.names})
    return {'forbidden_parameters': bad, 'imports': imports, 'import_from': any(isinstance(n, ast.ImportFrom) for n in ast.walk(tree))}


def analyse_run(run):
    run = Path(run).resolve()
    report = json.loads((run / 'run_report.json').read_text())
    s = analyse_sensors(run)
    cl = closed_loop(run)
    regression, el0, dims = {}, {}, {}
    for config in ('static', 'moving'):
        step = lambda name: run / (config + '_' + name) / 'v3_steps.jsonl'
        regression[config] = {
            'off_vs_el0_r1': aa.compare_steps(step('off'), EL0_R1 / ('v3_' + config + '_p2') / 'v3_steps.jsonl'),
            's_l1c_vs_sl1c_c01': aa.compare_steps(step('s_l1c'), SL1C_C01 / (config + '_s_l1c_p2') / 'v3_steps.jsonl'),
            's_l1d_vs_s_l1c_diagnostic_only': aa.compare_steps(step('s_l1d_p2'), step('s_l1c'))}
        el0[config] = aa.compare_steps(step('s_l1d_p1'), step('s_l1d_p2'))
        for variant in ('off', 's_l1c', 's_l1d_p1', 's_l1d_p2'):
            dims[config + '_' + variant] = json.loads((run / (config + '_' + variant) / 'process_done.json').read_text()).get('actor_obs_dim')
    fw = firewall_ast()
    gates = {
        'D-ORACLE': s['oracle_support_mismatch'] == 0 and s['el0_support_mismatch'] == 0 and bool(s['rows']),
        'D-FIXTURES': s['gated'] > 0 and s['agree'] == s['gated'] and s['alone_occlusion'] == 0 and s['zero_support_full'] == 0,
        'D-SEPARATION': not fw['forbidden_parameters'] and fw['imports'] == ['math', 'os', 'torch'] and not fw['import_from'] and bool(cl),
        'D-REACQ': all('identity_confirmed_reacquisitions' in v and 'fused_valid_returns' in v for v in cl.values()),
        'D-REGRESSION': all(v['identical'] for c in regression.values() for v in c.values()),
        'D-EL0': all(v['identical'] for v in el0.values()),
        'D-FIREWALL': all(v == 898 for v in dims.values()),
        'D-INTEGRITY': report.get('status') == 'COMPLETED' and s['flags'] == (True, True),
    }
    return {'schema': 'motar.s-l1d-analysis.v1', 'run': str(run), 'source_sha': report.get('source_commit'),
            'gates': {k: 'PASS' if v else 'FAIL' for k, v in gates.items()},
            'decision': 'S-L1D_QUALIFIED' if all(gates.values()) else 'S-L1D_FAIL',
            'sensors': {k: v for k, v in s.items() if k != 'rows'}, 'fixture_rows': s['rows'], 'closed_loop': cl,
            'regression': regression, 'e_l0': el0, 'actor_obs_dim': dims, 'firewall_ast': fw,
            'known_limits': ['Constructed fixture counts are not population rates; attitude passes are identical inputs (unique = 1/3).',
                             'Occluders within tau of the target surface are indistinguishable in depth (reported, not gated).',
                             'Occluders beyond the 10 m obstacle far plane are invisible to the obstacle depth.',
                             'The online state is only as good as the track prediction; closed-loop agreement is reported, not gated.']}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args(argv)
    result = analyse_run(args.run)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as fh:
        json.dump(result, fh, indent=2, default=str)
        fh.write('\n')
    print(json.dumps({'decision': result['decision'], 'gates': result['gates'], 'out': str(out)}, indent=2))


if __name__ == '__main__':
    main()
