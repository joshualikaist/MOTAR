#!/usr/bin/env python3
"""Independent CPU analysis of SL1D-C02 (S-L1D amendment 1): undetected partial/centre-only/scale.

No production import. Evaluator truth is the GPU E-L0 class plus the float64 oracle on the
recorded geometry; the online check recomputes the oracle online class on the delivered GPU depth
with the primed belief. Only the level scene pass of each scene counts (attitude duplicates are
verified identical). Nothing is excluded because its rendered class differs from the design.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools/audits'), str(ROOT / 'tools')]
import s_l1d_reference as rd  # noqa: E402
import s_l1d_analysis as da  # noqa: E402

ra = rd.ref
PASSES = ('level_scene', 'level_alone', 'roll15_scene', 'roll15_alone', 'pitch15_scene', 'pitch15_alone')
TIGHT = np.eye(3) * 1e-6
AMENDMENT = json.loads((ROOT / 'docs/plans/s_l1d_v1_amendment1.json').read_text())


def _arrays(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def load(run, mode):
    d = Path(run) / ('sensors_' + mode)
    return {'meta': json.loads((d / 'd2_meta.json').read_text()), 'sensor': json.loads((d / 's_l1d_sensor_meta.json').read_text()),
            'z': _arrays(d / 'd2_renders.npz'), 'p': _arrays(d / 'validity_probes.npz')}


def duplicates_identical(data, i):
    z, p = data['z'], data['p']
    same = True
    for group in ((0, 2, 4), (1, 3, 5)):
        ref = group[0]
        for k in group[1:]:
            same &= bool(np.array_equal(z[PASSES[k] + '__depth'][i], z[PASSES[ref] + '__depth'][i]))
            for key in ('camera_visible', 'el0_class', 'el0_alone_px', 'el0_visible_px', 'online_code', 'online_support_px', 'online_occluded_px'):
                same &= bool(np.array_equal(p['pass%d__%s' % (k, key)][i], p['pass%d__%s' % (ref, key)][i]))
    return same


def classify_case(data, i, k):
    meta, z, p = data['meta'], data['z'], data['p']
    min_px = data['sensor']['min_target_pixels']
    scene_mode = PASSES[k].endswith('_scene')
    drone = p['drone_world'][i].astype(np.float64)
    target = p['target_world'][i].astype(np.float64) - drone
    boxes = [list(np.asarray(b['centre']) - drone) for b in meta['layout'][i]] if scene_mode else []
    halves = [b['half'] for b in meta['layout'][i]] if scene_mode else []
    ev = rd.evaluator_class(boxes, halves, target)
    depth = z[PASSES[k] + '__depth'][i].astype(np.float64)
    detected = bool(p['pass%d__camera_visible' % k][i])
    oracle_online = rd.online_class(depth, detected, True, target, TIGHT)
    el0 = data['sensor']['el0_classes'][int(p['pass%d__el0_class' % k][i])]
    alone = int(p['pass%d__el0_alone_px' % k][i])
    online = rd.ONLINE[int(p['pass%d__online_code' % k][i])]
    support = int(p['pass%d__online_support_px' % k][i])
    s = rd.predicted_support(target)
    mask = s['mask']
    gaps = (s['hit_m'] - depth)[mask & (ev['render_class'] == ra.BOX)]
    tau = float(p['pass%d__online_tau_m' % k][i])
    ambiguous = rd.ambiguous_support_pixels(target)
    allowed = None
    if not detected:
        allowed = da.required(el0, alone, min_px, ev['centre_robust'], ev['centre_blocked'])
        if alone < min_px:
            allowed = ('PRED_SCALE_LOSS',)
    return {
        'pass': PASSES[k], 'el0_class': el0, 'oracle_class': ev['class'], 'alone_px_el0': alone, 'alone_px_oracle': ev['alone_px'],
        'visible_px_rendered': int(p['pass%d__el0_visible_px' % k][i]), 'visible_px_oracle': ev['visible_px'], 'detected': detected,
        'centre_blocked_oracle': ev['centre_blocked'], 'centre_robust_oracle': ev['centre_robust'],
        'delivered_depth_min_over_support_m': float(depth[mask].min()) if mask.any() else None,
        'predicted_surface_min_m': float(s['hit_m'][mask].min()) if mask.any() else None,
        'occluder_gap_min_m': float(gaps.min()) if gaps.size else None, 'occluder_within_tau': bool(gaps.size and (gaps <= tau).any()),
        'primed_belief_body_m': [round(float(x), 6) for x in target], 'primed_cov_diag': [1e-6] * 3, 'tau_m': tau,
        'online_class': online, 'online_support_px': support, 'online_occluded_px': int(p['pass%d__online_occluded_px' % k][i]),
        'oracle_online_class_on_gpu_depth': oracle_online['class'], 'oracle_online_support_px': oracle_online.get('support'),
        'oracle_online_occluded_px': oracle_online.get('occluded'), 'ambiguous_px': ambiguous,
        'support_agrees': abs(support - alone) <= ambiguous and abs(support - ev['alone_px']) <= ambiguous,
        'online_agrees_with_oracle': online == oracle_online['class'] and (detected or support == oracle_online.get('support')),
        'mapping_allowed': allowed, 'mapping_ok': allowed is None or online in allowed}


def analyse(run):
    run = Path(run).resolve()
    report = json.loads((run / 'run_report.json').read_text())
    c, dd = load(run, 's_l1c'), load(run, 's_l1d')
    ids = dd['meta']['scenes']
    design = AMENDMENT['scene_design']['expected_from_design']
    same = lambda x, y: np.array_equal(x, y, equal_nan=x.dtype.kind == 'f')  # NaN marks unprimed probes
    nonint = all(same(c['z'][k], dd['z'][k]) for k in dd['z']) and set(c['z']) == set(dd['z']) and all(
        np.array_equal(c['p']['pass%d__camera_visible' % k], dd['p']['pass%d__camera_visible' % k]) for k in range(6))
    scenes = []
    for i, sid in enumerate(ids):
        level = classify_case(dd, i, 0)
        alone = classify_case(dd, i, 1)
        scenes.append({'id': sid, 'design': design.get(sid), 'duplicates_identical': duplicates_identical(dd, i),
                       'level_scene': level, 'level_alone': alone})
    min_px = dd['sensor']['min_target_pixels']
    def realised(pred):
        return [s['id'] for s in scenes if pred(s['level_scene'])]
    good = lambda r: r['support_agrees'] and r['online_agrees_with_oracle'] and r['mapping_ok']
    scale_cases = realised(lambda r: not r['detected'] and r['alone_px_el0'] in (0, 1))
    partial_cases = realised(lambda r: not r['detected'] and r['el0_class'] == 'PARTIAL' and r['alone_px_el0'] >= min_px)
    centre_cases = realised(lambda r: not r['detected'] and r['el0_class'] == 'CENTRE_ONLY' and r['alone_px_el0'] >= min_px
                            and r['centre_robust_oracle'])
    by_id = {s['id']: s['level_scene'] for s in scenes}
    scale_ok = [x for x in scale_cases if good(by_id[x]) and by_id[x]['online_class'] == 'PRED_SCALE_LOSS']
    partial_ok = [x for x in partial_cases if good(by_id[x])]
    centre_ok = [x for x in centre_cases if good(by_id[x]) and by_id[x]['online_class'] == 'OCCLUSION_CENTRE_EVIDENCE']
    violations = [s['id'] for s in scenes if not good(s['level_scene'])]
    alone_occlusion = [s['id'] for s in scenes if s['level_alone']['online_class'].startswith('OCCLUSION')]
    flags_ok = (dd['sensor']['perception_s_l1c'], dd['sensor']['perception_s_l1d']) == (True, True) and (
        c['sensor']['perception_s_l1c'], c['sensor']['perception_s_l1d']) == (True, False)
    gates = {
        'C02-UNIQUE': all(s['duplicates_identical'] for s in scenes),
        'C02-ORACLE': all(s['level_scene']['support_agrees'] and s['level_scene']['online_agrees_with_oracle'] for s in scenes),
        'C02-MAPPING': not violations and not alone_occlusion,
        'C02-SCALE': bool(scale_ok),
        'C02-PARTIAL': bool(partial_ok),
        'C02-CENTRE': bool(centre_ok),
        'C02-SENSOR-NONINTERFERENCE': nonint,
        'C02-INTEGRITY': report.get('status') == 'COMPLETED' and flags_ok and bool(report['preflight'].get('production_identical_to_76f0f4c')),
    }
    realised_required = bool(scale_cases) and bool(partial_cases) and bool(centre_cases)
    violated_required = any(x not in scale_ok for x in scale_cases) or any(x not in partial_ok for x in partial_cases) or any(
        x not in centre_ok for x in centre_cases)
    if all(gates.values()):
        decision = 'GO'
    elif violated_required or violations:
        decision = 'FAIL'
    else:
        decision = 'INCOMPLETE'
    return {'schema': 'motar.s-l1d-c02-analysis.v1', 'run': str(run), 'source_sha': report.get('source_commit'),
            'gates': {k: 'PASS' if v else 'FAIL' for k, v in gates.items()}, 'decision': decision,
            'realised': {'scale': scale_cases, 'partial': partial_cases, 'centre_only': centre_cases},
            'passing': {'scale': scale_ok, 'partial': partial_ok, 'centre_only': centre_ok},
            'violations': violations, 'alone_pass_occlusion': alone_occlusion, 'realised_all_required': realised_required,
            'scenes': scenes,
            'limits': ['Fixture counts are constructed, not population rates; one unique pass per scene.',
                       'GO does not claim real-robot sensor accuracy or Recovery performance.']}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--run', required=True)
    ap.add_argument('--out', required=True)
    args = ap.parse_args(argv)
    result = analyse(args.run)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('x') as fh:
        json.dump(result, fh, indent=2, default=str)
        fh.write('\n')
    print(json.dumps({'decision': result['decision'], 'gates': result['gates'], 'realised': result['realised'],
                      'passing': result['passing'], 'violations': result['violations']}, indent=2))


if __name__ == '__main__':
    main()
