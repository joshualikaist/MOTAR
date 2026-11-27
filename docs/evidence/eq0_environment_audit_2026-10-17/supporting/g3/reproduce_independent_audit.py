#!/usr/bin/env python3
"""NEW faithful standalone reproducer of EQ-0 G3's earlier in-memory CPU checks.

It uses an independently declared binary schema and NumPy/scalar arithmetic.
It imports no project, simulator, torch or production parser/tracker code.
Default: read immutable OCC-P2 and print JSON. Optional output is exclusive-create
and restricted to this audit's /tmp directory. No GPU probing or execution.

CUDA_VISIBLE_DEVICES='' PYTHONNOUSERSITE=1 \
 /home/fair/miniconda3/envs/aerialgym/bin/python -B \
 /tmp/eq0_g3_20261017/reproduce_independent_audit.py
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np

RAW = Path('/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/occlusion_confirmatory_v1/OCC-P2')
SCRATCH = Path('/tmp/eq0_g3_20261017')
WIRE = np.dtype([
    ('env', '<u2'), ('frame', '<u2'), ('episode', '<u4'),
    ('p_pos', '<f4', (3,)), ('p_vel', '<f4', (3,)),
    ('p_quat', '<f4', (4,)), ('p_vehicle_quat', '<f4', (4,)),
    ('t_pos', '<f4', (3,)), ('t_vel', '<f4', (3,)),
    ('flags', 'u1'), ('probe_center_blocked', 'i1'),
    ('probe_visible_pixels', '<i4'),
])
KNOBS = [
    'NAVRL_PERCEPTION_PERTURB', 'NAVRL_EVAL_ACTION_MODE',
    'NAVRL_DETECTOR_CHECKPOINT', 'NAVRL_DETECTION_LATENCY_S',
    'NAVRL_LIDAR_RANGE_ONLY_UPDATE', 'NAVRL_LIDAR_TARGET_ASSOC',
    'NAVRL_LIDAR_SILENT_CORRECT', 'NAVRL_TARGET_MASK_BACKFILL',
    'NAVRL_APP_HUE_DEG', 'NAVRL_APP_LIGHT_GAIN', 'NAVRL_APP_ALBEDO_JITTER',
    'NAVRL_APP_TEXTURE_STD', 'NAVRL_APP_MOTION_BLUR',
    'NAVRL_CAM_MOUNT_ROT_DEG', 'NAVRL_CAM_MOUNT_TRANS_M',
    'NAVRL_CAM_FOV_SCALE_ERR',
]
CONDITION_KEYS = [
    'action_selection', 'policy_output_dim',
    'policy_z_output_overwritten_by_altitude_pi',
    'policy_z_persisted_in_prev_action_observation', 'target_render_mode',
    'distractor_count', 'p9_empirical_error_enabled', 'speed_governor_mode',
    'speed_governor_fixed_mps', 'speed_governor_free_mps',
    'pursuer_max_speed_mps', 'pursuer_max_horizontal_request_norm_mps',
    'speed_governor_yaw_cap_radps', 'speed_governor_width_per_mps',
    'speed_governor_width_per_open_m', 'runtime_sim_config_class',
]


def episodes(frames):
    order = np.lexsort((frames['frame'], frames['episode'], frames['env']))
    f = frames[order]
    keys = np.stack((f['env'], f['episode']), axis=1)
    bounds = np.r_[0, np.flatnonzero(np.any(keys[1:] != keys[:-1], axis=1)) + 1, len(f)]
    return [f[b:e] for b, e in zip(bounds[:-1], bounds[1:])]


def audit(root):
    assert WIRE.itemsize == 94
    totals = defaultdict(Counter)
    z_values, velocities, conditions = [], [], []
    knobs, expiry = Counter(), Counter()
    cells, logs = 0, 0
    # Enumerated from the frozen preregistration, not from analysis outputs.
    for seed in range(4301, 4315):
        for density in (70, 115, 160, 205):
            for group, arm, model in [('static', 'A1', 'E0_static'), ('moving', 'A3', 'E2_obstacle_aware')]:
                directory = root / group / arm / f'{model}__{density}bars__seed{seed}'
                meta = json.loads((directory / 'occlusion_trace_summary.json').read_text())
                payload = (directory / 'occlusion_frames.bin').read_bytes()
                assert hashlib.sha256(payload).hexdigest() == meta['files']['occlusion_frames.bin']
                assert len(payload) % 94 == 0
                f = np.frombuffer(payload, dtype=WIRE)
                ledger = [v for v in map(json.loads, (directory / 'episodes.jsonl').read_text().splitlines()) if v['in_quota']]
                assert len(ledger) == 512
                env_counts = Counter(v['env_index'] for v in ledger)
                assert set(env_counts) == set(range(128)) and set(env_counts.values()) == {4}
                keys = {(v['env_index'], v['env_episode_index']) for v in ledger}
                assert len(keys) == 512
                mask = np.array([(int(v['env']), int(v['episode'])) in keys for v in f])
                f = f[mask]
                flags = f['flags']
                camera, fused = (flags & 1) > 0, (flags & 2) > 0
                active, lidar = (flags & 4) > 0, (flags & 32) > 0
                pixels = f['probe_visible_pixels']
                known = pixels >= 0
                c = totals[group]
                for name, value in [
                    ('frames', len(f)), ('camera_valid', camera.sum()),
                    ('lidar_only', (~camera & lidar).sum()), ('fused_valid', fused.sum()),
                    ('camera_invalid', (~camera).sum()),
                    ('fused_union_mismatch', (fused != (camera | lidar)).sum()),
                    ('fused_active_mismatch', (fused & ~active).sum()),
                    ('lidar_camera_overlap', (lidar & camera).sum()),
                    ('known_pixel_count', known.sum()),
                    ('camera_vs_rendered_pixels_ge2_mismatch', ((camera != (pixels >= 2)) & known).sum()),
                    ('track_active_no_fused', (active & ~fused).sum()),
                    ('pixel_lt2_fused_lidar', ((pixels < 2) & known & fused & lidar).sum()),
                ]:
                    c[name] += int(value)
                z_values.append(f['p_pos'][:, 2])
                velocities.append(np.linalg.norm(f['p_vel'][:, :2], axis=1))
                for ep in episodes(f):
                    assert np.array_equal(ep['frame'], np.arange(len(ep)))
                    fl = ep['flags']
                    fu, tr, ca = (fl & 2) > 0, (fl & 4) > 0, (fl & 1) > 0
                    seen, miss, max_miss, camera_miss, max_camera_miss, deactivations = False, 0, 0, 0, 0, 0
                    for i in range(len(ep)):
                        if fu[i]:
                            seen, miss = True, 0
                        elif seen:
                            miss += 1
                            max_miss = max(max_miss, miss)
                        if seen and not ca[i]:
                            camera_miss += 1
                            max_camera_miss = max(max_camera_miss, camera_miss)
                        else:
                            camera_miss = 0
                        if i and tr[i - 1] and not tr[i]:
                            deactivations += 1
                            expiry[f'{group}:{miss}'] += 1
                    c['episodes'] += 1
                    c['episodes_fused_miss_ge50'] += max_miss >= 50
                    c['episodes_fused_miss_ge51'] += max_miss >= 51
                    c['episodes_camera_miss_ge50_after_acq'] += max_camera_miss >= 50
                    c['track_active_to_inactive_transitions'] += deactivations
                    c['max_fused_miss_frames'] = max(c['max_fused_miss_frames'], max_miss)
                    c['max_camera_miss_frames_after_acq'] = max(c['max_camera_miss_frames_after_acq'], max_camera_miss)
                result = json.loads((directory / 'result.json').read_text())
                conditions.append(result['condition'])
                receipt = json.loads((directory / 'receipt.json').read_text())
                for name in KNOBS:
                    knobs[name + '=' + str(receipt['env_snapshot_navrl'].get(name, 'ABSENT_USE_SOURCE_DEFAULT'))] += 1
                logs += 'actor obs=898, critic states=906' in (directory / 'cell.log').read_text()
                cells += 1
    z, v = np.concatenate(z_values), np.concatenate(velocities)
    age, ages = np.float32(0), []
    for _ in range(52):
        age = np.float32(age + np.float32(.1))
        ages.append(float(age))
    target_logit = 3 * .88 - 2 * .08 - 2 * .045 - .9
    return {
        'schema': 'motar.eq0-g3.faithful-standalone-reproducer.v1',
        'new_reproducer_note': 'Reconstructed faithfully after original checks; not the original in-memory command text.',
        'raw_root': str(root), 'frames_sha_checked_cells': cells,
        'actor898_critic906_log_cells': logs, 'groups': dict(totals),
        'active_to_inactive_missed_fused_frames': dict(expiry),
        'pursuer_z_m': dict(min=float(z.min()), p01=float(np.quantile(z, .01)), median=float(np.median(z)), p99=float(np.quantile(z, .99)), max=float(z.max())),
        'pursuer_horizontal_actual_speed_mps': dict(p50=float(np.median(v)), p99=float(np.quantile(v, .99)), max=float(v.max())),
        'float32_age_after_misses': {str(n): ages[n - 1] for n in [49, 50, 51, 52]},
        'first_strict_age_gt5_missed_step': next(n for n, value in enumerate(ages, 1) if value > 5),
        'receipt_knob_presence': dict(knobs),
        'condition_invariants': {k: sorted({json.dumps(c.get(k), sort_keys=True) for c in conditions}) for k in CONDITION_KEYS},
        'independent_algebra': {
            'target_logit': target_logit, 'target_score': 1 / (1 + math.exp(-target_logit)),
            'threshold_logit': math.log(.55 / .45),
            'minimum_common_rgb_gain_for_threshold': (math.log(.55 / .45) + .9) / (3 * .88 - 2 * .08 - 2 * .045),
            'neutral_background_logit_formula': '-1.34*luminance-.9',
            'riskcap_at_clearance_m': {str(c): 2 + min(1, max(0, (c - 3) / 2)) * (3.53553390593 - 2) for c in [0, .1, .45, 1, 3, 4, 5]},
            'stopping_required_clearance_at_floor_m': .45 + 2 * .1 + 2 ** 2 / (2 * 2.9608856678),
            'stopping_margin_at_zero_clearance_floor_m': -(2 * .1 + 2 ** 2 / (2 * 2.9608856678)),
            'actor_dim_sum': 4 * 72 + 5 * 8 * 12 + 5 * 10 + 5 * 16,
            'critic_dim': 4 * 72 + 5 * 8 * 12 + 5 * 10 + 5 * 16 + 8,
            'unit_prior_lateral_variance_m2': 1,
            'legacy_diagonal_lateral_posterior_m2': .15 ** 2 / (1 + .15 ** 2),
            'range_only_unobserved_lateral_reference_m2': 1,
            'kalman_reference_scope': 'Independent information-assumption counterexample, not actual empirical tracker calibration.',
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw-root', type=Path, default=RAW)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    if args.out and args.out.resolve().parent != SCRATCH.resolve():
        parser.error('Output is restricted to /tmp/eq0_g3_20261017 and never overwrites.')
    result = audit(args.raw_root)
    rendered = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + '\n'
    if args.out:
        with args.out.open('x') as stream:
            stream.write(rendered)
    else:
        print(rendered, end='')


if __name__ == '__main__':
    main()
