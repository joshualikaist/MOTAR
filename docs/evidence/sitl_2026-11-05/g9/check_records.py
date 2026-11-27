"""Read-only G9 attempt-4 record checks; no simulator, torch, tests or CUDA import."""
import resource
resource.setrlimit(resource.RLIMIT_AS, (4 * 1024**3, 4 * 1024**3))

import datetime
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess

started = datetime.datetime.now().astimezone().isoformat()
tree = Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly')
base = Path('/home/fair/motar_tmp/g9_resume_20261105')
audit = base / 'results_review'
primary = tree.parent / 'aerial_gym_simulator/results/sitl_ref_v1_r2/g9_attempt4/g9_lee_steps.json'
report = json.loads(primary.read_text())
launch = json.loads((base / 'launch_attempt4.json').read_text())
complete = json.loads((base / 'completion_attempt4.json').read_text())
watchdog = json.loads((base / 'resources_attempt4.json').read_text())
head = subprocess.check_output(['git', '-C', str(tree), 'rev-parse', 'HEAD'], text=True).strip()
dirty = subprocess.check_output(['git', '-C', str(tree), 'status', '--porcelain'], text=True).strip()
assert head == report['commit'] == launch['source_head'] == complete['source_head'] == '06383eba74134a594bd38ba933cb5e0512f1d342'
assert not dirty and report['dirty'] is False and report['source_still_clean'] and complete['source_still_clean']
assert report['aerial_gym_origin'] == str(tree / 'aerial_gym/__init__.py')
assert report['validity'] == 'RECORDED_REVIEW_REQUIRED'
assert report['num_envs'] == 4 and report['phys_dt_s'] == 0.01 and report['substeps_per_action'] == 10
assert report['controller'] == 'lee_velocity_control_navrl' and report['robot'] == 'navrl_quad'

metric_keys = {'rise_10_90_s', 'onset_latency_s', 'overshoot_pct', 'settling_5pct_s', 'final_mean'}
steps = [(axis, step) for axis, rows in report['A3'].items() for step in rows]
steps += [('yaw', row) for row in report['A4']]
assert set(report['A3']) == {'x', 'y'}
assert all([(d['from'], d['to']) for d in report['A3'][axis]] == [(0, 1), (1, 2.5), (2.5, 0)] for axis in ('x', 'y'))
assert [d['cmd_rad_s'] for d in report['A4']] == [3, -3]
metrics = []
for axis, step in steps:
    assert len(step['per_env']) == 4 and step['median_over_envs']['envs'] == 4
    assert all(set(e) == metric_keys for e in step['per_env'])
    assert all(isinstance(v, (int, float)) and math.isfinite(v) for e in step['per_env'] for v in e.values())
    upper = {k: sorted(e[k] for e in step['per_env'])[2] for k in metric_keys}
    assert all(step['median_over_envs'][k] == v for k, v in upper.items())
    metrics.append({'axis': axis, 'from': step.get('from', 0), 'to': step.get('to', step.get('cmd_rad_s')),
                    'upper_middle_over_four': step['median_over_envs'],
                    'arithmetic_median_over_four': {k: statistics.median(e[k] for e in step['per_env']) for k in metric_keys},
                    'per_env_ranges': {k: [min(e[k] for e in step['per_env']), max(e[k] for e in step['per_env'])] for k in metric_keys}})

def numeric_leaves(value):
    if isinstance(value, dict):
        for child in value.values():
            yield from numeric_leaves(child)
    elif isinstance(value, list):
        for child in value:
            yield from numeric_leaves(child)
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        yield value

leaves = list(numeric_leaves(report))
assert all(math.isfinite(v) for v in leaves)
fields = ['hover_alt_mean_m_per_env', 'initial_position_m_per_env', 'bar_position_m_per_env',
          'final_position_m_per_env', 'contact_force_peak_N_per_env', 'alt_min_m_per_env',
          'alt_max_m_per_env', 'whole_run_tilt_max_deg_per_env', 'min_bar_dist_xy_m_per_env', 'start_xy_per_env']
assert all(len(report[k]) == 4 for k in fields)
assert report['initial_position_m_per_env'] == [[5, 5, 1]] * 4
assert report['bar_position_m_per_env'] == [[[38, 2, 1.5]]] * 4
assert report['start_xy_per_env'] == [[5, 5]] * 4
assert report['actual_obstacle_count'] == 1 and report['all_samples_finite'] is True
assert report['scene_contract'] == {
    'NAVRL_MAX_VELOCITY': '2.5', 'NAVRL_YAW_RATE_MAX': '3.0', 'NAVRL_MAX_TILT_DEG': '45',
    'NAVRL_ARENA_XY': '40', 'NAVRL_ARENA_Z': '3', 'NAVRL_NUM_BARS': '1', 'NAVRL_MAX_BARS': '1',
    'NAVRL_BAR_POOL': 'bars_h3', 'NAVRL_TARGET_DYNAMICS': 'legacy', 'NAVRL_DISTRACTOR_COUNT': '0'}
assert report['A5']['cmd_body'] == [2.5, 2.5]
assert all(len(report['A5'][k]) == 4 for k in ('hspeed_last1s_m_s_per_env', 'tilt_max_deg_per_env'))
assert report['contact_force_peak_N_per_env'] == [0] * 4

assert launch.items() <= complete.items()
assert complete['exit'] == watchdog['exit_code'] == 0
assert watchdog['stop_reason'] is None and watchdog['live_descendants_after_cleanup'] == 0
assert complete['host_before'] == launch['host_before']
assert complete['host_before']['fault_count'] == complete['host_after']['fault_count'] == report['host_faults_before'] == report['host_faults_after'] == 0
assert complete['host_before']['boot_id'] == complete['host_after']['boot_id']
assert complete['host_before']['online'] == complete['host_after']['online'] == '0-3,6-23'
assert watchdog['argv'] == launch['argv'][launch['argv'].index('--') + 1:]
assert watchdog['rss_limit_bytes'] == launch['aggregate_rss_watchdog_limit_bytes'] == 12 * 1024**3
assert watchdog['poll_interval_s'] == .25 and watchdog['timeout_s'] == 1200
assert watchdog['samples'] > 0 and watchdog['peak_aggregate_rss_bytes'] < watchdog['rss_limit_bytes']
assert watchdog['kernel_cgroup_limit_claimed'] is False and watchdog['subreaper_enabled'] is True
elapsed = (datetime.datetime.fromisoformat(watchdog['finished']) - datetime.datetime.fromisoformat(watchdog['started'])).total_seconds()
assert elapsed < 1200

log = (base / 'attempt4.log').read_text()
printed = json.loads(log[log.rfind('\n{\n') + 1:])
expected_printed = {'A3': {axis: [[s['from'], s['to'], s['median_over_envs']] for s in rows] for axis, rows in report['A3'].items()},
                    'A4': [[s['cmd_rad_s'], s['median_over_envs']] for s in report['A4']], 'A5': report['A5']}
assert printed == expected_printed

paths = [primary, base / 'launch_attempt4.json', base / 'completion_attempt4.json',
         base / 'resources_attempt4.json', base / 'attempt4.log', base / 'run_attempt4.py',
         tree / 'tools/sitl_g9_lee_steps_v2.py', Path('/home/fair/motar_tmp/g7_20261105/run_capped.py'),
         tree / 'docs/evidence/sitl_2026-11-05/g7/run_capped.py',
         tree / 'docs/evidence/sitl_2026-11-05/g7/resource_checks_prelaunch.json',
         tree / 'docs/plans/prereg_sitl_reference_g8g9_v1.md',
         tree / 'docs/plans/prereg_sitl_reference_g8g9_v1_amendment3_2026-11-05.md',
         tree / 'docs/plans/prereg_sitl_reference_g8g9_v1_amendment4_2026-11-05.md', Path(__file__)]
bindings = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
assert bindings[str(tree / 'tools/sitl_g9_lee_steps_v2.py')] == 'a41a4e1aadacabc023f37ae9b1efddbf01b212b18eeff559b53c35e87c2441a1'
assert bindings[str(paths[7])] == bindings[str(paths[8])] == '75e18306bb105a30f78dc9b67fe1f6db7c6e2df90896d08409ffb940afddcdf0'
out = {'started': started, 'finished': datetime.datetime.now().astimezone().isoformat(),
       'reviewer': 'OpenAI Codex /root/g9_results_review', 'model': 'GPT-6 per session instructions; exact serving variant not exposed',
       'audit_check': 'PASS for supplied record consistency and summary completeness only',
       'measured_record_usability_verdict': 'LIMITED_VALIDITY', 'source_head': head, 'source_dirty': bool(dirty),
       'complete_step_records': len(steps), 'complete_per_env_metric_values': len(steps) * 4 * len(metric_keys),
       'finite_primary_numeric_leaves': len(leaves), 'metrics': metrics,
       'diagnostics': {k: report[k] for k in fields}, 'A5': report['A5'],
       'watchdog_elapsed_s': elapsed, 'watchdog_peak_GiB': watchdog['peak_aggregate_rss_bytes'] / 1024**3,
       'watchdog': watchdog, 'sha256': bindings,
       'expected_from_source_only': {'simulation_duration_s': 42, 'physics_steps_per_env': 4200},
       'not_recorded': ['per-step traces', 'sample count', 'RNG seed', 'realized motor thrust constants and initial thrust states'],
       'limitations': ['No metric recomputation or independent finite-sample validation without per-step traces',
                       'Upper middle (third sorted value) used for four-env median, not mean of middle two',
                       'One physics step (0.01 s) within-step alignment uncertainty',
                       'Inherited motor-state/thrust-constant sampling and bar-pool geometry sampling remain',
                       'RSS watchdog is sampled, not a kernel hard cap',
                       'G9 is reported only, with no scientific PASS or PX4/real-flight qualification']}
with (audit / 'record_checks.json').open('x') as stream:
    json.dump(out, stream, indent=2)
    stream.write('\n')
print(json.dumps({'audit_check': out['audit_check'], 'measured_record_usability_verdict': out['measured_record_usability_verdict'],
                  'output': str(audit / 'record_checks.json'), 'started': started, 'finished': out['finished']}))
