"""Run exactly one bounded CPU command, binding inputs and host kernel fault records. Never overwrite evidence."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def dependencies(source):
    paths = set()
    for base in [ROOT/'src', ROOT/'tests', ROOT/'tools', source]:
        paths.update(base.glob('*.py'))
    paths.update(p for p in (ROOT/'snapshot').rglob('*')
                 if p.is_file() and '__pycache__' not in p.parts and p.suffix != '.pyc')
    paths.update((ROOT/'docs').glob('R1_*.md'))
    # check_static.py also reads these immutable external references and the bar URDFs.
    v7 = Path('/home/fair/motar_tmp/p0_v7_20261029')
    paths.update(v7/'baseline'/v/'certify_b.py' for v in ['v3_run4','v4_run5','v5_run6','v6_run7'])
    paths.add(v7/'src/certify_b.py')
    paths.update(Path('/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/resources/models/environment_assets/bars_h3').glob('*.urdf'))
    return {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def host():
    r = subprocess.run(["journalctl", "-k", "-b", "--no-pager", "-o", "cat"], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, timeout=20)
    if r.returncode:
        raise RuntimeError("Cannot read host kernel journal: " + r.stderr[:300])
    import re
    events = [s for s in r.stdout.splitlines() if re.search(
        r"Machine check|segfault|general protection|mce:.*Hardware Error|BERT:.*Hardware Error", s, re.I)]
    return {"boot_id": Path('/proc/sys/kernel/random/boot_id').read_text().strip(), "events": events,
            "count": len(events), "online_cpus": Path('/sys/devices/system/cpu/online').read_text().strip()}


def main():
    name, seconds, command = sys.argv[1], int(sys.argv[2]), sys.argv[3:]
    if not name.replace('_', '').isalnum() or not 1 <= seconds <= 180 or not command:
        raise SystemExit("Usage: run_focused.py NAME SECONDS COMMAND ...; limit 180s")
    out = ROOT / 'results' / name
    out.mkdir(exist_ok=False)
    # Host records are written before starting the only bounded child.
    before = host()
    (out/'host_before.json').write_text(json.dumps(before, indent=2)+'\n')
    env = dict(os.environ, CUDA_VISIBLE_DEVICES='', PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1',
               OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1', NUMEXPR_NUM_THREADS='1')
    source = Path(env.get('P0R_SRC', str(ROOT/'src'))).resolve(strict=True)
    env['P0R_SRC'] = str(source)
    bound = dependencies(source)
    (out/'inputs.json').write_text(json.dumps(bound, indent=2)+'\n')
    start = time.monotonic()
    with (out/'stdout.log').open('w') as stdout, (out/'stderr.log').open('w') as stderr:
        r = subprocess.run(['timeout', '-k', '5s', str(seconds)+'s']+command, cwd=str(ROOT), env=env,
                           stdout=stdout, stderr=stderr)
    after = host()
    valid = before['boot_id'] == after['boot_id'] and before['events'] == after['events']
    bound_after = dependencies(source)
    changed = [p for p in sorted(bound.keys() | bound_after.keys()) if bound.get(p) != bound_after.get(p)]
    (out/'inputs_after.json').write_text(json.dumps(bound_after, indent=2)+'\n')
    relevant_env = ['P0R_SRC', 'CUDA_VISIBLE_DEVICES', 'PYTHONNOUSERSITE', 'PYTHONDONTWRITEBYTECODE',
                    'OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS',
                    'LD_LIBRARY_PATH', 'LD_PRELOAD', 'PYTHONPATH', 'CONDA_PREFIX', 'CONDA_DEFAULT_ENV']
    (out/'host_after.json').write_text(json.dumps(after, indent=2)+'\n')
    reported_checks = {}
    for p in out.glob('*.json'):
        value = json.loads(p.read_text())
        if isinstance(value, dict):
            for key in ['pass', 'focused_checks_pass']:
                if key in value:
                    reported_checks[p.name+':'+key] = value[key] is True
    artifacts = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in out.iterdir() if p.is_file()}
    record = {'command': command, 'timeout_s': seconds, 'exit_code': r.returncode,
              'elapsed_s': time.monotonic()-start, 'host_valid': valid, 'inputs_unchanged': not changed,
              'changed_inputs': changed, 'host_before_count': before['count'], 'host_after_count': after['count'],
              'actual_source_dir': str(source), 'dependency_count': len(bound),
              'environment': {k: env[k] for k in relevant_env if k in env}, 'artifact_sha256': artifacts,
              'reported_checks': reported_checks, 'reported_checks_pass': all(reported_checks.values()),
              'recorded_at': datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat()}
    (out/'receipt.json').write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record))
    if not valid or changed:
        return 86
    return r.returncode if r.returncode else (0 if all(reported_checks.values()) else 85)


if __name__ == '__main__':
    raise SystemExit(main())
