"""Observe the unchanged PUBLIC_CPU runner and record every top-level test result.

Only this loaded runner's unittest reference is proxied. Global unittest, test
bodies/assertions, discovery, ordering and profile exclusions are not modified.
No simulator/GPU profile is exposed. Outputs are exclusively created, never replaced.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[3]
DISK_FLOOR = 3000000000


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(str(args.output))
    if os.environ.get('PYTHONNOUSERSITE') != '1' or os.environ.get('CUDA_VISIBLE_DEVICES') != '':
        raise RuntimeError('explicit pinned CPU-only environment required before interpreter startup')
    if shutil.disk_usage(ROOT).free < DISK_FLOOR:
        raise RuntimeError('disk below3GB; no test started')
    source = ROOT / 'tools/run_tests.py'
    spec = importlib.util.spec_from_file_location('_g4r3_public_cpu_runner', source)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    collected = []

    class ObservedResult(unittest.TextTestResult):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            self.rows, self.started, self.states, self.chains = [], {}, {}, {}
            self.disk_guard = False

        def startTest(self, test):
            super().startTest(test)
            self.started[test.id()] = time.perf_counter()
            self.states[test.id()] = 'PASS'
            if shutil.disk_usage(ROOT).free < DISK_FLOOR:
                self.disk_guard = True
                self.stop()

        def addError(self, test, err):
            self.states[test.id()] = 'ERROR'
            chain, exc = [], err[1]
            while exc is not None:
                chain.append({'type': type(exc).__name__, 'message': str(exc)})
                exc = exc.__cause__ or (None if exc.__suppress_context__ else exc.__context__)
            self.chains[test.id()] = chain
            super().addError(test, err)

        def addFailure(self, test, err):
            self.states[test.id()] = 'FAIL'
            super().addFailure(test, err)

        def addSkip(self, test, reason):
            self.states[test.id()] = 'SKIP'
            super().addSkip(test, reason)

        def addExpectedFailure(self, test, err):
            self.states[test.id()] = 'EXPECTED_FAILURE'
            super().addExpectedFailure(test, err)

        def addUnexpectedSuccess(self, test):
            self.states[test.id()] = 'UNEXPECTED_SUCCESS'
            super().addUnexpectedSuccess(test)

        def addSubTest(self, test, subtest, err):
            if err is not None:
                self.states[test.id()] = 'FAIL' if issubclass(err[0], test.failureException) else 'ERROR'
            super().addSubTest(test, subtest, err)

        def stopTest(self, test):
            self.rows.append({'id': test.id(), 'status': self.states.get(test.id(), 'UNKNOWN'),
                              'seconds': time.perf_counter() - self.started[test.id()]})
            super().stopTest(test)

    def observed_runner(*a, **kw):
        kw['resultclass'] = ObservedResult
        original = unittest.TextTestRunner(*a, **kw)
        run = original.run

        def record(suite):
            result = run(suite)
            collected.append(result)
            return result
        original.run = record
        return original

    class LocalProxy:
        def __getattr__(self, key):
            return observed_runner if key == 'TextTestRunner' else getattr(unittest, key)

    runner.unittest = LocalProxy()
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted((ROOT / 'motar_eval/boia').glob('*.py'))}
    started = time.perf_counter()
    code = runner.main(['--profile', 'PUBLIC_CPU'])
    elapsed = time.perf_counter() - started
    assert len(collected) == 1
    result = collected[0]
    report = {'profile': 'PUBLIC_CPU', 'exit_code': code, 'status': 'PASS' if result.wasSuccessful() else 'FAIL',
              'ran': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
              'skipped': len(result.skipped), 'seconds': elapsed, 'tests': result.rows,
              'error_chains': result.chains, 'disk_guard_triggered': result.disk_guard,
              'free_bytes_after': shutil.disk_usage(ROOT).free,
              'source_sha256': hashes, 'runner_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'environment': {key: os.environ.get(key) for key in
                              ('PYTHONNOUSERSITE', 'CUDA_VISIBLE_DEVICES', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'TMPDIR')},
              'interpreter': sys.executable,
              'instrumentation': 'result observer only; original runner selection/discovery/order/assertions unchanged'}
    if result.disk_guard:
        report['status'], report['exit_code'] = 'RESOURCE_STOP', 3
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    return report['exit_code']


if __name__ == '__main__':
    sys.exit(main())
