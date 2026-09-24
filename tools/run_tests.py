#!/usr/bin/env python3
"""Run the Python test suite under a named profile.

    python -B tools/run_tests.py                          # PUBLIC_CPU, the documented default
    python -B tools/run_tests.py --profile GPU_REQUIRED   # only the tests that need a CUDA device
    python -B tools/run_tests.py --profile FULL_RESEARCH  # everything; needs a CUDA device

PUBLIC_CPU hides the CUDA device and runs every test except those listed in GPU_REQUIRED, which are
reported as excluded rather than failed. No test or assertion is changed. The profile only chooses
which tests run.

A GPU_REQUIRED run without a CUDA device reports SKIPPED_NO_CUDA and runs nothing. A FULL_RESEARCH run
without one refuses (exit 3), because it could not claim to be full.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"

# Test ids (module, module.Class or module.Class.test) that cannot run without a CUDA device,
# with the reason. Keep this list exact: every entry must match a discovered test.
GPU_REQUIRED = {
    "test_navrl_nonoverlap_placement.TestFootprintClearancePlacement":
        "setUpClass starts a worker that imports aerial_gym (Isaac Gym and Warp need a CUDA device)",
    "test_navrl_physical_target_lower_contract.LowerContractTest.test_fresh_child_imports_isaacgym_before_packed_torch":
        "the child process imports isaacgym, which needs a CUDA device",
}
PROFILES = ("PUBLIC_CPU", "GPU_REQUIRED", "FULL_RESEARCH")


def required_env(profile):
    env = {"PYTHONNOUSERSITE": "1"}
    if profile == "PUBLIC_CPU":
        env["CUDA_VISIBLE_DEVICES"] = ""
    return env


def cuda_available():
    code = "import torch, sys; sys.exit(0 if torch.cuda.is_available() else 1)"
    try:
        return subprocess.run([sys.executable, "-c", code], cwd=ROOT, timeout=120,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def flatten(suite):
    for item in suite:
        if isinstance(item, unittest.TestSuite):
            yield from flatten(item)
        else:
            yield item


def matches(test_id, entry):
    return test_id == entry or test_id.startswith(entry + ".")


def select(profile, tests):
    chosen, excluded = [], []
    for test in tests:
        entry = next((e for e in GPU_REQUIRED if matches(test.id(), e)), None)
        if profile == "FULL_RESEARCH" or (entry is not None) == (profile == "GPU_REQUIRED"):
            chosen.append(test)
        else:
            excluded.append(test.id())
    return chosen, excluded


def regroup(tests):
    """Rebuild class-level suites so setUpClass and tearDownClass still run once per class."""
    suite = unittest.TestSuite()
    for test in tests:
        suite.addTest(test)
    return suite


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--profile", choices=PROFILES, default="PUBLIC_CPU")
    parser.add_argument("--list", action="store_true", help="print the selected test ids and exit")
    args = parser.parse_args(argv)

    wanted = required_env(args.profile)
    if any(os.environ.get(k) != v for k, v in wanted.items()):
        # CUDA visibility and the user-site switch must be set before the interpreter imports torch.
        env = dict(os.environ, **wanted)
        os.execve(sys.executable, [sys.executable, "-B", __file__] + list(argv or sys.argv[1:]), env)

    report = {"profile": args.profile}
    if args.profile != "PUBLIC_CPU" and not cuda_available():
        report.update(status="SKIPPED_NO_CUDA" if args.profile == "GPU_REQUIRED" else "REFUSED_NO_CUDA",
                      not_run=sorted(GPU_REQUIRED), reasons=GPU_REQUIRED)
        print(json.dumps(report, indent=2))
        return 0 if args.profile == "GPU_REQUIRED" else 3

    # Mirror `python -m unittest discover -s tests` run from the repository root: the root, not this
    # script's directory, is first on the import path, and the tests directory follows.
    os.chdir(ROOT)
    sys.path[0] = str(ROOT)
    sys.path.insert(1, str(TESTS))
    discovered = list(flatten(unittest.defaultTestLoader.discover(str(TESTS), pattern="test_*.py",
                                                                  top_level_dir=str(TESTS))))
    stale = [e for e in GPU_REQUIRED if not any(matches(t.id(), e) for t in discovered)]
    if stale:
        print(json.dumps({"profile": args.profile, "status": "FAIL", "stale_gpu_required_entries": stale},
                         indent=2))
        return 2
    chosen, excluded = select(args.profile, discovered)
    if args.list:
        print("\n".join(t.id() for t in chosen))
        return 0

    result = unittest.TextTestRunner(verbosity=1).run(regroup(chosen))
    ok = result.wasSuccessful()
    report.update(status="PASS" if ok else "FAIL", ran=result.testsRun, failures=len(result.failures),
                  errors=len(result.errors), skipped=len(result.skipped), excluded=len(excluded),
                  excluded_reasons={e: GPU_REQUIRED[e] for e in GPU_REQUIRED} if excluded else {})
    print(json.dumps(report, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
